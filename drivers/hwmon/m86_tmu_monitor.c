// SPDX-License-Identifier: GPL-2.0-only
/* Exynos7420 P0 telemetry, hardware threshold interrupt-driven thermal zones, and critical shutdown. */
#include <linux/hwmon.h>
#include <linux/clk.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/m86-tmu-transition.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#define EXYNOS7420_TMU_REG_TRIMINFO		0x0000
#define EXYNOS7420_TMU_REG_CONTROL		0x0020
#define EXYNOS7420_TMU_REG_STATUS		0x0028
#define EXYNOS7420_TMU_REG_SAMPLING_INTERVAL	0x002c
#define EXYNOS7420_TMU_REG_CURRENT_TEMP0	0x0040
#define EXYNOS7420_TMU_THD_TEMP_RISE7_6		0x0050
#define EXYNOS7420_TMU_THD_TEMP_RISE5_4		0x0054
#define EXYNOS7420_TMU_THD_TEMP_RISE3_2		0x0058
#define EXYNOS7420_TMU_THD_TEMP_RISE1_0		0x005c
#define EXYNOS7420_TMU_THD_TEMP_FALL7_6		0x0060
#define EXYNOS7420_TMU_THD_TEMP_FALL5_4		0x0064
#define EXYNOS7420_TMU_THD_TEMP_FALL3_2		0x0068
#define EXYNOS7420_TMU_THD_TEMP_FALL1_0		0x006c
#define EXYNOS7420_TMU_INTEN			0x0110
#define EXYNOS7420_TMU_INTPEND			0x0118
#define EXYNOS7420_TMU_CLEAR_ALL_INT		0x00ff00ff
#define EXYNOS7420_TMU_TRIP_INT_EN		0x00070007

struct m86_tmu;

struct m86_tmu_bank {
	struct m86_tmu *tmu;
	int id;
	int irq;
};

struct m86_tmu {
	void __iomem *base[4];
	u32 trim[4];
	struct m86_tmu_bank banks[4];
	struct device *dev;
	struct device *hwmon;
	struct thermal_zone_device *tzd;
	struct thermal_trip trips[3];
};

/* Serializes sensor reads and the complete bounded G3D power-on sequence.
 * Global lock also pins the devm-managed instance until a callback returns.
 */
static DEFINE_MUTEX(m86_tmu_access);
static struct m86_tmu *m86_tmu_instance;

static const struct thermal_trip m86_tmu_trips[] = {
	[0] = {
		.temperature = 75000,
		.hysteresis = 5000,
		.type = THERMAL_TRIP_PASSIVE,
	},
	[1] = {
		.temperature = 85000,
		.hysteresis = 5000,
		.type = THERMAL_TRIP_HOT,
	},
	[2] = {
		.temperature = 95000,
		.hysteresis = 5000,
		.type = THERMAL_TRIP_CRITICAL,
	},
};

static u32 m86_tmu_temp_to_code(u32 trim, int temp_c)
{
	int e1 = trim & 0x1ff;
	int e2 = (trim >> 9) & 0x1ff;
	int code;

	if (trim & BIT(23)) {
		if (e2 <= e1)
			return 0;
		code = (temp_c - 25) * (e2 - e1) / 60 + e1;
	} else {
		code = temp_c - 25 + e1;
	}
	return clamp_val(code, 0, 0x1ff);
}

static void m86_tmu_set_thresholds(struct m86_tmu *t, int bank)
{
	void __iomem *b = t->base[bank];
	u32 trim = t->trim[bank];
	u32 r0, r1, r2, f0, f1, f2;
	int j;

	r0 = m86_tmu_temp_to_code(trim, 75);
	f0 = m86_tmu_temp_to_code(trim, 70);
	r1 = m86_tmu_temp_to_code(trim, 85);
	f1 = m86_tmu_temp_to_code(trim, 80);
	r2 = m86_tmu_temp_to_code(trim, 95);
	f2 = m86_tmu_temp_to_code(trim, 90);

	writel((r1 << 16) | r0, b + EXYNOS7420_TMU_THD_TEMP_RISE1_0);
	writel((f1 << 16) | f0, b + EXYNOS7420_TMU_THD_TEMP_FALL1_0);
	writel(r2, b + EXYNOS7420_TMU_THD_TEMP_RISE3_2);
	writel(f2, b + EXYNOS7420_TMU_THD_TEMP_FALL3_2);
	writel(0, b + EXYNOS7420_TMU_THD_TEMP_RISE5_4);
	writel(0, b + EXYNOS7420_TMU_THD_TEMP_FALL5_4);
	writel(0, b + EXYNOS7420_TMU_THD_TEMP_RISE7_6);
	writel(0, b + EXYNOS7420_TMU_THD_TEMP_FALL7_6);

	/* Clear pending interrupts */
	writel(EXYNOS7420_TMU_CLEAR_ALL_INT, b + EXYNOS7420_TMU_INTPEND);

	/* Enable rise 0..2 (bits 0..2) and fall 0..2 (bits 16..18) */
	for (j = 0; j < 5; j++)
		writel(EXYNOS7420_TMU_TRIP_INT_EN, b + EXYNOS7420_TMU_INTEN + j * 0x10);
}

static umode_t m86_visible(const void *data, enum hwmon_sensor_types type,
			   u32 attr, int channel) { return 0444; }

static int m86_read_locked(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, long *val)
{
	struct m86_tmu *t = dev_get_drvdata(dev);
	void __iomem *b = t->base[channel];
	u32 status, trim, code;
	int e1, e2, ret, attempt;
	trim = t->trim[channel];
	if (!(readl(b + EXYNOS7420_TMU_REG_CONTROL) & BIT(0))) {
		dev_err_ratelimited(dev, "TMU%d sensing disabled control=%08x\n",
				    channel, readl(b + EXYNOS7420_TMU_REG_CONTROL));
		return -ENODATA;
	}
	/* Vendor cal_tmu7420.h STATUS[12]=VALID_P0, STATUS[0]=idle,
	 * not the same signal. No stale-temperature fallback. Bounded wait
	 * matches existing startup20ms allowance, then verifies sample validity.
	 */
	for (attempt = 0; attempt < 2; attempt++) {
		ret = readl_poll_timeout(b + EXYNOS7420_TMU_REG_STATUS, status,
					 status & BIT(12), 100, 20000);
		if (ret) {
			dev_err_ratelimited(dev, "TMU%d VALID_P0 timeout control=%08x status=%08x code=%08x\n",
					    channel, readl(b + EXYNOS7420_TMU_REG_CONTROL),
					    status, readl(b + EXYNOS7420_TMU_REG_CURRENT_TEMP0));
			return ret;
		}
		code = readl(b + EXYNOS7420_TMU_REG_CURRENT_TEMP0) & 0x1ff;
		if ((readl(b + EXYNOS7420_TMU_REG_STATUS) & BIT(12)) &&
		    (readl(b + EXYNOS7420_TMU_REG_CONTROL) & BIT(0)))
			break;
	}
	if (attempt == 2) {
		dev_err_ratelimited(dev, "TMU%d sample invalidated twice status=%08x\n",
				    channel, readl(b + EXYNOS7420_TMU_REG_STATUS));
		return -EAGAIN;
	}
	e1 = trim & 0x1ff; e2 = (trim >> 9) & 0x1ff;
	if (!e1) return -ENODATA; /* Do not disguise missing efuse by fallback. */
	if (trim & BIT(23)) {
		if (e2 <= e1) return -ERANGE;
		*val = ((int)code - e1) * 60000 / (e2 - e1) + 25000;
	} else {
		*val = ((int)code - e1 + 25) * 1000;
	}
	return 0;
}

static int m86_read(struct device *dev, enum hwmon_sensor_types type,
		    u32 attr, int channel, long *val)
{
	int ret;
	mutex_lock(&m86_tmu_access);
	ret = m86_read_locked(dev, type, attr, channel, val);
	mutex_unlock(&m86_tmu_access);
	return ret;
}

/* Vendor EXYNOS_TMU_CORE_GPU is bank 2 at 0x10068000. Only CONTROL[0]
 * changes; calibration, thermal-trip enable and thresholds are preserved.
 * Callback must be bounded and must not read thermal/hwmon or enter devfreq.
 */
int m86_tmu_gpu_transition(int (*transition)(void *), void *data)
{
	struct m86_tmu *t;
	void __iomem *b;
	u32 control, status;
	long temperature;
	int ret, restore_ret;
	if (!transition)
		return -EINVAL;
	mutex_lock(&m86_tmu_access);
	t = m86_tmu_instance;
	if (!t) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}
	b = t->base[2];
	control = readl(b + EXYNOS7420_TMU_REG_CONTROL);
	if (control & BIT(0)) {
		writel(control & ~BIT(0), b + EXYNOS7420_TMU_REG_CONTROL);
		ret = readl_poll_timeout(b + EXYNOS7420_TMU_REG_STATUS, status,
					 status & BIT(0), 10, 20000);
	} else {
		ret = 0;
	}
	if (!ret)
		ret = transition(data);
	/* Restore sensing and re-program thresholds after domain-on operation completes. */
	writel(control | BIT(0), b + EXYNOS7420_TMU_REG_CONTROL);
	m86_tmu_set_thresholds(t, 2);
	restore_ret = m86_read_locked(t->hwmon, hwmon_temp,
				      hwmon_temp_input, 2, &temperature);
	if (restore_ret)
		dev_err(t->hwmon, "GPU TMU restore failed: %d (transition: %d)\n",
			restore_ret, ret);
	if (!ret)
		ret = restore_ret;
unlock:
	mutex_unlock(&m86_tmu_access);
	return ret;
}
EXPORT_SYMBOL_GPL(m86_tmu_gpu_transition);

/* Disable TMU2 (EXYNOS_TMU_CORE_GPU) before cutting BLK_G3D power so that
 * the unpowered G3D remote thermal diode does not glitch and trigger
 * hardware THERM_TRIP or critical thermal shutdown. Leave TMU2 disabled
 * while BLK_G3D remains off; m86_tmu_gpu_transition re-enables it on power-on.
 */
int m86_tmu_gpu_power_off(int (*transition)(void *), void *data)
{
	struct m86_tmu *t;
	void __iomem *b;
	u32 control, status;
	int ret, j;
	if (!transition)
		return -EINVAL;
	mutex_lock(&m86_tmu_access);
	t = m86_tmu_instance;
	if (!t) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}
	b = t->base[2];

	/* Mask INTEN = 0 and clear INTPEND before TMU2 power-off */
	for (j = 0; j < 5; j++)
		writel(0, b + EXYNOS7420_TMU_INTEN + j * 0x10);
	writel(0xffffffff, b + EXYNOS7420_TMU_INTPEND);

	control = readl(b + EXYNOS7420_TMU_REG_CONTROL);
	if (control & BIT(0)) {
		writel(control & ~BIT(0), b + EXYNOS7420_TMU_REG_CONTROL);
		ret = readl_poll_timeout(b + EXYNOS7420_TMU_REG_STATUS, status,
					 status & BIT(0), 10, 20000);
		if (ret) {
			writel(control, b + EXYNOS7420_TMU_REG_CONTROL);
			m86_tmu_set_thresholds(t, 2);
			goto unlock;
		}
	}
	ret = transition(data);
	if (ret) {
		writel(control | BIT(0), b + EXYNOS7420_TMU_REG_CONTROL);
		m86_tmu_set_thresholds(t, 2);
	}
unlock:
	mutex_unlock(&m86_tmu_access);
	return ret;
}
EXPORT_SYMBOL_GPL(m86_tmu_gpu_power_off);

static const struct hwmon_ops m86_ops = {
	.is_visible = m86_visible, .read = m86_read,
};
static const struct hwmon_channel_info * const m86_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT, HWMON_T_INPUT,
			   HWMON_T_INPUT, HWMON_T_INPUT), NULL
};
static const struct hwmon_chip_info m86_chip = { .ops = &m86_ops, .info = m86_info };

static int m86_thermal_temp(struct thermal_zone_device *tz, int *temp)
{
	struct m86_tmu *t = thermal_zone_device_priv(tz);
	long value;
	int i, ret, maximum = INT_MIN;
	mutex_lock(&m86_tmu_access);
	for (i = 0; i < 4; i++) {
		/* Skip GPU TMU2 while sensing is disabled during BLK_G3D power-off. */
		if (i == 2 && !(readl(t->base[2] + EXYNOS7420_TMU_REG_CONTROL) & BIT(0)))
			continue;
		ret = m86_read_locked(t->hwmon, hwmon_temp, hwmon_temp_input, i, &value);
		if (ret) {
			mutex_unlock(&m86_tmu_access);
			return ret;
		}
		maximum = max(maximum, (int)value);
	}
	mutex_unlock(&m86_tmu_access);
	if (maximum == INT_MIN)
		return -ENODATA;
	*temp = maximum;
	return 0;
}

static struct thermal_zone_device_ops m86_thermal_ops = {
	.get_temp = m86_thermal_temp,
};

static irqreturn_t m86_tmu_irq_handler(int irq, void *data)
{
	struct m86_tmu_bank *bank = data;
	void __iomem *b = bank->tmu->base[bank->id];

	/* Clear pending interrupts */
	writel(EXYNOS7420_TMU_CLEAR_ALL_INT, b + EXYNOS7420_TMU_INTPEND);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t m86_tmu_irq_thread(int irq, void *data)
{
	struct m86_tmu_bank *bank = data;
	struct m86_tmu *t = bank->tmu;

	if (t->tzd)
		thermal_zone_device_update(t->tzd, THERMAL_EVENT_UNSPECIFIED);

	return IRQ_HANDLED;
}

static void m86_forget_instance(void *data)
{
	mutex_lock(&m86_tmu_access);
	if (m86_tmu_instance == data)
		m86_tmu_instance = NULL;
	mutex_unlock(&m86_tmu_access);
}

static void m86_unregister_zone(void *data)
{
	thermal_zone_device_unregister(data);
}

static int m86_tmu_hw_init_bank(struct m86_tmu *t, int i)
{
	void __iomem *b = t->base[i];
	u32 control, status, trim, reference;
	int j, ret;

	/* Start sensing before waiting for the banked P0 trim selection. */
	control = readl(b + EXYNOS7420_TMU_REG_CONTROL);
	writel(control & ~BIT(0), b + EXYNOS7420_TMU_REG_CONTROL);
	ret = readl_poll_timeout(b + EXYNOS7420_TMU_REG_STATUS, status,
				 status & BIT(0), 10, 20000);
	if (ret)
		return ret;

	writel(0xe10, b + EXYNOS7420_TMU_REG_SAMPLING_INTERVAL);
	writel(control | BIT(0), b + EXYNOS7420_TMU_REG_CONTROL);
	ret = readl_poll_timeout(b + EXYNOS7420_TMU_REG_STATUS, status,
				 !((status >> 8) & 7), 1, 20000);
	if (ret)
		return ret;

	trim = readl(b + EXYNOS7420_TMU_REG_TRIMINFO);
	if ((readl(b + EXYNOS7420_TMU_REG_STATUS) >> 8) & 7)
		return -EAGAIN;
	if (!(trim & 0x1ff))
		return -ENODATA;

	t->trim[i] = trim;
	reference = i == 1 ? 16 : 17;
	if (i != 1 && ((trim >> 18) & 0x1f))
		reference = (trim >> 18) & 0x1f;

	control = readl(b + EXYNOS7420_TMU_REG_CONTROL);
	/* Preserve inherited CONTROL[12] THERM_TRIP_EN; no guessed trip enable. */
	control = (control & 0xe07f10fe) | BIT(23) | (4 << 13) |
		  ((i == 1 ? 8 : 9) << 8) | (reference << 24);
	writel(control & ~BIT(0), b + EXYNOS7420_TMU_REG_CONTROL);
	for (j = 0; j < 5; j++)
		writel(0, b + EXYNOS7420_TMU_INTEN + j * 0x10);
	writel(0xe10, b + EXYNOS7420_TMU_REG_SAMPLING_INTERVAL);
	writel(control | BIT(0), b + EXYNOS7420_TMU_REG_CONTROL);
	ret = readl_poll_timeout(b + EXYNOS7420_TMU_REG_STATUS, status,
				 status & BIT(12), 100, 20000);
	if (ret)
		return ret;

	dev_info(t->dev, "TMU%d inherited_trip_en=%u rise7_6=%08x fall7_6=%08x control=%#x status=%#x\n",
		 i, !!(control & BIT(12)),
		 readl(b + EXYNOS7420_TMU_THD_TEMP_RISE7_6),
		 readl(b + EXYNOS7420_TMU_THD_TEMP_FALL7_6),
		 readl(b + EXYNOS7420_TMU_REG_CONTROL),
		 readl(b + EXYNOS7420_TMU_REG_STATUS));

	return 0;
}

static int m86_probe(struct platform_device *pdev)
{
	struct m86_tmu *t;
	struct device *hwmon;
	struct thermal_zone_device *tz;
	struct clk *clk;
	int i, ret;

	clk = devm_clk_get_enabled(&pdev->dev, "tmu_apbif");
	if (IS_ERR(clk)) return dev_err_probe(&pdev->dev, PTR_ERR(clk), "TMU APB clock failed\n");
	clk = devm_clk_get_enabled(&pdev->dev, "tmu_sclk");
	if (IS_ERR(clk)) return dev_err_probe(&pdev->dev, PTR_ERR(clk), "TMU sensing clock failed\n");

	t = devm_kzalloc(&pdev->dev, sizeof(*t), GFP_KERNEL);
	if (!t) return -ENOMEM;
	t->dev = &pdev->dev;
	platform_set_drvdata(pdev, t);

	for (i = 0; i < 4; i++) {
		t->base[i] = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(t->base[i])) return PTR_ERR(t->base[i]);

		ret = m86_tmu_hw_init_bank(t, i);
		if (ret)
			return dev_err_probe(&pdev->dev, ret, "TMU%d hw init failed\n", i);

		m86_tmu_set_thresholds(t, i);
	}

	hwmon = devm_hwmon_device_register_with_info(&pdev->dev, "m86_tmu",
						    t, &m86_chip, NULL);
	if (IS_ERR(hwmon)) return PTR_ERR(hwmon);
	t->hwmon = hwmon;

	/* Interrupt-driven thermal zones: polling_delay = 0, passive_delay = 1000 */
	memcpy(t->trips, m86_tmu_trips, sizeof(m86_tmu_trips));
	tz = thermal_zone_device_register_with_trips("m86-tmu-max", t->trips,
						    ARRAY_SIZE(t->trips), 0, t,
						    &m86_thermal_ops, NULL,
						    1000, 0);
	if (IS_ERR(tz)) return PTR_ERR(tz);
	t->tzd = tz;

	ret = devm_add_action_or_reset(&pdev->dev, m86_unregister_zone, tz);
	if (ret) return ret;

	/* Request threaded IRQs for all 4 banks */
	for (i = 0; i < 4; i++) {
		int irq = platform_get_irq(pdev, i);
		if (irq < 0)
			return dev_err_probe(&pdev->dev, irq, "Failed to get TMU%d irq\n", i);
		t->banks[i].tmu = t;
		t->banks[i].id = i;
		t->banks[i].irq = irq;
		ret = devm_request_threaded_irq(&pdev->dev, irq,
						m86_tmu_irq_handler,
						m86_tmu_irq_thread,
						IRQF_ONESHOT,
						"m86_tmu", &t->banks[i]);
		if (ret)
			return dev_err_probe(&pdev->dev, ret, "Failed to request TMU%d irq: %d\n", i, ret);
	}

	ret = thermal_zone_device_enable(tz);
	if (ret) return ret;

	mutex_lock(&m86_tmu_access);
	if (m86_tmu_instance) {
		mutex_unlock(&m86_tmu_access);
		return -EBUSY;
	}
	m86_tmu_instance = t;
	mutex_unlock(&m86_tmu_access);

	return devm_add_action_or_reset(&pdev->dev, m86_forget_instance, t);
}

static int m86_tmu_suspend(struct device *dev)
{
	struct m86_tmu *t = dev_get_drvdata(dev);
	int i, j;

	mutex_lock(&m86_tmu_access);
	for (i = 0; i < 4; i++) {
		if (t->base[i]) {
			for (j = 0; j < 5; j++)
				writel(0, t->base[i] + EXYNOS7420_TMU_INTEN + j * 0x10);
			writel(EXYNOS7420_TMU_CLEAR_ALL_INT, t->base[i] + EXYNOS7420_TMU_INTPEND);
		}
	}
	mutex_unlock(&m86_tmu_access);
	return 0;
}

static int m86_tmu_resume(struct device *dev)
{
	struct m86_tmu *t = dev_get_drvdata(dev);
	int i;

	mutex_lock(&m86_tmu_access);
	for (i = 0; i < 4; i++) {
		if (t->base[i]) {
			m86_tmu_hw_init_bank(t, i);
			m86_tmu_set_thresholds(t, i);
		}
	}
	mutex_unlock(&m86_tmu_access);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(m86_pm_ops, m86_tmu_suspend, m86_tmu_resume);

static const struct of_device_id m86_of[] = {
	{ .compatible = "meizu,m86-tmu-monitor" }, {}
};
MODULE_DEVICE_TABLE(of, m86_of);

static struct platform_driver m86_driver = {
	.driver = {
		.name = "m86-tmu-monitor",
		.of_match_table = m86_of,
		.pm = pm_sleep_ptr(&m86_pm_ops),
	},
	.probe = m86_probe,
};
module_platform_driver(m86_driver);
MODULE_LICENSE("GPL");
