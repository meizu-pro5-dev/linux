// SPDX-License-Identifier: GPL-2.0-only
/* Exynos7420 P0 telemetry and conservative software critical shutdown. */
#include <linux/hwmon.h>
#include <linux/clk.h>
#include <linux/thermal.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/m86-tmu-transition.h>
#include <linux/of.h>
#include <linux/platform_device.h>
struct m86_tmu {
	void __iomem *base[4];
	u32 trim[4];
	struct device *hwmon;
	struct thermal_trip trip;
};
/* Serializes sensor reads and the complete bounded G3D power-on sequence.
 * Global lock also pins the devm-managed instance until a callback returns.
 */
static DEFINE_MUTEX(m86_tmu_access);
static struct m86_tmu *m86_tmu_instance;

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
	if (!(readl(b + 0x20) & BIT(0))) {
		dev_err_ratelimited(dev, "TMU%d sensing disabled control=%08x\n", channel, readl(b + 0x20));
		return -ENODATA;
	}
	/* Vendor cal_tmu7420.h STATUS[12]=VALID_P0, STATUS[0]=idle,
	 * not the same signal. No stale-temperature fallback. Bounded wait
	 * matches existing startup20ms allowance, then verifies sample validity.
	 */
	for (attempt = 0; attempt < 2; attempt++) {
		ret = readl_poll_timeout(b + 0x28, status, status & BIT(12), 100, 20000);
		if (ret) {
			dev_err_ratelimited(dev, "TMU%d VALID_P0 timeout control=%08x status=%08x code=%08x\n",
				channel, readl(b + 0x20), status, readl(b + 0x40));
			return ret;
		}
		code = readl(b + 0x40) & 0x1ff;
		if ((readl(b + 0x28) & BIT(12)) && (readl(b + 0x20) & BIT(0)))
			break;
	}
	if (attempt == 2) {
		dev_err_ratelimited(dev, "TMU%d sample invalidated twice status=%08x\n", channel, readl(b + 0x28));
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
 control = readl(b + 0x20);
 if (!(control & BIT(0))) {
  ret = -ENODATA;
  goto unlock;
 }
 writel(control & ~BIT(0), b + 0x20);
 ret = readl_poll_timeout(b + 0x28, status, status & BIT(0), 10, 20000);
 if (!ret)
  ret = transition(data);
 /* Restore sensing even after an unsuccessful domain operation. A caller
  * must treat either failure as failed resume and never touch GPU MMIO.
  */
 writel(control, b + 0x20);
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
	for (i = 0; i < 4; i++) {
		ret = m86_read(t->hwmon, hwmon_temp, hwmon_temp_input, i, &value);
		if (ret) return ret;
		maximum = max(maximum, (int)value);
	}
	*temp = maximum;
	return 0;
}
static struct thermal_zone_device_ops m86_thermal_ops = {
	.get_temp = m86_thermal_temp,
};
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
static int m86_probe(struct platform_device *pdev)
{
	struct m86_tmu *t;
	struct device *hwmon;
	struct thermal_zone_device *tz;
	struct clk *clk;
	u32 control, status, trim, reference;
	int i, j, ret;
	clk = devm_clk_get_enabled(&pdev->dev, "tmu_apbif");
	if (IS_ERR(clk)) return dev_err_probe(&pdev->dev, PTR_ERR(clk), "TMU APB clock failed\n");
	clk = devm_clk_get_enabled(&pdev->dev, "tmu_sclk");
	if (IS_ERR(clk)) return dev_err_probe(&pdev->dev, PTR_ERR(clk), "TMU sensing clock failed\n");
	t = devm_kzalloc(&pdev->dev, sizeof(*t), GFP_KERNEL);
	if (!t) return -ENOMEM;
	for (i = 0; i < 4; i++) {
		t->base[i] = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(t->base[i])) return PTR_ERR(t->base[i]);
		/* Start sensing before waiting for the banked P0 trim selection. */
		control = readl(t->base[i] + 0x20);
		writel(control & ~BIT(0), t->base[i] + 0x20);
		ret = readl_poll_timeout(t->base[i] + 0x28, status,
					status & BIT(0), 10, 20000);
		if (ret) return dev_err_probe(&pdev->dev, ret, "TMU idle timeout\n");
		writel(0xe10, t->base[i] + 0x2c);
		writel(control | BIT(0), t->base[i] + 0x20);
		ret = readl_poll_timeout(t->base[i] + 0x28, status,
					!((status >> 8) & 7), 1, 20000);
		if (ret) return dev_err_probe(&pdev->dev, ret, "TMU P0 trim timeout\n");
		trim = readl(t->base[i]);
		if ((readl(t->base[i] + 0x28) >> 8) & 7)
			return -EAGAIN;
		if (!(trim & 0x1ff))
			return dev_err_probe(&pdev->dev, -ENODATA, "TMU P0 efuse missing\n");
		t->trim[i] = trim;
		reference = i == 1 ? 16 : 17;
		if (i != 1 && ((trim >> 18) & 0x1f))
			reference = (trim >> 18) & 0x1f;
		control = readl(t->base[i] + 0x20);
		/* Preserve inherited CONTROL[12] THERM_TRIP_EN; no guessed trip enable. */
		control = (control & 0xe07f10fe) | BIT(23) | (4 << 13) |
			  ((i == 1 ? 8 : 9) << 8) | (reference << 24);
		writel(control & ~BIT(0), t->base[i] + 0x20);
		for (j = 0; j < 5; j++)
			writel(0, t->base[i] + 0x110 + j * 0x10);
		writel(0xe10, t->base[i] + 0x2c);
		writel(control | BIT(0), t->base[i] + 0x20);
		ret = readl_poll_timeout(t->base[i] + 0x28, status,
					status & BIT(12), 100, 20000);
		if (ret) return dev_err_probe(&pdev->dev, ret, "TMU sample invalid\n");
		dev_info(&pdev->dev, "TMU%d inherited_trip_en=%u rise7_6=%08x fall7_6=%08x\n", i,
			 !!(control & BIT(12)), readl(t->base[i] + 0x50), readl(t->base[i] + 0x60));
		dev_info(&pdev->dev, "TMU%d control=%#x status=%#x\n", i,
			 readl(t->base[i] + 0x20), readl(t->base[i] + 0x28));
	}
	hwmon = devm_hwmon_device_register_with_info(&pdev->dev, "m86_tmu",
						    t, &m86_chip, NULL);
	if (IS_ERR(hwmon)) return PTR_ERR(hwmon);
	t->hwmon = hwmon;
	/* Conservative bring-up policy; not a vendor-calibrated cooling model. */
	t->trip.temperature = 95000;
	t->trip.type = THERMAL_TRIP_CRITICAL;
	tz = thermal_zone_device_register_with_trips("m86-tmu-max", &t->trip,
						    1, 0, t, &m86_thermal_ops,
						    NULL, 0, 500);
	if (IS_ERR(tz)) return PTR_ERR(tz);
	ret = devm_add_action_or_reset(&pdev->dev, m86_unregister_zone, tz);
	if (ret) return ret;
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
static const struct of_device_id m86_of[] = {
	{ .compatible = "meizu,m86-tmu-monitor" }, {}
};
MODULE_DEVICE_TABLE(of, m86_of);
static struct platform_driver m86_driver = {
	.driver = { .name = "m86-tmu-monitor", .of_match_table = m86_of },
	.probe = m86_probe,
};
module_platform_driver(m86_driver);
MODULE_LICENSE("GPL");
