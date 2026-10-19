// SPDX-License-Identifier: GPL-2.0-only
/*
 * Meizu PRO 5 (m86) FPC1155 mBack capacitive touch and TEE IRQ bridge driver.
 *
 * Provides Linux input events for mBack touch (KEY_BACK + BTN_TOUCH) while
 * preserving sysfs compatibility with Android TEE fingerprint hal (libfpc_tac.so).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/m86-bus-qos.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

struct fpc1155_data {
	struct device *dev;
	struct input_dev *input;
	struct gpio_desc *irq_gpio;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;
	struct clk *pclk;
	struct clk *sclk;
	int irq;

	struct mutex lock;
	bool clocks_enabled;
	bool wakeup_enabled;
	bool mback_input_enable;

	/* Bus QoS */
	struct device *bus_dev;
	struct dev_pm_qos_request bus_qos_req;
	bool bus_qos_active;
};

static ssize_t irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpc1155_data *fpc = dev_get_drvdata(dev);
	int val = gpiod_get_value_cansleep(fpc->irq_gpio);

	return sysfs_emit(buf, "%d\n", val < 0 ? 0 : val);
}

static ssize_t irq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/* TEE TAC writes an IRQ ack; consume cleanly */
	return count;
}
static DEVICE_ATTR_RW(irq);

static ssize_t hw_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "reset\n");
}

static ssize_t hw_reset_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct fpc1155_data *fpc = dev_get_drvdata(dev);

	if (fpc->reset_gpio) {
		gpiod_set_value_cansleep(fpc->reset_gpio, 0);
		msleep(5);
		gpiod_set_value_cansleep(fpc->reset_gpio, 1);
		msleep(10);
	}
	return count;
}
static DEVICE_ATTR_RW(hw_reset);

static ssize_t wakeup_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpc1155_data *fpc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", fpc->wakeup_enabled ? 1 : 0);
}

static ssize_t wakeup_enable_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct fpc1155_data *fpc = dev_get_drvdata(dev);
	bool enable;

	if (sysfs_streq(buf, "enable") || sysfs_streq(buf, "1"))
		enable = true;
	else if (sysfs_streq(buf, "disable") || sysfs_streq(buf, "0"))
		enable = false;
	else
		return -EINVAL;

	mutex_lock(&fpc->lock);
	if (enable != fpc->wakeup_enabled) {
		if (enable)
			enable_irq_wake(fpc->irq);
		else
			disable_irq_wake(fpc->irq);
		fpc->wakeup_enabled = enable;
	}
	mutex_unlock(&fpc->lock);

	return count;
}
static DEVICE_ATTR_RW(wakeup_enable);

static ssize_t clk_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpc1155_data *fpc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", fpc->clocks_enabled ? 1 : 0);
}

static ssize_t clk_enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fpc1155_data *fpc = dev_get_drvdata(dev);
	bool enable;

	if (!count)
		return -EINVAL;
	if (buf[0] == '1')
		enable = true;
	else if (buf[0] == '0')
		enable = false;
	else
		return -EINVAL;

	mutex_lock(&fpc->lock);
	if (enable && !fpc->clocks_enabled) {
		if (fpc->pclk)
			clk_prepare_enable(fpc->pclk);
		if (fpc->sclk)
			clk_prepare_enable(fpc->sclk);
		fpc->clocks_enabled = true;
	} else if (!enable && fpc->clocks_enabled) {
		if (fpc->sclk)
			clk_disable_unprepare(fpc->sclk);
		if (fpc->pclk)
			clk_disable_unprepare(fpc->pclk);
		fpc->clocks_enabled = false;
	}
	mutex_unlock(&fpc->lock);

	return count;
}
static DEVICE_ATTR_RW(clk_enable);

static ssize_t lock_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpc1155_data *fpc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", fpc->bus_qos_active ? 1 : 0);
}

static ssize_t lock_freq_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct fpc1155_data *fpc = dev_get_drvdata(dev);
	bool lock;

	if (sysfs_streq(buf, "lock") || sysfs_streq(buf, "1"))
		lock = true;
	else if (sysfs_streq(buf, "unlock") || sysfs_streq(buf, "0"))
		lock = false;
	else
		return -EINVAL;

	mutex_lock(&fpc->lock);
	if (lock && !fpc->bus_qos_active) {
		if (!fpc->bus_dev)
			fpc->bus_dev = exynos_bus_get_dev("bus-mif");
		if (fpc->bus_dev) {
			int ret = exynos_bus_qos_add_request(fpc->bus_dev, &fpc->bus_qos_req, 800000);
			if (ret >= 0)
				fpc->bus_qos_active = true;
		}
	} else if (!lock && fpc->bus_qos_active) {
		exynos_bus_qos_remove_request(&fpc->bus_qos_req);
		fpc->bus_qos_active = false;
	}
	mutex_unlock(&fpc->lock);

	return count;
}
static DEVICE_ATTR_RW(lock_freq);

static ssize_t mback_input_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpc1155_data *fpc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", fpc->mback_input_enable ? 1 : 0);
}

static ssize_t mback_input_enable_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct fpc1155_data *fpc = dev_get_drvdata(dev);
	bool enable;
	int ret = kstrtobool(buf, &enable);

	if (ret)
		return ret;

	fpc->mback_input_enable = enable;
	return count;
}
static DEVICE_ATTR_RW(mback_input_enable);

static struct attribute *fpc1155_attrs[] = {
	&dev_attr_irq.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_lock_freq.attr,
	&dev_attr_mback_input_enable.attr,
	NULL,
};
ATTRIBUTE_GROUPS(fpc1155);

static irqreturn_t fpc1155_irq_thread(int irq, void *data)
{
	struct fpc1155_data *fpc = data;
	int val = gpiod_get_value_cansleep(fpc->irq_gpio);

	if (fpc->wakeup_enabled)
		pm_wakeup_event(fpc->dev, 1500);

	sysfs_notify(&fpc->dev->kobj, NULL, "irq");

	if (fpc->mback_input_enable && fpc->input) {
		input_report_key(fpc->input, KEY_BACK, val);
		input_report_key(fpc->input, BTN_TOUCH, val);
		input_sync(fpc->input);
	}

	return IRQ_HANDLED;
}

static int fpc1155_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fpc1155_data *fpc;
	int ret;

	fpc = devm_kzalloc(dev, sizeof(*fpc), GFP_KERNEL);
	if (!fpc)
		return -ENOMEM;

	fpc->dev = dev;
	mutex_init(&fpc->lock);
	fpc->mback_input_enable = true;

	platform_set_drvdata(pdev, fpc);

	/* Clocks (pclk, sclk) */
	fpc->pclk = devm_clk_get_optional(dev, "pclk");
	if (IS_ERR(fpc->pclk))
		return PTR_ERR(fpc->pclk);

	fpc->sclk = devm_clk_get_optional(dev, "sclk");
	if (IS_ERR(fpc->sclk))
		return PTR_ERR(fpc->sclk);

	/* VDD regulator (1.8V) */
	fpc->vdd = devm_regulator_get_optional(dev, "vdd");
	if (!IS_ERR(fpc->vdd)) {
		ret = regulator_enable(fpc->vdd);
		if (ret)
			dev_warn(dev, "failed to enable vdd regulator: %d\n", ret);
	} else {
		fpc->vdd = NULL;
	}

	/* Reset GPIO */
	fpc->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(fpc->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(fpc->reset_gpio), "failed to get reset-gpio\n");

	/* Hardware reset sequence: low for 5ms, high for 10ms */
	if (fpc->reset_gpio) {
		gpiod_set_value_cansleep(fpc->reset_gpio, 0);
		msleep(5);
		gpiod_set_value_cansleep(fpc->reset_gpio, 1);
		msleep(10);
	}

	/* IRQ GPIO */
	fpc->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(fpc->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(fpc->irq_gpio), "failed to get irq-gpio\n");

	fpc->irq = gpiod_to_irq(fpc->irq_gpio);
	if (fpc->irq < 0)
		return dev_err_probe(dev, fpc->irq, "failed to get irq from gpio\n");

	/* Input device */
	fpc->input = devm_input_allocate_device(dev);
	if (!fpc->input)
		return -ENOMEM;

	fpc->input->name = "fpc1155-mback";
	input_set_capability(fpc->input, EV_KEY, KEY_BACK);
	input_set_capability(fpc->input, EV_KEY, BTN_TOUCH);

	ret = input_register_device(fpc->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	device_init_wakeup(dev, true);

	ret = devm_request_threaded_irq(dev, fpc->irq, NULL, fpc1155_irq_thread,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"fpc1155-mback", fpc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	dev_info(dev, "FPC1155 mBack capacitive touch and TEE IRQ bridge probed\n");
	return 0;
}

static int fpc1155_remove(struct platform_device *pdev)
{
	struct fpc1155_data *fpc = platform_get_drvdata(pdev);

	if (fpc->bus_qos_active) {
		exynos_bus_qos_remove_request(&fpc->bus_qos_req);
		fpc->bus_qos_active = false;
	}

	if (fpc->wakeup_enabled)
		disable_irq_wake(fpc->irq);

	if (fpc->clocks_enabled) {
		if (fpc->sclk)
			clk_disable_unprepare(fpc->sclk);
		if (fpc->pclk)
			clk_disable_unprepare(fpc->pclk);
		fpc->clocks_enabled = false;
	}

	if (fpc->vdd)
		regulator_disable(fpc->vdd);

	return 0;
}

static const struct of_device_id fpc1155_of_match[] = {
	{ .compatible = "meizu,m86-fpc1155" },
	{ .compatible = "fpc,fpc_irq" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpc1155_of_match);

static struct platform_driver fpc1155_driver = {
	.probe = fpc1155_probe,
	.remove = fpc1155_remove,
	.driver = {
		.name = "fpc_irq",
		.of_match_table = fpc1155_of_match,
		.dev_groups = fpc1155_groups,
	},
};
module_platform_driver(fpc1155_driver);

MODULE_AUTHOR("Antigravity Meizu Mainline Team");
MODULE_DESCRIPTION("Meizu PRO 5 FPC1155 mBack touch and TEE IRQ bridge driver");
MODULE_LICENSE("GPL");
