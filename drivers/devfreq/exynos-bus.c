// SPDX-License-Identifier: GPL-2.0-only
/*
 * Generic Exynos Bus frequency driver with DEVFREQ Framework
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 * Author : Chanwoo Choi <cw00.choi@samsung.com>
 *
 * This driver support Exynos Bus frequency feature by using
 * DEVFREQ framework and is based on drivers/devfreq/exynos/exynos4_bus.c.
 */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/devfreq-event.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/regulator/consumer.h>
#include <linux/m86-bus-qos.h>

#include <linux/workqueue.h>

#define DEFAULT_SATURATION_RATIO	40

struct exynos_bus {
	struct device *dev;
	struct platform_device *icc_pdev;

	struct devfreq *devfreq;
	struct devfreq_event_dev **edev;
	unsigned int edev_count;
	struct mutex lock;

	unsigned long curr_freq;

	int opp_token;
	struct clk *clk;
	unsigned int ratio;
	bool is_exynos7420_mif;
	bool is_exynos7420_int;
	struct regulator *reg_vdd;
	struct dev_pm_qos_request baseline_qos_req;
	struct delayed_work boot_handoff_work;
	u32 baseline_floor_khz;
	struct list_head node;
};

#include "exynos7420-int-mif.h"

static LIST_HEAD(exynos_bus_list);
static DEFINE_MUTEX(exynos_bus_list_lock);

struct device *exynos_bus_get_dev(const char *name)
{
	struct exynos_bus *bus;
	struct device *found = NULL;

	mutex_lock(&exynos_bus_list_lock);
	list_for_each_entry(bus, &exynos_bus_list, node) {
		if (!name || (bus->dev->of_node && of_node_name_eq(bus->dev->of_node, name))) {
			found = bus->dev;
			break;
		}
	}
	mutex_unlock(&exynos_bus_list_lock);

	return found;
}
EXPORT_SYMBOL_GPL(exynos_bus_get_dev);

/*
 * Control the devfreq-event device to get the current state of bus
 */
#define exynos_bus_ops_edev(ops)				\
static int exynos_bus_##ops(struct exynos_bus *bus)		\
{								\
	int i, ret;						\
								\
	for (i = 0; i < bus->edev_count; i++) {			\
		if (!bus->edev[i])				\
			continue;				\
		ret = devfreq_event_##ops(bus->edev[i]);	\
		if (ret < 0)					\
			return ret;				\
	}							\
								\
	return 0;						\
}
exynos_bus_ops_edev(enable_edev);
exynos_bus_ops_edev(disable_edev);
exynos_bus_ops_edev(set_event);

static int exynos_bus_get_event(struct exynos_bus *bus,
				struct devfreq_event_data *edata)
{
	struct devfreq_event_data event_data;
	unsigned long load_count = 0, total_count = 0;
	int i, ret = 0;

	for (i = 0; i < bus->edev_count; i++) {
		if (!bus->edev[i])
			continue;

		ret = devfreq_event_get_event(bus->edev[i], &event_data);
		if (ret < 0)
			return ret;

		if (i == 0 || event_data.load_count > load_count) {
			load_count = event_data.load_count;
			total_count = event_data.total_count;
		}
	}

	edata->load_count = load_count;
	edata->total_count = total_count;

	return ret;
}

/*
 * devfreq function for both simple-ondemand, powersave, and passive governor
 */
static int exynos_bus_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct exynos_bus *bus = dev_get_drvdata(dev);
	struct dev_pm_opp *new_opp;
	int ret = 0;

	/* Get correct frequency for bus. */
	new_opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(new_opp)) {
		dev_err(dev, "failed to get recommended opp instance\n");
		return PTR_ERR(new_opp);
	}

	dev_pm_opp_put(new_opp);

	/* Change voltage and frequency according to new OPP level */
	mutex_lock(&bus->lock);
	if (bus->is_exynos7420_mif) {
		mutex_lock(&exynos7420_hw.lock);
		ret = exynos7420_mif_target_locked(bus, *freq);
		mutex_unlock(&exynos7420_hw.lock);
	} else if (bus->is_exynos7420_int) {
		mutex_lock(&exynos7420_hw.lock);
		ret = exynos7420_int_target_locked(bus, *freq);
		mutex_unlock(&exynos7420_hw.lock);
	} else {
		if (bus->clk)
			ret = dev_pm_opp_set_rate(dev, *freq);
		if (!ret)
			bus->curr_freq = *freq;
	}

	mutex_unlock(&bus->lock);

	return ret;
}

static int exynos_bus_get_dev_status(struct device *dev,
				     struct devfreq_dev_status *stat)
{
	struct exynos_bus *bus = dev_get_drvdata(dev);
	struct devfreq_event_data edata;
	int ret;

	stat->current_frequency = bus->curr_freq;

	if (!bus->edev_count) {
		stat->busy_time = 0;
		stat->total_time = 0;
		return 0;
	}

	ret = exynos_bus_get_event(bus, &edata);
	if (ret < 0) {
		dev_err(dev, "failed to get event from devfreq-event devices\n");
		stat->total_time = stat->busy_time = 0;
		goto err;
	}

	stat->busy_time = (edata.load_count * 100) / bus->ratio;
	stat->total_time = edata.total_count;

	dev_dbg(dev, "Usage of devfreq-event : %lu/%lu\n", stat->busy_time,
							stat->total_time);

err:
	ret = exynos_bus_set_event(bus);
	if (ret < 0) {
		dev_err(dev, "failed to set event to devfreq-event devices\n");
		return ret;
	}

	return ret;
}

static void exynos_bus_boot_handoff_fn(struct work_struct *work)
{
	struct exynos_bus *bus = container_of(to_delayed_work(work),
					      struct exynos_bus,
					      boot_handoff_work);

	if (dev_pm_qos_request_active(&bus->baseline_qos_req) &&
	    bus->baseline_floor_khz == 1552000) {
		bus->baseline_floor_khz = 416000;
		dev_pm_qos_update_request(&bus->baseline_qos_req, 416000);
		dev_info(bus->dev, "boot handoff complete: baseline floor -> 416000 kHz\n");
	}
}

static ssize_t baseline_floor_khz_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct exynos_bus *bus = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", bus->baseline_floor_khz);
}

static ssize_t baseline_floor_khz_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct exynos_bus *bus = dev_get_drvdata(dev);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	cancel_delayed_work_sync(&bus->boot_handoff_work);
	bus->baseline_floor_khz = val;
	if (dev_pm_qos_request_active(&bus->baseline_qos_req))
		dev_pm_qos_update_request(&bus->baseline_qos_req, val);

	return count;
}
static DEVICE_ATTR_RW(baseline_floor_khz);

static void exynos_bus_exit(struct device *dev)
{
	struct exynos_bus *bus = dev_get_drvdata(dev);
	int ret;

	if (bus->is_exynos7420_mif) {
		cancel_delayed_work_sync(&bus->boot_handoff_work);
		if (dev_pm_qos_request_active(&bus->baseline_qos_req))
			dev_pm_qos_remove_request(&bus->baseline_qos_req);
	}

	mutex_lock(&exynos7420_hw.lock);
	if (bus->is_exynos7420_mif && exynos7420_hw.mif_bus == bus) {
		exynos7420_hw.mif_bus = NULL;
		exynos7420_hw.vdd_mif = NULL;
	} else if (bus->is_exynos7420_int && exynos7420_hw.int_bus == bus) {
		exynos7420_hw.int_bus = NULL;
		exynos7420_hw.vdd_int = NULL;
	}
	mutex_unlock(&exynos7420_hw.lock);

	mutex_lock(&exynos_bus_list_lock);
	list_del(&bus->node);
	mutex_unlock(&exynos_bus_list_lock);

	if (bus->edev_count > 0) {
		ret = exynos_bus_disable_edev(bus);
		if (ret < 0)
			dev_warn(dev, "failed to disable the devfreq-event devices\n");
	}

	platform_device_unregister(bus->icc_pdev);

	dev_pm_opp_of_remove_table(dev);
	if (bus->clk)
		clk_disable_unprepare(bus->clk);
	if (bus->opp_token > 0)
		dev_pm_opp_put_regulators(bus->opp_token);
}

static void exynos_bus_passive_exit(struct device *dev)
{
	struct exynos_bus *bus = dev_get_drvdata(dev);

	mutex_lock(&exynos7420_hw.lock);
	if (bus->is_exynos7420_int && exynos7420_hw.int_bus == bus) {
		exynos7420_hw.int_bus = NULL;
		exynos7420_hw.vdd_int = NULL;
	}
	mutex_unlock(&exynos7420_hw.lock);

	mutex_lock(&exynos_bus_list_lock);
	list_del(&bus->node);
	mutex_unlock(&exynos_bus_list_lock);

	platform_device_unregister(bus->icc_pdev);

	dev_pm_opp_of_remove_table(dev);
	if (bus->clk)
		clk_disable_unprepare(bus->clk);
}

static int exynos_bus_parent_parse_of(struct device_node *np,
					struct exynos_bus *bus)
{
	struct device *dev = bus->dev;
	const char *supplies[] = { "vdd", NULL };
	int i, ret, count, size;

	if (of_property_present(np, "vdd-supply") &&
	    !of_machine_is_compatible("samsung,exynos7420")) {
		ret = dev_pm_opp_set_regulators(dev, supplies);
		if (ret < 0) {
			dev_err(dev, "failed to set regulators %d\n", ret);
			return ret;
		}
		bus->opp_token = ret;
	}

	/*
	 * Get the devfreq-event devices to get the current utilization of
	 * buses if present in device tree.
	 */
	if (of_property_present(np, "devfreq-events")) {
		count = devfreq_event_get_edev_count(dev, "devfreq-events");
		if (count < 0) {
			dev_err(dev, "failed to get the count of devfreq-event dev\n");
			ret = count;
			goto err_regulator;
		}
		bus->edev_count = count;

		size = sizeof(*bus->edev) * count;
		bus->edev = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!bus->edev) {
			ret = -ENOMEM;
			goto err_regulator;
		}

		for (i = 0; i < count; i++) {
			bus->edev[i] = devfreq_event_get_edev_by_phandle(dev,
								"devfreq-events", i);
			if (IS_ERR(bus->edev[i])) {
				ret = -EPROBE_DEFER;
				goto err_regulator;
			}
		}
	} else {
		bus->edev_count = 0;
	}

	/*
	 * Optionally, Get the saturation ratio according to Exynos SoC
	 */
	if (of_property_read_u32(np, "exynos,saturation-ratio", &bus->ratio))
		bus->ratio = DEFAULT_SATURATION_RATIO;

	return 0;

err_regulator:
	if (bus->opp_token > 0)
		dev_pm_opp_put_regulators(bus->opp_token);

	return ret;
}

static int exynos_bus_parse_of(struct device_node *np,
			      struct exynos_bus *bus)
{
	struct device *dev = bus->dev;
	struct dev_pm_opp *opp;
	unsigned long rate = 0;
	int ret;

	/* Get the clock to provide each bus with source clock if present */
	bus->clk = devm_clk_get_optional(dev, "bus");
	if (IS_ERR(bus->clk)) {
		ret = PTR_ERR(bus->clk);
		dev_err(dev, "failed to get bus clock\n");
		return ret;
	}

	if (bus->clk) {
		ret = clk_prepare_enable(bus->clk);
		if (ret < 0) {
			dev_err(dev, "failed to get enable clock\n");
			return ret;
		}
		rate = clk_get_rate(bus->clk);
	}

	if (of_machine_is_compatible("samsung,exynos7420")) {
		struct regulator *vdd;

		ret = exynos7420_hw_bus_init_once(dev);
		if (ret)
			goto err_clk;

		vdd = devm_regulator_get_optional(dev, "vdd");
		if (IS_ERR(vdd)) {
			if (PTR_ERR(vdd) == -EPROBE_DEFER) {
				ret = -EPROBE_DEFER;
				goto err_clk;
			}
			vdd = NULL;
		}

		if (of_node_name_eq(np, "bus-mif")) {
			bus->is_exynos7420_mif = true;
			bus->reg_vdd = vdd;
			rate = 1552000000UL;
		} else if (of_node_name_eq(np, "bus-int")) {
			bus->is_exynos7420_int = true;
			bus->reg_vdd = vdd;
			rate = 400000000UL;
		}
	}

	/* Get the freq and voltage from OPP table to scale the bus freq */
	ret = dev_pm_opp_of_add_table(dev);
	if (ret < 0) {
		dev_err(dev, "failed to get OPP table\n");
		goto err_clk;
	}

	if (bus->is_exynos7420_mif) {
		int i;

		for (i = 0; i < EXYNOS7420_MIF_LV_COUNT; i++) {
			unsigned long f = (unsigned long)exynos7420_mif_opp_table[i].freq_khz * 1000UL;
			unsigned long u = exynos7420_mif_opp_table[i].volt_uv;

			dev_pm_opp_adjust_voltage(dev, f, u, u, u);
		}
	} else if (bus->is_exynos7420_int) {
		int i;

		for (i = 0; i < EXYNOS7420_INT_LV_COUNT; i++) {
			unsigned long f = (unsigned long)exynos7420_int_table[i].freq_khz * 1000UL;
			unsigned long u = exynos7420_int_table[i].volt_uv;

			dev_pm_opp_adjust_voltage(dev, f, u, u, u);
		}
	}

	opp = devfreq_recommended_opp(dev, &rate, 0);
	if (IS_ERR(opp)) {
		dev_err(dev, "failed to find dev_pm_opp\n");
		ret = PTR_ERR(opp);
		goto err_opp;
	}
	bus->curr_freq = dev_pm_opp_get_freq(opp);
	dev_pm_opp_put(opp);

	return 0;

err_opp:
	dev_pm_opp_of_remove_table(dev);
err_clk:
	if (bus->clk)
		clk_disable_unprepare(bus->clk);

	return ret;
}

static int exynos_bus_profile_init(struct exynos_bus *bus,
				   struct devfreq_dev_profile *profile)
{
	struct device *dev = bus->dev;
	struct devfreq_simple_ondemand_data *ondemand_data = NULL;
	const char *gov_name;
	int ret;

	/* Initialize the struct profile and governor data for parent device */
	profile->initial_freq = bus->curr_freq;
	profile->polling_ms = 50;
	profile->target = exynos_bus_target;
	profile->get_dev_status = exynos_bus_get_dev_status;
	profile->exit = exynos_bus_exit;

	if (bus->edev_count > 0) {
		ondemand_data = devm_kzalloc(dev, sizeof(*ondemand_data), GFP_KERNEL);
		if (!ondemand_data)
			return -ENOMEM;

		ondemand_data->upthreshold = 40;
		ondemand_data->downdifferential = 5;
		gov_name = DEVFREQ_GOV_SIMPLE_ONDEMAND;
	} else {
		/* No devfreq-event monitors (PPMU): drive by PM QoS using powersave governor */
		profile->polling_ms = 0;
		gov_name = DEVFREQ_GOV_POWERSAVE;
	}

	if (bus->is_exynos7420_mif) {
		INIT_DELAYED_WORK(&bus->boot_handoff_work, exynos_bus_boot_handoff_fn);
		bus->baseline_floor_khz = 1552000;
		dev_pm_qos_add_request(dev, &bus->baseline_qos_req,
				       DEV_PM_QOS_MIN_FREQUENCY,
				       bus->baseline_floor_khz);
	}

	/* Add devfreq device to monitor and handle the exynos bus */
	bus->devfreq = devm_devfreq_add_device(dev, profile, gov_name, ondemand_data);
	if (IS_ERR(bus->devfreq)) {
		dev_err(dev, "failed to add devfreq device (%ld)\n", PTR_ERR(bus->devfreq));
		if (bus->is_exynos7420_mif &&
		    dev_pm_qos_request_active(&bus->baseline_qos_req))
			dev_pm_qos_remove_request(&bus->baseline_qos_req);
		return PTR_ERR(bus->devfreq);
	}

	if (bus->is_exynos7420_mif)
		schedule_delayed_work(&bus->boot_handoff_work, 15 * HZ);

	/* Register opp_notifier to catch the change of OPP  */
	ret = devm_devfreq_register_opp_notifier(dev, bus->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to register opp notifier\n");
		return ret;
	}

	if (bus->edev_count > 0) {
		/* Enable devfreq-event to get raw data */
		ret = exynos_bus_enable_edev(bus);
		if (ret < 0) {
			dev_err(dev, "failed to enable devfreq-event devices\n");
			return ret;
		}

		ret = exynos_bus_set_event(bus);
		if (ret < 0) {
			dev_err(dev, "failed to set event to devfreq-event devices\n");
			goto err_edev;
		}
	}

	return 0;

err_edev:
	if (bus->edev_count > 0 && exynos_bus_disable_edev(bus))
		dev_warn(dev, "failed to disable the devfreq-event devices\n");

	return ret;
}

static int exynos_bus_profile_init_passive(struct exynos_bus *bus,
					   struct devfreq_dev_profile *profile)
{
	struct device *dev = bus->dev;
	struct devfreq_passive_data *passive_data;
	struct devfreq *parent_devfreq;

	/* Initialize the struct profile and governor data for passive device */
	profile->initial_freq = bus->curr_freq;
	profile->target = exynos_bus_target;
	profile->exit = exynos_bus_passive_exit;

	/* Get the instance of parent devfreq device */
	parent_devfreq = devfreq_get_devfreq_by_phandle(dev, "devfreq", 0);
	if (IS_ERR(parent_devfreq))
		return -EPROBE_DEFER;

	passive_data = devm_kzalloc(dev, sizeof(*passive_data), GFP_KERNEL);
	if (!passive_data)
		return -ENOMEM;

	passive_data->parent = parent_devfreq;

	/* Add devfreq device for exynos bus with passive governor */
	bus->devfreq = devm_devfreq_add_device(dev, profile, DEVFREQ_GOV_PASSIVE,
						passive_data);
	if (IS_ERR(bus->devfreq)) {
		dev_err(dev,
			"failed to add devfreq dev with passive governor\n");
		return PTR_ERR(bus->devfreq);
	}

	return 0;
}

static int exynos_bus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *node;
	struct devfreq_dev_profile *profile;
	struct exynos_bus *bus;
	int ret, max_state;
	unsigned long min_freq, max_freq;
	bool passive = false;

	if (!np) {
		dev_err(dev, "failed to find devicetree node\n");
		return -EINVAL;
	}

	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;
	mutex_init(&bus->lock);
	bus->dev = &pdev->dev;
	INIT_LIST_HEAD(&bus->node);
	platform_set_drvdata(pdev, bus);

	profile = devm_kzalloc(dev, sizeof(*profile), GFP_KERNEL);
	if (!profile)
		return -ENOMEM;

	node = of_parse_phandle(dev->of_node, "devfreq", 0);
	if (node) {
		of_node_put(node);
		passive = true;
	} else {
		ret = exynos_bus_parent_parse_of(np, bus);
		if (ret < 0)
			return ret;
	}

	/* Parse the device-tree to get the resource information */
	ret = exynos_bus_parse_of(np, bus);
	if (ret < 0)
		goto err_reg;

	if (passive)
		ret = exynos_bus_profile_init_passive(bus, profile);
	else
		ret = exynos_bus_profile_init(bus, profile);

	if (ret < 0)
		goto err;

	/* Create child platform device for the interconnect provider */
	if (of_property_present(dev->of_node, "#interconnect-cells")) {
		bus->icc_pdev = platform_device_register_data(
						dev, "exynos-generic-icc",
						PLATFORM_DEVID_AUTO, NULL, 0);

		if (IS_ERR(bus->icc_pdev)) {
			ret = PTR_ERR(bus->icc_pdev);
			goto err;
		}
	}

	mutex_lock(&exynos_bus_list_lock);
	list_add_tail(&bus->node, &exynos_bus_list);
	mutex_unlock(&exynos_bus_list_lock);

	if (bus->is_exynos7420_mif || bus->is_exynos7420_int)
		device_create_file(dev, &dev_attr_hw_status);
	if (bus->is_exynos7420_mif)
		device_create_file(dev, &dev_attr_baseline_floor_khz);

	max_state = bus->devfreq->max_state;
	min_freq = min(bus->devfreq->freq_table[0], bus->devfreq->freq_table[max_state - 1]) / 1000;
	max_freq = max(bus->devfreq->freq_table[0], bus->devfreq->freq_table[max_state - 1]) / 1000;
	pr_info("exynos-bus: new bus device registered: %s (%6ld KHz ~ %6ld KHz)\n",
		dev_name(dev), min_freq, max_freq);

	mutex_lock(&exynos7420_hw.lock);
	if (bus->is_exynos7420_mif) {
		exynos7420_hw.mif_bus = bus;
		exynos7420_hw.vdd_mif = bus->reg_vdd;
		if (bus->reg_vdd) {
			int uv = regulator_get_voltage(bus->reg_vdd);

			if (uv > 0)
				exynos7420_hw.cur_mif_volt_uv = uv;
		}
	} else if (bus->is_exynos7420_int) {
		exynos7420_hw.int_bus = bus;
		exynos7420_hw.vdd_int = bus->reg_vdd;
		if (bus->reg_vdd) {
			int uv = regulator_get_voltage(bus->reg_vdd);

			if (uv > 0)
				exynos7420_hw.cur_int_volt_uv = uv;
		}
		exynos7420_int_set_clocks(exynos7420_hw.cur_int_idx);
	}
	mutex_unlock(&exynos7420_hw.lock);

	return 0;

err:
	mutex_lock(&exynos7420_hw.lock);
	if (bus->is_exynos7420_mif && exynos7420_hw.mif_bus == bus) {
		exynos7420_hw.mif_bus = NULL;
		exynos7420_hw.vdd_mif = NULL;
	} else if (bus->is_exynos7420_int && exynos7420_hw.int_bus == bus) {
		exynos7420_hw.int_bus = NULL;
		exynos7420_hw.vdd_int = NULL;
	}
	mutex_unlock(&exynos7420_hw.lock);

	dev_pm_opp_of_remove_table(dev);
	if (bus->clk)
		clk_disable_unprepare(bus->clk);
err_reg:
	if (bus->opp_token > 0)
		dev_pm_opp_put_regulators(bus->opp_token);

	return ret;
}

static void exynos_bus_shutdown(struct platform_device *pdev)
{
	struct exynos_bus *bus = dev_get_drvdata(&pdev->dev);

	devfreq_suspend_device(bus->devfreq);
}

#ifdef CONFIG_PM_SLEEP
static int exynos_bus_resume(struct device *dev)
{
	struct exynos_bus *bus = dev_get_drvdata(dev);
	int ret;

	if (bus->is_exynos7420_mif) {
		mutex_lock(&bus->lock);
		mutex_lock(&exynos7420_hw.lock);
		if (exynos7420_hw.vdd_mif)
			exynos7420_hw.cur_mif_volt_uv = regulator_get_voltage(exynos7420_hw.vdd_mif);
		exynos7420_hw.cur_mif_idx = 0;
		bus->curr_freq = 1552000000UL;
		mutex_unlock(&exynos7420_hw.lock);
		mutex_unlock(&bus->lock);
	} else if (bus->is_exynos7420_int) {
		mutex_lock(&bus->lock);
		mutex_lock(&exynos7420_hw.lock);
		if (exynos7420_hw.vdd_int)
			exynos7420_hw.cur_int_volt_uv = regulator_get_voltage(exynos7420_hw.vdd_int);
		exynos7420_hw.cur_int_idx = 0;
		bus->curr_freq = 560000000UL;
		mutex_unlock(&exynos7420_hw.lock);
		mutex_unlock(&bus->lock);
	}

	if (bus->edev_count > 0) {
		ret = exynos_bus_enable_edev(bus);
		if (ret < 0) {
			dev_err(dev, "failed to enable the devfreq-event devices\n");
			return ret;
		}
	}

	return 0;
}

static int exynos_bus_suspend(struct device *dev)
{
	struct exynos_bus *bus = dev_get_drvdata(dev);
	int ret;

	if (bus->edev_count > 0) {
		ret = exynos_bus_disable_edev(bus);
		if (ret < 0) {
			dev_err(dev, "failed to disable the devfreq-event devices\n");
			return ret;
		}
	}

	if (bus->is_exynos7420_mif) {
		mutex_lock(&bus->lock);
		mutex_lock(&exynos7420_hw.lock);
		ret = exynos7420_mif_target_locked(bus, 1552000000UL);
		mutex_unlock(&exynos7420_hw.lock);
		mutex_unlock(&bus->lock);
		if (ret)
			return ret;
	} else if (bus->is_exynos7420_int) {
		/* Vendor exynos7_bus_int: INT 560 MHz strictly requires MIF >= 632 MHz */
		if (exynos7420_hw.mif_bus) {
			mutex_lock(&exynos7420_hw.mif_bus->lock);
			mutex_lock(&exynos7420_hw.lock);
			if (exynos7420_hw.mif_bus->curr_freq < 1552000000UL) {
				ret = exynos7420_mif_target_locked(exynos7420_hw.mif_bus, 1552000000UL);
				if (ret) {
					mutex_unlock(&exynos7420_hw.lock);
					mutex_unlock(&exynos7420_hw.mif_bus->lock);
					return ret;
				}
			}
			mutex_unlock(&exynos7420_hw.lock);
			mutex_unlock(&exynos7420_hw.mif_bus->lock);
		}
		mutex_lock(&bus->lock);
		mutex_lock(&exynos7420_hw.lock);
		ret = exynos7420_int_target_locked(bus, 560000000UL);
		mutex_unlock(&exynos7420_hw.lock);
		mutex_unlock(&bus->lock);
		if (ret)
			return ret;
	}

	return 0;
}
#endif

static const struct dev_pm_ops exynos_bus_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(exynos_bus_suspend, exynos_bus_resume)
};

static const struct of_device_id exynos_bus_of_match[] = {
	{ .compatible = "samsung,exynos-bus", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, exynos_bus_of_match);

static struct platform_driver exynos_bus_platdrv = {
	.probe		= exynos_bus_probe,
	.shutdown	= exynos_bus_shutdown,
	.driver = {
		.name	= "exynos-bus",
		.pm	= &exynos_bus_pm,
		.of_match_table = exynos_bus_of_match,
	},
};
module_platform_driver(exynos_bus_platdrv);

MODULE_SOFTDEP("pre: exynos_ppmu");
MODULE_DESCRIPTION("Generic Exynos Bus frequency driver with PM QoS aggregation");
MODULE_AUTHOR("Chanwoo Choi <cw00.choi@samsung.com>");
MODULE_LICENSE("GPL v2");
