/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __EXYNOS7420_INT_MIF_H
#define __EXYNOS7420_INT_MIF_H

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct devfreq;
struct devfreq_event_dev;
struct platform_device;
struct regulator;

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

extern struct device_attribute dev_attr_hw_status;

int exynos7420_hw_bus_init_once(struct device *dev);
int exynos7420_mif_parse_clocks(struct device *dev);
int exynos7420_int_parse_clocks(struct device *dev);
void exynos7420_adjust_opp_voltages(struct device *dev, bool is_mif);
void exynos7420_register_bus(struct exynos_bus *bus);
void exynos7420_unregister_bus(struct exynos_bus *bus);
int exynos7420_mif_target(struct exynos_bus *bus, unsigned long freq_hz);
int exynos7420_int_target(struct exynos_bus *bus, unsigned long freq_hz);
void exynos7420_resume_sync(struct exynos_bus *bus);
int exynos7420_suspend_clamp(struct exynos_bus *bus);

#endif /* __EXYNOS7420_INT_MIF_H */
