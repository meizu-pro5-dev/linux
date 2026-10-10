/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_M86_BUS_QOS_H
#define _LINUX_M86_BUS_QOS_H

#include <linux/device.h>
#include <linux/pm_qos.h>

struct device *exynos_bus_get_dev(const char *name);

static inline int exynos_bus_qos_add_request(struct device *bus_dev,
					     struct dev_pm_qos_request *req,
					     u32 freq_khz)
{
	if (!bus_dev || !req)
		return -EINVAL;
	return dev_pm_qos_add_request(bus_dev, req, DEV_PM_QOS_MIN_FREQUENCY, freq_khz);
}

static inline int exynos_bus_qos_update_request(struct dev_pm_qos_request *req,
						u32 freq_khz)
{
	if (!req || !dev_pm_qos_request_active(req))
		return -EINVAL;
	return dev_pm_qos_update_request(req, freq_khz);
}

static inline void exynos_bus_qos_remove_request(struct dev_pm_qos_request *req)
{
	if (req && dev_pm_qos_request_active(req))
		dev_pm_qos_remove_request(req);
}

#endif /* _LINUX_M86_BUS_QOS_H */
