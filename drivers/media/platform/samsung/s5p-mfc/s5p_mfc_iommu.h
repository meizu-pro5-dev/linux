/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2015 Samsung Electronics Co.Ltd
 * Authors: Marek Szyprowski <m.szyprowski@samsung.com>
 */

#ifndef S5P_MFC_IOMMU_H_
#define S5P_MFC_IOMMU_H_

/* IOVA of the firmware block, the lowest MFC buffer address (MFC v9+) */
#define S5P_MFC_IOMMU_FW_BASE	0x20000000

#if defined(CONFIG_EXYNOS_IOMMU)

#include <linux/iommu.h>

static inline bool exynos_is_iommu_available(struct device *dev)
{
	return dev_iommu_priv_get(dev) != NULL;
}

void exynos_iommu_master_ready(struct device *master);

#else

static inline bool exynos_is_iommu_available(struct device *dev)
{
	return false;
}

static inline void exynos_iommu_master_ready(struct device *master) { }

#endif

#endif /* S5P_MFC_IOMMU_H_ */
