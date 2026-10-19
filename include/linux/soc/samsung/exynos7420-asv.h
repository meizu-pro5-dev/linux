/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS7420_ASV_H
#define __LINUX_SOC_SAMSUNG_EXYNOS7420_ASV_H

enum exynos7420_asv_domain {
	EXYNOS7420_ASV_BIG = 0,
	EXYNOS7420_ASV_LIT = 1,
	EXYNOS7420_ASV_G3D = 2,
	EXYNOS7420_ASV_MIF = 3,
	EXYNOS7420_ASV_INT = 4,
	EXYNOS7420_ASV_DOMAINS_NR,
};

int exynos7420_asv_get_group(enum exynos7420_asv_domain dom, unsigned int level);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS7420_ASV_H */
