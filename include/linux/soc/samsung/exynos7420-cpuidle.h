/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS7420_CPUIDLE_H
#define __LINUX_SOC_SAMSUNG_EXYNOS7420_CPUIDLE_H

#ifdef CONFIG_ARM_EXYNOS7420_CPUIDLE
void exynos7420_atlas_cpd_inhibit(void);
void exynos7420_atlas_cpd_allow(void);
bool exynos7420_atlas_cpd_inhibited(void);
#else
static inline void exynos7420_atlas_cpd_inhibit(void) { }
static inline void exynos7420_atlas_cpd_allow(void) { }
static inline bool exynos7420_atlas_cpd_inhibited(void) { return false; }
#endif

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS7420_CPUIDLE_H */
