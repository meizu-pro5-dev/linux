/* drivers/gpu/arm/.../platform/gpu_pmqos.c
 *
 * Copyright 2011 by S.LSI. Samsung Electronics Inc.
 * San#24, Nongseo-Dong, Giheung-Gu, Yongin, Korea
 *
 * Samsung SoC Mali-T Series DVFS driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

/**
 * @file gpu_pmqos.c
 * DVFS
 */

#include <mali_kbase.h>

#include <linux/pm_qos.h>

#include "mali_kbase_platform.h"
#include "gpu_dvfs_handler.h"

struct pm_qos_request exynos5_g3d_mif_min_qos;
struct pm_qos_request exynos5_g3d_mif_max_qos;
struct pm_qos_request exynos5_g3d_int_qos;
struct pm_qos_request exynos5_g3d_cpu_cluster0_min_qos;
struct pm_qos_request exynos5_g3d_cpu_cluster1_max_qos;
struct pm_qos_request exynos5_g3d_cpu_cluster1_min_qos;
static int mif_max_request = PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE;

#ifdef CONFIG_MALI_DVFS_USER
struct pm_qos_request proactive_mif_min_qos;
struct pm_qos_request proactive_int_min_qos;
struct pm_qos_request proactive_apollo_min_qos;
struct pm_qos_request proactive_atlas_min_qos;
#endif

int gpu_pm_qos_command(struct exynos_context *platform, gpu_pmqos_state state)
{
	DVFS_ASSERT(platform);

	if (!platform->devfreq_status)
		return 0;

	switch (state) {
	case GPU_CONTROL_PM_QOS_INIT:
		mif_max_request = PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE;
		pm_qos_add_request(&exynos5_g3d_mif_min_qos, PM_QOS_BUS_THROUGHPUT, 0);
		if (platform->pmqos_mif_max_clock)
			pm_qos_add_request(&exynos5_g3d_mif_max_qos, PM_QOS_BUS_THROUGHPUT_MAX, PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE);
		if (!platform->pmqos_int_disable)
			pm_qos_add_request(&exynos5_g3d_int_qos, PM_QOS_DEVICE_THROUGHPUT, 0);
		pm_qos_add_request(&exynos5_g3d_cpu_cluster0_min_qos, PM_QOS_CLUSTER0_FREQ_MIN, 0);
		pm_qos_add_request(&exynos5_g3d_cpu_cluster1_max_qos, PM_QOS_CLUSTER1_FREQ_MAX, PM_QOS_CLUSTER1_FREQ_MAX_DEFAULT_VALUE);
		if (platform->boost_egl_min_lock)
			pm_qos_add_request(&exynos5_g3d_cpu_cluster1_min_qos, PM_QOS_CLUSTER1_FREQ_MIN, 0);
		break;
	case GPU_CONTROL_PM_QOS_DEINIT:
		pm_qos_remove_request(&exynos5_g3d_mif_min_qos);
		if (platform->pmqos_mif_max_clock)
			pm_qos_remove_request(&exynos5_g3d_mif_max_qos);
		if (!platform->pmqos_int_disable)
			pm_qos_remove_request(&exynos5_g3d_int_qos);
		pm_qos_remove_request(&exynos5_g3d_cpu_cluster0_min_qos);
		pm_qos_remove_request(&exynos5_g3d_cpu_cluster1_max_qos);
		if (platform->boost_egl_min_lock)
			pm_qos_remove_request(&exynos5_g3d_cpu_cluster1_min_qos);
		break;
	case GPU_CONTROL_PM_QOS_SET:
		KBASE_DEBUG_ASSERT(platform->step >= 0);
		gpu_mif_pmqos(platform, platform->table[platform->step].mem_freq);
		if (!platform->pmqos_int_disable)
			pm_qos_update_request(&exynos5_g3d_int_qos, platform->table[platform->step].int_freq);
		pm_qos_update_request(&exynos5_g3d_cpu_cluster0_min_qos, platform->table[platform->step].cpu_freq);
		if (!platform->boost_is_enabled)
			pm_qos_update_request(&exynos5_g3d_cpu_cluster1_max_qos, platform->table[platform->step].cpu_max_freq);
		break;
	case GPU_CONTROL_PM_QOS_RESET:
		pm_qos_update_request(&exynos5_g3d_mif_min_qos, 0);
		mif_max_request = PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE;
		if (platform->pmqos_mif_max_clock)
			pm_qos_update_request(&exynos5_g3d_mif_max_qos, PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE);
		if (!platform->pmqos_int_disable)
			pm_qos_update_request(&exynos5_g3d_int_qos, 0);
		pm_qos_update_request(&exynos5_g3d_cpu_cluster0_min_qos, 0);
		pm_qos_update_request(&exynos5_g3d_cpu_cluster1_max_qos, PM_QOS_CLUSTER1_FREQ_MAX_DEFAULT_VALUE);
		break;
	case GPU_CONTROL_PM_QOS_EGL_SET:
		pm_qos_update_request(&exynos5_g3d_cpu_cluster1_min_qos, platform->boost_egl_min_lock);
		break;
	case GPU_CONTROL_PM_QOS_EGL_RESET:
		pm_qos_update_request(&exynos5_g3d_cpu_cluster1_min_qos, 0);
		break;
	default:
		break;
	}

	return 0;
}

int gpu_mif_pmqos(struct exynos_context *platform, int mem_freq)
{
	int ceiling = PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE;
	DVFS_ASSERT(platform);

	if(!platform->devfreq_status)
		return 0;
	if (platform->pmqos_mif_max_clock && platform->step >= 0 &&
	    platform->table[platform->step].clock >= platform->pmqos_mif_max_clock_base)
		ceiling = platform->pmqos_mif_max_clock;
	/* Preserve the high-GPU protective cap without asking for a higher
	 * MIF floor ourselves. Release it when the GPU leaves that range.
	 * Order updates so transitions do not introduce a min > max window.
	 */
	mem_freq = min(mem_freq, ceiling);
	if (ceiling < mif_max_request)
		pm_qos_update_request(&exynos5_g3d_mif_min_qos, mem_freq);
	if (platform->pmqos_mif_max_clock)
		pm_qos_update_request(&exynos5_g3d_mif_max_qos, ceiling);
	if (ceiling >= mif_max_request)
		pm_qos_update_request(&exynos5_g3d_mif_min_qos, mem_freq);
	mif_max_request = ceiling;

	return 0;
}

#ifdef CONFIG_MALI_DVFS_USER
int proactive_pm_qos_command(struct exynos_context *platform, gpu_pmqos_state state)
{
	DVFS_ASSERT(platform);

	if (!platform->devfreq_status)
		return 0;

	switch (state) {
		case GPU_CONTROL_PM_QOS_INIT:
			pm_qos_add_request(&proactive_mif_min_qos, PM_QOS_BUS_THROUGHPUT, 0);
			pm_qos_add_request(&proactive_apollo_min_qos, PM_QOS_CLUSTER0_FREQ_MIN, 0);
			pm_qos_add_request(&proactive_atlas_min_qos, PM_QOS_CLUSTER1_FREQ_MIN, 0);
			if (!platform->pmqos_int_disable)
				pm_qos_add_request(&proactive_int_min_qos, PM_QOS_DEVICE_THROUGHPUT, 0);

#ifdef CONFIG_PWRCAL
			update_cal_table();
#endif
			break;
		case GPU_CONTROL_PM_QOS_DEINIT:
			pm_qos_remove_request(&proactive_mif_min_qos);
			pm_qos_remove_request(&proactive_apollo_min_qos);
			pm_qos_remove_request(&proactive_atlas_min_qos);
			if (!platform->pmqos_int_disable)
				pm_qos_remove_request(&proactive_int_min_qos);
			break;
		case GPU_CONTROL_PM_QOS_RESET:
			pm_qos_update_request(&proactive_mif_min_qos, 0);
			pm_qos_update_request(&proactive_apollo_min_qos, 0);
			pm_qos_update_request(&proactive_atlas_min_qos, 0);
		default:
			break;
	}

	return 0;
}

int gpu_mif_min_pmqos(struct exynos_context *platform, int mif_step)
{
	DVFS_ASSERT(platform);

	if(!platform->devfreq_status)
		return 0;

	pm_qos_update_request_timeout(&proactive_mif_min_qos, platform->mif_table[mif_step], 30000);

	return 0;
}

int gpu_int_min_pmqos(struct exynos_context *platform, int int_step)
{
	DVFS_ASSERT(platform);

	if(!platform->devfreq_status)
		return 0;

	pm_qos_update_request_timeout(&proactive_int_min_qos, platform->int_table[int_step], 30000);

	return 0;
}

int gpu_apollo_min_pmqos(struct exynos_context *platform, int apollo_step)
{
	DVFS_ASSERT(platform);

	if(!platform->devfreq_status)
		return 0;

	pm_qos_update_request_timeout(&proactive_apollo_min_qos, platform->apollo_table[apollo_step], 30000);

	return 0;
}

int gpu_atlas_min_pmqos(struct exynos_context *platform, int atlas_step)
{
	DVFS_ASSERT(platform);

	if(!platform->devfreq_status)
		return 0;

	pm_qos_update_request_timeout(&proactive_atlas_min_qos, platform->atlas_table[atlas_step], 30000);

	return 0;
}
#endif
