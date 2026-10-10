// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 7420 CPUIdle Driver for Linux 6.6
 *
 * Copyright (C) 2026 404698-FDU <24302010006@m.fudan.edu.cn>
 *
 * Provides native C-state management (C1/WFI) for Exynos 7420 big.LITTLE
 * (Cortex-A57/A53) on platforms with legacy PSCI 0.1 firmware where PSCI
 * CPU_SUSPEND is unavailable.
 */

#define pr_fmt(fmt) "exynos7420-cpuidle: " fmt

#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <asm/cpuidle.h>

static struct cpuidle_device __percpu *exynos7420_cpuidle_devices;
static enum cpuhp_state exynos7420_hp_state;

static int exynos7420_enter_wfi(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	cpu_do_idle();
	return index;
}

static struct cpuidle_driver exynos7420_idle_driver = {
	.name = "exynos7420_idle",
	.owner = THIS_MODULE,
	.states = {
		[0] = {
			.enter			= exynos7420_enter_wfi,
			.enter_s2idle		= exynos7420_enter_wfi,
			.exit_latency		= 1,
			.target_residency	= 2,
			.power_usage		= UINT_MAX,
			.flags			= CPUIDLE_FLAG_NONE,
			.name			= "WFI",
			.desc			= "ARM64 Core Clock Gating (WFI)",
		},
	},
	.safe_state_index = 0,
	.state_count = 1,
};

static int exynos7420_cpu_online(unsigned int cpu)
{
	struct cpuidle_device *dev;

	if (!exynos7420_cpuidle_devices)
		return 0;

	dev = per_cpu_ptr(exynos7420_cpuidle_devices, cpu);
	if (!dev->registered) {
		dev->cpu = cpu;
		if (cpuidle_register_device(dev)) {
			pr_notice("Failed to register cpuidle device for CPU %u\n", cpu);
			return -EIO;
		}
	}

	return 0;
}

static int exynos7420_cpu_offline(unsigned int cpu)
{
	struct cpuidle_device *dev;

	if (!exynos7420_cpuidle_devices)
		return 0;

	dev = per_cpu_ptr(exynos7420_cpuidle_devices, cpu);
	if (dev->registered)
		cpuidle_unregister_device(dev);

	return 0;
}

static int __init exynos7420_cpuidle_init(void)
{
	int ret;

	if (!of_machine_is_compatible("samsung,exynos7420"))
		return -ENODEV;

	exynos7420_cpuidle_devices = alloc_percpu(struct cpuidle_device);
	if (!exynos7420_cpuidle_devices)
		return -ENOMEM;

	exynos7420_idle_driver.cpumask = (struct cpumask *)cpu_possible_mask;

	ret = cpuidle_register_driver(&exynos7420_idle_driver);
	if (ret) {
		pr_err("cpuidle driver registration failed: %d\n", ret);
		free_percpu(exynos7420_cpuidle_devices);
		exynos7420_cpuidle_devices = NULL;
		return ret;
	}

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "cpuidle/exynos7420:online",
				exynos7420_cpu_online, exynos7420_cpu_offline);
	if (ret < 0) {
		pr_err("Failed to setup cpuhp state: %d\n", ret);
		cpuidle_unregister_driver(&exynos7420_idle_driver);
		free_percpu(exynos7420_cpuidle_devices);
		exynos7420_cpuidle_devices = NULL;
		return ret;
	}
	exynos7420_hp_state = ret;

	pr_info("Driver registered (%d states, %u CPUs)\n",
		exynos7420_idle_driver.state_count, num_possible_cpus());
	return 0;
}
device_initcall(exynos7420_cpuidle_init);

MODULE_AUTHOR("404698-FDU <24302010006@m.fudan.edu.cn>");
MODULE_DESCRIPTION("Samsung Exynos 7420 CPUIdle Driver");
MODULE_LICENSE("GPL");
