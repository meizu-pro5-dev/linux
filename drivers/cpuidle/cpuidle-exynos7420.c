// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 7420 CPUIdle Driver for Linux 6.6
 *
 * Copyright (C) 2026 404698-FDU <24302010006@m.fudan.edu.cn>
 *
 * Provides native C-state management for Exynos 7420 big.LITTLE:
 * - State 0 (C1): Core Clock Gating (WFI)
 * - State 1 (C2): Local Core Power Gating (PMU Central Sequencer cut-off + PSCI 0.1)
 */

#define pr_fmt(fmt) "exynos7420-cpuidle: " fmt

#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/arm-smccc.h>
#include <asm/cpuidle.h>
#include <asm/suspend.h>

#define EXYNOS_PMU_BASE_DEFAULT		0x105C0000
#define EXYNOS_PMU_SIZE			0x5000
#define EXYNOS_PMU_ATLAS_CPU0_CONFIG	0x2000
#define EXYNOS_PMU_CPU_STRIDE		0x80
#define EXYNOS_PMU_LOCAL_PWR_CFG	0x0F

#define PSCI_0_1_FN_CPU_SUSPEND		0xC4000001

static void __iomem *pmu_base;
static struct cpuidle_device __percpu *exynos7420_cpuidle_devices;
static enum cpuhp_state exynos7420_hp_state;

static inline u32 exynos7420_pmu_cpu_config_offset(unsigned int cpu)
{
	/*
	 * Apollo: Linux CPU 0..3 -> Cluster 1 (phys 4..7) -> 0x2200 + 0x80 * cpu
	 * Atlas:  Linux CPU 4..7 -> Cluster 0 (phys 0..3) -> 0x2000 + 0x80 * (cpu - 4)
	 */
	if (cpu < 4)
		return EXYNOS_PMU_ATLAS_CPU0_CONFIG + EXYNOS_PMU_CPU_STRIDE * (cpu + 4);
	else
		return EXYNOS_PMU_ATLAS_CPU0_CONFIG + EXYNOS_PMU_CPU_STRIDE * (cpu - 4);
}

static int exynos7420_enter_wfi(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	cpu_do_idle();
	return index;
}

static int exynos7420_c2_finisher(unsigned long arg)
{
	struct arm_smccc_res res;

	/*
	 * Call Samsung legacy PSCI 0.1 CPU_SUSPEND (0xC4000001):
	 * - arg0 = 0 (power_state parameter aligned with vendor DTS C2)
	 * - arg1 = physical resume entry address of cpu_resume
	 */
	arm_smccc_smc(PSCI_0_1_FN_CPU_SUSPEND, 0,
		      __pa_symbol_nodebug((unsigned long)cpu_resume),
		      0, 0, 0, 0, 0, &res);

	return res.a0;
}

static int exynos7420_enter_c2(struct cpuidle_device *dev,
			       struct cpuidle_driver *drv, int index)
{
	unsigned int cpu = dev->cpu;
	u32 cfg_off = exynos7420_pmu_cpu_config_offset(cpu);
	u32 val;
	int ret;

	/* 1. Save local CPU peripheral state (GIC Interface and local timers) */
	ret = cpu_pm_enter();
	if (ret)
		return drv->safe_state_index;

	/* 2. Clear LOCAL_PWR_CFG (0x0F) in PMU to signal power cut-off to Central Sequencer */
	val = readl_relaxed(pmu_base + cfg_off);
	writel_relaxed(val & ~EXYNOS_PMU_LOCAL_PWR_CFG, pmu_base + cfg_off);

	/* 3. Save CPU architectural context on stack and call SMC 0xC4000001 */
	ret = cpu_suspend(0, exynos7420_c2_finisher);

	/* 4. Unconditionally restore PMU LOCAL_PWR_CFG (0x0F) on wake or abort */
	val = readl_relaxed(pmu_base + cfg_off);
	writel_relaxed(val | EXYNOS_PMU_LOCAL_PWR_CFG, pmu_base + cfg_off);

	/* 5. Restore local CPU peripheral state */
	cpu_pm_exit();

	if (ret)
		return drv->safe_state_index;

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
		[1] = {
			.enter			= exynos7420_enter_c2,
			.enter_s2idle		= exynos7420_enter_c2,
			.exit_latency		= 100,
			.target_residency	= 1000,
			.power_usage		= UINT_MAX,
			.flags			= CPUIDLE_FLAG_TIMER_STOP,
			.name			= "C2",
			.desc			= "Exynos 7420 Core Power Gating",
		},
	},
	.safe_state_index = 0,
	.state_count = 2,
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
	struct device_node *np;
	int ret;

	if (!of_machine_is_compatible("samsung,exynos7420"))
		return -ENODEV;

	/* Map PMU register space */
	np = of_find_compatible_node(NULL, NULL, "samsung,exynos7-pmu");
	if (np) {
		pmu_base = of_iomap(np, 0);
		of_node_put(np);
	}
	if (!pmu_base) {
		pmu_base = ioremap(EXYNOS_PMU_BASE_DEFAULT, EXYNOS_PMU_SIZE);
		if (!pmu_base) {
			pr_err("Failed to map PMU registers\n");
			return -ENOMEM;
		}
	}

	exynos7420_cpuidle_devices = alloc_percpu(struct cpuidle_device);
	if (!exynos7420_cpuidle_devices) {
		iounmap(pmu_base);
		return -ENOMEM;
	}

	exynos7420_idle_driver.cpumask = (struct cpumask *)cpu_possible_mask;

	ret = cpuidle_register_driver(&exynos7420_idle_driver);
	if (ret) {
		pr_err("cpuidle driver registration failed: %d\n", ret);
		free_percpu(exynos7420_cpuidle_devices);
		exynos7420_cpuidle_devices = NULL;
		iounmap(pmu_base);
		return ret;
	}

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "cpuidle/exynos7420:online",
				exynos7420_cpu_online, exynos7420_cpu_offline);
	if (ret < 0) {
		pr_err("Failed to setup cpuhp state: %d\n", ret);
		cpuidle_unregister_driver(&exynos7420_idle_driver);
		free_percpu(exynos7420_cpuidle_devices);
		exynos7420_cpuidle_devices = NULL;
		iounmap(pmu_base);
		return ret;
	}
	exynos7420_hp_state = ret;

	pr_info("Driver registered (%d states, %u CPUs)\n",
		exynos7420_idle_driver.state_count, num_possible_cpus());
	return 0;
}
device_initcall(exynos7420_cpuidle_init);

MODULE_AUTHOR("404698-FDU <24302010006@m.fudan.edu.cn>");
MODULE_DESCRIPTION("Samsung Exynos 7420 CPUIdle Driver with C2 Support");
MODULE_LICENSE("GPL");
