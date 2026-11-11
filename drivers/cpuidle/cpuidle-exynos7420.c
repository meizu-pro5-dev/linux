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
#include <linux/cpumask.h>
#include <linux/cpu_pm.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tick.h>
#include <linux/arm-smccc.h>
#include <asm/cpuidle.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>
#include <asm/tlbflush.h>

#define EXYNOS_PMU_BASE_DEFAULT		0x105C0000
#define EXYNOS_PMU_SIZE			0x5000
#define EXYNOS_PMU_UP_SCHEDULER		0x0120
#define EXYNOS_PMU_ATLAS_CPU0_CONFIG	0x2000
#define EXYNOS_PMU_CPU_STRIDE		0x80
#define EXYNOS_PMU_LOCAL_PWR_CFG	0x0F

/* CMU_ATLAS / CMU_APOLLO & LPI_MASK registers (3.10 regs-clock-exynos7420.h & pmu_cal_sys_exynos7420.h) */
#define EXYNOS7420_ATLAS_STOPCTRL_PA		0x11801000
#define EXYNOS7420_ATLAS_PWR_CTRL_PA		0x11801020
#define EXYNOS7420_APOLLO_STOPCTRL_PA		0x11901000
#define EXYNOS7420_APOLLO_PWR_CTRL_PA		0x11901020
#define EXYNOS7420_LPI_MASK_ATLAS_PA		0x11820020
#define EXYNOS7420_USE_L2QACTIVE		BIT(14)
#define EXYNOS7420_ASATBZ_MSTIF_ATLAS_CAM1	BIT(4)
#define EXYNOS7420_ASATBZ_MSTIF_ATLAS_AUD	BIT(5)

/* PMU CPU / Cluster / Sequencer / TOP offsets (3.10 pmu_cal_sys_exynos7420.h & regs-pmu-exynos7420.h) */
#define EXYNOS_PMU_ATLAS_CPU0_OPTION		0x2008
#define EXYNOS_PMU_ATLAS_CPU0_DURATION0		0x2010
#define EXYNOS_PMU_APOLLO_CPU0_OPTION		0x2208
#define EXYNOS_PMU_ATLAS_NONCPU_OPTION		0x2408
#define EXYNOS_PMU_APOLLO_NONCPU_OPTION		0x2428
#define EXYNOS_PMU_ATLAS_DBG_CONFIGURATION	0x2440
#define EXYNOS_PMU_ATLAS_CPUSEQUENCER_OPTION	0x2488
#define EXYNOS_PMU_ATLAS_L2_OPTION		0x2608
#define EXYNOS_PMU_APOLLO_L2_OPTION		0x2628
#define EXYNOS_PMU_TOP_PWR_OPTION		0x2C48
#define EXYNOS_PMU_TOP_PWR_MIF_OPTION		0x2CC8
#define EXYNOS_PMU_XXTI_DURATION3		0x343C
#define EXYNOS_PMU_EXT_REGULATOR_DURATION3	0x361C
#define EXYNOS_PMU_ATLAS_BLK_STATUS		0x3A04

/* PMU option & duration bit definitions (3.10 pmu_cal_sys_exynos7420.c & pmu.c) */
#define EXYNOS_PMU_USE_SC_COUNTER		BIT(0)
#define EXYNOS_PMU_USE_SC_FEEDBACK		BIT(1)
#define EXYNOS_PMU_USE_MEMPWRDOWN		BIT(2)
#define EXYNOS_PMU_USE_STANDBYWFI		BIT(16)
#define EXYNOS_PMU_USE_STANDBYWFE		BIT(24)
#define EXYNOS_PMU_USE_SMPEN			BIT(28)
#define EXYNOS_PMU_DUR_WAIT_RESET		(0xFU << 20)
#define EXYNOS_PMU_DUR_SCALL_MASK		(0xFU << 4)
#define EXYNOS_PMU_DUR_SCALL_VAL		(0x1U << 4)
#define EXYNOS_PMU_USE_RETENTION		BIT(4)
#define EXYNOS_PMU_USE_STANDBYWFIL2		BIT(16)
#define EXYNOS_PMU_USE_AUTOMATIC_L2FLUSHREQ	BIT(17)
#define EXYNOS_PMU_DBG_INITIATE_WAKEUP		(0x3U << 16)
#define EXYNOS_PMU_USE_AUTOMATIC_PWRCTRL	BIT(0)
#define EXYNOS_PMU_SKIP_BLK_PWR_DOWN		BIT(8)
#define EXYNOS_PMU_ENABLE_ATLAS_CPU		BIT(0)
#define EXYNOS_PMU_ENABLE_APOLLO_CPU		BIT(1)
#define EXYNOS_PMU_DUR_STABLE_MASK		0xFFFFFU
#define EXYNOS_PMU_XXTI_DUR_STABLE		0x1D4CU
#define EXYNOS_PMU_EXTREG_DUR_STABLE		0x3FFFU

/* PMU offsets for read-only boot telemetry */
#define EXYNOS_PMU_AUD_STATUS			0x4004
#define EXYNOS_PMU_CAM0_STATUS			0x4044
#define EXYNOS_PMU_CAM1_STATUS			0x4064
#define EXYNOS_PMU_DISP_STATUS			0x4084
#define EXYNOS_PMU_G3D_STATUS			0x4104
#define EXYNOS_PMU_ISP0_STATUS			0x4144
#define EXYNOS_PMU_ISP1_STATUS			0x4184
#define EXYNOS_PMU_MFC_STATUS			0x41A4
#define EXYNOS_PMU_MSCL_STATUS			0x41C4
#define EXYNOS_PMU_VPP_STATUS			0x41E4

#define PSCI_0_1_FN_CPU_SUSPEND		0xC4000001
#define EXYNOS7420_PSCI_STATE_C2	0x00000000UL
#define EXYNOS7420_PSCI_STATE_CPD	0x01000000UL
#define EXYNOS7420_CPD_RESIDENCY_NS	(3000ULL * NSEC_PER_USEC)

static void __iomem *pmu_base;
static void __iomem *atlas_pwr_ctrl;
static void __iomem *apollo_pwr_ctrl;
static struct cpuidle_device __percpu *exynos7420_cpuidle_devices;
static enum cpuhp_state exynos7420_hp_state;
static struct cpumask exynos7420_apollo_cpumask;
static struct cpumask exynos7420_atlas_cpumask;
static struct cpumask exynos7420_atlas_c2_mask;
static DEFINE_RAW_SPINLOCK(exynos7420_cpd_lock);
static u64 exynos7420_atlas_cpd_gen;

static bool exynos7420_any_atlas_online(void)
{
	unsigned int cpu;

	for_each_online_cpu(cpu) {
		u64 mpidr = cpu_logical_map(cpu);

		if (MPIDR_AFFINITY_LEVEL(mpidr, 1) == 0)
			return true;
	}
	return false;
}

/*
 * 1:1 Exynos 7420 CPU/Cluster/Sequencer PMU hardware initialization from vendor 3.10:
 * - arch/arm64/mach-exynos/pmu.c:221-240 (exynos7420_pmu_init, exynos7420_atlas_dbg_pmu_init)
 * - arch/arm64/mach-exynos/pmu_cal_sys_exynos7420.c:378-535 (pmu_cal_sys_init)
 * - arch/arm64/mach-exynos/exynos-powermode.c:48-63 (exynos_idle_clock_down)
 *
 * Peripheral power domain OPTION registers (0x4008..0x41E8) are intentionally left to
 * drivers/pmdomain/samsung/exynos-pm-domains.c to preserve single-owner locking.
 */
static void __init exynos7420_init_pmu(void)
{
	static const u32 feed_offs[] = {
		EXYNOS_PMU_ATLAS_NONCPU_OPTION,
		EXYNOS_PMU_APOLLO_NONCPU_OPTION,
		EXYNOS_PMU_TOP_PWR_OPTION,
		EXYNOS_PMU_TOP_PWR_MIF_OPTION,
	};
	void __iomem *atlas_stopctrl, *apollo_stopctrl, *atlas_lpi_mask;
	unsigned int i;
	u32 tmp, lpi_val = 0;

	/* 1. Enable ATLAS_DBG wakeup so ATLAS_CPUSEQUENCER does not deadlock at STAT=0x80000 */
	tmp = readl_relaxed(pmu_base + EXYNOS_PMU_ATLAS_DBG_CONFIGURATION);
	tmp |= EXYNOS_PMU_DBG_INITIATE_WAKEUP;
	writel_relaxed(tmp, pmu_base + EXYNOS_PMU_ATLAS_DBG_CONFIGURATION);

	/* 2. Preserve BLK_ATLAS / CMU_ATLAS across automatic cluster power-down */
	tmp = readl_relaxed(pmu_base + EXYNOS_PMU_ATLAS_CPUSEQUENCER_OPTION);
	tmp |= EXYNOS_PMU_SKIP_BLK_PWR_DOWN;
	writel_relaxed(tmp, pmu_base + EXYNOS_PMU_ATLAS_CPUSEQUENCER_OPTION);

	/* 3. Use SC_FEEDBACK on NONCPU and TOP sequencers */
	for (i = 0; i < ARRAY_SIZE(feed_offs); i++) {
		tmp = readl_relaxed(pmu_base + feed_offs[i]);
		tmp &= ~EXYNOS_PMU_USE_SC_COUNTER;
		tmp |= EXYNOS_PMU_USE_SC_FEEDBACK;
		writel_relaxed(tmp, pmu_base + feed_offs[i]);
	}

	/* 4. L2 options: disable automatic L2 flush & retention, enable STANDBYWFIL2 */
	for (i = 0; i < 2; i++) {
		u32 off = EXYNOS_PMU_ATLAS_L2_OPTION + i * 0x20;

		tmp = readl_relaxed(pmu_base + off);
		tmp &= ~(EXYNOS_PMU_USE_AUTOMATIC_L2FLUSHREQ | EXYNOS_PMU_USE_RETENTION);
		tmp |= EXYNOS_PMU_USE_STANDBYWFIL2;
		writel_relaxed(tmp, pmu_base + off);
	}

	/* 5. Per-core options & reset/scall durations (phys 0..3 = Atlas, phys 4..7 = Apollo) */
	for (i = 0; i < 8; i++) {
		u32 opt_off = EXYNOS_PMU_ATLAS_CPU0_OPTION + i * EXYNOS_PMU_CPU_STRIDE;
		u32 dur_off = EXYNOS_PMU_ATLAS_CPU0_DURATION0 + i * EXYNOS_PMU_CPU_STRIDE;

		tmp = readl_relaxed(pmu_base + opt_off);
		if (i < 4)
			tmp |= EXYNOS_PMU_USE_MEMPWRDOWN;
		tmp |= EXYNOS_PMU_USE_SC_FEEDBACK | EXYNOS_PMU_USE_SC_COUNTER |
		       EXYNOS_PMU_USE_SMPEN | EXYNOS_PMU_USE_STANDBYWFI;
		tmp &= ~EXYNOS_PMU_USE_STANDBYWFE;
		writel_relaxed(tmp, pmu_base + opt_off);

		tmp = readl_relaxed(pmu_base + dur_off);
		tmp |= EXYNOS_PMU_DUR_WAIT_RESET;
		tmp &= ~EXYNOS_PMU_DUR_SCALL_MASK;
		tmp |= EXYNOS_PMU_DUR_SCALL_VAL;
		writel_relaxed(tmp, pmu_base + dur_off);
	}

	/* 6. Limit in-rush current on local CPU power-up and set stabilization durations */
	tmp = readl_relaxed(pmu_base + EXYNOS_PMU_UP_SCHEDULER);
	tmp |= EXYNOS_PMU_ENABLE_ATLAS_CPU | EXYNOS_PMU_ENABLE_APOLLO_CPU;
	writel_relaxed(tmp, pmu_base + EXYNOS_PMU_UP_SCHEDULER);

	tmp = readl_relaxed(pmu_base + EXYNOS_PMU_XXTI_DURATION3);
	tmp = (tmp & ~EXYNOS_PMU_DUR_STABLE_MASK) | EXYNOS_PMU_XXTI_DUR_STABLE;
	writel_relaxed(tmp, pmu_base + EXYNOS_PMU_XXTI_DURATION3);

	tmp = readl_relaxed(pmu_base + EXYNOS_PMU_EXT_REGULATOR_DURATION3);
	tmp = (tmp & ~EXYNOS_PMU_DUR_STABLE_MASK) | EXYNOS_PMU_EXTREG_DUR_STABLE;
	writel_relaxed(tmp, pmu_base + EXYNOS_PMU_EXT_REGULATOR_DURATION3);

	/* 7. Configure CMU STOPCTRL, PWR_CTRL (L2QACTIVE), and BLK_ATLAS LPI mask */
	apollo_stopctrl = ioremap(EXYNOS7420_APOLLO_STOPCTRL_PA, 4);
	if (apollo_stopctrl) {
		writel_relaxed(0x0, apollo_stopctrl);
		iounmap(apollo_stopctrl);
	}
	if (apollo_pwr_ctrl)
		writel_relaxed(EXYNOS7420_USE_L2QACTIVE, apollo_pwr_ctrl);

	atlas_stopctrl = ioremap(EXYNOS7420_ATLAS_STOPCTRL_PA, 4);
	atlas_lpi_mask = ioremap(EXYNOS7420_LPI_MASK_ATLAS_PA, 4);

	cpus_read_lock();
	if (exynos7420_any_atlas_online() &&
	    ((readl_relaxed(pmu_base + EXYNOS_PMU_ATLAS_BLK_STATUS) & 0xf) == 0xf)) {
		if (atlas_stopctrl)
			writel_relaxed(0x0, atlas_stopctrl);
		if (atlas_pwr_ctrl)
			writel_relaxed(EXYNOS7420_USE_L2QACTIVE, atlas_pwr_ctrl);
		if (atlas_lpi_mask) {
			tmp = readl_relaxed(atlas_lpi_mask);
			tmp |= EXYNOS7420_ASATBZ_MSTIF_ATLAS_AUD |
			       EXYNOS7420_ASATBZ_MSTIF_ATLAS_CAM1;
			writel_relaxed(tmp, atlas_lpi_mask);
			lpi_val = readl_relaxed(atlas_lpi_mask);
		}
	}
	cpus_read_unlock();

	if (atlas_stopctrl)
		iounmap(atlas_stopctrl);
	if (atlas_lpi_mask)
		iounmap(atlas_lpi_mask);

	pr_info("PMU options: ATLAS_DBG_CFG=%#x ATLAS_CPU0_OPT=%#x APOLLO_CPU0_OPT=%#x ATLAS_SEQ_OPT=%#x ATLAS_L2_OPT=%#x APOLLO_L2_OPT=%#x LPI_MASK_ATLAS=%#x\n",
		readl_relaxed(pmu_base + EXYNOS_PMU_ATLAS_DBG_CONFIGURATION),
		readl_relaxed(pmu_base + EXYNOS_PMU_ATLAS_CPU0_OPTION),
		readl_relaxed(pmu_base + EXYNOS_PMU_APOLLO_CPU0_OPTION),
		readl_relaxed(pmu_base + EXYNOS_PMU_ATLAS_CPUSEQUENCER_OPTION),
		readl_relaxed(pmu_base + EXYNOS_PMU_ATLAS_L2_OPTION),
		readl_relaxed(pmu_base + EXYNOS_PMU_APOLLO_L2_OPTION),
		lpi_val);

	pr_info("PMU block status: AUD=%#x CAM0=%#x CAM1=%#x DISP=%#x G3D=%#x ISP0=%#x ISP1=%#x MFC=%#x MSCL=%#x VPP=%#x\n",
		readl_relaxed(pmu_base + EXYNOS_PMU_AUD_STATUS) & 0xf,
		readl_relaxed(pmu_base + EXYNOS_PMU_CAM0_STATUS) & 0xf,
		readl_relaxed(pmu_base + EXYNOS_PMU_CAM1_STATUS) & 0xf,
		readl_relaxed(pmu_base + EXYNOS_PMU_DISP_STATUS) & 0xf,
		readl_relaxed(pmu_base + EXYNOS_PMU_G3D_STATUS) & 0xf,
		readl_relaxed(pmu_base + EXYNOS_PMU_ISP0_STATUS) & 0xf,
		readl_relaxed(pmu_base + EXYNOS_PMU_ISP1_STATUS) & 0xf,
		readl_relaxed(pmu_base + EXYNOS_PMU_MFC_STATUS) & 0xf,
		readl_relaxed(pmu_base + EXYNOS_PMU_MSCL_STATUS) & 0xf,
		readl_relaxed(pmu_base + EXYNOS_PMU_VPP_STATUS) & 0xf);
}

static inline u32 exynos7420_pmu_cpu_config_offset(unsigned int cpu)
{
	u64 mpidr = cpu_logical_map(cpu);
	unsigned int cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	unsigned int core = MPIDR_AFFINITY_LEVEL(mpidr, 0);

	/*
	 * Atlas:  MPIDR Aff1=0, Aff0=0..3 -> phys 0..3 -> 0x2000 + 0x80 * core
	 * Apollo: MPIDR Aff1=1, Aff0=0..3 -> phys 4..7 -> 0x2200 + 0x80 * core
	 */
	return EXYNOS_PMU_ATLAS_CPU0_CONFIG +
	       EXYNOS_PMU_CPU_STRIDE * ((cluster ? 4 : 0) + (core & 3));
}

static __cpuidle int exynos7420_enter_wfi(struct cpuidle_device *dev,
					  struct cpuidle_driver *drv, int index)
{
	cpu_do_idle();
	return index;
}

static int exynos7420_psci_finisher(unsigned long psci_state)
{
	struct arm_smccc_res res;

	/*
	 * Call Samsung legacy PSCI 0.1 CPU_SUSPEND (0xC4000001):
	 * - arg0 = 0x00000000 for single-core C2, 0x01000000 for Atlas CPD
	 *          (vendor exynos7420-codegen.dtsi CLUSTER0_C2 / CLUSTER0_LPM)
	 * - arg1 = physical resume entry address of cpu_resume
	 */
	arm_smccc_smc(PSCI_0_1_FN_CPU_SUSPEND, psci_state,
		      __pa_symbol_nodebug((unsigned long)cpu_resume),
		      0, 0, 0, 0, 0, &res);

	return res.a0;
}

static __cpuidle int exynos7420_enter_c2(struct cpuidle_device *dev,
					 struct cpuidle_driver *drv, int index)
{
	unsigned int cpu = dev->cpu;
	u32 cfg_off = exynos7420_pmu_cpu_config_offset(cpu);
	u32 val;
	int ret;

	/* 1. Save local CPU peripheral state (GIC Interface and local timers) */
	ret = cpu_pm_enter();
	if (ret) {
		cpu_do_idle();
		return drv->safe_state_index;
	}

	/* 2. Clear LOCAL_PWR_CFG (0x0F) in PMU to signal power cut-off to Central Sequencer */
	val = readl_relaxed(pmu_base + cfg_off);
	writel(val & ~EXYNOS_PMU_LOCAL_PWR_CFG, pmu_base + cfg_off);

	/* 3. Save CPU architectural context on stack and call SMC 0xC4000001 */
	ret = cpu_suspend(EXYNOS7420_PSCI_STATE_C2, exynos7420_psci_finisher);
	if (ret)
		local_flush_tlb_all();

	/* 4. Unconditionally restore PMU LOCAL_PWR_CFG (0x0F) on wake or abort */
	val = readl_relaxed(pmu_base + cfg_off);
	writel(val | EXYNOS_PMU_LOCAL_PWR_CFG, pmu_base + cfg_off);

	/* 5. Restore local CPU peripheral state */
	cpu_pm_exit();

	if (ret) {
		cpu_do_idle();
		return drv->safe_state_index;
	}

	return index;
}

static bool exynos7420_all_online_atlas_in_c2(void)
{
	unsigned int c;
	bool any = false;

	for_each_cpu_and(c, &exynos7420_atlas_cpumask, cpu_online_mask) {
		any = true;
		if (!cpumask_test_cpu(c, &exynos7420_atlas_c2_mask))
			return false;
	}
	return any;
}

/*
 * Coordinated Atlas C2 / Cluster Power Down (CPD) aligned with vendor 3.10:
 * - arch/arm64/mach-exynos/exynos-powermode.c:105-130, 213-284 (c2_state_mask, cpd_residency=3000us)
 * - arch/arm64/mach-exynos/pmu.c:131-149 (exynos_cpu_sequencer_ctrl)
 * - drivers/cpuidle/cpuidle-exynos64.c:94-116
 *
 * Every Atlas core entering C2 or CPD records itself in exynos7420_atlas_c2_mask
 * so a sibling already asleep in C2 does not block cluster power-down when the
 * last online Atlas core enters idle with >= 3 ms sleep length.
 */
static __cpuidle int exynos7420_enter_atlas_idle(struct cpuidle_device *dev,
						 struct cpuidle_driver *drv,
						 bool allow_cpd)
{
	unsigned int cpu = dev->cpu;
	u32 cfg_off = exynos7420_pmu_cpu_config_offset(cpu);
	unsigned long psci_state = EXYNOS7420_PSCI_STATE_C2;
	bool triggered_cpd = false;
	u64 gen_before;
	u32 val, seq_opt;
	int ret, entered_idx;

	ret = cpu_pm_enter();
	if (ret) {
		cpu_do_idle();
		return drv->safe_state_index;
	}

	val = readl_relaxed(pmu_base + cfg_off);
	writel(val & ~EXYNOS_PMU_LOCAL_PWR_CFG, pmu_base + cfg_off);

	raw_spin_lock(&exynos7420_cpd_lock);
	cpumask_set_cpu(cpu, &exynos7420_atlas_c2_mask);
	gen_before = exynos7420_atlas_cpd_gen;
	if (allow_cpd && exynos7420_all_online_atlas_in_c2()) {
		seq_opt = readl_relaxed(pmu_base + EXYNOS_PMU_ATLAS_CPUSEQUENCER_OPTION);
		writel(seq_opt | EXYNOS_PMU_USE_AUTOMATIC_PWRCTRL,
		       pmu_base + EXYNOS_PMU_ATLAS_CPUSEQUENCER_OPTION);
		psci_state = EXYNOS7420_PSCI_STATE_CPD;
		triggered_cpd = true;
		exynos7420_atlas_cpd_gen++;
	}
	raw_spin_unlock(&exynos7420_cpd_lock);

	ret = cpu_suspend(psci_state, exynos7420_psci_finisher);
	if (ret)
		local_flush_tlb_all();

	raw_spin_lock(&exynos7420_cpd_lock);
	cpumask_clear_cpu(cpu, &exynos7420_atlas_c2_mask);
	seq_opt = readl_relaxed(pmu_base + EXYNOS_PMU_ATLAS_CPUSEQUENCER_OPTION);
	if (seq_opt & EXYNOS_PMU_USE_AUTOMATIC_PWRCTRL)
		writel(seq_opt & ~EXYNOS_PMU_USE_AUTOMATIC_PWRCTRL,
		       pmu_base + EXYNOS_PMU_ATLAS_CPUSEQUENCER_OPTION);
	entered_idx = (!dev->states_usage[2].disable &&
		       (triggered_cpd || exynos7420_atlas_cpd_gen != gen_before)) ? 2 : 1;
	raw_spin_unlock(&exynos7420_cpd_lock);

	val = readl_relaxed(pmu_base + cfg_off);
	writel(val | EXYNOS_PMU_LOCAL_PWR_CFG, pmu_base + cfg_off);

	cpu_pm_exit();

	if (ret) {
		cpu_do_idle();
		return drv->safe_state_index;
	}

	return entered_idx;
}

static __cpuidle int exynos7420_enter_atlas_c2(struct cpuidle_device *dev,
					       struct cpuidle_driver *drv,
					       int index)
{
	bool allow_cpd = false;

	if (!dev->states_usage[2].disable &&
	    !(drv->states[2].flags & CPUIDLE_FLAG_OFF) &&
	    ktime_sub(tick_nohz_get_next_hrtimer(), ktime_get()) >=
	    EXYNOS7420_CPD_RESIDENCY_NS)
		allow_cpd = true;

	return exynos7420_enter_atlas_idle(dev, drv, allow_cpd);
}

static __cpuidle int exynos7420_enter_atlas_c2_s2idle(struct cpuidle_device *dev,
						      struct cpuidle_driver *drv,
						      int index)
{
	return exynos7420_enter_atlas_idle(dev, drv, false);
}

static __cpuidle int exynos7420_enter_cpd(struct cpuidle_device *dev,
					  struct cpuidle_driver *drv, int index)
{
	return exynos7420_enter_atlas_idle(dev, drv, true);
}

static struct cpuidle_driver exynos7420_apollo_idle_driver = {
	.name = "exynos7420_apollo_idle",
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
			.flags			= CPUIDLE_FLAG_TIMER_STOP | CPUIDLE_FLAG_RCU_IDLE,
			.name			= "C2",
			.desc			= "Apollo Core Power Gating (C2)",
		},
	},
	.safe_state_index = 0,
	.state_count = 2,
};

static struct cpuidle_driver exynos7420_atlas_idle_driver = {
	.name = "exynos7420_atlas_idle",
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
			.enter			= exynos7420_enter_atlas_c2,
			.enter_s2idle		= exynos7420_enter_atlas_c2_s2idle,
			.exit_latency		= 100,
			.target_residency	= 1000,
			.power_usage		= UINT_MAX,
			.flags			= CPUIDLE_FLAG_TIMER_STOP | CPUIDLE_FLAG_RCU_IDLE,
			.name			= "C2",
			.desc			= "Atlas Core Power Gating (C2)",
		},
		[2] = {
			.enter			= exynos7420_enter_cpd,
			.enter_s2idle		= exynos7420_enter_cpd,
			.exit_latency		= 300,
			.target_residency	= 3000,
			.power_usage		= UINT_MAX,
			.flags			= CPUIDLE_FLAG_TIMER_STOP | CPUIDLE_FLAG_RCU_IDLE,
			.name			= "CPD",
			.desc			= "Atlas Cluster Power Down (CPD)",
		},
	},
	.safe_state_index = 0,
	.state_count = 3,
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
	unsigned long flags;

	raw_spin_lock_irqsave(&exynos7420_cpd_lock, flags);
	cpumask_clear_cpu(cpu, &exynos7420_atlas_c2_mask);
	raw_spin_unlock_irqrestore(&exynos7420_cpd_lock, flags);

	if (!exynos7420_cpuidle_devices)
		return 0;

	dev = per_cpu_ptr(exynos7420_cpuidle_devices, cpu);
	if (dev->registered)
		cpuidle_unregister_device(dev);

	return 0;
}

static void __init exynos7420_cpuidle_cleanup_iomem(void)
{
	if (atlas_pwr_ctrl) {
		iounmap(atlas_pwr_ctrl);
		atlas_pwr_ctrl = NULL;
	}
	if (apollo_pwr_ctrl) {
		iounmap(apollo_pwr_ctrl);
		apollo_pwr_ctrl = NULL;
	}
	if (pmu_base) {
		iounmap(pmu_base);
		pmu_base = NULL;
	}
}

static int __init exynos7420_cpuidle_init(void)
{
	struct device_node *np;
	unsigned int cpu;
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

	/* Initialize CPU/Cluster/Sequencer PMU options & L2 Q-Channel clock down */
	atlas_pwr_ctrl  = ioremap(EXYNOS7420_ATLAS_PWR_CTRL_PA, 4);
	apollo_pwr_ctrl = ioremap(EXYNOS7420_APOLLO_PWR_CTRL_PA, 4);
	exynos7420_init_pmu();

	for_each_possible_cpu(cpu) {
		u64 mpidr = cpu_logical_map(cpu);

		if (MPIDR_AFFINITY_LEVEL(mpidr, 1) == 0)
			cpumask_set_cpu(cpu, &exynos7420_atlas_cpumask);
		else
			cpumask_set_cpu(cpu, &exynos7420_apollo_cpumask);
	}

	exynos7420_apollo_idle_driver.cpumask = &exynos7420_apollo_cpumask;
	exynos7420_atlas_idle_driver.cpumask = &exynos7420_atlas_cpumask;

	exynos7420_cpuidle_devices = alloc_percpu(struct cpuidle_device);
	if (!exynos7420_cpuidle_devices) {
		exynos7420_cpuidle_cleanup_iomem();
		return -ENOMEM;
	}

	ret = cpuidle_register_driver(&exynos7420_apollo_idle_driver);
	if (ret) {
		pr_err("Apollo cpuidle driver registration failed: %d\n", ret);
		free_percpu(exynos7420_cpuidle_devices);
		exynos7420_cpuidle_devices = NULL;
		exynos7420_cpuidle_cleanup_iomem();
		return ret;
	}

	ret = cpuidle_register_driver(&exynos7420_atlas_idle_driver);
	if (ret) {
		pr_err("Atlas cpuidle driver registration failed: %d\n", ret);
		cpuidle_unregister_driver(&exynos7420_apollo_idle_driver);
		free_percpu(exynos7420_cpuidle_devices);
		exynos7420_cpuidle_devices = NULL;
		exynos7420_cpuidle_cleanup_iomem();
		return ret;
	}

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "cpuidle/exynos7420:online",
				exynos7420_cpu_online, exynos7420_cpu_offline);
	if (ret < 0) {
		pr_err("Failed to setup cpuhp state: %d\n", ret);
		cpuidle_unregister_driver(&exynos7420_atlas_idle_driver);
		cpuidle_unregister_driver(&exynos7420_apollo_idle_driver);
		free_percpu(exynos7420_cpuidle_devices);
		exynos7420_cpuidle_devices = NULL;
		exynos7420_cpuidle_cleanup_iomem();
		return ret;
	}
	exynos7420_hp_state = ret;

	pr_info("Drivers registered (Apollo: %d states/%u CPUs, Atlas: %d states/%u CPUs, L2 QACTIVE enabled)\n",
		exynos7420_apollo_idle_driver.state_count,
		cpumask_weight(&exynos7420_apollo_cpumask),
		exynos7420_atlas_idle_driver.state_count,
		cpumask_weight(&exynos7420_atlas_cpumask));
	return 0;
}
device_initcall(exynos7420_cpuidle_init);

MODULE_AUTHOR("404698-FDU <24302010006@m.fudan.edu.cn>");
MODULE_DESCRIPTION("Samsung Exynos 7420 CPUIdle Driver with C2 and Atlas CPD Support");
MODULE_LICENSE("GPL");
