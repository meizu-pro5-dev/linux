// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos 7420 LPDDR4 Dynamic Memory Controller (DREX) Power Management
 *
 * Copyright (C) 2026 404698-FDU <24302010006@m.fudan.edu.cn>
 *
 * Configures the 4-channel DREX LPDDR4 memory controllers on Exynos 7420:
 * - Dynamic Self-Refresh (DSREF_EN, MEMCONTROL bit 5)
 * - Dynamic Power-Down (DPWRDN_EN, MEMCONTROL bit 1)
 * - Number of cycles for dynamic self refresh entry (PWRDNCONFIG bits [31:16] = 0x3ff)
 * - Internal Clock Gating (CGCONTROL bits [5:0] = 0x3f)
 */

#define pr_fmt(fmt) "exynos7420-dmc: " fmt

#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>
#include <linux/workqueue.h>
#include <linux/m86-bus-qos.h>

#define NUM_DREX_CHANNELS		4
#define DREX_MAP_SIZE			0x1000

static const phys_addr_t drex_phys_bases[NUM_DREX_CHANNELS] = {
	0x10800000,
	0x10900000,
	0x10A00000,
	0x10B00000,
};

#define DREX_MEMCONTROL			0x04
#define DREX_CGCONTROL			0x08
#define DREX_DIRECTCMD			0x10
#define DREX_PWRDNCONFIG		0x28
#define DREX_TIMINGAREF			0x30
#define DREX_MRSTATUS			0x54

#define DREX_DSREF_EN			BIT(5)
#define DREX_DPWRDN_EN			BIT(1)

#define DREX_DSREF_CYC_MASK		(0xFFFF << 16)
#define DREX_DSREF_CYC_DEFAULT		(0x03FF << 16)

#define DREX_CGCONTROL_ALL_EN		0x3F
#define DREX_MRSTATUS_THERMAL_LV_MASK	0x7

/* LPDDR4 MR4 temperature-aware tREFI auto-refresh rates (from 3.10 exynos7420_bus.c) */
#define DREX_AREF_RATE_TWO		0x001800BA	/* 2x tREFI (0.5x refresh rate) for MR4 <= 2 */
#define DREX_AREF_RATE_ONE		0x000C005D	/* 1x tREFI (normal refresh rate) for MR4 == 3 */
#define DREX_AREF_RATE_HALF		0x0006002E	/* 0.5x tREFI (2x refresh rate) for MR4 == 4 */
#define DREX_AREF_RATE_QUARTER		0x00030017	/* 0.25x tREFI (4x refresh rate) for MR4 >= 5 */

/* MRR command for MR4 (MA=4) on CS0 (chip=0) and CS1 (chip=1) */
#define DREX_DIRECTCMD_MR4_CS0		0x09001000
#define DREX_DIRECTCMD_MR4_CS1		0x09101000

static void __iomem *drex_bases[NUM_DREX_CHANNELS];
static struct platform_device *dmc_pdev;
static DEFINE_MUTEX(dmc_lock);
static struct delayed_work dmc_thermal_work;
static u32 dmc_mr4_cs0[NUM_DREX_CHANNELS];
static u32 dmc_mr4_cs1[NUM_DREX_CHANNELS];

static u32 exynos7420_dmc_get_autorate(u32 thermal_level)
{
	switch (thermal_level) {
	case 0:
	case 1:
	case 2:
		return DREX_AREF_RATE_TWO;
	case 3:
		return DREX_AREF_RATE_ONE;
	case 4:
		return DREX_AREF_RATE_HALF;
	default:
		return DREX_AREF_RATE_QUARTER;
	}
}

static unsigned int exynos7420_dmc_update_mr4_locked(void)
{
	unsigned int max_level = 0;
	int i;

	for (i = 0; i < NUM_DREX_CHANNELS; i++) {
		void __iomem *base = drex_bases[i];
		u32 cs0_lv, cs1_lv, ch_max, aref;

		if (!base)
			continue;

		writel_relaxed(DREX_DIRECTCMD_MR4_CS0, base + DREX_DIRECTCMD);
		cs0_lv = readl_relaxed(base + DREX_MRSTATUS) & DREX_MRSTATUS_THERMAL_LV_MASK;

		writel_relaxed(DREX_DIRECTCMD_MR4_CS1, base + DREX_DIRECTCMD);
		cs1_lv = readl_relaxed(base + DREX_MRSTATUS) & DREX_MRSTATUS_THERMAL_LV_MASK;

		dmc_mr4_cs0[i] = cs0_lv;
		dmc_mr4_cs1[i] = cs1_lv;

		ch_max = max(cs0_lv, cs1_lv);
		if (ch_max > max_level)
			max_level = ch_max;

		aref = exynos7420_dmc_get_autorate(ch_max);
		if (readl_relaxed(base + DREX_TIMINGAREF) != aref)
			writel_relaxed(aref, base + DREX_TIMINGAREF);
	}

	return max_level;
}

static void exynos7420_dmc_thermal_work_fn(struct work_struct *work)
{
	unsigned int max_level;
	unsigned long delay_ms;

	mutex_lock(&dmc_lock);
	exynos7420_bus_dvfs_read_lock();
	max_level = exynos7420_dmc_update_mr4_locked();
	exynos7420_bus_dvfs_read_unlock();
	mutex_unlock(&dmc_lock);

	delay_ms = (max_level >= 4) ? 300 : 2000;
	schedule_delayed_work(&dmc_thermal_work, msecs_to_jiffies(delay_ms));
}

static void exynos7420_dmc_apply_power_saving(void)
{
	int i;
	u32 val;

	mutex_lock(&dmc_lock);
	exynos7420_bus_dvfs_read_lock();
	for (i = 0; i < NUM_DREX_CHANNELS; i++) {
		void __iomem *base = drex_bases[i];

		if (!base)
			continue;

		/* 1. Temporarily clear DSREF before programming cycle threshold */
		val = readl_relaxed(base + DREX_MEMCONTROL);
		val &= ~DREX_DSREF_EN;
		writel_relaxed(val, base + DREX_MEMCONTROL);

		/* 2. Configure dynamic self-refresh cycles (0x3FF) */
		val = readl_relaxed(base + DREX_PWRDNCONFIG);
		val = (val & ~DREX_DSREF_CYC_MASK) | DREX_DSREF_CYC_DEFAULT;
		writel_relaxed(val, base + DREX_PWRDNCONFIG);

		/* 3. Enable Dynamic Self-Refresh (aligned with Samsung vendor 3.10 driver) */
		val = readl_relaxed(base + DREX_MEMCONTROL);
		val |= DREX_DSREF_EN;
		writel_relaxed(val, base + DREX_MEMCONTROL);
	}
	exynos7420_dmc_update_mr4_locked();
	exynos7420_bus_dvfs_read_unlock();
	mutex_unlock(&dmc_lock);
}

static int exynos7420_dmc_syscore_suspend(void)
{
	int i;

	/* Restore 1x tREFI before system suspend (matching 3.10 exynos7_devfreq_set_tREFI_1x) */
	for (i = 0; i < NUM_DREX_CHANNELS; i++) {
		void __iomem *base = drex_bases[i];

		if (!base)
			continue;
		if (readl_relaxed(base + DREX_TIMINGAREF) == DREX_AREF_RATE_TWO)
			writel_relaxed(DREX_AREF_RATE_ONE, base + DREX_TIMINGAREF);
	}
	return 0;
}

static void exynos7420_dmc_syscore_resume(void)
{
	int i;
	u32 val;

	for (i = 0; i < NUM_DREX_CHANNELS; i++) {
		void __iomem *base = drex_bases[i];

		if (!base)
			continue;

		val = readl_relaxed(base + DREX_MEMCONTROL);
		val &= ~DREX_DSREF_EN;
		writel_relaxed(val, base + DREX_MEMCONTROL);

		val = readl_relaxed(base + DREX_PWRDNCONFIG);
		val = (val & ~DREX_DSREF_CYC_MASK) | DREX_DSREF_CYC_DEFAULT;
		writel_relaxed(val, base + DREX_PWRDNCONFIG);

		val = readl_relaxed(base + DREX_MEMCONTROL);
		val |= DREX_DSREF_EN;
		writel_relaxed(val, base + DREX_MEMCONTROL);
	}
	exynos7420_dmc_update_mr4_locked();
}

static struct syscore_ops exynos7420_dmc_syscore_ops = {
	.suspend = exynos7420_dmc_syscore_suspend,
	.resume  = exynos7420_dmc_syscore_resume,
};

static ssize_t dmc_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i;

	mutex_lock(&dmc_lock);
	exynos7420_bus_dvfs_read_lock();
	exynos7420_dmc_update_mr4_locked();

	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "=== Exynos 7420 DREX LPDDR4 Power Status ===\n");

	for (i = 0; i < NUM_DREX_CHANNELS; i++) {
		void __iomem *base = drex_bases[i];
		u32 memctrl, pwrdn, cgctrl, aref;

		if (!base) {
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "Channel %d: unmapped\n", i);
			continue;
		}

		memctrl = readl_relaxed(base + DREX_MEMCONTROL);
		pwrdn   = readl_relaxed(base + DREX_PWRDNCONFIG);
		cgctrl  = readl_relaxed(base + DREX_CGCONTROL);
		aref    = readl_relaxed(base + DREX_TIMINGAREF);

		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "Channel %d [0x%08llx]: MEMCTRL=0x%08x (DSREF=%d DPWRDN=%d) PWRDN=0x%08x (cyc=0x%04x) CG=0x%08x AREF=0x%08x MR4=(%u,%u)\n",
				 i, (unsigned long long)drex_phys_bases[i],
				 memctrl,
				 !!(memctrl & DREX_DSREF_EN),
				 !!(memctrl & DREX_DPWRDN_EN),
				 pwrdn, (pwrdn >> 16) & 0xFFFF,
				 cgctrl, aref, dmc_mr4_cs0[i], dmc_mr4_cs1[i]);
	}
	exynos7420_bus_dvfs_read_unlock();
	mutex_unlock(&dmc_lock);

	return len;
}
static DEVICE_ATTR_RO(dmc_status);

static struct attribute *dmc_attrs[] = {
	&dev_attr_dmc_status.attr,
	NULL,
};
ATTRIBUTE_GROUPS(dmc);

static int __init exynos7420_dmc_init(void)
{
	int i, ret;

	if (!of_machine_is_compatible("samsung,exynos7420"))
		return -ENODEV;

	for (i = 0; i < NUM_DREX_CHANNELS; i++) {
		drex_bases[i] = ioremap(drex_phys_bases[i], DREX_MAP_SIZE);
		if (!drex_bases[i]) {
			pr_err("Failed to map DREX%d at 0x%08llx\n",
			       i, (unsigned long long)drex_phys_bases[i]);
			while (--i >= 0)
				iounmap(drex_bases[i]);
			return -ENOMEM;
		}
	}

	INIT_DEFERRABLE_WORK(&dmc_thermal_work, exynos7420_dmc_thermal_work_fn);
	exynos7420_dmc_apply_power_saving();
	register_syscore_ops(&exynos7420_dmc_syscore_ops);
	schedule_delayed_work(&dmc_thermal_work, msecs_to_jiffies(2000));

	dmc_pdev = platform_device_register_simple("exynos7420-dmc", -1, NULL, 0);
	if (!IS_ERR(dmc_pdev)) {
		ret = device_add_groups(&dmc_pdev->dev, dmc_groups);
		if (ret)
			pr_warn("Failed to add sysfs groups for exynos7420-dmc: %d\n", ret);
	}

	pr_info("LPDDR4 Dynamic Self-Refresh, Power-Down & MR4 thermal tREFI enabled across %d channels\n",
		NUM_DREX_CHANNELS);
	return 0;
}
device_initcall(exynos7420_dmc_init);

MODULE_AUTHOR("404698-FDU <24302010006@m.fudan.edu.cn>");
MODULE_DESCRIPTION("Exynos 7420 LPDDR4 DMC Dynamic Power Management");
MODULE_LICENSE("GPL");
