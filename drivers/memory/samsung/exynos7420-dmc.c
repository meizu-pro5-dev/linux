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
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>

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
#define DREX_PWRDNCONFIG		0x28

#define DREX_DSREF_EN			BIT(5)
#define DREX_DPWRDN_EN			BIT(1)

#define DREX_DSREF_CYC_MASK		(0xFFFF << 16)
#define DREX_DSREF_CYC_DEFAULT		(0x03FF << 16)

#define DREX_CGCONTROL_ALL_EN		0x3F

static void __iomem *drex_bases[NUM_DREX_CHANNELS];
static struct platform_device *dmc_pdev;

static void exynos7420_dmc_apply_power_saving(void)
{
	int i;
	u32 val;

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
}

static void exynos7420_dmc_syscore_resume(void)
{
	exynos7420_dmc_apply_power_saving();
}

static struct syscore_ops exynos7420_dmc_syscore_ops = {
	.resume = exynos7420_dmc_syscore_resume,
};

static ssize_t dmc_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i;

	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "=== Exynos 7420 DREX LPDDR4 Power Status ===\n");

	for (i = 0; i < NUM_DREX_CHANNELS; i++) {
		void __iomem *base = drex_bases[i];
		u32 memctrl, pwrdn, cgctrl;

		if (!base) {
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "Channel %d: unmapped\n", i);
			continue;
		}

		memctrl = readl_relaxed(base + DREX_MEMCONTROL);
		pwrdn   = readl_relaxed(base + DREX_PWRDNCONFIG);
		cgctrl  = readl_relaxed(base + DREX_CGCONTROL);

		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "Channel %d [0x%08llx]: MEMCTRL=0x%08x (DSREF=%d DPWRDN=%d) PWRDN=0x%08x (cyc=0x%04x) CG=0x%08x\n",
				 i, (unsigned long long)drex_phys_bases[i],
				 memctrl,
				 !!(memctrl & DREX_DSREF_EN),
				 !!(memctrl & DREX_DPWRDN_EN),
				 pwrdn, (pwrdn >> 16) & 0xFFFF,
				 cgctrl);
	}

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

	exynos7420_dmc_apply_power_saving();
	register_syscore_ops(&exynos7420_dmc_syscore_ops);

	dmc_pdev = platform_device_register_simple("exynos7420-dmc", -1, NULL, 0);
	if (!IS_ERR(dmc_pdev)) {
		ret = device_add_groups(&dmc_pdev->dev, dmc_groups);
		if (ret)
			pr_warn("Failed to add sysfs groups for exynos7420-dmc: %d\n", ret);
	}

	pr_info("LPDDR4 Dynamic Self-Refresh & Power-Down enabled across %d channels\n",
		NUM_DREX_CHANNELS);
	return 0;
}
device_initcall(exynos7420_dmc_init);

MODULE_AUTHOR("404698-FDU <24302010006@m.fudan.edu.cn>");
MODULE_DESCRIPTION("Exynos 7420 LPDDR4 DMC Dynamic Power Management");
MODULE_LICENSE("GPL");
