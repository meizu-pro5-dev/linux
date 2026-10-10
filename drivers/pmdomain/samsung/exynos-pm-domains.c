// SPDX-License-Identifier: GPL-2.0
//
// Exynos Generic power domain support.
//
// Copyright (c) 2012 Samsung Electronics Co., Ltd.
//		http://www.samsung.com
//
// Implementation of Exynos specific power domain control which is used in
// conjunction with runtime-pm. Support for both device-tree and non-device-tree
// based power domain support is included.

#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>

enum exynos_pd_block {
	PD_BLK_TOP0,
	PD_BLK_DISP,
	PD_BLK_VPP,
	PD_BLK_PMU,
	PD_NUM_BLKS,
};

/* RMW descriptor for the Exynos7420 power-up/down register sequences */
struct exynos_pd_seq {
	u8 blk;
	u16 off;
	u32 set;
	u32 clr;
};

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
	/* Value written to the OPTION register (base + 0x8); 0 = skip */
	u32 local_pwr_option;
	/* Optional pre/post power sequences (Exynos7420 DISP) */
	const struct exynos_pd_seq *pre;
	unsigned int n_pre;
	const struct exynos_pd_seq *post;
	unsigned int n_post;
};

/*
 * Exynos specific wrapper around the generic power domain
 */
struct exynos_pm_domain {
	void __iomem *base;
	struct generic_pm_domain pd;
	u32 local_pwr_cfg;
	u32 local_pwr_option;
	void __iomem *blk[PD_NUM_BLKS];
	const struct exynos_pd_seq *pre;
	unsigned int n_pre;
	const struct exynos_pd_seq *post;
	unsigned int n_post;
};

static const resource_size_t exynos7420_pd_blk_addr[PD_NUM_BLKS] = {
	[PD_BLK_TOP0] = 0x105d0000,
	[PD_BLK_DISP] = 0x13ad0000,
	[PD_BLK_VPP] = 0x13ed0000,
	[PD_BLK_PMU] = 0x105c0000,
};

static void exynos_pd_apply_seq(struct exynos_pm_domain *pd,
				const struct exynos_pd_seq *seq,
				unsigned int n, bool power_on)
{
	unsigned int i;

	for (i = 0; i < n; i++, seq++) {
		void __iomem *base = pd->blk[seq->blk];
		u32 set = power_on ? seq->set : seq->clr;
		u32 clr = power_on ? seq->clr : seq->set;
		u32 v;

		if (!base)
			continue;

		v = readl_relaxed(base + seq->off);
		v |= set;
		v &= ~clr;
		writel_relaxed(v, base + seq->off);
	}
}

static int exynos_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct exynos_pm_domain *pd;
	void __iomem *base;
	u32 timeout, pwr;
	char *op;

	pd = container_of(domain, struct exynos_pm_domain, pd);
	base = pd->base;

	pwr = power_on ? pd->local_pwr_cfg : 0;

	/* Exynos7420: bring up top/async clocks + PMU sys_pwr before power */
	if (power_on)
		exynos_pd_apply_seq(pd, pd->pre, pd->n_pre, true);

	if (power_on && pd->local_pwr_option)
		writel_relaxed(pd->local_pwr_option, base + 0x8);

	writel_relaxed(pwr, base);

	/* Wait max 1ms */
	timeout = 10;

	while ((readl_relaxed(base + 0x4) & pd->local_pwr_cfg) != pwr) {
		if (!timeout) {
			op = (power_on) ? "enable" : "disable";
			pr_err("Power domain %s %s failed\n", domain->name, op);
			return -ETIMEDOUT;
		}
		timeout--;
		cpu_relax();
		usleep_range(80, 100);
	}

	if (power_on)
		exynos_pd_apply_seq(pd, pd->post, pd->n_post, true);

	return 0;
}

static int exynos_pd_power_on(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, true);
}

static int exynos_pd_power_off(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, false);
}

static const struct exynos_pm_domain_config exynos4210_cfg = {
	.local_pwr_cfg		= 0x7,
};

static const struct exynos_pm_domain_config exynos5433_cfg = {
	.local_pwr_cfg		= 0xf,
};

/*
 * Exynos7420 DISP power sequence, ported from the vendor kernel
 * (arch/arm/mach-exynos/pm_domains-exynos7420.c + -cal.h). The generic
 * sequence is not enough on this SoC: top-to-local clocks, async-bridge
 * clocks and the PMU sys_pwr registers must be programmed around the local
 * power switch, and the OPTION register has to be written.
 */
static const struct exynos_pd_seq exynos7420_disp_pre[] = {
	{ PD_BLK_TOP0, 0x080c, BIT(28), 0 },
	{ PD_BLK_TOP0, 0x0a20, BIT(28) | BIT(24) | BIT(20) | BIT(16) | BIT(12), 0 },
	{ PD_BLK_VPP, 0x0800, 0xf, 0 },
	{ PD_BLK_PMU, 0x1450, 0, BIT(0) },
	{ PD_BLK_PMU, 0x1490, 0, BIT(0) },
	{ PD_BLK_PMU, 0x14d0, 0, BIT(0) },
	{ PD_BLK_PMU, 0x1510, 0, BIT(0) },
	{ PD_BLK_PMU, 0x1550, 0, BIT(9) | BIT(8) | BIT(5) | BIT(4) | BIT(1) | BIT(0) },
	{ PD_BLK_PMU, 0x1590, 0, BIT(0) },
};

static const struct exynos_pd_seq exynos7420_disp_post[] = {
	{ PD_BLK_DISP, 0x0800, BIT(31) | BIT(30) | BIT(29) | BIT(28), 0 },
};

static const struct exynos_pm_domain_config exynos7420_disp_cfg = {
	.local_pwr_cfg		= 0xf,
	.local_pwr_option	= 0x0102,
	.pre			= exynos7420_disp_pre,
	.n_pre			= ARRAY_SIZE(exynos7420_disp_pre),
	.post			= exynos7420_disp_post,
	.n_post			= ARRAY_SIZE(exynos7420_disp_post),
};

static const struct of_device_id exynos_pm_domain_of_match[] = {
	{
		.compatible = "samsung,exynos4210-pd",
		.data = &exynos4210_cfg,
	}, {
		.compatible = "samsung,exynos5433-pd",
		.data = &exynos5433_cfg,
	}, {
		.compatible = "samsung,exynos7420-pd",
		.data = &exynos7420_disp_cfg,
	},
	{ },
};

static const char *exynos_get_domain_name(struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = kbasename(node->full_name);
	return kstrdup_const(name, GFP_KERNEL);
}

static int exynos_pd_probe(struct platform_device *pdev)
{
	const struct exynos_pm_domain_config *pm_domain_cfg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct of_phandle_args child, parent;
	struct exynos_pm_domain *pd;
	int on, ret;

	pm_domain_cfg = of_device_get_match_data(dev);
	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->pd.name = exynos_get_domain_name(np);
	if (!pd->pd.name)
		return -ENOMEM;

	pd->base = of_iomap(np, 0);
	if (!pd->base) {
		kfree_const(pd->pd.name);
		return -ENODEV;
	}

	pd->pd.power_off = exynos_pd_power_off;
	pd->pd.power_on = exynos_pd_power_on;
	pd->local_pwr_cfg = pm_domain_cfg->local_pwr_cfg;
	pd->local_pwr_option = pm_domain_cfg->local_pwr_option;
	pd->pre = pm_domain_cfg->pre;
	pd->n_pre = pm_domain_cfg->n_pre;
	pd->post = pm_domain_cfg->post;
	pd->n_post = pm_domain_cfg->n_post;

	/* Exynos7420: map the clock/PMU register windows used by the seqs */
	if (pd->n_pre || pd->n_post) {
		int i;

		for (i = 0; i < PD_NUM_BLKS; i++) {
			pd->blk[i] = devm_ioremap(dev,
						  exynos7420_pd_blk_addr[i],
						  0x1000);
			if (!pd->blk[i])
				dev_warn(dev, "failed to map block %d\n", i);
		}
	}

	on = readl_relaxed(pd->base + 0x4) & pd->local_pwr_cfg;

	pm_genpd_init(&pd->pd, NULL, !on);
	ret = of_genpd_add_provider_simple(np, &pd->pd);

	if (ret == 0 && of_parse_phandle_with_args(np, "power-domains",
				      "#power-domain-cells", 0, &parent) == 0) {
		child.np = np;
		child.args_count = 0;

		if (of_genpd_add_subdomain(&parent, &child))
			pr_warn("%pOF failed to add subdomain: %pOF\n",
				parent.np, child.np);
		else
			pr_info("%pOF has as child subdomain: %pOF.\n",
				parent.np, child.np);
	}

	pm_runtime_enable(dev);
	return ret;
}

static struct platform_driver exynos_pd_driver = {
	.probe	= exynos_pd_probe,
	.driver	= {
		.name		= "exynos-pd",
		.of_match_table	= exynos_pm_domain_of_match,
		.suppress_bind_attrs = true,
	}
};

static __init int exynos4_pm_init_power_domain(void)
{
	return platform_driver_register(&exynos_pd_driver);
}
core_initcall(exynos4_pm_init_power_domain);
