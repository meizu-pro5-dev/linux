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

struct exynos7420_reg_bits {
	phys_addr_t pa;
	u32 mask;
};

struct exynos7420_unused_pd {
	const char *name;
	u32 pmu_off;
	phys_addr_t lpi_busmaster_pa;
	bool clear_a7is_option;
	bool is_aud;
	const struct exynos7420_reg_bits *top_clks;
	size_t num_top_clks;
	const struct exynos7420_reg_bits *local_clks;
	size_t num_local_clks;
	const struct exynos7420_reg_bits *async_clks;
	size_t num_async_clks;
	const struct exynos7420_reg_bits *sys_pwr_regs;
	size_t num_sys_pwr_regs;
};

static const struct exynos7420_reg_bits exynos7420_top_clks_mfc[] = {
	{ 0x10570804, BIT(8) },
};
static const struct exynos7420_reg_bits exynos7420_local_clks_mfc[] = {
	{ 0x15280800, BIT(31) | BIT(30) | BIT(19) },
};
static const struct exynos7420_reg_bits exynos7420_sys_pwr_mfc[] = {
	{ 0x105c1470, BIT(0) },
	{ 0x105c14b0, BIT(0) },
	{ 0x105c14f0, BIT(0) },
	{ 0x105c1530, BIT(0) },
	{ 0x105c1570, BIT(5) | BIT(4) | BIT(1) | BIT(0) },
	{ 0x105c15b0, BIT(0) },
};

static const struct exynos7420_reg_bits exynos7420_top_clks_mscl[] = {
	{ 0x10570804, BIT(20) },
};
static const struct exynos7420_reg_bits exynos7420_local_clks_mscl[] = {
	{ 0x150d0800, BIT(31) | BIT(30) | BIT(29) | BIT(28) | BIT(27) | BIT(26) },
};
static const struct exynos7420_reg_bits exynos7420_sys_pwr_mscl[] = {
	{ 0x105c1474, BIT(0) },
	{ 0x105c14b4, BIT(0) },
	{ 0x105c14f4, BIT(0) },
	{ 0x105c1534, BIT(0) },
	{ 0x105c1574, BIT(9) | BIT(8) | BIT(5) | BIT(4) | BIT(1) | BIT(0) },
	{ 0x105c15b4, BIT(0) },
};

static const struct exynos7420_reg_bits exynos7420_top_clks_isp0[] = {
	{ 0x105d0810, BIT(28) | BIT(24) | BIT(20) },
};
static const struct exynos7420_reg_bits exynos7420_local_clks_isp0[] = {
	{ 0x146d0800, BIT(17) | BIT(15) | BIT(13) | BIT(1) | BIT(0) },
	{ 0x146d0804, BIT(1) | BIT(0) },
	{ 0x14290800, BIT(1) | BIT(0) },
	{ 0x14290804, BIT(1) | BIT(0) },
};
static const struct exynos7420_reg_bits exynos7420_async_clks_isp0[] = {
	{ 0x145d0804, BIT(25) | BIT(10) },
	{ 0x145d0800, BIT(19) },
	{ 0x141f0800, BIT(4) },
	{ 0x144d0800, BIT(1) | BIT(0) },
	{ 0x140f0800, BIT(1) | BIT(0) },
};
static const struct exynos7420_reg_bits exynos7420_sys_pwr_isp0[] = {
	{ 0x105c1468, BIT(0) },
	{ 0x105c14a8, BIT(0) },
	{ 0x105c14e8, BIT(0) },
	{ 0x105c1528, BIT(0) },
	{ 0x105c1568, BIT(5) | BIT(4) | BIT(1) | BIT(0) },
	{ 0x105c15a8, BIT(0) },
};

static const struct exynos7420_reg_bits exynos7420_top_clks_isp1[] = {
	{ 0x105d0810, BIT(16) | BIT(12) },
};
static const struct exynos7420_reg_bits exynos7420_local_clks_isp1[] = {
	{ 0x147d0800, BIT(12) | BIT(9) | BIT(8) | BIT(1) | BIT(0) },
	{ 0x147d0900, BIT(0) },
	{ 0x142f0800, BIT(1) | BIT(0) },
	{ 0x142f0900, BIT(0) },
};
static const struct exynos7420_reg_bits exynos7420_async_clks_isp1[] = {
	{ 0x145d0804, BIT(26) | BIT(22) | BIT(11) },
	{ 0x144d0800, BIT(1) | BIT(0) },
	{ 0x140f0800, BIT(1) | BIT(0) },
};
static const struct exynos7420_reg_bits exynos7420_sys_pwr_isp1[] = {
	{ 0x105c146c, BIT(0) },
	{ 0x105c14ac, BIT(0) },
	{ 0x105c14ec, BIT(0) },
	{ 0x105c152c, BIT(0) },
	{ 0x105c156c, BIT(1) | BIT(0) },
	{ 0x105c15ac, BIT(0) },
};

static const struct exynos7420_reg_bits exynos7420_top_clks_cam0[] = {
	{ 0x105d0814, BIT(28) | BIT(24) | BIT(20) | BIT(16) | BIT(12) | BIT(8) },
	{ 0x105d0818, BIT(28) | BIT(24) | BIT(20) | BIT(16) },
};
static const struct exynos7420_reg_bits exynos7420_local_clks_cam0[] = {
	{ 0x144d0800, BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0) },
	{ 0x144d0804, BIT(18) | BIT(17) | BIT(16) | BIT(10) },
	{ 0x144d0900, BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0) },
	{ 0x144d0904, BIT(17) | BIT(16) },
	{ 0x140f0800, BIT(6) | BIT(5) | BIT(4) | BIT(1) | BIT(0) },
	{ 0x140f0900, BIT(6) | BIT(5) | BIT(4) | BIT(1) | BIT(0) },
};
static const struct exynos7420_reg_bits exynos7420_async_clks_cam0[] = {
	{ 0x145d0804, BIT(23) },
};
static const struct exynos7420_reg_bits exynos7420_sys_pwr_cam0[] = {
	{ 0x105c1448, BIT(0) },
	{ 0x105c1488, BIT(0) },
	{ 0x105c14c8, BIT(0) },
	{ 0x105c1508, BIT(0) },
	{ 0x105c1548, BIT(5) | BIT(4) | BIT(1) | BIT(0) },
	{ 0x105c1588, BIT(0) },
};

static const struct exynos7420_reg_bits exynos7420_top_clks_cam1[] = {
	{ 0x105d081c, BIT(28) | BIT(24) | BIT(20) | BIT(16) | BIT(12) | BIT(8) },
	{ 0x105d0a24, BIT(20) | BIT(8) | BIT(4) },
};
static const struct exynos7420_reg_bits exynos7420_local_clks_cam1[] = {
	{ 0x145d0800, BIT(3) | BIT(2) | BIT(0) },
	{ 0x145d0804, BIT(28) | BIT(26) | BIT(25) | BIT(23) | BIT(22) | BIT(11) | BIT(10) },
	{ 0x145d080c, BIT(3) | BIT(2) },
	{ 0x145d0900, BIT(3) | BIT(2) | BIT(0) },
	{ 0x141f0800, BIT(2) | BIT(1) | BIT(0) },
	{ 0x141f0900, BIT(2) | BIT(1) | BIT(0) },
};
static const struct exynos7420_reg_bits exynos7420_sys_pwr_cam1[] = {
	{ 0x105c144c, BIT(0) },
	{ 0x105c148c, BIT(0) },
	{ 0x105c14cc, BIT(0) },
	{ 0x105c150c, BIT(0) },
	{ 0x105c154c, BIT(5) | BIT(4) | BIT(1) | BIT(0) },
	{ 0x105c158c, BIT(0) },
};

static const struct exynos7420_unused_pd exynos7420_unused_pds[] = {
	{
		.name = "pd-mfc",
		.pmu_off = 0x41a0,
		.top_clks = exynos7420_top_clks_mfc,
		.num_top_clks = ARRAY_SIZE(exynos7420_top_clks_mfc),
		.local_clks = exynos7420_local_clks_mfc,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_mfc),
		.sys_pwr_regs = exynos7420_sys_pwr_mfc,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_mfc),
	},
	{
		.name = "pd-mscl",
		.pmu_off = 0x41c0,
		.top_clks = exynos7420_top_clks_mscl,
		.num_top_clks = ARRAY_SIZE(exynos7420_top_clks_mscl),
		.local_clks = exynos7420_local_clks_mscl,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_mscl),
		.sys_pwr_regs = exynos7420_sys_pwr_mscl,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_mscl),
	},
	{
		.name = "pd-isp0",
		.pmu_off = 0x4140,
		.lpi_busmaster_pa = 0x146e0000,
		.top_clks = exynos7420_top_clks_isp0,
		.num_top_clks = ARRAY_SIZE(exynos7420_top_clks_isp0),
		.local_clks = exynos7420_local_clks_isp0,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_isp0),
		.async_clks = exynos7420_async_clks_isp0,
		.num_async_clks = ARRAY_SIZE(exynos7420_async_clks_isp0),
		.sys_pwr_regs = exynos7420_sys_pwr_isp0,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_isp0),
	},
	{
		.name = "pd-isp1",
		.pmu_off = 0x4180,
		.lpi_busmaster_pa = 0x147e0000,
		.top_clks = exynos7420_top_clks_isp1,
		.num_top_clks = ARRAY_SIZE(exynos7420_top_clks_isp1),
		.local_clks = exynos7420_local_clks_isp1,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_isp1),
		.async_clks = exynos7420_async_clks_isp1,
		.num_async_clks = ARRAY_SIZE(exynos7420_async_clks_isp1),
		.sys_pwr_regs = exynos7420_sys_pwr_isp1,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_isp1),
	},
	{
		.name = "pd-cam0",
		.pmu_off = 0x4040,
		.lpi_busmaster_pa = 0x120e0000,
		.top_clks = exynos7420_top_clks_cam0,
		.num_top_clks = ARRAY_SIZE(exynos7420_top_clks_cam0),
		.local_clks = exynos7420_local_clks_cam0,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_cam0),
		.async_clks = exynos7420_async_clks_cam0,
		.num_async_clks = ARRAY_SIZE(exynos7420_async_clks_cam0),
		.sys_pwr_regs = exynos7420_sys_pwr_cam0,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_cam0),
	},
	{
		.name = "pd-cam1",
		.pmu_off = 0x4060,
		.lpi_busmaster_pa = 0x145e0000,
		.clear_a7is_option = true,
		.top_clks = exynos7420_top_clks_cam1,
		.num_top_clks = ARRAY_SIZE(exynos7420_top_clks_cam1),
		.local_clks = exynos7420_local_clks_cam1,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_cam1),
		.sys_pwr_regs = exynos7420_sys_pwr_cam1,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_cam1),
	},
};

static int exynos7420_rmw_phys(phys_addr_t pa, u32 set_mask, u32 clr_mask)
{
	void __iomem *base = ioremap(pa & PAGE_MASK, PAGE_SIZE);
	u32 off = pa & ~PAGE_MASK;
	u32 val;

	if (!base)
		return -ENOMEM;

	val = readl_relaxed(base + off);
	val |= set_mask;
	val &= ~clr_mask;
	writel_relaxed(val, base + off);
	readl_relaxed(base + off);
	iounmap(base);
	return 0;
}

static int exynos7420_apply_reg_list(const struct exynos7420_reg_bits *list,
				    size_t n, bool set)
{
	size_t i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = exynos7420_rmw_phys(list[i].pa,
					  set ? list[i].mask : 0,
					  set ? 0 : list[i].mask);
		if (ret)
			return ret;
	}
	return 0;
}

static int exynos7420_power_off_one_unused(void __iomem *pmu_base,
					  const struct exynos7420_unused_pd *pd)
{
	u32 stat = readl_relaxed(pmu_base + pd->pmu_off + 0x4) & 0xf;
	int timeout;

	if (stat == 0)
		return 0;

	exynos7420_apply_reg_list(pd->top_clks, pd->num_top_clks, true);
	udelay(10);
	exynos7420_apply_reg_list(pd->local_clks, pd->num_local_clks, true);
	if (pd->is_aud)
		exynos7420_rmw_phys(0x114c0200, 0, BIT(20) | BIT(16) | BIT(12));
	exynos7420_apply_reg_list(pd->async_clks, pd->num_async_clks, true);
	exynos7420_apply_reg_list(pd->sys_pwr_regs, pd->num_sys_pwr_regs, false);

	writel_relaxed(0x0102, pmu_base + pd->pmu_off + 0x8);
	writel_relaxed(0x0, pmu_base + pd->pmu_off + 0x0);

	timeout = 200;
	while ((readl_relaxed(pmu_base + pd->pmu_off + 0x4) & 0xf) != 0) {
		if (--timeout <= 0)
			break;
		usleep_range(10, 20);
	}

	if ((readl_relaxed(pmu_base + pd->pmu_off + 0x4) & 0xf) != 0 &&
	    pd->lpi_busmaster_pa) {
		exynos7420_rmw_phys(pd->lpi_busmaster_pa, 0xffffffff, 0);
		if (pd->clear_a7is_option) {
			u32 a7is = readl_relaxed(pmu_base + 0x2588);

			writel_relaxed(a7is & ~(0x7 << 15), pmu_base + 0x2588);
		}
		writel_relaxed(0x0, pmu_base + pd->pmu_off + 0x0);
		timeout = 200;
		while ((readl_relaxed(pmu_base + pd->pmu_off + 0x4) & 0xf) != 0) {
			if (--timeout <= 0)
				break;
			usleep_range(10, 20);
		}
	}

	stat = readl_relaxed(pmu_base + pd->pmu_off + 0x4) & 0xf;
	if (stat != 0) {
		pr_err("exynos-pd: failed to power off unused %s (stat=%#x)\n",
		       pd->name, readl_relaxed(pmu_base + pd->pmu_off + 0x4));
		return -EBUSY;
	}

	exynos7420_apply_reg_list(pd->top_clks, pd->num_top_clks, false);
	pr_info("exynos-pd: powered off unused %s (stat=%#x, top_clks gated)\n",
		pd->name, readl_relaxed(pmu_base + pd->pmu_off + 0x4));
	return 0;
}

static int exynos7420_power_on_one(void __iomem *pmu_base,
				   const struct exynos7420_unused_pd *cfg)
{
	int timeout = 200;

	/* 1. Enable top clocks */
	exynos7420_apply_reg_list(cfg->top_clks, cfg->num_top_clks, true);
	udelay(10);

	/* 2. Clear sys_pwr_regs before power on */
	exynos7420_apply_reg_list(cfg->sys_pwr_regs, cfg->num_sys_pwr_regs, false);

	/* 3. Write OPTION and PWR_CFG to power on */
	writel_relaxed(0x0102, pmu_base + cfg->pmu_off + 0x8);
	writel_relaxed(0xf, pmu_base + cfg->pmu_off + 0x0);

	/* 4. Poll status == 0xf (up to 2ms) */
	while ((readl_relaxed(pmu_base + cfg->pmu_off + 0x4) & 0xf) != 0xf) {
		if (--timeout <= 0)
			break;
		usleep_range(10, 20);
	}

	if ((readl_relaxed(pmu_base + cfg->pmu_off + 0x4) & 0xf) != 0xf) {
		pr_err("exynos-pd: failed to power on %s (stat=%#x)\n",
		       cfg->name, readl_relaxed(pmu_base + cfg->pmu_off + 0x4));
		return -ETIMEDOUT;
	}

	/* 4. Enable local clocks and async clocks */
	exynos7420_apply_reg_list(cfg->local_clks, cfg->num_local_clks, true);
	exynos7420_apply_reg_list(cfg->async_clks, cfg->num_async_clks, true);

	pr_info("exynos-pd: powered on %s (stat=%#x)\n",
		cfg->name, readl_relaxed(pmu_base + cfg->pmu_off + 0x4));
	return 0;
}

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
	const struct exynos7420_unused_pd *e7420_cfg;
	void __iomem *pmu_base;
};

static struct exynos_pm_domain *e7420_genpd[ARRAY_SIZE(exynos7420_unused_pds)];

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

	if (pd->e7420_cfg && pd->pmu_base)
		return power_on ? exynos7420_power_on_one(pd->pmu_base, pd->e7420_cfg)
				: exynos7420_power_off_one_unused(pd->pmu_base, pd->e7420_cfg);

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

static const char *exynos_get_domain_name(struct device *dev,
					  struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = kbasename(node->full_name);
	return devm_kstrdup_const(dev, name, GFP_KERNEL);
}

static int exynos_pd_probe(struct platform_device *pdev)
{
	const struct exynos_pm_domain_config *pm_domain_cfg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct of_phandle_args child, parent;
	struct exynos_pm_domain *pd;
	struct resource res;
	u32 pmu_off = 0;
	int on, ret;

	pm_domain_cfg = of_device_get_match_data(dev);
	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->pd.name = exynos_get_domain_name(dev, np);
	if (!pd->pd.name)
		return -ENOMEM;

	pd->base = of_iomap(np, 0);
	if (!pd->base)
		return -ENODEV;

	pd->pd.power_off = exynos_pd_power_off;
	pd->pd.power_on = exynos_pd_power_on;
	pd->local_pwr_cfg = pm_domain_cfg->local_pwr_cfg;
	pd->local_pwr_option = pm_domain_cfg->local_pwr_option;
	pd->pre = pm_domain_cfg->pre;
	pd->n_pre = pm_domain_cfg->n_pre;
	pd->post = pm_domain_cfg->post;
	pd->n_post = pm_domain_cfg->n_post;

	if (of_address_to_resource(np, 0, &res) == 0)
		pmu_off = res.start & 0xffff;

	if (of_device_is_compatible(np, "samsung,exynos7420-pd") && pmu_off != 0x4080) {
		int i;

		pd->n_pre = 0;
		pd->n_post = 0;
		pd->local_pwr_cfg = 0xf;
		pd->local_pwr_option = 0x0102;

		for (i = 0; i < ARRAY_SIZE(exynos7420_unused_pds); i++) {
			if (exynos7420_unused_pds[i].pmu_off == pmu_off) {
				pd->e7420_cfg = &exynos7420_unused_pds[i];
				pd->pmu_base = devm_ioremap(dev, 0x105c0000, 0x5000);
				e7420_genpd[i] = pd;
				break;
			}
		}
	}

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
	if (ret) {
		iounmap(pd->base);
		return ret;
	}

	if (of_parse_phandle_with_args(np, "power-domains",
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
	return 0;
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

/*
 * Initialize BLK_AUD (0x105c4000) and LPASS Top SFRs (0x11400000) before
 * postcore_initcall(pinctrl-samsung) and device_initcall(samsung-uart / i2s0):
 *  - Release gpz0/gpz1 pad retention (0x105c1340 = 0x1, 0x105c3028 = BIT(28))
 *  - Enable 32.768 kHz LPO clock on XCLKOUT2 for Broadcom BT/Wi-Fi
 *  - Deassert LPASS_CORE_SW_RESET (0x11400008 = 0x503: UART + I2S + MEM + DMA)
 *  - Route LPASS interrupts to AP GIC (0x11400058 = 0x73: DMA|GPIO|I2S|UART|SFR)
 *  - Bypass sysmmu_aud (0x114e0000 = 0x0) for direct ADMA physical addressing
 */
static int __init exynos7420_init_aud_lpass(void)
{
	void __iomem *pmu_base, *lpass_base, *smmu_base, *ccore_base, *rtc_base;
	u32 sw_reset, intr_mask;

	if (!of_machine_is_compatible("meizu,m86") &&
	    !of_machine_is_compatible("samsung,exynos7"))
		return 0;

	pmu_base = ioremap(0x105c0000, 0x5000);
	if (!pmu_base)
		return -ENOMEM;

	if ((readl_relaxed(pmu_base + 0x4004) & 0xf) != 0xf) {
		int timeout = 200;

		writel_relaxed(0x1, pmu_base + 0x1340);
		writel_relaxed(0x0102, pmu_base + 0x4008);
		writel_relaxed(0xf, pmu_base + 0x4000);
		while ((readl_relaxed(pmu_base + 0x4004) & 0xf) != 0xf && --timeout > 0)
			udelay(10);
	}

	writel_relaxed(0x1, pmu_base + 0x1340);
	writel_relaxed(BIT(28), pmu_base + 0x3028);

	/* Route 32.768 kHz RTC clock to XCLKOUT2 (PMU_RTC_CLKO_SEL = 0x0) */
	writel_relaxed(0x0, pmu_base + 0x001c);
	iounmap(pmu_base);

	/* Ensure PCLK_RTC (0x105b0900 BIT(2)) is enabled before touching RTCCON */
	ccore_base = ioremap(0x105b0000, 0x1000);
	rtc_base = ioremap(0x10590000, 0x100);
	if (ccore_base && rtc_base) {
		u32 pclk_ccore = readl_relaxed(ccore_base + 0x0900);

		writel_relaxed(pclk_ccore | BIT(2), ccore_base + 0x0900);
		readl_relaxed(ccore_base + 0x0900);
		writel_relaxed(readl_relaxed(rtc_base + 0x0040) | BIT(9),
			       rtc_base + 0x0040);
	}
	if (rtc_base)
		iounmap(rtc_base);
	if (ccore_base)
		iounmap(ccore_base);

	lpass_base = ioremap(0x11400000, 0x100);
	if (lpass_base) {
		writel_relaxed(0x0, lpass_base + 0x0008);
		udelay(100);
		/* LPASS_SW_RESET_UART(10) | I2S(8) | MEM(1) | DMA(0) */
		writel_relaxed(0x00000503, lpass_base + 0x0008);
		writel_relaxed(0x00000000, lpass_base + 0x0048);
		/* LPASS_INTR_DMA(6) | GPIO(5) | I2S(4) | UART(1) | SFR(0) */
		writel_relaxed(0x00000073, lpass_base + 0x0058);
		sw_reset = readl_relaxed(lpass_base + 0x0008);
		intr_mask = readl_relaxed(lpass_base + 0x0058);
		iounmap(lpass_base);

		pr_info("exynos-pd: initialized BLK_AUD LPASS (sw_reset=%#x intr_mask=%#x)\n",
			sw_reset, intr_mask);
	}

	smmu_base = ioremap(0x114e0000, 0x100);
	if (smmu_base) {
		writel_relaxed(0x0, smmu_base + 0x0000);
		iounmap(smmu_base);
	}

	return 0;
}
core_initcall(exynos7420_init_aud_lpass);

struct exynos7420_unused_pll {
	const char *name;
	phys_addr_t con_pa;
	phys_addr_t sel_pa;
	u32 sel_bit;
	phys_addr_t stat_pa;
	u32 stat_bit;
};

static const struct exynos7420_unused_pll exynos7420_unused_plls[] __initconst = {
	{ "aud_pll", 0x10570140, 0x10570204, 0,  0x10570404, 0  },
	{ "isp_pll", 0x105d0110, 0x105d0200, 24, 0x105d0400, 24 },
	{ "cam_pll", 0x105d0120, 0x105d0200, 20, 0x105d0400, 20 },
};

static void __init exynos7420_disable_unused_plls(void)
{
	size_t i;

	exynos7420_rmw_phys(0x105d0200, 0, BIT(0));
	udelay(5);
	exynos7420_rmw_phys(0x10570a04, 0, BIT(20));

	for (i = 0; i < ARRAY_SIZE(exynos7420_unused_plls); i++) {
		const struct exynos7420_unused_pll *p = &exynos7420_unused_plls[i];
		void __iomem *con_base = ioremap(p->con_pa & PAGE_MASK, PAGE_SIZE);
		u32 con_off = p->con_pa & ~PAGE_MASK;
		u32 sel_off = p->sel_pa & ~PAGE_MASK;
		u32 stat_off = p->stat_pa & ~PAGE_MASK;
		u32 con, sel, stat;
		int timeout = 1000;

		if (!con_base)
			continue;

		con = readl_relaxed(con_base + con_off);
		if (!(con & BIT(31))) {
			iounmap(con_base);
			continue;
		}

		sel = readl_relaxed(con_base + sel_off);
		sel &= ~BIT(p->sel_bit);
		writel_relaxed(sel, con_base + sel_off);

		while (--timeout > 0) {
			stat = (readl_relaxed(con_base + stat_off) >> p->stat_bit) & 0x7;
			if (stat == 0x1 || stat == 0x0)
				break;
			udelay(1);
		}

		con = readl_relaxed(con_base + con_off);
		con &= ~BIT(31);
		writel_relaxed(con, con_base + con_off);

		pr_info("exynos-pd: disabled unused %s (con=%#x)\n",
			p->name, readl_relaxed(con_base + con_off));
		iounmap(con_base);
	}
}

static const struct exynos7420_reg_bits exynos7420_unused_periph_clks[] __initconst = {
	/* CMU_PERIC0 (0x13610000): unused uart0, adcif, pwm (hsi2c0/9/10 active) */
	{ 0x13610900, BIT(21) | BIT(20) | BIT(16) },
	{ 0x13610a00, BIT(21) | BIT(16) },
	/* CMU_PERIC1 (0x14c80000): unused hsi2c3, uart1/3, spi0/1/2, pcm1, spdif (hsi2c2, spi4 active) */
	{ 0x14c80900, BIT(19) | BIT(18) | BIT(14) | BIT(13) | BIT(12) |
		      BIT(11) | BIT(9) | BIT(5) },
	{ 0x14c80a00, BIT(19) | BIT(18) | BIT(14) | BIT(13) | BIT(12) |
		      BIT(11) | BIT(9) },
	/* CMU_FSYS0 (0x10e90000) & CMU_FSYS1 (0x156e0000): unused mmc0/1 (mmc2 active) */
	{ 0x156e0800, BIT(30) | BIT(29) },
	/* CMU_TOP0 (0x105d0000): upstream sclk for unused peric0/1 blocks */
	{ 0x105d0a30, BIT(8) | BIT(4) },
	{ 0x105d0a34, BIT(20) | BIT(8) },
	{ 0x105d0a38, BIT(20) },
	{ 0x105d0a3c, BIT(16) | BIT(12) | BIT(4) },
	/* CMU_TOP1 (0x105e0000): upstream sclk for unused mmc0/1 */
	{ 0x105e0a2c, BIT(12) | BIT(0) },
};

static __init int exynos7420_power_off_unused_domains(void)
{
	void __iomem *pmu_base;
	size_t i;

	if (!of_machine_is_compatible("meizu,m86") &&
	    !of_machine_is_compatible("samsung,exynos7"))
		return 0;

	pmu_base = ioremap(0x105c0000, 0x5000);
	if (!pmu_base)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(exynos7420_unused_pds); i++) {
		exynos7420_power_off_one_unused(pmu_base, &exynos7420_unused_pds[i]);
		if (e7420_genpd[i])
			e7420_genpd[i]->pd.status = GENPD_STATE_OFF;
	}

	exynos7420_disable_unused_plls();
	exynos7420_apply_reg_list(exynos7420_unused_periph_clks,
				  ARRAY_SIZE(exynos7420_unused_periph_clks), false);
	pr_info("exynos-pd: gated unused PERIC0/PERIC1/FSYS0/FSYS1/TOP0/TOP1 peripheral clocks\n");

	iounmap(pmu_base);
	return 0;
}
late_initcall_sync(exynos7420_power_off_unused_domains);

