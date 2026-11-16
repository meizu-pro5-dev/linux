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

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/m86-tmu-transition.h>

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
	/* Value written to the OPTION register (base + 0x8); 0 = skip */
	u32 local_pwr_option;
};

struct exynos7420_reg_bits {
	phys_addr_t pa;
	u32 mask;
};

/* A CMU register kept across a power cycle, as the vendor sfr_save. */
struct exynos7420_sfr {
	phys_addr_t pa;
	u32 val;
	bool valid;
};

struct exynos7420_pd_cfg {
	const char *name;
	u32 pmu_off;
	bool is_aud;
	bool is_g3d;
	const struct exynos7420_reg_bits *local_clks;
	size_t num_local_clks;
	const struct exynos7420_reg_bits *async_clks;
	size_t num_async_clks;
	/* Set async_clks before power on, as the vendor on_pre does (DISP). */
	bool async_before_on;
	const struct exynos7420_reg_bits *sys_pwr_regs;
	size_t num_sys_pwr_regs;
	/* Registers of a CMU that no runtime PM restores, see the VPP entry. */
	struct exynos7420_sfr *save_regs;
	size_t num_save_regs;
	/* Content firewall IDs to protect again after every power on. */
	const u32 *cfw_ids;
	size_t num_cfw_ids;
};

/* Vendor include/linux/smc.h:48-49, 113-115. */
#define EXYNOS7420_SMC_SET_CFW_PROT	0xc20018a0
#define EXYNOS7420_SMC_DRM_CFW_PROT	0x10000000
#define EXYNOS7420_CFW_DISP_RW		3
#define EXYNOS7420_CFW_VPP0		5
#define EXYNOS7420_CFW_VPP1		6

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
static const struct exynos7420_reg_bits exynos7420_local_clks_cam1[] = {
	{ 0x145d0800, BIT(3) | BIT(2) | BIT(0) },
	{ 0x145d0804, BIT(28) | BIT(26) | BIT(25) | BIT(23) | BIT(22) | BIT(11) | BIT(10) },
	{ 0x145d080c, BIT(3) | BIT(2) },
	{ 0x145d0900, BIT(3) | BIT(2) | BIT(0) },
	{ 0x141f0800, BIT(2) | BIT(1) | BIT(0) },
	{ 0x141f0900, BIT(2) | BIT(1) | BIT(0) },
};
static const struct exynos7420_reg_bits exynos7420_async_clks_cam1[] = {
	{ 0x144d0804, BIT(16) },
};
static const struct exynos7420_reg_bits exynos7420_sys_pwr_cam1[] = {
	{ 0x105c144c, BIT(0) },
	{ 0x105c148c, BIT(0) },
	{ 0x105c14cc, BIT(0) },
	{ 0x105c150c, BIT(0) },
	{ 0x105c154c, BIT(5) | BIT(4) | BIT(1) | BIT(0) },
	{ 0x105c158c, BIT(0) },
};
static struct exynos7420_sfr exynos7420_save_cam1[] = {
	{ .pa = 0x145d0600 },	/* DIV_CAM1 */
	{ .pa = 0x145d0200 },	/* MUX_SEL_CAM10 */
	{ .pa = 0x145d0204 },	/* MUX_SEL_CAM11 */
	{ .pa = 0x145d0800 },	/* ENABLE_ACLK_CAM10 */
	{ .pa = 0x145d0804 },	/* ENABLE_ACLK_CAM11 */
	{ .pa = 0x145d0808 },	/* ENABLE_ACLK_CAM12 */
	{ .pa = 0x145d080c },	/* ENABLE_ACLK_CAM13 */
	{ .pa = 0x145d0900 },	/* ENABLE_PCLK_CAM10 */
	{ .pa = 0x145d0904 },	/* ENABLE_PCLK_CAM11 */
	{ .pa = 0x145d0908 },	/* ENABLE_PCLK_CAM12 */
	{ .pa = 0x145d0a00 },	/* ENABLE_SCLK_CAM10 */
	{ .pa = 0x145d0a08 },	/* ENABLE_SCLK_CAM12 */
};

static const struct exynos7420_reg_bits exynos7420_local_clks_vpp[] = {
	{ 0x13ed0800, BIT(15) | BIT(14) | BIT(3) | BIT(2) | BIT(1) | BIT(0) },
};
static const struct exynos7420_reg_bits exynos7420_sys_pwr_vpp[] = {
	{ 0x105c1478, BIT(0) },
	{ 0x105c14b8, BIT(0) },
	{ 0x105c14f8, BIT(0) },
	{ 0x105c1538, BIT(0) },
	{ 0x105c1578, BIT(9) | BIT(8) | BIT(5) | BIT(4) | BIT(1) | BIT(0) },
	{ 0x105c15b8, BIT(0) },
};
/*
 * CMU_VPP has no consumer, so its runtime PM never resumes it to restore
 * these after VPP was off: keep them here, as save_list_vpp does.
 */
static struct exynos7420_sfr exynos7420_save_vpp[] = {
	{ .pa = 0x13ed0600 },	/* DIV_VPP */
	{ .pa = 0x13ed0200 },	/* MUX_SEL_VPP */
	{ .pa = 0x13ed0800 },	/* ENABLE_ACLK_VPP */
	{ .pa = 0x13ed0900 },	/* ENABLE_PCLK_VPP */
};
static const u32 exynos7420_cfw_vpp[] = {
	EXYNOS7420_CFW_VPP0, EXYNOS7420_CFW_VPP1,
};

static const struct exynos7420_reg_bits exynos7420_local_clks_disp[] = {
	{ 0x13ad0800, BIT(31) | BIT(30) | BIT(29) | BIT(28) },
};
/* The DISP async bridges sit in CMU_VPP. */
static const struct exynos7420_reg_bits exynos7420_async_clks_disp[] = {
	{ 0x13ed0800, BIT(3) | BIT(2) | BIT(1) | BIT(0) },
};
static const struct exynos7420_reg_bits exynos7420_sys_pwr_disp[] = {
	{ 0x105c1450, BIT(0) },
	{ 0x105c1490, BIT(0) },
	{ 0x105c14d0, BIT(0) },
	{ 0x105c1510, BIT(0) },
	{ 0x105c1550, BIT(9) | BIT(8) | BIT(5) | BIT(4) | BIT(1) | BIT(0) },
	{ 0x105c1590, BIT(0) },
};
static const u32 exynos7420_cfw_disp[] = { EXYNOS7420_CFW_DISP_RW };

/*
 * BLK_AUD (vendor pm_domains-exynos7420-cal.h:424-449, lpass.c:586-654,
 * lpass-exynos7420.c:234-267): local clocks cover LPASS bridges/SRAM/DMAC/
 * SMMU/GPIO/UART/I2S; save_aud preserves CMU_AUD[0..5] and gpz0/gpz1 pads +
 * EINT configuration[6..23] across BLK_AUD power collapses.
 */
static const struct exynos7420_reg_bits exynos7420_local_clks_aud[] = {
	{ 0x114c0800, BIT(31) | BIT(30) | BIT(28) | BIT(27) | BIT(23) },
	{ 0x114c0900, BIT(31) | BIT(28) | BIT(27) | BIT(25) | BIT(20) },
	{ 0x114c0a00, BIT(29) | BIT(28) },
};
static const struct exynos7420_reg_bits exynos7420_sys_pwr_aud[] = {
	{ 0x105c1204, BIT(0) },
	{ 0x105c1340, BIT(0) },
	{ 0x105c1440, BIT(0) },
	{ 0x105c1480, BIT(0) },
	{ 0x105c14c0, BIT(0) },
	{ 0x105c1500, BIT(0) },
	{ 0x105c1540, BIT(1) | BIT(0) },
	{ 0x105c1580, BIT(0) },
};
static struct exynos7420_sfr exynos7420_save_aud[] = {
	/* [0..5] CMU_AUD */
	{ 0x114c0600 }, { 0x114c0604 }, { 0x114c0200 },
	{ 0x114c0800 }, { 0x114c0900 }, { 0x114c0a00 },
	/* [6..17] pinctrl_aud gpz0 & gpz1 CON/DAT/PUD/DRV/CON_PDN/PUD_PDN */
	{ 0x114b0000 }, { 0x114b0004 }, { 0x114b0008 },
	{ 0x114b000c }, { 0x114b0010 }, { 0x114b0014 },
	{ 0x114b0020 }, { 0x114b0024 }, { 0x114b0028 },
	{ 0x114b002c }, { 0x114b0030 }, { 0x114b0034 },
	/* [18..23] pinctrl_aud EINT CON/FLTCON/MASK */
	{ 0x114b0700 }, { 0x114b0704 },
	{ 0x114b0800 }, { 0x114b0804 },
	{ 0x114b0900 }, { 0x114b0904 },
};

static const struct exynos7420_reg_bits exynos7420_local_clks_g3d[] = {
	{ 0x14aa0800, BIT(1) },
};

static const struct exynos7420_reg_bits exynos7420_sys_pwr_g3d[] = {
	{ 0x105c1460, BIT(0) },
	{ 0x105c14a0, BIT(0) },
	{ 0x105c14e0, BIT(0) },
	{ 0x105c1520, BIT(0) },
	{ 0x105c1560, BIT(1) | BIT(0) },
	{ 0x105c15a0, BIT(0) },
};

static const struct exynos7420_pd_cfg exynos7420_pds[] = {
	{
		.name = "pd-aud",
		.pmu_off = 0x4000,
		.is_aud = true,
		.local_clks = exynos7420_local_clks_aud,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_aud),
		.sys_pwr_regs = exynos7420_sys_pwr_aud,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_aud),
		.save_regs = exynos7420_save_aud,
		.num_save_regs = ARRAY_SIZE(exynos7420_save_aud),
	},
	{
		.name = "pd-mfc",
		.pmu_off = 0x41a0,
		.local_clks = exynos7420_local_clks_mfc,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_mfc),
		.sys_pwr_regs = exynos7420_sys_pwr_mfc,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_mfc),
	},
	{
		.name = "pd-mscl",
		.pmu_off = 0x41c0,
		.local_clks = exynos7420_local_clks_mscl,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_mscl),
		.sys_pwr_regs = exynos7420_sys_pwr_mscl,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_mscl),
	},
	{
		.name = "pd-isp0",
		.pmu_off = 0x4140,
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
		.local_clks = exynos7420_local_clks_cam0,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_cam0),
		.async_clks = exynos7420_async_clks_cam0,
		.num_async_clks = ARRAY_SIZE(exynos7420_async_clks_cam0),
		.async_before_on = true,
		.sys_pwr_regs = exynos7420_sys_pwr_cam0,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_cam0),
	},
	{
		.name = "pd-cam1",
		.pmu_off = 0x4060,
		.local_clks = exynos7420_local_clks_cam1,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_cam1),
		.async_clks = exynos7420_async_clks_cam1,
		.num_async_clks = ARRAY_SIZE(exynos7420_async_clks_cam1),
		.async_before_on = true,
		.sys_pwr_regs = exynos7420_sys_pwr_cam1,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_cam1),
		.save_regs = exynos7420_save_cam1,
		.num_save_regs = ARRAY_SIZE(exynos7420_save_cam1),
	},
	{
		.name = "pd-vpp",
		.pmu_off = 0x41e0,
		.local_clks = exynos7420_local_clks_vpp,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_vpp),
		.sys_pwr_regs = exynos7420_sys_pwr_vpp,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_vpp),
		.save_regs = exynos7420_save_vpp,
		.num_save_regs = ARRAY_SIZE(exynos7420_save_vpp),
		.cfw_ids = exynos7420_cfw_vpp,
		.num_cfw_ids = ARRAY_SIZE(exynos7420_cfw_vpp),
	},
	{
		.name = "pd-disp",
		.pmu_off = 0x4080,
		.local_clks = exynos7420_local_clks_disp,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_disp),
		.async_clks = exynos7420_async_clks_disp,
		.num_async_clks = ARRAY_SIZE(exynos7420_async_clks_disp),
		.async_before_on = true,
		.sys_pwr_regs = exynos7420_sys_pwr_disp,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_disp),
		.cfw_ids = exynos7420_cfw_disp,
		.num_cfw_ids = ARRAY_SIZE(exynos7420_cfw_disp),
	},
	{
		.name = "pd-g3d",
		.pmu_off = 0x4100,
		.is_g3d = true,
		.local_clks = exynos7420_local_clks_g3d,
		.num_local_clks = ARRAY_SIZE(exynos7420_local_clks_g3d),
		.sys_pwr_regs = exynos7420_sys_pwr_g3d,
		.num_sys_pwr_regs = ARRAY_SIZE(exynos7420_sys_pwr_g3d),
	},
};

#define EXYNOS7420_PAGE_CACHE_MAX	32

struct exynos7420_page_map {
	phys_addr_t page;
	void __iomem *base;
};

static DEFINE_MUTEX(exynos7420_page_cache_lock);
static struct exynos7420_page_map exynos7420_page_cache[EXYNOS7420_PAGE_CACHE_MAX];
static size_t exynos7420_page_cache_cnt;

/* Cache permanent 4 KiB MMIO page mappings to avoid per-cycle ioremap/iounmap. */
static void __iomem *exynos7420_cached_page(phys_addr_t pa)
{
	phys_addr_t page = pa & PAGE_MASK;
	void __iomem *base = NULL;
	size_t i;

	mutex_lock(&exynos7420_page_cache_lock);
	for (i = 0; i < exynos7420_page_cache_cnt; i++) {
		if (exynos7420_page_cache[i].page == page) {
			base = exynos7420_page_cache[i].base;
			goto out;
		}
	}

	if (exynos7420_page_cache_cnt < EXYNOS7420_PAGE_CACHE_MAX) {
		base = ioremap(page, PAGE_SIZE);
		if (base) {
			exynos7420_page_cache[exynos7420_page_cache_cnt].page = page;
			exynos7420_page_cache[exynos7420_page_cache_cnt].base = base;
			exynos7420_page_cache_cnt++;
		}
	} else {
		pr_warn_once("exynos-pd: page cache full (%d entries)\n",
			     EXYNOS7420_PAGE_CACHE_MAX);
	}

out:
	mutex_unlock(&exynos7420_page_cache_lock);
	return base;
}

static int exynos7420_rmw_phys(phys_addr_t pa, u32 set_mask, u32 clr_mask)
{
	void __iomem *base = exynos7420_cached_page(pa);
	u32 off = pa & ~PAGE_MASK;
	u32 val;

	if (!base)
		return -ENOMEM;

	val = readl_relaxed(base + off);
	val |= set_mask;
	val &= ~clr_mask;
	writel_relaxed(val, base + off);
	readl_relaxed(base + off);
	return 0;
}

static void exynos7420_save_regs(const struct exynos7420_pd_cfg *pd)
{
	size_t i;

	for (i = 0; i < pd->num_save_regs; i++) {
		struct exynos7420_sfr *sfr = &pd->save_regs[i];
		void __iomem *base;

		/* Ensure pclk_gpio_aud (BIT 20) is gated on before reading gpz */
		if (pd->is_aud && i == 6)
			exynos7420_rmw_phys(0x114c0900, BIT(20), 0);

		base = exynos7420_cached_page(sfr->pa);
		if (!base)
			continue;
		sfr->val = readl_relaxed(base + (sfr->pa & ~PAGE_MASK));
		sfr->valid = true;
	}
}

static void exynos7420_restore_regs(const struct exynos7420_pd_cfg *pd)
{
	size_t i;

	for (i = 0; i < pd->num_save_regs; i++) {
		struct exynos7420_sfr *sfr = &pd->save_regs[i];
		void __iomem *base;

		if (!sfr->valid)
			continue;
		base = exynos7420_cached_page(sfr->pa);
		if (!base)
			continue;
		writel_relaxed(sfr->val, base + (sfr->pa & ~PAGE_MASK));
	}
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

static int exynos7420_apply_pmu_list(void __iomem *pmu_base,
				     const struct exynos7420_reg_bits *list,
				     size_t n, bool set)
{
	size_t i;

	for (i = 0; i < n; i++) {
		phys_addr_t pa = list[i].pa;
		u32 off, val;

		if (pa < 0x105c0000 || pa >= 0x105c5000)
			return -EINVAL;

		off = pa - 0x105c0000;
		val = readl_relaxed(pmu_base + off);
		if (set)
			val |= list[i].mask;
		else
			val &= ~list[i].mask;
		writel_relaxed(val, pmu_base + off);
		readl_relaxed(pmu_base + off);
	}
	return 0;
}

/*
 * Serialises every CAM/ISP/MFC/MSCL power sequence: the async_clks of one
 * domain sit in the CMU of another, so the owner's PMU STATUS must not change
 * between the check in exynos7420_apply_async_clks() and the register write.
 */
static DEFINE_MUTEX(exynos7420_pd_lock);

/* CMU blocks inside a gated power domain, keyed by their 64 KiB window. */
static const struct {
	phys_addr_t base;
	u32 pmu_off;
} exynos7420_cmu_owners[] = {
	{ 0x114c0000, 0x4000 },	/* CMU_AUD */
	{ 0x140f0000, 0x4040 },	/* CMU_CAM0_LOCAL */
	{ 0x144d0000, 0x4040 },	/* CMU_CAM0 */
	{ 0x141f0000, 0x4060 },	/* CMU_CAM1_LOCAL */
	{ 0x145d0000, 0x4060 },	/* CMU_CAM1 */
	{ 0x14290000, 0x4140 },	/* CMU_ISP0_LOCAL */
	{ 0x146d0000, 0x4140 },	/* CMU_ISP0 */
	{ 0x142f0000, 0x4180 },	/* CMU_ISP1_LOCAL */
	{ 0x147d0000, 0x4180 },	/* CMU_ISP1 */
	{ 0x13ed0000, 0x41e0 },	/* CMU_VPP */
};

/*
 * The ISP0 and ISP1 async_clks sit in both CMU_CAM0 and CMU_CAM1, the CAM0
 * ones in CMU_CAM1. Accessing a CMU whose domain is off raises a fatal AXI
 * SError, so, as the vendor exynos5_pd_enable_asyncbridge_clks() does, set
 * each gate only while its owning domain reports STATUS[3:0] == 0xf.
 * Called with exynos7420_pd_lock held.
 */
static void exynos7420_apply_async_clks(void __iomem *pmu_base,
					const struct exynos7420_pd_cfg *pd)
{
	size_t i, j;

	for (i = 0; i < pd->num_async_clks; i++) {
		phys_addr_t pa = pd->async_clks[i].pa;
		u32 stat;

		for (j = 0; j < ARRAY_SIZE(exynos7420_cmu_owners); j++)
			if ((pa & ~(phys_addr_t)0xffff) ==
			    exynos7420_cmu_owners[j].base)
				break;

		if (j < ARRAY_SIZE(exynos7420_cmu_owners)) {
			stat = readl_relaxed(pmu_base +
					     exynos7420_cmu_owners[j].pmu_off + 0x4);
			if ((stat & 0xf) != 0xf) {
				pr_debug("exynos-pd: %s skips async gate %pa, owner off (stat=%#x)\n",
					 pd->name, &pa, stat);
				continue;
			}
		}

		exynos7420_rmw_phys(pa, pd->async_clks[i].mask, 0);
	}
}

/*
 * Exynos specific wrapper around the generic power domain
 */
struct exynos_pm_domain {
	void __iomem *base;
	struct generic_pm_domain pd;
	u32 local_pwr_cfg;
	u32 local_pwr_option;
	const struct exynos7420_pd_cfg *e7420_cfg;
	void __iomem *pmu_base;
	struct clk_bulk_data *clks;
	int num_clks;
	bool clks_enabled;
};

static int exynos7420_g3d_tmu_power_off(void)
{
	if (IS_ENABLED(CONFIG_EXYNOS_THERMAL))
		return m86_tmu_gpu_power_off();
	return 0;
}

static int exynos7420_g3d_tmu_power_on(void)
{
	if (IS_ENABLED(CONFIG_EXYNOS_THERMAL))
		return m86_tmu_gpu_power_on();
	return 0;
}

static int exynos7420_g3d_restore_ema(void)
{
	void __iomem *sysreg_page = exynos7420_cached_page(0x10051224);

	if (!sysreg_page)
		return -ENOMEM;

	writel_relaxed(0x31, sysreg_page + 0x224);
	if (readl_relaxed(sysreg_page + 0x224) != 0x31)
		return -EIO;
	return 0;
}

static int exynos7420_power_off_one(struct exynos_pm_domain *pd)
{
	const struct exynos7420_pd_cfg *cfg = pd->e7420_cfg;
	void __iomem *pmu_base = pd->pmu_base;
	u32 stat = readl_relaxed(pmu_base + cfg->pmu_off + 0x4) & 0xf;
	int timeout;

	if (stat == 0) {
		if (pd->clks_enabled) {
			clk_bulk_disable(pd->num_clks, pd->clks);
			pd->clks_enabled = false;
		}
		return 0;
	}

	if (cfg->is_g3d) {
		int ret = exynos7420_g3d_tmu_power_off();

		if (ret)
			return ret;
	}

	if (!pd->clks_enabled && pd->num_clks > 0) {
		int ret = clk_bulk_enable(pd->num_clks, pd->clks);

		if (ret) {
			if (cfg->is_g3d)
				(void)exynos7420_g3d_tmu_power_on();
			return ret;
		}
		pd->clks_enabled = true;
	}
	udelay(10);
	exynos7420_save_regs(cfg);
	exynos7420_apply_reg_list(cfg->local_clks, cfg->num_local_clks, true);
	if (cfg->is_aud)
		exynos7420_rmw_phys(0x114c0200, 0, BIT(20) | BIT(16) | BIT(12));
	exynos7420_apply_async_clks(pmu_base, cfg);
	exynos7420_apply_pmu_list(pmu_base, cfg->sys_pwr_regs,
				  cfg->num_sys_pwr_regs, false);

	if (cfg->is_g3d)
		writel_relaxed(0xffff00c0, pmu_base + cfg->pmu_off + 0x10);
	writel_relaxed(0x0102, pmu_base + cfg->pmu_off + 0x8);
	writel_relaxed(0x0, pmu_base + cfg->pmu_off + 0x0);

	timeout = cfg->is_g3d ? 1000 : 200;
	while ((readl_relaxed(pmu_base + cfg->pmu_off + 0x4) & 0xf) != 0) {
		if (--timeout <= 0)
			break;
		usleep_range(10, 20);
	}

	/*
	 * Do not attempt the vendor exynos_pd_power_forced_off_pre() LPI
	 * busmaster mask fallback on timeout: in the vendor tree pd-cam0's
	 * 0x120e0000 was left over from Exynos 5433 (mach/map.h:89) and raises
	 * a fatal synchronous external abort / SError (ESR 0x96000010 /
	 * 0xbf000000), while the 0x144e0000/0x145e0000 LPI windows are blocked
	 * by TrustZone (SError ESR 0xbf000002) in non-secure Linux. Report
	 * -EBUSY cleanly instead.
	 */
	stat = readl_relaxed(pmu_base + cfg->pmu_off + 0x4) & 0xf;
	if (stat != 0) {
		pr_err("exynos-pd: failed to power off %s: conf %#x stat %#x option %#x\n",
		       cfg->name, readl_relaxed(pmu_base + cfg->pmu_off + 0x0),
		       readl_relaxed(pmu_base + cfg->pmu_off + 0x4),
		       readl_relaxed(pmu_base + cfg->pmu_off + 0x8));
		/*
		 * Writing the domain back on from here hangs the SoC on Exynos
		 * 7420 DISP, so only report it.
		 */
		return -EBUSY;
	}

	if (pd->clks_enabled) {
		clk_bulk_disable(pd->num_clks, pd->clks);
		pd->clks_enabled = false;
	}
	if (cfg->is_g3d)
		pr_debug("exynos-pd: powered off %s (stat=%#x, top_clks gated)\n",
			 cfg->name, readl_relaxed(pmu_base + cfg->pmu_off + 0x4));
	else
		pr_info("exynos-pd: powered off %s (stat=%#x, top_clks gated)\n",
			cfg->name, readl_relaxed(pmu_base + cfg->pmu_off + 0x4));
	return 0;
}

static void exynos7420_aud_post_power_on(void __iomem *pmu_base,
					 const struct exynos7420_pd_cfg *cfg)
{
	void __iomem *cmu = exynos7420_cached_page(0x114c0000);
	void __iomem *gpz = exynos7420_cached_page(0x114b0000);
	void __iomem *lpass = exynos7420_cached_page(0x11400000);
	void __iomem *smmu = exynos7420_cached_page(0x114e0000);
	size_t i;

	if (cmu) {
		if (cfg && cfg->save_regs[0].valid) {
			writel_relaxed(cfg->save_regs[0].val, cmu + 0x0600);
			writel_relaxed(cfg->save_regs[1].val, cmu + 0x0604);
			writel_relaxed(cfg->save_regs[2].val, cmu + 0x0200);
		} else {
			writel_relaxed(readl_relaxed(cmu + 0x0200) &
				       ~(BIT(20) | BIT(16) | BIT(12)),
				       cmu + 0x0200);
			writel_relaxed(0x0, cmu + 0x0600);
			writel_relaxed(0x0, cmu + 0x0604);
		}

		writel_relaxed(readl_relaxed(cmu + 0x0800) |
			       BIT(31) | BIT(30) | BIT(28) | BIT(27) | BIT(23),
			       cmu + 0x0800);
		writel_relaxed(readl_relaxed(cmu + 0x0900) |
			       BIT(31) | BIT(28) | BIT(27) | BIT(25) | BIT(20),
			       cmu + 0x0900);
		writel_relaxed(readl_relaxed(cmu + 0x0a00) | BIT(29) | BIT(28),
			       cmu + 0x0a00);
		readl_relaxed(cmu + 0x0a00);
	}

	if (gpz && cfg) {
		for (i = 6; i < cfg->num_save_regs; i++) {
			if (cfg->save_regs[i].valid)
				writel_relaxed(cfg->save_regs[i].val,
					       gpz + (cfg->save_regs[i].pa & 0xfff));
		}
		readl_relaxed(gpz + 0x0904);
	}

	if (lpass) {
		writel_relaxed(0x0, lpass + 0x0008);
		udelay(100);
		/* LPASS_SW_RESET_UART(10) | I2S(8) | MEM(1) | DMA(0) */
		writel_relaxed(0x00000503, lpass + 0x0008);
	}

	if (smmu)
		writel_relaxed(0x0, smmu + 0x0000);

	/* Release gpz0/gpz1 pad retention after pad/EINT registers are restored */
	writel_relaxed(0x1, pmu_base + 0x1340);
	writel_relaxed(BIT(28), pmu_base + 0x3028);

	if (lpass) {
		writel_relaxed(0x00000000, lpass + 0x0048);
		/* LPASS_INTR_DMA(6) | GPIO(5) | I2S(4) | UART(1) | SFR(0) */
		writel_relaxed(0x00000073, lpass + 0x0058);
		readl_relaxed(lpass + 0x0058);
	}

	if (cmu && cfg && cfg->save_regs[3].valid) {
		writel_relaxed(cfg->save_regs[3].val | BIT(30), cmu + 0x0800);
		writel_relaxed(cfg->save_regs[4].val, cmu + 0x0900);
		writel_relaxed(cfg->save_regs[5].val, cmu + 0x0a00);
		readl_relaxed(cmu + 0x0a00);
	}
}

static int exynos7420_power_on_one(struct exynos_pm_domain *pd)
{
	const struct exynos7420_pd_cfg *cfg = pd->e7420_cfg;
	void __iomem *pmu_base = pd->pmu_base;
	bool was_enabled = pd->clks_enabled;
	int timeout = 200;
	size_t i;

	/* 1. Enable top clocks */
	if (!pd->clks_enabled && pd->num_clks > 0) {
		int ret = clk_bulk_enable(pd->num_clks, pd->clks);

		if (ret)
			return ret;
		pd->clks_enabled = true;
	}
	udelay(10);
	if (cfg->async_before_on)
		exynos7420_apply_async_clks(pmu_base, cfg);

	/* 2. Clear sys_pwr_regs before power on */
	exynos7420_apply_pmu_list(pmu_base, cfg->sys_pwr_regs,
				  cfg->num_sys_pwr_regs, false);

	/* 3. Write OPTION and PWR_CFG to power on */
	if (cfg->is_g3d)
		writel_relaxed(0xffff00c0, pmu_base + cfg->pmu_off + 0x10);
	writel_relaxed(0x0102, pmu_base + cfg->pmu_off + 0x8);
	writel_relaxed(0xf, pmu_base + cfg->pmu_off + 0x0);

	/* 4. Poll status == 0xf (up to 2ms / 10ms for G3D) */
	if (cfg->is_g3d)
		timeout = 1000;
	while ((readl_relaxed(pmu_base + cfg->pmu_off + 0x4) & 0xf) != 0xf) {
		if (--timeout <= 0)
			break;
		usleep_range(10, 20);
	}

	if ((readl_relaxed(pmu_base + cfg->pmu_off + 0x4) & 0xf) != 0xf) {
		pr_err("exynos-pd: failed to power on %s (stat=%#x)\n",
		       cfg->name, readl_relaxed(pmu_base + cfg->pmu_off + 0x4));
		if (!was_enabled && pd->clks_enabled) {
			clk_bulk_disable(pd->num_clks, pd->clks);
			pd->clks_enabled = false;
		}
		return -ETIMEDOUT;
	}

	/* 5. Restore saved CMU registers, enable local and async clocks. */
	if (cfg->is_aud) {
		exynos7420_aud_post_power_on(pmu_base, cfg);
	} else {
		exynos7420_restore_regs(cfg);
		exynos7420_apply_reg_list(cfg->local_clks, cfg->num_local_clks,
					  true);
	}
	if (!cfg->async_before_on)
		exynos7420_apply_async_clks(pmu_base, cfg);

	if (cfg->is_g3d) {
		int ret = exynos7420_g3d_restore_ema();

		if (ret)
			return ret;
		ret = exynos7420_g3d_tmu_power_on();
		if (ret)
			return ret;
		pr_debug("exynos-pd: powered on %s (stat=%#x)\n",
			 cfg->name, readl_relaxed(pmu_base + cfg->pmu_off + 0x4));
	} else {
		pr_info("exynos-pd: powered on %s (stat=%#x)\n",
			cfg->name, readl_relaxed(pmu_base + cfg->pmu_off + 0x4));
	}

	/*
	 * 6. The secure world protects the block's content firewall again
	 * after every power on, as the vendor exynos_pd_power_on_post() asks
	 * it to (pm_domains-exynos7420.c:246-267); it answers 2.
	 */
	for (i = 0; i < cfg->num_cfw_ids; i++) {
		struct arm_smccc_res res;

		arm_smccc_smc(EXYNOS7420_SMC_SET_CFW_PROT,
			      EXYNOS7420_SMC_DRM_CFW_PROT, cfg->cfw_ids[i],
			      0, 0, 0, 0, 0, &res);
		if (res.a0 != 2) {
			pr_err("exynos-pd: %s: CFW %u protection failed: %ld\n",
			       cfg->name, cfg->cfw_ids[i], (long)res.a0);
			return -EBUSY;
		}
	}
	return 0;
}

static int exynos_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct exynos_pm_domain *pd;
	void __iomem *base;
	u32 timeout, pwr;
	char *op;

	pd = container_of(domain, struct exynos_pm_domain, pd);

	if (pd->e7420_cfg && pd->pmu_base) {
		int ret;

		mutex_lock(&exynos7420_pd_lock);
		ret = power_on ? exynos7420_power_on_one(pd)
			       : exynos7420_power_off_one(pd);
		mutex_unlock(&exynos7420_pd_lock);
		return ret;
	}

	base = pd->base;

	pwr = power_on ? pd->local_pwr_cfg : 0;

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

/* Domains without an exynos7420_pds[] entry only flip the local switch. */
static const struct exynos_pm_domain_config exynos7420_cfg = {
	.local_pwr_cfg		= 0xf,
	.local_pwr_option	= 0x0102,
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
		.data = &exynos7420_cfg,
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
	int i, on, ret;

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

	if (of_address_to_resource(np, 0, &res) == 0)
		pmu_off = res.start & 0xffff;

	if (of_device_is_compatible(np, "samsung,exynos7420-pd")) {
		for (i = 0; i < ARRAY_SIZE(exynos7420_pds); i++) {
			if (exynos7420_pds[i].pmu_off == pmu_off) {
				pd->e7420_cfg = &exynos7420_pds[i];
				pd->pmu_base = devm_ioremap(dev, 0x105c0000, 0x5000);
				break;
			}
		}
	}

	on = readl_relaxed(pd->base + 0x4) & pd->local_pwr_cfg;

	ret = devm_clk_bulk_get_all(dev, &pd->clks);
	if (ret < 0) {
		iounmap(pd->base);
		return ret;
	}
	pd->num_clks = ret;
	if (pd->num_clks > 0) {
		ret = clk_bulk_prepare(pd->num_clks, pd->clks);
		if (ret) {
			iounmap(pd->base);
			return ret;
		}
		if (on) {
			ret = clk_bulk_enable(pd->num_clks, pd->clks);
			if (ret) {
				clk_bulk_unprepare(pd->num_clks, pd->clks);
				iounmap(pd->base);
				return ret;
			}
			pd->clks_enabled = true;
		}
	}

	ret = pm_genpd_init(&pd->pd, NULL, !on);
	if (ret) {
		if (pd->clks_enabled)
			clk_bulk_disable(pd->num_clks, pd->clks);
		if (pd->num_clks > 0)
			clk_bulk_unprepare(pd->num_clks, pd->clks);
		iounmap(pd->base);
		return ret;
	}
	ret = of_genpd_add_provider_simple(np, &pd->pd);
	if (ret) {
		pm_genpd_remove(&pd->pd);
		if (pd->clks_enabled)
			clk_bulk_disable(pd->num_clks, pd->clks);
		if (pd->num_clks > 0)
			clk_bulk_unprepare(pd->num_clks, pd->clks);
		iounmap(pd->base);
		return ret;
	}

	/* Exynos7420 ISP0 has three parents (ISP1, CAM0, CAM1), as in the vendor DT. */
	child.np = np;
	child.args_count = 0;
	for (i = 0; !of_parse_phandle_with_args(np, "power-domains",
						"#power-domain-cells", i,
						&parent); i++) {
		if (of_genpd_add_subdomain(&parent, &child))
			pr_warn("%pOF failed to add subdomain: %pOF\n",
				parent.np, child.np);
		else
			pr_info("%pOF has as child subdomain: %pOF.\n",
				parent.np, child.np);
		of_node_put(parent.np);
	}

	platform_set_drvdata(pdev, pd);
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
	void __iomem *pmu_base, *lpass_base, *ccore_base, *rtc_base;
	u32 sw_reset, intr_mask, aud_status;

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
	aud_status = readl_relaxed(pmu_base + 0x4004);

	/* Route 32.768 kHz RTC clock to XCLKOUT2 (PMU_RTC_CLKO_SEL = 0x0) */
	writel_relaxed(0x0, pmu_base + 0x001c);

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

	/* LPASS and sysmmu_aud live in BLK_AUD: never touch them powered off */
	if ((aud_status & 0xf) != 0xf) {
		iounmap(pmu_base);
		pr_err("exynos-pd: BLK_AUD power-on timed out (status %#x), skipping LPASS init\n",
		       aud_status);
		return -ETIMEDOUT;
	}

	exynos7420_aud_post_power_on(pmu_base, NULL);
	iounmap(pmu_base);

	lpass_base = ioremap(0x11400000, 0x100);
	if (lpass_base) {
		sw_reset = readl_relaxed(lpass_base + 0x0008);
		intr_mask = readl_relaxed(lpass_base + 0x0058);
		iounmap(lpass_base);

		pr_info("exynos-pd: initialized BLK_AUD LPASS (sw_reset=%#x intr_mask=%#x)\n",
			sw_reset, intr_mask);
	}

	return 0;
}
core_initcall(exynos7420_init_aud_lpass);

