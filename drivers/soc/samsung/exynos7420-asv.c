// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos7420 Adaptive Supply Voltage (ASV) & CHIPID Driver
 */

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/seq_file.h>
#include <linux/soc/samsung/exynos7420-asv.h>
#include <linux/m86-asv-cal.h>

#define EXYNOS7420_CHIPID_BASE		0x10000000
#define EXYNOS7420_CHIPID_SIZE		0x1000

#define EXYNOS7420_CHIPID_REG_PRO_ID	0x0000
#define EXYNOS7420_CHIPID_REG_PKG_ID	0x0004
#define EXYNOS7420_CHIPID_REG_LOT_ID	0x0014
#define EXYNOS7420_CHIPID_REG_114	0x0114
#define EXYNOS7420_CHIPID_REG_118	0x0118
#define EXYNOS7420_CHIPID_REG_11C	0x011c
#define EXYNOS7420_CHIPID_REG_120	0x0120
#define EXYNOS7420_CHIPID_REG_124	0x0124

struct exynos7420_asv_data {
	u32 pro_id;
	u32 pkg_id;
	u32 lot_id;
	u32 chipid_114;
	u32 chipid_118;
	u32 chipid_11c;
	u32 chipid_120;
	u32 chipid_124;
	struct m86_asv_cal_info cal;
	bool initialized;
};

static struct exynos7420_asv_data asv_data;

int exynos7420_asv_get_group(enum exynos7420_asv_domain dom, unsigned int level)
{
	u32 subgrp, raw;

	if (!asv_data.initialized)
		return -EAGAIN;

	if (dom >= EXYNOS7420_ASV_DOMAINS_NR)
		return -EINVAL;

	if (asv_data.cal.asv_group_type) {
		int grp = (int)asv_data.cal.main_group;
		if (asv_data.cal.main_ssa_minus)
			grp -= (int)asv_data.cal.main_ssa;
		else
			grp += (int)asv_data.cal.main_ssa;
		if (grp < 0)
			grp = 0;
		if (grp >= M86_ASV_GRP_NR)
			grp = M86_ASV_GRP_NR - 1;
		return grp;
	}

	/* level is the vendor SYSC_DVFS_L* index; always map it through the
	 * vendor sub-group table (the old "level < 3" shortcut selected the
	 * wrong nibble for BIG/LIT L0..L2).
	 */
	subgrp = m86_asv_match_subgrp((enum m86_asv_domain)dom, level);

	raw = m86_asv_domain_group_raw(&asv_data.cal, (enum m86_asv_domain)dom);
	return (raw >> (subgrp * 4U)) & 0xfU;
}
EXPORT_SYMBOL_GPL(exynos7420_asv_get_group);

/*
 * Return the single ASV/eFuse snapshot read at subsys_initcall. Consumers
 * (cpufreq, GPU DVFS, MIF/INT, SRAM EMA) must share it instead of issuing
 * 16 SMC calls each.
 */
int exynos7420_asv_get_cal(struct m86_asv_cal_info *cal)
{
	if (!asv_data.initialized)
		return -EAGAIN;

	*cal = asv_data.cal;
	return 0;
}
EXPORT_SYMBOL_GPL(exynos7420_asv_get_cal);

int exynos7420_asv_adjust_g3d_opps(struct device *dev)
{
	static const struct {
		unsigned long hz;
		u8 cal_lvl;
	} g3d_levels[] = {
		{ 266000000UL, 7 },
		{ 350000000UL, 6 },
		{ 420000000UL, 5 },
		{ 544000000UL, 4 },
		{ 600000000UL, 3 },
		{ 700000000UL, 2 },
		{ 772000000UL, 1 },
	};
	unsigned int i;
	u32 min_uv = U32_MAX, max_uv = 0;

	if (!asv_data.initialized)
		return -EPROBE_DEFER;

	for (i = 0; i < ARRAY_SIZE(g3d_levels); i++) {
		u32 lvl = g3d_levels[i].cal_lvl;
		u32 volt = m86_asv_lookup_volt(&asv_data.cal, M86_ASV_G3D, lvl,
					       m86_g3d_asv_volt_v15[lvl - 1]);
		int ret;

		ret = dev_pm_opp_adjust_voltage(dev, g3d_levels[i].hz,
						volt, volt, volt);
		if (ret == -ENODEV)
			continue;
		if (ret)
			return ret;

		if (volt < min_uv)
			min_uv = volt;
		if (volt > max_uv)
			max_uv = volt;
	}

	if (min_uv <= max_uv)
		dev_info(dev, "exynos7420-asv: G3D group=0x%03x OPPs adjusted (%u..%u uV)\n",
			 m86_asv_domain_group_raw(&asv_data.cal, M86_ASV_G3D),
			 min_uv, max_uv);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos7420_asv_adjust_g3d_opps);

static const char * const domain_names[] = {
	[EXYNOS7420_ASV_BIG] = "BIG (ATLAS)",
	[EXYNOS7420_ASV_LIT] = "LIT (APOLLO)",
	[EXYNOS7420_ASV_G3D] = "G3D",
	[EXYNOS7420_ASV_MIF] = "MIF",
	[EXYNOS7420_ASV_INT] = "INT",
};

static int exynos7420_asv_summary_show(struct seq_file *s, void *data)
{
	int d;

	if (!asv_data.initialized) {
		seq_puts(s, "Exynos7420 ASV not initialized\n");
		return 0;
	}

	seq_printf(s, "pkg_id: 0x%08x\n", asv_data.pkg_id);
	seq_printf(s, "fused: %u\n", asv_data.cal.asv_method);
	seq_printf(s, "table_ver: %u\n", asv_data.cal.effective_ver);
	seq_printf(s, "group_type: %u\n", asv_data.cal.asv_group_type);
	seq_printf(s, "shift_type: %u\n", asv_data.cal.shift_type);
	seq_printf(s, "ssa0_enable: %u\n", asv_data.cal.ssa0_enable);
	seq_printf(s, "ssa1_enable: %u\n", asv_data.cal.ssa1_enable);
	if (asv_data.cal.asv_group_type)
		seq_printf(s, "main_group: %u (ssa=%s%u)\n",
			   asv_data.cal.main_group,
			   asv_data.cal.main_ssa_minus ? "-" : "+",
			   asv_data.cal.main_ssa);

	seq_puts(s, "\nPer-domain ASV groups across levels:\n");
	for (d = 0; d < EXYNOS7420_ASV_DOMAINS_NR; d++) {
		u32 raw = m86_asv_domain_group_raw(&asv_data.cal, (enum m86_asv_domain)d);
		u32 ssa0 = m86_asv_domain_ssa0(&asv_data.cal, (enum m86_asv_domain)d);
		u32 ssa1 = m86_asv_domain_ssa1(&asv_data.cal, (enum m86_asv_domain)d);

		seq_printf(s, "  %-12s: raw=0x%03x subgrp0=%d subgrp1=%d subgrp2=%d (ssa0=%u ssa1=%u)\n",
			   domain_names[d], raw,
			   exynos7420_asv_get_group(d, 0),
			   exynos7420_asv_get_group(d, 1),
			   exynos7420_asv_get_group(d, 2),
			   ssa0, ssa1);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(exynos7420_asv_summary);

static int __init exynos7420_asv_init(void)
{
	void __iomem *chipid;
	int ret;

	if (!of_machine_is_compatible("samsung,exynos7420"))
		return 0;

	chipid = ioremap(EXYNOS7420_CHIPID_BASE, EXYNOS7420_CHIPID_SIZE);
	if (!chipid) {
		pr_err("exynos7420-asv: failed to ioremap CHIPID\n");
		return -ENOMEM;
	}

	asv_data.pro_id = readl(chipid + EXYNOS7420_CHIPID_REG_PRO_ID);
	asv_data.pkg_id = readl(chipid + EXYNOS7420_CHIPID_REG_PKG_ID);
	asv_data.lot_id = readl(chipid + EXYNOS7420_CHIPID_REG_LOT_ID);
	asv_data.chipid_114 = readl(chipid + EXYNOS7420_CHIPID_REG_114);
	asv_data.chipid_118 = readl(chipid + EXYNOS7420_CHIPID_REG_118);
	asv_data.chipid_11c = readl(chipid + EXYNOS7420_CHIPID_REG_11C);
	asv_data.chipid_120 = readl(chipid + EXYNOS7420_CHIPID_REG_120);
	asv_data.chipid_124 = readl(chipid + EXYNOS7420_CHIPID_REG_124);
	iounmap(chipid);

	ret = m86_asv_cal_read(&asv_data.cal);
	if (ret) {
		pr_err("exynos7420-asv: failed to read ASV eFuse: %d\n", ret);
		return ret;
	}

	asv_data.initialized = true;

	pr_info("exynos7420-asv: initialized (pkg_id=0x%08x table_ver=%u fused=%u type=%u)\n",
		asv_data.pkg_id, asv_data.cal.effective_ver,
		asv_data.cal.asv_method, asv_data.cal.asv_group_type);

	debugfs_create_file("summary", 0444,
			    debugfs_create_dir("exynos7420_asv", NULL),
			    NULL, &exynos7420_asv_summary_fops);

	return 0;
}
subsys_initcall(exynos7420_asv_init);

MODULE_DESCRIPTION("Samsung Exynos7420 ASV and CHIPID Driver");
MODULE_LICENSE("GPL");
