// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos7420 SRAM EMA/MCS Hardware Monitor Driver
 */

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/m86-asv-cal.h>

#define SYSREG_APOLLO_BASE		0x11800000
#define SYSREG_ATLAS_BASE		0x11900000
#define SYSREG_G3D_BASE			0x10050000

#define SYSREG_EMA_SIZE			0x2000

#define SYSREG_APOLLO_EMA_CON		0x1008
#define SYSREG_ATLAS_EMA_CON		0x1008
#define SYSREG_G3D_EMA_CON		0x1224

struct exynos7420_sram_ema_data {
	void __iomem *apollo_base;
	void __iomem *atlas_base;
	void __iomem *g3d_base;
	struct m86_asv_cal_info cal;
	bool cal_valid;
};

static struct exynos7420_sram_ema_data ema_data;

static int exynos7420_sram_ema_show(struct seq_file *s, void *data)
{
	u32 val_apollo = 0, val_atlas = 0, val_g3d = 0;

	if (ema_data.apollo_base)
		val_apollo = readl_relaxed(ema_data.apollo_base + SYSREG_APOLLO_EMA_CON);
	if (ema_data.atlas_base)
		val_atlas = readl_relaxed(ema_data.atlas_base + SYSREG_ATLAS_EMA_CON);
	if (ema_data.g3d_base)
		val_g3d = readl_relaxed(ema_data.g3d_base + SYSREG_G3D_EMA_CON);

	seq_puts(s, "Exynos7420 SRAM EMA/MCS Hardware Monitor\n\n");
	seq_puts(s, "Live Hardware Registers:\n");
	seq_printf(s, "  SYSREG_APOLLO [0x11801008]: 0x%08x\n", val_apollo);
	seq_printf(s, "  SYSREG_ATLAS  [0x11901008]: 0x%08x\n", val_atlas);
	seq_printf(s, "  SYSREG_G3D    [0x10051224]: 0x%08x\n", val_g3d);

	if (ema_data.cal_valid) {
		u32 l1, l2;

		seq_puts(s, "\nCalibrated ASV Voltage Thresholds & EMA/MCS Settings:\n");

		seq_puts(s, "  ATLAS (Big Cluster):\n");
		m86_asv_get_atlas_ema(&ema_data.cal, 1000000U, &l1, &l2);
		seq_printf(s, "    >= 1000mV: L1=0x%x, L2=0x%x\n", l1, l2);
		m86_asv_get_atlas_ema(&ema_data.cal, 900000U, &l1, &l2);
		seq_printf(s, "    >=  900mV: L1=0x%x, L2=0x%x\n", l1, l2);
		m86_asv_get_atlas_ema(&ema_data.cal, 800000U, &l1, &l2);
		seq_printf(s, "    >=  800mV: L1=0x%x, L2=0x%x\n", l1, l2);
		m86_asv_get_atlas_ema(&ema_data.cal, 700000U, &l1, &l2);
		seq_printf(s, "    <   800mV: L1=0x%x, L2=0x%x\n", l1, l2);

		seq_puts(s, "  APOLLO (Little Cluster):\n");
		seq_printf(s, "    >= 1000mV: EMA=0x%x\n", m86_asv_get_apollo_ema(&ema_data.cal, 1000000U));
		seq_printf(s, "    >=  900mV: EMA=0x%x\n", m86_asv_get_apollo_ema(&ema_data.cal, 900000U));
		seq_printf(s, "    >=  800mV: EMA=0x%x\n", m86_asv_get_apollo_ema(&ema_data.cal, 800000U));
		seq_printf(s, "    <   800mV: EMA=0x%x\n", m86_asv_get_apollo_ema(&ema_data.cal, 700000U));

		seq_puts(s, "  G3D (Mali GPU MCS):\n");
		seq_printf(s, "    >=  900mV: MCS_REG=0x%02x\n", m86_asv_get_g3d_mcs_reg(&ema_data.cal, 900000U));
		seq_printf(s, "    >=  800mV: MCS_REG=0x%02x\n", m86_asv_get_g3d_mcs_reg(&ema_data.cal, 800000U));
		seq_printf(s, "    >=  700mV: MCS_REG=0x%02x\n", m86_asv_get_g3d_mcs_reg(&ema_data.cal, 700000U));
		seq_printf(s, "    <   700mV: MCS_REG=0x%02x\n", m86_asv_get_g3d_mcs_reg(&ema_data.cal, 600000U));
	} else {
		seq_puts(s, "\nASV Calibration Data: Not available or uncalibrated\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(exynos7420_sram_ema);

static int __init exynos7420_sram_ema_init(void)
{
	if (!of_machine_is_compatible("samsung,exynos7420"))
		return 0;

	ema_data.apollo_base = ioremap(SYSREG_APOLLO_BASE, SYSREG_EMA_SIZE);
	if (!ema_data.apollo_base)
		pr_warn("exynos7420-sram-ema: failed to ioremap SYSREG_APOLLO\n");

	ema_data.atlas_base = ioremap(SYSREG_ATLAS_BASE, SYSREG_EMA_SIZE);
	if (!ema_data.atlas_base)
		pr_warn("exynos7420-sram-ema: failed to ioremap SYSREG_ATLAS\n");

	ema_data.g3d_base = ioremap(SYSREG_G3D_BASE, SYSREG_EMA_SIZE);
	if (!ema_data.g3d_base)
		pr_warn("exynos7420-sram-ema: failed to ioremap SYSREG_G3D\n");

	if (!m86_asv_cal_read(&ema_data.cal))
		ema_data.cal_valid = true;

	debugfs_create_file("exynos7420_sram_ema", 0444, NULL, NULL,
			    &exynos7420_sram_ema_fops);

	pr_info("exynos7420-sram-ema: initialized debugfs monitor /sys/kernel/debug/exynos7420_sram_ema\n");
	return 0;
}
late_initcall(exynos7420_sram_ema_init);

MODULE_DESCRIPTION("Samsung Exynos7420 SRAM EMA/MCS Hardware Monitor");
MODULE_LICENSE("GPL");
