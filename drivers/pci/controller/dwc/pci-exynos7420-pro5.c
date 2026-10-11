// SPDX-License-Identifier: GPL-2.0-only
/*
 * PRO5 channel-1 enumeration candidate. See host-design.md for provenance.
 * PHY CAL and controller sequencing derived from Samsung vendor driver:
 * Copyright (C) 2013-2014 Samsung Electronics Co., Ltd.
 * Original PHY author: Kyoungil Kim <ki0351.kim@samsung.com>
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include "pcie-designware.h"

struct pro5_pcie {
	struct dw_pcie pci;
	void __iomem *elbi, *phy, *pcs;
	struct regmap *sysreg, *pmu, *cmu;
	struct clk_bulk_data clks[12];
	struct gpio_desc *perst, *reg_on;
	bool msi_installed;
};

static const char * const pro5_clock_names[] = {
	"aclk_xiu_wifi1x", "aclk_ahb2axi_pcie_wifi1",
	"aclk_pcie_wifi1_mstr_aclk", "aclk_pcie_wifi1_slv_aclk",
	"aclk_pcie_wifi1_dbi_aclk", "aclk_combo_phy_pcs_pclk_wifi1",
	"pclk_async_combo_phy_wifi1", "sclk_pcie_link_wifi1_gated",
	"sclk_combo_phy_wifi1_26m_gated", "sclk_phy_fsys1_gated",
	"phyclk_pcie_wifi1_tx0_gated", "phyclk_pcie_wifi1_rx0_gated",
};

/* Select only PCIe-specific mux fields, after manual probe logging begins. */
static int pro5_select_clock_parents(struct device *dev)
{
	static const char * const mux_names[] = {
		"combo-ref-mux", "phy-tx-mux", "phy-rx-mux", "phy-user-mux",
	};
	static const char * const parent_names[] = {
		"combo-ref-parent", "phy-tx-parent", "phy-rx-parent", "phy-user-parent",
	};
	struct clk *mux[4], *parent[4];
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(mux); i++) {
		mux[i] = devm_clk_get(dev, mux_names[i]);
		if (IS_ERR(mux[i]))
			return dev_err_probe(dev, PTR_ERR(mux[i]), "missing PCIe mux %s\n",
					     mux_names[i]);
		parent[i] = devm_clk_get(dev, parent_names[i]);
		if (IS_ERR(parent[i]))
			return dev_err_probe(dev, PTR_ERR(parent[i]), "missing parent %s\n",
					     parent_names[i]);
	}
	dev_info(dev, "stage 1: select four PCIe-only clock mux parents\n");
	for (i = 0; i < ARRAY_SIZE(mux); i++) {
		ret = clk_set_parent(mux[i], parent[i]);
		if (ret)
			return dev_err_probe(dev, ret, "cannot select parent for %s\n",
					     mux_names[i]);
	}
	return 0;
}

static struct pro5_pcie *to_pro5(struct dw_pcie *pci)
{
	return container_of(pci, struct pro5_pcie, pci);
}

/* Exact 24MHz Gen1 vendor CAL values, pci-exynos7420_cal.c:16-62. */
static int pro5_phy_init(struct pro5_pcie *p)
{
	static const u32 cmn[] = {
		0x01, 0x0f, 0xa6, 0x71, 0x90, 0x62, 0x20, 0x00, 0x00,
		0xa7, 0x0a, 0x37, 0x20, 0x08, 0xef, 0xfc, 0x96, 0x14,
		0x00, 0x10, 0x68, 0x01, 0x00, 0x00, 0x04, 0x20,
	};
	static const u32 trsv[] = {
		0x31, 0xf4, 0xf4, 0x80, 0x25, 0x40, 0xc0, 0x03, 0x35,
		0x55, 0x4c, 0xc3, 0x10, 0x54, 0x70, 0xc5, 0x00, 0x2f,
		0x44, 0xa4, 0x00, 0x3b, 0x34, 0xa7, 0x64, 0x00, 0x1f,
		0x83, 0x1b, 0x01, 0xe0, 0x00, 0x00, 0x02, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x1d, 0x00,
	};
	unsigned int i;
	int ret;

	ret = regmap_update_bits(p->sysreg, 0, GENMASK(5, 4), 0);
	if (ret)
		return ret;
	ret = regmap_update_bits(p->sysreg, 0, GENMASK(3, 2), BIT(2));
	if (ret)
		return ret;
	writel(1, p->elbi + 0x130);
	udelay(10);
	writel(0, p->elbi + 0x130);
	udelay(10);
	writel(1, p->elbi + 0x134);
	udelay(10);
	writel(0, p->elbi + 0x134);
	udelay(10);
	for (i = 0; i < ARRAY_SIZE(cmn); i++)
		writel(cmn[i], p->phy + i * 4);
	for (i = 0; i < ARRAY_SIZE(trsv); i++)
		writel(trsv[i], p->phy + (0x30 + i) * 4);
	writel(2, p->pcs);
	ret = regmap_update_bits(p->sysreg, 0, BIT(7), 0);
	if (ret)
		return ret;
	ret = regmap_update_bits(p->sysreg, 0, BIT(7), BIT(7));
	if (ret)
		return ret;
	udelay(20);
	return regmap_update_bits(p->sysreg, 0, BIT(7), 0);
}

/* 7420 uses exit-L1 guards, not the 5433 DBI sideband bit. */
static u32 pro5_read_dbi(struct dw_pcie *pci, void __iomem *base,
			 u32 reg, size_t size)
{
	struct pro5_pcie *p = to_pro5(pci);
	u32 val = ~0U;

	writel(1, p->elbi + 0x40);
	writel(0, p->elbi + 0xf4);
	dw_pcie_read(base + reg, size, &val);
	writel(1, p->elbi + 0xf4);
	writel(0, p->elbi + 0x40);
	return val;
}

static void pro5_write_dbi(struct dw_pcie *pci, void __iomem *base,
			  u32 reg, size_t size, u32 val)
{
	struct pro5_pcie *p = to_pro5(pci);

	writel(1, p->elbi + 0x40);
	writel(0, p->elbi + 0xf4);
	dw_pcie_write(base + reg, size, val);
	writel(1, p->elbi + 0xf4);
	writel(0, p->elbi + 0x40);
}

static int pro5_link_up(struct dw_pcie *pci)
{
	u32 state = readl(to_pro5(pci)->elbi + 0x74) & 0x1f;

	return state >= 0x0d && state <= 0x15;
}

static int pro5_validate_inbound(struct dw_pcie *pci)
{
	u32 saved, ctrl1, ctrl2, base_lo, base_hi, limit, target_lo, target_hi;
	u64 msi = pci->pp.msi_data;

	/* Viewport registers are relative to 0x904; every access is guarded. */
	saved = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT);
	dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT, PCIE_ATU_REGION_DIR_IB);
	ctrl1 = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_REGION_CTRL1);
	ctrl2 = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_REGION_CTRL2);
	base_lo = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_LOWER_BASE);
	base_hi = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_UPPER_BASE);
	limit = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_LIMIT);
	target_lo = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_LOWER_TARGET);
	target_hi = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_UPPER_TARGET);
	dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT, saved);
	dev_info(pci->dev, "pre-scan inbound0 ctrl=%08x/%08x base=%08x:%08x limit=%08x target=%08x:%08x MSI=%llx\n",
		 ctrl1, ctrl2, base_hi, base_lo, limit, target_hi, target_lo,
		 (unsigned long long)msi);
	if (ctrl1 != PCIE_ATU_TYPE_MEM || !(ctrl2 & PCIE_ATU_ENABLE) ||
	    (ctrl2 & PCIE_ATU_BAR_MODE_ENABLE) || base_lo != 0x40000000 ||
	    base_hi || limit != 0xffffffff || target_lo != 0x40000000 ||
	    target_hi || msi < 0x40000000ULL || msi > 0xffffffffULL)
		return dev_err_probe(pci->dev, -EINVAL,
			"inbound0/MSI target does not match verified 3GB identity DRAM aperture\n");
	return 0;
}

static void pro5_msi_irq_handler(struct irq_desc *desc);

static int pro5_start_link(struct dw_pcie *pci)
{
	struct pro5_pcie *p = to_pro5(pci);
	u32 state;
	u16 link_ctl;
	u8 cap;
	int ret;

	dev_info(pci->dev, "stage 7: root complex ready, disable ASPM and train Gen1\n");
	ret = pro5_validate_inbound(pci);
	if (ret)
		return ret;
	/* Standard PCIe capability: enumerate with ASPM disabled. */
	cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	if (!cap)
		return -ENODEV;
	link_ctl = dw_pcie_readw_dbi(pci, cap + PCI_EXP_LNKCTL);
	dw_pcie_writew_dbi(pci, cap + PCI_EXP_LNKCTL,
			    link_ctl & ~PCI_EXP_LNKCTL_ASPMC);
	writel(1, p->elbi + 0x2c);
	ret = readl_poll_timeout(p->elbi + 0x74, state,
		(state & 0x1f) >= 0x0d && (state & 0x1f) <= 0x15, 10, 10000);
	if (ret)
		dev_err(pci->dev, "PCIe link failed, LTSSM=0x%x\n", state & 0x1f);
	if (!ret) {
		if (!pci->pp.has_msi_ctrl || !pci->pp.irq_domain)
			return -ENODEV;
		/* DW MSI domain and target exist here; pci_host_probe has not run. */
		irq_set_chained_handler_and_data(pci->pp.msi_irq[0],
						pro5_msi_irq_handler, p);
		p->msi_installed = true;
		writel(readl(p->elbi + 0x004), p->elbi + 0x004);
		writel(readl(p->elbi + 0x000), p->elbi + 0x000);
		writel(BIT(1), p->elbi + 0x010);
		dev_info(pci->dev, "stage 8: pre-scan MSI IRQ=%d target=0x%llx LTSSM=0x%x\n",
			 pci->pp.msi_irq[0], (unsigned long long)pci->pp.msi_data,
			 state & 0x1f);
	}
	return ret;
}

static void pro5_stop_link(struct dw_pcie *pci)
{
	struct pro5_pcie *p = to_pro5(pci);

	writel(0, p->elbi + 0x010);
	if (p->msi_installed) {
		synchronize_irq(pci->pp.msi_irq[0]);
		/* DW frees the MSI domain and clears the handler next. */
		p->msi_installed = false;
	}
	writel(0, p->elbi + 0x2c);
}

static const struct dw_pcie_ops pro5_ops = {
	.read_dbi = pro5_read_dbi,
	.write_dbi = pro5_write_dbi,
	.link_up = pro5_link_up,
	.start_link = pro5_start_link,
	.stop_link = pro5_stop_link,
};

/* Same dispatch as DW's unexported dw_handle_msi_irq(), then ELBI W1C.
 * One32-vector bank suffices for the Broadcom single-MSI endpoint.
 */
static void pro5_msi_irq_handler(struct irq_desc *desc)
{
	struct pro5_pcie *p = irq_desc_get_handler_data(desc);
	struct dw_pcie_rp *pp = &p->pci.pp;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long status;
	unsigned int bit;
	u32 level, pulse;

	chained_irq_enter(chip, desc);
	status = dw_pcie_readl_dbi(&p->pci, PCIE_MSI_INTR0_STATUS);
	for_each_set_bit(bit, &status, MAX_MSI_IRQS_PER_CTRL)
		generic_handle_domain_irq(pp->irq_domain, bit);
	/* Vendor pci-exynos.c:646-663 acknowledges ELBI after MSI status. */
	level = readl(p->elbi + 0x004);
	writel(level, p->elbi + 0x004);
	pulse = readl(p->elbi + 0x000);
	writel(pulse, p->elbi + 0x000);
	chained_irq_exit(chip, desc);
}

/* Root config access must retain the 7420 exit-L1 DBI guard. */
static int pro5_rd_own_conf(struct pci_bus *bus, unsigned int devfn,
			 int where, int size, u32 *val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(bus->sysdata);

	if (PCI_SLOT(devfn)) {
		*val = ~0U;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	*val = dw_pcie_read_dbi(pci, where, size);
	return PCIBIOS_SUCCESSFUL;
}

static int pro5_wr_own_conf(struct pci_bus *bus, unsigned int devfn,
			 int where, int size, u32 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(bus->sysdata);

	if (PCI_SLOT(devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;
	dw_pcie_write_dbi(pci, where, size, val);
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops pro5_root_pci_ops = {
	.read = pro5_rd_own_conf,
	.write = pro5_wr_own_conf,
};

static int pro5_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct pro5_pcie *p = to_pro5(pci);
	u32 state;

	pp->bridge->ops = &pro5_root_pci_ops;
	/* Guarantee generic DW calls start_link and our pre-scan MSI hook. */
	writel(0, p->elbi + 0x2c);
	return readl_poll_timeout(p->elbi + 0x74, state,
		(state & 0x1f) < 0x0d || (state & 0x1f) > 0x15, 10, 10000);
}

static const struct dw_pcie_host_ops pro5_host_ops = {
	.host_init = pro5_host_init,
};

static void pro5_power_off(void *data)
{
	struct pro5_pcie *p = data;

	writel(0, p->elbi + 0x010);
	pro5_stop_link(&p->pci);
	gpiod_set_value_cansleep(p->perst, 1);
	gpiod_set_value_cansleep(p->reg_on, 0);
	clk_bulk_disable_unprepare(2, p->clks + 10);
	regmap_update_bits(p->pmu, 0x71c, BIT(0), 0);
	clk_bulk_disable_unprepare(10, p->clks);
}

static int pro5_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pro5_pcie *p;
	void __iomem **bases[3];
	static const char * const names[] = {
		"elbi", "phy", "pcs",
	};
	unsigned int i;
	u32 root_id, viewport;
	int ret;

	dev_info(dev, "stage 0: manual enumeration probe begins\n");
	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->pci.dev = dev;
	p->pci.ops = &pro5_ops;
	p->pci.pp.ops = &pro5_host_ops;
	platform_set_drvdata(pdev, p);
	if (!IS_ENABLED(CONFIG_PCI_MSI))
		return dev_err_probe(dev, -EOPNOTSUPP, "MSI kernel support required\n");
	p->pci.pp.msi_irq[0] = platform_get_irq_byname(pdev, "elbi");
	if (p->pci.pp.msi_irq[0] < 0)
		return dev_err_probe(dev, p->pci.pp.msi_irq[0], "ELBI IRQ missing\n");
	p->pci.pp.num_vectors = MAX_MSI_IRQS_PER_CTRL;
	bases[0] = &p->elbi;
	bases[1] = &p->phy;
	bases[2] = &p->pcs;
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		*bases[i] = devm_platform_ioremap_resource_byname(pdev, names[i]);
		if (IS_ERR(*bases[i]))
			return PTR_ERR(*bases[i]);
	}
	p->sysreg = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pcie-sysreg");
	if (IS_ERR(p->sysreg))
		return dev_err_probe(dev, PTR_ERR(p->sysreg), "SYSREG syscon missing\n");
	p->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pmu-syscon");
	if (IS_ERR(p->pmu))
		return dev_err_probe(dev, PTR_ERR(p->pmu), "PMU syscon missing\n");
	p->cmu = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pcie-cmu-control");
	if (IS_ERR(p->cmu))
		return dev_err_probe(dev, PTR_ERR(p->cmu), "CMU control syscon missing\n");
	p->pci.dbi_base = devm_platform_ioremap_resource_byname(pdev, "dbi");
	if (IS_ERR(p->pci.dbi_base))
		return PTR_ERR(p->pci.dbi_base);
	for (i = 0; i < ARRAY_SIZE(p->clks); i++)
		p->clks[i].id = pro5_clock_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(p->clks), p->clks);
	if (ret)
		return dev_err_probe(dev, ret, "missing channel-1 clocks\n");
	p->perst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(p->perst))
		return dev_err_probe(dev, PTR_ERR(p->perst), "PERST missing\n");
	p->reg_on = devm_gpiod_get(dev, "reg-on", GPIOD_OUT_LOW);
	if (IS_ERR(p->reg_on))
		return dev_err_probe(dev, PTR_ERR(p->reg_on), "REG_ON missing\n");
	ret = pro5_select_clock_parents(dev);
	if (ret)
		return ret;
	/* CAL is explicitly validated only for the vendor 24MHz Gen1 input. */
	if (clk_get_rate(p->clks[8].clk) != 24000000)
		return dev_err_probe(dev, -EINVAL, "PHY CAL requires a verified 24MHz input\n");
	dev_info(dev, "stage 1: parents selected and resources ready; PHY reference=%lu Hz\n",
		 clk_get_rate(p->clks[8].clk));
	dev_info(dev, "stage 2: enable controller/reference clocks and PMU isolation release\n");
	ret = clk_bulk_prepare_enable(10, p->clks);
	if (ret)
		return ret;
	ret = regmap_update_bits(p->pmu, 0x71c, BIT(0), BIT(0));
	if (ret) {
		clk_bulk_disable_unprepare(10, p->clks);
		return ret;
	}
	dev_info(dev, "stage 3: PHY power-down clear; REG_ON high and PERST release\n");
	/* pci-exynos.c:1113, PHY all-power-down clear. */
	writel(readl(p->phy + 0x15 * 4) & ~(0xf << 3), p->phy + 0x15 * 4);
	writel(0, p->phy + 0x4e * 4);
	writel(0, p->phy + 0x4f * 4);
	gpiod_set_value_cansleep(p->reg_on, 1);
	gpiod_set_value_cansleep(p->perst, 0);
	msleep(80);
	writel(1, p->elbi + 0xf4);
	writel(1, p->elbi + 0xf0);
	dev_info(dev, "stage 4: initialize CMU control and vendor 24MHz Gen1 PHY CAL\n");
	ret = regmap_update_bits(p->cmu, 0, BIT(0), BIT(0));
	if (!ret)
		ret = pro5_phy_init(p);
	if (!ret) {
		dev_info(dev, "stage 5: PHY CAL complete; enable PHY TX/RX clocks\n");
		ret = clk_bulk_prepare_enable(2, p->clks + 10);
	}
	if (ret) {
		gpiod_set_value_cansleep(p->perst, 1);
		gpiod_set_value_cansleep(p->reg_on, 0);
		regmap_update_bits(p->pmu, 0x71c, BIT(0), 0);
		clk_bulk_disable_unprepare(10, p->clks);
		return ret;
	}
	ret = devm_add_action_or_reset(dev, pro5_power_off, p);
	if (ret)
		return ret;
	writel(readl(p->elbi + 0x18) & ~BIT(1), p->elbi + 0x18);
	writel(1, p->elbi + 0x38);
	/* Keep interrupts masked while reading endpoint configuration space. */
	writel(0, p->elbi + 0x0c);
	writel(0, p->elbi + 0x10);
	writel(0, p->elbi + 0x14);
	dev_info(dev, "stage 6: all endpoint IRQs masked; initialize DesignWare bridge\n");
	/* Vendor7420 uses viewport iATU@0x900, within the4KiB DBI map.
	 * An all-ones read makes generic DW detection select unrolled iATU
	 * at DBI+3MiB. That address is not mapped or justified on this board.
	 */
	root_id = dw_pcie_readl_dbi(&p->pci, PCI_VENDOR_ID);
	viewport = dw_pcie_readl_dbi(&p->pci, PCIE_ATU_VIEWPORT);
	dev_info(dev, "stage 6: DBI vendor/device=0x%08x, iATU viewport=0x%08x\n",
		 root_id, viewport);
	if (root_id == ~0U || viewport == ~0U)
		return dev_err_probe(dev, -ENODEV,
			"invalid DBI/viewport read; refusing unverified unrolled iATU mapping\n");
	ret = dw_pcie_host_init(&p->pci.pp);
	if (ret)
		return ret;

	return 0;
}

static int pro5_remove(struct platform_device *pdev)
{
	struct pro5_pcie *p = platform_get_drvdata(pdev);

	dw_pcie_host_deinit(&p->pci.pp);
	return 0;
}

static int pro5_pcie_suspend_noirq(struct device *dev)
{
	/* Retain PCIe root complex, clocks, and L0 link state during s2idle to
	 * prevent AXI bus hang or lost link on BCM4356 endpoint.
	 */
	return 0;
}

static int pro5_pcie_resume_noirq(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops pro5_pcie_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(pro5_pcie_suspend_noirq,
				  pro5_pcie_resume_noirq)
};

static const struct of_device_id pro5_match[] = {
	{ .compatible = "samsung,exynos7420-pro5-pcie-enumeration" },
	{ }
};
/* No MODULE_DEVICE_TABLE alias: load this diagnostic driver only by insmod. */
static struct platform_driver pro5_driver = {
	.probe = pro5_probe,
	.remove = pro5_remove,
	.driver = {
		.name = "exynos7420-pro5-pcie-enumeration",
		.of_match_table = pro5_match,
		.pm = &pro5_pcie_pm_ops,
	},
};
module_platform_driver(pro5_driver);
MODULE_LICENSE("GPL");
