// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 * Author: Naveen Krishna Ch <naveenkrishna.ch@gmail.com>
*/

#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include "clk.h"
#include "clk-exynos-arm64.h"
#include <dt-bindings/clock/exynos7-clk.h>

/* Register Offset definitions for CMU_TOPC (0x10570000) */
#define CC_PLL_LOCK		0x0000
#define BUS0_PLL_LOCK		0x0004
#define BUS1_DPLL_LOCK		0x0008
#define MFC_PLL_LOCK		0x000C
#define AUD_PLL_LOCK		0x0010
#define CC_PLL_CON0		0x0100
#define BUS0_PLL_CON0		0x0110
#define BUS1_DPLL_CON0		0x0120
#define MFC_PLL_CON0		0x0130
#define AUD_PLL_CON0		0x0140
#define MUX_SEL_TOPC0		0x0200
#define MUX_SEL_TOPC1		0x0204
#define MUX_SEL_TOPC2		0x0208
#define MUX_SEL_TOPC3		0x020C
#define DIV_TOPC0		0x0600
#define DIV_TOPC1		0x0604
#define DIV_TOPC3		0x060C
#define ENABLE_ACLK_TOPC0	0x0800
#define ENABLE_ACLK_TOPC1	0x0804
#define ENABLE_SCLK_TOPC1	0x0A04

static const struct samsung_fixed_factor_clock topc_fixed_factor_clks[] __initconst = {
	FFACTOR(0, "ffac_topc_bus1_pll_div4", "ffac_topc_bus1_pll_div2", 1, 2, 0),
	FFACTOR(0, "ffac_topc_cc_pll_div4", "ffac_topc_cc_pll_div2", 1, 2, 0),
	FFACTOR(0, "ffac_topc_mfc_pll_div4", "ffac_topc_mfc_pll_div2", 1, 2, 0),
	FFACTOR(0, "ffac_topc_bus0_half_div2", "mout_topc_bus0_pll_half", 1, 2, 0),
	FFACTOR(0, "ffac_topc_bus0_half_div4", "ffac_topc_bus0_half_div2", 1, 2, 0),
	FFACTOR(0, "ffac_topc_bus0_pll_div2", "mout_topc_bus0_pll", 1, 2, 0),
	FFACTOR(0, "ffac_topc_bus0_pll_div4",
		"ffac_topc_bus0_pll_div2", 1, 2, 0),
	FFACTOR(0, "ffac_topc_bus1_pll_div2", "mout_topc_bus1_pll", 1, 2, 0),
	FFACTOR(0, "ffac_topc_cc_pll_div2", "mout_topc_cc_pll", 1, 2, 0),
	FFACTOR(0, "ffac_topc_mfc_pll_div2", "mout_topc_mfc_pll", 1, 2, 0),
};

/* List of parent clocks for Muxes in CMU_TOPC */
PNAME(mout_topc_aud_pll_ctrl_p)	= { "fin_pll", "fout_aud_pll" };
PNAME(mout_topc_bus0_pll_ctrl_p)	= { "fin_pll", "fout_bus0_pll" };
PNAME(mout_topc_bus1_pll_ctrl_p)	= { "fin_pll", "fout_bus1_pll" };
PNAME(mout_topc_cc_pll_ctrl_p)	= { "fin_pll", "fout_cc_pll" };
PNAME(mout_topc_mfc_pll_ctrl_p)	= { "fin_pll", "fout_mfc_pll" };

PNAME(mout_topc_group2) = { "mout_topc_bus0_pll_half",
	"mout_topc_bus1_pll_half", "mout_topc_cc_pll_half",
	"mout_topc_mfc_pll_half" };

PNAME(mout_topc_bus0_pll_half_p) = { "mout_topc_bus0_pll",
	"ffac_topc_bus0_pll_div2", "ffac_topc_bus0_pll_div4"};
PNAME(mout_topc_bus1_pll_half_p) = { "mout_topc_bus1_pll",
	"ffac_topc_bus1_pll_div2"};
PNAME(mout_topc_cc_pll_half_p) = { "mout_topc_cc_pll",
	"ffac_topc_cc_pll_div2"};
PNAME(mout_topc_mfc_pll_half_p) = { "mout_topc_mfc_pll",
	"ffac_topc_mfc_pll_div2"};


PNAME(mout_topc_bus0_pll_out_p) = {"mout_topc_bus0_pll",
	"ffac_topc_bus0_pll_div2"};

/* 7420 BUS0 selector has twelve sources, distinct from the four-source group. */
PNAME(mout_aclk_bus0_532_p) = { "mout_topc_bus0_pll_half",
	"mout_topc_bus1_pll", "mout_topc_cc_pll", "mout_topc_mfc_pll",
	"ffac_topc_bus0_half_div2", "ffac_topc_bus1_pll_div2",
	"ffac_topc_cc_pll_div2", "ffac_topc_mfc_pll_div2",
	"ffac_topc_bus0_half_div4", "ffac_topc_bus1_pll_div4",
	"ffac_topc_cc_pll_div4", "ffac_topc_mfc_pll_div4" };

static const unsigned long topc_clk_regs[] __initconst = {
	CC_PLL_LOCK,
	BUS0_PLL_LOCK,
	BUS1_DPLL_LOCK,
	MFC_PLL_LOCK,
	AUD_PLL_LOCK,
	CC_PLL_CON0,
	BUS0_PLL_CON0,
	BUS1_DPLL_CON0,
	MFC_PLL_CON0,
	AUD_PLL_CON0,
	MUX_SEL_TOPC0,
	MUX_SEL_TOPC1,
	MUX_SEL_TOPC2,
	MUX_SEL_TOPC3,
	DIV_TOPC0,
	DIV_TOPC1,
	DIV_TOPC3,
};

PNAME(m86_topc_cpu_safe_p) = { "mout_topc_bus0_pll",
	"ffac_topc_bus0_pll_div2", "mout_topc_bus1_pll", "mout_topc_cc_pll" };

static const struct samsung_mux_clock topc_mux_clks[] __initconst = {
	MUX_F(CLK_MOUT_SCLK_BUS_PLL_G3D, "mout_sclk_bus_pll_g3d", m86_topc_cpu_safe_p,
		MUX_SEL_TOPC1, 20, 2, CLK_GET_RATE_NOCACHE, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_BUS0_ATLAS, "mout_sclk_bus0_pll_atlas", m86_topc_cpu_safe_p,
		MUX_SEL_TOPC1, 4, 2, CLK_GET_RATE_NOCACHE, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_BUS0_APOLLO, "mout_sclk_bus0_pll_apollo", m86_topc_cpu_safe_p,
		MUX_SEL_TOPC1, 8, 2, CLK_GET_RATE_NOCACHE, CLK_MUX_READ_ONLY),
	MUX(CLK_MOUT_ACLK_BUS0_532, "mout_aclk_bus0_532", mout_aclk_bus0_532_p,
		MUX_SEL_TOPC2, 8, 4),
	MUX(0, "mout_topc_bus0_pll", mout_topc_bus0_pll_ctrl_p,
		MUX_SEL_TOPC0, 0, 1),
	MUX(0, "mout_topc_bus1_pll", mout_topc_bus1_pll_ctrl_p,
		MUX_SEL_TOPC0, 4, 1),
	MUX(0, "mout_topc_cc_pll", mout_topc_cc_pll_ctrl_p,
		MUX_SEL_TOPC0, 8, 1),
	MUX(0, "mout_topc_mfc_pll", mout_topc_mfc_pll_ctrl_p,
		MUX_SEL_TOPC0, 12, 1),
	MUX(0, "mout_topc_bus0_pll_half", mout_topc_bus0_pll_half_p,
		MUX_SEL_TOPC0, 16, 2),
	MUX(0, "mout_topc_bus1_pll_half", mout_topc_bus1_pll_half_p,
		MUX_SEL_TOPC0, 20, 1),
	MUX(0, "mout_topc_cc_pll_half", mout_topc_cc_pll_half_p,
		MUX_SEL_TOPC0, 24, 1),
	MUX(0, "mout_topc_mfc_pll_half", mout_topc_mfc_pll_half_p,
		MUX_SEL_TOPC0, 28, 1),

	MUX(0, "mout_topc_aud_pll", mout_topc_aud_pll_ctrl_p,
		MUX_SEL_TOPC1, 0, 1),
	MUX(0, "mout_topc_bus0_pll_out", mout_topc_bus0_pll_out_p,
		MUX_SEL_TOPC1, 16, 1),

	MUX(0, "mout_aclk_ccore_133", mout_topc_group2,	MUX_SEL_TOPC2, 4, 2),

	MUX(0, "mout_aclk_mfc_532", mout_topc_group2, MUX_SEL_TOPC3, 8, 2),
	MUX(0, "mout_aclk_mscl_532", mout_topc_group2, MUX_SEL_TOPC3, 20, 2),
	MUX(0, "mout_aclk_peris_66", mout_topc_group2, MUX_SEL_TOPC3, 24, 2),
};

static const struct samsung_div_clock topc_div_clks[] __initconst = {
	DIV_F(CLK_DOUT_SCLK_BUS_PLL_G3D, "dout_sclk_bus_pll_g3d",
		"mout_sclk_bus_pll_g3d", DIV_TOPC3, 20, 4,
		CLK_GET_RATE_NOCACHE, CLK_DIVIDER_READ_ONLY),
	DIV(CLK_DOUT_ACLK_BUS0_532, "dout_aclk_bus0_532", "mout_aclk_bus0_532",
		DIV_TOPC0, 8, 4),
	DIV(DOUT_ACLK_CCORE_133, "dout_aclk_ccore_133", "mout_aclk_ccore_133",
		DIV_TOPC0, 4, 4),

	DIV(DOUT_ACLK_MFC_532, "dout_aclk_mfc_532", "mout_aclk_mfc_532",
		DIV_TOPC1, 8, 4),
	DIV(DOUT_ACLK_MSCL_532, "dout_aclk_mscl_532", "mout_aclk_mscl_532",
		DIV_TOPC1, 20, 4),
	DIV(DOUT_ACLK_PERIS, "dout_aclk_peris_66", "mout_aclk_peris_66",
		DIV_TOPC1, 24, 4),

	DIV(DOUT_SCLK_BUS0_PLL, "dout_sclk_bus0_pll", "mout_topc_bus0_pll_out",
		DIV_TOPC3, 0, 4),
	DIV(DOUT_SCLK_BUS1_PLL, "dout_sclk_bus1_pll", "mout_topc_bus1_pll",
		DIV_TOPC3, 8, 4),
	DIV(DOUT_SCLK_CC_PLL, "dout_sclk_cc_pll", "mout_topc_cc_pll",
		DIV_TOPC3, 12, 4),
	DIV(DOUT_SCLK_MFC_PLL, "dout_sclk_mfc_pll", "mout_topc_mfc_pll",
		DIV_TOPC3, 16, 4),
	DIV(DOUT_SCLK_AUD_PLL, "dout_sclk_aud_pll", "mout_topc_aud_pll",
		DIV_TOPC3, 28, 4),
};

static const struct samsung_pll_rate_table pll1460x_24mhz_tbl[] __initconst = {
	PLL_36XX_RATE(24 * MHZ, 491519897, 20, 1, 0, 31457),
	{},
};

static const struct samsung_gate_clock topc_gate_clks[] __initconst = {
	GATE(CLK_TOP_SCLK_BUS_PLL_G3D, "top_sclk_bus_pll_g3d",
		"dout_sclk_bus_pll_g3d", ENABLE_SCLK_TOPC1, 9, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_ACLK_BUS0_532, "top_aclk_bus0_532", "dout_aclk_bus0_532",
		ENABLE_ACLK_TOPC0, 8, CLK_IS_CRITICAL, 0),
	GATE(ACLK_CCORE_133, "aclk_ccore_133", "dout_aclk_ccore_133",
		ENABLE_ACLK_TOPC0, 4, CLK_IS_CRITICAL, 0),

	GATE(ACLK_MFC_532, "aclk_mfc_532", "dout_aclk_mfc_532",
		ENABLE_ACLK_TOPC1, 8, 0, 0),
	GATE(ACLK_MSCL_532, "aclk_mscl_532", "dout_aclk_mscl_532",
		ENABLE_ACLK_TOPC1, 20, 0, 0),

	GATE(ACLK_PERIS_66, "aclk_peris_66", "dout_aclk_peris_66",
		ENABLE_ACLK_TOPC1, 24, CLK_IS_CRITICAL, 0),

	GATE(SCLK_AUD_PLL, "sclk_aud_pll", "dout_sclk_aud_pll",
		ENABLE_SCLK_TOPC1, 20, 0, 0),
	GATE(SCLK_MFC_PLL_B, "sclk_mfc_pll_b", "dout_sclk_mfc_pll",
		ENABLE_SCLK_TOPC1, 17, 0, 0),
	GATE(SCLK_MFC_PLL_A, "sclk_mfc_pll_a", "dout_sclk_mfc_pll",
		ENABLE_SCLK_TOPC1, 16, 0, 0),
	GATE(SCLK_BUS1_PLL_B, "sclk_bus1_pll_b", "dout_sclk_bus1_pll",
		ENABLE_SCLK_TOPC1, 13, 0, 0),
	GATE(SCLK_BUS1_PLL_A, "sclk_bus1_pll_a", "dout_sclk_bus1_pll",
		ENABLE_SCLK_TOPC1, 12, 0, 0),
	GATE(SCLK_BUS0_PLL_B, "sclk_bus0_pll_b", "dout_sclk_bus0_pll",
		ENABLE_SCLK_TOPC1, 5, 0, 0),
	GATE(SCLK_BUS0_PLL_A, "sclk_bus0_pll_a", "dout_sclk_bus0_pll",
		ENABLE_SCLK_TOPC1, 4, 0, 0),
	GATE(SCLK_CC_PLL_B, "sclk_cc_pll_b", "dout_sclk_cc_pll",
		ENABLE_SCLK_TOPC1, 1, 0, 0),
	GATE(SCLK_CC_PLL_A, "sclk_cc_pll_a", "dout_sclk_cc_pll",
		ENABLE_SCLK_TOPC1, 0, 0, 0),
};

static const struct samsung_pll_clock topc_pll_clks[] __initconst = {
	PLL(pll_1451x, 0, "fout_bus0_pll", "fin_pll", BUS0_PLL_LOCK,
		BUS0_PLL_CON0, NULL),
	PLL(pll_1452x, 0, "fout_cc_pll", "fin_pll", CC_PLL_LOCK,
		CC_PLL_CON0, NULL),
	PLL(pll_1452x, 0, "fout_bus1_pll", "fin_pll", BUS1_DPLL_LOCK,
		BUS1_DPLL_CON0, NULL),
	PLL(pll_1452x, 0, "fout_mfc_pll", "fin_pll", MFC_PLL_LOCK,
		MFC_PLL_CON0, NULL),
	PLL(pll_1460x, FOUT_AUD_PLL, "fout_aud_pll", "fin_pll", AUD_PLL_LOCK,
		AUD_PLL_CON0, pll1460x_24mhz_tbl),
};

static const struct samsung_cmu_info topc_cmu_info __initconst = {
	.pll_clks		= topc_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(topc_pll_clks),
	.mux_clks		= topc_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(topc_mux_clks),
	.div_clks		= topc_div_clks,
	.nr_div_clks		= ARRAY_SIZE(topc_div_clks),
	.gate_clks		= topc_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(topc_gate_clks),
	.fixed_factor_clks	= topc_fixed_factor_clks,
	.nr_fixed_factor_clks	= ARRAY_SIZE(topc_fixed_factor_clks),
	.nr_clk_ids		= TOPC_NR_CLK,
	.clk_regs		= topc_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(topc_clk_regs),
};

static void __init exynos7_clk_topc_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &topc_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_topc, "samsung,exynos7-clock-topc",
	exynos7_clk_topc_init);

/* Register Offset definitions for CMU_TOP0 (0x105D0000) */
#define MUX_SEL_TOP00			0x0200
#define MUX_SEL_TOP01			0x0204
#define MUX_SEL_TOP03			0x020C
#define MUX_SEL_TOP0_PERIC0		0x0230
#define MUX_SEL_TOP0_PERIC1		0x0234
#define MUX_SEL_TOP0_PERIC2		0x0238
#define MUX_SEL_TOP0_PERIC3		0x023C
#define MUX_SEL_TOP0_DISP		0x0220
#define DIV_TOP03			0x060C
#define DIV_TOP0_PERIC0			0x0630
#define DIV_TOP0_PERIC1			0x0634
#define DIV_TOP0_PERIC2			0x0638
#define DIV_TOP0_PERIC3			0x063C
#define DIV_TOP0_DISP			0x0620
#define ENABLE_ACLK_TOP03		0x080C
#define ENABLE_SCLK_TOP0_DISP		0x0A20
#define ENABLE_SCLK_TOP0_PERIC0		0x0A30
#define ENABLE_SCLK_TOP0_PERIC1		0x0A34
#define ENABLE_SCLK_TOP0_PERIC2		0x0A38
#define ENABLE_SCLK_TOP0_PERIC3		0x0A3C

/* List of parent clocks for Muxes in CMU_TOP0 */
PNAME(mout_top0_bus0_pll_user_p)	= { "fin_pll", "sclk_bus0_pll_a" };
PNAME(mout_top0_bus1_pll_user_p)	= { "fin_pll", "sclk_bus1_pll_a" };
PNAME(mout_top0_cc_pll_user_p)	= { "fin_pll", "sclk_cc_pll_a" };
PNAME(mout_top0_mfc_pll_user_p)	= { "fin_pll", "sclk_mfc_pll_a" };
PNAME(mout_top0_aud_pll_user_p)	= { "fin_pll", "sclk_aud_pll" };

PNAME(mout_top0_bus0_pll_half_p) = {"mout_top0_bus0_pll_user",
	"ffac_top0_bus0_pll_div2"};
PNAME(mout_top0_bus1_pll_half_p) = {"mout_top0_bus1_pll_user",
	"ffac_top0_bus1_pll_div2"};
PNAME(mout_top0_cc_pll_half_p) = {"mout_top0_cc_pll_user",
	"ffac_top0_cc_pll_div2"};
PNAME(mout_top0_mfc_pll_half_p) = {"mout_top0_mfc_pll_user",
	"ffac_top0_mfc_pll_div2"};

PNAME(mout_top0_group1) = {"mout_top0_bus0_pll_half",
	"mout_top0_bus1_pll_half", "mout_top0_cc_pll_half",
	"mout_top0_mfc_pll_half"};
PNAME(mout_top0_group3) = {"ioclk_audiocdclk0",
	"ioclk_audiocdclk1", "ioclk_spdif_extclk",
	"mout_top0_aud_pll_user", "mout_top0_bus0_pll_half",
	"mout_top0_bus1_pll_half"};
PNAME(mout_top0_group4) = {"ioclk_audiocdclk1", "mout_top0_aud_pll_user",
	"mout_top0_bus0_pll_half", "mout_top0_bus1_pll_half"};

PNAME(mout_top0_disp_p) = {"mout_top0_bus0_pll_half",
	"mout_top0_bus1_pll_half", "mout_top0_cc_pll_half",
	"mout_top0_mfc_pll_half"};
PNAME(mout_aclk_disp_400_p) = {"mout_top0_bus0_pll_user",
	"mout_top0_bus1_pll_user", "mout_top0_cc_pll_user",
	"mout_top0_mfc_pll_user", "ffac_top0_bus0_pll_div2",
	"ffac_top0_bus1_pll_div2", "ffac_top0_cc_pll_div2",
	"ffac_top0_mfc_pll_div2"};

static const unsigned long top0_clk_regs[] __initconst = {
	MUX_SEL_TOP00,
	MUX_SEL_TOP01,
	MUX_SEL_TOP03,
	MUX_SEL_TOP0_DISP,
	MUX_SEL_TOP0_PERIC0,
	MUX_SEL_TOP0_PERIC1,
	MUX_SEL_TOP0_PERIC2,
	MUX_SEL_TOP0_PERIC3,
	DIV_TOP03,
	DIV_TOP0_DISP,
	DIV_TOP0_PERIC0,
	DIV_TOP0_PERIC1,
	DIV_TOP0_PERIC2,
	DIV_TOP0_PERIC3,
	ENABLE_ACLK_TOP03,
	ENABLE_SCLK_TOP0_DISP,
	ENABLE_SCLK_TOP0_PERIC0,
	ENABLE_SCLK_TOP0_PERIC1,
	ENABLE_SCLK_TOP0_PERIC2,
	ENABLE_SCLK_TOP0_PERIC3,
};

static const struct samsung_mux_clock top0_mux_clks[] __initconst = {
	MUX(CLK_MOUT_SCLK_DSD, "mout_sclk_dsd", mout_top0_disp_p,
		MUX_SEL_TOP0_DISP, 16, 2),
	MUX(0, "mout_top0_aud_pll_user", mout_top0_aud_pll_user_p,
		MUX_SEL_TOP00, 0, 1),
	MUX(0, "mout_top0_mfc_pll_user", mout_top0_mfc_pll_user_p,
		MUX_SEL_TOP00, 4, 1),
	MUX(0, "mout_top0_cc_pll_user", mout_top0_cc_pll_user_p,
		MUX_SEL_TOP00, 8, 1),
	MUX(0, "mout_top0_bus1_pll_user", mout_top0_bus1_pll_user_p,
		MUX_SEL_TOP00, 12, 1),
	MUX(0, "mout_top0_bus0_pll_user", mout_top0_bus0_pll_user_p,
		MUX_SEL_TOP00, 16, 1),

	MUX(0, "mout_top0_mfc_pll_half", mout_top0_mfc_pll_half_p,
		MUX_SEL_TOP01, 4, 1),
	MUX(0, "mout_top0_cc_pll_half", mout_top0_cc_pll_half_p,
		MUX_SEL_TOP01, 8, 1),
	MUX(CLK_MOUT_TOP0_BUS1_PLL_HALF, "mout_top0_bus1_pll_half", mout_top0_bus1_pll_half_p,
		MUX_SEL_TOP01, 12, 1),
	MUX(0, "mout_top0_bus0_pll_half", mout_top0_bus0_pll_half_p,
		MUX_SEL_TOP01, 16, 1),

	MUX(0, "mout_aclk_peric1_66", mout_top0_group1, MUX_SEL_TOP03, 12, 2),
	MUX(0, "mout_aclk_peric0_66", mout_top0_group1, MUX_SEL_TOP03, 20, 2),

	MUX(0, "mout_sclk_spdif", mout_top0_group3, MUX_SEL_TOP0_PERIC0, 4, 3),
	MUX(0, "mout_sclk_pcm1", mout_top0_group4, MUX_SEL_TOP0_PERIC0, 8, 2),
	MUX(0, "mout_sclk_i2s1", mout_top0_group4, MUX_SEL_TOP0_PERIC0, 20, 2),

	MUX(0, "mout_sclk_spi1", mout_top0_group1, MUX_SEL_TOP0_PERIC1, 8, 2),
	MUX(0, "mout_sclk_spi0", mout_top0_group1, MUX_SEL_TOP0_PERIC1, 20, 2),

	MUX(0, "mout_sclk_spi3", mout_top0_group1, MUX_SEL_TOP0_PERIC2, 8, 2),
	MUX(0, "mout_sclk_spi2", mout_top0_group1, MUX_SEL_TOP0_PERIC2, 20, 2),
	MUX(0, "mout_sclk_uart3", mout_top0_group1, MUX_SEL_TOP0_PERIC3, 4, 2),
	MUX(0, "mout_sclk_uart2", mout_top0_group1, MUX_SEL_TOP0_PERIC3, 8, 2),
	MUX(0, "mout_sclk_uart1", mout_top0_group1, MUX_SEL_TOP0_PERIC3, 12, 2),
	MUX(0, "mout_sclk_uart0", mout_top0_group1, MUX_SEL_TOP0_PERIC3, 16, 2),
	MUX(0, "mout_sclk_spi4", mout_top0_group1, MUX_SEL_TOP0_PERIC3, 20, 2),

	MUX(CLK_MOUT_ACLK_DISP_400, "mout_aclk_disp_400", mout_aclk_disp_400_p,
		MUX_SEL_TOP03, 28, 3),
	MUX(CLK_MOUT_SCLK_DECON_INT_ECLK, "mout_sclk_decon_int_eclk",
		mout_top0_disp_p, MUX_SEL_TOP0_DISP, 28, 2),
	MUX(CLK_MOUT_SCLK_DECON_VCLK, "mout_sclk_decon_vclk",
		mout_top0_disp_p, MUX_SEL_TOP0_DISP, 20, 2),
};

static const struct samsung_div_clock top0_div_clks[] __initconst = {
	DIV(CLK_DOUT_SCLK_DSD, "dout_sclk_dsd", "mout_sclk_dsd",
		DIV_TOP0_DISP, 16, 4),
	DIV(DOUT_ACLK_PERIC1, "dout_aclk_peric1_66", "mout_aclk_peric1_66",
		DIV_TOP03, 12, 6),
	DIV(DOUT_ACLK_PERIC0, "dout_aclk_peric0_66", "mout_aclk_peric0_66",
		DIV_TOP03, 20, 6),

	DIV(0, "dout_sclk_spdif", "mout_sclk_spdif", DIV_TOP0_PERIC0, 4, 4),
	DIV(0, "dout_sclk_pcm1", "mout_sclk_pcm1", DIV_TOP0_PERIC0, 8, 12),
	DIV(0, "dout_sclk_i2s1", "mout_sclk_i2s1", DIV_TOP0_PERIC0, 20, 10),

	DIV(0, "dout_sclk_spi1", "mout_sclk_spi1", DIV_TOP0_PERIC1, 8, 12),
	DIV(0, "dout_sclk_spi0", "mout_sclk_spi0", DIV_TOP0_PERIC1, 20, 12),

	DIV(0, "dout_sclk_spi3", "mout_sclk_spi3", DIV_TOP0_PERIC2, 8, 12),
	DIV(0, "dout_sclk_spi2", "mout_sclk_spi2", DIV_TOP0_PERIC2, 20, 12),

	DIV(0, "dout_sclk_uart3", "mout_sclk_uart3", DIV_TOP0_PERIC3, 4, 4),
	DIV(0, "dout_sclk_uart2", "mout_sclk_uart2", DIV_TOP0_PERIC3, 8, 4),
	DIV(0, "dout_sclk_uart1", "mout_sclk_uart1", DIV_TOP0_PERIC3, 12, 4),
	DIV(0, "dout_sclk_uart0", "mout_sclk_uart0", DIV_TOP0_PERIC3, 16, 4),
	DIV(0, "dout_sclk_spi4", "mout_sclk_spi4", DIV_TOP0_PERIC3, 20, 12),

	DIV(CLK_DOUT_ACLK_DISP_400, "dout_aclk_disp_400", "mout_aclk_disp_400",
		DIV_TOP03, 28, 4),
	DIV(CLK_DOUT_SCLK_DECON_INT_ECLK, "dout_sclk_decon_int_eclk",
		"mout_sclk_decon_int_eclk", DIV_TOP0_DISP, 28, 4),
	DIV(CLK_DOUT_SCLK_DECON_VCLK, "dout_sclk_decon_vclk",
		"mout_sclk_decon_vclk", DIV_TOP0_DISP, 20, 4),
};

static const struct samsung_gate_clock top0_gate_clks[] __initconst = {
	GATE(CLK_TOP_SCLK_DSD, "top_sclk_dsd", "dout_sclk_dsd",
		ENABLE_SCLK_TOP0_DISP, 16, 0, 0),
	/*
	 * These clocks are required for the CMU_PERIC0/1 registers access,
	 * keep them enabled permanently until proper runtime PM support is
	 * added. Otherwise disabling a PERIC child clock (e.g. an UART gate)
	 * gates the CMU register interface and the next access hangs the bus.
	 */
	GATE(CLK_ACLK_PERIC0_66, "aclk_peric0_66", "dout_aclk_peric0_66",
		ENABLE_ACLK_TOP03, 20, CLK_SET_RATE_PARENT |
		CLK_IS_CRITICAL, 0),
	GATE(CLK_ACLK_PERIC1_66, "aclk_peric1_66", "dout_aclk_peric1_66",
		ENABLE_ACLK_TOP03, 12, CLK_SET_RATE_PARENT |
		CLK_IS_CRITICAL, 0),

	GATE(CLK_SCLK_SPDIF, "sclk_spdif", "dout_sclk_spdif",
		ENABLE_SCLK_TOP0_PERIC0, 4, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_SCLK_PCM1, "sclk_pcm1", "dout_sclk_pcm1",
		ENABLE_SCLK_TOP0_PERIC0, 8, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_SCLK_I2S1, "sclk_i2s1", "dout_sclk_i2s1",
		ENABLE_SCLK_TOP0_PERIC0, 20, CLK_SET_RATE_PARENT, 0),

	GATE(CLK_SCLK_SPI1, "sclk_spi1", "dout_sclk_spi1",
		ENABLE_SCLK_TOP0_PERIC1, 8, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_SCLK_SPI0, "sclk_spi0", "dout_sclk_spi0",
		ENABLE_SCLK_TOP0_PERIC1, 20, CLK_SET_RATE_PARENT, 0),

	GATE(CLK_SCLK_SPI3, "sclk_spi3", "dout_sclk_spi3",
		ENABLE_SCLK_TOP0_PERIC2, 8, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_SCLK_SPI2, "sclk_spi2", "dout_sclk_spi2",
		ENABLE_SCLK_TOP0_PERIC2, 20, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_SCLK_UART3, "sclk_uart3", "dout_sclk_uart3",
		ENABLE_SCLK_TOP0_PERIC3, 4, 0, 0),
	GATE(CLK_SCLK_UART2, "sclk_uart2", "dout_sclk_uart2",
		ENABLE_SCLK_TOP0_PERIC3, 8, 0, 0),
	GATE(CLK_SCLK_UART1, "sclk_uart1", "dout_sclk_uart1",
		ENABLE_SCLK_TOP0_PERIC3, 12, 0, 0),
	GATE(CLK_SCLK_UART0, "sclk_uart0", "dout_sclk_uart0",
		ENABLE_SCLK_TOP0_PERIC3, 16, 0, 0),
	GATE(CLK_SCLK_SPI4, "sclk_spi4", "dout_sclk_spi4",
		ENABLE_SCLK_TOP0_PERIC3, 20, CLK_SET_RATE_PARENT, 0),

	GATE(CLK_ACLK_DISP_400, "top_aclk_disp_400", "dout_aclk_disp_400",
		ENABLE_ACLK_TOP03, 28, 0, 0),
	GATE(CLK_SCLK_DECON_INT_ECLK, "top_sclk_decon_int_eclk",
		"dout_sclk_decon_int_eclk", ENABLE_SCLK_TOP0_DISP, 28, 0, 0),
	GATE(CLK_SCLK_DECON_VCLK, "top_sclk_decon_vclk",
		"dout_sclk_decon_vclk", ENABLE_SCLK_TOP0_DISP, 20, 0, 0),
};

static const struct samsung_fixed_factor_clock top0_fixed_factor_clks[] __initconst = {
	FFACTOR(0, "ffac_top0_bus0_pll_div2", "mout_top0_bus0_pll_user",
		1, 2, 0),
	FFACTOR(0, "ffac_top0_bus1_pll_div2", "mout_top0_bus1_pll_user",
		1, 2, 0),
	FFACTOR(0, "ffac_top0_cc_pll_div2", "mout_top0_cc_pll_user", 1, 2, 0),
	FFACTOR(0, "ffac_top0_mfc_pll_div2", "mout_top0_mfc_pll_user", 1, 2, 0),
};

static const struct samsung_cmu_info top0_cmu_info __initconst = {
	.mux_clks		= top0_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(top0_mux_clks),
	.div_clks		= top0_div_clks,
	.nr_div_clks		= ARRAY_SIZE(top0_div_clks),
	.gate_clks		= top0_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(top0_gate_clks),
	.fixed_factor_clks	= top0_fixed_factor_clks,
	.nr_fixed_factor_clks	= ARRAY_SIZE(top0_fixed_factor_clks),
	.nr_clk_ids		= TOP0_NR_CLK,
	.clk_regs		= top0_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(top0_clk_regs),
};

static void __init exynos7_clk_top0_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &top0_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_top0, "samsung,exynos7-clock-top0",
	exynos7_clk_top0_init);

/* Register Offset definitions for CMU_TOP1 (0x105E0000) */
#define MUX_SEL_TOP10			0x0200
#define MUX_SEL_TOP11			0x0204
#define MUX_SEL_TOP13			0x020C
#define MUX_SEL_TOP1_FSYS0		0x0224
#define MUX_SEL_TOP1_FSYS1		0x0228
#define MUX_SEL_TOP1_FSYS11		0x022C
#define DIV_TOP13			0x060C
#define DIV_TOP1_FSYS0			0x0624
#define DIV_TOP1_FSYS1			0x0628
#define DIV_TOP1_FSYS11			0x062C
#define ENABLE_ACLK_TOP13		0x080C
#define ENABLE_SCLK_TOP1_FSYS0		0x0A24
#define ENABLE_SCLK_TOP1_FSYS1		0x0A28
#define ENABLE_SCLK_TOP1_FSYS11		0x0A2C

/* List of parent clocks for Muxes in CMU_TOP1 */
PNAME(mout_top1_bus0_pll_user_p)	= { "fin_pll", "sclk_bus0_pll_b" };
PNAME(mout_top1_bus1_pll_user_p)	= { "fin_pll", "sclk_bus1_pll_b" };
PNAME(mout_top1_cc_pll_user_p)	= { "fin_pll", "sclk_cc_pll_b" };
PNAME(mout_top1_mfc_pll_user_p)	= { "fin_pll", "sclk_mfc_pll_b" };

PNAME(mout_top1_bus0_pll_half_p) = {"mout_top1_bus0_pll_user",
	"ffac_top1_bus0_pll_div2"};
PNAME(mout_top1_bus1_pll_half_p) = {"mout_top1_bus1_pll_user",
	"ffac_top1_bus1_pll_div2"};
PNAME(mout_top1_cc_pll_half_p) = {"mout_top1_cc_pll_user",
	"ffac_top1_cc_pll_div2"};
PNAME(mout_top1_mfc_pll_half_p) = {"mout_top1_mfc_pll_user",
	"ffac_top1_mfc_pll_div2"};

PNAME(mout_top1_group1) = {"mout_top1_bus0_pll_half",
	"mout_top1_bus1_pll_half", "mout_top1_cc_pll_half",
	"mout_top1_mfc_pll_half"};

static const unsigned long top1_clk_regs[] __initconst = {
	MUX_SEL_TOP10,
	MUX_SEL_TOP11,
	MUX_SEL_TOP13,
	MUX_SEL_TOP1_FSYS0,
	MUX_SEL_TOP1_FSYS1,
	MUX_SEL_TOP1_FSYS11,
	DIV_TOP13,
	DIV_TOP1_FSYS0,
	DIV_TOP1_FSYS1,
	DIV_TOP1_FSYS11,
	ENABLE_ACLK_TOP13,
	ENABLE_SCLK_TOP1_FSYS0,
	ENABLE_SCLK_TOP1_FSYS1,
	ENABLE_SCLK_TOP1_FSYS11,
};

static const struct samsung_mux_clock top1_mux_clks[] __initconst = {
	MUX(0, "mout_top1_mfc_pll_user", mout_top1_mfc_pll_user_p,
		MUX_SEL_TOP10, 4, 1),
	MUX(0, "mout_top1_cc_pll_user", mout_top1_cc_pll_user_p,
		MUX_SEL_TOP10, 8, 1),
	MUX(0, "mout_top1_bus1_pll_user", mout_top1_bus1_pll_user_p,
		MUX_SEL_TOP10, 12, 1),
	MUX(0, "mout_top1_bus0_pll_user", mout_top1_bus0_pll_user_p,
		MUX_SEL_TOP10, 16, 1),

	MUX(0, "mout_top1_mfc_pll_half", mout_top1_mfc_pll_half_p,
		MUX_SEL_TOP11, 4, 1),
	MUX(0, "mout_top1_cc_pll_half", mout_top1_cc_pll_half_p,
		MUX_SEL_TOP11, 8, 1),
	MUX(0, "mout_top1_bus1_pll_half", mout_top1_bus1_pll_half_p,
		MUX_SEL_TOP11, 12, 1),
	MUX(0, "mout_top1_bus0_pll_half", mout_top1_bus0_pll_half_p,
		MUX_SEL_TOP11, 16, 1),

	MUX(0, "mout_aclk_fsys1_200", mout_top1_group1, MUX_SEL_TOP13, 24, 2),
	MUX(0, "mout_aclk_fsys0_200", mout_top1_group1, MUX_SEL_TOP13, 28, 2),

	MUX(0, "mout_sclk_phy_fsys0_26m", mout_top1_group1,
		MUX_SEL_TOP1_FSYS0, 0, 2),
	MUX(0, "mout_sclk_mmc2", mout_top1_group1, MUX_SEL_TOP1_FSYS0, 16, 2),
	MUX(0, "mout_sclk_usbdrd300", mout_top1_group1,
		MUX_SEL_TOP1_FSYS0, 28, 2),

	MUX(0, "mout_sclk_phy_fsys1", mout_top1_group1,
		MUX_SEL_TOP1_FSYS1, 0, 2),
	MUX(0, "mout_sclk_ufsunipro20", mout_top1_group1,
		MUX_SEL_TOP1_FSYS1, 16, 2),

	MUX(0, "mout_sclk_mmc1", mout_top1_group1, MUX_SEL_TOP1_FSYS11, 0, 2),
	MUX(0, "mout_sclk_mmc0", mout_top1_group1, MUX_SEL_TOP1_FSYS11, 12, 2),
	MUX(0, "mout_sclk_phy_fsys1_26m", mout_top1_group1,
		MUX_SEL_TOP1_FSYS11, 24, 2),
};

static const struct samsung_div_clock top1_div_clks[] __initconst = {
	DIV(DOUT_ACLK_FSYS1_200, "dout_aclk_fsys1_200", "mout_aclk_fsys1_200",
		DIV_TOP13, 24, 4),
	DIV(DOUT_ACLK_FSYS0_200, "dout_aclk_fsys0_200", "mout_aclk_fsys0_200",
		DIV_TOP13, 28, 4),

	DIV(DOUT_SCLK_PHY_FSYS1, "dout_sclk_phy_fsys1",
		"mout_sclk_phy_fsys1", DIV_TOP1_FSYS1, 0, 6),

	DIV(DOUT_SCLK_UFSUNIPRO20, "dout_sclk_ufsunipro20",
		"mout_sclk_ufsunipro20",
		DIV_TOP1_FSYS1, 16, 6),

	DIV(DOUT_SCLK_MMC2, "dout_sclk_mmc2", "mout_sclk_mmc2",
		DIV_TOP1_FSYS0, 16, 10),
	DIV(0, "dout_sclk_usbdrd300", "mout_sclk_usbdrd300",
		DIV_TOP1_FSYS0, 28, 4),

	DIV(DOUT_SCLK_MMC1, "dout_sclk_mmc1", "mout_sclk_mmc1",
		DIV_TOP1_FSYS11, 0, 10),
	DIV(DOUT_SCLK_MMC0, "dout_sclk_mmc0", "mout_sclk_mmc0",
		DIV_TOP1_FSYS11, 12, 10),

	DIV(DOUT_SCLK_PHY_FSYS1_26M, "dout_sclk_phy_fsys1_26m",
		"mout_sclk_phy_fsys1_26m", DIV_TOP1_FSYS11, 24, 6),
};

static const struct samsung_gate_clock top1_gate_clks[] __initconst = {
	GATE(CLK_SCLK_MMC2, "sclk_mmc2", "dout_sclk_mmc2",
		ENABLE_SCLK_TOP1_FSYS0, 16, CLK_SET_RATE_PARENT, 0),
	GATE(0, "sclk_usbdrd300", "dout_sclk_usbdrd300",
		ENABLE_SCLK_TOP1_FSYS0, 28, 0, 0),

	GATE(CLK_SCLK_PHY_FSYS1, "sclk_phy_fsys1", "dout_sclk_phy_fsys1",
		ENABLE_SCLK_TOP1_FSYS1, 0, CLK_SET_RATE_PARENT, 0),

	GATE(CLK_SCLK_UFSUNIPRO20, "sclk_ufsunipro20", "dout_sclk_ufsunipro20",
		ENABLE_SCLK_TOP1_FSYS1, 16, CLK_SET_RATE_PARENT, 0),

	GATE(CLK_SCLK_MMC1, "sclk_mmc1", "dout_sclk_mmc1",
		ENABLE_SCLK_TOP1_FSYS11, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_SCLK_MMC0, "sclk_mmc0", "dout_sclk_mmc0",
		ENABLE_SCLK_TOP1_FSYS11, 12, CLK_SET_RATE_PARENT, 0),

	GATE(CLK_ACLK_FSYS0_200, "aclk_fsys0_200", "dout_aclk_fsys0_200",
		ENABLE_ACLK_TOP13, 28, CLK_SET_RATE_PARENT |
		CLK_IS_CRITICAL, 0),
	/*
	 * This clock is required for the CMU_FSYS1 registers access, keep it
	 * enabled permanently until proper runtime PM support is added.
	 */
	GATE(CLK_ACLK_FSYS1_200, "aclk_fsys1_200", "dout_aclk_fsys1_200",
		ENABLE_ACLK_TOP13, 24, CLK_SET_RATE_PARENT |
		CLK_IS_CRITICAL, 0),

	GATE(CLK_SCLK_PHY_FSYS1_26M, "sclk_phy_fsys1_26m",
		"dout_sclk_phy_fsys1_26m", ENABLE_SCLK_TOP1_FSYS11,
		24, CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_fixed_factor_clock top1_fixed_factor_clks[] __initconst = {
	FFACTOR(0, "ffac_top1_bus0_pll_div2", "mout_top1_bus0_pll_user",
		1, 2, 0),
	FFACTOR(0, "ffac_top1_bus1_pll_div2", "mout_top1_bus1_pll_user",
		1, 2, 0),
	FFACTOR(0, "ffac_top1_cc_pll_div2", "mout_top1_cc_pll_user", 1, 2, 0),
	FFACTOR(0, "ffac_top1_mfc_pll_div2", "mout_top1_mfc_pll_user", 1, 2, 0),
};

static const struct samsung_cmu_info top1_cmu_info __initconst = {
	.mux_clks		= top1_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(top1_mux_clks),
	.div_clks		= top1_div_clks,
	.nr_div_clks		= ARRAY_SIZE(top1_div_clks),
	.gate_clks		= top1_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(top1_gate_clks),
	.fixed_factor_clks	= top1_fixed_factor_clks,
	.nr_fixed_factor_clks	= ARRAY_SIZE(top1_fixed_factor_clks),
	.nr_clk_ids		= TOP1_NR_CLK,
	.clk_regs		= top1_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(top1_clk_regs),
};

static void __init exynos7_clk_top1_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &top1_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_top1, "samsung,exynos7-clock-top1",
	exynos7_clk_top1_init);

/* Register Offset definitions for CMU_CCORE (0x105B0000) */
#define MUX_SEL_CCORE			0x0200
#define DIV_CCORE			0x0600
#define ENABLE_ACLK_CCORE0		0x0800
#define ENABLE_ACLK_CCORE1		0x0804
#define ENABLE_PCLK_CCORE		0x0900

/*
 * List of parent clocks for Muxes in CMU_CCORE
 */
PNAME(mout_aclk_ccore_133_user_p)	= { "fin_pll", "aclk_ccore_133" };

static const unsigned long ccore_clk_regs[] __initconst = {
	MUX_SEL_CCORE,
	ENABLE_ACLK_CCORE0,
	ENABLE_PCLK_CCORE,
};

static const struct samsung_mux_clock ccore_mux_clks[] __initconst = {
	MUX(0, "mout_aclk_ccore_133_user", mout_aclk_ccore_133_user_p,
		MUX_SEL_CCORE, 1, 1),
};

static const struct samsung_gate_clock ccore_gate_clks[] __initconst = {
	GATE(PCLK_RTC, "pclk_rtc", "mout_aclk_ccore_133_user",
		ENABLE_PCLK_CCORE, 8, 0, 0),
	/*
	 * G3D async-bridge (LH) clocks.  The vendor enables them as part of the
	 * G3D power-on sequence; the GPU needs them for its bus accesses.  Keep
	 * them always on so the Mali GPU does not hit bus faults / job timeouts.
	 */
	GATE(0, "aclk_lh_g3d0", "mout_aclk_ccore_133_user",
		ENABLE_ACLK_CCORE0, 22, CLK_IS_CRITICAL, 0),
	GATE(0, "aclk_lh_g3d1", "mout_aclk_ccore_133_user",
		ENABLE_ACLK_CCORE0, 23, CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info ccore_cmu_info __initconst = {
	.mux_clks		= ccore_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(ccore_mux_clks),
	.gate_clks		= ccore_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(ccore_gate_clks),
	.nr_clk_ids		= CCORE_NR_CLK,
	.clk_regs		= ccore_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(ccore_clk_regs),
};

static void __init exynos7_clk_ccore_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &ccore_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_ccore, "samsung,exynos7-clock-ccore",
	exynos7_clk_ccore_init);

/* Register Offset definitions for CMU_PERIC0 (0x13610000) */
#define MUX_SEL_PERIC0			0x0200
#define ENABLE_PCLK_PERIC0		0x0900
#define ENABLE_SCLK_PERIC0		0x0A00

/* List of parent clocks for Muxes in CMU_PERIC0 */
PNAME(mout_aclk_peric0_66_user_p)	= { "fin_pll", "aclk_peric0_66" };
PNAME(mout_sclk_uart0_user_p)	= { "fin_pll", "sclk_uart0" };

static const unsigned long peric0_clk_regs[] __initconst = {
	MUX_SEL_PERIC0,
	ENABLE_PCLK_PERIC0,
	ENABLE_SCLK_PERIC0,
};

static const struct samsung_mux_clock peric0_mux_clks[] __initconst = {
	MUX(0, "mout_aclk_peric0_66_user", mout_aclk_peric0_66_user_p,
		MUX_SEL_PERIC0, 0, 1),
	MUX(0, "mout_sclk_uart0_user", mout_sclk_uart0_user_p,
		MUX_SEL_PERIC0, 16, 1),
};

static const struct samsung_gate_clock peric0_gate_clks[] __initconst = {
	GATE(PCLK_HSI2C0, "pclk_hsi2c0", "mout_aclk_peric0_66_user",
		ENABLE_PCLK_PERIC0, 8, 0, 0),
	GATE(PCLK_HSI2C1, "pclk_hsi2c1", "mout_aclk_peric0_66_user",
		ENABLE_PCLK_PERIC0, 9, 0, 0),
	GATE(PCLK_HSI2C4, "pclk_hsi2c4", "mout_aclk_peric0_66_user",
		ENABLE_PCLK_PERIC0, 10, 0, 0),
	GATE(PCLK_HSI2C5, "pclk_hsi2c5", "mout_aclk_peric0_66_user",
		ENABLE_PCLK_PERIC0, 11, 0, 0),
	GATE(PCLK_HSI2C9, "pclk_hsi2c9", "mout_aclk_peric0_66_user",
		ENABLE_PCLK_PERIC0, 12, 0, 0),
	GATE(PCLK_HSI2C10, "pclk_hsi2c10", "mout_aclk_peric0_66_user",
		ENABLE_PCLK_PERIC0, 13, 0, 0),
	GATE(PCLK_HSI2C11, "pclk_hsi2c11", "mout_aclk_peric0_66_user",
		ENABLE_PCLK_PERIC0, 14, 0, 0),
	GATE(PCLK_UART0, "pclk_uart0", "mout_aclk_peric0_66_user",
		ENABLE_PCLK_PERIC0, 16, 0, 0),
	GATE(PCLK_ADCIF, "pclk_adcif", "mout_aclk_peric0_66_user",
		ENABLE_PCLK_PERIC0, 20, 0, 0),
	GATE(PCLK_PWM, "pclk_pwm", "mout_aclk_peric0_66_user",
		ENABLE_PCLK_PERIC0, 21, 0, 0),

	GATE(SCLK_UART0, "sclk_uart0_user", "mout_sclk_uart0_user",
		ENABLE_SCLK_PERIC0, 16, 0, 0),
	GATE(SCLK_PWM, "sclk_pwm", "fin_pll", ENABLE_SCLK_PERIC0, 21, 0, 0),
};

static const struct samsung_cmu_info peric0_cmu_info __initconst = {
	.mux_clks		= peric0_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(peric0_mux_clks),
	.gate_clks		= peric0_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(peric0_gate_clks),
	.nr_clk_ids		= PERIC0_NR_CLK,
	.clk_regs		= peric0_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(peric0_clk_regs),
};

static void __init exynos7_clk_peric0_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &peric0_cmu_info);
}

/* Register Offset definitions for CMU_PERIC1 (0x14C80000) */
#define MUX_SEL_PERIC10			0x0200
#define MUX_SEL_PERIC11			0x0204
#define MUX_SEL_PERIC12			0x0208
#define ENABLE_PCLK_PERIC1		0x0900
#define ENABLE_SCLK_PERIC10		0x0A00

CLK_OF_DECLARE(exynos7_clk_peric0, "samsung,exynos7-clock-peric0",
	exynos7_clk_peric0_init);

/* List of parent clocks for Muxes in CMU_PERIC1 */
PNAME(mout_aclk_peric1_66_user_p)	= { "fin_pll", "aclk_peric1_66" };
PNAME(mout_sclk_uart1_user_p)	= { "fin_pll", "sclk_uart1" };
PNAME(mout_sclk_uart2_user_p)	= { "fin_pll", "sclk_uart2" };
PNAME(mout_sclk_uart3_user_p)	= { "fin_pll", "sclk_uart3" };
PNAME(mout_sclk_spi0_user_p)		= { "fin_pll", "sclk_spi0" };
PNAME(mout_sclk_spi1_user_p)		= { "fin_pll", "sclk_spi1" };
PNAME(mout_sclk_spi2_user_p)		= { "fin_pll", "sclk_spi2" };
PNAME(mout_sclk_spi3_user_p)		= { "fin_pll", "sclk_spi3" };
PNAME(mout_sclk_spi4_user_p)		= { "fin_pll", "sclk_spi4" };

static const unsigned long peric1_clk_regs[] __initconst = {
	MUX_SEL_PERIC10,
	MUX_SEL_PERIC11,
	MUX_SEL_PERIC12,
	ENABLE_PCLK_PERIC1,
	ENABLE_SCLK_PERIC10,
};

static const struct samsung_mux_clock peric1_mux_clks[] __initconst = {
	MUX(0, "mout_aclk_peric1_66_user", mout_aclk_peric1_66_user_p,
		MUX_SEL_PERIC10, 0, 1),

	MUX_F(0, "mout_sclk_spi0_user", mout_sclk_spi0_user_p,
		MUX_SEL_PERIC11, 0, 1, CLK_SET_RATE_PARENT, 0),
	MUX_F(0, "mout_sclk_spi1_user", mout_sclk_spi1_user_p,
		MUX_SEL_PERIC11, 4, 1, CLK_SET_RATE_PARENT, 0),
	MUX_F(0, "mout_sclk_spi2_user", mout_sclk_spi2_user_p,
		MUX_SEL_PERIC11, 8, 1, CLK_SET_RATE_PARENT, 0),
	MUX_F(0, "mout_sclk_spi3_user", mout_sclk_spi3_user_p,
		MUX_SEL_PERIC11, 12, 1, CLK_SET_RATE_PARENT, 0),
	MUX_F(0, "mout_sclk_spi4_user", mout_sclk_spi4_user_p,
		MUX_SEL_PERIC11, 16, 1, CLK_SET_RATE_PARENT, 0),
	MUX(0, "mout_sclk_uart1_user", mout_sclk_uart1_user_p,
		MUX_SEL_PERIC11, 20, 1),
	MUX(0, "mout_sclk_uart2_user", mout_sclk_uart2_user_p,
		MUX_SEL_PERIC11, 24, 1),
	MUX(0, "mout_sclk_uart3_user", mout_sclk_uart3_user_p,
		MUX_SEL_PERIC11, 28, 1),
};

static const struct samsung_gate_clock peric1_gate_clks[] __initconst = {
	GATE(PCLK_HSI2C2, "pclk_hsi2c2", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 4, 0, 0),
	GATE(PCLK_HSI2C3, "pclk_hsi2c3", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 5, 0, 0),
	GATE(PCLK_HSI2C6, "pclk_hsi2c6", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 6, 0, 0),
	GATE(PCLK_HSI2C7, "pclk_hsi2c7", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 7, 0, 0),
	GATE(PCLK_HSI2C8, "pclk_hsi2c8", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 8, 0, 0),
	GATE(PCLK_UART1, "pclk_uart1", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 9, 0, 0),
	GATE(PCLK_UART2, "pclk_uart2", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 10, 0, 0),
	GATE(PCLK_UART3, "pclk_uart3", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 11, 0, 0),
	GATE(PCLK_SPI0, "pclk_spi0", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 12, 0, 0),
	GATE(PCLK_SPI1, "pclk_spi1", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 13, 0, 0),
	GATE(PCLK_SPI2, "pclk_spi2", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 14, 0, 0),
	GATE(PCLK_SPI3, "pclk_spi3", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 15, 0, 0),
	GATE(PCLK_SPI4, "pclk_spi4", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 16, 0, 0),
	GATE(PCLK_I2S1, "pclk_i2s1", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 17, CLK_SET_RATE_PARENT, 0),
	GATE(PCLK_PCM1, "pclk_pcm1", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 18, 0, 0),
	GATE(PCLK_SPDIF, "pclk_spdif", "mout_aclk_peric1_66_user",
		ENABLE_PCLK_PERIC1, 19, 0, 0),

	GATE(SCLK_UART1, "sclk_uart1_user", "mout_sclk_uart1_user",
		ENABLE_SCLK_PERIC10, 9, 0, 0),
	GATE(SCLK_UART2, "sclk_uart2_user", "mout_sclk_uart2_user",
		ENABLE_SCLK_PERIC10, 10, 0, 0),
	GATE(SCLK_UART3, "sclk_uart3_user", "mout_sclk_uart3_user",
		ENABLE_SCLK_PERIC10, 11, 0, 0),
	GATE(SCLK_SPI0, "sclk_spi0_user", "mout_sclk_spi0_user",
		ENABLE_SCLK_PERIC10, 12, CLK_SET_RATE_PARENT, 0),
	GATE(SCLK_SPI1, "sclk_spi1_user", "mout_sclk_spi1_user",
		ENABLE_SCLK_PERIC10, 13, CLK_SET_RATE_PARENT, 0),
	GATE(SCLK_SPI2, "sclk_spi2_user", "mout_sclk_spi2_user",
		ENABLE_SCLK_PERIC10, 14, CLK_SET_RATE_PARENT, 0),
	GATE(SCLK_SPI3, "sclk_spi3_user", "mout_sclk_spi3_user",
		ENABLE_SCLK_PERIC10, 15, CLK_SET_RATE_PARENT, 0),
	GATE(SCLK_SPI4, "sclk_spi4_user", "mout_sclk_spi4_user",
		ENABLE_SCLK_PERIC10, 16, CLK_SET_RATE_PARENT, 0),
	GATE(SCLK_I2S1, "sclk_i2s1_user", "sclk_i2s1",
		ENABLE_SCLK_PERIC10, 17, CLK_SET_RATE_PARENT, 0),
	GATE(SCLK_PCM1, "sclk_pcm1_user", "sclk_pcm1",
		ENABLE_SCLK_PERIC10, 18, CLK_SET_RATE_PARENT, 0),
	GATE(SCLK_SPDIF, "sclk_spdif_user", "sclk_spdif",
		ENABLE_SCLK_PERIC10, 19, CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_cmu_info peric1_cmu_info __initconst = {
	.mux_clks		= peric1_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(peric1_mux_clks),
	.gate_clks		= peric1_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(peric1_gate_clks),
	.nr_clk_ids		= PERIC1_NR_CLK,
	.clk_regs		= peric1_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(peric1_clk_regs),
};

static void __init exynos7_clk_peric1_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &peric1_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_peric1, "samsung,exynos7-clock-peric1",
	exynos7_clk_peric1_init);

/* Register Offset definitions for CMU_PERIS (0x10040000) */
#define MUX_SEL_PERIS			0x0200
#define ENABLE_PCLK_PERIS		0x0900
#define ENABLE_PCLK_PERIS_SECURE_CHIPID	0x0910
#define ENABLE_SCLK_PERIS		0x0A00
#define ENABLE_SCLK_PERIS_SECURE_CHIPID	0x0A10

/* List of parent clocks for Muxes in CMU_PERIS */
PNAME(mout_aclk_peris_66_user_p) = { "fin_pll", "aclk_peris_66" };

static const unsigned long peris_clk_regs[] __initconst = {
	MUX_SEL_PERIS,
	ENABLE_PCLK_PERIS,
	ENABLE_PCLK_PERIS_SECURE_CHIPID,
	ENABLE_SCLK_PERIS,
	ENABLE_SCLK_PERIS_SECURE_CHIPID,
};

static const struct samsung_mux_clock peris_mux_clks[] __initconst = {
	MUX(0, "mout_aclk_peris_66_user",
		mout_aclk_peris_66_user_p, MUX_SEL_PERIS, 0, 1),
};

static const struct samsung_gate_clock peris_gate_clks[] __initconst = {
	GATE(PCLK_MCT, "pclk_mct", "mout_aclk_peris_66_user",
		ENABLE_PCLK_PERIS, 5, 0, 0),
	GATE(PCLK_WDT, "pclk_wdt", "mout_aclk_peris_66_user",
		ENABLE_PCLK_PERIS, 6, 0, 0),
	GATE(PCLK_TMU, "pclk_tmu_apbif", "mout_aclk_peris_66_user",
		ENABLE_PCLK_PERIS, 10, 0, 0),

	GATE(PCLK_CHIPID, "pclk_chipid", "mout_aclk_peris_66_user",
		ENABLE_PCLK_PERIS_SECURE_CHIPID, 0, 0, 0),
	GATE(SCLK_CHIPID, "sclk_chipid", "fin_pll",
		ENABLE_SCLK_PERIS_SECURE_CHIPID, 0, 0, 0),

	GATE(SCLK_TMU, "sclk_tmu", "fin_pll", ENABLE_SCLK_PERIS, 10, 0, 0),
};

static const struct samsung_cmu_info peris_cmu_info __initconst = {
	.mux_clks		= peris_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(peris_mux_clks),
	.gate_clks		= peris_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(peris_gate_clks),
	.nr_clk_ids		= PERIS_NR_CLK,
	.clk_regs		= peris_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(peris_clk_regs),
};

static void __init exynos7_clk_peris_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &peris_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_peris, "samsung,exynos7-clock-peris",
	exynos7_clk_peris_init);

/* Register Offset definitions for CMU_FSYS0 (0x10E90000) */
#define MUX_SEL_FSYS00			0x0200
#define MUX_SEL_FSYS01			0x0204
#define MUX_SEL_FSYS02			0x0208
#define ENABLE_ACLK_FSYS00		0x0800
#define ENABLE_ACLK_FSYS01		0x0804
#define ENABLE_SCLK_FSYS01		0x0A04
#define ENABLE_SCLK_FSYS02		0x0A08
#define ENABLE_SCLK_FSYS04		0x0A10

/*
 * List of parent clocks for Muxes in CMU_FSYS0
 */
PNAME(mout_aclk_fsys0_200_user_p)	= { "fin_pll", "aclk_fsys0_200" };
PNAME(mout_sclk_mmc2_user_p)		= { "fin_pll", "sclk_mmc2" };

PNAME(mout_sclk_usbdrd300_user_p)	= { "fin_pll", "sclk_usbdrd300" };
PNAME(mout_phyclk_usbdrd300_udrd30_phyclk_user_p)	= { "fin_pll",
				"phyclk_usbdrd300_udrd30_phyclock" };
PNAME(mout_phyclk_usbdrd300_udrd30_pipe_pclk_user_p)	= { "fin_pll",
				"phyclk_usbdrd300_udrd30_pipe_pclk" };

/* fixed rate clocks used in the FSYS0 block */
static const struct samsung_fixed_rate_clock fixed_rate_clks_fsys0[] __initconst = {
	FRATE(0, "phyclk_usbdrd300_udrd30_phyclock", NULL, 0, 60000000),
	FRATE(0, "phyclk_usbdrd300_udrd30_pipe_pclk", NULL, 0, 125000000),
};

static const unsigned long fsys0_clk_regs[] __initconst = {
	MUX_SEL_FSYS00,
	MUX_SEL_FSYS01,
	MUX_SEL_FSYS02,
	ENABLE_ACLK_FSYS00,
	ENABLE_ACLK_FSYS01,
	ENABLE_SCLK_FSYS01,
	ENABLE_SCLK_FSYS02,
	ENABLE_SCLK_FSYS04,
};

static const struct samsung_mux_clock fsys0_mux_clks[] __initconst = {
	MUX(0, "mout_aclk_fsys0_200_user", mout_aclk_fsys0_200_user_p,
		MUX_SEL_FSYS00, 24, 1),

	MUX(0, "mout_sclk_mmc2_user", mout_sclk_mmc2_user_p,
		MUX_SEL_FSYS01, 24, 1),
	MUX(0, "mout_sclk_usbdrd300_user", mout_sclk_usbdrd300_user_p,
		MUX_SEL_FSYS01, 28, 1),

	MUX(0, "mout_phyclk_usbdrd300_udrd30_pipe_pclk_user",
		mout_phyclk_usbdrd300_udrd30_pipe_pclk_user_p,
		MUX_SEL_FSYS02, 24, 1),
	MUX(0, "mout_phyclk_usbdrd300_udrd30_phyclk_user",
		mout_phyclk_usbdrd300_udrd30_phyclk_user_p,
		MUX_SEL_FSYS02, 28, 1),
};

static const struct samsung_gate_clock fsys0_gate_clks[] __initconst = {
	GATE(ACLK_PDMA1, "aclk_pdma1", "mout_aclk_fsys0_200_user",
			ENABLE_ACLK_FSYS00, 3, 0, 0),
	GATE(ACLK_PDMA0, "aclk_pdma0", "mout_aclk_fsys0_200_user",
			ENABLE_ACLK_FSYS00, 4, 0, 0),
	GATE(ACLK_AXIUS_USBDRD30X_FSYS0X, "aclk_axius_usbdrd30x_fsys0x",
		"mout_aclk_fsys0_200_user",
		ENABLE_ACLK_FSYS00, 19, 0, 0),

	GATE(ACLK_USBDRD300, "aclk_usbdrd300", "mout_aclk_fsys0_200_user",
		ENABLE_ACLK_FSYS01, 29, 0, 0),
	GATE(ACLK_MMC2, "aclk_mmc2", "mout_aclk_fsys0_200_user",
		ENABLE_ACLK_FSYS01, 31, 0, 0),

	GATE(SCLK_USBDRD300_SUSPENDCLK, "sclk_usbdrd300_suspendclk",
		"mout_sclk_usbdrd300_user",
		ENABLE_SCLK_FSYS01, 4, 0, 0),
	GATE(SCLK_USBDRD300_REFCLK, "sclk_usbdrd300_refclk", "fin_pll",
		ENABLE_SCLK_FSYS01, 8, 0, 0),

	GATE(PHYCLK_USBDRD300_UDRD30_PIPE_PCLK_USER,
		"phyclk_usbdrd300_udrd30_pipe_pclk_user",
		"mout_phyclk_usbdrd300_udrd30_pipe_pclk_user",
		ENABLE_SCLK_FSYS02, 24, 0, 0),
	GATE(PHYCLK_USBDRD300_UDRD30_PHYCLK_USER,
		"phyclk_usbdrd300_udrd30_phyclk_user",
		"mout_phyclk_usbdrd300_udrd30_phyclk_user",
		ENABLE_SCLK_FSYS02, 28, 0, 0),

	GATE(OSCCLK_PHY_CLKOUT_USB30_PHY, "oscclk_phy_clkout_usb30_phy",
		"fin_pll",
		ENABLE_SCLK_FSYS04, 28, 0, 0),
};

static const struct samsung_cmu_info fsys0_cmu_info __initconst = {
	.fixed_clks		= fixed_rate_clks_fsys0,
	.nr_fixed_clks		= ARRAY_SIZE(fixed_rate_clks_fsys0),
	.mux_clks		= fsys0_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(fsys0_mux_clks),
	.gate_clks		= fsys0_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(fsys0_gate_clks),
	.nr_clk_ids		= FSYS0_NR_CLK,
	.clk_regs		= fsys0_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(fsys0_clk_regs),
};

static void __init exynos7_clk_fsys0_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &fsys0_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_fsys0, "samsung,exynos7-clock-fsys0",
	exynos7_clk_fsys0_init);

/* Register Offset definitions for CMU_FSYS1 (0x156E0000) */
#define MUX_SEL_FSYS10			0x0200
#define MUX_SEL_FSYS11			0x0204
#define MUX_SEL_FSYS12			0x0208
#define DIV_FSYS1			0x0600
#define ENABLE_ACLK_FSYS1		0x0800
#define ENABLE_PCLK_FSYS1               0x0900
#define ENABLE_SCLK_FSYS11              0x0A04
#define ENABLE_SCLK_FSYS12              0x0A08
#define ENABLE_SCLK_FSYS13              0x0A0C

/*
 * List of parent clocks for Muxes in CMU_FSYS1
 */
PNAME(mout_aclk_fsys1_200_user_p)	= { "fin_pll", "aclk_fsys1_200" };
PNAME(mout_fsys1_group_p)	= { "fin_pll", "fin_pll_26m",
				"sclk_phy_fsys1_26m" };
PNAME(mout_pcie_wifi1_tx0_p) = { "fin_pll", "phyclk_pcie_wifi1_tx0" };
PNAME(mout_pcie_wifi1_rx0_p) = { "fin_pll", "phyclk_pcie_wifi1_rx0" };
PNAME(mout_sclk_phy_fsys1_user_p) = { "fin_pll", "sclk_phy_fsys1" };
PNAME(mout_sclk_mmc0_user_p)		= { "fin_pll", "sclk_mmc0" };
PNAME(mout_sclk_mmc1_user_p)		= { "fin_pll", "sclk_mmc1" };
PNAME(mout_sclk_ufsunipro20_user_p)  = { "fin_pll", "sclk_ufsunipro20" };
PNAME(mout_phyclk_ufs20_tx0_user_p) = { "fin_pll", "phyclk_ufs20_tx0_symbol" };
PNAME(mout_phyclk_ufs20_rx0_user_p) = { "fin_pll", "phyclk_ufs20_rx0_symbol" };
PNAME(mout_phyclk_ufs20_rx1_user_p) = { "fin_pll", "phyclk_ufs20_rx1_symbol" };

/* fixed rate clocks used in the FSYS1 block */
static const struct samsung_fixed_rate_clock fixed_rate_clks_fsys1[] __initconst = {
	/* Actual PHY output rate from official7420 vendor clock table. */
	FRATE(PHYCLK_PCIE_WIFI1_TX0, "phyclk_pcie_wifi1_tx0", NULL, 0, 250000000),
	FRATE(PHYCLK_PCIE_WIFI1_RX0, "phyclk_pcie_wifi1_rx0", NULL, 0, 250000000),
	FRATE(PHYCLK_UFS20_TX0_SYMBOL, "phyclk_ufs20_tx0_symbol", NULL,
			0, 300000000),
	FRATE(PHYCLK_UFS20_RX0_SYMBOL, "phyclk_ufs20_rx0_symbol", NULL,
			0, 300000000),
	FRATE(PHYCLK_UFS20_RX1_SYMBOL, "phyclk_ufs20_rx1_symbol", NULL,
			0, 300000000),
};

static const unsigned long fsys1_clk_regs[] __initconst = {
	MUX_SEL_FSYS10,
	MUX_SEL_FSYS11,
	MUX_SEL_FSYS12,
	DIV_FSYS1,
	ENABLE_ACLK_FSYS1,
	ENABLE_PCLK_FSYS1,
	ENABLE_SCLK_FSYS11,
	ENABLE_SCLK_FSYS12,
	ENABLE_SCLK_FSYS13,
};

static const struct samsung_mux_clock fsys1_mux_clks[] __initconst = {
	MUX(MOUT_PHYCLK_PCIE_WIFI1_TX0_USER, "mout_phyclk_pcie_wifi1_tx0_user",
		mout_pcie_wifi1_tx0_p, MUX_SEL_FSYS12, 12, 1),
	MUX(MOUT_PHYCLK_PCIE_WIFI1_RX0_USER, "mout_phyclk_pcie_wifi1_rx0_user",
		mout_pcie_wifi1_rx0_p, MUX_SEL_FSYS12, 8, 1),
	MUX(MOUT_SCLK_PHY_FSYS1_USER, "mout_sclk_phy_fsys1_user",
		mout_sclk_phy_fsys1_user_p, MUX_SEL_FSYS11, 16, 1),
	MUX(MOUT_FSYS1_PHYCLK_SEL1, "mout_fsys1_phyclk_sel1",
		mout_fsys1_group_p, MUX_SEL_FSYS10, 16, 2),
	MUX(MOUT_FSYS1_PHYCLK_SEL0, "mout_fsys1_phyclk_sel0", mout_fsys1_group_p,
		 MUX_SEL_FSYS10, 20, 2),
	MUX(0, "mout_aclk_fsys1_200_user", mout_aclk_fsys1_200_user_p,
		MUX_SEL_FSYS10, 28, 1),

	MUX(0, "mout_sclk_mmc1_user", mout_sclk_mmc1_user_p,
		MUX_SEL_FSYS11, 24, 1),
	MUX(0, "mout_sclk_mmc0_user", mout_sclk_mmc0_user_p,
		MUX_SEL_FSYS11, 28, 1),
	MUX(0, "mout_sclk_ufsunipro20_user", mout_sclk_ufsunipro20_user_p,
		MUX_SEL_FSYS11, 20, 1),

	MUX(0, "mout_phyclk_ufs20_rx1_symbol_user",
		mout_phyclk_ufs20_rx1_user_p, MUX_SEL_FSYS12, 16, 1),
	MUX(0, "mout_phyclk_ufs20_rx0_symbol_user",
		mout_phyclk_ufs20_rx0_user_p, MUX_SEL_FSYS12, 24, 1),
	MUX(0, "mout_phyclk_ufs20_tx0_symbol_user",
		mout_phyclk_ufs20_tx0_user_p, MUX_SEL_FSYS12, 28, 1),
};

static const struct samsung_div_clock fsys1_div_clks[] __initconst = {
	DIV(DOUT_PCLK_COMBO_PHY_WIFI1, "dout_pclk_combo_phy_wifi1",
		"mout_aclk_fsys1_200_user", DIV_FSYS1, 8, 2),
	DIV(DOUT_PCLK_FSYS1, "dout_pclk_fsys1", "mout_aclk_fsys1_200_user",
		DIV_FSYS1, 0, 2),
};

static const struct samsung_gate_clock fsys1_gate_clks[] __initconst = {
	GATE(ACLK_XIU_WIFI1X, "aclk_xiu_wifi1x", "mout_aclk_fsys1_200_user",
		ENABLE_ACLK_FSYS1, 24, 0, 0),
	GATE(ACLK_AHB2AXI_PCIE_WIFI1, "aclk_ahb2axi_pcie_wifi1", "mout_aclk_fsys1_200_user",
		ENABLE_ACLK_FSYS1, 13, 0, 0),
	GATE(ACLK_PCIE_WIFI1_MSTR, "aclk_pcie_wifi1_mstr_aclk", "mout_aclk_fsys1_200_user",
		ENABLE_ACLK_FSYS1, 12, 0, 0),
	GATE(ACLK_PCIE_WIFI1_SLV, "aclk_pcie_wifi1_slv_aclk", "mout_aclk_fsys1_200_user",
		ENABLE_ACLK_FSYS1, 11, 0, 0),
	GATE(ACLK_PCIE_WIFI1_DBI, "aclk_pcie_wifi1_dbi_aclk", "mout_aclk_fsys1_200_user",
		ENABLE_ACLK_FSYS1, 10, 0, 0),
	GATE(ACLK_COMBO_PHY_PCS_WIFI1, "aclk_combo_phy_pcs_pclk_wifi1", "mout_aclk_fsys1_200_user",
		ENABLE_ACLK_FSYS1, 9, 0, 0),
	GATE(PCLK_ASYNC_COMBO_PHY_WIFI1, "pclk_async_combo_phy_wifi1", "dout_pclk_combo_phy_wifi1",
		ENABLE_PCLK_FSYS1, 22, 0, 0),
	GATE(SCLK_PCIE_LINK_WIFI1, "sclk_pcie_link_wifi1_gated", "fin_pll",
		ENABLE_SCLK_FSYS11, 0, 0, 0),
	GATE(SCLK_COMBO_PHY_WIFI1_26M, "sclk_combo_phy_wifi1_26m_gated", "mout_fsys1_phyclk_sel0",
		ENABLE_SCLK_FSYS13, 28, 0, 0),
	GATE(SCLK_PHY_FSYS1_USER, "sclk_phy_fsys1_gated", "mout_sclk_phy_fsys1_user",
		ENABLE_SCLK_FSYS11, 16, 0, 0),
	GATE(PHYCLK_PCIE_WIFI1_TX0_USER, "phyclk_pcie_wifi1_tx0_gated", "mout_phyclk_pcie_wifi1_tx0_user",
		ENABLE_SCLK_FSYS12, 12, 0, 0),
	GATE(PHYCLK_PCIE_WIFI1_RX0_USER, "phyclk_pcie_wifi1_rx0_gated", "mout_phyclk_pcie_wifi1_rx0_user",
		ENABLE_SCLK_FSYS12, 8, 0, 0),

	GATE(SCLK_UFSUNIPRO20_USER, "sclk_ufsunipro20_user",
		"mout_sclk_ufsunipro20_user",
		ENABLE_SCLK_FSYS11, 20, 0, 0),

	GATE(ACLK_MMC1, "aclk_mmc1", "mout_aclk_fsys1_200_user",
		ENABLE_ACLK_FSYS1, 29, 0, 0),
	GATE(ACLK_MMC0, "aclk_mmc0", "mout_aclk_fsys1_200_user",
		ENABLE_ACLK_FSYS1, 30, 0, 0),

	GATE(ACLK_UFS20_LINK, "aclk_ufs20_link", "dout_pclk_fsys1",
		ENABLE_ACLK_FSYS1, 31, 0, 0),
	GATE(PCLK_GPIO_FSYS1, "pclk_gpio_fsys1", "mout_aclk_fsys1_200_user",
		ENABLE_PCLK_FSYS1, 30, 0, 0),

	GATE(PHYCLK_UFS20_RX1_SYMBOL_USER, "phyclk_ufs20_rx1_symbol_user",
		"mout_phyclk_ufs20_rx1_symbol_user",
		ENABLE_SCLK_FSYS12, 16, 0, 0),
	GATE(PHYCLK_UFS20_RX0_SYMBOL_USER, "phyclk_ufs20_rx0_symbol_user",
		"mout_phyclk_ufs20_rx0_symbol_user",
		ENABLE_SCLK_FSYS12, 24, 0, 0),
	GATE(PHYCLK_UFS20_TX0_SYMBOL_USER, "phyclk_ufs20_tx0_symbol_user",
		"mout_phyclk_ufs20_tx0_symbol_user",
		ENABLE_SCLK_FSYS12, 28, 0, 0),

	GATE(OSCCLK_PHY_CLKOUT_EMBEDDED_COMBO_PHY,
		"oscclk_phy_clkout_embedded_combo_phy",
		"fin_pll",
		ENABLE_SCLK_FSYS12, 4, CLK_IGNORE_UNUSED, 0),

	GATE(SCLK_COMBO_PHY_EMBEDDED_26M, "sclk_combo_phy_embedded_26m",
		"mout_fsys1_phyclk_sel1",
		ENABLE_SCLK_FSYS13, 24, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info fsys1_cmu_info __initconst = {
	.fixed_clks		= fixed_rate_clks_fsys1,
	.nr_fixed_clks		= ARRAY_SIZE(fixed_rate_clks_fsys1),
	.mux_clks		= fsys1_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(fsys1_mux_clks),
	.div_clks		= fsys1_div_clks,
	.nr_div_clks		= ARRAY_SIZE(fsys1_div_clks),
	.gate_clks		= fsys1_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(fsys1_gate_clks),
	.nr_clk_ids		= FSYS1_NR_CLK,
	.clk_regs		= fsys1_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(fsys1_clk_regs),
};

static void __init exynos7_clk_fsys1_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &fsys1_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_fsys1, "samsung,exynos7-clock-fsys1",
	exynos7_clk_fsys1_init);

#define MUX_SEL_MSCL			0x0200
#define DIV_MSCL			0x0600
#define ENABLE_ACLK_MSCL		0x0800
#define ENABLE_PCLK_MSCL		0x0900

/* List of parent clocks for Muxes in CMU_MSCL */
PNAME(mout_aclk_mscl_532_user_p)	= { "fin_pll", "aclk_mscl_532" };

static const unsigned long mscl_clk_regs[] __initconst = {
	MUX_SEL_MSCL,
	DIV_MSCL,
	ENABLE_ACLK_MSCL,
	ENABLE_PCLK_MSCL,
};

static const struct samsung_mux_clock mscl_mux_clks[] __initconst = {
	MUX(USERMUX_ACLK_MSCL_532, "usermux_aclk_mscl_532",
		mout_aclk_mscl_532_user_p, MUX_SEL_MSCL, 0, 1),
};
static const struct samsung_div_clock mscl_div_clks[] __initconst = {
	DIV(DOUT_PCLK_MSCL, "dout_pclk_mscl", "usermux_aclk_mscl_532",
			DIV_MSCL, 0, 3),
};
static const struct samsung_gate_clock mscl_gate_clks[] __initconst = {

	GATE(ACLK_MSCL_0, "aclk_mscl_0", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 31, 0, 0),
	GATE(ACLK_MSCL_1, "aclk_mscl_1", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 30, 0, 0),
	GATE(ACLK_JPEG, "aclk_jpeg", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 29, 0, 0),
	GATE(ACLK_G2D, "aclk_g2d", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 28, 0, 0),
	GATE(ACLK_LH_ASYNC_SI_MSCL_0, "aclk_lh_async_si_mscl_0",
			"usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 27, 0, 0),
	GATE(ACLK_LH_ASYNC_SI_MSCL_1, "aclk_lh_async_si_mscl_1",
			"usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 26, 0, 0),
	GATE(ACLK_XIU_MSCLX_0, "aclk_xiu_msclx_0", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 25, 0, 0),
	GATE(ACLK_XIU_MSCLX_1, "aclk_xiu_msclx_1", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 24, 0, 0),
	GATE(ACLK_AXI2ACEL_BRIDGE, "aclk_axi2acel_bridge",
			"usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 23, 0, 0),
	GATE(ACLK_QE_MSCL_0, "aclk_qe_mscl_0", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 22, 0, 0),
	GATE(ACLK_QE_MSCL_1, "aclk_qe_mscl_1", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 21, 0, 0),
	GATE(ACLK_QE_JPEG, "aclk_qe_jpeg", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 20, 0, 0),
	GATE(ACLK_QE_G2D, "aclk_qe_g2d", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 19, 0, 0),
	GATE(ACLK_PPMU_MSCL_0, "aclk_ppmu_mscl_0", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 18, 0, 0),
	GATE(ACLK_PPMU_MSCL_1, "aclk_ppmu_mscl_1", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 17, 0, 0),
	GATE(ACLK_MSCLNP_133, "aclk_msclnp_133", "usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 16, 0, 0),
	GATE(ACLK_AHB2APB_MSCL0P, "aclk_ahb2apb_mscl0p",
			"usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 15, 0, 0),
	GATE(ACLK_AHB2APB_MSCL1P, "aclk_ahb2apb_mscl1p",
			"usermux_aclk_mscl_532",
			ENABLE_ACLK_MSCL, 14, 0, 0),

	GATE(PCLK_MSCL_0, "pclk_mscl_0", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 31, 0, 0),
	GATE(PCLK_MSCL_1, "pclk_mscl_1", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 30, 0, 0),
	GATE(PCLK_JPEG, "pclk_jpeg", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 29, 0, 0),
	GATE(PCLK_G2D, "pclk_g2d", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 28, 0, 0),
	GATE(PCLK_QE_MSCL_0, "pclk_qe_mscl_0", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 27, 0, 0),
	GATE(PCLK_QE_MSCL_1, "pclk_qe_mscl_1", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 26, 0, 0),
	GATE(PCLK_QE_JPEG, "pclk_qe_jpeg", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 25, 0, 0),
	GATE(PCLK_QE_G2D, "pclk_qe_g2d", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 24, 0, 0),
	GATE(PCLK_PPMU_MSCL_0, "pclk_ppmu_mscl_0", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 23, 0, 0),
	GATE(PCLK_PPMU_MSCL_1, "pclk_ppmu_mscl_1", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 22, 0, 0),
	GATE(PCLK_AXI2ACEL_BRIDGE, "pclk_axi2acel_bridge", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 21, 0, 0),
	GATE(PCLK_PMU_MSCL, "pclk_pmu_mscl", "dout_pclk_mscl",
			ENABLE_PCLK_MSCL, 20, 0, 0),
};

static const struct samsung_cmu_info mscl_cmu_info __initconst = {
	.mux_clks		= mscl_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(mscl_mux_clks),
	.div_clks		= mscl_div_clks,
	.nr_div_clks		= ARRAY_SIZE(mscl_div_clks),
	.gate_clks		= mscl_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(mscl_gate_clks),
	.nr_clk_ids		= MSCL_NR_CLK,
	.clk_regs		= mscl_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(mscl_clk_regs),
};

static void __init exynos7_clk_mscl_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &mscl_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_mscl, "samsung,exynos7-clock-mscl",
		exynos7_clk_mscl_init);

/* Register Offset definitions for CMU_MFC (0x15280000) */
#define MUX_SEL_MFC		0x0200
#define DIV_MFC			0x0600
#define ENABLE_ACLK_MFC		0x0800
#define ENABLE_ACLK_MFC_SMMU	0x0804
#define ENABLE_PCLK_MFC		0x0900
#define ENABLE_PCLK_MFC_SMMU	0x0904

static const unsigned long mfc_clk_regs[] = {
	MUX_SEL_MFC,
	DIV_MFC,
	ENABLE_ACLK_MFC,
	ENABLE_ACLK_MFC_SMMU,
	ENABLE_PCLK_MFC,
	ENABLE_PCLK_MFC_SMMU,
};

static const char * const mout_aclk_mfc_532_user_p[] = { "fin_pll", "aclk_mfc_532" };

static const struct samsung_mux_clock mfc_mux_clks[] = {
	MUX(MOUT_ACLK_MFC_532_USER, "mout_aclk_mfc_532_user", mout_aclk_mfc_532_user_p,
		MUX_SEL_MFC, 0, 1),
};

static const struct samsung_div_clock mfc_div_clks[] = {
	DIV(DOUT_PCLK_MFC, "dout_pclk_mfc", "mout_aclk_mfc_532_user",
		DIV_MFC, 0, 2),
};

static const struct samsung_gate_clock mfc_gate_clks[] = {
	GATE(CLK_ACLK_MFC, "aclk_mfc", "mout_aclk_mfc_532_user",
		ENABLE_ACLK_MFC, 31, 0, 0),
	GATE(CLK_ACLK_LH_MFC_0, "aclk_lh_mfc_0", "mout_aclk_mfc_532_user",
		ENABLE_ACLK_MFC, 30, 0, 0),
	GATE(CLK_ACLK_LH_MFC_1, "aclk_lh_mfc_1", "mout_aclk_mfc_532_user",
		ENABLE_ACLK_MFC, 19, 0, 0),
	GATE(CLK_PCLK_MFC, "pclk_mfc", "dout_pclk_mfc",
		ENABLE_PCLK_MFC, 31, 0, 0),
	GATE(CLK_ACLK_SMMU_MFC_0, "aclk_smmu_mfc_0", "mout_aclk_mfc_532_user",
		ENABLE_ACLK_MFC_SMMU, 0, 0, 0),
	GATE(CLK_ACLK_SMMU_MFC_1, "aclk_smmu_mfc_1", "mout_aclk_mfc_532_user",
		ENABLE_ACLK_MFC_SMMU, 1, 0, 0),
	GATE(CLK_PCLK_SMMU_MFC_0, "pclk_smmu_mfc_0", "dout_pclk_mfc",
		ENABLE_PCLK_MFC_SMMU, 0, 0, 0),
	GATE(CLK_PCLK_SMMU_MFC_1, "pclk_smmu_mfc_1", "dout_pclk_mfc",
		ENABLE_PCLK_MFC_SMMU, 1, 0, 0),
};

static const struct samsung_cmu_info mfc_cmu_info = {
	.mux_clks		= mfc_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(mfc_mux_clks),
	.div_clks		= mfc_div_clks,
	.nr_div_clks		= ARRAY_SIZE(mfc_div_clks),
	.gate_clks		= mfc_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(mfc_gate_clks),
	.nr_clk_ids		= MFC_NR_CLK,
	.clk_regs		= mfc_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(mfc_clk_regs),
	.clk_name		= "aclk_mfc_532",
};

static int __init exynos7_cmu_probe(struct platform_device *pdev)
{
	return exynos_arm64_register_cmu_pm(pdev, false);
}

static const struct of_device_id exynos7_cmu_of_match[] = {
	{
		.compatible = "samsung,exynos7-clock-mfc",
		.data = &mfc_cmu_info,
	}, {
	},
};

static const struct dev_pm_ops exynos7_cmu_pm_ops = {
	SET_RUNTIME_PM_OPS(exynos_arm64_cmu_suspend, exynos_arm64_cmu_resume,
			   NULL)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				      pm_runtime_force_resume)
};

static struct platform_driver exynos7_cmu_driver __refdata = {
	.driver = {
		.name = "exynos7-cmu",
		.of_match_table = exynos7_cmu_of_match,
		.suppress_bind_attrs = true,
		.pm = &exynos7_cmu_pm_ops,
	},
	.probe = exynos7_cmu_probe,
};

static int __init exynos7_cmu_init(void)
{
	return platform_driver_register(&exynos7_cmu_driver);
}
core_initcall(exynos7_cmu_init);

/* Register Offset definitions for CMU_AUD (0x114C0000) */
#define	MUX_SEL_AUD			0x0200
#define	DIV_AUD0			0x0600
#define	DIV_AUD1			0x0604
#define	ENABLE_ACLK_AUD			0x0800
#define	ENABLE_PCLK_AUD			0x0900
#define	ENABLE_SCLK_AUD			0x0A00

/*
 * List of parent clocks for Muxes in CMU_AUD
 */
PNAME(mout_aud_pll_user_p) = { "fin_pll", "fout_aud_pll" };
PNAME(mout_aud_group_p) = { "dout_aud_cdclk", "ioclk_audiocdclk0" };

static const unsigned long aud_clk_regs[] __initconst = {
	MUX_SEL_AUD,
	DIV_AUD0,
	DIV_AUD1,
	ENABLE_ACLK_AUD,
	ENABLE_PCLK_AUD,
	ENABLE_SCLK_AUD,
};

static const struct samsung_mux_clock aud_mux_clks[] __initconst = {
	MUX(0, "mout_sclk_i2s", mout_aud_group_p, MUX_SEL_AUD, 12, 1),
	MUX(0, "mout_sclk_pcm", mout_aud_group_p, MUX_SEL_AUD, 16, 1),
	MUX(0, "mout_aud_pll_user", mout_aud_pll_user_p, MUX_SEL_AUD, 20, 1),
};

static const struct samsung_div_clock aud_div_clks[] __initconst = {
	DIV(0, "dout_aud_ca5", "mout_aud_pll_user", DIV_AUD0, 0, 4),
	DIV(0, "dout_aclk_aud", "dout_aud_ca5", DIV_AUD0, 4, 4),
	DIV(0, "dout_aud_pclk_dbg", "dout_aud_ca5", DIV_AUD0, 8, 4),

	DIV(0, "dout_sclk_i2s", "mout_sclk_i2s", DIV_AUD1, 0, 4),
	DIV(0, "dout_sclk_pcm", "mout_sclk_pcm", DIV_AUD1, 4, 8),
	DIV_F(0, "dout_sclk_uart", "dout_aud_cdclk",
			DIV_AUD1, 12, 4, CLK_SET_RATE_PARENT, 0),
	DIV(0, "dout_sclk_slimbus", "dout_aud_cdclk", DIV_AUD1, 16, 5),
	DIV(0, "dout_aud_cdclk", "mout_aud_pll_user", DIV_AUD1, 24, 4),
};

static const struct samsung_gate_clock aud_gate_clks[] __initconst = {
	GATE(SCLK_PCM, "sclk_pcm", "dout_sclk_pcm",
			ENABLE_SCLK_AUD, 27, CLK_SET_RATE_PARENT, 0),
	GATE(SCLK_I2S, "sclk_i2s", "dout_sclk_i2s",
			ENABLE_SCLK_AUD, 28,
			CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED, 0),
	GATE(SCLK_UART, "sclk_uart", "dout_sclk_uart",
			ENABLE_SCLK_AUD, 29, CLK_SET_RATE_PARENT, 0),
	GATE(0, "sclk_slimbus", "dout_sclk_slimbus",
			ENABLE_SCLK_AUD, 30, 0, 0),

	GATE(0, "pclk_dbg_aud", "dout_aud_pclk_dbg",
			ENABLE_PCLK_AUD, 19, CLK_IGNORE_UNUSED, 0),
	GATE(0, "pclk_gpio_aud", "dout_aclk_aud",
			ENABLE_PCLK_AUD, 20, CLK_IGNORE_UNUSED, 0),
	GATE(0, "pclk_wdt1", "dout_aclk_aud", ENABLE_PCLK_AUD, 22, 0, 0),
	GATE(0, "pclk_wdt0", "dout_aclk_aud", ENABLE_PCLK_AUD, 23, 0, 0),
	GATE(0, "pclk_slimbus", "dout_aclk_aud", ENABLE_PCLK_AUD, 24, 0, 0),
	GATE(PCLK_UART, "pclk_uart", "dout_aclk_aud",
			ENABLE_PCLK_AUD, 25, 0, 0),
	GATE(PCLK_PCM, "pclk_pcm", "dout_aclk_aud",
			ENABLE_PCLK_AUD, 26, CLK_SET_RATE_PARENT, 0),
	GATE(PCLK_I2S, "pclk_i2s", "dout_aclk_aud",
			ENABLE_PCLK_AUD, 27,
			CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED, 0),
	GATE(0, "pclk_timer", "dout_aclk_aud",
			ENABLE_PCLK_AUD, 28, CLK_IGNORE_UNUSED, 0),
	GATE(0, "pclk_smmu_aud", "dout_aclk_aud",
			ENABLE_PCLK_AUD, 31, CLK_IGNORE_UNUSED, 0),

	GATE(0, "aclk_axi_lh_async_si_top", "dout_aclk_aud",
			ENABLE_ACLK_AUD, 23, CLK_IGNORE_UNUSED, 0),
	GATE(0, "aclk_smmu_aud", "dout_aclk_aud",
			ENABLE_ACLK_AUD, 27, CLK_IGNORE_UNUSED, 0),
	GATE(0, "aclk_acel_lh_async_si_top", "dout_aclk_aud",
			ENABLE_ACLK_AUD, 28, CLK_IGNORE_UNUSED, 0),
	GATE(ACLK_ADMA, "aclk_dmac", "dout_aclk_aud",
			ENABLE_ACLK_AUD, 31, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info aud_cmu_info __initconst = {
	.mux_clks		= aud_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(aud_mux_clks),
	.div_clks		= aud_div_clks,
	.nr_div_clks		= ARRAY_SIZE(aud_div_clks),
	.gate_clks		= aud_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(aud_gate_clks),
	.nr_clk_ids		= AUD_NR_CLK,
	.clk_regs		= aud_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(aud_clk_regs),
};

static void __init exynos7_clk_aud_init(struct device_node *np)
{
	void __iomem *reg_base = of_iomap(np, 0);

	if (reg_base) {
		u32 val;

		/*
		 * Reparent mout_aud_pll_user (BIT 20) to fin_pll (24 MHz) and
		 * zero DIV_AUD0/DIV_AUD1 so BLK_AUD (serial_4, pinctrl_aud,
		 * i2s0, adma) runs off the 24 MHz oscillator without aud_pll.
		 */
		val = readl_relaxed(reg_base + MUX_SEL_AUD);
		val &= ~(BIT(20) | BIT(16) | BIT(12));
		writel_relaxed(val, reg_base + MUX_SEL_AUD);
		writel_relaxed(0x0, reg_base + DIV_AUD0);
		writel_relaxed(0x0, reg_base + DIV_AUD1);

		val = readl_relaxed(reg_base + ENABLE_ACLK_AUD);
		val |= BIT(31) | BIT(28) | BIT(27) | BIT(23);
		writel_relaxed(val, reg_base + ENABLE_ACLK_AUD);

		val = readl_relaxed(reg_base + ENABLE_PCLK_AUD);
		val |= BIT(31) | BIT(28) | BIT(27) | BIT(25) | BIT(20);
		writel_relaxed(val, reg_base + ENABLE_PCLK_AUD);

		val = readl_relaxed(reg_base + ENABLE_SCLK_AUD);
		val |= BIT(29) | BIT(28);
		writel_relaxed(val, reg_base + ENABLE_SCLK_AUD);

		iounmap(reg_base);
	}

	samsung_cmu_register_one(np, &aud_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_aud, "samsung,exynos7-clock-aud",
		exynos7_clk_aud_init);

/* Register Offset definitions for CMU_G3D (0x14AA0000) */
#define G3D_PLL_LOCK			0x0000
#define G3D_PLL_CON0			0x0100
#define MUX_SEL_G3D			0x0200
#define MUX_ENABLE_G3D			0x0300
#define MUX_STAT_G3D			0x0400
#define DIV_G3D				0x0600
#define DIV_STAT_G3D			0x0700
#define ENABLE_ACLK_G3D			0x0800
#define ENABLE_PCLK_G3D			0x0900
#define ENABLE_SCLK_G3D			0x0a00
#define ENABLE_IP_G3D0			0x0b00

/* Read inherited PLL settings; no enable, set_rate or voltage operations. */
static struct clk_hw g3d_pll_ro_hw;
static void __iomem *g3d_pll_ro_base;

static unsigned long g3d_pll_ro_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	u32 con = readl(g3d_pll_ro_base + G3D_PLL_CON0);
	u32 m = (con >> 16) & 0x3ff;
	u32 p = (con >> 8) & 0x3f;
	u32 s = con & 7;

	if (!p || !(con & BIT(31)) || !(con & BIT(29)))
		return 0;
	return div_u64((u64)parent_rate * m, p << s);
}

static const struct clk_ops g3d_pll_ro_ops = {
	.recalc_rate = g3d_pll_ro_recalc_rate,
};

static const struct clk_init_data g3d_pll_ro_init = {
	.name = "fout_g3d_pll",
	.ops = &g3d_pll_ro_ops,
	.parent_names = (const char * const[]) { "fin_pll" },
	.num_parents = 1,
	.flags = CLK_GET_RATE_NOCACHE,
};

PNAME(mout_g3d_pll_p) = { "fin_pll", "fout_g3d_pll" };
PNAME(mout_aclk_g3d_400_p) = { "mout_g3d_pll", "fin_pll" };

static const struct samsung_mux_clock g3d_mux_clks[] __initconst = {
	/* Vendor PLL bypass mux bit 0; preserve the inherited selection. */
	MUX_F(0, "mout_g3d_pll", mout_g3d_pll_p, MUX_SEL_G3D, 0, 1,
		CLK_GET_RATE_NOCACHE, CLK_MUX_READ_ONLY),
	/* Vendor mout_g3d is bit 8, not the currently unexplained bit 4. */
	MUX_F(CLK_MOUT_ACLK_G3D_400, "mout_aclk_g3d_400", mout_aclk_g3d_400_p,
		MUX_SEL_G3D, 8, 1, CLK_GET_RATE_NOCACHE, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock g3d_div_clks[] __initconst = {
	DIV(CLK_DIV_ACLK_G3D, "div_aclk_g3d", "mout_aclk_g3d_400",
			DIV_G3D, 0, 3),
	DIV(CLK_DIV_PCLK_G3D, "div_pclk_g3d", "div_aclk_g3d",
			DIV_G3D, 4, 3),
};

static const struct samsung_gate_clock g3d_gate_clks[] __initconst = {
	GATE(CLK_ACLK_G3D, "aclk_g3d", "div_aclk_g3d",
			ENABLE_ACLK_G3D, 0, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED, 0),
	GATE(CLK_PCLK_SYSREG_G3D, "pclk_sysreg_g3d", "div_pclk_g3d",
			ENABLE_PCLK_G3D, 0, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_PCLK_PMU_G3D, "pclk_pmu_g3d", "div_pclk_g3d",
			ENABLE_PCLK_G3D, 1, CLK_IGNORE_UNUSED, 0),
};

static const unsigned long g3d_clk_regs[] __initconst = {
	MUX_SEL_G3D,
	DIV_G3D,
	ENABLE_ACLK_G3D,
	ENABLE_PCLK_G3D,
	ENABLE_SCLK_G3D,
};

static const struct samsung_cmu_info g3d_cmu_info __initconst = {
	.mux_clks		= g3d_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(g3d_mux_clks),
	.div_clks		= g3d_div_clks,
	.nr_div_clks		= ARRAY_SIZE(g3d_div_clks),
	.gate_clks		= g3d_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(g3d_gate_clks),
	.nr_clk_ids		= G3D_NR_CLK,
	.clk_regs		= g3d_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(g3d_clk_regs),
};

/* Domain-only LPI gate has no normal consumer/CCF reference. Keep its
 * one-bit update under the actual CMU lock shared by ordinary CCF gates.
 * Full saved-context replay restores the original value on domain-on. */
static struct samsung_clk_provider *m86_g3d_owner;
int m86_exynos7_g3d_lpi_clock(struct regmap *pmu, struct mutex *transaction,
                            struct mutex *hardware, bool *was_enabled)
{
 unsigned long flags;
 unsigned int cfg, status;
 u32 v;
 int ret;
 if (!m86_g3d_owner || !pmu || !transaction || !hardware || !was_enabled)
  return -EINVAL;
 lockdep_assert_held(transaction);
 lockdep_assert_held(hardware);
 ret = regmap_read(pmu, 0x4100, &cfg);
 if (!ret)
  ret = regmap_read(pmu, 0x4104, &status);
 if (ret)
  return ret;
 if ((cfg & 15) != 15 || (status & 15) != 15)
  return -EHOSTDOWN;
 spin_lock_irqsave(&m86_g3d_owner->lock, flags);
 v = readl(m86_g3d_owner->reg_base + ENABLE_ACLK_G3D);
 *was_enabled = !!(v & BIT(1));
 writel(v | BIT(1), m86_g3d_owner->reg_base + ENABLE_ACLK_G3D);
 ret = readl(m86_g3d_owner->reg_base + ENABLE_ACLK_G3D) == (v | BIT(1)) ? 0 : -EIO;
 spin_unlock_irqrestore(&m86_g3d_owner->lock, flags);
 return ret;
}
EXPORT_SYMBOL_GPL(m86_exynos7_g3d_lpi_clock);

static void __init exynos7_clk_g3d_init(struct device_node *np)
{
	struct samsung_clk_provider *ctx;
	int ret;

	ctx = samsung_cmu_register_one(np, &g3d_cmu_info);
	if (!ctx)
		return;
	m86_g3d_owner = ctx;
	g3d_pll_ro_base = ctx->reg_base;
	g3d_pll_ro_hw.init = &g3d_pll_ro_init;
	ret = clk_hw_register(NULL, &g3d_pll_ro_hw);
	if (ret) {
		pr_err("G3D: read-only PLL registration failed: %d\n", ret);
		return;
	}
	samsung_clk_add_lookup(ctx, &g3d_pll_ro_hw, CLK_FOUT_G3D_PLL);
}

CLK_OF_DECLARE(exynos7_clk_g3d, "samsung,exynos7-clock-g3d",
		exynos7_clk_g3d_init);

/* Register Offset definitions for CMU_DISP (0x13AD0000) */
#define DISP_PLL_LOCK			0x0000
#define DISP_PLL_CON0			0x0100
#define MUX_SEL_DISP0			0x0200
#define MUX_SEL_DISP1			0x0204
#define MUX_SEL_DISP2			0x0208
#define MUX_SEL_DISP3			0x020C
#define DIV_DISP			0x0600
#define ENABLE_ACLK_DISP		0x0800
#define ENABLE_ACLK_DISP_RO_SMMU	0x0804
#define ENABLE_ACLK_DISP_RW_SMMU	0x0808
#define ENABLE_PCLK_DISP		0x0900
#define ENABLE_PCLK_DISP_RO_SMMU	0x0904
#define ENABLE_PCLK_DISP_RW_SMMU	0x0908
#define ENABLE_SCLK_DISP1		0x0A04
#define ENABLE_SCLK_DISP2		0x0A08

static const struct samsung_pll_rate_table disp_pll_tbl[] __initconst = {
	PLL_36XX_RATE(24 * MHZ, 252000000, 42, 2, 1, 0),
	PLL_36XX_RATE(24 * MHZ, 240000000, 20, 1, 1, 0),
	PLL_36XX_RATE(24 * MHZ, 142000000, 71, 3, 2, 0),
	PLL_36XX_RATE(24 * MHZ, 126000000, 21, 1, 2, 0),
	PLL_36XX_RATE(24 * MHZ, 96000000, 32, 1, 3, 0),
	{},
};

PNAME(mout_sub_sclk_decon_int_eclk_p) = { "usermux_sclk_decon_int_eclk",
	"disp_pll" };
PNAME(mout_sub_sclk_decon_int_vclk_p) = { "usermux_sclk_decon_vclk",
	"disp_pll" };

/* External clocks are descriptive vendor rates; gates control the inputs. */
static const struct samsung_fixed_rate_clock disp_fixed_clks[] __initconst = {
	FRATE(CLK_FOUT_MIPIDPHY0_BIT, "phyclk_mipidphy0_bit", NULL, 0, 187500000),
	FRATE(CLK_FOUT_MIPIDPHY0_RX, "phyclk_mipidphy0_rx", NULL, 0, 20000000),
	FRATE(0, "sclk_rgb_vclk0", NULL, 0, 300000000),
};
PNAME(mout_mipidphy0_bit_p) = { "fin_pll", "phyclk_mipidphy0_bit" };
PNAME(mout_mipidphy0_rx_p) = { "fin_pll", "phyclk_mipidphy0_rx" };

static const struct samsung_pll_clock disp_pll_clks[] __initconst = {
	PLL(pll_1460x, CLK_FOUT_DISP_PLL, "disp_pll", "fin_pll",
		DISP_PLL_LOCK, DISP_PLL_CON0, disp_pll_tbl),
};

PNAME(mout_aclk_disp_400_user_p) = { "fin_pll", "top_aclk_disp_400" };
PNAME(mout_decon_int_eclk_user_p) = { "fin_pll", "top_sclk_decon_int_eclk" };
PNAME(mout_decon_vclk_user_p) = { "fin_pll", "top_sclk_decon_vclk" };

PNAME(mout_sclk_dsd_user_p) = { "fin_pll", "top_sclk_dsd" };

static const struct samsung_mux_clock disp_mux_clks[] __initconst = {
	MUX(CLK_MOUT_SCLK_DSD_USER, "usermux_sclk_dsd", mout_sclk_dsd_user_p,
		MUX_SEL_DISP1, 16, 1),
	/* 7420 DISP ingress selectors; select TOP explicitly via assigned parents. */
	MUX(CLK_MOUT_ACLK_DISP_400_USER, "usermux_aclk_disp_400",
		mout_aclk_disp_400_user_p, MUX_SEL_DISP0, 24, 1),
	MUX(CLK_MOUT_DECON_INT_ECLK_USER, "usermux_sclk_decon_int_eclk",
		mout_decon_int_eclk_user_p, MUX_SEL_DISP1, 28, 1),
	MUX(CLK_MOUT_DECON_VCLK_USER, "usermux_sclk_decon_vclk",
		mout_decon_vclk_user_p, MUX_SEL_DISP1, 20, 1),
	MUX(CLK_MOUT_MIPIDPHY0_BIT, "usermux_mipidphy0_bit",
		mout_mipidphy0_bit_p, MUX_SEL_DISP2, 8, 1),
	MUX(CLK_MOUT_MIPIDPHY0_RX, "usermux_mipidphy0_rx",
		mout_mipidphy0_rx_p, MUX_SEL_DISP2, 12, 1),
	MUX(CLK_MOUT_SUB_SCLK_DECON_INT_ECLK, "mout_sub_sclk_decon_int_eclk",
		mout_sub_sclk_decon_int_eclk_p, MUX_SEL_DISP3, 20, 1),
	MUX(CLK_MOUT_SUB_SCLK_DECON_INT_VCLK, "mout_sub_sclk_decon_int_vclk",
		mout_sub_sclk_decon_int_vclk_p, MUX_SEL_DISP3, 12, 1),
};

static const struct samsung_div_clock disp_div_clks[] __initconst = {
	DIV(CLK_DOUT_PCLK_DISP, "dout_pclk_disp", "usermux_aclk_disp_400",
		DIV_DISP, 0, 2),
	DIV(CLK_DOUT_SUB_SCLK_DECON_INT_ECLK,
		"dout_sub_sclk_decon_int_eclk",
		"mout_sub_sclk_decon_int_eclk", DIV_DISP, 20, 3),
	DIV(CLK_DOUT_SUB_SCLK_DECON_INT_VCLK,
		"dout_sub_sclk_decon_int_vclk",
		"mout_sub_sclk_decon_int_vclk", DIV_DISP, 12, 3),
};

static const struct samsung_gate_clock disp_gate_clks[] __initconst = {
	GATE(CLK_SCLK_DSD, "user_sclk_dsd", "usermux_sclk_dsd",
		ENABLE_SCLK_DISP1, 16, 0, 0),
	GATE(0, "aclk_lh_async_si_top_disp", "usermux_aclk_disp_400",
		ENABLE_ACLK_DISP, 28, 0, 0),
	GATE(CLK_ACLK_XIU_DISP_RW, "aclk_xiu_disp_rw", "aclk_lh_async_si_top_disp",
		ENABLE_ACLK_DISP, 26, 0, 0),
	GATE(CLK_PHYCLK_MIPIDPHY0_BITCLKDIV8, "mipidphy0_bit",
		"usermux_mipidphy0_bit", ENABLE_SCLK_DISP2, 8, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_PHYCLK_MIPIDPHY0_RXCLKESC0, "mipidphy0_rx",
		"usermux_mipidphy0_rx", ENABLE_SCLK_DISP2, 12, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_SCLK_RGB_VCLK_TO_DSIM0, "rgb_vclk0",
		"sclk_rgb_vclk0", ENABLE_SCLK_DISP2, 28, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_ACLK_DECON0, "aclk_decon0", "aclk_xiu_disp_ro",
		ENABLE_ACLK_DISP, 31, CLK_IGNORE_UNUSED, 0),
	GATE(0, "aclk_xiu_disp_ro", "aclk_lh_async_si_r_top_disp",
		ENABLE_ACLK_DISP, 27, CLK_IGNORE_UNUSED, 0),
	GATE(0, "aclk_lh_async_si_r_top_disp", "usermux_aclk_disp_400",
		ENABLE_ACLK_DISP, 29, CLK_IGNORE_UNUSED, 0),

	GATE(CLK_PCLK_DECON0, "pclk_decon0", "dout_pclk_disp",
		ENABLE_PCLK_DISP, 31, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_PCLK_DSIM0, "pclk_dsim0", "dout_pclk_disp",
		ENABLE_PCLK_DISP, 29, CLK_IGNORE_UNUSED, 0),

	GATE(CLK_ACLK_DISP_RO_SYSMMU, "aclk_smmu_disp_ro",
		"usermux_aclk_disp_400", ENABLE_ACLK_DISP_RO_SMMU, 0,
		0, 0),
	GATE(CLK_ACLK_DISP_RW_SYSMMU, "aclk_smmu_disp_rw",
		"usermux_aclk_disp_400", ENABLE_ACLK_DISP_RW_SMMU, 0,
		0, 0),
	GATE(CLK_PCLK_DISP_RO_SYSMMU, "pclk_smmu_disp_ro",
		"dout_pclk_disp", ENABLE_PCLK_DISP_RO_SMMU, 0,
		0, 0),
	GATE(CLK_PCLK_DISP_RW_SYSMMU, "pclk_smmu_disp_rw",
		"dout_pclk_disp", ENABLE_PCLK_DISP_RW_SMMU, 0,
		0, 0),

	GATE(CLK_SCLK_DECON0_ECLK, "decon0_eclk",
		"dout_sub_sclk_decon_int_eclk", ENABLE_SCLK_DISP1, 28,
		CLK_IGNORE_UNUSED, 0),
	GATE(CLK_SCLK_DECON0_VCLK, "decon0_vclk",
		"dout_sub_sclk_decon_int_vclk", ENABLE_SCLK_DISP1, 8,
		CLK_IGNORE_UNUSED, 0),
};

static const unsigned long disp_clk_regs[] __initconst = {
	DISP_PLL_LOCK,
	DISP_PLL_CON0,
	MUX_SEL_DISP0,
	MUX_SEL_DISP1,
	MUX_SEL_DISP2,
	MUX_SEL_DISP3,
	DIV_DISP,
	ENABLE_ACLK_DISP,
	ENABLE_ACLK_DISP_RO_SMMU,
	ENABLE_ACLK_DISP_RW_SMMU,
	ENABLE_PCLK_DISP,
	ENABLE_PCLK_DISP_RO_SMMU,
	ENABLE_PCLK_DISP_RW_SMMU,
	ENABLE_SCLK_DISP1,
	ENABLE_SCLK_DISP2,
};

static const struct samsung_cmu_info disp_cmu_info __initconst = {
	.fixed_clks = disp_fixed_clks,
	.nr_fixed_clks = ARRAY_SIZE(disp_fixed_clks),
	.pll_clks		= disp_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(disp_pll_clks),
	.mux_clks		= disp_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(disp_mux_clks),
	.div_clks		= disp_div_clks,
	.nr_div_clks		= ARRAY_SIZE(disp_div_clks),
	.gate_clks		= disp_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(disp_gate_clks),
	.nr_clk_ids		= DISP_NR_CLK,
	.clk_regs		= disp_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(disp_clk_regs),
};

/* Official m86 DISP table requires signed 16-bit fractional K. */
static const struct samsung_pll_rate_table m86_disp_pll_tbl[] __initconst = {
	PLL_36XX_RATE(24 * MHZ, 266000000, 67, 3, 1, -32768),
	PLL_36XX_RATE(24 * MHZ, 252000000, 42, 2, 1, 0),
	PLL_36XX_RATE(24 * MHZ, 240000000, 20, 1, 1, 0),
	PLL_36XX_RATE(24 * MHZ, 133000000, 67, 3, 2, -32768),
	PLL_36XX_RATE(24 * MHZ, 126000000, 21, 1, 2, 0),
	PLL_36XX_RATE(24 * MHZ, 96000000, 32, 1, 3, 0),
	{},
};
static const struct samsung_pll_clock m86_disp_pll_clks[] __initconst = {
	PLL(pll_1460x_signed, CLK_FOUT_DISP_PLL, "disp_pll", "fin_pll",
		DISP_PLL_LOCK, DISP_PLL_CON0, m86_disp_pll_tbl),
};
static void __init m86_clk_disp_init(struct device_node *np)
{
	struct samsung_cmu_info cmu = disp_cmu_info;

	cmu.pll_clks = m86_disp_pll_clks;
	cmu.nr_pll_clks = ARRAY_SIZE(m86_disp_pll_clks);
	samsung_cmu_register_one(np, &cmu);
}
CLK_OF_DECLARE(m86_clk_disp, "meizu,m86-clock-disp", m86_clk_disp_init);

static void __init exynos7_clk_disp_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &disp_cmu_info);
}

CLK_OF_DECLARE(exynos7_clk_disp, "samsung,exynos7-clock-disp",
		exynos7_clk_disp_init);

/* CMU_BUS0 (0x13400000): display longhop master clocks. */
#define MUX_SEL_BUS0		0x0200
#define ENABLE_ACLK_BUS0	0x0800
PNAME(mout_aclk_bus0_532_user_p) = { "fin_pll", "top_aclk_bus0_532" };
static const struct samsung_mux_clock bus0_mux_clks[] __initconst = {
	MUX(CLK_MOUT_ACLK_BUS0_532_USER, "usermux_aclk_bus0_532",
		mout_aclk_bus0_532_user_p, MUX_SEL_BUS0, 0, 1),
};
static const struct samsung_gate_clock bus0_gate_clks[] __initconst = {
	/*
	 * Vendor CLK_GATE_ENABLE holds permanent prepare/enable references
	 * for both NoC paths and their TOP parent. Keep the shared bus alive
	 * when DECON drops its last longhop reference during runtime suspend.
	 */
	GATE(0, "aclk_noc_bus0_rt", "usermux_aclk_bus0_532",
		ENABLE_ACLK_BUS0, 0, CLK_IS_CRITICAL, 0),
	GATE(0, "aclk_noc_bus0_nrt", "usermux_aclk_bus0_532",
		ENABLE_ACLK_BUS0, 7, CLK_IS_CRITICAL, 0),
	GATE(CLK_ACLK_LH_DISP0, "aclk_lh_disp0", "usermux_aclk_bus0_532",
		ENABLE_ACLK_BUS0, 3, 0, 0),
	GATE(CLK_ACLK_LH_DISP1, "aclk_lh_disp1", "usermux_aclk_bus0_532",
		ENABLE_ACLK_BUS0, 4, 0, 0),
};
static const unsigned long bus0_clk_regs[] __initconst = {
	MUX_SEL_BUS0, ENABLE_ACLK_BUS0,
};
static const struct samsung_cmu_info bus0_cmu_info __initconst = {
	.mux_clks = bus0_mux_clks,
	.nr_mux_clks = ARRAY_SIZE(bus0_mux_clks),
	.gate_clks = bus0_gate_clks,
	.nr_gate_clks = ARRAY_SIZE(bus0_gate_clks),
	.nr_clk_ids = BUS0_NR_CLK,
	.clk_regs = bus0_clk_regs,
	.nr_clk_regs = ARRAY_SIZE(bus0_clk_regs),
};
static void __init exynos7_clk_bus0_init(struct device_node *np)
{
	samsung_cmu_register_one(np, &bus0_cmu_info);
}
CLK_OF_DECLARE(exynos7_clk_bus0, "samsung,exynos7-clock-bus0",
		exynos7_clk_bus0_init);
