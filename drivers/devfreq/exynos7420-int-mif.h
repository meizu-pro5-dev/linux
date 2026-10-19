/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung Exynos 7420 Hardware INT & MIF (LPDDR4) DVFS Implementation
 *
 * Ported 1:1 from Meizu PRO 5 (m86) vendor 3.10 kernel:
 * - drivers/devfreq/exynos7420_bus.c
 * - drivers/clk/samsung/clk-exynos7420.c
 * - drivers/clk/samsung/composite.c
 * - arch/arm64/mach-exynos/include/mach/regs-clock-exynos7420.h
 * - arch/arm64/mach-exynos/asv-exynos7420_cal.c
 */

#ifndef __EXYNOS7420_INT_MIF_H
#define __EXYNOS7420_INT_MIF_H

#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/regulator/consumer.h>
#include <linux/m86-asv-cal.h>
#include <linux/soc/samsung/exynos7420-asv.h>

#define EXYNOS7420_NUM_MIF_CH		4
#define EXYNOS7420_MIF_LV_COUNT		13
#define EXYNOS7420_INT_LV_COUNT		12

/* Vendor DMC_DQS_MODE2 CMOS_SE_OFFSET (exynos7420_bus.c line 4643) */
#define EXYNOS7420_CMOS_SE_OFFSET_UV	56250
#define EXYNOS7420_VOLT_STEP_UV		100000

/*
 * CMU_TOPC (0x10570000) offsets from regs-clock-exynos7420.h:
 * Note: MIF_LOCK is 0x0014, MIF_CON is 0x0150, ENABLE_SCLK_TOPC1 bit 6 is
 * sclk_bus0_pll_mif (clk-exynos7420.c line 2044).
 */
#define TOPC_MIF_PLL_LOCK		0x0014
#define TOPC_CC_PLL_CON0		0x0100
#define TOPC_BUS0_PLL_CON0		0x0110
#define TOPC_BUS1_PLL_CON0		0x0120
#define TOPC_MFC_PLL_CON0		0x0130
#define TOPC_MIF_PLL_CON0		0x0150
#define TOPC_MUX_SEL0			0x0200
#define TOPC_MUX_SEL1			0x0204
#define TOPC_MUX_SEL2			0x0208
#define TOPC_MUX_SEL3			0x020c
#define TOPC_MUX_SEL5			0x0214
#define TOPC_MUX_STAT1			0x0404
#define TOPC_MUX_STAT2			0x0408
#define TOPC_MUX_STAT3			0x040c
#define TOPC_MUX_STAT5			0x0414
#define TOPC_DIV0			0x0600
#define TOPC_DIV1			0x0604
#define TOPC_DIV3			0x060c
#define TOPC_DIV_STAT0			0x0700
#define TOPC_DIV_STAT1			0x0704
#define TOPC_DIV_STAT3			0x070c
#define TOPC_ENABLE_ACLK0		0x0800
#define TOPC_ENABLE_ACLK1		0x0804
#define TOPC_ENABLE_SCLK1		0x0a04
#define TOPC_PAUSE			0x1008

/* CMU_TOP0 (0x105d0000) offsets */
#define TOP0_MUX_SEL3			0x020c
#define TOP0_MUX_STAT3			0x040c
#define TOP0_DIV3			0x060c
#define TOP0_DIV_STAT3			0x070c
#define TOP0_ENABLE_ACLK3		0x080c

/* CMU_MIF[0..3] (0x10850000 + ch * 0x100000) offsets */
#define MIF_DIV0			0x0600
#define MIF_DIV_STAT0			0x0700
#define MIF_PAUSE			0x1000
#define MIF_MUX_CTRL_BY_CMUC		BIT(20)

/* PMU_ALIVE (0x105c0000) power domain status offsets */
#define PMU_MFC_STATUS			0x4064
#define PMU_MSCL_STATUS			0x4084
#define PMU_VPP_STATUS			0x40a4

/* SYSREG (0x10050000) offsets */
#define SYSREG_EMA_INT_CON		0x2758

/*
 * DREX[0..3] (0x10800000 + ch * 0x100000) offsets
 * (exynos7420_bus.c lines 4170-4190, 4537-4540, 4609-4610)
 */
#define DREX_MEMCONTROL			0x0004
#define DREX_CGCONTROL			0x0008
#define DREX_DIRECTCMD			0x0010
#define DREX_TimingRfcPb		0x0020
#define DREX_PWRDNCONFIG		0x0028
#define DREX_TIMINGAREF			0x0030
#define DREX_TimingRow_0		0x0034
#define DREX_TimingData_0		0x0038
#define DREX_TimingPower_0		0x003c
#define DREX_PhyStatus			0x0040
#define DREX_RdFetch_0			0x004c
#define DREX_RdFetch_1			0x0050
#define DREX_EtcControl_0		0x0058
#define DREX_EtcControl_1		0x005c
#define DREX_TimingRow_1		0x00e4
#define DREX_TimingData_1		0x00e8
#define DREX_TimingPower_1		0x00ec
#define DREX_Train_Timing_0		0x0410
#define DREX_Train_Timing_1		0x0414
#define DREX_Hw_Ptrain_Period_0		0x0420
#define DREX_Hw_Ptrain_Period_1		0x0424
#define DREX_Hwpr_Train_Config_0	0x0440
#define DREX_Hwpr_Train_Config_1	0x0444
#define DREX_Hwpr_Train_Control_0	0x0450
#define DREX_Hwpr_Train_Control_1	0x0454

#define DREX_TimingRfcPb_Set1_MASK	0x0000ff00U
#define DREX_TimingRfcPb_Set0_MASK	0x000000ffU
#define DREX_PB_REF_EN			BIT(27)
#define DREX_DSREF_EN			BIT(5)
#define DREX_PHY_CG_EN			BIT(4)

/*
 * LP4_PHY[0..3] (0x10820000 + ch * 0x100000) offsets & masks
 * (exynos7420_bus.c lines 3634-3657, 4192-4198)
 */
#define PHY_CAL_CON0			0x0004
#define PHY_DVFS_CON0			0x00b8
#define PHY_DVFS_CON1			0x00bc
#define PHY_DVFS_CON2			0x00c0
#define PHY_DVFS_CON3			0x00c4
#define PHY_DVFS_CON4			0x00c8
#define PHY_DVFS_CON5			0x00cc
#define PHY_ZQ_CON9			0x03ec

#define PHY_DVFS_CON0_SET1_MASK		((1U << 31) | (1U << 29) | (0x3U << 24) | (1U << 23) | (1U << 21) | (1U << 19) | (1U << 17) | (0xffU << 8))
#define PHY_DVFS_CON0_SET0_MASK		((1U << 30) | (1U << 28) | (0x3U << 24) | (1U << 22) | (1U << 20) | (1U << 18) | (1U << 16) | (0xffU << 0))
#define PHY_DVFS_CON1_SET1_MASK		((1U << 31) | (0x3fU << 24) | (0xfU << 12) | (0xfU << 8))
#define PHY_DVFS_CON1_SET0_MASK		((1U << 30) | (0x3fU << 16) | (0xfU << 4) | (0xfU << 0))
#define PHY_DVFS_CON2_SET1_MASK		(0x7fU << 25)
#define PHY_DVFS_CON2_SET0_MASK		(0x7fU << 18)
#define PHY_DVFS_CON3_SET1_MASK		((1U << 31) | (1U << 29) | (1U << 27) | (0xfffU << 12))
#define PHY_DVFS_CON3_SET0_MASK		((1U << 30) | (1U << 28) | (1U << 26) | (0xfffU << 0))
#define PHY_DVFS_CON4_SET1_MASK		((0x3fU << 18) | (0x3fU << 12))
#define PHY_DVFS_CON4_SET0_MASK		((0x3fU << 6) | (0x3fU << 0))
#define PHY_DVFS_CON5_SET1_MASK		((1U << 31) | (0xfU << 20) | (0xfU << 12) | (0xfU << 4))
#define PHY_DVFS_CON5_SET0_MASK		((1U << 30) | (0xfU << 16) | (0xfU << 8) | (0xfU << 0))

/*
 * VT_MON[0..3] (0x10890000 + ch * 0x100000) offsets
 * (exynos7420_bus.c lines 4200-4205, 4368-4376)
 */
#define VTMON_CON_OFFSET		0x0014
#define VTMON_MIF_DREX_WDATA0		0x001c
#define VTMON_DREX_TIMING_SW_SET	0x003c
#define VTMON_PER_UPDATE_STATUS		0x0064
#define VTMON_CNT_LIMIT_SET0_SW0	0x0080
#define VTMON_CNT_LIMIT_SET1_SW0	0x0084
#define VTMON_CNT_LIMIT_SET0_SW1	0x0088
#define VTMON_CNT_LIMIT_SET1_SW1	0x008c

#define VTMON_AREF_UPDATE_EN		BIT(28)
#define VTMON_PER_MRS_EN		BIT(17)

enum exynos7420_mif_lv_idx {
	MIF_LV0 = 0,	/* 1552 MHz */
	MIF_LV1,	/* 1456 MHz */
	MIF_LV2,	/* 1264 MHz */
	MIF_LV3,	/* 1026 MHz */
	MIF_LV4,	/*  828 MHz */
	MIF_LV5,	/*  632 MHz */
	MIF_LV6,	/*  543 MHz */
	MIF_LV7,	/*  416 MHz */
	MIF_LV8,	/*  348 MHz */
	MIF_LV9,	/*  276 MHz */
	MIF_LV10,	/*  167 MHz */
	MIF_LV11,	/*  133 MHz */
	MIF_LV12,	/*  100 MHz */
};

/*
 * Vendor DREX timing parameter structure (exynos7420_bus.c lines 3493-3511)
 */
struct dmc_drex_dfs_mif_table {
	u32 drex_TimingRfcPb;
	u32 drex_TimingRow;
	u32 drex_TimingData;
	u32 drex_TimingPower;
	u32 drex_RdFetch;
	u32 drex_EtcControl;
	u32 drex_Train_Timing;
	u32 drex_Hw_Ptrain_Period;
	u32 drex_Hwpr_Train_Config;
	u32 drex_Hwpr_Train_Control;
	u32 drex_DirectCmd_mr13;
	u32 drex_DirectCmd_mr2;
	u32 drex_DirectCmd_mr22;
	u32 drex_DirectCmd_mr11;
	u32 drex_DirectCmd_mr14;
	u32 vtmon_Drex_Timing_Set_Sw;
};

/*
 * Vendor PHY DVFS parameter structure (exynos7420_bus.c lines 3513-3538)
 */
struct dmc_phy_dfs_mif_table {
	u32 phy_Dvfs_Con0_set1;
	u32 phy_Dvfs_Con0_set0;
	u32 phy_Dvfs_Con0_set1_mask;
	u32 phy_Dvfs_Con0_set0_mask;
	u32 phy_Dvfs_Con1_set1;
	u32 phy_Dvfs_Con1_set0;
	u32 phy_Dvfs_Con1_set1_mask;
	u32 phy_Dvfs_Con1_set0_mask;
	u32 phy_Dvfs_Con2_set1;
	u32 phy_Dvfs_Con2_set0;
	u32 phy_Dvfs_Con2_set1_mask;
	u32 phy_Dvfs_Con2_set0_mask;
	u32 phy_Dvfs_Con3_set1;
	u32 phy_Dvfs_Con3_set0;
	u32 phy_Dvfs_Con3_set1_mask;
	u32 phy_Dvfs_Con3_set0_mask;
	u32 phy_Dvfs_Con4_set1;
	u32 phy_Dvfs_Con4_set0;
	u32 phy_Dvfs_Con4_set1_mask;
	u32 phy_Dvfs_Con4_set0_mask;
	u32 phy_Dvfs_Con5_set1;
	u32 phy_Dvfs_Con5_set0;
	u32 phy_Dvfs_Con5_set1_mask;
	u32 phy_Dvfs_Con5_set0_mask;
};

/*
 * Vendor VTMON parameter structure (exynos7420_bus.c lines 3540-3544)
 */
struct dmc_vtmon_dfs_mif_table {
	u32 vtmon_Cnt_Limit_set0;
	u32 vtmon_Cnt_Limit_set1;
};

#define DREX_TIMING_PARA(rfcpb, row, data, power, fetch, etc_con, train_timing, hw_ptrain_period, \
		hwpr_train_config, hwpr_train_control, mr13, mr2, mr22, mr11, mr14, vtmon_drex_timing_set_sw) \
{ \
	.drex_TimingRfcPb	= (rfcpb), \
	.drex_TimingRow		= (row), \
	.drex_TimingData	= (data), \
	.drex_TimingPower	= (power), \
	.drex_RdFetch		= (fetch), \
	.drex_EtcControl	= (etc_con), \
	.drex_Train_Timing	= (train_timing), \
	.drex_Hw_Ptrain_Period	= (hw_ptrain_period), \
	.drex_Hwpr_Train_Config	= (hwpr_train_config), \
	.drex_Hwpr_Train_Control = (hwpr_train_control), \
	.drex_DirectCmd_mr13	= (mr13), \
	.drex_DirectCmd_mr2	= (mr2), \
	.drex_DirectCmd_mr22	= (mr22), \
	.drex_DirectCmd_mr11	= (mr11), \
	.drex_DirectCmd_mr14	= (mr14), \
	.vtmon_Drex_Timing_Set_Sw = (vtmon_drex_timing_set_sw), \
}

#define PHY_DVFS_CON(con0_set1, con0_set0, con0_set1_mask, con0_set0_mask, con1_set1, con1_set0, con1_set1_mask, con1_set0_mask, \
		con2_set1, con2_set0, con2_set1_mask, con2_set0_mask, con3_set1, con3_set0, con3_set1_mask, con3_set0_mask, \
		con4_set1, con4_set0, con4_set1_mask, con4_set0_mask, con5_set1, con5_set0, con5_set1_mask, con5_set0_mask) \
{ \
	.phy_Dvfs_Con0_set1		= (con0_set1), \
	.phy_Dvfs_Con0_set0		= (con0_set0), \
	.phy_Dvfs_Con0_set1_mask	= (con0_set1_mask), \
	.phy_Dvfs_Con0_set0_mask	= (con0_set0_mask), \
	.phy_Dvfs_Con1_set1		= (con1_set1), \
	.phy_Dvfs_Con1_set0		= (con1_set0), \
	.phy_Dvfs_Con1_set1_mask	= (con1_set1_mask), \
	.phy_Dvfs_Con1_set0_mask	= (con1_set0_mask), \
	.phy_Dvfs_Con2_set1		= (con2_set1), \
	.phy_Dvfs_Con2_set0		= (con2_set0), \
	.phy_Dvfs_Con2_set1_mask	= (con2_set1_mask), \
	.phy_Dvfs_Con2_set0_mask	= (con2_set0_mask), \
	.phy_Dvfs_Con3_set1		= (con3_set1), \
	.phy_Dvfs_Con3_set0		= (con3_set0), \
	.phy_Dvfs_Con3_set1_mask	= (con3_set1_mask), \
	.phy_Dvfs_Con3_set0_mask	= (con3_set0_mask), \
	.phy_Dvfs_Con4_set1		= (con4_set1), \
	.phy_Dvfs_Con4_set0		= (con4_set0), \
	.phy_Dvfs_Con4_set1_mask	= (con4_set1_mask), \
	.phy_Dvfs_Con4_set0_mask	= (con4_set0_mask), \
	.phy_Dvfs_Con5_set1		= (con5_set1), \
	.phy_Dvfs_Con5_set0		= (con5_set0), \
	.phy_Dvfs_Con5_set1_mask	= (con5_set1_mask), \
	.phy_Dvfs_Con5_set0_mask	= (con5_set0_mask), \
}

/*
 * Exact vendor DMC_DQS_MODE2 (with L6_RL_WL_DECREASE and L9_RL_WL_DECREASE)
 * dfs_drex_mif_table[] from exynos7420_bus.c lines 3659-3697.
 */
static const struct dmc_drex_dfs_mif_table dfs_drex_mif_table[EXYNOS7420_MIF_LV_COUNT] = {
	/* LV0: 1552 MHz */
	DREX_TIMING_PARA(0x00002323, 0x46588652, 0x4732BBE0, 0x404A0336, 0x00004E02, 0x00000000, 0x319BA14A, 0x00005DC0, 0x01010101, 0x00000003,
			0x00211400, 0x00200800 | (0x2D << 2), 0x00221800 | (0x26 << 2), 0x00210c00 | (0x04 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV1: 1456 MHz */
	DREX_TIMING_PARA(0x00002121, 0x42588611, 0x4732BBE0, 0x3C460336, 0x00004902, 0x00000000, 0x319BA14A, 0x00005DC0, 0x01010101, 0x00000003,
			0x00211400, 0x00200800 | (0x2D << 2), 0x00221800 | (0x26 << 2), 0x00210c00 | (0x04 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV2: 1264 MHz */
	DREX_TIMING_PARA(0x00001D1D, 0x3947754F, 0x4632B3DC, 0x343D0335, 0x00004002, 0x00000000, 0x319BA13A, 0x00005DC0, 0x01010101, 0x00000003,
			0x00211400, 0x00200800 | (0x24 << 2), 0x00221800 | (0x26 << 2), 0x00210c00 | (0x04 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV3: 1026 MHz */
	DREX_TIMING_PARA(0x00001919, 0x3146648C, 0x3532B39C, 0x2C340335, 0x00003602, 0x00000000, 0x319BA13A, 0x00005DC0, 0x01010101, 0x00000003,
			0x00211400, 0x00200800 | (0x24 << 2), 0x00221800 | (0x26 << 2), 0x00210c00 | (0x04 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV4: 828 MHz */
	DREX_TIMING_PARA(0x00001313, 0x2635538A, 0x3422AAD6, 0x24280234, 0x00002A02, 0x00000000, 0x219BA12A, 0x00FFFFFF, 0x00000000, 0x00000000,
			0x00211400, 0x00200800 | (0x1B << 2), 0x00221800 | (0x26 << 2), 0x00210c00 | (0x04 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV5: 632 MHz */
	DREX_TIMING_PARA(0x00000F0F, 0x1D2442C8, 0x2322A310, 0x1C1F0233, 0x00002003, 0x00000000, 0x019BA10B, 0x00FFFFFF, 0x00000000, 0x00000000,
			0x00211400, 0x00200800 | (0x12 << 2), 0x00221800 | (0x26 << 2), 0x00210c00 | (0x04 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV6: 543 MHz (L6_RL_WL_DECREASE) */
	DREX_TIMING_PARA(0x00000D0D, 0x19233247, 0x23229ACC, 0x181F0233, 0x00001C02, 0x00000000, 0x019BA10A, 0x00FFFFFF, 0x00000000, 0x00000000,
			0x00211400, 0x00200800 | (0x09 << 2), 0x00221800 | (0x26 << 2), 0x00210c00 | (0x04 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV7: 416 MHz */
	DREX_TIMING_PARA(0x00000A0A, 0x192331C5, 0x22229ACC, 0x141F0233, 0x00001502, 0x00000000, 0x019BA0FA, 0x00FFFFFF, 0x00000000, 0x00000000,
			0x00211400, 0x00200800 | (0x09 << 2), 0x00221800 | (0x24 << 2), 0x00210c00 | (0x00 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV8: 348 MHz */
	DREX_TIMING_PARA(0x00000808, 0x19232185, 0x22229ACC, 0x101F0233, 0x00001202, 0x00000000, 0x019BA0FA, 0x00FFFFFF, 0x00000000, 0x00000000,
			0x00211400, 0x00200800 | (0x09 << 2), 0x00221800 | (0x24 << 2), 0x00210c00 | (0x00 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV9: 276 MHz (L9_RL_WL_DECREASE) */
	DREX_TIMING_PARA(0x00000707, 0x19232144, 0x22229286, 0x0C1F0233, 0x00000E02, 0x00000000, 0x019BA0FA, 0x00FFFFFF, 0x00000000, 0x00000000,
			0x00211400, 0x00200800 | (0x00 << 2), 0x00221800 | (0x24 << 2), 0x00210c00 | (0x00 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV10: 167 MHz */
	DREX_TIMING_PARA(0x00000404, 0x192320C3, 0x22229286, 0x081F0233, 0x00000902, 0x00000000, 0x019BA0EA, 0x00FFFFFF, 0x00000000, 0x00000000,
			0x00211400, 0x00200800 | (0x00 << 2), 0x00221800 | (0x24 << 2), 0x00210c00 | (0x00 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV11: 133 MHz */
	DREX_TIMING_PARA(0x00000303, 0x192320C2, 0x22229286, 0x081F0233, 0x00000702, 0x00000000, 0x019BA0EA, 0x00FFFFFF, 0x00000000, 0x00000000,
			0x00211400, 0x00200800 | (0x00 << 2), 0x00221800 | (0x24 << 2), 0x00210c00 | (0x00 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
	/* LV12: 100 MHz */
	DREX_TIMING_PARA(0x00000303, 0x192320C2, 0x22229286, 0x081F0233, 0x00000502, 0x00000000, 0x019BA0EA, 0x00FFFFFF, 0x00000000, 0x00000000,
			0x00211400, 0x00200800 | (0x00 << 2), 0x00221800 | (0x24 << 2), 0x00210c00 | (0x00 << 2), 0x00211800 | (0x0F << 2), 0x00000000),
};

/*
 * Exact vendor DMC_DQS_MODE2 (dvfs_dqs_osc_en=0, dvfs_offset=0, L6/L9_RL_WL_DECREASE)
 * dfs_phy_mif_table[] from exynos7420_bus.c lines 3699-3797.
 */
static const struct dmc_phy_dfs_mif_table dfs_phy_mif_table[EXYNOS7420_MIF_LV_COUNT] = {
	/* LV0: 1552 MHz */
	PHY_DVFS_CON(0x80200000, 0x40100000, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x2000B000, 0x002000B0, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x0860E000, 0x0400060E, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x80700000, 0x40070000, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV1: 1456 MHz */
	PHY_DVFS_CON(0x82200000, 0x41100000, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x2000B000, 0x002000B0, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x085B0000, 0x040005B0, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x80700000, 0x40070000, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV2: 1264 MHz */
	PHY_DVFS_CON(0x82200000, 0x41100000, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x1C00B000, 0x001C00B0, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x084F0000, 0x040004F0, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x80700000, 0x40070000, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV3: 1026 MHz */
	PHY_DVFS_CON(0x82200000, 0x41100000, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x1C00B000, 0x001C00B0, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x0842C000, 0x0400042C, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x80700000, 0x40070000, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV4: 828 MHz */
	PHY_DVFS_CON(0x82200000, 0x41100000, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x16004000, 0x00160040, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x0033C000, 0x0000033C, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x00501000, 0x00050100, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV5: 632 MHz */
	PHY_DVFS_CON(0x82200000, 0x41100000, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x10004000, 0x00100040, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x00278000, 0x00000278, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x00501000, 0x00050100, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV6: 543 MHz (L6_RL_WL_DECREASE) */
	PHY_DVFS_CON(0x82200000, 0x41100000, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x0C004000, 0x000C0040, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x0021F000, 0x0000021F, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x00501000, 0x00050100, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV7: 416 MHz */
	PHY_DVFS_CON(0x82200000, 0x41500000, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x0C004000, 0x400C0040, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x001A0000, 0x000001A0, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x00501000, 0x0005010F, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV8: 348 MHz */
	PHY_DVFS_CON(0x82A80000, 0x41540000, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x8C004000, 0x400C0040, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x0015C000, 0x0000015C, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x005010F0, 0x0005010F, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV9: 276 MHz (L9_RL_WL_DECREASE) */
	PHY_DVFS_CON(0x82A80000, 0x41540000, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x86004000, 0x40060040, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x00114000, 0x00000114, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x005010F0, 0x0005010F, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV10: 167 MHz */
	PHY_DVFS_CON(0x82803000, 0x41400030, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x86004000, 0x40060040, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x000A7000, 0x000000A7, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x005010F0, 0x0005010F, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV11: 133 MHz */
	PHY_DVFS_CON(0x82803000, 0x41400030, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x86004000, 0x40060040, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x00085000, 0x00000085, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x005010F0, 0x0005010F, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
	/* LV12: 100 MHz */
	PHY_DVFS_CON(0x82803000, 0x41400030, PHY_DVFS_CON0_SET1_MASK, PHY_DVFS_CON0_SET0_MASK,
		     0x86004000, 0x40060040, PHY_DVFS_CON1_SET1_MASK, PHY_DVFS_CON1_SET0_MASK,
		     0x20000000, 0x00400000, PHY_DVFS_CON2_SET1_MASK, PHY_DVFS_CON2_SET0_MASK,
		     0x00064000, 0x00000064, PHY_DVFS_CON3_SET1_MASK, PHY_DVFS_CON3_SET0_MASK,
		     0x00000000, 0x00000000, PHY_DVFS_CON4_SET1_MASK, PHY_DVFS_CON4_SET0_MASK,
		     0x005010F0, 0x0005010F, PHY_DVFS_CON5_SET1_MASK, PHY_DVFS_CON5_SET0_MASK),
};

/*
 * Exact vendor dfs_vtmon_mif_table[] from exynos7420_bus.c lines 4067-4082.
 */
static const struct dmc_vtmon_dfs_mif_table dfs_vtmon_mif_table[EXYNOS7420_MIF_LV_COUNT] = {
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
	{ 0x00480060, 0x000003e8 },
};

/*
 * Exact vendor MIF frequency, ASV voltage (table_version=15, group=8),
 * PLL1450x PMS (clk-exynos7420.c table_mif[] lines 702-718),
 * clkdiv_sclk_hpm[] (exynos7420_bus.c lines 4426-4440), and
 * CCORE clock mux/div (exynos7420_bus.c lines 3451-3481).
 */
struct exynos7420_mif_opp_info {
	u32 freq_khz;
	u32 volt_uv;
	u16 mdiv;
	u8 pdiv;
	u8 sdiv;
	u8 hpm_div;
	u8 ccore_532_mux;
	u8 ccore_532_div;
	u8 ccore_133_div;
};

static struct exynos7420_mif_opp_info exynos7420_mif_opp_table[EXYNOS7420_MIF_LV_COUNT] = {
	/* LV0: 1552 MHz (MIF_PLL 3104 MHz: P=3, M=388, S=0), ccore_532=532MHz (mux=2, div=0), ccore_133=80MHz (div=4) */
	[MIF_LV0]  = { 1552000, 837500, 388, 3, 0, 2,  2, 0, 4 },
	/* LV1: 1456 MHz (MIF_PLL 2912 MHz: P=3, M=364, S=0), ccore_532=532MHz (mux=2, div=0), ccore_133=80MHz (div=4) */
	[MIF_LV1]  = { 1456000, 800000, 364, 3, 0, 2,  2, 0, 4 },
	/* LV2: 1264 MHz (MIF_PLL 2528 MHz: P=3, M=316, S=0), ccore_532=468MHz (mux=3, div=0), ccore_133=80MHz (div=4) */
	[MIF_LV2]  = { 1264000, 768750, 316, 3, 0, 1,  3, 0, 4 },
	/* LV3: 1026 MHz (MIF_PLL 2052 MHz: P=2, M=171, S=0), ccore_532=400MHz (mux=0, div=1), ccore_133=80MHz (div=4) */
	[MIF_LV3]  = { 1026000, 743750, 171, 2, 0, 1,  0, 1, 4 },
	/* LV4:  828 MHz (MIF_PLL 1656 MHz: P=3, M=207, S=0), ccore_532=334MHz (mux=5, div=0), ccore_133=80MHz (div=4) */
	[MIF_LV4]  = {  828000, 725000, 207, 3, 0, 1,  5, 0, 4 },
	/* LV5:  632 MHz (MIF_PLL 1264 MHz: P=3, M=316, S=1), ccore_532=334MHz (mux=5, div=0), ccore_133=67MHz (div=5) */
	[MIF_LV5]  = {  632000, 681250, 316, 3, 1, 1,  5, 0, 5 },
	/* LV6:  543 MHz (MIF_PLL 1086 MHz: P=2, M=181, S=1), ccore_532=266MHz (mux=6, div=0), ccore_133=50MHz (div=7) */
	[MIF_LV6]  = {  543000, 668750, 181, 2, 1, 1,  6, 0, 7 },
	/* LV7:  416 MHz (MIF_PLL  832 MHz: P=3, M=208, S=1), ccore_532=200MHz (mux=4, div=1), ccore_133=50MHz (div=7) */
	[MIF_LV7]  = {  416000, 600000, 208, 3, 1, 0,  4, 1, 7 },
	/* LV8:  348 MHz (MIF_PLL  696 MHz: P=3, M=348, S=2), ccore_532=167MHz (mux=9, div=0), ccore_133=50MHz (div=7) */
	[MIF_LV8]  = {  348000, 600000, 348, 3, 2, 0,  9, 0, 7 },
	/* LV9:  276 MHz (MIF_PLL  552 MHz: P=3, M=276, S=2), ccore_532=167MHz (mux=9, div=0), ccore_133=50MHz (div=7) */
	[MIF_LV9]  = {  276000, 600000, 276, 3, 2, 0,  9, 0, 7 },
	/* LV10: 167 MHz (MIF_PLL  334 MHz: P=3, M=334, S=3), ccore_532=133MHz (mux=10, div=0), ccore_133=50MHz (div=7) */
	[MIF_LV10] = {  167000, 600000, 334, 3, 3, 0, 10, 0, 7 },
	/* LV11: 133 MHz (MIF_PLL  266 MHz: P=3, M=266, S=3), ccore_532=100MHz (mux=8, div=1), ccore_133=50MHz (div=7) */
	[MIF_LV11] = {  133000, 600000, 266, 3, 3, 0,  8, 1, 7 },
	/* LV12: 100 MHz (MIF_PLL  200 MHz: P=3, M=200, S=3), ccore_532=100MHz (mux=8, div=1), ccore_133=50MHz (div=7) */
	[MIF_LV12] = {  100000, 600000, 200, 3, 3, 0,  8, 1, 7 },
};

struct exynos7420_int_level {
	u32 freq_khz;
	u32 volt_uv;
	u8 bus0_532_mux, bus0_532_div;
	u8 bus1_532_mux, bus1_532_div;
	u8 bus1_200_mux, bus1_200_div;
	u8 pclk_133_div;
	u8 imem_266_mux, imem_266_div;
	u8 imem_200_mux, imem_200_div;
	u8 mfc_532_mux, mfc_532_div;
	u8 mscl_532_mux, mscl_532_div;
	u8 vpp_400_mux, vpp_400_div;
};

/*
 * Vendor 3.10 INT table (exynos7420_bus.c int_clk_list and ASV table_version=15).
 * Voltages are dynamically populated at boot from ChipID eFuse ASV CAL.
 */
static struct exynos7420_int_level exynos7420_int_table[EXYNOS7420_INT_LV_COUNT] = {
	[0]  = { 560000, 806250, 2, 0, 2, 0, 0, 1,  2, 2, 1, 1, 3, 3, 0, 2, 0, 0, 1 },
	[1]  = { 550000, 806250, 2, 0, 2, 0, 0, 1,  2, 2, 1, 2, 3, 0, 0, 3, 0, 0, 1 },
	[2]  = { 540000, 750000, 0, 0, 0, 0, 0, 1,  3, 0, 1, 2, 3, 1, 1, 0, 0, 0, 1 },
	[3]  = { 530000, 750000, 0, 0, 0, 0, 0, 1,  3, 0, 1, 2, 3, 1, 1, 0, 0, 0, 1 },
	[4]  = { 520000, 750000, 0, 0, 0, 0, 0, 1,  3, 0, 1, 2, 3, 1, 1, 0, 0, 0, 1 },
	[5]  = { 510000, 750000, 0, 0, 0, 0, 0, 1,  3, 0, 1, 2, 3, 1, 1, 0, 0, 0, 1 },
	[6]  = { 500000, 750000, 0, 0, 0, 0, 0, 1,  3, 0, 1, 2, 3, 1, 1, 0, 0, 0, 1 },
	[7]  = { 400000, 725000, 0, 0, 0, 0, 0, 1,  3, 0, 1, 2, 3, 1, 1, 0, 0, 1, 1 },
	[8]  = { 334000, 700000, 5, 0, 5, 0, 1, 3,  4, 1, 3, 0, 3, 2, 1, 1, 1, 2, 1 },
	[9]  = { 266000, 700000, 6, 0, 6, 0, 2, 3,  5, 2, 3, 1, 7, 0, 1, 2, 1, 3, 1 },
	[10] = { 200000, 700000, 4, 0, 4, 0, 0, 3,  7, 0, 3, 0, 7, 0, 1, 0, 1, 3, 2 },
	[11] = { 100000, 700000, 4, 1, 4, 1, 0, 3, 15, 0, 3, 0, 7, 0, 1, 0, 1, 3, 3 },
};

/*
 * Mapped INT target index for each MIF level when scaling MIF down, ensuring
 * INT always scales down BEFORE MIF drops below the exynos7420_mif_table_for_int
 * hardware floor (exynos7_bus_int.c lines 39-52, 144-162).
 */
static const u8 exynos7420_mif_to_int_idx[EXYNOS7420_MIF_LV_COUNT] = {
	[MIF_LV0]  = 0,  /* 1552 MHz -> INT 560 MHz */
	[MIF_LV1]  = 2,  /* 1456 MHz -> INT 540 MHz */
	[MIF_LV2]  = 4,  /* 1264 MHz -> INT 520 MHz */
	[MIF_LV3]  = 6,  /* 1026 MHz -> INT 500 MHz */
	[MIF_LV4]  = 8,  /*  828 MHz -> INT 334 MHz */
	[MIF_LV5]  = 9,  /*  632 MHz -> INT 266 MHz */
	[MIF_LV6]  = 10, /*  543 MHz -> INT 200 MHz */
	[MIF_LV7]  = 10, /*  416 MHz -> INT 200 MHz */
	[MIF_LV8]  = 10, /*  348 MHz -> INT 200 MHz */
	[MIF_LV9]  = 10, /*  276 MHz -> INT 200 MHz */
	[MIF_LV10] = 10, /*  167 MHz -> INT 200 MHz */
	[MIF_LV11] = 10, /*  133 MHz -> INT 200 MHz */
	[MIF_LV12] = 11, /*  100 MHz -> INT 100 MHz */
};

struct exynos7420_hw_bus {
	bool initialized;
	struct mutex lock;
	void __iomem *topc;
	void __iomem *top0;
	void __iomem *pmu;
	void __iomem *sysreg;
	void __iomem *cmu_mif[EXYNOS7420_NUM_MIF_CH];
	void __iomem *drex[EXYNOS7420_NUM_MIF_CH];
	void __iomem *phy[EXYNOS7420_NUM_MIF_CH];
	void __iomem *vtmon[EXYNOS7420_NUM_MIF_CH];
	u32 g_phy_cg_en;
	u32 per_mrs_en;
	u32 int_ema_reg;
	int pll_safe_idx;
	int cur_mif_idx;
	int cur_int_idx;
	u32 cur_mif_volt_uv;
	u32 cur_int_volt_uv;
	unsigned long mif_switch_count;
	unsigned long int_switch_count;
	s64 last_mif_switch_us;
	s64 last_int_switch_us;
	struct exynos_bus *mif_bus;
	struct exynos_bus *int_bus;
	struct regulator *vdd_mif;
	struct regulator *vdd_int;
};

static struct exynos7420_hw_bus exynos7420_hw = {
	.lock = __MUTEX_INITIALIZER(exynos7420_hw.lock),
	.int_ema_reg = 0x12,
	.pll_safe_idx = MIF_LV7,
	.cur_mif_idx = MIF_LV0,
	.cur_int_idx = 7,
	.cur_mif_volt_uv = 837500,
	.cur_int_volt_uv = 725000,
};

void exynos7420_bus_dvfs_read_lock(void)
{
	mutex_lock(&exynos7420_hw.lock);
}
EXPORT_SYMBOL_GPL(exynos7420_bus_dvfs_read_lock);

void exynos7420_bus_dvfs_read_unlock(void)
{
	mutex_unlock(&exynos7420_hw.lock);
}
EXPORT_SYMBOL_GPL(exynos7420_bus_dvfs_read_unlock);

u32 exynos7420_mif_effective_floor_khz(u32 requested_khz)
{
	return requested_khz;
}
EXPORT_SYMBOL_GPL(exynos7420_mif_effective_floor_khz);

u32 exynos7420_int_effective_floor_khz(u32 requested_khz)
{
	return requested_khz;
}
EXPORT_SYMBOL_GPL(exynos7420_int_effective_floor_khz);

/*
 * Exact physical base addresses from exynos7420_bus.c lines 4368-4371, 5307-5350:
 * - DREX[0..3]:    0x10800000, 0x10900000, 0x10a00000, 0x10b00000
 * - LP4_PHY[0..3]: 0x10820000, 0x10920000, 0x10a20000, 0x10b20000
 * - CMU_MIF[0..3]: 0x10850000, 0x10950000, 0x10a50000, 0x10b50000
 * - VT_MON[0..3]:  0x10890000, 0x10990000, 0x10a90000, 0x10b90000
 */
static int exynos7420_hw_bus_init_once(struct device *dev)
{
	static const phys_addr_t drex_phys[EXYNOS7420_NUM_MIF_CH] = {
		0x10800000, 0x10900000, 0x10a00000, 0x10b00000
	};
	static const phys_addr_t phy_phys[EXYNOS7420_NUM_MIF_CH] = {
		0x10820000, 0x10920000, 0x10a20000, 0x10b20000
	};
	static const phys_addr_t cmu_mif_phys[EXYNOS7420_NUM_MIF_CH] = {
		0x10850000, 0x10950000, 0x10a50000, 0x10b50000
	};
	static const phys_addr_t vtmon_phys[EXYNOS7420_NUM_MIF_CH] = {
		0x10890000, 0x10990000, 0x10a90000, 0x10b90000
	};
	struct m86_asv_cal_info cal;
	int i, ret;

	if (exynos7420_hw.initialized)
		return 0;

	ret = exynos7420_asv_get_cal(&cal);
	if (ret)
		ret = m86_asv_cal_read(&cal);
	if (ret == 0) {
		for (i = 0; i < EXYNOS7420_MIF_LV_COUNT; i++) {
			exynos7420_mif_opp_table[i].volt_uv =
				m86_asv_lookup_volt(&cal, M86_ASV_MIF, i,
						    m86_mif_asv_volt_v15[i]);
		}
		for (i = 0; i < EXYNOS7420_INT_LV_COUNT; i++) {
			u32 v = m86_asv_lookup_volt(&cal, M86_ASV_INT, i,
						    m86_int_asv_volt_v15[i]);
			/* Preserve 700mV display/INT floor on lower levels */
			exynos7420_int_table[i].volt_uv = max(v, 700000U);
		}
		exynos7420_hw.cur_mif_volt_uv = exynos7420_mif_opp_table[MIF_LV0].volt_uv;
		exynos7420_hw.cur_int_volt_uv = exynos7420_int_table[7].volt_uv;
		exynos7420_hw.int_ema_reg = m86_asv_get_int_ema_reg(&cal);
		dev_info(dev, "MIF/INT ASV CAL: ver=%u mif_grp=0x%03x (%u..%u uV) int_grp=0x%03x (%u..%u uV) int_ema=0x%02x\n",
			 cal.effective_ver,
			 m86_asv_domain_group_raw(&cal, M86_ASV_MIF),
			 exynos7420_mif_opp_table[EXYNOS7420_MIF_LV_COUNT - 1].volt_uv,
			 exynos7420_mif_opp_table[0].volt_uv,
			 m86_asv_domain_group_raw(&cal, M86_ASV_INT),
			 exynos7420_int_table[EXYNOS7420_INT_LV_COUNT - 1].volt_uv,
			 exynos7420_int_table[0].volt_uv,
			 exynos7420_hw.int_ema_reg);
	} else {
		dev_warn(dev, "MIF/INT ASV CAL read failed (%d), keeping default v15/group8 table\n", ret);
	}

	exynos7420_hw.topc = ioremap(0x10570000, SZ_64K);
	exynos7420_hw.top0 = ioremap(0x105d0000, SZ_64K);
	exynos7420_hw.pmu = ioremap(0x105c0000, 0x5000);
	exynos7420_hw.sysreg = ioremap(0x10050000, 0x3000);
	if (!exynos7420_hw.topc || !exynos7420_hw.top0 ||
	    !exynos7420_hw.pmu || !exynos7420_hw.sysreg)
		return -ENOMEM;

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		exynos7420_hw.drex[i] = ioremap(drex_phys[i], SZ_64K);
		exynos7420_hw.phy[i] = ioremap(phy_phys[i], SZ_64K);
		exynos7420_hw.cmu_mif[i] = ioremap(cmu_mif_phys[i], SZ_64K);
		exynos7420_hw.vtmon[i] = ioremap(vtmon_phys[i], SZ_64K);
		if (!exynos7420_hw.drex[i] || !exynos7420_hw.phy[i] ||
		    !exynos7420_hw.cmu_mif[i] || !exynos7420_hw.vtmon[i])
			return -ENOMEM;
	}

	exynos7420_hw.initialized = true;
	return 0;
}

static int exynos7420_find_mif_idx(unsigned long freq_hz)
{
	u32 khz = freq_hz / 1000;
	int i;

	for (i = 0; i < EXYNOS7420_MIF_LV_COUNT; i++) {
		if (exynos7420_mif_opp_table[i].freq_khz == khz)
			return i;
	}
	return -EINVAL;
}

static int exynos7420_find_int_idx(unsigned long freq_hz)
{
	u32 khz = freq_hz / 1000;
	int i;

	for (i = 0; i < EXYNOS7420_INT_LV_COUNT; i++) {
		if (exynos7420_int_table[i].freq_khz == khz)
			return i;
	}
	return -EINVAL;
}

static int exynos7420_switch_div_only(void __iomem *div_reg, u8 div_shift,
				      u8 div_width, void __iomem *div_stat_reg,
				      u8 div_stat_bit, u8 new_div)
{
	u32 mask = ((1U << div_width) - 1U) << div_shift;
	u32 val, old_div;

	val = readl(div_reg);
	old_div = (val & mask) >> div_shift;
	if (old_div == new_div)
		return 0;

	val = (val & ~mask) | ((u32)new_div << div_shift);
	writel(val, div_reg);
	return readl_poll_timeout_atomic(div_stat_reg, val,
					 !(val & BIT(div_stat_bit)), 1, 10000);
}

/*
 * Safe composite clock switching matching vendor drivers/clk/samsung/composite.c:
 * - When increasing divider (new_div > old_div): write DIV first, then MUX.
 * - When decreasing/keeping divider (new_div <= old_div): write MUX first, then DIV.
 */
static int exynos7420_switch_composite(void __iomem *mux_reg, u8 mux_shift,
				       u8 mux_width, void __iomem *mux_stat_reg,
				       u8 mux_stat_shift, u8 mux_stat_width,
				       u8 new_mux,
				       void __iomem *div_reg, u8 div_shift,
				       u8 div_width, void __iomem *div_stat_reg,
				       u8 div_stat_bit, u8 new_div)
{
	u32 mux_mask = ((1U << mux_width) - 1U) << mux_shift;
	u32 div_mask = ((1U << div_width) - 1U) << div_shift;
	u32 busy_bit = BIT(mux_stat_shift + mux_stat_width - 1);
	u32 val, old_mux, old_div;
	int ret;

	old_mux = (readl(mux_reg) & mux_mask) >> mux_shift;
	old_div = (readl(div_reg) & div_mask) >> div_shift;

	if (new_div > old_div) {
		val = (readl(div_reg) & ~div_mask) | ((u32)new_div << div_shift);
		writel(val, div_reg);
		ret = readl_poll_timeout_atomic(div_stat_reg, val,
						!(val & BIT(div_stat_bit)),
						1, 10000);
		if (ret)
			return ret;

		if (new_mux != old_mux) {
			val = (readl(mux_reg) & ~mux_mask) |
			      ((u32)new_mux << mux_shift);
			writel(val, mux_reg);
			if (mux_stat_reg) {
				ret = readl_poll_timeout_atomic(mux_stat_reg, val,
								!(val & busy_bit),
								1, 10000);
				if (ret)
					return ret;
			}
		}
	} else {
		if (new_mux != old_mux) {
			val = (readl(mux_reg) & ~mux_mask) |
			      ((u32)new_mux << mux_shift);
			writel(val, mux_reg);
			if (mux_stat_reg) {
				ret = readl_poll_timeout_atomic(mux_stat_reg, val,
								!(val & busy_bit),
								1, 10000);
				if (ret)
					return ret;
			}
		}

		if (new_div != old_div) {
			val = (readl(div_reg) & ~div_mask) |
			      ((u32)new_div << div_shift);
			writel(val, div_reg);
			ret = readl_poll_timeout_atomic(div_stat_reg, val,
							!(val & BIT(div_stat_bit)),
							1, 10000);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int exynos7420_int_set_clocks(int target_idx)
{
	const struct exynos7420_int_level *lv = &exynos7420_int_table[target_idx];
	void __iomem *topc = exynos7420_hw.topc;
	int ret;

	/* Always-on CMU_TOPC INT clocks */
	ret = exynos7420_switch_composite(topc + TOPC_MUX_SEL2, 8, 4,
					  topc + TOPC_MUX_STAT2, 8, 4,
					  lv->bus0_532_mux,
					  topc + TOPC_DIV0, 8, 4,
					  topc + TOPC_DIV_STAT0, 8,
					  lv->bus0_532_div);
	if (ret)
		return ret;

	ret = exynos7420_switch_composite(topc + TOPC_MUX_SEL2, 12, 4,
					  topc + TOPC_MUX_STAT2, 12, 4,
					  lv->bus1_532_mux,
					  topc + TOPC_DIV0, 12, 4,
					  topc + TOPC_DIV_STAT0, 12,
					  lv->bus1_532_div);
	if (ret)
		return ret;

	ret = exynos7420_switch_composite(topc + TOPC_MUX_SEL2, 16, 4,
					  topc + TOPC_MUX_STAT2, 16, 4,
					  lv->bus1_200_mux,
					  topc + TOPC_DIV0, 16, 4,
					  topc + TOPC_DIV_STAT0, 16,
					  lv->bus1_200_div);
	if (ret)
		return ret;

	ret = exynos7420_switch_div_only(topc + TOPC_DIV1, 28, 4,
					 topc + TOPC_DIV_STAT1, 28,
					 lv->pclk_133_div);
	if (ret)
		return ret;

	ret = exynos7420_switch_composite(topc + TOPC_MUX_SEL2, 20, 2,
					  topc + TOPC_MUX_STAT2, 20, 4,
					  lv->imem_266_mux,
					  topc + TOPC_DIV0, 20, 4,
					  topc + TOPC_DIV_STAT0, 20,
					  lv->imem_266_div);
	if (ret)
		return ret;

	ret = exynos7420_switch_composite(topc + TOPC_MUX_SEL2, 24, 2,
					  topc + TOPC_MUX_STAT2, 24, 4,
					  lv->imem_200_mux,
					  topc + TOPC_DIV0, 24, 4,
					  topc + TOPC_DIV_STAT0, 24,
					  lv->imem_200_div);
	if (ret)
		return ret;

	return 0;
}

static int exynos7420_mif_target_locked(struct exynos_bus *bus,
					unsigned long target_freq);

static int exynos7420_int_required_mif_idx(int int_idx)
{
	if (int_idx <= 1)
		return MIF_LV5; /* 632 MHz for INT 560..550 MHz */
	if (int_idx <= 7)
		return MIF_LV6; /* 543 MHz for INT 540..400 MHz */
	return EXYNOS7420_MIF_LV_COUNT - 1; /* Lowest MIF level for INT <= 334 MHz */
}

static int exynos7420_int_target_locked(struct exynos_bus *bus,
					unsigned long target_freq)
{
	int old_idx = exynos7420_hw.cur_int_idx;
	int target_idx = exynos7420_find_int_idx(target_freq);
	u32 old_uv = exynos7420_hw.cur_int_volt_uv;
	u32 target_uv;
	ktime_t t0;
	int req_mif_idx;
	int ret;

	if (target_idx < 0)
		return target_idx;
	if (target_idx == old_idx) {
		bus->curr_freq = target_freq;
		return 0;
	}

	/* Vendor exynos7420_mif_table_for_int: ensure MIF meets prerequisite */
	req_mif_idx = exynos7420_int_required_mif_idx(target_idx);
	if (exynos7420_hw.mif_bus && exynos7420_hw.cur_mif_idx > req_mif_idx) {
		unsigned long mif_hz =
			(unsigned long)exynos7420_mif_opp_table[req_mif_idx].freq_khz * 1000UL;
		ret = exynos7420_mif_target_locked(exynos7420_hw.mif_bus, mif_hz);
		if (ret)
			return ret;
		if (exynos7420_hw.cur_mif_idx > req_mif_idx)
			return -EBUSY;
	}

	t0 = ktime_get();
	target_uv = exynos7420_int_table[target_idx].volt_uv;

	if (old_idx > target_idx) {
		/* Scaling UP: voltage -> EMA -> clocks */
		if (exynos7420_hw.vdd_int && target_uv != old_uv) {
			ret = regulator_set_voltage(exynos7420_hw.vdd_int,
						    target_uv,
						    target_uv + EXYNOS7420_VOLT_STEP_UV);
			if (ret)
				return ret;
			exynos7420_hw.cur_int_volt_uv = target_uv;
		}
		writel(exynos7420_hw.int_ema_reg, exynos7420_hw.sysreg + SYSREG_EMA_INT_CON);
		ret = exynos7420_int_set_clocks(target_idx);
		if (ret)
			return ret;
	} else {
		/* Scaling DOWN: clocks -> EMA -> voltage */
		ret = exynos7420_int_set_clocks(target_idx);
		if (ret)
			return ret;
		writel(exynos7420_hw.int_ema_reg, exynos7420_hw.sysreg + SYSREG_EMA_INT_CON);
		if (exynos7420_hw.vdd_int && target_uv != old_uv) {
			ret = regulator_set_voltage(exynos7420_hw.vdd_int,
						    target_uv,
						    target_uv + EXYNOS7420_VOLT_STEP_UV);
			if (ret)
				return ret;
			exynos7420_hw.cur_int_volt_uv = target_uv;
		}
	}

	exynos7420_hw.cur_int_idx = target_idx;
	exynos7420_hw.int_switch_count++;
	exynos7420_hw.last_int_switch_us = ktime_us_delta(ktime_get(), t0);
	bus->curr_freq = target_freq;
	if (bus->devfreq)
		bus->devfreq->previous_freq = target_freq;
	return 0;
}

/*
 * ============================================================================
 * Vendor 3.10 MIF (LPDDR4) functions ported 1:1 from exynos7420_bus.c
 * ============================================================================
 */

/* DisablePhyCgEn / RestorePhyCgEn (exynos7420_bus.c lines 4611-4640) */
static void exynos7420_mif_disable_phy_cg_en(void)
{
	u32 tmp;
	int i;

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		tmp = __raw_readl(exynos7420_hw.drex[i] + DREX_CGCONTROL);
		if (i == 0)
			exynos7420_hw.g_phy_cg_en = tmp & DREX_PHY_CG_EN;
		tmp &= ~DREX_PHY_CG_EN;
		__raw_writel(tmp, exynos7420_hw.drex[i] + DREX_CGCONTROL);
	}
}

static void exynos7420_mif_restore_phy_cg_en(void)
{
	u32 tmp;
	int i;

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		tmp = __raw_readl(exynos7420_hw.drex[i] + DREX_CGCONTROL);
		tmp |= exynos7420_hw.g_phy_cg_en;
		__raw_writel(tmp, exynos7420_hw.drex[i] + DREX_CGCONTROL);
	}
}

/* exynos7_devfreq_mif_set_volt (exynos7420_bus.c lines 4352-4366) */
static int exynos7420_mif_set_volt(u32 volt_uv)
{
	int ret;

	if (!exynos7420_hw.vdd_mif || exynos7420_hw.cur_mif_volt_uv == volt_uv)
		return 0;

	exynos7420_mif_disable_phy_cg_en();
	ret = regulator_set_voltage(exynos7420_hw.vdd_mif, volt_uv,
				    volt_uv + EXYNOS7420_VOLT_STEP_UV);
	exynos7420_mif_restore_phy_cg_en();
	if (!ret)
		exynos7420_hw.cur_mif_volt_uv = volt_uv;
	return ret;
}

/* disable_VT_MON_compensation / restore_VT_MON_compensation (lines 4378-4424) */
static int exynos7420_mif_disable_vt_mon(void)
{
	u32 temp, val;
	int ret, i;

	exynos7420_hw.per_mrs_en =
		__raw_readl(exynos7420_hw.vtmon[0] + VTMON_CON_OFFSET) &
		(VTMON_AREF_UPDATE_EN | VTMON_PER_MRS_EN);

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		temp = __raw_readl(exynos7420_hw.vtmon[i] + VTMON_CON_OFFSET);
		temp &= ~(VTMON_AREF_UPDATE_EN | VTMON_PER_MRS_EN);
		__raw_writel(temp, exynos7420_hw.vtmon[i] + VTMON_CON_OFFSET);
		ret = readl_poll_timeout_atomic(exynos7420_hw.vtmon[i] + VTMON_PER_UPDATE_STATUS,
						val, !(val & 0x1f), 1, 10000);
		if (ret) {
			pr_err("DEVFREQ(MIF): disable: timeout waiting per_update ch%d\n", i);
			return ret;
		}
	}
	return 0;
}

static int exynos7420_mif_restore_vt_mon(void)
{
	u32 temp, val;
	int ret, i;

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		temp = __raw_readl(exynos7420_hw.vtmon[i] + VTMON_CON_OFFSET);
		temp |= exynos7420_hw.per_mrs_en;
		__raw_writel(temp, exynos7420_hw.vtmon[i] + VTMON_CON_OFFSET);
		ret = readl_poll_timeout_atomic(exynos7420_hw.vtmon[i] + VTMON_PER_UPDATE_STATUS,
						val, !(val & 0x1f), 1, 10000);
		if (ret) {
			pr_err("DEVFREQ(MIF): restore: timeout waiting per_update ch%d\n", i);
			return ret;
		}
	}
	return 0;
}

/* exynos7_devfreq_waiting_pause (exynos7420_bus.c lines 4106-4132) */
static int exynos7420_mif_waiting_pause(void)
{
	u32 val;
	int ret, i;

	ret = readl_poll_timeout_atomic(exynos7420_hw.topc + TOPC_PAUSE, val,
					!(val & (0x07U << 16)), 1, 10000);
	if (ret) {
		pr_err("DEVFREQ(MIF): timeout waiting TOPC pause\n");
		return ret;
	}

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		ret = readl_poll_timeout_atomic(exynos7420_hw.cmu_mif[i] + MIF_PAUSE, val,
						!(val & (0x07U << 16)), 1, 10000);
		if (ret) {
			pr_err("DEVFREQ(MIF): timeout waiting CMU_MIF%d pause\n", i);
			return ret;
		}
	}
	return 0;
}

/* exynos7_devfreq_check_mif_mux_ctrl_by_cmuc (exynos7420_bus.c lines 4136-4153) */
static int exynos7420_mif_check_mux_ctrl_by_cmuc(void)
{
	u32 val;
	int ret, i;

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		ret = readl_poll_timeout_atomic(exynos7420_hw.cmu_mif[i] + MIF_PAUSE, val,
						(val & MIF_MUX_CTRL_BY_CMUC), 1, 10000);
		if (ret) {
			pr_err("DEVFREQ(MIF): timeout waiting mif_mux_ctrl_by_cmuc ch%d\n", i);
			return ret;
		}
	}
	return 0;
}

/* exynos7_devfreq_waiting_mux (exynos7420_bus.c lines 4155-4168) */
static int exynos7420_mif_waiting_mux(void)
{
	u32 val;
	int ret;

	/* check whether ACLK_MIF_PLL_CLK_STAT0/1/2/3 is 0 */
	ret = readl_poll_timeout_atomic(exynos7420_hw.topc + TOPC_MUX_STAT5, val,
					!(val & (0x0fU << 24)), 1, 10000);
	if (ret) {
		pr_err("DEVFREQ(MIF): timeout waiting mux completion\n");
		return ret;
	}
	return 0;
}

/* exynos7420_set_clkdiv_sclk_hpm (exynos7420_bus.c lines 4444-4483) */
static int exynos7420_set_clkdiv_sclk_hpm(int target_idx)
{
	u32 hpm_div = exynos7420_mif_opp_table[target_idx].hpm_div;
	u32 tmp, val;
	int ret, i;

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		void __iomem *cmu = exynos7420_hw.cmu_mif[i];

		tmp = __raw_readl(cmu + MIF_DIV0);
		tmp = (tmp & ~(0x07U << 4)) | ((hpm_div & 0x07U) << 4);
		__raw_writel(tmp, cmu + MIF_DIV0);
		ret = readl_poll_timeout_atomic(cmu + MIF_DIV_STAT0, val,
						!(val & BIT(4)), 1, 10000);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * exynos7_devfreq_ccore_set_freq (exynos7420_bus.c lines 4485-4535)
 * Note: in vendor clk-exynos7420.c line 1001, mout_aclk_ccore_532 uses
 * MUX_SEL_TOPC2[3:0] (no MUX_STAT busy wait) and DIV_TOPC0[3:0], while
 * dout_aclk_ccore_133 uses DIV_TOPC0[7:4].
 */
static int exynos7420_ccore_set_freq(int target_idx)
{
	const struct exynos7420_mif_opp_info *opp = &exynos7420_mif_opp_table[target_idx];
	void __iomem *topc = exynos7420_hw.topc;
	int ret;

	ret = exynos7420_switch_composite(topc + TOPC_MUX_SEL2, 0, 4,
					  NULL, 0, 0,
					  opp->ccore_532_mux,
					  topc + TOPC_DIV0, 0, 4,
					  topc + TOPC_DIV_STAT0, 0,
					  opp->ccore_532_div);
	if (ret)
		return ret;

	return exynos7420_switch_div_only(topc + TOPC_DIV0, 4, 4,
					  topc + TOPC_DIV_STAT0, 4,
					  opp->ccore_133_div);
}

/*
 * DOUT_SCLK_BUS0_PLL_MIF divider in TOPC_DIV3[6:4] (clk-exynos7420.c line 1356):
 * - 1600 MHz -> div = 0 (MIF switching clock = 1600/2 = 800 MHz, MIF_LV4)
 * -  800 MHz -> div = 1 (MIF switching clock =  800/2 = 400 MHz, MIF_LV7)
 */
static int exynos7420_set_dout_sclk_bus0_pll_mif(u32 div_val)
{
	void __iomem *topc = exynos7420_hw.topc;
	u32 tmp, val;

	tmp = __raw_readl(topc + TOPC_DIV3);
	if (((tmp >> 4) & 0x7U) == div_val)
		return 0;
	tmp = (tmp & ~(0x7U << 4)) | ((div_val & 0x7U) << 4);
	__raw_writel(tmp, topc + TOPC_DIV3);
	return readl_poll_timeout_atomic(topc + TOPC_DIV_STAT3, val,
					 !(val & BIT(4)), 1, 10000);
}

/*
 * samsung_pll145xx_set_rate for MIF_PLL (composite.c lines 316-365):
 * - If M and P are unchanged and only S changes: write S directly without
 *   clearing bit 31 (avoids PLL unlock when only post-scaler S changes).
 * - If M or P changes: write pdiv * 150 to TOPC_MIF_PLL_LOCK (0x0014),
 *   clear bit 31 with new M/P/S, set bit 31, and wait for bit 29 (LOCKED).
 */
static int exynos7420_mif_pll_set_rate(int target_idx)
{
	const struct exynos7420_mif_opp_info *opp = &exynos7420_mif_opp_table[target_idx];
	void __iomem *topc = exynos7420_hw.topc;
	u32 pll_con, cur_m, cur_p, cur_s, val;

	pll_con = __raw_readl(topc + TOPC_MIF_PLL_CON0);
	cur_m = (pll_con >> 16) & 0x3ffU;
	cur_p = (pll_con >> 8) & 0x3fU;
	cur_s = (pll_con >> 0) & 0x7U;

	if (cur_m == opp->mdiv && cur_p == opp->pdiv) {
		if (cur_s == opp->sdiv)
			return 0;
		pll_con &= ~0x7U;
		pll_con |= (opp->sdiv & 0x7U);
		__raw_writel(pll_con, topc + TOPC_MIF_PLL_CON0);
		return 0;
	}

	__raw_writel((u32)opp->pdiv * 150U, topc + TOPC_MIF_PLL_LOCK);

	pll_con &= ~((0x3ffU << 16) | (0x3fU << 8) | 0x7U);
	pll_con |= ((u32)opp->mdiv << 16) | ((u32)opp->pdiv << 8) | (u32)opp->sdiv;
	pll_con &= ~BIT(31);
	__raw_writel(pll_con, topc + TOPC_MIF_PLL_CON0);

	pll_con |= BIT(31);
	__raw_writel(pll_con, topc + TOPC_MIF_PLL_CON0);

	return readl_poll_timeout_atomic(topc + TOPC_MIF_PLL_CON0, val,
					 (val & BIT(29)), 1, 10000);
}

/*
 * exynos7_devfreq_mif_preset (exynos7420_bus.c lines 4211-4348)
 * Ported line-by-line without any modification to register offsets or values.
 */
static int exynos7420_mif_preset(int target_idx, int timing_set_num)
{
	const struct dmc_drex_dfs_mif_table *cur_drex_param = &dfs_drex_mif_table[target_idx];
	const struct dmc_phy_dfs_mif_table *cur_phy_param = &dfs_phy_mif_table[target_idx];
	const struct dmc_vtmon_dfs_mif_table *cur_vtmon_param = &dfs_vtmon_mif_table[target_idx];
	u32 soc_vref[EXYNOS7420_NUM_MIF_CH][2];
	u32 tmp;
	int i;

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		soc_vref[i][1] = (__raw_readl(exynos7420_hw.phy[i] + PHY_ZQ_CON9) & 0x3f00U) >> 8;
		soc_vref[i][0] = (__raw_readl(exynos7420_hw.phy[i] + PHY_ZQ_CON9) & 0x003fU) >> 0;
	}

	if (timing_set_num) {
		for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
			void __iomem *drex = exynos7420_hw.drex[i];
			void __iomem *phy = exynos7420_hw.phy[i];
			void __iomem *vtmon = exynos7420_hw.vtmon[i];

			/* DREX */
			tmp = __raw_readl(drex + DREX_TimingRfcPb);
			tmp &= ~(DREX_TimingRfcPb_Set1_MASK);
			tmp |= (cur_drex_param->drex_TimingRfcPb & DREX_TimingRfcPb_Set1_MASK);
			__raw_writel(tmp, drex + DREX_TimingRfcPb);
			__raw_writel(cur_drex_param->drex_TimingRow, drex + DREX_TimingRow_1);
			__raw_writel(cur_drex_param->drex_TimingData, drex + DREX_TimingData_1);
			__raw_writel(cur_drex_param->drex_TimingPower, drex + DREX_TimingPower_1);
			__raw_writel(cur_drex_param->drex_RdFetch, drex + DREX_RdFetch_1);
			__raw_writel((0x1U << 7) | cur_drex_param->drex_EtcControl, drex + DREX_EtcControl_1);
			__raw_writel(cur_drex_param->drex_Train_Timing, drex + DREX_Train_Timing_1);
			__raw_writel(cur_drex_param->drex_Hw_Ptrain_Period, drex + DREX_Hw_Ptrain_Period_1);
			__raw_writel(cur_drex_param->drex_Hwpr_Train_Config, drex + DREX_Hwpr_Train_Config_1);
			__raw_writel(cur_drex_param->drex_Hwpr_Train_Control, drex + DREX_Hwpr_Train_Control_1);

			/* LP4_PHY */
			tmp = __raw_readl(phy + PHY_DVFS_CON0);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con0_set1_mask);
			tmp |= cur_phy_param->phy_Dvfs_Con0_set1;
			__raw_writel(tmp, phy + PHY_DVFS_CON0);

			tmp = __raw_readl(phy + PHY_DVFS_CON1);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con1_set1_mask);
			tmp |= cur_phy_param->phy_Dvfs_Con1_set1;
			__raw_writel(tmp, phy + PHY_DVFS_CON1);

			tmp = __raw_readl(phy + PHY_DVFS_CON2);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con2_set1_mask);
			tmp |= cur_phy_param->phy_Dvfs_Con2_set1;
			__raw_writel(tmp, phy + PHY_DVFS_CON2);

			tmp = __raw_readl(phy + PHY_DVFS_CON3);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con3_set1_mask);
			tmp |= cur_phy_param->phy_Dvfs_Con3_set1;
			__raw_writel(tmp, phy + PHY_DVFS_CON3);

			tmp = __raw_readl(phy + PHY_DVFS_CON4);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con4_set1_mask);
			tmp |= (cur_phy_param->phy_Dvfs_Con4_set1 | (soc_vref[i][1] << 18) | (soc_vref[i][0] << 12));
			__raw_writel(tmp, phy + PHY_DVFS_CON4);

			tmp = __raw_readl(phy + PHY_DVFS_CON5);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con5_set1_mask);
			tmp |= cur_phy_param->phy_Dvfs_Con5_set1;
			__raw_writel(tmp, phy + PHY_DVFS_CON5);

			__raw_writel(cur_vtmon_param->vtmon_Cnt_Limit_set0, vtmon + VTMON_CNT_LIMIT_SET0_SW1);
			__raw_writel(cur_vtmon_param->vtmon_Cnt_Limit_set1, vtmon + VTMON_CNT_LIMIT_SET1_SW1);

			/* DIRECTCMD */
			__raw_writel((0x40U << 2) | cur_drex_param->drex_DirectCmd_mr13, drex + DREX_DIRECTCMD);
			__raw_writel(cur_drex_param->drex_DirectCmd_mr2, drex + DREX_DIRECTCMD);
			__raw_writel(cur_drex_param->drex_DirectCmd_mr22, drex + DREX_DIRECTCMD);
			__raw_writel(cur_drex_param->drex_DirectCmd_mr11, drex + DREX_DIRECTCMD);

			/* VTMON */
			__raw_writel((0x1U << 0) | cur_drex_param->vtmon_Drex_Timing_Set_Sw, vtmon + VTMON_DREX_TIMING_SW_SET);
			__raw_writel((0xC0U << 2) | cur_drex_param->drex_DirectCmd_mr13, vtmon + VTMON_MIF_DREX_WDATA0);
		}
	} else {
		for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
			void __iomem *drex = exynos7420_hw.drex[i];
			void __iomem *phy = exynos7420_hw.phy[i];
			void __iomem *vtmon = exynos7420_hw.vtmon[i];

			/* DREX */
			tmp = __raw_readl(drex + DREX_TimingRfcPb);
			tmp &= ~(DREX_TimingRfcPb_Set0_MASK);
			tmp |= (cur_drex_param->drex_TimingRfcPb & DREX_TimingRfcPb_Set0_MASK);
			__raw_writel(tmp, drex + DREX_TimingRfcPb);
			__raw_writel(cur_drex_param->drex_TimingRow, drex + DREX_TimingRow_0);
			__raw_writel(cur_drex_param->drex_TimingData, drex + DREX_TimingData_0);
			__raw_writel(cur_drex_param->drex_TimingPower, drex + DREX_TimingPower_0);
			__raw_writel(cur_drex_param->drex_RdFetch, drex + DREX_RdFetch_0);
			__raw_writel((0x0U << 7) | cur_drex_param->drex_EtcControl, drex + DREX_EtcControl_0);
			__raw_writel(cur_drex_param->drex_Train_Timing, drex + DREX_Train_Timing_0);
			__raw_writel(cur_drex_param->drex_Hw_Ptrain_Period, drex + DREX_Hw_Ptrain_Period_0);
			__raw_writel(cur_drex_param->drex_Hwpr_Train_Config, drex + DREX_Hwpr_Train_Config_0);
			__raw_writel(cur_drex_param->drex_Hwpr_Train_Control, drex + DREX_Hwpr_Train_Control_0);

			/* LP4_PHY */
			tmp = __raw_readl(phy + PHY_DVFS_CON0);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con0_set0_mask);
			tmp |= cur_phy_param->phy_Dvfs_Con0_set0;
			__raw_writel(tmp, phy + PHY_DVFS_CON0);

			tmp = __raw_readl(phy + PHY_DVFS_CON1);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con1_set0_mask);
			tmp |= cur_phy_param->phy_Dvfs_Con1_set0;
			__raw_writel(tmp, phy + PHY_DVFS_CON1);

			tmp = __raw_readl(phy + PHY_DVFS_CON2);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con2_set0_mask);
			tmp |= cur_phy_param->phy_Dvfs_Con2_set0;
			__raw_writel(tmp, phy + PHY_DVFS_CON2);

			tmp = __raw_readl(phy + PHY_DVFS_CON3);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con3_set0_mask);
			tmp |= cur_phy_param->phy_Dvfs_Con3_set0;
			__raw_writel(tmp, phy + PHY_DVFS_CON3);

			tmp = __raw_readl(phy + PHY_DVFS_CON4);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con4_set0_mask);
			tmp |= (cur_phy_param->phy_Dvfs_Con4_set0 | (soc_vref[i][1] << 6) | (soc_vref[i][0] << 0));
			__raw_writel(tmp, phy + PHY_DVFS_CON4);

			tmp = __raw_readl(phy + PHY_DVFS_CON5);
			tmp &= ~(cur_phy_param->phy_Dvfs_Con5_set0_mask);
			tmp |= cur_phy_param->phy_Dvfs_Con5_set0;
			__raw_writel(tmp, phy + PHY_DVFS_CON5);

			__raw_writel(cur_vtmon_param->vtmon_Cnt_Limit_set0, vtmon + VTMON_CNT_LIMIT_SET0_SW0);
			__raw_writel(cur_vtmon_param->vtmon_Cnt_Limit_set1, vtmon + VTMON_CNT_LIMIT_SET1_SW0);

			/* DIRECTCMD */
			__raw_writel((0x80U << 2) | cur_drex_param->drex_DirectCmd_mr13, drex + DREX_DIRECTCMD);
			__raw_writel(cur_drex_param->drex_DirectCmd_mr2, drex + DREX_DIRECTCMD);
			__raw_writel(cur_drex_param->drex_DirectCmd_mr22, drex + DREX_DIRECTCMD);
			__raw_writel(cur_drex_param->drex_DirectCmd_mr11, drex + DREX_DIRECTCMD);

			/* VTMON */
			__raw_writel((0x0U << 0) | cur_drex_param->vtmon_Drex_Timing_Set_Sw, vtmon + VTMON_DREX_TIMING_SW_SET);
			__raw_writel((0x0U << 2) | cur_drex_param->drex_DirectCmd_mr13, vtmon + VTMON_MIF_DREX_WDATA0);
		}
	}

	return 0;
}

/* DRAM_POWER_DYNAMIC_SAVE helpers (exynos7420_bus.c lines 4543-4605) */
static void exynos7420_mif_set_dynamic_sref_cycle(int target_idx)
{
	u32 dsref_cyc;
	u32 tmp;
	int i;

	if (target_idx <= MIF_LV4)
		dsref_cyc = 0x3ff;
	else if (target_idx <= MIF_LV7)
		dsref_cyc = 0x1ff;
	else
		dsref_cyc = 0x90;

	/* dsref disable */
	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		tmp = __raw_readl(exynos7420_hw.drex[i] + DREX_MEMCONTROL);
		tmp &= ~DREX_DSREF_EN;
		__raw_writel(tmp, exynos7420_hw.drex[i] + DREX_MEMCONTROL);
	}

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		tmp = __raw_readl(exynos7420_hw.drex[i] + DREX_PWRDNCONFIG);
		tmp = (tmp & ~(0xffffU << 16)) | (dsref_cyc << 16);
		__raw_writel(tmp, exynos7420_hw.drex[i] + DREX_PWRDNCONFIG);
	}

	/* dsref enable */
	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		tmp = __raw_readl(exynos7420_hw.drex[i] + DREX_MEMCONTROL);
		tmp |= DREX_DSREF_EN;
		__raw_writel(tmp, exynos7420_hw.drex[i] + DREX_MEMCONTROL);
	}
}

static void exynos7420_mif_set_pbr_en(bool en)
{
	u32 tmp;
	int i;

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		tmp = __raw_readl(exynos7420_hw.drex[i] + DREX_MEMCONTROL);
		tmp = (tmp & ~DREX_PB_REF_EN) | ((u32)en << 27);
		__raw_writel(tmp, exynos7420_hw.drex[i] + DREX_MEMCONTROL);
	}
}

static void exynos7420_mif_set_refresh_method_pre_dvfs(int target_idx, int old_idx)
{
	if (old_idx < MIF_LV4 && target_idx >= MIF_LV4)
		exynos7420_mif_set_pbr_en(false);
}

static void exynos7420_mif_set_refresh_method_post_dvfs(int target_idx, int old_idx)
{
	if (old_idx >= MIF_LV4 && target_idx < MIF_LV4)
		exynos7420_mif_set_pbr_en(true);
}

/*
 * exynos7_devfreq_mif_set_freq (exynos7420_bus.c lines 4648-4829)
 * Ported 1:1 with DMC_DQS_MODE2 voltage & CCORE sequencing.
 */
static int exynos7420_mif_target_locked(struct exynos_bus *bus,
					unsigned long target_freq)
{
	int old_idx = exynos7420_hw.cur_mif_idx;
	int target_idx = exynos7420_find_mif_idx(target_freq);
	int pll_safe_idx;
	int ret;
	u32 voltage = 0;
	u32 tmp;
	ktime_t t0;

	if (old_idx < 0)
		return -EIO;
	if (target_idx < 0)
		return target_idx;
	if (target_idx == old_idx) {
		bus->curr_freq = target_freq;
		return 0;
	}

	/*
	 * Vendor exynos7_bus_int.c (exynos7420_mif_table_for_int[]) requires
	 * MIF >= 632 MHz for INT 560..550 MHz and MIF >= 543 MHz for INT 540..400 MHz.
	 *
	 * When scaling MIF down:
	 * 1. Check INT's active QoS min_freq to ensure MIF does not drop below
	 *    what INT's floor requires.
	 * 2. Pre-scale INT down before MIF drops, respecting INT's QoS floor.
	 */
	if (target_idx > old_idx && exynos7420_hw.int_bus) {
		unsigned long int_min_hz = 0;
		int int_floor_idx = EXYNOS7420_INT_LV_COUNT - 1;
		int req_mif_by_int, req_int_idx, i;

		if (exynos7420_hw.int_bus->dev) {
			s32 q = dev_pm_qos_read_value(exynos7420_hw.int_bus->dev,
						      DEV_PM_QOS_MIN_FREQUENCY);
			if (q > 0)
				int_min_hz = max(int_min_hz, (unsigned long)q * 1000UL);
		}
		if (exynos7420_hw.int_bus->devfreq &&
		    exynos7420_hw.int_bus->devfreq->scaling_min_freq) {
			int_min_hz = max(int_min_hz,
					 exynos7420_hw.int_bus->devfreq->scaling_min_freq);
		}

		if (int_min_hz > 0) {
			for (i = EXYNOS7420_INT_LV_COUNT - 1; i >= 0; i--) {
				if ((unsigned long)exynos7420_int_table[i].freq_khz * 1000UL >= int_min_hz) {
					int_floor_idx = i;
					break;
				}
			}
		}

		req_mif_by_int = exynos7420_int_required_mif_idx(int_floor_idx);
		if (target_idx > req_mif_by_int) {
			target_idx = req_mif_by_int;
			target_freq = (unsigned long)exynos7420_mif_opp_table[target_idx].freq_khz * 1000UL;
		}

		if (target_idx == old_idx) {
			bus->curr_freq = target_freq;
			return 0;
		}

		req_int_idx = exynos7420_mif_to_int_idx[target_idx];
		if (req_int_idx > int_floor_idx)
			req_int_idx = int_floor_idx;

		if (exynos7420_hw.cur_int_idx < req_int_idx) {
			unsigned long int_hz =
				(unsigned long)exynos7420_int_table[req_int_idx].freq_khz * 1000UL;
			ret = exynos7420_int_target_locked(exynos7420_hw.int_bus, int_hz);
			if (ret)
				return ret;
		}
	}

	t0 = ktime_get();

	ret = exynos7420_mif_check_mux_ctrl_by_cmuc();
	if (ret)
		return ret;

	exynos7420_mif_set_refresh_method_pre_dvfs(target_idx, old_idx);

	if (old_idx <= MIF_LV4 && target_idx <= MIF_LV4) {
		ret = exynos7420_set_dout_sclk_bus0_pll_mif(0); /* 1600 MHz (/1) */
		if (ret)
			goto err_pre_switch;
		exynos7420_hw.pll_safe_idx = MIF_LV4;
	} else {
		ret = exynos7420_set_dout_sclk_bus0_pll_mif(1); /* 800 MHz (/2) */
		if (ret)
			goto err_pre_switch;
		exynos7420_hw.pll_safe_idx = MIF_LV7;
	}

	pll_safe_idx = exynos7420_hw.pll_safe_idx;

	/* 1st voltage stage: Change volt for switching PLL if needed */
	if (old_idx <= MIF_LV4 && target_idx <= MIF_LV4) {
		voltage = 0;
	} else {
		if (pll_safe_idx <= old_idx)
			voltage = exynos7420_mif_opp_table[pll_safe_idx].volt_uv +
				  EXYNOS7420_CMOS_SE_OFFSET_UV;
		else if (target_idx < old_idx && MIF_LV4 <= old_idx)
			voltage = exynos7420_mif_opp_table[target_idx].volt_uv;
		else
			voltage = 0;
	}

	if (voltage) {
		ret = exynos7420_mif_set_volt(voltage);
		if (ret)
			goto err_pre_switch;
	}

	/* Disable VT MON DQS OSC compensation and wait for completion */
	ret = exynos7420_mif_disable_vt_mon();
	if (ret)
		goto err_pre_switch;

	/* Change CCORE clock for switching PLL */
	if (pll_safe_idx >= old_idx && pll_safe_idx >= target_idx) {
		ret = exynos7420_ccore_set_freq(pll_safe_idx);
		if (ret)
			goto err_vt_mon;
	} else if (old_idx <= target_idx) {
		ret = exynos7420_ccore_set_freq(target_idx);
		if (ret)
			goto err_vt_mon;
	}

	/* Apply timing set 1 for switching PLL */
	exynos7420_mif_preset(pll_safe_idx, 1);

	/* clk enable for switching PLL (sclk_bus0_pll_mif = ENABLE_SCLK_TOPC1 bit 6) */
	tmp = __raw_readl(exynos7420_hw.topc + TOPC_ENABLE_SCLK1);
	tmp |= BIT(6);
	__raw_writel(tmp, exynos7420_hw.topc + TOPC_ENABLE_SCLK1);

	/* sclk_hpm_mif dfs for cl-dvfs */
	ret = exynos7420_set_clkdiv_sclk_hpm(pll_safe_idx);
	if (ret)
		goto err_sclk;

	/* Change to switching PLL (sclk_bus0_pll_mif: MUX_SEL_TOPC5 bit 20 = 0), wait pause & mux */
	tmp = __raw_readl(exynos7420_hw.topc + TOPC_MUX_SEL5);
	tmp &= ~BIT(20);
	__raw_writel(tmp, exynos7420_hw.topc + TOPC_MUX_SEL5);
	ret = exynos7420_mif_waiting_pause();
	if (ret)
		goto err_switching_mux;
	ret = exynos7420_mif_waiting_mux();
	if (ret)
		goto err_switching_mux;

	/* 2nd voltage stage (DMC_DQS_MODE2): set voltage prior to target freq for gate re-training */
	if (old_idx <= MIF_LV4 && target_idx <= MIF_LV4) {
		voltage = exynos7420_mif_opp_table[target_idx].volt_uv;
	} else {
		if (pll_safe_idx <= old_idx && target_idx < pll_safe_idx)
			voltage = exynos7420_mif_opp_table[target_idx].volt_uv;
		else if (old_idx <= MIF_LV3 && target_idx <= MIF_LV3)
			voltage = exynos7420_mif_opp_table[target_idx].volt_uv;
		else if (old_idx < pll_safe_idx && pll_safe_idx <= target_idx)
			voltage = exynos7420_mif_opp_table[pll_safe_idx].volt_uv +
				  EXYNOS7420_CMOS_SE_OFFSET_UV;
		else
			voltage = 0;
	}

	if (voltage) {
		ret = exynos7420_mif_set_volt(voltage);
		if (ret)
			goto err_on_switching_pll;
	}

	/* Change mif_pll rate for target freq */
	ret = exynos7420_mif_pll_set_rate(target_idx);
	if (ret)
		goto err_on_switching_pll;

	/* Apply timing set 0 for target freq */
	exynos7420_mif_preset(target_idx, 0);

	/* sclk_hpm_mif dfs for cl-dvfs */
	ret = exynos7420_set_clkdiv_sclk_hpm(target_idx);
	if (ret)
		goto err_on_switching_pll;

	/* Change to target PLL (mif_pll: MUX_SEL_TOPC5 bit 20 = 1), wait pause & mux */
	tmp = __raw_readl(exynos7420_hw.topc + TOPC_MUX_SEL5);
	tmp |= BIT(20);
	__raw_writel(tmp, exynos7420_hw.topc + TOPC_MUX_SEL5);
	ret = exynos7420_mif_waiting_pause();
	if (ret)
		goto err_target_mux;
	ret = exynos7420_mif_waiting_mux();
	if (ret)
		goto err_target_mux;

	/* clk disable for switching PLL (sclk_bus0_pll_mif = ENABLE_SCLK_TOPC1 bit 6) */
	tmp = __raw_readl(exynos7420_hw.topc + TOPC_ENABLE_SCLK1);
	tmp &= ~BIT(6);
	__raw_writel(tmp, exynos7420_hw.topc + TOPC_ENABLE_SCLK1);

	/* Change CCORE clock for target */
	if (pll_safe_idx >= old_idx && pll_safe_idx >= target_idx) {
		ret = exynos7420_ccore_set_freq(target_idx);
		if (ret)
			goto err_committed;
	} else if (old_idx <= target_idx) {
		;
	} else {
		ret = exynos7420_ccore_set_freq(target_idx);
		if (ret)
			goto err_committed;
	}

	/* Restore VT MON DQS OSC compensation and wait for completion */
	ret = exynos7420_mif_restore_vt_mon();
	if (ret)
		goto err_committed;

	/* 3rd voltage stage: Change volt for target if needed */
	if (old_idx <= MIF_LV4 && target_idx <= MIF_LV4) {
		voltage = 0;
	} else {
		if (pll_safe_idx <= old_idx && pll_safe_idx < target_idx)
			voltage = exynos7420_mif_opp_table[target_idx].volt_uv;
		else if (old_idx < target_idx && old_idx < pll_safe_idx && MIF_LV4 <= target_idx)
			voltage = exynos7420_mif_opp_table[target_idx].volt_uv;
		else
			voltage = 0;
	}

	if (voltage) {
		ret = exynos7420_mif_set_volt(voltage);
		if (ret)
			goto err_committed;
	}

	exynos7420_mif_set_refresh_method_post_dvfs(target_idx, old_idx);
	exynos7420_mif_set_dynamic_sref_cycle(target_idx);

	exynos7420_hw.cur_mif_idx = target_idx;
	exynos7420_hw.mif_switch_count++;
	exynos7420_hw.last_mif_switch_us = ktime_us_delta(ktime_get(), t0);
	bus->curr_freq = target_freq;
	return 0;

err_committed:
	/*
	 * Hardware (PLL, DREX timings, MUX) is already committed to target_idx!
	 * Update state cache so subsequent calls know hardware is at target_idx
	 * and will not incorrectly assume old_idx.
	 */
	exynos7420_hw.cur_mif_idx = target_idx;
	bus->curr_freq = target_freq;
	return ret;

err_on_switching_pll:
err_target_mux:
	exynos7420_mif_restore_vt_mon();
	/* Clock / PLL state is unknown/in-between; mark unknown */
	exynos7420_hw.cur_mif_idx = -1;
	bus->curr_freq = 0;
	return ret;

err_sclk:
	tmp = __raw_readl(exynos7420_hw.topc + TOPC_ENABLE_SCLK1);
	tmp &= ~BIT(6);
	__raw_writel(tmp, exynos7420_hw.topc + TOPC_ENABLE_SCLK1);
err_switching_mux:
	/* If we failed during switching MUX, try to ensure VT_MON is restored */
err_vt_mon:
	exynos7420_mif_restore_vt_mon();
err_pre_switch:
	if (old_idx < MIF_LV4 && target_idx >= MIF_LV4)
		exynos7420_mif_set_pbr_en(true);
	return ret;
}

static ssize_t hw_status_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	u32 con0, sel5, stat5, div3, sel2, sel3, div0, div1, top0_sel3, top0_div3;
	u32 p, m, s, pll_khz = 0, logic_khz = 0;
	int mif_uv = 0, int_uv = 0, i;

	if (!exynos7420_hw.initialized)
		return scnprintf(buf, PAGE_SIZE, "uninitialized\n");

	mutex_lock(&exynos7420_hw.lock);
	con0 = readl(exynos7420_hw.topc + TOPC_MIF_PLL_CON0);
	sel5 = readl(exynos7420_hw.topc + TOPC_MUX_SEL5);
	stat5 = readl(exynos7420_hw.topc + TOPC_MUX_STAT5);
	div3 = readl(exynos7420_hw.topc + TOPC_DIV3);
	sel2 = readl(exynos7420_hw.topc + TOPC_MUX_SEL2);
	sel3 = readl(exynos7420_hw.topc + TOPC_MUX_SEL3);
	div0 = readl(exynos7420_hw.topc + TOPC_DIV0);
	div1 = readl(exynos7420_hw.topc + TOPC_DIV1);
	top0_sel3 = readl(exynos7420_hw.top0 + TOP0_MUX_SEL3);
	top0_div3 = readl(exynos7420_hw.top0 + TOP0_DIV3);

	m = (con0 >> 16) & 0x3ff;
	p = (con0 >> 8) & 0x3f;
	s = con0 & 0x7;
	if (p) {
		pll_khz = (24000U * m) / (p << s);
		logic_khz = pll_khz / 2;
	}
	if (exynos7420_hw.vdd_mif)
		mif_uv = regulator_get_voltage(exynos7420_hw.vdd_mif);
	if (exynos7420_hw.vdd_int)
		int_uv = regulator_get_voltage(exynos7420_hw.vdd_int);

	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "MIF: lv=%d freq=%u kHz (MIF_PLL=%u kHz, M=%u P=%u S=%u) volt=%d uV switches=%lu last_us=%lld\n"
			 "  CMU_TOPC: MIF_PLL_CON0=0x%08x MUX_SEL5=0x%08x MUX_STAT5=0x%08x DIV3=0x%08x\n",
			 exynos7420_hw.cur_mif_idx, logic_khz, pll_khz, m, p, s,
			 mif_uv, exynos7420_hw.mif_switch_count,
			 (long long)exynos7420_hw.last_mif_switch_us,
			 con0, sel5, stat5, div3);

	for (i = 0; i < EXYNOS7420_NUM_MIF_CH; i++) {
		void __iomem *cmu = exynos7420_hw.cmu_mif[i];
		void __iomem *drex = exynos7420_hw.drex[i];
		void __iomem *phy = exynos7420_hw.phy[i];
		void __iomem *vtmon = exynos7420_hw.vtmon[i];

		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "  CH%d: MIF_DIV0=0x%08x PAUSE=0x%08x MEMCTRL=0x%08x PWRDN=0x%08x RFCPB=0x%08x TROW0=0x%08x TDATA0=0x%08x TPWR0=0x%08x PHY_CON0=0x%08x CON1=0x%08x CON3=0x%08x VTMON_SW=0x%08x\n",
				 i,
				 readl(cmu + MIF_DIV0),
				 readl(cmu + MIF_PAUSE),
				 readl(drex + DREX_MEMCONTROL),
				 readl(drex + DREX_PWRDNCONFIG),
				 readl(drex + DREX_TimingRfcPb),
				 readl(drex + DREX_TimingRow_0),
				 readl(drex + DREX_TimingData_0),
				 readl(drex + DREX_TimingPower_0),
				 readl(phy + PHY_DVFS_CON0),
				 readl(phy + PHY_DVFS_CON1),
				 readl(phy + PHY_DVFS_CON3),
				 readl(vtmon + VTMON_DREX_TIMING_SW_SET));
	}

	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "INT: lv=%d freq=%u kHz volt=%d uV switches=%lu last_us=%lld\n"
			 "  CMU_TOPC: MUX_SEL2=0x%08x MUX_SEL3=0x%08x DIV0=0x%08x DIV1=0x%08x\n"
			 "  CMU_TOP0: MUX_SEL3=0x%08x DIV3=0x%08x EMA_INT=0x%08x\n",
			 exynos7420_hw.cur_int_idx,
			 exynos7420_int_table[exynos7420_hw.cur_int_idx].freq_khz,
			 int_uv, exynos7420_hw.int_switch_count,
			 (long long)exynos7420_hw.last_int_switch_us,
			 sel2, sel3, div0, div1, top0_sel3, top0_div3,
			 readl(exynos7420_hw.sysreg + SYSREG_EMA_INT_CON));
	mutex_unlock(&exynos7420_hw.lock);

	return len;
}
static DEVICE_ATTR_RO(hw_status);

#endif /* __EXYNOS7420_INT_MIF_H */
