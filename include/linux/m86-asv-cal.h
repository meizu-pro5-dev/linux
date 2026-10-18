/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung Exynos 7420 Common ASV (Adaptive Supply Voltage) & EMA/MCS CAL
 * Ported 1:1 from vendor arch/arm64/mach-exynos/asv-exynos7420_cal.c and
 * include/mach/asv-exynos7420.h (table_version 15, 16 ASV groups 0..15).
 */
#ifndef __LINUX_M86_ASV_CAL_H
#define __LINUX_M86_ASV_CAL_H

#include <linux/arm-smccc.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <asm/barrier.h>

#define M86_ASV_BASE		0x101e0160U
#define M86_ASV_WORDS		16
#define M86_ASV_GRP_NR		16
#define M86_SMC_CMD_REG		((u32)-101)
#define M86_SMC_SFR_READ	(3U << 30)

enum m86_asv_domain {
	M86_ASV_BIG = 0,
	M86_ASV_LIT = 1,
	M86_ASV_G3D = 2,
	M86_ASV_MIF = 3,
	M86_ASV_INT = 4,
	M86_ASV_CAM = 5,
};

struct m86_asv_cal_info {
	u32 words[M86_ASV_WORDS];
	u32 table_ver;
	u32 effective_ver;
	bool asv_group_type;
	bool shift_type;
	bool ssa1_enable;
	bool ssa0_enable;
	bool asv_method;
	u32 main_group;
	u32 main_ssa;
	bool main_ssa_minus;
};

/*
 * volt_table_big_asv_v15 (asv-exynos7420.h:1420-1446)
 * Levels L4 (2100 MHz) .. L17 (800 MHz) used by m86 Atlas cluster.
 * Index 0 in this array corresponds tovendor SYSC_DVFS_L4 (2100 MHz).
 */
static const u32 m86_big_asv_volt_v15[14][M86_ASV_GRP_NR] = {
	/* L4:  2100 MHz */ { 1143750, 1137500, 1131250, 1125000, 1112500, 1100000, 1087500, 1075000, 1062500, 1050000, 1037500, 1025000, 1018750, 1006250, 1000000,  987500 },
	/* L5:  2000 MHz */ { 1106250, 1100000, 1093750, 1087500, 1075000, 1062500, 1050000, 1037500, 1025000, 1012500, 1000000,  987500,  981250,  968750,  962500,  950000 },
	/* L6:  1896 MHz */ { 1068750, 1062500, 1056250, 1050000, 1037500, 1025000, 1012500, 1000000,  987500,  975000,  962500,  950000,  943750,  931250,  925000,  912500 },
	/* L7:  1800 MHz */ { 1031250, 1025000, 1018750, 1012500, 1000000,  987500,  975000,  962500,  950000,  937500,  925000,  912500,  906250,  893750,  887500,  875000 },
	/* L8:  1704 MHz */ { 1000000,  993750,  987500,  981250,  968750,  956250,  943750,  931250,  918750,  906250,  893750,  881250,  875000,  862500,  856250,  843750 },
	/* L9:  1600 MHz */ {  968750,  962500,  956250,  950000,  937500,  925000,  912500,  900000,  887500,  875000,  862500,  850000,  843750,  831250,  825000,  812500 },
	/* L10: 1500 MHz */ {  937500,  931250,  925000,  918750,  906250,  893750,  881250,  868750,  856250,  843750,  831250,  818750,  812500,  806250,  800000,  793750 },
	/* L11: 1400 MHz */ {  912500,  906250,  900000,  893750,  881250,  868750,  856250,  843750,  831250,  818750,  806250,  793750,  787500,  781250,  775000,  768750 },
	/* L12: 1300 MHz */ {  887500,  881250,  875000,  868750,  856250,  843750,  831250,  818750,  806250,  793750,  781250,  768750,  762500,  756250,  750000,  743750 },
	/* L13: 1200 MHz */ {  868750,  862500,  856250,  850000,  837500,  825000,  812500,  800000,  787500,  775000,  762500,  750000,  743750,  737500,  731250,  725000 },
	/* L14: 1100 MHz */ {  843750,  837500,  831250,  825000,  812500,  800000,  787500,  775000,  762500,  750000,  737500,  725000,  718750,  712500,  706250,  700000 },
	/* L15: 1000 MHz */ {  818750,  812500,  806250,  800000,  787500,  775000,  762500,  750000,  737500,  725000,  712500,  700000,  700000,  700000,  700000,  700000 },
	/* L16:  900 MHz */ {  800000,  793750,  787500,  781250,  768750,  756250,  743750,  731250,  718750,  706250,  693750,  681250,  675000,  668750,  662500,  656250 },
	/* L17:  800 MHz */ {  781250,  775000,  768750,  762500,  750000,  737500,  725000,  712500,  700000,  687500,  675000,  662500,  650000,  643750,  637500,  625000 },
};

/*
 * volt_table_lit_asv_v15 (asv-exynos7420.h:1448-1469)
 * Levels L5 (1500 MHz) .. L16 (400 MHz) used by m86 Apollo cluster.
 * Index 0 in this array corresponds to vendor SYSC_DVFS_L5 (1500 MHz).
 */
static const u32 m86_lit_asv_volt_v15[12][M86_ASV_GRP_NR] = {
	/* L5:  1500 MHz */ { 1162500, 1150000, 1137500, 1125000, 1112500, 1100000, 1087500, 1075000, 1062500, 1050000, 1037500, 1025000, 1018750, 1012500, 1006250, 1000000 },
	/* L6:  1400 MHz */ { 1112500, 1100000, 1087500, 1075000, 1062500, 1050000, 1037500, 1025000, 1012500, 1000000,  987500,  975000,  968750,  962500,  956250,  950000 },
	/* L7:  1296 MHz */ { 1062500, 1050000, 1037500, 1025000, 1012500, 1000000,  987500,  975000,  962500,  950000,  937500,  925000,  918750,  912500,  906250,  900000 },
	/* L8:  1200 MHz */ { 1025000, 1012500, 1000000,  987500,  975000,  962500,  950000,  937500,  925000,  912500,  900000,  887500,  881250,  875000,  868750,  862500 },
	/* L9:  1104 MHz */ {  987500,  975000,  962500,  950000,  937500,  925000,  912500,  900000,  887500,  875000,  862500,  850000,  843750,  837500,  831250,  825000 },
	/* L10: 1000 MHz */ {  950000,  937500,  925000,  912500,  900000,  887500,  875000,  862500,  850000,  837500,  825000,  812500,  806250,  800000,  793750,  787500 },
	/* L11:  900 MHz */ {  912500,  900000,  887500,  875000,  862500,  850000,  837500,  825000,  812500,  800000,  787500,  775000,  768750,  762500,  756250,  750000 },
	/* L12:  800 MHz */ {  875000,  862500,  850000,  837500,  825000,  812500,  800000,  787500,  775000,  762500,  750000,  737500,  731250,  718750,  712500,  700000 },
	/* L13:  700 MHz */ {  843750,  831250,  818750,  806250,  793750,  781250,  768750,  756250,  743750,  731250,  718750,  706250,  700000,  687500,  681250,  668750 },
	/* L14:  600 MHz */ {  818750,  806250,  793750,  781250,  768750,  756250,  743750,  731250,  718750,  706250,  693750,  681250,  675000,  662500,  656250,  643750 },
	/* L15:  500 MHz */ {  793750,  781250,  768750,  756250,  743750,  731250,  718750,  706250,  693750,  681250,  668750,  656250,  643750,  631250,  625000,  612500 },
	/* L16:  400 MHz */ {  768750,  756250,  743750,  731250,  718750,  706250,  693750,  681250,  668750,  656250,  643750,  631250,  625000,  618750,  618750,  606250 },
};

/*
 * volt_table_g3d_asv_v15 (asv-exynos7420.h:1471-1483)
 * Levels L1 (772 MHz) .. L7 (266 MHz).
 */
static const u32 m86_g3d_asv_volt_v15[7][M86_ASV_GRP_NR] = {
	/* L1: 772 MHz */ { 931250, 925000, 918750, 912500, 900000, 887500, 875000, 862500, 850000, 837500, 825000, 812500, 806250, 800000, 793750, 787500 },
	/* L2: 700 MHz */ { 893750, 887500, 881250, 875000, 862500, 850000, 837500, 825000, 812500, 800000, 787500, 775000, 768750, 762500, 756250, 750000 },
	/* L3: 600 MHz */ { 850000, 843750, 837500, 831250, 818750, 806250, 793750, 781250, 768750, 756250, 743750, 731250, 725000, 718750, 712500, 706250 },
	/* L4: 544 MHz */ { 812500, 806250, 800000, 793750, 781250, 775000, 756250, 750000, 731250, 725000, 706250, 700000, 693750, 681250, 662500, 662500 },
	/* L5: 420 MHz */ { 775000, 768750, 762500, 756250, 743750, 731250, 718750, 706250, 693750, 681250, 668750, 656250, 650000, 643750, 625000, 618750 },
	/* L6: 350 MHz */ { 768750, 762500, 756250, 750000, 737500, 725000, 712500, 700000, 687500, 675000, 662500, 650000, 643750, 637500, 618750, 612500 },
	/* L7: 266 MHz */ { 762500, 756250, 750000, 743750, 731250, 718750, 706250, 693750, 681250, 668750, 656250, 643750, 637500, 631250, 612500, 606250 },
};

/*
 * volt_table_mif_asv_v15 (asv-exynos7420.h:1485-1500)
 * Levels L0 (1552 MHz) .. L12 (100 MHz).
 */
static const u32 m86_mif_asv_volt_v15[13][M86_ASV_GRP_NR] = {
	/* L0:  1552 MHz */ { 937500, 925000, 912500, 900000, 887500, 875000, 862500, 850000, 837500, 825000, 812500, 800000, 793750, 787500, 781250, 775000 },
	/* L1:  1456 MHz */ { 900000, 887500, 875000, 862500, 850000, 837500, 825000, 812500, 800000, 787500, 775000, 762500, 756250, 750000, 743750, 737500 },
	/* L2:  1264 MHz */ { 868750, 856250, 843750, 831250, 818750, 806250, 793750, 781250, 768750, 756250, 743750, 731250, 725000, 718750, 712500, 706250 },
	/* L3:  1026 MHz */ { 831250, 818750, 806250, 793750, 781250, 768750, 768750, 756250, 743750, 731250, 718750, 706250, 700000, 693750, 687500, 682250 },
	/* L4:   828 MHz */ { 812500, 800000, 787500, 775000, 762500, 750000, 750000, 737500, 725000, 712500, 700000, 687500, 687500, 687500, 687500, 682250 },
	/* L5:   632 MHz */ { 781250, 768750, 756250, 743750, 731250, 718750, 706250, 693750, 681250, 668750, 656250, 643750, 637500, 631250, 625000, 625000 },
	/* L6:   543 MHz */ { 768750, 756250, 743750, 731250, 718750, 706250, 693750, 681250, 668750, 656250, 643750, 631250, 625000, 625000, 625000, 625000 },
	/* L7:   416 MHz */ { 700000, 687500, 675000, 662500, 650000, 637500, 625000, 612500, 600000, 587500, 575000, 575000, 575000, 575000, 575000, 575000 },
	/* L8:   348 MHz */ { 700000, 687500, 675000, 662500, 650000, 637500, 625000, 612500, 600000, 587500, 575000, 575000, 575000, 575000, 575000, 575000 },
	/* L9:   276 MHz */ { 700000, 687500, 675000, 662500, 650000, 637500, 625000, 612500, 600000, 587500, 575000, 575000, 575000, 575000, 575000, 575000 },
	/* L10:  167 MHz */ { 700000, 687500, 675000, 662500, 650000, 637500, 625000, 612500, 600000, 587500, 575000, 575000, 575000, 575000, 575000, 575000 },
	/* L11:  133 MHz */ { 700000, 687500, 675000, 662500, 650000, 637500, 625000, 612500, 600000, 587500, 575000, 575000, 575000, 575000, 575000, 575000 },
	/* L12:  100 MHz */ { 700000, 687500, 675000, 662500, 650000, 637500, 625000, 612500, 600000, 587500, 575000, 575000, 575000, 575000, 575000, 575000 },
};

/*
 * volt_table_int_asv_v15 (asv-exynos7420.h:1502-1516)
 * Levels L0 (560 MHz) .. L11 (100 MHz).
 */
static const u32 m86_int_asv_volt_v15[12][M86_ASV_GRP_NR] = {
	/* L0:  560 MHz */ { 906250, 893750, 881250, 868750, 856250, 843750, 831250, 818750, 806250, 793750, 781250, 768750, 762500, 756250, 750000, 743750 },
	/* L1:  550 MHz */ { 906250, 893750, 881250, 868750, 856250, 843750, 831250, 818750, 806250, 793750, 781250, 768750, 762500, 756250, 750000, 743750 },
	/* L2:  540 MHz */ { 850000, 837500, 825000, 812500, 800000, 787500, 775000, 762500, 750000, 737500, 725000, 712500, 706250, 700000, 693750, 687500 },
	/* L3:  530 MHz */ { 850000, 837500, 825000, 812500, 800000, 787500, 775000, 762500, 750000, 737500, 725000, 712500, 706250, 700000, 693750, 687500 },
	/* L4:  520 MHz */ { 850000, 837500, 825000, 812500, 800000, 787500, 775000, 762500, 750000, 737500, 725000, 712500, 706250, 700000, 693750, 687500 },
	/* L5:  510 MHz */ { 850000, 837500, 825000, 812500, 800000, 787500, 775000, 762500, 750000, 737500, 725000, 712500, 706250, 700000, 693750, 687500 },
	/* L6:  500 MHz */ { 850000, 837500, 825000, 812500, 800000, 787500, 775000, 762500, 750000, 737500, 725000, 712500, 706250, 700000, 693750, 687500 },
	/* L7:  400 MHz */ { 825000, 812500, 800000, 787500, 775000, 762500, 750000, 737500, 725000, 712500, 700000, 687500, 681250, 675000, 668750, 662500 },
	/* L8:  334 MHz */ { 787500, 775000, 762500, 750000, 737500, 725000, 712500, 700000, 687500, 675000, 662500, 650000, 650000, 643750, 637500, 631250 },
	/* L9:  266 MHz */ { 750000, 737500, 725000, 712500, 700000, 687500, 675000, 662500, 650000, 650000, 650000, 650000, 650000, 643750, 637500, 612500 },
	/* L10: 200 MHz */ { 750000, 737500, 725000, 712500, 700000, 687500, 675000, 662500, 650000, 650000, 650000, 650000, 650000, 643750, 637500, 612500 },
	/* L11: 100 MHz */ { 750000, 737500, 725000, 712500, 700000, 687500, 675000, 662500, 650000, 650000, 650000, 650000, 650000, 643750, 637500, 612500 },
};

static inline int m86_asv_cal_read(struct m86_asv_cal_info *info)
{
	unsigned int i;

	for (i = 0; i < M86_ASV_WORDS; i++) {
		struct arm_smccc_res res = { 0 };
		u32 addr = M86_ASV_BASE + 4U * i;

		dsb(sy);
		arm_smccc_smc((unsigned long)M86_SMC_CMD_REG,
			      (unsigned long)(M86_SMC_SFR_READ | (addr >> 2)),
			      0, 0, 0, 0, 0, 0, &res);
		if (res.a0)
			return -EIO;
		info->words[i] = (u32)res.a2;
	}

	info->table_ver = info->words[3] & 0xfU;
	info->effective_ver = (info->table_ver > 11 && info->table_ver < 15) ?
			      15U : info->table_ver;
	info->asv_group_type = (info->words[3] >> 4) & 0x1U;
	info->shift_type = (info->words[3] >> 8) & 0x1U;
	info->ssa1_enable = (info->words[3] >> 9) & 0x1U;
	info->ssa0_enable = (info->words[3] >> 10) & 0x1U;
	info->asv_method = (info->words[3] >> 16) & 0x1U;

	info->main_group = info->words[4] & 0xfU;
	info->main_ssa = (info->words[4] >> 4) & 0x7U;
	info->main_ssa_minus = (info->words[4] >> 7) & 0x1U;

	if (!info->asv_method || info->effective_ver != 15)
		return -EOPNOTSUPP;

	return 0;
}

/*
 * Vendor cal_get_match_subgrp() (asv-exynos7420_cal.c:318-407)
 * @vendor_lv is the SYSC_DVFS_L* index in the vendor table:
 * - BIG: L4(2100)..L17(800) -> <= L9(1600): subgrp 0; <= L15(1000): subgrp 1; else 2
 * - LIT: L5(1500)..L16(400) -> <= L9(1104): subgrp 0; <= L13(700):  subgrp 1; else 2
 * - G3D: L1(772)..L7(266)   -> <= L1(772):  subgrp 0; <= L3(600):   subgrp 1; else 2
 * - MIF: L0(1552)..L12(100) -> <= L1(1456): subgrp 0; <= L4(828):   subgrp 1; else 2
 * - INT: L0(560)..L11(100)  -> == L0(560):  subgrp 0; == L1(550):   subgrp 1; else 2
 */
static inline u32 m86_asv_match_subgrp(enum m86_asv_domain dom, u32 vendor_lv)
{
	switch (dom) {
	case M86_ASV_BIG:
		return (vendor_lv <= 9) ? 0 : (vendor_lv <= 15) ? 1 : 2;
	case M86_ASV_LIT:
		return (vendor_lv <= 9) ? 0 : (vendor_lv <= 13) ? 1 : 2;
	case M86_ASV_G3D:
		return (vendor_lv <= 1) ? 0 : (vendor_lv <= 3) ? 1 : 2;
	case M86_ASV_MIF:
		return (vendor_lv <= 1) ? 0 : (vendor_lv <= 4) ? 1 : 2;
	case M86_ASV_INT:
		return (vendor_lv == 0) ? 0 : (vendor_lv == 1) ? 1 : 2;
	default:
		return 0;
	}
}

static inline u32 m86_asv_domain_group_raw(const struct m86_asv_cal_info *info,
					   enum m86_asv_domain dom)
{
	switch (dom) {
	case M86_ASV_BIG: return info->words[0] & 0xfffU;
	case M86_ASV_LIT: return (info->words[0] >> 16) & 0xfffU;
	case M86_ASV_G3D: return info->words[1] & 0xfffU;
	case M86_ASV_MIF: return (info->words[1] >> 16) & 0xfffU;
	case M86_ASV_INT: return info->words[2] & 0xfffU;
	case M86_ASV_CAM: return (info->words[2] >> 16) & 0xfffU;
	default:          return 0;
	}
}

static inline u32 m86_asv_domain_ssa0(const struct m86_asv_cal_info *info,
				      enum m86_asv_domain dom)
{
	switch (dom) {
	case M86_ASV_BIG: return (info->words[0] >> 12) & 0xfU;
	case M86_ASV_LIT: return (info->words[0] >> 28) & 0xfU;
	case M86_ASV_G3D: return (info->words[1] >> 12) & 0xfU;
	case M86_ASV_MIF: return (info->words[1] >> 28) & 0xfU;
	case M86_ASV_INT: return (info->words[2] >> 12) & 0xfU;
	case M86_ASV_CAM: return (info->words[2] >> 28) & 0xfU;
	default:          return 0;
	}
}

static inline u32 m86_asv_domain_ssa1(const struct m86_asv_cal_info *info,
				      enum m86_asv_domain dom)
{
	switch (dom) {
	case M86_ASV_BIG: return (info->words[4] >> 8) & 0xfU;
	case M86_ASV_LIT: return (info->words[4] >> 12) & 0xfU;
	case M86_ASV_G3D: return (info->words[4] >> 16) & 0xfU;
	case M86_ASV_MIF: return (info->words[4] >> 20) & 0xfU;
	case M86_ASV_INT: return (info->words[4] >> 24) & 0xfU;
	case M86_ASV_CAM: return (info->words[4] >> 28) & 0xfU;
	default:          return 0;
	}
}

/*
 * Vendor cal_get_asv_grp() + cal_get_volt() (asv-exynos7420_cal.c:490-678)
 */
static inline u32 m86_asv_lookup_volt(const struct m86_asv_cal_info *info,
				      enum m86_asv_domain dom,
				      u32 vendor_lv,
				      const u32 *row_16)
{
	u32 subgrp = m86_asv_match_subgrp(dom, vendor_lv);
	int grp;
	u32 volt;

	if (info->asv_group_type) {
		grp = (int)info->main_group;
		if (info->main_ssa_minus)
			grp -= (int)info->main_ssa;
		else
			grp += (int)info->main_ssa;
		if (grp < 0)
			grp = 0;
		if (grp >= M86_ASV_GRP_NR)
			grp = M86_ASV_GRP_NR - 1;
	} else {
		u32 raw = m86_asv_domain_group_raw(info, dom);

		grp = (raw >> (subgrp * 4U)) & 0xfU;
	}

	volt = row_16[grp];

	if (!info->asv_group_type) {
		u32 ssa_volt = 0;

		if (subgrp < 2) {
			if (info->ssa1_enable) {
				u32 ssa1 = m86_asv_domain_ssa1(info, dom);
				u32 add = (subgrp == 0) ?
					  (12500U + (ssa1 & 0x3U) * 12500U) :
					  (12500U + ((ssa1 >> 2) & 0x3U) * 12500U);
				ssa_volt = volt + add;
			}
		} else {
			if (info->ssa0_enable) {
				u32 ssa0 = m86_asv_domain_ssa0(info, dom);

				if (ssa0 != 0)
					ssa_volt = 550000U + ssa0 * 25000U;
			}
		}
		if (ssa_volt > volt)
			volt = ssa_volt;
	}

	return volt;
}

/*
 * Vendor cal_set_ema() (asv-exynos7420_cal.c:868-1027)
 */
static inline void m86_asv_get_atlas_ema(const struct m86_asv_cal_info *info,
					 u32 uv, u32 *l1_out, u32 *l2_out)
{
	u32 w14 = info->words[14];
	u32 w15 = info->words[15];
	u32 l1, l2;

	if (uv >= 1000000U) {
		l1 = (w14 >> 16) & 0xfU;
		l2 = (w15 >> 0) & 0xfU;
		if (!l1) l1 = 6U;
		if (!l2) l2 = 5U;
	} else if (uv >= 900000U) {
		l1 = (w14 >> 20) & 0xfU;
		l2 = (w15 >> 4) & 0xfU;
		if (!l1) l1 = 6U;
		if (!l2) l2 = 5U;
	} else if (uv >= 800000U) {
		l1 = (w14 >> 24) & 0xfU;
		l2 = (w15 >> 8) & 0xfU;
		if (!l1) l1 = 8U;
		if (!l2) l2 = 11U;
	} else {
		l1 = (w14 >> 28) & 0xfU;
		l2 = (w15 >> 12) & 0xfU;
		if (!l1) l1 = 7U;
		if (!l2) l2 = 9U;
	}
	*l1_out = l1;
	*l2_out = l2;
}

static inline u32 m86_asv_get_apollo_ema(const struct m86_asv_cal_info *info,
					 u32 uv)
{
	u32 w5 = info->words[5];
	u32 val;

	if (uv >= 1000000U)
		val = (w5 >> 9) & 0x7U;
	else if (uv >= 900000U)
		val = (w5 >> 12) & 0x7U;
	else if (uv >= 800000U)
		val = (w5 >> 15) & 0x7U;
	else
		val = (w5 >> 18) & 0x7U;

	return val ? val : 2U;
}

static inline u32 m86_asv_get_g3d_mcs_reg(const struct m86_asv_cal_info *info,
					  u32 uv)
{
	u32 w5 = info->words[5];
	u32 val;

	if (uv >= 900000U) {
		val = (w5 >> 21) & 0x3U;
		if (!val) val = 1U;
	} else if (uv >= 800000U) {
		val = (w5 >> 23) & 0x3U;
		if (!val) val = 1U;
	} else if (uv >= 700000U) {
		val = (w5 >> 25) & 0x3U;
		if (!val) val = 1U;
	} else {
		val = (w5 >> 27) & 0x3U;
		if (!val) val = 3U;
	}

	return (0x11U & ~(0x3U << 4)) | (val << 4);
}

static inline u32 m86_asv_get_int_ema_reg(const struct m86_asv_cal_info *info)
{
	u32 val = (info->words[15] >> 16) & 0x7U;

	if (!val)
		val = 2U;
	return (0x12U & ~0x7U) | val;
}

#endif /* __LINUX_M86_ASV_CAL_H */
