// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos7420 INT/DISP/SRAM (BUCK4/BUCK5/LDO8) voltage coupler for Meizu PRO 5.
 *
 * Ensures LDO8 (vdd_int_m) stays at least:
 *   max(max(vdd_int, vdd_disp) + int_delta, int_vthr)
 * Raising LDO8 before raising vdd_int/vdd_disp, and lowering vdd_int/vdd_disp
 * before lowering LDO8.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/arm-smccc.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/regulator/coupler.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/string.h>
#include <linux/suspend.h>

#define to_exynos7420_coupler(x) \
	container_of(x, struct exynos7420_regulator_coupler, coupler)

struct exynos7420_regulator_coupler {
	struct regulator_coupler coupler;
	struct regulator_dev *vdd_int;
	struct regulator_dev *vdd_disp;
	struct regulator_dev *vdd_int_m;
	int int_vthr;
	int int_delta;
};

static int exynos7420_rail_desired_uv(struct regulator_dev *rdev,
				      suspend_state_t state)
{
	int min_uV = 0;
	int max_uV = INT_MAX;
	int ret;

	ret = regulator_check_consumers(rdev, &min_uV, &max_uV, state);
	if (ret < 0)
		return ret;

	if (min_uV == 0) {
		ret = regulator_get_voltage_rdev(rdev);
		if (ret < 0)
			return ret;
		min_uV = ret;
	}

	ret = regulator_check_voltage(rdev, &min_uV, &max_uV);
	if (ret < 0)
		return ret;

	return min_uV;
}

static int exynos7420_balance_voltage(struct regulator_coupler *coupler,
				      struct regulator_dev *rdev,
				      suspend_state_t state)
{
	struct exynos7420_regulator_coupler *c = to_exynos7420_coupler(coupler);
	int int_uv, disp_uv, cur_ldo8, req_ldo8, ldo8_min, ldo8_max, ret;

	if (state != PM_SUSPEND_ON)
		return regulator_do_balance_voltage(rdev, state, true);

	if (rdev == c->vdd_int_m) {
		if (rdev->use_count == 0)
			return regulator_do_balance_voltage(rdev, state, true);
		return -EPERM;
	}

	if (!c->vdd_int || !c->vdd_disp || !c->vdd_int_m)
		return -EPROBE_DEFER;

	int_uv = exynos7420_rail_desired_uv(c->vdd_int, state);
	if (int_uv < 0)
		return int_uv;

	disp_uv = exynos7420_rail_desired_uv(c->vdd_disp, state);
	if (disp_uv < 0)
		return disp_uv;

	cur_ldo8 = regulator_get_voltage_rdev(c->vdd_int_m);
	if (cur_ldo8 < 0)
		return cur_ldo8;

	ldo8_min = c->vdd_int_m->constraints->min_uV;
	ldo8_max = c->vdd_int_m->constraints->max_uV;
	req_ldo8 = max(max(int_uv, disp_uv) + c->int_delta, c->int_vthr);
	req_ldo8 = clamp_t(int, req_ldo8, ldo8_min, ldo8_max);

	if (req_ldo8 > cur_ldo8) {
		ret = regulator_set_voltage_rdev(c->vdd_int_m, req_ldo8,
						 ldo8_max, state);
		if (ret)
			return ret;
		return regulator_do_balance_voltage(rdev, state, true);
	}

	ret = regulator_do_balance_voltage(rdev, state, true);
	if (ret)
		return ret;

	if (req_ldo8 != cur_ldo8)
		return regulator_set_voltage_rdev(c->vdd_int_m, req_ldo8,
						  ldo8_max, state);

	return 0;
}

static int exynos7420_coupler_attach(struct regulator_coupler *coupler,
				     struct regulator_dev *rdev)
{
	struct exynos7420_regulator_coupler *c = to_exynos7420_coupler(coupler);
	const char *name = rdev_get_name(rdev);

	if (rdev->coupling_desc.n_coupled > 3)
		return 1;

	if (!strcmp(name, "vdd_int")) {
		if (c->vdd_int)
			return -EINVAL;
		c->vdd_int = rdev;
		return 0;
	}

	if (!strcmp(name, "vdd_disp")) {
		if (c->vdd_disp)
			return -EINVAL;
		c->vdd_disp = rdev;
		return 0;
	}

	if (!strcmp(name, "vdd_int_m")) {
		if (c->vdd_int_m)
			return -EINVAL;
		c->vdd_int_m = rdev;
		return 0;
	}

	return 1;
}

static int exynos7420_coupler_detach(struct regulator_coupler *coupler,
				     struct regulator_dev *rdev)
{
	struct exynos7420_regulator_coupler *c = to_exynos7420_coupler(coupler);

	if (rdev == c->vdd_int)
		c->vdd_int = NULL;
	else if (rdev == c->vdd_disp)
		c->vdd_disp = NULL;
	else if (rdev == c->vdd_int_m)
		c->vdd_int_m = NULL;

	return 0;
}

static struct exynos7420_regulator_coupler exynos7420_coupler = {
	.coupler = {
		.attach_regulator = exynos7420_coupler_attach,
		.detach_regulator = exynos7420_coupler_detach,
		.balance_voltage  = exynos7420_balance_voltage,
	},
};

static int __init exynos7420_coupler_init(void)
{
	static const int vthr_table[4] = {
		750000, 800000, 700000, 850000
	};
	struct device_node *np;
	bool enable_coupler = false;
	struct arm_smccc_res res = { 0 };
	unsigned int vthr_idx = 0, delta_idx = 0;

	for_each_compatible_node(np, NULL, "samsung,s2mps15-pmic") {
		if (of_property_read_bool(np, "meizu,int-ldo8-coupler")) {
			enable_coupler = true;
			of_node_put(np);
			break;
		}
	}
	if (!enable_coupler)
		return 0;

	arm_smccc_smc((u32)-101, (3U << 30) | (0x101e0198U >> 2),
		      0, 0, 0, 0, 0, 0, &res);
	if ((int)res.a0 == 0) {
		vthr_idx = (res.a2 >> 12) & 0x3;
		delta_idx = (res.a2 >> 14) & 0x3;
	}

	exynos7420_coupler.int_vthr = vthr_table[vthr_idx];
	exynos7420_coupler.int_delta = delta_idx ? (delta_idx * 25000 + 25000) : 0;

	pr_info("INT LDO8 coupler initialized: vthr=%d delta=%d\n",
		exynos7420_coupler.int_vthr, exynos7420_coupler.int_delta);

	return regulator_coupler_register(&exynos7420_coupler.coupler);
}
arch_initcall(exynos7420_coupler_init);
