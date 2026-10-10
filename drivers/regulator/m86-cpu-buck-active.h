#include "m86-cpu-rail-api.h"
/* SPDX-License-Identifier: GPL-2.0-only */
/* Candidate only. Included by s2mps11.c after struct s2mps11_info definition.
 * Source: PRO5 vendor drivers/regulator/s2mps15.c:360-396,507-560.
 * CPU sync uses LDODVS1/3 (LDO7/9), not the INT LDO8/B4/B5 algorithm.
 */
struct m86_cpu_rail_state {
	unsigned int selector, ctrl, dvs, ramp;
	unsigned int delta_uv, ramp_uv_us;
	bool sync;
};

/* Coordinator handshake: only CPU-specific provider contexts are accepted. */
struct m86_cpu_rail_context {
	struct s2mps11_info *info;
	unsigned int index;
	bool linear, bit0_observed;
	unsigned long linear_hz;
	int cached_selector;
	struct m86_cpu_rail_state state;
	bool state_valid;
};
static DEFINE_MUTEX(m86_context_lock);
static struct m86_cpu_rail_context *m86_active_context[2];

static void m86_clear_context(void *data)
{
	struct m86_cpu_rail_context *ctx = data;
	mutex_lock(&m86_context_lock);
	if (m86_active_context[ctx->index] == ctx)
		m86_active_context[ctx->index] = NULL;
	mutex_unlock(&m86_context_lock);
}

static int m86_cpu_read_register(struct regulator_dev *rdev,
				 unsigned int reg, unsigned int *value)
{
	int ret = regmap_read(rdev->regmap, reg, value);

	if (ret)
		dev_err_ratelimited(&rdev->dev,
			"CPU rail read failed regulator_id=%u reg=%#x ret=%d\n",
			rdev_get_id(rdev), reg, ret);
	return ret;
}

static int m86_cpu_read_state(struct regulator_dev *rdev,
			      struct m86_cpu_rail_state *s)
{
	struct m86_cpu_rail_context *ctx = rdev_get_drvdata(rdev);
	struct s2mps11_info *info = ctx->info;
	unsigned int id = rdev_get_id(rdev), index, revision, delta;
	int ret;

	if (id != S2MPS15_BUCK2 && id != S2MPS15_BUCK3)
		return -EINVAL;
	index = id == S2MPS15_BUCK2 ? 0 : 1;
	ret = m86_cpu_read_register(rdev, S2MPS15_REG_ID, &revision);
	if (ret)
		return ret;
	/* This candidate is bounded to the measured ID12 Rev1-range device. */
	if (revision != 0x12) {
		dev_err_ratelimited(&rdev->dev,
			"CPU rail ID rejected cluster=%u revision=%#x expected=0x12\n",
			index, revision);
		return -EOPNOTSUPP;
	}
	ret = m86_cpu_read_register(rdev, rdev->desc->enable_reg, &s->ctrl);
	if (ret)
		return ret;
	ret = m86_cpu_read_register(rdev, rdev->desc->vsel_reg, &s->selector);
	if (ret)
		return ret;
	ret = m86_cpu_read_register(rdev, index ? S2MPS15_REG_LDODVS3 :
			  S2MPS15_REG_LDODVS1, &s->dvs);
	if (ret)
		return ret;
	ret = m86_cpu_read_register(rdev, S2MPS15_REG_BRAMP, &s->ramp);
	if (ret)
		return ret;
	/* Keep CTRL1 bits7:1 exact. Available vendor code preserves unnamed
	 * bit0 in mode/PWM updates and does not test it for CPU selector writes.
	 * No CTRL1 write is added; DVS threshold/reserved guards stay exact.
	 */
	if ((s->ctrl & 0x7e) != (info->m86_cpu_ctrl[index] & 0x7e) ||
	    (s->dvs & 0xe3) != 0x80 || s->selector > 255) {
		dev_err_ratelimited(&rdev->dev,
			"CPU rail state rejected cluster=%u revision=%#x ctrl=%#x expected_ctrl=%#x dvs=%#x dvs_masked=%#x selector=%#x ramp=%#x\n",
			index, revision, s->ctrl, info->m86_cpu_ctrl[index],
			s->dvs, s->dvs & 0xe3, s->selector, s->ramp);
		return -EOPNOTSUPP;
	}
	if ((s->ctrl & (BIT(7) | BIT(0))) && !ctx->bit0_observed) {
		ctx->bit0_observed = true;
		dev_info(&rdev->dev,
			 "CPU rail accepted CTRL1 mode/unnamed bits cluster=%u ctrl=%#x expected_ctrl=%#x; bits6:1 and rail guards validated, no CTRL1 write\n",
			 index, s->ctrl, info->m86_cpu_ctrl[index]);
	}
	delta = (s->dvs >> 3) & 3;
	s->delta_uv = delta ? (delta - 1) * 25000 + 50000 : 0;
	s->sync = !!(s->dvs & BIT(2));
	/* Vendor BRAMP BUCK2 shift6, BUCK3 shift4; 6mV/us * 2^code.
	 * Apply slower consumer ramp constraint, but never rewrite BRAMP.
	 */
	s->ramp_uv_us = 6000U << ((s->ramp >> (index ? 4 : 6)) & 3);
	if (rdev->constraints->ramp_delay > 0)
		s->ramp_uv_us = min(s->ramp_uv_us,
				    (unsigned int)rdev->constraints->ramp_delay);
	return 0;
}

static void m86_cpu_wait(unsigned int from_uv, unsigned int to_uv,
			 unsigned int ramp_uv_us)
{
	unsigned int delay = DIV_ROUND_UP(abs((int)to_uv - (int)from_uv),
					 ramp_uv_us);

	if (delay)
		usleep_range(delay, delay + 10);
}

static int m86_cpu_write_selector(struct regulator_dev *rdev, unsigned int sel,
				 unsigned int old_uv, unsigned int ramp)
{
	unsigned int actual;
	int ret;

	ret = regmap_write(rdev->regmap, rdev->desc->vsel_reg, sel);
	if (ret)
		return ret;
	/* Settle before a subsequent DVS-bit write or callback return. */
	m86_cpu_wait(old_uv, 300000 + sel * 6250, ramp);
	ret = regmap_read(rdev->regmap, rdev->desc->vsel_reg, &actual);
	if (ret)
		return ret;
	return actual == sel ? 0 : -EIO;
}

static int m86_cpu_get_selector(struct regulator_dev *rdev)
{
	struct m86_cpu_rail_context *ctx = rdev_get_drvdata(rdev);
	struct s2mps11_info *info = ctx->info;
	int ret;

	mutex_lock(&info->m86_cpu_lock);
	if (!ctx->state_valid) {
		ret = m86_cpu_read_state(rdev, &ctx->state);
		if (ret) {
			mutex_unlock(&info->m86_cpu_lock);
			return ret;
		}
		ctx->cached_selector = ctx->state.selector;
		ctx->state_valid = true;
	}
	ret = ctx->cached_selector;
	mutex_unlock(&info->m86_cpu_lock);
	/* The entire CTRL2 byte is the CPU voltage selector, not CTRL1 mode.
	 * SRAM DVS delta/sync do not add an offset to this CPU buck selector.
	 */
	return ret;
}

static int m86_cpu_set_selector(struct regulator_dev *rdev, unsigned int sel)
{
	struct m86_cpu_rail_context *ctx = rdev_get_drvdata(rdev);
	struct s2mps11_info *info = ctx->info;
	unsigned int new_uv, old_uv, index, dvs_reg;
	bool wanted_sync;
	int ret;

	if (sel > 255 || !rdev->constraints->min_uV ||
	    !rdev->constraints->max_uV)
		return -EINVAL;
	new_uv = 300000 + sel * 6250;
	if (new_uv < 500000 ||
	    new_uv > (rdev_get_id(rdev) == S2MPS15_BUCK2 ? 1300000 : 1200000))
		return -EINVAL;
	if (new_uv < rdev->constraints->min_uV ||
	    new_uv > rdev->constraints->max_uV)
		return -EINVAL;
	index = rdev_get_id(rdev) == S2MPS15_BUCK2 ? 0 : 1;
	dvs_reg = index ? S2MPS15_REG_LDODVS3 : S2MPS15_REG_LDODVS1;
	mutex_lock(&info->m86_cpu_lock);
	if (info->m86_cpu_faulted) {
		ret = -EIO;
		goto unlock;
	}
	if (!ctx->state_valid) {
		ret = m86_cpu_read_state(rdev, &ctx->state);
		if (ret)
			goto unlock;
		ctx->cached_selector = ctx->state.selector;
		ctx->state_valid = true;
	}
	if (sel == ctx->cached_selector) {
		ret = 0;
		goto unlock;
	}
	old_uv = 300000 + ctx->cached_selector * 6250;
	if (ctx->linear && ctx->state.sync != (new_uv + ctx->state.delta_uv <= 1175000)) {
		ret = -EOPNOTSUPP; /* No writes: linear API never invokes sync bridge. */
		goto unlock;
	}
	/* Only a synchronous coordinator wrapper may change voltage. The token
	 * is tied to this rail context, caller task, exact voltage and clock.
	 */
	if (!info->m86_cpu_authorized ||
	    info->m86_cpu_authorized_context != ctx ||
	    info->m86_cpu_authorized_task != current ||
	    info->m86_cpu_authorized_uv != new_uv ||
	    clk_get_rate(info->m86_cpu_authorized_clock) !=
		(ctx->linear ? ctx->linear_hz :
		 (ctx->index ? 400000000UL : 800000000UL))) {
		ret = -EPERM;
		goto unlock;
	}
	info->m86_cpu_authorized = false; /* consume once, before first write */
	wanted_sync = new_uv + ctx->state.delta_uv <= 1175000;
	if (wanted_sync != ctx->state.sync) {
		/* Vendor crossing sequence: temporarily 1.075V, settle, change
		 * only sync bit2, then set requested selector. Do not replay
		 * probe's LDO7/LDO9 or delta/vthr initialization.
		 */
		if (1075000 < rdev->constraints->min_uV ||
		    1075000 > rdev->constraints->max_uV) {
			ret = -EINVAL;
			goto unlock;
		}
		ret = m86_cpu_write_selector(rdev, 0x7c, old_uv,
					     ctx->state.ramp_uv_us);
		if (ret)
			goto fault;
		old_uv = 1075000;
		ret = regmap_update_bits(rdev->regmap, dvs_reg, BIT(2),
					 wanted_sync ? BIT(2) : 0);
		if (ret)
			goto fault;
		ctx->state.sync = wanted_sync;
	}
	ret = m86_cpu_write_selector(rdev, sel, old_uv, ctx->state.ramp_uv_us);
	if (ret)
		goto fault;
	ctx->cached_selector = sel;
	ctx->state.selector = sel;
	goto unlock;
fault:
	/* An error may leave the intermediate 1.075V / changed sync state.
	 * Stop subsequent writes; don't attempt unproven voltage rollback.
	 */
	info->m86_cpu_faulted = true;
	dev_err(&rdev->dev, "CPU rail transaction failed %d; hardware state uncertain, further writes blocked\n", ret);
unlock:
	mutex_unlock(&info->m86_cpu_lock);
	return ret;
}

/* Fixed staging rates are part of the measured bin15 coordination contract.
 * Coordinator must own clock exclusivity and verify vendor EMA beforehand.
 */
static int m86_cpu_rail_set_internal(struct regulator *rail, struct clk *cpu_clock,
			       unsigned int uv, unsigned int cluster_index,
			       bool linear, unsigned long expected_hz)
{
	struct m86_cpu_rail_context *ctx;
	struct s2mps11_info *info;
	unsigned long stage;
	const char *name;
	int ret;

	if (!rail || !cpu_clock || cluster_index > 1)
		return -EINVAL;
	mutex_lock(&m86_context_lock);
	ctx = m86_active_context[cluster_index];
	if (!ctx || regulator_get_drvdata(rail) != ctx) {
		ret = -ENODEV;
		goto out;
	}
	name = __clk_get_name(cpu_clock);
	stage = linear ? expected_hz :
		(cluster_index ? 400000000UL : 800000000UL);
	if (linear && (stage < (cluster_index ? 400000000UL : 800000000UL) ||
		       stage > (cluster_index ? 1500000000UL : 2100000000UL))) {
		ret = -EINVAL;
		goto out;
	}
	if (strcmp(name, cluster_index ? "m86_apollo_cpu" : "m86_atlas_cpu") ||
	    clk_get_rate(cpu_clock) != stage) {
		ret = -EPERM;
		goto out;
	}
	usleep_range(2, 5);
	if (clk_get_rate(cpu_clock) != stage) {
		ret = -EPERM;
		goto out;
	}
	info = ctx->info;
	mutex_lock(&info->m86_cpu_lock);
	if (info->m86_cpu_faulted || info->m86_cpu_authorized) {
		ret = -EIO;
		mutex_unlock(&info->m86_cpu_lock);
		goto out;
	}
	ctx->linear = linear;
	ctx->linear_hz = stage;
	info->m86_cpu_authorized_context = ctx;
	info->m86_cpu_authorized_task = current;
	info->m86_cpu_authorized_clock = cpu_clock;
	info->m86_cpu_authorized_uv = uv;
	info->m86_cpu_authorized = true;
	mutex_unlock(&info->m86_cpu_lock);
	ret = regulator_set_voltage(rail, uv, uv);
	mutex_lock(&info->m86_cpu_lock);
	info->m86_cpu_authorized = false;
	ctx->linear = false;
	ctx->linear_hz = 0;
	info->m86_cpu_authorized_task = NULL;
	info->m86_cpu_authorized_context = NULL;
	info->m86_cpu_authorized_clock = NULL;
	mutex_unlock(&info->m86_cpu_lock);
out:
	mutex_unlock(&m86_context_lock);
	return ret;
}
int m86_cpu_rail_set_coordinated(struct regulator *rail, struct clk *cpu_clock,
			       unsigned int uv, unsigned int index)
{
	return m86_cpu_rail_set_internal(rail, cpu_clock, uv, index, false, 0);
}
EXPORT_SYMBOL_GPL(m86_cpu_rail_set_coordinated);

/* Caller exclusively owns clock and has verified actual known OPP+EMA.
 * Rising rail: expected_hz is old OPP; falling rail: target OPP is already set.
 * No sync-bit change or hidden1.075V intermediate is permitted by this API.
 */
int m86_cpu_rail_set_linear(struct regulator *rail, struct clk *cpu_clock,
			  unsigned int uv, unsigned int index,
			  unsigned long expected_hz)
{
	return m86_cpu_rail_set_internal(rail, cpu_clock, uv, index, true, expected_hz);
}
EXPORT_SYMBOL_GPL(m86_cpu_rail_set_linear);

static const struct regulator_ops m86_cpu_buck_active_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.is_enabled = regulator_is_enabled_regmap,
	.get_voltage_sel = m86_cpu_get_selector,
	.set_voltage_sel = m86_cpu_set_selector,
	/* All steps settle internally. No enable/disable/mode/ramp-write ops. */
};
