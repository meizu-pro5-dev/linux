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

struct m86_cpu_rail_context {
	struct s2mps11_info *info;
	unsigned int index;
	bool bit0_observed;
	int cached_selector;
	struct m86_cpu_rail_state state;
	bool state_valid;
};

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
	int ret;

	ret = regmap_write(rdev->regmap, rdev->desc->vsel_reg, sel);
	if (ret)
		return ret;
	/* Settle before a subsequent DVS-bit write or callback return. */
	m86_cpu_wait(old_uv, 300000 + sel * 6250, ramp);
	return 0;
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
	/*
	 * TD-R4: An error may leave the intermediate 1.075V or unchanged selector.
	 * Abort the current transaction and keep the safe high-voltage step.
	 * Invalidate cached state so the next transaction re-reads actual hardware
	 * registers via m86_cpu_read_state(), enabling retry recovery.
	 * Do NOT lock info->m86_cpu_faulted = true.
	 */
	ctx->state_valid = false;
	dev_err_ratelimited(&rdev->dev,
		"CPU rail write failed %d; transaction aborted, state invalidated for retry\n", ret);
unlock:
	mutex_unlock(&info->m86_cpu_lock);
	return ret;
}

static const struct regulator_ops m86_cpu_buck_active_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.is_enabled = regulator_is_enabled_regmap,
	.get_voltage_sel = m86_cpu_get_selector,
	.set_voltage_sel = m86_cpu_set_selector,
	/* All steps settle internally. No enable/disable/mode/ramp-write ops. */
};
