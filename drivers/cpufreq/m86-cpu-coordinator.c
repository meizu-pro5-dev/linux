// SPDX-License-Identifier: GPL-2.0-only
/* Voltage/EMA/clock coordination and native EAS for the measured m86 bin. */
#include <linux/clk.h>
#include <linux/arch_topology.h>
#include <linux/energy_model.h>
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/reboot.h>
#include <linux/regulator/consumer.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>
#include <linux/m86-bus-qos.h>
#include <linux/m86-asv-cal.h>

struct m86_opp { unsigned int khz, uv, bus_qos; };
#include "m86-opp.h"
struct m86_cluster {
	struct clk *clk;
	struct regulator *supply;
	cpumask_t cpus;
	const struct m86_opp *opp;
	unsigned int nr, thermal_cap, max_khz;
	struct cpufreq_frequency_table *table;
	struct thermal_cooling_device *cooling;
	struct freq_qos_request qos;
	bool qos_live, cooling_ready;
	struct mutex lock;
	unsigned int cur_khz;
	unsigned int cur_uv;
	unsigned int cur_bus_qos;
};
struct m86_coord {
	struct device *dev;
	struct mutex lock;
	struct m86_cluster cluster[2]; /* 0 atlas, 1 apollo */
	struct m86_asv_cal_info asv_cal;
	struct regmap *ema; /* Existing shared SYSREG regmap. */
	void __iomem *topc;
	struct clk *fin;
	struct thermal_zone_device *zone;
	struct delayed_work thermal_work;
	bool faulted, registered;
	struct mutex bus_lock;
	unsigned int required_bus_qos;
	struct device *bus_dev;
	struct dev_pm_qos_request bus_qos_req;
	bool bus_qos_active;
	unsigned int cur_bus_qos;
};
static struct m86_coord *coordinator;
/* First bring-up only; disable after collecting manual transition evidence. */
static bool debug_transitions;
module_param(debug_transitions, bool, 0644);
MODULE_PARM_DESC(debug_transitions, "Log manual CPU DVFS actual clock/rail/EMA evidence");
#include "m86-cpu-rail-api.h"

static int cluster_index(unsigned int cpu)
{
	int i;
	if (!coordinator)
		return -ENODEV;
	for (i = 0; i < 2; i++)
		if (cpumask_test_cpu(cpu, &coordinator->cluster[i].cpus))
			return i;
	return -ENODEV;
}

/* Vendor asv-exynos7420_cal.c:868-964; uses fused EMA values when non-zero,
 * falling back to vendor default EMA values when fuse bits are zero.
 * Preserve unrelated register bits.
 */
static int set_ema(struct m86_coord *d, unsigned int index, unsigned int uv)
{
	static const u32 offsets[] = { 0x38, 0x138, 0x2908 };
	u32 masks[2], values[2], old, actual;
	int ret;
	unsigned int first, n, i;

	if (index) {
		u32 apollo_ema = m86_asv_get_apollo_ema(&d->asv_cal, uv);

		first = 0; n = 1;
		masks[0] = (7U << 20) | (7U << 8);
		values[0] = (apollo_ema << 20) | (apollo_ema << 8);
	} else {
		u32 l1, l2;

		m86_asv_get_atlas_ema(&d->asv_cal, uv, &l1, &l2);
		first = 1; n = 2;
		masks[0] = 15U << 4; values[0] = l1 << 4;
		masks[1] = 15U << 8; values[1] = l2 << 8;
	}
	for (i = 0; i < n; i++) {
		ret = regmap_read(d->ema, offsets[first + i], &old);
		if (ret) return ret;
		ret = regmap_update_bits(d->ema, offsets[first + i], masks[i], values[i]);
		if (ret) return ret;
		ret = regmap_read(d->ema, offsets[first + i], &actual);
		if (ret) return ret;
		if (actual != ((old & ~masks[i]) | values[i])) return -EIO;
	}
	return 0;
}

static int verify_ema(struct m86_coord *d, unsigned int index, unsigned int uv)
{
	u32 a, b;
	int ret;

	if (index) {
		u32 apollo_ema = m86_asv_get_apollo_ema(&d->asv_cal, uv);

		ret = regmap_read(d->ema, 0x38, &a);
		return ret ?: (a == (0x010011U | (apollo_ema << 20) | (apollo_ema << 8)) ? 0 : -EIO);
	}
	{
		u32 l1, l2;

		m86_asv_get_atlas_ema(&d->asv_cal, uv, &l1, &l2);
		ret = regmap_read(d->ema, 0x138, &a);
		if (ret) return ret;
		ret = regmap_read(d->ema, 0x2908, &b);
		if (ret) return ret;
		return a == (2U | (l1 << 4)) &&
		       b == (0x72U | (l2 << 8)) ? 0 : -EIO;
	}
}

static int set_voltage_ema(struct m86_coord *d, unsigned int index,
			   unsigned int uv, unsigned long expected_hz)
{
	struct m86_cluster *c = &d->cluster[index];
	int old_uv = regulator_get_voltage(c->supply), ret;

	if (old_uv <= 0)
		return old_uv ?: -EIO;
	/* Vendor exynos-mp-cpufreq.c:507-521 rising voltage before EMA;
	 * 556-559 falling voltage after EMA. */
	if (uv <= old_uv) {
		ret = set_ema(d, index, uv);
		if (ret)
			return ret;
	}
	ret = m86_cpu_rail_set_linear(c->supply, c->clk, uv, index, expected_hz);
	if (ret)
		return ret;
	if (regulator_get_voltage(c->supply) != uv)
		return -EIO;
	return set_ema(d, index, uv);
}

static int set_rate_checked(struct m86_cluster *c, unsigned int khz)
{
	int ret = clk_set_rate(c->clk, (unsigned long)khz * 1000);
	if (ret)
		return ret;
	return clk_get_rate(c->clk) == (unsigned long)khz * 1000 ? 0 : -EIO;
}

/* Vendor exynos7420_bus.c:3291-3304 and 4762: logical MIF kHz is
 * mif_pll_hz / (2 * 1000). PHY LP4 cell div8/4 is a separate branch.
 * Only inspect PLL/MUX selectors; vendor STAT5 mapping is inconsistent
 * with captured status, so never claim its zero field validates the PLL.
 */
static int fixed_bus_floor(struct m86_coord *d)
{
	u32 con, sel, con2, sel2;
	u32 m, p, s, req_khz;
	u64 logical_khz;

	exynos7420_bus_dvfs_read_lock();
	con = readl(d->topc + 0x150);
	sel = readl(d->topc + 0x214);
	con2 = readl(d->topc + 0x150);
	sel2 = readl(d->topc + 0x214);
	req_khz = exynos7420_mif_effective_floor_khz(READ_ONCE(d->required_bus_qos));
	exynos7420_bus_dvfs_read_unlock();

	m = (con >> 16) & 0x3ff;
	p = (con >> 8) & 0x3f;
	s = con & 7;
	if (con != con2 || sel != sel2 ||
	    !(con & BIT(31)) || !(con & BIT(29)) || !m || !p ||
	    !(sel & BIT(16)) || !(sel & BIT(20)) || clk_get_rate(d->fin) != 24000000)
		return -EIO;
	logical_khz = div_u64(24000000ULL * m, (p << s) * 2000);
	return logical_khz >= req_khz ? 0 : -ERANGE;
}

static int temperature(struct m86_coord *d, int *temp)
{
	int ret = fixed_bus_floor(d);
	if (ret) {
		dev_err_ratelimited(d->dev, "temperature guard: fixed MIF failed ret=%d\n", ret);
		return ret;
	}
	ret = thermal_zone_get_temp(d->zone, temp);
	if (ret) dev_err_ratelimited(d->dev, "temperature guard: TMU read failed ret=%d\n", ret);
	/* TMU monitor checks all four banks' enable, valid status and trim.
	 * Plausibility catches obviously invalid data; does not prove freshness. */
	if (!ret && (*temp < -20000 || *temp > 125000))
		ret = -ERANGE;
	return ret;
}

/* Must hold d->lock. Keep rails/EMA unchanged, attempt only clock throttling.
 * Error state is sticky. Temperature blindness or transaction failure requests
 * shutdown even if clock throttle succeeds; no indefinite high-power freeze.
 */
static void fail_closed(struct m86_coord *d, int error)
{
	unsigned int i;
	if (d->faulted)
		return;
	d->faulted = true;
	mutex_lock(&d->bus_lock);
	if (d->bus_qos_active)
		dev_pm_qos_update_request(&d->bus_qos_req, 1552000);
	d->cur_bus_qos = 1552000;
	WRITE_ONCE(d->required_bus_qos, 1552000);
	mutex_unlock(&d->bus_lock);
	for (i = 0; i < 2; i++) {
		int r;
		mutex_lock(&d->cluster[i].lock);
		r = set_rate_checked(&d->cluster[i], d->cluster[i].opp[0].khz);
		mutex_unlock(&d->cluster[i].lock);
		if (r && r != -EHOSTDOWN)
			dev_err(d->dev, "cluster%u emergency clock throttle failed\n", i);
	}
	dev_err(d->dev, "DVFS permanently blocked after %d; requesting orderly shutdown\n", error);
	orderly_poweroff(true);
}

static unsigned int cap_for_temp(struct m86_cluster *c, int temp)
{
	/* Conservative bring-up thresholds, not vendor-certified production limits. */
	if (temp >= 85000)
		return c->opp[0].khz;
	if (temp >= 75000)
		return 900000;
	if (temp >= 65000)
		return 1000000;
	return c->max_khz;
}

static void trace_target(struct m86_coord *d, struct m86_cluster *c,
                         unsigned int ci, unsigned long old_hz, int old_uv,
                         unsigned int target, unsigned int target_uv, int ret, ktime_t started)
{
	u32 apollo = 0, l1 = 0, l2 = 0;
	int ra = 0, r1 = 0, r2 = 0, actual_uv = 0;
	unsigned long actual_hz = 0;

	if (!debug_transitions) return;
	ra = regmap_read(d->ema, 0x38, &apollo);
	r1 = regmap_read(d->ema, 0x138, &l1);
	r2 = regmap_read(d->ema, 0x2908, &l2);
	actual_uv = regulator_get_voltage(c->supply);
	actual_hz = clk_get_rate(c->clk);
	dev_info(d->dev, "dvfs cluster=%s old_hz=%lu target_hz=%lu actual_hz=%lu old_uv=%d target_uv=%u actual_uv=%d ret=%d ema_apollo=%08x/%d ema_l1=%08x/%d ema_l2=%08x/%d cap_khz=%u fault=%u duration_us=%lld\n",
		ci ? "apollo" : "atlas", old_hz, target * 1000UL, actual_hz,
		old_uv, target_uv, actual_uv, ret, apollo, ra, l1, r1, l2, r2,
		c->thermal_cap, d->faulted, (long long)ktime_us_delta(ktime_get(), started));
}

static int m86_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct m86_coord *d = coordinator;
	struct m86_cluster *c = policy->driver_data;
	unsigned int ci = c == &d->cluster[1], target;
	unsigned long old_hz;
	int old_uv;
	int temp, ret;
	bool do_fail_closed = false;
	const char *phase = "fresh thermal/MIF guard";
	ktime_t started = ktime_get();

	if (index >= c->nr)
		return -EINVAL;
	target = c->opp[index].khz;
	mutex_lock(&c->lock);
	if (c->cur_khz == target) {
		mutex_unlock(&c->lock);
		return 0;
	}
	old_hz = c->cur_khz ? (unsigned long)c->cur_khz * 1000UL : clk_get_rate(c->clk);
	old_uv = c->cur_uv ? c->cur_uv : regulator_get_voltage(c->supply);
	if (READ_ONCE(d->faulted)) { ret = -EIO; goto out; }
	if (!c->cooling_ready) {
		ret = old_hz == target * 1000UL ? 0 : -EAGAIN;
		goto out;
	}
	ret = temperature(d, &temp);
	if (ret) { do_fail_closed = true; goto out; }
	if (temp >= 105000) { ret = -ERANGE; do_fail_closed = true; goto out; }
	/* Check fresh temperature on every target, independent of governor/QoS lag. */
	if (target > min(c->thermal_cap, cap_for_temp(c, temp))) {
		ret = -EAGAIN;
		/* Fresh reading can outrun published QoS: expedite correction.
		 * Do not return success for a different unnotified frequency. */
		mod_delayed_work(system_wq, &d->thermal_work, 0);
		goto out;
	}
	/* Check current operating point before authorizing a directional move.
	 * Voltage tables are monotonic; CCF temporary BUS0 dividers bound the
	 * transient core frequency to <=min(old,target), unlike vendor raw BUS0.
	 */
	phase = "verify old OPP/EMA";
	/* Preserve the provider's read error instead of collapsing it to EIO. */
	if (old_uv < 0) {
		phase = "read old voltage";
		ret = old_uv;
		goto fault;
	}
	{
		unsigned int j;
		bool known = false;
		for (j = 0; j < c->nr; j++)
			if (old_hz == c->opp[j].khz * 1000UL && old_uv == c->opp[j].uv)
				known = true;
		if (!known) { ret = -EIO; goto fault; }
		ret = verify_ema(d, ci, old_uv);
		if (ret) goto fault;
	}
	unsigned int target_bus_qos = c->opp[index].bus_qos;
	unsigned int max_bus_qos;

	if (target * 1000UL < old_hz) {
		/* Lower clock/readback first; target low-voltage EMA then rail. */
		phase = "linear down clock";
		ret = set_rate_checked(c, target);
		if (ret) goto fault;
		phase = "linear down voltage/EMA";
		ret = set_voltage_ema(d, ci, c->opp[index].uv, target * 1000UL);
		if (ret) goto fault;
		mutex_lock(&d->bus_lock);
		c->cur_bus_qos = target_bus_qos;
		max_bus_qos = max(target_bus_qos, d->cluster[1 - ci].cur_bus_qos);
		if (max_bus_qos < d->cur_bus_qos) {
			d->cur_bus_qos = max_bus_qos;
			WRITE_ONCE(d->required_bus_qos, max_bus_qos);
			if (d->bus_qos_active)
				dev_pm_qos_update_request(&d->bus_qos_req, max_bus_qos);
		}
		mutex_unlock(&d->bus_lock);
	} else if (target * 1000UL > old_hz) {
		unsigned int old_c_bus_qos = c->cur_bus_qos;
		unsigned int old_d_bus_qos = d->cur_bus_qos;

		mutex_lock(&d->bus_lock);
		c->cur_bus_qos = target_bus_qos;
		max_bus_qos = max(target_bus_qos, d->cluster[1 - ci].cur_bus_qos);
		if (max_bus_qos > d->cur_bus_qos) {
			if (d->bus_qos_active)
				dev_pm_qos_update_request(&d->bus_qos_req, max_bus_qos);
			d->cur_bus_qos = max_bus_qos;
			WRITE_ONCE(d->required_bus_qos, max_bus_qos);
		}
		mutex_unlock(&d->bus_lock);

		/* Verify MIF hardware has achieved the requested floor before raising CPU clock */
		ret = fixed_bus_floor(d);
		if (ret) {
			mutex_lock(&d->bus_lock);
			c->cur_bus_qos = old_c_bus_qos;
			if (d->cur_bus_qos != old_d_bus_qos) {
				d->cur_bus_qos = old_d_bus_qos;
				WRITE_ONCE(d->required_bus_qos, old_d_bus_qos);
				if (d->bus_qos_active)
					dev_pm_qos_update_request(&d->bus_qos_req, old_d_bus_qos);
			}
			mutex_unlock(&d->bus_lock);
			ret = -EAGAIN;
			goto out;
		}

		/* Raise rail/readback and EMA at old OPP before clock increase. */
		phase = "linear up voltage/EMA";
		ret = set_voltage_ema(d, ci, c->opp[index].uv, old_hz);
		if (ret) goto fault;
		phase = "linear up clock";
		ret = set_rate_checked(c, target);
		if (ret) goto fault;
	} else {
		ret = 0; /* Known exact clock/rail OPP, no hardware transaction. */
		mutex_lock(&d->bus_lock);
		c->cur_bus_qos = target_bus_qos;
		mutex_unlock(&d->bus_lock);
	}
	c->cur_khz = target;
	c->cur_uv = c->opp[index].uv;
	goto out;
fault:
	dev_err(d->dev, "target fault cluster=%s phase=%s ret=%d\n", ci ? "apollo" : "atlas", phase, ret);
	do_fail_closed = true;
out:
	trace_target(d, c, ci, old_hz, old_uv, target, c->opp[index].uv, ret, started);
	mutex_unlock(&c->lock);
	if (do_fail_closed) {
		mutex_lock(&d->lock);
		fail_closed(d, ret);
		mutex_unlock(&d->lock);
	}
	return ret;
}

static unsigned int m86_get(unsigned int cpu)
{
	int i = cluster_index(cpu);
	return i < 0 ? 0 : clk_get_rate(coordinator->cluster[i].clk) / 1000;
}

static int m86_policy_init(struct cpufreq_policy *policy)
{
	struct m86_cluster *c;
	int i = cluster_index(policy->cpu), ret;
	if (i < 0) return i;
	c = &coordinator->cluster[i];
	/* cpufreq re-runs init after the last CPU left an inactive policy.
	 * Provider CPUHP precedes cpufreq CPUHP. Validate actual handoff before
	 * accepting a governor again; never lower the retained rail here. */
	{
		unsigned int j;
		u32 a, b;
		int t, r;
		unsigned long hz = clk_get_rate(c->clk);
		int uv = regulator_get_voltage(c->supply);
		bool known = false;
		for (j = 0; j < c->nr; j++)
			if (hz == c->opp[j].khz * 1000UL && uv == c->opp[j].uv)
				known = true;
		if (coordinator->faulted || !known) return -EIO;
		/* temperature() first invokes fixed_bus_floor(): MIF CON0/MUX
		 * stable reads, enabled/locked PLL and logical-kHz DT floor. */
		r = temperature(coordinator, &t);
		if (r || t >= 85000) return r ?: -ERANGE;
		if (i) {
			r = regmap_read(coordinator->ema, 0x38, &a);
			if (r) return r;
			if (a != 0x210211) return -EIO;
		} else {
			unsigned int l1 = uv >= 900000 ? 6 : uv >= 800000 ? 8 : 7;
			unsigned int l2 = uv >= 900000 ? 5 : uv >= 800000 ? 11 : 9;
			r = regmap_read(coordinator->ema, 0x138, &a);
			if (r) return r;
			r = regmap_read(coordinator->ema, 0x2908, &b);
			if (r) return r;
			if (a != (2U | (l1 << 4)) || b != (0x72U | (l2 << 8))) return -EIO;
		}
	}
	c->cur_khz = clk_get_rate(c->clk) / 1000;
	c->cur_uv = regulator_get_voltage(c->supply);
	policy->driver_data = c;
	policy->clk = c->clk;
	cpufreq_generic_init(policy, c->table, 10000000);
	/* GPIO-I2C transitions measured at roughly 30-45 ms on r71.
	 * Use the slow schedutil path; this is not the Android 1 ms fast path. */
	policy->transition_delay_us = 50000;
	/* generic_init sets all policy CPUs: restore the actual DT cluster mask. */
	cpumask_copy(policy->cpus, &c->cpus);
	ret = freq_qos_add_request(&policy->constraints, &c->qos,
				  FREQ_QOS_MAX, min(c->thermal_cap, (unsigned int)(clk_get_rate(c->clk) / 1000)));
	if (ret < 0) return ret;
	c->qos_live = true;
	return 0;
}

static void m86_policy_ready(struct cpufreq_policy *policy)
{
	struct m86_cluster *c = policy->driver_data;
	c->cooling = cpufreq_cooling_register(policy);
	mutex_lock(&coordinator->lock);
	if (IS_ERR_OR_NULL(c->cooling))
		fail_closed(coordinator, c->cooling ? PTR_ERR(c->cooling) : -ENODEV);
	else
		c->cooling_ready = true;
	mutex_unlock(&coordinator->lock);
	if (c->cooling_ready && freq_qos_update_request(&c->qos, c->thermal_cap) < 0) {
		mutex_lock(&coordinator->lock);
		fail_closed(coordinator, -EIO);
		mutex_unlock(&coordinator->lock);
	}
}

static int m86_policy_exit(struct cpufreq_policy *policy)
{
	struct m86_cluster *c = policy->driver_data;
	c->cooling_ready = false;
	if (!IS_ERR_OR_NULL(c->cooling)) cpufreq_cooling_unregister(c->cooling);
	c->cooling = NULL;
	if (c->qos_live) freq_qos_remove_request(&c->qos);
	c->qos_live = false;
	return 0;
}

/* Android M0 IPA-derived per-core dynamic powers in micro-Watts, not measured
 * package/battery power. Same v15/group8 OPPs as m86-opp.h. Native EM models
 * per-CPU cost; the old separate 4% shared-cluster heuristic is not duplicated.
 */
static const unsigned long atlas_power_uw[] = {
	231000, 267000, 314000, 374000, 430000, 490000, 569000, 639000, 731000, 832000, 958000, 1074000, 1227000, 1392000
};
static const unsigned long apollo_power_uw[] = {
	29000, 40000, 51000, 65000, 80000, 100000, 122000, 145000, 172000, 203000, 242000, 286000
};

static int m86_em_power(struct device *dev, unsigned long *power,
                       unsigned long *khz)
{
 int ci = cluster_index(dev->id);
 struct m86_cluster *c;
 const unsigned long *powers;
 unsigned int i;
 if (ci < 0) return ci;
 c = &coordinator->cluster[ci];
 powers = ci ? apollo_power_uw : atlas_power_uw;
 for (i = 0; i < c->nr; i++) {
  if (c->opp[i].khz < *khz) continue;
  *khz = c->opp[i].khz;
  *power = powers[i];
  return 0;
 }
 return -EINVAL;
}

static void m86_register_em(struct cpufreq_policy *policy)
{
 struct m86_cluster *c = policy->driver_data;
 struct em_data_callback cb = EM_DATA_CB(m86_em_power);
 struct em_perf_domain *pd = em_cpu_get(policy->cpu);
 unsigned int i;
 int ret;
 BUILD_BUG_ON(ARRAY_SIZE(atlas_power_uw) != ARRAY_SIZE(atlas_opp));
 BUILD_BUG_ON(ARRAY_SIZE(apollo_power_uw) != ARRAY_SIZE(apollo_opp));
 /* CPU EM domains survive policy destruction/recreation. Reuse only an
  * exactly matching domain, including the complete frequency/power table. */
 if (pd) {
  if (pd->nr_perf_states != c->nr ||
      !cpumask_equal(em_span_cpus(pd), policy->related_cpus))
   goto invalid_em;
  for (i = 0; i < c->nr; i++) {
   unsigned long khz = c->opp[i].khz, power;
   if (m86_em_power(get_cpu_device(policy->cpu), &power, &khz) ||
       pd->table[i].frequency != khz || pd->table[i].power != power)
    goto invalid_em;
  }
  return;
 }
 ret = em_dev_register_perf_domain(get_cpu_device(policy->cpu), c->nr,
                                  &cb, policy->related_cpus, true);
 if (ret) {
  mutex_lock(&coordinator->lock);
  fail_closed(coordinator, ret);
  mutex_unlock(&coordinator->lock);
  return;
 }
 dev_info(coordinator->dev,
          "EAS M0 uncalibrated per-core EM registered CPUs=%*pbl states=%u; native cpufreq frequency invariance\n",
          cpumask_pr_args(policy->related_cpus), c->nr);
 return;
invalid_em:
 mutex_lock(&coordinator->lock);
 fail_closed(coordinator, -EINVAL);
 mutex_unlock(&coordinator->lock);
}

static struct cpufreq_driver m86_cpufreq = {
	.name = "m86-coordinator",
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = m86_target, .get = m86_get,
	.init = m86_policy_init, .ready = m86_policy_ready,
	.register_em = m86_register_em,
	.exit = m86_policy_exit, .attr = cpufreq_generic_attr,
};

static void monitor(struct work_struct *work)
{
	struct m86_coord *d = container_of(to_delayed_work(work), struct m86_coord, thermal_work);
	unsigned int caps[2], i;
	int temp, ret;

	mutex_lock(&d->lock);
	ret = temperature(d, &temp);
	if (ret || temp >= 90000) {
		fail_closed(d, ret ?: -ERANGE);
		mutex_unlock(&d->lock);
		return;
	}
	/* 3C hysteresis: do not relax a cap until below its boundary-3000. */
	for (i = 0; i < 2; i++) {
		struct m86_cluster *c = &d->cluster[i];
		caps[i] = cap_for_temp(c, temp);
		if (caps[i] > c->thermal_cap &&
		    cap_for_temp(c, temp + 3000) < caps[i]) caps[i] = c->thermal_cap;
		if (caps[i] != c->thermal_cap)
			dev_info(d->dev, "thermal cap cluster=%u temp=%d old=%u new=%u\n", i, temp, c->thermal_cap, caps[i]);
		c->thermal_cap = caps[i];
	}
	mutex_unlock(&d->lock);
	cpus_read_lock();
	/* QoS callbacks can schedule policy work: never hold transition mutex. */
	for (i = 0; i < 2; i++) {
		struct m86_cluster *c = &d->cluster[i];
		unsigned int cpu;
		unsigned long actual_hz;
		int changed;

		if (!c->qos_live) continue;
		/* Publish reduced capacity to PELT/EAS as well as the frequency cap. */
		arch_update_thermal_pressure(&c->cpus, caps[i]);
		changed = freq_qos_update_request(&c->qos, caps[i]);
		if (changed < 0) {
			mutex_lock(&d->lock); fail_closed(d, changed); mutex_unlock(&d->lock);
			break;
		}
		/* Userspace .limits() ignores target errors. A fresh-temperature
		 * EAGAIN can reject its first throttle, then the next monitor may
		 * publish the same cap (QoS=0, no notifier). Retry enforcement while
		 * actual clock still exceeds cap, without holding transition mutex.
		 * cpus_read_lock pins policy lifetime; core acquires policy->rwsem.
		 */
		cpu = cpumask_any_and(&c->cpus, cpu_online_mask);
		if (!changed && cpu < nr_cpu_ids &&
		    clk_get_rate(c->clk) > caps[i] * 1000UL) {
			dev_info_ratelimited(d->dev, "thermal cap retry cluster=%u cap=%u\n", i, caps[i]);
			cpufreq_update_policy(cpu);
			/* Refresh is void and userspace limits ignores target status.
			 * Check real hardware again; unchanged QoS retries next poll.
			 * Never relax a cap or report success from policy metadata.
			 */
			actual_hz = clk_get_rate(c->clk);
			if (actual_hz > caps[i] * 1000UL)
				dev_warn_ratelimited(d->dev,
					"thermal cap still exceeded cluster=%u actual_hz=%lu cap=%u; retry next poll\n",
					i, actual_hz, caps[i]);
		}
	}
	cpus_read_unlock();
	if (!d->faulted) schedule_delayed_work(&d->thermal_work, msecs_to_jiffies(250));
}

static void put_zone(void *data)
{
	struct thermal_zone_device *zone = data;
	put_device(&zone->device);
}
static void release_clock_exclusive(void *data)
{
	clk_rate_exclusive_put(data);
}

static int link_clock_supplier(struct platform_device *pdev, unsigned int index)
{
	struct of_phandle_args args;
	struct platform_device *supplier;
	int ret;

	ret = of_parse_phandle_with_args(pdev->dev.of_node, "clocks", "#clock-cells", index, &args);
	if (ret) return ret;
	supplier = of_find_device_by_node(args.np);
	of_node_put(args.np);
	if (!supplier) return -EPROBE_DEFER;
	if (!device_link_add(&pdev->dev, &supplier->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) ret = -EINVAL;
	put_device(&supplier->dev);
	return ret;
}

static int m86_probe(struct platform_device *pdev)
{
	struct m86_coord *d;
	struct device_node *node;
	struct platform_device *supplier;
	unsigned int cpu, i, j;
	u32 bus_floor, ceilings[2];
	int temp, ret;

	if (coordinator || !of_machine_is_compatible("meizu,m86") ||
	    !of_property_read_bool(pdev->dev.of_node, "meizu,cpu-dvfs-active"))
		return dev_err_probe(&pdev->dev, -ENODEV, "bootstrap singleton/machine/activation at source line %u\n", __LINE__);
	if (!IS_ENABLED(CONFIG_CPU_FREQ) || !IS_ENABLED(CONFIG_CPU_FREQ_THERMAL) ||
	    !IS_ENABLED(CONFIG_ENERGY_MODEL))
		return dev_err_probe(&pdev->dev, -EOPNOTSUPP, "bootstrap CPU_FREQ/CPU_FREQ_THERMAL config at source line %u\n", __LINE__);
	bool has_fixed_floor = !of_property_read_u32(pdev->dev.of_node, "meizu,validated-fixed-bus-qos-floor", &bus_floor);
	if (!has_fixed_floor)
		bus_floor = 100000;
	if (of_property_read_u32_array(pdev->dev.of_node, "meizu,max-validated-khz", ceilings, 2))
		return dev_err_probe(&pdev->dev, -EINVAL, "bootstrap ceiling DT calibration at source line %u\n", __LINE__);
	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d) return dev_err_probe(&pdev->dev, -ENOMEM, "bootstrap gate at source line %u\n", __LINE__);
	d->dev = &pdev->dev;
	d->required_bus_qos = bus_floor;
	ret = m86_asv_cal_read(&d->asv_cal);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "bootstrap CPU ASV eFuse read/version at source line %u\n", __LINE__);
	dev_info(d->dev, "CPU ASV CAL: ver=%u type=%u atlas_grp=0x%03x apollo_grp=0x%03x ssa0_en=%u ssa1_en=%u\n",
		 d->asv_cal.effective_ver, d->asv_cal.asv_group_type,
		 m86_asv_domain_group_raw(&d->asv_cal, M86_ASV_BIG),
		 m86_asv_domain_group_raw(&d->asv_cal, M86_ASV_LIT),
		 d->asv_cal.ssa0_enable, d->asv_cal.ssa1_enable);
	mutex_init(&d->lock);
	mutex_init(&d->bus_lock);
	for (i = 0; i < 2; i++)
		mutex_init(&d->cluster[i].lock);
	INIT_DELAYED_WORK(&d->thermal_work, monitor);
	d->fin = devm_clk_get(&pdev->dev, "fin");
	if (IS_ERR(d->fin)) return dev_err_probe(&pdev->dev, PTR_ERR(d->fin), "bootstrap fin clock at source line %u\n", __LINE__);
	d->topc = devm_ioremap(&pdev->dev, 0x10570000, 0x1000);
	if (!d->topc) return dev_err_probe(&pdev->dev, -ENOMEM, "bootstrap TOPC mapping at source line %u\n", __LINE__);
	d->ema = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "meizu,ema-syscon");
	if (IS_ERR(d->ema)) return dev_err_probe(&pdev->dev, PTR_ERR(d->ema), "bootstrap EMA syscon at source line %u\n", __LINE__);
	/* Attach to Exynos Bus Device PM QoS provider if specified or present */
	node = of_parse_phandle(pdev->dev.of_node, "samsung,bus", 0);
	if (node) {
		supplier = of_find_device_by_node(node);
		of_node_put(node);
		if (!supplier)
			return dev_err_probe(&pdev->dev, -EPROBE_DEFER, "bootstrap bus supplier at source line %u\n", __LINE__);
		if (!device_link_add(&pdev->dev, &supplier->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
			put_device(&supplier->dev);
			return dev_err_probe(&pdev->dev, -EINVAL, "bootstrap bus device link at source line %u\n", __LINE__);
		}
		d->bus_dev = &supplier->dev;
		put_device(&supplier->dev);
	}
	if (!d->bus_dev)
		d->bus_dev = exynos_bus_get_dev("bus-mif");
	if (!d->bus_dev)
		d->bus_dev = exynos_bus_get_dev(NULL);
	if (!d->bus_dev && has_fixed_floor && bus_floor < 828000)
		return dev_err_probe(&pdev->dev, -EINVAL, "bootstrap static bus floor too low without bus QoS at source line %u\n", __LINE__);
	/* Supplier link ensures consumer removal precedes TMU resource teardown. */
	node = of_parse_phandle(pdev->dev.of_node, "meizu,thermal-monitor", 0);
	if (!node) return dev_err_probe(&pdev->dev, -EINVAL, "bootstrap thermal monitor phandle at source line %u\n", __LINE__);
	supplier = of_find_device_by_node(node);
	of_node_put(node);
	if (!supplier) return dev_err_probe(&pdev->dev, -EPROBE_DEFER, "bootstrap thermal supplier at source line %u\n", __LINE__);
	if (!device_link_add(&pdev->dev, &supplier->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(&supplier->dev);
		return dev_err_probe(&pdev->dev, -EINVAL, "bootstrap thermal device link at source line %u\n", __LINE__);
	}
	put_device(&supplier->dev);
	d->zone = thermal_zone_get_zone_by_name("m86-tmu-max");
	if (IS_ERR(d->zone)) return dev_err_probe(&pdev->dev, -EPROBE_DEFER, "bootstrap thermal zone at source line %u\n", __LINE__);
	get_device(&d->zone->device);
	ret = devm_add_action_or_reset(&pdev->dev, put_zone, d->zone);
	if (ret) return dev_err_probe(&pdev->dev, ret, "bootstrap gate at source line %u\n", __LINE__);
	ret = temperature(d, &temp);
	if (ret || temp >= 85000) return dev_err_probe(&pdev->dev, ret ?: -ERANGE, "bootstrap temperature or fixed MIF guard at source line %u\n", __LINE__);
	/* Require measured unit handoff controls; shared SYSREG reads only. */
	{
		u32 apollo, l1, l2;
		ret = regmap_read(d->ema, 0x38, &apollo);
		if (ret) return dev_err_probe(&pdev->dev, ret, "bootstrap gate at source line %u\n", __LINE__);
		ret = regmap_read(d->ema, 0x138, &l1);
		if (ret) return dev_err_probe(&pdev->dev, ret, "bootstrap gate at source line %u\n", __LINE__);
		ret = regmap_read(d->ema, 0x2908, &l2);
		if (ret) return dev_err_probe(&pdev->dev, ret, "bootstrap gate at source line %u\n", __LINE__);
		dev_info(d->dev, "bootstrap EMA apollo=%08x atlas_l1=%08x atlas_l2=%08x\n", apollo, l1, l2);
		if ((apollo & ~((7U << 20) | (7U << 8))) != 0x010011 ||
		    (l1 & ~(15U << 4)) != 0x2 || (l2 & ~(15U << 8)) != 0x72)
			return dev_err_probe(&pdev->dev, -EOPNOTSUPP, "bootstrap boot EMA nonfield controls at source line %u\n", __LINE__);
	}
	for_each_possible_cpu(cpu) {
		node = of_get_cpu_node(cpu, NULL);
		if (!node) return dev_err_probe(&pdev->dev, -EINVAL, "bootstrap CPU DT node at source line %u\n", __LINE__);
		if (of_device_is_compatible(node, "arm,cortex-a57")) i = 0;
		else if (of_device_is_compatible(node, "arm,cortex-a53")) i = 1;
		else { of_node_put(node); return dev_err_probe(&pdev->dev, -EINVAL, "bootstrap CPU DT compatible at source line %u\n", __LINE__); }
		cpumask_set_cpu(cpu, &d->cluster[i].cpus);
		of_node_put(node);
	}
	for (i = 0; i < 2; i++) {
		struct m86_cluster *c = &d->cluster[i];
		const struct m86_opp *tmpl = i ? apollo_opp : atlas_opp;
		unsigned int total_nr = i ? ARRAY_SIZE(apollo_opp) : ARRAY_SIZE(atlas_opp);
		struct m86_opp *dyn_opp;
		const char *name = i ? "apollo" : "atlas";
		int uv;
		bool known = false;

		if (cpumask_empty(&c->cpus)) return dev_err_probe(&pdev->dev, -ENODEV, "bootstrap empty cluster at source line %u\n", __LINE__);
		dyn_opp = devm_kcalloc(&pdev->dev, total_nr, sizeof(*dyn_opp), GFP_KERNEL);
		if (!dyn_opp) return dev_err_probe(&pdev->dev, -ENOMEM, "bootstrap dynamic OPP alloc at source line %u\n", __LINE__);
		for (j = 0; j < total_nr; j++) {
			dyn_opp[j] = tmpl[j];
			if (i == 0) {
				/* Atlas: j=0 is 800MHz (vendor L17, row 13), j=13 is 2100MHz (vendor L4, row 0) */
				u32 row = 13U - j;
				u32 vendor_lv = 17U - j;

				dyn_opp[j].uv = m86_asv_lookup_volt(&d->asv_cal, M86_ASV_BIG,
								    vendor_lv,
								    m86_big_asv_volt_v15[row]);
			} else {
				/* Apollo: j=0 is 400MHz (vendor L16, row 11), j=11 is 1500MHz (vendor L5, row 0) */
				u32 row = 11U - j;
				u32 vendor_lv = 16U - j;

				dyn_opp[j].uv = m86_asv_lookup_volt(&d->asv_cal, M86_ASV_LIT,
								    vendor_lv,
								    m86_lit_asv_volt_v15[row]);
			}
		}
		c->opp = dyn_opp;
		c->nr = total_nr;
		c->max_khz = ceilings[i];
		while (c->nr && c->opp[c->nr - 1].khz > c->max_khz) c->nr--;
		if (!c->nr || c->opp[c->nr - 1].khz != c->max_khz) return dev_err_probe(&pdev->dev, -EINVAL, "bootstrap validated ceiling absent from OPP table at source line %u\n", __LINE__);
		dev_info(d->dev, "CPU ASV %s table: %u..%u kHz -> %u..%u uV (%u states)\n",
			 name, c->opp[0].khz, c->opp[c->nr - 1].khz,
			 c->opp[0].uv, c->opp[c->nr - 1].uv, c->nr);
		if (!d->bus_dev) {
			for (j = 0; j < c->nr; j++)
				if (c->opp[j].bus_qos > bus_floor) return dev_err_probe(&pdev->dev, -EINVAL, "bootstrap OPP bus vote exceeds fixed MIF floor at source line %u\n", __LINE__);
		}
		ret = link_clock_supplier(pdev, i);
		if (ret) return dev_err_probe(&pdev->dev, ret, "bootstrap clock supplier link at source line %u\n", __LINE__);
		c->clk = devm_clk_get(&pdev->dev, name);
		if (IS_ERR(c->clk)) return dev_err_probe(&pdev->dev, PTR_ERR(c->clk), "bootstrap cluster clock at source line %u\n", __LINE__);
		if (clk_round_rate(c->clk, c->max_khz * 1000UL) != c->max_khz * 1000UL)
			return dev_err_probe(&pdev->dev, -EOPNOTSUPP, "bootstrap clock round_rate ceiling at source line %u\n", __LINE__);
		ret = clk_rate_exclusive_get(c->clk);
		if (ret) return dev_err_probe(&pdev->dev, ret, "bootstrap clock exclusive ownership at source line %u\n", __LINE__);
		ret = devm_add_action_or_reset(&pdev->dev, release_clock_exclusive, c->clk);
		if (ret) return dev_err_probe(&pdev->dev, ret, "bootstrap gate at source line %u\n", __LINE__);
		node = of_parse_phandle(pdev->dev.of_node, i ? "apollo-supply" : "atlas-supply", 0);
		if (!node) return dev_err_probe(&pdev->dev, -EINVAL, "bootstrap CPU supply phandle at source line %u\n", __LINE__);
		ret = of_property_read_bool(node, "meizu,cpu-dvfs-active");
		of_node_put(node);
		if (!ret) return dev_err_probe(&pdev->dev, -EOPNOTSUPP, "bootstrap CPU rail active DT gate at source line %u\n", __LINE__);
		c->supply = devm_regulator_get(&pdev->dev, name);
		if (IS_ERR(c->supply)) return dev_err_probe(&pdev->dev, PTR_ERR(c->supply), "bootstrap CPU regulator at source line %u\n", __LINE__);
		uv = regulator_get_voltage(c->supply);
		if (uv <= 0) return dev_err_probe(&pdev->dev, uv ?: -EIO, "bootstrap CPU voltage readback at source line %u\n", __LINE__);
		c->cur_uv = uv;
		c->cur_khz = clk_get_rate(c->clk) / 1000;
		/* No registration-time voltage/clock/EMA write. Require known boot OPP. */
		for (j = 0; j < c->nr; j++)
			if (clk_get_rate(c->clk) == c->opp[j].khz * 1000UL && uv == c->opp[j].uv) {
				known = true;
				c->cur_bus_qos = c->opp[j].bus_qos;
			}
		if (!known) dev_err(d->dev, "bootstrap cluster=%s unknown clock=%lu uv=%d\n", name, clk_get_rate(c->clk), uv);
		if (!known) return dev_err_probe(&pdev->dev, -EOPNOTSUPP, "bootstrap unknown boot frequency/voltage OPP at source line %u\n", __LINE__);
		/* Boot EMA must already match the fused/default vendor policy. */
		{
			u32 a, b;
			if (i) {
				u32 apollo_ema = m86_asv_get_apollo_ema(&d->asv_cal, uv);

				ret = regmap_read(d->ema, 0x38, &a);
				if (ret) return dev_err_probe(&pdev->dev, ret, "bootstrap gate at source line %u\n", __LINE__);
				if ((a & ((7U << 20) | (7U << 8))) != ((apollo_ema << 20) | (apollo_ema << 8)))
					return dev_err_probe(&pdev->dev, -EOPNOTSUPP, "bootstrap APOLLO boot EMA band at source line %u\n", __LINE__);
			} else {
				u32 l1, l2;

				m86_asv_get_atlas_ema(&d->asv_cal, uv, &l1, &l2);
				ret = regmap_read(d->ema, 0x138, &a);
				if (ret) return dev_err_probe(&pdev->dev, ret, "bootstrap gate at source line %u\n", __LINE__);
				ret = regmap_read(d->ema, 0x2908, &b);
				if (ret) return dev_err_probe(&pdev->dev, ret, "bootstrap gate at source line %u\n", __LINE__);
				if ((a & (15U << 4)) != (l1 << 4) || (b & (15U << 8)) != (l2 << 8))
					return dev_err_probe(&pdev->dev, -EOPNOTSUPP, "bootstrap ATLAS boot EMA band at source line %u\n", __LINE__);
			}
		}
		c->table = devm_kcalloc(&pdev->dev, c->nr + 1, sizeof(*c->table), GFP_KERNEL);
		if (!c->table) return dev_err_probe(&pdev->dev, -ENOMEM, "bootstrap gate at source line %u\n", __LINE__);
		for (j = 0; j < c->nr; j++) c->table[j].frequency = c->opp[j].khz;
		c->table[c->nr].frequency = CPUFREQ_TABLE_END;
		c->thermal_cap = cap_for_temp(c, temp);
	}
	d->cur_bus_qos = max(d->cluster[0].cur_bus_qos, d->cluster[1].cur_bus_qos);
	if (!d->cur_bus_qos)
		d->cur_bus_qos = bus_floor;
	d->required_bus_qos = d->cur_bus_qos;

	if (d->bus_dev) {
		ret = dev_pm_qos_add_request(d->bus_dev, &d->bus_qos_req,
					     DEV_PM_QOS_MIN_FREQUENCY, d->cur_bus_qos);
		if (ret >= 0) {
			d->bus_qos_active = true;
			dev_info(d->dev, "Device PM QoS request registered on %s: initial floor %u kHz\n",
				 dev_name(d->bus_dev), d->cur_bus_qos);
		} else {
			dev_warn(d->dev, "Device PM QoS add request failed (%d)\n", ret);
		}
	}
	coordinator = d;
	platform_set_drvdata(pdev, d);
	ret = cpufreq_register_driver(&m86_cpufreq);
	if (ret) {
		if (d->bus_qos_active) {
			dev_pm_qos_remove_request(&d->bus_qos_req);
			d->bus_qos_active = false;
		}
		coordinator = NULL;
		return dev_err_probe(&pdev->dev, ret, "bootstrap cpufreq registration at source line %u\n", __LINE__);
	}
	d->registered = true;
	schedule_delayed_work(&d->thermal_work, 0);
	return 0;
}

static int m86_remove(struct platform_device *pdev)
{
	struct m86_coord *d = platform_get_drvdata(pdev);
	int atlas_ret, apollo_ret;
	cancel_delayed_work_sync(&d->thermal_work);
	mutex_lock(&d->lock);
	d->faulted = true;
	atlas_ret = set_rate_checked(&d->cluster[0], d->cluster[0].opp[0].khz);
	apollo_ret = set_rate_checked(&d->cluster[1], d->cluster[1].opp[0].khz);
	if ((atlas_ret && atlas_ret != -EHOSTDOWN) ||
	    (apollo_ret && apollo_ret != -EHOSTDOWN))
		orderly_poweroff(true);
	mutex_unlock(&d->lock);
	if (d->registered) cpufreq_unregister_driver(&m86_cpufreq);
	if (d->bus_qos_active) {
		dev_pm_qos_remove_request(&d->bus_qos_req);
		d->bus_qos_active = false;
	}
	coordinator = NULL;
	return 0;
}
static const struct of_device_id m86_match[] = {
	{ .compatible = "meizu,m86-cpu-coordinator" }, { }
};
MODULE_DEVICE_TABLE(of, m86_match);
static struct platform_driver m86_driver = {
	.probe = m86_probe, .remove = m86_remove,
	.driver = { .name = "m86-cpu-coordinator", .of_match_table = m86_match },
};
module_platform_driver(m86_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Review candidate m86 DVFS with staged voltage/EMA and fail-closed thermal QoS");
