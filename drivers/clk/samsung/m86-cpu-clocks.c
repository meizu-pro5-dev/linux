// SPDX-License-Identifier: GPL-2.0-only
/* Review candidate: no writes until an explicit clk_set_rate request. */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/clk-provider.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define PLL_MASK ((0x3ff << 16) | (0x3f << 8) | 7)
#define POLL_US 10000
struct m86_rate { unsigned long hz; u32 div0, div1, pms; };
#include "m86-rate-tables.h"
struct m86_cpuclk {
	struct clk_hw hw;
	struct device *dev;
	void __iomem *base, *topc;
	struct clk *safe, *fin;
	const struct m86_rate *rates;
	unsigned int nr_rates, shift;
	bool atlas, failed;
	unsigned long ceiling;
	cpumask_t cpus;
	struct mutex lock;
	struct hlist_node hotplug_node;
	bool online;
};
#define to_m86(hw) container_of(hw, struct m86_cpuclk, hw)

static unsigned long pll_rate(u32 con, unsigned long fin)
{
	u32 p = (con >> 8) & 0x3f;
	if (!p || !(con & BIT(31)) || !(con & BIT(29)))
		return 0;
	return div_u64((u64)fin * ((con >> 16) & 0x3ff), p << (con & 7));
}

/* Verify TOPC CPU route against independently decoded BUS0, not a/b gates. */
static unsigned long safe_rate(struct m86_cpuclk *c)
{
	u32 sel = (readl(c->topc + 0x204) >> c->shift) & 3;
	u32 stat = (readl(c->topc + 0x404) >> c->shift) & 0xf;
	unsigned long rate;

	if (sel > 1 || stat != sel || !(readl(c->topc + 0x200) & 1) ||
	    (readl(c->topc + 0x400) & 7) != 1)
		return 0;
	rate = pll_rate(readl(c->topc + 0x110), clk_get_rate(c->fin));
	rate >>= sel;
	return rate == clk_get_rate(c->safe) ? rate : 0;
}

static bool accessible(struct m86_cpuclk *c)
{
	return c->online;
}

/* Caller holds provider mutex; CPUHP callbacks gate cluster accessibility. */
static unsigned long current_rate(struct m86_cpuclk *c)
{
	u32 div, root;
	unsigned long rate;
	if (!accessible(c))
		return 0;
	if (readl(c->base + 0x700) || readl(c->base + 0x704))
		return 0;
	root = readl(c->base + 0x408) & 7;
	if (root != (readl(c->base + 0x208) & 1))
		return 0;
	if (!root) {
		if (!(readl(c->base + 0x200) & 1) ||
		    (readl(c->base + 0x400) & 7) != 1)
			return 0;
		rate = pll_rate(readl(c->base + 0x100), clk_get_rate(c->fin));
	} else if (root == 1 && (readl(c->base + 0x204) & 1) &&
		   (readl(c->base + 0x404) & 7) == 1) {
		rate = safe_rate(c);
	} else {
		return 0;
	}
	div = readl(c->base + 0x600);
	return rate / ((div & 0xf) + 1) / (((div >> 4) & 0xf) + 1);
}

static unsigned long m86_recalc(struct clk_hw *hw, unsigned long parent_rate)
{
	struct m86_cpuclk *c = to_m86(hw);
	unsigned long rate;
	mutex_lock(&c->lock);
	rate = c->failed ? 0 : current_rate(c);
	mutex_unlock(&c->lock);
	return rate;
}

static long m86_round(struct clk_hw *hw, unsigned long request, unsigned long *parent)
{
	struct m86_cpuclk *c = to_m86(hw);
	unsigned long best = 0;
	unsigned int i;
	for (i = 0; i < c->nr_rates; i++)
		if (c->rates[i].hz <= request && c->rates[i].hz <= c->ceiling)
			best = max(best, c->rates[i].hz);
	return best ? best : -EINVAL;
}

static int wait_clear(struct m86_cpuclk *c, u32 offset, u32 mask)
{
	u32 val;
	return readl_poll_timeout(c->base + offset, val, !(val & mask), 1, POLL_US);
}

static int set_divs(struct m86_cpuclk *c, u32 d0, u32 d1)
{
	int ret;
	writel(d0, c->base + 0x600);
	ret = wait_clear(c, 0x700, c->atlas ? 0x4101111 : 0x1111111);
	if (ret || readl(c->base + 0x600) != d0)
		return ret ?: -EIO;
	writel(d1, c->base + 0x604);
	ret = wait_clear(c, 0x704, c->atlas ? 0x111 : 0x11);
	if (ret || readl(c->base + 0x604) != d1)
		return ret ?: -EIO;
	return 0;
}

static int select_root(struct m86_cpuclk *c, u32 which)
{
	u32 val, original = readl(c->base + 0x208);
	writel((original & ~1U) | which, c->base + 0x208);
	return readl_poll_timeout(c->base + 0x408, val, (val & 7) == which,
				 1, POLL_US);
}

static int program_pll(struct m86_cpuclk *c, u32 pms)
{
	u32 con = readl(c->base + 0x100), val, second;
	u32 lock_cycles = ((pms >> 8) & 0x3f) * 150;
	u32 settle_us = DIV_ROUND_UP(lock_cycles, 24U) + 10U;
	int ret;

	writel(lock_cycles, c->base + 0x000);
	writel((con & ~PLL_MASK) | pms, c->base + 0x100);
	/* Flush writes, then allow a complete programmed 24MHz lock interval.
	 * Do not require observing a possibly very short LOCK deassertion.
	 * Interrupts remain enabled; this callback runs under a mutex.
	 */
	val = readl(c->base + 0x100);
	if ((val & PLL_MASK) != pms)
		return -EIO;
	usleep_range(settle_us, settle_us + 10);
	ret = readl_poll_timeout(c->base + 0x100, val,
		(val & PLL_MASK) == pms && (val & BIT(29)), 1, POLL_US);
	if (ret)
		return ret;
	usleep_range(2, 5);
	second = readl(c->base + 0x100);
	return second == val && (second & BIT(31)) ? 0 : -EIO;
}

/* Field-wise maximum never speeds up a divider while its root is unchanged. */
static u32 max_fields(u32 a, u32 b, const u8 *shifts, unsigned int n)
{
	unsigned int i;
	for (i = 0; i < n; i++) {
		u32 shift = shifts[i], mask = 7U << shift;
		a = (a & ~mask) | (max((a >> shift) & 7, (b >> shift) & 7) << shift);
	}
	return a;
}

static int m86_set(struct clk_hw *hw, unsigned long hz, unsigned long parent)
{
	static const u8 atlas_fields[] = { 0, 4, 8, 12, 20, 26 };
	static const u8 apollo_fields[] = { 0, 4, 8, 12, 16, 20, 24 };
	static const u8 div1_fields[] = { 0, 4, 8 };
	struct m86_cpuclk *c = to_m86(hw);
	const struct m86_rate *target = NULL;
	u32 old0, old1, temp0, temp1, target0, target1, core_div;
	unsigned long old, safe;
	unsigned int i;
	int ret = -EINVAL;
	for (i = 0; i < c->nr_rates; i++)
		if (c->rates[i].hz == hz)
			target = &c->rates[i];
	if (!target || hz > c->ceiling || c->failed)
		return c->failed ? -EIO : -EINVAL;
	mutex_lock(&c->lock);
	/* A fully offline cluster is inaccessible, not a poisoned PLL. */
	if (!accessible(c)) { ret = -EHOSTDOWN; goto out; }
	old = current_rate(c);
	safe = safe_rate(c);
	if (!old || !safe || (readl(c->base + 0x408) & 7) ||
	    !(readl(c->base + 0x204) & 1) ||
	    (readl(c->base + 0x404) & 7) != 1)
		goto out;
	if (hz == old) { ret = 0; goto out; }
	if (clk_get_rate(c->fin) != 24000000) { ret = -EINVAL; goto out; }
	old0 = readl(c->base + 0x600);
	old1 = readl(c->base + 0x604);
	/* Refuse reserved bit3 in each core divider or an unrepresentable bypass. */
	if (old0 & 0x88) { ret = -EINVAL; goto out; }
	core_div = DIV_ROUND_UP(safe, min(old, hz));
	if (!core_div || core_div > 8) { ret = -ERANGE; goto out; }
	/* Preserve non-divider bits; vendor masks are not whole-register authority. */
	target0 = old0;
	for (i = 0; i < (c->atlas ? ARRAY_SIZE(atlas_fields) : ARRAY_SIZE(apollo_fields)); i++) {
		u32 s = c->atlas ? atlas_fields[i] : apollo_fields[i];
		target0 = (target0 & ~(7U << s)) | (target->div0 & (7U << s));
	}
	target1 = old1;
	for (i = 0; i < (c->atlas ? 3 : 2); i++)
		target1 = (target1 & ~(7U << (i * 4))) | (target->div1 & (7U << (i * 4)));
	temp0 = max_fields(old0, target0,
		c->atlas ? atlas_fields : apollo_fields,
		c->atlas ? ARRAY_SIZE(atlas_fields) : ARRAY_SIZE(apollo_fields));
	temp0 = (temp0 & ~7U) | max(temp0 & 7, core_div - 1);
	temp1 = max_fields(old1, target1, div1_fields, c->atlas ? 3 : 2);
	ret = set_divs(c, temp0, temp1);
	if (ret) goto failed;
	ret = select_root(c, 1);
	if (ret) goto failed;
	/* Core on BUS0 <= min(old,new); auxiliary dividers never reduced here. */
	ret = program_pll(c, target->pms);
	if (ret) goto failed;
	ret = select_root(c, 0);
	if (ret) goto failed;
	ret = set_divs(c, target0, target1);
	if (ret) goto failed;
	if (current_rate(c) != hz) { ret = -EIO; goto failed; }
	goto out;
failed:
	/* Never guess a rollback rate. Preserve observed state and prohibit retries. */
	c->failed = true;
	dev_err(c->dev, "transition failed=%d; observed_hz=%lu; later changes locked\n",
		ret, current_rate(c));
out:
	mutex_unlock(&c->lock);
	return ret;
}

static const struct clk_ops m86_ops = {
	.recalc_rate = m86_recalc, .round_rate = m86_round, .set_rate = m86_set,
};

/* Callbacks never invoke CCF, avoiding prepare_lock / CPU hotplug ABBA. */
static int m86_hotplug_state;
static int m86_cpu_online(unsigned int cpu, struct hlist_node *node)
{
	struct m86_cpuclk *c = container_of(node, struct m86_cpuclk, hotplug_node);
	if (cpumask_test_cpu(cpu, &c->cpus)) {
		mutex_lock(&c->lock);
		c->online = true;
		mutex_unlock(&c->lock);
	}
	return 0;
}

static int m86_cpu_offline(unsigned int cpu, struct hlist_node *node)
{
	struct m86_cpuclk *c = container_of(node, struct m86_cpuclk, hotplug_node);
	cpumask_t remaining;
	if (cpumask_test_cpu(cpu, &c->cpus)) {
		mutex_lock(&c->lock);
		cpumask_and(&remaining, &c->cpus, cpu_online_mask);
		cpumask_clear_cpu(cpu, &remaining);
		c->online = !cpumask_empty(&remaining);
		mutex_unlock(&c->lock);
	}
	return 0;
}

static void m86_remove_hotplug(void *data)
{
	struct m86_cpuclk *c = data;
	cpuhp_state_remove_instance(m86_hotplug_state, &c->hotplug_node);
}

static int m86_probe(struct platform_device *pdev)
{
	struct m86_cpuclk *c;
	struct clk_init_data init = { .ops = &m86_ops, .flags = CLK_GET_RATE_NOCACHE };
	unsigned int cpu;
	int ret;
	u32 ceiling;
	if (!of_machine_is_compatible("meizu,m86")) return -ENODEV;
	c = devm_kzalloc(&pdev->dev, sizeof(*c), GFP_KERNEL);
	if (!c) return -ENOMEM;
	c->dev = &pdev->dev;
	mutex_init(&c->lock);
	c->atlas = of_device_is_compatible(pdev->dev.of_node, "meizu,m86-atlas-clock");
	c->rates = c->atlas ? atlas_rates : apollo_rates;
	c->nr_rates = c->atlas ? ARRAY_SIZE(atlas_rates) : ARRAY_SIZE(apollo_rates);
	c->shift = c->atlas ? 4 : 8;
	for_each_possible_cpu(cpu) {
		struct device *dev = get_cpu_device(cpu);
		if (dev && dev->of_node && of_device_is_compatible(dev->of_node,
			c->atlas ? "arm,cortex-a57" : "arm,cortex-a53"))
			cpumask_set_cpu(cpu, &c->cpus);
	}
	if (cpumask_empty(&c->cpus)) return -ENODEV;
	ret = cpuhp_state_add_instance(m86_hotplug_state, &c->hotplug_node);
	if (ret) return ret;
	ret = devm_add_action_or_reset(&pdev->dev, m86_remove_hotplug, c);
	if (ret) return ret;
	if (!accessible(c)) return -EPROBE_DEFER;
	c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(c->base)) return PTR_ERR(c->base);
	c->topc = devm_ioremap(&pdev->dev, 0x10570000, 0x1000);
	if (!c->topc) return -ENOMEM;
	c->safe = devm_clk_get(&pdev->dev, "safe");
	if (IS_ERR(c->safe)) return PTR_ERR(c->safe);
	c->fin = devm_clk_get(&pdev->dev, "fin");
	if (IS_ERR(c->fin)) return PTR_ERR(c->fin);
	ret = of_property_read_u32(pdev->dev.of_node,
		"meizu,max-validated-frequency", &ceiling);
	if (ret || !ceiling) return -EINVAL;
	c->ceiling = ceiling;
	init.name = c->atlas ? "m86_atlas_cpu" : "m86_apollo_cpu";
	c->hw.init = &init;
	ret = devm_clk_hw_register(&pdev->dev, &c->hw);
	if (ret) return ret;
	return devm_of_clk_add_hw_provider(&pdev->dev, of_clk_hw_simple_get, &c->hw);
}
static const struct of_device_id m86_matches[] = {
	{ .compatible = "meizu,m86-atlas-clock" },
	{ .compatible = "meizu,m86-apollo-clock" }, { }
};
MODULE_DEVICE_TABLE(of, m86_matches);
static struct platform_driver m86_driver = {
	.probe = m86_probe,
	.driver = { .name = "m86-cpu-clocks", .of_match_table = m86_matches },
};
static int __init m86_clocks_init(void)
{
	int ret;
	m86_hotplug_state = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN,
		"m86/cpu-clocks:online", m86_cpu_online, m86_cpu_offline);
	if (m86_hotplug_state < 0)
		return m86_hotplug_state;
	ret = platform_driver_register(&m86_driver);
	if (ret)
		cpuhp_remove_multi_state(m86_hotplug_state);
	return ret;
}
static void __exit m86_clocks_exit(void)
{
	platform_driver_unregister(&m86_driver);
	cpuhp_remove_multi_state(m86_hotplug_state);
}
module_init(m86_clocks_init);
module_exit(m86_clocks_exit);
