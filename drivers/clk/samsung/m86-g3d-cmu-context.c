// SPDX-License-Identifier: GPL-2.0
/* Unregistered experiment: exact vendor saved-context replay, not new DVFS. */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/iopoll.h>
#include <linux/lockdep.h>
#include <linux/math64.h>
#include "m86-g3d-cmu-context.h"

#define PLL_LOCK 0x0000
#define PLL_CON0 0x0100
#define PLL_CON1 0x0104
#define PLL_CON2 0x0108
#define MUX 0x0200
#define MUX_STAT 0x0400
#define DIV 0x0600
#define DIV_STAT 0x0700
#define ACLK 0x0800
#define PCLK 0x0900
#define SCLK 0x0a00
#define PLL_ENABLED BIT(31)
#define PLL_LOCKED BIT(29)
#define FIN_SELECTED BIT(8)
#define PLL_SELECTED BIT(0)
#define MUX_BUSY (BIT(2) | BIT(10))
#define DIV_BUSY (BIT(0) | BIT(4) | BIT(8))
#define EMA 0x1224

static int access_valid(const struct m86_g3d_cmu_access *a)
{
	if (!a || !a->cmu || !a->sysreg || !a->transaction || !a->hardware ||
	    !a->domain_on || !a->validate_rail)
		return -EINVAL;
	lockdep_assert_held(a->transaction);
	lockdep_assert_held(a->hardware);
	return a->domain_on(a->data) ? 0 : -EHOSTDOWN;
}

static int pair_valid(const struct m86_g3d_cmu_access *a,
		      const struct m86_g3d_cmu_context *c)
{
	u32 m, p, s, expected_selector, expected_hpm, expected_pms;

	switch (c->hz) {
	case 266000000: expected_selector = 0x39; expected_pms = (133U << 16) | (6U << 8) | 1; break;
	case 350000000: expected_selector = 0x3a; expected_pms = (175U << 16) | (6U << 8) | 1; break;
	case 420000000: expected_selector = 0x3b; expected_pms = (105U << 16) | (3U << 8) | 1; break;
	case 544000000: expected_selector = 0x41; expected_pms = (68U << 16) | (3U << 8); break;
	case 600000000: expected_selector = 0x47; expected_pms = (75U << 16) | (3U << 8); break;
	case 700000000: expected_selector = 0x4e; expected_pms = (175U << 16) | (6U << 8); break;
	case 772000000: expected_selector = 0x54; expected_pms = (193U << 16) | (6U << 8); break;
	default: return -EOPNOTSUPP;
	}
	if (c->hz > a->cap || c->selector != expected_selector || c->ema != 0x31)
		return -EOPNOTSUPP;
	m = (c->con0 >> 16) & 1023;
	p = (c->con0 >> 8) & 63;
	s = c->con0 & 7;
	expected_hpm = c->hz >= 544000000 ? 2 : 1;
	if (!m || !p || !(c->con0 & PLL_ENABLED) || !(c->con0 & PLL_LOCKED) ||
	    !(c->mux & PLL_SELECTED) || (c->mux & FIN_SELECTED) ||
	    (c->div & 7) || ((c->div >> 8) & 3) != expected_hpm ||
	    !c->lock || c->lock > 24000 ||
	    (c->con0 & ((1023U << 16) | (63U << 8) | 7)) != expected_pms ||
	    div_u64(24000000ULL * m, p << s) != c->hz)
		return -EOPNOTSUPP;
	return a->validate_rail(a->data, c->selector);
}

static int write_exact(const struct m86_g3d_cmu_access *a, u32 offset, u32 value)
{
	writel(value, a->cmu + offset);
	return readl(a->cmu + offset) == value ? 0 : -EIO;
}

static int write_mux(const struct m86_g3d_cmu_access *a, u32 value)
{
	u32 status;
	int ret = write_exact(a, MUX, value);

	if (ret)
		return ret;
	return readl_poll_timeout(a->cmu + MUX_STAT, status,
				 !(status & MUX_BUSY) &&
				 (status & 7) == !!(value & PLL_SELECTED), 1, 10000);
}

int m86_g3d_cmu_save(const struct m86_g3d_cmu_access *a,
		    struct m86_g3d_cmu_context *c, unsigned long hz, u32 selector)
{
	int ret;

	if (!c)
		return -EINVAL;
	c->valid = false;
	ret = access_valid(a);
	if (ret)
		return ret;
	if ((readl(a->cmu + MUX_STAT) & MUX_BUSY) ||
	    (readl(a->cmu + DIV_STAT) & DIV_BUSY))
		return -EBUSY;
	/* All nine words match vendor save_list_g3d; unknown writable fields
	 * are replayed from this live snapshot, never invented constants. */
	c->div = readl(a->cmu + DIV);
	c->lock = readl(a->cmu + PLL_LOCK);
	c->con0 = readl(a->cmu + PLL_CON0);
	c->con1 = readl(a->cmu + PLL_CON1);
	c->con2 = readl(a->cmu + PLL_CON2);
	c->mux = readl(a->cmu + MUX);
	c->aclk = readl(a->cmu + ACLK);
	c->pclk = readl(a->cmu + PCLK);
	c->sclk = readl(a->cmu + SCLK);
	c->hz = hz;
	c->selector = selector;
	ret = regmap_read(a->sysreg, EMA, &c->ema);
	if (ret)
		return ret;
	ret = pair_valid(a, c);
	if (ret)
		return ret;
	/* Serialization guarantees coherence; final check rejects power loss. */
	if (!a->domain_on(a->data))
		return -EHOSTDOWN;
	c->valid = true;
	return 0;
}

int m86_g3d_cmu_restore(const struct m86_g3d_cmu_access *a,
		       struct m86_g3d_cmu_context *c)
{
	struct m86_g3d_cmu_context final;
	u32 value, p, delay, ema;
	int ret;

	if (!c || !c->valid)
		return -EINVAL;
	/* A failed partial restore cannot later reuse a stale saved context. */
	c->valid = false;
	ret = access_valid(a);
	if (ret)
		return ret;
	ret = pair_valid(a, c); /* Rail-only callback: no local rate required yet. */
	if (ret)
		return ret;
	/* Vendor composite.c defines PLL status OSC=0/PLL=1/CHANGE=4 at
	 * bits2:0; the core mux separately has busy bit10. Route core to FIN. */
	ret = write_mux(a, readl(a->cmu + MUX) | FIN_SELECTED);
	if (ret)
		return ret;
	/* PLL enable is documented; LOCKED is read-only status, not a desired
	 * configuration bit. Configure auxiliary words with the PLL disabled. */
	writel(c->con0 & ~(PLL_ENABLED | PLL_LOCKED), a->cmu + PLL_CON0);
	if ((readl(a->cmu + PLL_CON0) & ~PLL_LOCKED) !=
	    (c->con0 & ~(PLL_ENABLED | PLL_LOCKED)))
		return -EIO;
	ret = write_exact(a, PLL_CON1, c->con1);
	if (ret)
		return ret;
	ret = write_exact(a, PLL_CON2, c->con2);
	if (ret)
		return ret;
	ret = write_exact(a, PLL_LOCK, c->lock);
	if (ret)
		return ret;
	writel(c->con0 & ~PLL_LOCKED, a->cmu + PLL_CON0);
	p = (c->con0 >> 8) & 63;
	/* Wait the saved lock counter's full 24MHz interval, at least vendor's
	 * 150*P, then poll fresh lock and compare all configuration bits. */
	delay = DIV_ROUND_UP(max(c->lock, p * 150U), 24U) + 10;
	if (delay > 10000)
		return -ERANGE;
	usleep_range(delay, delay + 10);
	ret = readl_poll_timeout(a->cmu + PLL_CON0, value,
		(value & PLL_LOCKED) && (value & ~PLL_LOCKED) ==
		(c->con0 & ~PLL_LOCKED), 1, 10000);
	if (ret)
		return ret;
	/* Restore DIV while core still on FIN, wait every defined divider. */
	ret = write_exact(a, DIV, c->div);
	if (ret)
		return ret;
	ret = readl_poll_timeout(a->cmu + DIV_STAT, value,
				 !(value & DIV_BUSY), 1, 10000);
	if (ret)
		return ret;
	ret = write_mux(a, c->mux | FIN_SELECTED);
	if (ret)
		return ret;
	ret = write_mux(a, c->mux);
	if (ret)
		return ret;
	/* Full saved gate words, including provider-forced ACLK bit1, are
	 * replayed only after clocks are valid. CCF exclusivity is mandatory. */
	ret = write_exact(a, ACLK, c->aclk);
	if (ret)
		return ret;
	ret = write_exact(a, PCLK, c->pclk);
	if (ret)
		return ret;
	ret = write_exact(a, SCLK, c->sclk);
	if (ret)
		return ret;
	/* Vendor cal_set_ema writes the complete saved/validated 0x31 word.
	 * Cold reset EMA=0 is expected loss of context, not a rail violation. */
	ret = regmap_write(a->sysreg, EMA, c->ema);
	if (ret)
		return ret;
	ret = regmap_read(a->sysreg, EMA, &ema);
	if (ret || ema != 0x31)
		return ret ?: -EIO;
	if (!a->domain_on(a->data))
		return -EHOSTDOWN;
	final = *c;
	final.con0 = readl(a->cmu + PLL_CON0);
	final.div = readl(a->cmu + DIV);
	final.mux = readl(a->cmu + MUX);
	final.ema = ema;
	if (final.con0 != c->con0 || final.div != c->div || final.mux != c->mux ||
	    readl(a->cmu + PLL_CON1) != c->con1 ||
	    readl(a->cmu + PLL_CON2) != c->con2 ||
	    readl(a->cmu + PLL_LOCK) != c->lock ||
	    readl(a->cmu + ACLK) != c->aclk ||
	    readl(a->cmu + PCLK) != c->pclk ||
	    readl(a->cmu + SCLK) != c->sclk ||
	    (readl(a->cmu + MUX_STAT) & MUX_BUSY) ||
	    (readl(a->cmu + DIV_STAT) & DIV_BUSY))
		return -EIO;
	return pair_valid(a, &final); /* Real PLL/div/mux + unchanged rail. */
}
