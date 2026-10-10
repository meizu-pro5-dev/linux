/* SPDX-License-Identifier: GPL-2.0 */
#ifndef M86_G3D_CMU_CONTEXT_H
#define M86_G3D_CMU_CONTEXT_H
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/types.h>

/* Experimental caller owns this context; never expose it to userspace. */
struct m86_g3d_cmu_context {
	u32 div, lock, con0, con1, con2, mux, aclk, pclk, sclk;
	u32 selector, ema;
	unsigned long hz;
	bool valid;
};

struct m86_g3d_cmu_access {
	void __iomem *cmu;
	struct regmap *sysreg;
	struct mutex *transaction, *hardware;
	void *data;
	unsigned long cap;
	/* Both callbacks run with transaction AND hardware held. They must not
	 * call clk_get_rate/set_rate, runtime PM, devfreq or thermal get_temp.
	 * validate_rail checks the unchanged exact Normal ON fields/selector;
	 * it performs no PMIC writes and obtains the PMIC device lock itself.
	 */
	bool (*domain_on)(void *data);
	int (*validate_rail)(void *data, u32 selector);
};

/* Caller must additionally serialize every CCF user of these registers,
 * including divider/gate callbacks. These mutexes alone cannot do that.
 * Save occurs before provider off writes. Restore occurs after CFG/STATUS=f,
 * inside the TMU transition callback and before Panfrost reset/first job.
 * Any failure must reach provider abort/quarantine; never blindly retry.
 */
int m86_g3d_cmu_save(const struct m86_g3d_cmu_access *access,
		    struct m86_g3d_cmu_context *context, unsigned long hz,
		    u32 selector);
int m86_g3d_cmu_restore(const struct m86_g3d_cmu_access *access,
		       struct m86_g3d_cmu_context *context);
#endif
