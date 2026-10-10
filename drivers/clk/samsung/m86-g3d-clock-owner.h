/* SPDX-License-Identifier: GPL-2.0 */
#ifndef M86_G3D_CLOCK_OWNER_HELPER_H
#define M86_G3D_CLOCK_OWNER_HELPER_H
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
struct m86_g3d_top_gate {
 struct clk *clock;
 struct mutex lock;
 bool held;
};
/* np is the actual CMU_TOPC provider; experimental clock registration required. */
int m86_g3d_top_gate_init(struct device *dev, struct device_node *np,
                        struct m86_g3d_top_gate *gate);
/* Call outside coordinator hardware mutex: CCF prepare/enable locks are taken.
 * Coordinator transaction may surround this call if no CCF callback takes it.
 */
int m86_g3d_top_gate_set(struct m86_g3d_top_gate *gate, bool enable);
/* Direct owner API installed by clock-owner.patch. Caller holds transaction
 * and hardware; original local bit1 is preserved in the saved CMU context.
 * No CCF API inside this call. It uses the real CMU provider's spinlock.
 */
int m86_exynos7_g3d_lpi_clock(struct regmap *pmu, struct mutex *transaction,
                            struct mutex *hardware, bool *was_enabled);
#endif
