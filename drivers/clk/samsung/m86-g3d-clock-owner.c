// SPDX-License-Identifier: GPL-2.0
#include <linux/err.h>
#include <linux/module.h>
#include "m86-g3d-clock-owner.h"
/* Matches new experimental TOPC binding id; no DT consumer is added here. */
#define M86_TOP_G3D_GATE_ID 29
static void release_top(void *arg)
{
 struct m86_g3d_top_gate *gate = arg;
 /* Built-in bootstrap lease: never shut an inherited running domain or
  * TOP leaf down during failed probe unwind. No unbind interface. */
 pr_warn("m86 G3D TOP bootstrap reference retained across probe cleanup (held=%u)\n",gate->held);
}
int m86_g3d_top_gate_init(struct device *dev, struct device_node *np,
                        struct m86_g3d_top_gate *gate)
{
 struct of_phandle_args spec = { .np = np, .args_count = 1,
                                 .args = { M86_TOP_G3D_GATE_ID } };
 if (!dev || !np || !gate)
  return -EINVAL;
 gate->clock = of_clk_get_from_provider(&spec);
 if (IS_ERR(gate->clock))
  return PTR_ERR(gate->clock);
 gate->held = false;
 mutex_init(&gate->lock);
 /* Initial domain is already on: own one reference before first off. */
 {
  int ret = clk_prepare_enable(gate->clock);
  if (ret) {
   clk_put(gate->clock);
   return ret;
  }
 }
 gate->held = true;
 return devm_add_action_or_reset(dev, release_top, gate);
}
int m86_g3d_top_gate_set(struct m86_g3d_top_gate *gate, bool enable)
{
 int ret = 0;
 if (!gate || IS_ERR_OR_NULL(gate->clock))
  return -EINVAL;
 mutex_lock(&gate->lock);
 if (enable && !gate->held) {
  ret = clk_prepare_enable(gate->clock);
  if (!ret)
   gate->held = true;
 } else if (!enable && gate->held) {
  clk_disable_unprepare(gate->clock);
  gate->held = false;
 }
 mutex_unlock(&gate->lock);
 /* Dropping our reference is not evidence the physical gate became zero. */
 return ret;
}
MODULE_LICENSE("GPL");
