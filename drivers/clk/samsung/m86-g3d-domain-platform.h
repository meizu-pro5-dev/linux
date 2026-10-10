/* SPDX-License-Identifier: GPL-2.0 */
#ifndef M86_G3D_DOMAIN_PLATFORM_H
#define M86_G3D_DOMAIN_PLATFORM_H
#include <linux/types.h>
struct regmap;
struct device;
/* Unimplemented platform integration must not register this provider.
 * All callbacks are synchronous. No callback may call runtime PM, devfreq,
 * thermal get_temp, or acquire the genpd lock recursively.
 */
struct m86_g3d_domain_ops {
 /* Serialize coordinator/CCF ownership and validate retained rails. For off,
  * save full CMU context while powered, with no gate/PMU/rail writes.
  * Success holds serialization until
  * finish/abort. Failure must unwind its own partial preparation.
  */
 int (*prepare)(void *data, bool on);
 /* Off preflight EBUSY: no hardware write has occurred. Release prepare
  * serialization while keeping the original ON state and allowing retry. */
 void (*cancel_prepare)(void *data);
 /* Existing clock owner implements exact TOPC1 bit9 and local ACLK bit1
  * through its serialized CCF/context protocol, never a second MMIO owner. */
 int (*top_gate)(void *data, bool enable);
 int (*local_lpi_clock)(void *data);
 /* Consumer-owned powered access: all SHADER/TILER/L2 READY and PWRTRANS
  * must be zero. void Panfrost power-off is not a success guarantee. */
 int (*check_idle)(void *data);
 int (*validate_mode)(void *data);
 /* Use m86_tmu_gpu_transition to wrap on+restore; no TMU change on off. */
 int (*run_on)(void *data, int (*transition)(void *), void *arg);
 /* Restore PLL/div/mux/gates and EMA, validate exact state, still serialized. */
 int (*restore)(void *data);
 void (*finish)(void *data, bool on);
 /* Infallibly quarantine consumer/coordinator before releasing serialization.
  * prepared indicates whether prepare succeeded; quarantine is also required
  * before prepare when power state is already inconsistent. status is low4 PMU bits; mixed states are not proof of safe local MMIO.
  * Do not silently re-enable consumer or blindly roll back voltage/context.
  */
 void (*abort)(void *data, int error, u32 status, bool prepared);
};
struct m86_g3d_domain_platform_data {
 const struct m86_g3d_domain_ops *ops;
 void *data;
 /* Borrowed existing PMU syscon regmap (physical105c0000), boot-long. */
 struct regmap *pmu;
 /* Registered coordinator; link is added from the registered provider probe. */
 struct device *supplier;
};
#endif
