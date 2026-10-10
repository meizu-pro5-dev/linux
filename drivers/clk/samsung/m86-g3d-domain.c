// SPDX-License-Identifier: GPL-2.0
/* Compile-review candidate only. No DT enables this experimental provider. */
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include "m86-g3d-domain-platform.h"
#define G3D_CFG 0x4100
#define G3D_STAT 0x4104
#define G3D_OPT 0x4108
#define G3D_DURATION 0x4110
#define TOP_GATE 0x0a04
#define LOCAL_GATE 0x0800
struct m86_domain {
 struct generic_pm_domain genpd;
 struct device *dev;
 struct regmap *pmu;
 const struct m86_g3d_domain_platform_data *platform;
 struct mutex lock;
 bool failed;
};
static int prepare_sys_power(struct m86_domain *d)
{
 static const u32 offsets[] = {0x1460, 0x14a0, 0x14e0, 0x1520,
                              0x1560, 0x15a0};
 unsigned int i;
 int ret;
 for (i = 0; i < ARRAY_SIZE(offsets); i++) {
  ret = regmap_update_bits(d->pmu, offsets[i],
                           offsets[i] == 0x1560 ? 3 : 1, 0);
  if (ret)
   return ret;
 }
 return 0;
}
static int switch_domain(struct m86_domain *d, bool on)
{
 unsigned int v;
 int ret;
 ret = regmap_write(d->pmu, G3D_OPT, 0x0102);
 if (ret)
  return ret;
 /* Exact vendor G3D duration value; no invented PMIC/GPIO sequence. */
 ret = regmap_write(d->pmu, G3D_DURATION, 0xffff00c0);
 if (ret)
  return ret;
 ret = regmap_write(d->pmu, G3D_CFG, on ? 15 : 0);
 if (ret)
  return ret;
 return regmap_read_poll_timeout(d->pmu, G3D_STAT, v,
                                (v & 15) == (on ? 15 : 0), 8, 1000);
}
static int turn_on(void *arg)
{
 struct m86_domain *d = arg;
 int ret;
 ret = prepare_sys_power(d);
 if (ret)
  return ret;
 ret = switch_domain(d, true);
 if (ret)
  return ret;
 /* Local MMIO is only legal after the status confirms all local power bits. */
 return d->platform->ops->restore(d->platform->data);
}
static int transition(struct generic_pm_domain *genpd, bool on)
{
 struct m86_domain *d = container_of(genpd, struct m86_domain, genpd);
 const struct m86_g3d_domain_ops *ops = d->platform->ops;
 int ret;
 u32 status = ~0U, cfg = ~0U;
 mutex_lock(&d->lock);
 if (d->failed) {
  ret = -EIO;
  goto out;
 }
 ret = regmap_read(d->pmu, G3D_CFG, &cfg);
 if (!ret)
  ret = regmap_read(d->pmu, G3D_STAT, &status);
 if (ret || (cfg & 15) != (on ? 0 : 15) ||
     (status & 15) != (on ? 0 : 15)) {
  /* Do not touch local MMIO on an unexpected/mixed power state. */
  status = ret ? ~0U : status & 15;
  ret = ret ?: -EIO;
  d->failed = true;
  ops->abort(d->platform->data, ret, status, false);
  goto out;
 }
 ret = ops->prepare(d->platform->data, on);
 if (ret)
  goto out;
 if (on) {
  /* TMU API encloses PMU-on and full context restore, returns only after
   * restoring sensor CONTROL and obtaining a valid sample. */
  ret = ops->top_gate(d->platform->data, true);
  if (!ret)
   ret = ops->run_on(d->platform->data, turn_on, d);
 } else {
  ret = ops->check_idle(d->platform->data);
  if (ret == -EBUSY) {
   /* Consumer may not have probed/suspended yet. No provider gate or
    * PMU operation has started; keep ON and permit a later retry. */
   ops->cancel_prepare(d->platform->data);
   goto out;
  }
  if (!ret)
   ret = ops->local_lpi_clock(d->platform->data);
  if (!ret)
   ret = prepare_sys_power(d);
  if (!ret)
   ret = switch_domain(d, false);
  if (!ret)
   ret = ops->top_gate(d->platform->data, false);
 }
 if (ret) {
  if (regmap_read(d->pmu, G3D_STAT, &status))
   status = ~0U;
  else
   status &= 15;
  d->failed = true;
  ops->abort(d->platform->data, ret, status, true);
  dev_err(d->dev, "G3D %s failed %d status=%#x; consumer quarantined\n",
          on ? "on" : "off", ret, status);
 } else {
  ops->finish(d->platform->data, on);
  /* Separate once sites: no repeated runtime-transition log traffic. The
   * PMU poll proves STATUS; on also completed CMU and TMU wrapper restore. */
  if (on)
   dev_info_once(d->dev, "G3D external on: PMU STATUS=0xf, CMU/TMU restore complete\n");
  else
   dev_info_once(d->dev, "G3D external off: PMU STATUS=0, TOP reference released\n");
 }
out:
 mutex_unlock(&d->lock);
 return ret;
}
static int power_on(struct generic_pm_domain *genpd)
{
 return transition(genpd, true);
}
static int power_off(struct generic_pm_domain *genpd)
{
 return transition(genpd, false);
}
static int probe(struct platform_device *pdev)
{
 const struct m86_g3d_domain_platform_data *p = dev_get_platdata(&pdev->dev);
 struct m86_domain *d;
 unsigned int cfg, status;
 int ret;
 if (!pdev->dev.of_node || !of_machine_is_compatible("meizu,m86"))
  return -ENODEV;
 if (!p || !p->ops || !p->ops->prepare || !p->ops->run_on ||
     !p->ops->restore || !p->ops->finish || !p->ops->abort ||
     !p->ops->top_gate || !p->ops->local_lpi_clock ||
     !p->ops->validate_mode || !p->ops->check_idle ||
     !p->ops->cancel_prepare || IS_ERR_OR_NULL(p->pmu) ||
     IS_ERR_OR_NULL(p->supplier))
  return dev_err_probe(&pdev->dev, -EOPNOTSUPP,
                       "G3D context/TMU/ownership integration missing\n");
 /* Name and bus are valid here. Both drivers may still be PROBING;
  * the coordinator has published the live callbacks before device_add.
  * Core supports a consumer-probe link to this functional probing supplier.
  * Failed probe removes this managed link; no runtime-PM link is requested. */
 if (!device_is_registered(p->supplier))
  return -EINVAL;
 if (!device_link_add(&pdev->dev, p->supplier,
                      DL_FLAG_AUTOREMOVE_CONSUMER))
  return -EINVAL;
 d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
 if (!d)
  return -ENOMEM;
 d->dev = &pdev->dev;
 d->platform = p;
 d->pmu = p->pmu;
 ret = p->ops->validate_mode(p->data);
 if (ret)
  return ret;
 /* First attachment only accepts the proven inherited powered-on state. */
 ret = regmap_read(d->pmu, G3D_CFG, &cfg);
 if (ret)
  return ret;
 ret = regmap_read(d->pmu, G3D_STAT, &status);
 if (ret)
  return ret;
 if ((cfg & 15) != 15 || (status & 15) != 15)
  return -EOPNOTSUPP;
 mutex_init(&d->lock);
 d->genpd.name = "m86-g3d-retained-rails";
 d->genpd.power_on = power_on;
 d->genpd.power_off = power_off;
 ret = pm_genpd_init(&d->genpd, NULL, false);
 if (ret)
  return ret;
 ret = of_genpd_add_provider_simple(pdev->dev.of_node, &d->genpd);
 if (ret)
  pm_genpd_remove(&d->genpd);
 return ret;
}
/* Built-in experiment: platform glue and ops must have boot-long lifetime.
 * No unload/unbind path may discard a live genpd or its callbacks. */
static struct platform_driver m86_g3d_driver = {
 .probe = probe,
 .driver = { .name = "m86-g3d-domain-experiment", .suppress_bind_attrs = true },
};
builtin_platform_driver(m86_g3d_driver);

MODULE_LICENSE("GPL");
