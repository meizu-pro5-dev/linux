// SPDX-License-Identifier: GPL-2.0-only
/* Review candidate: retained external domain, explicit runtime-PM transaction. */
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>
#include <asm/barrier.h>
#include <linux/m86-gpu-dvfs.h>
#include <linux/m86-tmu-transition.h>
#include "m86-topc-int-readback.h"
#include <linux/m86-bus-qos.h>
#include <linux/soc/samsung/exynos7420-asv.h>
#include <linux/m86-asv-cal.h>
struct gpu_rate { unsigned long hz; u32 uv, mif, int_qos; u16 m; u8 p, s, hpm, cal_lvl; };
/* Vendor table_g3d, v15 group10 default, hpm_freq_table. 852 (L0) is ASV-only, not vendor DVFS. */
static const struct gpu_rate rates[] = {
 {266000000,656250,416000,160000,133,6,1,2,7},
 {350000000,662500,543000,200000,175,6,1,2,6},
 {420000000,668750,1026000,267000,105,3,1,2,5},
 {544000000,706250,1026000,413000,68,3,0,3,4},
 {600000000,743750,1552000,413000,75,3,0,3,3},
 {700000000,787500,1552000,400000,175,6,0,3,2},
 {772000000,825000,1552000,400000,193,6,0,3,1},
};
struct gpu_dvfs {
 struct device *dev, *pmic;
 struct device_node *gpu_node;
 struct clk_hw hw;
 struct clk *clock;
 struct regmap *pmu, *sysreg;
 struct thermal_zone_device *zone;
 void __iomem *cmu, *dvs, *gpio, *topc;
 struct mutex transaction, hardware;
 unsigned long cap, current_rate;
 u32 mcs[4];
 u32 asv_uv[ARRAY_SIZE(rates)];
 u32 asv_sel[ARRAY_SIZE(rates)];
 u32 cur_sel, cur_ramp;
 bool active_voltage, normal_on_voltage, failed, normal_on_bit0_observed;
 struct task_struct *caller;
 struct device *bus_dev;
 struct dev_pm_qos_request bus_qos_req;
 bool bus_qos_active;
 u32 cur_mif_qos, want_mif;
 struct work_struct mif_work;
};
static DEFINE_MUTEX(instance_lock);
static struct gpu_dvfs *instance;
#define PMS_MASK ((0x3ffU << 16) | (0x3fU << 8) | 7U)
static bool domain_on(struct gpu_dvfs *d)
{
 unsigned int cfg, status;
 return !regmap_read(d->pmu,0x4100,&cfg) && !regmap_read(d->pmu,0x4104,&status) &&
        (cfg & 15) == 15 && (status & 15) == 15;
}
static unsigned long actual_rate(struct gpu_dvfs *d)
{
 u32 con, mux, stat, p, m, s;
 unsigned long rate;
 /* hardware serializes ordinary rate reads against domain-off preflight. */
 if (!domain_on(d)) return d->current_rate ? d->current_rate : 266000000;
 mux = readl(d->cmu+0x200); stat = readl(d->cmu+0x400);
 if ((stat & BIT(10)) || (readl(d->cmu+0x600)&7)) return d->current_rate ? d->current_rate : 266000000;
 if (mux & BIT(8)) return 24000000;
 if (!(mux & BIT(0))) return 24000000;
 con = readl(d->cmu+0x100);
 if ((con & (BIT(31)|BIT(29))) != (BIT(31)|BIT(29))) return d->current_rate ? d->current_rate : 266000000;
 p=(con>>8)&63; m=(con>>16)&1023; s=con&7;
 if (!p || !m || con != readl(d->cmu+0x100) || mux != readl(d->cmu+0x200)) return d->current_rate ? d->current_rate : 266000000;
 rate = div_u64(24000000ULL*m,p<<s);
 d->current_rate = rate;
 return rate;
}
static unsigned long recalc(struct clk_hw *hw, unsigned long parent)
{
 struct gpu_dvfs *d=container_of(hw,struct gpu_dvfs,hw);
 unsigned long rate;
 mutex_lock(&d->hardware); rate=actual_rate(d); mutex_unlock(&d->hardware);
 return rate;
}
static long round_rate(struct clk_hw *hw,unsigned long rate,unsigned long *parent)
{
 struct gpu_dvfs *d=container_of(hw,struct gpu_dvfs,hw);
 unsigned int i;
 unsigned long best=0, delta=~0UL;
 for(i=0;i<ARRAY_SIZE(rates);i++) {
  unsigned long diff=rate>rates[i].hz ? rate-rates[i].hz : rates[i].hz-rate;
  if(rates[i].hz<=d->cap && diff<delta) {best=rates[i].hz;delta=diff;}
 }
 return best ? best : -EINVAL;
}
/* Vendor clk-mux.c polls only bit(stat_shift+width-1): bit10 for G3D.
 * Lower STAT bits are not interpreted as a CPU-style parent encoding. */
static int select_fin(struct gpu_dvfs *d,bool fin)
{
 u32 sel=readl(d->cmu+0x200), val;
 sel=fin ? sel|BIT(8) : sel&~BIT(8);
 writel(sel,d->cmu+0x200);
 if(readl(d->cmu+0x200)!=sel) return -EIO;
 return readl_poll_timeout(d->cmu+0x400,val,!(val&BIT(10)),1,10000);
}
static int set_rate(struct clk_hw *hw,unsigned long hz,unsigned long parent)
{
 struct gpu_dvfs *d=container_of(hw,struct gpu_dvfs,hw);
 const struct gpu_rate *r=NULL;
 u32 old, desired, val, div;
 unsigned int i, settle;
 int ret=0;
 mutex_lock(&d->hardware);
 if(d->failed || d->caller!=current || hz>d->cap || parent!=24000000 || !domain_on(d)) {
  ret=-EPERM; goto out;
 }
 for(i=0;i<ARRAY_SIZE(rates);i++) if(rates[i].hz==hz) r=&rates[i];
 if(!r) {ret=-EINVAL;goto out;}
 if(actual_rate(d)==hz) goto out;
 /* Retained bypass must select PLL; never alter bit0 or unexplained bit4. */
 if(!(readl(d->cmu+0x200)&BIT(0))) {ret=-EOPNOTSUPP;goto out;}
 ret=select_fin(d,true); if(ret) goto fault;
 old=readl(d->cmu+0x100);
 if(!(old&BIT(31))) {ret=-EIO;goto fault;}
 desired=(old&~PMS_MASK)|(r->m<<16)|(r->p<<8)|r->s;
 writel(r->p*150,d->cmu); writel(desired,d->cmu+0x100);
 if((readl(d->cmu+0x100)&PMS_MASK)!=(desired&PMS_MASK)) {ret=-EIO;goto fault;}
 /* Vendor PLL1452 shares maximum150*P cycles. Flush then wait full24MHz
  * lock interval+10us; no requirement to observe transient unlock pulse. */
 settle=DIV_ROUND_UP(r->p*150,24)+10;
 usleep_range(settle,settle+10);
 ret=readl_poll_timeout(d->cmu+0x100,val,
  (val&BIT(29)) && (val&PMS_MASK)==(desired&PMS_MASK),1,10000);
 if(ret) goto fault;
 usleep_range(2,5);
 val=readl(d->cmu+0x100);
 if((val&(PMS_MASK|BIT(29)|BIT(31)))!=((desired&PMS_MASK)|BIT(29)|BIT(31))) {
  ret=-EIO;goto fault;
 }
 /* HPMdivider at bits9:8, busybit8. Change while core runs safe24MHz. */
 div=readl(d->cmu+0x600);
 div=(div&~(3U<<8))|((r->hpm-1)<<8);
 writel(div,d->cmu+0x600);
 if(readl(d->cmu+0x600)!=div) {ret=-EIO;goto fault;}
 ret=readl_poll_timeout(d->cmu+0x700,val,!(val&BIT(8)),1,10000);
 if(ret) goto fault;
 ret=select_fin(d,false); if(ret) goto fault;
 if(actual_rate(d)!=hz) {ret=-EIO;goto fault;}
 d->current_rate=hz;
 goto out;
fault:
 /* No blind PLL/mux rollback and no regulator rollback. Failure poisons all
  * future transitions; recalc returns0 so no lower-voltage proof is possible. */
 d->failed=true;
 dev_err(d->dev,"GPU clock failure %d; voltage frozen, switching blocked\n",ret);
out:
 mutex_unlock(&d->hardware);return ret;
}
static const struct clk_ops clock_ops={.recalc_rate=recalc,.round_rate=round_rate,.set_rate=set_rate};
static int validate_asv(struct gpu_dvfs *d)
{
 struct m86_asv_cal_info cal;
 unsigned int i;
 int ret = exynos7420_asv_get_cal(&cal);
 if (ret)
  ret = m86_asv_cal_read(&cal);
 if (ret)
  return ret;
 for (i = 0; i < ARRAY_SIZE(rates); i++) {
  u32 lvl = rates[i].cal_lvl; /* L7(266MHz)..L1(772MHz), row in m86_g3d_asv_volt_v15 is lvl - 1 */
  u32 volt = m86_asv_lookup_volt(&cal, M86_ASV_G3D, lvl,
                                 m86_g3d_asv_volt_v15[lvl - 1]);
  d->asv_uv[i] = volt;
  d->asv_sel[i] = (volt - 300000) / 6250;
 }
 dev_info(d->dev, "GPU ASV CAL: group=0x%03x ssa0=%u ver=%u type=%u shift=0x%08x -> 266..772MHz sel=[%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
          m86_asv_domain_group_raw(&cal, M86_ASV_G3D),
          m86_asv_domain_ssa0(&cal, M86_ASV_G3D),
          cal.effective_ver, cal.asv_group_type, cal.words[4],
          d->asv_sel[0], d->asv_sel[1], d->asv_sel[2], d->asv_sel[3],
          d->asv_sel[4], d->asv_sel[5], d->asv_sel[6]);
 d->mcs[0] = (cal.words[5] >> 21) & 3;
 d->mcs[1] = (cal.words[5] >> 23) & 3;
 d->mcs[2] = (cal.words[5] >> 25) & 3;
 d->mcs[3] = (cal.words[5] >> 27) & 3;
 if (!(d->mcs[0]==3 && d->mcs[1]==3 && d->mcs[2]==3 && !d->mcs[3])) {
  dev_warn(d->dev, "Non-standard MCS calibration [%u,%u,%u,%u]; applying default handoff MCS\n",
           d->mcs[0], d->mcs[1], d->mcs[2], d->mcs[3]);
  d->mcs[0]=3; d->mcs[1]=3; d->mcs[2]=3; d->mcs[3]=0;
 }
 return 0;
}
static int pmic_read(struct gpu_dvfs *d,u32 reg,u32 *v)
{
 struct regmap *map=dev_get_regmap(d->pmic,NULL);
 int maxreg;
 if(!d->pmic->driver || !map) return -EPROBE_DEFER;
 maxreg=regmap_get_max_register(map);
 if(maxreg<0 || reg>(u32)maxreg) return -EOPNOTSUPP;
 return regmap_read(map,reg,v);
}
/* Must hold PMIC device_lock; protects map lifetime. Never write ownership. */
static int check_rail(struct gpu_dvfs *d,u32 *selector,u32 *ramp)
{
 u32 id,ctrl,trim,c0,ldo,sync,dvs4;
 int ret;
 if(!d->normal_on_voltage && (readl(d->dvs)!=0 || readl(d->dvs+4)!=1 || ((readl(d->gpio)>>16)&15)!=2))
  return -EOPNOTSUPP;
 ret=pmic_read(d,0,&id);if(ret)return ret;
 ret=pmic_read(d,0x23,&ctrl);if(ret)return ret;
 ret=pmic_read(d,0x24,selector);if(ret)return ret;
 ret=pmic_read(d,0x2f,ramp);if(ret)return ret;
 if(d->normal_on_voltage) {
  /* Independent fixed-LDO protocol: never change mode, sync or DVS fields. */
  /* Bit0 is undefined in the available PMIC documentation. Vendor CTRL1
   * updates preserve it and selector writes do not test it. Observed f9
   * returns to f8 after load; keep bits7:1 exact and never write CTRL1. */
  if(id!=0x12 || (ctrl&0xfe)!=0xf8 || *selector < d->asv_sel[0] || *selector > d->asv_sel[ARRAY_SIZE(rates)-1]) {
   dev_err(d->dev,"Normal ON rail mismatch stage=id-control-selector ID=%#x CTRL=%#x selector=%#x ret=%d\n",
           id,ctrl,*selector,-EOPNOTSUPP);
   return -EOPNOTSUPP;
  }
  ret=pmic_read(d,0x3d,&ldo);if(ret)return ret;
  ret=pmic_read(d,0x52,&sync);if(ret)return ret;
  if(ldo!=0xd4 || (sync&0x40)) {
   dev_err(d->dev,"Normal ON rail mismatch stage=ldo-sync LDO=%#x sync=%#x ret=%d\n",
           ldo,sync,-EOPNOTSUPP);
   return -EOPNOTSUPP;
  }
  if((ctrl&1) && !d->normal_on_bit0_observed) {
   d->normal_on_bit0_observed=true;
   dev_info(d->dev,"Normal ON accepted CTRL1=%#x selector=%#x: preserving undefined bit0, bits7:1 validated\n",ctrl,*selector);
  }
  return 0;
 }
 if(id!=0x12 || (ctrl!=0x38 && (d->active_voltage || ctrl!=0xf8)))return -EOPNOTSUPP;
 if(!d->active_voltage)return 0;
 ret=pmic_read(d,0x59,&trim);if(ret)return ret;
 ret=pmic_read(d,0xc0,&c0);if(ret)return ret;
 ret=pmic_read(d,0x3d,&ldo);if(ret)return ret;
 ret=pmic_read(d,0x52,&sync);if(ret)return ret;
 ret=pmic_read(d,0x33,&dvs4);if(ret)return ret;
 /* r39 true-read normal path: no workaround and no selector compensation. */
 if((trim!=0x78 && trim!=0x38) || c0!=0 || (ldo&0xc0) || !(sync&0x40) || dvs4!=0x84)
  return -EOPNOTSUPP;
 return 0;
}
/* One board's probe-only ownership transfer. Runtime never changes these
 * enable/sync fields. Failed writes are not rolled back or retried. */
static bool normal_on_takeover_failed;
struct takeover_snapshot {
 u32 id, ctrl, selector, ldo, sync, dvs4, trim, c0, ctrl3;
 u32 cfg, status, gpio, dvs_ctrl, dvs_status, mcs;
 unsigned long rate;
};
static int takeover_snapshot(struct gpu_dvfs *d,struct takeover_snapshot *s)
{
 const u32 regs[]={0,0x23,0x24,0x3d,0x52,0x33,0x59,0xc0,0x57};
 u32 *values[]={&s->id,&s->ctrl,&s->selector,&s->ldo,&s->sync,
               &s->dvs4,&s->trim,&s->c0,&s->ctrl3};
 int i,ret,first=0;
 memset(s,0xff,sizeof(*s));
 for(i=0;i<ARRAY_SIZE(regs);i++) {
  ret=pmic_read(d,regs[i],values[i]);if(ret && !first)first=ret;
 }
 ret=regmap_read(d->pmu,0x4100,&s->cfg);if(ret && !first)first=ret;
 ret=regmap_read(d->pmu,0x4104,&s->status);if(ret && !first)first=ret;
 ret=regmap_read(d->sysreg,0x1224,&s->mcs);if(ret && !first)first=ret;
 s->gpio=readl(d->gpio);s->dvs_ctrl=readl(d->dvs);
 s->dvs_status=readl(d->dvs+4);
 mutex_lock(&d->hardware);s->rate=actual_rate(d);mutex_unlock(&d->hardware);
 return first;
}
static void takeover_report(struct gpu_dvfs *d,const char *stage,int error)
{
 struct takeover_snapshot s;
 int snapshot_ret=takeover_snapshot(d,&s);
 dev_err(d->dev,"Normal ON takeover failed stage=%s ret=%d snapshot_ret=%d ID=%#x CTRL=%#x selector=%#x LDO=%#x sync=%#x DVS4=%#x trim=%#x C0=%#x CTRL3=%#x PMU=%#x/%#x GPIO=%#x DVS=%#x/%#x MCS=%#x rate=%lu; retaining achieved state, no rollback/provider/retry\n",
  stage,error,snapshot_ret,s.id,s.ctrl,s.selector,s.ldo,s.sync,s.dvs4,
  s.trim,s.c0,s.ctrl3,s.cfg,s.status,s.gpio,s.dvs_ctrl,s.dvs_status,s.mcs,s.rate);
}
/* Called with the PMIC device locked and the unbound GPU locked. The explicit
 * board opt-in allows only the captured GPIO state -> retained Normal ON.
 * Vendor gives the masked ON operation and GPIO setup's opposite order;
 * this reverse transfer is inferred, not a copied vendor takeover function. */
static int takeover_normal_on(struct gpu_dvfs *d)
{
 struct takeover_snapshot s;
 struct regmap *map=dev_get_regmap(d->pmic,NULL);
 struct i2c_client *client=i2c_verify_client(d->pmic);
 u32 value,selector,ramp;
 const char *stage="preflight";
 int ret;
 if(normal_on_takeover_failed)return -EIO;
 if(!client || strcmp(client->name,"s2mps15-pmic") ||
    strcmp(dev_name(d->pmic),"5-0066") || client->addr!=0x66 ||
    !of_device_is_compatible(d->pmic->of_node,"samsung,s2mps15-pmic"))return -ENODEV;
 ret=takeover_snapshot(d,&s);if(ret)goto fail;
 /* Already Normal ON is idempotent and must pass every runtime rail check. */
 if((s.ctrl&0xfe)==0xf8) {
  ret=check_rail(d,&selector,&ramp);
  if(!ret && selector!=d->asv_sel[0])ret=-EOPNOTSUPP;
  if(!ret)return 0;
  goto fail;
 }
 if(s.id!=0x12 || s.ctrl!=0x38 || s.selector!=d->asv_sel[0] || s.ldo!=0x14 ||
    s.sync!=0x4f || s.dvs4!=0x84 || (s.trim!=0x78 && s.trim!=0x38) || s.c0 || s.ctrl3!=0x20 ||
    (s.cfg&15)!=15 || (s.status&15)!=15 || ((s.gpio>>16)&15)!=2 ||
    s.dvs_ctrl || s.dvs_status!=1 || s.mcs!=0x31 || s.rate!=266000000) {
  ret=-EOPNOTSUPP;goto fail;
 }
 /* Recheck immediately before the first write; no registered GPU/provider
  * can turn this retained domain off during the transfer. */
 mutex_lock(&d->hardware);
 ret=domain_on(d) && actual_rate(d)==266000000 ? 0 : -EHOSTDOWN;
 mutex_unlock(&d->hardware);
 if(ret)goto fail;
 stage="LDO10-ON";
 normal_on_takeover_failed=true;
 ret=regmap_update_bits(map,0x3d,0xc0,0xc0);if(ret)goto fail;
 ret=pmic_read(d,0x3d,&value);if(ret)goto fail;
 if(value!=0xd4){ret=-EIO;goto fail;}
 /* Vendor S2MPS15 Rev1 LDO and BUCK6 enable times are both 128us. */
 usleep_range(200,300);
 stage="BUCK6-ON";
 ret=regmap_update_bits(map,0x23,0xc0,0xc0);if(ret)goto fail;
 ret=pmic_read(d,0x23,&value);if(ret)goto fail;
 if((value&0xfe)!=0xf8){ret=-EIO;goto fail;}
 usleep_range(200,300);
 stage="both-ON-before-sync";
 ret=pmic_read(d,0x3d,&value);if(ret)goto fail;
 if(value!=0xd4){ret=-EIO;goto fail;}
 ret=pmic_read(d,0x23,&value);if(ret)goto fail;
 if((value&0xfe)!=0xf8){ret=-EIO;goto fail;}
 stage="sync-independent";
 ret=regmap_update_bits(map,0x52,0x40,0);if(ret)goto fail;
 ret=pmic_read(d,0x52,&value);if(ret)goto fail;
 if(value!=0x0f){ret=-EIO;goto fail;}
 usleep_range(200,300);
 stage="final-validation";
 ret=check_rail(d,&selector,&ramp);if(ret)goto fail;
 ret=takeover_snapshot(d,&s);if(ret)goto fail;
 if(selector!=d->asv_sel[0] || s.selector!=d->asv_sel[0] || s.mcs!=0x31 ||
    s.rate!=266000000 || (s.cfg&15)!=15 || (s.status&15)!=15 ||
    s.ctrl3!=0x20){ret=-EIO;goto fail;}
 normal_on_takeover_failed=false;
 dev_info(d->dev,"probe-only GPIO-to-Normal-ON takeover complete: BUCK6=f8/f9 selector=%#x (%u uV) LDO10=d4 sync0f; fixed selectors retained\n",
          s.selector, d->asv_uv[0]);
 return 0;
fail:
 takeover_report(d,stage,ret);
 return ret;
}

/* devfreq calls target with its lock held, and from PM QoS notifiers with the
 * global dev_pm_qos_mtx held (sysfs min/max_freq, cooling). Updating the MIF
 * request there self-deadlocks, and from the monitor it inverts against those
 * writers. Apply it from a work; a requeue picks up the latest value. */
static void mif_work_fn(struct work_struct *work)
{
 struct gpu_dvfs *d=container_of(work,struct gpu_dvfs,mif_work);
 dev_pm_qos_update_request(&d->bus_qos_req,READ_ONCE(d->want_mif));
}
static void mif_vote(struct gpu_dvfs *d,u32 khz)
{
 if(!d->bus_qos_active)return;
 d->cur_mif_qos=khz;
 WRITE_ONCE(d->want_mif,khz);
 queue_work(system_highpri_wq,&d->mif_work);
}

/* Runtime resume has not marked the GPU ACTIVE yet. No PM get or devfreq
 * calls here: validate retained hardware before Panfrost reset/first job
 * without issuing 6 redundant GPIO-I2C reads, and restore the MIF QoS vote
 * for the active OPP.
 */
int m86_gpu_resume_validate(struct device *gpu)
{
 struct gpu_dvfs *d;
 unsigned long hz;
 unsigned int i;
 u32 mif_khz = rates[0].mif;
 int ret=0;
 mutex_lock(&instance_lock);d=instance;
 if(!d || gpu->of_node!=d->gpu_node){ret=-ENODEV;goto unlock;}
 if(!d->normal_on_voltage)goto unlock;
 mutex_lock(&d->transaction);
 if(d->failed){ret=-EIO;goto done;}
 hz=clk_get_rate(d->clock);
 if(!hz || hz>d->cap) {
  ret=-ERANGE;
  mutex_lock(&d->hardware);d->failed=true;mutex_unlock(&d->hardware);
  dev_err(d->dev,"GPU retained resume state rejected: rate=%lu error=%d\n",hz,ret);
  goto done;
 }
 for(i=0;i<ARRAY_SIZE(rates);i++) {
  if(rates[i].hz==hz){mif_khz=rates[i].mif;break;}
 }
 if(d->bus_qos_active && d->cur_mif_qos!=mif_khz)
  mif_vote(d,mif_khz);
done:
 mutex_unlock(&d->transaction);
unlock:
 mutex_unlock(&instance_lock);
 return ret;
}
EXPORT_SYMBOL_GPL(m86_gpu_resume_validate);

/* Release the GPU MIF QoS floor while Panfrost is runtime-suspended so
 * an idle or pre-suspend high-OPP vote does not pin MIF/INT while G3D
 * shaders/tiler/L2 are powered down.
 */
void m86_gpu_suspend_notify(struct device *gpu)
{
 struct gpu_dvfs *d;
 mutex_lock(&instance_lock);d=instance;
 if(!d || gpu->of_node!=d->gpu_node)goto unlock;
 mutex_lock(&d->transaction);
 if(d->bus_qos_active && d->cur_mif_qos!=0)
  mif_vote(d,0);
 mutex_unlock(&d->transaction);
unlock:
 mutex_unlock(&instance_lock);
}
EXPORT_SYMBOL_GPL(m86_gpu_suspend_notify);

static u32 calibrated_mcs(struct gpu_dvfs *d,u32 uv)
{
 unsigned int band=uv>=900000?0:uv>=800000?1:uv>=700000?2:3;
 return d->mcs[band] ?: (band==3 ? 3 : 1);
}
static int set_mcs(struct gpu_dvfs *d,u32 uv)
{
 u32 value,old,mcs=calibrated_mcs(d,uv);
 int ret=regmap_read(d->sysreg,0x1224,&old);
 if(ret)return ret;
 if((old&~0x30U)!=1)return -EOPNOTSUPP;
 ret=regmap_update_bits(d->sysreg,0x1224,0x30,mcs<<4);if(ret)return ret;
 ret=regmap_read(d->sysreg,0x1224,&value);
 return ret ?: (value==((old&~0x30U)|(mcs<<4)) ? 0 : -EIO);
}
/* Normal ON: old/ramp come from cached state initialized during probe
 * check_rail() and updated on each successful selector write. */
static int set_voltage(struct gpu_dvfs *d,u32 uv,u32 old,u32 ramp)
{
 struct regmap *map;
 u32 sel,actual,wait;
 unsigned int i;
 bool valid_uv=false;
 int ret;
 if((!d->active_voltage && !d->normal_on_voltage) ||
    uv<300000 || (uv-300000)%6250)return -EPERM;
 for(i=0;i<ARRAY_SIZE(rates);i++)if(d->asv_uv[i]==uv){valid_uv=true;break;}
 if(d->normal_on_voltage && !valid_uv)return -EPERM;
 sel=(uv-300000)/6250; /* r39 trim78/bit5set: normal vendor path, NO OCP+5. */
 if(sel>255)return -ERANGE;
 device_lock(d->pmic);
 if(!d->normal_on_voltage){ret=check_rail(d,&old,&ramp);if(ret)goto out;}
 map=dev_get_regmap(d->pmic,NULL);
 ret=regmap_write(map,0x24,sel);if(ret)goto out;
 wait=DIV_ROUND_UP(abs((int)sel-(int)old)*6250,6000U<<(ramp&3))+10;
 usleep_range(wait,wait+10);
 d->cur_sel=sel;
 if(!d->normal_on_voltage) {
  ret=pmic_read(d,0x24,&actual);
  if(!ret && actual!=sel)ret=-EIO;
  if(!ret)ret=check_rail(d,&actual,&ramp);
 }
out:
 device_unlock(d->pmic);return ret;
}
static int set_clock(struct gpu_dvfs *d,unsigned long rate)
{
 int ret;
 mutex_lock(&d->hardware);d->caller=current;mutex_unlock(&d->hardware);
 ret=clk_set_rate(d->clock,rate);
 mutex_lock(&d->hardware);d->caller=NULL;mutex_unlock(&d->hardware);
 if(!ret && clk_get_rate(d->clock)!=rate)ret=-EIO;
 return ret;
}
static int bus_floor(struct gpu_dvfs *d,u32 floor)
{
 u32 con,sel,con2,sel2,p,m,s;
 exynos7420_bus_dvfs_read_lock();
 con=readl(d->topc+0x150);sel=readl(d->topc+0x214);
 con2=readl(d->topc+0x150);sel2=readl(d->topc+0x214);
 exynos7420_bus_dvfs_read_unlock();
 p=(con>>8)&63;m=(con>>16)&1023;s=con&7;
 if(!p || !m || (con&(BIT(31)|BIT(29)))!=(BIT(31)|BIT(29)) ||
    !(sel&BIT(16)) || !(sel&BIT(20)) || con!=con2 || sel!=sel2)return -EIO;
 return div_u64(24000000ULL*m,(p<<s)*2000)>=floor ? 0 : -ERANGE;
}
/* The vote normally lands within a MIF switch; under a QoS writer it cannot
 * run until target returns, so give up after 5 ms and defer to the next poll. */
static int bus_floor_wait(struct gpu_dvfs *d,u32 floor)
{
 ktime_t end=ktime_add_us(ktime_get(),5000);
 int ret;
 while((ret=bus_floor(d,floor)) && ktime_before(ktime_get(),end))usleep_range(200,300);
 return ret;
}
int m86_gpu_devfreq_target(struct device *gpu,unsigned long *freq)
{
 struct gpu_dvfs *d;
 const struct gpu_rate *r=NULL;
 unsigned long old=0,requested=*freq;
 const char *stage="entry";
 u32 selector=~0U,ramp=~0U,mcs=~0U,target_uv=0;
 unsigned int i, idx=0;
 int ret,pmref,temp=(-2147483647-1);
 mutex_lock(&instance_lock);d=instance;
 if(!d || gpu->of_node!=d->gpu_node){ret=-ENODEV;goto unlock;}
 /* Never runtime_resume while devfreq lock is held: resume callback re-enters
  * devfreq_resume_device. get_if_active pins only already-active state. */
 pmref=pm_runtime_get_if_active(gpu,true);
 if(pmref<0){ret=pmref;goto unlock;}
 if(!pmref){
  /* Suspended GPU has no throughput demand. No GPU hardware writes here. */
  *freq=d->current_rate ? d->current_rate : 266000000;ret=0;goto unlock;
 }
 mutex_lock(&d->transaction);
 if(d->failed){*freq=d->current_rate ? d->current_rate : 266000000;ret=0;goto put;}
 for(i=0;i<ARRAY_SIZE(rates);i++)if(rates[i].hz==*freq){r=&rates[i];idx=i;}
 if(!r || r->hz>d->cap){ret=-ERANGE;goto put;}
 if(!d->active_voltage && !d->normal_on_voltage && *freq==266000000) {*freq=266000000;ret=0;goto put;}
  stage="old-rate";
  old=clk_get_rate(d->clock);
  if(!old){ret=-EIO;goto fault;}
  /* When already at minimum 266MHz and requesting 266MHz, thermal capping
   * (>75C -> 266MHz) is a no-op and the 500ms thermal zone poll handles
   * critical trips; skip redundant 20Hz TMU & GPIO-I2C reads. */
  if(old==266000000 && r->hz==266000000){
   if(d->bus_qos_active && d->cur_mif_qos!=rates[0].mif)
    mif_vote(d,rates[0].mif);
   ret=0;goto finish;
  }
  stage="thermal";
  ret=m86_tmu_get_cached_temp(&temp);
  if((ret==-ENODATA || ret==-EPROBE_DEFER) && d->zone)
   ret=thermal_zone_get_temp(d->zone,&temp);
  if(ret || temp < -20000) {ret=ret ?: -ERANGE;goto fault;}
  /* 50ms Panfrost governor invokes target even for unchanged frequency.
   * Fresh TMU measurement precedes the same-rate fast path when >266MHz.
   * Valid high temperature caps normally, including >=90C; the thermal zone
   * retains its independent 95C critical shutdown. */
  if(temp>=75000 && r->hz>266000000) {r=&rates[0];idx=0;*freq=r->hz;}
  target_uv = d->asv_uv[idx];
  if(old==r->hz){
   if(d->bus_qos_active && d->cur_mif_qos!=r->mif)
    mif_vote(d,r->mif);
   ret=0;goto finish;
  } /* Fast-path unchanged rate without GPIO-I2C storm. */
  if(r->hz>old && d->bus_qos_active && r->mif > d->cur_mif_qos)
   mif_vote(d,r->mif);
  stage="MIF-floor";
  ret=bus_floor_wait(d,r->mif);
  if(ret) {
   /* Deferral is not a failure: keep the old rate and retry on the next
    * poll. -EAGAIN made devfreq_monitor() dev_err() every 50 ms, and the
    * serial console write blocks CPU0 IRQs (~5 ms/line), starving I2S DMA. */
   dev_dbg(d->dev, "GPU MIF floor not met (%u kHz), deferring up-switch\n", r->mif);
   *freq = old;
   ret = 0;
   goto put;
  }
  {
   struct pro5_int_always_on_rates int_rates;
   stage="INT-read";
   exynos7420_bus_dvfs_read_lock();
   ret=pro5_topc_int_read_stable(d->topc,24000000,&int_rates);
   exynos7420_bus_dvfs_read_unlock();
   if(ret)goto fault;
   stage="INT-floor";
   ret=pro5_topc_int_always_on_floor(&int_rates,r->int_qos);
   if(ret) {
    dev_dbg(d->dev,"GPU INT floor not met (%u kHz), deferring up-switch\n", r->int_qos);
    *freq = old;
    ret = 0;
    goto put;
   }
  }
  if(d->normal_on_voltage) {
   selector=d->cur_sel;
   ramp=d->cur_ramp;
  }
  stage="MCS-before";
  ret=regmap_read(d->sysreg,0x1224,&mcs);if(ret)goto fault;
  if(mcs!=0x31){ret=-EOPNOTSUPP;goto fault;}
  if(!d->normal_on_voltage) {
   stage="rail-before";
   device_lock(d->pmic);ret=check_rail(d,&selector,&ramp);device_unlock(d->pmic);
   if(ret)goto fault;
  }
  if(!d->active_voltage && !d->normal_on_voltage){ret=-EPERM;goto put;}
  dev_dbg(d->dev,"GPU transition start old=%lu target=%lu uv=%u\n",old,r->hz,target_uv);
  if(r->hz>old) {
   stage="up-voltage";
   ret=set_voltage(d,target_uv,selector,ramp);if(ret)goto fault;
   if(!d->normal_on_voltage) {stage="MCS-set";ret=set_mcs(d,target_uv);if(ret)goto fault;}
   stage="up-clock";
   ret=set_clock(d,r->hz);if(ret)goto fault;
  } else {
   stage="down-clock";
   ret=set_clock(d,r->hz);if(ret)goto fault;
   if(!d->normal_on_voltage) {stage="MCS-set";ret=set_mcs(d,target_uv);if(ret)goto fault;}
   stage="down-voltage";
   ret=set_voltage(d,target_uv,selector,ramp);if(ret)goto fault;
   if(d->bus_qos_active && r->mif < d->cur_mif_qos)
    mif_vote(d,r->mif);
  }
  dev_dbg(d->dev,"GPU transition complete actual=%lu target_uv=%u\n",clk_get_rate(d->clock),target_uv);
 finish:
  ret=0;goto put;
 fault:
  dev_err(d->dev,"GPU first guard failure stage=%s ret=%d requested=%lu target=%lu old=%lu temp=%d selector=%#x MCS=%#x (old0/tempINT_MIN/0xffffffff=unsampled, before throttle)\n",
          stage,ret,requested,r ? r->hz : 0,old,temp,selector,mcs);
  if(!d->failed && clk_get_rate(d->clock)>266000000)
   if(set_clock(d,266000000))dev_err(d->dev,"emergency GPU clock throttle failed\n");
  if(d->bus_qos_active && d->cur_mif_qos > rates[0].mif)
   mif_vote(d,rates[0].mif);
  /* Throughput vote has no electrical safety role, including after failure. */
  mutex_lock(&d->hardware);d->failed=true;mutex_unlock(&d->hardware);
  dev_err(d->dev,"GPU transaction failed%d; no voltage rollback; all switching frozen\n",ret);
 put:
  mutex_unlock(&d->transaction);
  pm_runtime_mark_last_busy(gpu);pm_runtime_put_autosuspend(gpu);
 unlock:
  mutex_unlock(&instance_lock);return ret;
 }
 EXPORT_SYMBOL_GPL(m86_gpu_devfreq_target);
 static int m86_gpu_cold_power_on(struct gpu_dvfs *d)
 {
  struct regmap *map;
  u32 val, sel;
  int ret, timeout;
 
  if(!d->pmic)return -ENODEV;
  device_lock(d->pmic);
  map=dev_get_regmap(d->pmic,NULL);
  if(!map) { device_unlock(d->pmic); return -EPROBE_DEFER; }
 
  /* 1. PMIC S2MPS15 BUCK6: eFuse-calibrated 266MHz selector, ramp 0x20, LDO10 ON, BUCK6 Normal-ON, sync bit6 clear */
  ret=regmap_write(map,0x24,d->asv_sel[0]);if(ret){device_unlock(d->pmic);return ret;}
  ret=regmap_write(map,0x57,0x20);if(ret){device_unlock(d->pmic);return ret;}
  ret=regmap_update_bits(map,0x3d,0xc0,0xc0);if(ret){device_unlock(d->pmic);return ret;}
  usleep_range(200,300);
  ret=regmap_update_bits(map,0x23,0xfe,0xf8);if(ret){device_unlock(d->pmic);return ret;}
  usleep_range(200,300);
  ret=regmap_update_bits(map,0x52,0x40,0);
  device_unlock(d->pmic);
  if(ret)return ret;
  usleep_range(200,300);
 
  /* 2. PMU G3D Power Domain */
  ret=regmap_write(d->pmu,0x4100,0x0f);if(ret)return ret;
  timeout=1000;
  while(timeout--) {
   ret=regmap_read(d->pmu,0x4104,&val);
   if(!ret && (val&0x0f)==0x0f)break;
   usleep_range(10,20);
  }
  if(timeout<0)return -ETIMEDOUT;
 
  /* 3. CMU_G3D PLL & Clock Setup (266MHz) */
  sel=readl(d->cmu+0x200);
  sel|=BIT(0)|BIT(8);
  writel(sel,d->cmu+0x200);
  ret=readl_poll_timeout(d->cmu+0x400,val,!(val&BIT(10)),1,10000);
  if(ret)return ret;
 
  writel(6*150,d->cmu+0x000);
  val=BIT(31)|(133<<16)|(6<<8)|1;
  writel(val,d->cmu+0x100);
  usleep_range(48,60);
  ret=readl_poll_timeout(d->cmu+0x100,val,
   (val&BIT(29)) && ((val&(BIT(31)|PMS_MASK))==(BIT(31)|(133<<16)|(6<<8)|1)),1,10000);
  if(ret)return ret;
 
  val=readl(d->cmu+0x600);
  val=(val&~((3U<<8)|7U))|(1U<<8);
  writel(val,d->cmu+0x600);
  ret=readl_poll_timeout(d->cmu+0x700,val,!(val&(BIT(8)|BIT(4)|BIT(0))),1,10000);
  if(ret)return ret;
 
  sel=readl(d->cmu+0x200);
  sel&=~BIT(8);
  writel(sel,d->cmu+0x200);
  ret=readl_poll_timeout(d->cmu+0x400,val,!(val&BIT(10)),1,10000);
  if(ret)return ret;
 
  val=readl(d->cmu+0x800);
  writel(val|1,d->cmu+0x800);
 
  /* 4. SYSREG MCS 0x31 */
  ret=regmap_write(d->sysreg,0x1224,0x31);
  if(ret)return ret;
 
  return 0;
 }
 static void put_zone(void *data){put_device(&((struct thermal_zone_device *)data)->device);}
 static void put_pmic(void *data){put_device(data);}
 static void put_node(void *data){of_node_put(data);}
 static void put_clock(void *data){clk_put(data);}
 static void put_exclusive(void *data){clk_rate_exclusive_put(data);}
 static void put_bus_qos(void *data)
 {
  struct gpu_dvfs *d = data;
  if (d->bus_qos_active) {
   cancel_work_sync(&d->mif_work);
   dev_pm_qos_remove_request(&d->bus_qos_req);
   d->bus_qos_active = false;
  }
 }

static int gpu_probe(struct platform_device *pdev)
{
 struct gpu_dvfs *d;
 struct clk_init_data init={};
 struct clk *fin;
 u32 cap,selector,ramp,mcs;
 unsigned long initial_rate;
 int ret;
 if(!of_machine_is_compatible("meizu,m86") || !of_property_read_bool(pdev->dev.of_node,"meizu,retained-g3d-ownership"))return -ENODEV;
 d=devm_kzalloc(&pdev->dev,sizeof(*d),GFP_KERNEL);if(!d)return -ENOMEM;
 d->dev=&pdev->dev;mutex_init(&d->transaction);mutex_init(&d->hardware);
 d->active_voltage=of_property_read_bool(pdev->dev.of_node,"meizu,validated-buck6-voltage");
 d->normal_on_voltage=of_property_read_bool(pdev->dev.of_node,"meizu,validated-buck6-normal-on");
 if(d->active_voltage && d->normal_on_voltage)return -EINVAL;
 if(of_property_read_bool(pdev->dev.of_node,"meizu,gpu-normal-on-takeover") &&
    !d->normal_on_voltage)return -EINVAL;
 if(of_property_read_u32(pdev->dev.of_node,"meizu,max-validated-frequency",&cap))return -EINVAL;
 d->cap=cap;
 if(d->normal_on_voltage) {
  if(cap!=266000000 && cap!=350000000 && cap!=420000000 && cap!=544000000 && cap!=600000000 && cap!=700000000 && cap!=772000000)return -EINVAL;
 } else if(cap<266000000 || cap>772000000 || (!d->active_voltage && cap>266000000))return -EINVAL;
 d->gpu_node=of_parse_phandle(pdev->dev.of_node,"meizu,gpu",0);if(!d->gpu_node)return -EINVAL;
 ret=devm_add_action_or_reset(&pdev->dev,put_node,d->gpu_node);if(ret)return ret;
 if(of_find_property(d->gpu_node,"power-domains",NULL))
  return dev_err_probe(&pdev->dev,-EOPNOTSUPP,"GPU power domain is not retained\n");
 fin=devm_clk_get(&pdev->dev,"fin");if(IS_ERR(fin))return PTR_ERR(fin);
 if(clk_get_rate(fin)!=24000000)
  return dev_err_probe(&pdev->dev,-EOPNOTSUPP,"fin clock is %lu Hz, expected 24000000\n",clk_get_rate(fin));
 d->pmu=syscon_regmap_lookup_by_compatible("samsung,exynos7-pmu");if(IS_ERR(d->pmu))return PTR_ERR(d->pmu);
 d->sysreg=syscon_regmap_lookup_by_phandle(pdev->dev.of_node,"meizu,sysreg");if(IS_ERR(d->sysreg))return PTR_ERR(d->sysreg);
 d->cmu=devm_ioremap(&pdev->dev,0x14aa0000,0x1000);
 d->dvs=devm_ioremap(&pdev->dev,0x105c6100,8);
 d->gpio=devm_ioremap(&pdev->dev,0x13470000,4);
 d->topc=devm_ioremap(&pdev->dev,0x10570000,0x1000);
 if(!d->cmu||!d->dvs||!d->gpio||!d->topc)return -ENOMEM;
 d->pmic=bus_find_device_by_name(&i2c_bus_type,NULL,"5-0066");
 if(!d->pmic)return dev_err_probe(&pdev->dev,-EPROBE_DEFER,"PMIC 5-0066 is not present\n");
 ret=devm_add_action_or_reset(&pdev->dev,put_pmic,d->pmic);if(ret)return ret;
 if(!device_link_add(&pdev->dev,d->pmic,DL_FLAG_AUTOREMOVE_CONSUMER))return -EINVAL;
 if(!d->pmic->driver || !dev_get_regmap(d->pmic,NULL))return -EPROBE_DEFER;
 ret=validate_asv(d);
 if(ret)return dev_err_probe(&pdev->dev,ret,"GPU ASV handoff validation failed\n");
 if(!domain_on(d) || actual_rate(d)!=266000000 || !(readl(d->cmu+0x800)&1)) {
  bool on=domain_on(d);
  dev_info(&pdev->dev,"GPU cold power on: domain=%d rate=%lu coregate=%#x\n",
           on,on ? actual_rate(d) : 0,on ? readl(d->cmu+0x800) : 0);
  ret=m86_gpu_cold_power_on(d);
  if(ret)return dev_err_probe(&pdev->dev,ret,"GPU cold power on failed\n");
 }
 if(!domain_on(d))return dev_err_probe(&pdev->dev,-EHOSTDOWN,"retained G3D domain is off\n");
 initial_rate=actual_rate(d);
 if(initial_rate!=266000000 || !(readl(d->cmu+0x800)&1))
  return dev_err_probe(&pdev->dev,-EOPNOTSUPP,"retained GPU clock mismatch: rate=%lu coregate=%#x\n",
                       initial_rate,readl(d->cmu+0x800));
 d->current_rate=initial_rate;
 if(d->active_voltage || d->normal_on_voltage) {
  struct device_node *node;
  struct platform_device *supplier;
  int temp;
  if(!exynos_bus_get_dev("bus-mif") || !exynos_bus_get_dev("bus-int"))
   return -EPROBE_DEFER;
  /* GPU consumes these six always-on INT leaves. Logical INT QoS is not
   * one leaf'sMHz: compare vendor level's minimum vector without INT writes. */
  struct pro5_int_always_on_rates int_rates;
  ret=pro5_topc_int_read_stable(d->topc,24000000,&int_rates);if(ret)return ret;
  ret=pro5_topc_int_always_on_floor(&int_rates,200000);if(ret)return ret;
  node=of_parse_phandle(pdev->dev.of_node,"meizu,thermal-monitor",0);
  if(!node)return -EINVAL;
  supplier=of_find_device_by_node(node);of_node_put(node);
  if(!supplier)return -EPROBE_DEFER;
  if(!device_link_add(&pdev->dev,&supplier->dev,DL_FLAG_AUTOREMOVE_CONSUMER)) {
   put_device(&supplier->dev);return -EINVAL;
  }
  put_device(&supplier->dev);
  d->zone=thermal_zone_get_zone_by_name("m86-tmu-max");
  if(IS_ERR(d->zone))return -EPROBE_DEFER;
  get_device(&d->zone->device);
  ret=devm_add_action_or_reset(&pdev->dev,put_zone,d->zone);if(ret)return ret;
  ret=thermal_zone_get_temp(d->zone,&temp);if(ret)return ret;
  if(temp < -20000)return -ERANGE;
  /* Probe stays at 266MHz, the runtime thermal cap level. */
  if(temp>=75000)dev_warn(&pdev->dev,"GPU handoff at %d mC, held at 266MHz until it cools\n",temp);
 }
 ret=regmap_read(d->sysreg,0x1224,&mcs);if(ret)return ret;
 if(mcs!=0x31)return dev_err_probe(&pdev->dev,-EOPNOTSUPP,"GPU MCS=%#x, expected 0x31\n",mcs);
 /* A takeover may only run before any GPU consumer binds. Lock its device
  * across PMIC initialization; do not register clock/domain providers yet. */
 { struct platform_device *gpu=of_find_device_by_node(d->gpu_node);
  bool takeover=of_property_read_bool(pdev->dev.of_node,"meizu,gpu-normal-on-takeover");
  if(takeover && !gpu)return -EPROBE_DEFER;
  if(takeover)device_lock(&gpu->dev);
  device_lock(d->pmic);
 /* Device creation precedes PMIC binding/regmap publication during boot.
  * Retry handoff after supplier readiness; retain all hardware checks. */
 if(!d->pmic->driver || !dev_get_regmap(d->pmic,NULL))ret=-EPROBE_DEFER;
 else if(takeover && gpu->dev.driver)ret=-EBUSY;
 else if(takeover) {
  ret=takeover_normal_on(d);
  if(!ret)ret=check_rail(d,&selector,&ramp);
 } else ret=check_rail(d,&selector,&ramp);
 device_unlock(d->pmic);
 if(takeover)device_unlock(&gpu->dev);
 if(gpu)put_device(&gpu->dev);
 }
 if(ret)return dev_err_probe(&pdev->dev,ret,"GPU PMIC readiness/rail handoff check failed\n");
 if(selector!=d->asv_sel[0])return dev_err_probe(&pdev->dev,-EOPNOTSUPP,"GPU BUCK6 selector=%#x, expected %#x\n",selector, d->asv_sel[0]);
 d->cur_sel=selector;
 d->cur_ramp=ramp;
 init.name="m86_gpu_dvfs_clock";init.ops=&clock_ops;init.flags=CLK_GET_RATE_NOCACHE;
 { const struct clk_hw *parent=__clk_get_hw(fin); init.parent_hws=&parent;init.num_parents=1;
  d->hw.init=&init;ret=devm_clk_hw_register(&pdev->dev,&d->hw); }
 if(ret)return ret;
 d->clock=clk_hw_get_clk(&d->hw,NULL);if(IS_ERR(d->clock))return PTR_ERR(d->clock);
 ret=devm_add_action_or_reset(&pdev->dev,put_clock,d->clock);if(ret)return ret;
 ret=clk_rate_exclusive_get(d->clock);if(ret)return ret;
 ret=devm_add_action_or_reset(&pdev->dev,put_exclusive,d->clock);if(ret)return ret;
 ret=devm_of_clk_add_hw_provider(&pdev->dev,of_clk_hw_simple_get,&d->hw);if(ret)return ret;
 {
  struct device_node *bnode=of_parse_phandle(pdev->dev.of_node,"samsung,bus",0);
  if(bnode) {
   struct platform_device *bsup=of_find_device_by_node(bnode);
   of_node_put(bnode);
   if(bsup) {
    if(device_link_add(&pdev->dev,&bsup->dev,DL_FLAG_AUTOREMOVE_CONSUMER))
     d->bus_dev=&bsup->dev;
    put_device(&bsup->dev);
   }
  }
  if(!d->bus_dev)d->bus_dev=exynos_bus_get_dev("bus-mif");
  if(!d->bus_dev)d->bus_dev=exynos_bus_get_dev(NULL);
  if(d->bus_dev) {
   d->cur_mif_qos=d->want_mif=rates[0].mif;
   INIT_WORK(&d->mif_work,mif_work_fn);
   ret=dev_pm_qos_add_request(d->bus_dev,&d->bus_qos_req,DEV_PM_QOS_MIN_FREQUENCY,d->cur_mif_qos);
   if(ret >= 0) {
    d->bus_qos_active=true;
    ret=devm_add_action_or_reset(&pdev->dev,put_bus_qos,d);
    if(ret)return ret;
   }
  }
 }
 mutex_lock(&instance_lock);
 if(instance){mutex_unlock(&instance_lock);return -EBUSY;}
 instance=d;platform_set_drvdata(pdev,d);mutex_unlock(&instance_lock);
 dev_info(&pdev->dev,"retained GPU handoff: rate=%lu cap=%lu raw_selector=%#x voltage_writes=%u MCS=%#x\n",
  clk_get_rate(d->clock),d->cap,selector,d->active_voltage || d->normal_on_voltage,mcs);
 if(d->normal_on_voltage)dev_info(d->dev,"explicit Normal ON fixed-LDO protocol, cap %luMHz\n",d->cap/1000000);
 return 0;
}
static int gpu_remove(struct platform_device *pdev)
{
 struct gpu_dvfs *d = platform_get_drvdata(pdev);
 if (d && d->bus_qos_active) {
  cancel_work_sync(&d->mif_work);
  dev_pm_qos_remove_request(&d->bus_qos_req);
  d->bus_qos_active = false;
 }
 mutex_lock(&instance_lock);instance=NULL;mutex_unlock(&instance_lock);
 return 0; /* No voltage/clock/EMA changes during remove. */
}
static const struct of_device_id matches[]={{.compatible="meizu,m86-gpu-dvfs-retained"},{}};
MODULE_DEVICE_TABLE(of,matches);
static struct platform_driver gpu_driver={.probe=gpu_probe,.remove=gpu_remove,
 .driver={.name="m86-gpu-dvfs-retained",.of_match_table=matches,.suppress_bind_attrs=true}};
module_platform_driver(gpu_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Review-only retained-domain PRO5 GPU PLL/MCS/voltage transactions");
