/* SPDX-License-Identifier: GPL-2.0-only */
/* Pure decoder: caller supplies two identical snapshots from CMU_TOPC.
 * No clock writes, resource mapping, voltage changes or OPP vote is made here.
 * Vendor clk-exynos7420.c topc_group1/group2 and divider definitions.
 */
#ifndef PRO5_TOPC_INT_READBACK_H
#define PRO5_TOPC_INT_READBACK_H
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/io.h>
#include <linux/string.h>

/* TOPC base10570000, PLL CON0 offsets100/110/120/130 (CC/BUS0/BUS1/MFC).
 * mux0=200 mux2=208 mux3=20c div0=600 div1=604
 * div_busy0=700 div_busy1=704. Snapshot twice; reject changed fields.
 */
struct pro5_topc_int_snapshot {
 u32 pll[4], mux0, mux2, mux3, div0, div1, div_busy0, div_busy1;
 u32 mux_busy0, mux_busy2, mux_busy3, gate0, gate1;
};
struct pro5_int_always_on_rates {
 u64 bus0, bus1, bus1_low, pclk, imem, imem_low;
 u64 pclk_parent_hz;
};
static inline int pro5_topc_int_decode(const struct pro5_topc_int_snapshot *r,
                               u64 fin_hz,
                               struct pro5_int_always_on_rates *o)
{
 u64 src[4], half[4], group2[12];
 bool valid[4], half_valid[4], group_valid[12];
 u64 *out[] = { &o->bus0, &o->bus1, &o->bus1_low,
               &o->pclk, &o->imem, &o->imem_low };
 const unsigned int muxshift[] = {8, 12, 16, 28, 20, 24};
 const unsigned int divshift[] = {8, 12, 16, 28, 20, 24};
 unsigned int i, idx, div, halfsel;
 u32 m, p, s, con, selector, divider;

 /* All six leaf muxes are consumed, but unrelated source muxes are not. */
 if ((r->mux_busy2 & (BIT(11)|BIT(15)|BIT(19)|BIT(23)|BIT(27))) ||
     (r->mux_busy3 & BIT(31))) return -EAGAIN;
 if ((r->gate0 & (BIT(8)|BIT(12)|BIT(16)|BIT(20)|BIT(24))) !=
                  (BIT(8)|BIT(12)|BIT(16)|BIT(20)|BIT(24)) ||
     (r->gate1 & (BIT(28)|BIT(29))) != (BIT(28)|BIT(29)))
  return -EAGAIN;
 if (!fin_hz || (r->div_busy0 & (BIT(8)|BIT(12)|BIT(16)|BIT(20)|BIT(24))) ||
     (r->div_busy1 & BIT(28)))
  return -EAGAIN;
 /* Source order for group1/group2 is BUS0,BUS1,CC,MFC. */
 for (i=0; i<4; i++) {
  unsigned int pi = i == 0 ? 1 : i == 1 ? 2 : i == 2 ? 0 : 3;
  con = r->pll[pi];
  valid[i]=!(r->mux_busy0 & BIT(i*4+2));
  src[i]=0;
  if (!(r->mux0 & BIT(i*4))) src[i]=fin_hz;
  else {
   if ((con & (BIT(31)|BIT(29))) != (BIT(31)|BIT(29))) valid[i]=false;
   m=(con>>16)&0x3ff; p=(con>>8)&0x3f; s=con&7;
   if (!m || !p) valid[i]=false;
   else src[i]=div64_u64(fin_hz*m, (u64)p<<s);
  }
  halfsel=(r->mux0>>(16+i*4)) & (i==0 ? 3 : 1);
  half_valid[i]=valid[i] && halfsel<=2 &&
                !(r->mux_busy0 & BIT(i==0 ? 19 : 18+i*4));
  half[i]=halfsel<=2 ? src[i]>>halfsel : 0;
 }
 group2[0]=half[0]; group_valid[0]=half_valid[0];
 for (i=1;i<4;i++) { group2[i]=src[i]; group_valid[i]=valid[i]; }
 for (i=0;i<4;i++) {
  group2[i+4]=group2[i]/2;
  group2[i+8]=group2[i]/4;
  group_valid[i+4]=group_valid[i]; group_valid[i+8]=group_valid[i];
 }
 for (i=0;i<6;i++) {
  selector=i==3 ? r->mux3 : r->mux2;
  divider=i==3 ? r->div1 : r->div0;
  idx=(selector>>muxshift[i]) & (i<2 ? 15 : 3);
  div=((divider>>divshift[i])&15)+1;
  if (i<2) {
   if (idx>=12) return -EINVAL;
   if (!group_valid[idx]) return -EAGAIN;
   *out[i]=group2[idx]/div;
  } else {
   if (!half_valid[idx]) return -EAGAIN;
   *out[i]=half[idx]/div;
   if (i==3) o->pclk_parent_hz=half[idx];
  }
 }
 return 0;
}
static inline void pro5_topc_int_read(void __iomem *base,
                               struct pro5_topc_int_snapshot *r)
{
 r->pll[0]=readl(base+0x100); r->pll[1]=readl(base+0x110);
 r->pll[2]=readl(base+0x120); r->pll[3]=readl(base+0x130);
 r->mux0=readl(base+0x200); r->mux2=readl(base+0x208);
 r->mux3=readl(base+0x20c); r->div0=readl(base+0x600);
 r->div1=readl(base+0x604); r->div_busy0=readl(base+0x700);
 r->div_busy1=readl(base+0x704); r->mux_busy0=readl(base+0x400);
 r->mux_busy2=readl(base+0x408); r->mux_busy3=readl(base+0x40c);
 r->gate0=readl(base+0x800); r->gate1=readl(base+0x804);
}
static inline int pro5_topc_int_read_stable(void __iomem *base, u64 fin_hz,
                                   struct pro5_int_always_on_rates *rates)
{
 struct pro5_topc_int_snapshot a, b;
 pro5_topc_int_read(base, &a);
 pro5_topc_int_read(base, &b);
 if (memcmp(&a, &b, sizeof(a))) return -EAGAIN;
 return pro5_topc_int_decode(&a, fin_hz, rates);
}
/* Minimum always-on vector for vendor INT_LV10 (QoS200000kHz).
 * This is a hardware floor check, not a complete devfreq/voltage vote.
 */
/* Select the lowest vendor logical OPP meeting QoS, then compare its six
 * always-on physical rates. Do not compare QoS numerically with a leaf.
 */
static inline int pro5_topc_int_level(u32 qos_khz)
{
 static const u32 logical[]={560000,550000,540000,530000,520000,510000,
                            500000,400000,334000,266000,200000,100000};
 unsigned int level=0;
 if (!qos_khz || qos_khz>logical[0]) return -EINVAL;
 while(level+1<12 && logical[level+1]>=qos_khz) level++;
 return level;
}
static inline int pro5_topc_int_floor_minima(
 const struct pro5_int_always_on_rates *r, u32 qos_khz, u64 minimum[6])
{
 static const u16 mhz[][6] = {
  {532,532,200,134,266,167},{532,532,200,134,266,133},
  {400,400,200,100,200,133},{400,400,200,100,200,133},
  {400,400,200,100,200,133},{400,400,200,100,200,133},
  {400,400,200,100,200,133},{400,400,200,100,200,133},
  {334,334,167,80,167,100},{266,266,133,67,133,84},
  {200,200,100,50,100,50},{100,100,80,25,80,50},
 };
 /* Known vendor PLL table outputs after legal group1 half selections.
  * PCLK states=NULL: its inherited selected parent must be identified.
  * Unknown/bypassed/underclocked parents cannot lower the floor arbitrarily.
  */
 static const u16 pclk_parents_mhz[]={100,200,400,800,1600,334,668,266,532,234,468};
 unsigned int i;
 int level=pro5_topc_int_level(qos_khz);
 u64 parent=r->pclk_parent_hz, request, divisor;
 bool known=false;
 if (level<0) return level;
 for (i=0;i<6;i++) minimum[i]=(u64)mhz[level][i]*1000000ULL;
 /* IMEMlow LV9: BUS1_PLL668MHz / ceil(668/84)=8 =>83.5MHz. */
 if (level==9) minimum[5]=83500000ULL;
 for (i=0;i<sizeof(pclk_parents_mhz)/sizeof(pclk_parents_mhz[0]);i++)
  if (parent==(u64)pclk_parents_mhz[i]*1000000ULL) known=true;
 request=minimum[3];
 if (!known || parent<request) return -EOPNOTSUPP;
 divisor=div64_u64(parent+request-1,request);
 if (divisor>16) return -EOPNOTSUPP;
 minimum[3]=div64_u64(parent,divisor);
 return 0;
}
static inline int pro5_topc_int_always_on_floor(
 const struct pro5_int_always_on_rates *r, u32 qos_khz)
{
 u64 minimum[6];
 const u64 rates[]={r->bus0,r->bus1,r->bus1_low,r->pclk,r->imem,r->imem_low};
 unsigned int i;
 int ret=pro5_topc_int_floor_minima(r,qos_khz,minimum);
 if (ret) return ret;
 for(i=0;i<6;i++) if(rates[i]<minimum[i]) return -ERANGE;
 return 0;
}
static inline bool pro5_topc_int200_always_on_ok(const struct pro5_int_always_on_rates *r)
{
 return pro5_topc_int_always_on_floor(r,200000)==0;
}
#endif
