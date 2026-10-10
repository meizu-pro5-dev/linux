/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef M86_CPU_RAIL_API_H
#define M86_CPU_RAIL_API_H
struct regulator;
struct clk;
/* cluster0=Atlas800MHz, cluster1=Apollo400MHz fixed staging.
 * Caller owns clk exclusivity, has checked staged rate + vendor EMA readback,
 * and must latch its whole policy on any error without voltage rollback.
 */
int m86_cpu_rail_set_coordinated(struct regulator *rail, struct clk *cpu_clock,
			       unsigned int uv, unsigned int cluster_index);
/* Trusted coordinator only: caller owns clock exclusivity, verifies exact
 * ASV OPP voltage and current EMA, and orders rise before clock/fall after clock.
 * PMIC wrapper does not independently establish an ASV minimum voltage.
 * Same-sync-only, exact verified old/target frequency authorization. */
int m86_cpu_rail_set_linear(struct regulator *rail, struct clk *cpu_clock,
			  unsigned int uv, unsigned int cluster_index,
			  unsigned long expected_hz);
#endif
