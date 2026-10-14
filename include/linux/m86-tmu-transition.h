/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_M86_TMU_TRANSITION_H
#define _LINUX_M86_TMU_TRANSITION_H
/* Synchronous, sleepable, bounded callback. Must not call thermal, hwmon or
 * devfreq APIs. Serializes all monitor readers until GPU sensing is valid.
 */
int m86_tmu_gpu_transition(int (*transition)(void *), void *data);
int m86_tmu_gpu_power_off(int (*transition)(void *), void *data);
#endif
