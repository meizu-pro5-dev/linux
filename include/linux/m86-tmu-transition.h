/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_M86_TMU_TRANSITION_H
#define _LINUX_M86_TMU_TRANSITION_H

#include <linux/notifier.h>
/* Cached maximum TMU temperature, live max read, and threshold-IRQ notifications. */
int m86_tmu_get_cached_temp(int *temp);
int m86_tmu_read_max_temp(int *temp);
int m86_tmu_register_notifier(struct notifier_block *nb);
int m86_tmu_unregister_notifier(struct notifier_block *nb);
int m86_tmu_gpu_power_off(void);
int m86_tmu_gpu_power_on(void);
#endif
