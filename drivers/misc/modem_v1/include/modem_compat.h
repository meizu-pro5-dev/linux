/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Linux 3.10 vendor API shims for the Samsung modem_v1 driver on Linux 6.6.
 */
#ifndef __MODEM_COMPAT_H__
#define __MODEM_COMPAT_H__

#include <linux/arm-smccc.h>
#include <linux/completion.h>
#include <linux/pm_wakeup.h>
#include <linux/sched/clock.h>
#include <linux/time64.h>
#include <linux/timekeeping.h>

#define WAKE_LOCK_SUSPEND	0

struct wake_lock {
	struct wakeup_source *ws;
};

static inline void wake_lock_init(struct wake_lock *lock, int type,
				  const char *name)
{
	lock->ws = wakeup_source_register(NULL, name);
}

static inline void wake_lock_destroy(struct wake_lock *lock)
{
	wakeup_source_unregister(lock->ws);
	lock->ws = NULL;
}

static inline void wake_lock(struct wake_lock *lock)
{
	__pm_stay_awake(lock->ws);
}

static inline void wake_lock_timeout(struct wake_lock *lock, long timeout)
{
	__pm_wakeup_event(lock->ws, jiffies_to_msecs(timeout));
}

static inline void wake_unlock(struct wake_lock *lock)
{
	__pm_relax(lock->ws);
}

static inline int wake_lock_active(struct wake_lock *lock)
{
	return lock->ws && lock->ws->active;
}

#define timespec		timespec64
#define getnstimeofday(ts)	ktime_get_real_ts64(ts)
#define INIT_COMPLETION(x)	reinit_completion(&(x))
#define ACCESS_ONCE(x)		READ_ONCE(x)
#define __devinit

/* CP secure boot service of the EL3 monitor (vendor SMC_CMD 0x82000700). */
static inline int exynos_smc(unsigned long cmd, unsigned long arg1,
			     unsigned long arg2, unsigned long arg3)
{
	struct arm_smccc_res res;

	arm_smccc_smc(cmd, arg1, arg2, arg3, 0, 0, 0, 0, &res);
	return (int)res.a0;
}

#define MAX_TRACE_SIZE		4096

#include "modem_debug.h"

#endif /* __MODEM_COMPAT_H__ */
