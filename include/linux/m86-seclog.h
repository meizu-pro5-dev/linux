/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_M86_SECLOG_H
#define _LINUX_M86_SECLOG_H

#include <linux/init.h>
#include <linux/printk.h>

#ifdef CONFIG_M86_SECLOG
void __init m86_seclog_init(void);
#define m86_seclog_mark(stage) \
	pr_notice("PRO5-SECLOG-seclog-r1: %s\n", stage)
#else
static inline void m86_seclog_init(void) { }
#define m86_seclog_mark(stage) do { } while (0)
#endif

#endif
