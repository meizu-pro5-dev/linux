/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_M86_FTS_H
#define __LINUX_M86_FTS_H

#include <linux/types.h>

#if IS_ENABLED(CONFIG_TOUCHSCREEN_M86_FTS)
void m86_fts_set_blank(bool blank);
#else
static inline void m86_fts_set_blank(bool blank) {}
#endif

#endif /* __LINUX_M86_FTS_H */
