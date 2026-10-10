/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_M86_INITRD_H
#define __ASM_M86_INITRD_H

#include <linux/init.h>
#include <linux/types.h>

#ifdef CONFIG_M86_BUILTIN_DTB
int __init m86_import_firmware_initrd(void *builtin, phys_addr_t builtin_pa,
				     phys_addr_t firmware_pa,
				     phys_addr_t kernel_start,
				     phys_addr_t kernel_end);
#else
static inline int m86_import_firmware_initrd(void *builtin,
		phys_addr_t builtin_pa, phys_addr_t firmware_pa,
		phys_addr_t kernel_start, phys_addr_t kernel_end)
{
	return 0;
}
#endif
#endif
