/*
 *
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 * SPDX-License-Identifier: GPL-2.0
 *
 *//*
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include "linux/mman.h"
#include "../mali_kbase.h"

/* mali_kbase_mmap.c
 *
 * This file contains Linux specific implementation of
 * kbase_get_unmapped_area() interface.
 */


/**
 * kbase_unmapped_area_topdown() - allocates new areas top-down from
 *                                 below the stack limit.
 * @info:              Information about the memory area to allocate.
 * @is_shader_code:    Boolean which denotes whether the allocated area is
 *                      intended for the use by shader core in which case a
 *                      special alignment requirements apply.
 *
 * The unmapped_area_topdown() function in the Linux kernel is not exported
 * using EXPORT_SYMBOL_GPL macro. To allow us to call this function from a
 * module and also make use of the fact that some of the requirements for
 * the unmapped area are known in advance, we implemented an extended version
 * of this function and prefixed it with 'kbase_'.
 *
 * The difference in the call parameter list comes from the fact that
 * kbase_unmapped_area_topdown() is called with additional parameter which
 * is provided to denote whether the allocation is for a shader core memory
 * or not. This is significant since the executable shader core memory has
 * additional alignment requirements.
 *
 * The modification of the original Linux function lies in how the computation
 * of the highest gap address at the desired alignment is performed once the
 * gap with desirable properties is found. For this purpose a special function
 * is introduced (@ref align_and_check()) which beside computing the gap end
 * at the desired alignment also performs additional alignment check for the
 * case when the memory is executable shader core memory. For such case, it is
 * ensured that the gap does not end on a 4GB boundary.
 *
 * Return: address of the found gap end (high limit) if area is found;
 *         -ENOMEM if search is unsuccessful
*/

static unsigned long kbase_unmapped_area_topdown(struct vm_unmapped_area_info
		*info, bool is_shader_code)
{
	/*
	 * The original implementation was a private copy of the kernel's
	 * unmapped_area_topdown() that walked mm->mm_rb and vm_rb directly,
	 * because unmapped_area_topdown() was not exported. It was written for
	 * kernels before 4.17.
	 *
	 * Two things changed since:
	 *   - vm_unmapped_area() is exported and implements the same topdown
	 *     search, including the align_offset/align_mask computation that
	 *     align_and_check() used to duplicate;
	 *   - Linux 6.1 replaced the VMA rbtree (mm_rb, vm_rb, vm_prev,
	 *     rb_subtree_gap) with the maple tree, so the private walk cannot
	 *     be compiled at all.
	 *
	 * The search now uses the exported helper. The one constraint the
	 * generic helper cannot express is preserved here: executable shader
	 * memory must not cross a 4GB boundary, which the Mali GPU cannot
	 * address. That is a restriction on the result rather than on the
	 * search, so it is enforced by rejecting a candidate and resuming the
	 * search below the 4GB window that produced it. Each retry strictly
	 * lowers high_limit, so the loop terminates.
	 */
	unsigned int retries = 0;

	while (retries++ < 4096) {
		unsigned long addr = vm_unmapped_area(info);

		if (IS_ERR_VALUE(addr))
			return addr;

		if (!is_shader_code)
			return addr;

		/* "Ends on" or "starts at" a 4GB boundary counts as crossing
		 * it, matching the original align_and_check() test.
		 */
		if (addr & BASE_MEM_MASK_4GB)
			return addr;
		if ((addr + info->length) & BASE_MEM_MASK_4GB)
			return addr;

		/*
		 * Reject and resume in the 4GB window below this one. The
		 * original subtracted align_offset (or the length when there is
		 * no alignment offset) when nudging an aligned address; doing
		 * the same here also keeps the value aligned, which matters
		 * because vm_unmapped_area() re-applies align_mask on every
		 * call.
		 */
		{
			unsigned long step = info->align_offset ?
				info->align_offset : info->length;
			unsigned long boundary =
				(addr & ~BASE_MEM_MASK_4GB) - step;

			if (boundary < info->low_limit)
				return -ENOMEM;

			info->high_limit = boundary;
		}
	}

	return -ENOMEM;
}


/* This function is based on Linux kernel's arch_get_unmapped_area, but
 * simplified slightly. Modifications come from the fact that some values
 * about the memory area are known in advance.
 */
unsigned long kbase_get_unmapped_area(struct file *filp,
		const unsigned long addr, const unsigned long len,
		const unsigned long pgoff, const unsigned long flags)
{
	struct kbase_context *kctx = filp->private_data;
	struct mm_struct *mm = current->mm;
	struct vm_unmapped_area_info info;
	unsigned long align_offset = 0;
	unsigned long align_mask = 0;
	unsigned long high_limit = mm->mmap_base;
	unsigned long low_limit = PAGE_SIZE;
	int cpu_va_bits = BITS_PER_LONG;
	int gpu_pc_bits =
	      kctx->kbdev->gpu_props.props.core_props.log2_program_counter_size;
	bool is_shader_code = false;
	unsigned long ret;

	/* err on fixed address */
	if ((flags & MAP_FIXED) || addr)
		return -EINVAL;

#ifdef CONFIG_64BIT
	/* too big? */
	if (len > TASK_SIZE - SZ_2M)
		return -ENOMEM;

	if (!kbase_ctx_flag(kctx, KCTX_COMPAT)) {

		if (kbase_hw_has_feature(kctx->kbdev,
						BASE_HW_FEATURE_33BIT_VA)) {
			high_limit = kctx->same_va_end << PAGE_SHIFT;
		} else {
			high_limit = min_t(unsigned long, mm->mmap_base,
					(kctx->same_va_end << PAGE_SHIFT));
			if (len >= SZ_2M) {
				align_offset = SZ_2M;
				align_mask = SZ_2M - 1;
			}
		}

		low_limit = SZ_2M;
	} else {
		cpu_va_bits = 32;
	}
#endif /* CONFIG_64BIT */
	if ((PFN_DOWN(BASE_MEM_COOKIE_BASE) <= pgoff) &&
		(PFN_DOWN(BASE_MEM_FIRST_FREE_ADDRESS) > pgoff)) {
			int cookie = pgoff - PFN_DOWN(BASE_MEM_COOKIE_BASE);
			struct kbase_va_region *reg =
					kctx->pending_regions[cookie];

			if (!reg)
				return -EINVAL;

			if (!(reg->flags & KBASE_REG_GPU_NX)) {
				if (cpu_va_bits > gpu_pc_bits) {
					align_offset = 1ULL << gpu_pc_bits;
					align_mask = align_offset - 1;
					is_shader_code = true;
				}
			} else if (reg->flags & KBASE_REG_TILER_ALIGN_TOP) {
				unsigned long extent_bytes =
				     (unsigned long)(reg->extent << PAGE_SHIFT);
				/* kbase_check_alloc_sizes() already satisfies
				 * these checks, but they're here to avoid
				 * maintenance hazards due to the assumptions
				 * involved */
				WARN_ON(reg->extent > (ULONG_MAX >> PAGE_SHIFT));
				WARN_ON(reg->initial_commit > (ULONG_MAX >> PAGE_SHIFT));
				WARN_ON(!is_power_of_2(extent_bytes));
				align_mask = extent_bytes - 1;
				align_offset =
				      extent_bytes - (reg->initial_commit << PAGE_SHIFT);
			}
#ifndef CONFIG_64BIT
	} else {
		return current->mm->get_unmapped_area(filp, addr, len, pgoff,
						      flags);
#endif
	}

	info.flags = 0;
	info.length = len;
	info.low_limit = low_limit;
	info.high_limit = high_limit;
	info.align_offset = align_offset;
	info.align_mask = align_mask;

	ret = kbase_unmapped_area_topdown(&info, is_shader_code);

	if (IS_ERR_VALUE(ret) && high_limit == mm->mmap_base &&
			high_limit < (kctx->same_va_end << PAGE_SHIFT)) {
		/* Retry above mmap_base */
		info.low_limit = mm->mmap_base;
		info.high_limit = min_t(u64, TASK_SIZE,
					(kctx->same_va_end << PAGE_SHIFT));

		ret = kbase_unmapped_area_topdown(&info, is_shader_code);
	}

	return ret;
}
