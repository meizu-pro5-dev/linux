/*
 *
 * (C) COPYRIGHT 2010-2017 ARM Limited. All rights reserved.
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
 */

#ifndef _KBASE_DMA_FENCE_H_
#define _KBASE_DMA_FENCE_H_

#ifdef CONFIG_MALI_DMA_FENCE

#include <linux/list.h>
/* linux/reservation.h was removed in Linux 5.4; the reservation object API
 * lives in linux/dma-resv.h from that release onwards.
 */
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))

#include <linux/dma-resv.h>

/*
 * Compatibility shim for the 5.4 reservation-object rename.
 *
 * The driver was written against the pre-5.4 API. The object type and the
 * concept are unchanged - only the names move, and the shared/exclusive
 * distinction moves from the function name into an enum dma_resv_usage
 * argument. Mapping the old names here keeps the driver's fence handling
 * byte-for-byte identical to the vendor implementation and keeps the port
 * limited to the API boundary.
 *
 * The 1:1 mapping of behaviour:
 *   shared fence    -> DMA_RESV_USAGE_WRITE
 *   exclusive fence -> DMA_RESV_USAGE_WRITE (kernel internal bookkeeping)
 *   read fences     -> DMA_RESV_USAGE_READ
 *
 * Note: the vendor reservation_object_reserve_shared() took a single
 * argument, so the num_fences argument is supplied here as 1 - see the
 * matching call site in mali_kbase_dma_fence.c.
 */
#define reservation_object			dma_resv
#define reservation_object_lock(obj, ctx)	dma_resv_lock((obj), (ctx))
#define reservation_object_unlock(obj)		dma_resv_unlock(obj)
#define reservation_object_reserve_shared(obj)	dma_resv_reserve_fences((obj), 1)
#define reservation_object_add_shared_fence(obj, fence) \
	dma_resv_add_fence((obj), (fence), DMA_RESV_USAGE_WRITE)
#define reservation_object_add_excl_fence(obj, fence) \
	dma_resv_add_fence((obj), (fence), DMA_RESV_USAGE_WRITE)

static inline int
reservation_object_get_fences_rcu(struct dma_resv *obj,
				  struct dma_fence **pfence_excl,
				  unsigned *pshared_count,
				  struct dma_fence ***pshared)
{
	struct dma_fence **fences = NULL;
	unsigned int count = 0;
	int ret;

	*pfence_excl = NULL;
	*pshared_count = 0;
	*pshared = NULL;

	ret = dma_resv_get_fences(obj, DMA_RESV_USAGE_READ, &count, &fences);
	if (ret)
		return ret;

	if (count == 0) {
		kfree(fences);
		return 0;
	}

	/*
	 * The pre-5.4 helper handed back one exclusive fence plus an array of
	 * shared ones. dma_resv_get_fences() returns a single flat array, so
	 * split it: the first entry becomes the exclusive fence and the rest
	 * are moved one slot down in place to form the shared array.
	 *
	 * References: dma_resv_get_fences() returns one reference per fence.
	 * The exclusive slot takes the first reference; the shared array keeps
	 * the remaining references. The caller releases all of them with
	 * dma_fence_put() and frees the array.
	 */
	*pfence_excl = fences[0];
	dma_fence_get(*pfence_excl);

	if (count > 1) {
		memmove(fences, fences + 1, sizeof(*fences) * (count - 1));
		*pshared = fences;
		*pshared_count = count - 1;
	} else {
		kfree(fences);
	}

	return 0;
}

#else /* < 5.4 */

#include <linux/reservation.h>

#endif /* >= 5.4 */
#include <mali_kbase_fence.h>


/* Forward declaration from mali_kbase_defs.h */
struct kbase_jd_atom;
struct kbase_context;

/**
 * struct kbase_dma_fence_resv_info - Structure with list of reservation objects
 * @resv_objs:             Array of reservation objects to attach the
 *                         new fence to.
 * @dma_fence_resv_count:  Number of reservation objects in the array.
 * @dma_fence_excl_bitmap: Specifies which resv_obj are exclusive.
 *
 * This is used by some functions to pass around a collection of data about
 * reservation objects.
 */
struct kbase_dma_fence_resv_info {
	struct reservation_object **resv_objs;
	unsigned int dma_fence_resv_count;
	unsigned long *dma_fence_excl_bitmap;
};

/**
 * kbase_dma_fence_add_reservation() - Adds a resv to the array of resv_objs
 * @resv:      Reservation object to add to the array.
 * @info:      Pointer to struct with current reservation info
 * @exclusive: Boolean indicating if exclusive access is needed
 *
 * The function adds a new reservation_object to an existing array of
 * reservation_objects. At the same time keeps track of which objects require
 * exclusive access in dma_fence_excl_bitmap.
 */
void kbase_dma_fence_add_reservation(struct reservation_object *resv,
				     struct kbase_dma_fence_resv_info *info,
				     bool exclusive);

/**
 * kbase_dma_fence_wait() - Creates a new fence and attaches it to the resv_objs
 * @katom: Katom with the external dependency.
 * @info:  Pointer to struct with current reservation info
 *
 * Return: An error code or 0 if succeeds
 */
int kbase_dma_fence_wait(struct kbase_jd_atom *katom,
			 struct kbase_dma_fence_resv_info *info);

/**
 * kbase_dma_fence_cancel_ctx() - Cancel all dma-fences blocked atoms on kctx
 * @kctx: Pointer to kbase context
 *
 * This function will cancel and clean up all katoms on @kctx that is waiting
 * on dma-buf fences.
 *
 * Locking: jctx.lock needs to be held when calling this function.
 */
void kbase_dma_fence_cancel_all_atoms(struct kbase_context *kctx);

/**
 * kbase_dma_fence_cancel_callbacks() - Cancel only callbacks on katom
 * @katom: Pointer to katom whose callbacks are to be canceled
 *
 * This function cancels all dma-buf fence callbacks on @katom, but does not
 * cancel the katom itself.
 *
 * The caller is responsible for ensuring that jd_done_nolock is called on
 * @katom.
 *
 * Locking: jctx.lock must be held when calling this function.
 */
void kbase_dma_fence_cancel_callbacks(struct kbase_jd_atom *katom);

/**
 * kbase_dma_fence_signal() - Signal katom's fence and clean up after wait
 * @katom: Pointer to katom to signal and clean up
 *
 * This function will signal the @katom's fence, if it has one, and clean up
 * the callback data from the katom's wait on earlier fences.
 *
 * Locking: jctx.lock must be held while calling this function.
 */
void kbase_dma_fence_signal(struct kbase_jd_atom *katom);

/**
 * kbase_dma_fence_term() - Terminate Mali dma-fence context
 * @kctx: kbase context to terminate
 */
void kbase_dma_fence_term(struct kbase_context *kctx);

/**
 * kbase_dma_fence_init() - Initialize Mali dma-fence context
 * @kctx: kbase context to initialize
 */
int kbase_dma_fence_init(struct kbase_context *kctx);


#else /* CONFIG_MALI_DMA_FENCE */
/* Dummy functions for when dma-buf fence isn't enabled. */

static inline int kbase_dma_fence_init(struct kbase_context *kctx)
{
	return 0;
}

static inline void kbase_dma_fence_term(struct kbase_context *kctx) {}
#endif /* CONFIG_MALI_DMA_FENCE */
#endif
