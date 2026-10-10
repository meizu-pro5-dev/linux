/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef M86_GPU_DVFS_API_H
#define M86_GPU_DVFS_API_H
struct device;
/* Called from devfreq target while devfreq lock is held. Takes only an ACTIVE
 * runtime PM reference, never resumes recursively through devfreq callbacks. */
int m86_gpu_devfreq_target(struct device *gpu, unsigned long *freq);
/* No runtime-PM acquisition; caller invokes before GPU reset/resume. */
int m86_gpu_resume_validate(struct device *gpu);
#endif
