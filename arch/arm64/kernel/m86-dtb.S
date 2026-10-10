/* SPDX-License-Identifier: GPL-2.0-only */
/* A private first-boot FDT; the shared firmware DTB remains untouched. */
	.section .rodata.m86_dtb,"a"
	.balign 8
	.global __m86_builtin_dtb
__m86_builtin_dtb:
	.incbin "arch/arm64/boot/dts/exynos/exynos7420-m86.dtb"
	.global __m86_builtin_dtb_end
__m86_builtin_dtb_end:
	.balign 8
