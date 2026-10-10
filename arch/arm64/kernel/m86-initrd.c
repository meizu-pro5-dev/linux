// SPDX-License-Identifier: GPL-2.0-only
/* Preserve U-Boot's initrd placement while retaining the private m86 DT. */
#define pr_fmt(fmt) "PRO5-INITRD-r2: " fmt

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/mm.h>
#include <linux/sizes.h>
#include <linux/types.h>
#include <asm/m86-initrd.h>
#include <asm/page.h>

/* The conservative 3 GiB map in the project's first-boot DTS. */
#define M86_DRAM_START 0x40000000ULL
#define M86_DRAM_END   0x100000000ULL

static bool __init m86_dram_range(u64 start, u64 size)
{
	return size && start >= M86_DRAM_START && start < M86_DRAM_END &&
	       size <= M86_DRAM_END - start;
}

static bool __init m86_overlap(u64 start, u64 end, u64 other, u64 size)
{
	/* Callers have already checked other + size for overflow. */
	return size && start < other + size && other < end;
}

static int __init m86_address(const void *fdt, int node, const char *name,
			     u64 *value, int *width)
{
	const void *p;
	int len;

	p = fdt_getprop(fdt, node, name, &len);
	if (!p || (len != 4 && len != 8))
		return -EINVAL;
	*value = len == 4 ? fdt32_ld(p) : fdt64_ld(p);
	*width = len;
	return 0;
}

static int __init m86_set_address(void *fdt, int node, const char *name,
				 u64 value, int width)
{
	if (width == 4) {
		if (value > U32_MAX)
			return -ERANGE;
		return fdt_setprop_inplace_u32(fdt, node, name, value);
	}
	if (width == 8)
		return fdt_setprop_inplace_u64(fdt, node, name, value);
	return -EINVAL;
}

static u64 __init m86_cells(const fdt32_t *p, int count)
{
	return count == 1 ? fdt32_ld(p) :
	       ((u64)fdt32_ld(p) << 32) | fdt32_ld(p + 1);
}

static int __init m86_reserved_conflict(const void *fdt, u64 start, u64 end)
{
	const fdt32_t *reg;
	const void *ranges;
	int parent, node, ac, sc, len, step, i;
	u64 base, size;

	parent = fdt_path_offset(fdt, "/reserved-memory");
	if (parent < 0)
		return -EINVAL;
	ac = fdt_address_cells(fdt, parent);
	sc = fdt_size_cells(fdt, parent);
	if ((ac != 1 && ac != 2) || (sc != 1 && sc != 2))
		return -EINVAL;
	ranges = fdt_getprop(fdt, parent, "ranges", &len);
	if (!ranges || len)
		return -EINVAL; /* Only the current identity-mapped reserved bus. */
	step = ac + sc;
	fdt_for_each_subnode(node, fdt, parent) {
		reg = fdt_getprop(fdt, node, "reg", &len);
		if (!reg || len <= 0 || len % (step * sizeof(*reg)))
			return -EINVAL; /* Dynamic reservations are not this contract. */
		for (i = 0; i < len / (int)sizeof(*reg); i += step) {
			base = m86_cells(reg + i, ac);
			size = m86_cells(reg + i + ac, sc);
			if (size > U64_MAX - base)
				return -EINVAL;
			if (m86_overlap(start, end, base, size)) {
				pr_err("reject overlap reserved-memory/%s [%#llx,%#llx)\n",
				       fdt_get_name(fdt, node, NULL), base, base + size);
				return -EBUSY;
			}
		}
	}
	return node == -FDT_ERR_NOTFOUND ? 0 : -EINVAL;
}

int __init m86_import_firmware_initrd(void *builtin, phys_addr_t builtin_pa,
				     phys_addr_t firmware_pa,
				     phys_addr_t kernel_start,
				     phys_addr_t kernel_end)
{
	const struct fdt_header *header;
	const unsigned char *gzip;
	const void *firmware = NULL;
	u64 old_start, old_end, start, end, expected_size;
	u32 total;
	int chosen, fw_chosen, start_width, end_width, fw_width, ret;
	const char *reason = "invalid embedded initrd properties";

	if (fdt_check_header(builtin))
		return -EINVAL; /* setup_machine_fdt also rejects an invalid tree. */
	chosen = fdt_path_offset(builtin, "/chosen");
	if (chosen < 0)
		return -EINVAL;
	ret = m86_address(builtin, chosen, "linux,initrd-start", &old_start,
			  &start_width);
	ret |= m86_address(builtin, chosen, "linux,initrd-end", &old_end,
			   &end_width);
	if (ret) {
		/* Prevent a surviving half of a malformed pair from being used. */
		fdt_nop_property(builtin, chosen, "linux,initrd-start");
		fdt_nop_property(builtin, chosen, "linux,initrd-end");
		pr_err("reject: %s; initrd disabled\n", reason);
		return -EINVAL;
	}

	pr_notice("firmware_fdt=%pa builtin_fdt=%pa template=[%#llx,%#llx)\n",
		  &firmware_pa, &builtin_pa, old_start, old_end);
	/* On every subsequent failure, stale fixed addresses stay disabled. */
	ret = m86_set_address(builtin, chosen, "linux,initrd-start", 0, start_width);
	ret |= m86_set_address(builtin, chosen, "linux,initrd-end", 0, end_width);
	if (ret)
		return -EINVAL;
	if (old_end <= old_start)
		goto reject;
	expected_size = old_end - old_start;
	if (expected_size < 3)
		goto reject;
	reason = "kernel or embedded FDT outside bounded DRAM";
	if (kernel_end <= kernel_start ||
	    !m86_dram_range(kernel_start, kernel_end - kernel_start) ||
	    !m86_dram_range(builtin_pa, fdt_totalsize(builtin)))
		goto reject;

	reason = "firmware FDT pointer outside bounded DRAM or unaligned";
	if ((firmware_pa & 7) ||
	    !m86_dram_range(firmware_pa, sizeof(*header)))
		goto reject;
	header = early_memremap(firmware_pa, sizeof(*header));
	reason = "cannot map firmware FDT header";
	if (!header)
		goto reject;
	total = fdt_totalsize(header);
	ret = fdt_magic(header) == FDT_MAGIC;
	early_memunmap((void *)header, sizeof(*header));
	reason = "firmware FDT size/magic or early mapping limit";
	if (!ret || total < sizeof(*header) ||
	    !m86_dram_range(firmware_pa, total) ||
	    total > SZ_256K - offset_in_page(firmware_pa))
		goto reject;
	reason = "firmware FDT overlaps kernel or embedded FDT";
	if (kernel_end <= kernel_start ||
	    m86_overlap(firmware_pa, firmware_pa + total, kernel_start,
			kernel_end - kernel_start) ||
	    m86_overlap(firmware_pa, firmware_pa + total, builtin_pa,
			fdt_totalsize(builtin)))
		goto reject;
	firmware = early_memremap(firmware_pa, total);
	reason = "firmware FDT header or complete mapping invalid";
	if (!firmware || fdt_totalsize(firmware) != total ||
	    fdt_check_header(firmware))
		goto reject;
	fw_chosen = fdt_path_offset(firmware, "/chosen");
	reason = "firmware chosen/initrd pair missing or not 32/64-bit cells";
	if (fw_chosen < 0 ||
	    m86_address(firmware, fw_chosen, "linux,initrd-start", &start, &fw_width) ||
	    m86_address(firmware, fw_chosen, "linux,initrd-end", &end, &fw_width))
		goto reject;
	pr_notice("firmware initrd=[%#llx,%#llx) expected_size=%#llx\n",
		  start, end, expected_size);
	early_memunmap((void *)firmware, total);
	firmware = NULL;
	reason = "initrd range/length outside build contract";
	if (end <= start || end - start != expected_size ||
	    !m86_dram_range(start, end - start))
		goto reject;
	reason = "initrd overlaps kernel, firmware FDT or embedded FDT";
	if (m86_overlap(start, end, kernel_start, kernel_end - kernel_start) ||
	    m86_overlap(start, end, firmware_pa, total) ||
	    m86_overlap(start, end, builtin_pa, fdt_totalsize(builtin)))
		goto reject;
	reason = "initrd conflicts with embedded reserved-memory contract";
	if (m86_reserved_conflict(builtin, start, end))
		goto reject;
	reason = "runtime address does not fit embedded property width";
	if ((start_width == 4 && start > U32_MAX) ||
	    (end_width == 4 && end > U32_MAX))
		goto reject;
	reason = "initrd gzip signature unavailable or invalid";
	gzip = early_memremap(start, 3);
	if (!gzip)
		goto reject;
	ret = gzip[0] == 0x1f && gzip[1] == 0x8b && gzip[2] == 8;
	early_memunmap((void *)gzip, 3);
	if (!ret)
		goto reject;
	ret = m86_set_address(builtin, chosen, "linux,initrd-start", start, start_width);
	ret |= m86_set_address(builtin, chosen, "linux,initrd-end", end, end_width);
	if (ret) {
		m86_set_address(builtin, chosen, "linux,initrd-start", 0, start_width);
		m86_set_address(builtin, chosen, "linux,initrd-end", 0, end_width);
		reason = "in-place initrd update failed";
		goto reject;
	}
	pr_notice("accepted initrd=[%#llx,%#llx) size=%#llx widths=%d/%d; private DT retained\n",
		  start, end, end - start, start_width, end_width);
	return 0;

reject:
	if (firmware)
		early_memunmap((void *)firmware, total);
	pr_err("reject: %s; initrd disabled\n", reason);
	return -EINVAL;
}
