// SPDX-License-Identifier: GPL-2.0-only
/*
 * PRO 5 diagnostic console using the existing SSCR CLOG/LLOG format.
 *
 * This address was independently reported by the vendor Android and Recovery
 * SECLOG drivers on the target device. The private m86 DT must reserve the
 * entire range with no-map. No read interface or firmware changes are added;
 * vendor Recovery rotates CLOG to LLOG and exposes it through /dev/panic_msg.
 */
#include <linux/build_bug.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/m86-seclog.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>

#include <asm/barrier.h>
#include <asm/cacheflush.h>
#include <asm/fixmap.h>

#define M86_SECLOG_BASE		0xf9780000ULL
#define M86_SECLOG_SIZE		SZ_512K
#define M86_SECLOG_BANK_SIZE	SZ_256K
#define M86_SECLOG_CLOG		0x474f4c43U
#define M86_SECLOG_LLOG		0x474f4c4cU

struct m86_seclog_header {
	__le32 magic;
	__le32 cursor;
	__le32 align_word;
	u8 full;
	u8 padding[3];
};

#define M86_SECLOG_CAPACITY \
	(M86_SECLOG_BANK_SIZE - sizeof(struct m86_seclog_header))

static struct m86_seclog_header *current_bank;
static DEFINE_RAW_SPINLOCK(m86_seclog_lock);

/* Payload must reach PoC before the cursor which makes it visible. */
static void notrace m86_seclog_persist(const void *addr, size_t len)
{
	unsigned long start = (unsigned long)addr;

	if (len)
		dcache_clean_poc(start, start + len);
	/* Complete writeback before reset, including header-only updates. */
	dsb(sy);
}

static void notrace m86_seclog_write(struct console *con, const char *s,
			     unsigned int count)
{
	struct m86_seclog_header *header = current_bank;
	unsigned long flags;
	unsigned int pos, first, remaining;
	char *data;
	bool wrapped;

	if (!header || !count)
		return;
	/* Avoid a recursive console/panic deadlock; never allocate or printk. */
	if (!raw_spin_trylock_irqsave(&m86_seclog_lock, flags))
		return;

	if (count > M86_SECLOG_CAPACITY) {
		s += count - M86_SECLOG_CAPACITY;
		count = M86_SECLOG_CAPACITY;
	}
	data = (char *)(header + 1);
	pos = le32_to_cpu(READ_ONCE(header->cursor));
	if (unlikely(pos >= M86_SECLOG_CAPACITY))
		goto out;
	first = min_t(unsigned int, count, M86_SECLOG_CAPACITY - pos);
	remaining = count - first;
	wrapped = count >= M86_SECLOG_CAPACITY - pos;

	memcpy(data + pos, s, first);
	m86_seclog_persist(data + pos, first);
	if (remaining) {
		memcpy(data, s + first, remaining);
		m86_seclog_persist(data, remaining);
	}
	pos += count;
	if (pos >= M86_SECLOG_CAPACITY)
		pos -= M86_SECLOG_CAPACITY;
	if (wrapped)
		WRITE_ONCE(header->full, 1);
	WRITE_ONCE(header->cursor, cpu_to_le32(pos));
	m86_seclog_persist(header, sizeof(*header));
out:
	raw_spin_unlock_irqrestore(&m86_seclog_lock, flags);
}

/*
 * A permanent console: neither the mapping nor callback lives in init memory.
 * No tty binding, so the real UART may still become /dev/console later.
 * Registering this real console early intentionally supersedes boot consoles.
 */
static struct console m86_seclog_console = {
	.name = "m86log",
	.write = m86_seclog_write,
	.flags = CON_ENABLED | CON_PRINTBUFFER | CON_ANYTIME,
	.index = -1,
};

static bool __init m86_seclog_bank_valid(struct m86_seclog_header *header)
{
	return le32_to_cpu(READ_ONCE(header->cursor)) < M86_SECLOG_CAPACITY &&
		READ_ONCE(header->full) <= 1;
}

void __init m86_seclog_init(void)
{
	static const char marker[] = "PRO5-SECLOG-seclog-r1: mapped-before-console\n";

	struct m86_seclog_header *bank[2];
	unsigned long base;
	unsigned int page, selected;
	int i, previous = -1;

	BUILD_BUG_ON(sizeof(struct m86_seclog_header) != 16);
	BUILD_BUG_ON(offsetof(struct m86_seclog_header, full) != 12);
	BUILD_BUG_ON(M86_SECLOG_BASE & (PAGE_SIZE - 1));
	BUILD_BUG_ON(M86_SECLOG_SIZE % PAGE_SIZE);
	BUILD_BUG_ON((FIX_M86_SECLOG_BEGIN - FIX_M86_SECLOG_END + 1) *
		     PAGE_SIZE != M86_SECLOG_SIZE);

	if (current_bank)
		return;
	/*
	 * Permanent fixmaps exist immediately after early_fixmap_init(). Their
	 * page tables are sized from FIXADDR_TOT_START and copied by
	 * fixmap_copy() during paging_init(), including any PMD crossing.
	 * Use normal cacheable RAM with explicit PoC writeback, never Device
	 * memory; the DT no-map reservation prevents a conflicting alias.
	 */
	for (page = 0; page < M86_SECLOG_SIZE / PAGE_SIZE; page++)
		__set_fixmap(FIX_M86_SECLOG_BEGIN - page,
			     M86_SECLOG_BASE + page * PAGE_SIZE, PAGE_KERNEL);
	base = fix_to_virt(FIX_M86_SECLOG_BEGIN);
	for (i = 0; i < 2; i++) {
		bank[i] = (void *)(base + i * M86_SECLOG_BANK_SIZE);
		if (previous < 0 &&
		    le32_to_cpu(READ_ONCE(bank[i]->magic)) == M86_SECLOG_CLOG &&
		    m86_seclog_bank_valid(bank[i]))
			previous = i;
	}
	/* Preserve a valid previous record even after an incomplete boot. */
	if (previous < 0)
		for (i = 0; i < 2; i++)
			if (le32_to_cpu(READ_ONCE(bank[i]->magic)) ==
			    M86_SECLOG_LLOG && m86_seclog_bank_valid(bank[i])) {
				previous = i;
				break;
			}

	selected = previous < 0 ? 0 : 1 - previous;
	memset(bank[selected], 0, M86_SECLOG_BANK_SIZE);
	m86_seclog_persist(bank[selected], M86_SECLOG_BANK_SIZE);
	if (previous >= 0)
		bank[previous]->magic = cpu_to_le32(M86_SECLOG_LLOG);
	else
		bank[1 - selected]->magic = 0;
	m86_seclog_persist(bank[1 - selected], sizeof(*bank[0]));
	/* Publish last: vendor Recovery must never observe two CLOG banks. */
	bank[selected]->magic = cpu_to_le32(M86_SECLOG_CLOG);
	m86_seclog_persist(bank[selected], sizeof(*bank[0]));
	current_bank = bank[selected];

	m86_seclog_write(&m86_seclog_console, marker, sizeof(marker) - 1);
	register_console(&m86_seclog_console);
	pr_notice("PRO5-SECLOG-seclog-r1: ready base=%#llx size=%#x bank=%u previous=%d capacity=%#lx\n",
		  M86_SECLOG_BASE, M86_SECLOG_SIZE, selected, previous,
		  (unsigned long)M86_SECLOG_CAPACITY);
}
