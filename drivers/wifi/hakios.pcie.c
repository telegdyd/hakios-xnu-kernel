/*
 * SM8450 RC0 / QCA6490, C2 of wifi/WAVE0.md. No MHI, MSI or SMMU here.
 *
 * Write provenance (line numbers are physical lines of the local patch files):
 *  GCC GDSCR, PIPE_MUX, RCHNG, votes, CLKREF: slop 0120:118-130.
 *  GCC AUX XO rate: slop 0122:289-302. GCC BCR/PHY resets: 0120:160-164,177-204.
 *  TLMM GPIO output/config: slop 0120:140-144; eGPIO/CLKREQ: iOSPlus10 0262:83-118.
 *  BT_EN low / XO / WLAN_EN timing: 0262:127-153 (cnss2 power.c).
 *  Six RPMh resources/voltages: our live.dts:13859-13872, PMIC phandles;
 *   cmd-db lookup: slop 0109:37-91; TCS protocol: slop 0062:131-154,312-439,
 *   completed ACTIVE reuse / voltage-enable order: slop 0122:102-244.
 *  PARF RC, power, decode and halt: slop 0120:178-190, 0262:175-205.
 *  PHY writes INCLUDING power, reset, start: our live.dts:20833, in DT order.
 *  DBI RO enable, class, hotplug, bus, command: slop 0120:238-246.
 *  DBI target generation, retries/DLLLA+ELBI: 0262:271-329.
 *  iATU 0/1 fields: slop 0120:214-225,247-248.
 *  Endpoint BAR0 and decode/master: PCI config-space ABI; base/alignment from
 *   iOSPlus10 0261:306-359 and our live.dts:20788 ranges (2 MiB at 0x60400000).
 *  Failure host dump: iOSPlus10 0343:119-141. No read-clear chip registers.
 *  Down: reset pins first, disable decode/LTSSM, reverse register journal (shared
 *   GCC vote words: only the bits this driver set, re-read at down), then
 *   remove this client's rail enables in reverse order. Never restores GPIO
 *   output highs; never writes SMMU registers. Failed writes are journaled too.
 *
 * [V] DT addresses, PHY triples, supplies and GPIO numbers. [R] reference link
 * recipe/timing. [I] dedicated ACTIVE TCS2 reservation: current hakios SM8450
 * RSC writers use TCS0; checked under hakios_rsc_lock at claim. Keep this slot
 * exclusively ours until down so panic shutdown never needs that mutex.
 * Every MMIO access uses the shared SEA-catching core primitive. A kernel
 * without that primitive fails closed (ENOTSUP), with no unguarded fallback.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/hakios.pcie.h>
#include <sys/hakios.wifi.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
extern vm_offset_t ml_io_map_wcomb(vm_offset_t, vm_size_t);
extern lck_mtx_t hakios_rsc_lock;
extern int hakios_mmio_try_read32(const volatile void *, unsigned *) __attribute__((weak));
extern int hakios_mmio_try_write32(volatile void *, unsigned) __attribute__((weak));
extern kern_return_t thread_terminate(thread_t);
extern void loader_log_putc(char) __attribute__((weak));

#define GCC_PA 0x00100000UL
#define TLMM_PA 0x0f100000UL
#define PARF_PA 0x01c00000UL
#define PHY_PA 0x01c06000UL
#define DBI_PA 0x60000000UL
#define CFG_PA 0x60100000UL
#define BAR_PA 0x60400000UL
#define BAR_SIZE 0x200000UL
#define DB_PA 0x80860000UL
#define DB_SIZE 0x20000u
#define RSC_PA 0x17a20000UL
#define TCS_BASE 0xd00u
#define TCS_STRIDE 0x2a0u
#define TCS_SLOT 2u
#define TCS_BIT (1u << TCS_SLOT)
#define TCS(reg) (TCS_BASE + TCS_SLOT * TCS_STRIDE + (reg))
#define GDSCR 0x7b004
#define PIPE_MUX 0x7b060
#define RCHNG 0x7b048
#define AUX 0x7b064
#define VOTE0 0x62000
#define VOTE2 0x62008
#define VOTE0_BITS ((1u << 11) | (1u << 12) | (1u << 19) | (1u << 22))
#define PIN(n) ((n) * 0x1000u)
#define CHECK(call) do { int e_ = (call); if (e_) return e_; } while (0)

static vm_offset_t gcc, tlmm, parf, phy, dbi, cfg, db, rsc, bar;
static unsigned generation = 2, ncpt, rail_count;
static bool started, linked, endpoint, tcs_owned, tcs_poisoned;
/* mask 0: restore the whole old word. mask != 0: a shared vote word (GCC VOTE0/VOTE2,
 * also set by touch and the GPU); down clears only these bits, with a fresh read. */
static struct { vm_offset_t address; unsigned old, mask; } undo[512];
static unsigned undo_count;
static struct { const char *name; unsigned mv, address; } rails[] = {
	{ "smpe2", 1012, 0 }, { "smpb11", 966, 0 }, { "smpb10", 1800, 0 },
	{ "smpc1", 1900, 0 }, { "smpb12", 1350, 0 }, { "ldoe7", 2800, 0 }
};
static const unsigned short phy_sequence[][3] = {
	{ 0x240, 0x3, 0 },
	{ 0x94, 0x8, 0 },
	{ 0x154, 0x34, 0 },
	{ 0x16c, 0x8, 0 },
	{ 0x58, 0xf, 0 },
	{ 0xa4, 0x42, 0 },
	{ 0x110, 0x24, 0 },
	{ 0x11c, 0x3, 0 },
	{ 0x118, 0xb4, 0 },
	{ 0x10c, 0x2, 0 },
	{ 0x1bc, 0x11, 0 },
	{ 0xbc, 0x82, 0 },
	{ 0xd4, 0x3, 0 },
	{ 0xd0, 0x55, 0 },
	{ 0xcc, 0x55, 0 },
	{ 0xb0, 0x1a, 0 },
	{ 0xac, 0xa, 0 },
	{ 0xc4, 0x68, 0 },
	{ 0xe0, 0x2, 0 },
	{ 0xdc, 0xaa, 0 },
	{ 0xd8, 0xab, 0 },
	{ 0xb8, 0x34, 0 },
	{ 0xb4, 0x14, 0 },
	{ 0x158, 0x1, 0 },
	{ 0x74, 0x6, 0 },
	{ 0x7c, 0x16, 0 },
	{ 0x84, 0x36, 0 },
	{ 0x78, 0x6, 0 },
	{ 0x80, 0x16, 0 },
	{ 0x88, 0x36, 0 },
	{ 0x1b0, 0x1e, 0 },
	{ 0x1ac, 0xca, 0 },
	{ 0x1b8, 0x18, 0 },
	{ 0x1b4, 0xa2, 0 },
	{ 0x50, 0x7, 0 },
	{ 0x10, 0x1, 0 },
	{ 0x1c, 0x31, 0 },
	{ 0x20, 0x1, 0 },
	{ 0x24, 0xde, 0 },
	{ 0x28, 0x7, 0 },
	{ 0x30, 0x4c, 0 },
	{ 0x34, 0x6, 0 },
	{ 0xee4, 0x20, 0 },
	{ 0xe84, 0x75, 0 },
	{ 0xe90, 0x3f, 0 },
	{ 0x115c, 0x7f, 0 },
	{ 0x1160, 0xff, 0 },
	{ 0x1164, 0xbf, 0 },
	{ 0x1168, 0x3f, 0 },
	{ 0x116c, 0xd8, 0 },
	{ 0x1170, 0xdc, 0 },
	{ 0x1174, 0xdc, 0 },
	{ 0x1178, 0x5c, 0 },
	{ 0x117c, 0x34, 0 },
	{ 0x1180, 0xa6, 0 },
	{ 0x1190, 0x34, 0 },
	{ 0x1194, 0x38, 0 },
	{ 0x10d8, 0xf, 0 },
	{ 0xe3c, 0x12, 0 },
	{ 0xe40, 0x1, 0 },
	{ 0x10dc, 0x0, 0 },
	{ 0x104c, 0x8, 0 },
	{ 0x1050, 0x8, 0 },
	{ 0x1044, 0xf0, 0 },
	{ 0x11a4, 0x38, 0 },
	{ 0x10cc, 0xf0, 0 },
	{ 0x10f4, 0x7, 0 },
	{ 0x1008, 0x9, 0 },
	{ 0x1014, 0x5, 0 },
	{ 0x694, 0x0, 0 },
	{ 0x654, 0x0, 0 },
	{ 0x6a8, 0xf, 0 },
	{ 0x48, 0x90, 0 },
	{ 0x620, 0xc1, 0 },
	{ 0x388, 0x77, 0 },
	{ 0x398, 0xb, 0 },
	{ 0x2dc, 0x5, 0 },
	{ 0x200, 0x0, 0 },
	{ 0x244, 0x3, 0 },
};

static void
say(const char *format, ...) __printflike(1, 2);
static void
say(const char *format, ...)
{
	char line[320];
	va_list ap;
	va_start(ap, format);
	vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);
	printf("%s", line);
	if (loader_log_putc) for (const char *p = line; *p; p++) loader_log_putc(*p);
}

int
hakios_pcie_try_read32(const volatile void *va, unsigned *value)
{
	if (!va || !value || ((uintptr_t)va & 3)) return EINVAL;
	if (!hakios_mmio_try_read32) return ENOTSUP;
	return hakios_mmio_try_read32(va, value);
}
int
hakios_pcie_try_write32(volatile void *va, unsigned value)
{
	if (!va || ((uintptr_t)va & 3)) return EINVAL;
	if (!hakios_mmio_try_write32) return ENOTSUP;
	return hakios_mmio_try_write32(va, value);
}
static int rd(vm_offset_t a, unsigned *v) { return hakios_pcie_try_read32((const volatile void *)a, v); }
static int wr(vm_offset_t a, unsigned v) { return hakios_pcie_try_write32((volatile void *)a, v); }

/* Read before every write. Journal before attempting it: an abort may be imprecise
 * about whether the device took the store. Command/W1C registers use direct wr. */
static int
save_write(vm_offset_t a, unsigned v)
{
	unsigned old;
	CHECK(rd(a, &old));
	if (undo_count == sizeof(undo) / sizeof(undo[0])) return ENOSPC;
	undo[undo_count].address = a;
	undo[undo_count].mask = 0;
	undo[undo_count++].old = old;
	return wr(a, v);
}
/* Shared vote words: journal only the bits that were 0 and that we set. [I] the
 * read-modify-write is not locked against touch/GPU; their votes are one-shot. */
static int
vote_bits(vm_offset_t a, unsigned set)
{
	unsigned old, mine;
	CHECK(rd(a, &old));
	if (!(mine = set & ~old)) return 0;
	if (undo_count == sizeof(undo) / sizeof(undo[0])) return ENOSPC;
	undo[undo_count].address = a;
	undo[undo_count].mask = mine;
	undo[undo_count++].old = 0;
	return wr(a, old | mine);
}
static int
vote_clear(vm_offset_t a, unsigned mine)
{
	unsigned v;
	CHECK(rd(a, &v));
	return wr(a, v & ~mine);
}
static int
bits(vm_offset_t a, unsigned clear, unsigned set)
{
	unsigned v;
	CHECK(rd(a, &v));
	return save_write(a, (v & ~clear) | set);
}
static int
wait_bits(vm_offset_t a, unsigned mask, unsigned want, unsigned us)
{
	unsigned v;
	for (unsigned elapsed = 0; elapsed < us; elapsed += 100) {
		CHECK(rd(a, &v));
		if (v != ~0u && (v & mask) == want) return 0;
		delay(100);
	}
	return ETIMEDOUT;
}
static int
gpio(unsigned pin, bool high)
{
	unsigned config, io;
	CHECK(rd(tlmm + PIN(pin), &config));
	CHECK(rd(tlmm + PIN(pin) + 4, &io));
	/* Preserve other bits; Gunyah requires STR Wn rather than STR WZR. */
	CHECK(wr(tlmm + PIN(pin) + 4, (io & ~2u) | (high ? 2u : 0)));
	return wr(tlmm + PIN(pin), (7u << 6) | (1u << 9) |
	    ((config & (1u << 11)) ? (1u << 12) : 0));
}
static int
map_blocks(void)
{
	/* Map only on up. Cached mappings survive down; borrowed BAR access does not. */
	if (!gcc) gcc = ml_io_map(GCC_PA, 0x100000);
	if (!tlmm) tlmm = ml_io_map(TLMM_PA, 0x100000);
	if (!parf) parf = ml_io_map(PARF_PA, 0x3000);
	if (!phy) phy = ml_io_map(PHY_PA, 0x2000);
	if (!dbi) dbi = ml_io_map(DBI_PA, 0x2000);
	if (!cfg) cfg = ml_io_map(CFG_PA, 0x1000);
	if (!rsc) rsc = ml_io_map(RSC_PA, 0x10000);
	if (!db) db = ml_io_map_wcomb(DB_PA, DB_SIZE);
	return gcc && tlmm && parf && phy && dbi && cfg && rsc && db ? 0 : ENOMEM;
}
static int
db16(unsigned off, unsigned *v)
{
	unsigned word;
	if (off > DB_SIZE - 2 || (off & 1)) return EINVAL;
	CHECK(rd(db + (off & ~3u), &word));
	*v = (word >> (8 * (off & 2))) & 0xffff;
	return 0;
}
static int
resource(const char *name, unsigned *address)
{
	unsigned key[2] = { 0 }, magic;
	for (unsigned i = 0; name[i] && i < 8; i++) key[i / 4] |= (unsigned)(unsigned char)name[i] << (8 * (i % 4));
	CHECK(rd(db + 4, &magic));
	if (magic != 0x0c0330db) return ENODEV;
	for (unsigned s = 0; s < 8; s++) {
		unsigned id, header, count, a, b, addr;
		CHECK(db16(8 + 16 * s, &id));
		CHECK(db16(10 + 16 * s, &header));
		CHECK(db16(14 + 16 * s, &count));
		if (id != 4) continue; /* VRM, not ARC or BCM */
		if (count > 1024 || header > DB_SIZE - 144 || count > (DB_SIZE - 144 - header) / 24) return EINVAL;
		for (unsigned j = 0; j < count; j++) {
			unsigned e = 144 + header + 24 * j;
			CHECK(rd(db + e, &a)); CHECK(rd(db + e + 4, &b));
			if (a != key[0] || b != key[1]) continue;
			CHECK(rd(db + e + 16, &addr));
			if (!addr || (addr & 15)) return EINVAL;
			*address = addr;
			return 0;
		}
	}
	return ENOENT;
}
static int
tcs_idle(unsigned slot)
{
	unsigned base = TCS_BASE + slot * TCS_STRIDE, idle, control, enabled;
	CHECK(rd(rsc + base + 0x18, &idle));
	CHECK(rd(rsc + base + 0x14, &control));
	CHECK(rd(rsc + base + 0x1c, &enabled));
	if (!idle || idle == ~0u || (control & (1u << 24)) || (enabled >> ncpt)) return EBUSY;
	for (unsigned c = 0; c < ncpt; c++) if (enabled & (1u << c)) {
		unsigned status;
		CHECK(rd(rsc + base + 0x3c + 20 * c, &status));
		if (status == ~0u || (status & 0x10100) != 0x10100) return EBUSY;
	}
	return 0;
}
static int
claim_tcs(void)
{
	unsigned id, config;
	CHECK(rd(rsc, &id)); CHECK(rd(rsc + 0x0c, &config));
	ncpt = (config >> 27) & 31;
	if (((id >> 16) & 255) < 1 || ((id >> 16) & 255) > 2 || config == ~0u ||
	    ((config >> 12) & 63) < 7 || ncpt < 3 || ncpt > 16) return ENODEV;
	/* Other in-tree SM8450 clients use slot 0. Never steal an armed sleep/wake
	 * slot, nor an outstanding ACTIVE request matching one of our rails. */
	for (unsigned s = 0; s < 3; s++) {
		unsigned enabled;
		CHECK(rd(rsc + TCS_BASE + s * TCS_STRIDE + 0x1c, &enabled));
		int idle_error = tcs_idle(s);
		if (!idle_error) continue;
		if (idle_error != EBUSY) return idle_error;
		for (unsigned c = 0; c < ncpt; c++) if (enabled & (1u << c)) {
			unsigned a;
			CHECK(rd(rsc + TCS_BASE + s * TCS_STRIDE + 0x34 + 20 * c, &a));
			for (unsigned i = 0; i < 6; i++) if ((a & ~15u) == rails[i].address) return EBUSY;
		}
	}
	CHECK(tcs_idle(TCS_SLOT));
	tcs_owned = true;
	return 0;
}
static int
tcs_sync(unsigned off, unsigned v)
{
	unsigned old, actual;
	CHECK(rd(rsc + off, &old));
	CHECK(wr(rsc + off, v));
	for (unsigned i = 0; i < 1000; i++) {
		CHECK(rd(rsc + off, &actual));
		if (actual == v) return 0;
		delay(1);
	}
	return ETIMEDOUT;
}
static int
vote(unsigned index, bool on)
{
	unsigned address = rails[index].address, count = on ? 2 : 1;
	unsigned offsets[2] = { on ? 0u : 4u, 4u }, values[2] = { on ? rails[index].mv : 0u, 1u };
	unsigned irq, idle, status, mask = (1u << count) - 1;
	if (!tcs_owned || tcs_poisoned) return EBUSY;
	CHECK(tcs_idle(TCS_SLOT));
	tcs_poisoned = true; /* do not recycle an uncertain transaction */
	CHECK(tcs_sync(TCS(0x14), 0));
	CHECK(tcs_sync(TCS(0x1c), 0));
	CHECK(tcs_sync(TCS(0x10), 0));
	CHECK(wr(rsc + TCS_BASE + 8, TCS_BIT));
	CHECK(rd(rsc + TCS_BASE + 4, &irq));
	if (irq & TCS_BIT) return EIO;
	for (unsigned c = 0; c < count; c++) {
		CHECK(tcs_sync(TCS(0x30 + 20 * c), 0x10108));
		CHECK(tcs_sync(TCS(0x34 + 20 * c), address + offsets[c]));
		CHECK(tcs_sync(TCS(0x38 + 20 * c), values[c]));
	}
	CHECK(tcs_sync(TCS(0x10), mask));
	CHECK(tcs_sync(TCS(0x1c), mask));
	CHECK(tcs_sync(TCS(0x14), 1u << 16));
	/* IRQ completion status is polled; do not modify the shared IRQ-enable word.
	 * IRQ_STATUS is latched independently of IRQ_ENABLE (rpmh-rsc layout). */
	CHECK(wr(rsc + TCS(0x14), (1u << 16) | (1u << 24)));
	bool done = false;
	for (unsigned us = 0; us < 100000; us += 50) {
		CHECK(rd(rsc + TCS_BASE + 4, &irq));
		CHECK(rd(rsc + TCS(0x18), &idle));
		done = (irq & TCS_BIT) && idle && idle != ~0u;
		for (unsigned c = 0; c < count; c++) {
			CHECK(rd(rsc + TCS(0x3c + 20 * c), &status));
			done &= status != ~0u && (status & 0x10100) == 0x10100;
		}
		if (done) break;
		delay(50);
	}
	if (!done) return ETIMEDOUT;
	CHECK(tcs_sync(TCS(0x14), 0));
	CHECK(tcs_sync(TCS(0x1c), 0));
	CHECK(tcs_sync(TCS(0x10), 0));
	CHECK(wr(rsc + TCS_BASE + 8, TCS_BIT));
	tcs_poisoned = false;
	return 0;
}
static int
power(void)
{
	unsigned v;
	CHECK(gpio(94, false)); CHECK(gpio(80, false));
	CHECK(gpio(81, false)); /* Bluetooth bring-up is separate; assert off. */
	CHECK(rd(tlmm + PIN(95), &v));
	CHECK(save_write(tlmm + PIN(95), 1u << 2 | 3u | ((v & (1u << 11)) ? 1u << 12 : 0)));
	for (unsigned i = 0; i < 6; i++) CHECK(resource(rails[i].name, &rails[i].address));
	lck_mtx_lock(&hakios_rsc_lock);
	int error = claim_tcs();
	lck_mtx_unlock(&hakios_rsc_lock);
	if (error) return error;
	for (unsigned i = 0; i < 6; i++) {
		rail_count = i + 1; /* uncertain enable is still ours to remove */
		CHECK(vote(i, true));
	}
	delay(100000);
	CHECK(gpio(204, false)); delay(10000);
	CHECK(gpio(204, true)); delay(200);
	CHECK(gpio(80, true)); delay(6000);
	CHECK(gpio(204, false)); delay(50000);
	return 0;
}
static int
clocks_phy(void)
{
	CHECK(bits(gcc + GDSCR, 1, 0));
	CHECK(wait_bits(gcc + GDSCR, 1u << 31, 1u << 31, 10000));
	CHECK(save_write(gcc + PIPE_MUX, 2));
	CHECK(save_write(gcc + RCHNG + 4, 1u << 8 | 11));
	CHECK(bits(gcc + RCHNG, 0, 1)); CHECK(wait_bits(gcc + RCHNG, 1, 0, 10000));
	CHECK(save_write(gcc + AUX + 4, 1));
	CHECK(bits(gcc + AUX, 0, 1)); CHECK(wait_bits(gcc + AUX, 1, 0, 10000));
	CHECK(vote_bits(gcc + VOTE0, VOTE0_BITS));
	CHECK(vote_bits(gcc + VOTE2, 0x2f));
	CHECK(bits(gcc + 0x9c004, 0, 1)); delay(100);
	CHECK(wait_bits(gcc + 0x7b030, 1u << 31, 0, 10000));
	CHECK(wait_bits(gcc + 0x7b034, 1u << 31, 0, 10000));
	CHECK(save_write(gcc + 0x7b000, 1)); delay(1000);
	CHECK(save_write(gcc + 0x7b000, 0)); delay(1000);
	CHECK(save_write(parf + 0x1000, 4));
	CHECK(bits(parf + 0x40, 1, 0));
	/* Consistent vendor decode recipe, not Linux addresses mixed with vendor size. */
	CHECK(save_write(parf + 0x350, 0));
	CHECK(save_write(parf + 0x358, 0x4000000));
	CHECK(bits(parf, 1u << 29, 0));
	CHECK(bits(parf + 0x174, 0, 1u << 4));
	CHECK(bits(parf + 0x20, 1u << 5, 0));
	CHECK(bits(parf + 0x1a8, 0, 1u << 31));
	CHECK(save_write(gcc + 0x7c01c, 1));
	CHECK(save_write(gcc + 0x7c020, 1)); delay(300);
	CHECK(save_write(gcc + 0x7c01c, 0));
	for (unsigned i = 0; i < sizeof(phy_sequence) / sizeof(phy_sequence[0]); i++) {
		/* The final DT SW_RESET/START writes follow NOCSR reset release below. */
		if (phy_sequence[i][0] == 0x200) CHECK(save_write(gcc + 0x7c020, 0));
		CHECK(save_write(phy + phy_sequence[i][0], phy_sequence[i][1]));
		if (phy_sequence[i][2]) delay(phy_sequence[i][2]);
	}
	delay(1200);
	CHECK(save_write(gcc + PIPE_MUX, 0)); CHECK(vote_bits(gcc + VOTE2, 1u << 4));
	CHECK(wait_bits(phy + 0x214, 1u << 6, 0, 20000));
	unsigned id;
	CHECK(rd(dbi, &id));
	return id == ~0u ? ENODEV : 0;
}
static int
atu(unsigned index, unsigned type, unsigned from, unsigned limit, unsigned to)
{
	vm_offset_t a = dbi + 0x1000 + (index << 9);
	CHECK(save_write(a + 8, from)); CHECK(save_write(a + 12, 0));
	CHECK(save_write(a + 16, limit)); CHECK(save_write(a + 32, 0));
	CHECK(save_write(a + 20, to)); CHECK(save_write(a + 24, 0));
	CHECK(save_write(a, type)); CHECK(save_write(a + 4, 1u << 31));
	return wait_bits(a + 4, 1u << 31, 1u << 31, 10000);
}
static int
train(struct hakios_pcie_link *link)
{
	unsigned status = 0, sys = 0;
	CHECK(bits(dbi + 0x8bc, 0, 1));
	CHECK(bits(dbi + 8, ~0xffu, 0x0604u << 16));
	CHECK(bits(dbi + 0x84, 1u << 6, 0));
	CHECK(bits(dbi + 0x8bc, 1, 0));
	CHECK(save_write(dbi + 0x10, 0)); CHECK(save_write(dbi + 0x14, 0));
	CHECK(save_write(dbi + 0x18, 0x10100));
	/* PCI command/status has W1C bits in upper half; never echo them back. */
	unsigned old_command;
	CHECK(rd(dbi + 4, &old_command));
	if (undo_count == sizeof(undo) / sizeof(undo[0])) return ENOSPC;
	undo[undo_count].address = dbi + 4;
	undo[undo_count].mask = 0;
	undo[undo_count++].old = old_command & 0xffff;
	CHECK(hakios_pcie_cfg_write(0, 0, 4, 2, (old_command & 0xffff) | 6));
	CHECK(bits(dbi + 0xa0, 15, generation));
	CHECK(atu(0, 4, CFG_PA, CFG_PA + 0xfffff, 1u << 24));
	CHECK(atu(1, 0, 0x60300000, 0x63ffffff, 0x60300000));
	for (unsigned t = 0; t < 3; t++) {
		delay(100000); CHECK(gpio(94, true)); delay(1000);
		CHECK(bits(parf + 0x1b0, 0, 1u << 8));
		for (unsigned us = 0; us < 1000000; us += 10000) {
			CHECK(rd(dbi + 0x80, &status)); CHECK(rd(dbi + 0xf28, &sys));
			if (status != ~0u && sys != ~0u && (status & (1u << 29)) && (sys & (1u << 10))) {
				link->gen = (status >> 16) & 15; link->width = (status >> 20) & 63;
				if (!link->gen || link->gen > generation || link->width != 1) return EIO;
				linked = true;
				return 0;
			}
			delay(10000);
		}
		CHECK(bits(parf + 0x1b0, 1u << 8, 0)); CHECK(gpio(94, false));
	}
	return ETIMEDOUT;
}

/* Config accesses are serialized by their caller, same as up/down. Only bus0
 * root port and bus1/device0/function0 exist. Endpoint forbidden until link up. */
static int
config_address(unsigned rc, unsigned bdf, unsigned off, unsigned size, vm_offset_t *a)
{
	if (rc) return ENODEV;
	if ((size != 1 && size != 2 && size != 4) || (off & (size - 1)) || off > 4096 - size) return EINVAL;
	if (!started || !dbi) return ENXIO;
	if (bdf == 0) *a = dbi + (off & ~3u);
	else if (bdf == HAKIOS_PCIE_WIFI_BDF && linked) *a = cfg + (off & ~3u);
	else return ENODEV;
	return 0;
}
int
hakios_pcie_cfg_read(unsigned rc, unsigned bdf, unsigned off, unsigned size, unsigned *value)
{
	vm_offset_t a;
	unsigned v;
	if (!value) return EINVAL;
	CHECK(config_address(rc, bdf, off, size, &a)); CHECK(rd(a, &v));
	*value = (v >> (8 * (off & 3))) & (size == 4 ? ~0u : ((1u << (8 * size)) - 1));
	return 0;
}
/* W1C bits outside a narrow store must be written as zero. Discover standard
 * capability position rather than assuming endpoint/root-port layouts match.
 * These are PCI/PCIe-defined status registers; MSI control has no W1C bits. */
static int
config_w1c(unsigned rc, unsigned bdf, unsigned aligned, unsigned *mask)
{
	unsigned ptr, header;
	*mask = aligned == 4 ? 0xf9000000u : 0;
	if (bdf == 0 && aligned == 0x1c) *mask = 0xf9000000u;
	CHECK(hakios_pcie_cfg_read(rc, bdf, 0x34, 1, &ptr));
	unsigned seen[2] = { 0, 0 };
	for (unsigned n = 0; ptr && n < 48; n++) {
		if (ptr < 0x40 || ptr > 0xfc || (ptr & 3)) return EIO;
		unsigned bit = (ptr - 0x40) / 4;
		if (seen[bit / 32] & (1u << (bit % 32))) return EIO;
		seen[bit / 32] |= 1u << (bit % 32);
		CHECK(hakios_pcie_cfg_read(rc, bdf, ptr, 4, &header));
		if (header == ~0u) return EIO;
		if ((header & 255) == 0x10) {
			if (aligned == ptr + 8) *mask = 0x003f0000u; /* DEVSTA */
			if (aligned == ptr + 0x10) *mask = 0xc0000000u; /* LNKSTA */
			if (aligned == ptr + 0x18) *mask = 0x011f0000u; /* SLOTSTA */
			if (aligned == ptr + 0x20) *mask = 0x00010000u; /* ROOTSTA */
		}
		ptr = (header >> 8) & 255;
	}
	return ptr ? EIO : 0;
}
int
hakios_pcie_cfg_write(unsigned rc, unsigned bdf, unsigned off, unsigned size, unsigned value)
{
	vm_offset_t a;
	unsigned v, shift = 8 * (off & 3);
	CHECK(config_address(rc, bdf, off, size, &a)); CHECK(rd(a, &v));
	if (size != 4) {
		unsigned mask = ((1u << (8 * size)) - 1) << shift;
		v = (v & ~mask) | ((value << shift) & mask);
		unsigned w1c;
		CHECK(config_w1c(rc, bdf, off & ~3u, &w1c));
		v &= ~(w1c & ~mask);
	} else v = value;
	return wr(a, v);
}
static int
enumerate(struct hakios_pcie_link *link)
{
	unsigned id, original, mask, hi = 0, command;
	CHECK(hakios_pcie_cfg_read(0, 0x100, 0, 4, &id));
	if (id != 0x110317cb) return ENODEV;
	CHECK(hakios_pcie_cfg_read(0, 0x100, 4, 2, &command));
	CHECK(hakios_pcie_cfg_write(0, 0x100, 4, 2, command & ~6u));
	CHECK(hakios_pcie_cfg_read(0, 0x100, 0x10, 4, &original));
	if (original & 1) return ENODEV;
	if ((original & 6) != 0 && (original & 6) != 4) return ENODEV;
	if ((original & 6) == 4) CHECK(hakios_pcie_cfg_read(0, 0x100, 0x14, 4, &hi));
	/* BAR size probe while decode is off. Restore even if readback aborts. */
	int error = hakios_pcie_cfg_write(0, 0x100, 0x10, 4, ~0u);
	if (!error) error = hakios_pcie_cfg_read(0, 0x100, 0x10, 4, &mask);
	int restore = hakios_pcie_cfg_write(0, 0x100, 0x10, 4, original);
	if (error || restore) return error ? error : restore;
	if ((~(mask & ~15u) + 1) != BAR_SIZE) return ENODEV;
	CHECK(hakios_pcie_cfg_write(0, 0x100, 0x10, 4, BAR_PA | (original & 15)));
	if ((original & 6) == 4) CHECK(hakios_pcie_cfg_write(0, 0x100, 0x14, 4, 0));
	(void)hi; /* no restore after down: endpoint is held in reset */
	CHECK(hakios_pcie_cfg_write(0, 0x100, 4, 2, command | 6));
	CHECK(hakios_pcie_cfg_read(0, 0x100, 0x10, 4, &mask));
	CHECK(hakios_pcie_cfg_read(0, 0x100, 4, 2, &command));
	if ((mask & ~15u) != BAR_PA || (command & 6) != 6) return EIO;
	if (!bar) bar = ml_io_map(BAR_PA, BAR_SIZE);
	if (!bar) return ENOMEM;
	endpoint = true;
	link->vendor = id & 0xffff; link->device = id >> 16;
	link->bar0_pa = BAR_PA; link->bar0_va = (void *)bar; link->bar0_size = BAR_SIZE;
	return 0;
}
/* Parse exact strings: PE_parse_boot_argn would accept numeric prefixes or
 * turn malformed values into zero. Reject unknown values before any mapping. */
static int
numeric_word(const char *name, unsigned *value, unsigned low, unsigned high)
{
	char text[32];
	if (!PE_parse_boot_arg_str(name, text, sizeof(text))) return 0;
	if (text[0] < '0' || text[0] > '9' || text[1]) return EINVAL;
	unsigned n = (unsigned)(text[0] - '0');
	if (n < low || n > high) return EINVAL;
	*value = n;
	return 0;
}
static void
failure_dump(void)
{
	/* Only host config/PARF, never chip space or read-clear reset cause. */
	static const unsigned offsets[] = { 0, 4, 0x1c, 0x80, 0x88, 0xf28 };
	for (unsigned i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
		unsigned v;
		if (dbi && !rd(dbi + offsets[i], &v)) say("pcie: failure DBI[%#x]=%#x\n", offsets[i], v);
	}
	unsigned v;
	if (parf && !rd(parf + 0x1b0, &v)) say("pcie: failure LTSSM=%#x\n", v);
	if (linked && !rd(cfg + 4, &v)) say("pcie: failure endpoint command/status=%#x\n", v);
}
void
hakios_pcie_down(unsigned rc)
{
	if (rc || !started) return;
	/* No mutex, mapping, allocation or thread waits; safe when panic owns a lock.
	 * GPIOs first: even if the endpoint no longer answers, stop its bus traffic. */
	int errors = 0;
	if (tlmm) { errors += !!gpio(94, false); errors += !!gpio(80, false); errors += !!gpio(204, false); }
	linked = endpoint = false;
	while (undo_count) {
		unsigned n = --undo_count;
		errors += !!(undo[n].mask ? vote_clear(undo[n].address, undo[n].mask) :
		    wr(undo[n].address, undo[n].old));
	}
	while (rail_count) errors += !!vote(--rail_count, false);
	if (!tcs_poisoned) tcs_owned = false;
	/* A timed-out TCS stays quarantined for the boot. Never clear or retrigger it. */
	started = false;
	if (errors) say("pcie: shutdown incomplete: %d refused/timed-out operations; reset pins attempted\n", errors);
}
int
hakios_pcie_up(unsigned rc, struct hakios_pcie_link *link)
{
	unsigned stage = 1;
	int error;
	if (!link) return EINVAL;
	bzero(link, sizeof(*link));
	if (rc) return ENODEV;
	if (started) return EBUSY;
	generation = 2;
	if ((error = numeric_word("pcie_gen", &generation, 1, 2))) goto failed;
	if (generation != 1 && generation != 2) { error = EINVAL; goto failed; }
	if (!hakios_mmio_try_read32 || !hakios_mmio_try_write32) { error = ENOTSUP; goto failed; }
	/* RC0 is explicitly selected by the caller; no redwood auto-detection. */
	if (tcs_poisoned) { error = EBUSY; goto failed; }
	if ((error = map_blocks())) goto failed;
	started = true;
	if ((error = power())) goto failed;
	hakios_wifi_stage(1, "rails", true, "pcie: rails (6 incl ant switch), BT_EN low, wlan_en high");
	stage = 2;
	if ((error = clocks_phy()) || (error = train(link))) goto failed;
	hakios_wifi_stage(2, "link", true, "pcie: link up gen %u x%u", link->gen, link->width);
	stage = 3;
	if ((error = enumerate(link))) goto failed;
	hakios_wifi_stage(3, "endpoint", true, "pcie: 17cb:1103 BAR0 %#llx + %#lx decode + master", link->bar0_pa, link->bar0_size);
	return 0;
failed:
	hakios_wifi_stage(stage, stage == 1 ? "rails" : stage == 2 ? "link" : "endpoint", false, "pcie errno %d", error);
	if (started && stage > 1) failure_dump();
	hakios_pcie_down(rc);
	bzero(link, sizeof(*link));
	return error;
}
static void
standalone_thread(void *arg, wait_result_t result)
{
	struct hakios_pcie_link link;
	(void)arg; (void)result;
	int error = hakios_pcie_up(0, &link);
	hakios_pcie_down(0);
	say("pcie: standalone up/down complete: errno %d\n", error);
	thread_terminate(current_thread());
}
static void
standalone_init(void)
{
	unsigned on = 0, wifi = 0;
	thread_t thread;
	if (numeric_word("pcie", &on, 0, 1)) { say("pcie: invalid pcie value\n"); return; }
	if (!on) return;
	if (numeric_word("wifi", &wifi, 0, 1) || wifi) { say("pcie: pcie=1 requires wifi=0\n"); return; }
	if (kernel_thread_start(standalone_thread, NULL, &thread) != KERN_SUCCESS) {
		say("pcie: standalone thread unavailable\n"); return;
	}
	thread_deallocate(thread);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, standalone_init);
