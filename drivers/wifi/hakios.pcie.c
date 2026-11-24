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
#include <mach/vm_param.h>
#include <os/atomic_private.h>
#include <vm/memory_types.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/hakios.pcie.h>
#include <sys/hakios.wifi.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>
#include <sys/sysctl.h>

/* From the kernel's own map, as redwood's Wi-Fi does: the io_submap of plain I/O mappings holds 8 MB in all and is
 * full by the time the chain runs; a mapping that does not fit there panics (KMA_NOFAIL), which reset the phone in
 * round 1 (2026-11-12). */
extern vm_offset_t ml_io_map_unmappable(vm_offset_t, vm_size_t, unsigned int);
#define MAP_IO(pa, size) ml_io_map_unmappable((pa), (size), VM_WIMG_IO)
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
#define TCS_OWN_SLOT 2u
#define TCS_SHARED_SLOT 0u
static unsigned tcs_slot = TCS_OWN_SLOT;
#define TCS_SLOT tcs_slot
#define TCS_BIT (1u << tcs_slot)
#define TCS(reg) (TCS_BASE + tcs_slot * TCS_STRIDE + (reg))
#define GDSCR 0x7b004
#define PIPE_MUX 0x7b060
#define RCHNG 0x7b048
#define AUX 0x7b064
#define VOTE0 0x62000
#define VOTE2 0x62008
#define VOTE0_BITS ((1u << 11) | (1u << 12) | (1u << 19) | (1u << 22))
#define PIN(n) ((n) * 0x1000u)
#define CHECK(call) do { int e_ = (call); if (e_) return e_; } while (0)

static vm_offset_t gcc_vote, gcc_pcie, gcc_clkref, tlmm, parf, phy, dbi, cfg, db, rsc, bar;
/*
 * GCC is mapped only where this file touches it, as the other drivers do (speaker/goodix: the first 0x60000,
 * touch: single pages): mapping the whole 1 MiB block reset the phone at once in round 1 (2026-11-12, the
 * last line "pcie: step 0a map GCC"). Three windows: the APCS vote words (0x62000/0x62008), PCIe0's block and
 * its PHY reset (0x7b000-0x7c020), the CLKREF (0x9c004). An offset outside them is a bug: it gives 0, which
 * the guarded accessors refuse (EINVAL), and says so.
 */
#define GCC_VOTE_WIN 0x60000u
#define GCC_PCIE_WIN 0x78000u
#define GCC_CLKREF_WIN 0x9c000u
static void say(const char *format, ...) __printflike(1, 2);
static vm_offset_t
G(unsigned off)
{
	if (off >= GCC_VOTE_WIN && off < GCC_VOTE_WIN + 0x4000u && gcc_vote) return gcc_vote + (off - GCC_VOTE_WIN);
	if (off >= GCC_PCIE_WIN && off < GCC_PCIE_WIN + 0x8000u && gcc_pcie) return gcc_pcie + (off - GCC_PCIE_WIN);
	if (off >= GCC_CLKREF_WIN && off < GCC_CLKREF_WIN + 0x1000u && gcc_clkref) return gcc_clkref + (off - GCC_CLKREF_WIN);
	say("pcie: GCC offset %#x is outside the mapped windows\n", off);
	return 0;
}
static unsigned generation = 2, ncpt, rail_count;
static bool started, linked, endpoint, tcs_owned, tcs_poisoned;
static unsigned down_errors, up_count, down_count, rail_mask;
/* Link sleep (hakios_pcie_sleep/wake): the journal entries from ctl_mark on are the controller's (clocks, GDSC,
 * PHY, DBI); those before are the rails' and pins'. Asleep: the controller is off, the radio powered, in M3. */
static unsigned ctl_mark;
static bool drv_mode;
static int drv_wake(void);
static volatile bool asleep;
static unsigned sleeps, wakes, sleep_errors, wake_errors, l23_missed, refused_bar;
static unsigned long long asleep_since, asleep_ns;
static char pm_description[768] = "no completed enumeration";
static struct pm_port {
	unsigned bdf, pc, ss, lc, ctl, sta, dc2, dctl2, sc, s1, s2, dc;
} pm_ports[2];
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
/*
 * pcie_pause=MS: a line before each group of accesses of stage 1, then that long a wait, so that the last line
 * reaches the USB log before a reset the group may cause (default 0: no lines, no waits).
 */
static unsigned step_pause_ms;
static void
step(const char *format, ...) __printflike(1, 2);
static void
step(const char *format, ...)
{
	char line[200];
	va_list ap;
	if (!step_pause_ms) return;
	va_start(ap, format);
	vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);
	say("pcie: step %s\n", line);
	delay(step_pause_ms * 1000);
}
static int
map_blocks(void)
{
	/* Map only on up. Cached mappings survive down; borrowed BAR access does not. */
	step("0a map GCC: votes, PCIe0, CLKREF windows");
	if (!gcc_vote) gcc_vote = MAP_IO(GCC_PA + GCC_VOTE_WIN, 0x4000);
	if (!gcc_pcie) gcc_pcie = MAP_IO(GCC_PA + GCC_PCIE_WIN, 0x8000);
	if (!gcc_clkref) gcc_clkref = MAP_IO(GCC_PA + GCC_CLKREF_WIN, 0x1000);
	step("0b map TLMM");
	if (!tlmm) tlmm = MAP_IO(TLMM_PA, 0x100000);
	step("0c map PARF, PHY, DBI, config");
	if (!parf) parf = MAP_IO(PARF_PA, 0x3000);
	if (!phy) phy = MAP_IO(PHY_PA, 0x2000);
	if (!dbi) dbi = MAP_IO(DBI_PA, 0x2000);
	if (!cfg) cfg = MAP_IO(CFG_PA, 0x1000);
	step("0d map RSC");
	if (!rsc) rsc = MAP_IO(RSC_PA, 0x10000);
	step("0e map cmd-db");
	if (!db) db = ml_io_map_unmappable(DB_PA, DB_SIZE, VM_WIMG_WCOMB);
	step("0f all mapped");
	return gcc_vote && gcc_pcie && gcc_clkref && tlmm && parf && phy && dbi && cfg && rsc && db ? 0 : ENOMEM;
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
	unsigned base = TCS_BASE + slot * TCS_STRIDE, idle, control, enabled, waited;
	CHECK(rd(rsc + base + 0x18, &idle));
	CHECK(rd(rsc + base + 0x14, &control));
	CHECK(rd(rsc + base + 0x1c, &enabled));
	CHECK(rd(rsc + base + 0x10, &waited));
	/* ABL leaves completed ACTIVE requests triggered (idle=1). Unwaited
	 * commands have status 0x101, not the waited response's 0x10101. The
	 * no-delay boot reaches this before touch/audio have recycled TCS0.
	 * Keep refusing armed sleep/wake requests and incomplete responses. */
	if (!idle || idle == ~0u || ((control & (1u << 24)) && !(control & (1u << 16))) || (enabled >> ncpt)) return EBUSY;
	for (unsigned c = 0; c < ncpt; c++) if (enabled & (1u << c)) {
		unsigned status;
		CHECK(rd(rsc + base + 0x3c + 20 * c, &status));
		unsigned complete = waited & (1u << c) ? 0x10100u : 0x100u;
		if (status == ~0u || (status & complete) != complete) return EBUSY;
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
static int vote_slot(unsigned index, bool on);
/*
 * TCS 0 is the one the audio driver (hakios.adsp.c) and UEFI use, under hakios_rsc_lock: taken here with a bounded
 * try (100 ms), never by blocking, because hakios_pcie_down() runs on the panic path too; not had, the vote is not made.
 */
static int
vote(unsigned index, bool on)
{
	int error;
	if (tcs_slot != TCS_SHARED_SLOT) return vote_slot(index, on);
	for (unsigned tries = 0; !lck_mtx_try_lock(&hakios_rsc_lock); tries++) {
		if (tries == 1000) {
			say("pcie: TCS 0 sequencer lock busy for 100 ms\n");
			return EBUSY;
		}
		delay(100);
	}
	error = vote_slot(index, on);
	lck_mtx_unlock(&hakios_rsc_lock);
	return error;
}
/* What the sequencer shows for the slot, on a vote that did not finish (all reads). */
static void
tcs_dump(const char *why)
{
	unsigned v[9] = { 0 };
	(void)rd(rsc + TCS_BASE + 0, &v[0]); (void)rd(rsc + TCS_BASE + 4, &v[1]);
	(void)rd(rsc + TCS(0x10), &v[2]); (void)rd(rsc + TCS(0x14), &v[3]); (void)rd(rsc + TCS(0x18), &v[4]);
	(void)rd(rsc + TCS(0x1c), &v[5]); (void)rd(rsc + TCS(0x30), &v[6]); (void)rd(rsc + TCS(0x3c), &v[7]);
	(void)rd(rsc + TCS(0x3c + 20), &v[8]);
	say("pcie: TCS %u %s: irq enable %#x status %#x; wait %#x control %#x idle %#x enabled %#x; cmd0 msgid %#x status %#x, cmd1 status %#x\n",
	    tcs_slot, why, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
}
static int
vote_slot(unsigned index, bool on)
{
	unsigned address = rails[index].address, count = on ? 2 : 1;
	unsigned offsets[2] = { on ? 0u : 4u, 4u }, values[2] = { on ? rails[index].mv : 0u, 1u };
	unsigned irq, idle, status, mask = (1u << count) - 1;
	if (!tcs_owned || tcs_poisoned) return EBUSY;
	int ready = tcs_idle(TCS_SLOT);
	if (ready) { tcs_dump("not reusable"); return ready; }
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
	if (!done) { tcs_dump("timed out"); return ETIMEDOUT; }
	CHECK(tcs_sync(TCS(0x14), 0));
	CHECK(tcs_sync(TCS(0x1c), 0));
	CHECK(tcs_sync(TCS(0x10), 0));
	CHECK(wr(rsc + TCS_BASE + 8, TCS_BIT));
	tcs_poisoned = false;
	if (on) rail_mask |= 1u << index;
	else rail_mask &= ~(1u << index);
	return 0;
}
static int
power(void)
{
	unsigned v;
	step("1 gpio 94 (PERST) low");
	CHECK(gpio(94, false));
	step("2 gpio 80 (WLAN_EN) low");
	CHECK(gpio(80, false));
	step("3 gpio 81 (BT_EN) low");
	CHECK(gpio(81, false)); /* Bluetooth bring-up is separate; assert off. */
	step("4 gpio 95 (CLKREQ) function");
	CHECK(rd(tlmm + PIN(95), &v));
	CHECK(save_write(tlmm + PIN(95), 1u << 2 | 3u | ((v & (1u << 11)) ? 1u << 12 : 0)));
	step("5 cmd-db: the six rails");
	for (unsigned i = 0; i < 6; i++) CHECK(resource(rails[i].name, &rails[i].address));
	for (unsigned i = 0; i < 6; i++) step("5 %s at %#x", rails[i].name, rails[i].address);
	step("6 claim TCS %u", TCS_SLOT);
	lck_mtx_lock(&hakios_rsc_lock);
	int error = claim_tcs();
	lck_mtx_unlock(&hakios_rsc_lock);
	if (error) return error;
	for (unsigned i = 0; i < 6; i++) {
		step("7 vote %s %u mV on (TCS %u, %u commands per slot)", rails[i].name, rails[i].mv, TCS_SLOT, ncpt);
		rail_count = i + 1; /* uncertain enable is still ours to remove */
		int verror = vote(i, true);
		if (verror == ETIMEDOUT && tcs_slot == TCS_OWN_SLOT) {
			say("pcie: TCS %u did not finish the vote: TCS %u from here on, under the sequencer lock (as hakios.adsp.c)\n",
			    TCS_OWN_SLOT, TCS_SHARED_SLOT);
			tcs_slot = TCS_SHARED_SLOT;
			tcs_poisoned = false;
			verror = vote(i, true);
		}
		CHECK(verror);
	}
	step("8 rails voted; gpio 204 (XO_CLK) low");
	delay(100000);
	CHECK(gpio(204, false)); delay(10000);
	step("9 XO_CLK pulse, gpio 80 (WLAN_EN) high");
	CHECK(gpio(204, true)); delay(200);
	CHECK(gpio(80, true)); delay(6000);
	CHECK(gpio(204, false)); delay(50000);
	step("10 power sequence done");
	return 0;
}
static int
clocks_phy(void)
{
	step("11 GCC PCIe0 GDSC on");
	CHECK(bits(G(GDSCR), 1, 0));
	CHECK(wait_bits(G(GDSCR), 1u << 31, 1u << 31, 10000));
	step("12 GCC pipe mux, RCHNG, AUX rate");
	CHECK(save_write(G(PIPE_MUX), 2));
	CHECK(save_write(G(RCHNG + 4), 1u << 8 | 11));
	CHECK(bits(G(RCHNG), 0, 1)); CHECK(wait_bits(G(RCHNG), 1, 0, 10000));
	CHECK(save_write(G(AUX + 4), 1));
	CHECK(bits(G(AUX), 0, 1)); CHECK(wait_bits(G(AUX), 1, 0, 10000));
	step("13 GCC clock votes 0x62000/0x62008");
	CHECK(vote_bits(G(VOTE0), VOTE0_BITS));
	CHECK(vote_bits(G(VOTE2), 0x2f));
	step("14 CLKREF, BCR reset pulse");
	CHECK(bits(G(0x9c004), 0, 1)); delay(100);
	CHECK(wait_bits(G(0x7b030), 1u << 31, 0, 10000));
	CHECK(wait_bits(G(0x7b034), 1u << 31, 0, 10000));
	CHECK(save_write(G(0x7b000), 1)); delay(1000);
	CHECK(save_write(G(0x7b000), 0)); delay(1000);
	step("15 PARF");
	CHECK(save_write(parf + 0x1000, 4));
	CHECK(bits(parf + 0x40, 1, 0));
	/* Consistent vendor decode recipe, not Linux addresses mixed with vendor size. */
	CHECK(save_write(parf + 0x350, 0));
	CHECK(save_write(parf + 0x358, 0x4000000));
	CHECK(bits(parf, 1u << 29, 0));
	CHECK(bits(parf + 0x174, 0, 1u << 4));
	CHECK(bits(parf + 0x20, 1u << 5, 0));
	CHECK(bits(parf + 0x1a8, 0, 1u << 31));
	step("16 PHY reset and init table");
	CHECK(save_write(G(0x7c01c), 1));
	CHECK(save_write(G(0x7c020), 1)); delay(300);
	CHECK(save_write(G(0x7c01c), 0));
	for (unsigned i = 0; i < sizeof(phy_sequence) / sizeof(phy_sequence[0]); i++) {
		/* The final DT SW_RESET/START writes follow NOCSR reset release below. */
		if (phy_sequence[i][0] == 0x200) CHECK(save_write(G(0x7c020), 0));
		CHECK(save_write(phy + phy_sequence[i][0], phy_sequence[i][1]));
		if (phy_sequence[i][2]) delay(phy_sequence[i][2]);
	}
	delay(1200);
	CHECK(save_write(G(PIPE_MUX), 0)); CHECK(vote_bits(G(VOTE2), 1u << 4));
	CHECK(wait_bits(phy + 0x214, 1u << 6, 0, 20000));
	step("17 DBI first read");
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
	step("18 DBI root port setup, iATU");
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
	step("19 PERST high, LTSSM");
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

/*
 * The controller's BDF-to-SID table (Android: pci-msm.c msm_pcie_config_sid, on every link enable; iOSPlus10 0325):
 * the core reset of clocks_phy() leaves it in its reset state, BYPASS on, and the chip's DMA then carries no SID our
 * context matches -- round 1 (2026-11-12) reset the phone at the chip's first read of host memory, the BHI image.
 * BYPASS off (journaled), the 256 entries cleared, BDF 0x0100 -> SID offset 1 (0x1c00 + 1 = 0x1c01, iommu-map) at
 * crc8(BDF big endian, poly 0x07). The entries are not journaled: down leaves BYPASS on again, which ignores them.
 */
#define PARF_BDF_TO_SID_TABLE 0x2000u
#define PARF_BDF_TO_SID_CFG 0x2c00u
static int
bdf_to_sid(void)
{
	const unsigned bdf = 0x0100u;
	unsigned char crc = 0, bytes[2] = { bdf >> 8, bdf & 0xff };
	unsigned value = bdf << 16 | 1u << 8, got, cfg;
	for (unsigned b = 0; b < 2; b++) {
		crc ^= bytes[b];
		for (int i = 0; i < 8; i++) crc = (unsigned char)((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
	}
	CHECK(rd(parf + PARF_BDF_TO_SID_CFG, &cfg));
	CHECK(save_write(parf + PARF_BDF_TO_SID_CFG, cfg & ~1u));
	for (unsigned i = 0; i < 256; i++) CHECK(wr(parf + PARF_BDF_TO_SID_TABLE + 4 * i, 0));
	CHECK(wr(parf + PARF_BDF_TO_SID_TABLE + 4 * crc, value));
	__asm__ volatile ("dsb sy" ::: "memory");
	CHECK(rd(parf + PARF_BDF_TO_SID_TABLE + 4 * crc, &got));
	CHECK(rd(parf + PARF_BDF_TO_SID_CFG, &cfg));
	say("pcie: BDF-to-SID: entry %#x = %#x, CFG %#x%s\n", (unsigned)crc, got, cfg,
	    got == value && !(cfg & 1u) ? " (BDF 0x100 -> SID 0x1c01)" : " - DID NOT TAKE: the chip may carry SID 0x1c00");
	return 0;
}

/* Config accesses are serialized by their caller, same as up/down. Only bus0
 * root port and bus1/device0/function0 exist. Endpoint forbidden until link up. */
static int
config_address(unsigned rc, unsigned bdf, unsigned off, unsigned size, vm_offset_t *a)
{
	if (rc) return ENODEV;
	if ((size != 1 && size != 2 && size != 4) || (off & (size - 1)) || off > 4096 - size) return EINVAL;
	if (!started || !dbi || asleep) return ENXIO;
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
#include "hakios.pcie-pm.h"

/* Read-only capability snapshot at enumeration, while this caller owns the link.
 * The sysctl serves this cache; it never touches a powered-down config window. */
static int pm_describe(unsigned bdf, char *text, size_t size)
{
	unsigned ptr, header, pc = 0, ss = 0, ltr = 0, seen[32] = {0};
	CHECK(hakios_pcie_cfg_read(0, bdf, 0x34, 1, &ptr));
	for (unsigned n = 0; ptr && n < 48; n++) {
		if (ptr < 0x40 || ptr > 0xfc || (ptr & 3)) return EIO;
		unsigned bit = ptr / 4;
		if (seen[bit / 32] & (1u << (bit % 32))) return EIO;
		seen[bit / 32] |= 1u << (bit % 32);
		CHECK(hakios_pcie_cfg_read(0, bdf, ptr, 4, &header));
		if (header == ~0u) return EIO;
		if ((header & 255) == 0x10) pc = ptr;
		ptr = (header >> 8) & 255;
	}
	if (ptr || !pc) return EIO;
	bzero(seen, sizeof(seen)); ptr = 0x100;
	for (unsigned n = 0; ptr && n < 256; n++) {
		if (ptr < 0x100 || ptr > 0xffc || (ptr & 3)) return EIO;
		unsigned bit = ptr / 4;
		if (seen[bit / 32] & (1u << (bit % 32))) return EIO;
		seen[bit / 32] |= 1u << (bit % 32);
		CHECK(hakios_pcie_cfg_read(0, bdf, ptr, 4, &header));
		if (!header || header == ~0u) { ptr = 0; break; }
		if ((header & 0xffff) == 0x1e) ss = ptr;
		if ((header & 0xffff) == 0x18) ltr = ptr;
		ptr = (header >> 20) & 0xfff;
	}
	if (ptr || pc > 0xd4 || (ss && ss > 0xff0)) return EIO;
	unsigned lc, ctl, sta, dc, dc2, dctl2, sc = 0, s1 = 0, s2 = 0;
	CHECK(hakios_pcie_cfg_read(0, bdf, pc + 4, 4, &dc));
	CHECK(hakios_pcie_cfg_read(0, bdf, pc + 0xc, 4, &lc));
	CHECK(hakios_pcie_cfg_read(0, bdf, pc + 0x10, 2, &ctl));
	CHECK(hakios_pcie_cfg_read(0, bdf, pc + 0x12, 2, &sta));
	CHECK(hakios_pcie_cfg_read(0, bdf, pc + 0x24, 4, &dc2));
	CHECK(hakios_pcie_cfg_read(0, bdf, pc + 0x28, 2, &dctl2));
	if (ss) {
		CHECK(hakios_pcie_cfg_read(0, bdf, ss + 4, 4, &sc));
		CHECK(hakios_pcie_cfg_read(0, bdf, ss + 8, 4, &s1));
		CHECK(hakios_pcie_cfg_read(0, bdf, ss + 12, 4, &s2));
	}
	snprintf(text, size, "%s pcie %#x lnkcap %#x lnkctl %#x lnksta %#x devcap %#x devcap2 %#x devctl2 %#x l1ss %#x cap %#x ctl1 %#x ctl2 %#x ltr %#x",
	    bdf ? "endpoint" : "root", pc, lc, ctl, sta, dc, dc2, dctl2, ss, sc, s1, s2, ltr);
	pm_ports[bdf ? 1 : 0] = (struct pm_port){bdf, pc, ss, lc, ctl, sta, dc2, dctl2, sc, s1, s2, dc};
	return 0;
}
static void pm_snapshot(void)
{
	char root[320] = {0}, ep[320] = {0};
	bzero(pm_ports, sizeof(pm_ports));
	int a = pm_describe(0, root, sizeof(root));
	int b = pm_describe(HAKIOS_PCIE_WIFI_BDF, ep, sizeof(ep));
	pm_valid = !a && !b;
	snprintf(pm_description, sizeof(pm_description), "last enumeration (cached), errors %d/%d\n%s\n%s", a, b, root, ep);
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
	if (!bar) bar = MAP_IO(BAR_PA, BAR_SIZE);
	if (!bar) return ENOMEM;
	endpoint = true;
	link->vendor = id & 0xffff; link->device = id >> 16;
	link->bar0_pa = BAR_PA; link->bar0_va = (void *)bar; link->bar0_size = BAR_SIZE;
	pm_snapshot();
	pm_error = pm_prepare(pm_requested);
	if (pm_error) {
		int restore = pm_restore();
		say("pcie: link PM prepare failed %d; restoration %d, ASPM left off\n", pm_error, restore);
		if (restore) return restore;
	}
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
	if (asleep && drv_mode) {
		(void)drv_wake();       /* (the DSP gives the link back first; then the ordinary shutdown) */
	}
	if (asleep) {
		/* the controller is already off (its journal undone), the endpoint in reset: only pins and rails remain */
		pm_owned = false;
		asleep = false;
	}
	errors += !!pm_restore();
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
	down_errors += errors;
	down_count++;
	if (!errors) (void)hakios_pcie_bandwidth(false); /* uncertain shutdown: keep its bus floor */
	if (errors) say("pcie: shutdown incomplete: %d refused/timed-out operations; reset pins attempted\n", errors);
}
int hakios_pcie_down_status(void)
{
	return started ? EBUSY : down_errors || tcs_poisoned || rail_mask ? EIO : 0;
}

/*
 * ---- link sleep with the radio powered (the vendor's non-DRV suspend: pci-msm.c msm_pcie_pm_suspend/resume, cnss2's
 * cnss_pci_suspend_bus: MHI in M3 first, then the endpoint's config saved, D3hot, PME_Turn_Off, L23_Ready, PERST, and the
 * controller off; the radio keeps its rails, WLAN_EN and its association, and asks for the host with WAKE#, GPIO 96).
 * Off, nothing of APSS's holds CX for PCIe: no GDSC, no clock, no SN0 floor.
 */
#define EP_BDF HAKIOS_PCIE_WIFI_BDF
static unsigned ep_cfg[64], ep_msi, ep_pmc, ep_pcie;
static bool ep_saved;
static int
ep_cap(unsigned id, unsigned *at)
{
	unsigned ptr, header;
	*at = 0;
	CHECK(hakios_pcie_cfg_read(0, EP_BDF, 0x34, 1, &ptr));
	for (unsigned n = 0; ptr && n < 48; n++) {
		if (ptr < 0x40 || ptr > 0xfc || (ptr & 3)) return EIO;
		CHECK(hakios_pcie_cfg_read(0, EP_BDF, ptr, 4, &header));
		if (header == ~0u) return EIO;
		if ((header & 255) == id) { *at = ptr; return 0; }
		ptr = (header >> 8) & 255;
	}
	return 0;
}
static int
ep_save(void)
{
	for (unsigned i = 0; i < 64; i++) CHECK(hakios_pcie_cfg_read(0, EP_BDF, 4 * i, 4, &ep_cfg[i]));
	if (ep_cfg[0] == ~0u || ep_cfg[0] == 0) return EIO;
	CHECK(ep_cap(0x05, &ep_msi));
	CHECK(ep_cap(0x01, &ep_pmc));
	CHECK(ep_cap(0x10, &ep_pcie));
	ep_saved = true;
	return 0;
}
/* Linux pci_restore_state's order: D0 first, PCIe control registers, the header (the command word last), MSI
 * (its control word last). Status words are not written back (W1C). */
static int
ep_restore(void)
{
	unsigned id;
	if (!ep_saved) return EINVAL;
	CHECK(hakios_pcie_cfg_read(0, EP_BDF, 0, 4, &id));
	if (id != ep_cfg[0]) return ENODEV;
	if (ep_pmc) {
		CHECK(hakios_pcie_cfg_write(0, EP_BDF, ep_pmc + 4, 2, ep_cfg[(ep_pmc + 4) / 4] & ~3u & 0xffff));
		delay(10000); /* D3hot -> D0: 10 ms (PCI PM 1.2) */
	}
	if (ep_pcie) {
		CHECK(hakios_pcie_cfg_write(0, EP_BDF, ep_pcie + 0x08, 2, ep_cfg[(ep_pcie + 0x08) / 4] & 0xffff));       /* DEVCTL */
		CHECK(hakios_pcie_cfg_write(0, EP_BDF, ep_pcie + 0x10, 2, ep_cfg[(ep_pcie + 0x10) / 4] & 0xffff & ~3u)); /* LNKCTL, ASPM off */
		CHECK(hakios_pcie_cfg_write(0, EP_BDF, ep_pcie + 0x28, 2, ep_cfg[(ep_pcie + 0x28) / 4] & 0xffff));       /* DEVCTL2 */
	}
	for (int i = 15; i >= 2; i--) CHECK(hakios_pcie_cfg_write(0, EP_BDF, 4 * (unsigned)i, 4, ep_cfg[i]));
	if (ep_msi) {
		unsigned control = ep_cfg[ep_msi / 4] >> 16, wide = control & (1u << 7);
		CHECK(hakios_pcie_cfg_write(0, EP_BDF, ep_msi + 4, 4, ep_cfg[(ep_msi + 4) / 4]));
		if (wide) {
			CHECK(hakios_pcie_cfg_write(0, EP_BDF, ep_msi + 8, 4, ep_cfg[(ep_msi + 8) / 4]));
			CHECK(hakios_pcie_cfg_write(0, EP_BDF, ep_msi + 12, 2, ep_cfg[(ep_msi + 12) / 4] & 0xffff));
		} else {
			CHECK(hakios_pcie_cfg_write(0, EP_BDF, ep_msi + 8, 2, ep_cfg[(ep_msi + 8) / 4] & 0xffff));
		}
		if (control & (1u << 8)) { /* per-vector masking */
			unsigned mask_at = ep_msi + (wide ? 16 : 12);
			CHECK(hakios_pcie_cfg_write(0, EP_BDF, mask_at, 4, ep_cfg[mask_at / 4]));
		}
		CHECK(hakios_pcie_cfg_write(0, EP_BDF, ep_msi + 2, 2, control));
	}
	CHECK(hakios_pcie_cfg_write(0, EP_BDF, 4, 2, ep_cfg[1] & 0xffff));
	return 0;
}

static unsigned long long
now_ns(void)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns;
}

/*
 * DRV sleep (pci-msm.c msm_pcie_drv_suspend/resume): the link stays trained and in L1.2; the audio DSP takes RC0 over
 * (ENABLE), APSS lets go of its clocks, its GDSC vote and its bus floor, and the DSP may power the controller down
 * whenever the link has been in L1ss for 5 ms (ENABLE_L1SS_SLEEP; qcom,drv-l1ss-timeout-us = 5000). The radio still
 * reaches the host's memory through it (its paging: run47's link-off WoW woke for WOW_REASON_PAGE_FAULT at once).
 */
extern int hakios_adsp_pcie_drv(uint32_t command, uint32_t argument) __attribute__((weak));
#define DRV_ENABLE 0xc0000000u
#define DRV_DISABLE 0xc0000001u
#define DRV_ENABLE_L1SS_SLEEP 0xc0000005u
#define DRV_DISABLE_L1SS_SLEEP 0xc0000006u
static unsigned sleep_mode = 1;         /* 1 DRV, 2 the link off (PERST), 3 kept by APSS in L1ss (no hand-off) */
/* 0: DRV sleep without ENABLE_L1SS_SLEEP (the DSP holds the link and never powers the controller down) */
static unsigned drv_l1ss_sleep = 1;
static bool drv_mode, drv_l1ss_requested;
static bool drv_cx_ready;

bool
hakios_pcie_drv_cx_ready(void)
{
	return os_atomic_load(&drv_cx_ready, acquire);
}
static unsigned drv_errors, drv_last_error, drv_pm_mode;
static unsigned drv_gdsc_off;
SYSCTL_DECL(_kern_hakios);
SYSCTL_UINT(_kern_hakios, OID_AUTO, pcie_drv_gdsc, CTLFLAG_RW | CTLFLAG_LOCKED, &drv_gdsc_off, 0,
    "1: DRV sleep also collapses the PCIe0 GDSC (SW_COLLAPSE); 0 (Linux: PWRSTS_RET_ON) leaves it on");
SYSCTL_UINT(_kern_hakios, OID_AUTO, pcie_sleep_mode, CTLFLAG_RW | CTLFLAG_LOCKED, &sleep_mode, 0,
    "PCIe0 link sleep: 1 handed to the audio DSP (DRV, link in L1.2), 2 link off (PERST; the radio cannot page), "
    "3 kept by APSS with its clocks and link PM (L1ss): nothing handed over");
SYSCTL_UINT(_kern_hakios, OID_AUTO, pcie_drv_l1ss_sleep, CTLFLAG_RW | CTLFLAG_LOCKED, &drv_l1ss_sleep, 0,
    "1: DRV sleep lets the DSP power the controller down after 5 ms in L1ss (ENABLE_L1SS_SLEEP); 0: it never does");
/*
 * The radio in D3hot while the link sleeps (PME enabled), D0 and its config back on wake: what upstream's PCI core does
 * at system suspend and cnss2's cnss_pci_suspend does (pci_set_power_state(D3hot), "Failed to set D3Hot"). The firmware
 * picks how it asks for the host by it (amss20.bin: "D3 Hot State, PERST HIGH. Issue PME#"; in D0 with PERST# high it
 * asserts nothing). run63: in D0 no WoW wake in any link mode, the link fully APSS's included. At wake the endpoint's
 * PME_Status and the root port's (RTSTA, cap 0x70 + 0x20) say whether the radio signalled.
 */
static unsigned ep_d3 = 1;
SYSCTL_UINT(_kern_hakios, OID_AUTO, pcie_ep_d3, CTLFLAG_RW | CTLFLAG_LOCKED, &ep_d3, 0,
    "1: the radio in D3hot (PME on) while the APSS-kept link sleeps (mode 3); 2: in DRV sleep too; 0: it stays in D0");
static void pme_settle(void);
static int pme_setup(void);
/* ms between the caller's MHI M3 and D3hot (run68: 95 of 109 wakes were the radio's PME the instant it entered D3hot,
 * PAGE_FAULT or no reason: is the firmware still finishing its WoW entry, reaching host memory, when it is cut off?) */
static unsigned d3_delay_ms;
SYSCTL_UINT(_kern_hakios, OID_AUTO, pcie_d3_delay_ms, CTLFLAG_RW | CTLFLAG_LOCKED, &d3_delay_ms, 0,
    "ms the radio is left in D0 after MHI M3 before D3hot (0: at once)");
static bool ep_in_d3;
static unsigned pme_ep, pme_rc, d3_entered, d3_errors, pmcsr_at_wake, rtsta_at_wake;
static int
ep_d3_enter(void)
{
	int error;
	if (!ep_d3 || !ep_pmc) return 0;
	if ((error = pme_setup())) { d3_errors++; return error; }
	if (d3_delay_ms) delay((int)(d3_delay_ms > 5000 ? 5000 : d3_delay_ms) * 1000);
	if ((error = ep_save())) { d3_errors++; return error; }	/* (as it is now: link PM on, for the way back) */
	(void)wr(dbi + 0x90, 1u << 16);				/* the root port's PME status cleared (W1C) */
	unsigned pmcsr = (ep_cfg[(ep_pmc + 4) / 4] & 0xffff & ~3u) | 3u | (1u << 8) | (1u << 15); /* D3hot, PME_En, PME_Status W1C */
	if ((error = hakios_pcie_cfg_write(0, EP_BDF, ep_pmc + 4, 2, pmcsr))) { d3_errors++; return error; }
	delay(10000); /* D0 -> D3hot: 10 ms */
	ep_in_d3 = true;
	d3_entered++;
	return 0;
}
static int
ep_d3_leave(void)
{
	unsigned pmcsr = 0, rtsta = 0;
	int error;
	if (!ep_in_d3) return 0;
	ep_in_d3 = false;
	(void)hakios_pcie_cfg_read(0, EP_BDF, ep_pmc + 4, 2, &pmcsr);
	(void)rd(dbi + 0x90, &rtsta);
	pmcsr_at_wake = pmcsr;
	rtsta_at_wake = rtsta;
	if (pmcsr & (1u << 15)) pme_ep++;
	if (rtsta & (1u << 16)) pme_rc++;
	if ((error = ep_restore())) { d3_errors++; return error; }	/* (D0, then the header and MSI) */
	if (ep_pcie) {	/* LNKCTL as saved, its ASPM enables included (ep_restore leaves them off) */
		error = hakios_pcie_cfg_write(0, EP_BDF, ep_pcie + 0x10, 2, ep_cfg[(ep_pcie + 0x10) / 4] & 0xffff);
		if (error) { d3_errors++; return error; }
	}
	/*
	 * The PME cleared from its source down: the radio's PME_Status (in D0 now, PME_En off: ep_restore's saved word), the
	 * root port's RTSTA status (a pending PME behind it moves up, and is cleared too), then the PARF event. run66: the
	 * PARF cleared first latched again from the root port's still-set status, and every second doze ended at ~80 ms.
	 */
	(void)hakios_pcie_cfg_write(0, EP_BDF, ep_pmc + 4, 2, (ep_cfg[(ep_pmc + 4) / 4] & 0xffff & ~3u & ~(1u << 8)) | (1u << 15));
	for (unsigned n = 0; n < 4; n++) {
		if (rd(dbi + 0x90, &rtsta) || !(rtsta & (1u << 16))) break;
		(void)wr(dbi + 0x90, 1u << 16);
	}
	pme_settle();
	return 0;
}
static int
drv_votes(bool on)
{
	/* APSS's own clock votes of the controller's journal (the shared vote words): off and on again */
	int errors = 0;
	for (unsigned n = ctl_mark; n < undo_count; n++) {
		if (!undo[n].mask) continue;
		unsigned v;
		if (rd(undo[n].address, &v)) { errors++; continue; }
		errors += !!wr(undo[n].address, on ? v | undo[n].mask : v & ~undo[n].mask);
	}
	return errors;
}
static int
drv_sleep(bool fast_mhi)
{
	int error;
	unsigned v;
	if (!hakios_adsp_pcie_drv) return ENOTSUP;
	if (pm_active != 3) {
		if (!pm_owned && (error = pm_prepare(3))) return error;
		if ((error = pm_activate(3))) return error;
	}
	drv_pm_mode = pm_active;
	/* (D3hot only with pcie_ep_d3=2: cnss_pci_suspend_bus leaves the radio in D0 with DRV, and lx_mhi.c's fast M3 has it
	 * ask for the host by MSI; run64..75 in D3hot: a PME ~80 ms after every entry) */
	if (ep_d3 == 2 && (error = ep_d3_enter())) return error;
	if ((error = hakios_adsp_pcie_drv(DRV_ENABLE, 0))) {
		drv_errors++; drv_last_error = error;
		(void)ep_d3_leave();
		return error;
	}
	asleep = true;          /* (no config or BAR access of APSS's from here) */
	drv_mode = true;
	int errors = drv_votes(false);
	/* the GDSC stays on: Linux's pcie_0_gdsc is PWRSTS_RET_ON (never collapsed by software; retention is the hardware's
	 * when the domain under it goes down). SW_COLLAPSE here powered the controller off under the radio (run53: no MSI
	 * in M3, the data path gone after); pcie_drv_gdsc=1 keeps the old way */
	if (drv_gdsc_off) {
		CHECK(rd(G(GDSCR), &v));
		errors += !!wr(G(GDSCR), v | 1);
	}
	(void)hakios_pcie_bandwidth(false);
	drv_l1ss_requested = drv_l1ss_sleep != 0;
	error = drv_l1ss_requested ? hakios_adsp_pcie_drv(DRV_ENABLE_L1SS_SLEEP, 5) : 0;
	if (error) { drv_errors++; drv_last_error = error; }
	sleeps++;
	asleep_since = now_ns();
	/* Only the verified D0/fast-MHI path may retain the domain across CX collapse. */
	os_atomic_store(&drv_cx_ready,
	    fast_mhi && !ep_in_d3 && !drv_gdsc_off && errors == 0 && drv_l1ss_requested && error == 0,
	    release);
	if (sleeps <= 3 || !(sleeps & (sleeps - 1)))
		say("pcie: DRV sleep %u: handed to the DSP; %d vote writes refused; L1ss requested %u, error %d\n",
		    sleeps, errors, drv_l1ss_requested, error);
	return 0;
}
static int
drv_wake(void)
{
	unsigned v, status = 0;
	int error = 0, e;
	unsigned long long began = now_ns();
	os_atomic_store(&drv_cx_ready, false, release);
	asleep_ns += began - asleep_since;
	if (drv_l1ss_requested) {
		e = hakios_adsp_pcie_drv(DRV_DISABLE_L1SS_SLEEP, 0);
		if (e) { drv_errors++; drv_last_error = e; }
	}
	if (!hakios_pcie_bandwidth(true)) error = EIO;
	CHECK(rd(G(GDSCR), &v));
	CHECK(wr(G(GDSCR), v & ~1u));
	if ((e = wait_bits(G(GDSCR), 1u << 31, 1u << 31, 10000))) error = e;
	/* APSS's clock votes back before any controller register is read (PARF/DBI unclocked: the bus hangs); then a
	 * CLKREQ override the DSP left: the pipe mux through XO and back (as the vendor's resume), the override off */
	if (drv_votes(true)) error = EIO;
	delay(10);
	if (!rd(parf + 0x2b0, &v) && (v & 2)) { (void)wr(G(PIPE_MUX), 2); delay(10); (void)wr(G(PIPE_MUX), 0); }
	if (!rd(parf + 0x2b0, &v) && (v & 0xa)) (void)wr(parf + 0x2b0, v & ~0xau);
	if ((e = hakios_adsp_pcie_drv(DRV_DISABLE, 0))) { drv_errors++; drv_last_error = e; error = error ? error : e; }
	asleep = false;
	drv_mode = false;
	/* the link still up, the endpoint there */
	if (rd(dbi + 0x80, &status) || !(status & (1u << 29))) error = error ? error : ENOLINK;
	if (!error && (e = hakios_pcie_cfg_read(0, EP_BDF, 0, 4, &v)) == 0 && v != ep_cfg[0] && ep_saved) error = ENODEV;
	if (!error && (e = ep_d3_leave())) error = e;
	wakes++;
	if (error) { wake_errors++; say("pcie: DRV wake: error %d (link status %#x)\n", error, status); }
	else if (wakes <= 3 || !(wakes & (wakes - 1)))
		say("pcie: DRV wake %u in %llu us\n", wakes, (now_ns() - began) / 1000);
	return error;
}

/* Caller: MHI already in M3 (no DMA, no doorbells), BAR users stopped, serialized with up/down/wake. */
int
hakios_pcie_sleep(bool fast_mhi)
{
	unsigned v;
	int error;
	if (!started || !linked || !endpoint || asleep) return EINVAL;
	if (sleep_mode == 3) {
		/* (the radio in WoW and MHI in M3 over a link that stays as it is awake: APSS's clocks and votes, L1ss by
		 * itself; the test of whether the radio works in WoW when nothing about the link changes) */
		if (pm_active != 3) {
			if (!pm_owned && (error = pm_prepare(3))) { sleep_errors++; return error; }
			if ((error = pm_activate(3))) { sleep_errors++; return error; }
		}
		if ((error = ep_d3_enter())) { sleep_errors++; return error; }
		asleep = true;	/* (no BAR access in D3hot) */
		sleeps++;
		asleep_since = now_ns();
		return 0;
	}
	if (sleep_mode == 1) {
		if (!ep_saved) (void)ep_save();
		if ((error = drv_sleep(fast_mhi))) { sleep_errors++; return error; }
		return 0;
	}
	if ((error = ep_save())) { sleep_errors++; return error; }
	if (pm_owned) {
		/* link PM off before D3 (pm_disable: both ends' L1/L1SS enables); pm_activate() puts it back after wake */
		if ((error = pm_disable())) { sleep_errors++; return error; }
		pm_active = 0;
	}
	if (ep_pmc && (error = hakios_pcie_cfg_write(0, EP_BDF, ep_pmc + 4, 2, (ep_cfg[(ep_pmc + 4) / 4] & 0xfffcu) | 3u))) {
		sleep_errors++; return error;
	}
	delay(10000); /* D0 -> D3hot: 10 ms */
	/* PME_Turn_Off (ELBI SYS_CTRL bit 4), then L23_Ready (PARF PM_STTS bit 5), at most 100 ms as the vendor's */
	(void)wr(dbi + 0xf24, 1u << 4);
	__asm__ volatile ("dsb sy" ::: "memory");
	bool l23 = false;
	for (unsigned us = 0; us < 100000; us += 1000) {
		if (!rd(parf + 0x24, &v) && v != ~0u && (v & (1u << 5))) { l23 = true; break; }
		delay(1000);
	}
	if (!l23) l23_missed++;
	/* from here the link is gone: no endpoint access, the controller off */
	asleep = true;
	linked = false;
	(void)gpio(94, false);
	delay(1000);
	int errors = 0;
	while (undo_count > ctl_mark) {
		unsigned n = --undo_count;
		errors += !!(undo[n].mask ? vote_clear(undo[n].address, undo[n].mask) : wr(undo[n].address, undo[n].old));
	}
	pm_owned = false;
	sleeps++;
	asleep_since = now_ns();
	if (errors) {
		sleep_errors++;
		say("pcie: link sleep: %d controller writes refused; its floor kept\n", errors);
		return EIO; /* asleep, but the bandwidth floor stays (as an uncertain down) */
	}
	(void)hakios_pcie_bandwidth(false);
	if (sleeps <= 3 || !(sleeps & (sleeps - 1)))
		say("pcie: link sleep %u: L23_Ready %s; controller off\n", sleeps, l23 ? "seen" : "NOT seen");
	return 0;
}

/* Back: floor, controller (clocks_phy, train, SID table), the endpoint's config, link PM as it was. */
int
hakios_pcie_wake(void)
{
	struct hakios_pcie_link link;
	int error;
	if (!started) return EINVAL;
	if (!asleep) return 0;
	if (drv_mode) return drv_wake();
	if (linked && sleep_mode == 3) {
		asleep_ns += now_ns() - asleep_since;
		asleep = false;
		wakes++;
		if ((error = ep_d3_leave())) { wake_errors++; say("pcie: D3hot leave failed %d\n", error); return error; }
		return 0;
	}
	if (!hakios_pcie_bandwidth(true)) { wake_errors++; return EIO; }
	asleep = false; /* (the controller's own accesses below; the endpoint stays refused until linked) */
	unsigned long long began = now_ns();
	asleep_ns += began - asleep_since;
	bzero(&link, sizeof(link));
	if ((error = clocks_phy()) || (error = train(&link)) || (error = bdf_to_sid()) || (error = ep_restore())) {
		wake_errors++;
		say("pcie: link wake failed %d\n", error);
		failure_dump();
		return error;
	}
	if (pm_requested && pm_valid) {
		if ((error = pm_prepare(pm_requested)) || (error = pm_activate(pm_requested))) {
			say("pcie: link PM after wake failed %d; restored %d\n", error, pm_restore());
			error = 0;
		}
	}
	wakes++;
	unsigned long long took = now_ns() - began;
	if (wakes <= 3 || !(wakes & (wakes - 1)))
		say("pcie: link wake %u: gen %u x%u in %llu us\n", wakes, link.gen, link.width, took / 1000);
	return 0;
}
bool hakios_pcie_asleep(void) { return asleep; }

/* pci-msm prevent_l1/allow_l1 around the fast MHI resume, with APSS already owning RC0. */
int
hakios_pcie_l1_restore(unsigned saved)
{
	unsigned link, pm;
	uint16_t linkctl;

	CHECK(rd(parf + 0x20, &pm));
	CHECK(wr(parf + 0x20, (pm & ~(1u << 5)) | (saved & (1u << 5))));
	CHECK(rd(dbi + 0x80, &link));
	linkctl = (link & ~2u) | (saved & 2u);
	return wr(dbi + 0x80, linkctl); /* upper half is Link Status, including W1C bits */
}

int
hakios_pcie_l1_hold(unsigned *saved)
{
	unsigned link, pm, ltssm;
	int error;

	if (!started || !linked || asleep) return EINVAL;
	CHECK(rd(dbi + 0x80, &link));
	CHECK(rd(parf + 0x20, &pm));
	*saved = (link & 2u) | (pm & (1u << 5));
	error = wr(dbi + 0x80, link & 0xfffdu);
	if (!error) error = wr(parf + 0x20, pm | (1u << 5));
	if (!error) {
		for (unsigned us = 0; us < 100000; us += 100) {
			error = rd(parf + 0x1b0, &ltssm);
			if (error) break;
			if ((ltssm & 63u) == 0x11 || (ltssm & 63u) == 0x12) return 0;
			delay(100);
		}
	}
	int restore = hakios_pcie_l1_restore(*saved);
	if (restore) return restore;
	return error ? error : ETIMEDOUT;
}

/* The borrowed BAR pointers' accessors (lx.c): refused, not attempted, while the controller is off. */
int
hakios_pcie_bar_read32(const volatile void *va, unsigned *value)
{
	if (asleep) { refused_bar++; return ENXIO; }
	return hakios_pcie_try_read32(va, value);
}
int
hakios_pcie_bar_write32(volatile void *va, unsigned value)
{
	if (asleep) { refused_bar++; return ENXIO; }
	return hakios_pcie_try_write32(va, value);
}

/* ---- WAKE# (GPIO 96, active low): PDC pin 104 (Linux pinctrl-sm8450.c { 96, 104 }), pdc-ranges <94 609 31>: SPI 619,
 * INTID 651; level low at the PDC, so it wakes a collapsed CX as the touch line does. The handler masks it; arm unmasks. */
#define WAKE_PIN 96u
#define WAKE_PDC_PA 0x0b220000UL
#define WAKE_PDC_PIN 104u
#define WAKE_INTID 651u
extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *, uint32_t), void *context);
extern void gic_v3_spi_mask(uint32_t intid);
extern void gic_v3_spi_unmask(uint32_t intid);
static vm_offset_t wake_pdc;
static void (*wake_cb)(void);
static bool wake_ready;
static unsigned wake_irqs, wake_low_at_irq;
static unsigned long long wake_armed_at, wake_after_us[8];
static void
wake_arrived(__unused void *context, uint32_t intid)
{
	unsigned io = 1;
	gic_v3_spi_mask(intid);
	if (tlmm) (void)rd(tlmm + PIN(WAKE_PIN) + 4, &io);
	if (!(io & 1)) wake_low_at_irq++;
	wake_after_us[wake_irqs % 8] = wake_armed_at ? (now_ns() - wake_armed_at) / 1000 : 0;
	wake_irqs++;
	if (wake_cb) wake_cb();
}
int
hakios_pcie_wake_irq(void (*cb)(void))
{
	unsigned cfg, version;
	if (wake_ready) { wake_cb = cb; return 0; }
	if (!tlmm) return ENXIO;
	if (!wake_pdc) wake_pdc = MAP_IO(WAKE_PDC_PA, 0x2000);
	if (!wake_pdc) return ENOMEM;
	/* GPIO 96: function 0, input, pull-up (the radio drives it low); its TLMM summary interrupt off */
	CHECK(rd(tlmm + PIN(WAKE_PIN), &cfg));
	CHECK(wr(tlmm + PIN(WAKE_PIN), (cfg & ~0x3ffu) | 3u | (1u << 6)));
	CHECK(rd(tlmm + PIN(WAKE_PIN) + 8, &cfg));
	CHECK(wr(tlmm + PIN(WAKE_PIN) + 8, cfg & ~1u));
	CHECK(wr(tlmm + PIN(WAKE_PIN) + 0xc, 0));
	version = *(volatile unsigned *)(wake_pdc + 0x1000);
	*(volatile unsigned *)(wake_pdc + 0x110 + 4 * WAKE_PDC_PIN) = version >= 0x30200u ? 8u : 0u; /* level low (+ enable) */
	if (version < 0x30200u) {
		volatile unsigned *bank = (volatile unsigned *)(wake_pdc + 0x10 + 4 * (WAKE_PDC_PIN / 32));
		*bank |= 1u << (WAKE_PDC_PIN % 32);
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	wake_cb = cb;
	if (gic_v3_spi_attach(WAKE_INTID, false, wake_arrived, NULL) != KERN_SUCCESS) return EIO;
	gic_v3_spi_mask(WAKE_INTID);
	wake_ready = true;
	say("pcie: WAKE# GPIO %u -> PDC pin %u (version %#x) -> GIC %u\n", WAKE_PIN, WAKE_PDC_PIN, version, WAKE_INTID);
	return 0;
}
/*
 * The root port's PME (run64: with the radio in D3hot every WoW wake is a PM_PME message -- the radio's PME_Status and
 * the root port's RTSTA PME set at wake -- and nothing took it). The controller's global interrupt (DT pcie0
 * interrupt-map entry 0: GIC SPI 0x8c, INTID 172, level) carries it: PARF_INT_ALL_STATUS/CLEAR/MASK 0x224/0x228/0x22c
 * (pcie-qcom.c), the events numbered as pci-msm.c's MSM_PCIE_INT_EVT_* (LINK_UP bit 13 and MSI_0..7 bits 23..30 there
 * are upstream's too): PME_LEGACY bit 16, PLS_PME bit 17. The root port raises it with Root Control's PME Interrupt
 * Enable (cap 0x70 + 0x1c, bit 3). The handler only masks and calls the wake (no PARF access: in DRV sleep APSS's
 * clocks of the controller are off); the wake reads and clears it once they are back (pme_settle).
 */
#define PME_INTID 172u
#define PARF_INT_ALL_STATUS 0x224u
#define PARF_INT_ALL_CLEAR 0x228u
#define PARF_INT_ALL_MASK 0x22cu
#define PARF_INT_PME ((1u << 16) | (1u << 17))
static bool pme_ready;
static unsigned pme_irqs, parf_at_wake, parf_pme_seen;
static void
pme_arrived(__unused void *context, uint32_t intid)
{
	gic_v3_spi_mask(intid);
	pme_irqs++;
	if (wake_cb) wake_cb();
}
static int
pme_setup(void)
{
	unsigned v;
	if (pme_ready) return 0;
	if (!parf || !dbi || asleep) return ENXIO;
	CHECK(rd(parf + PARF_INT_ALL_MASK, &v));
	CHECK(wr(parf + PARF_INT_ALL_MASK, v | PARF_INT_PME));
	CHECK(wr(parf + PARF_INT_ALL_CLEAR, PARF_INT_PME));
	CHECK(wr(dbi + 0x90, 1u << 16));	/* RTSTA PME status (W1C) */
	CHECK(rd(dbi + 0x8c, &v));
	CHECK(wr(dbi + 0x8c, (v & 0xffffu) | (1u << 3)));	/* Root Control: PME Interrupt Enable */
	if (gic_v3_spi_attach(PME_INTID, false, pme_arrived, NULL) != KERN_SUCCESS) return EIO;
	gic_v3_spi_mask(PME_INTID);
	pme_ready = true;
	say("pcie: the root port's PME -> PARF event bits 16/17 -> GIC %u\n", PME_INTID);
	return 0;
}
/* at wake, the controller clocked again: what the PARF said, cleared */
static void
pme_settle(void)
{
	unsigned v = 0;
	if (!pme_ready || !parf || rd(parf + PARF_INT_ALL_STATUS, &v)) return;
	parf_at_wake = v;
	if (v & PARF_INT_PME) parf_pme_seen++;
	(void)wr(parf + PARF_INT_ALL_CLEAR, v & PARF_INT_PME);
}
void
hakios_pcie_wake_arm(bool on)
{
	if (wake_ready) { if (on) { wake_armed_at = now_ns(); gic_v3_spi_unmask(WAKE_INTID); } else gic_v3_spi_mask(WAKE_INTID); }
	if (pme_ready) {
		if (on && ep_in_d3) gic_v3_spi_unmask(PME_INTID);
		else gic_v3_spi_mask(PME_INTID);
	}
}
unsigned hakios_pcie_sleep_mode(void) { return sleep_mode; }
bool
hakios_pcie_wake_line(void)
{
	unsigned io;
	return tlmm && !rd(tlmm + PIN(WAKE_PIN) + 4, &io) && !(io & 1); /* true: WAKE# asserted (low) */
}

SYSCTL_DECL(_kern_hakios);
static int sysctl_pcie_sleep SYSCTL_HANDLER_ARGS
{
#pragma unused(arg1, arg2)
	char text[640];
	unsigned long long ns = asleep_ns + (asleep ? now_ns() - asleep_since : 0);
	int at = snprintf(text, sizeof(text), "mode %u (drv %u, errors %u last %u, pm %u) asleep %u sleeps %u wakes %u sleep_errors %u wake_errors %u l23_missed %u refused_bar %u "
	    "wake_irqs %u (line low at it %u) wake_line %u asleep_ms %llu; irq after arm us", sleep_mode, drv_mode, drv_errors,
	    drv_last_error, drv_pm_mode, asleep, sleeps, wakes, sleep_errors,
	    wake_errors, l23_missed, refused_bar, wake_irqs, wake_low_at_irq, hakios_pcie_wake_line(), ns / 1000000);
	at += snprintf(text + at, sizeof(text) - (size_t)at, "; D3hot %u (errors %u), PME at wake: radio %u root port %u, "
	    "last PMCSR %#x RTSTA %#x; PME irqs %u (ready %u), PARF PME at wake %u, last PARF status %#x;", d3_entered,
	    d3_errors, pme_ep, pme_rc, pmcsr_at_wake, rtsta_at_wake, pme_irqs, pme_ready, parf_pme_seen, parf_at_wake);
	for (unsigned k = wake_irqs > 8 ? wake_irqs - 8 : 0; k < wake_irqs && at < (int)sizeof(text) - 16; k++)
		at += snprintf(text + at, sizeof(text) - (size_t)at, " %llu", wake_after_us[k % 8]);
	return sysctl_handle_string(oidp, text, 0, req);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, pcie_sleep, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sysctl_pcie_sleep, "A", "PCIe0 link sleep with the radio powered: counts and time asleep");
static int sysctl_pcie_power SYSCTL_HANDLER_ARGS
{
#pragma unused(arg1, arg2)
	char text[320];
	snprintf(text, sizeof(text), "started %u linked %u endpoint %u rails %#x tcs_owned %u tcs_poisoned %u down_errors %u ups %u downs %u down_status %d",
	    started, linked, endpoint, rail_mask, tcs_owned, tcs_poisoned, down_errors, up_count, down_count, hakios_pcie_down_status());
	return sysctl_handle_string(oidp, text, 0, req);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, pcie_power, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sysctl_pcie_power, "A", "Cached PCIe lifecycle and acknowledged rail enables");
SYSCTL_STRING(_kern_hakios, OID_AUTO, pcie_pm, CTLFLAG_RD | CTLFLAG_LOCKED,
    pm_description, sizeof(pm_description), "PCIe power capabilities at last enumeration");
static int pm_audit_request;
static char pm_audit[512] = "not requested";
SYSCTL_INT(_kern_hakios, OID_AUTO, pcie_pm_audit_request, CTLFLAG_RW | CTLFLAG_LOCKED,
    &pm_audit_request, 0, "Request one configuration audit on the existing PCI worker");
SYSCTL_STRING(_kern_hakios, OID_AUTO, pcie_pm_audit, CTLFLAG_RD | CTLFLAG_LOCKED,
    pm_audit, sizeof(pm_audit), "Cached configuration audit; requesting it briefly wakes the link");
static int sysctl_pcie_aspm SYSCTL_HANDLER_ARGS
{
#pragma unused(arg1, arg2)
	int value = pm_requested, e = sysctl_handle_int(oidp, &value, 0, req);
	if (e || req->newptr == USER_ADDR_NULL) return e;
	if (value < 0 || value > 3) return EINVAL;
	pm_requested = value;
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, pcie_aspm, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sysctl_pcie_aspm, "I", "Link PM on next Wi-Fi start: 0 inherited, 1 L1, 2 L1.1, 3 L1.2");
static int sysctl_pcie_pm_stats SYSCTL_HANDLER_ARGS
{
#pragma unused(arg1, arg2)
	char text[320];
	snprintf(text, sizeof(text), "requested %d active %d owned %u error %d samples %llu l1 %llu l1ss %llu last_ltssm %#x last_pm_status %#x",
	    pm_requested, pm_active, pm_owned, pm_error, pm_samples, pm_l1_samples, pm_ss_samples, pm_last_ltssm, pm_last_status);
	return sysctl_handle_string(oidp, text, 0, req);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, pcie_pm_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sysctl_pcie_pm_stats, "A", "Link-state observations on existing PCI worker wakes; not residency");
int hakios_pcie_pm_start(void)
{
	if (!pm_prepared || !pm_owned) return pm_error;
	pm_error = pm_activate(pm_prepared);
	if (pm_error) {
		int restore = pm_restore();
		say("pcie: link PM activation failed %d; restoration %d\n", pm_error, restore);
		if (restore) { down_errors++; return restore; }
	} else say("pcie: link PM profile %d enabled after firmware ready\n", pm_active);
	return pm_error;
}
void hakios_pcie_pm_sample(void)
{
	unsigned ltssm, status;
	/* Only the PCI worker calls this, before servicing BARs. Removal joins
	 * that worker before C2 restores registers or powers down the controller. */
	if (!pm_active || !started || !linked || asleep) return; /* (asleep: its clocks are not ours; a read would hang the bus) */
	if (pm_audit_request) {
		unsigned ctl[2] = {0}, ss[2] = {0}, dc[2] = {0}, sys = 0, clkreq = 0, ack = 0;
		int e = 0;
		pm_audit_request = 0;
		for (unsigned i = 0; i < 2; i++) {
			e |= hakios_pcie_cfg_read(0, pm_ports[i].bdf, pm_ports[i].pc + 0x10, 2, &ctl[i]);
			e |= hakios_pcie_cfg_read(0, pm_ports[i].bdf, pm_ports[i].ss + 8, 4, &ss[i]);
			e |= hakios_pcie_cfg_read(0, pm_ports[i].bdf, pm_ports[i].pc + 0x28, 2, &dc[i]);
		}
		e |= rd(parf, &sys); e |= rd(parf + 0x2b0, &clkreq); e |= rd(dbi + 0x70c, &ack);
		snprintf(pm_audit, sizeof(pm_audit), "error %d root ctl %#x ss %#x devctl2 %#x endpoint ctl %#x ss %#x devctl2 %#x parf_sys %#x clkreq %#x ack %#x",
		    e, ctl[0], ss[0], dc[0], ctl[1], ss[1], dc[1], sys, clkreq, ack);
	}
	if (rd(parf + 0x1b0, &ltssm) || rd(parf + 0x24, &status)) return;
	pm_samples++;
	pm_last_ltssm = ltssm & 63; pm_last_status = status;
	if ((ltssm & 63) == 0x14) pm_l1_samples++;
	if (status & (1u << 8)) pm_ss_samples++; /* vendor's L1ss indicator */
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
	if (down_errors || rail_mask) return EIO; /* uncertain shutdown: do not reuse */
	static bool pm_args_read;
	if (!pm_args_read) {
		unsigned mode = 0;
		if ((error = numeric_word("pcie_aspm", &mode, 0, 3))) goto failed;
		pm_requested = mode; pm_args_read = true;
	}
	generation = 2;
	if ((error = numeric_word("pcie_gen", &generation, 1, 2))) goto failed;
	if (generation != 1 && generation != 2) { error = EINVAL; goto failed; }
	if (!hakios_mmio_try_read32 || !hakios_mmio_try_write32) { error = ENOTSUP; goto failed; }
	/* RC0 is explicitly selected by the caller; no redwood auto-detection. */
	if (tcs_poisoned) { error = EBUSY; goto failed; }
	step_pause_ms = 0;
	if (PE_parse_boot_argn("pcie_pause", &step_pause_ms, sizeof(step_pause_ms)) && step_pause_ms > 10000) step_pause_ms = 10000;
	step("0 C2 up: rc 0, pcie_gen %u", generation);
	if ((error = map_blocks())) goto failed;
	started = true;
	if (!hakios_pcie_bandwidth(true)) { error = EIO; goto failed; }
	if ((error = power())) goto failed;
	ctl_mark = undo_count;
	asleep = false;
	hakios_wifi_stage(1, "rails", true, "pcie: rails (6 incl ant switch), BT_EN low, wlan_en high");
	stage = 2;
	if ((error = clocks_phy()) || (error = train(link))) goto failed;
	hakios_wifi_stage(2, "link", true, "pcie: link up gen %u x%u", link->gen, link->width);
	stage = 3;
	if ((error = bdf_to_sid())) goto failed;
	if ((error = enumerate(link))) goto failed;
	up_count++;
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
