/*
 * Can this kernel use the GIC's ITS and LPIs under the hypervisor? Only with boot-arg its_probe=1
 * (WIFI-PLAN.md section 7.3, P1a: the first check before an ITS driver is written).
 *
 * Today every PCIe interrupt of the Wi-Fi chip goes to a page of RAM and a 1 ms thread polls it. On SM8450 the
 * PCIe MSIs are meant for the GIC-600's ITS (the device tree: msi-controller@17140000, PCIe0's msi-map gives the
 * radio DeviceID 0x5981). Linux on Android uses it, so Gunyah lets its guest have an ITS; whether that is the real
 * one or an emulation, and whether HLOS may program it and the redistributors' LPI registers, is what this finds out.
 *
 * Part 1, read: GICD_TYPER (LPIS, MBIS, IDbits), the IIDRs, every redistributor's TYPER/CTLR/PROPBASER/PENDBASER,
 * the ITS's CTLR/TYPER/CBASER/BASER0-7. Part 2, try-write (guarded: a refused access is a line, not a reset), each
 * value read back and the old one put back: this CPU's PROPBASER and PENDBASER (only while its LPIs are off), the
 * ITS's CBASER and its first BASER (only while the ITS is off). Part 3, only when part 2 found all of them writable:
 * one LPI end to end. Tables in RAM, this CPU's LPIs on, the ITS on, MAPC/MAPD/MAPTI for DeviceID 0x5981 event 0 ->
 * LPI 8192, then the ITS's own INT command makes the interrupt as if the radio had written it; with interrupts off
 * on this CPU, ICC_HPPIR1 must show 8192, and it is acknowledged and ended here. Then everything is unmapped and the
 * ITS switched off; the tables stay allocated, because a redistributor's EnableLPIs may not go back to 0.
 * The last line is "its: RESULT ..." with what P1a takes from it.
 *
 * Nothing here runs without its_probe=1. A 64-bit register is read and written whole: Gunyah's virtual GIC (IIDR
 * implementer 0x070) refuses or drops 32-bit halves of them, which real GIC-600 would take.
 *
 * Boot-args:
 *   its_probe=1        run the three parts once, a second after start
 *   its_probe=2        parts 1 and 2 only (no LPI is switched on)
 */
#include <kern/clock.h>
#include <kern/cpu_number.h>
#include <kern/startup.h>
#include <kern/processor.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/systm.h>
#include <vm/memory_types.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map_unmappable(vm_offset_t, vm_size_t, unsigned int);
extern uint64_t kvtophys(vm_offset_t va);
extern void FlushPoC_DcacheRegion(vm_offset_t va, size_t length);
extern int hakios_mmio_try_read32(const volatile void *, unsigned *) __attribute__((weak));
extern int hakios_mmio_try_write32(volatile void *, unsigned) __attribute__((weak));
extern int hakios_mmio_try_read64(const volatile void *, unsigned long long *) __attribute__((weak));
extern int hakios_mmio_try_write64(volatile void *, unsigned long long) __attribute__((weak));
extern kern_return_t thread_terminate(thread_t);
extern void loader_log_putc(char) __attribute__((weak));

/* SM8450 (live.dts: interrupt-controller@17100000, msi-controller@17140000, redistributor-stride 0x40000) */
#define GICD_PA                 0x17100000UL
#define GICD_SIZE               0x10000UL
#define GICR_PA                 0x17180000UL
#define GICR_SIZE               0x200000UL
#define GICR_STRIDE             0x40000UL
#define GITS_PA                 0x17140000UL
#define GITS_SIZE               0x20000UL
#define RADIO_DEVICE_ID         0x5981u         /* PCIe0 msi-map: RID 0x100 -> 0x5981 */

#define GICD_CTLR               0x0000
#define GICD_TYPER              0x0004
#define GICD_IIDR               0x0008
#define GICD_TYPER2             0x000c
#define GICD_TYPER_MBIS         (1u << 16)
#define GICD_TYPER_LPIS         (1u << 17)

#define GICR_CTLR               0x0000
#define GICR_IIDR               0x0004
#define GICR_TYPER              0x0008
#define GICR_PROPBASER          0x0070
#define GICR_PENDBASER          0x0078
#define GICR_CTLR_ENABLE_LPIS   (1u << 0)
#define GICR_CTLR_RWP           (1u << 3)
#define GICR_TYPER_PLPIS        (1u << 0)
#define GICR_TYPER_DIRECTLPI    (1u << 3)
#define GICR_TYPER_LAST         (1u << 4)

#define GITS_CTLR               0x0000
#define GITS_IIDR               0x0004
#define GITS_TYPER              0x0008
#define GITS_CBASER             0x0080
#define GITS_CWRITER            0x0088
#define GITS_CREADR             0x0090
#define GITS_BASER(n)           (0x0100 + 8 * (n))
#define GITS_CTLR_ENABLED       (1u << 0)
#define GITS_CTLR_QUIESCENT     (1u << 31)

/* table attributes: inner shareable, inner write-back read/write-allocate (Linux's GIC_BASER_CACHE_RaWaWb) */
#define BASER_VALID             (1ULL << 63)
#define BASER_INNER_WB          (7ULL << 59)
#define BASER_SHARE_INNER       (1ULL << 10)
#define BASER_TYPE(v)           (((v) >> 56) & 7)
#define BASER_ESIZE(v)          ((((v) >> 48) & 0x1f) + 1)
#define RDIST_INNER_WB          (7ULL << 7)
#define RDIST_SHARE_INNER       (1ULL << 10)

#define LPI_FIRST               8192u
#define LPI_ID_BITS             14u             /* LPIs 8192..16383: 8 KiB of properties, 2 KiB of pending bits */
#define LPI_PRIORITY            0x80u           /* above the timer's 0xa0, so ICC_HPPIR1 shows it first */
#define ITT_EVENT_BITS          5u

static vm_offset_t gicd, gicr, gits;
static int failures;

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[300];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	if (loader_log_putc) {
		for (const char *c = line; *c != 0; c++) {
			loader_log_putc(*c);
		}
	}
	return printf("%s", line);
}

/* Guarded accesses; a refused one is said once per register and counted. */
static bool
rd32(vm_offset_t base, unsigned offset, uint32_t *value, const char *name)
{
	unsigned v = 0;
	int error = hakios_mmio_try_read32((const volatile void *)(base + offset), &v);
	if (error) {
		say("its: read of %s refused: errno %d\n", name, error);
		failures++;
		return false;
	}
	*value = v;
	return true;
}

static bool
wr32(vm_offset_t base, unsigned offset, uint32_t value, const char *name)
{
	int error = hakios_mmio_try_write32((volatile void *)(base + offset), value);
	if (error) {
		say("its: write of %s refused: errno %d\n", name, error);
		failures++;
		return false;
	}
	return true;
}

static bool
rd64(vm_offset_t base, unsigned offset, uint64_t *value, const char *name)
{
	unsigned long long v = 0;
	int error = hakios_mmio_try_read64((const volatile void *)(base + offset), &v);
	if (error) {
		say("its: read of %s refused: errno %d\n", name, error);
		failures++;
		return false;
	}
	*value = v;
	return true;
}

/* One 64-bit access: Gunyah's virtual GIC refuses the 32-bit halves of its 64-bit registers (p1a-its, 2026-11-13). */
static bool
wr64(vm_offset_t base, unsigned offset, uint64_t value, const char *name)
{
	int error = hakios_mmio_try_write64((volatile void *)(base + offset), value);
	if (error) {
		say("its: write of %s refused: errno %d\n", name, error);
		failures++;
		return false;
	}
	return true;
}

/* Write, read back, put the old value back; true when the bits in `care` stuck. */
static bool
try64(vm_offset_t base, unsigned offset, uint64_t test, uint64_t care, const char *name)
{
	uint64_t old, seen = 0, back = 0;
	if (!rd64(base, offset, &old, name)) {
		return false;
	}
	if (!wr64(base, offset, test, name)) {
		return false;
	}
	(void)rd64(base, offset, &seen, name);
	(void)wr64(base, offset, old, name);
	(void)rd64(base, offset, &back, name);
	bool stuck = (seen & care) == (test & care);
	say("its: try %s: was %#llx, wrote %#llx, read %#llx (%s), restored %#llx%s\n", name, old, test, seen,
	    stuck ? "WRITABLE" : "not kept", back, back == old ? "" : " (NOT the old value)");
	return stuck;
}

/* This CPU's redistributor frame: GICR_TYPER[63:32] is its affinity (Aff3.Aff2.Aff1.Aff0). */
static vm_offset_t
my_frame(uint64_t *typer_out, vm_offset_t *pa_out)
{
	uint64_t mpidr = __builtin_arm_rsr64("MPIDR_EL1");
	uint32_t affinity = (uint32_t)(((mpidr >> 32) & 0xff) << 24 | (mpidr & 0xffffff));
	for (vm_offset_t off = 0; off + GICR_STRIDE <= GICR_SIZE; off += GICR_STRIDE) {
		uint64_t typer;
		if (!rd64(gicr + off, GICR_TYPER, &typer, "GICR_TYPER")) {
			return 0;
		}
		if ((uint32_t)(typer >> 32) == affinity) {
			*typer_out = typer;
			*pa_out = GICR_PA + off;
			return gicr + off;
		}
		if (typer & GICR_TYPER_LAST) {
			break;
		}
	}
	return 0;
}

static void
part1(void)
{
	uint32_t ctlr = 0, typer = 0, iidr = 0, typer2 = 0;
	(void)rd32(gicd, GICD_CTLR, &ctlr, "GICD_CTLR");
	(void)rd32(gicd, GICD_TYPER, &typer, "GICD_TYPER");
	(void)rd32(gicd, GICD_IIDR, &iidr, "GICD_IIDR");
	(void)rd32(gicd, GICD_TYPER2, &typer2, "GICD_TYPER2");
	say("its: GICD ctlr %#x typer %#x (LPIS %u, MBIS %u, IDbits %u, ITLines %u) typer2 %#x iidr %#x (implementer %#x product %#x)\n",
	    ctlr, typer, !!(typer & GICD_TYPER_LPIS), !!(typer & GICD_TYPER_MBIS), ((typer >> 19) & 0x1f) + 1, typer & 0x1f,
	    typer2, iidr, iidr & 0xfff, iidr >> 24);
	for (vm_offset_t off = 0; off + GICR_STRIDE <= GICR_SIZE; off += GICR_STRIDE) {
		uint64_t rt = 0, prop = 0, pend = 0;
		uint32_t rc = 0, ri = 0;
		if (!rd64(gicr + off, GICR_TYPER, &rt, "GICR_TYPER")) {
			break;
		}
		(void)rd32(gicr + off, GICR_CTLR, &rc, "GICR_CTLR");
		(void)rd32(gicr + off, GICR_IIDR, &ri, "GICR_IIDR");
		(void)rd64(gicr + off, GICR_PROPBASER, &prop, "GICR_PROPBASER");
		(void)rd64(gicr + off, GICR_PENDBASER, &pend, "GICR_PENDBASER");
		say("its: GICR %#lx: affinity %#x processor %u PLPIS %u DirectLPI %u ctlr %#x (EnableLPIs %u) iidr %#x propbaser %#llx pendbaser %#llx\n",
		    GICR_PA + off, (uint32_t)(rt >> 32), (uint32_t)((rt >> 8) & 0xffff), !!(rt & GICR_TYPER_PLPIS),
		    !!(rt & GICR_TYPER_DIRECTLPI), rc, rc & GICR_CTLR_ENABLE_LPIS, ri, prop, pend);
		if (rt & GICR_TYPER_LAST) {
			break;
		}
	}
	uint32_t gctlr = 0, giidr = 0;
	uint64_t gtyper = 0, cbaser = 0, creadr = 0, cwriter = 0;
	(void)rd32(gits, GITS_CTLR, &gctlr, "GITS_CTLR");
	(void)rd32(gits, GITS_IIDR, &giidr, "GITS_IIDR");
	(void)rd64(gits, GITS_TYPER, &gtyper, "GITS_TYPER");
	(void)rd64(gits, GITS_CBASER, &cbaser, "GITS_CBASER");
	(void)rd64(gits, GITS_CWRITER, &cwriter, "GITS_CWRITER");
	(void)rd64(gits, GITS_CREADR, &creadr, "GITS_CREADR");
	say("its: GITS ctlr %#x (enabled %u quiescent %u) iidr %#x typer %#llx (physical %u, ITT entry %llu B, IDbits %llu, Devbits %llu, PTA %u, HCC %llu)"
	    " cbaser %#llx cwriter %#llx creadr %#llx\n", gctlr, gctlr & GITS_CTLR_ENABLED, !!(gctlr & GITS_CTLR_QUIESCENT), giidr,
	    gtyper, (unsigned)(gtyper & 1), ((gtyper >> 4) & 0xf) + 1, ((gtyper >> 8) & 0x1f) + 1, ((gtyper >> 13) & 0x1f) + 1,
	    (unsigned)((gtyper >> 19) & 1), (gtyper >> 24) & 0xff, cbaser, cwriter, creadr);
	for (unsigned n = 0; n < 8; n++) {
		uint64_t b = 0;
		if (rd64(gits, GITS_BASER(n), &b, "GITS_BASER")) {
			say("its: GITS_BASER%u %#llx (type %llu, entry %llu B, valid %u, indirect %u, page size code %llu, size %llu)\n", n, b,
			    BASER_TYPE(b), BASER_ESIZE(b), (unsigned)(b >> 63), (unsigned)((b >> 62) & 1), (b >> 8) & 3, (b & 0xff) + 1);
		}
	}
}

/* Contiguous, zeroed, physically aligned to `align`; never freed (see the header). */
static vm_offset_t
table(vm_size_t size, vm_size_t align, uint64_t *pa)
{
	vm_offset_t va = 0;
	ppnum_t pnum_mask = align > PAGE_SIZE ? (ppnum_t)((align >> PAGE_SHIFT) - 1) : 0;
	size = round_page(size);
	if (kmem_alloc_contig(kernel_map, &va, size, PAGE_MASK, 0, pnum_mask, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return 0;
	}
	*pa = kvtophys(va);
	if (*pa & (align - 1)) {
		say("its: table at %#llx is not %#lx-aligned\n", *pa, (unsigned long)align);
		return 0;
	}
	FlushPoC_DcacheRegion(va, size);
	return va;
}

static bool
part2(vm_offset_t frame, uint64_t scratch_pa, bool *rd_ok, bool *its_ok)
{
	uint32_t rc = 0, gctlr = 0;
	*rd_ok = *its_ok = false;
	if (rd32(frame, GICR_CTLR, &rc, "GICR_CTLR") && !(rc & GICR_CTLR_ENABLE_LPIS)) {
		bool p = try64(frame, GICR_PROPBASER, scratch_pa | RDIST_SHARE_INNER | RDIST_INNER_WB | (LPI_ID_BITS - 1),
		    0x000ffffffffff000ULL | 0x1f, "this CPU's GICR_PROPBASER");
		bool q = try64(frame, GICR_PENDBASER, scratch_pa | RDIST_SHARE_INNER | RDIST_INNER_WB,
		    0x000fffffffff0000ULL, "this CPU's GICR_PENDBASER");
		*rd_ok = p && q;
	} else {
		say("its: this CPU's LPIs are already on (GICR_CTLR %#x): its PROPBASER/PENDBASER cannot be changed\n", rc);
	}
	if (rd32(gits, GITS_CTLR, &gctlr, "GITS_CTLR") && !(gctlr & GITS_CTLR_ENABLED)) {
		uint64_t b0 = 0;
		bool c = try64(gits, GITS_CBASER, scratch_pa | BASER_INNER_WB | BASER_SHARE_INNER, 0x000ffffffffff000ULL, "GITS_CBASER");
		bool d = false;
		if (rd64(gits, GITS_BASER(0), &b0, "GITS_BASER0") && BASER_TYPE(b0) != 0) {
			d = try64(gits, GITS_BASER(0), (b0 & (7ULL << 56)) | scratch_pa | BASER_INNER_WB | BASER_SHARE_INNER,
			    0x0000fffffffff000ULL, "GITS_BASER0 (valid 0)");
		} else {
			say("its: GITS_BASER0 has no table type (%#llx): not written\n", b0);
		}
		*its_ok = c && d;
	} else {
		say("its: the ITS is already on (GITS_CTLR %#x): its tables cannot be changed\n", gctlr);
	}
	return *rd_ok && *its_ok;
}

/* ---- part 3: one LPI through the ITS ---- */

static uint64_t *queue;                 /* 4 KiB command queue: 128 commands of 32 bytes */
static unsigned queue_next;

static uint64_t
micros(void)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000;
}

static bool
command(const char *what, uint64_t d0, uint64_t d1, uint64_t d2)
{
	uint64_t *c = queue + 4 * queue_next;
	c[0] = d0; c[1] = d1; c[2] = d2; c[3] = 0;
	FlushPoC_DcacheRegion((vm_offset_t)c, 32);
	__builtin_arm_dsb(0xf);
	queue_next = (queue_next + 1) % 128;
	if (!wr64(gits, GITS_CWRITER, (uint64_t)queue_next * 32, "GITS_CWRITER")) {
		return false;
	}
	uint64_t start = micros(), r = 0;
	for (;;) {
		if (!rd64(gits, GITS_CREADR, &r, "GITS_CREADR")) {
			return false;
		}
		if (r & 1) {
			say("its: command %s stalled the queue (CREADR %#llx)\n", what, r);
			return false;
		}
		if ((r & 0xfffe0) == (uint64_t)queue_next * 32) {
			return true;
		}
		if (micros() - start > 10000) {
			say("its: command %s not taken in 10 ms (CREADR %#llx, CWRITER %#x)\n", what, r, queue_next * 32);
			return false;
		}
	}
}

static const char *
part3(vm_offset_t frame, vm_offset_t frame_pa, uint64_t frame_typer, uint64_t *latency_us)
{
	uint64_t gtyper = 0, prop_pa = 0, pend_pa = 0, itt_pa = 0, queue_pa = 0, x = 0;
	uint32_t v = 0;
	if (!rd64(gits, GITS_TYPER, &gtyper, "GITS_TYPER")) {
		return "GITS_TYPER unreadable";
	}
	unsigned devbits = (unsigned)((gtyper >> 13) & 0x1f) + 1, itt_entry = (unsigned)((gtyper >> 4) & 0xf) + 1;
	unsigned hcc = (unsigned)((gtyper >> 24) & 0xff);
	bool pta = (gtyper >> 19) & 1;
	if (devbits < 15) {
		return "the ITS has fewer than 15 DeviceID bits";
	}

	/* the redistributor's tables, then its LPIs on */
	uint8_t *prop = (uint8_t *)table((1u << LPI_ID_BITS) - LPI_FIRST, PAGE_SIZE, &prop_pa);
	vm_offset_t pend = table(0x10000, 0x10000, &pend_pa);
	vm_offset_t itt = table((1u << ITT_EVENT_BITS) * itt_entry, 256, &itt_pa);
	queue = (uint64_t *)table(4096, 0x10000, &queue_pa);
	if (!prop || !pend || !itt || !queue) {
		return "no memory for the tables";
	}
	prop[0] = LPI_PRIORITY | 2 | 1;         /* LPI 8192: priority, RES1, enabled */
	FlushPoC_DcacheRegion((vm_offset_t)prop, 64);
	if (!wr64(frame, GICR_PROPBASER, prop_pa | RDIST_SHARE_INNER | RDIST_INNER_WB | (LPI_ID_BITS - 1), "GICR_PROPBASER") ||
	    !wr64(frame, GICR_PENDBASER, pend_pa | RDIST_SHARE_INNER | RDIST_INNER_WB | (1ULL << 62), "GICR_PENDBASER")) {
		return "redistributor table write refused";
	}
	(void)rd64(frame, GICR_PROPBASER, &x, "GICR_PROPBASER");
	say("its: GICR_PROPBASER now %#llx (shareability %llu)\n", x, (x >> 10) & 3);
	if (!rd32(frame, GICR_CTLR, &v, "GICR_CTLR") || !wr32(frame, GICR_CTLR, v | GICR_CTLR_ENABLE_LPIS, "GICR_CTLR")) {
		return "GICR_CTLR refused";
	}
	__builtin_arm_dsb(0xf);
	(void)rd32(frame, GICR_CTLR, &v, "GICR_CTLR");
	say("its: this CPU's GICR_CTLR now %#x\n", v);
	if (!(v & GICR_CTLR_ENABLE_LPIS)) {
		return "EnableLPIs did not stay on";
	}

	/* the ITS's tables: device (type 1) and, without room in the ITS itself, collections (type 4) */
	for (unsigned n = 0; n < 8; n++) {
		uint64_t b = 0, t_pa;
		if (!rd64(gits, GITS_BASER(n), &b, "GITS_BASER")) {
			return "GITS_BASER unreadable";
		}
		unsigned type = (unsigned)BASER_TYPE(b);
		vm_size_t bytes;
		if (type == 1) {
			bytes = (vm_size_t)(RADIO_DEVICE_ID + 1) * BASER_ESIZE(b);
		} else if (type == 4 && hcc == 0) {
			bytes = 4096;
		} else {
			continue;
		}
		bytes = (bytes + 0xfff) & ~(vm_size_t)0xfff;
		if (bytes > 256 * 4096) {
			return "the device table would need the two-level form";
		}
		if (!table(bytes, 0x10000, &t_pa)) {
			return "no memory for an ITS table";
		}
		uint64_t w = BASER_VALID | BASER_INNER_WB | (b & (7ULL << 56)) | BASER_SHARE_INNER | t_pa | (bytes / 4096 - 1);
		if (!wr64(gits, GITS_BASER(n), w, "GITS_BASER")) {
			return "GITS_BASER refused";
		}
		(void)rd64(gits, GITS_BASER(n), &x, "GITS_BASER");
		say("its: GITS_BASER%u type %u: wrote %#llx, reads %#llx\n", n, type, w, x);
		if (!(x >> 63) || (x & 3ULL << 8) != 0) {
			return "an ITS table was not taken as written (valid or page size)";
		}
	}
	if (!wr64(gits, GITS_CBASER, BASER_VALID | BASER_INNER_WB | BASER_SHARE_INNER | queue_pa, "GITS_CBASER") ||
	    !wr64(gits, GITS_CWRITER, 0, "GITS_CWRITER")) {
		return "GITS_CBASER refused";
	}
	queue_next = 0;
	(void)rd64(gits, GITS_CBASER, &x, "GITS_CBASER");
	say("its: GITS_CBASER now %#llx\n", x);
	if (!rd32(gits, GITS_CTLR, &v, "GITS_CTLR") || !wr32(gits, GITS_CTLR, v | GITS_CTLR_ENABLED, "GITS_CTLR")) {
		return "GITS_CTLR refused";
	}
	(void)rd32(gits, GITS_CTLR, &v, "GITS_CTLR");
	say("its: GITS_CTLR now %#x\n", v);

	uint64_t rdbase = pta ? (frame_pa >> 16) : ((frame_typer >> 8) & 0xffff);
	uint64_t dev = (uint64_t)RADIO_DEVICE_ID << 32;
	const char *result = NULL;
	if (!command("MAPC", 0x09, 0, (1ULL << 63) | (rdbase << 16) | 0) ||
	    !command("SYNC", 0x05, 0, rdbase << 16) ||
	    !command("MAPD", 0x08 | dev, ITT_EVENT_BITS - 1, (1ULL << 63) | itt_pa) ||
	    !command("MAPTI", 0x0a | dev, 0 | ((uint64_t)LPI_FIRST << 32), 0) ||
	    !command("INV", 0x0c | dev, 0, 0) ||
	    !command("SYNC", 0x05, 0, rdbase << 16)) {
		result = "an ITS command failed";
	}

	if (result == NULL) {
		boolean_t was = ml_set_interrupts_enabled(FALSE);
		uint64_t start = micros(), seen = 0, iar = 0;
		bool ok = command("INT", 0x03 | dev, 0, 0);
		while (ok && micros() - start < 10000) {
			uint64_t h = __builtin_arm_rsr64("S3_0_C12_C12_2");     /* ICC_HPPIR1_EL1 */
			if ((h & 0xffffff) == LPI_FIRST) {
				seen = micros() - start;
				iar = __builtin_arm_rsr64("ICC_IAR1_EL1");
				__builtin_arm_wsr64("ICC_EOIR1_EL1", iar);
				__builtin_arm_isb(0xf);
				break;
			}
		}
		ml_set_interrupts_enabled(was);
		if (!ok) {
			result = "the INT command failed";
		} else if (seen == 0 && iar == 0) {
			result = "no LPI 8192 pending at this CPU within 10 ms";
		} else if ((iar & 0xffffff) != LPI_FIRST) {
			say("its: ICC_HPPIR1 showed 8192 but ICC_IAR1 gave %llu (ended)\n", iar & 0xffffff);
			result = "the acknowledged interrupt was another";
		} else {
			*latency_us = seen;
		}
	}

	/* undo: the mapping, the ITS off, this CPU's LPIs off where the GIC allows it */
	(void)command("DISCARD", 0x0f | dev, 0, 0);
	(void)command("MAPD", 0x08 | dev, 0, 0);
	/* no MAPC with V=0: Gunyah's ITS stalls its queue on it (p1a-its2, CREADR 0x121 at that command); Linux never unmaps one */
	(void)command("SYNC", 0x05, 0, rdbase << 16);
	if (rd32(gits, GITS_CTLR, &v, "GITS_CTLR")) {
		(void)wr32(gits, GITS_CTLR, v & ~GITS_CTLR_ENABLED, "GITS_CTLR");
	}
	uint64_t start = micros();
	while (rd32(gits, GITS_CTLR, &v, "GITS_CTLR") && !(v & GITS_CTLR_QUIESCENT) && micros() - start < 10000) {
	}
	say("its: ITS off: GITS_CTLR %#x\n", v);
	if (rd32(frame, GICR_CTLR, &v, "GICR_CTLR")) {
		(void)wr32(frame, GICR_CTLR, v & ~GICR_CTLR_ENABLE_LPIS, "GICR_CTLR");
		start = micros();
		while (rd32(frame, GICR_CTLR, &v, "GICR_CTLR") && (v & GICR_CTLR_RWP) && micros() - start < 10000) {
		}
		say("its: this CPU's LPIs off asked: GICR_CTLR %#x (%s)\n", v,
		    v & GICR_CTLR_ENABLE_LPIS ? "EnableLPIs stays 1: the tables stay allocated" : "off");
	}
	return result;
}

static void
probe_thread(void *arg, wait_result_t result)
{
	unsigned mode = (unsigned)(uintptr_t)arg;
	(void)result;
	assert_wait_timeout((event_t)&gits, THREAD_UNINT, 1000, NSEC_PER_MSEC);
	(void)thread_block(THREAD_CONTINUE_NULL);

	gicd = ml_io_map_unmappable(GICD_PA, GICD_SIZE, VM_WIMG_IO);
	gicr = ml_io_map_unmappable(GICR_PA, GICR_SIZE, VM_WIMG_IO);
	gits = ml_io_map_unmappable(GITS_PA, GITS_SIZE, VM_WIMG_IO);
	if (!gicd || !gicr || !gits) {
		say("its: RESULT not run: could not map the GIC\n");
		goto done;
	}
	part1();

	/* the rest on one CPU: its redistributor is the one written */
	thread_bind(current_processor());
	thread_block(THREAD_CONTINUE_NULL);
	uint64_t frame_typer = 0, scratch_pa = 0, latency = 0;
	vm_offset_t frame_pa = 0, frame = my_frame(&frame_typer, &frame_pa);
	bool rd_ok = false, its_ok = false;
	if (frame == 0) {
		say("its: RESULT not run: no redistributor frame for this CPU (cpu %d)\n", cpu_number());
		goto unbind;
	}
	say("its: on cpu %d, redistributor %#lx\n", cpu_number(), (unsigned long)frame_pa);
	if (!(frame_typer & GICR_TYPER_PLPIS)) {
		say("its: RESULT no LPIs: GICR_TYPER.PLPIS is 0; P1a falls back to the SPI doorbell or the timer\n");
		goto unbind;
	}
	if (!table(0x10000, 0x10000, &scratch_pa)) {
		say("its: RESULT not run: no memory\n");
		goto unbind;
	}
	bool writable = part2(frame, scratch_pa, &rd_ok, &its_ok);
	if (!writable || mode == 2) {
		say("its: RESULT part 2: redistributor LPI registers %s, ITS tables %s, %d refused accesses%s\n",
		    rd_ok ? "writable" : "NOT writable", its_ok ? "writable" : "NOT writable", failures,
		    mode == 2 ? " (its_probe=2: no LPI switched on)" : "; no LPI test");
		goto unbind;
	}
	const char *why = part3(frame, frame_pa, frame_typer, &latency);
	if (why == NULL) {
		say("its: RESULT LPI works: DeviceID %#x event 0 -> LPI %u reached cpu %d in %llu us through the ITS; %d refused accesses\n",
		    RADIO_DEVICE_ID, LPI_FIRST, cpu_number(), latency, failures);
	} else {
		say("its: RESULT LPI test failed: %s; %d refused accesses\n", why, failures);
	}
unbind:
	thread_bind(PROCESSOR_NULL);
done:
	thread_terminate(current_thread());
}

static void
its_init(void)
{
	unsigned mode = 0;
	thread_t thread;
	if (!PE_parse_boot_argn("its_probe", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	if (mode > 2 || !hakios_mmio_try_read32 || !hakios_mmio_try_write32 || !hakios_mmio_try_read64 || !hakios_mmio_try_write64) {
		say("its: its_probe=%u not run (1 or 2; needs the guarded MMIO accessors)\n", mode);
		return;
	}
	if (kernel_thread_start(probe_thread, (void *)(uintptr_t)mode, &thread) != KERN_SUCCESS) {
		say("its: no thread\n");
		return;
	}
	thread_deallocate(thread);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, its_init);
