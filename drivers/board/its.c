/*
 * The GIC's ITS: message-signalled interrupts as LPIs (C9, sys/board/its.h; WIFI-PLAN.md 7.3, P1a). SM8450 under Gunyah.
 *
 * A PCIe device raises an interrupt by writing an event number to GITS_TRANSLATER; the ITS looks the device (its DeviceID,
 * which the SMMU/PCIe side gives the write) and the event up in tables in RAM and makes an LPI of it, which a redistributor
 * hands to its processor like any other interrupt (gic_v3_irq.c calls hakios_its_lpi() for every INTID from 8192 up).
 *
 * What the phone showed (its_probe=1, runs p1a-its and p1a-its2, 2026-11-13): the GIC is Gunyah's virtual one (IIDR
 * 0x47000070, implementer 0x070), GICD_TYPER LPIS 1 IDbits 14; every redistributor PLPIS 1 and EnableLPIs 0 at start;
 * GITS_TYPER 0x150003e731 (ITT entry 4 bytes, 8 event bits, 32 DeviceID bits, PTA 0: a collection names a processor
 * number, HCC 0: the collection table is in RAM); BASER0 the device table (8-byte entries), BASER1 the collections, flat,
 * 4 KiB pages. Its 64-bit registers take only whole 64-bit accesses (32-bit halves fault or are dropped). MAPC and MAPD with V=0
 * stall the command queue, so neither a collection nor a device is ever unmapped; a redistributor's EnableLPIs cannot be cleared again,
 * so the tables live for the rest of the boot. An ITS command INT for DeviceID 0x5981 reached CPU 7 in 6 us.
 *
 * What this keeps: LPIs on for processor 0 only (one collection, ICID 0): every LPI goes to it. 256 LPIs (8192..8447),
 * handed out to at most 4 devices; a device keeps its LPIs and ITT across off/on. Nothing is touched until a driver asks
 * (hakios_its_device()) or its_probe=1 runs the self-test.
 *
 * Boot-args:
 *   its_probe=1        a second after start: the GIC's registers said, then the self-test (DeviceID 1, one event,
 *                      the ITS's own INT command, the handler must run) -> "its: RESULT ..."
 *   its_probe=2        the registers only
 */
#include <kern/clock.h>
#include <kern/cpu_number.h>
#include <kern/locks.h>
#include <kern/startup.h>
#include <kern/processor.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <machine/machine_routines.h>
#include <os/atomic_private.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/board/its.h>
#include <sys/systm.h>

#include "board.h"
#include <vm/memory_types.h>
#include <vm/vm_kern.h>
#include <vm/vm_kern_xnu.h>  /* xnu-12377: the KMA flags and kmem_alloc moved to the internal header */

extern processor_t thread_bind(processor_t processor);   /* (osfmk's, not in the headers BSD code sees) */

extern vm_offset_t ml_io_map_unmappable(vm_offset_t, vm_size_t, unsigned int);
extern uint64_t kvtophys(vm_offset_t va);
extern void FlushPoC_DcacheRegion(vm_offset_t va, size_t length);
extern int hakios_mmio_try_read32(const volatile void *, unsigned *) __attribute__((weak));
extern int hakios_mmio_try_write32(volatile void *, unsigned) __attribute__((weak));
extern int hakios_mmio_try_read64(const volatile void *, unsigned long long *) __attribute__((weak));
extern int hakios_mmio_try_write64(volatile void *, unsigned long long) __attribute__((weak));
extern kern_return_t thread_terminate(thread_t);
extern void loader_log_putc(char) __attribute__((weak));
bool hakios_its_lpi(uint32_t intid);

/* where the GIC is: the SoC file's platform { gic-dist gic-redist gic-redist-size gic-redist-stride gic-its } (SM8450: the
 * live tree's interrupt-controller@17100000, msi-controller@17140000, redistributor-stride 0x40000) */
#define GICD_PA                 ((unsigned long)hakios_board()->gic.dist)
#define GICD_SIZE               0x10000UL
#define GICR_PA                 ((unsigned long)hakios_board()->gic.redist)
#define GICR_SIZE               ((unsigned long)(hakios_board()->gic.redist_size ? hakios_board()->gic.redist_size : 0x100000))   /* (1 MB if not said, as the loader takes it) */
#define GICR_STRIDE             ((unsigned long)(hakios_board()->gic.redist_stride ? hakios_board()->gic.redist_stride : 0x20000))
#define GITS_PA                 ((unsigned long)hakios_board()->gic.its)
#define GITS_SIZE               0x20000UL
#define GITS_TRANSLATER_PA      (GITS_PA + 0x10040UL)

#define GICD_CTLR               0x0000
#define GICD_TYPER              0x0004
#define GICD_IIDR               0x0008
#define GICD_TYPER2             0x000c
#define GICR_CTLR               0x0000
#define GICR_IIDR               0x0004
#define GICR_TYPER              0x0008
#define GICR_PROPBASER          0x0070
#define GICR_PENDBASER          0x0078
#define GICR_CTLR_ENABLE_LPIS   (1u << 0)
#define GICR_TYPER_PLPIS        (1u << 0)
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

#define BASER_VALID             (1ULL << 63)
#define BASER_INNER_WB          (7ULL << 59)
#define BASER_SHARE_INNER       (1ULL << 10)
#define BASER_TYPE(v)           (((v) >> 56) & 7)
#define BASER_ESIZE(v)          ((((v) >> 48) & 0x1f) + 1)
#define RDIST_INNER_WB          (7ULL << 7)
#define RDIST_SHARE_INNER       (1ULL << 10)

#define LPI_FIRST               8192u
#define LPI_ID_BITS             14u             /* the property table covers 8192..16383 (8 KiB) */
#define LPIS_MAX                256u            /* handed out: 8192..8447 */
#define LPI_PRIORITY            0xa0u           /* gic_v3_irq.c's default */
#define DEVICE_IDS              0x10000u        /* the device table covers DeviceIDs below this (PCIe RIDs) */
#define DEVICES_MAX             4u
#define QUEUE_COMMANDS          128u            /* 4 KiB */

static vm_offset_t gicd, gicr, gits;
static vm_offset_t rdist;               /* processor 0's redistributor frame */

struct its_device {
	unsigned id, events, lpi_first;
	bool used, mapped, itt_mapped;   /* itt_mapped: MAPD done; never undone (Gunyah stalls on MAPD V=0) */
	vm_offset_t itt;
	uint64_t itt_pa;
	void (*handler)(void *context, unsigned event);
	void *context;
};
static struct its_device devices[DEVICES_MAX];
static struct {
	struct its_device *device;      /* NULL: nobody's */
	unsigned event;
	uint64_t count;
} lpis[LPIS_MAX];
static uint32_t lpi_busy;               /* handlers running now */
static unsigned lpi_next;               /* the next LPI to hand out, from LPI_FIRST */
static bool its_on;
static int its_error;                   /* why it cannot be switched on; 0: not tried or on */
static uint64_t rdbase;                 /* MAPC/SYNC target: processor 0's GIC processor number (PTA 0) */
static unsigned itt_entry;
static uint8_t *prop;
static uint64_t *queue;
static unsigned queue_next;
static LCK_GRP_DECLARE(its_group, "hakios-its");
static LCK_MTX_DECLARE(its_lock, &its_group);

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

static bool
rd32(vm_offset_t base, unsigned offset, uint32_t *value, const char *name)
{
	unsigned v = 0;
	int error = hakios_mmio_try_read32((const volatile void *)(base + offset), &v);
	if (error) {
		say("its: read of %s refused: errno %d\n", name, error);
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
		return false;
	}
	return true;
}

/* 64-bit registers: one whole access (Gunyah refuses the 32-bit halves) */
static bool
rd64(vm_offset_t base, unsigned offset, uint64_t *value, const char *name)
{
	unsigned long long v = 0;
	int error = hakios_mmio_try_read64((const volatile void *)(base + offset), &v);
	if (error) {
		say("its: read of %s refused: errno %d\n", name, error);
		return false;
	}
	*value = v;
	return true;
}

static bool
wr64(vm_offset_t base, unsigned offset, uint64_t value, const char *name)
{
	int error = hakios_mmio_try_write64((volatile void *)(base + offset), value);
	if (error) {
		say("its: write of %s refused: errno %d\n", name, error);
		return false;
	}
	return true;
}

static bool
mapped(void)
{
	if (GICD_PA == 0 || GICR_PA == 0 || GITS_PA == 0) {
		return false;           /* (the SoC file names no ITS: the radio's interrupts are polled) */
	}
	if (!gicd) {
		gicd = ml_io_map_unmappable(GICD_PA, GICD_SIZE, VM_WIMG_IO);
		gicr = ml_io_map_unmappable(GICR_PA, GICR_SIZE, VM_WIMG_IO);
		gits = ml_io_map_unmappable(GITS_PA, GITS_SIZE, VM_WIMG_IO);
	}
	return gicd && gicr && gits;
}

static uint64_t
micros(void)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000;
}

/* Contiguous, zeroed, physically aligned to `align`; never freed (EnableLPIs stays on). */
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

/* One command, and its completion (its_lock held). */
static bool
command(const char *what, uint64_t d0, uint64_t d1, uint64_t d2)
{
	uint64_t *c = queue + 4 * queue_next;
	c[0] = d0; c[1] = d1; c[2] = d2; c[3] = 0;
	FlushPoC_DcacheRegion((vm_offset_t)c, 32);
	__builtin_arm_dsb(0xf);
	queue_next = (queue_next + 1) % QUEUE_COMMANDS;
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
			its_error = EIO;
			return false;
		}
		if ((r & 0xfffe0) == (uint64_t)queue_next * 32) {
			return true;
		}
		if (micros() - start > 10000) {
			say("its: command %s not taken in 10 ms (CREADR %#llx, CWRITER %#x)\n", what, r, queue_next * 32);
			its_error = EIO;
			return false;
		}
	}
}

#define CMD_SYNC()              command("SYNC", 0x05, 0, rdbase << 16)

/* Processor 0's LPIs and the ITS on, run on processor 0 (its_up). 0, or an errno (kept: not tried again). */
static int
its_up_here(void)
{
	uint64_t typer = 0, gtyper = 0, x = 0, pa;
	uint32_t v = 0;
	if (its_on) {
		return 0;
	}
	if (its_error) {
		return its_error;
	}
	its_error = ENXIO;
	if (!hakios_mmio_try_read64 || !hakios_mmio_try_write64 || !mapped()) {
		say("its: not switched on: no guarded 64-bit MMIO or no mapping\n");
		return its_error;
	}
	/* this processor's (0's) redistributor: the frame whose GICR_TYPER[63:32] is its affinity */
	uint64_t mpidr = __builtin_arm_rsr64("MPIDR_EL1");
	uint32_t affinity = (uint32_t)(((mpidr >> 32) & 0xff) << 24 | (mpidr & 0xffffff));
	rdist = 0;
	for (vm_offset_t off = 0; off + GICR_STRIDE <= GICR_SIZE && rdist == 0; off += GICR_STRIDE) {
		if (!rd64(gicr + off, GICR_TYPER, &typer, "GICR_TYPER")) {
			return its_error;
		}
		if ((uint32_t)(typer >> 32) == affinity) {
			rdist = gicr + off;
		} else if (typer & GICR_TYPER_LAST) {
			break;
		}
	}
	if (rdist == 0) {
		say("its: not switched on: no redistributor for affinity %#x (cpu %d)\n", affinity, cpu_number());
		return its_error;
	}
	if (!rd64(rdist, GICR_TYPER, &typer, "GICR_TYPER") || !rd32(rdist, GICR_CTLR, &v, "GICR_CTLR") ||
	    !rd64(gits, GITS_TYPER, &gtyper, "GITS_TYPER")) {
		return its_error;
	}
	if (!(typer & GICR_TYPER_PLPIS)) {
		say("its: not switched on: processor 0's redistributor has no LPIs (GICR_TYPER %#llx)\n", typer);
		return its_error;
	}
	if (v & GICR_CTLR_ENABLE_LPIS) {
		say("its: not switched on: processor 0's LPIs were left on (GICR_CTLR %#x) by an earlier kernel, with its tables\n", v);
		return its_error;
	}
	if (!rd32(gits, GITS_CTLR, &v, "GITS_CTLR") || (v & GITS_CTLR_ENABLED)) {
		say("its: not switched on: the ITS is already on (GITS_CTLR %#x)\n", v);
		return its_error;
	}
	if (((gtyper >> 13) & 0x1f) + 1 < 16 || ((gtyper >> 19) & 1)) {
		say("its: not switched on: GITS_TYPER %#llx (fewer than 16 DeviceID bits, or PTA 1)\n", gtyper);
		return its_error;
	}
	itt_entry = (unsigned)((gtyper >> 4) & 0xf) + 1;
	unsigned hcc = (unsigned)((gtyper >> 24) & 0xff);
	rdbase = (typer >> 8) & 0xffff;

	uint64_t prop_pa = 0, pend_pa = 0, queue_pa = 0;
	prop = (uint8_t *)table((1u << LPI_ID_BITS) - LPI_FIRST, PAGE_SIZE, &prop_pa);
	vm_offset_t pend = table(0x10000, 0x10000, &pend_pa);
	queue = (uint64_t *)table(4096, 0x10000, &queue_pa);
	if (!prop || !pend || !queue) {
		say("its: not switched on: no memory for its tables\n");
		its_error = ENOMEM;
		return its_error;
	}
	if (!wr64(rdist, GICR_PROPBASER, prop_pa | RDIST_SHARE_INNER | RDIST_INNER_WB | (LPI_ID_BITS - 1), "GICR_PROPBASER") ||
	    !wr64(rdist, GICR_PENDBASER, pend_pa | RDIST_SHARE_INNER | RDIST_INNER_WB | (1ULL << 62), "GICR_PENDBASER") ||
	    !rd64(rdist, GICR_PROPBASER, &x, "GICR_PROPBASER") || (x & 0x000ffffffffff000ULL) != prop_pa) {
		say("its: not switched on: GICR_PROPBASER reads %#llx\n", x);
		return its_error;
	}
	/* Set from another processor, Gunyah left it 0 (p1b-its: GICR_CTLR 0x4); from its own it took (p1a-its2). Waited for. */
	if (!rd32(rdist, GICR_CTLR, &v, "GICR_CTLR") || !wr32(rdist, GICR_CTLR, v | GICR_CTLR_ENABLE_LPIS, "GICR_CTLR")) {
		return its_error;
	}
	__builtin_arm_dsb(0xf);
	uint64_t since = micros();
	while (rd32(rdist, GICR_CTLR, &v, "GICR_CTLR") && !(v & GICR_CTLR_ENABLE_LPIS) && micros() - since < 10000) {
		delay(10);
	}
	if (!(v & GICR_CTLR_ENABLE_LPIS)) {
		say("its: not switched on: EnableLPIs does not stay on (GICR_CTLR %#x, cpu %d)\n", v, cpu_number());
		return its_error;
	}
	for (unsigned n = 0; n < 8; n++) {
		uint64_t b = 0;
		if (!rd64(gits, GITS_BASER(n), &b, "GITS_BASER")) {
			return its_error;
		}
		vm_size_t bytes;
		if (BASER_TYPE(b) == 1) {
			bytes = (vm_size_t)DEVICE_IDS * BASER_ESIZE(b);
		} else if (BASER_TYPE(b) == 4 && hcc == 0) {
			bytes = 4096;
		} else {
			continue;
		}
		bytes = (bytes + 0xfff) & ~(vm_size_t)0xfff;
		if (bytes > 256 * 4096 || !table(bytes, 0x10000, &pa)) {
			say("its: not switched on: no %lu bytes for ITS table %u\n", (unsigned long)bytes, n);
			return its_error;
		}
		uint64_t w = BASER_VALID | BASER_INNER_WB | (b & (7ULL << 56)) | BASER_SHARE_INNER | pa | (bytes / 4096 - 1);
		if (!wr64(gits, GITS_BASER(n), w, "GITS_BASER") || !rd64(gits, GITS_BASER(n), &x, "GITS_BASER") ||
		    !(x >> 63) || (x & 0x0000fffffffff000ULL) != pa || (x & (3ULL << 8)) != 0) {
			say("its: not switched on: GITS_BASER%u wrote %#llx, reads %#llx\n", n, w, x);
			return its_error;
		}
	}
	if (!wr64(gits, GITS_CBASER, BASER_VALID | BASER_INNER_WB | BASER_SHARE_INNER | queue_pa, "GITS_CBASER") ||
	    !wr64(gits, GITS_CWRITER, 0, "GITS_CWRITER") || !rd32(gits, GITS_CTLR, &v, "GITS_CTLR") ||
	    !wr32(gits, GITS_CTLR, v | GITS_CTLR_ENABLED, "GITS_CTLR")) {
		return its_error;
	}
	queue_next = 0;
	its_error = 0;
	if (!command("MAPC", 0x09, 0, BASER_VALID | (rdbase << 16) | 0) || !CMD_SYNC()) {
		return its_error ? its_error : (its_error = EIO);
	}
	its_on = true;
	say("its: on: processor 0's LPIs (GIC processor %llu), %u LPIs from %u, DeviceIDs below %#x, ITT entry %u bytes\n",
	    rdbase, LPIS_MAX, LPI_FIRST, DEVICE_IDS, itt_entry);
	return 0;
}

/* its_up_here() on processor 0 itself: Gunyah takes a redistributor's EnableLPIs only from its own processor. */
static int
its_up(void)
{
	if (its_on || its_error) {
		return its_on ? 0 : its_error;
	}
	processor_t was = thread_bind(cpu_to_processor(0));
	thread_block(THREAD_CONTINUE_NULL);
	int error;
	if (cpu_number() == 0) {
		error = its_up_here();
	} else {
		say("its: not switched on: could not run on processor 0 (on %d)\n", cpu_number());
		error = ENXIO;
	}
	if (error && !its_error) {
		its_error = error;
	}
	thread_bind(was);
	thread_block(THREAD_CONTINUE_NULL);
	return error;
}

int
hakios_its_device(unsigned device_id, unsigned events, void (*handler)(void *context, unsigned event), void *context,
    unsigned long long *doorbell)
{
	struct its_device *d = NULL;
	int error;
	if (device_id >= DEVICE_IDS || events == 0 || events > 256 || (events & (events - 1)) || handler == NULL || doorbell == NULL) {
		return EINVAL;
	}
	lck_mtx_lock(&its_lock);
	if ((error = its_up()) != 0) {
		goto out;
	}
	for (unsigned i = 0; i < DEVICES_MAX && d == NULL; i++) {
		if (devices[i].used && devices[i].id == device_id) {
			d = &devices[i];
		}
	}
	if (d && (d->mapped || d->events < events)) {
		error = EBUSY;
		goto out;
	}
	if (d == NULL) {
		for (unsigned i = 0; i < DEVICES_MAX && d == NULL; i++) {
			if (!devices[i].used) {
				d = &devices[i];
			}
		}
		if (d == NULL || lpi_next + events > LPIS_MAX) {
			error = ENOSPC;
			goto out;
		}
		bzero(d, sizeof(*d));
		if (!(d->itt = table(events * itt_entry, 256, &d->itt_pa))) {
			error = ENOMEM;
			goto out;
		}
		d->used = true;
		d->id = device_id;
		d->events = events;
		d->lpi_first = lpi_next;
		lpi_next += events;
	}
	d->handler = handler;
	d->context = context;
	for (unsigned e = 0; e < events; e++) {
		lpis[d->lpi_first + e].event = e;
		os_atomic_store(&lpis[d->lpi_first + e].device, d, release);
		prop[d->lpi_first + e] = LPI_PRIORITY | 2 | 1;
	}
	FlushPoC_DcacheRegion((vm_offset_t)prop, PAGE_SIZE);
	__builtin_arm_dsb(0xf);
	uint64_t dev = (uint64_t)device_id << 32;
	unsigned bits = 0;
	while ((1u << bits) < events) {
		bits++;
	}
	if (!d->itt_mapped && !command("MAPD", 0x08 | dev, bits ? bits - 1 : 0, BASER_VALID | d->itt_pa)) {
		error = EIO;
		goto out;
	}
	d->itt_mapped = true;
	for (unsigned e = 0; e < events; e++) {
		uint64_t lpi = LPI_FIRST + d->lpi_first + e;
		if (!command("MAPTI", 0x0a | dev, e | (lpi << 32), 0) || !command("INV", 0x0c | dev, e, 0)) {
			error = EIO;
			goto out;
		}
	}
	if (!CMD_SYNC()) {
		error = EIO;
		goto out;
	}
	d->mapped = true;
	*doorbell = GITS_TRANSLATER_PA;
	say("its: device %#x: %u events -> LPIs %u..%u, doorbell %#lx\n", device_id, events, LPI_FIRST + d->lpi_first,
	    LPI_FIRST + d->lpi_first + events - 1, (unsigned long)GITS_TRANSLATER_PA);
out:
	lck_mtx_unlock(&its_lock);
	return error;
}

void
hakios_its_device_off(unsigned device_id)
{
	lck_mtx_lock(&its_lock);
	for (unsigned i = 0; i < DEVICES_MAX; i++) {
		struct its_device *d = &devices[i];
		if (!d->used || d->id != device_id || !d->mapped) {
			continue;
		}
		uint64_t dev = (uint64_t)device_id << 32;
		for (unsigned e = 0; e < d->events; e++) {
			(void)command("DISCARD", 0x0f | dev, e, 0);
			prop[d->lpi_first + e] = LPI_PRIORITY | 2;
		}
		/* no MAPD with V=0: Gunyah's ITS stalls its queue on it (p1b-its3), as on MAPC V=0 (p1a-its2); the device stays
		 * mapped to its ITT with no event left, and a later hakios_its_device() maps the events again */
		(void)CMD_SYNC();
		FlushPoC_DcacheRegion((vm_offset_t)prop, PAGE_SIZE);
		for (unsigned e = 0; e < d->events; e++) {
			os_atomic_store(&lpis[d->lpi_first + e].device, NULL, release);
		}
		d->mapped = false;
	}
	lck_mtx_unlock(&its_lock);
	while (os_atomic_load(&lpi_busy, acquire) != 0) {
		delay(10);
	}
}

unsigned long long
hakios_its_count(unsigned device_id, unsigned event)
{
	for (unsigned i = 0; i < DEVICES_MAX; i++) {
		if (devices[i].used && devices[i].id == device_id && event < devices[i].events) {
			return lpis[devices[i].lpi_first + event].count;
		}
	}
	return 0;
}

/*
 * System sleep (psci_idle.c's hakios_sleep_wait(), before its interrupts go off and after they are on again): every
 * mapped device's LPIs disabled in the property table (an INV each), or enabled again. A disabled LPI that comes is kept
 * pending and taken once it is enabled: nothing is lost, and the radio's MSIs no longer end the sleep's WFI at once
 * (run-20261126-040608: every test sleep woken by LPI 8205/8213/8214 within 23..2706 ms). 0, or EIO.
 */
int hakios_its_sleep_mask(bool masked);
int
hakios_its_sleep_mask(bool masked)
{
	int error = 0;

	lck_mtx_lock(&its_lock);
	for (unsigned i = 0; i < DEVICES_MAX; i++) {
		struct its_device *d = &devices[i];
		if (!d->used || !d->mapped) {
			continue;
		}
		for (unsigned e = 0; e < d->events; e++) {
			prop[d->lpi_first + e] = LPI_PRIORITY | 2 | (masked ? 0 : 1);
		}
	}
	FlushPoC_DcacheRegion((vm_offset_t)prop, PAGE_SIZE);
	__builtin_arm_dsb(0xf);
	for (unsigned i = 0; i < DEVICES_MAX; i++) {
		struct its_device *d = &devices[i];
		if (!d->used || !d->mapped) {
			continue;
		}
		uint64_t dev = (uint64_t)d->id << 32;
		for (unsigned e = 0; e < d->events; e++) {
			if (!command("INV", 0x0c | dev, e, 0)) {
				error = EIO;
			}
		}
	}
	if (!CMD_SYNC()) {
		error = EIO;
	}
	lck_mtx_unlock(&its_lock);
	return error;
}

/* From gic_v3_irq.c, in interrupt context, for every INTID >= 8192: true when it was a device's. */
bool
hakios_its_lpi(uint32_t intid)
{
	unsigned n = intid - LPI_FIRST;
	if (intid < LPI_FIRST || n >= LPIS_MAX) {
		return false;
	}
	os_atomic_inc(&lpi_busy, acquire);
	struct its_device *d = os_atomic_load(&lpis[n].device, acquire);
	if (d != NULL) {
		lpis[n].count++;
		d->handler(d->context, lpis[n].event);
	}
	os_atomic_dec(&lpi_busy, release);
	return d != NULL;
}

/* ---- its_probe ---- */

static void
registers(void)
{
	uint32_t ctlr = 0, typer = 0, iidr = 0, typer2 = 0;
	(void)rd32(gicd, GICD_CTLR, &ctlr, "GICD_CTLR");
	(void)rd32(gicd, GICD_TYPER, &typer, "GICD_TYPER");
	(void)rd32(gicd, GICD_IIDR, &iidr, "GICD_IIDR");
	(void)rd32(gicd, GICD_TYPER2, &typer2, "GICD_TYPER2");
	say("its: GICD ctlr %#x typer %#x (LPIS %u, MBIS %u, IDbits %u) typer2 %#x iidr %#x\n", ctlr, typer, (typer >> 17) & 1,
	    (typer >> 16) & 1, ((typer >> 19) & 0x1f) + 1, typer2, iidr);
	for (vm_offset_t off = 0; off + GICR_STRIDE <= GICR_SIZE; off += GICR_STRIDE) {
		uint64_t rt = 0, prop_r = 0, pend_r = 0;
		uint32_t rc = 0;
		if (!rd64(gicr + off, GICR_TYPER, &rt, "GICR_TYPER")) {
			break;
		}
		(void)rd32(gicr + off, GICR_CTLR, &rc, "GICR_CTLR");
		(void)rd64(gicr + off, GICR_PROPBASER, &prop_r, "GICR_PROPBASER");
		(void)rd64(gicr + off, GICR_PENDBASER, &pend_r, "GICR_PENDBASER");
		say("its: GICR %#lx: affinity %#x processor %u PLPIS %u ctlr %#x propbaser %#llx pendbaser %#llx\n", GICR_PA + off,
		    (uint32_t)(rt >> 32), (uint32_t)((rt >> 8) & 0xffff), (unsigned)(rt & GICR_TYPER_PLPIS), rc, prop_r, pend_r);
		if (rt & GICR_TYPER_LAST) {
			break;
		}
	}
	uint32_t gctlr = 0, giidr = 0;
	uint64_t gtyper = 0, cbaser = 0;
	(void)rd32(gits, GITS_CTLR, &gctlr, "GITS_CTLR");
	(void)rd32(gits, GITS_IIDR, &giidr, "GITS_IIDR");
	(void)rd64(gits, GITS_TYPER, &gtyper, "GITS_TYPER");
	(void)rd64(gits, GITS_CBASER, &cbaser, "GITS_CBASER");
	say("its: GITS ctlr %#x iidr %#x typer %#llx cbaser %#llx\n", gctlr, giidr, gtyper, cbaser);
	for (unsigned n = 0; n < 8; n++) {
		uint64_t b = 0;
		if (rd64(gits, GITS_BASER(n), &b, "GITS_BASER") && b) {
			say("its: GITS_BASER%u %#llx\n", n, b);
		}
	}
}

static uint64_t selftest_at;

static void
selftest_handler(__unused void *context, unsigned event)
{
	if (event == 0 && selftest_at == 0) {
		selftest_at = micros();
	}
}

#define SELFTEST_DEVICE         1u

static void
selftest(void)
{
	unsigned long long doorbell = 0;
	int error = hakios_its_device(SELFTEST_DEVICE, 1, selftest_handler, NULL, &doorbell);
	if (error) {
		say("its: RESULT self-test: the device was not mapped: errno %d\n", error);
		return;
	}
	lck_mtx_lock(&its_lock);
	uint64_t start = micros();
	bool sent = command("INT", 0x03 | ((uint64_t)SELFTEST_DEVICE << 32), 0, 0);
	lck_mtx_unlock(&its_lock);
	while (sent && selftest_at == 0 && micros() - start < 20000) {
		delay(10);
	}
	uint64_t count = hakios_its_count(SELFTEST_DEVICE, 0);
	hakios_its_device_off(SELFTEST_DEVICE);
	if (!sent) {
		say("its: RESULT self-test: the INT command failed\n");
	} else if (selftest_at == 0) {
		say("its: RESULT self-test: no LPI in 20 ms (count %llu)\n", count);
	} else {
		say("its: RESULT self-test ok: INT -> its LPI -> handler on processor 0 in %llu us (count %llu); command queue %s\n",
		    selftest_at - start, count, its_error ? "STALLED" : "fine after the switch-off");
	}
}

static void
probe_thread(void *arg, wait_result_t result)
{
	unsigned mode = (unsigned)(uintptr_t)arg;
	(void)result;
	assert_wait_timeout((event_t)&gits, THREAD_UNINT, 1000, NSEC_PER_MSEC);
	(void)thread_block(THREAD_CONTINUE_NULL);
	if (!mapped()) {
		say("its: RESULT not run: could not map the GIC\n");
	} else {
		registers();
		if (mode == 1) {
			selftest();
		}
	}
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
