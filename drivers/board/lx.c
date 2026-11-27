/*
 * HakiOS: the kernel's side of the Wi-Fi driver's object.
 *
 * The driver of the phone's radio is Linux's ath11k, compiled unmodified outside this tree on a
 * layer that gives it the kernel interfaces it expects, and tied into one object that the
 * kernel links when it is there (kernel/lx/build.sh and kernel/lx in the repository; the kernel
 * is built with that object when xnu-build.sh finds it). The object shows one name, lx_start(),
 * and asks for the calls of sys/board/lx.h, which are here: memory, time, threads, one lock
 * to sleep under, the kernel's options. What it asks of the radio itself -- its firmware's
 * files, its view of memory, the processor its firmware runs on -- is board/wifi.c's to
 * answer, which knows the phone.
 *
 * A kernel built without the object has the lx_start() at the end of this file, which says so.
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <os/atomic_private.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/random.h>
#include <sys/board/input.h>
#include <sys/board/its.h>
#include <sys/board/lx.h>
#include <sys/board/net.h>
#include <sys/board/smmu_ctx.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <vm/memory_types.h>
#include <vm/vm_kern.h>
#include <vm/vm_kern_xnu.h>  /* xnu-12377: the KMA flags and kmem_alloc moved to the internal header */
#include <net/kpi_interface.h>
#include <netinet/in.h>

#include "kernel.h"
#include "cache.h"
#include "board.h"

/* board/wifi_pci.c: the PCIe path (the board's wifi-qca6490) of the Wi-Fi; declared where they are used, as board/wifi.c does */
extern bool hakios_wifi_pci_selected(void);
extern void hakios_wifi_pci_report(unsigned n, int ok, const char *detail);
extern unsigned hakios_wifi_pci_seen(unsigned n);
extern int hakios_wifi_pci_firmware(const char *name, const uint8_t **data, uint64_t *size);
/* board/qrtr.c: the router's own calls that have no header */
extern void hakios_qrtr_poll(void);
extern void hakios_qrtr_wait(uint32_t ms);
extern void hakios_qrtr_wake(void);
extern bool hakios_screen_off(void);
extern void hakios_qrtr_stats(uint32_t *peer, uint32_t *hellos, uint32_t *in, uint32_t *out, uint32_t *services);
extern const struct hakios_qrtr_ops hakios_qrtr_router_ops;

#define say hakios_wifi_say

static LCK_GRP_DECLARE(lx_lock_group, "hakios-lx");
static LCK_MTX_DECLARE(lx_lock, &lx_lock_group);
static int pcie_runtime_phase, pcie_runtime_error;
SYSCTL_DECL(_kern_hakios);
static int lx_thread_slots_used, lx_thread_slots_high;
void lx_os_thread_slots(unsigned used, unsigned high)
{
	lx_thread_slots_used = used; lx_thread_slots_high = high;
}
SYSCTL_INT(_kern_hakios, OID_AUTO, lx_thread_slots_used, CTLFLAG_RD | CTLFLAG_LOCKED,
    &lx_thread_slots_used, 0, "Cached allocated LinuxKPI thread slots");
SYSCTL_INT(_kern_hakios, OID_AUTO, lx_thread_slots_high, CTLFLAG_RD | CTLFLAG_LOCKED,
    &lx_thread_slots_high, 0, "Maximum allocated LinuxKPI thread slots");
static unsigned mhi_state, mhi_wake, mhi_pending, mhi_m0, mhi_m2, mhi_errors, mhi_blocked;
void lx_os_mhi_power(unsigned state, unsigned wake, unsigned pending, unsigned m0, unsigned m2, unsigned errors, unsigned blocked)
{
	mhi_state = state; mhi_wake = wake; mhi_pending = pending;
	mhi_m0 = m0; mhi_m2 = m2; mhi_errors = errors; mhi_blocked = blocked;
}
static int sysctl_mhi_power SYSCTL_HANDLER_ARGS
{
	char s[192];
	snprintf(s, sizeof(s), "state %#x wake %u pending %u m0 %u m2 %u errors %u blocked %u",
	    mhi_state, mhi_wake, mhi_pending, mhi_m0, mhi_m2, mhi_errors, mhi_blocked);
	return SYSCTL_OUT(req, s, strlen(s) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, mhi_power, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    0, 0, sysctl_mhi_power, "A", "Cached MHI idle state and counters");
SYSCTL_INT(_kern_hakios, OID_AUTO, pcie_runtime_phase, CTLFLAG_RD | CTLFLAG_LOCKED,
    &pcie_runtime_phase, 0, "PCIe runtime: 0 off, 1 stopping, 2 probing, 3 firmware ready, 4 station ready, 5 failed");
SYSCTL_INT(_kern_hakios, OID_AUTO, pcie_runtime_error, CTLFLAG_RD | CTLFLAG_LOCKED,
    &pcie_runtime_error, 0, "Last runtime PCIe error");
void lx_os_runtime_state(int phase, int error)
{
	pcie_runtime_error = error;
	pcie_runtime_phase = phase;
	hakios_wifi_tell("lx: runtime PCIe phase %d error %d\n", phase, error);
}

/* ---- memory ------------------------------------------------------------------------------------------ */

void *
lx_os_alloc(unsigned long size)
{
	return kalloc_data(size, Z_WAITOK | Z_ZERO);
}

void
lx_os_free(void *memory, unsigned long size)
{
	kfree_data(memory, size);
}

/* A radio whose driver wants a large pool in one piece (ath11k on PCIe: 80 MiB, kernel/lx/src/lx_base.c)
 * has it kept from before userland fragments RAM: the board's wifi { } early_pool_mb says how much.
 * Only lx_os_radio_memory may map it to the radio, after PCIe and the SMMU have been initialized.
 * Boot lifetime, like the driver's pool itself. A failed reservation leaves the normal allocation
 * path available; no device or page table is touched here. */
static vm_offset_t lx_early_pool;
static unsigned long lx_early_pool_bytes;

static void
lx_reserve_radio_pool(void)
{
    const struct hakios_board *board = hakios_board();

    if (board == NULL || board->wifi.early_pool_mb == 0 || lx_early_pool != 0) return;
    lx_early_pool_bytes = (unsigned long)board->wifi.early_pool_mb << 20;
    if (kmem_alloc_contig(kernel_map, &lx_early_pool, lx_early_pool_bytes,
        0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
        lx_early_pool = 0;
        say("lx: early %u MiB radio pool reservation failed\n", board->wifi.early_pool_mb);
        return;
    }
    say("lx: reserved a %u MiB radio pool before userland; not mapped to the radio yet\n", board->wifi.early_pool_mb);
}

/*
 * In one physical piece, so that it can be mapped a second time without this processor's cache
 * in between (as board/adreno.c does with what it shares with the GPU); the first, cached
 * view is written out and never used again.
 */
void *
lx_os_radio_memory(unsigned long size, unsigned long long *address)
{
	vm_offset_t cached = 0, uncached;
	uint64_t physical;
	bool early = false;

	size = round_page(size);
    if (size <= lx_early_pool_bytes && lx_early_pool != 0) {
        /* (a pool as large as the board asked for, or larger: what the driver asks for fits in it) */
        cached = lx_early_pool;
        lx_early_pool = 0; /* exactly one consumer; the pool retains ownership */
        early = true;
        say("lx: using the early radio pool\n");
    } else if (kmem_alloc_contig(kernel_map, &cached, size, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		say("lx: no %lu bytes of memory in one piece\n", size);
		return NULL;
	}
	physical = kvtophys(cached);
	bzero((void *)cached, size);
	hakios_cache_flush(cached, cached + size);
	uncached = ml_io_map_unmappable(physical, size, VM_WIMG_WCOMB);
	if (uncached == 0 || !hakios_wifi_radio_map(physical, size, address)) {
		/* (given back as it was had: the memory in one piece, the early pool for the next try, the second view) */
		say("lx: the radio cannot be shown %lu bytes of ours: given back\n", size);
		if (uncached != 0) {
			ml_io_unmap(uncached, size);
		}
		if (early) {
			lx_early_pool = cached;
		} else {
			kmem_free(kernel_map, cached, size);
		}
		return NULL;
	}
	return (void *)uncached;
}

/*
 * The driver maps what the radio's firmware says its registers are (qmi.c: the device-info
 * reply), and reads and writes there as the radio's: only inside the window the radio's
 * registers can be in (board/wifi.c) is that let through.
 */
void *
lx_os_map_registers(unsigned long long physical, unsigned long size)
{
	unsigned long long page = trunc_page(physical);
	uint64_t window = 0, window_size = 0;
	vm_offset_t mapped;

	if (!hakios_wifi_radio_window(&window, &window_size) || size == 0 || physical < window || physical - window > window_size ||
	    size > window_size - (physical - window)) {
		say("lx: %#lx bytes at %#llx are not where the radio's registers can be: not mapped\n", size, physical);
		return NULL;
	}
	mapped = ml_io_map_unmappable(page, round_page(physical + size) - page, VM_WIMG_IO);
	return mapped == 0 ? NULL : (void *)(mapped + (physical - page));
}

/* ---- saying things ----------------------------------------------------------------------------------- */

void
lx_os_print(const char *text)
{
	say("%s", text);
}

void
lx_os_tell(const char *text)
{
	hakios_wifi_tell("%s", text);
}

void
lx_os_stage(unsigned n, const char *name, int ok, const char *fmt, ...)
{
	char detail[256];
	va_list args;

	va_start(args, fmt);
	vsnprintf(detail, sizeof(detail), fmt, args);
	va_end(args);
	if (hakios_wifi_pci_selected()) {
		/* the PCIe path: said once, and never before the stage it needs (board/wifi_pci.c); may wait a moment */
		hakios_wifi_pci_report(n, ok, detail);
		return;
	}
	hakios_wifi_stage(n, name, ok, "%s", detail);
}

void
lx_os_panic(const char *text)
{
	panic("%s", text);
}

/* ---- time, threads, sleeping --------------------------------------------------------------------------- */

unsigned long long
lx_os_nanoseconds(void)
{
	uint64_t nanoseconds;

	absolutetime_to_nanoseconds(mach_absolute_time(), &nanoseconds);
	return nanoseconds;
}

void *
lx_os_thread_self(void)
{
	return current_thread();
}

struct lx_thread {
	void (*function)(void *);
	void *argument;
};

static void
lx_thread_runs(void *parameter, __unused wait_result_t result)
{
	struct lx_thread start = *(struct lx_thread *)parameter;

	kfree_type(struct lx_thread, parameter);
	start.function(start.argument);
	lx_thread_release();
	thread_terminate(current_thread());
}

int
lx_os_thread(const char *name, void (*function)(void *), void *argument)
{
	struct lx_thread *start = kalloc_type(struct lx_thread, Z_WAITOK | Z_ZERO);
	thread_t thread;

	if (start == NULL) {
		return ENOMEM;
	}
	start->function = function;
	start->argument = argument;
	if (kernel_thread_start(lx_thread_runs, start, &thread) != KERN_SUCCESS) {
		kfree_type(struct lx_thread, start);
		return EAGAIN;
	}
	thread_set_thread_name(thread, name);
	thread_deallocate(thread);
	return 0;
}

void
lx_os_lock(void)
{
	lck_mtx_lock(&lx_lock);
}

void
lx_os_unlock(void)
{
	lck_mtx_unlock(&lx_lock);
}

int
lx_os_screen_off(void)
{
	return hakios_screen_off() ? 1 : 0;
}

int
lx_os_sleep(const void *address, unsigned long long nanoseconds)
{
	uint64_t deadline;

	if (nanoseconds == 0) {
		return lck_mtx_sleep(&lx_lock, LCK_SLEEP_DEFAULT, (event_t)address, THREAD_UNINT) == THREAD_TIMED_OUT;
	}
	nanoseconds_to_absolutetime(nanoseconds, &deadline);
	deadline += mach_absolute_time();
	return lck_mtx_sleep_deadline(&lx_lock, LCK_SLEEP_DEFAULT, (event_t)address, THREAD_UNINT, deadline) == THREAD_TIMED_OUT;
}

void
lx_os_wakeup(const void *address)
{
	thread_wakeup((event_t)address);
}

/* ---- the rest of the machine ------------------------------------------------------------------------- */

void
lx_os_random(void *buffer, unsigned long size)
{
	read_random(buffer, (u_int)size);
}

unsigned
lx_os_option(const char *name)
{
	return hakios_wifi_option(name);
}

unsigned
lx_os_radio_model(void)
{
	const struct hakios_board *board = hakios_board();

	return board != NULL ? board->wifi.radio_model : 0;     /* the board's wifi { } radio_model */
}

int
lx_os_firmware(const char *name, const void **data, unsigned long *size)
{
	uint64_t length = 0;
	const uint8_t *file = NULL;

	if (hakios_wifi_pci_selected()) {
		/* the PCIe path's table (C8): ENODEV is "board.bin and no wifi_board": the layer chooses by the board id and asks again */
		int error = hakios_wifi_pci_firmware(name, &file, &length);

		if (error != 0) {
			return error;
		}
		*data = file;
		*size = (unsigned long)length;
		return 0;
	}
	file = hakios_wifi_driver_file(name, &length);
	if (file == NULL) {
		return ENOENT;
	}
	*data = file;
	*size = (unsigned long)length;
	return 0;
}

int
lx_os_rproc_boot(void)
{
	return hakios_wifi_dsp_is_up() ? 0 : ENXIO;
}

int
lx_os_radio_address(unsigned char *address)
{
	return hakios_wifi_radio_address(address);
}

int
lx_os_network(unsigned char *name, unsigned *name_length, unsigned char *passphrase, unsigned *passphrase_length)
{
	return hakios_wifi_network(name, name_length, passphrase, passphrase_length);
}

/* ---- frames: the radio as the link behind en0 (sys/board/net.h) ------------------------------------ */

/*
 * A frame from the network stack comes on whatever thread sends it, and the driver's side keeps
 * a record of every thread that enters it: so frames are put in a queue here, and one thread of
 * ours takes them to the driver. A queue that is full drops what comes (TCP sends it again).
 */
#define LX_QUEUE        256

struct lx_frame {
	uint16_t length;
	uint16_t flags;                         /* lx_transmit()'s: the WMM class's TID, LX_TX_CSUM */
	uint8_t bytes[HAKIOS_NET_FRAME_MAX];
};

/*
 * WIFI-PLAN P1d. wifi_offload=0: the stack does every checksum itself (none left to the radio, none of the radio's
 * results believed). wifi_wmm=0: every frame best effort (TID 0). Both on by default.
 */
static unsigned lx_offload = 1, lx_wmm = 1;

/* The frame's service class as the 802.11 TID of its WMM access class (as Apple's drivers map them). */
static unsigned
lx_frame_tid(mbuf_t packet)
{
	if (!lx_wmm) {
		return 0;
	}
	switch (mbuf_get_service_class(packet)) {
	case MBUF_SC_BK_SYS:
	case MBUF_SC_BK:
		return 1;                       /* background */
	case MBUF_SC_AV:
	case MBUF_SC_RV:
	case MBUF_SC_VI:
		return 5;                       /* video */
	case MBUF_SC_VO:
	case MBUF_SC_CTL:
		return 6;                       /* voice */
	default:
		return 0;                       /* best effort: BE, RD, OAM */
	}
}

/* LX_TX_CSUM if the stack left checksums of the frame for the interface to fill in (it was told it may: lx_link.offload). */
static unsigned
lx_frame_csum(mbuf_t packet)
{
	mbuf_csum_request_flags_t request = 0;
	uint32_t value = 0;

	if (!lx_offload || mbuf_get_csum_requested(packet, &request, &value) != 0) {
		return 0;
	}
	return request & (MBUF_CSUM_REQ_IP | MBUF_CSUM_REQ_TCP | MBUF_CSUM_REQ_UDP | MBUF_CSUM_REQ_TCPIPV6 |
	       MBUF_CSUM_REQ_UDPIPV6) ? LX_TX_CSUM : 0;
}

static bool lx_link_attached;
static bool lx_queue_made;                      /* the queue and its thread: made once, whatever the link's attach said */
static LCK_MTX_DECLARE(lx_queue_lock, &lx_lock_group);
static struct lx_frame *lx_queue;
static uint32_t lx_queue_in, lx_queue_out;      /* frames put in and taken out, counted from the start */
static uint64_t lx_queue_dropped;

static void
lx_queue_runs(__unused void *parameter, __unused wait_result_t result)
{
	for (;;) {
		struct lx_frame *frame;

		lck_mtx_lock(&lx_queue_lock);
		while (lx_queue_in == lx_queue_out) {
			lck_mtx_sleep(&lx_queue_lock, LCK_SLEEP_DEFAULT, (event_t)&lx_queue_in, THREAD_UNINT);
		}
		frame = &lx_queue[lx_queue_out % LX_QUEUE];     /* (stays ours until lx_queue_out moves) */
		lck_mtx_unlock(&lx_queue_lock);
		lx_transmit(frame->bytes, frame->length, frame->flags);
		lck_mtx_lock(&lx_queue_lock);
		lx_queue_out++;
		lck_mtx_unlock(&lx_queue_lock);
	}
}

static bool
lx_link_transmit(mbuf_t packet, size_t length)
{
	struct lx_frame *frame;

	lck_mtx_lock(&lx_queue_lock);
	if (length > HAKIOS_NET_FRAME_MAX || lx_queue_in - lx_queue_out == LX_QUEUE) {
		lx_queue_dropped++;
		lck_mtx_unlock(&lx_queue_lock);
		return false;
	}
	frame = &lx_queue[lx_queue_in % LX_QUEUE];
	if (mbuf_copydata(packet, 0, length, frame->bytes) != 0) {
		lck_mtx_unlock(&lx_queue_lock);
		return false;
	}
	frame->length = (uint16_t)length;
	frame->flags = (uint16_t)(lx_frame_tid(packet) | lx_frame_csum(packet));
	lx_queue_in++;
	lck_mtx_unlock(&lx_queue_lock);
	thread_wakeup((event_t)&lx_queue_in);
	return true;
}

static struct hakios_net_link lx_link = {
	.name = "the phone's radio (ath11k)",
	.wireless = true,
	.transmit = lx_link_transmit,
};

/* (kern.hakios.wifi_doze and wifi_ps, defined with their sysctls below; their boot args are read here) */
static unsigned wifi_doze, wifi_ps;

void
lx_os_net_attach(const unsigned char *address)
{
	thread_t thread;

	if (lx_link_attached) {
		if (memcmp(lx_link.address, address, sizeof(lx_link.address)) != 0) {
			/* (en0 was made early, with wifi_mac; the radio ended with another address: no address is said) */
			say("lx: THE RADIO'S ADDRESS IS NOT THE ONE en0 WAS MADE WITH (wifi_mac, given early): frames will not match it\n");
		}
		return;
	}
	/*
	 * (a call after one whose attach was refused tries the attach again, and only that: a second queue would leave the
	 * first's memory behind, and a second thread would take frames out of the same queue as the first)
	 */
	if (!lx_queue_made) {
		if (lx_queue == NULL) {
			lx_queue = kalloc_data(LX_QUEUE * sizeof(*lx_queue), Z_WAITOK | Z_ZERO);
		}
		if (lx_queue == NULL || kernel_thread_start(lx_queue_runs, NULL, &thread) != KERN_SUCCESS) {
			say("lx: no memory or no thread for the frames to send: the radio's frames go nowhere\n");
			return;
		}
		thread_set_thread_name(thread, "wifi-send");
		thread_deallocate(thread);
		lx_queue_made = true;
	}
	PE_parse_boot_argn("wifi_offload", &lx_offload, sizeof(lx_offload));
	PE_parse_boot_argn("wifi_wmm", &lx_wmm, sizeof(lx_wmm));
	PE_parse_boot_argn("wifi_doze", &wifi_doze, sizeof(wifi_doze));
	PE_parse_boot_argn("wifi_ps", &wifi_ps, sizeof(wifi_ps));
	/* (ath11k's TCL fills in IPv4 header, TCP and UDP checksums, v4 and v6, for a frame given as CHECKSUM_PARTIAL: dp_tx.c) */
	lx_link.offload = lx_offload ? HAKIOS_NET_CSUM_TX : 0;
	memcpy(lx_link.address, address, sizeof(lx_link.address));
	lx_link_attached = hakios_net_attach(&lx_link);
	say("lx: %s\n", lx_link_attached ? "the radio is the link behind en0" : "EN0 HAS A LINK ALREADY: the radio's frames go "
	    "nowhere");
}

/*
 * With wifi=1 the interface is made when the kernel starts, before the radio is up: Darwin's Wi-Fi
 * side looks for it and for its registry entry as soon as it runs, once, and a radio that
 * turns up twelve seconds later is one it has already failed to open.
 */
void
hakios_lx_net_early(void)
{
	uint8_t address[6];

	if (hakios_wifi_radio_address(address)) {
        lx_reserve_radio_pool();
		lx_os_net_attach(address);
	}
}

void
lx_os_net_link(int up)
{
	if (lx_link_attached) {
		hakios_net_link_state(up != 0);
	}
}

void
lx_os_net_input(const unsigned char *frame, unsigned long length, int checked)
{
	if (lx_link_attached && length <= HAKIOS_NET_FRAME_MAX) {
		hakios_net_input_checked(0, frame, length, checked && lx_offload);
	}
}

/* ---- the radio's interrupts ------------------------------------------------------------------------------- */

extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *context, uint32_t intid), void *context);
extern void gic_v3_spi_mask(uint32_t intid);
extern void gic_v3_spi_unmask(uint32_t intid);
extern void gic_v3_spi_detach(uint32_t intid);
extern uint64_t gic_v3_spi_count(uint32_t intid);

#define LX_VECTORS      32

static struct {
	void (*function)(void *);
	void *argument;
	bool attached;
} lx_vector[LX_VECTORS];
static uint32_t lx_irq_first;                   /* the GIC's number of vector 0; 0: no interrupts */
static uint32_t lx_irq_pending;                 /* the vectors that have fired and are not dealt with yet */
static uint32_t lx_irq_running;                 /* the vectors whose function the thread is in */
static thread_t lx_irq_thread;
static LCK_MTX_DECLARE(lx_irq_lock, &lx_lock_group);

/* In interrupt context: the vector masked, and the thread told. */
static void
lx_irq_arrived(__unused void *context, uint32_t intid)
{
	gic_v3_spi_mask(intid);
	os_atomic_or(&lx_irq_pending, 1u << (intid - lx_irq_first), relaxed);
	thread_wakeup((event_t)&lx_irq_pending);
}

static void
lx_irq_runs(__unused void *parameter, __unused wait_result_t result)
{
	for (;;) {
		uint32_t pending;

		for (;;) {
			assert_wait((event_t)&lx_irq_pending, THREAD_UNINT);
			if ((pending = os_atomic_xchg(&lx_irq_pending, 0, acquire)) != 0) {
				clear_wait(current_thread(), THREAD_AWAKENED);
				break;
			}
			thread_block(THREAD_CONTINUE_NULL);
		}
		for (unsigned v = 0; v < LX_VECTORS; v++) {
			if (!(pending & 1u << v)) {
				continue;
			}
			lck_mtx_lock(&lx_irq_lock);
			if (!lx_vector[v].attached) {
				lck_mtx_unlock(&lx_irq_lock);
				continue;
			}
			lx_irq_running |= 1u << v;
			lck_mtx_unlock(&lx_irq_lock);
			lx_vector[v].function(lx_vector[v].argument);
			lck_mtx_lock(&lx_irq_lock);
			lx_irq_running &= ~(1u << v);
			lck_mtx_unlock(&lx_irq_lock);
			thread_wakeup((event_t)&lx_irq_running);
		}
	}
}

int
lx_os_irq_msi(unsigned long long *address, unsigned *data)
{
	uint32_t first = 0;

	if (!hakios_wifi_radio_msi(address, data, &first)) {
		return 0;
	}
	lx_irq_first = first;
	return 1;
}

int
lx_os_irq_attach(unsigned vector, void (*function)(void *), void *argument)
{
	kern_return_t kr;

	if (vector >= LX_VECTORS || lx_irq_first == 0 || function == NULL) {
		return EINVAL;
	}
	if (lx_irq_thread == THREAD_NULL) {
		hakios_wifi_tell("lx: wifi_irq: THE GIC'S DISTRIBUTOR IS WRITTEN for the radio's interrupts: each vector's SPI (%u + v) edge-"
		    "triggered, group 1, routed to this processor, then enabled\n", lx_irq_first);
		if (kernel_thread_start(lx_irq_runs, NULL, &lx_irq_thread) != KERN_SUCCESS) {
			lx_irq_thread = THREAD_NULL;
			return ENOMEM;
		}
		thread_set_thread_name(lx_irq_thread, "wifi-irq");
	}
	lck_mtx_lock(&lx_irq_lock);
	lx_vector[vector].function = function;
	lx_vector[vector].argument = argument;
	lx_vector[vector].attached = true;
	lck_mtx_unlock(&lx_irq_lock);
	if ((kr = gic_v3_spi_attach(lx_irq_first + vector, true, lx_irq_arrived, NULL)) != KERN_SUCCESS) {
		lck_mtx_lock(&lx_irq_lock);
		lx_vector[vector].attached = false;
		lck_mtx_unlock(&lx_irq_lock);
		return kr == KERN_NAME_EXISTS ? EBUSY : EINVAL;
	}
	gic_v3_spi_mask(lx_irq_first + vector);
	return 0;
}

void
lx_os_irq_enable(unsigned vector)
{
	if (vector >= LX_VECTORS) {
		return;
	}
	/* (looked at under the lock: a vector being let go of is not unmasked again by a function that finishes meanwhile) */
	lck_mtx_lock(&lx_irq_lock);
	if (lx_vector[vector].attached) {
		gic_v3_spi_unmask(lx_irq_first + vector);
	}
	lck_mtx_unlock(&lx_irq_lock);
}

void
lx_os_irq_disable(unsigned vector)
{
	if (vector >= LX_VECTORS || !lx_vector[vector].attached) {
		return;
	}
	gic_v3_spi_mask(lx_irq_first + vector);
	if (current_thread() == lx_irq_thread) {
		return;                         /* (from the vector's own function, or another's: nothing to wait for) */
	}
	lck_mtx_lock(&lx_irq_lock);
	while (lx_irq_running & 1u << vector) {
		lck_mtx_sleep(&lx_irq_lock, LCK_SLEEP_DEFAULT, (event_t)&lx_irq_running, THREAD_UNINT);
	}
	lck_mtx_unlock(&lx_irq_lock);
}

void
lx_os_irq_detach_all(void)
{
	bool was[LX_VECTORS];

	/*
	 * Not attached any more first, and the functions running finished: only then are the lines let go of, so that none
	 * finishing in between unmasks a line that has no handler left (lx_os_irq_enable())
	 */
	lck_mtx_lock(&lx_irq_lock);
	for (unsigned v = 0; v < LX_VECTORS; v++) {
		was[v] = lx_vector[v].attached;
		lx_vector[v].attached = false;
	}
	while (lx_irq_running != 0 && current_thread() != lx_irq_thread) {
		lck_mtx_sleep(&lx_irq_lock, LCK_SLEEP_DEFAULT, (event_t)&lx_irq_running, THREAD_UNINT);
	}
	lck_mtx_unlock(&lx_irq_lock);
	for (unsigned v = 0; v < LX_VECTORS; v++) {
		if (was[v]) {
			gic_v3_spi_mask(lx_irq_first + v);
			gic_v3_spi_detach(lx_irq_first + v);
		}
	}
	os_atomic_store(&lx_irq_pending, 0, relaxed);
}

unsigned long long
lx_os_irq_count(unsigned vector)
{
	return vector < LX_VECTORS && lx_irq_first != 0 ? gic_v3_spi_count(lx_irq_first + vector) : 0;
}

/* ---- the PCIe bus, the radio's memory and the router's link (C4; the board's wifi-qca6490, board/wifi_pci.c) ----------------------- */

/*
 * C2 (board/pcie.c) owns the root complex and refuses a second up(). The chain of board/wifi_pci.c brings the link
 * up first (stages 1-3) and hands it over with hakios_lx_pcie_adopt(); ath11k's probe then asks lx_os_pcie_up() and is given the
 * link that is already active, never a second up. lx_os_pcie_down() (ath11k's removal, or a failed probe) powers the radio down
 * (C2 is idempotent: the chain's own teardown asks again).
 */
static struct {
	bool active;
	struct hakios_pcie_link link;
} lx_pcie;

void
hakios_lx_pcie_adopt(const struct hakios_pcie_link *link)
{
	lx_pcie.link = *link;
	lx_pcie.active = true;
}

void
hakios_lx_pcie_forget(void)
{
	lx_pcie.active = false;
	bzero(&lx_pcie.link, sizeof(lx_pcie.link));
}

int
lx_os_pcie_up(unsigned rc, struct hakios_pcie_link *link)
{
	int error;

	bzero(link, sizeof(*link));
	if (rc != HAKIOS_PCIE_WIFI_RC) {
		return ENODEV;
	}
	if (!lx_pcie.active) {
		/* nobody has brought it up (a run of the driver on its own): do it here, once */
		if ((error = hakios_pcie_up(rc, &lx_pcie.link)) != 0) {
			return error;
		}
		lx_pcie.active = true;
	}
	*link = lx_pcie.link;
	return 0;
}

void
lx_os_pcie_down(unsigned rc)
{
	if (rc != HAKIOS_PCIE_WIFI_RC) {
		return;
	}
	hakios_pcie_down(rc);
	hakios_lx_pcie_forget();
}

int lx_os_pcie_down_status(void) { return hakios_pcie_down_status(); }

/* ---- doze (lx_80211.c): the link's sleep, WAKE#, and its words ---- */
int lx_os_pcie_sleep(bool fast_mhi) { return hakios_pcie_sleep(fast_mhi); }
int lx_os_pcie_wake(void) { return hakios_pcie_wake(); }
int lx_os_pcie_l1_hold(unsigned *saved) { return hakios_pcie_l1_hold(saved); }
int lx_os_pcie_l1_restore(unsigned saved) { return hakios_pcie_l1_restore(saved); }
int lx_os_pcie_wake_irq(void (*cb)(void)) { return hakios_pcie_wake_irq(cb); }
void lx_os_pcie_wake_arm(int on) { hakios_pcie_wake_arm(on != 0); }
int lx_os_pcie_wake_line(void) { return hakios_pcie_wake_line() ? 1 : 0; }
unsigned lx_os_pcie_sleep_mode(void) { return hakios_pcie_sleep_mode(); }

/* kern.hakios.wifi_doze: 1 the radio dozes (WoW, M3, link off) with the screen off after wifi_doze_idle_ms without a
 * frame; kern.hakios.wifi_ps: 1 station power save. Both on by default, as negroni-dev's run86 had them (93 of 120 s in
 * CX collapse unplugged, every wake answered).
 * wifi_doze=0: the radio never dozes (default 1: with the screen off, after 1 s without a frame).
 * wifi_ps=0: no station power save (default 1: the firmware sleeps between beacons). */
static unsigned wifi_doze = 1, wifi_doze_idle_ms = 1000, wifi_ps = 1, wifi_doze_patterns = 0xf;
SYSCTL_UINT(_kern_hakios, OID_AUTO, wifi_doze_patterns, CTLFLAG_RW | CTLFLAG_LOCKED, &wifi_doze_patterns, 0,
    "doze wake events: 1 unicast to us, 2 IPv6 solicited-node multicast, 4 disconnect, 8 GTK rekey failure");
unsigned lx_os_doze_patterns(void) { return wifi_doze_patterns; }
static unsigned wifi_doze_kick = 2;	/* (run51: in M3 no interrupt came with new events, so 1; run86's wakes come by the WAKE MSI) */
SYSCTL_UINT(_kern_hakios, OID_AUTO, wifi_doze_kick, CTLFLAG_RW | CTLFLAG_LOCKED, &wifi_doze_kick, 0,
    "the radio's MHI interrupt in M3 ends a doze: 0 never, 1 always, 2 only with new events (Linux)");
unsigned lx_os_doze_kick_mode(void) { return wifi_doze_kick; }
static char wifi_doze_text[2048] = "not started (no network joined yet)";
static const void *doze_event;
void lx_os_doze_event(const void *event) { doze_event = event; }
static int
sysctl_wifi_doze SYSCTL_HANDLER_ARGS
{
#pragma unused(arg1, arg2)
	unsigned value = wifi_doze;
	int error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || req->newptr == USER_ADDR_NULL) return error;
	wifi_doze = value;
	if (doze_event) thread_wakeup((event_t)doze_event);
	return 0;
}
/*
 * The firmware's receive data filter (WMI_HW_DATA_FILTER: bit 0 drop broadcasts but ARP, bit 1 drop multicasts but
 * ICMPv6), set by a write: 0 off, else those bits on. ath11k sets it only for WoW and clears it at resume. run69/70: no
 * broadcast ever reached the host, multicast did -- is it on from the start?
 */
static unsigned wifi_rx_filter, wifi_rx_filter_asked;
static int
sysctl_wifi_rx_filter SYSCTL_HANDLER_ARGS
{
#pragma unused(arg1, arg2)
	unsigned value = wifi_rx_filter;
	int error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || req->newptr == USER_ADDR_NULL) return error;
	wifi_rx_filter = value & 3;
	wifi_rx_filter_asked = 1;
	if (doze_event) thread_wakeup((event_t)doze_event);
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, wifi_rx_filter, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, sysctl_wifi_rx_filter,
    "I", "a write sends the firmware's receive data filter: 0 off, 1 drop broadcasts but ARP, 2 drop multicasts but ICMPv6");
bool
lx_os_rx_filter_asked(unsigned *mode)
{
	if (!wifi_rx_filter_asked) return false;
	wifi_rx_filter_asked = 0;
	*mode = wifi_rx_filter;
	return true;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, wifi_doze, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, sysctl_wifi_doze, "I",
    "1: Wi-Fi dozes (WoW, MHI M3, the PCIe link to the DSP) with the screen off and no traffic");
SYSCTL_UINT(_kern_hakios, OID_AUTO, wifi_doze_idle_ms, CTLFLAG_RW | CTLFLAG_LOCKED, &wifi_doze_idle_ms, 0,
    "milliseconds without a frame before Wi-Fi dozes (200-600000)");
SYSCTL_UINT(_kern_hakios, OID_AUTO, wifi_ps, CTLFLAG_RW | CTLFLAG_LOCKED, &wifi_ps, 0,
    "1: Wi-Fi station power save (the firmware sleeps between beacons)");
SYSCTL_STRING(_kern_hakios, OID_AUTO, wifi_doze_stats, CTLFLAG_RD | CTLFLAG_LOCKED, wifi_doze_text, sizeof(wifi_doze_text),
    "Wi-Fi doze counts");
void
lx_os_doze_config(unsigned *on, unsigned *idle_ms, unsigned *ps)
{
	unsigned idle = wifi_doze_idle_ms;
	*on = wifi_doze != 0 && hakios_wifi_pci_selected();
	*idle_ms = idle < 200 ? 200 : idle > 600000 ? 600000 : idle;
	*ps = wifi_ps != 0;
}
void
lx_os_doze_stats(const char *text)
{
	strlcpy(wifi_doze_text, text, sizeof(wifi_doze_text));
}
/* en0's first IPv4 address, network order (for the firmware's ARP offload). 0, or an errno. */
int
lx_os_wifi_ipv4(unsigned *be_addr)
{
	ifnet_t ifp = NULL;
	ifaddr_t *list = NULL;
	int error = ENOENT;

	if (ifnet_find_by_name("en0", &ifp) != 0 || ifp == NULL) {
		return ENXIO;
	}
	if (ifnet_get_address_list_family(ifp, &list, AF_INET) == 0 && list != NULL) {
		for (unsigned i = 0; list[i] != NULL; i++) {
			struct sockaddr_in sin;
			if (ifaddr_address(list[i], (struct sockaddr *)&sin, sizeof(sin)) == 0 && sin.sin_family == AF_INET &&
			    sin.sin_addr.s_addr != 0) {
				*be_addr = sin.sin_addr.s_addr;
				error = 0;
				break;
			}
		}
		ifnet_free_address_list(list);
	}
	ifnet_release(ifp);
	return error;
}
/* Return the assigned unicast IPv6 addresses in wire format. */
int
lx_os_wifi_ipv6_addresses(uint8_t addresses[][16], unsigned capacity)
{
	ifnet_t ifp = NULL;
	ifaddr_t *list = NULL;
	int count = 0;

	if (ifnet_find_by_name("en0", &ifp) != 0 || ifp == NULL) return -ENXIO;
	if (ifnet_get_address_list_family(ifp, &list, AF_INET6) != 0) {
		ifnet_release(ifp);
		return -EIO;
	}
	for (unsigned i = 0; list != NULL && list[i] != NULL; i++) {
		struct sockaddr_in6 address;
		unsigned index;

		if (ifaddr_address(list[i], (struct sockaddr *)&address, sizeof(address)) != 0 ||
		    address.sin6_family != AF_INET6 || IN6_IS_ADDR_UNSPECIFIED(&address.sin6_addr) ||
		    IN6_IS_ADDR_MULTICAST(&address.sin6_addr)) continue;
		if (IN6_IS_ADDR_LINKLOCAL(&address.sin6_addr)) {
			/* XNU can store the interface scope in the address itself. */
			address.sin6_addr.s6_addr[2] = 0;
			address.sin6_addr.s6_addr[3] = 0;
		}
		for (index = 0; index < (unsigned)count; index++)
			if (memcmp(addresses[index], address.sin6_addr.s6_addr, 16) == 0) break;
		if (index < (unsigned)count) continue;
		if ((unsigned)count == capacity) {
			count = -ENOSPC;
			break;
		}
		memcpy(addresses[count++], address.sin6_addr.s6_addr, 16);
	}
	if (list != NULL) ifnet_free_address_list(list);
	ifnet_release(ifp);
	return count;
}

int lx_os_pcie_pm_start(void) { return hakios_pcie_pm_start(); }
void lx_os_pcie_pm_sample(void) { hakios_pcie_pm_sample(); }

int
lx_os_pcie_cfg_read(unsigned rc, unsigned bdf, unsigned off, unsigned size, unsigned *value)
{
	return hakios_pcie_cfg_read(rc, bdf, off, size, value);
}

int
lx_os_pcie_cfg_write(unsigned rc, unsigned bdf, unsigned off, unsigned size, unsigned value)
{
	return hakios_pcie_cfg_write(rc, bdf, off, size, value);
}

int
lx_os_pcie_try_read32(const volatile void *va, unsigned *value)
{
	return hakios_pcie_bar_read32(va, value);
}

int
lx_os_pcie_try_write32(volatile void *va, unsigned value)
{
	return hakios_pcie_bar_write32(va, value);
}

/*
 * ONE contiguous pool for everything the radio reads and writes: its rings, the firmware's memory segments, the buffers of MHI,
 * the page its interrupts are written to. Mapped in the stage-1 context of SID 0x1c01 at HAKIOS_WIFI_IOVA_BASE (translate), so the
 * radio, which has 32-bit addresses, sees all of it below 4 GiB whatever the physical addresses are; or, with wifi_smmu=bypass
 * (diagnosis), at its own physical addresses, which is why that mode takes memory from below 4 GiB or refuses. Like
 * lx_os_radio_memory() it is reached through a second, uncached mapping, so that nothing is left in the processor's cache for
 * the radio not to see (and the other way round).
 *
 * The chain claims the context first (hakios_lx_dma_context()). hakios_lx_dma_release() takes the pool out of the context again
 * when the chain is undone, with the radio already powered down; the memory itself is KEPT for the rest of the boot and shown
 * again if the radio is started again: it has an uncached alias, and giving that memory back to the kernel while the alias exists
 * is the one mistake this way of doing it must not make.
 */
static struct {
	struct hakios_smmu_context *context;
	bool have_context, bypass;
	bool allocated, mapped;
	unsigned long size;
	vm_offset_t cached, uncached;
	uint64_t physical;
	unsigned long long iova;
} lx_pool;
static LCK_MTX_DECLARE(lx_pool_lock, &lx_lock_group);

void
hakios_lx_dma_context(struct hakios_smmu_context *context, bool bypass)
{
	lck_mtx_lock(&lx_pool_lock);
	lx_pool.context = context;
	lx_pool.bypass = bypass;
	lx_pool.have_context = context != NULL || bypass;
	lck_mtx_unlock(&lx_pool_lock);
}

/* The memory itself, once (the pool lock is held). 0, or an errno. */
static int
lx_pool_allocate(unsigned long size)
{
	vm_offset_t cached = 0, uncached;
	uint64_t physical;

	/* (a bypass pool must lie below 4 GiB: its IOVA is its physical address; the limit is the highest page number,
	 * in ppnum_t units, which are 16 KiB pages on arm64, hence atop and not 0xfffff) */
	if (kmem_alloc_contig(kernel_map, &cached, size, 0, lx_pool.bypass ? (ppnum_t)(atop(0x100000000ull) - 1) : 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		say("lx: no %lu bytes of memory in one piece for the radio\n", size);
		return ENOMEM;
	}
	physical = kvtophys(cached);
	if (lx_pool.bypass && physical + size > 0x100000000ull) {
		kmem_free(kernel_map, cached, size);     /* (no alias yet: it can go back) */
		return ENOMEM;
	}
	bzero((void *)cached, size);
	for (vm_offset_t line = cached; line < cached + size; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy");
	uncached = ml_io_map_unmappable(physical, size, VM_WIMG_WCOMB);
	if (uncached == 0) {
		kmem_free(kernel_map, cached, size);
		return ENOMEM;
	}
	lx_pool.cached = cached;
	lx_pool.uncached = uncached;
	lx_pool.physical = physical;
	lx_pool.size = size;
	lx_pool.allocated = true;
	return 0;
}

/*
 * The pool's memory taken at the kernel's start (hakios_wifi_pci_init, before the chain's thread): 64 MiB in one piece is
 * there then, and not three minutes later (round 1, 2026-11-12: stage 4 errno 12 after wifi_delay=120). The context
 * comes later (stage 4); lx_os_dma_pool() then finds the memory allocated and only maps it. bypass: as wifi_smmu=bypass.
 */
int
hakios_lx_pool_reserve(bool bypass)
{
	int error = 0;

	lck_mtx_lock(&lx_pool_lock);
	if (!lx_pool.allocated) {
		lx_pool.bypass = bypass;
		error = lx_pool_allocate(HAKIOS_WIFI_POOL_SIZE);
		if (error == 0) {
			say("lx: the radio's memory, %lu MiB in one piece, taken at the start (physical %#llx)\n",
			    HAKIOS_WIFI_POOL_SIZE >> 20, (unsigned long long)lx_pool.physical);
		}
	}
	lck_mtx_unlock(&lx_pool_lock);
	return error;
}

int
lx_os_dma_pool(unsigned long size, void **va, unsigned long long *iova)
{
	int error = 0;

	*va = NULL;
	*iova = 0;
	if (size == 0 || (size & PAGE_MASK) != 0 || size > HAKIOS_WIFI_POOL_SIZE) {
		return EINVAL;
	}
	lck_mtx_lock(&lx_pool_lock);
	if (!lx_pool.have_context) {
		lck_mtx_unlock(&lx_pool_lock);
		return ENODEV;                  /* (stage 4's context is not there: nothing may be shown to the radio) */
	}
	if (lx_pool.mapped) {
		if (lx_pool.size != size) {
			error = EBUSY;
		} else {
			*va = (void *)lx_pool.uncached;
			*iova = lx_pool.iova;
		}
		lck_mtx_unlock(&lx_pool_lock);
		return error;
	}
	if (!lx_pool.allocated) {
		error = lx_pool_allocate(size);
	} else if (lx_pool.size != size) {
		error = EBUSY;
	} else {
		bzero((void *)lx_pool.uncached, size);  /* (shown again: nothing of the last run is in it) */
	}
	if (error == 0) {
		lx_pool.iova = lx_pool.bypass ? lx_pool.physical : HAKIOS_WIFI_IOVA_BASE;
		if (lx_pool.iova + size > 0x100000000ull) {
			error = EINVAL;         /* (the radio has 32-bit addresses: every IOVA of the pool lies below 4 GiB, or there is none) */
		} else if (!lx_pool.bypass) {
			error = hakios_smmu_map(lx_pool.context, lx_pool.iova, lx_pool.physical, size, HAKIOS_SMMU_READ | HAKIOS_SMMU_WRITE);
		}
	}
	if (error == 0) {
		lx_pool.mapped = true;
		*va = (void *)lx_pool.uncached;
		*iova = lx_pool.iova;
	}
	lck_mtx_unlock(&lx_pool_lock);
	return error;
}

/* The pool out of the SMMU context (the radio is powered down); the memory stays. */
void
hakios_lx_dma_release(void)
{
	lck_mtx_lock(&lx_pool_lock);
	if (lx_pool.mapped) {
		int error = 0;

		if (!lx_pool.bypass && lx_pool.context != NULL) {
			hakios_smmu_faults(lx_pool.context);     /* (what the radio was refused, if anything: said and cleared) */
			error = hakios_smmu_unmap(lx_pool.context, lx_pool.iova, lx_pool.size);
		}
		if (error != 0) {
			say("lx: the radio's memory could not be taken out of the SMMU context: errno %d (it is kept, not reused)\n", error);
		}
		lx_pool.mapped = false;
	}
	lck_mtx_unlock(&lx_pool_lock);
}

/*
 * The router's link, the IPCR channels of MHI (kernel/lx/src/lx_mhi.c). The router (board/qrtr.c) wants hakios_qrtr_poll() now and
 * then, or a lookup on a link that has nothing to say is answered only at the next packet: the watcher below calls it after each
 * lookup (hakios_qrtr_wait(): 100 ms at most, 1 s with the screen off) while the link is up. It also looks for the Wi-Fi firmware's service itself, with a port of its own, and says
 * stage 7 when the HELLO has come and the service is there (never before stage 6; after 20 s it says it is not). No lock of
 * ours is held across a call into the router, and the link is the caller's until link_down() has returned.
 */
#define LX_SERVICE_WLFW         0x45

static struct {
	const struct hakios_qrtr_link *link;
	volatile bool stop, running, wlfw_there;
	volatile uint32_t wlfw_node, wlfw_port;
} lx_qrtr;
static LCK_MTX_DECLARE(lx_qrtr_lock, &lx_lock_group);
void lx_os_qrtr_diagnostic(void)
{
	uint32_t peer, hellos, in, out, services;
	hakios_qrtr_stats(&peer, &hellos, &in, &out, &services);
	hakios_wifi_tell("lx: runtime QRTR link %u running %u stop %u wlfw %u peer %u hellos %u in %u out %u services %u\n",
	    lx_qrtr.link != NULL, lx_qrtr.running, lx_qrtr.stop, lx_qrtr.wlfw_there, peer, hellos, in, out, services);
}

static void
lx_qrtr_server(__unused void *context, bool arrived, __unused uint32_t service, __unused uint32_t version, __unused uint32_t instance,
    uint32_t node, uint32_t port)
{
	lx_qrtr.wlfw_node = arrived ? node : 0;
	lx_qrtr.wlfw_port = arrived ? port : 0;
	lx_qrtr.wlfw_there = arrived;
}

static void
lx_qrtr_datagram(__unused void *context, __unused uint32_t node, __unused uint32_t port, __unused const void *data, __unused size_t length)
{
}

static uint64_t
lx_milliseconds(void)
{
	return lx_os_nanoseconds() / 1000000ull;
}

static void
lx_qrtr_watch(__unused void *parameter, __unused wait_result_t result)
{
	struct hakios_qrtr_port *port = hakios_qrtr_router_ops.open(lx_qrtr_datagram, lx_qrtr_server, NULL);
	uint64_t began = lx_milliseconds();
	bool said = false;

	if (port == NULL || hakios_qrtr_router_ops.lookup(port, LX_SERVICE_WLFW, 1, 0) != 0) {
		lx_os_stage(7, "qrtr", 0, "no router port to look the Wi-Fi firmware's service up with");
		said = true;
	}
	while (!lx_qrtr.stop) {
		hakios_qrtr_poll();
		if (!said && lx_qrtr.wlfw_there && hakios_wifi_pci_seen(6) == 1) {
			uint32_t peer, hellos, in, out, services;

			hakios_qrtr_stats(&peer, &hellos, &in, &out, &services);
			lx_os_stage(7, "qrtr", 1, "qrtr: hello from node %u, WLFW at %u:%u", peer, lx_qrtr.wlfw_node, lx_qrtr.wlfw_port);
			said = true;
		} else if (!said && lx_milliseconds() - began > 20000) {
			uint32_t peer, hellos, in, out, services;

			hakios_qrtr_stats(&peer, &hellos, &in, &out, &services);
			lx_os_stage(7, "qrtr", 0, "no WLFW server after 20 s: %u hellos, %u services, %u packets in, %u out", hellos, services,
			    in, out);
			said = true;
		}
		hakios_qrtr_wait(hakios_screen_off() ? 1000 : 100);
	}
	if (port != NULL) {
		hakios_qrtr_router_ops.close(port);
	}
	lx_thread_release();
	lck_mtx_lock(&lx_qrtr_lock);
	lx_qrtr.running = false;
	lck_mtx_unlock(&lx_qrtr_lock);
	thread_wakeup((event_t)&lx_qrtr.running);
	thread_terminate(current_thread());
}

int
lx_os_qrtr_link(const struct hakios_qrtr_link *link)
{
	thread_t thread;
	int error;

	if (link == NULL) {
		const struct hakios_qrtr_link *old = lx_qrtr.link;

		if (old == NULL) {
			return 0;
		}
		lx_qrtr.stop = true;
		hakios_qrtr_wake();
		lck_mtx_lock(&lx_qrtr_lock);
		while (lx_qrtr.running) {
			lck_mtx_sleep(&lx_qrtr_lock, LCK_SLEEP_DEFAULT, (event_t)&lx_qrtr.running, THREAD_UNINT);
		}
		lck_mtx_unlock(&lx_qrtr_lock);
		hakios_qrtr_link_down(old);             /* returns with no callback of the link running */
		lx_qrtr.link = NULL;
		return 0;
	}
	if (lx_qrtr.link == link) {
		return 0;                       /* (the same one again: it is up, and its watcher runs) */
	}
	if (lx_qrtr.link != NULL) {
		return EBUSY;
	}
	if ((error = hakios_qrtr_link_up(link)) != 0) {
		return error;
	}
	lx_qrtr.link = link;
	lx_qrtr.stop = false;
	lx_qrtr.wlfw_there = false;
	lx_qrtr.wlfw_node = lx_qrtr.wlfw_port = 0;
	lx_qrtr.running = true;
	if (kernel_thread_start(lx_qrtr_watch, NULL, &thread) != KERN_SUCCESS) {
		lx_qrtr.running = false;
		hakios_qrtr_link_down(link);
		lx_qrtr.link = NULL;
		return EAGAIN;
	}
	thread_set_thread_name(thread, "wifi-qrtr");
	thread_deallocate(thread);
	return 0;
}

int
lx_os_qrtr_receive(const struct hakios_qrtr_link *link, const void *packet, unsigned long len)
{
	return hakios_qrtr_receive(link, packet, len);
}

/* ---- the station (sys/board/sta.h) ---------------------------------------------------------------------- */

void
lx_os_station(const struct hakios_sta_ops *ops)
{
	hakios_sta_attach(ops);
}

void
lx_os_station_lost(int error)
{
	hakios_sta_lost(error);
}

/* ---- the driver ---------------------------------------------------------------------------------------- */

/* The object has its own; this is the one of a kernel built without it. */
__attribute__((weak, noinline)) void
lx_thread_release(void)
{
}

__attribute__((weak)) int
lx_start(void)
{
	say("lx: THE DRIVER IS NOT BUILT IN: its object was not there when this kernel was built (kernel/lx/build.sh, then "
	    "build the kernel again)\n");
	return ENXIO;
}

__attribute__((weak)) void
lx_stop(void)
{
}

__attribute__((weak)) int
lx_prepare(void)
{
	return 0;
}

__attribute__((weak)) int
lx_transmit(__unused const unsigned char *frame, __unused unsigned long length, __unused unsigned flags)
{
	return ENXIO;
}

/* (the system's sleep, GenericARMPE::sleepKernel(): no radio to doze) */
__attribute__((weak)) int
lx_doze_for_sleep(__unused unsigned ms)
{
	return -2;
}

__attribute__((weak)) void
lx_doze_sleep_over(void)
{
}

int
hakios_lx_start(void)
{
	return lx_start();
}

void
hakios_lx_stop(void)
{
	lx_stop();
}

int
hakios_lx_prepare(void)
{
	return lx_prepare();
}

/* ---- the PCIe radio's MSIs through the GIC's ITS (C9, board/its.c; WIFI-PLAN.md P1b) ------------------------
 *
 * The radio writes its vector number to the ITS's doorbell, mapped as device memory at HAKIOS_WIFI_DOORBELL_IOVA of its
 * SMMU context (just past the pool), or at its physical address with wifi_smmu=bypass. The ITS turns that into an LPI on
 * processor 0; its handler only notes the vector and wakes lx_pci.c's thread, which runs ath11k's handlers as before.
 * wifi_its=0: not used (the RAM mailbox and the 1 ms poll). */
#define LX_ITS_DEVICE   0x5981u                 /* PCIe0 msi-map: RID 0x100 -> DeviceID 0x5981 (live.dts) */

static uint32_t lx_msi_fired;
static bool lx_msi_on, lx_msi_doorbell;

static void
lx_msi_arrived(__unused void *context, unsigned event)
{
	os_atomic_or(&lx_msi_fired, 1u << (event & 31), relaxed);
	thread_wakeup((event_t)&lx_msi_fired);
}

int
lx_os_pci_msi(unsigned vectors, unsigned long long *address, unsigned *data)
{
	unsigned on = 1;
	unsigned long long doorbell = 0;
	int error;

	if (PE_parse_boot_argn("wifi_its", &on, sizeof(on)) && on == 0) {
		return -1;
	}
	if (vectors == 0 || vectors > 32 || lx_msi_on) {
		return EINVAL;
	}
	if ((error = hakios_its_device(LX_ITS_DEVICE, vectors, lx_msi_arrived, NULL, &doorbell)) != 0) {
		return error;
	}
	lck_mtx_lock(&lx_pool_lock);
	if (lx_pool.bypass) {
		*address = doorbell;
	} else if (lx_pool.context == NULL) {
		error = ENXIO;
	} else {
		error = hakios_smmu_map(lx_pool.context, HAKIOS_WIFI_DOORBELL_IOVA, doorbell & ~0xfffull, 4096,
		    HAKIOS_SMMU_READ | HAKIOS_SMMU_WRITE | HAKIOS_SMMU_MMIO);
		if (error == 0) {
			lx_msi_doorbell = true;
			*address = HAKIOS_WIFI_DOORBELL_IOVA + (doorbell & 0xfffull);
		}
	}
	lck_mtx_unlock(&lx_pool_lock);
	if (error != 0) {
		hakios_its_device_off(LX_ITS_DEVICE);
		return error;
	}
	*data = 0;
	os_atomic_store(&lx_msi_fired, 0, relaxed);
	lx_msi_on = true;
	return 0;
}

unsigned
lx_os_pci_msi_wait(unsigned milliseconds)
{
	uint32_t fired;

	/*
	 * The quiet sweep (every handler once, after milliseconds without an MSI: 10) is a net for lost MSIs and for work queued
	 * from other threads; with the screen off it comes every 200 ms (STANDBY-PLAN.md). An MSI still ends the wait at once.
	 */
	if (hakios_screen_off() && milliseconds < 200) {
		milliseconds = 200;
	}
	assert_wait_timeout((event_t)&lx_msi_fired, THREAD_UNINT, milliseconds, NSEC_PER_MSEC);
	if ((fired = os_atomic_xchg(&lx_msi_fired, 0, acquire)) != 0) {
		clear_wait(current_thread(), THREAD_AWAKENED);
		return fired;
	}
	(void)thread_block(THREAD_CONTINUE_NULL);
	return os_atomic_xchg(&lx_msi_fired, 0, acquire);
}

void
lx_os_pci_msi_off(void)
{
	if (!lx_msi_on) {
		return;
	}
	hakios_its_device_off(LX_ITS_DEVICE);
	lck_mtx_lock(&lx_pool_lock);
	if (lx_msi_doorbell && lx_pool.context != NULL) {
		(void)hakios_smmu_unmap(lx_pool.context, HAKIOS_WIFI_DOORBELL_IOVA, 4096);
	}
	lx_msi_doorbell = false;
	lck_mtx_unlock(&lx_pool_lock);
	lx_msi_on = false;
}

unsigned long long
lx_os_pci_msi_count(unsigned vector)
{
	return hakios_its_count(LX_ITS_DEVICE, vector);
}
