/*
 * Return-to-loader debug channel.
 *
 * On hardware without a serial port there is no way to get the kernel log
 * out. The XNU loader therefore leaves UEFI intact (no ExitBootServices) and
 * publishes the physical address of a return stub in /chosen/loader-return.
 * When the kernel has nothing more to do (panic, halt, timer) it collects its
 * message buffer, the panic buffer and any kprintf output, maps the stub 1:1
 * through TTBR0 and jumps to it with that log. The loader then restores the UEFI environment and writes
 * the log to storage.
 *
 * Only valid while the kernel does not drive hardware that UEFI also uses.
 */
#include <mach/mach_types.h>
#include <machine/atomic.h>
#include <kern/cpu_number.h>
#include <kern/debug.h>
#include <kern/startup.h>
#include <kern/thread_call.h>
#include <kern/clock.h>
#include <arm/cpu_internal.h>
#include <arm/machine_routines.h>
#include <machine/machine_routines.h>
#include <arm64/psci_smp.h>
#include <arm64/proc_reg.h>
#include <pexpert/pexpert.h>
#include <pexpert/arm64/board_config.h>
#include <vm/vm_page.h>
#include <vm/vm_compressor.h>

/*
 * The console log handed back to the loader. A long session writes more than fits, and both
 * ends matter: the start of the boot is kept as it is, and of everything after it the most
 * recent part (a ring); room is left for what is appended on the way out (the kernel message
 * buffer, a panic's text). loader_log_linearize() puts the pieces in order before returning.
 */
#define LOADER_LOG_SIZE (1536 * 1024)
#define LOADER_LOG_HEAD (256 * 1024)
#define LOADER_LOG_RING (1024 * 1024)

#if defined(GENERIC_GIC_V3_IRQ)
extern void gic_v3_irq_quiesce(void);
extern void gic_v3_irq_cpu_shutdown(void);
#endif

void loader_log_putc(char c);
void loader_return(void);
extern void hakios_adreno_quiesce(void);
extern void hakios_adreno_stop(void);
extern void hakios_adreno_minute(void);
extern void hakios_wifi_stop(void);
extern void hakios_wifi_quiesce(void);
extern void hakios_adsp_stop(void);
extern void hakios_adsp_quiesce(void);
extern void hakios_mpss_stop(void);
extern void hakios_mpss_quiesce(void);
extern void hakios_ddr_quiesce(void);
extern void hakios_haptic_quiesce(void);
extern void hakios_torch_quiesce(void);
extern void hakios_camera_quiesce(void);
extern void hakios_mic_quiesce(void);
extern void hakios_ufs_quiesce(void);
extern void hakios_usb_net_quiesce(void);      /* drivers/net/hakios.usb_net.c: the USB controller, if the kernel took it */
extern void hakios_var_save_on_return(void);   /* drivers/storage/hakios.ufs.c: the data volume into the flash */
void loader_checkpoint(uint64_t n);

/* Also referenced from the early exception vector in start.s. */
char     loader_log[LOADER_LOG_SIZE];
uint32_t loader_log_len;
static bool     loader_return_entered;

/* One L1 table and one L2 table are enough to map a single 2 MB block. */
static uint64_t loader_l1[512] __attribute__((aligned(4096)));
static uint64_t loader_l2[512] __attribute__((aligned(4096)));

static uint64_t loader_log_total;        /* characters ever put in: the live log's positions (loader_log_copy) */
static uint32_t loader_log_ring_at;      /* where the next character goes in the ring */
static bool     loader_log_wrapped;
static bool     loader_log_linear;       /* the pieces have been put in order: only appending from now on */
static char     loader_log_scratch[LOADER_LOG_RING];

/*
 * Characters come from every processor, and from interrupt handlers on top of whatever was
 * writing one. The lock is of the simplest kind, good from the first line of C code to the
 * last of a panic; whoever cannot have it for a long while (its holder is gone: a panic
 * inside this very function) goes ahead without it.
 */
static uint32_t loader_log_lock;

static void loader_log_put_locked(char c);

void
loader_log_putc(char c)
{
	uint64_t daif;
	bool locked = false;

	if (c == '\r') {
		return;
	}
	daif = __builtin_arm_rsr64("DAIF");
	__builtin_arm_wsr("DAIFSet", DAIFSC_IRQF | DAIFSC_FIQF);
	for (uint32_t tries = 0; !locked && tries < 10000000; tries++) {
		locked = os_atomic_cmpxchg(&loader_log_lock, 0, 1, acquire);
	}
	loader_log_put_locked(c);
	if (locked) {
		os_atomic_store(&loader_log_lock, 0, release);
	}
	__builtin_arm_wsr64("DAIF", daif);
}

/*
 * A whole line at once, under the lock: lines written from several processors at the same time
 * (tools/latency.py's "tl:" lines, a frame's on one and a touch's on another) are not mixed up.
 */
void loader_log_line(const char *line);

void
loader_log_line(const char *line)
{
	uint64_t daif;
	bool locked = false;

	daif = __builtin_arm_rsr64("DAIF");
	__builtin_arm_wsr("DAIFSet", DAIFSC_IRQF | DAIFSC_FIQF);
	for (uint32_t tries = 0; !locked && tries < 10000000; tries++) {
		locked = os_atomic_cmpxchg(&loader_log_lock, 0, 1, acquire);
	}
	for (; *line != 0; line++) {
		if (*line != '\r') {
			loader_log_put_locked(*line);
		}
	}
	if (locked) {
		os_atomic_store(&loader_log_lock, 0, release);
	}
	__builtin_arm_wsr64("DAIF", daif);
}

static void
loader_log_put_locked(char c)
{
	loader_log_total++;
	if (loader_log_linear || loader_log_len < LOADER_LOG_HEAD) {
		if (loader_log_len < LOADER_LOG_SIZE) {
			loader_log[loader_log_len++] = c;
		}
	} else {
		loader_log[LOADER_LOG_HEAD + loader_log_ring_at] = c;
		if (++loader_log_ring_at == LOADER_LOG_RING) {
			loader_log_ring_at = 0;
			loader_log_wrapped = true;
		}
		if (!loader_log_wrapped) {
			loader_log_len = LOADER_LOG_HEAD + loader_log_ring_at;
		}
	}
}

/*
 * The live log (drivers/net/hakios.usb_net.c, TCP 5140): characters from *pos on, at most max, *pos moved on.
 * Positions count every character ever put in: the head (the first LOADER_LOG_HEAD) stays as it is, the ring
 * keeps the last LOADER_LOG_RING of the rest; a reader that fell further behind than the ring (less a margin
 * for the writer) skips ahead. No lock: a character torn by a writer on another processor is the worst that
 * can happen, and the reader must never wait for one.
 */
uint32_t loader_log_copy(uint64_t *pos, char *out, uint32_t max);

uint32_t
loader_log_copy(uint64_t *pos, char *out, uint32_t max)
{
	uint64_t total = os_atomic_load(&loader_log_total, relaxed);
	uint32_t n = 0;

	if (loader_log_linear) {
		return 0;               /* (the run is ending: the pieces are being put in order) */
	}
	if (*pos >= LOADER_LOG_HEAD && total > *pos && total - *pos > LOADER_LOG_RING - 65536) {
		*pos = total - (LOADER_LOG_RING - 65536);
	}
	while (n < max && *pos < total) {
		out[n++] = *pos < LOADER_LOG_HEAD ? loader_log[*pos] :
		    loader_log[LOADER_LOG_HEAD + (*pos - LOADER_LOG_HEAD) % LOADER_LOG_RING];
		(*pos)++;
	}
	return n;
}

/* Oldest first: the head, a note about what was dropped, then the ring from its oldest character. */
static void
loader_log_linearize(void)
{
	static const char note[] = "\n\n[... the log between the first 256 KB and the last 1 MB was dropped ...]\n\n";
	uint32_t older = LOADER_LOG_RING - loader_log_ring_at, n = LOADER_LOG_HEAD;

	if (loader_log_linear) {
		return;
	}
	loader_log_linear = true;
	if (!loader_log_wrapped) {
		return;
	}
	__builtin_memcpy(loader_log_scratch, loader_log + LOADER_LOG_HEAD + loader_log_ring_at, older);
	__builtin_memcpy(loader_log_scratch + older, loader_log + LOADER_LOG_HEAD, loader_log_ring_at);
	/* the note takes the place of the oldest characters of the ring */
	__builtin_memcpy(loader_log + n, note, sizeof(note) - 1);
	n += sizeof(note) - 1;
	__builtin_memcpy(loader_log + n, loader_log_scratch + sizeof(note) - 1, LOADER_LOG_RING - (sizeof(note) - 1));
	loader_log_len = LOADER_LOG_HEAD + LOADER_LOG_RING;
}

/* Patched by the loader: { magic, PA of its return stub, checkpoint to stop at, PA - VA }. See start.s. */
extern uint64_t loader_stub_slot[4];

/*
 * For a loader that watches from another processor core while the kernel runs (it starts the
 * kernel on a core of its choice and stays on the first one): a sign of life, and where the log
 * is, so that the log of a kernel that hangs -- or dies before it has said a word through any
 * other channel -- is not lost with it. The loader finds this by its first word in the kernel
 * it has loaded. The addresses are the kernel's virtual ones, filled in by the linker and so
 * valid from the first instruction; the loader knows the distance to the physical ones.
 */
struct loader_live {
	uint64_t magic;                 /* "XNULIVE!" */
	uint64_t heartbeat;             /* goes up every HV_WDT_PET_SECONDS while the kernel is alive */
	const char *log;                /* loader_log */
	const uint32_t *log_len;        /* how much of it is in use */
	const uint32_t *log_ring_at;    /* where the ring part is being written */
	const bool *log_wrapped;        /* the ring has gone round */
	uint64_t log_head, log_ring;    /* sizes of the two parts */
	const uint64_t *recording;      /* loader_recording: { physical address, length } of a screen recording */
	volatile uint64_t stop;         /* the loader's to write: the host has asked for the run to end */
	uint64_t magic2;                /* "XNULIVE2": what follows is there (a loader may meet an older kernel) */
	const uint64_t *sound;          /* loader_sound: the same for a recording of the sound */
};

/*
 * A screen recording (the display driver makes one with boot-arg rec_secs): where it is, for the
 * loader to hand to the host once the kernel is back. { physical address, bytes, kernel address }.
 */
uint64_t loader_recording[3];
/* ... and of the sound (/dev/hakios-audio makes one with boot-arg rec_sound). */
uint64_t loader_sound[3];

struct loader_live loader_live = {
	.magic = 0x214556494c554e58ULL, .log = loader_log, .log_len = &loader_log_len, .log_ring_at = &loader_log_ring_at,
	.log_wrapped = &loader_log_wrapped, .log_head = LOADER_LOG_HEAD, .log_ring = LOADER_LOG_RING,
	.recording = loader_recording,
	.magic2 = 0x324556494c554e58ULL, .sound = loader_sound,
};

/* Valid from the first instruction of C code: does not depend on gVirtBase/gPhysBase being set up. */
static inline uint64_t
static_kvtophys(const void *va)
{
	return (uint64_t)va + loader_stub_slot[3];
}

/*
 * Jump to the loader's return stub. reason: 0 = orderly end, 2 = checkpoint.
 * Returns only if no loader is listening.
 */
static void
loader_jump(uint64_t reason, uint64_t code, bool lean)
{
#if ARM_PGSHIFT == 12
	uint64_t stub = loader_stub_slot[1];
	uint64_t tcr, t0sz;
	uint32_t own_core = 0;

	if (stub == 0 || loader_return_entered) {
		return;
	}

	tcr = __builtin_arm_rsr64("TCR_EL1");
	t0sz = tcr & 0x3f;
	/* The TTBR0 walk must start at L1 (1 GB per entry) and reach the stub. */
	if (t0sz < 25 || t0sz > 33 || (stub >> (64 - t0sz)) != 0) {
		return;
	}
	loader_return_entered = true;
	/*
	 * A "lean" jump is for the earliest checkpoints, before PE_init_platform(): the interrupt
	 * controller has not been set up and the boot arguments cannot be parsed yet, so touching
	 * either here could itself be what stops the run (and say nothing).
	 */
#if defined(GENERIC_GIC_V3_IRQ)
	if (!lean) {
	gic_v3_irq_quiesce();
	/*
	 * With a core of its own (bootcpu=) the kernel does not go back into the loader's code on
	 * this processor: the stub leaves the log where the loader's core finds it and has the
	 * firmware switch this one off. It must go as the other cores do (psci_smp_cpu_off()), with
	 * nothing pending: after a panic the "stop" signal and the timer are.
	 */
	if (PE_parse_boot_argn("bootcpu", &own_core, sizeof(own_core))) {
		__builtin_arm_wsr64("CNTV_CTL_EL0", CNTV_CTL_EL0_IMASKED);
		gic_v3_irq_cpu_shutdown();
	}
	}
#endif

	loader_l1[stub >> 30] = static_kvtophys(loader_l2) | ARM_TTE_TYPE_TABLE | ARM_TTE_VALID;
	loader_l2[(stub >> 21) & 0x1ff] = (stub & ~0x1fffffULL) | ARM_TTE_VALID | ARM_TTE_TYPE_BLOCK |
	    ARM_TTE_BLOCK_AF | ARM_TTE_BLOCK_SH(SH_OUTER_MEMORY) | ARM_TTE_BLOCK_AP(AP_RWNA) |
	    ARM_TTE_BLOCK_ATTRINDX(CACHE_ATTRINDX_WRITEBACK);

	/*
	 * The stub turns the MMU off, and on real hardware dirty cache lines do
	 * not survive that: write the log back to RAM first.
	 */
	for (uintptr_t p = (uintptr_t)loader_log & ~63UL; p < (uintptr_t)loader_log + loader_log_len; p += 64) {
		__asm__ volatile ("dc cvac, %0" : : "r" (p));
	}
	/* ... and a screen recording, and the note of where it is */
	for (uintptr_t p = loader_recording[2] & ~63UL; p < loader_recording[2] + loader_recording[1]; p += 64) {
		__asm__ volatile ("dc cvac, %0" : : "r" (p));
	}
	__asm__ volatile ("dc cvac, %0" : : "r" ((uintptr_t)loader_recording));
	__asm__ volatile ("dc cvac, %0" : : "r" ((uintptr_t)loader_recording + 16));
	for (uintptr_t p = loader_sound[2] & ~63UL; p < loader_sound[2] + loader_sound[1]; p += 64) {
		__asm__ volatile ("dc cvac, %0" : : "r" (p));
	}
	__asm__ volatile ("dc cvac, %0" : : "r" ((uintptr_t)loader_sound));
	__asm__ volatile ("dc cvac, %0" : : "r" ((uintptr_t)loader_sound + 16));
	__asm__ volatile ("dsb sy");

	register uint64_t r_ttbr0 __asm__("x0") = static_kvtophys(loader_l1);
	register uint64_t r_log __asm__("x2") = static_kvtophys(loader_log);
	register uint64_t r_len __asm__("x3") = loader_log_len;
	register uint64_t r_tcr __asm__("x4") = tcr;
	register uint64_t r_stub __asm__("x5") = stub;
	register uint64_t r_reason __asm__("x6") = reason;
	register uint64_t r_code __asm__("x7") = code;

	__asm__ volatile (
                "msr DAIFSet, #0xf\n"
                "bic x4, x4, #0x80\n"           /* TCR_EL1.EPD0: allow TTBR0 walks */
                "msr TCR_EL1, x4\n"
                "msr TTBR0_EL1, x0\n"
                "isb\n"
                "tlbi vmalle1\n"
                "dsb sy\n"
                "isb\n"
                "mov x0, x2\n"
                "mov x1, x3\n"
                "mov x2, x6\n"
                "mov x3, x7\n"
                "br x5\n"
                : : "r" (r_ttbr0), "r" (r_log), "r" (r_len), "r" (r_tcr), "r" (r_stub), "r" (r_reason), "r" (r_code));
	__builtin_unreachable();
#else
	(void)reason; (void)code; (void)lean;
#endif /* ARM_PGSHIFT == 12 */
}

extern char *debug_buf_base, *debug_buf_ptr;

/* bsd/sys/msgbuf.h (not includable from osfmk) */
struct msgbuf {
	int   msg_magic;
	int   msg_size;
	int   msg_bufx;         /* write pointer */
	int   msg_bufr;         /* read pointer */
	char *msg_bufc;         /* buffer */
};
extern struct msgbuf *msgbufp;

static void
loader_log_puts(const char *s)
{
	for (; *s; s++) {
		loader_log_putc(*s);
	}
}

/*
 * Everything printf()/IOLog()/os_log wrote ends up in the kernel message
 * buffer (what dmesg shows), whether or not a console displayed it.
 */
static void
loader_log_append_msgbuf(void)
{
	struct msgbuf *mb = msgbufp;

	if (mb == NULL || mb->msg_bufc == NULL || mb->msg_size <= 0 || mb->msg_bufx < 0 || mb->msg_bufx > mb->msg_size) {
		return;
	}
	loader_log_puts("\n--- kernel message buffer ---\n");
	/* oldest data first: [bufx, size) is only meaningful once the buffer has wrapped */
	for (int pass = 0; pass < 2; pass++) {
		int from = pass == 0 ? mb->msg_bufx : 0;
		int to = pass == 0 ? mb->msg_size : mb->msg_bufx;
		for (int i = from; i < to; i++) {
			if (mb->msg_bufc[i] != 0) {
				loader_log_putc(mb->msg_bufc[i]);
			}
		}
	}
}

/*
 * reason 0: back to the loader, which waits for the host's next command. reason 3: the same, and the
 * loader then resets the phone (Android again): what "sweet-sh.py reboot" asks for.
 */
/*
 * The reason asked for, kept where the boot processor finds it: when another processor asks,
 * the boot processor makes the return with loader_return(), which knows no reason of its own.
 * The highest asked for wins (a reset asked for is not lost to a plain return after it).
 * Before this, sweet-sh.py reboot with cpus=8 left the phone in the loader (2026-11-09).
 */
static uint64_t loader_reason_wanted;

static void
loader_return_reason(uint64_t reason)
{
	if (reason > __atomic_load_n(&loader_reason_wanted, __ATOMIC_ACQUIRE)) {
		__atomic_store_n(&loader_reason_wanted, reason, __ATOMIC_RELEASE);
	}
#if defined(GENERIC_PSCI_SMP)
	/*
	 * With more than one processor: the others switch themselves off first, and the rest of
	 * this is done by the boot processor, the one the loader gave us. This call comes back
	 * on that processor only (on another one it has it call loader_return() in its turn).
	 */
	psci_smp_stop_others();
#endif
	reason = __atomic_load_n(&loader_reason_wanted, __ATOMIC_ACQUIRE);
	hakios_usb_net_quiesce();       /* the USB controller stopped, if usb_net=1 took it from the loader's UEFI */
	hakios_ufs_quiesce();          /* this kernel's requests to the storage done, UEFI's list as it was, for the log */
	hakios_adreno_quiesce();       /* the GPU block, if it was started, has tables in memory that is about to be reused */
	hakios_wifi_quiesce();         /* the Wi-Fi DSP, if it still runs (a panic), is not left to itself */
	hakios_adsp_quiesce();         /* nor the audio DSP */
	hakios_mpss_quiesce();         /* nor the modem */
	hakios_ddr_quiesce();          /* and the vote for the memory's bandwidth taken away */
	hakios_haptic_quiesce();       /* the vibration motor's chip back into reset, if it was let out */
	hakios_torch_quiesce();        /* the flashlight off, if it was lit */
	hakios_camera_quiesce();       /* the camera powered down, if it was powered up */
	hakios_mic_quiesce();          /* (what the microphones' path did, said again: it happens in the log's middle) */
	/*
	 * A panic's text is first collected in the panic buffer and only later
	 * flushed to the console; if the console itself is what failed, that
	 * never happens. Append the buffer so the loader gets it either way.
	 */
	loader_log_linearize();
	if (!loader_return_entered) {
		loader_log_append_msgbuf();
	}
	if (!loader_return_entered && debug_buf_base != NULL && debug_buf_ptr > debug_buf_base &&
	    (size_t)(debug_buf_ptr - debug_buf_base) < 256 * 1024) {
		loader_log_puts("\n--- panic buffer ---\n");
		for (const char *c = debug_buf_base; c < debug_buf_ptr; c++) {
			loader_log_putc(*c);
		}
	}
	loader_jump(reason, 0, false);
}

void
loader_return(void)
{
	loader_return_reason(0);
}

/*
 * Bring-up aid: stop here if the loader asked for checkpoint n. Numbers below 11 are before
 * the platform expert exists (arm_init() up to PE_init_platform()), so they use the lean jump.
 */
void
loader_checkpoint(uint64_t n)
{
	if (loader_stub_slot[2] == n) {
		loader_jump(2, n, n < 11);
	}
}

/*
 * boot-arg loader_return_after=<seconds>: hand control back to the loader
 * after that long, whatever the kernel is doing. This is how the log gets out
 * when the kernel does not panic but simply waits (e.g. for a root device).
 */
/*
 * For trying the ways out when there is more than one processor: boot-arg return_cpu=<n> has
 * that processor make the timed return, from an interrupt handler -- any but the boot processor
 * has to hand it over (psci_smp_stop_others()) --, and return_panic=1 makes a panic of it.
 */
static void
loader_return_test(void *as_panic)
{
	if (as_panic != (void *)1) {
		panic("return_panic: the panic that was asked for, on cpu %d", cpu_number());
	}
	loader_return();
}

static void
loader_return_timer(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	static const char banner[] = "\n--- loader_return_after expired ---\n";
	uint32_t cpu = 0, as_panic = 0;

	for (const char *c = banner; *c; c++) {
		loader_log_putc(*c);
	}
	PE_parse_boot_argn("return_panic", &as_panic, sizeof(as_panic));
	if (!as_panic) {
		hakios_var_save_on_return();   /* the data volume kept, while the system is still whole */
		hakios_adreno_stop();          /* a GPU kept for the system is given back while there is a thread to wait in */
		hakios_wifi_stop();            /* and the Wi-Fi DSP, if it runs, shut down */
		hakios_adsp_stop();            /* and the audio DSP */
		hakios_mpss_stop();            /* and the modem */
	}
	if (PE_parse_boot_argn("return_cpu", &cpu, sizeof(cpu)) && cpu <= (uint32_t)ml_get_max_cpu_number() &&
	    cpu_xcall((int)cpu, loader_return_test, (void *)(uintptr_t)(as_panic ? 2 : 1)) == KERN_SUCCESS) {
		for (;;) {
			delay_for_interval(1000, NSEC_PER_MSEC);        /* that processor ends it */
		}
	}
	loader_return_test((void *)(uintptr_t)(as_panic ? 2 : 1));
}

__startup_func
static void
loader_return_timer_init(void)
{
	uint32_t seconds = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("loader_return_after", &seconds, sizeof(seconds)) || seconds == 0) {
		return;
	}
	thread_call_t call = thread_call_allocate(loader_return_timer, NULL);
	clock_interval_to_deadline(seconds, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, loader_return_timer_init);

/*
 * A loader that watches from another core also listens to the host, and passes on a request to
 * end the run ("xl stop") by setting loader_live.stop. Look once a second.
 */
static thread_call_t loader_stop_call;

/*
 * kern.hakios.loader_request (hakios.platform.c), written by sweetd for "sweet-sh.py loader|reboot":
 * 1 = back to the loader and stay there, 2 = back to the loader, which then resets the phone.
 * The poll below, once a second, does the work: a system call must not be the thread that stops the system.
 */
static volatile int loader_request;

void
loader_request_return(int mode)
{
	if (mode == 1 || mode == 2) {
		loader_request = mode;
		if (loader_stop_call != NULL) {
			thread_call_enter(loader_stop_call);    /* (now, rather than at the next look) */
		}
	}
}

static void
loader_stop_poll(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint64_t deadline;
	int request = loader_request;

	if (loader_live.stop || request) {
		const char *banner = request == 2 ? "\n--- the system asked for a reset: back to the loader, then Android ---\n" :
		    request == 1 ? "\n--- the system asked for the loader ---\n" :
		    "\n--- the host asked for the end of the run: back to the loader ---\n";
		for (const char *c = banner; *c; c++) {
			loader_log_putc(*c);
		}
		hakios_var_save_on_return();
		hakios_adreno_stop();
		hakios_wifi_stop();
		hakios_adsp_stop();
		hakios_mpss_stop();
		loader_return_reason(request == 2 ? 3 : 0);
	}
	/* (a look a second, or up to 2 s later with another wake: on its own it was one of cpu0's wakes, run22) */
	uint64_t leeway;
	clock_interval_to_deadline(1, NSEC_PER_SEC, &deadline);
	nanoseconds_to_absolutetime(2 * NSEC_PER_SEC, &leeway);
	thread_call_enter_delayed_with_leeway(loader_stop_call, NULL, deadline, leeway, THREAD_CALL_DELAY_LEEWAY);
}

__startup_func
static void
loader_stop_poll_init(void)
{
	loader_stop_call = thread_call_allocate(loader_stop_poll, NULL);
	loader_stop_poll(NULL, NULL);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, loader_stop_poll_init);

/*
 * boot-arg hv_wdt_pet=1: the loader has armed the Qualcomm hypervisor's
 * virtual watchdog (it resets the phone when a kernel hangs, so that test
 * runs need nobody at the device). Its timeout is at most 65 seconds; feed
 * it from a timer so that a healthy kernel can run for as long as it likes.
 */
#define VIRT_WDT_PET            0x86000007u
#define HV_WDT_PET_SECONDS      10

static thread_call_t hv_wdt_call;
static uint32_t hv_wdt_enabled;

#define MEMORY_REPORT_BEATS     6               /* a line about memory every so many heartbeats */

extern unsigned int memorystatus_available_pages, memorystatus_available_pages_critical,
    memorystatus_available_pages_critical_base;
extern uint32_t c_segment_pages_compressed;
extern _Atomic uint64_t ramfs_pages_in_use;
extern void MiniIOSurfaceUsage(uint32_t *count, uint64_t *bytes);

/*
 * Where the memory is, in megabytes: for reading a session's log afterwards. "available" is what
 * the kernel's own count says (free and file-backed): below the first threshold it kills idle
 * daemons, and once none is left and it is below the second, whatever is next in line -- the
 * window server included.
 */
void hakios_memory_line(char *buf, size_t len);

/* The line itself (also kern.hakios_memory, for a job list to keep: bsd/miscfs/apfs/apfs_vfsops.c) */
void
hakios_memory_line(char *buf, size_t len)
{
#define MB(pages)       ((unsigned int)(((uint64_t)(pages) * PAGE_SIZE) >> 20))
	uint32_t surfaces = 0;
	uint64_t surface_bytes = 0;

	MiniIOSurfaceUsage(&surfaces, &surface_bytes);
	snprintf(buf, len, "memory: free %u, files %u, programs %u, wired %u (%u surfaces %u, ramfs %u), compressor %u "
	    "(holding %u) MB; available %u, kills below %u (the idle) and %u (anybody)\n",
	    MB(vm_page_free_count), MB(vm_page_pageable_external_count), MB(vm_page_pageable_internal_count),
	    MB(vm_page_wire_count), surfaces, (unsigned int)(surface_bytes >> 20),
	    MB(os_atomic_load(&ramfs_pages_in_use, relaxed)),
	    MB(VM_PAGE_COMPRESSOR_COUNT), MB(c_segment_pages_compressed),
	    MB(memorystatus_available_pages),
	    MB(memorystatus_available_pages_critical), MB(memorystatus_available_pages_critical_base));
#undef MB
}

static void
memory_report(void)
{
	char line[256];

	hakios_memory_line(line, sizeof(line));
	printf("%s", line);
	hakios_adreno_minute();                /* (and how warm the GPU is, if it is in use) */
}

/* Every HV_WDT_PET_SECONDS: feed the watchdog if there is one, and show any watching loader a sign of life. */
static void
hv_wdt_pet(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint64_t deadline;

	if (hv_wdt_enabled) {
		register uint64_t x0 __asm__("x0") = VIRT_WDT_PET;
		register uint64_t x1 __asm__("x1") = 0;
		register uint64_t x2 __asm__("x2") = 0;
		register uint64_t x3 __asm__("x3") = 0;

		__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : :
		    "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17");
	}
	if (++loader_live.heartbeat % MEMORY_REPORT_BEATS == 0) {
		memory_report();
	}

	clock_interval_to_deadline(HV_WDT_PET_SECONDS, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(hv_wdt_call, deadline);
}

__startup_func
static void
hv_wdt_pet_init(void)
{
	PE_parse_boot_argn("hv_wdt_pet", &hv_wdt_enabled, sizeof(hv_wdt_enabled));
	hv_wdt_call = thread_call_allocate(hv_wdt_pet, NULL);
	hv_wdt_pet(NULL, NULL);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, hv_wdt_pet_init);

/*
 * Hang detector (boot-arg hang_detect=0 turns it off). A hung kernel otherwise meets the hypervisor's
 * watchdog, which resets the phone and takes the log with it. Every HANG_TICK_MS a timer on each processor
 * notes the time and where that processor was interrupted, and a thread call notes that threads still run.
 * When the thread call, or a running processor's timer, has been quiet for HANG_LIMIT_MS, the next timer
 * to run panics with every processor's last interrupted place: the panic goes to the loader with the log.
 * It starts HANG_START_SECONDS after boot. A processor switched off for a while (psci_smp.c's screen_off_cores)
 * has no timer meanwhile, and gets one again when it is back.
 */
#include <kern/timer_call.h>
#include <arm/cpu_data_internal.h>

#define HANG_TICK_MS            500
#define HANG_LIMIT_MS           5000
#define HANG_LEEWAY_MS          1500
#define HANG_START_SECONDS      30
#define HANG_FRAMES             4

struct hang_cpu {
	_Atomic uint64_t at;            /* when its timer last ran (0: not yet) */
	uint64_t pc, lr, frame[HANG_FRAMES];
	uint32_t ticks;
	uint8_t user;
};

extern vm_offset_t vm_kernel_slide;

static struct hang_cpu hang_cpus[MAX_CPUS];
static struct timer_call hang_timers[MAX_CPUS];
static thread_call_t hang_thread_call, hang_start_call;
static _Atomic uint64_t hang_thread_at;
static uint64_t hang_tick_abs, hang_limit_abs, hang_leeway_abs;
static uint32_t hang_detect = 1;
static _Atomic uint32_t hang_fired;
static _Atomic uint32_t hang_off;       /* a bit per processor that is switched off, or on its way */
static char hang_text[2048];

/* where this processor was when the timer interrupted it: pc, lr and the first few return addresses */
static void
hang_note(struct hang_cpu *c)
{
	arm_saved_state_t *st = getCpuDatap()->cpu_int_state;

	c->pc = c->lr = 0;
	for (int i = 0; i < HANG_FRAMES; i++) {
		c->frame[i] = 0;
	}
	if (st == NULL) {
		return;
	}
	c->user = PSR64_IS_USER(get_saved_state_cpsr(st)) ? 1 : 0;
	c->pc = get_saved_state_pc(st);
	c->lr = get_saved_state_lr(st);
	if (c->user) {
		return;
	}
	uint64_t fp = get_saved_state_fp(st);
	for (int i = 0; i < HANG_FRAMES; i++) {
		uint64_t pair[2];

		if (fp < VM_MIN_KERNEL_ADDRESS || (fp & 0x7) != 0 ||
		    ml_nofault_copy((vm_offset_t)fp, (vm_offset_t)pair, sizeof(pair)) != sizeof(pair)) {
			break;
		}
		c->frame[i] = pair[1];
		fp = pair[0];
	}
}

static void
hang_check(uint64_t now, int self)
{
	uint64_t t = os_atomic_load(&hang_thread_at, acquire);
	const char *why = NULL;
	int quiet = -1;
	size_t n = 0;

	if (t != 0 && now > t && now - t > hang_limit_abs) {
		why = "no thread call has run";
	}
	for (int i = 0; why == NULL && i <= ml_get_max_cpu_number() && i < MAX_CPUS; i++) {
		cpu_data_t *cd = (cpu_data_t *)CpuDataEntries[i].cpu_data_vaddr;
		uint64_t a = os_atomic_load(&hang_cpus[i].at, acquire);

		if (i != self && cd != NULL && cd->cpu_running && a != 0 && now > a && now - a > hang_limit_abs &&
		    !(os_atomic_load(&hang_off, acquire) & (1u << i))) {
			why = "a processor's timer has not run";
			quiet = i;
		}
	}
	if (why == NULL || os_atomic_xchg(&hang_fired, 1, relaxed) != 0) {
		return;
	}
	for (int i = 0; i <= ml_get_max_cpu_number() && i < MAX_CPUS && n < sizeof(hang_text) - 200; i++) {
		struct hang_cpu *c = &hang_cpus[i];
		uint64_t a = os_atomic_load(&c->at, relaxed), ago_ns = 0;

		if (a != 0 && now > a) {
			absolutetime_to_nanoseconds(now - a, &ago_ns);
		}
		n += (size_t)snprintf(hang_text + n, sizeof(hang_text) - n,
		    "hang: cpu %d%s: last tick %llu ms ago (%u ticks), %s pc 0x%llx lr 0x%llx frames 0x%llx 0x%llx 0x%llx 0x%llx\n",
		    i, i == self ? " (this one)" : "", a ? ago_ns / NSEC_PER_MSEC : 0ull, c->ticks, c->user ? "user" : "kernel",
		    c->pc, c->lr, c->frame[0], c->frame[1], c->frame[2], c->frame[3]);
	}
	uint64_t thread_ns = 0;
	if (t != 0 && now > t) {
		absolutetime_to_nanoseconds(now - t, &thread_ns);
	}
	panic("hang detector: %s for over %u ms (noticed on cpu %d, quiet cpu %d, last thread call %llu ms ago, "
	    "slide 0x%lx)\n%s", why, HANG_LIMIT_MS, self, quiet, thread_ns / NSEC_PER_MSEC, (unsigned long)vm_kernel_slide,
	    hang_text);
}

/*
 * A processor's next tick: HANG_TICK_MS from now, or up to HANG_LEEWAY_MS later, when that processor wakes for
 * something else (XNU's timer coalescing). A tick of its own on each idle core woke the SoC out of CX collapse every
 * 500 ms per core (run16: 2 a second on each of cores 1-3); at most HANG_TICK_MS + HANG_LEEWAY_MS apart, well inside
 * HANG_LIMIT_MS.
 */
static void
hang_timer_enter(timer_call_t call, uint64_t now)
{
	timer_call_enter_with_leeway(call, NULL, now + hang_tick_abs, hang_leeway_abs,
	    TIMER_CALL_SYS_CRITICAL | TIMER_CALL_LOCAL | TIMER_CALL_LEEWAY, FALSE);
}

static void
hang_tick(timer_call_param_t p0, __unused timer_call_param_t p1)
{
	int cpu = cpu_number();
	uint64_t now = mach_absolute_time();
	struct hang_cpu *c = &hang_cpus[cpu];

	hang_note(c);
	c->ticks++;
	os_atomic_store(&c->at, now, release);
	hang_check(now, cpu);
	hang_timer_enter(&hang_timers[(uintptr_t)p0], now);
	if (os_atomic_load(&hang_off, seq_cst) & (1u << cpu)) {
		/* it is leaving: no local timer may be left on it (timer_queue_shutdown()) */
		timer_call_cancel(&hang_timers[(uintptr_t)p0]);
	}
}

/*
 * A processor leaves for a while (processor_exit(): on the processor that asks) or is back (GenericARMCPU::initCPU(),
 * on the processor itself, interrupts off).
 */
static bool
hang_cpu_event(__unused void *param, enum cpu_event event, unsigned int cpu)
{
	if (cpu >= MAX_CPUS) {
		return true;
	}
	if (event == CPU_EXIT_REQUESTED) {
		os_atomic_or(&hang_off, 1u << cpu, seq_cst);
		os_atomic_store(&hang_cpus[cpu].at, 0, release);
		timer_call_cancel(&hang_timers[cpu]);
	} else if (event == CPU_BOOTED && (int)cpu == cpu_number()) {
		os_atomic_store(&hang_cpus[cpu].at, mach_absolute_time(), release);
		os_atomic_andnot(&hang_off, 1u << cpu, seq_cst);
		hang_timer_enter(&hang_timers[cpu], mach_absolute_time());
	}
	return true;
}

/* (in each processor's interrupt context, from the broadcast) */
static void
hang_arm(__unused void *arg)
{
	int cpu = cpu_number();

	hang_timer_enter(&hang_timers[cpu], mach_absolute_time());
}

static void
hang_thread_beat(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint64_t now = mach_absolute_time();

	os_atomic_store(&hang_thread_at, now, release);
	/* (with the per-processor ticks' leeway: 2 a second of cpu0's wakes otherwise, run22; still well inside the limit) */
	thread_call_enter_delayed_with_leeway(hang_thread_call, NULL, now + hang_tick_abs, hang_leeway_abs,
	    THREAD_CALL_DELAY_LEEWAY);
}

static void
hang_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	hang_thread_beat(NULL, NULL);
	cpu_event_register_callback(hang_cpu_event, NULL);
	unsigned int armed = cpu_broadcast_xcall_simple(TRUE, hang_arm, NULL);
	printf("hang detector: on, %u processors, a tick every %u ms, panics after %u ms quiet\n", armed, HANG_TICK_MS,
	    HANG_LIMIT_MS);
}

__startup_func
static void
hang_detect_init(void)
{
	uint64_t deadline;

	PE_parse_boot_argn("hang_detect", &hang_detect, sizeof(hang_detect));
	if (!hang_detect) {
		return;
	}
	nanoseconds_to_absolutetime((uint64_t)HANG_TICK_MS * NSEC_PER_MSEC, &hang_tick_abs);
	nanoseconds_to_absolutetime((uint64_t)HANG_LEEWAY_MS * NSEC_PER_MSEC, &hang_leeway_abs);
	nanoseconds_to_absolutetime((uint64_t)HANG_LIMIT_MS * NSEC_PER_MSEC, &hang_limit_abs);
	for (uintptr_t i = 0; i < MAX_CPUS; i++) {
		timer_call_setup(&hang_timers[i], hang_tick, (timer_call_param_t)i);
	}
	hang_thread_call = thread_call_allocate(hang_thread_beat, NULL);
	hang_start_call = thread_call_allocate(hang_start, NULL);
	clock_interval_to_deadline(HANG_START_SECONDS, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(hang_start_call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, hang_detect_init);
