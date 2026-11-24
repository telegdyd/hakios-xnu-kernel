/*
 * More than one processor on generic (non-Apple) ARMv8 boards.
 *
 * On Apple hardware the other cores are powered up by closed drivers. Here
 * the firmware does it, through the standard power interface of ARM systems
 * (PSCI): CPU_ON starts a core at an address of the caller's choice, CPU_OFF
 * stops the calling core for good. The address we give is the kernel's own
 * reset vector (LowResetVectorBase in start.s), so that a new core takes the
 * path Apple's cores take: it finds its cpu data by its MPIDR, turns the MMU
 * on and comes out in arm_init_cpu(). The calls are made with SMC or, under
 * a hypervisor that implements them itself, with HVC; the boot loader says
 * which (/chosen/psci-method).
 *
 * The platform expert (iokit/Kernel/GenericARMPE.cpp) registers the cores of
 * the device tree and starts them here, one after the other. Signals between
 * processors are interrupts of the GIC (gic_v3_irq.c).
 *
 * A new core begins with its MMU and caches off, and what it reads then it
 * reads from memory. On real hardware that is not the same as what the
 * running processors see: under Qualcomm's hypervisor (the phone this was
 * written for) a line that is dirty in another core's cache is simply not
 * there yet. QEMU has no cache and shows none of this. So before CPU_ON
 * everything that core will read before its MMU is on is written back to
 * memory (psci_smp_start_cpu()), and what it writes in that state is read
 * only after throwing away the reader's cached copy.
 *
 * The other half is the way out. The kernel ends by returning to the boot
 * loader (loader_return.c), and the loader gets back only what it started:
 * one core, the boot processor's. psci_smp_stop_others() makes every other
 * core switch itself off first, wherever it is.
 *
 * In between, a core may be switched off for a while and started again:
 * while the display is off, the cores of the boot-arg screen_off_cores
 * (psci_smp_display()). The scheduler moves their work away and back
 * (processor_exit() -> psci_smp_cpu_unplug(), processor_start() ->
 * psci_smp_start_cpu()).
 */
#include <mach/mach_types.h>
#include <kern/clock.h>
#include <kern/cpu_number.h>
#include <kern/debug.h>
#include <kern/kalloc.h>
#include <kern/misc_protos.h>
#include <kern/processor.h>
#include <kern/startup.h>
#include <kern/task.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <kern/recount.h>
#include <mach/thread_info.h>
#include <libkern/kernel_mach_header.h>
#include <machine/atomic.h>
#include <arm/caches_internal.h>
#include <arm/cpu_data_internal.h>
#include <arm/cpu_internal.h>
#include <arm/machine_routines.h>
#include <arm64/proc_reg.h>
#include <arm64/psci_smp.h>
#include <pexpert/pexpert.h>
#include <pexpert/device_tree.h>
#include <pexpert/arm64/board_config.h>
#include <vm/pmap.h>

/* drivers/power/hakios.rpmh-arc.h (with hakios.cpufreq.c): absent in kernels without it */
extern void hakios_rpmh_llcc_down(void) __attribute__((weak));

#if defined(GENERIC_PSCI_SMP)

#define PSCI_CPU_OFF            0x84000002u
#define PSCI_CPU_ON             0xC4000003u
#define PSCI_AFFINITY_INFO      0xC4000004u
#define PSCI_AFFINITY_OFF       1
#define PSCI_ALREADY_ON         (-4)
#define PSCI_ON_PENDING         (-5)

#define PSCI_SMP_START_SECONDS  5       /* how long a started core is waited for */
#define PSCI_SMP_GONE_MS        1000    /* ... and one that switched itself off, to be off before it is started again */
#define PSCI_SMP_STOP_SECONDS   3       /* ... and the cores that were told to stop */

extern void loader_return(void);
extern void loader_log_putc(char c);
extern unsigned long gVirtBase, gPhysBase;

/* The firmware is called with HVC, not SMC. (start.s reads this too, at its physical address.) */
uint32_t psci_smp_hvc;

/*
 * Left by a core that took an exception between its first instruction and the kernel's vectors
 * (Lsecondary_early_exception in start.s). That core writes it with its cache off; it has a
 * cache line to itself so that looking at memory does not disturb anything else.
 */
struct psci_smp_start_fault {
	uint64_t valid, mpidr, esr, elr, far, lr, sp, unused;
} psci_smp_start_fault __attribute__((aligned(64)));
_Static_assert(sizeof(struct psci_smp_start_fault) == 64, "one cache line");

static uint32_t psci_smp_cpu_stage[MAX_CPUS];   /* PSCI_SMP_CPU_*, by cpu number */
static uint32_t psci_smp_started;               /* a bit per core that CPU_ON was granted for */

static int      psci_smp_stop_owner = -1;       /* the processor that began the return to the boot loader */
static bool     psci_smp_boot_wanted;           /* it was not the boot processor: that one is to finish */
static bool     psci_smp_boot_busy;             /* ... and has begun to */
static bool     psci_smp_stopping;              /* every other core switches itself off */

static int64_t
psci_call(uint64_t function, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
	register uint64_t x0 __asm__("x0") = function;
	register uint64_t x1 __asm__("x1") = arg1;
	register uint64_t x2 __asm__("x2") = arg2;
	register uint64_t x3 __asm__("x3") = arg3;

	if (psci_smp_hvc) {
		__asm__ volatile ("hvc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : :
                    "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17");
	} else {
		__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : :
                    "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17");
	}
	return (int64_t)x0;
}

static const char *
psci_error(int64_t status)
{
	static const char *const names[] = {
		"success", "not supported", "invalid parameters", "denied", "already on", "on pending",
		"internal failure", "not present", "disabled", "invalid address",
	};

	return (status <= 0 && status > -(int64_t)(sizeof(names) / sizeof(names[0]))) ? names[-status] : "?";
}

void
psci_smp_init(void)
{
	DTEntry chosen;
	void const *prop;
	unsigned int size;

	if (SecureDTLookupEntry(NULL, "/chosen", &chosen) == kSuccess &&
	    SecureDTGetProperty(chosen, "psci-method", &prop, &size) == kSuccess &&
	    size >= 3 && strncmp((const char *)prop, "hvc", 3) == 0) {
		psci_smp_hvc = 1;
	}
}

/* A processor other than the boot processor says how far it has come. */
void
psci_smp_cpu_reached(uint32_t stage)
{
	os_atomic_store(&psci_smp_cpu_stage[cpu_number()], stage, release);
}

/* ... all the way: from GenericARMCPU::initCPU(), where the scheduler takes it. */
void
psci_smp_cpu_running(void)
{
	psci_smp_cpu_reached(PSCI_SMP_CPU_RUNNING);
}

static void
psci_smp_clean_segment(const char *name)
{
	unsigned long size = 0;
	vm_offset_t start = (vm_offset_t)getsegdatafromheader(&_mh_execute_header, name, &size);

	if (start != 0 && size != 0) {
		CleanPoC_DcacheRegion(start, size);
	}
}

/*
 * What a core reads with its MMU off on its way in at the reset vector (a start, or the way back from a power-down in
 * idle): the code and the page tables of start.s, and what the reset vector finds its way by (ResetHandlerData,
 * CpuDataEntries, the boot arguments, cpu_ttep, psci_smp_hvc ...), all of it in the kernel's own segments; and the cpu
 * data of every processor, which the reset vector goes through looking for its own. Clean, not flush: the processors
 * that are running keep using all of it.
 */
static void
psci_smp_clean_for_reset(void)
{
	psci_smp_clean_segment("__TEXT");
	psci_smp_clean_segment("__DATA_CONST");
	psci_smp_clean_segment("__TEXT_EXEC");
	psci_smp_clean_segment("__DATA");
	psci_smp_clean_segment("__BOOTDATA");
	for (int other = 0; other <= ml_get_max_cpu_number(); other++) {
		if (cpu_datap(other) != NULL) {
			CleanPoC_DcacheRegion((vm_offset_t)cpu_datap(other), sizeof(cpu_data_t));
		}
	}
	__builtin_arm_dsb(DSB_SY);
}

/*
 * Start a registered processor: the platform's half of cpu_start(). Waits until the core has
 * joined the scheduler, and says what became of it if it has not: on a phone the log is all
 * there is to go by.
 */
kern_return_t
psci_smp_start_cpu(int cpu)
{
	cpu_data_t *cdp = cpu_datap(cpu);
	uint32_t phys_id = cdp->cpu_phys_id, stage;
	uint64_t entry = ml_static_vtop((vm_offset_t)&LowResetVectorBase);
	uint64_t now, deadline;
	int64_t status;

	psci_smp_start_fault.valid = 0;
	psci_smp_clean_for_reset();

	/*
	 * A core that was switched off for a while (psci_smp_cpu_unplug()) has told the kernel it is done just before it
	 * asked the firmware: it may not be off yet, and CPU_ON then answers "already on" or "on pending".
	 */
	bool again = os_atomic_load(&psci_smp_cpu_stage[cpu], acquire) == PSCI_SMP_CPU_OFF;

	clock_interval_to_deadline(PSCI_SMP_GONE_MS, NSEC_PER_MSEC, &deadline);
	if (again) {
		while (psci_call(PSCI_AFFINITY_INFO, phys_id, 0, 0) == 0 && mach_absolute_time() < deadline) {
			delay_for_interval(100, NSEC_PER_USEC);
		}
	}
	os_atomic_store(&psci_smp_cpu_stage[cpu], PSCI_SMP_CPU_ASKED, release);
	status = psci_call(PSCI_CPU_ON, phys_id, entry, 0);
	while (again && (status == PSCI_ALREADY_ON || status == PSCI_ON_PENDING) && mach_absolute_time() < deadline) {
		delay_for_interval(100, NSEC_PER_USEC);
		status = psci_call(PSCI_CPU_ON, phys_id, entry, 0);
	}
	if (status != 0) {
		os_atomic_store(&psci_smp_cpu_stage[cpu], 0, release);
		printf("psci: cpu %d (MPIDR 0x%x) was not started: the firmware answers %lld (%s)\n",
		    cpu, phys_id, status, psci_error(status));
		return KERN_FAILURE;
	}
	os_atomic_or(&psci_smp_started, 1u << cpu, relaxed);

	clock_interval_to_deadline(PSCI_SMP_START_SECONDS, NSEC_PER_SEC, &deadline);
	do {
		stage = os_atomic_load(&psci_smp_cpu_stage[cpu], acquire);
		if (stage >= PSCI_SMP_CPU_RUNNING) {
			return KERN_SUCCESS;
		}
		/* memory, not this processor's copy of it */
		FlushPoC_DcacheRegion((vm_offset_t)&psci_smp_start_fault, sizeof(psci_smp_start_fault));
		if (psci_smp_start_fault.valid != 0) {
			break;
		}
		delay_for_interval(1, NSEC_PER_MSEC);
		now = mach_absolute_time();
	} while (now < deadline);

	if (psci_smp_start_fault.valid != 0) {
		struct psci_smp_start_fault *f = &psci_smp_start_fault;
		uint64_t to_virtual = gVirtBase - gPhysBase;

		printf("psci: cpu %d (MPIDR 0x%llx) took an exception on its way in and has switched itself off: "
		    "ESR 0x%llx ELR 0x%llx FAR 0x%llx LR 0x%llx SP 0x%llx (ELR and LR as kernel addresses: 0x%llx 0x%llx)\n",
		    cpu, f->mpidr, f->esr, f->elr, f->far, f->lr, f->sp, f->elr + to_virtual, f->lr + to_virtual);
		os_atomic_andnot(&psci_smp_started, 1u << cpu, relaxed);        /* nothing of it is left to stop */
	} else {
		printf("psci: cpu %d (MPIDR 0x%x) was started %d seconds ago and has not joined: %s\n",
		    cpu, phys_id, PSCI_SMP_START_SECONDS,
		    stage >= PSCI_SMP_CPU_IN_KERNEL ? "it got as far as arm_init_cpu()" : "no sign of it since the firmware took the request");
	}
	return KERN_FAILURE;
}

/*
 * The calling processor switches itself off. Its interrupted work, if any, is abandoned where
 * it was: only for the end of the system.
 */
void
psci_smp_cpu_off(void)
{
	__builtin_arm_wsr("DAIFSet", DAIFSC_ALL);
	/* leave nothing behind that would interrupt a core that is no longer there */
	__builtin_arm_wsr64("CNTV_CTL_EL0", CNTV_CTL_EL0_IMASKED);
	gic_v3_irq_cpu_shutdown();
	os_atomic_store(&psci_smp_cpu_stage[cpu_number()], PSCI_SMP_CPU_OFF, release);
	__builtin_arm_dsb(DSB_SY);

	(void) psci_call(PSCI_CPU_OFF, 0, 0, 0);
	for (;;) {
		__builtin_arm_wfi();    /* the firmware refused: stay out of the way all the same */
	}
}

/*
 * The calling processor is switched off for a while, the system goes on (the end of processor_exit(): cpu_sleep() ->
 * GenericARMCPU::quiesceCPU()). Its threads and timers are elsewhere by now; the devices' interrupts that came to it go to
 * the boot processor until it is back. Its cache needs no cleaning: the firmware cleans a core it switches off. It comes
 * back through psci_smp_start_cpu(), at the reset vector, and the kernel resumes it where processor_offline() left it.
 */
void
psci_smp_cpu_unplug(void)
{
	cpu_data_t *cdp = getCpuDatap();
	int cpu = cpu_number();

	__builtin_arm_wsr("DAIFSet", DAIFSC_ALL);
	__builtin_arm_wsr64("CNTV_CTL_EL0", CNTV_CTL_EL0_IMASKED);
	gic_v3_irq_cpu_unplug();
	os_atomic_andnot(&psci_smp_started, 1u << cpu, relaxed);        /* nothing of it to stop on the way to the loader */
	os_atomic_store(&psci_smp_cpu_stage[cpu], PSCI_SMP_CPU_OFF, release);
	/* what the processor that asked waits for (cpu_exit_wait()): this one is done with the kernel's data */
	cdp->cpu_sleep_token = ARM_CPU_ON_SLEEP_PATH;
	__builtin_arm_dsb(DSB_SY);

	(void) psci_call(PSCI_CPU_OFF, 0, 0, 0);
	for (;;) {
		__builtin_arm_wfi();
	}
}

/*
 * The display off and on (MiniIOMobileFramebuffer's RequestPowerChange, as the screen locks and wakes): while it is
 * off, the processors of the boot-arg screen_off_cores (a bit per processor number; 0: none) are switched off. The way
 * is the scheduler's own for powering cores down for a while (Apple's CLPC's, sched_perfcontrol_update_powered_cores()):
 * a thread of its own starts and stops them, so this does not wait; the counts user space sees (hw.activecpu) stay as
 * they are. Never the boot processor. On the phone every core but the boot one (0xfe): a core that only waits still
 * costs power, and one Cortex-A510 carries what runs with the screen off. With 0xf0 (the fast four off) the other
 * A510s took threads cpu0 readied, each a wake of its own (POWER-COLLAPSE-PLAN.md 5f: CX collapsed 59% of the time
 * with four cores, 80% with one).
 */
TUNABLE_WRITEABLE(uint64_t, psci_smp_screen_off_cores, "screen_off_cores", 0);
static bool psci_smp_display_off;

static void
psci_smp_apply_display(void)
{
	uint64_t off = psci_smp_screen_off_cores & ~(1ULL << master_cpu) & ((1ULL << MAX_CPUS) - 1);

	if (ml_get_cpu_count() < 2) {
		return;
	}
	if (psci_smp_display_off && off != 0) {
		sched_perfcontrol_update_powered_cores(~off, REASON_CLPC_SYSTEM, 0);
		printf("psci: display off: processors 0x%llx switched off\n", off);
	} else {
		sched_perfcontrol_update_powered_cores(ALL_CORES_POWERED, REASON_CLPC_SYSTEM, 0);
		printf("psci: display %s: every processor on\n", psci_smp_display_off ? "off" : "on");
	}
}

/*
 * The display's state for the drivers that look at hardware over and over (STANDBY-PLAN.md): while it is off they
 * look less often. hakios_screen_off() says whether it is off; hakios_screen_pause(on_ms, off_ms) waits on_ms while
 * it is on and off_ms while it is off, and a display that comes on ends an off pause at once. Boot-arg
 * screen_off_slow=0 (or sysctl kern.hakios.screen_off_slow): never off as far as these two are concerned (the old periods).
 */
TUNABLE_WRITEABLE(uint32_t, hakios_screen_slow, "screen_off_slow", 1);
static uint32_t hakios_screen_changes;          /* (its address is the event of a change) */

bool
hakios_screen_off(void)
{
	return hakios_screen_slow != 0 && os_atomic_load(&psci_smp_display_off, relaxed);
}

/*
 * hakios_screen_pause_leeway: the same, and while the display is off the wait may end up to leeway_ms later, at the
 * next time its processor wakes for something else (XNU's timer coalescing). A poll that keeps its own time (the
 * cpufreq governor) then costs no wake of its own: with the screen off a core that wakes on a timer of its own, apart
 * from every other, keeps the SoC out of CX collapse (run16: the governor's 100 ms the most frequent timer of all).
 */
void
hakios_screen_pause_leeway(uint32_t on_ms, uint32_t off_ms, uint32_t leeway_ms)
{
	if (!hakios_screen_off() || off_ms <= on_ms) {
		delay_for_interval(on_ms, NSEC_PER_MSEC);
		return;
	}
	if (leeway_ms != 0) {
		assert_wait_timeout_with_leeway((event_t)&hakios_screen_changes, THREAD_UNINT, TIMEOUT_URGENCY_LEEWAY, off_ms,
		    leeway_ms, NSEC_PER_MSEC);
	} else {
		assert_wait_timeout((event_t)&hakios_screen_changes, THREAD_UNINT, off_ms, NSEC_PER_MSEC);
	}
	if (!hakios_screen_off()) {
		clear_wait(current_thread(), THREAD_AWAKENED);
		return;
	}
	(void)thread_block(THREAD_CONTINUE_NULL);
}

void
hakios_screen_pause(uint32_t on_ms, uint32_t off_ms)
{
	hakios_screen_pause_leeway(on_ms, off_ms, 0);
}

/*
 * assert_wait_timeout() for a net: a wait an interrupt (or another thread) normally ends, with a timeout only in case
 * that is missed. While the display is off the net may go off up to as late again, with the next wake of its processor
 * for something else: the nets of touch, buttons, the audio DSP's link and the sound watcher, a second each, kept apart
 * from each other and from everything else, were each a wake of their own (run19). The ring still ends it at once.
 */
wait_result_t
hakios_assert_wait_net(event_t event, uint32_t ms)
{
	if (!hakios_screen_off()) {
		return assert_wait_timeout(event, THREAD_UNINT, ms, NSEC_PER_MSEC);
	}
	return assert_wait_timeout_with_leeway(event, THREAD_UNINT, TIMEOUT_URGENCY_LEEWAY, ms, ms, NSEC_PER_MSEC);
}

void
psci_smp_display(bool on)
{
	if (psci_smp_display_off == !on) {
		return;
	}
	os_atomic_store(&psci_smp_display_off, !on, relaxed);
	os_atomic_inc(&hakios_screen_changes, relaxed);
	thread_wakeup((event_t)&hakios_screen_changes);
	if (psci_smp_screen_off_cores != 0) {
		psci_smp_apply_display();
	}
}

/*
 * Idle states (POWER-COLLAPSE-PLAN.md step 1). An idle core used to wait in WFI only. Under the phone's hypervisor
 * (Gunyah) that is not even clock gating of the core as far as the firmware is concerned: a vCPU in WFI counts as active,
 * so the physical core never powers down. Here a core whose next timer deadline is far enough away asks for its power-down
 * state with PSCI CPU_SUSPEND instead (the device tree's idle-states, the same parameter for both core types):
 *
 *   A510s ("small" cores, l2-access-penalty)   rail-pc  0x40000004  least residency 4090 us
 *   A710s and X2                               rail-pc  0x40000004  least residency 4791 us
 *
 * The hook is XNU's own: cpu_idle() calls the processor's cpu_idle_notify just before its WFI, and cpu_idle_exit() calls
 * it again on the way out (GenericARMCPU registers psci_idle_notify as the processor_idle). A CPU_SUSPEND that returns did
 * not power down (an interrupt was pending, or the firmware said no): cpu_idle() then waits in WFI as before. One that
 * powers down comes back at the reset vector, at the address given (the one CPU_ON uses), and the reset handler that
 * cpu_machine_idle_init() put there for every processor (resume_idle_cpu) takes it to arm_init_idle_cpu() -> cpu_idle_exit(TRUE):
 * MMU, stacks, the current thread, the timer's control and the counter's EL0 access are set up there. What that path
 * does not restore is done in psci_idle_notify(FALSE): the GIC's CPU interface (gic_v3_irq_cpu_resume()) and the timer's
 * deadline (setPop()).
 *
 * The timer of a vCPU that is down: the hypervisor is expected to keep it (Gunyah does for a vCPU blocked in WFI). In case
 * it does not, every core that goes idle or comes back looks at the others: one still down 2 ms after its deadline gets
 * a signal ("kicked" in kern.hakios.wakes). A core whose power-downs keep ending before the residency (3 in a row) waits
 * in WFI for its next 8 idles.
 *
 * Cluster states (step 4). All eight cores are one power domain in the device tree (psci/cluster-pd), with two states
 * beyond the cores':
 *
 *   l3-off    0x41000044  least residency 5309 us   (the L3 off)
 *   llcc-off  0x4100c344  least residency 13959 us  (and the system cache)
 *
 * Their parameters are whole: the core's state is in them (StateID 4 in [3:0]). The firmware's FEATURES(CPU_SUSPEND)
 * has bit 0 (OS-initiated mode) on the 10 Pro, and psci_idle_init() asks for that mode (SET_SUSPEND_MODE). Then the
 * cluster's state is the last core's to choose: a core going down that finds every other processor that is up already
 * down asks for the deepest cluster state that the soonest of their deadlines (its own included) leaves room for. The
 * firmware refuses (DENIED) a choice that a core waking meanwhile made wrong; that core then waits in WFI as when an
 * interrupt is pending (about half of the last cores' asks, with the display off). Where the mode cannot be had
 * (platform-coordinated, the default), each core votes for the deepest state its own deadline allows; the 10 Pro's
 * firmware refuses every such vote.
 *
 * Never while another core is being started (CPU_ON asked, not running yet) or has never been started at all: asking
 * for l3-off early in boot, before every core had been on once, reset the phone (cluster round 1, variants b and p).
 * Linux brings every core up before its cpuidle starts; here the idle states start first.
 *
 * Boot-args: cpu_idle (0: WFI only; the 10 Pro's configuration has 1), cpu_idle_cores (the processors that may power down,
 * a bit per number; all by default), cpu_idle_states (bit 0 the cores' power-down, 1 l3-off, 2 llcc-off; all three by
 * default; diag 0x40 asks for llcc-off at every other moment l3-off would do: POWER-COLLAPSE-PLAN.md step 4).
 * The parameters are the SM8450's: a board whose firmware does not know them answers INVALID_PARAMETERS and that state is
 * not asked for again (the core's: that core stays in WFI). Counters: kern.hakios.wakes, lines per processor.
 *
 * The census of the signals between processors (IPIs) is here too: what each one taken asked for (psci_idle_irq(), from
 * the GIC's handler), who sent it to whom (psci_ipi_sent()), which functions cross-calls ran, and for every exit from
 * idle the first interrupt taken after it (what woke the core).
 */
#define PSCI_FEATURES           0x8400000Au
#define PSCI_CPU_SUSPEND64      0xC4000001u
#define PSCI_SET_SUSPEND_MODE   0x8400000Fu
#define PSCI_MODE_OSI           1
#define PSCI_INVALID_PARAMETERS (-2)
#define PSCI_DENIED             (-3)
#define PSCI_FEATURE_OSI        (1u << 0)
#define PSCI_FEATURE_EXTENDED_STATEID   (1u << 1)
#define PSCI_IDLE_PARAM         0x40000004u     /* rail-pc: power-down (bit 30), StateID 4 */
#define PSCI_IDLE_SMALL_US      4090
#define PSCI_IDLE_BIG_US        4791
#define PSCI_IDLE_KICK_US       2000
#define PSCI_IDLE_SHORT_LIMIT   3
#define PSCI_IDLE_SHORT_SKIP    8

/* the states: the core's, then the cluster's (deepest last) */
#define PSCI_STATE_CORE         0
#define PSCI_STATE_L3           1
#define PSCI_STATE_LLCC         2
#define PSCI_STATES             3
static const struct {
	const char      *name;
	uint32_t        param, residency_us;
} psci_cluster_state[PSCI_STATES] = {
	[PSCI_STATE_CORE] = { "core", PSCI_IDLE_PARAM, 0 },
	[PSCI_STATE_L3]   = { "l3", 0x41000044u, 5309 },
	[PSCI_STATE_LLCC] = { "llcc", 0x4100c344u, 13959 },
};

/* the IPI census: what the signal taken asked for */
#define PSCI_IPI_NOP            0       /* nothing: a wake from idle (machine_signal_idle()) */
#define PSCI_IPI_AST            1       /* an AST check: a preemption, a thread made runnable for it */
#define PSCI_IPI_XCALL          2
#define PSCI_IPI_DEC            3
#define PSCI_IPI_OTHER          4
#define PSCI_IPI_KINDS          5
static const char *const psci_ipi_kind_name[PSCI_IPI_KINDS] = { "nop", "ast", "xcall", "dec", "other" };
#define PSCI_IPI_CPUS           8       /* the sender/target table */
#define PSCI_WOKE_INTIDS        6       /* the device interrupts that woke a core, per processor */
#define PSCI_XCALL_FUNCS        8

extern int setPop(uint64_t time);               /* osfmk/arm/rtclock.c */

TUNABLE(uint32_t, psci_idle_level, "cpu_idle", 0);
TUNABLE(uint64_t, psci_idle_cores, "cpu_idle_cores", ~0ULL);
TUNABLE(uint32_t, psci_idle_states, "cpu_idle_states", 7);

struct psci_idle_cpu {
	uint32_t        param;          /* 0: this processor stays in WFI */
	bool            down;           /* between its CPU_SUSPEND and the way back */
	bool            waking;         /* out of idle; the next interrupt taken is what woke it */
	uint8_t         state;          /* while down: the state asked for (PSCI_STATE_*) */
	uint32_t        shorts, skip;
	int32_t         last_rc;
	uint64_t        residency;      /* mach time */
	uint64_t        t0;
	uint64_t        due;            /* while down: its timer deadline, until it is back or kicked (other cores read it) */
	/* counters: written by the processor itself, but for kicked */
	uint64_t        entered, resumed, returned, denied, shorts_total, timer_wakes, kicked;
	uint64_t        down_abs, late_abs;
	uint64_t        calls, soon, soon_abs;      /* idles seen; of them with the deadline too close (and their gaps) */
	/* per state: asked for, came back from, returned at once, refused; time down; how late after the deadline */
	uint64_t        st_entered[PSCI_STATES], st_resumed[PSCI_STATES], st_returned[PSCI_STATES], st_denied[PSCI_STATES];
	uint64_t        st_down_abs[PSCI_STATES], st_late_abs[PSCI_STATES];
	uint64_t        last_tries, last_soon;      /* found itself the last one up; of them, the cluster's deadline too close */
	uint64_t        last_starting;              /* no cluster state: another processor was being started */
	/* llcc-off diagnostics: the cluster's gap at last_tries, by bucket (psci_gap_us); who held the soonest deadline
	 * when it was too close for llcc-off */
	uint64_t        last_gap[7], llcc_holder[MAX_CPUS];
	/* IPIs taken, by kind; sent, by target; what woke it out of idle */
	uint64_t        ipi_kind[PSCI_IPI_KINDS], ipi_sent[PSCI_IPI_CPUS];
	uint64_t        woke_timer, woke_ipi[PSCI_IPI_KINDS], woke_dev;
	uint32_t        woke_intid[PSCI_WOKE_INTIDS];
	uint64_t        woke_intid_n[PSCI_WOKE_INTIDS];
} __attribute__((aligned(128)));

static struct psci_idle_cpu psci_idle[MAX_CPUS];
static bool     psci_idle_ready;
static bool     psci_idle_osi;                  /* OS-initiated mode: the last core chooses the cluster's state */
static uint32_t psci_idle_states_live;          /* cpu_idle_states, less what the firmware refused as unknown */
static uint64_t psci_idle_state_abs[PSCI_STATES];
static uint64_t psci_idle_entry;                /* physical: the reset vector */
static uint64_t psci_idle_kick_abs;
static int64_t  psci_idle_features, psci_idle_mode_rc;
static struct {
	void            *func;
	uint64_t        n;
} psci_xcall_funcs[PSCI_XCALL_FUNCS];

/*
 * llcc-off diagnostics (POWER-COLLAPSE-PLAN.md step 4): the timer callouts (kind 0), running timers (1) and thread calls
 * (2) each processor ran, by function and, for a thread's own timeout (thread_timer_expire), by thread.
 */
#define PSCI_TCALL_FUNCS        128
static const uint32_t psci_gap_us[6] = { 1000, 2000, 5309, 8000, 11000, 13959 };
static uint64_t psci_gap_abs[6];
static uint32_t psci_diag_flip;
static struct {
	void            *func;
	uint64_t        tid;
	uint32_t        kind;
	uint64_t        n;
} psci_tcall[MAX_CPUS][PSCI_TCALL_FUNCS];
static uint64_t psci_tcall_lost[MAX_CPUS];
/* the timer callout running now (interrupts off), and the interrupt being handled: for the wakers below */
static void *psci_cur_callout[MAX_CPUS];
static uint32_t psci_cur_intid[MAX_CPUS];

/*
 * Wake diagnostics: who made each thread runnable (thread_unblock), by woken thread, waker and what the waker was
 * running: a thread call's function (thread_call.c, hakios_thread_call_func), a timer callout, or an interrupt (by
 * intid, the waker 1 << 63 | intid). A program's threads by the program (1 << 62 | pid): its workqueue threads come and
 * go, a row each by thread filled the table (run23: 3057 wakes lost in 120 s).
 */
#define PSCI_WAKERS             192
#define PSCI_WAKE_PID           (1ULL << 62)
extern void proc_name(int pid, char *buf, int size);

/* a thread, or its program's pid (flagged) if it is not the kernel's */
static uint64_t
psci_wake_who(thread_t t)
{
	task_t task = get_threadtask(t);

	if (task != kernel_task && task != TASK_NULL) {
		int pid = task_pid(task);
		return PSCI_WAKE_PID | (uint32_t)(pid < 0 ? 0 : pid);
	}
	return t->thread_id;
}
static struct {
	uint64_t        woken, by;
	void            *func;
	uint64_t        n;
} psci_waker[MAX_CPUS][PSCI_WAKERS];
static uint64_t psci_waker_lost[MAX_CPUS];
static _Atomic bool psci_waker_clear[MAX_CPUS];  /* read: each processor starts its table again, at its next note */

void hakios_tcall_note(void *func, void *param0, uint32_t kind);
void hakios_tcall_end(void);
void hakios_wake_note(thread_t woken, thread_t waker);
extern void *hakios_thread_call_func(thread_t thread);

void
hakios_tcall_end(void)
{
	int cpu = cpu_number();

	if (cpu >= 0 && cpu < MAX_CPUS) {
		psci_cur_callout[cpu] = NULL;
	}
}

/* thread_unblock(), the thread locked at splsched */
void
hakios_wake_note(thread_t woken, thread_t waker)
{
	int cpu = cpu_number();
	uint64_t by, who;
	void *func;
	int i;

	if (cpu < 0 || cpu >= MAX_CPUS || woken == NULL) {
		return;
	}
	if (os_atomic_xchg(&psci_waker_clear[cpu], false, relaxed)) {
		bzero(psci_waker[cpu], sizeof(psci_waker[cpu]));
		psci_waker_lost[cpu] = 0;
	}
	if (ml_at_interrupt_context()) {
		func = psci_cur_callout[cpu];
		by = func != NULL ? 0 : (1ULL << 63) | psci_cur_intid[cpu];
	} else {
		by = waker != NULL ? psci_wake_who(waker) : 0;
		func = waker != NULL ? hakios_thread_call_func(waker) : NULL;
	}
	who = psci_wake_who(woken);
	for (i = 0; i < PSCI_WAKERS; i++) {
		if (psci_waker[cpu][i].n == 0) {
			psci_waker[cpu][i].woken = who;
			psci_waker[cpu][i].by = by;
			psci_waker[cpu][i].func = func;
		}
		if (psci_waker[cpu][i].woken == who && psci_waker[cpu][i].by == by &&
		    psci_waker[cpu][i].func == func) {
			psci_waker[cpu][i].n++;
			break;
		}
	}
	if (i == PSCI_WAKERS) {
		psci_waker_lost[cpu]++;
	}
}
void
hakios_tcall_note(void *func, void *param0, uint32_t kind)
{
	boolean_t istate = ml_set_interrupts_enabled(FALSE);
	int cpu = cpu_number();
	uint64_t tid = 0;
	int i;

	if (func == (void *)thread_timer_expire && param0 != NULL) {
		tid = ((thread_t)param0)->thread_id;
	}
	if (cpu >= 0 && cpu < MAX_CPUS) {
		if (kind == 0) {
			psci_cur_callout[cpu] = func;
		}
		for (i = 0; i < PSCI_TCALL_FUNCS; i++) {
			if (psci_tcall[cpu][i].func == NULL) {
				psci_tcall[cpu][i].tid = tid;
				psci_tcall[cpu][i].kind = kind;
				psci_tcall[cpu][i].func = func;
			}
			if (psci_tcall[cpu][i].func == func && psci_tcall[cpu][i].tid == tid && psci_tcall[cpu][i].kind == kind) {
				psci_tcall[cpu][i].n++;
				break;
			}
		}
		if (i == PSCI_TCALL_FUNCS) {
			psci_tcall_lost[cpu]++;
		}
	}
	ml_set_interrupts_enabled(istate);
}

/* the kind of a signal, from the bits pending at the target */
static int
psci_ipi_kind(unsigned int bits)
{
	bits &= ~SIGPdisabled;
	if (bits & (SIGPxcall | SIGPxcallImm)) {
		return PSCI_IPI_XCALL;
	}
	if (bits & SIGPast) {
		return PSCI_IPI_AST;
	}
	if (bits & SIGPdec) {
		return PSCI_IPI_DEC;
	}
	return bits == 0 ? PSCI_IPI_NOP : PSCI_IPI_OTHER;
}

static void
psci_xcall_note(void *func)
{
	if (func == NULL) {
		return;
	}
	for (int i = 0; i < PSCI_XCALL_FUNCS; i++) {
		void *f = os_atomic_load(&psci_xcall_funcs[i].func, relaxed);

		if (f == NULL && os_atomic_cmpxchg(&psci_xcall_funcs[i].func, NULL, func, relaxed)) {
			f = func;
		}
		if (f == func) {
			os_atomic_inc(&psci_xcall_funcs[i].n, relaxed);
			return;
		}
	}
}

/*
 * The GIC's handler, for every interrupt acknowledged, before it is handled (interrupts off): kind 1 the timer, 2 the
 * signal between processors, 0 any other. Counts the IPIs by what they ask for, and the first interrupt after an exit
 * from idle as what woke the core.
 */
void
psci_idle_irq(uint32_t intid, int kind)
{
	cpu_data_t *cdp = getCpuDatap();
	int cpu = cdp->cpu_number;
	struct psci_idle_cpu *c;
	int ipi = 0;

	if (cpu < 0 || cpu >= MAX_CPUS) {
		return;
	}
	c = &psci_idle[cpu];
	psci_cur_intid[cpu] = intid;
	if (kind == 2) {
		ipi = psci_ipi_kind(os_atomic_load(&cdp->cpu_signal, relaxed));
		c->ipi_kind[ipi]++;
		if (ipi == PSCI_IPI_XCALL) {
			psci_xcall_note(cdp->cpu_xcall_p0);
			psci_xcall_note(cdp->cpu_imm_xcall_p0);
		}
	}
	if (!c->waking) {
		return;
	}
	c->waking = false;
	if (kind == 1) {
		c->woke_timer++;
	} else if (kind == 2) {
		c->woke_ipi[ipi]++;
	} else {
		c->woke_dev++;
		for (int i = 0; i < PSCI_WOKE_INTIDS; i++) {
			if (c->woke_intid_n[i] == 0) {
				c->woke_intid[i] = intid;
			}
			if (c->woke_intid[i] == intid) {
				c->woke_intid_n[i]++;
				break;
			}
		}
	}
}

/*
 * gic_v3_irq_signal(): this processor signals the one with that MPIDR. A target that is down is on its way up from
 * now: its due goes (as psci_idle_kick_late() does), so that this processor, going idle right after, does not take
 * itself for the last one up and ask for a cluster state the firmware then denies (run16/17: a third of the llcc-off
 * asks DENIED, most of the wakes of cores 1-3 a signal from cpu0, sent just before cpu0 went idle itself).
 */
void
psci_ipi_sent(uint32_t phys_id)
{
	int self = cpu_number();

	if (self < 0 || self >= MAX_CPUS) {
		return;
	}
	for (int cpu = 0; cpu <= ml_get_max_cpu_number() && cpu < PSCI_IPI_CPUS; cpu++) {
		cpu_data_t *cdp = cpu_datap(cpu);

		if (cdp != NULL && cdp->cpu_phys_id == phys_id) {
			os_atomic_inc(&psci_idle[self].ipi_sent[cpu], relaxed);
			if (cpu != self) {
				os_atomic_store(&psci_idle[cpu].due, 0, release);
			}
			return;
		}
	}
}

/* Any processor that is down 2 ms past its deadline is signalled: its timer did not wake it. */
static void
psci_idle_kick_late(int self, uint64_t now)
{
	for (int cpu = 0; cpu <= ml_get_max_cpu_number() && cpu < MAX_CPUS; cpu++) {
		struct psci_idle_cpu *c = &psci_idle[cpu];
		uint64_t due = os_atomic_load(&c->due, relaxed);

		if (cpu == self || due == 0 || now < due + psci_idle_kick_abs) {
			continue;
		}
		if (os_atomic_cmpxchg(&c->due, due, 0, relaxed)) {
			os_atomic_inc(&c->kicked, relaxed);
			gic_v3_irq_signal(cpu_datap(cpu)->cpu_phys_id);
		}
	}
}

/* Back at cpu_idle_exit(TRUE) from a power-down: interrupts off, MMU on, the core's registers fresh from reset. */
static void
psci_idle_resumed(cpu_data_t *cdp, struct psci_idle_cpu *c, uint64_t now)
{
	gic_v3_irq_cpu_resume();
	setPop(cdp->rtcPop);            /* the timer's deadline (arm_init_idle_cpu() enabled the timer) */
	os_atomic_store(&c->due, 0, relaxed);
	c->down = false;
	c->resumed++;
	c->st_resumed[c->state]++;

	uint64_t slept = now - c->t0;
	c->down_abs += slept;
	c->st_down_abs[c->state] += slept;
	if (now >= cdp->rtcPop) {
		c->timer_wakes++;
		c->late_abs += now - cdp->rtcPop;
		c->st_late_abs[c->state] += now - cdp->rtcPop;
	}
	if (slept < c->residency) {
		c->shorts_total++;
		if (++c->shorts >= PSCI_IDLE_SHORT_LIMIT) {
			c->shorts = 0;
			c->skip = PSCI_IDLE_SHORT_SKIP;
		}
	} else {
		c->shorts = 0;
	}
}

/*
 * The state for a core about to go down whose own deadline (pop) leaves room for its power-down: a cluster state when
 * the cluster's deadline leaves room for one. In OS-initiated mode only the last processor up may ask, and the cluster's
 * deadline is the soonest of all; it reads the others' due, set before their CPU_SUSPEND (and its own is set already).
 */
static int
psci_idle_cluster_choice(int self, uint64_t pop, uint64_t now)
{
	uint32_t states = os_atomic_load(&psci_idle_states_live, relaxed);
	uint64_t due = pop;
	int holder = self;

	if ((states & ((1u << PSCI_STATE_L3) | (1u << PSCI_STATE_LLCC))) == 0) {
		return PSCI_STATE_CORE;
	}
	if (psci_idle_osi) {
		uint64_t up = psci_smp_cores_up();

		for (int cpu = 0; cpu <= ml_get_max_cpu_number() && cpu < MAX_CPUS; cpu++) {
			uint64_t d;
			uint32_t stage = os_atomic_load(&psci_smp_cpu_stage[cpu], acquire);

			/* one being started (CPU_ON asked, not running yet) is not counted up, but it is not off either:
			 * no cluster state then (the reset of cluster round 1 came in early boot, while the others started) */
			if (cpu != self && (stage == PSCI_SMP_CPU_ASKED || stage == PSCI_SMP_CPU_IN_KERNEL ||
			    /* ... nor while one has never been started at all: early in boot, asking for l3-off with cores
			     * the firmware has never had on resets the phone (variant p); after one CPU_ON/CPU_OFF it is fine */
			    (stage == 0 && cpu != master_cpu && cpu_datap(cpu) != NULL))) {
				psci_idle[self].last_starting++;
				return PSCI_STATE_CORE;
			}
			if (cpu == self || !(up & (1ULL << cpu))) {
				continue;
			}
			d = os_atomic_load(&psci_idle[cpu].due, acquire);
			if (d == 0) {
				return PSCI_STATE_CORE;         /* that one is running, in WFI, or on its way back */
			}
			if (d < due) {
				due = d;
				holder = cpu;
			}
		}
		psci_idle[self].last_tries++;
		{
			uint64_t gap = due > now ? due - now : 0;
			int b = 0;

			while (b < 6 && gap >= psci_gap_abs[b]) {
				b++;
			}
			psci_idle[self].last_gap[b]++;
			if (gap < psci_idle_state_abs[PSCI_STATE_LLCC] && holder < MAX_CPUS) {
				psci_idle[self].llcc_holder[holder]++;
			}
		}
	}
	for (int state = PSCI_STATES - 1; state > PSCI_STATE_CORE; state--) {
		uint64_t need = psci_idle_state_abs[state];

		/* diag 0x40: ask for llcc-off at every other moment l3-off would do, to tell a refusal of the state from a race
		 * (the other half asks for l3-off under the same conditions) */
		if (state == PSCI_STATE_LLCC && (psci_idle_states & 0x40) && (os_atomic_inc(&psci_diag_flip, relaxed) & 1)) {
			need = psci_idle_state_abs[PSCI_STATE_L3];
		}
		if ((states & (1u << state)) && due > now && due - now >= need) {
			return state;
		}
	}
	if (psci_idle_osi) {
		psci_idle[self].last_soon++;
	}
	return PSCI_STATE_CORE;
}

/* The processor's cpu_idle_notify: enter = TRUE just before cpu_idle()'s WFI, FALSE in cpu_idle_exit(). Interrupts off. */
void
psci_idle_notify(__unused cpu_id_t cpu_id, boolean_t enter, __unused uint64_t *new_timeout_ticks)
{
	if (!os_atomic_load(&psci_idle_ready, acquire)) {
		return;
	}
	cpu_data_t *cdp = getCpuDatap();
	int cpu = cdp->cpu_number;
	struct psci_idle_cpu *c = &psci_idle[cpu];
	uint64_t now = mach_absolute_time();

	if (!enter) {
		if (c->down) {
			psci_idle_resumed(cdp, c, now);
		}
		c->waking = true;
		psci_idle_kick_late(cpu, now);
		return;
	}
	psci_idle_kick_late(cpu, now);
	c->calls++;
	if (c->param == 0) {
		return;
	}
	if (c->skip != 0) {
		c->skip--;
		return;
	}
	uint64_t pop = cdp->rtcPop;
	if (pop <= now || pop - now < c->residency) {
		c->soon++;
		c->soon_abs += pop > now ? pop - now : 0;
		return;
	}
	c->t0 = now;
	c->entered++;
	c->down = true;
	os_atomic_store(&c->due, pop, release);
	__builtin_arm_dsb(DSB_SY);

	int state = psci_idle_cluster_choice(cpu, pop, now);
	uint32_t param = state == PSCI_STATE_CORE ? c->param : psci_cluster_state[state].param;

	/* the last core up, about to ask for the state in which the RSC runs its sleep set: the ARC entries armed or not
	 * (drivers/power/hakios.rpmh-arc.h; as Linux flushes its sleep set at the cluster's power-down) */
	if (state == PSCI_STATE_LLCC && hakios_rpmh_llcc_down != NULL) {
		hakios_rpmh_llcc_down();
	}

	c->state = (uint8_t)state;
	c->st_entered[state]++;
	int64_t rc = (int32_t)psci_call(PSCI_CPU_SUSPEND64, param, psci_idle_entry, (uint64_t)cpu);

	/* still here: it did not power down */
	os_atomic_store(&c->due, 0, relaxed);
	c->down = false;
	if (rc == 0) {
		c->returned++;
		c->st_returned[state]++;
	} else {
		c->denied++;
		c->st_denied[state]++;
		c->last_rc = (int32_t)rc;
		if (rc == PSCI_INVALID_PARAMETERS) {
			if (state == PSCI_STATE_CORE) {
				c->param = 0;   /* not this firmware's state: never again on this processor */
			} else {
				os_atomic_andnot(&psci_idle_states_live, 1u << state, relaxed);  /* nor this cluster state */
			}
		}
	}
}

/* GenericARMPE::start(), every processor registered (not all of them started yet), after psci_smp_init(). */
void
psci_idle_init(void)
{
	const ml_topology_info_t *topology = ml_get_topology_info();
	uint32_t cores = 0;

	if (psci_idle_level == 0) {
		printf("psci: idle: WFI only (cpu_idle=0)\n");
		return;
	}
	psci_idle_features = (int32_t)psci_call(PSCI_FEATURES, PSCI_CPU_SUSPEND64, 0, 0);
	if (psci_idle_features < 0 || !(psci_idle_features & PSCI_FEATURE_EXTENDED_STATEID)) {
		printf("psci: idle: WFI only: the firmware's CPU_SUSPEND %s (features %lld)\n",
		    psci_idle_features < 0 ? "is not there" : "takes the original StateID format", psci_idle_features);
		return;
	}
	if (!(psci_idle_states & (1u << PSCI_STATE_CORE))) {
		printf("psci: idle: WFI only (cpu_idle_states 0x%x: no power-down of the cores)\n", psci_idle_states);
		return;
	}
	nanoseconds_to_absolutetime((uint64_t)PSCI_IDLE_KICK_US * NSEC_PER_USEC, &psci_idle_kick_abs);
	for (int state = 0; state < PSCI_STATES; state++) {
		nanoseconds_to_absolutetime((uint64_t)psci_cluster_state[state].residency_us * NSEC_PER_USEC,
		    &psci_idle_state_abs[state]);
	}
	for (int b = 0; b < 6; b++) {
		nanoseconds_to_absolutetime((uint64_t)psci_gap_us[b] * NSEC_PER_USEC, &psci_gap_abs[b]);
	}
	psci_idle_states_live = psci_idle_states & ((1u << PSCI_STATES) - 1);
	if (psci_idle_states_live & ~1u) {
		/* no other processor is suspended yet: the mode can be changed now (PSCI 1.0, 5.20) */
		if (psci_idle_features & PSCI_FEATURE_OSI) {
			psci_idle_mode_rc = (int32_t)psci_call(PSCI_SET_SUSPEND_MODE, PSCI_MODE_OSI, 0, 0);
			psci_idle_osi = psci_idle_mode_rc == 0;
		} else {
			psci_idle_mode_rc = 1;      /* not asked */
		}
		printf("psci: idle: cluster states 0x%x, %s (SET_SUSPEND_MODE %lld)\n", psci_idle_states_live,
		    psci_idle_osi ? "OS-initiated: the last core chooses" : "platform-coordinated: every core votes",
		    psci_idle_mode_rc);
	}
	for (unsigned int i = 0; i < topology->num_cpus; i++) {
		const ml_topology_cpu_t *t = &topology->cpus[i];
		struct psci_idle_cpu *c;
		bool small = t->l2_access_penalty != 0;

		if (t->cpu_id >= MAX_CPUS || !(psci_idle_cores & (1ULL << t->cpu_id))) {
			continue;
		}
		c = &psci_idle[t->cpu_id];
		nanoseconds_to_absolutetime((uint64_t)(small ? PSCI_IDLE_SMALL_US : PSCI_IDLE_BIG_US) * NSEC_PER_USEC, &c->residency);
		c->param = PSCI_IDLE_PARAM;
		cores |= 1u << t->cpu_id;
	}
	psci_idle_entry = ml_static_vtop((vm_offset_t)&LowResetVectorBase);
	psci_smp_clean_for_reset();
	os_atomic_store(&psci_idle_ready, true, release);
	printf("psci: idle: CPU_SUSPEND features 0x%llx; processors 0x%x power down when idle (0x%x, %d us on the small "
	    "cores, %d us on the others)\n", psci_idle_features, cores, PSCI_IDLE_PARAM, PSCI_IDLE_SMALL_US, PSCI_IDLE_BIG_US);
}

/*
 * kern.hakios.wakes (GenericARMPE.cpp): what wakes the processors, as text. A line per processor (interrupts taken,
 * of them signals from other processors, timer interrupts), then a line per kernel thread that has been switched to
 * at least 16 times (its id, its name or -, its context switches). Read it twice and subtract (STANDBY-PLAN.md).
 */
size_t
hakios_wakes_text(char *buf, size_t size)
{
	size_t at = 0;
	thread_t thread;

#define WAKES_PUT(...) do { \
	        int n_ = snprintf(buf + at, size - at, __VA_ARGS__); \
	        at += n_ > 0 ? MIN((size_t)n_, size - at - 1) : 0; \
	} while (0)
	if (size == 0) {
		return 0;
	}
	buf[0] = '\0';
	WAKES_PUT("display %s up 0x%llx\n", os_atomic_load(&psci_smp_display_off, relaxed) ? "off" : "on", psci_smp_cores_up());
	WAKES_PUT("idle %s features 0x%llx mode %s rc %lld states 0x%x\n", os_atomic_load(&psci_idle_ready, relaxed) ? "on" : "off",
	    psci_idle_features, psci_idle_osi ? "osi" : "pc", psci_idle_mode_rc, os_atomic_load(&psci_idle_states_live, relaxed));
	/* the functions that cross-calls ran, and one of the kernel's own for the slide (nm hakios-kernel) */
	WAKES_PUT("xcall_ref psci_idle_notify %p\n", (void *)VM_KERNEL_UNSLIDE(psci_idle_notify));
	for (int i = 0; i < PSCI_XCALL_FUNCS; i++) {
		void *f = os_atomic_load(&psci_xcall_funcs[i].func, relaxed);

		if (f != NULL) {
			WAKES_PUT("xcall %p %llu\n", (void *)VM_KERNEL_UNSLIDE(f), os_atomic_load(&psci_xcall_funcs[i].n, relaxed));
		}
	}
	for (int cpu = 0; cpu <= ml_get_max_cpu_number() && cpu < MAX_CPUS; cpu++) {
		cpu_data_t *cdp = cpu_datap(cpu);

		if (cdp != NULL) {
			struct psci_idle_cpu *c = &psci_idle[cpu];
			uint64_t down_ns, late_ns, soon_ns;

			absolutetime_to_nanoseconds(os_atomic_load(&c->down_abs, relaxed), &down_ns);
			absolutetime_to_nanoseconds(os_atomic_load(&c->late_abs, relaxed), &late_ns);
			absolutetime_to_nanoseconds(os_atomic_load(&c->soon_abs, relaxed), &soon_ns);
			/* idle: power-downs asked for, came back from, returned at once, refused (last code), shorter than the
			 * residency, woken at or after the deadline (and how late in all), signalled for being late; time down */
			WAKES_PUT("cpu %d irq %llu ipi %llu timer %llu idle entered %llu resumed %llu returned %llu denied %llu rc %d "
			    "short %llu timer_wakes %llu late_us %llu kicked %llu down_ms %llu calls %llu soon %llu soon_us %llu\n", cpu,
			    os_atomic_load(&cdp->cpu_stat.irq_ex_cnt, relaxed),
			    os_atomic_load(&cdp->cpu_stat.ipi_cnt, relaxed), os_atomic_load(&cdp->cpu_stat.timer_cnt, relaxed),
			    os_atomic_load(&c->entered, relaxed), os_atomic_load(&c->resumed, relaxed),
			    os_atomic_load(&c->returned, relaxed), os_atomic_load(&c->denied, relaxed), c->last_rc,
			    os_atomic_load(&c->shorts_total, relaxed), os_atomic_load(&c->timer_wakes, relaxed), late_ns / NSEC_PER_USEC,
			    os_atomic_load(&c->kicked, relaxed), down_ns / NSEC_PER_MSEC, os_atomic_load(&c->calls, relaxed),
			    os_atomic_load(&c->soon, relaxed), soon_ns / NSEC_PER_USEC);
			/* per state: asked for, came back from, returned at once, refused, time down, late after the deadline */
			WAKES_PUT("states %d last %llu last_soon %llu starting %llu", cpu, c->last_tries, c->last_soon, c->last_starting);
			for (int s = 0; s < PSCI_STATES; s++) {
				uint64_t sd_ns, sl_ns;

				absolutetime_to_nanoseconds(c->st_down_abs[s], &sd_ns);
				absolutetime_to_nanoseconds(c->st_late_abs[s], &sl_ns);
				WAKES_PUT(" %s %llu/%llu/%llu/%llu down_ms %llu late_us %llu", psci_cluster_state[s].name,
				    c->st_entered[s], c->st_resumed[s], c->st_returned[s], c->st_denied[s],
				    sd_ns / NSEC_PER_MSEC, sl_ns / NSEC_PER_USEC);
			}
			/* the cluster's gap when last, by bucket (<1, <2, <5.3, <8, <11, <14 ms, more); who held it under 14 ms */
			WAKES_PUT("\ngaps %d", cpu);
			for (int b = 0; b < 7; b++) {
				WAKES_PUT("%c%llu", b ? ',' : ' ', c->last_gap[b]);
			}
			WAKES_PUT(" holder");
			for (int t = 0; t < PSCI_IPI_CPUS; t++) {
				WAKES_PUT("%c%llu", t ? ',' : ' ', c->llcc_holder[t]);
			}
			/* IPIs taken by kind; sent to each processor */
			WAKES_PUT("\nipi %d", cpu);
			for (int k = 0; k < PSCI_IPI_KINDS; k++) {
				WAKES_PUT(" %s %llu", psci_ipi_kind_name[k], c->ipi_kind[k]);
			}
			WAKES_PUT(" to");
			for (int t = 0; t < PSCI_IPI_CPUS; t++) {
				WAKES_PUT("%c%llu", t ? ',' : ' ', os_atomic_load(&c->ipi_sent[t], relaxed));
			}
			/* what woke it out of idle: the timer, an IPI by kind, a device interrupt (the most frequent intids) */
			WAKES_PUT("\nwoke %d timer %llu", cpu, c->woke_timer);
			for (int k = 0; k < PSCI_IPI_KINDS; k++) {
				WAKES_PUT(" %s %llu", psci_ipi_kind_name[k], c->woke_ipi[k]);
			}
			WAKES_PUT(" dev %llu", c->woke_dev);
			for (int i = 0; i < PSCI_WOKE_INTIDS && c->woke_intid_n[i] != 0; i++) {
				WAKES_PUT(" irq%u:%llu", c->woke_intid[i], c->woke_intid_n[i]);
			}
			WAKES_PUT("\n");
			/* cpu_idle() (osfmk/arm64/cpu.c): calls, gone back at once (idle not allowed, timer due); idle time */
			{
				extern uint64_t hakios_idle_exits[MAX_CPUS][3];
				extern bool ml_cpu_idle_time(int cpu_id, uint64_t *idle);
				uint64_t idle_abs = 0, idle_ns;

				(void)ml_cpu_idle_time(cpu, &idle_abs);
				absolutetime_to_nanoseconds(idle_abs, &idle_ns);
				WAKES_PUT("idlepath %d calls %llu disallowed %llu due %llu signal 0x%x idle_ms %llu\n", cpu,
				    hakios_idle_exits[cpu][2], hakios_idle_exits[cpu][0], hakios_idle_exits[cpu][1],
				    os_atomic_load(&cdp->cpu_signal, relaxed), idle_ns / NSEC_PER_MSEC);
			}
		}
	}
	lck_mtx_lock(&tasks_threads_lock);
	queue_iterate(&threads, thread, thread_t, threads) {
		char name[MAXTHREADNAMESIZE];

		task_t task = get_threadtask(thread);
		uint64_t run_ns;

		/* kernel threads switched to 16 times or more; any thread with half a second of processor time */
		absolutetime_to_nanoseconds(recount_thread_time_mach(thread), &run_ns);
		if ((task != kernel_task || thread->c_switch < 16) && run_ns < 500 * NSEC_PER_MSEC) {
			continue;
		}
		name[0] = '\0';
		thread_get_thread_name(thread, name);
		name[sizeof(name) - 1] = '\0';
		WAKES_PUT("thread %llu %s %u run_ms %llu pid %d", thread->thread_id, name[0] ? name : "-", thread->c_switch,
		    run_ns / NSEC_PER_MSEC, task == kernel_task ? 0 : task_pid(task));
		/* llcc-off diagnostics: where a kernel thread waits (its continuation, wait event, and saved stack) */
		if (task == kernel_task) {
			vm_offset_t stack = thread->kernel_stack;

			WAKES_PUT(" cont %p wait %p", (void *)VM_KERNEL_UNSLIDE(thread->continuation),
			    (void *)VM_KERNEL_UNSLIDE_OR_PERM(thread->wait_event));
			if (stack != 0 && thread->machine.kstackptr != 0 && (thread->state & TH_WAIT) && !(thread->state & TH_RUN)) {
				arm_kernel_saved_state_t *ss = &((thread_kernel_state_t)thread->machine.kstackptr)->machine.ss;
				uint64_t fp = ss->fp;

				WAKES_PUT(" pc %p", (void *)VM_KERNEL_UNSLIDE(ss->lr));
				for (int f = 0; f < 8 && fp >= stack && fp + 16 <= stack + kernel_stack_size && (fp & 7) == 0; f++) {
					uint64_t lr = ((uint64_t *)fp)[1];

					WAKES_PUT(",%p", (void *)VM_KERNEL_UNSLIDE(lr));
					fp = ((uint64_t *)fp)[0];
				}
			}
		}
		WAKES_PUT("\n");
		if (at + 1 >= size) {
			break;
		}
	}
	lck_mtx_unlock(&tasks_threads_lock);
	/* llcc-off diagnostics: timer callouts (0), running timers (1), thread calls (2) per processor, by function/thread */
	for (int cpu = 0; cpu <= ml_get_max_cpu_number() && cpu < MAX_CPUS; cpu++) {
		for (int i = 0; i < PSCI_TCALL_FUNCS; i++) {
			void *f = psci_tcall[cpu][i].func;

			if (f == NULL) {
				break;
			}
			WAKES_PUT("tc %d %u %p %llu %llu\n", cpu, psci_tcall[cpu][i].kind, (void *)VM_KERNEL_UNSLIDE(f),
			    psci_tcall[cpu][i].tid, psci_tcall[cpu][i].n);
		}
		if (psci_tcall_lost[cpu] != 0) {
			WAKES_PUT("tc_lost %d %llu\n", cpu, psci_tcall_lost[cpu]);
		}
	}
	/* wake diagnostics: wk cpu woken-thread waker (a thread, 0 a timer callout, 1 << 63 | intid an interrupt) func count,
	 * since the last read (each processor starts again at its next note) */
	for (int cpu = 0; cpu <= ml_get_max_cpu_number() && cpu < MAX_CPUS; cpu++) {
		if (os_atomic_load(&psci_waker_clear[cpu], relaxed)) {
			continue;       /* nothing noted there since the last read (a processor that is off) */
		}
		for (int i = 0; i < PSCI_WAKERS && psci_waker[cpu][i].n != 0; i++) {
			WAKES_PUT("wk %d 0x%llx 0x%llx %p %llu", cpu, psci_waker[cpu][i].woken, psci_waker[cpu][i].by,
			    (void *)VM_KERNEL_UNSLIDE(psci_waker[cpu][i].func), psci_waker[cpu][i].n);
			/* the programs' names: woken, then waker (- for a kernel thread, an interrupt or a callout) */
			for (int w = 0; w < 2; w++) {
				uint64_t x = w ? psci_waker[cpu][i].by : psci_waker[cpu][i].woken;
				char name[32] = "-";

				if ((x >> 62) == 1) {
					proc_name((int)(uint32_t)x, name, sizeof(name));
				}
				WAKES_PUT(" %s", name[0] ? name : "?");
			}
			WAKES_PUT("\n");
		}
		if (psci_waker_lost[cpu] != 0) {
			WAKES_PUT("wk_lost %d %llu\n", cpu, psci_waker_lost[cpu]);
		}
		os_atomic_store(&psci_waker_clear[cpu], true, relaxed);
	}
#undef WAKES_PUT
	return at;
}

/* sysctl kern.hakios.screen_off_cores (GenericARMPE.cpp): a new set, in effect at once if the display is off */
void
psci_smp_set_screen_off_cores(uint64_t cores)
{
	psci_smp_screen_off_cores = cores;
	psci_smp_apply_display();
}

/* The processors that are up (not switched off, nor on their way off): a bit per processor number. */
uint64_t
psci_smp_cores_up(void)
{
	uint64_t up = 0;

	for (int cpu = 0; cpu <= ml_get_max_cpu_number() && cpu < MAX_CPUS; cpu++) {
		processor_t processor = processor_array[cpu];

		if (processor != PROCESSOR_NULL && processor->state != PROCESSOR_OFF_LINE &&
		    processor->state != PROCESSOR_PENDING_OFFLINE && processor->state != PROCESSOR_SHUTDOWN) {
			up |= 1ULL << cpu;
		}
	}
	return up;
}

static void
psci_smp_note(const char *what, int cpu)
{
	static const char prefix[] = "\npsci: cpu ";

	for (const char *c = prefix; *c; c++) {
		loader_log_putc(*c);
	}
	loader_log_putc((char)('0' + cpu));
	for (const char *c = what; *c; c++) {
		loader_log_putc(*c);
	}
}

/* Tell every core but this one to switch itself off (psci_smp_stop_here() is where they hear it). */
static void
psci_smp_stop_rest(void)
{
	int me = cpu_number(), max_cpu = ml_get_max_cpu_number();

	os_atomic_store(&psci_smp_stopping, true, release);
	__builtin_arm_dsb(DSB_SY);
	for (int cpu = 0; cpu <= max_cpu; cpu++) {
		cpu_data_t *cdp = cpu_datap(cpu);

		if (cpu != me && cdp != NULL && (cpu == master_cpu || (psci_smp_started & (1u << cpu)))) {
			gic_v3_irq_signal_stop(cdp->cpu_phys_id);
		}
	}
	/* for those that wait with their interrupts off (wait_while_mp_kdp_trap(), after a panic) */
	__builtin_arm_sev();
}

/*
 * The system is going back to the boot loader (loader_return()). Returns on the boot processor
 * only, once the other cores are off -- or have been given up on. On any other processor it
 * does not return: that one hands the job to the boot processor, which the loader's side of
 * the return belongs to, and switches itself off like the rest.
 */
void
psci_smp_stop_others(void)
{
	int me, max_cpu;
	uint64_t now, deadline;

	if (psci_smp_started == 0) {
		return;                 /* there is only this one */
	}
	__builtin_arm_wsr("DAIFSet", DAIFSC_ALL);
	me = cpu_number();
	max_cpu = ml_get_max_cpu_number();
	if (!os_atomic_cmpxchg(&psci_smp_stop_owner, -1, me, acq_rel) && psci_smp_stop_owner != me && me != master_cpu) {
		psci_smp_cpu_off();     /* another processor is at it already */
	}

	if (me != master_cpu) {
		/*
		 * Nobody is stopped before the boot processor has answered: a core that is switched
		 * off while it holds a lock the boot processor is waiting for would take the return
		 * with it.
		 */
		psci_smp_note(" ends the system: the return to the boot loader is the boot processor's to make\n", me);
		os_atomic_store(&psci_smp_boot_wanted, true, release);
		__builtin_arm_dsb(DSB_SY);
		gic_v3_irq_signal_stop(cpu_datap(master_cpu)->cpu_phys_id);
		__builtin_arm_sev();
		now = mach_absolute_time();
		nanoseconds_to_absolutetime(PSCI_SMP_STOP_SECONDS * NSEC_PER_SEC, &deadline);
		deadline += now;
		while (!os_atomic_load(&psci_smp_stopping, acquire) && now < deadline) {
			now = mach_absolute_time();
		}
		if (!os_atomic_load(&psci_smp_stopping, acquire)) {
			/*
			 * It does not hear (stuck with its interrupts off). There is no return without
			 * it; a loader that watches from its own core will find the kernel silent and
			 * take the log as it is, and so this goes into the log.
			 */
			psci_smp_note(" cannot return to the boot loader and the boot processor does not answer: all cores stop here\n", me);
			psci_smp_stop_rest();
		}
		psci_smp_cpu_off();
	}

	psci_smp_stop_rest();
	now = mach_absolute_time();
	nanoseconds_to_absolutetime(PSCI_SMP_STOP_SECONDS * NSEC_PER_SEC, &deadline);
	deadline += now;
	for (int cpu = 0; cpu <= max_cpu; cpu++) {
		cpu_data_t *cdp = cpu_datap(cpu);

		if (cpu == me || cdp == NULL || !(psci_smp_started & (1u << cpu))) {
			continue;
		}
		while (now < deadline) {
			if (os_atomic_load(&psci_smp_cpu_stage[cpu], acquire) == PSCI_SMP_CPU_OFF) {
				/* it has said it is going; the firmware knows when it has gone (if it can tell at all) */
				int64_t state = psci_call(PSCI_AFFINITY_INFO, cdp->cpu_phys_id, 0, 0);

				if (state == PSCI_AFFINITY_OFF || state < 0) {
					break;
				}
			}
			now = mach_absolute_time();
		}
		if (os_atomic_load(&psci_smp_cpu_stage[cpu], acquire) != PSCI_SMP_CPU_OFF &&
		    psci_call(PSCI_AFFINITY_INFO, cdp->cpu_phys_id, 0, 0) != PSCI_AFFINITY_OFF) {
			psci_smp_note(" did not stop: it is left running\n", cpu);
		}
	}
}

/*
 * Where a processor learns that the system is ending: the interrupt psci_smp_stop_rest() sends,
 * and the loop in which the processors wait while another one panics (they have their
 * interrupts off there). Comes back if there is nothing of the kind going on.
 */
void
psci_smp_stop_here(void)
{
	if (psci_smp_started == 0) {
		return;
	}
	if (cpu_number() != master_cpu) {
		if (os_atomic_load(&psci_smp_stopping, acquire)) {
			psci_smp_cpu_off();
		}
		return;
	}
	if (os_atomic_load(&psci_smp_boot_wanted, acquire) && !psci_smp_boot_busy) {
		psci_smp_boot_busy = true;
		loader_return();        /* stops the rest (psci_smp_stop_others()); back only if no loader listens */
		for (;;) {
			__builtin_arm_wfi();
		}
	}
}

#endif /* defined(GENERIC_PSCI_SMP) */
