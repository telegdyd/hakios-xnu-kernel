/*
 * The idle states of the cores and the cores off while the display is off (POWER-COLLAPSE-PLAN.md, from
 * negroni-dev's osfmk/arm64/psci_smp.c): an idle core powers down through PSCI CPU_SUSPEND instead of only
 * waiting in WFI, the last core up taking the cluster's states with it where the board's cpu-idle { } has them;
 * the processors of screen_off_cores are switched off while the display is off; the drivers' loops look less
 * often then (hakios_screen_pause(), hakios_assert_wait_net()); and kern.hakios.wakes / kern.hakios.idle say
 * what wakes the processors. The way a core goes down and comes back is psci_smp.c's.
 */
#include <mach/mach_types.h>
#include <stdarg.h>
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
#include <machine/atomic.h>
#include <arm/cpu_data_internal.h>
#include <arm/cpu_internal.h>
#include <arm/machine_routines.h>
#include <arm64/proc_reg.h>
#include <arm64/psci_smp.h>
#include <arm/rtclock.h>
#include <arm/commpage/commpage.h>
#include <kern/timer_queue.h>
#include <pexpert/pexpert.h>
#include <pexpert/arm64/board_config.h>
#include <sys/errno.h>

/* drivers/board/rpmh_sleep.h: the last core's hook before llcc-off (an empty one where no board driver has it) */
void hakios_rpmh_llcc_down(void);
__attribute__((weak)) void
hakios_rpmh_llcc_down(void)
{
}

#if defined(GENERIC_PSCI_SMP)

/* psci_smp.c's, for this file */
#define psci_call(f, a1, a2, a3)        psci_smp_call((f), (a1), (a2), (a3))
#define psci_smp_boot_cpu()             psci_smp_boot_cpu_number()
#define psci_smp_cpu_stage_of(cpu)      psci_smp_stage_of(cpu)
#define psci_smp_loader_entry           psci_smp_loader_entry_pa()
#define psci_smp_clean_for_reset()      psci_smp_clean_for_reset_all()

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
#define PSCI_SCREEN_OFF_BOARD   (~0ULL)         /* (no boot-arg: the board's cpu-idle { } screen-off-cores) */
/*
 * What this says about the idle states and the display: into the loader's log too, the way the live log of a run gets
 * it (the kernel's printf alone does not reach it once user space runs).
 */
extern int vsnprintf(char *, size_t, const char *, va_list) __printflike(3, 0);
static void psci_say(const char *format, ...) __printflike(1, 2);
static void
psci_say(const char *format, ...)
{
	extern void loader_log_line(const char *line);
	char line[200];
	va_list ap;

	va_start(ap, format);
	vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);
	printf("%s", line);
	loader_log_line(line);
}

/* screen_off_cores=MASK: processors off while the display is off (default: the board's cpu-idle { }; 0: none) */
TUNABLE_WRITEABLE(uint64_t, psci_smp_screen_off_cores, "screen_off_cores", PSCI_SCREEN_OFF_BOARD);
static bool psci_smp_display_off;

static void
psci_smp_apply_display(void)
{
	uint64_t off = psci_smp_screen_off_cores & ~(1ULL << psci_smp_boot_cpu()) & ((1ULL << MAX_CPUS) - 1);

	if (ml_get_cpu_count() < 2) {
		return;
	}
	if (psci_smp_display_off && off != 0) {
		sched_perfcontrol_update_powered_cores(~off, REASON_CLPC_SYSTEM, 0);
		psci_say("psci: display off: processors 0x%llx switched off\n", off);
	} else {
		sched_perfcontrol_update_powered_cores(ALL_CORES_POWERED, REASON_CLPC_SYSTEM, 0);
		psci_say("psci: display %s: every processor on\n", psci_smp_display_off ? "off" : "on");
	}
}

/*
 * The display's state for the drivers that look at hardware over and over (STANDBY-PLAN.md): while it is off they
 * look less often. hakios_screen_off() says whether it is off; hakios_screen_pause(on_ms, off_ms) waits on_ms while
 * it is on and off_ms while it is off, and a display that comes on ends an off pause at once. Boot-arg
 * screen_off_slow=0 (or sysctl kern.hakios.screen_off_slow): never off as far as these two are concerned (the old periods).
 */
/* screen_off_slow=0: the drivers' loops look as often with the display off */
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
#define PSCI_IDLE_KICK_US       2000
#define PSCI_IDLE_SHORT_LIMIT   3
#define PSCI_IDLE_SHORT_SKIP    8

/* the states: the core's, then the cluster's (deepest last) */
#define PSCI_STATE_CORE         0
#define PSCI_STATE_L3           1
#define PSCI_STATE_LLCC         2
#define PSCI_STATES             3
/* (the parameters and least residencies are the board's: soc cpu-idle { }, drivers/board/board.c) */
static struct {
	const char      *name;
	uint32_t        param, residency_us;
} psci_cluster_state[PSCI_STATES] = {
	[PSCI_STATE_CORE] = { "core", 0, 0 },
	[PSCI_STATE_L3]   = { "l3", 0, 0 },
	[PSCI_STATE_LLCC] = { "llcc", 0, 0 },
};
static uint32_t psci_idle_core_us[2];           /* a core's least residency: the small cores', the others' */
extern bool hakios_cpu_idle_data(uint32_t *core, uint32_t core_us[2], uint32_t l3[2], uint32_t llcc[2],
    uint64_t *screen_off_cores, uint32_t *soc_cores);

/* the IPI census: what the signal taken asked for */
#define PSCI_IPI_NOP            0       /* nothing: a wake from idle (machine_signal_idle()) */
#define PSCI_IPI_AST            1       /* an AST check: a preemption, a thread made runnable for it */
#define PSCI_IPI_XCALL          2
#define PSCI_IPI_DEC            3       /* the local timer update (SIGPTimerLocal) */
#define PSCI_IPI_OTHER          4
#define PSCI_IPI_KINDS          5
static const char *const psci_ipi_kind_name[PSCI_IPI_KINDS] = { "nop", "ast", "xcall", "dec", "other" };
#define PSCI_IPI_CPUS           8       /* the sender/target table */
#define PSCI_WOKE_INTIDS        6       /* the device interrupts that woke a core, per processor */
#define PSCI_XCALL_FUNCS        8

extern int setPop(uint64_t time);               /* osfmk/arm/rtclock.c */

/* cpu_idle=0: idle cores wait in WFI only (on where the board has cpu-idle { }) */
TUNABLE(uint32_t, psci_idle_level, "cpu_idle", 1);
/* cpu_idle_cores=MASK: the processors that may power down when idle (all) */
TUNABLE(uint64_t, psci_idle_cores, "cpu_idle_cores", ~0ULL);
/* cpu_idle_states=MASK: bit 0 the cores' power-down, 1 l3-off, 2 llcc-off (all the board has) */
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

/* llcc-off diagnostics (POWER-COLLAPSE-PLAN.md step 4): the cluster's gap by bucket; diag 0x40 alternates l3/llcc */
static const uint32_t psci_gap_us[6] = { 1000, 2000, 5309, 8000, 11000, 13959 };
static uint64_t psci_gap_abs[6];
static uint32_t psci_diag_flip;

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
	if (bits & SIGPTimerLocal) {		/* (xnu-12377: the timer update, where 8796 had SIGPdec) */
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
			uint32_t stage = psci_smp_cpu_stage_of(cpu);

			/* one being started (CPU_ON asked, not running yet) is not counted up, but it is not off either:
			 * no cluster state then (the reset of cluster round 1 came in early boot, while the others started) */
			if (cpu != self && (stage == PSCI_SMP_CPU_ASKED || stage == PSCI_SMP_CPU_IN_KERNEL ||
			    /* ... nor while one has never been started at all: early in boot, asking for l3-off with cores
			     * the firmware has never had on resets the phone (variant p); after one CPU_ON/CPU_OFF it is fine */
			    (stage == 0 && cpu != psci_smp_boot_cpu() && cpu_datap(cpu) != NULL))) {
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
	if (state == PSCI_STATE_LLCC) {
		hakios_rpmh_llcc_down();
	}

	c->state = (uint8_t)state;
	c->st_entered[state]++;
	/* (back through the loader's entry, as a start: a core that powered down lost its pointer-authentication keys too) */
	int64_t rc = psci_smp_loader_entry != 0 ?
	    (int32_t)psci_call(PSCI_CPU_SUSPEND64, param, psci_smp_loader_entry, psci_idle_entry) :
	    (int32_t)psci_call(PSCI_CPU_SUSPEND64, param, psci_idle_entry, (uint64_t)cpu);

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
	uint32_t cores = 0, core = 0, l3[2] = { 0, 0 }, llcc[2] = { 0, 0 }, soc_cores = 0;
	uint64_t off = 0;
	bool board = hakios_cpu_idle_data(&core, psci_idle_core_us, l3, llcc, &off, &soc_cores);

	/* (screen_off_cores without the boot-arg: the board's, none without its cpu-idle { }) */
	if (psci_smp_screen_off_cores == PSCI_SCREEN_OFF_BOARD) {
		psci_smp_screen_off_cores = board ? off : 0;
	}
	if (psci_idle_level == 0) {
		psci_say("psci: idle: WFI only (cpu_idle=0)\n");
		return;
	}
	if (!board) {
		psci_say("psci: idle: WFI only (the board has no cpu-idle)\n");
		return;
	}
	psci_cluster_state[PSCI_STATE_CORE].param = core;
	psci_cluster_state[PSCI_STATE_L3].param = l3[0];
	psci_cluster_state[PSCI_STATE_L3].residency_us = l3[1];
	psci_cluster_state[PSCI_STATE_LLCC].param = llcc[0];
	psci_cluster_state[PSCI_STATE_LLCC].residency_us = llcc[1];
	psci_idle_features = (int32_t)psci_call(PSCI_FEATURES, PSCI_CPU_SUSPEND64, 0, 0);
	if (psci_idle_features < 0 || !(psci_idle_features & PSCI_FEATURE_EXTENDED_STATEID)) {
		psci_say("psci: idle: WFI only: the firmware's CPU_SUSPEND %s (features %lld)\n",
		    psci_idle_features < 0 ? "is not there" : "takes the original StateID format", psci_idle_features);
		return;
	}
	if (!(psci_idle_states & (1u << PSCI_STATE_CORE))) {
		psci_say("psci: idle: WFI only (cpu_idle_states 0x%x: no power-down of the cores)\n", psci_idle_states);
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
	for (int state = PSCI_STATE_L3; state < PSCI_STATES; state++) {
		if (psci_cluster_state[state].param == 0) {
			psci_idle_states_live &= ~(1u << state);        /* (not one of this SoC's) */
		}
	}
	/*
	 * A core of the SoC that is not the kernel's (bootcpu=N cpus=N-1: the loader stays on core 0, its UEFI waiting for
	 * the kernel's return) never powers down: the cluster cannot go down with it, and the last of the kernel's cores
	 * would only be refused. Asking for a cluster state with a core the firmware sees as on has reset the phone
	 * (negroni-dev, cluster round 1): not asked for at all.
	 */
	if ((psci_idle_states_live & ~1u) && soc_cores > topology->num_cpus) {
		psci_say("psci: idle: no cluster states: %u of the SoC's %u cores are the kernel's (one is the loader's)\n",
		    topology->num_cpus, soc_cores);
		psci_idle_states_live &= 1u;
	}
	if (psci_idle_states_live & ~1u) {
		/* no other processor is suspended yet: the mode can be changed now (PSCI 1.0, 5.20) */
		if (psci_idle_features & PSCI_FEATURE_OSI) {
			psci_idle_mode_rc = (int32_t)psci_call(PSCI_SET_SUSPEND_MODE, PSCI_MODE_OSI, 0, 0);
			psci_idle_osi = psci_idle_mode_rc == 0;
		} else {
			psci_idle_mode_rc = 1;      /* not asked */
		}
		psci_say("psci: idle: cluster states 0x%x, %s (SET_SUSPEND_MODE %lld)\n", psci_idle_states_live,
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
		nanoseconds_to_absolutetime((uint64_t)psci_idle_core_us[small ? 0 : 1] * NSEC_PER_USEC, &c->residency);
		c->param = psci_cluster_state[PSCI_STATE_CORE].param;
		cores |= 1u << t->cpu_id;
	}
	psci_idle_entry = ml_static_vtop((vm_offset_t)&LowResetVectorBase);
	psci_smp_clean_for_reset();
	os_atomic_store(&psci_idle_ready, true, release);
	psci_say("psci: idle: CPU_SUSPEND features 0x%llx; processors 0x%x power down when idle (0x%x, %u us on the small "
	    "cores, %u us on the others)\n", psci_idle_features, cores, psci_cluster_state[PSCI_STATE_CORE].param,
	    psci_idle_core_us[0], psci_idle_core_us[1]);
}

/*
 * kern.hakios.wakes (GenericARMPE.cpp): what wakes the processors, as text. A line per processor (interrupts taken,
 * of them signals from other processors, timer interrupts), then a line per kernel thread that has been switched to
 * at least 16 times (its id, its name or -, its context switches). Read it twice and subtract (STANDBY-PLAN.md).
 * kern.hakios.idle (brief): the first lines and a cpu line per processor only, under 4 KB (sweetd's sysctl reads that).
 */
static size_t hakios_wakes_text_of(char *buf, size_t size, bool brief);

size_t
hakios_wakes_text(char *buf, size_t size)
{
	return hakios_wakes_text_of(buf, size, false);
}

size_t
hakios_idle_text(char *buf, size_t size)
{
	return hakios_wakes_text_of(buf, size, true);
}

static size_t
hakios_wakes_text_of(char *buf, size_t size, bool brief)
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
	if (!brief) {
		WAKES_PUT("xcall_ref psci_idle_notify %p\n", (void *)VM_KERNEL_UNSLIDE(psci_idle_notify));
	}
	for (int i = 0; !brief && i < PSCI_XCALL_FUNCS; i++) {
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
			if (brief) {
				continue;
			}
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
		}
	}
	if (brief) {
		return at;
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
		WAKES_PUT("\n");
		if (at + 1 >= size) {
			break;
		}
	}
	lck_mtx_unlock(&tasks_threads_lock);
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
		    processor->state != PROCESSOR_PENDING_OFFLINE) {
			up |= 1ULL << cpu;
		}
	}
	return up;
}

/*
 * System sleep, suspend to idle (COMMON-PORT.md "System sleep", step 1): GenericARMPE::sleepKernel() calls this instead
 * of IOCPUSleepKernel(), whose way back is a warm reset this port has none of. The other processors are switched off
 * through the scheduler's powered cores (as with the display off), this thread moves to the boot processor, and that one
 * waits in WFI with its interrupts off and every SPI masked but the wake sources (hakios_sleep_wake_source()). Its timer
 * wakes it every SLEEP_PET_SECONDS to feed the watchdog and show the loader a beat; a wake source, or the end of a test
 * sleep (seconds != 0), ends the wait. Nothing else runs meanwhile: userland, kernel threads and timers wait, and
 * whatever was due is done at once after it.
 */
#define SLEEP_PET_SECONDS       5
#define SLEEP_WAKES_MOST        4
#define SLEEP_INTID_VTIMER      27
#define SLEEP_INTID_NONE_MIN    1020
#define SLEEP_INTID_NONE_MOST   1023
extern uint64_t gic_v3_sleep_mask(bool (*wakes)(uint32_t intid));
extern void gic_v3_sleep_unmask(uint64_t masked);
extern uint32_t gic_v3_pending(void);
extern uint32_t gic_v3_spi_bits(uint32_t intid);
extern uint32_t gic_v3_sleep_take(void);
extern uint64_t wake_abstime;           /* osfmk/arm/cpu_common.c: machdep.wake_abstime */
extern void gic_v3_spi_set_pending(uint32_t intid);
#define GIC_INTID_SPI_MIN_SLEEP 32
extern void hakios_sleep_pet(void);
extern int hakios_its_sleep_mask(bool masked) __attribute__((weak));  /* drivers/board/its.c: the MSIs' LPIs */
int hakios_sleep_wait(uint32_t seconds, uint32_t *woken_by, uint64_t *asleep_ns, uint32_t *pets);
void hakios_sleep_wake_source(uint32_t intid);
static uint32_t sleep_wakes[SLEEP_WAKES_MOST];
static unsigned sleep_wakes_n;

/* A driver's interrupt that wakes the system (the power key's): kept unmasked while it sleeps. */
void
hakios_sleep_wake_source(uint32_t intid)
{
	for (unsigned i = 0; i < sleep_wakes_n; i++) {
		if (sleep_wakes[i] == intid) {
			return;
		}
	}
	if (sleep_wakes_n < SLEEP_WAKES_MOST) {
		sleep_wakes[sleep_wakes_n++] = intid;
		psci_say("sleep: interrupt %u wakes the system\n", intid);
	}
}

static bool
sleep_wakes_on(uint32_t intid)
{
	for (unsigned i = 0; i < sleep_wakes_n; i++) {
		if (sleep_wakes[i] == intid) {
			return true;
		}
	}
	return false;
}

/* 0, or ENODEV (nothing could end it), EBUSY (the other processors did not go). *woken_by: the interrupt that ended it. */
int
hakios_sleep_wait(uint32_t seconds, uint32_t *woken_by, uint64_t *asleep_ns, uint32_t *pets)
{
	int boot = psci_smp_boot_cpu();
	uint64_t boot_bit = 1ULL << boot, masked, ctl, cval, freq, start, end, now;
	uint32_t by = 0, fed = 0, wakes = 0;
	uint64_t pet_at;
	processor_t processor;
	boolean_t enabled;

	*woken_by = 0;
	*asleep_ns = 0;
	*pets = 0;
	if (sleep_wakes_n == 0 && seconds == 0) {
		return ENODEV;
	}
	if (ml_get_cpu_count() > 1) {
		sched_perfcontrol_update_powered_cores(boot_bit, REASON_CLPC_SYSTEM, 0);
		for (int i = 0; i < 200 && psci_smp_cores_up() != boot_bit; i++) {
			delay_for_interval(10, NSEC_PER_MSEC);
		}
		if (psci_smp_cores_up() != boot_bit) {
			psci_say("sleep: refused: processors 0x%llx still up\n", psci_smp_cores_up());
			psci_smp_apply_display();
			return EBUSY;
		}
	}
	processor = processor_array[boot];
	thread_bind(processor);
	thread_block(THREAD_CONTINUE_NULL);     /* (onto it) */

	/* (the LPIs too: the radio's MSIs, no wake source yet, ended every sleep at once; they stay pending) */
	int lpis = hakios_its_sleep_mask ? hakios_its_sleep_mask(true) : 0;
	enabled = ml_set_interrupts_enabled(FALSE);
	masked = gic_v3_sleep_mask(sleep_wakes_on);
	ctl = __builtin_arm_rsr64("CNTV_CTL_EL0");
	cval = __builtin_arm_rsr64("CNTV_CVAL_EL0");
	freq = __builtin_arm_rsr64("CNTFRQ_EL0");
	start = __builtin_arm_rsr64("CNTVCT_EL0");
	end = seconds != 0 ? start + (uint64_t)seconds * freq : UINT64_MAX;
	/*
	 * The pets on a fixed schedule, every SLEEP_PET_SECONDS from the start, and one now. The first version set its timer to
	 * "now + 5 s" at every wake, early ones included: woken early again and again, the pet moved on each time and was never
	 * due (1 pet in a 20 s sleep, 0 or 1 in 10 s ones), and a 600 s test sleep starved the watchdog into a reset
	 * (run-20261126-044816: the phone came back in iOS 16).
	 */
	/*
	 * mach_absolute_time() stands still across the sleep, as on a wake from S2R: cpu_sleep() takes wake_abstime here and
	 * cpu_timebase_init() rebases the clock on it at the wake (below). clock_wakeup_calendar() (the root domain, at the
	 * wake) then counts the sleep from the RTC: mach_continuous_time() moves past mach_absolute_time() and
	 * machdep.wake_abstime says when. With the clock ticking on, the sleep was one of no length to userland:
	 * SystemWake's sleep monitor never saw it end, and the first power key press after a wake did nothing
	 * (run-20261126-055748, -060515).
	 */
	wake_abstime = ml_get_timebase();
	hakios_sleep_pet();
	fed++;
	pet_at = start + SLEEP_PET_SECONDS * freq;
	for (;;) {
		uint64_t next;
		uint32_t intid;

		now = __builtin_arm_rsr64("CNTVCT_EL0");
		if (now >= end) {
			by = SLEEP_INTID_VTIMER;
			break;
		}
		if (now >= pet_at) {
			hakios_sleep_pet();
			fed++;
			while (pet_at <= now) {
				pet_at += SLEEP_PET_SECONDS * freq;
			}
		}
		next = pet_at < end ? pet_at : end;
		__builtin_arm_wsr64("CNTV_CVAL_EL0", next);
		__builtin_arm_wsr64("CNTV_CTL_EL0", 1);         /* enabled, not masked */
		__builtin_arm_isb(ISB_SY);
		__builtin_arm_dsb(DSB_SY);
		__builtin_arm_wfi();
		wakes++;
		/*
		 * A wake source by its own pending bit: the timer's interrupt was left pending at the CPU interface, ahead of the
		 * power key's, so every WFI ended at once (300 million wakes in a 60 s sleep) and the key never showed there while
		 * its distributor bit said pending (run-20261126-055053). Now taken off (below), it is a few wakes a sleep.
		 */
		for (unsigned i = 0; i < sleep_wakes_n && by == 0; i++) {
			if (gic_v3_spi_bits(sleep_wakes[i]) & 2) {
				by = sleep_wakes[i];
			}
		}
		if (by != 0) {
			break;
		}
		intid = gic_v3_pending();
		if (intid == SLEEP_INTID_VTIMER) {
			/* (taken and ended, so it stops ending every WFI; a wake source taken here instead is made pending again) */
			uint32_t took = gic_v3_sleep_take();

			if (took != SLEEP_INTID_VTIMER && took < SLEEP_INTID_NONE_MIN) {
				if (sleep_wakes_on(took)) {
					gic_v3_spi_set_pending(took);
					by = took;
					break;
				}
				if (took >= GIC_INTID_SPI_MIN_SLEEP) {
					gic_v3_spi_set_pending(took);
				}
			}
		}
		if (intid == SLEEP_INTID_VTIMER || (intid >= SLEEP_INTID_NONE_MIN && intid <= SLEEP_INTID_NONE_MOST)) {
			continue;       /* (the pet and the end are looked at above) */
		}
		by = intid;     /* (left pending: its handler takes it once interrupts are on again) */
		break;
	}
	now = __builtin_arm_rsr64("CNTVCT_EL0");
	__builtin_arm_wsr64("CNTV_CVAL_EL0", cval);     /* (the kernel's deadline, passed by now: taken at once) */
	__builtin_arm_wsr64("CNTV_CTL_EL0", ctl);
	__builtin_arm_isb(ISB_SY);
	/* (the other processors are off: each takes rtclock_base_abstime when it starts again, cpu_timebase_init()) */
	rtclock_base_abstime = wake_abstime - ml_get_hwclock();
	getCpuDatap()->cpu_base_timebase = rtclock_base_abstime;
	commpage_update_timebase();
	getCpuDatap()->rtcPop = EndOfAllTime;   /* (the deadline anew, in the clock as it now is) */
	timer_resync_deadlines();
	gic_v3_sleep_unmask(masked);
	ml_set_interrupts_enabled(enabled);
	if (hakios_its_sleep_mask) {
		(void)hakios_its_sleep_mask(false);
	}
	if (lpis != 0) {
		psci_say("sleep: the LPIs could not all be masked (%d)\n", lpis);
	}

	thread_bind(PROCESSOR_NULL);
	thread_block(THREAD_CONTINUE_NULL);
	psci_smp_apply_display();
	*woken_by = by;
	*asleep_ns = (now - start) / freq * NSEC_PER_SEC + (now - start) % freq * NSEC_PER_SEC / freq;
	*pets = fed;
	psci_say("sleep: %u wakes of the wait, %u pets\n", wakes, fed);
	return 0;
}

#endif /* defined(GENERIC_PSCI_SMP) */
