/*
 * The clock speed of the processor cores the kernel runs on, on the POCO X5 Pro (redwood).
 * Only with boot-arg cpufreq=1: the addresses are this phone's.
 *
 * The boot loader leaves each group of cores at a modest frequency and expects the operating
 * system to ask for more. On this SoC that is one register write: the hardware ("EPSS") has
 * a table of frequencies per group and switches the clock source and the supply voltage by
 * itself when told which row is wanted. There is no reason for us ever to want less than the
 * top row, for every group that has a core of ours in it (one core, unless boot-arg cpus=N
 * says more).
 *
 * Register layout: Linux drivers/cpufreq/qcom-cpufreq-hw.c ("epss" variant).
 *
 * Elsewhere (boot-arg cpufreq_epss=<pa>, the OnePlus 10 Pro's 0x17d91000): the same, and a cap the thermal
 * driver lowers when the phone gets hot (hakios_cpufreq_cap(); never below the row the firmware left), and
 * sysctl kern.hakios.cpufreq ("d0 1785/1785 d1 2496/2496 d2 2995/2995 cap 0": asked / allowed by the hardware).
 * The firmware's own limits (LMh) act in the hardware whatever is asked; the second number shows them.
 * There, a governor picks each group's row from how busy its cores are (cpufreq_gov, below), instead of the top row
 * all the time: idle at the top rows the cores sat at 67-75 C, held there only by the firmware's limiter.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <os/atomic_private.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/hakios.mmio.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
extern void loader_log_putc(char c);
extern void hakios_tl(const char *format, ...) __printflike(1, 2);
extern bool ml_cpu_idle_time(int cpu_id, uint64_t *idle);
extern void hakios_screen_pause_leeway(uint32_t on_ms, uint32_t off_ms, uint32_t leeway_ms);
extern bool hakios_screen_off(void);
extern void thread_set_thread_name(thread_t thread, const char *name);

/*
 * The loader's log as well: this runs early, and the kernel's own log of a run keeps only its end. With
 * interrupts off -- on the way back to the loader (hakios_ddr_quiesce()) -- the loader's log only: the
 * kernel's console may not be written there, and printf was a panic ("Console I/O from interrupt-disabled
 * context") at the end of every run once the memory vote was taken away on the way back.
 */
static void
tell(const char *format, ...) __printflike(1, 2);

static void
tell(const char *format, ...)
{
	char line[256];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	if (ml_get_interrupts_enabled()) {
		printf("%s", line);
	}
}

#define EPSS_PHYS               0x18591000UL    /* three groups ("domains") of 0x1000: small cores, big, biggest */
#define EPSS_PHYS_8450          0x17d91000UL
#define EPSS_DOMAIN_SIZE        0x1000UL
#define EPSS_DOMAINS            3
#define EPSS_DOMAIN_STATE       0x20            /* [7:0]: the multiple of 19.2 MHz the hardware allows now (LMh's limit) */
#define EPSS_FREQ_TABLE         0x100           /* a word per row: [31:30] clock source, [7:0] multiple of 19.2 MHz */
#define EPSS_TABLE_ROWS         40
#define EPSS_PERF_STATE         0x320           /* the row that is wanted */
#define XO_KHZ                  19200
#define ALTERNATE_KHZ           300000          /* source 0: a fixed 300 MHz */

/*
 * Boot-arg cpufreq_epss=<pa>: the first domain's registers, when this is not the reference phone (the OnePlus 10 Pro,
 * SM8450: 0x17d91000, dt qcom,cpufreq-hw). The memory vote below (ddr=) has the reference phone's addresses and is
 * only made there unless ddr= asks for it.
 */
static vm_offset_t epss_phys = EPSS_PHYS;

/*
 * What was found and done per domain: its table, the row the firmware left, the top row, the row asked for now, and
 * the governor's: the row the load wants (gov_row; the top row when there is no governor), and its bookkeeping.
 */
static struct {
	vm_offset_t     base;
	uint32_t        rows, khz[EPSS_TABLE_ROWS];
	uint32_t        firmware_row, top_row, row;
	bool            ours;
	uint32_t        gov_row, lower_wanted, lower_best;
	uint64_t        mhz_time, busy_time, time, changes;
} dom[EPSS_DOMAINS];
static uint32_t cap_khz;                        /* 0: none; hakios_cpufreq_cap() */
static LCK_GRP_DECLARE(cpufreq_lock_group, "hakios-cpufreq");
static LCK_MTX_DECLARE(cpufreq_lock, &cpufreq_lock_group);     /* dom[].row and its register: the cap and the governor */
static volatile bool cpufreq_leaving;          /* on the way back to the loader: the rows are no longer ours */
static char cpufreq_line[160];

SYSCTL_DECL(_kern_hakios);
SYSCTL_STRING(_kern_hakios, OID_AUTO, cpufreq, CTLFLAG_RD | CTLFLAG_LOCKED, cpufreq_line, 0,
    "per domain of cores: the MHz asked for, and the MHz the hardware allows (lmh): \"d0 1785/1785 d1 ...\"; cap");

static uint32_t
row_khz(uint32_t entry)
{
	return (entry >> 30) ? XO_KHZ * (entry & 0xff) : ALTERNATE_KHZ;
}

/* The sysctl's line, read from the hardware: what was asked, what LMh allows now. */
static void
cpufreq_describe(void)
{
	int at = 0;

	cpufreq_line[0] = 0;
	for (uint32_t d = 0; d < EPSS_DOMAINS; d++) {
		if (!dom[d].ours) {
			continue;
		}
		uint32_t asked = *(volatile uint32_t *)(dom[d].base + EPSS_PERF_STATE);
		uint32_t lmh = (*(volatile uint32_t *)(dom[d].base + EPSS_DOMAIN_STATE) & 0xff) * XO_KHZ;

		at += snprintf(cpufreq_line + at, sizeof(cpufreq_line) - at, "d%u %u/%u ", d,
		    asked < dom[d].rows ? dom[d].khz[asked] / 1000 : 0, lmh / 1000);
		if (at >= (int)sizeof(cpufreq_line) - 1) {
			return;
		}
	}
	snprintf(cpufreq_line + at, sizeof(cpufreq_line) - at, "cap %u", cap_khz / 1000);
}

/* How fast a simple loop runs, in thousands of iterations per millisecond. */
static uint32_t
loop_speed(void)
{
	volatile uint64_t sink = 0;
	uint64_t start = mach_absolute_time(), nanos;

	for (uint32_t i = 0; i < 2000000; i++) {
		sink += i;
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - start, &nanos);
	return nanos ? (uint32_t)(2000000ULL * 1000 / nanos) : 0;
}

/* The group of the core with that number in its MPIDR (the device tree's "reg": the number is the second field). */
static uint32_t
domain_of(uint32_t phys_id)
{
	uint32_t core = (phys_id >> 8) & 0xff;

	return core < 4 ? 0 : core < 7 ? 1 : 2;
}

/* The highest row not above the cap, and never below the row the firmware left (a known-good one). */
static uint32_t
row_for_cap(uint32_t d)
{
	uint32_t row = dom[d].top_row;

	while (cap_khz != 0 && row > dom[d].firmware_row && dom[d].khz[row] > cap_khz) {
		row--;
	}
	return row;
}

/* The row a domain is to have: what the governor wants, no higher than the cap allows; written when it changed. With cpufreq_lock. */
static void
apply(uint32_t d)
{
	uint32_t row = MIN(dom[d].gov_row, row_for_cap(d));

	if (row != dom[d].row && !cpufreq_leaving) {
		dom[d].row = row;
		dom[d].changes++;
		*(volatile uint32_t *)(dom[d].base + EPSS_PERF_STATE) = row;
	}
}

static void
raise_domain(uint32_t domain, uint32_t cores)
{
	uint32_t top = 0, top_khz = 0, was, at = 0;
	char table[200];
	vm_offset_t base;

	base = ml_io_map(epss_phys + domain * EPSS_DOMAIN_SIZE, EPSS_DOMAIN_SIZE);
	table[0] = 0;
	for (uint32_t row = 0; row < EPSS_TABLE_ROWS; row++) {
		uint32_t khz = row_khz(*(volatile uint32_t *)(base + EPSS_FREQ_TABLE + 4 * row));

		if (row > 0 && khz <= top_khz) {
			break;                  /* the table ends by repeating its last row */
		}
		dom[domain].khz[row] = khz;
		top = row;
		top_khz = khz;
		if (at < sizeof(table) - 8) {
			at += snprintf(table + at, sizeof(table) - at, " %u", khz / 1000);
		}
	}
	was = *(volatile uint32_t *)(base + EPSS_PERF_STATE);
	dom[domain].base = base;
	dom[domain].rows = top + 1;
	dom[domain].top_row = top;
	dom[domain].firmware_row = was <= top ? was : top;
	dom[domain].ours = true;
	dom[domain].gov_row = top;
	dom[domain].row = row_for_cap(domain);
	*(volatile uint32_t *)(base + EPSS_PERF_STATE) = dom[domain].row;
	delay_for_interval(20, NSEC_PER_MSEC);
	tell("cpufreq: group %u (%u of our cores): table (MHz):%s\n", domain, cores, table);
	tell("cpufreq: group %u: row %u (%u MHz, the firmware's) -> %u, %u MHz; the hardware allows %u MHz\n", domain, was,
	    was <= top ? dom[domain].khz[was] / 1000 : 0, dom[domain].row, dom[domain].khz[dom[domain].row] / 1000,
	    (*(volatile uint32_t *)(base + EPSS_DOMAIN_STATE) & 0xff) * XO_KHZ / 1000);
}

/*
 * For the thermal driver (hakios.thermal.c): no domain above mhz (0: no cap), never below the row the firmware left.
 * Called from one thread only (the thermal one), after hakios_cpufreq_init(); the governor's thread may write the rows too.
 */
void hakios_cpufreq_cap(uint32_t mhz);

void
hakios_cpufreq_cap(uint32_t mhz)
{
	lck_mtx_lock(&cpufreq_lock);
	cap_khz = mhz * 1000;
	for (uint32_t d = 0; d < EPSS_DOMAINS; d++) {
		if (dom[d].ours) {
			apply(d);
		}
	}
	lck_mtx_unlock(&cpufreq_lock);
	cpufreq_describe();
	tell("cpufreq: cap %u MHz: %s\n", mhz, cpufreq_line);
}

/* The line of kern.hakios.cpufreq, fresh (the thermal driver logs it with its own). */
const char *hakios_cpufreq_line(void);

const char *
hakios_cpufreq_line(void)
{
	cpufreq_describe();
	return cpufreq_line;
}

/*
 * The governor. Boot-arg cpufreq_gov=0: none, every group at its top row (as before); 1 (the default with cpufreq_epss=,
 * the OnePlus 10 Pro, where it was measured; elsewhere only when asked for): every cpufreq_gov_ms milliseconds (default 20) it looks at how busy the busiest of our cores of each
 * group was meanwhile (the time it was not idle, as cpuload= reports it), at the frequency the group really ran at (the
 * row asked, or less when the firmware's limiter allows less), and picks:
 *   busier than GOV_UP percent      the top row, at once (whatever the cap allows): a busy core is not kept waiting
 *   otherwise                       the lowest row at which that core would be about GOV_AIM percent busy
 * A higher row is taken at once; a lower one only once GOV_DOWN periods in a row have wanted lower, and then the highest
 * of what they wanted, so that a short pause in the work does not drop the clock. Any row of the table may be picked,
 * down to the lowest (300-800 MHz): the hardware sets the voltage for each row itself. The thermal cap stays a ceiling.
 * sysctl kern.hakios.cpufreq_gov, every second: per group the average MHz since start, the busiest core's average
 * percent, and how many times the row changed: "d0 412 MHz 6% 1520 ...".
 */
#define GOV_UP          80
#define GOV_AIM         70
#define GOV_DOWN        5
static uint32_t gov_ms = 20;
static char gov_line[160];

SYSCTL_STRING(_kern_hakios, OID_AUTO, cpufreq_gov, CTLFLAG_RD | CTLFLAG_LOCKED, gov_line, 0,
    "per group of cores, since start: the average MHz, the busiest core's average percent, the row changes");

/* The row a group's load wants: busy (percent) of its busiest core, at khz. */
static uint32_t
gov_want(uint32_t d, uint32_t busy, uint32_t khz)
{
	uint64_t target;
	uint32_t row = 0;

	if (busy > GOV_UP) {
		return dom[d].top_row;
	}
	target = (uint64_t)khz * busy / GOV_AIM;
	while (row < dom[d].top_row && dom[d].khz[row] < target) {
		row++;
	}
	return row;
}

static void
gov_describe(void)
{
	int at = 0;

	gov_line[0] = 0;
	for (uint32_t d = 0; d < EPSS_DOMAINS; d++) {
		if (!dom[d].ours || dom[d].time == 0) {
			continue;
		}
		at += snprintf(gov_line + at, sizeof(gov_line) - at, "%sd%u %llu MHz %llu%% %llu", at ? " " : "", d,
		    dom[d].mhz_time / dom[d].time, dom[d].busy_time / dom[d].time, dom[d].changes);
		if (at >= (int)sizeof(gov_line) - 1) {
			return;
		}
	}
}

static void floor_follow(const uint32_t khz[EPSS_DOMAINS]);  /* the DDR/LLCC floor (step 5b, below) */

static void
gov_thread(void *arg __unused, wait_result_t wr __unused)
{
	const ml_topology_info_t *topology = ml_get_topology_info();
	uint64_t idle_before[MAX_CPUS] = { 0 }, then = mach_absolute_time(), described = then, second;

	nanoseconds_to_absolutetime(NSEC_PER_SEC, &second);
	for (unsigned int i = 0; i < topology->num_cpus && i < MAX_CPUS; i++) {
		(void)ml_cpu_idle_time((int)topology->cpus[i].cpu_id, &idle_before[i]);
	}
	while (!cpufreq_leaving) {
		uint32_t busiest[EPSS_DOMAINS] = { 0 }, asked_khz[EPSS_DOMAINS] = { 0 };
		uint64_t now;

		/*
		 * (the load is light with the screen off; STANDBY-PLAN.md): 400 ms, or up to 600 ms later with another wake.
		 * At 100 + 150 it was still cpu0's most frequent timer of its own, 5 a second (run19); a burst with the screen
		 * off waits up to a second for its step up.
		 */
		hakios_screen_pause_leeway(gov_ms, MAX(gov_ms, 400), 600);
		now = mach_absolute_time();
		if (now == then) {
			continue;
		}
		for (unsigned int i = 0; i < topology->num_cpus && i < MAX_CPUS; i++) {
			uint64_t idle = 0, idled, busy;
			uint32_t d = domain_of(topology->cpus[i].phys_id);

			if (!ml_cpu_idle_time((int)topology->cpus[i].cpu_id, &idle)) {
				continue;
			}
			idled = idle >= idle_before[i] ? idle - idle_before[i] : 0;
			idle_before[i] = idle;
			busy = idled < now - then ? 100 * (now - then - idled) / (now - then) : 0;
			busiest[d] = MAX(busiest[d], (uint32_t)busy);
		}
		lck_mtx_lock(&cpufreq_lock);
		for (uint32_t d = 0; d < EPSS_DOMAINS; d++) {
			uint32_t asked, lmh, khz, want;

			if (!dom[d].ours) {
				continue;
			}
			asked = dom[d].khz[dom[d].row];
			lmh = (*(volatile uint32_t *)(dom[d].base + EPSS_DOMAIN_STATE) & 0xff) * XO_KHZ;
			khz = lmh != 0 ? MIN(asked, lmh) : asked;
			dom[d].mhz_time += (uint64_t)(khz / 1000) * (now - then);
			dom[d].busy_time += (uint64_t)busiest[d] * (now - then);
			dom[d].time += now - then;
			want = gov_want(d, busiest[d], khz);
			if (want >= dom[d].gov_row) {
				dom[d].gov_row = want;
				dom[d].lower_wanted = 0;
			} else {
				dom[d].lower_best = dom[d].lower_wanted ? MAX(dom[d].lower_best, want) : want;
				if (++dom[d].lower_wanted >= GOV_DOWN) {
					dom[d].gov_row = dom[d].lower_best;
					dom[d].lower_wanted = 0;
				}
			}
			apply(d);
			asked_khz[d] = dom[d].khz[dom[d].row];
		}
		lck_mtx_unlock(&cpufreq_lock);
		floor_follow(asked_khz);
		then = now;
		if (now - described >= second) {
			gov_describe();
			cpufreq_describe();
			described = now;
		}
	}
}

static void
gov_start(void)
{
	uint32_t on = epss_phys != EPSS_PHYS;
	thread_t thread;

	PE_parse_boot_argn("cpufreq_gov", &on, sizeof(on));
	PE_parse_boot_argn("cpufreq_gov_ms", &gov_ms, sizeof(gov_ms));
	gov_ms = MAX(5, MIN(gov_ms, 1000));
	if (!on) {
		tell("cpufreq: no governor (cpufreq_gov=0): every group at its top row\n");
		return;
	}
	if (kernel_thread_start(gov_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "cpufreq-gov");
		thread_deallocate(thread);
		tell("cpufreq: governor every %u ms: the top row above %u%% busy, else about %u%% busy; down after %u periods\n",
		    gov_ms, GOV_UP, GOV_AIM, GOV_DOWN);
	}
}

/*
 * Boot-arg membench=1: how fast this processor reads, writes and copies memory, in MB a second
 * (32 MB at a time, far more than any cache): what the memory's clock allows a processor, which
 * nobody here asks to be raised.
 */
static void
memory_bench(void)
{
	const vm_size_t size = 32u << 20;
	vm_offset_t a = 0, b = 0;
	uint64_t began, ns[3] = { 0, 0, 0 };
	volatile uint64_t sum = 0;

	if (kmem_alloc(kernel_map, &a, size, KMA_KOBJECT | KMA_DATA, VM_KERN_MEMORY_DIAG) != KERN_SUCCESS ||
	    kmem_alloc(kernel_map, &b, size, KMA_KOBJECT | KMA_DATA, VM_KERN_MEMORY_DIAG) != KERN_SUCCESS) {
		printf("membench: no memory\n");
		return;
	}
	memset((void *)a, 1, size);
	memset((void *)b, 2, size);
	for (int round = 0; round < 4; round++) {
		uint64_t t;

		began = mach_absolute_time();
		memset((void *)a, round, size);
		absolutetime_to_nanoseconds(mach_absolute_time() - began, &t);
		ns[0] += t;
		began = mach_absolute_time();
		for (const uint64_t *p = (const uint64_t *)b, *end = (const uint64_t *)(b + size); p < end; p += 8) {
			sum += p[0] + p[1] + p[2] + p[3] + p[4] + p[5] + p[6] + p[7];
		}
		absolutetime_to_nanoseconds(mach_absolute_time() - began, &t);
		ns[1] += t;
		began = mach_absolute_time();
		memcpy((void *)b, (const void *)a, size);
		absolutetime_to_nanoseconds(mach_absolute_time() - began, &t);
		ns[2] += t;
	}
	tell("membench: on core %llu: writing %llu MB/s, reading %llu MB/s, copying %llu MB/s\n",
	    (__builtin_arm_rsr64("MPIDR_EL1") >> 8) & 0xff, 4ull * (size >> 20) * 1000000000ull / ns[0],
	    4ull * (size >> 20) * 1000000000ull / ns[1], 4ull * (size >> 20) * 1000000000ull / ns[2]);
	kmem_free(kernel_map, a, size);
	kmem_free(kernel_map, b, size);
}

/*
 * Boot-arg membench=2: the same on every core of ours at once (a thread each, started together, 16 MB
 * at a time), 2 s after this starts, so that all the cores are up: the sum of what they get is what
 * the memory gives at the clock it has been left at.
 */
#define BENCH_THREADS   8
static struct {
	volatile uint32_t ready, go, done;
	uint64_t ns[BENCH_THREADS][3];
	uint32_t core[BENCH_THREADS];
} bench;

static void
bench_thread(void *arg, wait_result_t wr __unused)
{
	const vm_size_t size = 16u << 20;
	uintptr_t index = (uintptr_t)arg;
	vm_offset_t a = 0, b = 0;
	volatile uint64_t sum = 0;

	if (kmem_alloc(kernel_map, &a, size, KMA_KOBJECT | KMA_DATA, VM_KERN_MEMORY_DIAG) != KERN_SUCCESS ||
	    kmem_alloc(kernel_map, &b, size, KMA_KOBJECT | KMA_DATA, VM_KERN_MEMORY_DIAG) != KERN_SUCCESS) {
		os_atomic_inc(&bench.ready, relaxed);
		os_atomic_inc(&bench.done, relaxed);
		return;
	}
	memset((void *)a, 1, size);
	memset((void *)b, 2, size);
	os_atomic_inc(&bench.ready, relaxed);
	while (!os_atomic_load(&bench.go, acquire)) {
	}
	bench.core[index] = (uint32_t)(__builtin_arm_rsr64("MPIDR_EL1") >> 8) & 0xff;
	for (int round = 0; round < 4; round++) {
		uint64_t began, t;

		began = mach_absolute_time();
		memset((void *)a, round, size);
		absolutetime_to_nanoseconds(mach_absolute_time() - began, &t);
		bench.ns[index][0] += t;
		began = mach_absolute_time();
		for (const uint64_t *p = (const uint64_t *)b, *end = (const uint64_t *)(b + size); p < end; p += 8) {
			sum += p[0] + p[1] + p[2] + p[3] + p[4] + p[5] + p[6] + p[7];
		}
		absolutetime_to_nanoseconds(mach_absolute_time() - began, &t);
		bench.ns[index][1] += t;
		began = mach_absolute_time();
		memcpy((void *)b, (const void *)a, size);
		absolutetime_to_nanoseconds(mach_absolute_time() - began, &t);
		bench.ns[index][2] += t;
	}
	os_atomic_inc(&bench.done, release);
	kmem_free(kernel_map, a, size);
	kmem_free(kernel_map, b, size);
}

static void
bench_all(void *arg __unused, wait_result_t wr __unused)
{
	const uint32_t threads = MIN(ml_get_topology_info()->num_cpus, BENCH_THREADS);
	const uint64_t mb = 4 * 16;                     /* per thread and test */
	uint64_t total[3] = { 0, 0, 0 };

	delay_for_interval(2000, NSEC_PER_MSEC);
	for (uintptr_t i = 0; i < threads; i++) {
		thread_t thread;

		if (kernel_thread_start(bench_thread, (void *)i, &thread) == KERN_SUCCESS) {
			thread_deallocate(thread);
		} else {
			os_atomic_inc(&bench.ready, relaxed);
			os_atomic_inc(&bench.done, relaxed);
		}
	}
	while (os_atomic_load(&bench.ready, relaxed) < threads) {
		delay_for_interval(1, NSEC_PER_MSEC);
	}
	os_atomic_store(&bench.go, 1, release);
	while (os_atomic_load(&bench.done, acquire) < threads) {
		delay_for_interval(1, NSEC_PER_MSEC);
	}
	for (uint32_t i = 0; i < threads; i++) {
		tell("membench: all at once: core %u: writing %llu MB/s, reading %llu MB/s, copying %llu MB/s\n", bench.core[i],
		    bench.ns[i][0] ? mb * 1000000000ull / bench.ns[i][0] : 0, bench.ns[i][1] ? mb * 1000000000ull / bench.ns[i][1] : 0,
		    bench.ns[i][2] ? mb * 1000000000ull / bench.ns[i][2] : 0);
		for (int k = 0; k < 3; k++) {
			total[k] += bench.ns[i][k] ? mb * 1000000000ull / bench.ns[i][k] : 0;
		}
	}
	tell("membench: all %u at once, together: writing %llu MB/s, reading %llu MB/s, copying %llu MB/s\n", threads, total[0],
	    total[1], total[2]);
}

/*
 * Boot-arg cpuload=MS: every MS milliseconds, how busy each of our cores was meanwhile, in
 * percent, to the loader's log (tools/frames.py shows it per gesture):
 *   tl: u at US 7:NN 6:NN ... 1:NN      the core's number (as in its MPIDR), then its share
 * of the time it was not idle (interrupts included).
 */
static uint32_t load_every_ms;

static void
load_thread(void *arg __unused, wait_result_t wr __unused)
{
	const ml_topology_info_t *topology = ml_get_topology_info();
	uint64_t idle_before[MAX_CPUS] = { 0 }, then = mach_absolute_time();

	for (;;) {
		char line[192];
		int at;
		uint64_t now, us;

		delay_for_interval(load_every_ms, NSEC_PER_MSEC);
		now = mach_absolute_time();
		absolutetime_to_nanoseconds(now, &us);
		at = snprintf(line, sizeof(line), "tl: u at %llu", us / 1000);
		for (unsigned int i = 0; i < topology->num_cpus && i < MAX_CPUS; i++) {
			uint64_t idle = 0, busy;

			if (!ml_cpu_idle_time((int)i, &idle)) {
				continue;
			}
			busy = (now - then) > (idle - idle_before[i]) ? (now - then) - (idle - idle_before[i]) : 0;
			idle_before[i] = idle;
			if (at < (int)sizeof(line) - 12) {
				at += snprintf(line + at, sizeof(line) - at, " %u:%llu", (topology->cpus[i].phys_id >> 8) & 0xff,
				    100 * busy / (now - then));
			}
		}
		then = now;
		hakios_tl("%s\n", line);
	}
}

static void
load_start(void)
{
	thread_t thread;

	if (!PE_parse_boot_argn("cpuload", &load_every_ms, sizeof(load_every_ms)) || load_every_ms == 0) {
		return;
	}
	if (kernel_thread_start(load_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

/*
 * Boot-arg ddr=1: the memory's bus clock managers ("BCM") in the command database the boot loader
 * left, with what is needed to vote for bandwidth on them -- only read. (membench=2 says the cores
 * together get no more from memory than one alone: 12 GB/s; nobody here votes for more.)
 */
#define CMD_DB_PHYS             0x80860000UL
#define CMD_DB_SIZE             0x20000UL
#define CMD_DB_BCM              5               /* the kind of resource in a slave's id: bus clock managers */

struct cmd_db_slave {
	uint16_t id, header_offset, data_offset, count, version, reserved[3];
};
struct cmd_db_header {
	uint32_t version;
	uint8_t magic[4];
	struct cmd_db_slave slave[8];
	uint32_t checksum, reserved;
	uint8_t data[];
};
/*
 * The command database (memory, not registers: mapped so that reads of any size and alignment are allowed), mapped
 * once. ss_resource() mapped its 128 KB again for every RSC slot it named, on every read of kern.hakios.rpmh_stats,
 * and never let go: about a dozen reads used up the kernel's I/O mappings, and the panic, from a system call, reset the
 * phone with nothing logged (run21-run31).
 */
static const struct cmd_db_header *cmd_db_mapped(void);

struct cmd_db_entry {
	char id[8];
	uint32_t priority[2];
	uint32_t address;
	uint16_t length, offset;
};
struct cmd_db_bcm {                             /* a BCM's auxiliary data (Linux: struct bcm_db) */
	uint32_t unit;
	uint16_t width;
	uint8_t vcd, reserved;
};

/* (the same condition as the cores' clocks: the addresses are this phone's) */
static bool
cpufreq_wanted(void)
{
	uint32_t on = 0;

	return PE_parse_boot_argn("cpufreq", &on, sizeof(on)) && on != 0;
}

static const struct cmd_db_header *
cmd_db_mapped(void)
{
	static const struct cmd_db_header *_Atomic db;
	const struct cmd_db_header *mine = os_atomic_load(&db, acquire), *none = NULL;

	if (mine == NULL) {
		mine = (const struct cmd_db_header *)ml_io_map_wcomb(CMD_DB_PHYS, CMD_DB_SIZE);
		if (!os_atomic_cmpxchgv(&db, none, mine, &none, acq_rel)) {
			mine = none;    /* (another one's mapping won; this one stays mapped, once) */
		}
	}
	return mine;
}

static void
ddr_look(void)
{
	/* (memory, not registers: mapped so that reads of any size and alignment are allowed) */
	const struct cmd_db_header *db = cmd_db_mapped();

	if (db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		tell("ddr: no command database\n");
		return;
	}
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &db->slave[s];
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);

		if (slave->id != CMD_DB_BCM || slave->header_offset >= CMD_DB_SIZE - 0x1000 || slave->count > 512) {
			continue;
		}
		for (unsigned i = 0; i < slave->count; i++) {
			struct cmd_db_bcm aux = { 0 };
			char name[9] = { 0 };

			memcpy(name, entry[i].id, 8);
			/* (the data starts after the header, so the header counts: this keeps the read inside the mapped database) */
			if (entry[i].length >= sizeof(aux) &&
			    sizeof(*db) + slave->data_offset + entry[i].offset + sizeof(aux) <= CMD_DB_SIZE) {
				memcpy(&aux, db->data + slave->data_offset + entry[i].offset, sizeof(aux));
			}
			tell("ddr: BCM %-8s at %#x: unit %u, width %u, vcd %u (aux %u bytes)\n", name, entry[i].address, aux.unit,
			    aux.width, aux.vcd, entry[i].length);
		}
	}
}

/*
 * ... and what the boot loader's firmware left in the application processor's RPMh command slots
 * ("DRV 2"; Linux: rpmh-rsc.c): 2 active, 3 sleep, 3 wake slot sets and a control one, of 16 commands
 * each (sc7280's qcom,tcs-config): every command with an address, and which of them go to the BCMs
 * above -- the bandwidth it asked for, and that is in force whenever the processors are awake.
 */
#define RSC_PHYS                0x18220000UL
#define RSC_SIZE                0x10000UL
#define RSC_TCS(n, reg)         (0xd00 + 0x2a0 * (n) + (reg))
#define RSC_TCS_COUNT           9
#define RSC_CMDS                16

static void
ddr_slots(void)
{
	vm_offset_t rsc = ml_io_map(RSC_PHYS, RSC_SIZE);

	for (unsigned t = 0; t < RSC_TCS_COUNT; t++) {
		uint32_t enabled = *(volatile uint32_t *)(rsc + RSC_TCS(t, 0x1c));

		for (unsigned c = 0; c < RSC_CMDS; c++) {
			uint32_t msgid = *(volatile uint32_t *)(rsc + RSC_TCS(t, 0x30 + 20 * c));
			uint32_t address = *(volatile uint32_t *)(rsc + RSC_TCS(t, 0x34 + 20 * c));
			uint32_t data = *(volatile uint32_t *)(rsc + RSC_TCS(t, 0x38 + 20 * c));

			if (address == 0 && data == 0) {
				continue;
			}
			tell("ddr: slot set %u (%s) command %u%s: message %#x, address %#x, data %#x%s\n", t,
			    t < 2 ? "active" : t < 5 ? "sleep" : t < 8 ? "wake" : "control", c, enabled & (1u << c) ? " (on)" : "",
			    msgid, address, data, (address >> 16 & 0xf) == CMD_DB_BCM ? " <- a BCM" : "");
		}
	}
}

/*
 * The memory's bandwidth (on by default; boot-arg ddr=0: not): a vote through RPMh -- the most there
 * is -- on the bus clock managers of the memory controller (MC0: the memory's clock) and of the
 * shared hub in front of it with the system cache (SH0), as Linux's interconnect driver casts for
 * a busy GPU (icc-bcm-voter.c: a BCM command is commit, valid, the average and the peak, 14 bits each,
 * in the BCM's units; the BCM takes the highest of its levels the vote reaches), and the memory kept
 * on for this processor (ACV, bit 3, which the boot loader's firmware had set already, the same).
 * All cores together got 12.4 GB/s writing without it (membench=2: what one core gets alone), 14.8
 * with it; frames over 16.7 ms 28% instead of 34-36%. ddr_vote=N: N instead of the most (16383).
 * When the kernel goes back to the loader (hakios_ddr_quiesce(), on every way back, a panic's too)
 * the vote goes down to ddr_rest=N (default 1024). What the firmware had voted cannot be read back,
 * but what the cores get from memory can be measured (membench=2, writing): with any vote from 4
 * to 4096 12.0 to 12.4 GB/s, as before the first vote (12.4); with the most, 15.4; with none at all
 * ("no request", valid clear: what Linux sends for nothing) 0.2 -- the memory at its floor, and the
 * loader's USB transfers at 8 MB/s instead of 30. So the firmware had a vote of its own in our name,
 * of that middle range, and one from the middle of it is left.
 */
#define BCM_COMMIT              (1u << 30)
#define BCM_VALID               (1u << 29)
#define BCM_VOTE(x, y)          (BCM_COMMIT | BCM_VALID | ((uint32_t)(x) & 0x3fff) << 14 | ((uint32_t)(y) & 0x3fff))
#define TCS_REG(reg)            (ddr_rsc + 0xd00 + (reg))       /* command slot set 0, an "active" one */
#define BCM_ACV_APPS            8
#define DDR_REST                1024            /* the vote left for the loader (ddr_rest=N); see hakios_ddr_quiesce() */

static vm_offset_t ddr_rsc;
static uint32_t ddr_mc0, ddr_sh0, ddr_acv;
static bool ddr_voted;

static uint32_t
cmd_db_address(const char *name)
{
	const struct cmd_db_header *db = cmd_db_mapped();
	char id[8] = { 0 };

	if (db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &db->slave[s];
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);

		if (slave->id != CMD_DB_BCM || slave->header_offset >= CMD_DB_SIZE - 0x1000 || slave->count > 512) {
			continue;
		}
		for (unsigned i = 0; i < slave->count; i++) {
			if (memcmp(entry[i].id, id, sizeof(id)) == 0) {
				return entry[i].address;
			}
		}
	}
	return 0;
}

/* Up to three writes through command slot set 0, and the wait until RPMh has done them (as drivers/input/hakios.goodix.c) */
static bool
rpmh_write_commands(const uint32_t *address, const uint32_t *data, unsigned count, bool check_ack)
{
	uint32_t irq_enable = *(volatile uint32_t *)TCS_REG(0x00);
	bool done = false;

	for (int i = 0; *(volatile uint32_t *)TCS_REG(0x18) == 0; i++) {
		if (i > 2000) {
			tell("ddr: RPMh: the command slots stay busy\n");
			return false;
		}
		delay_for_interval(100, NSEC_PER_USEC);
	}
	*(volatile uint32_t *)TCS_REG(0x14) &= ~(1u << 24);
	*(volatile uint32_t *)TCS_REG(0x14) &= ~(1u << 16);
	*(volatile uint32_t *)TCS_REG(0x08) = 1;
	for (unsigned i = 0; i < count; i++) {
		*(volatile uint32_t *)TCS_REG(0x30 + 20 * i) = 0x10008 | 0x100;        /* a 4-byte write, with a response */
		*(volatile uint32_t *)TCS_REG(0x34 + 20 * i) = address[i];
		*(volatile uint32_t *)TCS_REG(0x38 + 20 * i) = data[i];
	}
	*(volatile uint32_t *)TCS_REG(0x10) = (1u << count) - 1;
	*(volatile uint32_t *)TCS_REG(0x1c) = (1u << count) - 1;
	*(volatile uint32_t *)TCS_REG(0x00) = irq_enable | 1;
	*(volatile uint32_t *)TCS_REG(0x14) = 1u << 16;
	*(volatile uint32_t *)TCS_REG(0x14) = 1u << 16 | 1u << 24;
	for (int i = 0; i < 2000 && !done; i++) {
		done = (*(volatile uint32_t *)TCS_REG(0x04) & 1) != 0;
		if (!done) {
			delay_for_interval(100, NSEC_PER_USEC);
		}
	}
	if (done && check_ack) {
		for (unsigned i = 0; i < count; i++) {
			uint32_t status = *(volatile uint32_t *)TCS_REG(0x3c + 20 * i);
			if ((status & 0x10100u) != 0x10100u) {
				done = false;
			}
		}
	}
	*(volatile uint32_t *)TCS_REG(0x14) &= ~(1u << 24);
	*(volatile uint32_t *)TCS_REG(0x14) &= ~(1u << 16);
	*(volatile uint32_t *)TCS_REG(0x1c) = 0;
	*(volatile uint32_t *)TCS_REG(0x10) = 0;
	*(volatile uint32_t *)TCS_REG(0x08) = 1;
	*(volatile uint32_t *)TCS_REG(0x00) = irq_enable;
	if (!done) {
		tell("ddr: RPMh: no answer to the vote\n");
	}
	return done;
}

static bool
ddr_rpmh_write(const uint32_t *address, const uint32_t *data, unsigned count)
{
	return rpmh_write_commands(address, data, count, false);
}

static bool
ddr_vote(uint32_t vote)
{
	uint32_t address[3] = { ddr_mc0, ddr_sh0, ddr_acv };
	uint32_t data[3] = { BCM_VOTE(vote, vote), BCM_VOTE(vote, vote), BCM_VOTE(0, BCM_ACV_APPS) };

	return ddr_rpmh_write(address, data, 3);
}

static void
ddr_raise(void)
{
	uint32_t vote = 0x3fff;

	PE_parse_boot_argn("ddr_vote", &vote, sizeof(vote));
	ddr_mc0 = cmd_db_address("MC0");
	ddr_sh0 = cmd_db_address("SH0");
	ddr_acv = cmd_db_address("ACV");
	if (ddr_mc0 == 0 || ddr_sh0 == 0 || ddr_acv == 0 || vote == 0 || vote > 0x3fff) {
		tell("ddr: no vote (MC0 %#x, SH0 %#x, ACV %#x, vote %u)\n", ddr_mc0, ddr_sh0, ddr_acv, vote);
		return;
	}
	ddr_rsc = ml_io_map(RSC_PHYS, RSC_SIZE);
	ddr_voted = ddr_vote(vote);
	tell("ddr: memory bandwidth vote %u on MC0 (%#x) and SH0 (%#x)%s\n", vote, ddr_mc0, ddr_sh0, ddr_voted ? "" : ": FAILED");
}

/*
 * ---- The APSS DRV's bandwidth votes, summed (POWER-COLLAPSE-PLAN.md step 5b; the OnePlus 10 Pro) ----
 *
 * hakios_rpmh_bw(client, bcm, ab, ib): what one user of the APSS DRV wants of a BCM, in the BCM's units. Per BCM the
 * clients' averages are added and the highest peak is taken (as Linux's icc-bcm-voter.c; ACV's peak is a mask, ORed),
 * and the sum is written through TCS 0 under hakios_rsc_lock only when it changed; nothing at all: "no request"
 * (commit, valid clear). Every RPMh write for a BCM of this DRV goes through here, so that one user's write does not
 * undo another's (the DRV has one vote per BCM). Clients: hakios.input.h HAKIOS_RPMH_*.
 *
 * The processors' client is the DDR/LLCC floor: MC0 (DDR) and SH0 (the LLCC's hub) follow the governor's clocks, as
 * Android's memlat floor does from a table, lower with the screen off; ACV bit 3 (APPS) keeps DDR on for us (5c clears
 * it in the sleep set). A BCM's vote for f MHz: f x width x 10^6 / unit, width (bytes a clock) and unit (bytes/s a
 * vote) from the command database (the BCM's aux data; MC0 16 and 3200000, SH0 64 and 4200000) [V, rpmh-keep/run1: the
 * boot loader's MC0 9936 is 1987 MHz by this, and DDR ran at 2092, the next level up]. The AOP picks the lowest level
 * that meets it; below 547 MHz it keeps 547 while the processors are up [V]. The first vote overwrites what the boot
 * loader left in our name (5a: MC0 210/9936, SH0 160/7570). ddr_floor=0: no floor, the boot loader's votes stay (and
 * DDR at 2092 MHz).
 */
#define KEEP_RSC_PHYS           0x17a20000UL    /* rsc@17a00000, drv 2 */
#define KEEP_BCMS               16
#define KEEP_CLIENTS            HAKIOS_RPMH_CLIENTS

static LCK_MTX_DECLARE(keep_lock, &cpufreq_lock_group);
static struct {
	char            name[8];
	uint32_t        address, unit, width;
	uint32_t        ab[KEEP_CLIENTS], ib[KEEP_CLIENTS];
	uint32_t        sent;                   /* the last command written; 0: none yet */
} keep[KEEP_BCMS];
static unsigned keep_n, keep_writes, keep_failures;
static bool keep_on;

extern lck_mtx_t hakios_rsc_lock;
static bool keep_flush_active(void);

/* a BCM's aux data (unit, width), from the command database */
static bool
bcm_aux(const char *name, uint32_t *unit, uint32_t *width)
{
	const struct cmd_db_header *db = cmd_db_mapped();
	char id[8] = { 0 };

	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &db->slave[s];
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);

		if (slave->id != CMD_DB_BCM || slave->header_offset >= CMD_DB_SIZE - 0x1000 || slave->count > 512) {
			continue;
		}
		for (unsigned i = 0; i < slave->count; i++) {
			struct cmd_db_bcm aux = { 0 };

			if (memcmp(entry[i].id, id, sizeof(id)) != 0 || entry[i].length < sizeof(aux) ||
			    sizeof(*db) + slave->data_offset + entry[i].offset + sizeof(aux) > CMD_DB_SIZE) {
				continue;
			}
			memcpy(&aux, db->data + slave->data_offset + entry[i].offset, sizeof(aux));
			*unit = aux.unit;
			*width = aux.width;
			return aux.unit != 0;
		}
	}
	return false;
}

/* with keep_lock: the BCM's slot, found or made */
static int
keep_slot(const char *name)
{
	char id[8] = { 0 };

	strlcpy(id, name, sizeof(id));
	for (unsigned k = 0; k < keep_n; k++) {
		if (memcmp(keep[k].name, id, sizeof(id)) == 0) {
			return (int)k;
		}
	}
	if (keep_n == KEEP_BCMS || cmd_db_address(name) == 0) {
		return -1;
	}
	memcpy(keep[keep_n].name, id, sizeof(id));
	keep[keep_n].address = cmd_db_address(name);
	(void)bcm_aux(name, &keep[keep_n].unit, &keep[keep_n].width);
	return (int)keep_n++;
}

/* with keep_lock: the sum of the clients in `clients` (a bit each) for slot k, as a command */
static uint32_t
keep_sum(unsigned k, uint32_t clients)
{
	uint32_t ab = 0, ib = 0;
	bool mask = strncmp(keep[k].name, "ACV", 8) == 0;

	for (unsigned c = 0; c < KEEP_CLIENTS; c++) {
		if (!(clients & (1u << c))) {
			continue;
		}
		ab = MIN(ab + keep[k].ab[c], 0x3fffu);
		ib = mask ? (ib | keep[k].ib[c]) : MAX(ib, keep[k].ib[c]);
	}
	return ab == 0 && ib == 0 ? BCM_COMMIT : BCM_VOTE(ab, MIN(ib, 0x3fffu));
}

/*
 * ---- The sleep and wake sets (POWER-COLLAPSE-PLAN.md step 5c) ----
 *
 * The APSS DRV's TCSs 3-4 are its sleep set and 5-6 its wake set (dt qcom,tcs-config: 3 active, 2 sleep, 2 wake, no
 * control; 7 the fast path). When the last core takes llcc-off the RSC sends the sleep set to RPMh, and the wake set on
 * the way up, before a core runs (Linux: rpmh_flush() at the cluster's power-down; the commands, not a trigger, are
 * all it writes). Here both are written whenever a vote changes: while a core runs this code the cluster is up and
 * the sets are idle. Sleep: every client's votes but the processors' (HAKIOS_RPMH_CPU: the floor and its ACV bit, as
 * Linux's QCOM_ICC_TAG_ACTIVE_ONLY), so DDR and the LLCC can go down while no core runs; the display client keeps
 * scanout's while the screen is on. Wake: what was last written through TCS 0, back. No RSC wake timer: there is no
 * control TCS on SM8450 (upstream sm8450.dtsi CONTROL_TCS 0, so rpmh_rsc_write_next_wakeup() does nothing there
 * either); waking from llcc-off is the firmware's. rpmh_sets=0: the sets stay empty (as before 5c).
 */
#define KEEP_SLEEP_TCS          3
#define KEEP_WAKE_TCS           5
/* (USB: its SN2 vote held DDR at 547 MHz through llcc-off, and USB kept working without it there; 5d, rpmh-devices/) */
#define KEEP_ACTIVE_ONLY        (1u << HAKIOS_RPMH_CPU | 1u << HAKIOS_RPMH_USB)
#define KEEP_ALL                ((1u << KEEP_CLIENTS) - 1)
#define SET_REG(t, reg)         (ddr_rsc + 0xd00 + 0x2a0 * (t) + (reg))

static bool keep_sets;
static unsigned keep_sets_n, keep_set_writes;
static uint32_t keep_slept[KEEP_BCMS], keep_woke[KEEP_BCMS];  /* what each set holds per slot; 0: not in it */
static uint32_t keep_probe_mhz;         /* diag (boot-arg rpmh_sleep_probe=MHz): MC0 at this in the sleep set only */
static int keep_drop;                   /* diag (sysctl kern.hakios.rpmh_sleep_drop=mask): these clients left out of the sleep set too */
static uint32_t keep_mhz(int k, uint32_t mhz);
static int keep_mc0_slot(void);

/* with keep_lock: slot k's sleep-set command (0: left out) */
static uint32_t
keep_sleep_cmd(unsigned k)
{
	if (keep[k].sent == 0) {
		return 0;
	}
	if (keep_probe_mhz != 0 && (int)k == keep_mc0_slot()) {
		return BCM_VOTE(0, keep_mhz((int)k, keep_probe_mhz));
	}
	return keep_sum(k, KEEP_ALL & ~KEEP_ACTIVE_ONLY & ~(uint32_t)keep_drop);
}

/* with keep_lock: the sets brought up to date, if a slot's sleep or wake command is not what they hold */
static void
keep_sets_write(void)
{
	uint32_t address[KEEP_BCMS], sleep[KEEP_BCMS], wake[KEEP_BCMS];
	unsigned n = 0;
	bool same = true;

	if (!keep_sets) {
		return;
	}
	for (unsigned k = 0; k < keep_n; k++) {
		uint32_t s = keep_sleep_cmd(k);

		same = same && keep_slept[k] == s && keep_woke[k] == keep[k].sent;
		if (keep[k].sent == 0) {
			continue;               /* never voted: left out of both */
		}
		address[n] = keep[k].address;
		sleep[n] = s;
		wake[n++] = keep[k].sent;
	}
	if (same && n == keep_sets_n) {
		return;
	}
	lck_mtx_lock(&hakios_rsc_lock);
	for (unsigned t = KEEP_SLEEP_TCS; t <= KEEP_WAKE_TCS; t += KEEP_WAKE_TCS - KEEP_SLEEP_TCS) {
		*(volatile uint32_t *)SET_REG(t, 0x1c) = 0;
		for (unsigned i = 0; i < n; i++) {
			*(volatile uint32_t *)SET_REG(t, 0x30 + 20 * i) = 0x10008 | 0x100;     /* a 4-byte write, with a response */
			*(volatile uint32_t *)SET_REG(t, 0x34 + 20 * i) = address[i];
			*(volatile uint32_t *)SET_REG(t, 0x38 + 20 * i) = t == KEEP_SLEEP_TCS ? sleep[i] : wake[i];
		}
		*(volatile uint32_t *)SET_REG(t, 0x1c) = n ? (1u << n) - 1 : 0;
	}
	lck_mtx_unlock(&hakios_rsc_lock);
	for (unsigned k = 0; k < keep_n; k++) {
		keep_slept[k] = keep_sleep_cmd(k);
		keep_woke[k] = keep[k].sent;
	}
	keep_sets_n = n;
	keep_set_writes++;
}

/* with keep_lock: the slots whose sum changed, written in one go; then the sets */
static bool
keep_flush(void)
{
	bool done = keep_flush_active();

	keep_sets_write();
	return done;
}

static bool
keep_flush_active(void)
{
	uint32_t address[KEEP_BCMS], data[KEEP_BCMS];
	unsigned n = 0, which[KEEP_BCMS];
	bool done;

	for (unsigned k = 0; k < keep_n; k++) {
		uint32_t command = keep_sum(k, KEEP_ALL);

		if (command != keep[k].sent) {
			address[n] = keep[k].address;
			data[n] = command;
			which[n++] = k;
		}
	}
	if (n == 0) {
		return true;
	}
	lck_mtx_lock(&hakios_rsc_lock);
	done = ddr_rpmh_write(address, data, n);
	lck_mtx_unlock(&hakios_rsc_lock);
	keep_writes++;
	if (!done) {
		keep_failures++;
		return false;
	}
	for (unsigned i = 0; i < n; i++) {
		keep[which[i]].sent = data[i];
	}
	return true;
}

bool
hakios_rpmh_bw(unsigned client, const char *bcm, uint32_t ab, uint32_t ib)
{
	bool done = false;
	int k;

	if (client >= KEEP_CLIENTS || ddr_rsc == 0) {
		return false;
	}
	lck_mtx_lock(&keep_lock);
	if ((k = keep_slot(bcm)) >= 0) {
		keep[k].ab[client] = ab;
		keep[k].ib[client] = ib;
		done = keep_flush();
	}
	lck_mtx_unlock(&keep_lock);
	return done;
}

/* f MHz as a vote on slot k (DDR: MC0; LLCC: SH0) */
static uint32_t
keep_mhz(int k, uint32_t mhz)
{
	uint64_t vote;

	if (k < 0 || keep[k].unit == 0) {
		return 0;
	}
	vote = ((uint64_t)mhz * keep[k].width * 1000000 +keep[k].unit - 1) / keep[k].unit;
	return (uint32_t)MIN(vote, 0x3fffu);
}

/*
 * The floor: the first row whose clocks the processors are under (the small cores' group, the highest of the big
 * ones; kHz). Android's waipio memlat floors are about these (547 MHz DDR for little work, 3196 with the biggest core
 * at its top). With the screen off the first row drops to DDR's and the LLCC's lowest.
 */
static const struct {
	uint32_t        small_khz, big_khz, ddr_mhz, llcc_mhz;
} floor_rows[] = {
	{ 1100000,    1100000,    547,  300 },
	{ 1400000,    1500000,    1017, 466 },
	{ 1800000,    2000000,    1555, 600 },
	{ UINT32_MAX, 2600000,    2092, 806 },
	{ UINT32_MAX, UINT32_MAX, 3196, 1066 },
};
#define FLOOR_OFF_DDR           200
#define FLOOR_OFF_LLCC          300

static int keep_mc0 = -1, keep_sh0 = -1, keep_acv = -1;
static uint32_t floor_ddr, floor_llcc, floor_changes;
static bool floor_display, floor_display_known, floor_usb, floor_usb_known;

static int
keep_mc0_slot(void)
{
	return keep_mc0;
}
static char floor_line[200];

SYSCTL_STRING(_kern_hakios, OID_AUTO, ddr_floor, CTLFLAG_RD | CTLFLAG_LOCKED, floor_line, 0,
    "the processors' DDR/LLCC floor now (MHz, MC0/SH0 votes), its changes, the APSS BCM writes and failures");

static void
floor_describe(void)
{
	snprintf(floor_line, sizeof(floor_line), "ddr %u MHz (MC0 %u) llcc %u MHz (SH0 %u) changes %u writes %u failed %u%s"
	    " sets %s %u/%u display %s probe %u usb %s drop %#x",
	    floor_ddr, keep_mhz(keep_mc0, floor_ddr), floor_llcc, keep_mhz(keep_sh0, floor_llcc), floor_changes, keep_writes,
	    keep_failures, keep_on ? "" : " (off)", keep_sets ? "on" : "off", keep_sets_n, keep_set_writes,
	    floor_display ? "on" : "off", keep_probe_mhz, floor_usb ? "in" : "out", keep_drop);
}

static void
floor_set(uint32_t ddr_mhz, uint32_t llcc_mhz)
{
	if (!keep_on || (ddr_mhz == floor_ddr && llcc_mhz == floor_llcc)) {
		return;
	}
	lck_mtx_lock(&keep_lock);
	if (keep_mc0 >= 0) {
		keep[keep_mc0].ab[HAKIOS_RPMH_CPU] = 0;
		keep[keep_mc0].ib[HAKIOS_RPMH_CPU] = keep_mhz(keep_mc0, ddr_mhz);
	}
	if (keep_sh0 >= 0) {
		keep[keep_sh0].ab[HAKIOS_RPMH_CPU] = 0;
		keep[keep_sh0].ib[HAKIOS_RPMH_CPU] = keep_mhz(keep_sh0, llcc_mhz);
	}
	(void)keep_flush();
	lck_mtx_unlock(&keep_lock);
	floor_ddr = ddr_mhz;
	floor_llcc = llcc_mhz;
	floor_changes++;
	floor_describe();
}

/*
 * ---- Device votes (POWER-COLLAPSE-PLAN.md step 5d) ----
 *
 * The boot loader's firmware left more votes in our name than DDR's (5a, TCS 0): MM0 2180/4255 and MM1 0/1 (the
 * multimedia NoC: the splash's scanout), CN0 0/1 (the config NoC: the processors' register reads), SN2 1265/1264 (the
 * system NoC its USB and UFS transfers took). Each now has an owner, so the sets carry it only while that owner needs
 * it: MM0/MM1 the display's while the screen is on; CN0 the processors' (active only, as Linux's cpu-cfg paths); SN2
 * the USB client's while a cable is in, active only: in the sleep set it alone held DDR at 547 MHz (rpmh_sleep_drop
 * per client, rpmh-devices/run3), and the link kept answering without it there. They go back as the loader had them in
 * hakios_ddr_quiesce().
 */
static const struct {
	const char      *bcm;
	unsigned        client;
	uint32_t        ab, ib;
} keep_loader[] = {
	{ "MM0", HAKIOS_RPMH_DISPLAY, 2180, 4255 },
	{ "MM1", HAKIOS_RPMH_DISPLAY, 0, 1 },
	{ "CN0", HAKIOS_RPMH_CPU, 0, 1 },
	{ "SN2", HAKIOS_RPMH_USB, 1265, 1264 },
};
#define KEEP_LOADER             (sizeof(keep_loader) / sizeof(keep_loader[0]))
static int keep_loader_slot[KEEP_LOADER] = { -1, -1, -1, -1 };

/* with keep_lock: the loader's votes of `client`, on or off */
static void
keep_loader_set(unsigned client, bool on)
{
	for (unsigned i = 0; i < KEEP_LOADER; i++) {
		int k = keep_loader_slot[i];

		if (k >= 0 && keep_loader[i].client == client) {
			keep[k].ab[client] = on ? keep_loader[i].ab : 0;
			keep[k].ib[client] = on ? keep_loader[i].ib : 0;
		}
	}
}

/*
 * Scanout's need while the screen is on: the first row's DDR and LLCC and ACV and the loader's MMNoC votes, from the
 * display client, which the sleep set keeps (the panel reads its frame from DDR while no core runs).
 */
static void
floor_display_set(bool on)
{
	if (!keep_on) {
		return;
	}
	lck_mtx_lock(&keep_lock);       /* (the governor and the backlight's work loop both set it) */
	if (floor_display_known && on == floor_display) {
		lck_mtx_unlock(&keep_lock);
		return;
	}
	if (keep_mc0 >= 0) {
		keep[keep_mc0].ib[HAKIOS_RPMH_DISPLAY] = on ? keep_mhz(keep_mc0, floor_rows[0].ddr_mhz) : 0;
	}
	if (keep_sh0 >= 0) {
		keep[keep_sh0].ib[HAKIOS_RPMH_DISPLAY] = on ? keep_mhz(keep_sh0, floor_rows[0].llcc_mhz) : 0;
	}
	if (keep_acv >= 0) {
		keep[keep_acv].ib[HAKIOS_RPMH_DISPLAY] = on ? BCM_ACV_APPS : 0;
	}
	keep_loader_set(HAKIOS_RPMH_DISPLAY, on);
	(void)keep_flush();
	floor_display = on;
	floor_display_known = true;
	lck_mtx_unlock(&keep_lock);
	floor_describe();
}

/*
 * The panel scanning out (hakios.backlight.cpp: from the start of its wake to the end of its sleep), whatever the
 * screen's state: the display's votes held for that time, at once. The governor alone set them, from the screen's
 * state at its next period: a wake's first frames (up to 250 ms of them) and every display power cycle, which leaves
 * the screen off, were read from DDR through a multimedia NoC nobody voted for (run18: the frames slowed to 3 in 50 ms
 * and the controller's transfer never finished, the panel garbled).
 */
void hakios_display_bandwidth(bool on);
static _Atomic bool floor_scanout;

void
hakios_display_bandwidth(bool on)
{
	os_atomic_store(&floor_scanout, on, release);
	floor_display_set(on || !hakios_screen_off());
}

/* the USB client: SN2 while a cable is in (a computer or a charger, as the battery manager last said; in until it says) */
static void
floor_usb_set(void)
{
	int soc, present, charger = 1;
	bool in;

	(void)hakios_battery_get(&soc, &present, &charger);
	in = charger != 0;
	if (!keep_on || (floor_usb_known && in == floor_usb)) {
		return;
	}
	lck_mtx_lock(&keep_lock);
	keep_loader_set(HAKIOS_RPMH_USB, in);
	(void)keep_flush();
	lck_mtx_unlock(&keep_lock);
	floor_usb = in;
	floor_usb_known = true;
	floor_describe();
}

static bool pcie_bw_needed;
static int pcie_bw_slot = -1;

/* RC0 still needs the shared SFNoC floor while APSS sleeps: either SN0 or SN3
 * held at 1/1 preserves Wi-Fi, releasing both stalls its firmware. Own the
 * measured SN0 minimum with PCIe, not touch, until link low-power support can
 * release it. Read the desired state under the keeper lock so the
 * governor cannot overwrite a new synchronous acquisition with an old value. */
static bool
floor_pcie_set(void)
{
	bool ok = false;
	if (!keep_on) return !os_atomic_load(&pcie_bw_needed, acquire);
	lck_mtx_lock(&keep_lock);
	bool on = os_atomic_load(&pcie_bw_needed, acquire);
	if (on && pcie_bw_slot < 0) pcie_bw_slot = keep_slot("SN0");
	if (pcie_bw_slot >= 0) {
		keep[pcie_bw_slot].ab[HAKIOS_RPMH_PCIE] = on ? 1 : 0;
		keep[pcie_bw_slot].ib[HAKIOS_RPMH_PCIE] = on ? 1 : 0;
		ok = keep_flush();
	} else ok = !on;
	lck_mtx_unlock(&keep_lock);
	return ok;
}

bool
hakios_pcie_bandwidth(bool on)
{
	os_atomic_store(&pcie_bw_needed, on, release);
	/* Down is also used on panic: no mutex/TCS access there. The ordinary
	 * governor releases it later; acquisition must complete before RC0 starts. */
	return !on || floor_pcie_set();
}

/* diag: the clients in the mask left out of the sleep set as well (all: what DDR does with nothing of APSS's voted while no core runs) */
static int
sysctl_rpmh_sleep_drop SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int value = keep_drop, error = sysctl_handle_int(oidp, &value, 0, req);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	lck_mtx_lock(&keep_lock);
	keep_drop = value & KEEP_ALL;
	keep_sets_write();
	lck_mtx_unlock(&keep_lock);
	floor_describe();
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_sleep_drop, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    sysctl_rpmh_sleep_drop, "I", "diag: a bit per client (sys/hakios.input.h HAKIOS_RPMH_*) left out of the APSS sleep set");

/* from the governor, after each period: the clocks it asked for (kHz per group) */
static void
floor_follow(const uint32_t khz[EPSS_DOMAINS])
{
	uint32_t big = MAX(khz[1], khz[2]);
	unsigned r = 0;

	floor_display_set(!hakios_screen_off() || os_atomic_load(&floor_scanout, acquire));
	floor_usb_set();
	(void)floor_pcie_set();
	/* (either group over its column moves the floor up) */
	while (r < sizeof(floor_rows) / sizeof(floor_rows[0]) - 1 &&
	    (khz[0] > floor_rows[r].small_khz || big > floor_rows[r].big_khz)) {
		r++;
	}
	if (r == 0 && hakios_screen_off()) {
		floor_set(FLOOR_OFF_DDR, FLOOR_OFF_LLCC);
	} else {
		floor_set(floor_rows[r].ddr_mhz, floor_rows[r].llcc_mhz);
	}
}

static void arc_default_start(void);   /* hakios.rpmh-arc.h, included below */

static void
keep_start(void)
{
	uint32_t on = 1, sets = 1;

	PE_parse_boot_argn("ddr_floor", &on, sizeof(on));
	PE_parse_boot_argn("rpmh_sets", &sets, sizeof(sets));
	PE_parse_boot_argn("rpmh_sleep_probe", &keep_probe_mhz, sizeof(keep_probe_mhz));
	keep_probe_mhz = MIN(keep_probe_mhz, 3196u);
	ddr_rsc = ml_io_map(KEEP_RSC_PHYS, RSC_SIZE);
	lck_mtx_lock(&keep_lock);
	keep_mc0 = keep_slot("MC0");
	keep_sh0 = keep_slot("SH0");
	keep_acv = keep_slot("ACV");
	for (unsigned i = 0; i < KEEP_LOADER; i++) {
		keep_loader_slot[i] = keep_slot(keep_loader[i].bcm);
	}
	lck_mtx_unlock(&keep_lock);
	/* the DRV's TCS counts (DRV_PRNT_CHLD_CONFIG: 6 bits per DRV, commands per TCS in 31:27) and its solver mode */
	tell("ddr: rsc drv 2 config %#x (tcs %u, cmds %u), solver %#x\n", *(volatile uint32_t *)(ddr_rsc + 0x0c),
	    *(volatile uint32_t *)(ddr_rsc + 0x0c) >> 12 & 0x3f, *(volatile uint32_t *)(ddr_rsc + 0x0c) >> 27 & 0x1f,
	    *(volatile uint32_t *)(ddr_rsc + 0x04));
	tell("ddr: MC0 unit %u width %u, SH0 unit %u width %u: DDR 547/2092/3196 MHz = %u/%u/%u, LLCC 300/806 = %u/%u\n",
	    keep_mc0 >= 0 ? keep[keep_mc0].unit : 0, keep_mc0 >= 0 ? keep[keep_mc0].width : 0,
	    keep_sh0 >= 0 ? keep[keep_sh0].unit : 0, keep_sh0 >= 0 ? keep[keep_sh0].width : 0, keep_mhz(keep_mc0, 547),
	    keep_mhz(keep_mc0, 2092), keep_mhz(keep_mc0, 3196), keep_mhz(keep_sh0, 300), keep_mhz(keep_sh0, 806));
	if (!on || keep_mc0 < 0 || keep_sh0 < 0 || keep[keep_mc0].unit == 0 || keep[keep_sh0].unit == 0) {
		floor_describe();
		tell("ddr: no floor (ddr_floor=%u, MC0 %d, SH0 %d): the boot loader's votes stay\n", on, keep_mc0, keep_sh0);
		return;
	}
	keep_on = true;
	keep_sets = sets != 0;
	(void)hakios_rpmh_bw(HAKIOS_RPMH_CPU, "ACV", 0, BCM_ACV_APPS);
	/* SM8450's SN0 is keepalive in Linux's interconnect map: 1/1 in active/wake,
	 * not sleep (bcm-voter.c). Touch used to supply this accidentally. Own that
	 * floor with the CPU client so an idle QUP can release its independent vote. */
	(void)hakios_rpmh_bw(HAKIOS_RPMH_CPU, "SN0", 1, 1);
	/* SH1: the boot loader left a vote of its own here through this DRV (0x20000001, read back 2026-11-15) that no sleep
	 * set took away: it held the SoC awake until run5 sent it as no-request in sleep (POWER-COLLAPSE-PLAN.md 5e). The
	 * loader's 0/1 kept while a core runs, as an active-only vote of the processors' client. */
	(void)hakios_rpmh_bw(HAKIOS_RPMH_CPU, "SH1", 0, 1);
	lck_mtx_lock(&keep_lock);
	keep_loader_set(HAKIOS_RPMH_CPU, true);
	(void)keep_flush();
	lck_mtx_unlock(&keep_lock);
	floor_display_set(true);        /* the screen is on at start; the governor's first period corrects it */
	floor_usb_set();
	floor_set(2092, 806);           /* what the boot loader had, until the governor's first period */
	tell("ddr: floor %s\n", floor_line);
	if (keep_sets) {
		arc_default_start();    /* XO off and CX at retention in the sleep set (hakios.rpmh-arc.h) */
	}
}

#include "hakios.rpmh-arc.h"

#ifdef HAKIOS_CAM_PROBE
static LCK_MTX_DECLARE(cam_power_lock, &cpufreq_lock_group);
static unsigned cam_power_clients;
static bool cam_power_ready;
static struct cam_arc_vote {
	uint32_t address, saved;
	bool changed;
} cam_mmcx, cam_mxc;

bool
hakios_cam_rails_ready(void)
{
	return epss_phys == EPSS_PHYS_8450 && ddr_rsc != 0;
}

/* Use the shared BCM slots so releasing the diagnostic preserves the display and DDR votes. */
static bool
cam_bandwidth(unsigned client, bool on)
{
	static const struct {
		const char *name;
		uint32_t buswidth, channels;
	} paths[] = {
		{ "CN0", 0, 0 }, { "MM1", 0, 0 }, { "MM0", 32, 2 },
		{ "SH0", 16, 4 }, { "MC0", 4, 4 },
	};
	bool ok = true;
	lck_mtx_lock(&keep_lock);
	for (unsigned i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
		int slot = keep_slot(paths[i].name);
		if (slot < 0 || (on && paths[i].buswidth && (!keep[slot].unit || !keep[slot].width))) {
			ok = false;
			break;
		}
		uint32_t ab = 0, ib = 0;
		if (on && paths[i].buswidth) {
			/* Bytes/s scaled by the node's width and the command DB units. */
			uint64_t unit = (uint64_t)paths[i].buswidth * keep[slot].unit;
			ab = MIN(1000000000ull * keep[slot].width / (unit * paths[i].channels), 0x3fffull);
			ib = MIN(3000000000ull * keep[slot].width / unit, 0x3fffull);
		} else if (on) {
			ib = 1;
		}
		keep[slot].ab[client] = ab;
		keep[slot].ib[client] = ib;
	}
	ok = keep_flush() && ok;
	lck_mtx_unlock(&keep_lock);
	return ok;
}

/* Raise an ARC resource to at least NOM (hlvl 256), never lower; restore the read-back vote. */
static bool
cam_arc(const char *name, struct cam_arc_vote *saved, bool on)
{
	if (!on && !saved->changed) {
		return true;
	}
	lck_mtx_lock(&hakios_rsc_lock);
	bool ok = false;
	uint32_t vote = saved->saved;
	if (on) {
		uint16_t levels[16];
		unsigned count, kind = 0, level = 0;
		uint32_t status;
		saved->address = rpmh_db_find(name, levels, &count, &kind);
		while (level < count && levels[level] < 256) {
			level++;
		}
		if (!saved->address || kind != CMD_DB_ARC || level == count ||
		    !rpmh_read(saved->address, &saved->saved, &status) || saved->saved >= count) {
			goto out;
		}
		vote = MAX(level, saved->saved);
	}
	/* A timed-out command still needs a restore attempt. */
	saved->changed = true;
	ok = rpmh_write_commands(&saved->address, &vote, 1, true);
	if (ok) {
		hakios_rpmh_arc_active(saved->address, vote);
		saved->changed = on;
	}
out:
	lck_mtx_unlock(&hakios_rsc_lock);
	return ok;
}

/*
 * MMCX feeds the Titan GDSCs. MXC feeds the CamCC PLLs: camcc-waipio.c lists
 * cam_cc_pll0 at 1200 MHz above VDD_LOW (1066 MHz), so it needs SVS_L1 or more.
 */
bool
hakios_cam_power_vote(struct hakios_cam_power *saved, unsigned client, bool on)
{
	if (client != HAKIOS_RPMH_CAMERA && client != HAKIOS_RPMH_IRIS && client != HAKIOS_RPMH_ICP) {
		return false;
	}
	const unsigned bit = 1u << client;
	bool ok = true;
	lck_mtx_lock(&cam_power_lock);
	if (on) {
		if ((cam_power_clients & bit) || (cam_power_clients && !cam_power_ready)) {
			ok = false;
			goto out;
		}
		/* Only the first client snapshots the rail votes, including a partial acquire. */
		saved->claimed = true;
		if (cam_power_clients == 0) {
			cam_power_ready = cam_arc("mmcx.lvl", &cam_mmcx, true) && cam_arc("mxc.lvl", &cam_mxc, true);
		}
		cam_power_clients |= bit;
		ok = cam_power_ready;
		if (ok) {
			saved->bandwidth_attempted = true;
			ok = cam_bandwidth(client, true);
		}
	} else if (saved->claimed) {
		ok = !saved->bandwidth_attempted || cam_bandwidth(client, false);
		saved->bandwidth_attempted = !ok;
		if (cam_power_clients == bit) {
			/* Block new clients after an incomplete restore until cleanup can retry. */
			cam_power_ready = false;
			bool mxc_ok = cam_arc("mxc.lvl", &cam_mxc, false);
			ok = cam_arc("mmcx.lvl", &cam_mmcx, false) && mxc_ok && ok;
		}
		if (ok) {
			cam_power_clients &= ~bit;
			saved->claimed = false;
		}
	}
out:
	lck_mtx_unlock(&cam_power_lock);
	return ok;
}

/*
 * Active VRM requests use voltage, enable and mode at +0/+4/+8. Hold the
 * same TCS lock as the bandwidth and ARC clients across read/modify/verify.
 */
static bool
cam_rail_read(uint32_t address, int mode, struct hakios_cam_rail *rail)
{
	uint32_t status;
	bool ok = rpmh_read(address, &rail->voltage, &status) &&
	          rpmh_read(address + 4, &rail->enable, &status) &&
	          (mode < 0 || rpmh_read(address + 8, &rail->mode, &status));
	/* Response words contain additional bits outside the VRM fields. */
	rail->voltage &= 0x1fffu;
	rail->enable &= 1u;
	rail->mode &= 7u;
	return ok;
}

int
hakios_cam_rail_vote(const char *name, uint32_t mv, int mode, struct hakios_cam_rail *saved, bool on)
{
	if (!hakios_cam_rails_ready()) {
		return ENODEV;
	}
	if (!on && !saved->changed) {
		return 0;
	}
	for (unsigned i = 0; !lck_mtx_try_lock(&hakios_rsc_lock); i++) {
		if (i == 100) {
			return EBUSY;
		}
		delay_for_interval(1000, NSEC_PER_USEC);
	}

	int error = EIO;
	if (on) {
		uint16_t levels[16];
		unsigned count, kind = 0;
		saved->address = rpmh_db_find(name, levels, &count, &kind);
		if (kind != 4 || saved->address == 0 || (saved->address & 15)) {
			error = ENODEV;
			goto out;
		}
		if (!cam_rail_read(saved->address, mode, saved) || saved->voltage > 6000) {
			goto out;
		}
		if (saved->enable) {
			/* Preserve an existing enable and its programmed settings. */
			error = saved->voltage == mv && (mode < 0 || saved->mode == (uint32_t)mode) ? 0 : EBUSY;
			goto out;
		}
	}

	uint32_t address = saved->address;
	uint32_t settings[] = { address, address + 8 };
	uint32_t values[] = { on ? mv : saved->voltage, on ? (uint32_t)mode : saved->mode };
	uint32_t enable_address = address + 4;
	uint32_t enable = on ? 1 : saved->enable;
	unsigned settings_count = mode < 0 ? 1 : 2;
	bool written;
	if (on) {
		/* A timed-out request may still have taken effect. Restore it on exit. */
		saved->changed = true;
		written = rpmh_write_commands(settings, values, settings_count, true) &&
		          rpmh_write_commands(&enable_address, &enable, 1, true);
	} else {
		written = rpmh_write_commands(&enable_address, &enable, 1, true) &&
		          rpmh_write_commands(settings, values, settings_count, true);
	}

	struct hakios_cam_rail actual = { 0 };
	/* A disabled VRM can report its last applied settings. The restore
	 * commands must be acknowledged and its enable readback must be clear. */
	if (written && cam_rail_read(address, mode, &actual) && actual.enable == enable &&
	    (!on || (actual.voltage == mv && (mode < 0 || actual.mode == (uint32_t)mode)))) {
		saved->changed = on;
		error = 0;
	} else {
		printf("cam_probe: RPMh %s %s failed (read %u mV, enable %u, mode %u)\n",
		    name, on ? "acquire" : "restore", actual.voltage, actual.enable, actual.mode);
	}
out:
	lck_mtx_unlock(&hakios_rsc_lock);
	return error;
}
#endif /* HAKIOS_CAM_PROBE */

/* On the way back to the loader (osfmk/arm64/loader_return.c: loader_return(), whatever the reason; the other processors are stopped) */
void hakios_ddr_quiesce(void);

void
hakios_ddr_quiesce(void)
{
	uint32_t address[2] = { ddr_mc0, ddr_sh0 };
	uint32_t rest = DDR_REST, data[2];
	bool done;

	/*
	 * The governor stops writing, and each group goes back to the row it had before there was a governor (the top row,
	 * or the cap's): the loader runs on the first group, and must not be left at its lowest row. No lock: the other
	 * processors are stopped, and interrupts are off.
	 */
	cpufreq_leaving = true;
	arc_disarm();                   /* the experiment's ARC sleep and wake commands out (hakios.rpmh-arc.h) */
	for (uint32_t d = 0; d < EPSS_DOMAINS; d++) {
		if (dom[d].ours) {
			dom[d].row = row_for_cap(d);
			*(volatile uint32_t *)(dom[d].base + EPSS_PERF_STATE) = dom[d].row;
		}
	}
	if (keep_on) {
		/* the boot loader's own votes back (5a), whatever the floor was: the loader's USB runs on them (no lock) */
		uint32_t a[2 + KEEP_LOADER] = { keep[keep_mc0].address, keep[keep_sh0].address },
		    v[2 + KEEP_LOADER] = { BCM_VOTE(210, 9936), BCM_VOTE(160, 7570) };
		unsigned n = 2;

		for (unsigned i = 0; i < KEEP_LOADER; i++) {
			if (keep_loader_slot[i] >= 0) {
				a[n] = keep[keep_loader_slot[i]].address;
				v[n++] = BCM_VOTE(keep_loader[i].ab, keep_loader[i].ib);
			}
		}
		keep_on = false;
		if (keep_sets) {
			/* the sets emptied, as the loader found them */
			*(volatile uint32_t *)SET_REG(KEEP_SLEEP_TCS, 0x1c) = 0;
			*(volatile uint32_t *)SET_REG(KEEP_WAKE_TCS, 0x1c) = 0;
			keep_sets = false;
		}
		done = ddr_rpmh_write(a, v, n);
		tell("ddr: the boot loader's memory and NoC votes back for the loader (%u): %s\n", n, done ? "done" : "FAILED");
		return;
	}
	if (!ddr_voted) {
		return;
	}
	ddr_voted = false;
	PE_parse_boot_argn("ddr_rest", &rest, sizeof(rest));
	rest = MIN(rest, 0x3fff);
	data[0] = data[1] = rest != 0 ? BCM_VOTE(rest, rest) : BCM_COMMIT;     /* 0: no request */
	done = ddr_rpmh_write(address, data, 2);
	tell("ddr: memory bandwidth vote %u for the loader: %s\n", rest, done ? "done" : "FAILED");
}

/*
 * ---- The SoC's sleep, read only (POWER-COLLAPSE-PLAN.md step 5a; the OnePlus 10 Pro only) ----
 *
 * kern.hakios.rpmh_stats, a snapshot of what the always-on processor (AOP) counts:
 *  - its sleep records (dt soc-sleep-stats@c3f0000, qcom,rpmh-sleep-stats; OnePlus soc_sleep_stats.c rpmh_data: base
 *    0xc3f0000 | [0xc3f0004], 3 records of 32 bytes: name, count, last entered, last exited, accumulated, 19.2 MHz
 *    ticks): "aosd" the AOSS asleep (XO off), "cxsd" CX collapsed, and a third [I: DDR's];
 *  - DDR's statistics (base 0xc3f0000 | [0xc3f001c], magic 0xa1157a75, entries of 16 bytes: name, count, duration):
 *    low-power modes (name bits 15:8 = 0) and the time at each frequency (bits 15:8 = 1: MHz in 31:16, CP in 4:0),
 *    brought up to date by "{class: ddr, action: freqsync}"; after the entries each DRV's DDR vote (ab 27:14,
 *    ib 13:0), filled in by "{class: ddr, res: drvs_ddr_votes}" (18 of them there);
 *  - who kept CX up (dt sys-pm-vx@c320000, qcom,sys-pm-waipio; sys_pm_vx.c): once "{class: lpm_mon, type: cxpc, ...}"
 *    has started the AOP's monitor (the first read does), a log of lines, each a time and a byte per DRV, not 0
 *    where that DRV held CX; an all-zero line marks an entry into or exit from CX collapse;
 *  - the APSS DRV's command slots (rsc@17a00000 drv 2, qcom,tcs-config: TCS 0-2 active, 3-4 sleep, 5-6 wake, 7 fast
 *    path): what was written last in each. Empty sleep and wake slots mean nothing changes when the cluster goes down.
 * kern.hakios.rpmh_log: every 30 s from start, the battery's current and voltage, the charger, and how much aosd and
 * cxsd grew and at which DDR frequency the time went: to read after a time off the cable.
 * Every read of the AOP's memory goes through hakios_mmio_try_read32: a word the hypervisor does not map is said
 * so, not a panic. Nothing here writes a register; the AOP gets three kinds of message (above).
 */
#define SS_PHYS                 0x0c3f0000UL
#define SS_WINDOW               0x10000UL       /* base = 0xc3f0000 | the offset word: inside this window */
#define SS_RECORDS              3
#define SS_RECORD               32
#define DDR_MAGIC               0xa1157a75u
#define DDR_MAX_MODES           0x14
#define DDR_DRVS                18
#define DDR_SIZE                (8 + 16 * DDR_MAX_MODES + 4 * DDR_DRVS)
#define VX_PHYS                 0x0c320000UL
#define VX_SIZE                 0x400
#define VX_DRVS                 20
#define VX_WORDS                ((VX_DRVS + 3) / 4)
#define RSC_PHYS_8450           0x17a20000UL
#define RSC_TCS_8450            8
#define AOP_TICKS_PER_MS        19200ull
#define LOG_SECONDS             30
#define LOG_SAMPLES             480             /* 4 hours */
#define LOG_FREQS               12

static const char *const vx_drv[VX_DRVS] = {            /* sys_pm_vx.c drv_names_waipio */
	"TZ", "HYP", "HLOS", "L3", "SECPROC", "AUDIO", "SENSOR", "AOP", "DEBUG", "GPU", "DISPLAY", "COMPUTE_DSP",
	"TME_SW", "TME_HW", "MDM_SW", "MDM_HW", "WLAN_RF", "WLAN_BB", "DDR_AUX", "ARC_CPRF",
};

static LCK_MTX_DECLARE(ss_lock, &cpufreq_lock_group);  /* the maps, the AOP's messages, the texts */
static vm_offset_t ss_window, ss_vx, ss_rsc;
static uint32_t ss_rec_off, ss_ddr_off, ss_ddr_n;     /* 0: not there */
static bool ss_started, ss_vx_asked;
static char ss_text[8192];

static struct ss_sample {
	uint32_t        t;                      /* seconds since boot */
	int32_t         ma, mv, charger;
	uint32_t        count[2];               /* aosd, cxsd */
	uint64_t        ticks[2];
	uint64_t        freq_ticks[LOG_FREQS];  /* DDR's time at each frequency row (ss_freq_mhz) */
} ss_log[LOG_SAMPLES];
static uint32_t ss_logged, ss_freq_mhz[LOG_FREQS], ss_freqs;
static char ss_log_text[LOG_SAMPLES * 96 + 512];

static bool
ss_rd(vm_offset_t base, uint32_t offset, uint32_t *value)
{
	unsigned v = 0;

	if (base == 0 || hakios_mmio_try_read32((const volatile void *)(base + offset), &v) != 0) {
		return false;
	}
	*value = v;
	return true;
}

static bool
ss_rd64(vm_offset_t base, uint32_t offset, uint64_t *value)
{
	uint32_t lo, hi;

	if (!ss_rd(base, offset, &lo) || !ss_rd(base, offset + 4, &hi)) {
		return false;
	}
	*value = (uint64_t)hi << 32 | lo;
	return true;
}

static int ss_at;

static void
ss_say(const char *format, ...) __printflike(1, 2);

static void
ss_say(const char *format, ...)
{
	va_list list;

	if (ss_at >= (int)sizeof(ss_text) - 1) {
		return;
	}
	va_start(list, format);
	ss_at += vsnprintf(ss_text + ss_at, sizeof(ss_text) - (size_t)ss_at, format, list);
	va_end(list);
	if (ss_at > (int)sizeof(ss_text) - 1) {
		ss_at = (int)sizeof(ss_text) - 1;
	}
}

/* the region's offset word, checked: base = 0xc3f0000 | word must lie in the window with size bytes after it */
static uint32_t
ss_region(uint32_t word_at, uint32_t size)
{
	uint32_t word;

	if (!ss_rd(ss_window, word_at, &word)) {
		return 0;
	}
	word = (uint32_t)((SS_PHYS | word) - SS_PHYS);
	if (word == 0 || (word & 3) || word > SS_WINDOW - size) {
		return 0;
	}
	return word;
}

/* once, under ss_lock */
static void
ss_map(void)
{
	uint32_t magic, n;

	if (ss_started) {
		return;
	}
	ss_started = true;
	ss_window = ml_io_map(SS_PHYS, SS_WINDOW);
	ss_vx = ml_io_map(VX_PHYS, PAGE_SIZE);
	ss_rsc = ml_io_map(RSC_PHYS_8450, 0x10000);
	ss_rec_off = ss_region(0x4, SS_RECORDS * SS_RECORD);
	ss_ddr_off = ss_region(0x1c, DDR_SIZE);
	if (ss_ddr_off != 0 && ss_rd(ss_window, ss_ddr_off, &magic) && ss_rd(ss_window, ss_ddr_off + 4, &n) &&
	    magic == DDR_MAGIC && n > 0 && n <= DDR_MAX_MODES) {
		ss_ddr_n = n;
	} else {
		ss_ddr_off = 0;
	}
	printf("rpmh-stats: sleep records at %#lx%s, DDR statistics at %#lx%s (%u entries)\n", SS_PHYS + ss_rec_off,
	    ss_rec_off ? "" : " NOT READABLE", SS_PHYS + ss_ddr_off, ss_ddr_off ? "" : " NOT READABLE", ss_ddr_n);
}

/* a name for an RPMh address, from the boot loader's command database ("" if none; "+N" past a resource's base) */
static void
ss_resource(uint32_t address, char *out, size_t size)
{
	const struct cmd_db_header *db = cmd_db_mapped();

	out[0] = 0;
	if (db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		return;
	}
	/* pass 0: the exact address (BCMs are 4 bytes apart); pass 1: the nearest base below it (a VRM's or ARC's +4, +8) */
	for (unsigned pass = 0; pass < 2; pass++) {
		uint32_t best = 0, best_s = 0, best_i = 0;

		for (unsigned s = 0; s < 8; s++) {
			const struct cmd_db_slave *slave = &db->slave[s];
			const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);

			if (slave->header_offset >= CMD_DB_SIZE - 0x1000 || slave->count > 512) {
				continue;
			}
			for (unsigned i = 0; i < slave->count; i++) {
				uint32_t base = entry[i].address;

				if (pass == 0 ? base == address : (base < address && address - base < 0x10 && base > best)) {
					best = base;
					best_s = s;
					best_i = i;
					if (pass == 0) {
						break;
					}
				}
			}
			if (pass == 0 && best != 0) {
				break;
			}
		}
		if (best != 0) {
			const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + db->slave[best_s].header_offset);
			char id[9] = { 0 };

			memcpy(id, entry[best_i].id, 8);
			if (address == best) {
				snprintf(out, size, "%s", id);
			} else {
				snprintf(out, size, "%s+%u", id, address - best);
			}
			return;
		}
	}
}

static bool
ss_record(int i, char name[5], uint32_t *count, uint64_t *ticks, bool *in)
{
	uint32_t b = ss_rec_off + (uint32_t)i * SS_RECORD, type;
	uint64_t entered, exited;

	if (ss_rec_off == 0 || !ss_rd(ss_window, b, &type) || !ss_rd(ss_window, b + 4, count) ||
	    !ss_rd64(ss_window, b + 0x8, &entered) || !ss_rd64(ss_window, b + 0x10, &exited) || !ss_rd64(ss_window, b + 0x18, ticks)) {
		return false;
	}
	for (int k = 0; k < 4; k++) {
		char c = (char)(type >> (8 * k));
		name[k] = c >= 0x20 && c < 0x7f ? c : '.';
	}
	name[4] = 0;
	*in = entered > exited;
	return true;
}

/* DDR's entry i: its name word, count and duration */
static bool
ss_ddr_entry(uint32_t i, uint32_t *name, uint32_t *count, uint64_t *ticks)
{
	uint32_t b = ss_ddr_off + 8 + 16 * i;

	return ss_ddr_off != 0 && ss_rd(ss_window, b, name) && ss_rd(ss_window, b + 4, count) && ss_rd64(ss_window, b + 8, ticks);
}

static void
ss_aop_say(const char *text, uint32_t settle_ms)
{
	if (hakios_aop_message(text)) {
		delay_for_interval(settle_ms, NSEC_PER_MSEC);
	} else {
		ss_say("(the AOP did not take \"%s\")\n", text);
	}
}

/* under ss_lock */
static void
ss_snapshot(void)
{
	uint64_t now_ns;
	char name[5], res[16];
	uint32_t count, word;
	uint64_t ticks, total = 0;
	bool in;

	ss_at = 0;
	ss_text[0] = 0;
	ss_map();
	absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
	ss_say("uptime %llu ms\n", now_ns / 1000000);

	for (int i = 0; i < SS_RECORDS; i++) {
		if (!ss_record(i, name, &count, &ticks, &in)) {
			ss_say("sleep record %d: not readable\n", i);
			continue;
		}
		ss_say("sleep %s: count %u, time %llu ms%s\n", name, count, ticks / AOP_TICKS_PER_MS, in ? " (in it now)" : "");
	}

	if (ss_ddr_off != 0) {
		ss_aop_say("{class: ddr, action: freqsync}", 20);
		for (uint32_t i = 0; i < ss_ddr_n; i++) {
			uint32_t n;
			if (ss_ddr_entry(i, &n, &count, &ticks) && ((n >> 8) & 0xff) == 1) {
				total += ticks;
			}
		}
		for (uint32_t i = 0; i < ss_ddr_n; i++) {
			uint32_t n;
			if (!ss_ddr_entry(i, &n, &count, &ticks)) {
				ss_say("ddr entry %u: not readable\n", i);
			} else if (ticks == 0 && ((n >> 8) & 0xff) != 0) {
				continue;       /* a level never used: left out (sweetd's 4 KB) */
			} else if (((n >> 8) & 0xff) == 0) {
				ss_say("ddr lpm %#x: count %u, time %llu ms\n", n & 0xff, count, ticks / AOP_TICKS_PER_MS);
			} else if (((n >> 8) & 0xff) == 1 && (n >> 16) != 0) {
				ss_say("ddr freq %u MHz (cp %u): count %u, time %llu ms (%llu%%)\n", n >> 16, n & 0x1f, count,
				    ticks / AOP_TICKS_PER_MS, total ? ticks * 100 / total : 0);
			} else {
				ss_say("ddr entry %u: name %#x, count %u, time %llu ms\n", i, n, count, ticks / AOP_TICKS_PER_MS);
			}
		}
		ss_aop_say("{class: ddr, res: drvs_ddr_votes}", 20);
		ss_say("ddr votes by DRV (ab/ib; names as sys_pm_vx's [I]):");
		for (uint32_t i = 0; i < DDR_DRVS; i++) {
			if (!ss_rd(ss_window, ss_ddr_off + 8 + 16 * ss_ddr_n + 4 * i, &word)) {
				ss_say(" %u:?", i);
			} else if (word == 0xdeaddead) {
				ss_say(" %u(%s):absent", i, vx_drv[i]);
			} else if (word == 0xffffdead) {
				ss_say(" %u(%s):invalid", i, vx_drv[i]);
			} else {
				ss_say(" %u(%s):%u/%u", i, vx_drv[i], (word >> 14) & 0x3fff, word & 0x3fff);
			}
		}
		ss_say("\n");
	}

	/* who kept CX up */
	if (!ss_vx_asked) {
		ss_vx_asked = true;
		ss_aop_say("{class: lpm_mon, type: cxpc, dur: 1000, flush: 5, ts_adj: 1}", 20);
		ss_say("cx monitor: started now; read again later\n");
	}
	uint32_t w0, w1;
	if (!ss_rd(ss_vx, 0, &w0) || !ss_rd(ss_vx, 4, &w1)) {
		ss_say("cx monitor: not readable\n");
	} else {
		uint32_t logsize = (w0 >> 8) & 0xff, shift = (w1 >> 16) & 0xff, lines = 0, marks = 0, held[VX_DRVS] = { 0 };
		uint32_t top[VX_DRVS] = { 0 }, at = 8;

		ss_say("cx monitor: mode %#x, %u ms a line, time shift %u, flush %u, up to %u lines\n", w0 & 0xff, w1 & 0xffff,
		    shift, w1 >> 24, logsize);
		for (uint32_t l = 0; l < logsize && at + 4 * (1 + VX_WORDS) <= VX_SIZE; l++) {
			uint32_t ts, v[VX_DRVS], any = 0;

			if (!ss_rd(ss_vx, at, &ts) || ts == 0) {
				break;
			}
			at += 4;
			for (uint32_t k = 0; k < VX_WORDS; k++, at += 4) {
				uint32_t word4 = 0;
				(void)ss_rd(ss_vx, at, &word4);
				for (uint32_t j = 0; j < 4 && 4 * k + j < VX_DRVS; j++) {
					v[4 * k + j] = (word4 >> (8 * j)) & 0xff;
					any |= v[4 * k + j];
				}
			}
			lines++;
			if (!any) {
				marks++;
			}
			for (uint32_t j = 0; j < VX_DRVS; j++) {
				held[j] += v[j] != 0;
				top[j] = MAX(top[j], v[j]);
			}
			if (l < 6) {
				ss_say("cx line %u at %#x:", l, ts << shift);
				for (uint32_t j = 0; j < VX_DRVS; j++) {
					if (v[j]) {
						ss_say(" %s=%u", vx_drv[j], v[j]);
					}
				}
				ss_say(any ? "\n" : " (collapse entry/exit)\n");
			}
		}
		ss_say("cx monitor: %u lines, %u all-zero; held CX in (lines, most):", lines, marks);
		for (uint32_t j = 0; j < VX_DRVS; j++) {
			if (held[j]) {
				ss_say(" %s %u/%u%s", vx_drv[j], held[j], top[j], held[j] == lines ? " (every line)" : "");
			}
		}
		ss_say("\n");
	}

	/* what the vote keeper (step 5b) sums and last wrote */
	ss_say("floor: %s\n", floor_line);
	lck_mtx_lock(&keep_lock);
	for (unsigned k = 0; k < keep_n; k++) {
		ss_say("keep %.8s %#x (unit %u width %u): sent %#x;", keep[k].name, keep[k].address, keep[k].unit, keep[k].width,
		    keep[k].sent);
		for (unsigned c = 0; c < KEEP_CLIENTS; c++) {
			if (keep[k].ab[c] || keep[k].ib[c]) {
				static const char *const owners[KEEP_CLIENTS] = { "cpu", "touch", "modem", "display", "usb", "pcie", "camera", "iris", "icp" };
				ss_say(" client %u(%s) %u/%u", c, owners[c], keep[k].ab[c], keep[k].ib[c]);
			}
		}
		ss_say("\n");
	}
	lck_mtx_unlock(&keep_lock);

	/* Descriptors are programming/history, not a readback of live ARC/VRM votes. */
	ss_say("rsc descriptors (not live votes; active may be stale):\n");
	/* the APSS DRV's command slots */
	for (unsigned t = 0; ss_rsc != 0 && t < RSC_TCS_8450; t++) {
		uint32_t enabled = *(volatile uint32_t *)(ss_rsc + RSC_TCS(t, 0x1c));

		for (unsigned c = 0; c < RSC_CMDS; c++) {
			uint32_t msgid = *(volatile uint32_t *)(ss_rsc + RSC_TCS(t, 0x30 + 20 * c));
			uint32_t address = *(volatile uint32_t *)(ss_rsc + RSC_TCS(t, 0x34 + 20 * c));
			uint32_t data = *(volatile uint32_t *)(ss_rsc + RSC_TCS(t, 0x38 + 20 * c));

			/* (an active TCS's commands that are not on are done: history, and the keeper lines say what is in
			 * force; left out so the snapshot stays under sweetd's 4 KB) */
			if ((address == 0 && data == 0) || (t < 3 && !(enabled & (1u << c)))) {
				continue;
			}
			ss_resource(address, res, sizeof(res));
			/* (the msg id only where it is not the sets' usual 4-byte write with a response) */
			if (msgid == 0x10108 && t >= 3) {
				ss_say("rsc tcs %u (%s) cmd %u%s: %s %#x = %#x\n", t, t < 5 ? "sleep" : t < 7 ? "wake" : "fast", c,
				    enabled & (1u << c) ? " on" : "", res[0] ? res : "?", address, data);
				continue;
			}
			ss_say("rsc tcs %u (%s) cmd %u%s: %s %#x = %#x (msg %#x)\n", t,
			    t < 3 ? "active" : t < 5 ? "sleep" : t < 7 ? "wake" : "fast", c, enabled & (1u << c) ? " on" : "",
			    res[0] ? res : "?", address, data, msgid);
		}
	}
	ss_say("rsc: end (slots not listed are empty)\n");
}

/* every LOG_SECONDS, under ss_lock */
static void
ss_sample(void)
{
	struct ss_sample *s = &ss_log[ss_logged % LOG_SAMPLES];
	uint64_t now_ns;
	char name[5];
	bool in;

	ss_map();
	bzero(s, sizeof(*s));
	absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
	s->t = (uint32_t)(now_ns / NSEC_PER_SEC);
	(void)hakios_battery_power(&s->ma, &s->mv, &s->charger);
	for (int i = 0; i < 2; i++) {
		(void)ss_record(i, name, &s->count[i], &s->ticks[i], &in);
	}
	if (ss_ddr_off != 0 && hakios_aop_message("{class: ddr, action: freqsync}")) {
		delay_for_interval(20, NSEC_PER_MSEC);
		for (uint32_t i = 0; i < ss_ddr_n; i++) {
			uint32_t n, count, f;
			uint64_t ticks;

			if (!ss_ddr_entry(i, &n, &count, &ticks) || ((n >> 8) & 0xff) != 1 || (n >> 16) == 0) {
				continue;
			}
			for (f = 0; f < ss_freqs && ss_freq_mhz[f] != n >> 16; f++) {
			}
			if (f == ss_freqs && ss_freqs < LOG_FREQS) {
				ss_freq_mhz[ss_freqs++] = n >> 16;
			}
			if (f < LOG_FREQS) {
				s->freq_ticks[f] = ticks;
			}
		}
	}
	ss_logged++;
}

__dead2 static void
ss_thread(__unused void *arg, __unused wait_result_t wr)
{
	thread_set_thread_name(current_thread(), "rpmh-stats");
	for (;;) {
		lck_mtx_lock(&ss_lock);
		ss_sample();
		lck_mtx_unlock(&ss_lock);
		assert_wait_timeout((event_t)&ss_logged, THREAD_UNINT, LOG_SECONDS, NSEC_PER_SEC);
		thread_block(THREAD_CONTINUE_NULL);
	}
}

static int
rpmh_stats_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int error;

	if (req->oldptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, ss_text, sizeof(ss_text));       /* the size only: the snapshot is taken by the read */
	}
	lck_mtx_lock(&ss_lock);
	ss_snapshot();
	error = SYSCTL_OUT(req, ss_text, strlen(ss_text) + 1);
	lck_mtx_unlock(&ss_lock);
	return error;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, rpmh_stats_sysctl, "A",
    "the AOP's sleep records (aosd, cxsd), DDR's time per frequency and votes per DRV, who held CX, the APSS RSC's slots");

/* the log: a line per sample with what grew since the one before */
static void
ss_log_describe(void)
{
	uint32_t first = ss_logged > LOG_SAMPLES ? ss_logged - LOG_SAMPLES : 0;
	int at = 0;
	int64_t off_ma = 0;
	uint32_t off_n = 0;

	at += snprintf(ss_log_text + at, sizeof(ss_log_text) - (size_t)at,
	    "every %u s: t(s) mA mV charger | aosd +n +ms | cxsd +n +ms | ddr: MHz of most time (%% of it)\n", LOG_SECONDS);
	for (uint32_t k = first + 1; k < ss_logged && at < (int)sizeof(ss_log_text) - 100; k++) {
		const struct ss_sample *a = &ss_log[(k - 1) % LOG_SAMPLES], *b = &ss_log[k % LOG_SAMPLES];
		uint64_t grew = 0, best = 0;
		uint32_t best_mhz = 0;

		for (uint32_t f = 0; f < ss_freqs; f++) {
			uint64_t d = b->freq_ticks[f] >= a->freq_ticks[f] ? b->freq_ticks[f] - a->freq_ticks[f] : 0;
			grew += d;
			if (d > best) {
				best = d;
				best_mhz = ss_freq_mhz[f];
			}
		}
		at += snprintf(ss_log_text + at, sizeof(ss_log_text) - (size_t)at, "%u %d %d %d | %u %llu | %u %llu | %u (%llu%%)\n",
		    b->t, b->ma, b->mv, b->charger, b->count[0] - a->count[0], (b->ticks[0] - a->ticks[0]) / AOP_TICKS_PER_MS,
		    b->count[1] - a->count[1], (b->ticks[1] - a->ticks[1]) / AOP_TICKS_PER_MS, best_mhz, grew ? best * 100 / grew : 0);
		if (!b->charger) {
			off_ma += b->ma;
			off_n++;
		}
	}
	if (at < (int)sizeof(ss_log_text) - 100) {
		snprintf(ss_log_text + at, sizeof(ss_log_text) - (size_t)at, "%u samples; without a charger: %u, mean %lld mA\n",
		    ss_logged - first, off_n, off_n ? off_ma / (int64_t)off_n : 0);
	}
}

static int
rpmh_log_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int error;

	if (req->oldptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, ss_log_text, sizeof(ss_log_text));
	}
	lck_mtx_lock(&ss_lock);
	ss_log_describe();
	error = SYSCTL_OUT(req, ss_log_text, strlen(ss_log_text) + 1);
	lck_mtx_unlock(&ss_lock);
	return error;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_log, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, rpmh_log_sysctl, "A",
    "every 30 s: battery mA/mV/charger, aosd and cxsd growth, the DDR frequency the time went to");

static void
ss_start(void)
{
	thread_t thread;

	if (kernel_thread_start(ss_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

void
hakios_cpufreq_init(void)
{
	const ml_topology_info_t *topology = ml_get_topology_info();
	uint32_t enabled = 0, membench = 0, cores[EPSS_DOMAINS] = { 0 }, before, here;

	load_start();
	(void)PE_parse_boot_argn("cpufreq_epss", &epss_phys, sizeof(epss_phys));
	/* the vote's addresses are the reference phone's: elsewhere only when ddr= asks */
	enabled = epss_phys == EPSS_PHYS ? 2 : 0;
	PE_parse_boot_argn("ddr", &enabled, sizeof(enabled));
	if (enabled == 1) {
		ddr_look();
		ddr_slots();
	} else if (enabled == 2 && cpufreq_wanted()) {
		ddr_raise();
	}
	if (epss_phys == EPSS_PHYS_8450) {
		keep_start();   /* the APSS DRV's bandwidth votes and the DDR/LLCC floor (hakios_rpmh_bw) */
		ss_start();     /* the SoC's sleep statistics and the power log (kern.hakios.rpmh_stats, rpmh_log) */
	}
	enabled = 0;
	if (!PE_parse_boot_argn("cpufreq", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	for (unsigned int i = 0; i < topology->num_cpus; i++) {
		cores[domain_of(topology->cpus[i].phys_id)]++;
	}
	/* (whichever core this thread happens to be on, and stays on or not: it only goes to the log) */
	here = (uint32_t)(__builtin_arm_rsr64("MPIDR_EL1") >> 8) & 0xff;
	before = loop_speed();
	for (uint32_t domain = 0; domain < EPSS_DOMAINS; domain++) {
		if (cores[domain] != 0) {
			raise_domain(domain, cores[domain]);
		}
	}
	printf("cpufreq: on core %u a loop ran at %u, now at %u\n", here, before, loop_speed());
	cpufreq_describe();
	gov_start();
	/* membench has its own variable: when the boot-arg is absent, the cpufreq value (2 means on) must not start the benchmark */
	PE_parse_boot_argn("membench", &membench, sizeof(membench));
	if (membench == 1) {
		memory_bench();
	} else if (membench == 2) {
		thread_t thread;

		if (kernel_thread_start(bench_all, NULL, &thread) == KERN_SUCCESS) {
			thread_deallocate(thread);
		}
	}
}
