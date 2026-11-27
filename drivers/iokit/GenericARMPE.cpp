/*
 * GenericARMPE: a minimal platform expert for non-Apple ARMv8 boards.
 *
 * On Apple hardware the platform expert comes from a closed kext
 * (AppleARMPlatform). Without one, IOKit falls through to IOPanicPlatform
 * ("Unable to find driver for this platform"). This class is the least that
 * lets IOKit start on a board described by a loader-built device tree:
 * it publishes the device-tree nodes as nubs and names the machine.
 */
extern "C" {
#include <pexpert/pexpert.h>

#include <pexpert/arm64/board_config.h>
#include <kern/cpu_number.h>
}
#include <machine/machine_routines.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOCPU.h>
#include <IOKit/IOPlatformActions.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <IOKit/pwr_mgt/IOPMPrivate.h>
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <sys/errno.h>
#include <sys/sysctl.h>

#include "MiniDrivers.h"
#include "build_date.h"

extern "C" {
void ml_cpu_init_completed(void);      /* osfmk/arm64/machine_routines.c: every processor started (the scheduler's) */
kern_return_t processor_start(processor_t processor);
kern_return_t processor_exit(processor_t processor);
extern uint32_t processor_avail_count;
int hakios_sleep_wait(uint32_t seconds, uint32_t *woken_by, uint64_t *asleep_ns, uint32_t *pets);    /* psci_idle.c */
size_t hakios_sleep_deep_text(char *buf, size_t size);                                                  /* psci_idle.c */
void hakios_sleep_keep_lpis(bool keep);                                                                 /* psci_idle.c */
int hakios_ufs_sleep_prepare(unsigned ms, int *why) __attribute__((weak));                              /* ufshc_power.h */
bool hakios_soc_sleep_records(uint32_t count[3], uint64_t ms[3]) __attribute__((weak));                 /* soc_sleep.c */
size_t hakios_soc_sleep_arc_delta(bool start, char *buf, size_t size) __attribute__((weak));             /* soc_sleep.c */
int lx_doze_for_sleep(unsigned ms);                                                                     /* lx_80211.c */
void lx_doze_sleep_over(void);
int hakios_gpu_sleep_now(void) __attribute__((weak));                                                   /* BoardGPU.cpp */
uint32_t hakios_adsp_wake_intid(void) __attribute__((weak));                                            /* adsp.c */
uint32_t hakios_usb_wake_intid(void) __attribute__((weak));                                             /* usb_net.c */
int hakios_sound_sleep_prepare(void) __attribute__((weak));                                             /* sound.c */
extern "C" void hakios_charger_arrived(void);
static IOService * gThePE;              /* (start(): the device requestUserActive() names) */
static void alarmsRegister(IOService * pe);
void loader_log_line(const char *line);

/* osfmk/arm64/psci_smp.c and gic_v3_irq.c */
void psci_smp_init(void);
int psci_smp_boot_cpu_number(void);     /* (the kernel's master_cpu is never set on this kernel: it stays -1) */
kern_return_t psci_smp_start_cpu(int cpu);
void psci_smp_cpu_running(void);
void psci_smp_cpu_unplug(void) __dead2;
void psci_smp_set_screen_off_cores(uint64_t cores);
uint64_t psci_smp_cores_up(void);
size_t hakios_wakes_text(char *buf, size_t size);
size_t hakios_idle_text(char *buf, size_t size);
extern uint32_t hakios_screen_slow;
extern uint64_t psci_smp_screen_off_cores;
void gic_v3_irq_signal(uint32_t phys_id);

/* the kernel's version string, with the date it was built (libkern/version.h, which the build makes from a template) */
extern const char version[];
void psci_idle_init(void);
void psci_idle_notify(cpu_id_t cpu_id, boolean_t enter, uint64_t *new_timeout_ticks);
}

/*
 * GenericARMCPU: registers a device-tree CPU node with the kernel. The closed
 * AppleARMCPU driver does this on Apple hardware; without it the kernel waits
 * forever for the CPU count (ml_wait_max_cpus) and never calls
 * cpu_machine_init() on the boot processor.
 *
 * The other processors are started through the firmware (PSCI CPU_ON, see
 * osfmk/arm64/psci_smp.c) and signalled with interrupts of the GIC
 * (osfmk/arm64/gic_v3_irq.c). How many there are is the device tree's and the
 * boot-arg cpus=N's to say (ml_parse_cpu_topology()); with one, nothing of
 * that is used.
 */
class GenericARMCPU : public IOCPU
{
	OSDeclareDefaultStructors(GenericARMCPU);

	uint32_t physID;        /* the device tree's "reg": the low affinity fields of the core's MPIDR */

public:
	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
	virtual void initCPU(bool boot) APPLE_KEXT_OVERRIDE;
	virtual void quiesceCPU(void) APPLE_KEXT_OVERRIDE;
	virtual kern_return_t startCPU(vm_offset_t start_paddr, vm_offset_t arg_paddr) APPLE_KEXT_OVERRIDE;
	virtual void haltCPU(void) APPLE_KEXT_OVERRIDE;
	virtual void signalCPU(IOCPU * target) APPLE_KEXT_OVERRIDE;
	virtual const OSSymbol * getCPUName(void) APPLE_KEXT_OVERRIDE;

	static int cpuNumberOfNub(IOService * nub);
	void run(void);
};

OSDefineMetaClassAndStructors(GenericARMCPU, IOCPU);

/* The kernel's number for the processor of a /cpus node, -1 if it is not to be used (boot-arg cpus=N). */
int
GenericARMCPU::cpuNumberOfNub(IOService * nub)
{
	OSData * reg = OSDynamicCast(OSData, nub->getProperty("reg"));

	if (nub->getProperty("state") == NULL || reg == NULL || reg->getLength() < sizeof(uint32_t)) {
		return -1;
	}
	return ml_get_cpu_number(*(const uint32_t *)reg->getBytesNoCopy());
}

/* Registers the processor; run() starts it. */
bool
GenericARMCPU::start(IOService * provider)
{
	ml_processor_info_t info;
	perfmon_interrupt_handler_func pmi_handler = NULL;
	int cpu = cpuNumberOfNub(provider);

	if (cpu < 0 || !IOCPU::start(provider)) {
		return false;
	}
	const ml_topology_cpu_t * topology = &ml_get_topology_info()->cpus[cpu];

	physID = topology->phys_id;
	setCPUNumber((UInt32)cpu);

	bzero(&info, sizeof(info));
	info.cpu_id = (cpu_id_t)this;
	info.log_id = getCPUNumber();
	info.phys_id = physID;
	info.cluster_id = topology->cluster_id;
	info.cluster_type = topology->cluster_type;
	info.processor_idle = psci_idle_notify;         /* the idle states (psci_smp.c): around cpu_idle()'s WFI */
	if (ml_processor_register(&info, &machProcessor, &ipi_handler, &pmi_handler) != KERN_SUCCESS) {
		IOLog("GenericARMCPU: ml_processor_register failed\n");
		return false;
	}

	setCPUState(kIOCPUStateUninitalized);
	registerService();
	IOLog("GenericARMCPU: cpu %u (phys id 0x%x) registered\n", (unsigned)getCPUNumber(), physID);
	return true;
}

/*
 * The boot processor: runs cpu_machine_init() -> initCPU(true) on it, right here.
 * Any other: cpu_start() -> startCPU(), and the new core comes to initCPU() by itself.
 */
void
GenericARMCPU::run(void)
{
	processor_start(machProcessor);
}

/* On the processor itself, with its interrupts off, when it joins (or rejoins) the scheduler. */
void
GenericARMCPU::initCPU(bool boot)
{
	/* The interrupt controller and timer were set up in arm_init / arm_init_cpu (pe_init_fiq). */
	setCPUState(kIOCPUStateRunning);
	if (ml_get_cpu_count() > 1) {
		psci_smp_cpu_running();
		/*
		 * A processor takes no signals until it has handled an interrupt for one
		 * (cpu_signal_handler() lifts SIGPdisabled), and until then it does not go to
		 * sleep when idle either: send it its first.
		 */
		gic_v3_irq_signal(physID);
	}
}

/*
 * On the processor itself, at the end of processor_exit(): it is switched off, and may be started again (while the
 * display is off, psci_smp.c's screen_off_cores).
 */
void
GenericARMCPU::quiesceCPU(void)
{
	if (getCPUNumber() != (UInt32)psci_smp_boot_cpu_number()) {
		setCPUState(kIOCPUStateStopped);
		psci_smp_cpu_unplug();
	}
}

kern_return_t
GenericARMCPU::startCPU(vm_offset_t start_paddr, vm_offset_t arg_paddr)
{
	return psci_smp_start_cpu((int)getCPUNumber());
}

void
GenericARMCPU::haltCPU(void)
{
	if (getCPUNumber() != (UInt32)psci_smp_boot_cpu_number()) {
		processor_exit(machProcessor);
	}
}

void
GenericARMCPU::signalCPU(IOCPU * target)
{
	gic_v3_irq_signal(((GenericARMCPU *)target)->physID);
}

const OSSymbol *
GenericARMCPU::getCPUName(void)
{
	char name[32];
	snprintf(name, sizeof(name), "Primary%u", (unsigned)getCPUNumber());
	return OSSymbol::withCString(name);
}

/*
 * kern.hakios.screen_off_cores: the processors switched off while the display is off (boot-arg screen_off_cores, a bit
 * per processor number; 0: none). kern.hakios.cpus_up: the processors that are up now.
 */
SYSCTL_DECL(_kern_hakios);

static int
screenOffCoresSysctl(__unused struct sysctl_oid * oidp, __unused void * arg1, __unused int arg2, struct sysctl_req * req)
{
	uint64_t value = psci_smp_screen_off_cores;
	int error = sysctl_io_number(req, (long long)value, sizeof(value), &value, NULL);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (value & (1ULL << psci_smp_boot_cpu_number())) {
		return EINVAL;
	}
	psci_smp_set_screen_off_cores(value);
	return 0;
}

SYSCTL_PROC(_kern_hakios, OID_AUTO, screen_off_cores, CTLTYPE_QUAD | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    screenOffCoresSysctl, "QU", "the processors switched off while the display is off (screen_off_cores)");

static int
cpusUpSysctl(__unused struct sysctl_oid * oidp, __unused void * arg1, __unused int arg2, struct sysctl_req * req)
{
	uint64_t value = psci_smp_cores_up();

	return SYSCTL_OUT(req, &value, sizeof(value));
}

SYSCTL_PROC(_kern_hakios, OID_AUTO, cpus_up, CTLTYPE_QUAD | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    cpusUpSysctl, "QU", "the processors that are up now, a bit per processor number");

/* kern.hakios.wakes: interrupts per processor and context switches per kernel thread, as text (psci_smp.c) */
#define WAKES_TEXT      196608  /* (with the wake diagnostics: under the 256 KB the readers ask with) */
#define IDLE_TEXT       4000    /* (sweetd's sysctl reads 4 KB) */

static int
wakesSysctl(__unused struct sysctl_oid * oidp, __unused void * arg1, __unused int arg2, struct sysctl_req * req)
{
	char * text = (char *)IOMallocData(WAKES_TEXT);
	int error;

	if (text == NULL) {
		return ENOMEM;
	}
	size_t length = hakios_wakes_text(text, WAKES_TEXT);
	error = SYSCTL_OUT(req, text, length + 1);
	IOFreeData(text, WAKES_TEXT);
	return error;
}

SYSCTL_PROC(_kern_hakios, OID_AUTO, wakes, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    wakesSysctl, "A", "interrupts per processor and context switches per kernel thread (STANDBY-PLAN.md)");

/* kern.hakios.idle: the display, the idle states and a line per processor of the same, under 4 KB */
static int
idleSysctl(__unused struct sysctl_oid * oidp, __unused void * arg1, __unused int arg2, struct sysctl_req * req)
{
	char * text = (char *)IOMallocData(IDLE_TEXT);
	int error;

	if (text == NULL) {
		return ENOMEM;
	}
	size_t length = hakios_idle_text(text, IDLE_TEXT);
	error = SYSCTL_OUT(req, text, length + 1);
	IOFreeData(text, IDLE_TEXT);
	return error;
}

SYSCTL_PROC(_kern_hakios, OID_AUTO, idle, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    idleSysctl, "A", "the display, the idle states, and each processor's power-downs (kern.hakios.wakes in brief)");

/*
 * kern.hakios.screen_off_test (root, write 1 / 0): the processors and the drivers' looks as if the display had gone off /
 * come on (what RequestPowerChange does, without the panel): for runs where nothing locks the screen.
 */
static int
screenOffTestSysctl(__unused struct sysctl_oid * oidp, __unused void * arg1, __unused int arg2, struct sysctl_req * req)
{
	int value = 0, error = sysctl_handle_int(oidp, &value, 0, req);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (value < 0 || value > 1) {
		return EINVAL;
	}
	psci_smp_display(value == 0);
	return 0;
}

SYSCTL_PROC(_kern_hakios, OID_AUTO, screen_off_test, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    screenOffTestSysctl, "I", "test: 1 as if the display had gone off, 0 come on (processors, drivers' looks)");

SYSCTL_UINT(_kern_hakios, OID_AUTO, screen_off_slow, CTLFLAG_RW | CTLFLAG_LOCKED, &hakios_screen_slow, 0,
    "the drivers' loops look less often while the display is off (screen_off_slow)");

class GenericARMPE : public IODTPlatformExpert
{
	OSDeclareDefaultStructors(GenericARMPE);

public:
	virtual IOService * probe(IOService * provider, SInt32 * score) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
	virtual const char * deleteList(void) APPLE_KEXT_OVERRIDE;
	virtual const char * excludeList(void) APPLE_KEXT_OVERRIDE;
	virtual bool getMachineName(char * name, int maxLength) APPLE_KEXT_OVERRIDE;
	virtual long getGMTTimeOfDay(void) APPLE_KEXT_OVERRIDE;
	virtual void setGMTTimeOfDay(long secs) APPLE_KEXT_OVERRIDE;
	virtual void getUTCTimeOfDay(clock_sec_t * secs, clock_nsec_t * nsecs) APPLE_KEXT_OVERRIDE;
	virtual void setUTCTimeOfDay(clock_sec_t secs, clock_nsec_t nsecs) APPLE_KEXT_OVERRIDE;
	virtual void sleepKernel(void) APPLE_KEXT_OVERRIDE;
};

#define super IODTPlatformExpert

OSDefineMetaClassAndStructors(GenericARMPE, IODTPlatformExpert);

IOService *
GenericARMPE::probe(IOService * provider, SInt32 * score)
{
	/*
	 * IODTPlatformExpert::probe() wants an IONameMatch against the root
	 * node; this expert is the only one built in, so it takes any board.
	 */
	if (!IOPlatformExpert::probe(provider, score)) {
		return NULL;
	}
	return this;
}

bool
GenericARMPE::start(IOService * provider)
{
	if (!super::start(provider)) {
		return false;
	}
	gThePE = this;
	IOLog("GenericARMPE: platform \"%s\"\n", provider->getName());
	/*
	 * The time of day is this platform expert's (getGMTTimeOfDay(): the loader's boot epoch), not an
	 * RTC driver's: say so, or bsd_init's IOKitInitializeTime() waits 30 seconds for an "IORTC" that
	 * never comes before it reads the calendar from here all the same.
	 */
	publishResource("IORTC", this);
	alarmsRegister(this);

	/*
	 * super::start() published the /cpus children as nubs. Attach a CPU driver to each
	 * one the kernel has a number for, register them all -- that settles the CPU count --
	 * and then start them, the boot processor first.
	 */
	static GenericARMCPU * cpus[MAX_CPUS];        /* (static: the slower cores may be started later, see below) */
	unsigned int count = 0;
	OSIterator * iter = getChildIterator(gIOServicePlane);
	if (iter != NULL) {
		OSObject * obj;
		while ((obj = iter->getNextObject()) != NULL) {
			IOService * nub = OSDynamicCast(IOService, obj);
			int number = nub ? GenericARMCPU::cpuNumberOfNub(nub) : -1;

			if (number < 0 || number >= MAX_CPUS || cpus[number] != NULL) {
				continue;
			}
			GenericARMCPU * cpu = new GenericARMCPU;
			if (cpu != NULL && cpu->init() && cpu->attach(nub)) {
				if (cpu->start(nub)) {
					cpus[number] = cpu;     /* kept: the kernel has its address as the cpu's id */
					count++;
					continue;
				}
				cpu->detach(nub);
			}
			if (cpu != NULL) {
				cpu->release();
			}
		}
		iter->release();
	}
	int boot = psci_smp_boot_cpu_number();         /* (the boot processor's: the kernel's master_cpu stays -1 here) */

	if (boot >= 0 && boot < MAX_CPUS && cpus[boot] != NULL) {
		/* Unblocks everything that waits for the CPU count to be known. */
		ml_set_max_cpus(count);
		if (count > 1) {
			psci_smp_init();
			psci_idle_init();
		}
		/*
		 * The slower cores (an L2 access penalty, the loader's "small-core") are started 28 seconds
		 * later (boot-arg small_cores_after=S; 0: at once). With all seven from the start the lock
		 * screen came 1.5 s later than with the four big ones alone (26.5-27.0 s after the kernel
		 * began instead of 25.3-25.5: start-up's other work shares memory and cache with
		 * SpringBoard's), while that work as a whole was done earlier (34 s instead of 38). With
		 * the slower cores joining once the lock screen is there: 25.3 s and 36 s.
		 */
		uint32_t later = 28;
		static thread_call_t later_call;
		static bool held;               /* (some core waits for that: not on a machine whose cores are all alike) */

		PE_parse_boot_argn("small_cores_after", &later, sizeof(later));
		cpus[boot]->run();
		for (unsigned int number = 0; number < MAX_CPUS; number++) {
			if (cpus[number] != NULL && number != (unsigned int)boot) {
				if (later != 0 && ml_get_topology_info()->cpus[number].l2_access_penalty != 0) {
					held = true;
					continue;
				}
				cpus[number]->run();
			}
		}
		/* the slower cores' start, if any wait; and, a moment after the last has started, how many run */
		uint64_t deadline;

		later_call = thread_call_allocate([](thread_call_param_t, thread_call_param_t) {
			for (unsigned int number = 0; held && number < MAX_CPUS; number++) {
				if (cpus[number] != NULL && ml_get_topology_info()->cpus[number].l2_access_penalty != 0) {
					cpus[number]->run();
				}
			}
			IOLog("GenericARMPE: %s%u processor%s running\n", held ? "the slower cores started too: " : "all started: ",
			    processor_avail_count, processor_avail_count == 1 ? "" : "s");
			/*
			 * Every processor has been started: the scheduler's own bookkeeping of which are powered may begin (as
			 * AppleARMSMP says it). Until then it takes no request to power a core down or up, and screen_off_cores
			 * (psci_smp.c) was a no-op: the cores stayed up with the display off.
			 */
			ml_cpu_init_completed();
		}, NULL);
		clock_interval_to_deadline(held ? later : 1, NSEC_PER_SEC, &deadline);
		thread_call_enter_delayed(later_call, deadline);
		if (count > 1) {
			IOLog("GenericARMPE: %u of %u processors are running\n", processor_avail_count, count);
		}
	} else {
		/*
		 * The boot core's own node gave no CPU (no state or reg, or its start refused): nothing would ever say how many
		 * there are, and whatever waits in ml_wait_max_cpus() would wait for good, without a word. Said, and the count
		 * set all the same (the boot core runs whatever this says: it is the one running this).
		 */
		IOLog("GenericARMPE: THE BOOT PROCESSOR (%d) HAS NO CPU OBJECT: %u others made, none started\n", boot, count);
		ml_set_max_cpus(count ? count : 1);
	}

	/* BSD looks the platform expert up by class (IOSecureBSDRoot). */
	registerService();
	/*
	 * The clock is this platform expert (getGMTTimeOfDay below). IOKitInitializeTime waits up to 30 s for a
	 * resource "IORTC", which a real-time clock's driver publishes, before it sets the calendar: without it
	 * every boot of xnu-12377 stood still for those 30 s at "bsd_init: calling IOKitInitializeTime".
	 */
	publishResource("IORTC", this);

	/*
	 * Boot-arg iokit_busy=S: S seconds after this, the services that are still busy -- matching, starting or
	 * waiting -- with their busy count, and the registry root's. iOS's configd makes the network's default
	 * configuration only once the registry is quiet (its InterfaceNamer waits up to 300 s otherwise); this
	 * says what keeps it from being so. (printf: on the console, which the loader keeps whole, where the
	 * kernel's message buffer has lost its first minutes by the end of a run.)
	 */
	uint32_t busy_after = 0;
	if (PE_parse_boot_argn("iokit_busy", &busy_after, sizeof(busy_after)) && busy_after != 0) {
		static thread_call_t busy_call;
		uint64_t deadline;

		busy_call = thread_call_allocate([](thread_call_param_t, thread_call_param_t) {
			IORegistryIterator * iter = IORegistryIterator::iterateOver(gIOServicePlane, kIORegistryIterateRecursively);
			IOService * root = IOService::getServiceRoot();
			unsigned busy = 0;

			printf("iokit_busy: the registry root is %s (busy %u)\n", root && root->getBusyState() ? "BUSY" : "quiet",
			    root ? root->getBusyState() : 0);
			while (iter != NULL) {
				OSObject * obj;
				while ((obj = iter->getNextObject()) != NULL) {
					IOService * service = OSDynamicCast(IOService, obj);
					if (service != NULL && service->getBusyState() != 0) {
						printf("iokit_busy: %s (class %s) busy %u, %s\n", service->getName(), service->getMetaClass()->getClassName(),
						    service->getBusyState(), service->isInactive() ? "inactive" : "active");
						busy++;
					}
				}
				if (iter->isValid()) {
					break;
				}
				iter->reset();
			}
			OSSafeReleaseNULL(iter);
			printf("iokit_busy: %u busy services\n", busy);
		}, NULL);
		clock_interval_to_deadline(busy_after, NSEC_PER_SEC, &deadline);
		thread_call_enter_delayed(busy_call, deadline);
	}

	/* Stand-ins for the closed graphics drivers that the Darwin window server talks to. */
	MiniIOSurfaceStart(this);
	MiniIOMobileFramebufferStart(this);
	MiniAppleKeyStoreStart();
	MiniIONetworkStackStart();
	return true;
}

/*
 * The time of day. There is no driver for a real-time clock here; whoever started the kernel
 * says what time it was then -- the boot-arg epoch= (seconds since 1970, put there by the
 * host scripts) or, failing that, the property boot-epoch of /chosen (the loader, from the
 * firmware's clock) -- and the kernel's own timer counts from there. Without either, or with
 * one earlier than this kernel was built, the calendar starts at the build (build_date.h):
 * a time we know to have passed, where 1970 has every daemon catch up on fifty-six years once
 * the network says the date. Process 1 may raise it further from what the volumes last wrote.
 *
 * The count is the generic timer's own (ml_get_hwclock(), CNTVCT), which runs on while the system sleeps, not
 * mach_absolute_time(), which the sleep holds still (psci_idle.c): clock_wakeup_calendar() counts the sleep from this
 * clock, and on the absolute time every sleep was one of no length -- the calendar fell behind by each, and powerd sent
 * an alarm already rung over and over (run-20261127-063920).
 */
static long gBootEpoch;         /* the calendar time at which the system timer read zero */

extern "C" uint64_t ml_get_hwclock(void);       /* osfmk/arm64: CNTVCT (machine_routines.h keeps it private) */

/* The generic timer's count since it started, which a sleep does not stop. */
static void
hwUptime(clock_sec_t * secs, clock_nsec_t * nsecs)
{
	uint64_t ns = 0;

	absolutetime_to_nanoseconds(ml_get_hwclock(), &ns);
	*secs = (clock_sec_t) (ns / NSEC_PER_SEC);
	*nsecs = (clock_nsec_t) (ns % NSEC_PER_SEC);
}
static bool gBootEpochKnown;

static void
findBootEpoch(void)
{
	uint64_t epoch = 0;
	long earliest;
	const char * from = "epoch=";
	clock_sec_t secs;
	clock_nsec_t nsecs;

	if (gBootEpochKnown) {
		return;
	}
	gBootEpochKnown = true;
	earliest = build_date_floor(version);
	if (!PE_parse_boot_argn("epoch", &epoch, sizeof(epoch)) || epoch == 0) {
		IORegistryEntry * chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);
		OSData * data = chosen ? OSDynamicCast(OSData, chosen->getProperty("boot-epoch")) : NULL;

		from = "the loader's boot-epoch";
		if (data != NULL && data->getLength() == sizeof(epoch)) {
			memcpy(&epoch, data->getBytesNoCopy(), sizeof(epoch));
		}
		OSSafeReleaseNULL(chosen);
	}
	hwUptime(&secs, &nsecs);
	if (epoch < (uint64_t) earliest) {
		if (epoch == 0) {
			printf("GenericARMPE: no time given: the calendar starts at %ld, this kernel's build\n", earliest);
		} else {
			printf("GenericARMPE: %s (%llu) is before this kernel's build: the calendar starts at %ld instead\n", from,
			    (unsigned long long) epoch, earliest);
		}
		epoch = (uint64_t) earliest;
	}
	gBootEpoch = (long) epoch - (long) secs;
}

long
GenericARMPE::getGMTTimeOfDay(void)
{
	clock_sec_t secs;
	clock_nsec_t nsecs;

	getUTCTimeOfDay(&secs, &nsecs);
	return (long) secs;
}

void
GenericARMPE::setGMTTimeOfDay(long secs)
{
	setUTCTimeOfDay((clock_sec_t) secs, 0);
}

/* (what PEGetUTCTimeOfDay() asks, clock_wakeup_calendar() among others: to the nanosecond) */
void
GenericARMPE::getUTCTimeOfDay(clock_sec_t * secs, clock_nsec_t * nsecs)
{
	clock_sec_t up;
	clock_nsec_t upns;

	findBootEpoch();
	if (gBootEpoch == 0) {
		*secs = 0;
		*nsecs = 0;
		return;
	}
	hwUptime(&up, &upns);
	*secs = (clock_sec_t) gBootEpoch + up;
	*nsecs = upns;
}

void
GenericARMPE::setUTCTimeOfDay(clock_sec_t secs, __unused clock_nsec_t nsecs)
{
	clock_sec_t up;
	clock_nsec_t upns;

	hwUptime(&up, &upns);
	gBootEpoch = (long) secs - (long) up;
	gBootEpochKnown = true;
}

/*
 * Alarms (COMMON-PORT.md "System sleep", step 3): the wakes powerd schedules (IOPMSchedulePowerEvent, its own
 * maintenance and sleep-service wakes) reach the root domain as PM settings, for a controller to set the RTC's alarm
 * with (a Mac's AppleRTC). Here the sleep's own wait keeps the time: the boot processor's timer, which runs on in
 * llcc-off, ends the wait at the earliest alarm still to come: a dark wake (sleepKernel()). powerd hands the next one
 * over as the system goes to sleep. Each is a calendar date in UTC (IOPMCalendarStruct, year 0: none) but the relative
 * one, seconds from the sleep's start. An alarm that has fired is forgotten; powerd sets the next.
 */
enum { kAlarmCalendar, kAlarmMaintenance, kAlarmSleepService, kAlarmKinds };
static const char * const gAlarmKeys[kAlarmKinds] = {
	kIOPMSettingAutoWakeCalendarKey, kIOPMSettingMaintenanceWakeCalendarKey, kIOPMSettingSleepServiceWakeCalendarKey
};
static IOLock * gAlarmLock;
static uint64_t gAlarmUTC[kAlarmKinds];         /* seconds since 1970; 0: none */
static uint32_t gAlarmRelative;                 /* WakeRelativeToSleep: seconds; 0: none */
static uint32_t gAlarmSets, gAlarmFired[kAlarmKinds + 1];
#define ALARM_RING 6
static struct { uint64_t due, woke; } gAlarmRing[ALARM_RING];  /* the last that rang: when due, when it woke (UTC s) */
static uint32_t gAlarmRingN;
static OSObject * gAlarmHandle;

/* (IOPMrootDomain's IOPMConvertCalendarToSeconds() is its own: days from 1970-01-01, the proleptic Gregorian way) */
static uint64_t
alarmSeconds(const IOPMCalendarStruct * c)
{
	int64_t y = (int64_t) c->year - (c->month <= 2);
	int64_t era = (y >= 0 ? y : y - 399) / 400;
	int64_t yoe = y - era * 400;
	int64_t doy = (153 * (c->month + (c->month > 2 ? -3 : 9)) + 2) / 5 + c->day - 1;
	int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
	int64_t days = era * 146097 + doe - 719468;

	return days < 0 ? 0 : (uint64_t) days * 86400 + c->hour * 3600u + c->minute * 60u + c->second;
}

static IOReturn
alarmSetting(__unused OSObject * target, const OSSymbol * type, OSObject * value, __unused uintptr_t refcon)
{
	char line[160];

	IOLockLock(gAlarmLock);
	gAlarmSets++;
	if (type->isEqualTo(kIOPMSettingDebugWakeRelativeKey)) {
		OSNumber * n = OSDynamicCast(OSNumber, value);
		gAlarmRelative = n != NULL ? n->unsigned32BitValue() : 0;
		snprintf(line, sizeof(line), "sleep: alarm %s: %u s after the sleep's start\n", type->getCStringNoCopy(),
		    gAlarmRelative);
	} else {
		for (int k = 0; k < kAlarmKinds; k++) {
			if (!type->isEqualTo(gAlarmKeys[k])) {
				continue;
			}
			OSData * d = OSDynamicCast(OSData, value);
			const IOPMCalendarStruct * c = (d != NULL && d->getLength() == sizeof(IOPMCalendarStruct)) ?
			    (const IOPMCalendarStruct *) d->getBytesNoCopy() : NULL;
			gAlarmUTC[k] = (c != NULL && c->year != 0) ? alarmSeconds(c) : 0;
			if (c != NULL && c->year != 0) {
				snprintf(line, sizeof(line), "sleep: alarm %s: %04u-%02u-%02u %02u:%02u:%02u UTC\n",
				    type->getCStringNoCopy(), c->year, c->month, c->day, c->hour, c->minute, c->second);
			} else {
				snprintf(line, sizeof(line), "sleep: alarm %s: none\n", type->getCStringNoCopy());
			}
		}
	}
	IOLockUnlock(gAlarmLock);
	loader_log_line(line);
	return kIOReturnSuccess;
}

static void
alarmsRegister(IOService * pe)
{
	IOPMrootDomain * root = IOService::getPMRootDomain();
	const OSSymbol * keys[kAlarmKinds + 2];
	IOReturn r;

	if (gAlarmLock == NULL) {
		gAlarmLock = IOLockAlloc();
	}
	if (root == NULL) {
		IOLog("GenericARMPE: no root domain yet: powerd's alarms do not reach the sleep\n");
		return;
	}
	for (int k = 0; k < kAlarmKinds; k++) {
		keys[k] = OSSymbol::withCString(gAlarmKeys[k]);
	}
	keys[kAlarmKinds] = OSSymbol::withCString(kIOPMSettingDebugWakeRelativeKey);
	keys[kAlarmKinds + 1] = NULL;
	r = root->registerPMSettingController(keys, alarmSetting, pe, 0, &gAlarmHandle);
	for (int k = 0; k <= kAlarmKinds; k++) {
		keys[k]->release();
	}
	IOLog("GenericARMPE: powerd's alarms end the sleep (the wait's timer): 0x%x\n", r);
}

/*
 * The alarm that ends a sleep starting now: seconds to wait (0: none) and which (kAlarmKinds: the relative one).
 * Calendar alarms already past are left out.
 */
static uint32_t
alarmNext(int * kind)
{
	clock_sec_t now;
	clock_usec_t us;
	uint64_t best = 0;

	*kind = -1;
	if (gAlarmLock == NULL) {
		return 0;
	}
	clock_get_calendar_microtime(&now, &us);
	IOLockLock(gAlarmLock);
	for (int k = 0; k < kAlarmKinds; k++) {
		if (gAlarmUTC[k] > (uint64_t) now && (best == 0 || gAlarmUTC[k] - now < best)) {
			best = gAlarmUTC[k] - now;
			*kind = k;
		}
	}
	if (gAlarmRelative != 0 && (best == 0 || gAlarmRelative < best)) {
		best = gAlarmRelative;
		*kind = kAlarmKinds;
	}
	IOLockUnlock(gAlarmLock);
	return best > 3600 * 24 * 365 ? 3600 * 24 * 365 : (uint32_t) best;
}

/* An alarm that ended the sleep: forgotten (one shot), counted. */
static void
alarmFired(int kind)
{
	clock_sec_t up, now;
	clock_nsec_t upns;

	/*
	 * (the platform's clock, the generic timer's: the calendar takes the sleep in only later, at the root domain's
	 * clock_wakeup_calendar(), and read here it was a whole sleep behind -- run-20261127-065451: "late -143 s")
	 */
	hwUptime(&up, &upns);
	now = (clock_sec_t) gBootEpoch + up;
	IOLockLock(gAlarmLock);
	if (kind >= 0 && kind < kAlarmKinds) {
		gAlarmRing[gAlarmRingN % ALARM_RING].due = gAlarmUTC[kind];
		gAlarmRing[gAlarmRingN % ALARM_RING].woke = now;
		gAlarmRingN++;
	}
	if (kind == kAlarmKinds) {
		gAlarmRelative = 0;
	} else if (kind >= 0) {
		gAlarmUTC[kind] = 0;
	}
	if (kind >= 0) {
		gAlarmFired[kind]++;
	}
	IOLockUnlock(gAlarmLock);
}

static int
alarmsSysctl(__unused struct sysctl_oid * oidp, __unused void * arg1, __unused int arg2, struct sysctl_req * req)
{
	char text[700];
	clock_sec_t now;
	clock_usec_t us;
	size_t at;

	if (gAlarmLock == NULL) {
		return SYSCTL_OUT(req, "not started", sizeof("not started"));
	}
	clock_get_calendar_microtime(&now, &us);
	IOLockLock(gAlarmLock);
	at = (size_t) snprintf(text, sizeof(text), "now %lu; registered %s; settings %u; in s from now (0 none): calendar %lld, maintenance %lld, "
	    "sleep service %lld; relative %u s; fired: calendar %u, maintenance %u, sleep service %u, relative %u",
	    (unsigned long) now, gAlarmHandle != NULL ? "yes" : "NO", gAlarmSets,
	    gAlarmUTC[0] ? (long long) gAlarmUTC[0] - (long long) now : 0LL,
	    gAlarmUTC[1] ? (long long) gAlarmUTC[1] - (long long) now : 0LL,
	    gAlarmUTC[2] ? (long long) gAlarmUTC[2] - (long long) now : 0LL, gAlarmRelative,
	    gAlarmFired[0], gAlarmFired[1], gAlarmFired[2], gAlarmFired[3]);
	/* (the last that rang, oldest first: due@woke, in UTC seconds) */
	uint32_t first = gAlarmRingN > ALARM_RING ? gAlarmRingN - ALARM_RING : 0;
	for (uint32_t i = first; i < gAlarmRingN && at < sizeof(text); i++) {
		at += (size_t) snprintf(text + at, sizeof(text) - at, "%s%llu@%llu", i == first ? "; rang " : " ",
		    (unsigned long long) gAlarmRing[i % ALARM_RING].due,
		    (unsigned long long) gAlarmRing[i % ALARM_RING].woke);
	}
	IOLockUnlock(gAlarmLock);
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}

SYSCTL_PROC(_kern_hakios, OID_AUTO, sleep_alarms, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    alarmsSysctl, "A", "powerd's scheduled wakes as the sleep sees them");

/*
 * System sleep (COMMON-PORT.md "System sleep", step 1): what the root domain calls once every driver is asleep, in place
 * of IOCPUSleepKernel() -- whose boot processor is to come back through a warm reset this port has not got (with more
 * than one processor it waited for good). Suspend to idle: psci_idle.c's hakios_sleep_wait() keeps the boot processor
 * in WFI, the others off, until a wake source (the power key) or the end of a test sleep. The time and the clocks go on
 * meanwhile (the generic timer does not stop), so the calendar needs nothing at the wake. Boot-arg syssleep=0 refuses
 * system sleep (0003-iokit.patch).
 *
 * kern.hakios.sleep_test=S (root): the system to sleep now, the test's timer to wake it after S seconds (1 to 3600).
 * kern.hakios.sleeps: the counts.
 */
static uint32_t gSleepTestSeconds;
static struct {
	uint32_t sleeps, refused, by_timer, by_interrupt, last_by, pets;
	uint64_t asleep_ns, last_ns;
	int last_error;
	/* the last sleep: the radio at its start (2: dozing; -1: no doze thread; -2: no Wi-Fi), the AOP's records' growth
	 * over it (aosd, cxsd, DDR's low-power modes: counts and ms; known: the records were readable) */
	int wifi_doze;
	uint32_t wifi_dozed;
	bool records_known;
	uint32_t record_n[3];
	uint64_t record_ms[3];
	uint32_t test_asks;             /* kern.hakios.sleep_test writes the root domain took */
	int gpu;                        /* the GPU at the last sleep's start: 1 in slumber, 0 awake, -1 none */
	uint32_t dark;                  /* wakes left dark (not the key's nor the test's: the display stays off) */
	uint32_t by_dsp;                /* of those, by the audio DSP's doorbells (its battery manager: a charger plugged in) */
	uint32_t by_alarm;              /* wakes at an alarm's time (powerd's scheduled wakes) */
	uint32_t by_usb;                /* of those, by USB's event interrupt (the host's traffic, on the cable) */
	uint32_t charger_lit;           /* a charger plugged in that lit the display (hakios_charger_arrived()) */
	char set_line[420];             /* what the llcc-off asks found while it slept (soc_sleep.c) */
	bool lpis_kept;                 /* the last sleep left the radio's MSIs (LPIs) unmasked: a wake source */
	int ufs, ufs_why;               /* UFS's domain at the last sleep's start: 1 off, 0 on (why: 1 busy, 2 not in Hibern8,
	                                 * 3 power-down off), -1 none */
	int sound;                      /* the speakers' stream at the last sleep's start: 1 stopped, 0 RUNNING, -1 none */
} gSleeps;
/* kern.hakios.sleep_lpis: 1 (default) a dozing radio's MSIs wake the system; 0 masked while it sleeps (as step 1) */
static uint32_t gSleepLpis = 1;
SYSCTL_UINT(_kern_hakios, OID_AUTO, sleep_lpis, CTLFLAG_RW | CTLFLAG_LOCKED, &gSleepLpis, 0,
    "1: a dozing radio's MSIs (LPIs) wake the sleeping system (Wi-Fi reaches it); 0: masked while it sleeps");

static int
sleepTestSysctl(__unused struct sysctl_oid * oidp, __unused void * arg1, __unused int arg2, struct sysctl_req * req)
{
	int value = 0, error = sysctl_handle_int(oidp, &value, 0, req);
	IOPMrootDomain * root = IOService::getPMRootDomain();
	IOReturn result;

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (value < 1 || value > 3600) {
		return EINVAL;
	}
	if (root == NULL) {
		return ENODEV;
	}
	gSleepTestSeconds = (uint32_t) value;
	result = root->sleepSystemOptions(NULL);
	if (result != kIOReturnSuccess) {
		gSleepTestSeconds = 0;
		return result == kIOReturnNotPermitted ? EPERM : EIO;
	}
	gSleeps.test_asks++;
	return 0;
}

SYSCTL_PROC(_kern_hakios, OID_AUTO, sleep_test, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    sleepTestSysctl, "I", "test: the system sleeps now and its timer wakes it after this many seconds (not with syssleep=0)");

static int
sleepsSysctl(__unused struct sysctl_oid * oidp, __unused void * arg1, __unused int arg2, struct sysctl_req * req)
{
	char text[1500];
	size_t at;

	at = (size_t) snprintf(text, sizeof(text), "sleeps %u (refused %u, last error %d); woken by the test's timer %u, by an "
	    "interrupt %u (last: %u, after %llu ms); asleep %llu ms in all; watchdog fed %u times; ", gSleeps.sleeps,
	    gSleeps.refused, gSleeps.last_error, gSleeps.by_timer, gSleeps.by_interrupt, gSleeps.last_by,
	    gSleeps.last_ns / NSEC_PER_MSEC, gSleeps.asleep_ns / NSEC_PER_MSEC, gSleeps.pets);
	if (at < sizeof(text)) {
		at += hakios_sleep_deep_text(text + at, sizeof(text) - at);
	}
	if (at < sizeof(text)) {
		if (gSleeps.records_known) {
			at += (size_t) snprintf(text + at, sizeof(text) - at, "; last sleep: aosd +%u (+%llu ms), cxsd +%u "
			    "(+%llu ms), ddr lpm +%u (+%llu ms)", gSleeps.record_n[0], gSleeps.record_ms[0], gSleeps.record_n[1],
			    gSleeps.record_ms[1], gSleeps.record_n[2], gSleeps.record_ms[2]);
		} else {
			at += (size_t) snprintf(text + at, sizeof(text) - at, "; last sleep: the AOP's records not read");
		}
	}
	if (at < sizeof(text)) {
		(void) snprintf(text + at, sizeof(text) - at, "; Wi-Fi at the last sleep's start: %s (dozed for %u sleeps); "
		    "GPU %s; UFS %s; sound %s; %s; test sleeps asked %u; dark wakes %u (by the DSP %u, by USB %u); alarms %u; a charger lit it %u; LPIs %s", gSleeps.wifi_doze == 2 ? "dozing" : gSleeps.wifi_doze == -2 ? "none" :
		    gSleeps.wifi_doze == -1 ? "no doze thread" : "awake", gSleeps.wifi_dozed,
		    gSleeps.gpu == 1 ? "in slumber" : gSleeps.gpu == 0 ? "AWAKE" : "none",
		    gSleeps.ufs == 1 ? "off" : gSleeps.ufs == -1 ? "none" : gSleeps.ufs_why == 1 ? "ON (busy)" :
		    gSleeps.ufs_why == 2 ? "ON (not in Hibern8)" : "ON (power-down off)",
		    gSleeps.sound == 1 ? "stopped" : gSleeps.sound == 0 ? "RUNNING" : "none",
		    gSleeps.set_line[0] ? gSleeps.set_line : "sleep set: not read", gSleeps.test_asks, gSleeps.dark,
		    gSleeps.by_dsp, gSleeps.by_usb, gSleeps.by_alarm, gSleeps.charger_lit, gSleeps.lpis_kept ? "live" : "masked");
	}
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}

SYSCTL_PROC(_kern_hakios, OID_AUTO, sleeps, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    sleepsSysctl, "A", "system sleeps: how many, what woke them, how long");

void
GenericARMPE::sleepKernel(void)
{
	IOPMrootDomain * root = getPMRootDomain();
	uint32_t seconds = gSleepTestSeconds, by = 0, pets = 0;
	uint64_t ns = 0;
	char line[160];
	int error;

	uint32_t n0[3] = { 0 }, n1[3] = { 0 };
	uint64_t ms0[3] = { 0 }, ms1[3] = { 0 };
	bool known;
	int alarm = -1;                 /* the alarm whose time ends this sleep (alarmNext()); -1: none */

	gSleepTestSeconds = 0;
	if (seconds == 0) {
		seconds = alarmNext(&alarm);
	}
	/*
	 * Step 2: the radio dozes first (threads still run), so that PCIe gives up its bus vote and CX can collapse while
	 * the system sleeps; frames sent meanwhile wait for the wake.
	 */
	gSleeps.sound = hakios_sound_sleep_prepare != NULL ? hakios_sound_sleep_prepare() : -1;
	gSleeps.gpu = hakios_gpu_sleep_now != NULL ? hakios_gpu_sleep_now() : -1;
	gSleeps.wifi_doze = lx_doze_for_sleep(2500);
	hakios_sleep_keep_lpis(gSleeps.wifi_doze == 2 && gSleepLpis != 0);  /* (a dozing radio calls by MSI: a wake source) */
	/* (storage last: what the radio and the GPU did may have written; its domain on keeps CX up the whole sleep) */
	gSleeps.ufs = hakios_ufs_sleep_prepare != NULL ? hakios_ufs_sleep_prepare(1500, &gSleeps.ufs_why) : -1;
	gSleeps.lpis_kept = gSleeps.wifi_doze == 2 && gSleepLpis != 0;
	if (gSleeps.wifi_doze == 2) {
		gSleeps.wifi_dozed++;
	}
	known = hakios_soc_sleep_records != NULL && hakios_soc_sleep_records(n0, ms0);
	if (hakios_soc_sleep_arc_delta != NULL) {
		(void) hakios_soc_sleep_arc_delta(true, NULL, 0);
	}
	snprintf(line, sizeof(line), "sleep: the system sleeps%s%s; Wi-Fi %s\n",
	    alarm >= 0 ? " until an alarm: " : seconds != 0 ? " (a test: its timer wakes it)" : "",
	    alarm < 0 ? "" : alarm == kAlarmKinds ? kIOPMSettingDebugWakeRelativeKey : gAlarmKeys[alarm], gSleeps.wifi_doze == 2 ? "dozing" : "not dozing");
	loader_log_line(line);
	IOPlatformActionsPreSleep();
	if (root != NULL) {
		root->stop_watchdog_timer();    /* (the sleep/wake watchdog: as IOCPUSleepKernel() does around the sleep) */
	}
	error = hakios_sleep_wait(seconds, &by, &ns, &pets);
	if (root != NULL) {
		root->start_watchdog_timer();
	}
	IOPlatformActionsPostResume();
	lx_doze_sleep_over();
	gSleeps.records_known = known && hakios_soc_sleep_records(n1, ms1);
	gSleeps.set_line[0] = 0;
	if (hakios_soc_sleep_arc_delta != NULL) {
		(void) hakios_soc_sleep_arc_delta(false, gSleeps.set_line, sizeof(gSleeps.set_line));
		snprintf(line, sizeof(line), "sleep: %.150s\n", gSleeps.set_line);
		loader_log_line(line);
	}
	for (int i = 0; i < 3; i++) {
		gSleeps.record_n[i] = n1[i] - n0[i];
		gSleeps.record_ms[i] = ms1[i] - ms0[i];
	}

	gSleeps.last_error = error;
	if (error != 0) {
		gSleeps.refused++;
		snprintf(line, sizeof(line), "sleep: NOT ASLEEP (%d: %s)\n", error,
		    error == ENODEV ? "nothing could wake it" : "the other processors did not go");
		loader_log_line(line);
		return;
	}
	gSleeps.sleeps++;
	gSleeps.last_by = by;
	gSleeps.last_ns = ns;
	gSleeps.asleep_ns += ns;
	gSleeps.pets += pets;
	if (by == 27) {
		gSleeps.by_timer++;
	} else {
		gSleeps.by_interrupt++;
	}
	snprintf(line, sizeof(line), "sleep: woken by %s %u after %llu ms (watchdog fed %u times)\n",
	    by == 27 ? "the test's timer, interrupt" : "interrupt", by, ns / NSEC_PER_MSEC, pets);
	loader_log_line(line);
	snprintf(line, sizeof(line), "sleep: aosd +%u (+%llu ms), cxsd +%u (+%llu ms), ddr lpm +%u (+%llu ms)%s\n",
	    gSleeps.record_n[0], gSleeps.record_ms[0], gSleeps.record_n[1], gSleeps.record_ms[1], gSleeps.record_n[2],
	    gSleeps.record_ms[2], gSleeps.records_known ? "" : " (not read)");
	loader_log_line(line);
	if (root == NULL) {
		return;
	}
	if (by == 27 && alarm >= 0) {
		/*
		 * An alarm's time: a dark wake, whichever key brought it. powerd sends every scheduled wake as
		 * WakeByCalendarDate, the push service's keepalive and the activity scheduler's alike (run-20261127-063920:
		 * six in seven minutes, UserVisible 0); what has something to show (an alarm clock's alert) lights the display
		 * itself, as on an iPhone.
		 */
		alarmFired(alarm);
		gSleeps.by_alarm++;
		gSleeps.dark++;
		snprintf(line, sizeof(line), "sleep: the alarm %s ended it (a dark wake)\n",
		    alarm == kAlarmKinds ? kIOPMSettingDebugWakeRelativeKey : gAlarmKeys[alarm]);
		loader_log_line(line);
		root->claimSystemWakeEvent(this, kIOPMWakeEventSource, "hakios alarm");
	} else if (by == 27 || by == 513) {
		/*
		 * The power key (the PMIC arbiter's line), and the test's timer standing in for it: the user's wake. Without
		 * the claim the root domain took a dark wake, the display stayed off and nothing drew (run-20261126-040608:
		 * presents 955 -> 955). requestUserActive() is powerd's message to light the display (no display wrangler on
		 * this platform).
		 */
		root->claimSystemWakeEvent(this, kIOPMWakeEventSource | kIOPMWakeEventUserPME,
		    by == 27 ? "hakios sleep test timer" : "hakios power key");
		root->requestUserActive(this, by == 27 ? "sleep test wake" : "power key wake");
	} else {
		/*
		 * Step 3: any other wake source (the dozing radio's MSI, an LPI; its WAKE#, 651) is a dark wake: the display
		 * stays off, what woke it is handled (the radio's frames, a sweetd request), and powerd sleeps the system again.
		 */
		bool dsp = hakios_adsp_wake_intid != NULL && by == hakios_adsp_wake_intid();
		bool usb = hakios_usb_wake_intid != NULL && by == hakios_usb_wake_intid();

		gSleeps.dark++;
		gSleeps.by_dsp += dsp;
		gSleeps.by_usb += usb;
		root->claimSystemWakeEvent(this, kIOPMWakeEventSource,
		    (by == 651 || by >= 8192) ? "hakios wifi wake" : dsp ? "hakios DSP wake" : usb ? "hakios USB wake" :
		    "hakios wake source");
	}
}

/*
 * A charger plugged in (battery.c, as the DSP's battery manager or the loader's cache first says so): the display comes
 * on, as on an iPhone. Asleep, the battery manager's notice rang the DSP's doorbells, a dark wake (above); this makes it
 * the user's. Awake with the display off, the same. (requestUserActive() is a message to powerd: any thread.)
 */
extern "C" void
hakios_charger_arrived(void)
{
	IOPMrootDomain * root = IOService::getPMRootDomain();

	if (root == NULL || gThePE == NULL) {
		return;
	}
	gSleeps.charger_lit++;
	loader_log_line("sleep: a charger plugged in: the display on\n");
	root->requestUserActive(gThePE, "charger plugged in");
}

/* Device-tree nodes to drop entirely. */
const char *
GenericARMPE::deleteList(void)
{
	return "('packages', 'psuedo-usb', 'psuedo-hid', 'multiboot', 'rtas')";
}

/* Top-level nodes that describe boot state rather than devices: no nubs for these. */
const char *
GenericARMPE::excludeList(void)
{
	return "('chosen', 'memory', 'options', 'aliases', 'defaults', 'pram', 'cpus')";
}

/*
 * hw.machine. On Apple's devices it is the product type ("iPhone10,3"), from the device tree's
 * "model", and Darwin decides from it what kind of device it is on: with anything it does not know
 * MobileGestalt settles for an iPod touch.
 */
bool
GenericARMPE::getMachineName(char * name, int maxLength)
{
	uint32_t real = 1;

	/*
	 * hw.machine is the model the loader put into the device tree, "iPhone10,3". It has to
	 * begin with "iPhone": CoreFoundation picks the device-specific variants of resources
	 * (Something~iphone.plist, Key~iphone in an Info.plist) by this string, and for a name it
	 * does not know it picks none at all -- Control Center, for one, then lays its modules out
	 * without their sizes. (boot-arg machine_name=0: the plain "arm64" this used to be.)
	 */
	PE_parse_boot_argn("machine_name", &real, sizeof(real));
	if (!real || !IODTPlatformExpert::getMachineName(name, maxLength) || name[0] == 0) {
		strlcpy(name, "arm64", maxLength);
	}
	return true;
}
