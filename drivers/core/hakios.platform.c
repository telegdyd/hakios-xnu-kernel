/*
 * What the drivers share, kept apart from any one of them, so that each can be left out of the kernel on its own
 * (drivers/<subsystem>/conf, CONFIG_HAKIOS_*: docs/drivers-config.md).
 *
 *   the locks      of the hardware several drivers take turns at: the PMIC arbiter (the buttons, the flashlight, the
 *                  camera's regulators, the PMIC survey), the always-on processor's mailbox, the shared memory's
 *                  allocation and RPMh's sequencer (the Wi-Fi, audio DSP and modem bring-ups, the touch screen's
 *                  supply), and the SMMU's free match registers (Wi-Fi, sound, camera, modem). They were in
 *                  hakios.buttons.c and hakios.wifi.c, which the others then needed in the kernel to link.
 *   kern.hakios    the sysctl node the drivers hang theirs under (it was hakios.battery.c's)
 *   the list       of the drivers this kernel was built with: sysctl kern.hakios.drivers, and a line in the boot log,
 *                  so that a run says what was in it (hakios.drivers.h is made by tools/hakios-config.py kernel).
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/startup.h>
#include <libkern/libkern.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

void hakios_spmi_lock(void);
void hakios_spmi_unlock(void);

#include "hakios.drivers.h"

/* kern.hakios: the node every driver of this port hangs its sysctls under (it was hakios.battery.c's) */
SYSCTL_NODE(_kern, OID_AUTO, hakios, CTLFLAG_RW | CTLFLAG_LOCKED, 0, "redwood phone bits");

static LCK_GRP_DECLARE(hakios_platform_lock_group, "hakios-platform");

/*
 * kern.hakios.loader_request: 1 = hand the phone back to the loader and stay there, 2 = hand it back and let the
 * loader reset it (Android again), 3 = reset into the boot loader's fastboot (below). Root only; the return happens
 * within a second (osfmk/arm64/loader_return.c). Written by sweetd for "sweet-sh.py loader" and "sweet-sh.py reboot";
 * with boot-arg reboot_fastboot=8450, "reboot" (2) is 3. Reads as 0.
 */
extern void loader_request_return(int mode) __attribute__((weak));
extern void hakios_adsp_stop(void) __attribute__((weak));
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/*
 * Reset into fastboot, as Android's "reboot bootloader" does on the OnePlus 10 Pro (SM8450; boot-arg
 * reboot_fastboot=8450): the reboot reason 2 ("bootloader"; 1 is "recovery") into the PMK8350's SDAM 0x7100 at 0x48, bits 7:1 (its
 * device tree: reboot_reason -> sdam@7100 restart@48, bits <1 7>; Linux qcom-reboot-mode / nvmem-reboot-mode), the
 * cookie 0x77665500 into IMEM's restart_reason (0x146aa000 + 0x65c, msm-poweroff), then PSCI SYSTEM_RESET. The boot
 * loader (ABL) reads the reason after the reset and stays in fastboot (serial 6684e565): a test can then RAM-boot a
 * loader without anybody at the keys. The SDAM byte is written through the arbiter's channel for that peripheral,
 * only when this system (EE 0) owns it.
 */
#define FB_ARB_CORE             0x0c400000UL    /* qcom,spmi@c42d000: core, cnfg, chnls */
#define FB_ARB_CNFG             0x0c42d000UL
#define FB_ARB_CHNLS            0x0c500000UL
#define FB_ARB_TABLE            0x2000          /* a word per channel: [19:8] slave and peripheral */
#define FB_SDAM_PPID            0x071           /* slave 0 (PMK8350), peripheral 0x71 */
#define FB_SDAM_REASON          0x48
#define FB_REASON_BOOTLOADER    0x02            /* Linux pmk8350.dtsi reboot-mode: mode-recovery 1, mode-bootloader 2 */
#define FB_IMEM_RESTART         0x146aa65cUL
#define FB_IMEM_BOOTLOADER      0x77665500u
#define FB_PSCI_SYSTEM_RESET    0x84000009ull

static uint32_t reboot_fastboot;
extern void loader_log_putc(char c) __attribute__((weak));

static void fb_say(const char *format, ...) __printflike(1, 2);
static void
fb_say(const char *format, ...)
{
	char line[200];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	printf("%s", line);
	for (const char *c = line; *c != 0 && loader_log_putc != NULL; c++) {
		loader_log_putc(*c);
	}
}

static inline uint32_t
fb_rd(vm_offset_t a)
{
	return *(volatile uint32_t *)a;
}

static inline void
fb_wr(vm_offset_t a, uint32_t v)
{
	*(volatile uint32_t *)a = v;
	__asm__ volatile ("dsb sy" ::: "memory");
}

/* One SPMI command on a channel (read: 1 << 27, write: 0), its status waited for: true when done without an error. */
static bool
fb_spmi(vm_offset_t channel, uint32_t command, uint32_t *data, bool write)
{
	uint32_t status = 0;

	hakios_spmi_lock();
	if (write) {
		fb_wr(channel + 0x10, *data);
	}
	fb_wr(channel + 0x00, command);
	for (unsigned us = 0; us < 2000; us++) {
		status = fb_rd(channel + 0x08);
		if (status != 0) {
			break;
		}
		delay_for_interval(1, NSEC_PER_USEC);
	}
	if (!write && status == 1) {
		*data = fb_rd(channel + 0x18);
	}
	hakios_spmi_unlock();
	return status == 1;
}

static int
fastboot_reset(void)
{
	vm_offset_t core = ml_io_map(FB_ARB_CORE, 0x3000), cnfg = ml_io_map(FB_ARB_CNFG, 0x4000), channel, imem;
	uint32_t apid = 1024, owner, before = 0, after = 0, value;

	for (uint32_t n = 0; n < 1024 && core != 0; n++) {
		if (((fb_rd(core + FB_ARB_TABLE + 4 * n) >> 8) & 0xfff) == FB_SDAM_PPID) {
			apid = n;
			break;
		}
	}
	if (apid == 1024 || cnfg == 0) {
		fb_say("hakios: fastboot: the SDAM 0x7100 has no channel of the arbiter: no reset\n");
		return ENODEV;
	}
	owner = fb_rd(cnfg + 4 * apid) & 7;
	if (owner != 0) {
		fb_say("hakios: fastboot: the SDAM 0x7100 (channel %u) belongs to system %u, not ours: no reset\n", apid, owner);
		return EPERM;
	}
	channel = ml_io_map(FB_ARB_CHNLS + 0x1000 * (vm_offset_t)apid, 0x1000);
	if (channel == 0 || !fb_spmi(channel, 1u << 27 | FB_SDAM_REASON << 4, &before, false)) {
		fb_say("hakios: fastboot: the reason byte cannot be read (channel %u): no reset\n", apid);
		return EIO;
	}
	value = (before & 1) | FB_REASON_BOOTLOADER << 1;
	if (!fb_spmi(channel, FB_SDAM_REASON << 4, &value, true) ||
	    !fb_spmi(channel, 1u << 27 | FB_SDAM_REASON << 4, &after, false) || (after & 0xff) != value) {
		fb_say("hakios: fastboot: the reason byte was %#x, written %#x, reads %#x: no reset\n", before & 0xff, value,
		    after & 0xff);
		return EIO;
	}
	imem = ml_io_map(FB_IMEM_RESTART & ~0xfffUL, 0x1000);
	if (imem != 0) {
		fb_wr(imem + (FB_IMEM_RESTART & 0xfff), FB_IMEM_BOOTLOADER);
	}
	fb_say("hakios: fastboot: reboot reason %#x -> %#x (bootloader), IMEM cookie %s; the audio DSP stopped; PSCI reset\n",
	    before & 0xff, after & 0xff, imem != 0 ? "set" : "NOT set");
	if (hakios_adsp_stop != NULL) {
		hakios_adsp_stop();     /* (for the next kernel to find it stopped, not left running) */
	}
	delay_for_interval(200, NSEC_PER_MSEC);  /* (the line out to the USB log) */
	(void)ml_set_interrupts_enabled(FALSE);
	{
		register uint64_t x0 __asm__("x0") = FB_PSCI_SYSTEM_RESET;

		__asm__ volatile ("smc #0" : "+r" (x0) : : "x1", "x2", "x3", "memory");
	}
	return EIO;                     /* (not reached) */
}

static int
hakios_loader_request_sysctl(struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2,
    struct sysctl_req *req)
{
	int mode = 0, error;

	error = sysctl_handle_int(oidp, &mode, 0, req);
	if (error || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (reboot_fastboot == 0) {
		PE_parse_boot_argn("reboot_fastboot", &reboot_fastboot, sizeof(reboot_fastboot));
		reboot_fastboot = reboot_fastboot == 8450 ? 8450 : 1;
	}
	if (mode == 2 && reboot_fastboot == 8450) {
		mode = 3;
	}
	if (mode == 3) {
		return reboot_fastboot == 8450 ? fastboot_reset() : EINVAL;
	}
	if ((mode != 1 && mode != 2) || loader_request_return == NULL) {
		return EINVAL;
	}
	loader_request_return(mode);
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, loader_request, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    0, 0, hakios_loader_request_sysctl, "I", "1: back to the loader, 2: back to the loader and reset (Android), 3: fastboot");

/*
 * One transfer at a time through the PMIC arbiter for this system, whoever asks: with two at once the arbiter drops
 * one ("the transfer was dropped"), as Linux never lets happen (one lock for the bus). A spin lock: it is taken
 * around a transfer of some tens of microseconds.
 */
static LCK_SPIN_DECLARE(spmi_lock, &hakios_platform_lock_group);

void
hakios_spmi_lock(void)
{
	lck_spin_lock(&spmi_lock);
}

void
hakios_spmi_unlock(void)
{
	lck_spin_unlock(&spmi_lock);
}

/*
 * What the DSPs' starts use while another may be at work beside them: the mailbox to the always-on processor, one
 * message at a time, and the allocation of items in the shared memory (whose hardware lock both take under the same
 * number, so it does not keep the two apart); RPMh's sequencer (TCS 0 of the RSC); a free match register and context
 * of the SMMU, found and taken.
 */
LCK_MTX_DECLARE(hakios_aop_mailbox_lock, &hakios_platform_lock_group);
LCK_MTX_DECLARE(hakios_smem_alloc_lock, &hakios_platform_lock_group);
LCK_MTX_DECLARE(hakios_rsc_lock, &hakios_platform_lock_group);
LCK_MTX_DECLARE(hakios_smmu_lock, &hakios_platform_lock_group);

static char hakios_drivers_string[] = HAKIOS_DRIVERS_LIST;

SYSCTL_STRING(_kern_hakios, OID_AUTO, drivers, CTLFLAG_RD | CTLFLAG_LOCKED, hakios_drivers_string, 0,
    "the drivers this kernel was built with (CONFIG_HAKIOS_* of its configuration, lowercase)");

static void
hakios_platform_init(void)
{
	printf("hakios-kernel: %d drivers built in (%s): %s\n", HAKIOS_DRIVERS_COUNT, HAKIOS_DRIVERS_CONFIG, hakios_drivers_string);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, hakios_platform_init);
