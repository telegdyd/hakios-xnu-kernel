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
#include <kern/locks.h>
#include <kern/startup.h>
#include <libkern/libkern.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include "hakios.drivers.h"

/* kern.hakios: the node every driver of this port hangs its sysctls under (it was hakios.battery.c's) */
SYSCTL_NODE(_kern, OID_AUTO, hakios, CTLFLAG_RW | CTLFLAG_LOCKED, 0, "redwood phone bits");

static LCK_GRP_DECLARE(hakios_platform_lock_group, "hakios-platform");

/*
 * kern.hakios.loader_request: 1 = hand the phone back to the loader and stay there, 2 = hand it back and let the
 * loader reset it (Android again). Root only; the return happens within a second (osfmk/arm64/loader_return.c).
 * Written by sweetd for "sweet-sh.py loader" and "sweet-sh.py reboot". Reads as 0.
 */
extern void loader_request_return(int mode) __attribute__((weak));

static int
hakios_loader_request_sysctl(struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2,
    struct sysctl_req *req)
{
	int mode = 0, error;

	error = sysctl_handle_int(oidp, &mode, 0, req);
	if (error || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if ((mode != 1 && mode != 2) || loader_request_return == NULL) {
		return EINVAL;
	}
	loader_request_return(mode);
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, loader_request, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    0, 0, hakios_loader_request_sysctl, "I", "1: back to the loader, 2: back to the loader and reset (Android)");

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
