/*
 * Reboot into the boot loader's fastboot directly from Darwin, without returning to XNULOADER:
 *
 *   1. hakios_fastboot_prepare(): map the PMIC arbiter and IMEM once, verify EE ownership of the
 *      board's SDAM peripheral (board: reboot { }), write the bootloader reason into bits 7:1 while
 *      preserving bit 0, read it back to confirm, and write the optional IMEM restart_reason cookie.
 *      Doing this synchronously in sysctl kern.hakios.loader_request guarantees that any fault or
 *      failure either aborts cleanly (returning an errno with no reset) or still lands in fastboot.
 *   2. hakios_fastboot_reset(): quiesce touch (S3908) and PMIC button interrupts, stop the audio DSP,
 *      wait briefly so the live USB log transmits, disable interrupts, and invoke PSCI SYSTEM_RESET.
 *
 * Unlike xnu-8796's 8 MB io_submap (REBOOT-FAULT.md), xnu-12377 allocates ml_io_map() from kernel_map
 * directly; mapping once and writing the reason first is still preserved as good hygiene.
 */
#include <kern/clock.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <sys/board/input.h>
#include <sys/errno.h>
#include <sys/systm.h>

#include "kernel.h"
#include "board.h"
#include "log.h"

HAKIOS_DEFINE_SAY(ml_get_interrupts_enabled())

#define HAKIOS_MMIO_SYNC
#include "mmio.h"
#include "spmi.h"

#define PSCI_SYSTEM_RESET       0x84000009u
#define SPMI_RETRIES            3

extern void hakios_s3908_quiesce(void) __attribute__((weak));
extern void hakios_buttons_quiesce(void) __attribute__((weak));
extern void hakios_adsp_stop(void) __attribute__((weak));
extern int64_t psci_smp_call(uint64_t function, uint64_t arg1, uint64_t arg2, uint64_t arg3) __attribute__((weak));

static bool mapped;
static struct hakios_spmi arb;
static bool sdam_found;
static uint32_t sdam_channel, sdam_owner;
static vm_offset_t sdam_write, imem_va;

static void
fastboot_map(const struct hakios_board *board)
{
	if (mapped) {
		return;
	}
	mapped = true;
	if (!hakios_spmi_map(&arb)) {
		return;
	}
	for (uint32_t n = 0; n < arb.channels; n++) {
		uint32_t map = hakios_spmi_channel_map(&arb, n);

		if (map == 0 || SPMI_ID_OF(map) != board->reboot.sdam) {
			continue;
		}
		uint32_t owner = hakios_spmi_channel_owner(&arb, n);

		if (!sdam_found || owner == arb.system) {
			sdam_found = true;
			sdam_channel = n;
			sdam_owner = owner;
		}
	}
	if (sdam_found && sdam_owner == arb.system) {
		sdam_write = hakios_spmi_map_write(&arb, sdam_channel);
	}
	if (board->reboot.imem != 0) {
		imem_va = ml_io_map((vm_offset_t)(board->reboot.imem & ~0xfffu), 0x1000);
	}
}

static const char *
sdam_read_byte(uint8_t reg, uint8_t *out)
{
	const char *problem = NULL;

	for (unsigned attempt = 0; attempt < SPMI_RETRIES; attempt++) {
		hakios_spmi_lock();
		problem = hakios_spmi_read(hakios_spmi_observer(&arb, sdam_channel), reg, out, 1);
		hakios_spmi_unlock();
		if (problem == NULL || strcmp(problem, SPMI_SAYS_DROPPED) != 0) {
			break;
		}
	}
	return problem;
}

static const char *
sdam_write_byte(uint8_t reg, uint8_t val)
{
	const char *problem = NULL;

	for (unsigned attempt = 0; attempt < SPMI_RETRIES; attempt++) {
		hakios_spmi_lock();
		problem = hakios_spmi_write(sdam_write, reg, val);
		hakios_spmi_unlock();
		if (problem == NULL || strcmp(problem, SPMI_SAYS_DROPPED) != 0) {
			break;
		}
	}
	return problem;
}

int hakios_fastboot_prepare(void);
void hakios_fastboot_reset(void);

int
hakios_fastboot_prepare(void)
{
	const struct hakios_board *board = hakios_board();
	const char *problem;
	uint8_t before = 0, after = 0, value;

	if (board == NULL || board->reboot.sdam == 0) {
		return ENOTSUP;
	}
	fastboot_map(board);
	if (!sdam_found) {
		say("reboot: fastboot: SDAM %#x has no channel of the arbiter: no reset\n", board->reboot.sdam);
		return ENODEV;
	}
	if (sdam_owner != arb.system) {
		say("reboot: fastboot: SDAM %#x (channel %u) belongs to system %u, not %u: no reset\n",
		    board->reboot.sdam, sdam_channel, sdam_owner, arb.system);
		return EPERM;
	}
	if (sdam_write == 0) {
		say("reboot: fastboot: SDAM %#x write channel could not be mapped: no reset\n", board->reboot.sdam);
		return ENODEV;
	}
	problem = sdam_read_byte(board->reboot.offset, &before);
	if (problem != NULL) {
		say("reboot: fastboot: SDAM %#x offset %#x read failed (%s): no reset\n",
		    board->reboot.sdam, board->reboot.offset, problem);
		return EIO;
	}
	value = (uint8_t)((before & 1u) | ((uint32_t)(board->reboot.bootloader & 0x7fu) << 1));
	problem = sdam_write_byte(board->reboot.offset, value);
	if (problem == NULL) {
		problem = sdam_read_byte(board->reboot.offset, &after);
	}
	if (problem != NULL || after != value) {
		say("reboot: fastboot: reason byte was %#x, written %#x, reads %#x (%s): no reset\n",
		    before, value, after, problem != NULL ? problem : "mismatch");
		return EIO;
	}
	if (imem_va != 0) {
		wr(imem_va + (board->reboot.imem & 0xfffu), board->reboot.imem_bootloader);
	}
	say("reboot: fastboot: reboot reason %#x -> %#x (bootloader, SDAM %#x ch %u), IMEM cookie %s\n",
	    before, after, board->reboot.sdam, sdam_channel, imem_va != 0 ? "set" : "NOT set");
	return 0;
}

void
hakios_fastboot_reset(void)
{
	if (hakios_s3908_quiesce != NULL) {
		hakios_s3908_quiesce();
	}
	if (hakios_buttons_quiesce != NULL) {
		hakios_buttons_quiesce();
	}
	if (hakios_adsp_stop != NULL) {
		hakios_adsp_stop();
	}
	say("reboot: fastboot: touch/buttons quiesced, ADSP stopped; PSCI SYSTEM_RESET now\n");
#ifndef HAKIOS_MMIO_TEST
	delay_for_interval(200, NSEC_PER_MSEC);
#endif
	(void)ml_set_interrupts_enabled(FALSE);
	if (psci_smp_call != NULL) {
		int64_t rc = psci_smp_call(PSCI_SYSTEM_RESET, 0, 0, 0);

		(void)ml_set_interrupts_enabled(TRUE);
		say("reboot: fastboot: PSCI SYSTEM_RESET returned %lld\n", (long long)rc);
		return;
	}
#ifdef __aarch64__
	{
		register uint64_t x0 __asm__("x0") = PSCI_SYSTEM_RESET;

		__asm__ volatile ("smc #0" : "+r" (x0) : : "x1", "x2", "x3", "memory");
		(void)ml_set_interrupts_enabled(TRUE);
		say("reboot: fastboot: PSCI SYSTEM_RESET returned %#llx\n", (unsigned long long)x0);
	}
#endif
}
