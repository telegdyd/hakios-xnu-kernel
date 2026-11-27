/*
 * Which board this is (board/board.h): the row of the generated table whose name the loader put in
 * /chosen/darwin-board. Also as sysctls, for user space and for a person at a shell:
 *
 *   hw.board.name          the package's name; "unknown" when the loader named none or the table has not got it
 *   hw.board.soc           "sm7350"
 *   hw.board.description   "Maker Model"
 *   hw.board.features      "cpufreq-epss ddr-rpmh ..."
 *
 * The table is made when the kernel is built (tools/boards.py gen, by kernel/build.sh), with a row for
 * every device package that was there: a kernel knows the boards it was built with.
 */
#include <libkern/libkern.h>
#include <machine/machine_routines.h>
#include <pexpert/device_tree.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include "kernel.h"
#include "board.h"
#include "board_table.h"
#include "log.h"

static const struct hakios_board *found;
static bool looked;

/*
 * Is this the board the loader named (/chosen/darwin-board, a string with its end)? The name is the
 * loader's word and nothing else is: a board is never inferred from hw.machine, which is an iPhone's
 * for Darwin's sake, and a machine whose loader named none is no board at all -- which is not permission
 * to touch anything of a phone.
 */
static bool
loader_named(const char *name)
{
	DTEntry chosen;
	const void *value;
	unsigned int size;
	size_t length = strlen(name) + 1;

	return SecureDTLookupEntry(NULL, "/chosen", &chosen) == kSuccess &&
	       SecureDTGetProperty(chosen, "darwin-board", &value, &size) == kSuccess &&
	       size == length && memcmp(value, name, length) == 0;
}

const struct hakios_board *
hakios_board(void)
{
	if (!looked) {
		/* (twice at once finds the same row twice: nothing to guard) */
		for (unsigned i = 0; i < hakios_board_count; i++) {
			if (loader_named(hakios_boards[i].name)) {
				found = &hakios_boards[i];
				break;
			}
		}
		looked = true;
	}
	return found;
}

bool
hakios_board_has(uint64_t feature)
{
	const struct hakios_board *board = hakios_board();

	return board != NULL && (board->features & feature) != 0;
}

/*
 * For the display driver (iokit/MiniIOMobileFramebuffer.cpp, fb_flip), which does not see this file's table: where the
 * display subsystem and the panel's interface are, and how much of the firmware's display memory there is from the
 * panel's on. False on a board without the feature display-flip.
 */
bool
hakios_display_flip_data(uint64_t *mdss, uint64_t *intf, uint64_t *splash_size)
{
	const struct hakios_board *board = hakios_board();

	if (board == NULL || !(board->features & HAKIOS_HAS_DISPLAY_FLIP) || board->display.mdss == 0 ||
	    board->display.intf == 0 || board->display.splash_size == 0) {
		return false;
	}
	*mdss = board->display.mdss;
	*intf = board->display.intf;
	*splash_size = board->display.splash_size;
	return true;
}

/*
 * The cores' idle states for arch/arm64/kernel/psci_smp.c (feature cpu-idle): a core's power-down parameter and least
 * residency on the small and the other cores; the cluster's l3 and llcc states, parameter and least residency each (0:
 * not there); the processors switched off while the display is off; the SoC's cores. False without the feature.
 */
bool
hakios_cpu_idle_data(uint32_t *core, uint32_t core_us[2], uint32_t l3[2], uint32_t llcc[2], uint64_t *screen_off_cores,
    uint32_t *soc_cores)
{
	const struct hakios_board *board = hakios_board();

	if (board == NULL || !(board->features & HAKIOS_HAS_CPU_IDLE) || board->cpu_idle.core == 0) {
		return false;
	}
	*core = board->cpu_idle.core;
	for (int i = 0; i < 2; i++) {
		core_us[i] = board->cpu_idle.core_us[i];
		l3[i] = board->cpu_idle.l3[i];
		llcc[i] = board->cpu_idle.llcc[i];
	}
	*screen_off_cores = board->cpu_idle.screen_off_cores;
	*soc_cores = board->cpu_idle.soc_cores;
	return true;
}

unsigned
hakios_board_cpu_groups(void)
{
	const struct hakios_board *board = hakios_board();
	unsigned groups = 0;

	while (board != NULL && groups < HAKIOS_CPU_GROUPS && board->cpufreq.domain_cores[groups] != 0) {
		groups++;
	}
	return groups;
}

int
hakios_board_cpu_group_of(unsigned core)
{
	const struct hakios_board *board = hakios_board();
	unsigned first = 0;

	for (unsigned group = 0; board != NULL && group < HAKIOS_CPU_GROUPS && board->cpufreq.domain_cores[group] != 0; group++) {
		if (core < first + board->cpufreq.domain_cores[group]) {
			return (int)group;
		}
		first += board->cpufreq.domain_cores[group];
	}
	return -1;
}

/* "cpufreq-epss ddr-rpmh", or "" */
static void
features_text(const struct hakios_board *board, char *text, size_t size)
{
	size_t at = 0;

	text[0] = 0;
	for (unsigned bit = 0; board != NULL && bit < hakios_feature_count; bit++) {
		if (board->features & (1ull << bit)) {
			at += (size_t)snprintf(text + at, size - at, "%s%s", at ? " " : "", hakios_feature_names[bit]);
			if (at >= size) {
				break;
			}
		}
	}
}

void
hakios_board_init(void)
{
	const struct hakios_board *board = hakios_board();
	char features[HAKIOS_FEATURES_TEXT], line[160 + HAKIOS_FEATURES_TEXT];   /* (a board with every feature fits) */

	features_text(board, features, sizeof(features));
	if (board != NULL) {
		/* (the SoC only when it is named apart from the board: a package's soc.dtsi takes the board's name) */
		bool soc_of_its_own = strcmp(board->soc, board->name) != 0;

		snprintf(line, sizeof(line), "board: %s (%s%s%s): %s\n", board->name, board->description,
		    soc_of_its_own ? ", " : "", soc_of_its_own ? board->soc : "", features[0] ? features : "no features listed");
	} else {
		snprintf(line, sizeof(line), "board: not one of the %u this kernel knows (/chosen/darwin-board): no device "
		    "of a phone is touched\n", hakios_board_count);
	}
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	if (ml_get_interrupts_enabled()) {
		printf("%s", line);
	}
}

SYSCTL_NODE(_hw, OID_AUTO, board, CTLFLAG_RD | CTLFLAG_LOCKED, 0, "the board this kernel runs on");

static int
sysctl_board SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1)
	const struct hakios_board *board = hakios_board();
	char features[HAKIOS_FEATURES_TEXT];
	const char *text;

	switch (arg2) {
	case 0: text = board ? board->name : "unknown"; break;
	case 1: text = board ? board->soc : "unknown"; break;
	case 2: text = board ? board->description : "unknown"; break;
	default:
		features_text(board, features, sizeof(features));
		text = features;
		break;
	}
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}

SYSCTL_PROC(_hw_board, OID_AUTO, name, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0, sysctl_board, "A", "");
SYSCTL_PROC(_hw_board, OID_AUTO, soc, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 1, sysctl_board, "A", "");
SYSCTL_PROC(_hw_board, OID_AUTO, description, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 2, sysctl_board, "A", "");
SYSCTL_PROC(_hw_board, OID_AUTO, features, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 3, sysctl_board, "A", "");

/*
 * kern.hakios.loader_request: 1 = the run ends, back to the loader, which stays; 2 = back to the loader, which resets
 * the phone (Android) once it has the log (or fastboot when the board has reboot { } and reboot_fastboot != 0);
 * 3 = reboot directly into the boot loader's fastboot (board/reboot.c). Root only (no CTLFLAG_ANYBODY); written by
 * sweetd for "loader" and "reboot". Reads as 0.
 */
extern void loader_request_return(int mode) __attribute__((weak));     /* osfmk/arm64/loader_return.c */
extern int hakios_fastboot_prepare(void) __attribute__((weak));        /* bsd/dev/board/reboot.c */
extern void hakios_fastboot_reset(void) __attribute__((weak));         /* bsd/dev/board/reboot.c */

static int
sysctl_loader_request SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	const struct hakios_board *board = hakios_board();
	int mode = 0, changed = 0, error;
	uint32_t fastboot = 1;

	error = sysctl_io_number(req, 0, sizeof(mode), &mode, &changed);
	if (error || !changed) {
		return error;
	}
	if (mode < 1 || mode > 3) {
		return EINVAL;
	}
	/* boot-arg reboot_fastboot=0: kern.hakios.loader_request 2 resets through the loader instead of fastboot (default 1 on boards with reboot { }) */
	PE_parse_boot_argn("reboot_fastboot", &fastboot, sizeof(fastboot));
	if (mode == 2 && board != NULL && board->reboot.sdam != 0 && fastboot != 0) {
		mode = 3;
	}
	if (mode == 3) {
		if (hakios_fastboot_prepare == NULL) {
			return ENOTSUP;
		}
		error = hakios_fastboot_prepare();
		if (error != 0) {
			return error;
		}
		if (loader_request_return != NULL) {
			hakios_say("board: loader_request 3: reboot into fastboot\n");
			loader_request_return(3);
			return 0;
		}
		if (hakios_fastboot_reset != NULL) {
			hakios_fastboot_reset();
			return EIO;
		}
		return ENOTSUP;
	}
	if (loader_request_return == NULL) {
		return ENOTSUP;         /* (a kernel without the loader's return path) */
	}
	hakios_say("board: loader_request %d: %s\n", mode, mode == 2 ? "back to the loader, then a reset" : "back to the loader");
	loader_request_return(mode);
	return 0;
}
SYSCTL_DECL(_kern_hakios);
SYSCTL_PROC(_kern_hakios, OID_AUTO, loader_request, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    sysctl_loader_request, "I", "1: back to the loader, 2: reset (fastboot if board has reboot { }, else Android), 3: fastboot");
