/*
 * The panel's light, for any device: one driver; what differs between devices is the board's data
 * (board/board.h, from arch/arm64/boot/dts/NAME/board.dts), and the arithmetic is board/backlight_core.h
 * (tested on the host: workbench/backlight_test.c, and the driver itself: workbench/backlight_driver_test.c).
 *
 * What Darwin sees: the sysctls hw.backlight.* (any user may write the first four; the injected library
 * components/brightness/inject_brightness.c is who does, for the slider of Control Center and Settings and
 * for the switch of automatic brightness):
 *
 *   level      0 to 1000: the slider (Darwin's DisplayBrightness times 1000). Written while the brightness
 *              is automatic, it moves the automatic curve by the difference, as on an iPhone
 *   auto       0 or 1: automatic brightness (Darwin's DisplayBrightnessAuto)
 *   power      the display: 1 on, 0 off (the panel dark), 2 dimmed (at most level 150); written by
 *              components/brightness/inject_backlight.c from Darwin's display modes (the side button, the idle timer)
 *   lux        the ambient light in thousandths of a lux. Written: a reading of the "virtual" sensor
 *              (QEMU, tests, or user space with a sensor of its own); read: the light in use
 *   cap        the most it may show while the phone is hot (bsd/dev/board/thermal.c), 1000 otherwise
 *   shown      what is on the panel now, 0 to 1000 (it moves towards the level, see bl_step())
 *   raw        the last number the panel was given (backend dcs)
 *   offset     the user's move of the automatic curve
 *   backend    "dcs", "soft" or "none"; als: where the light readings come from
 *
 * A driver of a real light sensor reports with hakios_als_report(millilux, name).
 *
 * The backends:
 *
 *   dcs    MIPI DCS set_display_brightness (0x51) to a DSI panel, through a Qualcomm DSI controller
 *          (2.2 or later: SM7325, SM7350, SM8x50 ...) in its command FIFO -- the controller's "test
 *          pattern" store, which Linux's display driver uses where it may not use memory. The
 *          controller is the one the firmware set the panel going with; nothing of its setup is
 *          changed, only a command added. Every read and write is behind the same checks of clocks as
 *          the display scan (bsd/dev/board/adreno.c): the display block's power switch and the bus
 *          clock of its registers. A command that could not be sent is tried again a moment later;
 *          three in a row the controller does not report sent within 20 ms stop this backend for the
 *          run (but for one last try at the run's end, to leave the panel lit for the loader).
 *   soft   no panel to tell: the display driver (MiniIOMobileFramebuffer.cpp) darkens every pixel on
 *          its way to the panel, from a copy in memory, so a new level is shown at once even when no
 *          new frame comes. Works anywhere (QEMU); costs a copy of each frame.
 *   none   nothing (the default where nothing is known and backlight=0).
 *
 * Boot-args (each overrides the profile of the board, so another device can be tried without a new
 * kernel):
 *
 *   backlight=0        off (default): nothing touched, no sysctls answer anything but "none"
 *   backlight=1        the profile's backend, read only: the DSI controller's version and state logged
 *   backlight=2        the profile's backend in use
 *   bl_backend=dcs|soft|none
 *   bl_dsi=ADDRESS     the DSI controller's registers (physical)
 *   bl_min= bl_max=    the panel's numbers for the lowest and the highest brightness
 *   bl_bytes=1|2       how many bytes the panel's 0x51 takes; bl_le=1: least significant first
 *   bl_level=N         the level at boot (default 600)
 *   bl_auto=1          automatic brightness from the start
 *   bl_floor=N         backend soft: what is kept of a pixel at level 0, of 256 (default 24)
 *
 * Adding a device: backlight { } in its arch/arm64/boot/dts/NAME/board.dts -- the panel's
 * qcom,mdss-dsi-bl-min-level / -max-level, qcom,mdss-dsi-bl-inverted-dbv (MSB first), the panel's mode --
 * and in its SoC's file the DSI controller (mdss_dsi0 "dsi_ctrl") and the display clock controller's
 * offsets (Linux dispcc-*.c). `tools/boards.py new --dts` reads most of it from the device tree. Until
 * then the boot-args above do the same, and a board not described at all gets backend soft.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <sys/kauth.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include "kernel.h"
#include "backlight_core.h"
#include "board.h"
#include "log.h"

#define say hakios_say

/* ---- what differs between devices ----------------------------------------------------------------- */

enum bl_backend { BL_NONE, BL_DCS, BL_SOFT };
static const char *const bl_backend_names[] = { "none", "dcs", "soft" };

/*
 * What is used: the board's values (board/board.h: arch/arm64/boot/dts/NAME/board.dts backlight { }, and its SoC's
 * clocks { } and display { }) with the boot-args over them.
 */
struct bl_profile {
	const char *board;              /* /chosen darwin-board */
	enum bl_backend backend;
	/* backend dcs */
	uint32_t dsi;                   /* the DSI controller's registers (dt: mdss_dsi0 "dsi_ctrl") */
	uint32_t raw_min, raw_max;      /* the panel's numbers: qcom,mdss-dsi-bl-min-level, -max-level (without HBM) */
	uint32_t raw_restore;           /* what the panel is left at for the loader (qcom,mdss-brightness-init-level) */
	uint8_t bytes;                  /* 0x51 takes 1 or 2 bytes */
	bool little_endian;             /* (2 bytes) least significant first: no qcom,mdss-dsi-bl-inverted-dbv */
	bool video_mode;                /* a video-mode panel: commands go between frames */
	/* the clocks the DSI controller's registers need (the same chain as the display scan) */
	uint32_t gcc, gcc_disp_ahb_cbcr;
	uint32_t dispcc, dispcc_core_gdscr, dispcc_ahb_cbcr;
	/* display-power: panel sleep/wake and SM8450 MDSS/DSI hardware power-down */
	uint32_t mdss, intf, vbif;
	uint64_t splash_base;
	uint32_t splash_size;
	bool has_display_power;
};

/* the profile in use: the board's, with the boot-args over it */
static struct bl_profile profile;
static uint32_t mode;                   /* backlight=: 0 off, 1 look only, 2 in use */
static bool chosen;
/* (the display driver asks, from its own thread, while the drivers are started: the second to ask waits for the first's answer) */
static LCK_GRP_DECLARE(bl_lock_group, "backlight");
static LCK_MTX_DECLARE(bl_choose_lock, &bl_lock_group);

static void
bl_choose(void)
{
	const struct hakios_board *board = hakios_board();
	char backend[8];
	uint32_t value;

	lck_mtx_lock(&bl_choose_lock);
	if (chosen) {
		lck_mtx_unlock(&bl_choose_lock);
		return;
	}
	bzero(&profile, sizeof(profile));
	if (board == NULL) {
		profile.backend = BL_SOFT;      /* a board the table has not got: only what is safe anywhere */
		profile.board = "unknown";
	} else {
		profile.board = board->name;
		profile.backend = board->backlight.type == HAKIOS_BACKLIGHT_DCS ? BL_DCS :
		    board->backlight.type == HAKIOS_BACKLIGHT_SOFT ? BL_SOFT : BL_NONE;
		profile.dsi = board->backlight.dsi;
		profile.raw_min = board->backlight.raw_min;
		profile.raw_max = board->backlight.raw_max;
		profile.raw_restore = board->backlight.raw_restore;
		profile.bytes = board->backlight.bytes;
		profile.little_endian = board->backlight.lsb_first != 0;
		profile.video_mode = board->backlight.video_mode != 0;
		profile.gcc = board->clocks.gcc;
		profile.gcc_disp_ahb_cbcr = board->clocks.gcc_disp_ahb_cbcr;
		profile.dispcc = board->clocks.dispcc;
		profile.dispcc_core_gdscr = board->clocks.dispcc_mdss_gdscr;
		profile.dispcc_ahb_cbcr = board->clocks.dispcc_mdss_ahb_cbcr;
		profile.mdss = board->display.mdss;
		profile.intf = board->display.intf;
		profile.vbif = board->display.vbif;
		profile.splash_base = board->display.splash_base;
		profile.splash_size = board->display.splash_size;
		profile.has_display_power = hakios_board_has(HAKIOS_HAS_DISPLAY_POWER);
	}
	if (!PE_parse_boot_argn("backlight", &mode, sizeof(mode))) {
		mode = 0;
	}
	mode = mode > 2 ? 2 : mode;
	if (PE_parse_boot_argn("bl_backend", backend, sizeof(backend))) {
		profile.backend = strcmp(backend, "dcs") == 0 ? BL_DCS : strcmp(backend, "soft") == 0 ? BL_SOFT : BL_NONE;
	}
	if (PE_parse_boot_argn("bl_dsi", &value, sizeof(value))) {
		profile.dsi = value;
	}
	if (PE_parse_boot_argn("bl_min", &value, sizeof(value))) {
		profile.raw_min = value;
	}
	if (PE_parse_boot_argn("bl_max", &value, sizeof(value))) {
		profile.raw_max = value;
	}
	if (PE_parse_boot_argn("bl_bytes", &value, sizeof(value))) {
		profile.bytes = (uint8_t)value;
	}
	if (PE_parse_boot_argn("bl_le", &value, sizeof(value))) {
		profile.little_endian = value != 0;
	}
	if (mode == 0) {
		profile.backend = BL_NONE;
	}
	/*
	 * A lowest number of 0 would make the slider's lowest a black panel (Darwin's lowest brightness is dim,
	 * not dark: dark is the display off); one byte holds no more than 255.
	 */
	if (profile.backend == BL_DCS && (profile.dsi == 0 || profile.dispcc == 0 || profile.gcc == 0 ||
	    profile.raw_min == 0 || profile.raw_max <= profile.raw_min || (profile.bytes != 1 && profile.bytes != 2) ||
	    (profile.bytes == 1 && profile.raw_max > 0xff))) {
		say("backlight: %s: no complete DCS profile (DSI %#x, %u to %u, %u bytes): nothing done\n", profile.board,
		    profile.dsi, profile.raw_min, profile.raw_max, profile.bytes);
		profile.backend = BL_NONE;
	}
	chosen = true;                  /* (last: whoever finds it set finds the profile filled) */
	lck_mtx_unlock(&bl_choose_lock);
}

/* ---- state ------------------------------------------------------------------------------------- */

static volatile uint32_t user_level = 600;      /* the slider */
static volatile uint32_t automatic;             /* automatic brightness */
static volatile uint32_t power = 1;             /* the display: 0 off, 1 on, 2 dimmed (hw.backlight.power) */
static volatile uint32_t thermal_cap = BL_LEVEL_MAX;    /* the most the panel may show while the phone is hot */
static volatile int32_t auto_offset;            /* the user's move of the curve */
static volatile uint32_t ambient_mlx = 300000;  /* the light around, thousandths of a lux (an office until told) */
static const char *ambient_source = "virtual";
static volatile uint32_t shown_level;           /* on the panel now, 0 to 1000 */
static volatile uint32_t shown_raw;
static volatile uint32_t soft_dim = 256;        /* backend soft: what the display driver keeps of a pixel */
static uint32_t soft_floor = 24;
static bool broken;                             /* backend dcs: something went wrong, nothing more is written */
static uint32_t unsent;                         /* backend dcs: commands not sent in time, one after another */
static bool error_said;                         /* backend dcs: the controller's error said, for the commands since */
static int poke;                                /* the thread's wait channel */
static void (*soft_redraw)(void);
/* Boot-arg display_sleep=0: disable panel sleep/wake on display-power boards (DBV 0 only; default 1). */
static TUNABLE(uint32_t, display_sleep_arg, "display_sleep", 1);
static LCK_MTX_DECLARE(dsi_lock, &bl_lock_group);
static LCK_MTX_DECLARE(dp_cycle_lock, &bl_lock_group);
static vm_offset_t intf, dsi_buf;
static uint64_t dsi_buf_phys;
static bool dsi_buf_failed, can_sleep;
static volatile bool panel_asleep;
static uint32_t sleep_autorefresh, dp_tries;
static uint32_t wake_sent, wake_failed, wake_frames;
static bool dp_cycle_busy, dp_cycle_done;
static uint32_t dp_cycle_ms;
static int dp_cycle_rc;

static volatile uint32_t pokes;                 /* how many times it was poked: what a turn of the thread read may be older */

static void
bl_poke(void)
{
	pokes++;
	thread_wakeup(&poke);
}

/* ---- backend dcs: a Qualcomm DSI controller (dsi_ctrl_reg.h, dsi_ctrl_hw_cmn.c of Linux's SDE driver) --- */

#define DSI_HW_VERSION                  0x000
#define DSI_CTRL                        0x004
#define   DSI_CTRL_ENABLE               (1u << 0)
#define   DSI_CTRL_VIDEO_MODE           (1u << 1)
#define   DSI_CTRL_CMD_MODE             (1u << 2)
#define DSI_STATUS                      0x008
#define   DSI_STATUS_CMD_DMA_BUSY       (1u << 1)
#define   DSI_STATUS_CMD_MDP_BUSY       (1u << 2)
#define DSI_COMMAND_MODE_DMA_CTRL       0x03c
#define   DMA_CTRL_EMBEDDED             (1u << 28)
#define   DMA_CTRL_LOW_POWER            (1u << 26)
#define   DMA_CTRL_BROADCAST            (3u << 30)
#define DSI_DMA_CMD_OFFSET              0x048
#define DSI_DMA_CMD_LENGTH              0x04c
#define DSI_DMA_FIFO_CTRL               0x050
#define DSI_TRIG_CTRL                   0x084
#define   DSI_TRIGGER_SW                4u
#define DSI_CMD_MODE_DMA_SW_TRIGGER     0x090
#define DSI_INT_CTRL                    0x110
#define   INT_CMD_DMA_DONE              (1u << 0)
#define   INT_VIDEO_DONE                (1u << 16)
#define   INT_ERROR                     (1u << 24)
#define   DSI_INT_ENABLES               0xaaaaaaaau
/* the status bits of INT_CTRL (written 1: cleared); the others enable interrupts and are kept as they are */
#define   INT_STATUS_BITS               (0x1u | 0x100u | 0x400u | 0x1000u | 0x4000u | 0x10000u | 0x100000u | \
	                                 0x1000000u | 0x10000000u | 0x40000000u)
#define DSI_TEST_PATTERN_GEN_CTRL       0x15c
#define   TPG_CMD_DMA_FIFO              ((1u << 1) | (1u << 2) | (3u << 16))   /* TPG on, its DMA FIFO, our pattern */
#define DSI_TEST_PATTERN_GEN_CMD_DMA_INIT_VAL 0x17c
#define DSI_TPG_DMA_FIFO_RESET          0x1ec
#define DSI_SIZE                        0x400
#define DSI_MAX_PAYLOAD                 60
#define DSI_MEM_MAX_PAYLOAD             1024
#define DSI_CMD_BUFFER_SIZE             0x4000
#define INTF_FRAME_COUNT                0x0ac
#define INTF_TEAR_AUTOREFRESH_CONFIG    0x2b4

#define CBCR_ENABLE                     (1u << 0)
#define CBCR_OFF                        (1u << 31)
#define GDSC_POWER_ON                   (1u << 31)

static vm_offset_t dsi, gcc, dispcc;

/* (each write has reached the controller before the next line runs: a command follows its payload) */
#define HAKIOS_MMIO_SYNC
#include "mmio.h"
#include "time.h"

SYSCTL_DECL(_kern_hakios);
#include "display_power.h"

/*
 * May the DSI controller's registers be touched now? The display block powered, the bus clock of its
 * registers running. With MAY_ENABLE (level 2, and every command) a clock the firmware left off is
 * turned on, as the display scan does (the same two writes, and only those), and GCC's put back as it
 * was found when the display block turns out to be powered down after all. Without it (level 1, which
 * only looks) nothing is written: a clock that is off is an answer, and no register behind it is read.
 * Every command looks again: whoever else might take the clock away.
 */
static bool
dsi_reachable(bool may_enable)
{
	uint32_t clock, found;

	if (gcc == 0 || dispcc == 0 || dsi == 0) {
		return false;
	}
	found = rd(gcc + profile.gcc_disp_ahb_cbcr);
	if (!(found & CBCR_ENABLE)) {
		if (!may_enable) {
			return false;
		}
		wr(gcc + profile.gcc_disp_ahb_cbcr, found | CBCR_ENABLE);
		delay(10000);
		if (!(rd(gcc + profile.gcc_disp_ahb_cbcr) & CBCR_ENABLE)) {
			wr(gcc + profile.gcc_disp_ahb_cbcr, found);
			return false;
		}
	}
	/* (read through the clock just made sure of: the display clock controller's registers need it) */
	if (!(rd(dispcc + profile.dispcc_core_gdscr) & GDSC_POWER_ON)) {
		if (!(found & CBCR_ENABLE)) {
			wr(gcc + profile.gcc_disp_ahb_cbcr, found);
		}
		return false;
	}
	clock = rd(dispcc + profile.dispcc_ahb_cbcr);
	if (clock & CBCR_OFF) {
		if (!may_enable) {
			return false;
		}
		wr(dispcc + profile.dispcc_ahb_cbcr, clock | CBCR_ENABLE);
		delay(10000);
		if (rd(dispcc + profile.dispcc_ahb_cbcr) & CBCR_OFF) {
			return false;
		}
	}
	return true;
}

static bool
dcs_start(void)
{
	uint32_t version, control;

	gcc = ml_io_map(profile.gcc, 0x80000);
	dispcc = ml_io_map(profile.dispcc, 0x20000);
	dsi = ml_io_map(profile.dsi, DSI_SIZE);
	if (!dsi_reachable(mode >= 2)) {
		say("backlight: dcs: the display block is off or its registers have no clock%s: nothing done\n",
		    mode >= 2 ? "" : " (backlight=1 turns none on)");
		return false;
	}
	version = rd(dsi + DSI_HW_VERSION);
	control = rd(dsi + DSI_CTRL);
	say("backlight: dcs: DSI controller at %#x: version %u.%u (%#x), ctrl %#x (%s%s%s), status %#x, dma ctrl %#x, int %#x\n",
	    profile.dsi, version >> 28, version >> 16 & 0xfff, version, control, control & DSI_CTRL_ENABLE ? "on" : "OFF",
	    control & DSI_CTRL_VIDEO_MODE ? ", video" : "", control & DSI_CTRL_CMD_MODE ? ", command" : "", rd(dsi + DSI_STATUS),
	    rd(dsi + DSI_COMMAND_MODE_DMA_CTRL), rd(dsi + DSI_INT_CTRL));
	if (version >> 28 != 2 || (version >> 16 & 0xfff) < 2) {
		say("backlight: dcs: not a DSI controller of 2.2 or later (no command FIFO): nothing done\n");
		return false;
	}
	if (!(control & DSI_CTRL_ENABLE)) {
		say("backlight: dcs: the controller is off (the firmware did not leave the panel lit through it): nothing done\n");
		return false;
	}
	if (((control & DSI_CTRL_VIDEO_MODE) != 0) != profile.video_mode) {
		say("backlight: dcs: the controller runs a %s-mode panel, the profile says %s mode: going by the controller\n",
		    control & DSI_CTRL_VIDEO_MODE ? "video" : "command", profile.video_mode ? "video" : "command");
		profile.video_mode = (control & DSI_CTRL_VIDEO_MODE) != 0;
	}
	if (profile.has_display_power && mode >= 2) {
		if (profile.intf != 0) {
			intf = ml_io_map((vm_offset_t)(profile.intf & ~0x3fffu), 0x4000) + (profile.intf & 0x3fffu);
		}
		if (profile.splash_base != 0 && profile.splash_size >= DSI_CMD_BUFFER_SIZE) {
			dsi_buf_phys = profile.splash_base + profile.splash_size - DSI_CMD_BUFFER_SIZE;
			dsi_buf = ml_io_map_wcomb((vm_offset_t)dsi_buf_phys, DSI_CMD_BUFFER_SIZE);
		}
		can_sleep = display_sleep_arg != 0 && intf != 0 && dsi_buf != 0;
		say("backlight: display sleep %s (buffer %#llx %s, intf %#x %s); display hardware power-down: %s\n",
		    can_sleep ? "on" : "off", (unsigned long long)dsi_buf_phys,
		    dsi_buf != 0 ? "mapped" : "NOT mapped", profile.intf, intf != 0 ? "mapped" : "NOT mapped",
		    can_sleep && dp_enabled() ? "on" : "off");
	}
	return true;
}

/* commands not sent in time, one after another, that stop the backend for the run */
#define DCS_UNSENT_MAX                  3

/* wait until `bit` of INT_CTRL is set, at most `us` microseconds */
static bool
int_wait(uint32_t bit, uint64_t us)
{
	uint64_t until = now_us() + us;

	while (!(rd(dsi + DSI_INT_CTRL) & bit)) {
		if (now_us() > until) {
			return false;
		}
		delay(20);
	}
	return true;
}

static void
int_clear(uint32_t bits)
{
	wr(dsi + DSI_INT_CTRL, (rd(dsi + DSI_INT_CTRL) & ~INT_STATUS_BITS) | bits);
}

/*
 * The DMA control and command mode as the firmware left them, taken once: a send cut off half way (the other cores stopped
 * at the run's end while this one was in dcs_send()) leaves them changed, and the next send -- the run's end's own -- is not
 * to take that for what was found and put it back so.
 */
static bool found_known, found_cmd_mode;
static uint32_t found_dma_control;

static bool
dcs_send_mode(uint32_t raw, bool quiescing)
{
	uint32_t words[4], length = 0, control, dma_control;
	unsigned n = bl_dcs_brightness_words(raw, profile.bytes, profile.little_endian, 0, words, &length);
	bool sent, error;
	uint64_t began = now_us();

	if (n == 0) {
		return false;
	}
	if (!quiescing) {
		lck_mtx_lock(&dsi_lock);
	}
	/* (`broken' is the caller's to look at: the run's end tries once more even so) */
	if (!dp_regs_on() || !dsi_reachable(true)) {
		if (!quiescing) {
			lck_mtx_unlock(&dsi_lock);
		}
		return false;
	}
	control = rd(dsi + DSI_CTRL);
	if (!(control & DSI_CTRL_ENABLE)) {
		say("backlight: dcs: the controller has been switched off: no more commands this run\n");
		broken = true;
		if (!quiescing) {
			lck_mtx_unlock(&dsi_lock);
		}
		return false;
	}
	if (profile.video_mode) {
		/* between two frames: the end of one, then the command in the blanking after it */
		int_clear(INT_VIDEO_DONE);
		(void)int_wait(INT_VIDEO_DONE, 40000);
	} else {
		/* not while a frame is on its way to a command-mode panel (about half of every frame) */
		uint64_t until = now_us() + 20000;

		while ((rd(dsi + DSI_STATUS) & DSI_STATUS_CMD_MDP_BUSY) && now_us() < until) {
			delay(50);
		}
	}
	if (!found_known) {
		found_dma_control = rd(dsi + DSI_COMMAND_MODE_DMA_CTRL);
		found_cmd_mode = (control & DSI_CTRL_CMD_MODE) != 0;
		found_known = true;
	}
	if (!found_cmd_mode) {
		wr(dsi + DSI_CTRL, control | DSI_CTRL_CMD_MODE);       /* (Linux's driver does the same for a video panel) */
	}
	/*
	 * The error bit too: it stays set until it is cleared, and one the firmware left (or a command of
	 * long ago) would otherwise be taken for this command's.
	 */
	int_clear(INT_CMD_DMA_DONE | INT_ERROR);
	/* (an empty FIFO to begin with: a send cut off half way may have left words of its own in it) */
	wr(dsi + DSI_TEST_PATTERN_GEN_CTRL, 0);
	wr(dsi + DSI_TPG_DMA_FIFO_RESET, 1);
	delay(1);
	wr(dsi + DSI_TPG_DMA_FIFO_RESET, 0);
	/* the command into the FIFO, then the length, embedded mode, high speed, this controller alone, and go */
	wr(dsi + DSI_TEST_PATTERN_GEN_CTRL, TPG_CMD_DMA_FIFO);
	for (unsigned i = 0; i < n; i++) {
		wr(dsi + DSI_TEST_PATTERN_GEN_CMD_DMA_INIT_VAL, words[i]);
	}
	dma_control = found_dma_control;
	wr(dsi + DSI_COMMAND_MODE_DMA_CTRL, (dma_control & ~(DMA_CTRL_BROADCAST | DMA_CTRL_LOW_POWER)) | DMA_CTRL_EMBEDDED);
	wr(dsi + DSI_DMA_CMD_LENGTH, length);
	wr(dsi + DSI_CMD_MODE_DMA_SW_TRIGGER, 1);
	sent = int_wait(INT_CMD_DMA_DONE, 20000);
	/* the FIFO emptied and the pattern generator off, as after every command of Linux's */
	wr(dsi + DSI_TEST_PATTERN_GEN_CTRL, 0);
	wr(dsi + DSI_TPG_DMA_FIFO_RESET, 1);
	delay(1);
	wr(dsi + DSI_TPG_DMA_FIFO_RESET, 0);
	wr(dsi + DSI_COMMAND_MODE_DMA_CTRL, dma_control);
	if (!found_cmd_mode) {
		wr(dsi + DSI_CTRL, rd(dsi + DSI_CTRL) & ~DSI_CTRL_CMD_MODE);
	}
	int_clear(INT_CMD_DMA_DONE);
	if (!sent) {
		/* (one may be lost to a moment's trouble; more in a row and the controller is not listening) */
		broken = broken || ++unsent >= DCS_UNSENT_MAX;
		say("backlight: dcs: brightness %u NOT SENT in 20 ms (int %#x, status %#x)%s\n", raw, rd(dsi + DSI_INT_CTRL),
		    rd(dsi + DSI_STATUS), broken ? ": no more commands this run" : "");
		if (!quiescing) {
			lck_mtx_unlock(&dsi_lock);
		}
		return false;
	}
	unsent = 0;
	/* said once, not again for every command after it while the controller keeps saying it */
	error = (rd(dsi + DSI_INT_CTRL) & INT_ERROR) != 0;
	if (error && !error_said) {
		say("backlight: dcs: the controller reports an error after brightness %u (int %#x)\n", raw, rd(dsi + DSI_INT_CTRL));
	}
	error_said = error;
	(void)began;
	if (!quiescing) {
		lck_mtx_unlock(&dsi_lock);
	}
	return true;
}

static bool
dcs_send(uint32_t raw)
{
	return dcs_send_mode(raw, false);
}

static bool
dsi_long_type(uint8_t type)
{
	return type == 0x39 || type == 0x29 || type == 0x0a;
}

/*
 * Send one DSI packet on virtual channel 0 (short 0x05/0x15 or long 0x39/0x29/0x0a),
 * using the TPG DMA FIFO up to 60 bytes and memory DMA from dsi_buf above that.
 */
static int
dsi_packet(uint8_t type, const uint8_t *buf, uint32_t len, bool low_power, bool spin)
{
	uint32_t words[2 + DSI_MAX_PAYLOAD / 4];
	uint32_t n = 0, i, bytes;
	bool is_long = dsi_long_type(type), memory = is_long && len > DSI_MAX_PAYLOAD;
	int rc = 0;

	if (dsi == 0 || len == 0 || (!is_long && len > 2) || len > DSI_MEM_MAX_PAYLOAD ||
	    (memory && (dsi_buf == 0 || dsi_buf_failed))) {
		return -1;
	}
	if (!is_long) {
		words[n++] = (1u << 31) | ((uint32_t)type << 16) | (len == 2 ? (uint32_t)buf[1] << 8 : 0u) | buf[0];
		bytes = 4;
		words[n++] = 0;
	} else if (!memory) {
		words[n++] = (1u << 31) | (1u << 30) | ((uint32_t)type << 16) | len;
		for (i = 0; i < len; i += 4) {
			uint32_t w = 0;
			for (uint32_t b = 0; b < 4 && i + b < len; b++) {
				w |= (uint32_t)buf[i + b] << (8 * b);
			}
			words[n++] = w;
		}
		bytes = n * 4;
		if (n & 1u) {
			words[n++] = 0;
		}
	} else {
		bytes = 4 + ((len + 3) & ~3u);
	}

	if (!spin) {
		lck_mtx_lock(&dsi_lock);
	}
	if (broken || !dp_regs_on() || !dsi_reachable(true)) {
		if (!spin) {
			lck_mtx_unlock(&dsi_lock);
		}
		return -1;
	}
	for (i = 0; i < 250 && (rd(dsi + DSI_STATUS) & DSI_STATUS_CMD_DMA_BUSY); i++) {
		delay(20);
	}
	if (rd(dsi + DSI_STATUS) & DSI_STATUS_CMD_DMA_BUSY) {
		rc = -2;
	} else {
		uint32_t tpg, dma, trig, fifo, ctrl;
		bool done;

		for (i = 0; i < 1000 && (rd(dsi + DSI_STATUS) & DSI_STATUS_CMD_MDP_BUSY); i++) {
			delay(20);
		}
		tpg = rd(dsi + DSI_TEST_PATTERN_GEN_CTRL);
		dma = rd(dsi + DSI_COMMAND_MODE_DMA_CTRL);
		trig = rd(dsi + DSI_TRIG_CTRL);
		fifo = rd(dsi + DSI_DMA_FIFO_CTRL);
		ctrl = (dma & ~(DMA_CTRL_BROADCAST | DMA_CTRL_LOW_POWER)) | DMA_CTRL_EMBEDDED |
		    (low_power ? DMA_CTRL_LOW_POWER : 0u);
		if ((trig & 0xfu) != DSI_TRIGGER_SW) {
			wr(dsi + DSI_TRIG_CTRL, (trig & ~0xfu) | DSI_TRIGGER_SW);
		}
		int_clear(INT_CMD_DMA_DONE | INT_ERROR);
		if (memory) {
			volatile uint8_t *bufp = (volatile uint8_t *)dsi_buf;
			uint32_t header = (1u << 31) | (1u << 30) | ((uint32_t)type << 16) | len;

			for (uint32_t b = 0; b < 4; b++) {
				bufp[b] = (uint8_t)(header >> (8 * b));
			}
			for (uint32_t b = 0; b < bytes - 4; b++) {
				bufp[4 + b] = b < len ? buf[b] : 0xff;
			}
#if !defined(HAKIOS_MMIO_TEST)
			__builtin_arm_dsb(0xf);
#endif
			wr(dsi + DSI_TEST_PATTERN_GEN_CTRL, 0);
			wr(dsi + DSI_COMMAND_MODE_DMA_CTRL, ctrl & ~((1u << 24) | (1u << 29)));
			wr(dsi + DSI_DMA_FIFO_CTRL, fifo | (1u << 20) | (1u << 16));
			wr(dsi + DSI_DMA_CMD_OFFSET, (uint32_t)dsi_buf_phys);
			wr(dsi + DSI_DMA_CMD_LENGTH, bytes & 0xffffffu);
		} else {
			wr(dsi + DSI_TEST_PATTERN_GEN_CTRL, 0);
			wr(dsi + DSI_TPG_DMA_FIFO_RESET, 1);
			delay(1);
			wr(dsi + DSI_TPG_DMA_FIFO_RESET, 0);
			wr(dsi + DSI_TEST_PATTERN_GEN_CTRL, TPG_CMD_DMA_FIFO);
			for (uint32_t w = 0; w < n; w++) {
				wr(dsi + DSI_TEST_PATTERN_GEN_CMD_DMA_INIT_VAL, words[w]);
			}
			wr(dsi + DSI_COMMAND_MODE_DMA_CTRL, ctrl);
			wr(dsi + DSI_DMA_CMD_LENGTH, bytes);
		}
		wr(dsi + DSI_CMD_MODE_DMA_SW_TRIGGER, 1);
		done = int_wait(INT_CMD_DMA_DONE, 25000);
		int_clear(INT_CMD_DMA_DONE);
		wr(dsi + DSI_TEST_PATTERN_GEN_CTRL, 0);
		wr(dsi + DSI_TPG_DMA_FIFO_RESET, 1);
		delay(1);
		wr(dsi + DSI_TPG_DMA_FIFO_RESET, 0);
		wr(dsi + DSI_TEST_PATTERN_GEN_CTRL, tpg);
		wr(dsi + DSI_COMMAND_MODE_DMA_CTRL, dma);
		if (memory) {
			wr(dsi + DSI_DMA_FIFO_CTRL, fifo);
		}
		if ((trig & 0xfu) != DSI_TRIGGER_SW) {
			wr(dsi + DSI_TRIG_CTRL, trig);
		}
		if (!done) {
			rc = -3;
			dsi_buf_failed = dsi_buf_failed || memory;
		}
	}
	if (rc != 0) {
		unsent++;
		say("backlight: DSI packet %#04x %#04x (%u bytes%s) failed %d, status %#010x int %#010x (failure %u)\n", type,
		    buf[0], len, memory ? ", from memory" : "", rc, rd(dsi + DSI_STATUS), rd(dsi + DSI_INT_CTRL), unsent);
		if (unsent >= DCS_UNSENT_MAX) {
			broken = true;
			say("backlight: three DSI commands failed in a row; none sent until the next boot\n");
		}
	} else {
		unsent = 0;
	}
	if (!spin) {
		lck_mtx_unlock(&dsi_lock);
	}
	return rc;
}

/* ---- the light, followed ------------------------------------------------------------------------- */

/* the light at `level': false when the panel could not be told (then it is to be tried again) */
static bool
apply(uint32_t level)
{
	shown_level = level;
	switch (profile.backend) {
	case BL_DCS: {
		uint32_t raw = bl_level_to_raw(level, profile.raw_min, profile.raw_max);

		if (raw == shown_raw) {
			break;
		}
		if (broken || !dcs_send(raw)) {
			return false;
		}
		shown_raw = raw;
		break;
	}
	case BL_SOFT: {
		uint32_t dim = bl_level_to_dim(level, soft_floor);

		if (dim != soft_dim) {
			soft_dim = dim;
			if (soft_redraw != NULL) {
				soft_redraw();
			}
		}
		break;
	}
	default:
		break;
	}
	return true;
}

/*
 * The display off: the panel dark (a DSI panel at brightness 0: black, and back at once; the software light:
 * every pixel black). Not the panel's display-off command, which would need its power-on sequence again.
 * False, as apply()'s, when the panel could not be told.
 */
static bool
apply_dark(void)
{
	shown_level = 0;
	switch (profile.backend) {
	case BL_DCS:
		if (shown_raw == 0) {
			break;
		}
		if (broken || !dcs_send(0)) {
			return false;
		}
		shown_raw = 0;
		break;
	case BL_SOFT:
		if (soft_dim != 0) {
			soft_dim = 0;
			if (soft_redraw != NULL) {
				soft_redraw();
			}
		}
		break;
	default:
		break;
	}
	return true;
}

/* dimmed (the idle timer's first step, or Darwin's dimmed display mode): at most this */
#define BL_DIMMED_MAX   150

#define TICK_MS         50

extern void hakios_fb_panel_asleep(bool asleep) __attribute__((weak));

static void
sleep_panel(void)
{
	uint64_t began = now_us();
	uint8_t off = 0x28, enter = 0x10;
	uint32_t autorefresh, i;
	int rc;

	if (hakios_fb_panel_asleep != NULL) {
		hakios_fb_panel_asleep(true);
	}
	sleep_ms(20);
	autorefresh = rd(intf + INTF_TEAR_AUTOREFRESH_CONFIG);
	if (autorefresh & (1u << 31)) {
		sleep_autorefresh = autorefresh;
		wr(intf + INTF_TEAR_AUTOREFRESH_CONFIG, autorefresh & ~(1u << 31));
	}
	for (i = 0; i < 50 && (rd(dsi + DSI_STATUS) & DSI_STATUS_CMD_MDP_BUSY); i++) {
		sleep_ms(1);
	}
	if (rd(dsi + DSI_STATUS) & DSI_STATUS_CMD_MDP_BUSY) {
		say("backlight: display off: a frame still in flight after 50 ms (DSI status %#x)\n", rd(dsi + DSI_STATUS));
	}
	rc = dsi_packet(0x05, &off, 1, true, false);
	sleep_ms(21);
	rc = rc != 0 ? rc : dsi_packet(0x05, &enter, 1, true, false);
	sleep_ms(101);
	panel_asleep = true;
	say("backlight: display off: panel asleep in %llu ms (DCS 0x28 0x10 %s)\n", (unsigned long long)((now_us() - began) / 1000),
	    rc == 0 ? "ok" : "FAILED");
}

static void
wake_panel(bool spin)
{
	uint64_t began = now_us();
	uint32_t autorefresh = rd(intf + INTF_TEAR_AUTOREFRESH_CONFIG), sent = 0, failed = 0, i = 0, j = 0;
	int first = 0;

	if (!(autorefresh & (1u << 31)) && (sleep_autorefresh & (1u << 31))) {
		autorefresh = sleep_autorefresh;
	}
	if (rd(intf + INTF_TEAR_AUTOREFRESH_CONFIG) & (1u << 31)) {
		wr(intf + INTF_TEAR_AUTOREFRESH_CONFIG, autorefresh & ~(1u << 31));
	}
	while (i + 2 <= sizeof(kPanelOn)) {
		uint8_t type = kPanelOn[i], len = kPanelOn[i + 1];

		if (type == 0xff) {
			hakios_wait_ms(len, spin);
			i += 2;
			continue;
		}
		if (i + 2 + len > sizeof(kPanelOn)) {
			break;
		}
		int rc = dsi_packet(type, &kPanelOn[i + 2], len, true, spin);

		if (rc == 0) {
			sent++;
		} else {
			failed++;
			first = first != 0 ? first : rc;
		}
		i += 2 + len;
	}
	if (autorefresh & (1u << 31)) {
		wr(intf + INTF_TEAR_AUTOREFRESH_CONFIG, autorefresh);
	}
	uint32_t frames0 = rd(intf + INTF_FRAME_COUNT);
	hakios_wait_ms(50, spin);
	uint32_t frames = rd(intf + INTF_FRAME_COUNT) - frames0;
	if (frames == 0 && (autorefresh & (1u << 31))) {
		wr(intf + INTF_TEAR_AUTOREFRESH_CONFIG, autorefresh & ~(1u << 31));
		delay(100);
		wr(intf + INTF_TEAR_AUTOREFRESH_CONFIG, autorefresh);
		if (dp_stat.ups != 0 && dp_mdss != 0) {
			wr(dp_mdss + 0x1601c, 1);
		}
		frames0 = rd(intf + INTF_FRAME_COUNT);
		hakios_wait_ms(50, spin);
		frames = rd(intf + INTF_FRAME_COUNT) - frames0;
		j = 1;
	}
	panel_asleep = false;
	shown_raw = 0;
	wake_sent = sent;
	wake_failed = failed;
	wake_frames = frames;
	if (hakios_fb_panel_asleep != NULL) {
		hakios_fb_panel_asleep(false);
	}
	say("backlight: display on: panel woken in %llu ms (%u of %u packets%s, error %d; auto-refresh %#x; in 50 ms "
	    "%u frames%s; CTL0 flush now %#x)\n", (unsigned long long)((now_us() - began) / 1000), sent,
	    (uint32_t)PANEL_ON_PACKETS, failed ? " FAILED" : "", first, autorefresh, frames,
	    j ? ", auto-refresh re-armed" : "", dp_mdss != 0 ? rd(dp_mdss + 0x16018) : 0);
}

static void
complete_power_cycle(int rc)
{
	lck_mtx_lock(&dp_cycle_lock);
	dp_cycle_ms = 0;
	dp_cycle_rc = rc;
	dp_cycle_done = true;
	lck_mtx_unlock(&dp_cycle_lock);
	thread_wakeup((event_t)&dp_cycle_done);
}

static void
run_power_cycle(uint32_t ms)
{
	bool again, good;

	if (!can_sleep || !panel_asleep) {
		snprintf(dp_stat.cycle, sizeof(dp_stat.cycle), "not tried: the panel is not asleep");
		complete_power_cycle(EIO);
		return;
	}
	if (!dp_down()) {
		dp_stat.cycle_failed++;
		snprintf(dp_stat.cycle, sizeof(dp_stat.cycle), "FAILED: not powered down (%s)", dp_stat.why);
		complete_power_cycle(EIO);
		return;
	}
	sleep_ms(ms);
	dp_up();
	if (!dp_regs_on()) {
		dp_stat.cycles++;
		dp_stat.cycle_failed++;
		snprintf(dp_stat.cycle, sizeof(dp_stat.cycle), "FAILED: the display did not come back on (%s)", dp_stat.why);
		say("display power: cycle %s\n", dp_stat.cycle);
		complete_power_cycle(EIO);
		return;
	}
	wake_panel(false);
	sleep_panel();
	again = dp_down();
	dp_stat.cycles++;
	dp_stat.cycle_frames += wake_frames;
	good = wake_failed == 0 && wake_frames != 0 && !dp_broken && again;
	if (!good) {
		dp_stat.cycle_failed++;
	}
	snprintf(dp_stat.cycle, sizeof(dp_stat.cycle), "%s: %u ms off, woken with %u of %u packets and %u frames in 50 ms; off "
	    "again %s", good ? "ok" : "FAILED", ms, wake_sent, (uint32_t)PANEL_ON_PACKETS, wake_frames, again ? "yes" : "NO");
	say("display power: cycle %s\n", dp_stat.cycle);
	complete_power_cycle(0);
}

static void
bl_thread(__unused void *argument, __unused wait_result_t result)
{
	uint32_t now = user_level << 10;        /* in 1024ths of a level */
	uint32_t reference_mlx = ambient_mlx, target = automatic ? bl_auto_target(ambient_mlx, auto_offset) : user_level;
	bool dark = false;
	bool behind = !apply(user_level);       /* the panel not told what `now' is: to be tried again */

	for (;;) {
		/*
		 * What the sysctls and the other drivers set, each read once a turn: one that changes in the
		 * middle of a turn is not seen as two different things in it (the change pokes the thread,
		 * and the next turn sees it).
		 */
		uint32_t seen = pokes, on = power, cap = thermal_cap, lux = ambient_mlx, goal, want;
		uint32_t cycle_ms = __atomic_load_n(&dp_cycle_ms, __ATOMIC_ACQUIRE);
		bool by_hand = automatic == 0;

		want = by_hand ? user_level : bl_auto_target(lux, auto_offset);
		if (on == 0) {
			bool retry_dp = false;

			/* dark once the panel was told; until then tried again every tick (no use once broken) */
			if (!dark) {
				dark = apply_dark();
			}
			if (dark && can_sleep && !panel_asleep) {
				sleep_panel();
				dp_tries = 0;
			}
			if (can_sleep && panel_asleep && !dp_down() && dp_retry() && dp_tries++ < 3) {
				retry_dp = true;
			}
			if (cycle_ms != 0) {
				run_power_cycle(cycle_ms);
				continue;
			}
			behind = true;                  /* lit again at once, where it was, when it is switched on */
			if ((dark || broken) && (!can_sleep || panel_asleep) && !retry_dp) {
				assert_wait(&poke, THREAD_UNINT);
			} else if (retry_dp) {
				assert_wait_timeout(&poke, THREAD_UNINT, 2000, NSEC_PER_MSEC);
			} else {
				assert_wait_timeout(&poke, THREAD_UNINT, TICK_MS, NSEC_PER_MSEC);
			}
			if (pokes != seen) {
				clear_wait(current_thread(), THREAD_AWAKENED);  /* (as at the loop's end: poked since `power' was read) */
				continue;
			}
			thread_block(THREAD_CONTINUE_NULL);
			continue;
		}
		if (cycle_ms != 0) {
			run_power_cycle(cycle_ms);
		}
		if (panel_asleep) {
			dp_up();
			if (dp_regs_on()) {
				wake_panel(false);
			} else {
				panel_asleep = false;
				if (hakios_fb_panel_asleep != NULL) {
					hakios_fb_panel_asleep(false);
				}
			}
			behind = true;
		}
		dark = false;

		if (by_hand) {
			target = want;
		} else if (bl_lux_moved(lux, reference_mlx) || (want > target ? want - target : target - want) > 20) {
			/* a real change of the light (not a flicker), or the user's own move of the curve */
			reference_mlx = lux;
			target = want;
		}
		goal = on == 2 && target > BL_DIMMED_MAX ? BL_DIMMED_MAX : target;
		goal = goal > cap ? cap : goal;
		if (now != goal << 10) {
			/* by hand: there in a fifth of a second; automatic: bl_step()'s pace */
			now = by_hand ? (now < goal << 10 ? now + ((goal << 10) - now + 3) / 4 :
			    now - (now - (goal << 10) + 3) / 4) : bl_step(now, goal << 10);
			behind = true;
		}
		if (behind) {
			behind = !apply((now + 512) >> 10);
		}
		/* asleep until poked only with nothing more to do: a send that failed is tried again at the next tick */
		if (now == goal << 10 && by_hand && (!behind || broken)) {
			assert_wait(&poke, THREAD_UNINT);
		} else {
			assert_wait_timeout(&poke, THREAD_UNINT, TICK_MS, NSEC_PER_MSEC);
		}
		/*
		 * A poke that came after this turn read what it acted on and before the wait above was
		 * declared has woken nobody: without this look the thread would sleep on what it no longer
		 * knows to be true -- for good, where it waits without a time.
		 */
		if (pokes != seen) {
			clear_wait(current_thread(), THREAD_AWAKENED);
			continue;
		}
		thread_block(THREAD_CONTINUE_NULL);
	}
}

/* ---- for other parts of the kernel -------------------------------------------------------------- */

/* the display driver: is the light made in software (then it keeps a copy of the frame to darken) */
bool
hakios_backlight_soft(void)
{
	bl_choose();
	return profile.backend == BL_SOFT;
}

/* ... how much of a pixel to keep (256: all), and whom to tell when that changes */
uint32_t
hakios_backlight_dim(void)
{
	return soft_dim;
}

void
hakios_backlight_set_redraw(void (*redraw)(void))
{
	soft_redraw = redraw;
}

/* the thermal limiter (bsd/dev/board/thermal.c): the most the panel may show, 0 to 1000 */
void
hakios_backlight_set_cap(uint32_t level)
{
	level = level > BL_LEVEL_MAX ? BL_LEVEL_MAX : level;
	if (level != thermal_cap) {
		thermal_cap = level;
		bl_poke();
	}
}

/* a light sensor's reading */
void
hakios_als_report(uint32_t millilux, const char *source)
{
	ambient_mlx = millilux;
	ambient_source = source;
	if (automatic) {
		bl_poke();
	}
}

/* the display driver: RequestPowerChange (0 off, 1 on; preserves dimmed=2 when already on) */
void
hakios_backlight_display(bool on)
{
	uint32_t want;

	if (!chosen || profile.backend == BL_NONE) {
		return;
	}
	want = on ? 1u : 0u;
	if ((on && power == 0) || (!on && power != 0)) {
		power = want;
		say("backlight: display %s (RequestPowerChange)\n", on ? "on" : "off");
		bl_poke();
	}
}

/* ---- sysctl hw.backlight and kern.hakios.display_power_cycle ------------------------------------- */

static int
display_power_cycle_sysctl(struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	uint32_t ms = dp_stat.cycles;
	int error = sysctl_io_number(req, ms, sizeof(ms), &ms, NULL);
	int changed = req->newptr != USER_ADDR_NULL;

	if (error != 0 || !changed) {
		return error;
	}
	if (suser(kauth_cred_get(), NULL) != 0) {
		return EPERM;
	}
	if (ms == 0 || ms > 60000 || profile.backend != BL_DCS || !can_sleep) {
		return EINVAL;
	}
	lck_mtx_lock(&dp_cycle_lock);
	while (dp_cycle_busy) {
		lck_mtx_sleep(&dp_cycle_lock, LCK_SLEEP_DEFAULT, (event_t)&dp_cycle_busy, THREAD_UNINT);
	}
	dp_cycle_busy = true;
	dp_cycle_done = false;
	dp_cycle_rc = 0;
	__atomic_store_n(&dp_cycle_ms, ms, __ATOMIC_RELEASE);
	lck_mtx_unlock(&dp_cycle_lock);
	bl_poke();
	lck_mtx_lock(&dp_cycle_lock);
	while (!dp_cycle_done) {
		lck_mtx_sleep(&dp_cycle_lock, LCK_SLEEP_DEFAULT, (event_t)&dp_cycle_done, THREAD_UNINT);
	}
	error = dp_cycle_rc;
	dp_cycle_busy = false;
	lck_mtx_unlock(&dp_cycle_lock);
	thread_wakeup((event_t)&dp_cycle_busy);
	return error;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, display_power_cycle, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    display_power_cycle_sysctl, "IU", "write ms with the panel asleep: the display off, that long, on, woken, asleep, off "
    "(read: cycles done; the result in display_power_stats)");

SYSCTL_NODE(_hw, OID_AUTO, backlight, CTLFLAG_RW | CTLFLAG_LOCKED, 0, "the panel's light");

static int
sysctl_level SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg2)
	uint32_t value = *(volatile uint32_t *)arg1;
	int changed = 0;
	int error = sysctl_io_number(req, value, sizeof(value), &value, &changed);

	if (error != 0 || !changed) {
		return error;
	}
	if (arg1 == &user_level) {
		value = value > BL_LEVEL_MAX ? BL_LEVEL_MAX : value;
		user_level = value;
		if (automatic) {
			/* by hand while automatic: the curve moves by the difference */
			auto_offset = (int32_t)value - (int32_t)bl_auto_level(ambient_mlx);
		}
	} else if (arg1 == &automatic) {
		value = value != 0;
		if (value != automatic) {
			auto_offset = 0;
		}
		automatic = value;
	} else if (arg1 == &power) {
		power = value > 2 ? 1 : value;
	} else if (arg1 == &ambient_mlx) {
		ambient_mlx = value;
		ambient_source = "virtual";
	}
	bl_poke();
	return 0;
}

SYSCTL_PROC(_hw_backlight, OID_AUTO, level, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    __DEVOLATILE(void *, &user_level), 0, sysctl_level, "IU", "the slider, 0 to 1000");
SYSCTL_PROC(_hw_backlight, OID_AUTO, auto, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    __DEVOLATILE(void *, &automatic), 0, sysctl_level, "IU", "automatic brightness");
SYSCTL_PROC(_hw_backlight, OID_AUTO, power, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    __DEVOLATILE(void *, &power), 0, sysctl_level, "IU", "the display: 0 off, 1 on, 2 dimmed");
SYSCTL_PROC(_hw_backlight, OID_AUTO, lux, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    __DEVOLATILE(void *, &ambient_mlx), 0, sysctl_level, "IU", "ambient light, thousandths of a lux");
SYSCTL_UINT(_hw_backlight, OID_AUTO, shown, CTLFLAG_RD | CTLFLAG_LOCKED, __DEVOLATILE(uint32_t *, &shown_level), 0,
    "on the panel now, 0 to 1000");
SYSCTL_UINT(_hw_backlight, OID_AUTO, raw, CTLFLAG_RD | CTLFLAG_LOCKED, __DEVOLATILE(uint32_t *, &shown_raw), 0,
    "the panel's number");
SYSCTL_UINT(_hw_backlight, OID_AUTO, cap, CTLFLAG_RD | CTLFLAG_LOCKED, __DEVOLATILE(uint32_t *, &thermal_cap), 0,
    "the most the panel may show while the phone is hot");
SYSCTL_INT(_hw_backlight, OID_AUTO, offset, CTLFLAG_RD | CTLFLAG_LOCKED, __DEVOLATILE(int32_t *, &auto_offset), 0,
    "the user's move of the automatic curve");

static int
sysctl_names SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	const char *name = arg2 == 0 ? bl_backend_names[profile.backend] : ambient_source;

	return SYSCTL_OUT(req, name, strlen(name) + 1);
}

SYSCTL_PROC(_hw_backlight, OID_AUTO, backend, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0, sysctl_names, "A",
    "dcs, soft or none");
SYSCTL_PROC(_hw_backlight, OID_AUTO, als, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 1, sysctl_names, "A",
    "where the light readings come from");

/* ---- start and end ------------------------------------------------------------------------------- */

void
hakios_backlight_init(void)
{
	thread_t thread;
	uint32_t value;

	bl_choose();
	if (PE_parse_boot_argn("bl_level", &value, sizeof(value))) {
		user_level = value > BL_LEVEL_MAX ? BL_LEVEL_MAX : value;
	}
	if (PE_parse_boot_argn("bl_auto", &value, sizeof(value))) {
		automatic = value != 0;
	}
	if (PE_parse_boot_argn("bl_floor", &value, sizeof(value))) {
		soft_floor = value > 256 ? 256 : value;
	}
	say("backlight: board %s, backend %s%s, level %u%s\n", profile.board, bl_backend_names[profile.backend],
	    mode == 1 ? " (backlight=1: looking only)" : "", user_level, automatic ? ", automatic" : "");
	if (profile.backend == BL_DCS) {
		if (!dcs_start() || mode < 2) {
			profile.backend = BL_NONE;
			return;
		}
		say("backlight: dcs: the panel's brightness is ours: %u to %u, %u byte%s%s, %s mode\n", profile.raw_min,
		    profile.raw_max, profile.bytes, profile.bytes == 1 ? "" : "s", profile.bytes == 2 ?
		    (profile.little_endian ? " LSB first" : " MSB first") : "", profile.video_mode ? "video" : "command");
	}
	if (profile.backend == BL_NONE) {
		return;
	}
	if (kernel_thread_start(bl_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "backlight");
		thread_deallocate(thread);
	}
}

/*
 * At the run's end (and on a panic's way out), before hakios_fb_quiesce(): if the panel is asleep
 * or the display hardware powered down, bring MDSS/DSI and the panel back so fb_flip can return
 * to the panel's memory and the loader sees a lit display. One core, interrupts off.
 */
void
hakios_display_power_quiesce(void)
{
	if (profile.backend != BL_DCS || !panel_asleep) {
		return;
	}
	if (!dp_regs_on()) {
		dp_up();
	}
	if (dp_regs_on()) {
		wake_panel(true);
	}
}

/*
 * At the run's end (and on a panic's way out): the panel left lit for the loader and whatever comes next (also when it was dark).
 * Tried also when the backend stopped for the run: a dark panel is worse than one more command the controller may not
 * send, and a send has its own limits of time.
 */
void
hakios_backlight_quiesce(void)
{
	if (profile.backend != BL_DCS || profile.raw_restore == 0 || !dp_regs_on()) {
		return;
	}
	if (dcs_send_mode(profile.raw_restore, true)) {
		say("backlight: at the run's end: the panel back at %u\n", profile.raw_restore);
	}
}
