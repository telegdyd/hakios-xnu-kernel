/*
 * The panel's brightness: AppleARMBacklight, the service CoreBrightness (in backboardd) sets the display's light
 * through, and the DSI commands that set it on the panel.
 *
 * Boot-arg backlight=8450: the OnePlus 10 Pro's (SM8450) panel, a Samsung AMB670YF08 (S6E3HC3 driver IC), AMOLED, on
 * DSI0 in command mode with DSC. The boot loader (Mu's DisplayDxe) set it up and left the display controller refreshing
 * it from memory by itself, so the DSI controller is running and nothing here has to start it. 0 (the default): no
 * service, nothing of the DSI is touched; on another SoC these addresses are somebody else's registers.
 *
 * Brightness is the DCS command 0x51 (set_display_brightness) with two bytes of level (DBV), the high byte first:
 * 1..0x7ff is the normal range, 0x800..0xfff the panel's high-brightness mode, which needs its own sequences and is
 * not used. Android sends it in low-power mode. The phone's device tree: qcom,mdss-dsi-bl-inverted-dbv,
 * oplus,dsi-bl-normal-max-level 0x7ff, qcom,bl-dsc-cmd-state "dsi_lp_mode"; the boot loader's init sequence already
 * sent 0x53 = 0x28 (brightness control and dimming on).
 *
 * A command goes through the DSI0 controller's command DMA fed from its own FIFO (the test pattern generator's DMA
 * port), as Linux's dsi_ctrl_hw_cmn_kickoff_fifo_command does: no buffer in memory, nothing for the SMMU. It is sent
 * between two of the frames the controller sends by itself (waited for up to 20 ms; the controller would queue it
 * behind a frame anyway), and the registers it changes are put back as found. iOSPlus10's patch 0250 is where this
 * comes from; its DCS writes returned done on this phone (its 2026-10-26 log, the panel wake's 0x11 and 0x29).
 * After three commands in a row that the controller did not report done, no more are sent until the next boot.
 *
 * CoreBrightness finds the service by {IOPropertyMatch: {backlight-control: true}} and a class that is an
 * AppleARMBacklight. Without "new-backlight-architecture" it takes its older path: it reads
 * IODisplayParameters {brightness {min, max, value}} and sets {brightness: a level in min..max, FadePeriod: seconds
 * as 16.16, backlight-level}. "brightness-nits" is not taken: CoreBrightness sends it as 0 right after the first level
 * (there is no nits calibration here), and taken as a level it left the panel dark after boot. The service
 * does not publish "digital-dimming-min-nits": with it CoreBrightness would dim by scaling the colours of every frame.
 * The level iOS sets is the DBV (0..backlight_max): iOS already maps its slider to light, and the panel's light is
 * close to linear in DBV.
 *
 * setProperties() only notes what was asked: the panel is written from this service's own work loop, in 16 ms steps
 * through a fade, so backboardd never waits for the DSI.
 *
 * Display sleep: the display's RequestPowerChange (backboardd, as the screen locks and wakes) comes here too. Off: DBV 0
 * and the panel into sleep (0x28, 0x10), so it sends no TE and the controller no frames; on: the panel woken with the
 * whole sequence the boot loader started it with, then the level iOS asks for (sleepPanel(), wakePanel()). Without the
 * buffer for long packets or INTF_1 the display goes dark with DBV 0 only.
 *
 * Boot-args: backlight=8450, backlight_max=DBV (default 2047, at most 4095), backlight_min=DBV (default 24: the slider's
 * bottom).
 *
 * Checked on the OnePlus 10 Pro (11 November): the Control Center slider moves the panel's light, DBV 24..2047, no
 * failed command; installed. Display sleep: locked and woken again and again, 101 of 101 packets, 6 frames in the 50 ms
 * after each wake, fb_flip still on. Not here: the panel's high-brightness mode, its panic wake.
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOWorkLoop.h>
#include <kern/clock.h>
#include <libkern/c++/OSCollectionIterator.h>
#include <libkern/libkern.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <stdarg.h>

#include "MiniDrivers.h"

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern "C" void loader_log_line(const char * line);

static TUNABLE(uint32_t, gBacklightSoC, "backlight", 0);
static TUNABLE(uint32_t, gBacklightMax, "backlight_max", 0x7ff);
/*
 * Boot-arg backlight_min=DBV (default 24): the lowest the slider takes the panel. Levels 1..max from iOS are spread over
 * backlight_min..max, so the whole slider still moves the light; 0 stays 0 (iOS turning the display dark). The panel's
 * own minimum is 1 (qcom,mdss-dsi-bl-min-level), but at the bottom of the slider that was too dark to see anything
 * (11 November, owner). sysctl kern.hakios.backlight_min changes it at once, for finding the right value.
 */
static TUNABLE_WRITEABLE(uint32_t, gBacklightMin, "backlight_min", 24);

/* Each line also into the loader's log, which the live USB log streams: the console is quiet once user space runs. */
static void bl_say(const char * format, ...) __printflike(1, 2);

static void
bl_say(const char * format, ...)
{
	char line[300];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	loader_log_line(line);
	IOLog("%s", line);
}

/* ---- DSI0: DCS writes through the command DMA's FIFO -------------------------------------------------------------- */

/* SM8450: the device tree's qcom,mdss_dsi_ctrl0@ae94000, reg <0xae94000 0x400>, "qcom,dsi-ctrl-hw-v2.6" */
#define DSI0_SM8450             0x0ae94000UL
#define DSI_SIZE                0x400
/* the DSI 2.x controller's registers (Linux's techpack/display dsi_ctrl_reg.h) */
#define DSI_HW_VERSION          0x000
#define DSI_CTRL                0x004           /* bit 0 enable, bit 2 command mode */
#define DSI_STATUS              0x008           /* bit 1 command DMA busy, bit 2 the controller's own frames busy */
#define DSI_CMD_DMA_CTRL        0x03c           /* 31 broadcast, 30 master, 28 embedded, 26 low power */
#define DSI_DMA_CMD_LENGTH      0x04c
#define DSI_TRIG_CTRL           0x084           /* bits 0-3: what starts the DMA; 4 is software */
#define DSI_CMD_DMA_SW_TRIGGER  0x090
#define DSI_INT_CTRL            0x110           /* even bits: what happened (a 1 clears it); odd bits: enables */
#define DSI_TPG_CTRL            0x15c           /* the test pattern generator: its DMA FIFO */
#define DSI_TPG_CMD_DMA_INIT    0x17c           /* the FIFO's port: one 32-bit word per write */
#define DSI_TPG_DMA_FIFO_RESET  0x1ec

#define DSI_STATUS_DMA_BUSY     (1u << 1)
#define DSI_STATUS_MDP_BUSY     (1u << 2)
#define DSI_INT_DMA_DONE        (1u << 0)
#define DSI_INT_ENABLES         0xaaaaaaaau
#define DSI_TPG_FIFO_ON         (3u << 16 | 1u << 2 | 1u << 1)
#define DSI_TRIGGER_SW          4u
#define DSI_MAX_PAYLOAD         60              /* bytes; the FIFO holds 64 with the header */

static volatile uint32_t * gDSI;
static IOLock * gDSILock;                       /* every command */
static bool gDSIDead;
static uint32_t gDSIFailures, gDSIBusyLines;

static inline uint32_t
dsi_rd(uint32_t offset)
{
	return gDSI[offset / 4];
}

static inline void
dsi_wr(uint32_t offset, uint32_t value)
{
	gDSI[offset / 4] = value;
}

/*
 * Packets longer than the FIFO holds (the panel's 129-byte 9E PPS when it wakes) go from memory, as the boot loader sent
 * that one: the last 16 KiB of its display memory (the live DT's cont_splash_region, 0xb8000000 + 0x2b00000), which
 * MiniIOMobileFramebuffer's flip buffers stop short of (FLIP_SPLASH_KEEP). The display's stream is not translated by
 * the SMMU (fb_smmu), so the DSI's DMA reads it at its physical address. A DMA from memory that fails is not tried
 * again until the next boot.
 */
#define DSI_CMD_BUFFER_SM8450   0xbaafc000UL
#define DSI_CMD_BUFFER_SIZE     0x4000
#define DSI_DMA_CMD_OFFSET      0x048           /* the command's address in memory */
#define DSI_DMA_FIFO_CTRL       0x050           /* bits 20 and 16: the DMA's watermarks off */
#define DSI_MEM_MAX_PAYLOAD     1024

static uint64_t gDSIBufferPhys;
static volatile uint8_t * gDSIBuffer;
static bool gDSIBufferFailed;

/* Map DSI0 and check that it is a running DSI 2.x controller in command mode, as the boot loader leaves it. */
static bool
dsi_start(uint64_t physical)
{
	gDSILock = IOLockAlloc();
	vm_offset_t virt = ml_io_map((vm_offset_t) physical, DSI_SIZE);
	if (gDSILock == NULL || virt == 0) {
		bl_say("backlight: DSI0 at %#llx not mapped; no brightness\n", physical);
		return false;
	}
	volatile uint32_t * d = (volatile uint32_t *) virt;
	uint32_t version = d[DSI_HW_VERSION / 4], ctrl = d[DSI_CTRL / 4];
	bl_say("backlight: DSI0 version %#010x ctrl %#010x status %#010x dma_ctrl %#010x trig %#010x tpg %#010x\n",
	    version, ctrl, d[DSI_STATUS / 4], d[DSI_CMD_DMA_CTRL / 4], d[DSI_TRIG_CTRL / 4], d[DSI_TPG_CTRL / 4]);
	if ((version >> 28) != 2 || (ctrl & 0x5u) != 0x5u) {
		bl_say("backlight: DSI0 is not a running command-mode DSI 2.x controller; no brightness\n");
		return false;
	}
	gDSI = d;
	return true;
}

/* The buffer in memory for long packets (no buffer: no display sleep, as a panel woken without its PPS is no picture). */
static void
dsi_buffer_start(uint64_t physical)
{
	IOMemoryDescriptor * md = IOMemoryDescriptor::withPhysicalAddress(physical, DSI_CMD_BUFFER_SIZE, kIODirectionInOut);
	IOMemoryMap * map = md != NULL ? md->map(kIOMapWriteCombineCache) : NULL;

	OSSafeReleaseNULL(md);                  /* (the map keeps it) */
	if (map != NULL) {
		gDSIBuffer = (volatile uint8_t *) map->getVirtualAddress();
		gDSIBufferPhys = physical;
	}
	bl_say("backlight: DSI0 long packets from memory at %#llx: %s\n", physical, gDSIBuffer != NULL ? "mapped" : "NOT mapped");
}

/* ---- INTF_1: the display controller's auto-refresh of the panel ------------------------------------------------- */

/*
 * The boot loader leaves the controller sending the panel a frame of its memory on every TE (DSIEnableAutoRefresh). While
 * the panel's on-sequence goes out that is paused, as Linux's _sde_encoder_autorefresh_disable_seq1 does before a
 * commit, so no frame goes between its packets; then it is put back and the frame counter looked at for 50 ms.
 */
#define INTF1_SM8450                    0x0ae36000UL
#define INTF_FRAME_COUNT                0x0ac
#define INTF_TEAR_AUTOREFRESH_CONFIG    0x2b4   /* bit 31 on, [15:0] frames per refresh */

static volatile uint32_t * gIntf;

static uint32_t
intf_rd(uint32_t offset)
{
	return gIntf[offset / 4];
}

static void
intf_wr(uint32_t offset, uint32_t value)
{
	gIntf[offset / 4] = value;
	__builtin_arm_dsb(0xf);
}

static bool
dsi_long_type(uint8_t type)
{
	return type == 0x39 || type == 0x29 || type == 0x0a;
}

/*
 * One packet to the panel on virtual channel 0, of DSI data type `type`: 0x05 (DCS, no parameter) or 0x15 (DCS, one)
 * carry their bytes in the header; 0x39 (DCS long), 0x29 (generic long) and 0x0a (PPS) a count and the bytes after
 * it. 0 when the controller reported the DMA done. Thread context only: it sleeps on the lock and spins up to 50 ms.
 */
static int
dsi_packet(uint8_t type, const uint8_t * buf, uint32_t len, bool lowPower)
{
	uint32_t words[2 + DSI_MAX_PAYLOAD / 4];
	uint32_t n = 0, i, bytes;
	bool isLong = dsi_long_type(type), memory = isLong && len > DSI_MAX_PAYLOAD;
	int rc = 0;

	if (gDSI == NULL || len == 0 || (!isLong && len > 2) || len > DSI_MEM_MAX_PAYLOAD ||
	    (memory && (gDSIBuffer == NULL || gDSIBufferFailed))) {
		return -1;
	}
	/* the header, embedded mode: [7:0] data 0 or count low, [15:8] data 1 or count high, [21:16] type, 30 long, 31 last */
	if (!isLong) {
		words[n++] = 1u << 31 | (uint32_t) type << 16 | (len == 2 ? (uint32_t) buf[1] << 8 : 0u) | buf[0];
		bytes = 4;
		words[n++] = 0;                 /* the FIFO takes an even number of words */
	} else if (!memory) {
		words[n++] = 1u << 31 | 1u << 30 | (uint32_t) type << 16 | len;
		for (i = 0; i < len; i += 4) {
			uint32_t w = 0;
			for (uint32_t b = 0; b < 4 && i + b < len; b++) {
				w |= (uint32_t) buf[i + b] << (8 * b);
			}
			words[n++] = w;
		}
		bytes = n * 4;
		if (n & 1u) {
			words[n++] = 0;
		}
	} else {
		bytes = 4 + ((len + 3) & ~3u);  /* the header, the bytes, 0xff up to a word (dsi_ctrl_copy_and_pad_cmd) */
	}

	IOLockLock(gDSILock);
	if (gDSIDead) {
		IOLockUnlock(gDSILock);
		return -1;
	}
	for (i = 0; i < 250 && (dsi_rd(DSI_STATUS) & DSI_STATUS_DMA_BUSY); i++) {
		IODelay(20);
	}
	if (dsi_rd(DSI_STATUS) & DSI_STATUS_DMA_BUSY) {
		rc = -2;                        /* a command that never finished: not to be touched */
	} else {
		/* between two of the controller's frames if it can be (one each TE: about 6 ms of 8.3 at 120 Hz) */
		for (i = 0; i < 1000 && (dsi_rd(DSI_STATUS) & DSI_STATUS_MDP_BUSY); i++) {
			IODelay(20);
		}
		if (i == 1000 && gDSIBusyLines++ < 4) {
			bl_say("backlight: DSI0 sending frames for 20 ms; command sent anyway\n");
		}
		uint32_t tpg = dsi_rd(DSI_TPG_CTRL), dma = dsi_rd(DSI_CMD_DMA_CTRL), trig = dsi_rd(DSI_TRIG_CTRL);
		uint32_t fifo = dsi_rd(DSI_DMA_FIFO_CTRL);
		uint32_t ctrl = (dma & ~(1u << 31 | 1u << 30 | 1u << 26)) | 1u << 28 | (lowPower ? 1u << 26 : 0u);
		if ((trig & 0xfu) != DSI_TRIGGER_SW) {
			dsi_wr(DSI_TRIG_CTRL, (trig & ~0xfu) | DSI_TRIGGER_SW);
		}
		dsi_wr(DSI_INT_CTRL, (dsi_rd(DSI_INT_CTRL) & DSI_INT_ENABLES) | DSI_INT_DMA_DONE);
		if (memory) {
			uint32_t header = 1u << 31 | 1u << 30 | (uint32_t) type << 16 | len;
			for (uint32_t b = 0; b < 4; b++) {
				gDSIBuffer[b] = (uint8_t) (header >> (8 * b));
			}
			for (uint32_t b = 0; b < bytes - 4; b++) {
				gDSIBuffer[4 + b] = b < len ? buf[b] : 0xff;
			}
			__builtin_arm_dsb(0xf);
			/* dsi_ctrl_hw_cmn_kickoff_command: from memory, not the FIFO; packet type and count select cleared */
			dsi_wr(DSI_TPG_CTRL, 0);
			dsi_wr(DSI_CMD_DMA_CTRL, ctrl & ~(1u << 24 | 1u << 29));
			dsi_wr(DSI_DMA_FIFO_CTRL, fifo | 1u << 20 | 1u << 16);
			dsi_wr(DSI_DMA_CMD_OFFSET, (uint32_t) gDSIBufferPhys);
			dsi_wr(DSI_DMA_CMD_LENGTH, bytes & 0xffffffu);
		} else {
			dsi_wr(DSI_TPG_CTRL, DSI_TPG_FIFO_ON);
			for (uint32_t w = 0; w < n; w++) {
				dsi_wr(DSI_TPG_CMD_DMA_INIT, words[w]);
			}
			dsi_wr(DSI_CMD_DMA_CTRL, ctrl);
			dsi_wr(DSI_DMA_CMD_LENGTH, bytes);
		}
		__builtin_arm_dsb(0xf);
		dsi_wr(DSI_CMD_DMA_SW_TRIGGER, 1);
		__builtin_arm_dsb(0xf);
		bool done = false;
		for (i = 0; i < 1250 && !done; i++) {   /* 25 ms */
			done = (dsi_rd(DSI_INT_CTRL) & DSI_INT_DMA_DONE) != 0;
			if (!done) {
				IODelay(20);
			}
		}
		dsi_wr(DSI_INT_CTRL, (dsi_rd(DSI_INT_CTRL) & DSI_INT_ENABLES) | DSI_INT_DMA_DONE);
		/* the FIFO emptied (dsi_ctrl_hw_cmn_reset_cmd_fifo), and the registers as they were */
		dsi_wr(DSI_TPG_CTRL, 0);
		dsi_wr(DSI_TPG_DMA_FIFO_RESET, 1);
		__builtin_arm_dsb(0xf);
		IODelay(1);
		dsi_wr(DSI_TPG_DMA_FIFO_RESET, 0);
		dsi_wr(DSI_TPG_CTRL, tpg);
		dsi_wr(DSI_CMD_DMA_CTRL, dma);
		if (memory) {
			dsi_wr(DSI_DMA_FIFO_CTRL, fifo);
		}
		if ((trig & 0xfu) != DSI_TRIGGER_SW) {
			dsi_wr(DSI_TRIG_CTRL, trig);
		}
		__builtin_arm_dsb(0xf);
		if (!done) {
			rc = -3;
			gDSIBufferFailed = gDSIBufferFailed || memory;
		}
	}
	if (rc != 0) {
		gDSIFailures++;
		bl_say("backlight: DSI packet %#04x %#04x (%u bytes%s) failed %d, status %#010x int %#010x (failure %u)\n", type,
		    buf[0], len, memory ? ", from memory" : "", rc, dsi_rd(DSI_STATUS), dsi_rd(DSI_INT_CTRL), gDSIFailures);
		if (gDSIFailures >= 3) {
			gDSIDead = true;
			bl_say("backlight: three DSI commands failed in a row; none sent until the next boot\n");
		}
	} else {
		gDSIFailures = 0;
	}
	IOLockUnlock(gDSILock);
	return rc;
}

/* One DCS write: buf[0] the command, then its parameters (0x05, 0x15 or 0x39 by its length). */
int
hakios_dsi_dcs_write(const uint8_t * buf, uint32_t len, bool lowPower)
{
	if (len == 0 || len > DSI_MAX_PAYLOAD) {
		return -1;
	}
	return dsi_packet(len == 1 ? 0x05 : len == 2 ? 0x15 : 0x39, buf, len, lowPower);
}

/* ---- AppleARMBacklight ------------------------------------------------------------------------------------------ */

#define BL_HW_MAX               0xfff           /* qcom,mdss-dsi-bl-max-level (high-brightness mode above 0x7ff) */
#define BL_STEP_MS              16
#define BL_FADE_MAX_MS          5000

class AppleARMBacklight : public IOService
{
	OSDeclareDefaultStructors(AppleARMBacklight);

	IOWorkLoop *            fWorkLoop;
	IOTimerEventSource *    fTimer;
	IOSimpleLock *          fState;         /* the request below */

	/* what was asked (any thread, fState held) */
	uint32_t                fTarget;
	uint32_t                fFadeFrom;
	uint64_t                fFadeStart, fFadePeriod;        /* absolute time */
	bool                    fDisplayOn;     /* RequestPowerChange: the display on */

	/* the work loop's */
	bool                    fAsleep;        /* the panel in sleep (DCS 0x28, 0x10) */
	bool                    fCanSleep;      /* the long-packet buffer and INTF_1 are there: it can be woken again */
	bool                    fKnown;         /* fLevel is what the panel has */
	uint32_t                fLevel;
	uint32_t                fWrites;
	uint64_t                fLastLine;
	uint32_t                fSaid;          /* the DBV the last line said */
	uint32_t                fSetLines;
	IOTimerEventSource *    fSettle;        /* the level the panel was left at, said 300 ms after the last write */

	static void timerFired(OSObject * owner, IOTimerEventSource * sender);
	static void settleFired(OSObject * owner, IOTimerEventSource * sender);
	void step(void);
	void apply(uint32_t level, bool last);
	void publish(uint32_t level);
	void sleepPanel(void);
	void wakePanel(void);

public:
	uint32_t                fMax;

	bool startBacklight(bool canSleep);
	void again(void);
	void displayPower(bool on);
	IOReturn setProperties(OSObject * properties) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(AppleARMBacklight, IOService);

static AppleARMBacklight * gBacklight;

void
AppleARMBacklight::publish(uint32_t level)
{
	OSDictionary * brightness = OSDictionary::withCapacity(3);
	OSDictionary * parameters = OSDictionary::withCapacity(1);

	if (brightness != NULL && parameters != NULL) {
		OSNumber * number;
		number = OSNumber::withNumber(0ULL, 32);
		brightness->setObject("min", number);
		OSSafeReleaseNULL(number);
		number = OSNumber::withNumber(fMax, 32);
		brightness->setObject("max", number);
		OSSafeReleaseNULL(number);
		number = OSNumber::withNumber(level, 32);
		brightness->setObject("value", number);
		OSSafeReleaseNULL(number);
		parameters->setObject("brightness", brightness);
		setProperty("IODisplayParameters", parameters);
	}
	OSSafeReleaseNULL(brightness);
	OSSafeReleaseNULL(parameters);
}

bool
AppleARMBacklight::startBacklight(bool canSleep)
{
	fCanSleep = canSleep;
	fDisplayOn = true;
	fState = IOSimpleLockAlloc();
	fWorkLoop = IOWorkLoop::workLoop();
	fTimer = IOTimerEventSource::timerEventSource(this, &AppleARMBacklight::timerFired);
	fSettle = IOTimerEventSource::timerEventSource(this, &AppleARMBacklight::settleFired);
	if (fState == NULL || fWorkLoop == NULL || fTimer == NULL || fSettle == NULL ||
	    fWorkLoop->addEventSource(fTimer) != kIOReturnSuccess || fWorkLoop->addEventSource(fSettle) != kIOReturnSuccess) {
		return false;
	}
	/* what the panel has is not known until the first write; the boot loader's panel file says 45 % */
	fTarget = fFadeFrom = fMax * 45 / 100;
	setProperty("backlight-control", true);
	publish(fTarget);
	registerService();
	bl_say("backlight: AppleARMBacklight up, DBV 0..%u, low-power DCS; display sleep %s\n", fMax,
	    fCanSleep ? "on (DCS 0x28 0x10, woken by the boot loader's on-sequence)" : "off (DBV 0 only)");
	return true;
}

/* the work loop: one DBV to the panel; last = the end of a fade (said at most four times a second) */
void
AppleARMBacklight::apply(uint32_t level, bool last)
{
	if (fKnown && level == fLevel) {
		return;
	}
	uint8_t command[3] = { 0x51, (uint8_t) (level >> 8), (uint8_t) level };
	int rc = hakios_dsi_dcs_write(command, sizeof(command), true);
	uint64_t now = mach_absolute_time(), second;

	fWrites++;
	nanoseconds_to_absolutetime(NSEC_PER_SEC, &second);
	if (rc != 0 || !fKnown || (last && now - fLastLine >= second / 4) || now - fLastLine >= second) {
		bl_say("backlight: panel DBV %u%s (%u writes since the last line)\n", level, rc == 0 ? "" : " FAILED", fWrites);
		fLastLine = now;
		fWrites = 0;
		fSaid = level;
	}
	if (rc == 0) {
		fKnown = true;
		fLevel = level;
		fSettle->setTimeoutMS(300);
	}
}

void
AppleARMBacklight::settleFired(OSObject * owner, __unused IOTimerEventSource * sender)
{
	AppleARMBacklight * me = (AppleARMBacklight *) owner;

	if (me->fKnown && me->fLevel != me->fSaid) {
		bl_say("backlight: panel DBV %u, settled (%u writes since the last line)\n", me->fLevel, me->fWrites);
		me->fSaid = me->fLevel;
		me->fWrites = 0;
		me->fLastLine = mach_absolute_time();
	}
}

void
AppleARMBacklight::step(void)
{
	uint64_t now = mach_absolute_time();
	IOInterruptState is = IOSimpleLockLockDisableInterrupt(fState);
	uint32_t to = fTarget, from = fFadeFrom;
	uint64_t t = now - fFadeStart, period = fFadePeriod;
	bool on = fDisplayOn;
	IOSimpleLockUnlockEnableInterrupt(fState, is);

	if (!on) {
		/* the display off: dark, and the panel to sleep; a level iOS sets meanwhile waits for the display on */
		apply(0, true);
		if (fCanSleep && !fAsleep) {
			sleepPanel();
		}
		return;
	}
	if (fAsleep) {
		wakePanel();
		now = mach_absolute_time();
		t = now - fFadeStart;
	}

	uint32_t level = to;
	bool more = false;
	if (period != 0 && t < period) {
		level = (uint32_t) ((int64_t) from + ((int64_t) to - (int64_t) from) * (int64_t) t / (int64_t) period);
		more = true;
	}
	/* iOS's 1..max onto floor..max (backlight_min) */
	uint32_t lowest = MIN(gBacklightMin, fMax);
	if (level != 0 && lowest > 1) {
		level = lowest + (uint32_t) ((uint64_t) (level - 1) * (fMax - lowest) / (fMax - 1));
	}
	apply(level, !more);
	if (more) {
		fTimer->setTimeoutMS(BL_STEP_MS);
	}
}

/*
 * The display off (the work loop, after DBV 0): the panel into sleep with the boot loader's own term sequence (its
 * DSITermSequence, the same as the device tree's qcom,mdss-dsi-off-command): 0x28 set_display_off, 21 ms, 0x10
 * enter_sleep_mode, 101 ms. In sleep the panel sends no TE, so the controller's auto-refresh stops by itself. The
 * framebuffer is told first: a flip it starts now would never be taken (MiniIOMobileFramebuffer's fb_flip would give up
 * after 100 ms), so until the wake its frames go into the panel's own memory; 20 ms let a flip already started be taken.
 */
void
AppleARMBacklight::sleepPanel(void)
{
	uint64_t began = mach_absolute_time(), ns;
	uint8_t off = 0x28, enter = 0x10;
	int rc;

	hakios_fb_panel_asleep(true);
	IOSleep(20);
	rc = dsi_packet(0x05, &off, 1, true);
	IOSleep(21);
	rc = rc != 0 ? rc : dsi_packet(0x05, &enter, 1, true);
	IOSleep(101);
	fAsleep = true;
	absolutetime_to_nanoseconds(mach_absolute_time() - began, &ns);
	bl_say("backlight: display off: panel asleep in %llu ms (DCS 0x28 0x10 %s)\n", ns / NSEC_PER_MSEC, rc == 0 ? "ok" : "FAILED");
}

/* Each entry: TYPE LEN BYTES... (the DSI data type), or 0xff MS (a wait). Made from the XML by devices/negroni/tools/panel-on-table.py. */
/* the boot loader's panel file (Mu-Silicium Binaries/negroni/RawFiles/Panel_samsung_amb670yf08_1440_3216_dsc_cmd.xml), <DSIInitSequence>: 101 packets, 7 waits */
static const uint8_t kPanelOn[720] = {
	0x39, 0x81, 0x9e, 0x12, 0x00, 0x00, 0xab, 0x30, 0x80, 0x09, 0x6c, 0x04, 0x38, 0x00, 0x24, 0x02, 0x1c, 0x02, 0x1c,
	0x02, 0x00, 0x02, 0x3b, 0x00, 0x20, 0x03, 0x35, 0x00, 0x07, 0x00, 0x0e, 0x03, 0x34, 0x02, 0xd4, 0x18, 0x00, 0x10,
	0xf0, 0x07, 0x10, 0x20, 0x00, 0x06, 0x0f, 0x0f, 0x33, 0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70, 0x77,
	0x79, 0x7b, 0x7d, 0x7e, 0x02, 0x02, 0x22, 0x00, 0x2a, 0x40, 0x2a, 0xbe, 0x3a, 0xfc, 0x3a, 0xfa, 0x3a, 0xf8, 0x3b,
	0x38, 0x3b, 0x78, 0x3b, 0xb6, 0x4b, 0xb6, 0x4b, 0xf4, 0x4b, 0xf4, 0x6c, 0x34, 0x84, 0x74, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x15, 0x02, 0x9d, 0x01,
	0x05, 0x01, 0x11,
	0xff, 0x0c,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x31, 0xc5,
	0x39, 0x22, 0xc5, 0x29, 0x14, 0x00, 0x00, 0x00, 0x11, 0x10, 0x50, 0x05, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84,
	0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x05, 0x01, 0x2c,
	0x05, 0x01, 0x00,
	0xff, 0x11,
	0x05, 0x01, 0x2c,
	0x05, 0x01, 0x00,
	0xff, 0x11,
	0x05, 0x01, 0x2c,
	0x05, 0x01, 0x00,
	0xff, 0x11,
	0x05, 0x01, 0x2c,
	0x05, 0x01, 0x00,
	0xff, 0x11,
	0x05, 0x01, 0x2c,
	0x05, 0x01, 0x00,
	0xff, 0x19,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x22, 0xb9,
	0x39, 0x03, 0xb9, 0xa1, 0xb1,
	0x39, 0x04, 0xb0, 0x00, 0x3a, 0xb9,
	0x15, 0x02, 0xb9, 0x02,
	0x39, 0x04, 0xb0, 0x00, 0x26, 0xb9,
	0x39, 0x03, 0xb9, 0x00, 0x00,
	0x15, 0x02, 0xf7, 0x0f,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x24, 0xb9,
	0x15, 0x02, 0xb9, 0x21,
	0x39, 0x04, 0xb0, 0x00, 0x38, 0xb9,
	0x15, 0x02, 0xb9, 0x02,
	0x39, 0x04, 0xb0, 0x00, 0x2a, 0xb9,
	0x39, 0x03, 0xb9, 0x00, 0x00,
	0x15, 0x02, 0xf7, 0x0f,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0xff, 0x79,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x16, 0xf2,
	0x39, 0x03, 0xf2, 0x1b, 0x50,
	0x39, 0x04, 0xb0, 0x00, 0x08, 0xcb,
	0x15, 0x02, 0xcb, 0x24,
	0x15, 0x02, 0x60, 0x00,
	0x39, 0x03, 0xbd, 0x23, 0x02,
	0x39, 0x04, 0xb0, 0x00, 0x10, 0xbd,
	0x15, 0x02, 0xbd, 0x00,
	0x39, 0x04, 0xb0, 0x00, 0x16, 0xbd,
	0x15, 0x02, 0xbd, 0x77,
	0x39, 0x04, 0xb0, 0x00, 0x14, 0xbd,
	0x15, 0x02, 0xbd, 0x00,
	0x15, 0x02, 0xf7, 0x0f,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x03, 0xf2, 0x00, 0x01,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x15, 0x02, 0x35, 0x00,
	0x15, 0x02, 0xb9, 0x00,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x05, 0x2a, 0x00, 0x00, 0x04, 0x37,
	0x39, 0x05, 0x2b, 0x00, 0x00, 0x09, 0x6b,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x15, 0x02, 0xc3, 0x89,
	0x39, 0x04, 0xb0, 0x00, 0x24, 0xc3,
	0x39, 0x12, 0xc3, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00,
	0xfd,
	0x39, 0x04, 0xb0, 0x00, 0x3a, 0xc3,
	0x39, 0x12, 0xc3, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00,
	0xb0,
	0x39, 0x04, 0xb0, 0x00, 0x50, 0xc3,
	0x39, 0x12, 0xc3, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00,
	0x43,
	0x39, 0x04, 0xb0, 0x00, 0x61, 0xc3,
	0x39, 0x17, 0xc3, 0xcc, 0xcc, 0xcc, 0xcc, 0xc0, 0xfe, 0x00, 0xfe, 0x00, 0xfe, 0x00, 0xfe, 0x00, 0xfe, 0x00, 0xfe,
	0x00, 0xfe, 0x00, 0xfe, 0x00, 0xfe,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x2b, 0xf6,
	0x39, 0x04, 0xf6, 0x60, 0x63, 0x69,
	0x15, 0x02, 0xf7, 0x0f,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x46, 0xf4,
	0x15, 0x02, 0xf4, 0x08,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x18, 0xb1,
	0x39, 0x02, 0xb1, 0x05,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x0d, 0x63,
	0x15, 0x02, 0x63, 0x01,
	0x39, 0x04, 0xb0, 0x00, 0x0c, 0x63,
	0x15, 0x02, 0x63, 0x20,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x15, 0x02, 0x53, 0x28,
	0x15, 0x02, 0xf7, 0x0f,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x15, 0x02, 0x55, 0x00,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x52, 0x1f,
	0x15, 0x02, 0x1f, 0x02,
	0x39, 0x04, 0xb0, 0x00, 0x54, 0x1f,
	0x15, 0x02, 0x1f, 0x00,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x05, 0x01, 0x29,
};
#define PANEL_ON_PACKETS 101

/*
 * The display on: the panel woken with the whole sequence the boot loader started it with (not 0x11 and 0x29 alone:
 * iOSPlus10's 0258 woke this panel so, and it stayed black with a purple strip, 26 October), with the auto-refresh
 * paused while it goes out: Samsung's PPS (9E, 129 bytes, from memory), compression on (9D 01), 0x11 and its waits with
 * the boot loader's empty write_memory_start frames, the panel's registers, TE on (35 00), the window (2A, 2B), 53 28,
 * 0x29. Android sends its on-command so too, after a reset of the panel this does not do. Then the auto-refresh back on,
 * its frames counted for 50 ms, and the framebuffer may flip again.
 */
void
AppleARMBacklight::wakePanel(void)
{
	uint64_t began = mach_absolute_time(), ns;
	uint32_t autorefresh = intf_rd(INTF_TEAR_AUTOREFRESH_CONFIG), sent = 0, failed = 0, i = 0, j;
	int first = 0;

	if (autorefresh & (1u << 31)) {
		intf_wr(INTF_TEAR_AUTOREFRESH_CONFIG, autorefresh & ~(1u << 31));
	}
	while (i + 2 <= sizeof(kPanelOn)) {
		uint8_t type = kPanelOn[i], len = kPanelOn[i + 1];

		if (type == 0xff) {
			IOSleep(len);
			i += 2;
			continue;
		}
		if (i + 2 + len > sizeof(kPanelOn)) {
			break;
		}
		int rc = dsi_packet(type, &kPanelOn[i + 2], len, true);
		if (rc == 0) {
			sent++;
		} else {
			failed++;
			first = first != 0 ? first : rc;
		}
		i += 2 + len;
	}
	if (autorefresh & (1u << 31)) {
		intf_wr(INTF_TEAR_AUTOREFRESH_CONFIG, autorefresh);
	}
	uint32_t frames0 = intf_rd(INTF_FRAME_COUNT);
	IOSleep(50);
	uint32_t frames = intf_rd(INTF_FRAME_COUNT) - frames0;
	if (frames == 0 && (autorefresh & (1u << 31))) {
		/* no frame: the auto-refresh off and on once, as a commit would (the tear check's TE count restarts at a wake) */
		intf_wr(INTF_TEAR_AUTOREFRESH_CONFIG, autorefresh & ~(1u << 31));
		IODelay(100);
		intf_wr(INTF_TEAR_AUTOREFRESH_CONFIG, autorefresh);
		frames0 = intf_rd(INTF_FRAME_COUNT);
		IOSleep(50);
		frames = intf_rd(INTF_FRAME_COUNT) - frames0;
		j = 1;
	} else {
		j = 0;
	}
	fAsleep = false;
	fKnown = false;                                 /* the level again, whatever the panel has after it */
	hakios_fb_panel_asleep(false);
	absolutetime_to_nanoseconds(mach_absolute_time() - began, &ns);
	bl_say("backlight: display on: panel woken in %llu ms (%u of %u packets%s, error %d; auto-refresh %#x; in 50 ms "
	    "%u frames%s)\n", ns / NSEC_PER_MSEC, sent, (uint32_t) PANEL_ON_PACKETS, failed ? " FAILED" : "", first,
	    autorefresh, frames, j ? ", auto-refresh re-armed" : "");
}

/* RequestPowerChange (MiniIOMobileFramebuffer): does not wait for the panel */
void
AppleARMBacklight::displayPower(bool on)
{
	IOInterruptState is = IOSimpleLockLockDisableInterrupt(fState);
	bool changed = fDisplayOn != on;
	fDisplayOn = on;
	IOSimpleLockUnlockEnableInterrupt(fState, is);
	if (changed) {
		bl_say("backlight: display %s (RequestPowerChange)\n", on ? "on" : "off");
		fTimer->setTimeoutUS(1);
	}
}

void
hakios_backlight_display(bool on)
{
	if (gBacklight != NULL) {
		gBacklight->displayPower(on);
	}
}

/* the floor changed: the panel again, at the level last asked */
void
AppleARMBacklight::again(void)
{
	fTimer->setTimeoutUS(1);
}

SYSCTL_DECL(_kern_hakios);

static int
backlight_min_sysctl(__unused struct sysctl_oid * oidp, __unused void * arg1, __unused int arg2, struct sysctl_req * req)
{
	uint32_t value = gBacklightMin;
	int error = sysctl_handle_int(oidp, &value, 0, req);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (value > BL_HW_MAX) {
		return EINVAL;
	}
	gBacklightMin = value;
	bl_say("backlight: floor now DBV %u (sysctl)\n", value);
	if (gBacklight != NULL) {
		gBacklight->again();
	}
	return 0;
}

SYSCTL_PROC(_kern_hakios, OID_AUTO, backlight_min, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    backlight_min_sysctl, "IU", "the lowest DBV the brightness slider gives (backlight_min)");

void
AppleARMBacklight::timerFired(OSObject * owner, __unused IOTimerEventSource * sender)
{
	((AppleARMBacklight *) owner)->step();
}

IOReturn
AppleARMBacklight::setProperties(OSObject * properties)
{
	OSDictionary * d = OSDynamicCast(OSDictionary, properties);
	if (d == NULL) {
		return kIOReturnBadArgument;
	}
	OSNumber * b = OSDynamicCast(OSNumber, d->getObject("brightness"));
	OSNumber * f = OSDynamicCast(OSNumber, d->getObject("FadePeriod"));
	if (b == NULL) {
		/* nothing of the light: which keys, once each kind (the rest is CoreBrightness's own business) */
		if (fSetLines < 24) {
			char name[32], keys[120] = "";
			OSCollectionIterator * it = OSCollectionIterator::withCollection(d);
			const OSSymbol * key;

			fSetLines++;
			proc_selfname(name, sizeof(name));
			while (it != NULL && (key = OSDynamicCast(OSSymbol, it->getNextObject())) != NULL) {
				strlcat(keys, " ", sizeof(keys));
				strlcat(keys, key->getCStringNoCopy(), sizeof(keys));
			}
			OSSafeReleaseNULL(it);
			bl_say("backlight: %s sets%s (no level)\n", name, keys);
		}
		return kIOReturnSuccess;
	}
	if (fSetLines < 24) {
		char name[32];
		OSNumber * l = OSDynamicCast(OSNumber, d->getObject("backlight-level"));

		fSetLines++;
		proc_selfname(name, sizeof(name));
		bl_say("backlight: %s sets %u keys: brightness %d FadePeriod %#x backlight-level %d\n", name, d->getCount(),
		    (int) b->unsigned32BitValue(), f ? f->unsigned32BitValue() : 0, l ? (int) l->unsigned32BitValue() : -1);
	}
	int32_t v = (int32_t) b->unsigned32BitValue();
	uint32_t to = v < 0 ? 0 : MIN((uint32_t) v, fMax);
	uint64_t ms = f != NULL ? ((uint64_t) f->unsigned32BitValue() * 1000) >> 16 : 0, period;
	nanoseconds_to_absolutetime(MIN(ms, (uint64_t) BL_FADE_MAX_MS) * NSEC_PER_MSEC, &period);
	uint64_t now = mach_absolute_time();

	IOInterruptState is = IOSimpleLockLockDisableInterrupt(fState);
	uint32_t current = fTarget;                     /* a new fade starts where the one running is now */
	if (fFadePeriod != 0 && now - fFadeStart < fFadePeriod) {
		current = (uint32_t) ((int64_t) fFadeFrom + ((int64_t) fTarget - (int64_t) fFadeFrom) *
		    (int64_t) (now - fFadeStart) / (int64_t) fFadePeriod);
	}
	fFadeFrom = current;
	fTarget = to;
	fFadeStart = now;
	fFadePeriod = period;
	IOSimpleLockUnlockEnableInterrupt(fState, is);
	publish(to);
	fTimer->setTimeoutUS(1);
	return kIOReturnSuccess;
}

/* MiniIOMobileFramebufferStart(): the service, under the display, once the display is up */
void
hakios_backlight_start(IOService * display)
{
	uint64_t physical, buffer, intf;

	if (gBacklightSoC == 0 || gBacklight != NULL) {
		return;
	}
	if (gBacklightSoC == 8450) {
		physical = DSI0_SM8450;
		buffer = DSI_CMD_BUFFER_SM8450;
		intf = INTF1_SM8450;
	} else {
		bl_say("backlight: backlight=%u is not a SoC known here (8450); no brightness\n", gBacklightSoC);
		return;
	}
	if (!dsi_start(physical)) {
		return;
	}
	dsi_buffer_start(buffer);
	gIntf = (volatile uint32_t *) ml_io_map((vm_offset_t) intf, 0x1000);
	AppleARMBacklight * bl = OSTypeAlloc(AppleARMBacklight);
	if (bl == NULL || !bl->init()) {
		OSSafeReleaseNULL(bl);
		return;
	}
	bl->fMax = gBacklightMax < 16 ? 16 : MIN(gBacklightMax, (uint32_t) BL_HW_MAX);
	if (!bl->attach(display)) {
		bl_say("backlight: not attached to the display; no brightness\n");
		bl->release();
		return;
	}
	if (!bl->startBacklight(gDSIBuffer != NULL && gIntf != NULL)) {
		bl_say("backlight: no work loop; no brightness\n");
		bl->detach(display);
		bl->release();
		return;
	}
	gBacklight = bl;                                /* keeps the reference */
}
