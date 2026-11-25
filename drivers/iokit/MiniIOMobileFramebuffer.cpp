/*
 * MiniIOMobileFramebuffer: a stand-in for Apple's display driver family, as seen by
 * IOMobileFramebuffer.framework (and through it by CoreAnimation's window server).
 *
 * The display is the framebuffer the boot loader left us (the one the kernel's text console
 * draws on). There is no hardware behind it that we drive: a "swap" copies the pixels of the
 * surfaces of its layers into that memory, and "vsync" is a timer.
 *
 * The protocol (selectors and the layout of the swap request) was worked out from the
 * user-space side of Darwin 16.5. Everything not implemented is logged once.
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSArray.h>
#include <libkern/c++/OSBoolean.h>
#include <libkern/c++/OSString.h>
#include <libkern/c++/OSSymbol.h>
#include <libkern/c++/OSCollectionIterator.h>
#include <kern/thread_call.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <sys/kauth.h>

extern "C" {
#include <console/video_console.h>
extern struct vc_info vinfo;            /* the boot framebuffer, as the text console knows it */
void loader_log_line(const char *line); /* osfmk/arm64/loader_return.c: the log the loader is handed */
}

#include <IOKit/IOBufferMemoryDescriptor.h>

#include "MiniIOSurface.h"

extern "C" void hakios_tl(const char * format, ...) __printflike(1, 2);
/* hakios: once user space owns the console the kernel's IOLog reaches no log we can read: say it in the loader's */
#define IOLog(...) hakios_tl(__VA_ARGS__)


/* IOConnectCallMethod selectors of the framebuffer user client (Darwin 16.5) */
enum {
	kFBMethodGetLayerDefaultSurface = 3,
	kFBMethodSwapBegin              = 4,
	kFBMethodSwapEnd                = 5,
	kFBMethodSwapWait               = 6,
	kFBMethodGetID                  = 7,
	kFBMethodGetDisplaySize         = 8,
	kFBMethodRequestPowerChange     = 12,
	kFBMethodSetDebugFlags          = 15,   /* Darwin 26 on (IOMobileFramebuffer.framework's kern_*) */
	kFBMethodSetGammaTable          = 17,
	kFBMethodIsMainDisplay          = 18,
	kFBMethodSwapSignal             = 20,
	kFBMethodSetDisplayDevice       = 22,
	kFBMethodGetDigitalOutState     = 25,
	kFBMethodGetGammaTable          = 27,
	kFBMethodGetDotPitch            = 28,
	kFBMethodGetDisplayArea         = 29,
	kFBMethodVideoPowerSavings      = 33,
	kFBMethodSetBrightnessCorrection = 50,
	kFBMethodSetColorRemapMode      = 51,
	kFBMethodSwapCancel             = 52,
	kFBMethodSetMatrix              = 55,
	kFBMethodGetColorRemapMode      = 57,
	kFBMethodSetParameter           = 68,
	kFBMethodSetContrast            = 84,
	kFBMethodOpenedByName           = 88,   /* sent by IOMobileFramebufferOpenByName */
	kFBMethodNotifications          = 72,
	kFBMethodChangeFrameInfo        = 73,
	kFBMethodSupportedFrameInfo     = 74,
	kFBMethodGetCurrentAbsoluteTime = 77,
	kFBMethodGetBlock               = 79,
	kFBMethodSwapCancelAll          = 81,
	kFBMethodGetProtectionOptions   = 82,
};

/* The swap request (IOMobileFramebufferSwapEnd): 0x46c bytes, of which we read these. */
#define SWAP_SIZE               0x46c
#define SWAP_ID                 0x50
#define SWAP_SURFACE(layer)     (0x54 + 4 * (layer))    /* surface number, 0 = none */
#define SWAP_SOURCE(layer)      (0x64 + 16 * (layer))   /* x, y, width, height in the surface */
#define SWAP_ENABLED(layer)     (0xa4 + 4 * (layer))
#define SWAP_DEST(layer)        (0xc4 + 16 * (layer))   /* x, y, width, height on the display */
#define SWAP_LAYERS_SET         0x104                   /* bit per layer given in this swap */
#define SWAP_LAYERS_CLEARED     0x108
#define SWAP26_SIZE             0x588                   /* Darwin 26's request ... */
#define SWAP26_SHIFT            0x48                    /* ... with these fields that much further on */
#define SWAP_LAYERS             4

#define kFBGammaSize            3084                    /* SetGammaTable / GetGammaTable: 3 x 257 words */

#define kFBNotifyVSync          5
#define kFBNotifyTypes          8
#define kFBRefreshHz            60                      /* matches the 60 Hz hardware scanout left by DisplayDxe; overridable via fb_hz */

#define FRAME_TIMES             64                      /* milliseconds told apart between two frames */
#define FRAME_PAUSE             90                      /* from so many on, nothing was moving: not a slow frame */
#define SWAP_GPU_MS             500                     /* a frame the GPU has not drawn by then is shown as it is */
#define SWAP_NEW_MS             33                      /* ... and one not yet given to the GPU at all is waited for this long (show()) */
#define SHOWN_SLOTS             8                       /* the surfaces whose last shown submission is kept (fShown) */
#define SWAP_HELD_MS            700                     /* a swap that cannot be taken by then is not shown at all */

#define FOURCC(a, b, c, d) ((uint32_t)(a) << 24 | (uint32_t)(b) << 16 | (uint32_t)(c) << 8 | (uint32_t)(d))

static uint64_t gFBLogged[2];           /* selectors already reported */
static uint8_t  gFBCalls[128];          /* how many calls of each selector were traced */

/*
 * The display user space is told about. The system carries the identity of an iPhone (the loader's
 * identity.h, from the IPSW's device tree), and Darwin knows what screen that phone has: with any other
 * size MobileGestalt reports an unknown screen class and the user interface has no layout for it.
 * So the display is always the iPhone's -- the loader puts its pixels in /chosen/display-size and its
 * scale in /chosen/display-scale -- and a swap scales the picture to the panel that is really there
 * (in QEMU the loader makes the panel exactly this size and nothing is scaled). Without the loader's
 * word it is the iPhone X's. Boot-args fbwidth= and fbheight= change the size reported (0: the panel's).
 */
#define kFBDefaultWidth         1125
#define kFBDefaultHeight        2436
#define kFBDotsPerInch          458             /* the iPhone X's: other widths are given in proportion */
#define kFBNotGiven             0xffffffffu

static TUNABLE(uint32_t, gFBWidth, "fbwidth", kFBNotGiven);
static TUNABLE(uint32_t, gFBHeight, "fbheight", kFBNotGiven);
/*
 * Boot-arg fbscale=N: the same points drawn at N pixels a point instead of the iPhone's own scale (fbscale=2 on
 * an iPhone X's 375 x 812 points). Everything that is drawn is drawn by the processor, and this is that many
 * fewer pixels to draw; the picture is then enlarged to the panel and is that much less sharp.
 */
#define kFBDefaultScale         3
static TUNABLE(uint32_t, gFBScale, "fbscale", 0);
/* Boot-arg fbtrace=N: log the first N calls of every selector (default 3). */
static TUNABLE(uint32_t, gFBTrace, "fbtrace", 3);

/*
 * A recording of the screen, for a machine one cannot look at while it runs (the phone is
 * reached only through the boot loader, before and after the kernel).
 * Boot-args rec_secs=N [rec_start=S] [rec_mb=M]: from S seconds after the first frame, for N
 * seconds, every frame that is shown is also kept -- one pixel of every REC_STEP in each
 * direction, three bytes a pixel -- in a piece of memory of M megabytes (default 480, which
 * is 520 frames) that is left in place when the kernel returns to the loader. The loader
 * hands it to the host ("bench recording"), where tools/recording.py makes a video of it.
 *
 *   struct rec_header, then per frame: struct rec_frame, width * height pixels (R, G, B)
 *
 * Boot-arg rec_ring=1: when the memory is full the recording does not stop but goes on over its
 * oldest frames, so that what it holds at the end is the last of the run: for something that
 * happens once in a long session, the run is ended right after it (five fingers) and the frames
 * before it are there. The header ("RWREC003") says where the oldest frame is (frames are all
 * the same size, one after the other) and when the recording began, in the kernel's time, for the
 * frames to be matched with what the window server's per-frame lines (glframes=1,
 * userland/src/eagl.c) say. rec_step=N (2 to 8, default 3) keeps one pixel of every N instead of 3:
 * N = 4 holds 1.8 times as many frames, 6 four times. The memory is taken from the system's for the
 * whole run: 640 MB of it under a long session of applications had Darwin killing processes for
 * memory, SpringBoard and the window server among them (two respring in one run); 256 did not.
 */
static TUNABLE(uint32_t, gRecSeconds, "rec_secs", 0);
static TUNABLE(uint32_t, gRecStart, "rec_start", 0);
static TUNABLE(uint32_t, gRecMegabytes, "rec_mb", 480);
static TUNABLE(uint32_t, gRecRing, "rec_ring", 0);
static TUNABLE(uint32_t, gRecStep, "rec_step", 3);
static TUNABLE(uint32_t, gDiff, "fb_diff", 0);
/*
 * Boot-arg fb_filter=0: the picture is scaled to the panel nearest neighbour, as it once was. The
 * iPhone's 1125 x 2436 on this panel's 1080 x 2400 is 45 columns and 36 rows fewer: nearest
 * neighbour leaves out every 25th column and every 67th row, and where one of them goes through a
 * letter or a slanting line, that is a step in it or a stroke a pixel thinner -- text and lines a
 * little crooked, here and there and never in the same places twice. By default every pixel of the
 * panel is worked out from the four of the picture around where it falls (bilinear).
 */
static TUNABLE(uint32_t, gFilter, "fb_filter", 1);
static TUNABLE(uint32_t, gRefreshHz, "fb_hz", kFBRefreshHz);
static TUNABLE(uint32_t, gOnTheBeat, "fb_beat", 0);
/*
 * Boot-arg fb_double=1: the frame is scaled into a copy in ordinary memory first and copied to the
 * panel's memory afterwards. Without a tearing-effect signal or a page flip that hides no tear, and on
 * the phone (30 October, rc.jobs.latency-bench, tools/smoothness.py) it cost the window server: the
 * presenter took 5.4 ms a frame instead of 3.1 (median) and twice as many of the window server's own
 * frames took over 16.7 ms, the copy's memory traffic beside the GPU's. Off by default.
 */
static TUNABLE(uint32_t, gDoubleBuffer, "fb_double", 0);
/*
 * Boot-arg fb_threads=N (1 to 8): so many processors scale a frame onto the panel together (see
 * drawRows()). Three by default: a frame is in the panel's memory 3 ms after its turn instead of 9
 * to 10 (the phone, 15 October; fb_threads=1 is how it was).
 */
static TUNABLE(uint32_t, gThreads, "fb_threads", 4);
/*
 * Boot-arg fb_yuv=1: a swap layer of two planes ('420f', '420v': a video's frames, the camera's preview) is drawn as
 * well, under the others, and the four-byte layers drawn after it in that swap are laid over it by their alpha
 * (premultiplied, as CoreAnimation renders them). iOS hands a video to the display as a layer of its own and leaves a
 * transparent hole for it in the layer of everything else; without, the hole is a black one (Camera.app's preview).
 * On by default (fb_yuv=0: such a layer is left out, as before); a swap without one is drawn as before either way.
 */
static TUNABLE(uint32_t, gYUV, "fb_yuv", 1);
/*
 * Boot-arg fb_small=0: the drawers anywhere, as before. By default, on a phone with slower cores
 * (three Cortex-A55 beside four A78 on a Snapdragon 780G), the presenter and its helpers are bound to those, one
 * each, and there are no more of them than there are such cores. Kernel threads run above every
 * thread of the user interface: drawing a frame on four of the fast cores at once took them from
 * SpringBoard, the window server and the application for 5 to 9 ms of every frame, and during a
 * launch interrupted by the home gesture all seven cores were busy (the phone, 30 October). The
 * scaling streams through memory and keeps up on the small cores.
 */
static TUNABLE(uint32_t, gSmallCores, "fb_small", 1);
extern "C" unsigned hakios_slow_cores(void);           /* osfmk/arm64/psci_smp.c */
extern "C" int hakios_bind_slow_core(unsigned n);

static void
bindDrawer(unsigned index, const char * who)
{
	int cpu;

	if (!gSmallCores || hakios_slow_cores() < 2) {
		return;
	}
	cpu = hakios_bind_slow_core(index);
	if (cpu >= 0) {
		IOLog("MiniIOMobileFramebuffer: %s on cpu %d, a slower core\n", who, cpu);
	} else {
		IOLog("MiniIOMobileFramebuffer: %s anywhere: no slower core up for it\n", who);
	}
}
#define SWAP_QUEUE              2                       /* frames that may wait for the panel at most (fb_queue) */
static TUNABLE(uint32_t, gQueue, "fb_queue", 2);
static inline unsigned
swapQueue(void)
{
	return gQueue >= 1 && gQueue <= SWAP_QUEUE ? gQueue : 1;
}
/*
 * Boot-arg touch_lat=1: a line for every swap handed over ("tl: s ID at US") and for every frame
 * put on the panel ("tl: v ID ready US shown US done US": the GPU done with it, its beat come, the
 * pixels in the panel's memory), in microseconds of mach_absolute_time, for measuring how long a
 * touch takes to show (tools/latency.py; the touch's own lines: bsd/dev/board/input.c).
 */
static TUNABLE(uint32_t, gTouchLat, "touch_lat", 0);

static inline uint64_t
micros(uint64_t absolute)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(absolute, &ns);
	return ns / 1000;
}

/*
 * Boot-arg fb_gpu=0: frames put on the panel by the processors, as before. By default, where the GPU is kept (gpu=1) and is
 * an A7XX, the GPU's 2D engine puts them there (MiniIOSurfacePresent(), BoardGPU.cpp): a submission of the kernel's own,
 * after whatever drew the frame, scaling it bilinear onto the panel. The processors' part is a few hundred words of
 * commands, and no processor reads a surface the GPU may be drawing into (shownReading). A frame the GPU cannot take (a
 * surface of a program's own memory, a layer partly off the panel, a surface no GPU has drawn into, a video's two planes,
 * no GPU) is drawn by the processors after the GPU is done with what it was given of the panel. The OnePlus 10 Pro's
 * GPU-PRESENT-PLAN.md step 1.
 */
static TUNABLE(uint32_t, gGPUPresent, "fb_gpu", 1);
static uint32_t gLastPresent;           /* the last frame's fence when the GPU put it on the panel (0: the processors did) */
static uint64_t gPanelPhys;
SYSCTL_DECL(_kern_hakios);
SYSCTL_QUAD(_kern_hakios, OID_AUTO, fb_panel_phys, CTLFLAG_RD | CTLFLAG_LOCKED, &gPanelPhys, "fb_gpu: the panel's memory");
/*
 * ... and whether it is the picture the processors would have drawn: the 60th frame the GPU puts on the panel, and every
 * 1800th after it (half a minute at 60 a second), is waited for, and 256 of its pixels on the panel are compared with
 * what drawLayer()'s bilinear makes of the surface there ("fb_gpu: check"). A frame whose surface was drawn into again
 * meanwhile is not counted.
 */
#define GPU_CHECK_FIRST         60
#define GPU_CHECK_EVERY         1800
/* sysctl kern.hakios.fb_gpu_check=N: the next N frames the GPU puts on the panel are checked as well */
static uint32_t gCheckNext;
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_gpu_check, CTLFLAG_RW | CTLFLAG_LOCKED, &gCheckNext, 0,
    "fb_gpu: so many of the next frames checked against the processors' bilinear");
extern "C" void flush_dcache64(addr64_t addr, unsigned count, int phys);
extern "C" vm_offset_t ml_vtophys(vm_offset_t vaddr);
extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
/* why frames went to the processors under fb_gpu, counted for the 5-second line */
enum { kRefusedNoPanel, kRefusedBlack, kRefusedOffPanel, kRefusedNotGPUDrawn, kRefusedNothing, kRefusedByGPU, kRefusedVideo,
       kRefusedKinds };

/*
 * The display controller, for fb_flip (the board's feature display-flip: its SoC file's display { } mdss and intf1, its
 * own display { } splash-size; drivers/board/board.c). The offsets in the MDSS are an SM8450's (waipio: Linux's
 * dpu_hw_catalog sm8450, the OnePlus 10 Pro's live DT qcom,sde-sspp-off/ctl-off); another DPU's are somebody else's
 * registers, which is why it is a feature a board lists. sysctl kern.hakios.fb_dpu reads its state each time it is asked:
 * each source pipe's address, size, crop, stride and format, each CTL's layers, flush and interfaces, and the panel
 * interface's counters, tear check and auto-refresh. Reads only.
 */
extern "C" bool hakios_display_flip_data(uint64_t * mdss, uint64_t * intf, uint64_t * splash_size);
#define MDSS_WINDOW_FIRST       0x01000                 /* what is mapped of it: the top's interrupts, the pipes, CTLs, mixers */
#define MDSS_WINDOW_END         0x47000
#define MDP_INTR_EN             0x01010                 /* (Linux's dpu_hw_interrupts.c, MDP_SSPP_TOP0) */
#define MDP_INTR_STATUS         0x01014
#define MDP_INTR_CLEAR          0x01018
#define MDP_INTR_INTF1_UNDERRUN (1u << 26)
#define SSPP_SRC_SIZE           0x00
#define SSPP_SRC_XY             0x08
#define SSPP_OUT_SIZE           0x0c
#define SSPP_OUT_XY             0x10
#define SSPP_SRC0_ADDR          0x14
#define SSPP_SRC1_ADDR          0x18
#define SSPP_SRC2_ADDR          0x1c
#define SSPP_SRC3_ADDR          0x20
#define SSPP_SRC_YSTRIDE0       0x24
#define SSPP_SRC_YSTRIDE1       0x28
#define SSPP_SRC_FORMAT         0x30
#define SSPP_SRC_UNPACK         0x34
#define SSPP_SRC_OP_MODE        0x38
#define SSPP_UBWC_ERROR_STATUS  0x138
#define SSPP_PE_C0_LR           0x100                   /* pixel extension: LR, TB, REQ_PIXELS of C0, C1C2 (+0x10), C3 (+0x20) */
/* fb_video: what a VIG pipe's fetch is paced by, copied from the pipe the boot loader set up (dpu_hw_sspp.c) */
static const uint16_t kSSPPQoS[] = { 0x048, 0x060, 0x064, 0x068, 0x06c, 0x074, 0x078, 0x130, 0x134, 0x150 };
/* ... and its scaler (QSEED3 lite, version 0x3001: the live DT's qcom,sde-qseed-scalar-version) and 10-bit CSC */
#define VIG_SCALER              0xa00                   /* qcom,sde-vig-qseed-off; dpu_hw_util.c's QSEED3_* below */
#define QSEED3_HW_VERSION       0x00
#define QSEED3_OP_MODE          0x04
#define QSEED3_PHASE_STEP_Y_H   0x10
#define QSEED3_PHASE_STEP_Y_V   0x14
#define QSEED3_PHASE_STEP_UV_H  0x18
#define QSEED3_PHASE_STEP_UV_V  0x1c
#define QSEED3_PRELOAD          0x20
#define QSEED3_SRC_SIZE_Y_RGB_A 0x40
#define QSEED3_SRC_SIZE_UV      0x44
#define QSEED3_DST_SIZE         0x48
#define QSEED3_PHASE_INIT_Y_H   0x90                    /* .. Y_V 0x94, UV_H 0x98, UV_V 0x9c */
#define VIG_CSC10               0x1a00                  /* qcom,sde-vig-csc-off: the op mode, then the matrix from +4 */
#define CSC10_EN                (1u << 0)
#define CSC10_SRC_YUV           (1u << 1)
/* the display's mixers (qcom,sde-mixer-off: LM0..LM4; CTL_LAYER0 + 4 * n is LMn's) and their blend stages */
#define LM_BASE(n)              (0x45000 + 0x1000 * (n))
#define LM_OP_MODE              0x00
#define LM_BLEND_OP(stage)      (0x20 + 0x18 * ((stage) - 1)) /* qcom,sde-mixer-blend-op-off, stage 1 (DPU_STAGE_0) on */
#define LM_BLEND_CONST(stage)   (LM_BLEND_OP(stage) + 4)
#define LM_BLEND_OPAQUE         0x100                   /* DPU_BLEND_FG_ALPHA_FG_CONST | DPU_BLEND_BG_ALPHA_BG_CONST */
#define LM_BLEND_PREMULTIPLIED  0x600                   /* FG_ALPHA_FG_CONST | BG_ALPHA_FG_PIXEL | BG_INV_ALPHA */
#define LM_BLEND_CONST_FG       0x00ff0000u             /* combined alpha: the foreground's 255, the background's 0 */
#define LM_FLUSH(n)             (6 + (n))               /* LM0..LM4's bits in CTL_FLUSH (dpu_hw_ctl.c) */
#define CTL_LAYER0              0x00                    /* LM0..LM4 at 0x00..0x10, LM5 at 0x24 */
#define CTL_TOP                 0x14
#define CTL_FLUSH               0x18
#define CTL_START               0x1c
#define CTL_INTF_ACTIVE         0xf4
#define CTL_FETCH_PIPE_ACTIVE   0xfc
#define CTL_MIX_SHIFT_VIG2      6                       /* a pipe's stage (stage + 1, 3 bits) in CTL_LAYER: VIG0 0, VIG1 3, */
#define CTL_MIX_SHIFT_VIG3      26                      /* ... VIG2 6, VIG3 26 (dpu_hw_ctl.c setup_blendstage) */
#define CTL_FETCH_VIG2          (1u << 18)              /* ... and in CTL_FETCH_PIPE_ACTIVE: VIG0 16 .. VIG3 19 */
#define CTL_FETCH_VIG3          (1u << 19)
#define INTF_TIMING_ENGINE_EN   0x000
#define INTF_FRAME_COUNT        0x0ac
#define INTF_LINE_COUNT         0x0b0
#define INTF_STATUS             0x26c
#define INTF_TEAR_TEAR_CHECK_EN 0x284
#define INTF_TEAR_SYNC_CONFIG_VSYNC 0x288
#define INTF_TEAR_SYNC_CONFIG_HEIGHT 0x28c
#define INTF_TEAR_INT_COUNT_VAL 0x298                   /* frame count << 16 | line */
#define INTF_TEAR_START_POS     0x2a0
#define INTF_TEAR_RD_PTR_IRQ    0x2a4
#define INTF_TEAR_LINE_COUNT    0x2b0
#define INTF_TEAR_AUTOREFRESH_CONFIG 0x2b4              /* bit 31 on, [15:0] frames per refresh */
static const struct {
	const char *    name;
	uint32_t        offset;
	uint8_t         flush;                          /* its bit in CTL_FLUSH (Linux's sde_hw_ctl.c sspp table) */
} kDPUPipes[] = {
	{ "VIG0", 0x05000, 0 }, { "VIG1", 0x07000, 1 }, { "VIG2", 0x09000, 2 }, { "VIG3", 0x0b000, 18 },
	{ "DMA0", 0x25000, 11 }, { "DMA1", 0x27000, 12 }, { "DMA2", 0x29000, 24 }, { "DMA3", 0x2b000, 25 },
};
static const uint32_t kDPUCTLs[] = { 0x16000, 0x17000, 0x18000, 0x19000, 0x1a000, 0x1b000 };
static vm_offset_t gMDSS;                       /* where MDSS_WINDOW_FIRST is mapped (0: not yet) */
static vm_offset_t gIntf;                       /* ... and the panel's interface */
static uint64_t gSplashSize;                    /* the firmware's display memory, from the panel's on */

static inline uint32_t
mdssRead(uint32_t offset)
{
	return *(volatile uint32_t *)(gMDSS + offset - MDSS_WINDOW_FIRST);
}

static inline void
mdssWrite(uint32_t offset, uint32_t value)
{
	*(volatile uint32_t *)(gMDSS + offset - MDSS_WINDOW_FIRST) = value;
}

static inline uint32_t
intfRead(uint32_t offset)
{
	return *(volatile uint32_t *)(gIntf + offset);
}

/* the registers mapped, once (false: the board has not the feature display-flip) */
static bool
mdssMapped(void)
{
	uint64_t mdss, intf, splash;

	if (gMDSS != 0) {
		return true;
	}
	if (!hakios_display_flip_data(&mdss, &intf, &splash) || (mdss & 0x3fffULL) != 0) {
		return false;
	}
	vm_offset_t window = ml_io_map((vm_offset_t) mdss + MDSS_WINDOW_FIRST, MDSS_WINDOW_END - MDSS_WINDOW_FIRST);
	vm_offset_t panel = ml_io_map((vm_offset_t) (intf & ~0x3fffULL), 0x4000) + (vm_offset_t) (intf & 0x3fffULL);

	if (window == 0 || panel == 0) {
		return false;
	}
	gSplashSize = splash;
	gIntf = panel;
	/* (mapped by another caller meanwhile: this mapping stays unused; rare, and small) */
	__sync_bool_compare_and_swap(&gMDSS, 0, window);
	return true;
}

static int
sysctl_fb_dpu SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	const size_t size = 4096;
	char *text;
	size_t used = 0;
	uint32_t frames, lines[12], tears[12];
	int error;

	if (!mdssMapped()) {
		return SYSCTL_OUT(req, "no display-flip on this board\n", 31);
	}
	text = (char *) IOMalloc(size);
	if (text == NULL) {
		return ENOMEM;
	}
#define SAY(...) (used += (size_t) snprintf(text + used, used < size ? size - used : 0, __VA_ARGS__), used = used < size ? used : size)
	for (unsigned p = 0; p < sizeof(kDPUPipes) / sizeof(kDPUPipes[0]); p++) {
		uint32_t b = kDPUPipes[p].offset;
		SAY("%s addr %#x size %#x xy %#x out %#x at %#x stride %#x format %#x unpack %#x op %#x (flush bit %u)\n",
		    kDPUPipes[p].name, mdssRead(b + SSPP_SRC0_ADDR), mdssRead(b + SSPP_SRC_SIZE), mdssRead(b + SSPP_SRC_XY),
		    mdssRead(b + SSPP_OUT_SIZE), mdssRead(b + SSPP_OUT_XY), mdssRead(b + SSPP_SRC_YSTRIDE0),
		    mdssRead(b + SSPP_SRC_FORMAT), mdssRead(b + SSPP_SRC_UNPACK), mdssRead(b + SSPP_SRC_OP_MODE),
		    kDPUPipes[p].flush);
	}
	for (unsigned c = 0; c < sizeof(kDPUCTLs) / sizeof(kDPUCTLs[0]); c++) {
		uint32_t b = kDPUCTLs[c];
		SAY("CTL%u layers %#x %#x %#x %#x %#x %#x top %#x flush %#x start %#x intf %#x fetch %#x\n", c,
		    mdssRead(b + CTL_LAYER0), mdssRead(b + CTL_LAYER0 + 4), mdssRead(b + CTL_LAYER0 + 8),
		    mdssRead(b + CTL_LAYER0 + 12), mdssRead(b + CTL_LAYER0 + 16), mdssRead(b + 0x24), mdssRead(b + CTL_TOP),
		    mdssRead(b + CTL_FLUSH), mdssRead(b + CTL_START), mdssRead(b + CTL_INTF_ACTIVE),
		    mdssRead(b + CTL_FETCH_PIPE_ACTIVE));
	}
	for (unsigned m = 0; m < 2; m++) {
		uint32_t b = LM_BASE(m);
		SAY("LM%u op %#x out %#x blend", m, mdssRead(b + LM_OP_MODE), mdssRead(b + 0x04));
		for (unsigned stage = 1; stage <= 5; stage++) {
			SAY(" %u:%#x/%#x", stage, mdssRead(b + LM_BLEND_OP(stage)), mdssRead(b + LM_BLEND_CONST(stage)));
		}
		SAY("\n");
	}
	for (unsigned p = 2; p < 4; p++) {
		uint32_t b = kDPUPipes[p].offset;
		SAY("%s scaler version %#x op %#x steps %#x %#x src %#x dst %#x; csc op %#x; addr1 %#x\n", kDPUPipes[p].name,
		    mdssRead(b + VIG_SCALER + QSEED3_HW_VERSION), mdssRead(b + VIG_SCALER + QSEED3_OP_MODE),
		    mdssRead(b + VIG_SCALER + QSEED3_PHASE_STEP_Y_H), mdssRead(b + VIG_SCALER + QSEED3_PHASE_STEP_Y_V),
		    mdssRead(b + VIG_SCALER + QSEED3_SRC_SIZE_Y_RGB_A), mdssRead(b + VIG_SCALER + QSEED3_DST_SIZE),
		    mdssRead(b + VIG_CSC10), mdssRead(b + SSPP_SRC1_ADDR));
	}
	SAY("MDP interrupts enabled %#x status %#x (INTF_1 underrun bit 26)\n", mdssRead(MDP_INTR_EN), mdssRead(MDP_INTR_STATUS));
	SAY("INTF timing %#x status %#x tearcheck %#x vsync %#x height %#x start %#x rdptr %#x autorefresh %#x\n",
	    intfRead(INTF_TIMING_ENGINE_EN), intfRead(INTF_STATUS), intfRead(INTF_TEAR_TEAR_CHECK_EN),
	    intfRead(INTF_TEAR_SYNC_CONFIG_VSYNC), intfRead(INTF_TEAR_SYNC_CONFIG_HEIGHT), intfRead(INTF_TEAR_START_POS),
	    intfRead(INTF_TEAR_RD_PTR_IRQ), intfRead(INTF_TEAR_AUTOREFRESH_CONFIG));
	/* the counters, every 700 us for about one refresh at 120 Hz; then how many frames in 100 ms */
	for (unsigned s = 0; s < 12; s++) {
		lines[s] = intfRead(INTF_LINE_COUNT);
		tears[s] = intfRead(INTF_TEAR_INT_COUNT_VAL);
		IODelay(700);
	}
	SAY("INTF line_count");
	for (unsigned s = 0; s < 12; s++) {
		SAY(" %u", lines[s]);
	}
	SAY("\nINTF tear frame:line");
	for (unsigned s = 0; s < 12; s++) {
		SAY(" %u:%u", tears[s] >> 16, tears[s] & 0xffff);
	}
	frames = intfRead(INTF_FRAME_COUNT);
	uint32_t tear = intfRead(INTF_TEAR_INT_COUNT_VAL) >> 16;
	IOSleep(100);
	SAY("\nINTF in 100 ms: frame_count +%u, tear frames +%u, tear line_count %u\n", intfRead(INTF_FRAME_COUNT) - frames,
	    ((intfRead(INTF_TEAR_INT_COUNT_VAL) >> 16) - tear) & 0xffff, intfRead(INTF_TEAR_LINE_COUNT));
#undef SAY
	error = SYSCTL_OUT(req, text, used);
	IOFree(text, size);
	return error;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, fb_dpu, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0, sysctl_fb_dpu, "A",
    "display-flip: the display controller's state, read now");

/*
 * Boot-arg fb_flip=0 (or sysctl kern.hakios.fb_flip=0 at any time; 1 back): frames copied into the panel's own memory,
 * the one buffer the display controller reads. By default, on a board with display-flip and with fb_gpu, frames go into
 * two buffers of the panel's size after the panel's own memory, in turn, and the display controller is pointed at the
 * one written once the GPU is done with it (each of its pipes' source address and stride, then CTL0's flush): it takes
 * the new address when it starts its next frame, so it never reads a buffer being written, and their rows (a multiple of
 * 64 bytes) the 2D engine writes as they are, with no copy of the GPU's own. The buffer it showed is written again only
 * once the flush is seen taken. A frame the GPU cannot take, or that does not cover the whole panel, goes into the
 * panel's own memory as before, and the controller back to it. If a flush is not taken in 100 ms, the controller goes
 * back to the panel's memory and fb_flip is off. The OnePlus 10 Pro's GPU-PRESENT-PLAN.md step 4: under the firmware's
 * auto-refresh a flush alone was taken at the frame's start, every one of the timed ones.
 */
static TUNABLE_WRITEABLE(uint32_t, gFlip, "fb_flip", 1);
/*
 * Boot-arg fb_video=0: a video's layer (two planes, '420v' or '420f') drawn by the processors as before (fb_yuv). By
 * default, with fb_flip, it is shown by the display controller, as an iPhone's display does: a VIG pipe (two, VIG2 and
 * VIG3, when it crosses from the left mixer's half into the right's, as the boot loader's VIG0 and VIG1 do) converts it
 * to RGB (its 10-bit CSC, BT.601 video or full range), scales it (QSEED3 lite, bilinear) and the mixers put it under
 * the frame, which is blended over it by its alpha (premultiplied: CoreAnimation leaves a transparent hole for the
 * video). The controller reads memory of one piece and a surface's is not: its two planes are copied as they are by the
 * GPU, in the submission that puts the frame into its buffer, into video memory of that buffer's own after fb_flip's
 * two (the rest of the boot loader's display memory). The pipes are set and the mixers changed with the flip that
 * shows the frame, in the same flush. A video the controller cannot take (off the panel, a frame not covering it, too
 * large for the memory) goes to the processors as before.
 */
static TUNABLE_WRITEABLE(uint32_t, gVideo, "fb_video", 1);
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_video, CTLFLAG_RW | CTLFLAG_LOCKED, &gVideo, 0,
    "fb_video: video layers shown by the display controller");
static uint32_t gVideoState, gVideoFrames;
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_video_state, CTLFLAG_RD | CTLFLAG_LOCKED, &gVideoState, 0,
    "fb_video: 0 not tried, 1 ready, 2 no room, 3 the mixers not as expected");
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_video_frames, CTLFLAG_RD | CTLFLAG_LOCKED, &gVideoFrames, 0,
    "fb_video: frames shown with a video on the display controller");
/*
 * sysctl kern.hakios.fb_video_test=SURFACE (root; 0 to stop): a two-plane surface of a test's own shown as a video
 * under every frame the GPU puts into an fb_flip buffer, 960 x 540 at 60, 800 of the panel (across the middle), with
 * the GPU cutting the frame's hole over it as CoreAnimation would (userland's video_test makes the surface and moves
 * its picture).
 */
static uint32_t gVideoTest;
static MiniIOSurface * gVideoTestSurface;
#define VIDEO_TEST_X            60
#define VIDEO_TEST_Y            800
#define VIDEO_TEST_W            960
#define VIDEO_TEST_H            540
static int
sysctl_fb_video_test SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	uint32_t value = gVideoTest;
	int error = sysctl_handle_int(oidp, &value, 0, req);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (suser(kauth_cred_get(), NULL) != 0) {
		return EPERM;
	}
	MiniIOSurface * s = value != 0 ? MiniIOSurfaceCopy(value) : NULL;

	if (value != 0 && (s == NULL || s->info.planeCount != 2)) {
		if (s != NULL) {
			MiniIOSurfaceDrop(s);
		}
		return EINVAL;
	}
	MiniIOSurface * was = __atomic_exchange_n(&gVideoTestSurface, s, __ATOMIC_ACQ_REL);

	gVideoTest = value;
	if (was != NULL) {
		IOSleep(100);           /* (the presenter may be using it: it holds its own reference only while it is) */
		MiniIOSurfaceDrop(was);
	}
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, fb_video_test, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, sysctl_fb_video_test,
    "I", "fb_video: a test's two-plane surface shown as a video under every frame (root; 0 off)");
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_flip, CTLFLAG_RW | CTLFLAG_LOCKED, &gFlip, 0, "fb_flip: frames shown by flipping between two buffers");
#define FLIP_SPLASH_KEEP        0x4000                  /* the end of the firmware's display memory: its DSI command buffer */
#define FLIP_CTL                0                       /* CTL0, which drives the panel's interface (kern.hakios.fb_dpu) */
#define FLIP_TAKEN_MS           100
#define FLIP_TIMED              16                      /* the first so many flips are timed to when they are taken */
static uint32_t gFlipState;
static const uint8_t * volatile gScreenBase;   /* the fb_flip buffer the controller reads (NULL: the panel's own memory) */
static volatile uint32_t gScreenRowBytes;
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_flip_state, CTLFLAG_RD | CTLFLAG_LOCKED, &gFlipState, 0,
    "fb_flip: 0 not tried, 1 ready, 2 no display-flip, 3 the controller not as expected, 4 no room, 5 not mapped, 6 a flush not taken");
/* the scan position: the tear check's read pointer, from INTF_TEAR_START_POS at the TE on */
static inline uint32_t
flipLine(void)
{
	return intfRead(INTF_TEAR_INT_COUNT_VAL) & 0xffff;
}

static inline uint32_t
refreshHz(void)
{
	return gRefreshHz >= 1 && gRefreshHz <= 240 ? gRefreshHz : kFBRefreshHz;
}
static inline uint32_t
recStep(void)
{
	return gRecStep >= 2 && gRecStep <= 8 ? gRecStep : 3;
}
#define REC_STEP        recStep()

/*
 * The panel's own refresh (boot-arg fb_scan, on by default; bsd/dev/board/adreno.c says where it is
 * read from). A command-mode panel: at each of its refreshes the display controller
 * reads this framebuffer out to it, top to bottom, in about half a refresh, and leaves it alone the
 * rest. Redwood's is a video one, read out all the time but its blanking (8.23 of 8.34 ms): a band then
 * has a refresh's time between its two readouts, the next band a line later. Without this its frames
 * were written whenever they came, and with frames coming faster than 60 a second the panel showed
 * bands of the frame before. Two things follow from knowing when:
 *   - the refresh told to the window server (the "vsync" of the user client below) is the panel's,
 *     not a timer of ours that drifts against it: a frame now and then shown twice and the next not
 *     at all is what such a drift looks like;
 *   - every band of rows of a frame is written only between the readout of it before (already
 *     read, the band may change) and the next (it must be done by then), all bands for the same
 *     next readout, the frame's target: that readout shows all of the new frame, none of it torn.
 * The model: a readout began at `start` (the counter said `frame`), they come every `period`, and a
 * line takes `line_ticks` to read out. It is kept up to date from the counters (scanObserve()).
 */
extern "C" bool hakios_display_scan_start(void);
extern "C" bool hakios_display_scan(uint32_t * frame, uint32_t * line);
extern "C" bool hakios_display_scan_video(void);
static TUNABLE(uint32_t, gScan, "fb_scan", 1);
/* fb_scan_lead=US: the first lead from a frame's start to its target, 1000 to 8000 microseconds (default 2000) */
static TUNABLE(uint32_t, gScanLeadUs, "fb_scan_lead", 2000);

struct panel_scan {
	bool            ok;
	uint32_t        lines;          /* lines a readout */
	uint64_t        line_ticks;     /* absolute time a line */
	uint64_t        period;         /* absolute time from one readout to the next */
	uint64_t        start;          /* when the latest readout seen began */
	uint32_t        frame;          /* ... and the frame counter during it */
	uint64_t        ref_start;      /* the first readout seen, for the period */
	uint32_t        ref_frame;
};
static struct panel_scan gPanel;
#define SCAN_MARGIN_US  250             /* kept between a band's writing and the readout of it */

static inline uint64_t
usToAbs(uint64_t us)
{
	uint64_t t;

	nanoseconds_to_absolutetime(us * NSEC_PER_USEC, &t);
	return t;
}

/* Read the counters once; while a readout is going on, it says exactly when it began. */
static void
scanObserve(void)
{
	uint32_t frame, line;
	uint64_t now;

	if (!gPanel.ok || !hakios_display_scan(&frame, &line)) {
		gPanel.ok = false;
		return;
	}
	now = mach_absolute_time();
	if (line < 8 || line > gPanel.lines) {
		return;                         /* between two readouts (the counter stands at 1) */
	}
	gPanel.start = now - (uint64_t) line * gPanel.line_ticks;
	gPanel.frame = frame;
	if (frame - gPanel.ref_frame >= 240) {   /* four seconds and more: the period, to a part in 10^5 */
		gPanel.period = (gPanel.start - gPanel.ref_start) / (frame - gPanel.ref_frame);
	}
}

/* The first readout to begin at `after` or later. */
static uint64_t
scanNextStart(uint64_t after)
{
	uint64_t start = gPanel.start, period = gPanel.period;

	if (after <= start) {
		return start;
	}
	return start + (after - start + period - 1) / period * period;
}

/*
 * Before rows [y0, y1) are written for the readout that begins at `target`: wait until the readout
 * before it has read them. Says false if it is too late for them to be in time (the readout of
 * them in `target` begun or about to): written then, the frame is torn there.
 */
static bool
scanWaitRows(int32_t y0, int32_t y1, uint64_t target)
{
	uint64_t open, now = mach_absolute_time(), margin = usToAbs(SCAN_MARGIN_US);

	if (target == 0 || y1 <= y0) {
		return true;
	}
	open = target - gPanel.period + (uint64_t) y1 * gPanel.line_ticks + margin;
	if (now < open && open - now < gPanel.period) {
		clock_delay_until(open);
		now = mach_absolute_time();
	}
	return now + margin < target + (uint64_t) y0 * gPanel.line_ticks;
}

/* ... and after: were they written in time? */
static inline bool
scanInTime(int32_t y0, uint64_t target)
{
	return target == 0 || mach_absolute_time() <= target + (uint64_t) y0 * gPanel.line_ticks;
}

struct rec_header {
	char     magic[8];              /* "RWREC003" */
	uint32_t width, height;
	uint32_t frames;
	uint32_t oldest;                /* the place of the oldest frame (rec_ring; 0 until the memory has once been full) */
	uint64_t began;                 /* when the first frame was recorded, in nanoseconds of the kernel's time */
};
struct rec_frame {
	uint64_t nanoseconds;           /* since the first frame of the recording */
};

extern "C" uint64_t loader_recording[3];        /* { physical address, bytes, kernel address }: osfmk/arm64/loader_return.c */

/*
 * The panel's light made in software (bsd/dev/board/backlight.c, backend soft: a machine whose panel's
 * light cannot be set, QEMU among them): the frame is drawn into a copy in memory (as with fb_double)
 * and darkened on its way to the panel, so that a new level can be shown at once from that copy, also
 * when no new frame comes.
 */
extern "C" bool hakios_backlight_soft(void);
extern "C" uint32_t hakios_backlight_dim(void);
extern "C" void hakios_backlight_set_redraw(void (*redraw)(void));

static inline uint32_t
dimPixel(uint32_t pixel, uint32_t dim)
{
	uint32_t rb = ((pixel & 0x00ff00ffu) * dim >> 8) & 0x00ff00ffu;
	uint32_t ga = (((pixel >> 8) & 0x00ff00ffu) * dim) & 0xff00ff00u;

	return rb | ga;
}

/* Rows from the copy in memory to the panel: as they are, or darkened (aligned 32-bit stores only). */
static void
toPanel(uint8_t * to, const uint8_t * from, size_t bytes)
{
	uint32_t dim = hakios_backlight_dim();

	if (dim >= 256) {
		memcpy(to, from, bytes);
		return;
	}
	volatile uint32_t * out = (volatile uint32_t *) to;
	const uint32_t * in = (const uint32_t *) from;

	for (size_t i = 0, n = bytes / 4; i < n; i++) {
		out[i] = dimPixel(in[i], dim);
	}
}

static uint32_t
swapWord(const uint8_t * swap, unsigned offset)
{
	uint32_t value;

	memcpy(&value, swap + offset, sizeof(value));
	return value;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

struct RowJob;

class IOMobileFramebuffer : public IOService
{
	OSDeclareDefaultStructors(IOMobileFramebuffer);

public:
	IOLock *        fLock;
	uint8_t *       fBase;                  /* the boot framebuffer: the panel */
	uint8_t *       fBackBuffer;            /* RAM backbuffer for tear-free double buffering */
	IOBufferMemoryDescriptor * fBackBufferDescriptor;
	uint32_t        fPanelWidth, fPanelHeight, fRowBytes;
	uint64_t        fPanelPhys;             /* ... its physical address (fb_gpu; 0: not one piece) */
	/* fb_gpu: the last frame the GPU put on the panel, for the check; what was done how, for the 5-second line */
	uint8_t *       fGPUBase;               /* where it went (the panel or an fb_flip buffer) */
	uint32_t        fGPURowBytes;
	struct MiniIOSurfacePresentLayer fGPUFirst;     /* its first layer, and how many it had */
	uint32_t        fGPUDrawn, fGPULayers, fGPUFrames;
	uint32_t        fStatGPUFrames, fStatCPUFrames, fStatRefused[kRefusedKinds];
	/* fb_flip */
	bool            fFlipTried, fFlipPending;
	int             fFlipShown;             /* the buffer the controller reads (-1: the panel's own memory) */
	unsigned        fFlipPipeCount;
	uint32_t        fFlipPipe[2], fFlipHomeStride[2], fFlipFlush, fFlipRowBytes, fFlipWrittenLine, fFlipCount;
	uint64_t        fFlipPhys[2], fFlipWritten;
	IOMemoryDescriptor * fFlipMemory[2];
	IOMemoryMap *   fFlipMap[2];
	uint8_t *       fFlipBase[2];
	uint32_t        fStatFlips, fStatFlipsHome, fStatFlipNotFull, fStatFlipWaited;
	uint64_t        fStatFlipWait;
	/* fb_video */
	struct VideoFrame {
		bool            valid;
		bool            full;                   /* '420f' (else '420v', video range) */
		uint32_t        sx, sy, sw, sh;         /* of the luma plane */
		uint32_t        dx, dy, dw, dh;         /* on the panel */
		uint32_t        lumaPitch, chromaPitch;
		uint32_t        chromaOffset;           /* from the buffer's start */
	}               fVideoNext;             /* the frame of the next flip's, put into the target's video buffer */
	uint64_t        fVideoRegionPhys, fVideoRegionSize, fVideoPhys[2], fVideoBufferSize;
	bool            fVideoOn;               /* the controller shows a video now */
	unsigned        fVideoLMs;              /* the mixers CTL0 drives (the boot loader's: 2) */
	uint32_t        fVideoUIStage;          /* the frame's stage in them (the boot loader's VIG0's) */
	uint32_t        fVideoHomeLayer[5], fVideoHomeFetch, fVideoHomeLMOp[5], fVideoHomeBlend[5][2];
	uint32_t        fVideoPipesOn;          /* 1: VIG2, 2: VIG3 */
	uint32_t        fStatVideoFrames, fStatVideoRefused, fStatVideoUnderruns;
	uint32_t        fWidth, fHeight;        /* the display as user space sees it */
	/* settings user space gives and may read back; nothing here applies them to the panel */
	uint8_t         fGamma[kFBGammaSize];
	bool            fGammaSet;
	uint32_t        fDebugFlags, fColorRemapMode;
	MiniIOSurface * fDefaultSurface;
	uint32_t        fSwapID;
	uint32_t        fSwapsShown;
	uint32_t        fLayersShown;           /* which layers the last swap had a surface in */
	bool            fGraphics;              /* the text console has been told to stop */
	/*
	 * A frame goes to the panel when the GPU is done with it: a thread of ours waits for that
	 * and copies it there, and whoever made the frame is back at the next one meanwhile. One
	 * frame waits at a time: the swap after it is held, in SwapEnd, until this one is being
	 * copied to the panel. Whoever draws is so at most two frames ahead of the panel, and
	 * needs a third surface to be that: CoreAnimation makes itself a new one for every frame.
	 * By default two frames may wait (boot-arg fb_queue=1: one, as before): CoreAnimation draws on a
	 * clock of its own, and a SwapEnd held past its next tick makes that tick late, the clock going
	 * on from there (rc.jobs.fps: 28% of the frames over 16.7 ms with two, 33% with one).
	 */
	IOLock *        fSwapLock;              /* around the next five */
	uint8_t         fSwap[SWAP_QUEUE][SWAP_SIZE];   /* the swaps on their way to the panel (fb_queue), from fSwapFirst on */
	MiniIOSurface * fSwapSurfaces[SWAP_QUEUE][SWAP_LAYERS]; /* ... their surfaces, held from SwapEnd on: their maker may let go of them */
	unsigned        fSwapFirst, fSwapsWaiting;
	/*
	 * ... and not whenever the GPU happens to be done, but on a beat of the refresh rate
	 * (boot-arg fb_beat=0: at once): frames then follow each other at an even pace whatever
	 * each took to draw, as long as there is one for every beat. The beat is this driver's own.
	 * (CoreAnimation does not go by the refresh notifications of this driver -- it never asks
	 * for them -- but by a clock of its own at the rate it reads here; and where the panel's
	 * scan is nobody can read: the display block's register clock is off.) It starts with a
	 * frame that comes later than a beat and a half after the one before -- the first after a
	 * pause, or every one of a renderer that does not keep up, the software one: such a frame
	 * is shown at once, a beat would only make it later still -- and gives way by an eighth of
	 * what a frame is early or late, so that frames are ready some six milliseconds before
	 * their beat: neither a slow one nor a quick one then lands on the wrong side of it.
	 */
	uint64_t        fBeat;                  /* the beat the last frame was shown on */
	uint64_t        fReady;                 /* when the last frame was ready to be shown */
	uint32_t        fStatOnBeat;            /* frames that waited for a refresh, and for how long together */
	uint64_t        fStatBeatWait;
	uint32_t        fStatSurfaces[8];       /* the first different surfaces the bottom layer was given */
	uint32_t        fStatHeld, fStatLost;   /* swaps that waited for the one before; that gave up waiting */
	uint64_t        fSwapTime;              /* when the last swap was handed over */
	uint32_t        fStatSwapsLate;         /* swaps that came more than a refresh and a quarter after the one before (and no pause) */
	uint64_t        fStatWaiting;           /* how long frames waited for the GPU here */
	struct {
		uint32_t sid, drawn;
	}               fShown[SHOWN_SLOTS];    /* show(): the GPU submission each surface was last shown after */
	unsigned        fShownNext;
	uint32_t        fStatStale, fStatStaleShown; /* swaps of a surface not drawn again since shown; still not after SWAP_NEW_MS */
	/* how the display is doing: frames and time spent drawing them here since fStatSince */
	uint32_t        fStatFrames;
	uint64_t        fStatSince, fStatDrawing;
	/* ... and how evenly: the time from each frame to the next, counted by the millisecond (the last count: longer) */
	uint64_t        fLastFrame;
	uint32_t        fFrameTimes[FRAME_TIMES + 1], fFramePauses, fFrameLongest;
	/* boot-arg fb_diff=1: which pixels of the panel each frame changed (it costs: the panel's memory is slow to read) */
	bool            fDiff;
	uint64_t        fDiffPixels;
	uint32_t        fDiffLeft, fDiffTop, fDiffRight, fDiffBottom;
	/* the screen recording, if one was asked for */
	bool            fOver;                  /* fb_yuv: the layer drawn now goes over what is there, by its alpha */
	uint32_t *      fColumn;                /* drawLayer(): source column for every panel column */
	uint16_t *      fColumnWeight;          /* ... with fb_filter: how much of the column after it, of 256 */
	uint32_t *      fBlendRow;              /* ... and a row of the picture blended from two (fBlendRowSize pixels) */
	uint32_t        fBlendRowSize;
	uint8_t *       fRecBase;
	IOBufferMemoryDescriptor * fRecMemory;  /* (whose bytes those are: kept for as long as the kernel runs, the loader reads them after) */
	uint64_t        fRecSize, fRecUsed;
	uint64_t        fFirstFrame, fRecFirst; /* when the first frame was shown, and the first recorded */
	bool            fRecDone;
	bool            fRecWrapped;            /* (rec_ring) the memory has been full once: frames go over the oldest */

	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
	void publishModes(void);
	MiniIOSurface * defaultSurface(void);
	void present(const uint8_t * swap);
	void show(const uint8_t * swap, MiniIOSurface ** surfaces);
	bool showOnGPU(const uint8_t * swap, MiniIOSurface ** surfaces, int * target);
	void checkOnGPU(void);
	void panelForProcessor(void);
	bool flipSetUp(void);
	bool flipTaken(uint32_t milliseconds);
	void flipTo(int buffer);
	void flipOff(const char * why);
	void flipHome(void);
	bool videoSetUp(void);
	bool videoTake(MiniIOSurface * s, const int32_t * src, int64_t qx0, int64_t qy0, int64_t qx1, int64_t qy1, int target,
	    struct MiniIOSurfacePresentVideo * v);
	uint32_t videoProgram(int buffer);
	void presenter(void);
	static void presenterMain(void * self, wait_result_t result);
	void reportFrameTimes(void);
	void drawLayer(MiniIOSurface * s, const int32_t * src, const int32_t * dst);
	bool drawYUV(MiniIOSurface * s, const int32_t * src, const int32_t * dst);
	void drawRows(const struct RowJob * job, int32_t y0, int32_t y1, uint32_t * blendRow);
	void drawShare(const struct RowJob * job, unsigned share, uint32_t * blendRow);
	static void helperMain(void * self, wait_result_t result);
	void copyBackBuffer(void);
	void redrawForLight(void);
	static void lightChanged(void);
	static void lightCall(thread_call_param_t self, thread_call_param_t unused);
	thread_call_t   fLightCall;             /* (backend soft) the copy shown again at a new light */
	/* the scaling shared out (fb_threads): fThreads drawers, the presenter and fThreads - 1 helpers */
	unsigned        fThreads;
	IOLock *        fHelperLock;
	const struct RowJob * fHelperJob;
	uint64_t        fHelperGeneration;
	unsigned        fHelpersBusy, fHelpersStarted;
	uint32_t *      fHelperBlend[7];
	/* the panel's readout this frame is written for (0: no model, write at once), and how far ahead */
	uint64_t        fScanTarget, fScanLastTarget;
	uint64_t        fScanLead;
	volatile uint32_t fScanLate;            /* bands of the frame written too late for their readout */
	uint32_t        fScanCleanFrames, fStatScanLate, fStatScanFrames;
	void scanCalibrate(void);
	void record(MiniIOSurface * s);
};

OSDefineMetaClassAndStructors(IOMobileFramebuffer, IOService);

static IOMobileFramebuffer * gLightFramebuffer;      /* (backend soft) whom a new light is shown by */
static IOMobileFramebuffer * gFlipFramebuffer;       /* fb_flip ready: hakios_fb_quiesce() brings its controller home */

static void
setNumber(OSDictionary * dict, const char * key, uint64_t value)
{
	OSNumber * n = OSNumber::withNumber(value, 32);

	if (n != NULL) {
		dict->setObject(key, n);
		n->release();
	}
}

/*
 * The modes of the display, the way CoreAnimation reads them (the properties TimingElements
 * and ColorElements; without a mode it draws nothing). One timing, the panel's size at
 * 60 Hz (boot-arg fb_hz=N: at N), with one colour mode, 8-bit RGB. The identifiers are what "the current mode"
 * (block 0x40) answers with.
 */
#define kFBTimingID     1
#define kFBColorID      1

void
IOMobileFramebuffer::publishModes(void)
{
	/*
	 * Darwin 26 takes the modes the way a real phone's display driver lays them out (an M1's AppleCLCD2,
	 * read with ioreg, is the model here): besides the attributes, each element carries its binary
	 * description (ElementData), and the timing says it is the preferred one. With only the attributes
	 * Darwin 16.5 was content; Darwin 26 found no preferred mode ("preferred mode (0 x 0)", current mode
	 * 16383 x 16383) and never drew. The porches are made up, the pixel clock follows from them.
	 */
	const uint32_t hz = refreshHz();
	const uint32_t hSync = 32, hBack = 42, hFront = 8, vSync = 8, vBack = 42, vFront = 32;
	const uint32_t hTotal = fWidth + hSync + hBack + hFront, vTotal = fHeight + vSync + vBack + vFront;
	const uint64_t pixelClock = (uint64_t) hTotal * vTotal * hz;
	const uint32_t lineRate = (uint32_t) (((pixelClock << 16) / hTotal) / 1000);     /* kHz, 16.16 */
	const uint32_t frameRate = hz << 16;                                            /* Hz, 16.16 */
	uint32_t timingData[20] = {
		0, hTotal, fWidth, hSync, hBack, hFront, lineRate, 1, 0,
		vTotal, fHeight, vSync, vBack, vFront, frameRate, 1, 0,
		(uint32_t) pixelClock, 13, 0,
	};
	uint8_t colorData[32] = { 8 };
	OSDictionary * timing = OSDictionary::withCapacity(24);
	OSDictionary * horizontal = OSDictionary::withCapacity(9);
	OSDictionary * vertical = OSDictionary::withCapacity(9);
	OSDictionary * color = OSDictionary::withCapacity(12);
	OSDictionary * refresh = OSDictionary::withCapacity(6);
	OSDictionary * attributes = OSDictionary::withCapacity(1);
	OSDictionary * product = OSDictionary::withCapacity(1);
	OSArray * timings = OSArray::withCapacity(1);
	OSArray * colors = OSArray::withCapacity(1);
	OSArray * none = OSArray::withCapacity(1);
	OSData * data;
	uint64_t frameTicks;

	if (timing == NULL || horizontal == NULL || vertical == NULL || color == NULL || refresh == NULL ||
	    attributes == NULL || product == NULL || timings == NULL || colors == NULL || none == NULL) {
		goto out;                       /* (what was made is let go again) */
	}
	colorData[12] = 16;                                     /* colorimetry */
	setNumber(color, "ID", kFBColorID);
	setNumber(color, "ElementType", 1);
	setNumber(color, "StandardType", 0);
	setNumber(color, "PixelEncoding", 0);                   /* RGB 4:4:4 */
	setNumber(color, "Depth", 8);
	setNumber(color, "Colorimetry", 16);
	setNumber(color, "EOTF", 0);
	setNumber(color, "DynamicRange", 0);
	setNumber(color, "SupportsDSC", 0);
	setNumber(color, "Score", 240999);
	color->setObject("IsVirtual", kOSBooleanFalse);
	if ((data = OSData::withBytes(colorData, sizeof(colorData))) != NULL) {
		color->setObject("ElementData", data);
		data->release();
	}
	colors->setObject(color);

	setNumber(horizontal, "Active", fWidth);
	setNumber(horizontal, "Total", hTotal);
	setNumber(horizontal, "SyncWidth", hSync);
	setNumber(horizontal, "BackPorch", hBack);
	setNumber(horizontal, "FrontPorch", hFront);
	setNumber(horizontal, "SyncPolarity", 1);
	setNumber(horizontal, "PixelRepetition", 0);
	setNumber(horizontal, "SyncRate", lineRate);
	setNumber(horizontal, "PreciseSyncRate", lineRate);
	setNumber(vertical, "Active", fHeight);
	setNumber(vertical, "Total", vTotal);
	setNumber(vertical, "SyncWidth", vSync);
	setNumber(vertical, "BackPorch", vBack);
	setNumber(vertical, "FrontPorch", vFront);
	setNumber(vertical, "SyncPolarity", 1);
	setNumber(vertical, "PixelRepetition", 0);
	setNumber(vertical, "SyncRate", frameRate);
	setNumber(vertical, "PreciseSyncRate", frameRate);
	setNumber(timing, "ID", kFBTimingID);
	setNumber(timing, "ElementType", 0);
	setNumber(timing, "StandardType", 0);
	setNumber(timing, "TimingType", 3);
	setNumber(timing, "ScanInformation", 0);
	setNumber(timing, "Score", 16346);
	setNumber(timing, "AspectRatio", 13);
	setNumber(timing, "PreciseAspectRatio", (uint64_t) fWidth * 65536 / fHeight);
	setNumber(timing, "ValidPixelEncodings", 1);
	setNumber(timing, "MinimumVariableRefreshRate", 0);
	setNumber(timing, "MaximumVariableRefreshRate", 0);
	timing->setObject("IsPreferred", kOSBooleanTrue);
	timing->setObject("IsVirtual", kOSBooleanFalse);
	timing->setObject("IsOverscanned", kOSBooleanFalse);
	timing->setObject("IsInterlaced", kOSBooleanFalse);
	timing->setObject("IsSplit", kOSBooleanFalse);
	timing->setObject("IsPromoted", kOSBooleanFalse);
	timing->setObject("HorizontalAttributes", horizontal);
	timing->setObject("VerticalAttributes", vertical);
	timing->setObject("ColorModes", colors);
	timing->setObject("DiscreteMediaRefreshRates", none);
	timing->setObject("UnsafeColorElementIDs", none);
	timing->setObject("DSCRequiredColorElementIDs", none);
	if ((data = OSData::withBytes(timingData, sizeof(timingData))) != NULL) {
		timing->setObject("ElementData", data);
		data->release();
	}
	timings->setObject(timing);

	setProperty("TimingElements", timings);
	setProperty("ColorElements", colors);
	setProperty("DPTimingModeId", kFBTimingID, 32);
	setProperty("DisplayWidth", fWidth, 32);
	setProperty("DisplayHeight", fHeight, 32);
	setProperty("PixelClock", pixelClock, 64);
	attributes->setObject("ProductAttributes", product);
	setProperty("DisplayAttributes", attributes);
	nanoseconds_to_absolutetime(NSEC_PER_SEC / hz, &frameTicks);
	setNumber(refresh, "displayMinRefreshInterval", (1ULL << 32) / hz);
	setNumber(refresh, "displayMaxRefreshInterval", (1ULL << 32) / hz);
	setNumber(refresh, "displayMinRefreshIntervalMachTime", frameTicks);
	setNumber(refresh, "displayMaxRefreshIntervalMachTime", frameTicks);
	setNumber(refresh, "displayRefreshStep", 0);
	setNumber(refresh, "displayRefreshStepMachTime", 0);
	setProperty("IOMFBDisplayRefresh", refresh);
out:
	OSSafeReleaseNULL(timings);
	OSSafeReleaseNULL(colors);
	OSSafeReleaseNULL(none);
	OSSafeReleaseNULL(timing);
	OSSafeReleaseNULL(horizontal);
	OSSafeReleaseNULL(vertical);
	OSSafeReleaseNULL(color);
	OSSafeReleaseNULL(refresh);
	OSSafeReleaseNULL(attributes);
	OSSafeReleaseNULL(product);
}

bool
IOMobileFramebuffer::start(IOService * provider)
{
	thread_t thread;

	if (!IOService::start(provider)) {
		return false;
	}
	if (vinfo.v_baseaddr == 0 || vinfo.v_width == 0 || vinfo.v_height == 0 || vinfo.v_depth != 32) {
		IOLog("MiniIOMobileFramebuffer: no usable boot framebuffer (%ux%u, depth %u)\n", vinfo.v_width, vinfo.v_height,
		    vinfo.v_depth);
		return false;
	}
	fLock = IOLockAlloc();
	fSwapLock = IOLockAlloc();
	if (kernel_thread_start(&IOMobileFramebuffer::presenterMain, this, &thread) != KERN_SUCCESS) {
		IOLog("MiniIOMobileFramebuffer: no thread to put frames on the panel with\n");
		return false;
	}
	thread_set_thread_name(thread, "frames to the panel");
	thread_deallocate(thread);
	fThreads = gThreads >= 1 && gThreads <= 8 ? gThreads : 1;
	if (gSmallCores && hakios_slow_cores() >= 2 && fThreads > hakios_slow_cores()) {
		fThreads = hakios_slow_cores();
	}
	fHelperLock = IOLockAlloc();
	for (unsigned i = 0; i + 1 < fThreads; i++) {
		if (kernel_thread_start(&IOMobileFramebuffer::helperMain, this, &thread) != KERN_SUCCESS) {
			fThreads = i + 1;
			break;
		}
		thread_set_thread_name(thread, "frames to the panel: a helper");
		thread_deallocate(thread);
	}
	fBase = (uint8_t *) vinfo.v_baseaddr;
	fPanelWidth = vinfo.v_width;
	fPanelHeight = vinfo.v_height;
	fRowBytes = vinfo.v_rowbytes;
	fFlipShown = -1;
	/*
	 * fb_gpu: where the panel's memory is. Not taken on trust from the console's v_physaddr: the mapping is asked, at its
	 * first byte and its last, which must be as far apart as in the kernel's view -- one piece of memory, as the boot
	 * loader's framebuffer is.
	 */
	{
		uint64_t last = (uint64_t) fRowBytes * fPanelHeight - 1;
		uint64_t first = ml_vtophys((vm_offset_t) fBase), end = ml_vtophys((vm_offset_t) fBase + last);

		fPanelPhys = first != 0 && end == first + last ? first : 0;
		gPanelPhys = first;
		if (gGPUPresent && fPanelPhys == 0) {
			IOLog("MiniIOMobileFramebuffer: fb_gpu: the panel's memory is not one piece (%#llx, and %#llx at its end): "
			    "frames by the processors\n", first, end);
		}
	}
	uint32_t width = kFBDefaultWidth, height = kFBDefaultHeight, ownScale = kFBDefaultScale;
	{
		IORegistryEntry * chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);
		OSData * size = chosen ? OSDynamicCast(OSData, chosen->getProperty("display-size")) : NULL;
		OSData * scale = chosen ? OSDynamicCast(OSData, chosen->getProperty("display-scale")) : NULL;

		if (size != NULL && size->getLength() == 2 * sizeof(uint32_t)) {
			const uint32_t * pixels = (const uint32_t *) size->getBytesNoCopy();
			width = pixels[0];
			height = pixels[1];
		}
		if (scale != NULL && scale->getLength() == sizeof(uint32_t) && *(const uint32_t *) scale->getBytesNoCopy() != 0) {
			ownScale = *(const uint32_t *) scale->getBytesNoCopy();
		}
		OSSafeReleaseNULL(chosen);
	}
	if (gFBWidth != kFBNotGiven) {
		width = gFBWidth;
	}
	if (gFBHeight != kFBNotGiven) {
		height = gFBHeight;
	}
	fWidth = width ? width : fPanelWidth;
	fHeight = height ? height : fPanelHeight;
	if (gFBScale != 0 && gFBScale != ownScale) {
		static const char * const places[][2] = { { "/chosen", "display-scale" }, { "/product", "artwork-scale-factor" } };
		uint32_t scale = gFBScale;

		fWidth = fWidth / ownScale * scale;
		fHeight = fHeight / ownScale * scale;
		/* what the loader's device tree says about the scale is where MobileGestalt has it from */
		for (unsigned i = 0; i < sizeof(places) / sizeof(places[0]); i++) {
			IORegistryEntry * node = IORegistryEntry::fromPath(places[i][0], gIODTPlane);

			if (node != NULL) {
				node->setProperty(places[i][1], &scale, sizeof(scale));
				node->release();
			}
		}
	}
	fColumn = IONewZero(uint32_t, fPanelWidth);
	fColumnWeight = IONewZero(uint16_t, fPanelWidth);
	fDiff = gDiff != 0;
	fDiffLeft = fDiffTop = ~0u;
	printf("MiniIOMobileFramebuffer: display %ux%u on a panel of %ux%u (%u bytes per row) at %#llx%s\n", fWidth, fHeight,
	    fPanelWidth, fPanelHeight, fRowBytes, fPanelPhys, gGPUPresent ? "; frames to it by the GPU where it can (fb_gpu)" : "");
	/* (the frame: rows of 4 bytes a pixel, padded to 64 bytes as IOSurface pads them, and a little to spare) */
	fScanLead = usToAbs(gScanLeadUs >= 1000 && gScanLeadUs <= 8000 ? gScanLeadUs : 2000);
	if (gScan) {
		scanCalibrate();
	}
	if (gDoubleBuffer || hakios_backlight_soft()) {
		vm_size_t backBufferSize = (vm_size_t) fPanelHeight * fRowBytes;
		IOBufferMemoryDescriptor * memory = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
		    kIODirectionInOut, backBufferSize, 64);
		if (memory != NULL) {
			fBackBufferDescriptor = memory;
			fBackBuffer = (uint8_t *) memory->getBytesNoCopy();
			bzero(fBackBuffer, backBufferSize);
			IOLog("MiniIOMobileFramebuffer: backbuffer allocated (%u KB) for double buffering\n",
			    (unsigned) (backBufferSize >> 10));
		} else {
			IOLog("MiniIOMobileFramebuffer: backbuffer allocation failed, using direct framebuffer\n");
			fBackBuffer = NULL;
			fBackBufferDescriptor = NULL;
		}
	} else {
		fBackBuffer = NULL;
		fBackBufferDescriptor = NULL;
	}
	if (fBackBuffer != NULL && hakios_backlight_soft()) {
		fLightCall = thread_call_allocate(&IOMobileFramebuffer::lightCall, this);
		gLightFramebuffer = this;
		hakios_backlight_set_redraw(&IOMobileFramebuffer::lightChanged);
		IOLog("MiniIOMobileFramebuffer: the panel's light is made here (hw.backlight, backend soft)\n");
	}

	if (gRecSeconds != 0) {
		/* one piece of physical memory: the loader will send it over USB as it lies */
		IOBufferMemoryDescriptor * memory = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut | kIOMemoryPhysicallyContiguous, (mach_vm_size_t) gRecMegabytes << 20, ~0ULL);

		if (memory != NULL) {
			struct rec_header * header = (struct rec_header *) memory->getBytesNoCopy();

			fRecMemory = memory;
			fRecBase = (uint8_t *) header;
			fRecSize = (uint64_t) gRecMegabytes << 20;
			fRecUsed = sizeof(*header);
			bzero(header, sizeof(*header));
			memcpy(header->magic, "RWREC003", 8);
			header->width = fWidth / REC_STEP;
			header->height = fHeight / REC_STEP;
			loader_recording[0] = memory->getPhysicalSegment(0, NULL);
			loader_recording[1] = fRecUsed;
			loader_recording[2] = (uint64_t) (uintptr_t) fRecBase;
			IOLog("MiniIOMobileFramebuffer: recording %u s of the screen at %ux%u from second %u, into %u MB at %#llx%s\n",
			    gRecSeconds, header->width, header->height, gRecStart, gRecMegabytes, loader_recording[0],
			    gRecRing ? ", the last of it kept (rec_ring)" : "");
		} else {
			IOLog("MiniIOMobileFramebuffer: no memory for a recording of %u MB\n", gRecMegabytes);
		}
	}

	setProperty(kIOUserClientClassKey, "IOMobileFramebufferUserClient");
	/* for who draws with the GPU (userland/src/eagl.c): a frame need not be waited for before it is handed over */
	setProperty("HakiosWaitsForGPU", kOSBooleanTrue);
	publishModes();
	setName("disp0");
	registerService();
	return true;
}

/* Keep a small copy of the frame that is about to be shown (the surface is the whole screen). */
void
IOMobileFramebuffer::record(MiniIOSurface * s)
{
	struct rec_header * header = (struct rec_header *) fRecBase;
	uint64_t now = mach_absolute_time(), since_first, since_start = 0, frame_size;
	struct rec_frame * frame;
	uint8_t * to;

	if (fFirstFrame == 0) {
		fFirstFrame = now;
	}
	if (fRecBase == NULL || fRecDone || s->info.width < fWidth || s->info.height < fHeight) {
		return;
	}
	absolutetime_to_nanoseconds(now - fFirstFrame, &since_first);
	if (since_first < (uint64_t) gRecStart * NSEC_PER_SEC) {
		return;
	}
	if (fRecFirst == 0) {
		uint64_t began;

		fRecFirst = now;
		absolutetime_to_nanoseconds(now, &began);
		header->began = began;
		IOLog("MiniIOMobileFramebuffer: the recording begins at %llu.%03llu s of the kernel's time (mach_absolute_time)\n",
		    began / NSEC_PER_SEC, began / NSEC_PER_MSEC % 1000);
	}
	absolutetime_to_nanoseconds(now - fRecFirst, &since_start);
	frame_size = sizeof(*frame) + (uint64_t) header->width * header->height * 3;
	if (since_start > (uint64_t) gRecSeconds * NSEC_PER_SEC || (fRecUsed + frame_size > fRecSize && !gRecRing)) {
		fRecDone = true;
		IOLog("MiniIOMobileFramebuffer: recording finished: %u frames, %llu MB\n", header->frames, fRecUsed >> 20);
		return;
	}
	if (fRecUsed + frame_size > fRecSize) {
		/* (rec_ring) round again: over the oldest, which is the next one */
		fRecUsed = sizeof(*header);
		fRecWrapped = true;
	}
	frame = (struct rec_frame *) (fRecBase + fRecUsed);
	/*
	 * The surface it came from and the low bits of its last GPU submission ride in the time's top bits -- a run is far
	 * shorter than 2^40 ns -- for telling which buffer a frame that flickers was: tools/recording.py writes them to tags.txt
	 */
	frame->nanoseconds = (since_start & ((1ull << 40) - 1)) | (uint64_t) (s->sid & 0xffff) << 40 | (uint64_t) (s->drawn & 0xff) << 56;
	to = (uint8_t *) (frame + 1);
	/* (a frame of another surface than the first, smaller, or one whose rows are not in its memory: not recorded) */
	if (header->width == 0 || header->height == 0 ||
	    !s->holds((uint64_t) ((header->height - 1) * REC_STEP + 1) * s->info.bytesPerRow,
	    (uint64_t) ((header->width - 1) * REC_STEP + 2) * 4)) {
		return;
	}
	for (uint32_t y = 0; y < header->height; y++) {
		const uint32_t * from = (const uint32_t *) (s->pixels() + (size_t) (y * REC_STEP + 1) * s->info.bytesPerRow);

		for (uint32_t x = 0; x < header->width; x++) {
			uint32_t pixel = from[x * REC_STEP + 1];        /* B G R A in memory */

			*to++ = (uint8_t) (pixel >> 16);
			*to++ = (uint8_t) (pixel >> 8);
			*to++ = (uint8_t) pixel;
		}
	}
	fRecUsed += frame_size;
	if (!fRecWrapped) {
		header->frames++;
		loader_recording[1] = fRecUsed;
	} else {
		/* the memory full of frames: all of them are handed over, from the one after this */
		header->oldest = (uint32_t) ((fRecUsed - sizeof(*header)) / frame_size);
		if (header->oldest >= header->frames) {
			header->oldest = 0;
		}
		loader_recording[1] = sizeof(*header) + (uint64_t) header->frames * frame_size;
	}
}

MiniIOSurface *
IOMobileFramebuffer::defaultSurface(void)
{
	IOLockLock(fLock);
	if (fDefaultSurface == NULL) {
		fDefaultSurface = MiniIOSurfaceCreateSimple(fWidth, fHeight, FOURCC('B', 'G', 'R', 'A'));
	}
	IOLockUnlock(fLock);
	return fDefaultSurface;
}

/*
 * Two pixels mixed: `w` of 256 of the second. Two of the four channels at a time, in one word.
 */
static inline uint32_t
blend(uint32_t a, uint32_t b, uint32_t w)
{
	uint32_t rb = ((a & 0xff00ff) * (256 - w) + (b & 0xff00ff) * w) >> 8 & 0xff00ff;
	uint32_t ga = ((a >> 8 & 0xff00ff) * (256 - w) + (b >> 8 & 0xff00ff) * w) & 0xff00ff00;

	return rb | ga;
}

/* A premultiplied pixel `s` over `d` (fb_yuv). */
static inline uint32_t
over(uint32_t s, uint32_t d)
{
	uint32_t a = s >> 24, k = 255 - a;

	if (a == 255) {
		return s;
	}
	if (a == 0 && (s & 0xffffff) == 0) {
		return d;
	}
	uint32_t rb = (s & 0xff00ff) + (((d & 0xff00ff) * k + 0x800080) >> 8 & 0xff00ff);
	uint32_t ga = (s >> 8 & 0xff00ff) + (((d >> 8 & 0xff00ff) * k + 0x800080) >> 8 & 0xff00ff);

	/* (premultiplied: a channel is at most the alpha, so each sum stays within its byte) */
	return (rb & 0xff00ff) | (ga & 0xff00ff) << 8;
}

/*
 * Where the middle of the `i`th of `out` pixels falls among `in` of them, in 256ths of a pixel,
 * kept within the first and the last.
 */
static inline int64_t
sample_at(int64_t i, int64_t in, int64_t out)
{
	int64_t at = ((2 * i + 1) * in - out) * 128 / out;

	return at < 0 ? 0 : at > (in - 1) * 256 ? (in - 1) * 256 : at;
}

/*
 * The rows of one layer, from `y0` to `y1` (panel rows), as drawLayer() has worked them out:
 * scaling a whole frame takes one processor some 9 ms, so with boot-arg fb_threads=N it is shared
 * out between N of them, each with rows of its own and a row of blending of its own.
 */
struct RowJob {
	const uint8_t * pixels;
	uint32_t        stride;
	int32_t         sx, sy, sw, sh, px0, py0, px1, py1, cx0, cx1;
	int32_t         cy0, cy1;               /* the rows of [py0, py1) that are on the panel (as cx0, cx1 its columns) */
	bool            filter, unscaled, over;
	uint64_t        target;                 /* the panel's readout it is for (scanWaitRows()) */
};

void
IOMobileFramebuffer::drawRows(const RowJob * job, int32_t y0, int32_t y1, uint32_t * blendRow)
{
	const uint8_t * pixels = job->pixels;
	uint32_t stride = job->stride;
	int32_t sx = job->sx, sy = job->sy, sw = job->sw, sh = job->sh;
	int32_t px0 = job->px0, py0 = job->py0, py1 = job->py1, cx0 = job->cx0, cx1 = job->cx1;
	bool filter = job->filter;
	bool timely = scanWaitRows(y0 > 0 ? y0 : 0, y1, job->target);

	for (int32_t y = y0; y < y1; y++) {
		const uint32_t * from;
		uint32_t * to;

		if (y < 0 || (uint32_t) y >= fPanelHeight) {
			continue;
		}
		to = (uint32_t *) ((fBackBuffer ? fBackBuffer : fBase) + (size_t) y * fRowBytes);
		if (job->unscaled) {
			/* not scaled at all */
			if (job->over) {
				const uint32_t * row = (const uint32_t *) (pixels + (size_t) (sy + y - py0) * stride) + sx;

				for (int32_t x = 0; x < sw; x++) {
					to[px0 + x] = over(row[x], to[px0 + x]);
				}
				continue;
			}
			memcpy(to + px0, pixels + (size_t) (sy + y - py0) * stride + (size_t) sx * 4, (size_t) sw * 4);
			continue;
		}
		if (filter) {
			int64_t at = sample_at(y - py0, sh, py1 - py0);
			uint32_t row = (uint32_t) (at >> 8), weight = (uint32_t) (at & 255);
			const uint32_t * upper = (const uint32_t *) (pixels + (size_t) (sy + row) * stride) + sx;

			if (weight == 0 || row + 1 >= (uint32_t) sh) {
				from = upper;
			} else {
				const uint32_t * lower = (const uint32_t *) (pixels + (size_t) (sy + row + 1) * stride) + sx;
				uint32_t first = fColumn[cx0], last = fColumn[cx1 - 1] + 1 < (uint32_t) sw ? fColumn[cx1 - 1] + 1 : fColumn[cx1 - 1];

				uint32_t * __restrict mixed = blendRow;

				for (uint32_t c = first; c <= last; c++) {
					uint32_t a = upper[c], b = lower[c];

					mixed[c] = a == b ? a : blend(a, b, weight);           /* (most of a screen is the same as next to it) */
				}
				from = blendRow;
			}
		} else {
			from = (const uint32_t *) (pixels + (size_t) (sy + (int64_t) (y - py0) * sh / (py1 - py0)) * stride);
		}
		if (fDiff) {
			for (int32_t x = cx0; x < cx1; x++) {
				uint32_t value = filter ? blend(from[fColumn[x]], from[fColumn[x] + (fColumnWeight[x] != 0)], fColumnWeight[x]) :
				    from[fColumn[x]];

				if (to[x] != value) {
					fDiffPixels++;
					fDiffLeft = (uint32_t) x < fDiffLeft ? (uint32_t) x : fDiffLeft;
					fDiffRight = (uint32_t) x > fDiffRight ? (uint32_t) x : fDiffRight;
					fDiffTop = (uint32_t) y < fDiffTop ? (uint32_t) y : fDiffTop;
					fDiffBottom = (uint32_t) y > fDiffBottom ? (uint32_t) y : fDiffBottom;
				}
			}
		}
		if (filter) {
			const uint32_t * __restrict column = fColumn + cx0;
			const uint16_t * __restrict weights = fColumnWeight + cx0;
			uint32_t * __restrict out = to + cx0;

			for (uint32_t i = 0, n = (uint32_t) (cx1 - cx0); i < n; i++) {
				uint32_t c = column[i], w = weights[i], a = from[c];
				uint32_t value = w == 0 || a == from[c + 1] ? a : blend(a, from[c + 1], w);

				out[i] = job->over ? over(value, out[i]) : value;
			}
		} else if (job->over) {
			for (int32_t x = cx0; x < cx1; x++) {
				to[x] = over(from[fColumn[x]], to[x]);
			}
		} else {
			for (int32_t x = cx0; x < cx1; x++) {
				to[x] = from[fColumn[x]];
			}
		}
	}
	if (!timely || !scanInTime(y0 > 0 ? y0 : 0, job->target)) {
		__atomic_fetch_add(&fScanLate, 1, __ATOMIC_RELAXED);
	}
}

/*
 * Share `share` of fThreads of a layer's rows. Written for a readout of the panel (job->target), the
 * shares are stripes taken in turn, so that all the drawers go down the frame together, top first,
 * ahead of the readout that comes down after them: the top of the frame is done almost at once and
 * the frame needs to be begun only a little before its readout. Without, each share is one block.
 */
#define SCAN_STRIPE     16

void
IOMobileFramebuffer::drawShare(const RowJob * job, unsigned share, uint32_t * blendRow)
{
	int32_t rows = job->cy1 - job->cy0;

	if (job->target == 0) {
		drawRows(job, job->cy0 + (int32_t) ((int64_t) rows * share / fThreads),
		    job->cy0 + (int32_t) ((int64_t) rows * (share + 1) / fThreads), blendRow);
		return;
	}
	for (int32_t y = job->cy0 + (int32_t) share * SCAN_STRIPE; y < job->cy1; y += (int32_t) fThreads * SCAN_STRIPE) {
		drawRows(job, y, y + SCAN_STRIPE < job->cy1 ? y + SCAN_STRIPE : job->cy1, blendRow);
	}
}

/* A helper of the scaling (fb_threads): waits for rows to do, does them, says so. */
void
IOMobileFramebuffer::helperMain(void * self, wait_result_t result __unused)
{
	IOMobileFramebuffer * fb = (IOMobileFramebuffer *) self;
	unsigned index;
	uint64_t done = 0;
	bool bound = false;

	IOLockLock(fb->fHelperLock);
	index = fb->fHelpersStarted++;
	IOLockUnlock(fb->fHelperLock);
	for (;;) {
		const RowJob * job;
		uint32_t * blend;

		IOLockLock(fb->fHelperLock);
		while (fb->fHelperGeneration == done) {
			IOLockSleep(fb->fHelperLock, &fb->fHelperGeneration, THREAD_UNINT);
		}
		done = fb->fHelperGeneration;
		job = fb->fHelperJob;
		IOLockUnlock(fb->fHelperLock);
		if (!bound) {
			bound = true;           /* (at the first frame: every core is up by then) */
			bindDrawer(index + 1, "a helper");
		}
		if (job->pixels == NULL) {
			int32_t y0 = (int32_t) ((int64_t) fb->fPanelHeight * (index + 1) / fb->fThreads);
			int32_t y1 = (int32_t) ((int64_t) fb->fPanelHeight * (index + 2) / fb->fThreads);
			if (y1 > y0 && fb->fBackBuffer != NULL && fb->fBase != NULL) {
				bool timely = scanWaitRows(y0, y1, job->target);

				toPanel(fb->fBase + (size_t) y0 * fb->fRowBytes, fb->fBackBuffer + (size_t) y0 * fb->fRowBytes, (size_t) (y1 - y0) * fb->fRowBytes);
				if (!timely || !scanInTime(y0, job->target)) {
					__atomic_fetch_add(&fb->fScanLate, 1, __ATOMIC_RELAXED);
				}
			}
		} else {
			blend = fb->fHelperBlend[index];
			fb->drawShare(job, index + 1, blend);
		}
		IOLockLock(fb->fHelperLock);
		fb->fHelpersBusy--;
		if (fb->fHelpersBusy == 0) {
			IOLockWakeup(fb->fHelperLock, &fb->fHelpersBusy, true);
		}
		IOLockUnlock(fb->fHelperLock);
	}
}

/*
 * The model of the panel's readouts (gPanel), from its counters, watched for up to a tenth of a second:
 * two lines of one readout give the time a line takes, the beginnings of two readouts the period.
 * Anything that does not look like a command-mode panel's readout leaves the model off, and this
 * driver as it was without one.
 */
void
IOMobileFramebuffer::scanCalibrate(void)
{
	uint64_t deadline = mach_absolute_time() + usToAbs(100000), at[2] = { 0, 0 }, start = 0, now;
	uint32_t frame, line, frames[2] = { 0, 0 }, lines[2] = { 0, 0 }, firstFrame = 0;
	bool counting = false;

	if (!hakios_display_scan_start()) {
		printf("MiniIOMobileFramebuffer: panel scan: no readout counters on this board: frames written at once\n");
		return;
	}
	gPanel.lines = fPanelHeight;
	while ((now = mach_absolute_time()) < deadline) {
		if (!hakios_display_scan(&frame, &line)) {
			return;
		}
		now = mach_absolute_time();
		if (line >= 100 && line + 100 < fPanelHeight) {
			if (!counting || frame != frames[0] || line < lines[0]) {
				at[0] = now, lines[0] = line, frames[0] = frame, counting = true;    /* (a readout of its own) */
			} else if (line >= lines[0] + 400) {
				at[1] = now, lines[1] = line, frames[1] = frame;
				gPanel.line_ticks = (at[1] - at[0]) / (lines[1] - lines[0]);
				start = at[1] - (uint64_t) lines[1] * gPanel.line_ticks;
				firstFrame = frame;
				break;
			}
		}
		IODelay(40);
	}
	if (start == 0) {
		printf("MiniIOMobileFramebuffer: panel scan: no readout seen in 100 ms: frames written at once\n");
		return;
	}
	/* the beginning of a later readout, for the period */
	while ((now = mach_absolute_time()) < deadline) {
		if (!hakios_display_scan(&frame, &line)) {
			return;
		}
		now = mach_absolute_time();
		if (frame != firstFrame && line >= 100 && line + 100 < fPanelHeight) {
			uint64_t later = now - (uint64_t) line * gPanel.line_ticks, period = (later - start) / (frame - firstFrame);
			uint64_t ns;

			absolutetime_to_nanoseconds(period, &ns);
			/* (a command-mode panel's readout takes half a refresh or so; a video panel's is the refresh but its blanking) */
			if (ns < NSEC_PER_SEC / 130 || ns > NSEC_PER_SEC / 45 ||
			    gPanel.line_ticks * fPanelHeight >= (hakios_display_scan_video() ? period : period * 9 / 10)) {
				printf("MiniIOMobileFramebuffer: panel scan: implausible (%llu us a refresh, %llu us a readout): "
				    "frames written at once\n", ns / 1000, micros(gPanel.line_ticks * fPanelHeight));
				return;
			}
			gPanel.period = period;
			gPanel.start = gPanel.ref_start = later;
			gPanel.frame = gPanel.ref_frame = frame;
			gPanel.ok = true;
			printf("MiniIOMobileFramebuffer: panel scan: a refresh every %llu us, read out in %llu us (%llu ns a line): "
			    "frames go to the panel between its readouts, the window server's refresh is the panel's\n", ns / 1000,
			    micros(gPanel.line_ticks * fPanelHeight), micros(gPanel.line_ticks * 1000));
			return;
		}
		IODelay(40);
	}
	printf("MiniIOMobileFramebuffer: panel scan: no second readout in 100 ms: frames written at once\n");
}
/*
 * (backend soft) The light has changed: the frame in memory shown again, darkened as it now is, at
 * once and not only with the next frame (a still screen may not get one for minutes). Under the lock
 * show() draws under, one processor: the helpers belong to show().
 */
void
IOMobileFramebuffer::lightChanged(void)
{
	if (gLightFramebuffer != NULL && gLightFramebuffer->fLightCall != NULL) {
		thread_call_enter(gLightFramebuffer->fLightCall);
	}
}

void
IOMobileFramebuffer::lightCall(thread_call_param_t self, thread_call_param_t unused __unused)
{
	((IOMobileFramebuffer *) self)->redrawForLight();
}

void
IOMobileFramebuffer::redrawForLight(void)
{
	IOLockLock(fLock);
	if (fBackBuffer != NULL && fGraphics) {
		toPanel(fBase, fBackBuffer, (size_t) fPanelHeight * fRowBytes);
	}
	IOLockUnlock(fLock);
}

void
IOMobileFramebuffer::copyBackBuffer(void)
{
	if (fBackBuffer == NULL || !fGraphics) {
		return;
	}
	if (fThreads <= 1 || fHelpersStarted + 1 < fThreads) {
		bool timely = scanWaitRows(0, (int32_t) fPanelHeight, fScanTarget);

		toPanel(fBase, fBackBuffer, (size_t) fPanelHeight * fRowBytes);
		if (!timely || !scanInTime(0, fScanTarget)) {
			__atomic_fetch_add(&fScanLate, 1, __ATOMIC_RELAXED);
		}
		return;
	}
	RowJob copyJob;
	bzero(&copyJob, sizeof(copyJob));
	copyJob.pixels = NULL;
	copyJob.target = fScanTarget;

	IOLockLock(fHelperLock);
	fHelperJob = &copyJob;
	fHelpersBusy = fThreads - 1;
	fHelperGeneration++;
	IOLockWakeup(fHelperLock, &fHelperGeneration, false);
	IOLockUnlock(fHelperLock);

	/* Thread 0 takes partition 0 */
	int32_t y0 = 0;
	int32_t y1 = (int32_t) ((int64_t) fPanelHeight / fThreads);
	bool timely = scanWaitRows(y0, y1, fScanTarget);

	toPanel(fBase, fBackBuffer, (size_t) (y1 - y0) * fRowBytes);
	if (!timely || !scanInTime(y0, fScanTarget)) {
		__atomic_fetch_add(&fScanLate, 1, __ATOMIC_RELAXED);
	}

	IOLockLock(fHelperLock);
	while (fHelpersBusy != 0) {
		IOLockSleep(fHelperLock, &fHelpersBusy, THREAD_UNINT);
	}
	IOLockUnlock(fHelperLock);
}


/* A layer whose rows would be read past its surface's memory (MiniIOSurface::holds()): said, a few times, and not drawn. */
static void
outside(MiniIOSurface * s, const char * what)
{
	static unsigned told;

	if (told++ < 8) {
		IOLog("MiniIOMobileFramebuffer: %s of surface %u (%ux%u, %u bytes a row, at %u of %u bytes) would be read past its "
		    "memory: not drawn\n", what, s->sid, s->info.width, s->info.height, s->info.bytesPerRow, s->info.offset,
		    s->info.allocSize);
	}
}

/*
 * A span of a layer's destination on the display, `at` and `length` long, as a span of the panel, [*p0, *p1). Worked out in
 * 64 bits, and a span more than 2^30 pixels off is not drawn: what is worked out from it (x - px0, px1 - px0) stays within
 * 32 bits, where a span of the client's numbers wrapped round onto the panel in 32 bits would read past the layer's rows
 * (fb_filter=0) when the panel is larger than the display (fbscale=). Whether there is a span at all.
 */
static bool
panelSpan(int32_t at, int32_t length, uint32_t display, uint32_t panel, int32_t * p0, int32_t * p1)
{
	int64_t from = (int64_t) at * panel / display, to = ((int64_t) at + length) * panel / display;

	if (to <= from || from < -(1LL << 30) || to > (1LL << 30)) {
		return false;
	}
	*p0 = (int32_t) from;
	*p1 = (int32_t) to;
	return true;
}

/*
 * fb_yuv: a rectangle of a two-plane YCbCr surface ('420f' full range, '420v' video range: luma, then interleaved
 * Cb Cr at half size each way) on the screen, by nearest neighbour, on this thread, BT.601 as CoreVideo's camera
 * buffers say. Where the layer's numbers do not make sense nothing is drawn. Whether it drew anything.
 */
bool
IOMobileFramebuffer::drawYUV(MiniIOSurface * s, const int32_t * src, const int32_t * dst)
{
	const struct minisurf_plane * luma = &s->planes[0], * chroma = &s->planes[1];
	const uint8_t * base = s->pixels();
	const bool video = s->info.pixelFormat == FOURCC('4', '2', '0', 'v');
	int32_t sx = src[0], sy = src[1], sw = src[2], sh = src[3];
	int32_t dx = dst[0], dy = dst[1], dw = dst[2], dh = dst[3];
	int32_t px0, py0, px1, py1;

	if (base == NULL || s->info.planeCount != 2 || luma->width == 0 || luma->height == 0 || chroma->width == 0 ||
	    chroma->height == 0) {
		return false;
	}
	if (sw <= 0 || sh <= 0) {
		sx = sy = 0;
		sw = (int32_t) luma->width;
		sh = (int32_t) luma->height;
	}
	if (dw <= 0 || dh <= 0) {
		dx = dy = 0;
		dw = (int32_t) fWidth;
		dh = (int32_t) fHeight;
	}
	{
		static unsigned told;

		if (told < 3) {
			told++;
			IOLog("MiniIOMobileFramebuffer: fb_yuv: surface %u '%c%c%c%c' %ux%u row %u, planes: %ux%u row %u at %u, %ux%u row %u at "
			    "%u (%u bytes an element); from %d,%d %dx%d to %d,%d %dx%d\n", s->sid, (int) (s->info.pixelFormat >> 24 & 0xff),
			    (int) (s->info.pixelFormat >> 16 & 0xff), (int) (s->info.pixelFormat >> 8 & 0xff), (int) (s->info.pixelFormat & 0xff),
			    s->info.width, s->info.height, s->info.bytesPerRow, luma->width, luma->height, luma->bytesPerRow, luma->offset,
			    chroma->width, chroma->height, chroma->bytesPerRow, chroma->offset, chroma->bytesPerElement, sx, sy, sw, sh, dx, dy,
			    dw, dh);
		}
	}
	if (sx < 0 || sy < 0 || (uint32_t) (sx + sw) > luma->width || (uint32_t) (sy + sh) > luma->height) {
		return false;
	}
	/* (the luma rows read, and the chroma rows -- half as many, the last one again past its height -- in the surface's memory) */
	{
		uint64_t chroma_row = MIN((uint32_t) (sy + sh - 1) / 2, chroma->height - 1);
		uint64_t chroma_bytes = (uint64_t) MIN((uint32_t) (sx + sw - 1) / 2, chroma->width - 1) * 2 + 2;

		if (!s->holds(luma->offset + (uint64_t) (sy + sh - 1) * luma->bytesPerRow, (uint64_t) (sx + sw)) ||
		    !s->holds(chroma->offset + chroma_row * chroma->bytesPerRow, chroma_bytes)) {
			outside(s, "a YCbCr layer");
			return false;
		}
	}
	if (!panelSpan(dx, dw, fWidth, fPanelWidth, &px0, &px1) || !panelSpan(dy, dh, fHeight, fPanelHeight, &py0, &py1)) {
		return false;
	}
	for (int32_t y = py0 > 0 ? py0 : 0; y < py1 && (uint32_t) y < fPanelHeight; y++) {
		const int32_t row = sy + (int32_t) ((int64_t) (y - py0) * sh / (py1 - py0));
		const uint8_t * yl = base + luma->offset + (size_t) row * luma->bytesPerRow;
		const uint8_t * cl = base + chroma->offset + (size_t) (row / 2 < (int32_t) chroma->height ? row / 2 :
		    (int32_t) chroma->height - 1) * chroma->bytesPerRow;
		uint32_t * to = (uint32_t *) ((fBackBuffer ? fBackBuffer : fBase) + (size_t) y * fRowBytes);

		for (int32_t x = px0 > 0 ? px0 : 0; x < px1 && (uint32_t) x < fPanelWidth; x++) {
			const int32_t column = sx + (int32_t) ((int64_t) (x - px0) * sw / (px1 - px0));
			const int32_t c2 = (column / 2 < (int32_t) chroma->width ? column / 2 : (int32_t) chroma->width - 1) * 2;
			int32_t yv = yl[column], cb = cl[c2] - 128, cr = cl[c2 + 1] - 128;
			int32_t r, g, b;

			if (video) {
				yv = (yv - 16) * 298 >> 8;
				cb = cb * 255 / 224;
				cr = cr * 255 / 224;
			}
			r = yv + (359 * cr >> 8);
			g = yv - ((88 * cb + 183 * cr) >> 8);
			b = yv + (454 * cb >> 8);
			r = r < 0 ? 0 : r > 255 ? 255 : r;
			g = g < 0 ? 0 : g > 255 ? 255 : g;
			b = b < 0 ? 0 : b > 255 ? 255 : b;
			to[x] = 0xff000000u | (uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b;
		}
	}
	return true;
}

/*
 * Put a rectangle of a 32-bit surface on the screen. `dst` is in display coordinates; the
 * panel may be of another size, and the layer itself may be scaled: both are done in one
 * step, bilinear (or nearest neighbour: fb_filter=0).
 */
void
IOMobileFramebuffer::drawLayer(MiniIOSurface * s, const int32_t * src, const int32_t * dst)
{
	int32_t sx = src[0], sy = src[1], sw = src[2], sh = src[3];
	int32_t dx = dst[0], dy = dst[1], dw = dst[2], dh = dst[3];
	int32_t px0, py0, px1, py1, cx0, cx1, cy0, cy1;
	bool filter = gFilter != 0;
	RowJob job;

	if (sw <= 0 || sh <= 0) {
		sx = sy = 0;
		sw = (int32_t) s->info.width;
		sh = (int32_t) s->info.height;
	}
	if (dw <= 0 || dh <= 0) {
		dx = dy = 0;
		dw = (int32_t) fWidth;
		dh = (int32_t) fHeight;
	}
	if (sx < 0 || sy < 0 || (uint32_t) (sx + sw) > s->info.width || (uint32_t) (sy + sh) > s->info.height) {
		return;
	}
	/* (the rows read are in the surface's memory: its last row's last pixel, four bytes a pixel, is) */
	if (!s->holds((uint64_t) (sy + sh - 1) * s->info.bytesPerRow, (uint64_t) (sx + sw) * 4)) {
		outside(s, "a layer");
		return;
	}
	/* the rectangle on the panel, and the part of it that is on the panel: none, nothing is drawn */
	if (!panelSpan(dx, dw, fWidth, fPanelWidth, &px0, &px1) || !panelSpan(dy, dh, fHeight, fPanelHeight, &py0, &py1)) {
		return;
	}
	/*
	 * (a layer wholly left or right of the panel has no columns, cx1 <= cx0: drawRows() would blend its rows from
	 * fColumn[cx0] to fColumn[cx1 - 1], read outside fColumn or left there by another layer -- past the surface's rows
	 * and blendRow both. One above or below the panel has no rows; of a tall one only those on the panel are gone through.)
	 */
	cx0 = px0 < 0 ? 0 : px0;
	cx1 = px1 > (int32_t) fPanelWidth ? (int32_t) fPanelWidth : px1;
	cy0 = py0 < 0 ? 0 : py0;
	cy1 = py1 > (int32_t) fPanelHeight ? (int32_t) fPanelHeight : py1;
	if (cx1 <= cx0 || cy1 <= cy0) {
		return;
	}
	/* (blending, a row of the picture mixed from two: room for it, for every one who draws rows) */
	if (filter && fBlendRowSize < (uint32_t) sw) {
		if (fBlendRow != NULL) {
			IODelete(fBlendRow, uint32_t, fBlendRowSize);
		}
		for (unsigned i = 0; i + 1 < fThreads; i++) {
			if (fHelperBlend[i] != NULL) {
				IODelete(fHelperBlend[i], uint32_t, fBlendRowSize);
			}
		}
		fBlendRowSize = (uint32_t) sw;
		fBlendRow = IONew(uint32_t, fBlendRowSize);
		for (unsigned i = 0; i + 1 < fThreads; i++) {
			fHelperBlend[i] = IONew(uint32_t, fBlendRowSize);
			if (fHelperBlend[i] == NULL && fBlendRow != NULL) {
				IODelete(fBlendRow, uint32_t, fBlendRowSize);
				fBlendRow = NULL;
			}
		}
		if (fBlendRow == NULL) {
			fBlendRowSize = 0;
		}
	}
	filter = filter && fBlendRow != NULL;
	/* which pixel of a source row each pixel of a panel row takes: worked out once, not 2.6 million times */
	for (int32_t x = cx0; x < cx1; x++) {
		if (filter) {
			int64_t at = sample_at(x - px0, sw, px1 - px0);

			fColumn[x] = (uint32_t) (at >> 8);              /* (in the blended row: from sx on) */
			fColumnWeight[x] = (uint16_t) (fColumn[x] + 1 < (uint32_t) sw ? at & 255 : 0);
		} else {
			fColumn[x] = (uint32_t) (sx + (int64_t) (x - px0) * sw / (px1 - px0));
		}
	}
	job.pixels = s->pixels();
	job.stride = s->info.bytesPerRow;
	job.sx = sx;
	job.sy = sy;
	job.sw = sw;
	job.sh = sh;
	job.px0 = px0;
	job.py0 = py0;
	job.px1 = px1;
	job.py1 = py1;
	job.cx0 = cx0;
	job.cx1 = cx1;
	job.cy0 = cy0;
	job.cy1 = cy1;
	job.filter = filter;
	job.unscaled = px1 - px0 == sw && py1 - py0 == sh && px0 >= 0 && (uint32_t) px1 <= fPanelWidth;
	job.target = fScanTarget;
	job.over = fOver;
	if (fThreads <= 1 || fDiff || fHelpersStarted + 1 < fThreads) {
		drawRows(&job, cy0, cy1, fBlendRow);
		return;
	}
	/* shared out: the helpers take their rows, this thread the first share, then waits for them */
	IOLockLock(fHelperLock);
	fHelperJob = &job;
	fHelpersBusy = fThreads - 1;
	fHelperGeneration++;
	IOLockWakeup(fHelperLock, &fHelperGeneration, false);
	IOLockUnlock(fHelperLock);
	drawShare(&job, 0, fBlendRow);
	IOLockLock(fHelperLock);
	while (fHelpersBusy != 0) {
		IOLockSleep(fHelperLock, &fHelpersBusy, THREAD_UNINT);
	}
	IOLockUnlock(fHelperLock);
}

/*
 * How evenly the frames of the last seconds came: how many followed the one before within the
 * time of one refresh at 120, 60, 40 and 30 a second, and the times half and 95 in 100 of them
 * stayed under. With the lock held.
 */
void
IOMobileFramebuffer::reportFrameTimes(void)
{
	static const uint32_t limits[] = { 10, 18, 26, 35, FRAME_TIMES + 1 };
	uint32_t within[sizeof(limits) / sizeof(limits[0])] = { 0 }, total = 0, sum = 0, half = 0, most = 0;

	for (uint32_t ms = 0; ms <= FRAME_TIMES; ms++) {
		for (uint32_t i = 0; i < sizeof(limits) / sizeof(limits[0]); i++) {
			if (ms < limits[i]) {
				within[i] += fFrameTimes[ms];
				break;
			}
		}
		total += fFrameTimes[ms];
	}
	for (uint32_t ms = 0; ms <= FRAME_TIMES && total != 0; ms++) {
		sum += fFrameTimes[ms];
		if (half == 0 && 2 * sum >= total) {
			half = ms + 1;
		}
		if (most == 0 && 100 * sum >= 95 * total) {
			most = ms + 1;
		}
	}
	if (total != 0) {
		IOLog("MiniIOMobileFramebuffer: from one frame to the next: %u times under 10 ms, %u under 18, %u under 26, %u under 35, "
		    "%u longer (the longest %u ms); half under %u ms, 95 in 100 under %u ms; %u pauses\n", within[0], within[1],
		    within[2], within[3], within[4], fFrameLongest, half, most, fFramePauses);
	}
	bzero(fFrameTimes, sizeof(fFrameTimes));
	fFramePauses = fFrameLongest = 0;
}

/* SwapEnd: the swap is handed to the thread that puts frames on the panel, as soon as that has no other waiting. */
void
IOMobileFramebuffer::present(const uint8_t * swap)
{
	AbsoluteTime deadline;

	uint64_t now = mach_absolute_time(), period, pause;

	/* (a frame that is late on the panel was late here already, or it was not: for telling whose it is) */
	nanoseconds_to_absolutetime(NSEC_PER_SEC / refreshHz() * 5 / 4, &period);
	nanoseconds_to_absolutetime(FRAME_PAUSE * NSEC_PER_MSEC, &pause);
	clock_interval_to_deadline(SWAP_HELD_MS, kMillisecondScale, &deadline);
	if (gTouchLat) {
		hakios_tl("tl: s %u at %llu\n", swapWord(swap, SWAP_ID), micros(now));
	}
	IOLockLock(fSwapLock);
	fStatSwapsLate += fSwapTime != 0 && now - fSwapTime > period && now - fSwapTime < pause;
	fSwapTime = now;
	fStatHeld += fSwapsWaiting >= swapQueue();
	while (fSwapsWaiting >= swapQueue()) {
		if (IOLockSleepDeadline(fSwapLock, &fSwapsWaiting, deadline, THREAD_UNINT) == THREAD_TIMED_OUT &&
		    fSwapsWaiting >= swapQueue()) {
			fStatLost++;                    /* (the GPU has stopped answering: the frame before is waiting for it) */
			IOLockUnlock(fSwapLock);
			return;
		}
	}
	unsigned slot = (fSwapFirst + fSwapsWaiting) % SWAP_QUEUE;

	memcpy(fSwap[slot], swap, SWAP_SIZE);
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		uint32_t sid = swapWord(swap, SWAP_SURFACE(layer));

		fSwapSurfaces[slot][layer] = sid != 0 ? MiniIOSurfaceCopy(sid) : NULL;
	}
	fSwapsWaiting++;
	IOLockWakeup(fSwapLock, fSwap, true);
	IOLockUnlock(fSwapLock);
}

void
IOMobileFramebuffer::presenterMain(void * self, wait_result_t result __unused)
{
	((IOMobileFramebuffer *) self)->presenter();
}

void
IOMobileFramebuffer::presenter(void)
{
	static uint8_t swap[SWAP_SIZE];         /* (one thread) */
	MiniIOSurface * surfaces[SWAP_LAYERS];
	bool bound = false;

	for (;;) {
		IOLockLock(fSwapLock);
		while (fSwapsWaiting == 0) {
			IOLockSleep(fSwapLock, fSwap, THREAD_UNINT);
		}
		memcpy(swap, fSwap[fSwapFirst], SWAP_SIZE);
		memcpy(surfaces, fSwapSurfaces[fSwapFirst], sizeof(surfaces));
		IOLockUnlock(fSwapLock);
		if (!bound) {
			bound = true;           /* (at the first frame: every core is up by then) */
			bindDrawer(0, "the presenter");
		}
		show(swap, surfaces);
		for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
			if (surfaces[layer] != NULL) {
				MiniIOSurfaceDrop(surfaces[layer]);
			}
		}
		MiniIOSurfaceClearKept();               /* (the next frame is some milliseconds away) */
	}
}

/*
 * Are the first `rows` rows of a surface, `columns` 32-bit pixels of each, inside the memory it has? Its stride and size
 * come from whoever made it: checked in 64 bits, so that nothing wraps.
 */
static bool
surfaceHolds(const MiniIOSurface * s, uint64_t rows, uint64_t columns)
{
	if (rows == 0 || columns == 0) {
		return true;
	}
	return (uint64_t) s->info.offset + (rows - 1) * s->info.bytesPerRow + columns * 4 <= s->info.allocSize;
}

/*
 * fb_gpu: the frame's layers onto the panel by the GPU, all of them or none (then the processors draw it, show()). The
 * rectangles are drawLayer()'s; a layer partly off the panel is not cut here, the processors draw that frame. A surface
 * no submission has named (drawn == 0) is one the processor drew: what it wrote may be in its cache yet, not in memory.
 */
bool
IOMobileFramebuffer::showOnGPU(const uint8_t * swap, MiniIOSurface ** surfaces, int * target)
{
	bool full = false;
	struct MiniIOSurfacePresentLayer layers[SWAP_LAYERS];
	uint32_t set = swapWord(swap, SWAP_LAYERS_SET);
	unsigned count = 0;
	uint32_t fence;
	/* fb_video: the frame's video layer (at most one), for the display controller */
	MiniIOSurface * video = NULL, * held = NULL;
	int32_t videoSrc[4] = { 0, 0, 0, 0 };
	int64_t vq[4] = { 0, 0, 0, 0 };
	struct MiniIOSurfacePresentVideo v;
	bool test = false;

	fVideoNext.valid = false;

	if (fPanelPhys == 0) {
		fStatRefused[kRefusedNoPanel]++;
		return false;
	}
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		uint32_t sid = swapWord(swap, SWAP_SURFACE(layer));
		MiniIOSurface * s = surfaces[layer];
		int32_t src[4], dst[4];

		if (sid == 0) {
			if (layer == 0 && (set & 1)) {
				fStatRefused[kRefusedBlack]++;
				return false;           /* (the screen made black: the processors' bzero()) */
			}
			continue;
		}
		if (s != NULL && s->info.planeCount == 2 && gYUV != 0) {
			int32_t vs[4], vd[4];

			memcpy(vs, swap + SWAP_SOURCE(layer), sizeof(vs));
			memcpy(vd, swap + SWAP_DEST(layer), sizeof(vd));
			if (vd[2] <= 0 || vd[3] <= 0) {
				vd[0] = vd[1] = 0;
				vd[2] = (int32_t) fWidth;
				vd[3] = (int32_t) fHeight;
			}
			/* (a second video, or none for the controller: drawYUV() by the processors) */
			if (!gVideo || *target < 0 || video != NULL) {
				fStatRefused[kRefusedVideo]++;
				return false;
			}
			video = s;
			memcpy(videoSrc, vs, sizeof(vs));
			vq[0] = (int64_t) vd[0] * fPanelWidth / fWidth;
			vq[1] = (int64_t) vd[1] * fPanelHeight / fHeight;
			vq[2] = ((int64_t) vd[0] + vd[2]) * fPanelWidth / fWidth;
			vq[3] = ((int64_t) vd[1] + vd[3]) * fPanelHeight / fHeight;
			continue;
		}
		if (s == NULL || s->info.bytesPerElement != 4 || s->info.planeCount != 0) {
			continue;                       /* (nothing drawn for it by the processors either) */
		}
		memcpy(src, swap + SWAP_SOURCE(layer), sizeof(src));
		memcpy(dst, swap + SWAP_DEST(layer), sizeof(dst));
		int32_t sx = src[0], sy = src[1], sw = src[2], sh = src[3];
		int32_t dx = dst[0], dy = dst[1], dw = dst[2], dh = dst[3];

		if (sw <= 0 || sh <= 0) {
			sx = sy = 0;
			sw = (int32_t) s->info.width;
			sh = (int32_t) s->info.height;
		}
		if (dw <= 0 || dh <= 0) {
			dx = dy = 0;
			dw = (int32_t) fWidth;
			dh = (int32_t) fHeight;
		}
		if (sx < 0 || sy < 0 || (uint32_t) (sx + sw) > s->info.width || (uint32_t) (sy + sh) > s->info.height ||
		    !surfaceHolds(s, (uint32_t) (sy + sh), (uint32_t) (sx + sw))) {
			continue;
		}
		int64_t qx0 = (int64_t) dx * fPanelWidth / fWidth;
		int64_t qy0 = (int64_t) dy * fPanelHeight / fHeight;
		int64_t qx1 = ((int64_t) dx + dw) * fPanelWidth / fWidth;
		int64_t qy1 = ((int64_t) dy + dh) * fPanelHeight / fHeight;

		if (qx1 <= qx0 || qy1 <= qy0 || qx1 <= 0 || qy1 <= 0 || qx0 >= fPanelWidth || qy0 >= fPanelHeight) {
			continue;                       /* (nothing of it on the panel) */
		}
		if (qx0 < 0 || qy0 < 0 || qx1 > fPanelWidth || qy1 > fPanelHeight) {
			fStatRefused[kRefusedOffPanel]++;
			return false;
		}
		if (s->drawn == 0) {
			fStatRefused[kRefusedNotGPUDrawn]++;
			return false;
		}
		if (!fGraphics) {
			fGraphics = true;
			PE_initialize_console(NULL, kPEGraphicsMode);
			PE_initialize_console(NULL, kPEReleaseScreen);
		}
		layers[count].surface = s;
		layers[count].sx = (uint32_t) sx;
		layers[count].sy = (uint32_t) sy;
		layers[count].sw = (uint32_t) sw;
		layers[count].sh = (uint32_t) sh;
		layers[count].dx = (uint32_t) qx0;
		layers[count].dy = (uint32_t) qy0;
		layers[count].dw = (uint32_t) (qx1 - qx0);
		layers[count].dh = (uint32_t) (qy1 - qy0);
		full = full || (qx0 == 0 && qy0 == 0 && qx1 == fPanelWidth && qy1 == fPanelHeight);
		count++;
	}
	if (count == 0) {
		fStatRefused[kRefusedNothing]++;
		return false;
	}
	/* (fb_flip: a frame into a buffer of its own only when it covers all of it; what was there is two frames old) */
	if (*target >= 0 && !full) {
		*target = -1;
		fStatFlipNotFull++;
	}
	/* fb_video_test: the test's surface as the video, under a hole of the frame's (when the frame has none of its own) */
	if (video == NULL && gVideoTest != 0 && gVideo && *target >= 0 && (held = MiniIOSurfaceCopy(gVideoTest)) != NULL) {
		video = held;
		test = true;
		vq[0] = VIDEO_TEST_X;
		vq[1] = VIDEO_TEST_Y;
		vq[2] = VIDEO_TEST_X + VIDEO_TEST_W;
		vq[3] = VIDEO_TEST_Y + VIDEO_TEST_H;
	}
	if (video != NULL && (*target < 0 || !videoTake(video, videoSrc, vq[0], vq[1], vq[2], vq[3], *target, &v))) {
		if (fStatVideoRefused++ < 4) {
			IOLog("MiniIOMobileFramebuffer: fb_video: surface %u (%ux%u, %u planes) from %d,%d %dx%d onto %lld,%lld .. "
			    "%lld,%lld of the panel is not one the controller takes%s: the processors draw it\n", video->sid,
			    video->info.width, video->info.height, video->info.planeCount, videoSrc[0], videoSrc[1], videoSrc[2],
			    videoSrc[3], vq[0], vq[1], vq[2], vq[3], *target < 0 ? " (the frame not in a buffer of fb_flip's)" : "");
		}
		fVideoNext.valid = false;
		if (held != NULL) {
			MiniIOSurfaceDrop(held);
			held = NULL;
		}
		if (!test) {
			fStatRefused[kRefusedVideo]++;
			return false;
		}
		video = NULL;
	}
	if (video != NULL && test) {
		v.hole = true;
		v.hx = VIDEO_TEST_X;
		v.hy = VIDEO_TEST_Y;
		v.hw = VIDEO_TEST_W;
		v.hh = VIDEO_TEST_H;
	}
	uint64_t phys = *target >= 0 ? fFlipPhys[*target] : fPanelPhys;
	uint32_t rowBytes = *target >= 0 ? fFlipRowBytes : fRowBytes;

	fence = MiniIOSurfacePresent(layers, count, phys, rowBytes, fPanelWidth, fPanelHeight, video != NULL ? &v : NULL);
	if (held != NULL) {
		MiniIOSurfaceDrop(held);        /* (the present holds it while the GPU reads it) */
	}
	if (fence == 0) {
		fVideoNext.valid = false;
		fStatRefused[kRefusedByGPU]++;
		return false;
	}
	fStatVideoFrames += video != NULL;
	gLastPresent = fence;
	fGPUBase = *target >= 0 ? fFlipBase[*target] : fBase;
	fGPURowBytes = rowBytes;
	fGPUFirst = layers[0];
	fGPUDrawn = layers[0].surface->drawn;
	fGPULayers = count;
	fGPUFrames++;
	return true;
}

/* fb_gpu: the panel's pixels the GPU wrote against drawLayer()'s bilinear of the same surface (see GPU_CHECK_FIRST). */
void
IOMobileFramebuffer::checkOnGPU(void)
{
	const struct MiniIOSurfacePresentLayer * l = &fGPUFirst;
	MiniIOSurface * s = l->surface;
	uint32_t drawn = fGPUDrawn, close = 0, near = 0, far = 0, worst = 0, wx = 0, wy = 0, expect = 0, got = 0;

	if (fGPULayers != 1 || !MiniIOSurfacePresentDone(gLastPresent, 100)) {
		IOLog("MiniIOMobileFramebuffer: fb_gpu: check: frame %u not checked (%u layers, or the GPU not done)\n", fGPUFrames,
		    fGPULayers);
		return;
	}
	for (uint32_t i = 0; i < 256; i++) {
		/* a 16 x 16 grid over the rectangle, each point moved a little so that they do not all fall on one phase */
		uint32_t x = l->dx + (uint32_t) ((uint64_t) l->dw * (i % 16) / 16) + (i * 7) % 13;
		uint32_t y = l->dy + (uint32_t) ((uint64_t) l->dh * (i / 16) / 16) + (i * 11) % 17;

		x = x < l->dx + l->dw ? x : l->dx + l->dw - 1;
		y = y < l->dy + l->dh ? y : l->dy + l->dh - 1;
		int64_t ax = sample_at(x - l->dx, l->sw, l->dw), ay = sample_at(y - l->dy, l->sh, l->dh);
		uint32_t c0 = (uint32_t) (ax >> 8), r0 = (uint32_t) (ay >> 8);
		uint32_t c1 = c0 + 1 < l->sw ? c0 + 1 : c0, r1 = r0 + 1 < l->sh ? r0 + 1 : r0;
		const uint8_t * upper = s->pixels() + (size_t) (l->sy + r0) * s->info.bytesPerRow + (size_t) l->sx * 4;
		const uint8_t * lower = s->pixels() + (size_t) (l->sy + r1) * s->info.bytesPerRow + (size_t) l->sx * 4;
		uint32_t * panel = (uint32_t *) (fGPUBase + (size_t) y * fGPURowBytes) + x;

		/* (what the processor's cache may hold of either is older than memory, where the GPU read and wrote) */
		flush_dcache64((addr64_t) (uintptr_t) (upper + c0 * 4), 8, FALSE);
		flush_dcache64((addr64_t) (uintptr_t) (lower + c0 * 4), 8, FALSE);
		flush_dcache64((addr64_t) (uintptr_t) panel, 4, FALSE);
		uint32_t top = blend(((const uint32_t *) upper)[c0], ((const uint32_t *) upper)[c1], (uint32_t) (ax & 255));
		uint32_t bottom = blend(((const uint32_t *) lower)[c0], ((const uint32_t *) lower)[c1], (uint32_t) (ax & 255));
		uint32_t want = blend(top, bottom, (uint32_t) (ay & 255)), have = *(volatile uint32_t *) panel, diff = 0;

		for (unsigned shift = 0; shift < 32; shift += 8) {
			int32_t d = (int32_t) (want >> shift & 0xff) - (int32_t) (have >> shift & 0xff);

			diff = (uint32_t) (d < 0 ? -d : d) > diff ? (uint32_t) (d < 0 ? -d : d) : diff;
		}
		close += diff <= 4;
		near += diff > 4 && diff <= 32;
		far += diff > 32;
		if (diff > worst) {
			worst = diff;
			wx = x;
			wy = y;
			expect = want;
			got = have;
		}
	}
	if (s->drawn != drawn) {
		IOLog("MiniIOMobileFramebuffer: fb_gpu: check: frame %u: its surface was drawn into again meanwhile: not counted\n",
		    fGPUFrames);
		return;
	}
	IOLog("MiniIOMobileFramebuffer: fb_gpu: check: frame %u (surface %u, %ux%u at %u,%u onto %ux%u at %u,%u): of 256 "
	    "pixels %u within 4 of the processors' bilinear, %u within 32, %u further; the worst %u at %u,%u (%#x, the "
	    "processors' %#x)\n", fGPUFrames, s->sid, l->sw, l->sh, l->sx, l->sy, l->dw, l->dh, l->dx, l->dy, close, near, far,
	    worst, wx, wy, got, expect);
}

/*
 * fb_flip: the controller as the boot loader left it (kern.hakios.fb_dpu: VIG0 and VIG1 fetch the panel's memory, its
 * left and right half, for CTL0 and INTF_1), and two buffers after the panel's memory in the boot loader's display memory.
 */
bool
IOMobileFramebuffer::flipSetUp(void)
{
	uint32_t ctl = kDPUCTLs[FLIP_CTL];
	uint64_t size, first;

	fFlipTried = true;
	fFlipShown = -1;
	if (!mdssMapped()) {
		IOLog("MiniIOMobileFramebuffer: fb_flip: the board has not the feature display-flip: off\n");
		gFlipState = 2;
		return false;
	}
	fFlipFlush = 1u << 17;
	for (unsigned p = 0; p < sizeof(kDPUPipes) / sizeof(kDPUPipes[0]); p++) {
		uint32_t b = kDPUPipes[p].offset;

		if (mdssRead(b + SSPP_SRC0_ADDR) != (uint32_t) fPanelPhys || fFlipPipeCount == 2) {
			continue;
		}
		fFlipHomeStride[fFlipPipeCount] = mdssRead(b + SSPP_SRC_YSTRIDE0);
		fFlipPipe[fFlipPipeCount++] = b;
		fFlipFlush |= 1u << kDPUPipes[p].flush;
	}
	if (fFlipPipeCount == 0 || (fPanelPhys >> 32) != 0 || (fFlipHomeStride[0] & 0xffff) != fRowBytes ||
	    !(mdssRead(ctl + CTL_INTF_ACTIVE) & 2) || mdssRead(ctl + CTL_FLUSH) != 0 ||
	    !(intfRead(INTF_TEAR_AUTOREFRESH_CONFIG) & 0x80000000u)) {
		IOLog("MiniIOMobileFramebuffer: fb_flip: the controller is not as expected (%u pipes at %#llx, stride %#x; CTL%u "
		    "interfaces %#x flush %#x; auto-refresh %#x): off\n", fFlipPipeCount, fPanelPhys, fFlipHomeStride[0], FLIP_CTL,
		    mdssRead(ctl + CTL_INTF_ACTIVE), mdssRead(ctl + CTL_FLUSH), intfRead(INTF_TEAR_AUTOREFRESH_CONFIG));
		gFlipState = 3;
		return false;
	}
	fFlipRowBytes = (fPanelWidth * 4 + 63) & ~63u;
	size = round_page_64((uint64_t) fFlipRowBytes * fPanelHeight);
	first = (fPanelPhys + (uint64_t) fRowBytes * fPanelHeight + 0xfffff) & ~0xfffffULL;
	if (first + 2 * size > fPanelPhys + gSplashSize - FLIP_SPLASH_KEEP) {
		IOLog("MiniIOMobileFramebuffer: fb_flip: no room for two buffers of %llu bytes after the panel's memory: off\n", size);
		gFlipState = 4;
		return false;
	}
	for (unsigned i = 0; i < 2; i++) {
		fFlipPhys[i] = first + i * size;
		fFlipMemory[i] = IOMemoryDescriptor::withAddressRange(fFlipPhys[i], size, kIODirectionInOut | kIOMemoryMapperNone, NULL);
		fFlipMap[i] = fFlipMemory[i] != NULL ? fFlipMemory[i]->createMappingInTask(kernel_task, 0,
		    kIOMapAnywhere | kIOMapWriteCombineCache) : NULL;
		if (fFlipMap[i] == NULL) {
			IOLog("MiniIOMobileFramebuffer: fb_flip: the buffer at %#llx cannot be mapped: off\n", fFlipPhys[i]);
			gFlipState = 5;
			return false;
		}
		fFlipBase[i] = (uint8_t *) fFlipMap[i]->getVirtualAddress();
	}
	gFlipState = 1;
	gFlipFramebuffer = this;
	IOLog("MiniIOMobileFramebuffer: fb_flip: frames into %#llx and %#llx (rows of %u bytes) in turn; %u pipes, CTL%u flush "
	    "bits %#x\n", fFlipPhys[0], fFlipPhys[1], fFlipRowBytes, fFlipPipeCount, FLIP_CTL, fFlipFlush);
	return true;
}


/*
 * fb_video: the controller's video memory (the rest of the boot loader's display memory after fb_flip's two buffers, one
 * half for each), and the mixers as the boot loader left them: CTL0's mixers with VIG0 (and VIG1) at one stage above
 * the first, VIG2 and VIG3 unused. Once, when the first video comes.
 */
bool
IOMobileFramebuffer::videoSetUp(void)
{
	uint32_t ctl = kDPUCTLs[FLIP_CTL];
	uint64_t flipSize = round_page_64((uint64_t) fFlipRowBytes * fPanelHeight);
	uint64_t start = (fFlipPhys[1] + flipSize + 0xffff) & ~0xffffULL;
	uint64_t end = (fPanelPhys + gSplashSize - FLIP_SPLASH_KEEP) & ~0xfffULL;
	uint32_t stage = 0;

	if (gVideoState != 0) {
		return gVideoState == 1;
	}
	if (end <= start || end - start < 4 * 1024 * 1024) {
		IOLog("MiniIOMobileFramebuffer: fb_video: no room for video memory after fb_flip's buffers (%#llx to %#llx): off\n",
		    start, end);
		gVideoState = 2;
		return false;
	}
	fVideoRegionPhys = start;
	fVideoRegionSize = end - start;
	fVideoBufferSize = (fVideoRegionSize / 2) & ~0xfffULL;
	fVideoPhys[0] = start;
	fVideoPhys[1] = start + fVideoBufferSize;
	fVideoLMs = 0;
	for (unsigned m = 0; m < 5; m++) {
		uint32_t layer = mdssRead(ctl + CTL_LAYER0 + 4 * m);

		fVideoHomeLayer[m] = layer;
		if ((layer & 7) == 0) {
			continue;
		}
		if (m != fVideoLMs || (stage != 0 && stage != (layer & 7) - 1) || ((layer >> CTL_MIX_SHIFT_VIG2) & 7) != 0 ||
		    ((layer >> CTL_MIX_SHIFT_VIG3) & 7) != 0) {
			stage = 0;
			break;
		}
		stage = (layer & 7) - 1;
		fVideoHomeLMOp[m] = mdssRead(LM_BASE(m) + LM_OP_MODE);
		fVideoHomeBlend[m][0] = mdssRead(LM_BASE(m) + LM_BLEND_OP(stage));
		fVideoHomeBlend[m][1] = mdssRead(LM_BASE(m) + LM_BLEND_CONST(stage));
		fVideoLMs++;
	}
	fVideoHomeFetch = mdssRead(ctl + CTL_FETCH_PIPE_ACTIVE);
	fVideoUIStage = stage;
	if (fVideoLMs == 0 || fVideoLMs > 2 || stage < 2 || (fVideoHomeFetch & (CTL_FETCH_VIG2 | CTL_FETCH_VIG3)) != 0 ||
	    mdssRead(kDPUPipes[2].offset + SSPP_SRC0_ADDR) != 0 || mdssRead(kDPUPipes[3].offset + SSPP_SRC0_ADDR) != 0) {
		IOLog("MiniIOMobileFramebuffer: fb_video: the mixers are not as expected (CTL%u layers %#x %#x, fetch %#x, the "
		    "frame's stage %u, VIG2 %#x VIG3 %#x): off\n", FLIP_CTL, fVideoHomeLayer[0], fVideoHomeLayer[1], fVideoHomeFetch,
		    stage, mdssRead(kDPUPipes[2].offset + SSPP_SRC0_ADDR), mdssRead(kDPUPipes[3].offset + SSPP_SRC0_ADDR));
		gVideoState = 3;
		return false;
	}
	gVideoState = 1;
	IOLog("MiniIOMobileFramebuffer: fb_video: videos by VIG2/VIG3 under the frame (stage %u of %u mixers, op %#x, blend "
	    "%#x/%#x), from video memory %#llx and %#llx (%llu bytes each); scaler version %#x\n", stage, fVideoLMs,
	    fVideoHomeLMOp[0], fVideoHomeBlend[0][0], fVideoHomeBlend[0][1], fVideoPhys[0], fVideoPhys[1], fVideoBufferSize,
	    mdssRead(kDPUPipes[2].offset + VIG_SCALER + QSEED3_HW_VERSION));
	return true;
}

/*
 * A video layer for the controller, into fVideoNext and V (for MiniIOSurfacePresent()), the rectangle SRC (of the luma
 * plane, display numbers: 0 x 0 the whole) onto QX0, QY0 .. QX1, QY1 of the panel. False: not one it can take.
 */
bool
IOMobileFramebuffer::videoTake(MiniIOSurface * s, const int32_t * src, int64_t qx0, int64_t qy0, int64_t qx1, int64_t qy1,
    int target, struct MiniIOSurfacePresentVideo * v)
{
	const struct minisurf_plane * luma = &s->planes[0], * chroma = &s->planes[1];
	struct VideoFrame f;
	int64_t sx = src[0], sy = src[1], sw = src[2], sh = src[3];

	bzero(&f, sizeof(f));
	if (target < 0 || target > 1 || !videoSetUp() || s->info.planeCount != 2 ||
	    (s->info.pixelFormat != FOURCC('4', '2', '0', 'v') && s->info.pixelFormat != FOURCC('4', '2', '0', 'f')) ||
	    luma->width < 2 || luma->height < 2 || chroma->width != (luma->width + 1) / 2 || chroma->height != (luma->height + 1) / 2) {
		return false;
	}
	if (sw <= 0 || sh <= 0) {
		sx = sy = 0;
		sw = luma->width;
		sh = luma->height;
	}
	/* (4:2:0: a rectangle of whole chroma samples) */
	sw += sx & 1;
	sh += sy & 1;
	sx &= ~1LL;
	sy &= ~1LL;
	sw &= ~1LL;
	sh &= ~1LL;
	if (sx < 0 || sy < 0 || sw < 2 || sh < 2 || sx + sw > luma->width || sy + sh > luma->height || qx0 < 0 || qy0 < 0 ||
	    qx1 > fPanelWidth || qy1 > fPanelHeight || qx1 - qx0 < 2 || qy1 - qy0 < 2 ||
	    sw > 4 * (qx1 - qx0) || sh > 4 * (qy1 - qy0) || sw > 4096 || sh > 4096) {
		return false;
	}
	f.lumaPitch = (luma->width + 63) & ~63u;
	f.chromaPitch = (chroma->width * 2 + 63) & ~63u;
	f.chromaOffset = (uint32_t) round_page_64((uint64_t) f.lumaPitch * luma->height);
	if ((uint64_t) f.chromaOffset + (uint64_t) f.chromaPitch * chroma->height > fVideoBufferSize) {
		return false;
	}
	f.valid = true;
	f.full = s->info.pixelFormat == FOURCC('4', '2', '0', 'f');
	f.sx = (uint32_t) sx;
	f.sy = (uint32_t) sy;
	f.sw = (uint32_t) sw;
	f.sh = (uint32_t) sh;
	f.dx = (uint32_t) qx0;
	f.dy = (uint32_t) qy0;
	f.dw = (uint32_t) (qx1 - qx0);
	f.dh = (uint32_t) (qy1 - qy0);
	bzero(v, sizeof(*v));
	v->surface = s;
	v->regionPhys = fVideoRegionPhys;
	v->regionSize = fVideoRegionSize;
	v->lumaPhys = fVideoPhys[target];
	v->chromaPhys = fVideoPhys[target] + f.chromaOffset;
	v->lumaPitch = f.lumaPitch;
	v->chromaPitch = f.chromaPitch;
	fVideoNext = f;
	return true;
}

/* the 10-bit CSC's matrix, clamps and biases (dpu_hw_util.c dpu_hw_csc_setup; Linux's dpu_csc10_YUV2RGB_601L for '420v') */
static void
videoCSC(uint32_t pipe, bool full)
{
	static const int32_t kVideo[9] = { 0x12a00, 0, 0x19880, 0x12a00, -0x6480, -0xd000, 0x12a00, 0x20480, 0 };
	static const int32_t kFull[9] = { 0x10000, 0, 0x166e9, 0x10000, -0x581a, -0xb6d2, 0x10000, 0x1c5a2, 0 };
	static const uint32_t kVideoPre[6] = { 0x40, 0x3ac, 0x40, 0x3c0, 0x40, 0x3c0 };
	static const uint32_t kFullPre[6] = { 0, 0x3ff, 0, 0x3ff, 0, 0x3ff };
	const int32_t * m = full ? kFull : kVideo;
	const uint32_t * pre = full ? kFullPre : kVideoPre;
	uint32_t b = pipe + VIG_CSC10 + 4;

#define S49(v) ((uint32_t) ((v) >> 7) & 0x1fff)
	mdssWrite(b + 0x00, S49(m[0]) | S49(m[1]) << 16);
	mdssWrite(b + 0x04, S49(m[2]) | S49(m[3]) << 16);
	mdssWrite(b + 0x08, S49(m[4]) | S49(m[5]) << 16);
	mdssWrite(b + 0x0c, S49(m[6]) | S49(m[7]) << 16);
	mdssWrite(b + 0x10, S49(m[8]));
#undef S49
	for (unsigned i = 0; i < 3; i++) {
		mdssWrite(b + 0x14 + 4 * i, pre[2 * i] << 16 | pre[2 * i + 1]);            /* pre clamp */
		mdssWrite(b + 0x20 + 4 * i, 0x3ffu);                                      /* post clamp 0 .. 1023 */
		mdssWrite(b + 0x2c + 4 * i, i == 0 ? (full ? 0 : 0xffc0u) : 0xfe00u);   /* pre bias: -16 (video), -128 */
		mdssWrite(b + 0x38 + 4 * i, 0);                                           /* post bias */
	}
	mdssWrite(pipe + VIG_CSC10, mdssRead(pipe + VIG_CSC10) | CSC10_EN | CSC10_SRC_YUV);
}

/*
 * For flipTo(): the video pipes and the mixers as the frame shown from BUFFER wants them (fVideoNext, or no video), to
 * be taken with its flush. Returns the flush bits that adds.
 */
uint32_t
IOMobileFramebuffer::videoProgram(int buffer)
{
	static const unsigned kPipes[2] = { 2, 3 };     /* kDPUPipes: VIG2, VIG3 */
	uint32_t ctl = kDPUCTLs[FLIP_CTL], flush = 0, used = 0;
	const struct VideoFrame * f = &fVideoNext;

	if (gVideoState != 1 || (!fVideoOn && (buffer < 0 || !f->valid))) {
		return 0;
	}
	if (buffer >= 0 && f->valid) {
		/* the pieces: the left mixer's half and the right's, a pipe each, as the boot loader's VIG0 and VIG1 */
		uint32_t seam = fVideoLMs == 2 ? fPanelWidth / 2 : fPanelWidth;
		uint32_t edges[3] = { f->dx, f->dx + f->dw, f->dx + f->dw };
		unsigned pieces = 1;

		if (f->dx < seam && f->dx + f->dw > seam) {
			edges[1] = seam;
			pieces = 2;
		}
		for (unsigned k = 0; k < pieces; k++) {
			uint32_t pipe = kDPUPipes[kPipes[k]].offset;
			uint32_t ax = edges[k], bx = edges[k + 1], ow = bx - ax;
			uint32_t x0 = f->sx + (uint32_t) ((uint64_t) (ax - f->dx) * f->sw / f->dw);
			uint32_t x1 = f->sx + (uint32_t) (((uint64_t) (bx - f->dx) * f->sw + f->dw - 1) / f->dw);
			uint64_t luma = fVideoPhys[buffer], chroma = luma + f->chromaOffset;

			x0 &= ~1u;
			x1 = (x1 + 1) & ~1u;
			x1 = x1 < f->sx + f->sw ? x1 : f->sx + f->sw;
			uint32_t sw = x1 - x0, sh = f->sh;

			if (sw < 2 || ow < 1) {
				continue;
			}
			if (!(fVideoPipesOn & (1u << k))) {
				/* (its fetch paced as the boot loader's pipe's) */
				for (unsigned q = 0; q < sizeof(kSSPPQoS) / sizeof(kSSPPQoS[0]); q++) {
					mdssWrite(pipe + kSSPPQoS[q], mdssRead(kDPUPipes[0].offset + kSSPPQoS[q]));
				}
			}
			mdssWrite(pipe + SSPP_SRC_SIZE, sh << 16 | sw);
			mdssWrite(pipe + SSPP_SRC_XY, f->sy << 16 | x0);
			mdssWrite(pipe + SSPP_OUT_SIZE, f->dh << 16 | ow);
			mdssWrite(pipe + SSPP_OUT_XY, f->dy << 16 | ax);
			mdssWrite(pipe + SSPP_SRC0_ADDR, (uint32_t) luma);
			mdssWrite(pipe + SSPP_SRC1_ADDR, (uint32_t) chroma);
			mdssWrite(pipe + SSPP_SRC2_ADDR, 0);
			mdssWrite(pipe + SSPP_SRC3_ADDR, 0);
			mdssWrite(pipe + SSPP_SRC_YSTRIDE0, f->chromaPitch << 16 | f->lumaPitch);
			mdssWrite(pipe + SSPP_SRC_YSTRIDE1, 0);
			/* NV12 (dpu_formats.c, setup_format): 4:2:0, pseudo-planar, 8 bits, Cb then Cr, two a fetch, YUV */
			mdssWrite(pipe + SSPP_SRC_FORMAT, 3u << 23 | 2u << 19 | 0x3f | 1u << 12 | 1u << 17 | 1u << 9 | 1u << 15);
			mdssWrite(pipe + SSPP_SRC_UNPACK, 2u << 8 | 1u);
			mdssWrite(pipe + SSPP_SRC_OP_MODE, 0x80000000u);        /* PE_OVERRIDE: the pixel extension below */
			mdssWrite(pipe + SSPP_UBWC_ERROR_STATUS, 1u << 31);
			/* pixel extension: none; what the scaler is fed, of luma, chroma (half each way) and alpha */
			for (unsigned c = 0; c < 3; c++) {
				uint32_t w = c == 1 ? sw / 2 : sw, h = c == 1 ? sh / 2 : sh;

				mdssWrite(pipe + SSPP_PE_C0_LR + 0x10 * c, 0);
				mdssWrite(pipe + SSPP_PE_C0_LR + 0x10 * c + 4, 0);
				mdssWrite(pipe + SSPP_PE_C0_LR + 0x10 * c + 8, h << 16 | w);
			}
			/* the scaler (dpu_plane.c _dpu_plane_setup_scaler3, dpu_hw_setup_scaler3): bilinear, chroma at half */
			uint32_t stepX = (uint32_t) (((uint64_t) sw << 21) / ow), stepY = (uint32_t) (((uint64_t) sh << 21) / f->dh);
			uint32_t sc = pipe + VIG_SCALER;

			mdssWrite(sc + QSEED3_PHASE_INIT_Y_H, 0);
			mdssWrite(sc + QSEED3_PHASE_INIT_Y_H + 4, 0);
			mdssWrite(sc + QSEED3_PHASE_INIT_Y_H + 8, 0);
			mdssWrite(sc + QSEED3_PHASE_INIT_Y_H + 12, 0);
			mdssWrite(sc + QSEED3_PHASE_STEP_Y_H, stepX & 0xffffff);
			mdssWrite(sc + QSEED3_PHASE_STEP_Y_V, stepY & 0xffffff);
			mdssWrite(sc + QSEED3_PHASE_STEP_UV_H, (stepX / 2) & 0xffffff);
			mdssWrite(sc + QSEED3_PHASE_STEP_UV_V, (stepY / 2) & 0xffffff);
			mdssWrite(sc + QSEED3_PRELOAD, 4u | 2u << 8 | 4u << 16 | 2u << 24);
			mdssWrite(sc + QSEED3_SRC_SIZE_Y_RGB_A, sh << 16 | sw);
			mdssWrite(sc + QSEED3_SRC_SIZE_UV, (sh / 2) << 16 | sw / 2);
			mdssWrite(sc + QSEED3_DST_SIZE, f->dh << 16 | ow);
			mdssWrite(sc + QSEED3_OP_MODE, 1u | 3u << 16 | 1u << 12 | 3u << 24 | 1u << 31 | 1u << 14);
			videoCSC(pipe, f->full);
			used |= 1u << k;
			flush |= 1u << kDPUPipes[kPipes[k]].flush;
		}
	}
	/* the mixers: the pipes used at the first stage, the frame over them by its alpha; or as the boot loader had them */
	for (unsigned m = 0; m < fVideoLMs; m++) {
		uint32_t b = LM_BASE(m), layer = fVideoHomeLayer[m];

		layer |= (used & 1) ? 2u << CTL_MIX_SHIFT_VIG2 : 0;
		layer |= (used & 2) ? 2u << CTL_MIX_SHIFT_VIG3 : 0;
		mdssWrite(ctl + CTL_LAYER0 + 4 * m, layer);
		if (used != 0) {
			mdssWrite(b + LM_BLEND_OP(1), LM_BLEND_OPAQUE);
			mdssWrite(b + LM_BLEND_CONST(1), LM_BLEND_CONST_FG);
			mdssWrite(b + LM_BLEND_OP(fVideoUIStage), LM_BLEND_PREMULTIPLIED);
			mdssWrite(b + LM_BLEND_CONST(fVideoUIStage), LM_BLEND_CONST_FG);
			mdssWrite(b + LM_OP_MODE, fVideoHomeLMOp[m] | 1u << 1);
		} else {
			mdssWrite(b + LM_BLEND_OP(fVideoUIStage), fVideoHomeBlend[m][0]);
			mdssWrite(b + LM_BLEND_CONST(fVideoUIStage), fVideoHomeBlend[m][1]);
			mdssWrite(b + LM_OP_MODE, fVideoHomeLMOp[m]);
		}
		flush |= 1u << LM_FLUSH(m);
	}
	mdssWrite(ctl + CTL_FETCH_PIPE_ACTIVE, fVideoHomeFetch | ((used & 1) ? CTL_FETCH_VIG2 : 0) | ((used & 2) ? CTL_FETCH_VIG3 : 0));
	/* (a pipe no longer used: flushed too, so that it is out of the mixers with them) */
	for (unsigned k = 0; k < 2; k++) {
		if ((fVideoPipesOn & (1u << k)) && !(used & (1u << k))) {
			flush |= 1u << kDPUPipes[kPipes[k]].flush;
		}
	}
	if (used != 0 && !fVideoOn) {
		IOLog("MiniIOMobileFramebuffer: fb_video: on: %ux%u at %u,%u of the luma onto %ux%u at %u,%u by %s ('420%c')\n",
		    f->sw, f->sh, f->sx, f->sy, f->dw, f->dh, f->dx, f->dy, used == 3 ? "VIG2 and VIG3" : used == 1 ? "VIG2" : "VIG3",
		    f->full ? 'f' : 'v');
	} else if (used == 0 && fVideoOn) {
		IOLog("MiniIOMobileFramebuffer: fb_video: off (%u frames with one)\n", gVideoFrames);
	}
	fVideoOn = used != 0;
	fVideoPipesOn = used;
	gVideoFrames += used != 0;
	return flush;
}

/* Whether the last flush has been taken, waiting so long at most. */
bool
IOMobileFramebuffer::flipTaken(uint32_t milliseconds)
{
	uint32_t ctl = kDPUCTLs[FLIP_CTL];
	uint64_t began = mach_absolute_time(), until;
	bool waited = false;

	if (!fFlipPending) {
		return true;
	}
	nanoseconds_to_absolutetime((uint64_t) milliseconds * NSEC_PER_MSEC, &until);
	until += began;
	while ((mdssRead(ctl + CTL_FLUSH) & fFlipFlush) != 0) {
		if (mach_absolute_time() >= until) {
			return false;
		}
		waited = true;
		IODelay(20);
	}
	if (waited) {
		fStatFlipWaited++;
		fStatFlipWait += mach_absolute_time() - began;
	}
	fFlipPending = false;
	return true;
}

/* The controller pointed at a buffer (-1: the panel's own memory), taken when it starts its next frame. */
void
IOMobileFramebuffer::flipTo(int buffer)
{
	uint32_t ctl = kDPUCTLs[FLIP_CTL];
	uint64_t phys = buffer >= 0 ? fFlipPhys[buffer] : fPanelPhys;
	uint32_t stride = buffer >= 0 ? fFlipRowBytes : fRowBytes;

	for (unsigned i = 0; i < fFlipPipeCount; i++) {
		mdssWrite(fFlipPipe[i] + SSPP_SRC0_ADDR, (uint32_t) phys);
		mdssWrite(fFlipPipe[i] + SSPP_SRC_YSTRIDE0, (fFlipHomeStride[i] & 0xffff0000u) | stride);
	}
	uint32_t videoFlush = videoProgram(buffer);    /* (fb_video: the video of this frame, or none, with the same flush) */

	__asm__ volatile ("dsb sy" ::: "memory");
	mdssWrite(ctl + CTL_FLUSH, fFlipFlush | videoFlush);
	fFlipWritten = mach_absolute_time();
	fFlipWrittenLine = flipLine();
	fFlipShown = buffer;
	gScreenRowBytes = stride;
	gScreenBase = buffer >= 0 ? fFlipBase[buffer] : NULL;
	fFlipPending = true;
	fFlipCount++;
	fStatFlips++;
	fStatFlipsHome += buffer < 0;
	if (fFlipCount <= FLIP_TIMED) {
		/* the first ones: when the flush is taken, and where the scan is then (a frame's start: at INTF_TEAR_START_POS) */
		uint64_t took, until;
		uint32_t line;
		bool taken;

		nanoseconds_to_absolutetime((uint64_t) FLIP_TAKEN_MS * NSEC_PER_MSEC, &until);
		until += fFlipWritten;
		while ((mdssRead(ctl + CTL_FLUSH) & fFlipFlush) != 0 && mach_absolute_time() < until) {
		}
		took = mach_absolute_time();
		line = flipLine();
		taken = (mdssRead(ctl + CTL_FLUSH) & fFlipFlush) == 0;
		IOLog("MiniIOMobileFramebuffer: fb_flip: flip %u to %d (%#llx): written at line %u, %s after %llu us at line %u "
		    "(start %u)\n", fFlipCount, buffer, phys, fFlipWrittenLine, taken ? "taken" : "NOT taken",
		    micros(took - fFlipWritten), line, intfRead(INTF_TEAR_START_POS));
		fFlipPending = !taken;
	}
}

/* fb_flip off: the controller back at the panel's own memory. */
void
IOMobileFramebuffer::flipOff(const char * why)
{
	IOLog("MiniIOMobileFramebuffer: fb_flip: off: %s (CTL%u flush %#x, line %u, %u flips)\n", why, FLIP_CTL,
	    mdssRead(kDPUCTLs[FLIP_CTL] + CTL_FLUSH), flipLine(), fFlipCount);
	gFlip = 0;
	gFlipState = 6;
	flipTo(-1);
	fFlipPending = false;
}

/*
 * At the run's end and on a panic's way out (hakios_fb_quiesce(), one processor, its interrupts off): the frame being
 * shown copied into the panel's own memory and the controller pointed back there, so what is drawn there next -- the
 * loader's screen, a panic's -- is seen, over the last frame rather than over one two frames old. No log, no lock: the
 * presenter is not running any more.
 */
void
IOMobileFramebuffer::flipHome(void)
{
	uint32_t ctl = kDPUCTLs[FLIP_CTL];
	const uint8_t * from;
	uint64_t until;
	char line[160];
	int was = fFlipShown;
	bool taken;

	if (fFlipShown < 0 || fFlipShown > 1 || fBase == NULL) {
		if (fVideoOn) {
			fVideoNext.valid = false;
			mdssWrite(ctl + CTL_FLUSH, videoProgram(-1));
		}
		snprintf(line, sizeof(line), "fb_flip: at the end the controller is on the panel's memory already (%u flips)\n",
		    fFlipCount);
		loader_log_line(line);
		return;
	}
	from = fFlipBase[fFlipShown];
	for (uint32_t y = 0; y < fPanelHeight; y++) {
		memcpy(fBase + (size_t) y * fRowBytes, from + (size_t) y * fFlipRowBytes, (size_t) fPanelWidth * 4);
	}
	flush_dcache64((addr64_t) (uintptr_t) fBase, fRowBytes * fPanelHeight, FALSE);
	for (unsigned i = 0; i < fFlipPipeCount; i++) {
		mdssWrite(fFlipPipe[i] + SSPP_SRC0_ADDR, (uint32_t) fPanelPhys);
		mdssWrite(fFlipPipe[i] + SSPP_SRC_YSTRIDE0, fFlipHomeStride[i]);
	}
	fVideoNext.valid = false;
	uint32_t videoFlush = videoProgram(-1);         /* (fb_video: the mixers as the boot loader had them) */

	__asm__ volatile ("dsb sy" ::: "memory");
	mdssWrite(ctl + CTL_FLUSH, fFlipFlush | videoFlush);
	nanoseconds_to_absolutetime((uint64_t) FLIP_TAKEN_MS * NSEC_PER_MSEC, &until);
	until += mach_absolute_time();
	while ((mdssRead(ctl + CTL_FLUSH) & fFlipFlush) != 0 && mach_absolute_time() < until) {
	}
	taken = (mdssRead(ctl + CTL_FLUSH) & fFlipFlush) == 0;
	snprintf(line, sizeof(line), "fb_flip: at the end the controller back from buffer %d to the panel's memory %#llx: flush "
	    "%s, pipe source %#x (%u flips)\n", was, fPanelPhys, taken ? "taken" : "NOT taken",
	    mdssRead(fFlipPipe[0] + SSPP_SRC0_ADDR), fFlipCount);
	loader_log_line(line);
	fFlipShown = -1;
	fFlipPending = false;
	gScreenBase = NULL;
	gScreenRowBytes = fRowBytes;
	gFlip = 0;
}

/* Before the processors write the panel: the GPU done with what it was given to write there (it would write over them). */
void
IOMobileFramebuffer::panelForProcessor(void)
{
	if (gLastPresent != 0) {
		MiniIOSurfacePresentDone(gLastPresent, 100);
		gLastPresent = 0;
	}
}

/* A swap's surfaces onto the panel, once the GPU is done with them. */
void
IOMobileFramebuffer::show(const uint8_t * swap, MiniIOSurface ** surfaces)
{
	uint32_t set = swapWord(swap, SWAP_LAYERS_SET);
	uint64_t began = mach_absolute_time(), nanos, ready, shown, composed, drawn;
	uint32_t layers = 0;
	MiniIOSurface * recorded = NULL;
	bool report;

	/*
	 * A swap can come before the GPU work that draws its surface has reached the kernel (the render server swaps once its
	 * command buffer is scheduled, and the runtime or Turnip may hand it over later): the surface's last submission is then the one
	 * of its frame two or three swaps ago, long done, and that old picture went on the panel -- the flicker of everything that moves
	 * (still frames are the same picture). A surface whose last submission has not changed since it was last shown is waited for, up
	 * to SWAP_NEW_MS, to be named by a newer one; then for the GPU, as before.
	 */
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		MiniIOSurface * s = surfaces[layer];
		unsigned slot;

		if (s == NULL) {
			continue;
		}
		for (slot = 0; slot < SHOWN_SLOTS && fShown[slot].sid != s->sid; slot++) {
		}
		if (slot < SHOWN_SLOTS && fShown[slot].drawn == s->drawn && s->drawn != 0) {
			uint64_t waited = 0;

			while (s->drawn == fShown[slot].drawn && waited < SWAP_NEW_MS) {
				IOSleep(1);
				waited++;
			}
			fStatStale++;
			if (s->drawn == fShown[slot].drawn) {
				fStatStaleShown++;
			}
		}
		if (slot == SHOWN_SLOTS) {
			slot = fShownNext++ % SHOWN_SLOTS;
			fShown[slot].sid = s->sid;
		}
		fShown[slot].drawn = s->drawn;
	}
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		if (surfaces[layer] != NULL && !MiniIOSurfaceDrawn(surfaces[layer], SWAP_GPU_MS)) {
			static unsigned unfinishedReports;

			if (unfinishedReports < 4) {
				unfinishedReports++;
				/* (printf, as the swap lines below: a serial console shows them) */
				printf("MiniIOMobileFramebuffer: swap %u: the GPU is not done with surface %u after %u ms; dropped, the "
				    "frame on the panel stays\n", swapWord(swap, SWAP_ID), swapWord(swap, SWAP_SURFACE(layer)), SWAP_GPU_MS);
			}
			fStatWaiting += mach_absolute_time() - began;
			/* nothing of it is read: only its queue slot is given back (the presenter drops its surfaces) */
			IOLockLock(fSwapLock);
			fSwapFirst = (fSwapFirst + 1) % SWAP_QUEUE;
			fSwapsWaiting--;
			fStatLost++;
			IOLockWakeup(fSwapLock, &fSwapsWaiting, true);
			IOLockUnlock(fSwapLock);
			return;
		}
	}
	fStatWaiting += mach_absolute_time() - began;
	began = ready = mach_absolute_time();
	if (gOnTheBeat) {
		uint64_t period, aim, next;

		nanoseconds_to_absolutetime(NSEC_PER_SEC / refreshHz(), &period);
		aim = period * 3 / 8;
		next = fReady;
		fReady = began;
		if (fBeat == 0 || began - next > period * 3 / 2) {
			fBeat = began;
		} else {
			for (next = fBeat + period; next < began; next += period) {
			}
			if (next - began > aim) {
				next -= (next - began - aim) / 8;
			} else {
				next += (aim - (next - began)) / 8;
			}
			clock_delay_until(next);
			fBeat = next;
			fStatOnBeat++;
			fStatBeatWait += mach_absolute_time() - began;
			began = mach_absolute_time();
		}
	}
	shown = began;
	/*
	 * fb_gpu: the frame into the GPU's ring, behind what drew it; fb_flip: into the buffer the controller does not read,
	 * once it has left it. A frame the GPU does not take is drawn by the processors below, into the panel's own memory.
	 */
	int target = -1;
	bool onGPU = false;

	if (gGPUPresent && gFlip && fPanelPhys != 0 && (fFlipTried ? gFlipState == 1 : flipSetUp())) {
		if (fFlipPending && !flipTaken(FLIP_TAKEN_MS)) {
			flipOff("a flush not taken in 100 ms");
		} else {
			target = fFlipShown == 0 ? 1 : 0;
		}
	}
	if (gGPUPresent) {
		onGPU = showOnGPU(swap, surfaces, &target);
	}
	if (!onGPU) {
		panelForProcessor();
	}
	fStatGPUFrames += onGPU;
	fStatCPUFrames += !onGPU;
	/*
	 * The panel's readout this frame is for: the first that begins a lead after now. The lead grows
	 * by half a millisecond after a frame with a stripe written too late for it, and shrinks by a
	 * tenth of one after 60 frames in a row in time (a video panel: after 10): as little delay as
	 * there is room for. (A video panel's next readout begins as this one ends; a lead grown to 4.9 ms
	 * by 4 late frames in 1591 had every other frame miss the readout after it: the home screen's pages
	 * at 56 frames a second, logs/103400-run.log.)
	 */
	fScanTarget = 0;
	fScanLate = 0;
	if (gPanel.ok && fGraphics && !onGPU) {
		scanObserve();
		if (gPanel.ok) {
			uint64_t target = scanNextStart(began + fScanLead);

			/*
			 * Never the readout the frame before is for: that one would never be seen, written over
			 * before its readout. The one after it, then -- its stripes wait for that readout to
			 * pass, and the window server, its swap held meanwhile, keeps to the panel's pace.
			 */
			if (fScanLastTarget != 0 && target < fScanLastTarget + gPanel.period / 2) {
				target = fScanLastTarget + gPanel.period;
			}
			fScanTarget = fScanLastTarget = target;
		}
	}
	IOLockLock(fLock);
	if (fLastFrame != 0) {
		absolutetime_to_nanoseconds(began - fLastFrame, &nanos);
		nanos /= NSEC_PER_MSEC;
		if (nanos >= FRAME_PAUSE) {
			fFramePauses++;
		} else {
			fFrameTimes[nanos < FRAME_TIMES ? nanos : FRAME_TIMES]++;
			fFrameLongest = nanos > fFrameLongest ? (uint32_t) nanos : fFrameLongest;
		}
	}
	fLastFrame = began;
	report = fSwapsShown < 8;
	if (report) {
		printf("MiniIOMobileFramebuffer: swap %u (#%u): surfaces %u %u %u %u, layers set 0x%x cleared 0x%x\n",
		    swapWord(swap, SWAP_ID), fSwapsShown, swapWord(swap, SWAP_SURFACE(0)), swapWord(swap, SWAP_SURFACE(1)),
		    swapWord(swap, SWAP_SURFACE(2)), swapWord(swap, SWAP_SURFACE(3)), set, swapWord(swap, SWAP_LAYERS_CLEARED));
	}
	fOver = false;
	/* fb_yuv: the video layers first, under the rest (see gYUV) */
	for (unsigned layer = 0; gYUV != 0 && !onGPU && layer < SWAP_LAYERS; layer++) {
		MiniIOSurface * s = surfaces[layer];
		int32_t src[4], dst[4];

		if (s == NULL || s->info.planeCount != 2) {
			continue;
		}
		memcpy(src, swap + SWAP_SOURCE(layer), sizeof(src));
		memcpy(dst, swap + SWAP_DEST(layer), sizeof(dst));
		OSIncrementAtomic((volatile SInt32 *) &s->shownReading);
		if (drawYUV(s, src, dst)) {
			fOver = true;
		}
		OSDecrementAtomic((volatile SInt32 *) &s->shownReading);
	}
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		uint32_t sid = swapWord(swap, SWAP_SURFACE(layer));
		int32_t src[4], dst[4];
		MiniIOSurface * s;

		layers |= sid != 0 ? 1u << layer : 0;
		/* the frames' surfaces: is it a few, in turn? */
		for (unsigned i = 0; layer == 0 && sid != 0 && i < 8; i++) {
			if (fStatSurfaces[i] == 0 || fStatSurfaces[i] == sid) {
				fStatSurfaces[i] = sid;
				break;
			}
		}
		if (sid == 0) {
			/* the bottom layer taken away: nothing is shown, the screen is black */
			if (layer == 0 && (set & 1) && fGraphics && !fOver && !onGPU) { /* (fb_yuv: not over the video drawn) */
				/*
				 * Plain aligned stores, not bzero: the panel's memory is device memory under the phone's
				 * hypervisor, and bzero's DC ZVA and unaligned stores fault there ("Unaligned kernel data
				 * abort" in IOMobileFramebuffer::show, the first time SpringBoard dropped the bottom layer).
				 */
				for (uint32_t y = 0; y < fPanelHeight; y++) {
					if (fBackBuffer != NULL) {
						bzero(fBackBuffer + (size_t) y * fRowBytes, (size_t) fPanelWidth * 4);
					} else {
						volatile uint64_t *row = (volatile uint64_t *) (fBase + (size_t) y * fRowBytes);
						for (uint32_t x = 0; x < fPanelWidth / 2; x++) {
							row[x] = 0;
						}
					}
				}
			}
			continue;
		}
		memcpy(src, swap + SWAP_SOURCE(layer), sizeof(src));
		memcpy(dst, swap + SWAP_DEST(layer), sizeof(dst));
		s = surfaces[layer];
		if (report) {
			printf("MiniIOMobileFramebuffer: swap %u layer %u: surface %u%s (%ux%u '%c%c%c%c') "
			    "from %d,%d %dx%d to %d,%d %dx%d, layers set 0x%x\n", swapWord(swap, SWAP_ID), layer, sid,
			    s ? "" : " MISSING", s ? s->info.width : 0, s ? s->info.height : 0,
			    s ? (int) (s->info.pixelFormat >> 24 & 0xff) : '?', s ? (int) (s->info.pixelFormat >> 16 & 0xff) : '?',
			    s ? (int) (s->info.pixelFormat >> 8 & 0xff) : '?', s ? (int) (s->info.pixelFormat & 0xff) : '?',
			    src[0], src[1], src[2], src[3], dst[0], dst[1], dst[2], dst[3], set);
		}
		if (s == NULL) {
			continue;
		}
		if (s->info.bytesPerElement == 4 && s->info.planeCount == 0 && fOver) {
			/* fb_yuv: how much of this layer lets the video through, row by row, for the first swaps with one */
			static unsigned profiled;

			if (profiled < 3 && s->info.height != 0 &&
			    s->holds((uint64_t) (s->info.height - 1) * s->info.bytesPerRow, (uint64_t) s->info.width * 4)) {
				char line[512];
				size_t n = 0;
				const uint8_t * px = s->pixels();

				profiled++;
				for (uint32_t y = 0; px != NULL && y < s->info.height && n + 16 < sizeof(line); y += s->info.height >= 24 ? s->info.height / 24 : 1) {
					const uint32_t * row = (const uint32_t *) (px + (size_t) y * s->info.bytesPerRow);
					uint32_t clear = 0, solid = 0;

					for (uint32_t x = 0; x < s->info.width; x += 8) {
						uint32_t a = row[x] >> 24;
						clear += a == 0;
						solid += a == 255;
					}
					n += (size_t) snprintf(line + n, sizeof(line) - n, " %u:%u/%u", y, clear, solid);
				}
				IOLog("MiniIOMobileFramebuffer: fb_yuv: layer %u over the video (%ux%u), row: clear/opaque of every 8th pixel:%s\n",
				    layer, s->info.width, s->info.height, line);
			}
		}
		if (s->info.bytesPerElement == 4 && s->info.planeCount == 0) {
			if (!fGraphics) {
				/* the first picture: the kernel's text console must stop drawing over it */
				fGraphics = true;
				PE_initialize_console(NULL, kPEGraphicsMode);
				PE_initialize_console(NULL, kPEReleaseScreen);
			}
			if (!onGPU) {
				OSIncrementAtomic((volatile SInt32 *) &s->shownReading);
				drawLayer(s, src, dst);
				OSDecrementAtomic((volatile SInt32 *) &s->shownReading);
			}
			if (layer == 0) {
				recorded = s;   /* (recorded once the frame is in the panel's memory: see below) */
			}
		}
	}
	composed = mach_absolute_time();
	if (!onGPU) {
		copyBackBuffer();
	}
	if (fScanTarget != 0) {
		uint64_t up = usToAbs(500), down = usToAbs(100), floor = usToAbs(1000), ceiling = usToAbs(8000);

		fStatScanFrames++;
		if (fScanLate != 0) {
			fStatScanLate++;
			fScanCleanFrames = 0;
			fScanLead = fScanLead + up < ceiling ? fScanLead + up : ceiling;
		} else if (++fScanCleanFrames >= (hakios_display_scan_video() ? 10u : 60u)) {
			fScanCleanFrames = 0;
			fScanLead = fScanLead > floor + down ? fScanLead - down : floor;
		}
		scanObserve();
	}
	/*
	 * Only now is the frame in the panel's memory: the swap after it may be handed over. Given back any
	 * earlier, the window server would draw a frame more ahead of the panel than fb_queue says.
	 */
	IOLockLock(fSwapLock);
	fSwapFirst = (fSwapFirst + 1) % SWAP_QUEUE;
	fSwapsWaiting--;
	IOLockWakeup(fSwapLock, &fSwapsWaiting, true);
	IOLockUnlock(fSwapLock);
	/*
	 * (fb_gpu: the frame is in the GPU's ring, and whatever the window server draws next is behind it there.) fb_flip: the
	 * controller pointed at the buffer once the GPU is done with it, or back at the panel's own memory after the
	 * processors drew there; after the swap is handed on, so the next frame is drawn meanwhile.
	 */
	if (onGPU && fFlipTried && target != fFlipShown) {
		if (MiniIOSurfacePresentDone(gLastPresent, FLIP_TAKEN_MS)) {
			flipTo(target);
		} else {
			flipOff("the GPU not done with a frame in 100 ms");
		}
	} else if (!onGPU && fFlipTried && fFlipShown >= 0) {
		flipTo(-1);
	}
	drawn = mach_absolute_time();
	if (recorded != NULL) {
		record(recorded);
	}
	if (onGPU && (fGPUFrames == GPU_CHECK_FIRST || (fGPUFrames > GPU_CHECK_FIRST &&
	    (fGPUFrames - GPU_CHECK_FIRST) % GPU_CHECK_EVERY == 0) || gCheckNext != 0)) {
		gCheckNext -= gCheckNext != 0;
		checkOnGPU();
	}
	fSwapsShown++;
	if (gTouchLat) {
		hakios_tl("tl: v %u ready %llu shown %llu done %llu composed %llu target %llu late %u lead %llu\n",
		    swapWord(swap, SWAP_ID), micros(ready), micros(shown), micros(drawn), micros(composed),
		    fScanTarget != 0 ? micros(fScanTarget) : 0, fScanLate, micros(fScanLead));
	}
	if (layers != fLayersShown) {
		printf("MiniIOMobileFramebuffer: swap #%u: surfaces in layers 0x%x now (0x%x before): %u %u %u %u\n", fSwapsShown, layers,
		    fLayersShown, swapWord(swap, SWAP_SURFACE(0)), swapWord(swap, SWAP_SURFACE(1)), swapWord(swap, SWAP_SURFACE(2)),
		    swapWord(swap, SWAP_SURFACE(3)));
		fLayersShown = layers;
	}

	/* every five seconds in which something was drawn: how many frames, and what each cost here */
	fStatFrames++;
	fStatDrawing += mach_absolute_time() - began;
	if (fStatSince == 0) {
		fStatSince = began;
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - fStatSince, &nanos);
	if (nanos >= 5 * NSEC_PER_SEC) {
		uint64_t drawing, waiting, beat;
		unsigned different;

		absolutetime_to_nanoseconds(fStatDrawing, &drawing);
		absolutetime_to_nanoseconds(fStatWaiting, &waiting);
		absolutetime_to_nanoseconds(fStatBeatWait, &beat);
		IOLog("MiniIOMobileFramebuffer: %u frames in %llu ms (%llu a second), %llu us each in the driver, after %llu us "
		    "waiting for the GPU and, %u of them, %llu us for the next refresh; %u swaps waited for the frame before, "
		    "%u gave up; %u swaps came a refresh late themselves\n", fStatFrames, nanos / NSEC_PER_MSEC,
		    fStatFrames * NSEC_PER_SEC / nanos, drawing / fStatFrames / NSEC_PER_USEC,
		    waiting / fStatFrames / NSEC_PER_USEC, fStatOnBeat, fStatOnBeat ? beat / fStatOnBeat / NSEC_PER_USEC : 0,
		    fStatHeld, fStatLost, fStatSwapsLate);
		IOLog("MiniIOMobileFramebuffer: %u swaps came before the GPU work of their surface, %u of them shown without it\n",
		    fStatStale, fStatStaleShown);
		/* fb_gpu: who put the frames on the panel, what the GPU's part cost, and why a frame was not its */
		{
			uint32_t presents, timed, mapped;
			uint64_t submitting, gpu;

			MiniIOSurfacePresentCost(&presents, &submitting, &gpu, &timed, &mapped);
			IOLog("MiniIOMobileFramebuffer: present: %u frames by the processors; %u by the GPU, %llu us each to hand over, "
			    "%llu us of the GPU's (%u timed); %u surfaces entered into its address space; not by the GPU: %u no panel "
			    "address, %u black, %u off the panel, %u drawn by a processor, %u nothing to draw, %u a video, %u refused "
			    "by it\n", fStatCPUFrames, fStatGPUFrames, submitting, gpu, timed, mapped, fStatRefused[kRefusedNoPanel],
			    fStatRefused[kRefusedBlack], fStatRefused[kRefusedOffPanel], fStatRefused[kRefusedNotGPUDrawn],
			    fStatRefused[kRefusedNothing], fStatRefused[kRefusedVideo], fStatRefused[kRefusedByGPU]);
			fStatCPUFrames = fStatGPUFrames = 0;
			bzero(fStatRefused, sizeof(fStatRefused));
			if (fFlipTried && (gFlipState == 1 || fStatFlips != 0)) {
				IOLog("MiniIOMobileFramebuffer: fb_flip: %u flips (%u back to the panel's memory), %u frames not covering the "
				    "panel; %u waited for the last flush to be taken, %llu us each; shown now %d\n", fStatFlips,
				    fStatFlipsHome, fStatFlipNotFull, fStatFlipWaited,
				    fStatFlipWaited ? micros(fStatFlipWait) / fStatFlipWaited : 0, fFlipShown);
				fStatFlips = fStatFlipsHome = fStatFlipNotFull = fStatFlipWaited = 0;
				fStatFlipWait = 0;
			}
			/* fb_video: frames with a video on the controller, and whether the panel's interface ran short meanwhile */
			if (gVideoState == 1 && (fStatVideoFrames != 0 || fVideoOn)) {
				bool under = (mdssRead(MDP_INTR_STATUS) & MDP_INTR_INTF1_UNDERRUN) != 0;

				fStatVideoUnderruns += under;
				mdssWrite(MDP_INTR_CLEAR, MDP_INTR_INTF1_UNDERRUN);
				IOLog("MiniIOMobileFramebuffer: fb_video: %u frames with a video by the display controller, %u not taken "
				    "(the processors drew them); interface underrun %s (%u times seen)\n", fStatVideoFrames,
				    fStatVideoRefused, under ? "seen" : "not seen", fStatVideoUnderruns);
				fStatVideoFrames = 0;
				fStatVideoRefused = 0;
			}
		}
		if (surfaces[0] != NULL) {
			IOLog("MiniIOMobileFramebuffer: the last swap's surface %u: its last GPU submission %u (0: none ever named it)\n",
			    surfaces[0]->sid, surfaces[0]->drawn);
		}
		fStatSwapsLate = fStatStale = fStatStaleShown = 0;
		/*
		 * The last swap as it came, every five seconds rather than for the first eight: the
		 * first ones are lost from a long run's log, and with CoreAnimation's GPU renderer the
		 * panel stayed on the loader's picture through hundreds of swaps that each cost 0 us
		 * here -- swaps with no surface in them, or with one not found. The words of the header
		 * that are not zero, offset:value, for what the window server put where.
		 */
		{
			char words[400];
			unsigned at = 0;

			for (unsigned off = 0; off < 0x110 && at < sizeof(words) - 24; off += 4) {
				uint32_t w = swapWord(swap, off);

				if (w != 0) {
					at += (unsigned) snprintf(words + at, sizeof(words) - at, " %x:%x", off, w);
				}
			}
			words[at] = '\0';
			IOLog("MiniIOMobileFramebuffer: the last swap: surfaces %u %u %u %u (found %d %d %d %d); words%s\n",
			    swapWord(swap, SWAP_SURFACE(0)), swapWord(swap, SWAP_SURFACE(1)), swapWord(swap, SWAP_SURFACE(2)),
			    swapWord(swap, SWAP_SURFACE(3)), surfaces[0] != NULL, surfaces[1] != NULL, surfaces[2] != NULL,
			    surfaces[3] != NULL, words);
		}
		for (different = 0; different < 8 && fStatSurfaces[different] != 0; different++) {
		}
		if (different == 8 && fStatFrames > 8) {
			uint32_t made, freed, again;
			uint64_t making, freeing;

			/* (CoreAnimation has made itself a new surface for every frame: see "the frames' surfaces" above) */
			MiniIOSurfaceCost(&made, &making, &freed, &freeing, &again);
			IOLog("MiniIOMobileFramebuffer: the frames came in surfaces %u, %u, %u, %u, %u and on, none twice; in that time "
			    "%u surfaces of all kinds were made, in %llu us each (%u of them of memory kept from one before), and %u "
			    "freed, in %llu us each\n", fStatSurfaces[0], fStatSurfaces[1], fStatSurfaces[2], fStatSurfaces[3],
			    fStatSurfaces[4], made, making, again, freed, freeing);
		}
		bzero(fStatSurfaces, sizeof(fStatSurfaces));
		fStatHeld = fStatLost = fStatOnBeat = 0;
		fStatWaiting = fStatBeatWait = 0;
		reportFrameTimes();
		if (fDiff) {
			IOLog("MiniIOMobileFramebuffer: %llu pixels of the panel changed a frame, all within %u,%u to %u,%u\n",
			    fDiffPixels / fStatFrames, fDiffLeft, fDiffTop, fDiffRight, fDiffBottom);
			fDiffPixels = 0;
			fDiffLeft = fDiffTop = ~0u;
			fDiffRight = fDiffBottom = 0;
		}
		fStatFrames = 0;
		fStatDrawing = 0;
		fStatSince = 0;
	}
	IOLockUnlock(fLock);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class IOMobileFramebufferUserClient : public IOUserClient
{
	OSDeclareDefaultStructors(IOMobileFramebufferUserClient);

	task_t                  fTask;
	IOMobileFramebuffer *   fOwner;
	IOLock *                fLock;
	mach_port_t             fPorts[kFBNotifyTypes];
	mach_vm_address_t       fCallbacks[kFBNotifyTypes];
	io_user_reference_t     fRefcons[kFBNotifyTypes];
	thread_call_t           fVSyncCall;
	uint64_t                fVSyncCount;
	uint64_t                fVSyncTime;             /* when the refresh being waited for is due */
	bool                    fVSyncRunning;
	bool                    fClosed;

	static void vsyncFired(thread_call_param_t self, thread_call_param_t unused);
	void vsync(void);
	void armVSync(void);

public:
	virtual bool initWithTask(task_t owningTask, void * securityID, UInt32 type, OSDictionary * properties) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments * args,
	    IOExternalMethodDispatch * dispatch, OSObject * target, void * reference) APPLE_KEXT_OVERRIDE;
private:
	IOReturn method(uint32_t selector, IOExternalMethodArguments * args);
};

OSDefineMetaClassAndStructors(IOMobileFramebufferUserClient, IOUserClient);

bool
IOMobileFramebufferUserClient::initWithTask(task_t owningTask, void * securityID, UInt32 type, OSDictionary * properties)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type, properties)) {
		return false;
	}
	/*
	 * (held, not only named: the send rights it makes go into this task's space. A program that hands its connection to
	 * another and exits is gone before the connection is, and the other's calls must not find a task that has been freed)
	 */
	fTask = owningTask;
	task_reference(fTask);
	{
		char procname[32] = "?";
		proc_selfname(procname, sizeof(procname));
		IOLog("MiniIOMobileFramebuffer: opened by %s[%d], type %u\n", procname, proc_selfpid(), (unsigned) type);
	}
	return true;
}

bool
IOMobileFramebufferUserClient::start(IOService * provider)
{
	fOwner = OSDynamicCast(IOMobileFramebuffer, provider);
	if (fOwner == NULL || !IOUserClient::start(provider)) {
		return false;
	}
	fLock = IOLockAlloc();
	fVSyncCall = thread_call_allocate(&IOMobileFramebufferUserClient::vsyncFired, this);
	return fLock != NULL && fVSyncCall != NULL;
}

IOReturn
IOMobileFramebufferUserClient::clientClose(void)
{
	if (fLock != NULL) {
		IOLockLock(fLock);
		fClosed = true;
		IOLockUnlock(fLock);
	}
	if (fVSyncCall != NULL) {
		thread_call_cancel_wait(fVSyncCall);
	}
	if (!isInactive()) {
		terminate();
	}
	return kIOReturnSuccess;
}

void
IOMobileFramebufferUserClient::free(void)
{
	if (fVSyncCall != NULL) {
		thread_call_cancel_wait(fVSyncCall);
		thread_call_free(fVSyncCall);
		fVSyncCall = NULL;
	}
	for (unsigned i = 0; i < kFBNotifyTypes; i++) {
		if (fPorts[i] != NULL) {
			releaseNotificationPort(fPorts[i]);
			fPorts[i] = NULL;
		}
	}
	if (fLock != NULL) {
		IOLockFree(fLock);
		fLock = NULL;
	}
	if (fTask != NULL) {
		task_deallocate(fTask);
		fTask = NULL;
	}
	IOUserClient::free();
}

IOReturn
IOMobileFramebufferUserClient::registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon)
{
	mach_port_t old;

	IOLog("MiniIOMobileFramebuffer: notification port for type %u\n", (unsigned) type);
	if (type >= kFBNotifyTypes) {
		return kIOReturnBadArgument;
	}
	IOLockLock(fLock);
	old = fPorts[type];
	fPorts[type] = port;
	if (type == kFBNotifyVSync && port != NULL && fCallbacks[type] != 0 && !fVSyncRunning && !fClosed) {
		/* (the refreshes asked for before there was a port to send them to stopped at the first: started again) */
		fVSyncRunning = true;
		armVSync();
	}
	IOLockUnlock(fLock);
	if (old != NULL) {
		releaseNotificationPort(old);
	}
	return kIOReturnSuccess;
}

void
IOMobileFramebufferUserClient::vsyncFired(thread_call_param_t self, thread_call_param_t unused)
{
	((IOMobileFramebufferUserClient *) self)->vsync();
}

/* With the lock held. */
void
IOMobileFramebufferUserClient::armVSync(void)
{
	uint64_t period, now = mach_absolute_time();

	/* one refresh after the last, not after whenever we got round to this: the rate does not drift */
	if (gPanel.ok) {
		/*
		 * The panel's own refresh: the next of its readouts to begin, at least half a refresh after
		 * the one told last (none twice, and none left out for a timer that ran late)
		 */
		scanObserve();
		period = gPanel.period;
		if (gPanel.ok) {
			uint64_t next = scanNextStart(now + usToAbs(500));

			if (fVSyncTime != 0 && next < fVSyncTime + period / 2) {
				next += period;
			}
			fVSyncTime = next;
			thread_call_enter_delayed(fVSyncCall, fVSyncTime);
			return;
		}
	}
	nanoseconds_to_absolutetime(NSEC_PER_SEC / refreshHz(), &period);
	fVSyncTime += period;
	if (fVSyncTime <= now || fVSyncTime > now + 2 * period) {
		fVSyncTime = now + period;
	}
	thread_call_enter_delayed(fVSyncCall, fVSyncTime);
}

/*
 * One refresh: tell the client, which gets three 64-bit numbers as six 32-bit halves --
 * the time of this refresh, the same again (the time it was presented), and a counter.
 */
void
IOMobileFramebufferUserClient::vsync(void)
{
	OSAsyncReference64 ref;
	io_user_reference_t args[6];
	uint64_t now;
	bool send = false;

	IOLockLock(fLock);
	now = fVSyncTime;
	if (fClosed || fCallbacks[kFBNotifyVSync] == 0 || fPorts[kFBNotifyVSync] == NULL) {
		fVSyncRunning = false;
	} else {
		fVSyncCount++;
		setAsyncReference64(ref, fPorts[kFBNotifyVSync], fCallbacks[kFBNotifyVSync], fRefcons[kFBNotifyVSync], fTask);
		args[0] = (uint32_t) now;
		args[1] = now >> 32;
		args[2] = (uint32_t) now;
		args[3] = now >> 32;
		args[4] = (uint32_t) fVSyncCount;
		args[5] = fVSyncCount >> 32;
		send = true;
		armVSync();
		/*
		 * sent with the lock held: registerNotificationPort() swaps the port under it and lets the old one go after, and
		 * a send after the lock was let go could be to a port freed meanwhile. (The send drops rather than waits: it does
		 * not hold the lock for long.)
		 */
		sendAsyncResult64WithOptions(ref, kIOReturnSuccess, args, 6, kIOUserNotifyOptionCanDrop);
	}
	IOLockUnlock(fLock);
	if (send) {
		uint64_t late;

		absolutetime_to_nanoseconds(mach_absolute_time() - now, &late);
		if (gTouchLat && late > NSEC_PER_MSEC) {
			/* tl: y COUNT due DUE late US: a refresh told to the window server over a millisecond late */
			hakios_tl("tl: y %llu due %llu late %llu\n", fVSyncCount, micros(now), late / NSEC_PER_USEC);
		}
	} else if (gTouchLat) {
		hakios_tl("tl: Y off at %llu\n", micros(mach_absolute_time()));
	}
}

/*
 * A call made asynchronously hands over a send right to its wake port, which is the driver's once the call succeeds
 * (MIG takes it back only on an error). None of these calls keeps it -- the answers go to the notification ports -- so it
 * is given back here, or every such call would leave a port behind for good.
 */
IOReturn
IOMobileFramebufferUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments * args,
    IOExternalMethodDispatch * dispatch, OSObject * target, void * reference)
{
	IOReturn result;

	/*
	 * (closed: its program is gone, or let go of it. Whoever else has the connection still -- it was handed on -- is not
	 * served in its place: the answers would be made in the space of the task that opened it)
	 */
	IOLockLock(fLock);
	result = fClosed ? kIOReturnNotOpen : kIOReturnSuccess;
	IOLockUnlock(fLock);
	if (result == kIOReturnSuccess) {
		result = method(selector, args);
	}

	(void) dispatch;
	(void) target;
	(void) reference;
	if (result == kIOReturnSuccess && args->asyncWakePort != MACH_PORT_NULL) {
		releaseAsyncReference64(args->asyncReference);
	}
	return result;
}

IOReturn
IOMobileFramebufferUserClient::method(uint32_t selector, IOExternalMethodArguments * args)
{
	const uint64_t * in = args->scalarInput;
	uint64_t * out = args->scalarOutput;
	MiniIOSurface * s;

	if (selector < 128 && gFBCalls[selector] < gFBTrace) {
		char procname[32] = "?";

		gFBCalls[selector]++;
		proc_selfname(procname, sizeof(procname));
		/* printf and not IOLog: both reach the kernel's log, which the loader brings back from the
		 * phone, and only printf reaches a serial console -- QEMU's, where these lines are how one
		 * sees whether the window server swaps at all */
		printf("MiniIOMobileFramebuffer: %s[%d] selector %u (scalars %u in [0x%llx 0x%llx 0x%llx 0x%llx], %u out; "
		    "structure %u in, %u out)\n", procname, proc_selfpid(), selector, args->scalarInputCount,
		    args->scalarInputCount > 0 ? in[0] : 0, args->scalarInputCount > 1 ? in[1] : 0,
		    args->scalarInputCount > 2 ? in[2] : 0, args->scalarInputCount > 3 ? in[3] : 0, args->scalarOutputCount,
		    args->structureInputSize, args->structureOutputSize);
	}

	switch (selector) {
	case kFBMethodGetDisplaySize:                   /* out: width, height */
		if (args->scalarOutputCount < 2) {
			return kIOReturnBadArgument;
		}
		out[0] = fOwner->fWidth;
		out[1] = fOwner->fHeight;
		args->scalarOutputCount = 2;
		return kIOReturnSuccess;

	case kFBMethodGetID:
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = 1;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodGetDotPitch:                      /* out: pixels per inch */
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = kFBDotsPerInch * fOwner->fWidth / kFBDefaultWidth;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodGetLayerDefaultSurface:           /* in: size wanted (or 0, 0); out: a send right for the surface */
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		s = fOwner->defaultSurface();
		if (s == NULL) {
			return kIOReturnNoMemory;
		}
		out[0] = MiniIOSurfaceMakeSendRight(fTask, s);
		args->scalarOutputCount = 1;
		return out[0] != 0 ? kIOReturnSuccess : kIOReturnNoMemory;

	case kFBMethodSwapBegin:                        /* out: the number of the new swap */
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = OSIncrementAtomic(&fOwner->fSwapID) + 1;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodSwapEnd:
		if (args->structureInput == NULL || args->structureInputSize < SWAP_SIZE) {
			IOLog("MiniIOMobileFramebuffer: swap request of %u bytes, expected %u\n", args->structureInputSize, SWAP_SIZE);
			return kIOReturnBadArgument;
		}
		if (args->structureInputSize >= SWAP26_SIZE) {
			/*
			 * Darwin 26's request (0x588 bytes) is Darwin 16.5's with 0x48 bytes more ahead of the swap's own
			 * fields: the ID, the layers' surfaces, source rectangles, flags, destination rectangles and the
			 * masks of the layers set and cleared all sit 0x48 further on (kern_SwapBegin/SwapSetLayer in
			 * IOMobileFramebuffer.framework). Seen from there it is the old one.
			 */
			uint8_t old[SWAP_SIZE];

			memcpy(old, (const uint8_t *) args->structureInput + SWAP26_SHIFT, SWAP_SIZE);
			fOwner->present(old);
			return kIOReturnSuccess;
		}
		fOwner->present((const uint8_t *) args->structureInput);
		return kIOReturnSuccess;

	case kFBMethodSwapWait:                         /* a swap is on the screen as soon as it is made */
		if (gTouchLat) {
			hakios_tl("tl: w %llu %#llx %#llx at %llu\n", args->scalarInputCount > 0 ? in[0] : 0,
			    args->scalarInputCount > 1 ? in[1] : 0, args->scalarInputCount > 2 ? in[2] : 0, micros(mach_absolute_time()));
		}
		return kIOReturnSuccess;
	case kFBMethodSwapSignal:
	case kFBMethodSwapCancel:
	case kFBMethodSwapCancelAll:
	case kFBMethodRequestPowerChange:
	case kFBMethodChangeFrameInfo:
		return kIOReturnSuccess;

	case kFBMethodNotifications:                    /* in: callback (0 = off), its argument, type, 0 */
		if (args->scalarInputCount < 3 || in[2] >= kFBNotifyTypes) {
			return kIOReturnBadArgument;
		}
		IOLockLock(fLock);
		fCallbacks[in[2]] = in[0];
		fRefcons[in[2]] = in[1];
		if (in[2] == kFBNotifyVSync && gTouchLat) {
			hakios_tl("tl: Y %s at %llu\n", in[0] != 0 ? (fVSyncRunning ? "on-running" : "on") : "asked-off",
			    micros(mach_absolute_time()));
		}
		if (in[2] == kFBNotifyVSync && in[0] != 0 && !fVSyncRunning && !fClosed) {
			fVSyncRunning = true;
			armVSync();
		}
		IOLockUnlock(fLock);
		return kIOReturnSuccess;

	case kFBMethodSupportedFrameInfo:               /* in: type; out: how many, and their tags: none */
		if (args->scalarOutputCount >= 1) {
			out[0] = 0;
			args->scalarOutputCount = 1;
		}
		if (args->structureOutput != NULL) {
			bzero(args->structureOutput, args->structureOutputSize);
		}
		return kIOReturnSuccess;

	case kFBMethodGetBlock:                         /* in: which block; out: its bytes */
		if (args->structureOutput == NULL) {
			return kIOReturnBadArgument;
		}
		bzero(args->structureOutput, args->structureOutputSize);
		if (args->scalarInputCount >= 1 && in[0] == 0x40 && args->structureOutputSize >= 12) {
			/* the current mode: its colour and timing identifiers */
			uint32_t ids[2] = { kFBColorID, kFBTimingID };

			memcpy((uint8_t *) args->structureOutput + 4, ids, sizeof(ids));
		}
		return kIOReturnSuccess;

	case kFBMethodGetCurrentAbsoluteTime:
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = mach_absolute_time();
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodIsMainDisplay:                    /* out: 1 = the main display; the only one here */
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = 1;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodSetDebugFlags:                    /* in: bits to clear, bits to set; out: the flags now */
		if (args->scalarInputCount < 2 || args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		fOwner->fDebugFlags = (fOwner->fDebugFlags & ~(uint32_t) in[0]) | (uint32_t) in[1];
		out[0] = fOwner->fDebugFlags;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodSetGammaTable:                    /* kept for GetGammaTable; the panel is not changed */
		if (args->structureInput == NULL || args->structureInputSize != kFBGammaSize) {
			return kIOReturnBadArgument;
		}
		memcpy(fOwner->fGamma, args->structureInput, kFBGammaSize);
		fOwner->fGammaSet = true;
		return kIOReturnSuccess;

	case kFBMethodGetGammaTable:                    /* the last one set, zeros before that */
		if (args->structureOutput == NULL || args->structureOutputSize != kFBGammaSize) {
			return kIOReturnBadArgument;
		}
		if (fOwner->fGammaSet) {
			memcpy(args->structureOutput, fOwner->fGamma, kFBGammaSize);
		} else {
			bzero(args->structureOutput, kFBGammaSize);
		}
		return kIOReturnSuccess;

	case kFBMethodGetDisplayArea:                   /* out: 8 bytes, width and height */
		if (args->structureOutput == NULL || args->structureOutputSize < 8) {
			return kIOReturnBadArgument;
		} else {
			uint32_t area[2] = { fOwner->fWidth, fOwner->fHeight };

			bzero(args->structureOutput, args->structureOutputSize);
			memcpy(args->structureOutput, area, sizeof(area));
		}
		return kIOReturnSuccess;

	case kFBMethodGetDigitalOutState:               /* out: no digital output connected */
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = 0;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodSetColorRemapMode:
		if (args->scalarInputCount < 1) {
			return kIOReturnBadArgument;
		}
		fOwner->fColorRemapMode = (uint32_t) in[0];
		return kIOReturnSuccess;

	case kFBMethodGetColorRemapMode:
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = fOwner->fColorRemapMode;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	/* settings for the panel's own colour, brightness and power hardware, which is not driven here: accepted */
	case kFBMethodSetDisplayDevice:
	case kFBMethodVideoPowerSavings:
	case kFBMethodSetBrightnessCorrection:
	case kFBMethodSetMatrix:
	case kFBMethodSetParameter:
	case kFBMethodSetContrast:
	case kFBMethodOpenedByName:
		return kIOReturnSuccess;

	default:
		break;
	}

	if (selector < 128 && !(gFBLogged[selector / 64] & (1ULL << (selector % 64)))) {
		gFBLogged[selector / 64] |= 1ULL << (selector % 64);
		IOLog("MiniIOMobileFramebuffer: selector %u not implemented (scalars %u in [0x%llx 0x%llx 0x%llx], %u out; "
		    "structure %u in, %u out)\n", selector, args->scalarInputCount, args->scalarInputCount > 0 ? in[0] : 0,
		    args->scalarInputCount > 1 ? in[1] : 0, args->scalarInputCount > 2 ? in[2] : 0, args->scalarOutputCount,
		    args->structureInputSize, args->structureOutputSize);
	}
	/* a request that only sets something is accepted; one that wants an answer gets zeros */
	if (args->scalarOutputCount != 0) {
		bzero(out, args->scalarOutputCount * sizeof(out[0]));
	}
	if (args->structureOutput != NULL && args->structureOutputSize != 0) {
		bzero(args->structureOutput, args->structureOutputSize);
	}
	return kIOReturnSuccess;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*
 * IOMobileFramebuffer.framework counts the displays it should wait for by looking for
 * children of /arm-io in the device tree whose device_type is "display-subsystem".
 */
static void
publishDisplayNode(void)
{
	IORegistryEntry * armio = IORegistryEntry::fromPath("/arm-io", gIODTPlane);
	IORegistryEntry * node;
	OSDictionary * props;
	OSData * data;

	if (armio == NULL) {
		IOLog("MiniIOMobileFramebuffer: no /arm-io in the device tree\n");
		return;
	}
	node = OSTypeAlloc(IORegistryEntry);
	props = OSDictionary::withCapacity(4);
	if (node != NULL && props != NULL) {
		data = OSData::withBytes("display-subsystem", sizeof("display-subsystem"));
		props->setObject("device_type", data);
		OSSafeReleaseNULL(data);
		data = OSData::withBytes("hakios,framebuffer", sizeof("hakios,framebuffer"));
		props->setObject("compatible", data);
		OSSafeReleaseNULL(data);
		data = OSData::withBytes("disp0", sizeof("disp0"));
		props->setObject("name", data);
		OSSafeReleaseNULL(data);
		if (node->init(props)) {
			node->setName("disp0");
			node->attachToParent(armio, gIODTPlane);
		}
	}
	OSSafeReleaseNULL(props);
	OSSafeReleaseNULL(node);
	armio->release();
}

/*
 * The graphics accelerator. There is none, but CoreAnimation waits half a minute for a
 * service of this class to appear before it looks for a GPU at all (once per process). An
 * accelerator without the properties that name its Metal and OpenGL ES drivers is passed
 * over by both, which leaves CoreAnimation with its software renderer, as before.
 *
 * With boot-arg metal=1 the service names our Metal plug-in from the start: MetalPluginName
 * "HakiosMetal", MetalPluginClassName "HakiosMetalDevice" (components/metal/plugin/plugin/HakiosMetalCA.m, installed as
 * /System/Library/Extensions/HakiosMetal.bundle), and a program that asks Metal for a device gets
 * ours -- CoreAnimation in backboardd among them, which then draws the screen with the GPU. A user
 * process may also remove the two, or put them back as they were (setProperties below): removing
 * them is the way back to the software renderer without a reboot. Without metal=1 (the default)
 * nothing changes: CoreAnimation keeps its software renderer, as in the RAM-only release. See
 * Documentation/metal-glass.md.
 */

/* Metal's step 2 reads exactly these two, in this order, off this service (docs/metal.md 1). */
#define kAcceleratorPluginNameKey      "MetalPluginName"
#define kAcceleratorPluginClassNameKey "MetalPluginClassName"
/* ... and the only values they take here: our plug-in's (components/metal/plugin) */
#define kAcceleratorPluginName         "HakiosMetal"
#define kAcceleratorPluginClassName    "HakiosMetalDevice"

static bool gAcceleratorTakesProperties;        /* boot-arg metal=1 */

class IOAcceleratorES : public IOService
{
	OSDeclareDefaultStructors(IOAcceleratorES);

public:
	virtual IOReturn setProperties(OSObject * properties) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(IOAcceleratorES, IOService);

/*
 * Switch Metal's own device discovery on, from user space: IORegistryEntrySetCFProperties() with
 * MetalPluginName and MetalPluginClassName. Metal reads the first (getMetalPluginClassForService,
 * 0x181a39ba4), builds "/System/Library/Extensions/" + it + ".bundle", opens that with NSBundle
 * and takes the class the second names; with the first missing it returns nil before it even
 * reads the second, and that single `cbz` is where the chain breaks on this port today.
 *
 * This method is the only thing that was in the way. IORegistryEntry::setProperties is
 * "return kIOReturnUnsupported" (IORegistryEntry.cpp) and IOService does not override it; the
 * MAC check in front of the call has no policy to ask, because this kernel registers no iokit
 * hook (bsd/kern/kern_minimac.c); and this service carries no property whitelist.
 *
 * Why it is gated on a boot-arg, which is a measured risk and not caution in general
 * (docs/metal.md 10.14): the conditions Metal puts on a device object are asserts, not soft
 * refusals (10.2-10.3). A process that asks Metal and is handed a half-made device does not
 * degrade to another renderer -- it aborts ("Assertion failed: ([self
 * conformsToProtocol:@protocol(MTLDeviceSPI)])", "failed assertion _limits.maxColorAttachments
 * > 0"), and an aborting daemon is restarted by launchd, which is a loop. Published instead at
 * registerService() time, the properties would exist before launchd, backboardd and SpringBoard
 * are born, so each of the three would take the Metal road on its first call, with nobody left
 * to read the log. Behind a boot-arg, a machine that has them is a machine booted on purpose to
 * have them, and the first such boot is meant to be RC=rc.darwin, where launchd is never born.
 *
 * An empty string removes a property rather than setting it. That is the way out of the loop
 * above without a reboot: the properties live only in the registry, so processes started after
 * they are gone go back to the software road.
 *
 * Everything else is refused, and refused whole, before anything is set: every process on the
 * machine can reach this method and the only thing it exists for is this one switch. That goes for
 * the values too: each property is either our plug-in's own name (kAcceleratorPluginName,
 * kAcceleratorPluginClassName) or empty. Metal makes a path of the first, so any other string
 * -- "../../../tmp/x", say -- would have every process that asks Metal for a device afterwards
 * load a bundle of the sender's choosing; compared whole with our name, no "/" or ".." gets by.
 */
IOReturn
IOAcceleratorES::setProperties(OSObject * properties)
{
	OSDictionary * dict = OSDynamicCast(OSDictionary, properties);
	OSCollectionIterator * keys;
	OSObject * next;
	char who[32] = "?";

	if (!gAcceleratorTakesProperties) {
		static unsigned said;

		if (said++ < 4) {
			proc_selfname(who, sizeof(who));
			IOLog("IOAcceleratorES: %s asks to set properties on it; boot-arg metal=1 was not given\n", who);
		}
		return kIOReturnUnsupported;
	}
	if (dict == NULL || dict->getCount() == 0) {
		return kIOReturnBadArgument;
	}
	keys = OSCollectionIterator::withCollection(dict);
	if (keys == NULL) {
		return kIOReturnNoMemory;
	}
	proc_selfname(who, sizeof(who));
	while ((next = keys->getNextObject()) != NULL) {
		const OSSymbol * key = OSDynamicCast(OSSymbol, next);

		if (key == NULL || (!key->isEqualTo(kAcceleratorPluginNameKey) &&
		    !key->isEqualTo(kAcceleratorPluginClassNameKey))) {
			IOLog("IOAcceleratorES: refusing what %s sent: only " kAcceleratorPluginNameKey " and "
			    kAcceleratorPluginClassNameKey " are taken here\n", who);
			keys->release();
			return kIOReturnUnsupported;
		}
		OSString * value = OSDynamicCast(OSString, dict->getObject(key));

		if (value == NULL) {
			IOLog("IOAcceleratorES: refusing \"%s\" from %s: Metal reads it as a CFString\n",
			    key->getCStringNoCopy(), who);
			keys->release();
			return kIOReturnBadArgument;
		}
		if (value->getLength() != 0 && !value->isEqualTo(key->isEqualTo(kAcceleratorPluginNameKey) ?
		    kAcceleratorPluginName : kAcceleratorPluginClassName)) {
			IOLog("IOAcceleratorES: refusing %s from %s: it is \"%s\" or nothing\n", key->getCStringNoCopy(), who,
			    key->isEqualTo(kAcceleratorPluginNameKey) ? kAcceleratorPluginName : kAcceleratorPluginClassName);
			keys->release();
			return kIOReturnBadArgument;
		}
	}
	keys->reset();
	while ((next = keys->getNextObject()) != NULL) {
		const OSSymbol * key = OSDynamicCast(OSSymbol, next);
		OSString * value = OSDynamicCast(OSString, dict->getObject(key));

		if (value->getLength() == 0) {
			removeProperty(key);
			IOLog("IOAcceleratorES: %s removed by %s\n", key->getCStringNoCopy(), who);
		} else if (setProperty(key, value)) {
			IOLog("IOAcceleratorES: %s = %s, set by %s\n", key->getCStringNoCopy(),
			    value->getCStringNoCopy(), who);
		} else {
			IOLog("IOAcceleratorES: %s could not be set\n", key->getCStringNoCopy());
			keys->release();
			return kIOReturnNoMemory;
		}
	}
	keys->release();
	return kIOReturnSuccess;
}

static void
publishAccelerator(IOService * provider)
{
	IOAcceleratorES * accelerator = OSTypeAlloc(IOAcceleratorES);
	uint32_t metal = 0;

	gAcceleratorTakesProperties = PE_parse_boot_argn("metal", &metal, sizeof(metal)) && metal != 0;
	if (accelerator != NULL && accelerator->init() && accelerator->attach(provider)) {
		accelerator->setName("sgx");
		/* (before registerService: whoever finds the service finds it with them) */
		if (gAcceleratorTakesProperties) {
			accelerator->setProperty(kAcceleratorPluginNameKey, kAcceleratorPluginName);
			accelerator->setProperty(kAcceleratorPluginClassNameKey, kAcceleratorPluginClassName);
			/* the first line to look for in the log of such a boot */
			IOLog("IOAcceleratorES: metal=%u: Metal's plug-in is HakiosMetal (HakiosMetalDevice); user space may "
			    "remove or restore " kAcceleratorPluginNameKey " and " kAcceleratorPluginClassNameKey "\n", metal);
		}
		accelerator->registerService();
	}
	OSSafeReleaseNULL(accelerator);
}

/*
 * A picture from inside the kernel (the GPU driver's first ones): rows of 32-bit pixels, put at
 * the top left corner of the panel as they are and cut to its size.
 */
extern "C" void
MiniIOMobileFramebufferShow(const uint8_t * pixels, uint32_t width, uint32_t height, uint32_t rowBytes)
{
	static bool graphics = false;
	uint32_t columns = width < vinfo.v_width ? width : (uint32_t) vinfo.v_width;
	uint32_t rows = height < vinfo.v_height ? height : (uint32_t) vinfo.v_height;

	if (vinfo.v_baseaddr == 0 || vinfo.v_depth != 32) {
		return;
	}
	if (!graphics) {
		/* Direct GPU presentation bypasses drawLayer's console handover. */
		graphics = true;
		PE_initialize_console(NULL, kPEGraphicsMode);
		PE_initialize_console(NULL, kPEReleaseScreen);
	}
	if (gLastPresent != 0) {
		MiniIOSurfacePresentDone(gLastPresent, 100);    /* (fb_gpu: the GPU is not to write over it) */
	}
	for (uint32_t y = 0; y < rows; y++) {
		memcpy((void *) (vinfo.v_baseaddr + (size_t) y * vinfo.v_rowbytes), pixels + (size_t) y * rowBytes,
		    (size_t) columns * 4);
	}
	/*
	 * fb_flip: the controller may be on one of its buffers, not the panel's memory: the picture into that one too, so it
	 * is seen (until the window server's next frame, as on the panel's memory)
	 */
	{
		uint8_t * shown = (uint8_t *) gScreenBase;
		uint32_t stride = gScreenRowBytes;

		if (shown != NULL && stride >= columns * 4) {
			for (uint32_t y = 0; y < rows; y++) {
				memcpy(shown + (size_t) y * stride, pixels + (size_t) y * rowBytes, (size_t) columns * 4);
			}
		}
	}
}

/* At the run's end, and on a panic's way out (osfmk/arm64/loader_return.c): fb_flip's controller back home. */
extern "C" void
hakios_fb_quiesce(void)
{
	if (gFlipFramebuffer != NULL) {
		gFlipFramebuffer->flipHome();
	}
}

/* The panel's size in pixels, as MiniIOMobileFramebufferShow() cuts to it (0 x 0: there is no framebuffer to show on). */
extern "C" void
MiniIOMobileFramebufferPanel(uint32_t * width, uint32_t * height)
{
	bool usable = vinfo.v_baseaddr != 0 && vinfo.v_depth == 32;

	*width = usable ? (uint32_t) vinfo.v_width : 0;
	*height = usable ? (uint32_t) vinfo.v_height : 0;
}

/*
 * kern.hakios.screen: what the panel shows, for root alone (userland/src/screencap.c makes a PNG of it). A header of
 * 24 bytes -- "HKSCREEN", then width, height, bytes a row and 0 for BGRA, little-endian words -- and the rows packed,
 * 4 bytes a pixel, read from the boot framebuffer the panel scans out (what MiniIOMobileFramebufferShow() and the
 * swaps copy into), or, while fb_flip has the controller on one of its buffers, from that one. Not one instant: rows a
 * frame is copied over while they are read may be of the next frame.
 */
struct screen_header {
	char magic[8];
	uint32_t width, height, rowBytes, format;
};

static int
screenSysctl(struct sysctl_oid * oidp __unused, void * arg1 __unused, int arg2 __unused, struct sysctl_req * req)
{
	struct screen_header h = { { 'H', 'K', 'S', 'C', 'R', 'E', 'E', 'N' }, 0, 0, 0, 0 };

	if (req->newptr != USER_ADDR_NULL) {
		return EPERM;
	}
	if (suser(kauth_cred_get(), NULL) != 0) {
		return EPERM;
	}
	if (vinfo.v_baseaddr == 0 || vinfo.v_depth != 32 || vinfo.v_width == 0 || vinfo.v_height == 0) {
		return ENODEV;
	}
	h.width = (uint32_t) vinfo.v_width;
	h.height = (uint32_t) vinfo.v_height;
	h.rowBytes = h.width * 4;
	if (req->oldptr == USER_ADDR_NULL) {
		req->oldidx = sizeof(h) + (size_t) h.rowBytes * h.height;
		return 0;
	}
	const uint8_t * base = gScreenBase != NULL ? gScreenBase : (const uint8_t *) vinfo.v_baseaddr;
	size_t rowBytes = gScreenBase != NULL ? gScreenRowBytes : vinfo.v_rowbytes;
	int error = SYSCTL_OUT(req, &h, sizeof(h));
	for (uint32_t y = 0; error == 0 && y < h.height; y++) {
		error = SYSCTL_OUT(req, (const void *) (base + (size_t) y * rowBytes), h.rowBytes);
	}
	return error;
}

SYSCTL_DECL(_kern_hakios);
SYSCTL_PROC(_kern_hakios, OID_AUTO, screen, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0, screenSysctl, "S",
    "what the panel shows: a 24-byte header (HKSCREEN, width, height, bytes a row, 0 = BGRA) and its rows (root only)");

void
MiniIOMobileFramebufferStart(IOService * provider)
{
	IOMobileFramebuffer * fb = OSTypeAlloc(IOMobileFramebuffer);

	if (fb == NULL || !fb->init() || !fb->attach(provider)) {
		OSSafeReleaseNULL(fb);
		return;
	}
	if (fb->start(provider)) {
		publishDisplayNode();
		publishAccelerator(provider);
	} else {
		fb->detach(provider);
	}
	fb->release();
}
