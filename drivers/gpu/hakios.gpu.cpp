/*
 * HakiosGPU: the phone's Adreno for programs to draw with.
 *
 * drivers/gpu/hakios.adreno.c wakes the GPU and, with boot-arg adreno_keep=1, keeps it on and
 * starts this service. A program that opens it speaks the interface of Linux's msm DRM driver,
 * the one Mesa's freedreno knows: every request of that interface is a call of the user client
 * with the request's number as the selector and the request's structure passed in and back
 * (userland: tools/mesa/hakios-drm, which stands in for libdrm inside Mesa). What Linux does
 * with mmap() on the device is the client memory of the type that is the buffer's handle.
 *
 * What there is of the interface is what freedreno uses to draw: parameters, buffers (made,
 * told their address in the GPU's address space, mapped, given back), submission of command
 * buffers, and waiting for one to be done. A buffer is wired kernel memory entered into the 3D
 * core's SMMU context; it is zero when made, and write-combined: what the program writes goes to
 * memory, where the GPU reads it, and not into this processor's cache (as Linux maps them for
 * freedreno; boot-arg gpu_cached=1 makes them ordinary memory, pushed out of the cache whenever
 * the GPU is given something to do: see submit()).
 *
 * One request is not Linux's: a buffer that is the memory of an IOSurface (surfaceBuffer()), for
 * drawing into what the display shows and for reading pictures where other programs have drawn
 * them. Linux hands such memory around as file descriptors; here it has a name already. That
 * memory is ordinary, cached memory, which everybody else reads and writes as such.
 *
 * Not here yet: interrupts (waiting is asking again five times a millisecond), more than one
 * program's worth of care (every client's buffers are in the one address space; a client that
 * goes takes its own away), recovery of a GPU that has stopped answering (it is reported, see
 * hakios_gpu_report(), and stays stopped), fences and buffers as file descriptors.
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOSubMemoryDescriptor.h>
#include <kern/clock.h>
#include <mach/mach_time.h>
#include <mach/vm_param.h>
extern "C" void flush_dcache64(addr64_t addr, unsigned count, int phys);
#include <sys/systm.h>
#include <sys/sysctl.h>

#include "MiniIOSurface.h"

extern "C" void hakios_tl(const char * format, ...) __printflike(1, 2);

extern "C" {
/* drivers/gpu/hakios.adreno.c ("the GPU kept on"; declared in bsd/sys/hakios.input.h, which is not seen from here) */
bool hakios_gpu_ready(void);
bool hakios_gpu_parameter(uint32_t which, uint64_t * value);
uint64_t hakios_gpu_address(uint64_t size);
void hakios_gpu_address_free(uint64_t address, uint64_t size);
bool hakios_gpu_map(uint64_t address, uint64_t phys, bool read_only);
void hakios_gpu_mapped(void);
void hakios_gpu_unmap(uint64_t address, uint64_t size);
void hakios_gpu_to_memory(void * start, uint64_t size);
uint32_t hakios_gpu_submit(const uint64_t * addresses, const uint32_t * words, uint32_t count);
extern uint32_t gpu_log;
bool hakios_gpu_done(uint32_t fence);
uint32_t hakios_gpu_last(void);
void hakios_gpu_worked(uint64_t * microseconds, uint32_t * submissions, uint32_t by[7]);
void hakios_gpu_report(const char * why);
void hakios_gpu_stop(void);

void HakiosGPUStart(void);
void HakiosGPUStop(void);
void MiniIOMobileFramebufferShow(const uint8_t * pixels, uint32_t width, uint32_t height, uint32_t rowBytes);
}

/* Linux's include/uapi/drm/msm_drm.h and drm.h: the requests' numbers and structures. */
enum {
	kDrmGemClose            = 0x09,
	kMsmGetParam            = 0x40,
	kMsmSetParam            = 0x41,
	kMsmGemNew              = 0x42,
	kMsmGemInfo             = 0x43,
	kMsmGemCpuPrep          = 0x44,
	kMsmGemCpuFini          = 0x45,
	kMsmGemSubmit           = 0x46,
	kMsmWaitFence           = 0x47,
	kMsmGemMadvise          = 0x48,
	kMsmSubmitQueueNew      = 0x4a,
	kMsmSubmitQueueClose    = 0x4b,
	kMsmSubmitQueueQuery    = 0x4c,
	kHakiosUserMemory      = 0x7d,         /* ours: pages of the caller's own as a buffer (Linux's userptr) */
	kHakiosSurface         = 0x7e,         /* ours: an IOSurface's memory as a buffer */
	kHakiosShow            = 0x7f,         /* ours: pixels of the caller's onto the panel, to see a test's result */
};

struct msm_timespec {
	int64_t seconds, nanoseconds;           /* of the clock that goes on while the machine sleeps */
};
struct msm_param {
	uint32_t pipe, param;
	uint64_t value;
	uint32_t len, pad;
};
struct msm_gem_new {
	uint64_t size;
	uint32_t flags, handle;
};
struct msm_gem_info {
	uint32_t handle, info;
	uint64_t value;
	uint32_t len, pad;
};
struct msm_gem_cpu_prep {
	uint32_t handle, op;
	struct msm_timespec timeout;
};
struct msm_gem_submit_cmd {
	uint32_t type, submit_idx, submit_offset, size, pad, nr_relocs;
	uint64_t relocs;
};
struct msm_gem_submit_bo {
	uint32_t flags, handle;
	uint64_t presumed;
};
struct msm_gem_submit {
	uint32_t flags, fence, nr_bos, nr_cmds;
	uint64_t bos, cmds;
	int32_t fence_fd;
	uint32_t queueid;
	uint64_t in_syncobjs, out_syncobjs;
	uint32_t nr_in_syncobjs, nr_out_syncobjs, syncobj_stride, pad;
};
struct msm_wait_fence {
	uint32_t fence, flags;
	struct msm_timespec timeout;
	uint32_t queueid;
};
struct msm_gem_madvise {
	uint32_t handle, madv, retained;
};
struct msm_submitqueue {
	uint32_t flags, prio, id;
};
struct msm_submitqueue_query {
	uint64_t data;
	uint32_t id, param, len, pad;
};
struct drm_gem_close {
	uint32_t handle, pad;
};
struct hakios_user_memory {
	uint64_t address, size;                 /* in the caller, in whole pages */
	uint32_t handle;                        /* back: the buffer */
	uint32_t flags;
};
struct hakios_surface {
	uint32_t surface;                       /* the IOSurface's ID */
	uint32_t handle;                        /* kHakiosDescribe: no buffer is wanted; else, back: the buffer, */
	uint64_t size;                          /* how big it is, */
	uint32_t offset, rowBytes;              /* where the pixels start in it and how long a row of them is, */
	uint32_t width, height, pixelFormat, planes;    /* and what they are */
};
struct hakios_show {
	uint64_t pixels;
	uint32_t width, height, rowBytes, pad;
};

#define kHakiosDescribe        0xffffffffu

#define MSM_INFO_GET_OFFSET     0
#define MSM_INFO_GET_IOVA       1
#define MSM_INFO_SET_NAME       2
#define MSM_INFO_GET_FLAGS      5
#define MSM_BO_GPU_READONLY     0x00000002
#define MSM_PREP_NOSYNC         0x04
#define MSM_SUBMIT_CMD_BUF      1
#define MSM_SUBMIT_FENCE_FD_OUT 0x20000000

#define GPU_BUFFERS             8192            /* a client's, at most */
#define GPU_BUFFER_MAX          (256u << 20)
#define GPU_SUBMIT_CMDS         128
#define GPU_SUBMIT_BOS          8192
#define GPU_PATIENCE_MS         5000            /* for a buffer or a submission that somebody waits for without a limit */
#define GPU_ASK_AGAIN_US        200             /* while waiting for the core: how long between two looks at how far it is */
#define GPU_PUSH_AGAIN_MS       4               /* a surface is pushed out of the cache again for a submission this long after the last */

static IOLock * gGPULock;                       /* around everything that is the GPU's (hakios_gpu_...) */
static bool gGPUReported;
static bool gGPUCached;                         /* boot-arg gpu_cached=1 */
static TUNABLE(uint32_t, gCopyGuard, "fb_copy_guard", 0);
static uint32_t gCopyWaits;
static uint32_t gCopyReads, gCopyTimeouts;
SYSCTL_DECL(_kern_hakios);
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_copy_reads, CTLFLAG_RD | CTLFLAG_LOCKED, &gCopyReads, 0, "protected framebuffer reads");
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_copy_waits, CTLFLAG_RD | CTLFLAG_LOCKED, &gCopyWaits, 0, "GPU submissions delayed by a framebuffer read");
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_copy_timeouts, CTLFLAG_RD | CTLFLAG_LOCKED, &gCopyTimeouts, 0, "framebuffer ownership waits that timed out");

/* What the last seconds' submissions were and cost here, for the log. */
static struct {
	uint64_t since, flushing, waiting, taking;      /* times of the processor's clock */
	uint64_t flushed;                       /* bytes sent out of the cache */
	uint32_t submissions, commands, buffers, surfaces, waits, over;
	uint32_t made, given_again;             /* buffers of their own made for programs, and of those kept ones given again */
	uint64_t making;                        /* ... and the time it took to make the others */
} gGPUStat;

/* Wait for the 3D core to be done with submission `fence`; with the lock held, which is let go of while waiting. */
static bool
gpuWait(uint32_t fence, uint64_t milliseconds)
{
	uint64_t began = mach_absolute_time(), patience;

	nanoseconds_to_absolutetime(milliseconds * NSEC_PER_MSEC, &patience);
	for (bool waited = false;; waited = true) {
		if (hakios_gpu_done(fence)) {
			if (waited) {
				gGPUStat.waits++;
				gGPUStat.waiting += mach_absolute_time() - began;
			}
			return true;
		}
		if (mach_absolute_time() - began >= patience || gGPUReported) {
			break;                  /* (a GPU that has stopped answering is not waited for again) */
		}
		IOLockUnlock(gGPULock);
		delay_for_interval(GPU_ASK_AGAIN_US, NSEC_PER_USEC);
		IOLockLock(gGPULock);
	}
	if (milliseconds >= 1000 && !gGPUReported) {
		gGPUReported = true;
		hakios_gpu_report("no answer");
	}
	return false;
}

bool
MiniIOSurfaceDrawn(MiniIOSurface * surface, uint64_t milliseconds)
{
	uint32_t fence = surface->drawn;
	bool done = true;

	if (fence != 0 && gGPULock != NULL) {
		IOLockLock(gGPULock);
		done = !hakios_gpu_ready() || gpuWait(fence, milliseconds);
		IOLockUnlock(gGPULock);
	}
	return done;
}

void MiniIOSurfaceBeginRead(MiniIOSurface * surface)
{
	if (!gCopyGuard || gGPULock == NULL) return;
	IOLockLock(gGPULock);
	surface->cpuReaders++;
	gCopyReads++;
	IOLockUnlock(gGPULock);
}

void MiniIOSurfaceEndRead(MiniIOSurface * surface)
{
	if (!gCopyGuard || gGPULock == NULL) return;
	IOLockLock(gGPULock);
	if (surface->cpuReaders != 0) surface->cpuReaders--;
	IOLockUnlock(gGPULock);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*
 * The framebuffer's frames onto the panel by the GPU (boot-arg fb_gpu=1, MiniIOMobileFramebuffer.cpp; GPU-PRESENT-PLAN.md
 * step 1). Without it the processor scales every frame onto the panel, 1125 x 2436 into 1080 x 2412: some 2 ms of three
 * processors a frame, 21 MB through their caches, and the GPU drawing the next frame into a surface the processor is
 * still reading (fb_copy_guard). Here the GPU's 2D engine does it, bilinear, in the same ring as and after what was drawn
 * into the surfaces: a submission of a few hundred words of the kernel's own, as hakios-metal's encodeCopy() does a
 * scaled copy on the A730 (render.cpp), with turnip's r2d_* (tu_clear_blit.cc) for what that does not write.
 *
 * The A7XX's (gen 7) registers only: the A6XX's 2D source registers are elsewhere and it is not tried (0: the processor
 * draws). The panel's memory is entered into the GPU's address space once; each surface shown is entered once and held,
 * a few at a time, and let go of when the GPU is done with it and it has not been shown for a while (CoreAnimation
 * draws into a few surfaces in turn; a surface made for every frame costs a mapping a frame).
 *
 * The 2D engine writes rows a multiple of 64 bytes apart (RB_2D_DST_PITCH counts 64-byte units), and this panel's are
 * 4320 bytes (1080 pixels; seen on the phone, 11 November). Two of them are 8640, 135 units. So the frame is scaled into
 * memory of the GPU's own with rows of 4352 bytes, and copied from there onto the panel by two blits that see the panel
 * as rows of two: the even rows are the left half of each, the odd rows the right half (from 4320 bytes on), each read
 * from every other row of the GPU's copy. Integer coordinates throughout, the filtering done once. A panel whose rows
 * are a multiple of 64 bytes already is scaled into directly.
 */
static TUNABLE(uint32_t, gGPUPresentMode, "fb_gpu", 0);
#define PRESENT_SLOTS           16             /* submissions of ours the GPU may be at, at most */
#define PRESENT_SLOT_WORDS      512             /* commands of one; its last 4 words: the GPU's clock at its start and end */
#define PRESENT_MAPS            6               /* surfaces kept in the GPU's address space */
#define PRESENT_TARGETS         3               /* memories frames are put into: the panel's, fb_flip's two */
#define PRESENT_IDLE_MS         1000            /* ... one not shown for so long is let go */
#define PRESENT_WAIT_MS         200             /* for a slot or a mapping the GPU is still at */

/* the registers (a6xx.xml, Mesa 25.0.7; A7XX- variants where there are two) and commands (adreno_pm4.xml) */
#define R2D_GRAS_BLIT_CNTL      0x8400
#define R2D_GRAS_SRC_TL_X       0x8401          /* .. SRC_BR_Y 0x8404, then GRAS_2D_DST_TL 0x8405, DST_BR 0x8406 */
#define R2D_RB_BLIT_CNTL        0x8c00
#define R2D_RB_UNKNOWN_8C01     0x8c01
#define R2D_RB_DST_INFO         0x8c17          /* .. RB_2D_DST lo, hi 0x8c18, RB_2D_DST_PITCH 0x8c1a */
#define R2D_RB_DST_FLAGS        0x8c20
#define R2D_SP_DST_FORMAT_A7XX  0xa9bf          /* (0xacc0 on the A6XX) */
#define R2D_SP_SRC_INFO_A7XX    0xb2c0          /* .. SIZE, SRC lo, hi, PITCH 0xb2c4 (0xb4c0 on the A6XX) */
#define R2D_SP_SRC_FLAGS_A7XX   0xb2ca
#define R2D_TPL1_SRC_CNTL_A7XX  0xb2d2
#define R2D_FMT_8888            0x30            /* FMT6_8_8_8_8_UNORM; swap WZYX (0) both ways: the bytes as they are */
#define R2D_IFMT_UNORM8         0x10
#define R2D_BLIT_CNTL           (R2D_FMT_8888 << 8 | 0xfu << 20 | R2D_IFMT_UNORM8 << 24)
#define R2D_SRC_FILTER          (1u << 16)
#define R2D_SRC_UNK20_UNK22     (1u << 20 | 1u << 22)
#define R2D_TEX_2D              (1u << 29)      /* TPL1_2D_SRC_CNTL.TYPE = A6XX_TEX_2D */
#define PM4_THREAD_CONTROL      0x17
#define PM4_WAIT_FOR_IDLE       0x26
#define PM4_BLIT                0x2c
#define PM4_REG_TO_MEM          0x3e
#define PM4_EVENT_WRITE7        0x46
#define PM4_SET_MARKER          0x65
#define PM4_THREAD_BR           1u
#define PM4_CONCURRENT_BIN_DISABLE (1u << 27)
#define PM4_RM6_DIRECT_RENDER   1u
#define PM4_BLIT_OP_SCALE       3u
#define EV7_CCU_INVALIDATE_COLOR 25
#define EV7_CCU_CLEAN_COLOR     0x21
#define EV7_CACHE_INVALIDATE    0x33
#define REG_ALWAYS_ON_COUNTER   0x0980

struct present_map {
	MiniIOSurface *         surface;        /* held; NULL: a free place */
	uint64_t                address, size;
	uint32_t                fence;          /* the last present that reads it */
	uint64_t                used;
};

static struct {
	bool                    tried, ready;
	IOBufferMemoryDescriptor * commands;    /* PRESENT_SLOTS slots, write-combined */
	uint64_t                commandAddress;
	uint32_t                slotFence[PRESENT_SLOTS];
	unsigned                next;
	struct {
		uint64_t        phys, address, size;
	}                       targets[PRESENT_TARGETS];
	uint64_t                panelAddress;   /* the present's target (one of targets[]) */
	IOBufferMemoryDescriptor * staging;     /* the frame scaled, rows of stagingPitch bytes (NULL: scaled onto the panel itself) */
	uint64_t                stagingAddress;
	uint32_t                stagingPitch;
	struct present_map      maps[PRESENT_MAPS];
	/* since MiniIOSurfacePresentCost() was asked */
	uint32_t                presents, mapped, timed;
	uint64_t                submitting, gpuTicks;
} gPresent;
/* kern.hakios.fb_gpu_state: 0 not tried yet, 1 ready, 2 not an A7XX, 3 no memory for the commands, 4 not entered into the GPU */
static uint32_t gPresentState;
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_gpu_state, CTLFLAG_RD | CTLFLAG_LOCKED, &gPresentState, 0, "fb_gpu: the GPU's present set up");
static uint32_t gPresentTotal;
SYSCTL_UINT(_kern_hakios, OID_AUTO, fb_gpu_presents, CTLFLAG_RD | CTLFLAG_LOCKED, &gPresentTotal, 0, "fb_gpu: frames put on the panel by the GPU");

static inline uint32_t
pm4Parity(uint32_t v)
{
	return 0x9669 >> (0xf & (v ^ v >> 4 ^ v >> 8 ^ v >> 12 ^ v >> 16 ^ v >> 20 ^ v >> 24 ^ v >> 28)) & 1;
}

struct present_stream {
	uint32_t * words;
	uint32_t   at;
};

static void
pm4Regs(struct present_stream * st, uint32_t reg, const uint32_t * values, uint32_t count)
{
	st->words[st->at++] = 4u << 28 | count | pm4Parity(count) << 7 | reg << 8 | pm4Parity(reg) << 27;
	for (uint32_t i = 0; i < count; i++) {
		st->words[st->at++] = values[i];
	}
}

static void
pm4Reg(struct present_stream * st, uint32_t reg, uint32_t value)
{
	pm4Regs(st, reg, &value, 1);
}

static void
pm4Packet(struct present_stream * st, uint32_t opcode, const uint32_t * values, uint32_t count)
{
	st->words[st->at++] = 7u << 28 | count | pm4Parity(count) << 15 | opcode << 16 | pm4Parity(opcode) << 23;
	for (uint32_t i = 0; i < count; i++) {
		st->words[st->at++] = values[i];
	}
}

static void
pm4Packet1(struct present_stream * st, uint32_t opcode, uint32_t value)
{
	pm4Packet(st, opcode, &value, 1);
}

/* the GPU's own clock (19.2 MHz) into memory, 64 bits */
static void
pm4Time(struct present_stream * st, uint64_t address)
{
	const uint32_t w[3] = { REG_ALWAYS_ON_COUNTER | 2u << 18 | 1u << 30, (uint32_t) address, (uint32_t) (address >> 32) };

	pm4Packet(st, PM4_REG_TO_MEM, w, 3);
}

/* Pages of physical memory, or of a descriptor, into the GPU's address space. With the lock held; 0: no room. */
static uint64_t
presentMapMemory(IOMemoryDescriptor * memory, uint64_t from, uint64_t phys, uint64_t size, bool readOnly)
{
	uint64_t address = hakios_gpu_address(size);
	bool mapped = address != 0;

	for (uint64_t offset = 0; offset < size && mapped; offset += 4096) {
		addr64_t page = phys + offset;

		if (memory != NULL) {
			IOByteCount length = 0;

			page = memory->getPhysicalSegment(from + offset, &length, kIOMemoryMapperNone);
		}
		mapped = page != 0 && hakios_gpu_map(address + offset, page, readOnly);
	}
	if (address == 0) {
		return 0;
	}
	hakios_gpu_mapped();
	if (!mapped) {
		hakios_gpu_unmap(address, size);
		hakios_gpu_address_free(address, size);
		return 0;
	}
	return address;
}

static void
presentUnmap(struct present_map * m)
{
	hakios_gpu_unmap(m->address, m->size);
	hakios_gpu_address_free(m->address, m->size);
	MiniIOSurfaceDrop(m->surface);
	bzero(m, sizeof(*m));
}

/* With the lock held: the command memory in the GPU's address space, once. */
static bool
presentSetUp(void)
{
	uint64_t chip = 0;
	uint64_t size = (uint64_t) PRESENT_SLOTS * PRESENT_SLOT_WORDS * 4;

	if (gPresent.tried) {
		return gPresent.ready;
	}
	gPresent.tried = true;
	if (!hakios_gpu_parameter(0x03, &chip) || (chip >> 24 & 0xff) != 7) {
		IOLog("HakiosGPU: fb_gpu: the GPU (chip %#llx) is not an A7XX: the processor puts frames on the panel\n", chip & 0xffffffff);
		gPresentState = 2;
		return false;
	}
	gPresent.commands = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut | kIOMapWriteCombineCache,
	    (vm_size_t) size, PAGE_SIZE);
	if (gPresent.commands == NULL) {
		IOLog("HakiosGPU: fb_gpu: no memory for the commands\n");
		gPresentState = 3;
		return false;
	}
	gPresent.commandAddress = presentMapMemory(gPresent.commands, 0, 0, size, false);     /* (the GPU writes its times there) */
	if (gPresent.commandAddress == 0) {
		IOLog("HakiosGPU: fb_gpu: the commands cannot be entered into the GPU's address space\n");
		gPresentState = 4;
		return false;
	}
	gPresent.ready = true;
	gPresentState = 1;
	return true;
}

/*
 * With the lock held: memory frames are put into (the panel's, or one of fb_flip's buffers: PANELPHYS, rows of
 * PANELROWBYTES) in the GPU's address space, once each, and the GPU's own copy of it when its rows are not a multiple of 64
 * bytes; gPresent.panelAddress is then its GPU address. False when it cannot be.
 */
static bool
presentTarget(uint64_t panelPhys, uint32_t panelRowBytes, uint32_t panelWidth, uint32_t panelHeight)
{
	uint64_t panelSize = round_page_64((uint64_t) panelRowBytes * panelHeight);
	unsigned free = PRESENT_TARGETS;

	for (unsigned i = 0; i < PRESENT_TARGETS; i++) {
		if (gPresent.targets[i].phys == panelPhys && gPresent.targets[i].size >= panelSize) {
			gPresent.panelAddress = gPresent.targets[i].address;
			return true;
		}
		free = gPresent.targets[i].phys == 0 && free == PRESENT_TARGETS ? i : free;
	}
	if (free == PRESENT_TARGETS) {
		return false;
	}
	/* rows the 2D engine cannot write one by one: two at a time (see above), from a copy of the GPU's own */
	if ((panelRowBytes & 63) != 0 && gPresent.staging == NULL) {
		uint64_t stagingSize;

		if ((panelRowBytes & 3) != 0 || ((2 * panelRowBytes) & 63) != 0 || panelRowBytes / 4 + panelWidth > 16384) {
			IOLog("HakiosGPU: fb_gpu: the panel's rows of %u bytes cannot be written by the 2D engine\n", panelRowBytes);
			gPresentState = 5;
			return false;
		}
		gPresent.stagingPitch = (panelWidth * 4 + 63) & ~63u;
		stagingSize = round_page_64((uint64_t) gPresent.stagingPitch * panelHeight);
		gPresent.staging = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut | kIOMapWriteCombineCache,
		    (vm_size_t) stagingSize, PAGE_SIZE);
		if (gPresent.staging == NULL || (gPresent.stagingAddress = presentMapMemory(gPresent.staging, 0, 0, stagingSize, false)) == 0) {
			IOLog("HakiosGPU: fb_gpu: no memory for the GPU's copy of the panel (%llu bytes)\n", stagingSize);
			OSSafeReleaseNULL(gPresent.staging);
			gPresentState = 3;
			return false;
		}
	}
	if ((panelRowBytes & 63) != 0 && (gPresent.stagingPitch < panelWidth * 4 ||
	    (uint64_t) gPresent.stagingPitch * panelHeight > gPresent.staging->getLength())) {
		return false;
	}
	uint64_t address = presentMapMemory(NULL, 0, panelPhys, panelSize, false);

	if (address == 0) {
		IOLog("HakiosGPU: fb_gpu: the memory at %#llx (%llu bytes) cannot be entered into the GPU's address space\n", panelPhys,
		    panelSize);
		gPresentState = 4;
		return false;
	}
	gPresent.targets[free].phys = panelPhys;
	gPresent.targets[free].size = panelSize;
	gPresent.targets[free].address = address;
	gPresent.panelAddress = address;
	IOLog("HakiosGPU: fb_gpu: frames go to %#llx (%llu bytes, rows of %u) by the GPU's 2D engine, at %#llx%s\n", panelPhys,
	    panelSize, panelRowBytes, address, (panelRowBytes & 63) ? ", by a copy of the GPU's own with rows of 64-byte units" : "");
	return true;
}

/*
 * One blit of the 2D engine, 32-bit pixels as they are (bilinear when the two rectangles differ in size): from a
 * rectangle of SW x SH pixels at SX, SY of memory at SRC (rows SPITCH bytes apart, W x H of them there) to one of DW x DH
 * at DX, DY of memory at DST (DPITCH). hakios-metal's encodeCopy(), the registers in its order.
 */
static void
pm4Blit(struct present_stream * st, uint64_t src, uint32_t spitch, uint32_t w, uint32_t h, uint32_t sx, uint32_t sy, uint32_t sw,
    uint32_t sh, uint64_t dst, uint32_t dpitch, uint32_t dx, uint32_t dy, uint32_t dw, uint32_t dh)
{
	bool scaled = sw != dw || sh != dh;
	const uint32_t rect[6] = {
		sx << 8, (sx + sw - 1) << 8, sy << 8, (sy + sh - 1) << 8,
		dx | dy << 16, (dx + dw - 1) | (dy + dh - 1) << 16,
	};
	const uint32_t source[5] = {
		R2D_FMT_8888 | R2D_SRC_UNK20_UNK22 | (scaled ? R2D_SRC_FILTER : 0), w | h << 15, (uint32_t) src, (uint32_t) (src >> 32),
		(spitch >> 6) << 9,
	};
	const uint32_t destination[4] = { R2D_FMT_8888, (uint32_t) dst, (uint32_t) (dst >> 32), dpitch >> 6 };
	const uint32_t none[3] = { 0, 0, 0 };

	pm4Regs(st, R2D_GRAS_SRC_TL_X, rect, 6);
	pm4Regs(st, R2D_SP_SRC_INFO_A7XX, source, 5);
	pm4Regs(st, R2D_SP_SRC_FLAGS_A7XX, none, 3);
	pm4Regs(st, R2D_RB_DST_INFO, destination, 4);
	pm4Regs(st, R2D_RB_DST_FLAGS, none, 3);
	pm4Packet1(st, PM4_BLIT, PM4_BLIT_OP_SCALE);
}

/* what the 2D engine wrote, in memory for the next to read: the colour cache cleaned, the GPU's caches invalidated */
static void
pm4BlitBarrier(struct present_stream * st)
{
	pm4Packet1(st, PM4_EVENT_WRITE7, EV7_CCU_CLEAN_COLOR);
	pm4Packet(st, PM4_WAIT_FOR_IDLE, NULL, 0);
	pm4Packet1(st, PM4_EVENT_WRITE7, EV7_CCU_INVALIDATE_COLOR);
	pm4Packet1(st, PM4_EVENT_WRITE7, EV7_CACHE_INVALIDATE);
	pm4Packet(st, PM4_WAIT_FOR_IDLE, NULL, 0);
}

/*
 * A rectangle of the GPU's copy onto the panel, unscaled, by rows of two (see above): rows Y.. of the copy (pitch CP) at
 * the same place on the panel (pitch PP, not a multiple of 64 bytes, twice it is).
 */
static void
pm4PanelRows(struct present_stream * st, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t panelRowBytes, uint32_t panelHeight)
{
	const uint32_t cp = gPresent.stagingPitch, pp = panelRowBytes;

	for (uint32_t odd = 0; odd < 2; odd++) {
		uint32_t first = (y & 1) == odd ? y : y + 1;            /* the first row of this kind */
		uint32_t rows = first < y + h ? (y + h - first + 1) / 2 : 0;

		if (rows == 0 || first >= panelHeight) {
			continue;
		}
		/* the copy's rows FIRST, FIRST + 2, ...: from its row FIRST on, every other; the panel's: their pair's half */
		pm4Blit(st, gPresent.stagingAddress + (uint64_t) first * cp, 2 * cp, x + w, rows, x, 0, w, rows,
		    gPresent.panelAddress + (uint64_t) (first - odd) * pp, 2 * pp, odd * (pp / 4) + x, 0, w, rows);
	}
}

/* With the lock held: a surface's GPU address (its pixels' first byte), its mapping made or found; 0: none to be had. */
static uint64_t
presentSurface(MiniIOSurface * s, uint64_t now)
{
	struct present_map * m = NULL, * oldest = NULL;
	uint64_t idle, size;

	nanoseconds_to_absolutetime((uint64_t) PRESENT_IDLE_MS * NSEC_PER_MSEC, &idle);
	for (unsigned i = 0; i < PRESENT_MAPS; i++) {
		struct present_map * k = &gPresent.maps[i];

		if (k->surface == s) {
			m = k;
		} else if (k->surface != NULL && now - k->used > idle && hakios_gpu_done(k->fence)) {
			presentUnmap(k);            /* (not shown for a while: its memory is not held for nothing) */
		}
	}
	if (m != NULL) {
		m->used = now;
		return m->address + s->info.offset;
	}
	/* (memory of a program's own with a surface made around it: not entered here) */
	if (s->client != NULL) {
		return 0;
	}
	for (unsigned i = 0; i < PRESENT_MAPS && m == NULL; i++) {
		struct present_map * k = &gPresent.maps[i];

		if (k->surface == NULL) {
			m = k;
		} else if (oldest == NULL || k->used < oldest->used) {
			oldest = k;
		}
	}
	if (m == NULL) {
		if (!gpuWait(oldest->fence, PRESENT_WAIT_MS)) {
			return 0;
		}
		presentUnmap(oldest);
		m = oldest;
	}
	/* the pixels, from the page after the surface's shared state on (as surfaceBuffer() enters them) */
	size = round_page_64(s->info.allocSize);
	if (size == 0 || size > GPU_BUFFER_MAX) {
		return 0;
	}
	m->address = presentMapMemory(s->memory, PAGE_SIZE, 0, size, true);
	if (m->address == 0) {
		return 0;
	}
	MiniIOSurface * held = MiniIOSurfaceCopy(s->sid);

	if (held != s) {
		/* (the surface is going: not held, not entered) */
		if (held != NULL) {
			MiniIOSurfaceDrop(held);
		}
		hakios_gpu_unmap(m->address, size);
		hakios_gpu_address_free(m->address, size);
		m->address = 0;
		return 0;
	}
	m->surface = s;
	m->size = size;
	m->fence = 0;
	m->used = now;
	gPresent.mapped++;
	return m->address + s->info.offset;
}

uint32_t
MiniIOSurfacePresent(const struct MiniIOSurfacePresentLayer * layers, unsigned count, uint64_t panelPhys, uint32_t panelRowBytes,
    uint32_t panelWidth, uint32_t panelHeight)
{
	uint64_t began = mach_absolute_time(), sources[4];
	uint32_t fence = 0, words;
	static uint32_t said;

	/* (why not, for kern.hakios.fb_gpu_state while nothing has been set up: 10 the arguments, 11 no GPU, 12 the GPU reported) */
	if (gGPULock == NULL || count == 0 || count > 4 || panelPhys == 0 || (panelPhys & 63)) {
		gPresentState = gPresentState == 0 ? 10 : gPresentState;
		return 0;
	}
	IOLockLock(gGPULock);
	if (!hakios_gpu_ready() || gGPUReported) {
		gPresentState = gPresentState == 0 ? (gGPUReported ? 12 : 11) : gPresentState;
		IOLockUnlock(gGPULock);
		return 0;
	}
	if (!presentSetUp() || !presentTarget(panelPhys, panelRowBytes, panelWidth, panelHeight)) {
		IOLockUnlock(gGPULock);
		return 0;
	}
	for (unsigned i = 0; i < count; i++) {
		const struct MiniIOSurfacePresentLayer * l = &layers[i];
		const MiniIOSurface * s = l->surface;

		/* what the 2D engine takes: 16-bit coordinates, rows a multiple of 64 bytes, 64-byte-aligned pixels */
		if (l->sw == 0 || l->sh == 0 || l->dw == 0 || l->dh == 0 || l->dx + l->dw > panelWidth || l->dy + l->dh > panelHeight ||
		    s->info.width > 16384 || s->info.height > 16384 || (s->info.bytesPerRow & 63) || (s->info.offset & 63) ||
		    (sources[i] = presentSurface(l->surface, began)) == 0) {
			if (said++ < 8) {
				IOLog("HakiosGPU: fb_gpu: layer %u (surface %u, %ux%u, %u bytes a row, at %u) is drawn by the processor\n", i,
				    s->sid, s->info.width, s->info.height, s->info.bytesPerRow, s->info.offset);
			}
			IOLockUnlock(gGPULock);
			return 0;
		}
	}
	/* the slot: the GPU done with what was in it before */
	unsigned slot = gPresent.next % PRESENT_SLOTS;

	if (gPresent.slotFence[slot] != 0 && !gpuWait(gPresent.slotFence[slot], PRESENT_WAIT_MS)) {
		IOLockUnlock(gGPULock);
		return 0;
	}
	uint32_t * base = (uint32_t *) gPresent.commands->getBytesNoCopy() + slot * PRESENT_SLOT_WORDS;
	uint64_t address = gPresent.commandAddress + (uint64_t) slot * PRESENT_SLOT_WORDS * 4;
	uint64_t times = address + (PRESENT_SLOT_WORDS - 4) * 4;
	volatile uint64_t * timesCpu = (volatile uint64_t *) (base + PRESENT_SLOT_WORDS - 4);
	struct present_stream st = { base, 0 };

	/* (what the GPU measured of the slot's last present) */
	if (gPresent.slotFence[slot] != 0 && timesCpu[1] > timesCpu[0]) {
		gPresent.gpuTicks += timesCpu[1] - timesCpu[0];
		gPresent.timed++;
	}
	timesCpu[0] = timesCpu[1] = 0;
	/* turnip's tu6_init_hw (A7XX) and r2d_setup: the BR thread, the colour cache and the GPU's caches invalidated */
	pm4Packet1(&st, PM4_THREAD_CONTROL, PM4_THREAD_BR | PM4_CONCURRENT_BIN_DISABLE);
	pm4Time(&st, times);
	pm4Packet1(&st, PM4_EVENT_WRITE7, EV7_CCU_INVALIDATE_COLOR);
	pm4Packet1(&st, PM4_EVENT_WRITE7, EV7_CACHE_INVALIDATE);
	pm4Packet(&st, PM4_WAIT_FOR_IDLE, NULL, 0);
	pm4Packet1(&st, PM4_SET_MARKER, PM4_RM6_DIRECT_RENDER);
	pm4Reg(&st, R2D_RB_UNKNOWN_8C01, 0);
	pm4Reg(&st, R2D_RB_BLIT_CNTL, R2D_BLIT_CNTL);
	pm4Reg(&st, R2D_GRAS_BLIT_CNTL, R2D_BLIT_CNTL);
	pm4Reg(&st, R2D_SP_DST_FORMAT_A7XX, R2D_FMT_8888 << 3 | 0xfu << 12);
	/*
	 * (TPL1_2D_SRC_CNTL, which turnip writes TYPE 2D for an image's blit, is left as it is: hakios-metal's 2D copies on
	 * this A730 never write it, and Metal's next submission is not to find it changed. If the picture comes out wrong,
	 * boot-arg fb_gpu=2 writes it.)
	 */
	if (gGPUPresentMode == 2) {
		pm4Reg(&st, R2D_TPL1_SRC_CNTL_A7XX, R2D_TEX_2D);
	}
	/* each layer scaled into the GPU's copy of the panel, or into the panel itself when its rows allow */
	bool staged = (panelRowBytes & 63) != 0;
	uint64_t target = staged ? gPresent.stagingAddress : gPresent.panelAddress;
	uint32_t targetPitch = staged ? gPresent.stagingPitch : panelRowBytes;

	for (unsigned i = 0; i < count; i++) {
		const struct MiniIOSurfacePresentLayer * l = &layers[i];
		const MiniIOSurface * s = l->surface;

		pm4Blit(&st, sources[i], s->info.bytesPerRow, s->info.width, s->info.height, l->sx, l->sy, l->sw, l->sh, target,
		    targetPitch, l->dx, l->dy, l->dw, l->dh);
	}
	pm4BlitBarrier(&st);
	/* ... and from the copy onto the panel, each layer's rectangle, by rows of two */
	if (staged) {
		for (unsigned i = 0; i < count; i++) {
			pm4PanelRows(&st, layers[i].dx, layers[i].dy, layers[i].dw, layers[i].dh, panelRowBytes, panelHeight);
		}
		pm4BlitBarrier(&st);
	}
	pm4Time(&st, times + 8);
	words = st.at;
	if (words > PRESENT_SLOT_WORDS - 4) {
		panic("HakiosGPU: fb_gpu: %u words of commands in a slot of %u", words, PRESENT_SLOT_WORDS - 4);
	}
	hakios_gpu_to_memory(NULL, 0);          /* (what was written is in memory: a barrier) */
	fence = hakios_gpu_submit(&address, &words, 1);
	if (fence == 0) {
		if (!gGPUReported) {
			gGPUReported = true;
			hakios_gpu_report("the ring is full (fb_gpu)");
		}
		IOLockUnlock(gGPULock);
		return 0;
	}
	gPresent.slotFence[slot] = fence;
	gPresent.next++;
	for (unsigned i = 0; i < count; i++) {
		for (unsigned k = 0; k < PRESENT_MAPS; k++) {
			if (gPresent.maps[k].surface == layers[i].surface) {
				gPresent.maps[k].fence = fence;
			}
		}
	}
	if (gpu_log) {
		hakios_tl("tl: P %u: %u layers, %u words\n", fence, count, words);
	}
	gPresent.presents++;
	gPresentTotal++;
	gPresent.submitting += mach_absolute_time() - began;
	IOLockUnlock(gGPULock);
	return fence;
}

bool
MiniIOSurfacePresentDone(uint32_t fence, uint64_t milliseconds)
{
	bool done = true;

	if (fence != 0 && gGPULock != NULL) {
		IOLockLock(gGPULock);
		done = !hakios_gpu_ready() || gpuWait(fence, milliseconds);
		IOLockUnlock(gGPULock);
	}
	return done;
}

void
MiniIOSurfacePresentCost(uint32_t * presents, uint64_t * submitting, uint64_t * gpu, uint32_t * timed, uint32_t * mapped)
{
	uint64_t ns;

	if (gGPULock == NULL) {
		*presents = *timed = *mapped = 0;
		*submitting = *gpu = 0;
		return;
	}
	IOLockLock(gGPULock);
	absolutetime_to_nanoseconds(gPresent.submitting, &ns);
	*presents = gPresent.presents;
	*submitting = gPresent.presents ? ns / NSEC_PER_USEC / gPresent.presents : 0;
	*timed = gPresent.timed;
	*gpu = gPresent.timed ? gPresent.gpuTicks * 10 / 192 / gPresent.timed : 0;
	*mapped = gPresent.mapped;
	gPresent.presents = gPresent.mapped = gPresent.timed = 0;
	gPresent.submitting = gPresent.gpuTicks = 0;
	IOLockUnlock(gGPULock);
}

/*
 * Boot-arg touch_lat=1: a line for every wait of a program's for the GPU that took a millisecond or
 * more ("tl: f KIND US at US"), for telling apart what holds a frame up (tools/frames.py).
 */
static TUNABLE(uint32_t, gWaitLines, "touch_lat", 0);

static void
slowWait(const char * kind, uint64_t began)
{
	uint64_t now = mach_absolute_time(), ns, at;

	if (!gWaitLines) {
		return;
	}
	absolutetime_to_nanoseconds(now - began, &ns);
	if (ns >= NSEC_PER_MSEC) {
		absolutetime_to_nanoseconds(now, &at);
		hakios_tl("tl: f %s %llu at %llu\n", kind, ns / 1000, at / 1000);
	}
}

/* How long from now a time of msm's is, in milliseconds; 0: now or past. */
static uint64_t
millisecondsUntil(const struct msm_timespec * when)
{
	uint64_t now;
	int64_t left, end;

	absolutetime_to_nanoseconds(mach_continuous_time(), &now);
	/* the seconds are the caller's: a number too big to be nanoseconds is a time far ahead (or far behind), not a wrapped one */
	if (__builtin_mul_overflow(when->seconds, (int64_t) NSEC_PER_SEC, &end) || __builtin_add_overflow(end, when->nanoseconds, &end)) {
		return when->seconds > 0 && when->nanoseconds >= 0 ? GPU_PATIENCE_MS : 0;
	}
	if (end <= 0) {
		return 0;                       /* (the clock's reading is never negative: this is past) */
	}
	left = end - (int64_t) now;
	if (left <= 0) {
		return 0;
	}
	return (uint64_t) left / NSEC_PER_MSEC > GPU_PATIENCE_MS ? GPU_PATIENCE_MS : (uint64_t) left / NSEC_PER_MSEC;
}

/*
 * A wait that was to end at a time that had passed when it was asked for, and the GPU was not
 * done: whoever asked goes on as if it were (a picture is shown half drawn). That is what comes
 * of a caller whose clock is not this one, so the first few are told.
 */
static void
waitOver(const struct msm_timespec * when, uint32_t fence)
{
	static uint32_t told;
	uint64_t now;

	gGPUStat.over++;
	if (told < 4) {
		told++;
		absolutetime_to_nanoseconds(mach_continuous_time(), &now);
		IOLog("HakiosGPU: a wait for submission %u, which is not done, until %lld.%09lld s, asked for at %llu.%09llu s: "
		    "over before it began\n", fence, when->seconds, when->nanoseconds, now / NSEC_PER_SEC, now % NSEC_PER_SEC);
	}
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class HakiosGPU : public IOService
{
	OSDeclareDefaultStructors(HakiosGPU);

public:
	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(HakiosGPU, IOService);

bool
HakiosGPU::start(IOService * provider)
{
	if (!IOService::start(provider)) {
		return false;
	}
	setProperty(kIOUserClientClassKey, "HakiosGPUUserClient");
	setName("HakiosGPU");
	registerService();
	return true;
}

void
HakiosGPUStart(void)
{
	IOService * provider = IOService::getPlatform();
	HakiosGPU * gpu;
	uint32_t cached = 0;

	gGPUCached = PE_parse_boot_argn("gpu_cached", &cached, sizeof(cached)) && cached != 0;
	gGPULock = IOLockAlloc();
	gpu = OSTypeAlloc(HakiosGPU);
	if (gpu == NULL || provider == NULL || !gpu->init() || !gpu->attach(provider)) {
		OSSafeReleaseNULL(gpu);
		IOLog("HakiosGPU: cannot create the service\n");
		return;
	}
	if (!gpu->start(provider)) {
		gpu->detach(provider);
		IOLog("HakiosGPU: the service did not start\n");
	}
	gpu->release();
}

/* The run is ending (hakios.adreno.c): nobody is in the middle of anything when the GPU goes. */
void
HakiosGPUStop(void)
{
	IOLockLock(gGPULock);
	hakios_gpu_stop();
	IOLockUnlock(gGPULock);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

struct gpu_buffer {
	IOMemoryDescriptor *            memory;         /* NULL: this handle is free */
	uint8_t *                       kernel;         /* where it is in the kernel */
	MiniIOSurface *                 surface;        /* whose memory it is, when not its own */
	bool                            client;         /* the client's own pages (no kernel view: out of the cache by their physical addresses) */
	uint64_t                        address;        /* the GPU's for it */
	uint64_t                        size;
	uint32_t                        flags;
	uint32_t                        seed;           /* the surface's when it was last pushed out of the cache, */
	uint64_t                        pushed;         /* ... and when that was */
	bool                            seen;           /* by the program: mapped, and from then on written by it */
	uint32_t                        fence;          /* the last submission that names it (0: none has) */
};

/*
 * A buffer given back while the GPU is still at it: its memory and its place in the GPU's
 * address space are kept until the submission is done, and nobody waits for that. (A window
 * server that makes a surface for every frame gives the last frame's back while the GPU draws
 * this one: waiting there, for all the GPU had been given, cost it some five milliseconds a
 * frame.)
 */
struct gpu_left {
	IOMemoryDescriptor *            memory;
	MiniIOSurface *                 surface;
	bool                            client;
	uint64_t                        address, size;
	uint32_t                        fence, flags;
};
#define GPU_LEFT                64

/*
 * A buffer of the program's own given back, and done with: kept, memory and place in the GPU's
 * address space, for the next one of the same size and kind. CoreAnimation makes a new texture to
 * draw a blur into for every frame of an animation with one (Search: 1082 x 1226, 6 MB with Mesa's
 * rounding), and gives them back only later, a score at a time; Mesa keeps what is given back for
 * a second and then lets it go. Each made afresh here took 2 to 3.5 ms of the window server's frame
 * (memory got, cleared, made write-combined, and entered page by page into the GPU's address
 * space). A buffer kept is given again as it is, not cleared: it was the same program's (Mesa's own
 * cache gives buffers back the same way). At most GPU_KEPT_BYTES are kept (the one kept longest
 * makes way), none for longer than GPU_KEPT_SECONDS. rc.jobs.fps, Search: 8 such textures made
 * slowly instead of 77, the window server's frames drawn in 6.8 ms instead of 9.3-10 (median).
 */
struct gpu_kept {
	IOMemoryDescriptor *            memory;         /* NULL: this place is free */
	uint8_t *                       kernel;
	uint64_t                        address, size, since;
	uint32_t                        flags;
};
#define GPU_KEPT                48
#define GPU_KEPT_BYTES          (160ull << 20)
#define GPU_KEPT_SECONDS        60              /* a buffer not asked for again in so long is let go */
#define GPU_KEEP_FROM           (256u << 10)    /* bytes: smaller ones are not worth it */
static TUNABLE(uint32_t, gGPUKeep, "gpu_keep", 1);      /* boot-arg gpu_keep=0 (a test): none kept; gpu_keep=2: kept ones cleared when given again */

class HakiosGPUUserClient : public IOUserClient
{
	OSDeclareDefaultStructors(HakiosGPUUserClient);

	task_t                  fTask;
	struct gpu_buffer *     fBuffers;               /* handle n is fBuffers[n - 1] */
	uint32_t                fNextQueue;
	uint32_t                fNextBuffer;            /* where the search for a free handle starts: one given back is not the next given */
	uint32_t                fBufferCount;
	uint64_t                fBufferBytes;
	uint32_t                fSubmits;
	uint32_t                fStrangers;             /* requests that named a buffer the client does not have */
	struct gpu_left         fLeft[GPU_LEFT];
	uint32_t                fLeftCount;
	struct gpu_kept         fKept[GPU_KEPT];
	uint64_t                fKeptBytes;

	struct gpu_buffer * buffer(uint32_t handle);
	struct gpu_buffer * newBuffer(IOMemoryDescriptor * memory, uint64_t from, uint64_t size, uint32_t flags, uint32_t * handle);
	void freeBuffer(struct gpu_buffer * b);
	bool keep(IOMemoryDescriptor * memory, uint8_t * kernel, uint64_t address, uint64_t size, uint32_t flags);
	void dropKept(struct gpu_kept * k);
	void reap(void);
	IOReturn gemNew(struct msm_gem_new * request);
	IOReturn surfaceBuffer(struct hakios_surface * request);
	IOReturn userBuffer(struct hakios_user_memory * request);
	IOReturn gemInfo(struct msm_gem_info * request);
	IOReturn submit(struct msm_gem_submit * request);
	IOReturn show(const struct hakios_show * request);

public:
	virtual bool initWithTask(task_t owningTask, void * securityID, UInt32 type, OSDictionary * properties) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientMemoryForType(UInt32 type, IOOptionBits * options, IOMemoryDescriptor ** memory) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments * args,
	    IOExternalMethodDispatch * dispatch, OSObject * target, void * reference) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(HakiosGPUUserClient, IOUserClient);

bool
HakiosGPUUserClient::initWithTask(task_t owningTask, void * securityID, UInt32 type, OSDictionary * properties)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type, properties)) {
		return false;
	}
	fTask = owningTask;
	return true;
}

bool
HakiosGPUUserClient::start(IOService * provider)
{
	if (!hakios_gpu_ready() || !IOUserClient::start(provider)) {
		return false;
	}
	fBuffers = IONewZero(struct gpu_buffer, GPU_BUFFERS);
	return fBuffers != NULL;
}

/*
 * With the lock held, and the GPU done with everything it has been given (`idle`). If it is
 * not -- it has stopped answering -- the buffer's memory stays the GPU's for good: it may yet be
 * written, and must not be somebody else's by then.
 */
void
HakiosGPUUserClient::freeBuffer(struct gpu_buffer * slot)
{
	/*
	 * gpuWait() below lets go of the lock: take the buffer out of its handle first and work on a copy, so that
	 * a second close of the same handle (or the client's closing) cannot free the same memory and address twice.
	 */
	struct gpu_buffer saved = *slot;
	struct gpu_buffer * b = &saved;

	slot->memory = NULL;
	slot->surface = NULL;
	slot->client = false;
	fBufferCount--;
	fBufferBytes -= b->size;

	bool busy = hakios_gpu_ready() && b->fence != 0 && !hakios_gpu_done(b->fence);

	if (busy && fLeftCount == GPU_LEFT) {
		busy = !gpuWait(b->fence, 1000);        /* (nowhere to keep it: this one is waited for) */
	}
	if (busy && fLeftCount < GPU_LEFT) {
		struct gpu_left * left = &fLeft[fLeftCount++];

		left->memory = b->memory;
		left->surface = b->surface;
		left->client = b->client;
		left->address = b->address;
		left->size = b->size;
		left->fence = b->fence;
		left->flags = b->flags;
	} else {
		if (busy) {
			static uint32_t said;

			if (b->surface == NULL) {
				b->memory->retain();
			} else {
				b->surface->memory->retain();
			}
			if (said++ < 8) {
				IOLog("HakiosGPU: the GPU is not done with what it was given: a buffer of %llu bytes stays its own\n", b->size);
			}
		} else if (b->surface == NULL && !b->client && hakios_gpu_ready() &&
		    keep(b->memory, b->kernel, b->address, b->size, b->flags)) {
			b->memory = NULL;               /* (now the kept one's) */
		} else if (hakios_gpu_ready()) {
			hakios_gpu_unmap(b->address, b->size);
			hakios_gpu_address_free(b->address, b->size);
		}
		if (b->memory != NULL) {
			b->memory->release();
		}
		if (b->surface != NULL) {
			MiniIOSurfaceDrop(b->surface);
		}
	}
}

/* The buffers that were given back early and that the GPU is done with now: freed. With the lock held. */
void
HakiosGPUUserClient::reap(void)
{
	for (uint32_t i = 0; i < fLeftCount;) {
		struct gpu_left * left = &fLeft[i];

		if (hakios_gpu_ready() && !hakios_gpu_done(left->fence)) {
			i++;
			continue;
		}
		if (left->surface == NULL && !left->client && hakios_gpu_ready() &&
		    keep(left->memory, NULL, left->address, left->size, left->flags)) {
			left->memory = NULL;
		} else if (hakios_gpu_ready()) {
			hakios_gpu_unmap(left->address, left->size);
			hakios_gpu_address_free(left->address, left->size);
		}
		if (left->memory != NULL) {
			left->memory->release();
		}
		if (left->surface != NULL) {
			MiniIOSurfaceDrop(left->surface);
		}
		*left = fLeft[--fLeftCount];
	}
}

IOReturn
HakiosGPUUserClient::clientClose(void)
{
	if (fBuffers != NULL) {
		IOLockLock(gGPULock);
		/* what the client had the GPU do last may still be going on, in these buffers */
		if (hakios_gpu_ready()) {
			gpuWait(hakios_gpu_last(), 1000);
		}
		for (uint32_t i = 0; i < GPU_BUFFERS; i++) {
			if (fBuffers[i].memory != NULL) {
				freeBuffer(&fBuffers[i]);
			}
		}
		reap();
		for (uint32_t i = 0; i < GPU_KEPT; i++) {
			dropKept(&fKept[i]);
		}
		if (fLeftCount != 0) {
			IOLog("HakiosGPU: the GPU is not done with %u buffers of a program that is gone: they stay its own\n", fLeftCount);
		}
		IOLockUnlock(gGPULock);
	}
	if (!isInactive()) {
		terminate();
	}
	return kIOReturnSuccess;
}

void
HakiosGPUUserClient::free(void)
{
	if (fBuffers != NULL) {
		IODelete(fBuffers, struct gpu_buffer, GPU_BUFFERS);
		fBuffers = NULL;
	}
	IOUserClient::free();
}

struct gpu_buffer *
HakiosGPUUserClient::buffer(uint32_t handle)
{
	if (handle == 0 || handle > GPU_BUFFERS || fBuffers[handle - 1].memory == NULL) {
		/* (a program that names a buffer it has given back draws into somebody else's next: worth knowing) */
		if (handle != 0 && fStrangers++ < 16) {
			IOLog("HakiosGPU: buffer %u is asked for and is not there\n", handle);
		}
		return NULL;
	}
	return &fBuffers[handle - 1];
}

/*
 * A handle and addresses of the GPU's for `size` bytes of a memory descriptor, from byte `from`
 * of it on (wired memory; both in whole pages). With the lock held; NULL: there is no room.
 */
struct gpu_buffer *
HakiosGPUUserClient::newBuffer(IOMemoryDescriptor * memory, uint64_t from, uint64_t size, uint32_t flags, uint32_t * handle)
{
	struct gpu_buffer * b = NULL;
	bool mapped = true;

	for (uint32_t n = 0; n < GPU_BUFFERS; n++) {
		uint32_t i = (fNextBuffer + n) % GPU_BUFFERS;

		if (fBuffers[i].memory == NULL) {
			b = &fBuffers[i];
			*handle = i + 1;
			fNextBuffer = i + 1;
			break;
		}
	}
	if (b == NULL || (b->address = hakios_gpu_address(size)) == 0) {
		IOLog("HakiosGPU: no %s left for a buffer of %llu bytes\n", b ? "addresses" : "handles", size);
		return NULL;
	}
	for (uint64_t offset = 0; offset < size && mapped; offset += 4096) {
		IOByteCount length = 0;
		addr64_t phys = memory->getPhysicalSegment(from + offset, &length, kIOMemoryMapperNone);

		mapped = phys != 0 && hakios_gpu_map(b->address + offset, phys, flags & MSM_BO_GPU_READONLY);
	}
	hakios_gpu_mapped();
	if (!mapped) {
		hakios_gpu_unmap(b->address, size);
		hakios_gpu_address_free(b->address, size);
		IOLog("HakiosGPU: a buffer of %llu bytes cannot be entered into the GPU's address space\n", size);
		return NULL;
	}
	b->size = size;
	b->flags = flags;
	b->seed = 0;
	b->pushed = 0;
	b->seen = false;
	b->fence = 0;
	b->surface = NULL;
	b->client = false;
	fBufferCount++;
	fBufferBytes += size;
	return b;
}

/* With the lock held. */
void
HakiosGPUUserClient::dropKept(struct gpu_kept * k)
{
	if (k->memory == NULL) {
		return;
	}
	if (hakios_gpu_ready()) {
		hakios_gpu_unmap(k->address, k->size);
		hakios_gpu_address_free(k->address, k->size);
	}
	k->memory->release();
	k->memory = NULL;
	fKeptBytes -= k->size;
}

/* With the lock held, the GPU done with the buffer: kept if it is worth it (those kept longest make way). */
bool
HakiosGPUUserClient::keep(IOMemoryDescriptor * memory, uint8_t * kernel, uint64_t address, uint64_t size, uint32_t flags)
{
	struct gpu_kept * place = NULL;

	if (gGPUKeep == 0 || size < GPU_KEEP_FROM || size > GPU_KEPT_BYTES / 4 || memory == NULL ||
	    (kernel == NULL && OSDynamicCast(IOBufferMemoryDescriptor, memory) == NULL)) {
		return false;
	}
	for (;;) {
		struct gpu_kept * oldest = NULL;

		place = NULL;
		for (uint32_t i = 0; i < GPU_KEPT; i++) {
			if (fKept[i].memory == NULL) {
				place = place != NULL ? place : &fKept[i];
			} else if (oldest == NULL || fKept[i].since < oldest->since) {
				oldest = &fKept[i];
			}
		}
		if (place != NULL && fKeptBytes + size <= GPU_KEPT_BYTES) {
			break;
		}
		dropKept(oldest);
	}
	place->memory = memory;
	place->kernel = kernel != NULL ? kernel : (uint8_t *) OSDynamicCast(IOBufferMemoryDescriptor, memory)->getBytesNoCopy();
	place->address = address;
	place->size = size;
	place->flags = flags;
	place->since = mach_absolute_time();
	fKeptBytes += size;
	return true;
}

IOReturn
HakiosGPUUserClient::gemNew(struct msm_gem_new * request)
{
	uint64_t size = round_page_64(request->size), began = mach_absolute_time();
	struct gpu_buffer * b;
	IOBufferMemoryDescriptor * memory;

	if (size == 0 || size > GPU_BUFFER_MAX) {
		return kIOReturnBadArgument;
	}
	/* one kept of this size and kind, if there is one (the one kept last: likeliest still in the caches' way) */
	IOLockLock(gGPULock);
	struct gpu_kept * found = NULL;
	uint64_t old;

	nanoseconds_to_absolutetime(GPU_KEPT_SECONDS * NSEC_PER_SEC, &old);
	for (uint32_t i = 0; i < GPU_KEPT; i++) {
		struct gpu_kept * k = &fKept[i];

		if (k->memory != NULL && began - k->since > old) {
			dropKept(k);
		}

		if (k->memory != NULL && k->size == size && k->flags == request->flags && (found == NULL || k->since > found->since)) {
			found = k;
		}
	}
	for (uint32_t n = 0; found != NULL && n < GPU_BUFFERS; n++) {
		uint32_t h = (fNextBuffer + n) % GPU_BUFFERS;

		if (fBuffers[h].memory == NULL) {
			b = &fBuffers[h];
			bzero(b, sizeof(*b));
			b->memory = found->memory;
			b->kernel = found->kernel;
			b->address = found->address;
			b->size = found->size;
			b->flags = found->flags;
			fNextBuffer = h + 1;
			fBufferCount++;
			fBufferBytes += size;
			request->handle = h + 1;
			found->memory = NULL;
			fKeptBytes -= size;
			gGPUStat.given_again++;
			IOLockUnlock(gGPULock);
			if (gGPUKeep == 2) {
				bzero(b->kernel, (size_t) b->size);
			}
			return kIOReturnSuccess;
		}
	}
	IOLockUnlock(gGPULock);
	/* (the memory comes zeroed, and making it write-combined pushes the zeroes out of the cache) */
	memory = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut | kIOMemoryKernelUserShared |
	    (gGPUCached ? 0 : kIOMapWriteCombineCache), (vm_size_t) size, PAGE_SIZE);
	if (memory == NULL) {
		IOLog("HakiosGPU: no memory for a buffer of %llu bytes (%u buffers, %llu MB with this client)\n", size, fBufferCount,
		    fBufferBytes >> 20);
		return kIOReturnNoMemory;
	}
	if (gGPUCached) {
		hakios_gpu_to_memory(memory->getBytesNoCopy(), size);
	}

	IOLockLock(gGPULock);
	b = newBuffer(memory, 0, size, request->flags, &request->handle);
	if (b != NULL) {
		b->memory = memory;
		b->kernel = (uint8_t *) memory->getBytesNoCopy();
		gGPUStat.made++;
		gGPUStat.making += mach_absolute_time() - began;
	}
	IOLockUnlock(gGPULock);
	if (b == NULL) {
		memory->release();
		return kIOReturnNoMemory;
	}
	return kIOReturnSuccess;
}

/*
 * The memory of an IOSurface as a buffer: its pixels, from the page after the surface's shared
 * state on (MiniIOSurface.h). The surface is held for as long as the buffer is there. A surface
 * that is a buffer of this client's already is that buffer again, as with Linux's handles: so
 * the client must not be giving that buffer back at the same moment (freedreno asks under the
 * lock it gives buffers back under). Asked only to describe (kHakiosDescribe), no buffer is made.
 */
IOReturn
HakiosGPUUserClient::surfaceBuffer(struct hakios_surface * request)
{
	MiniIOSurface * s = MiniIOSurfaceCopy(request->surface);
	struct gpu_buffer * b = NULL;
	IOMemoryDescriptor * memory;
	uint64_t size, began = mach_absolute_time();

	if (s == NULL) {
		return kIOReturnNotFound;
	}
	if (s->client != NULL) {
		/* (memory of a program's own with a surface made around it: not in whole pages, and seldom in rows the GPU takes) */
		MiniIOSurfaceDrop(s);
		return kIOReturnUnsupported;
	}
	size = round_page_64(s->info.allocSize);
	memory = IOSubMemoryDescriptor::withSubRange(s->memory, PAGE_SIZE, size, kIODirectionInOut);
	if (memory == NULL || size == 0 || size > GPU_BUFFER_MAX) {
		OSSafeReleaseNULL(memory);
		MiniIOSurfaceDrop(s);
		return kIOReturnNoMemory;
	}
	request->size = size;
	request->offset = s->info.offset;
	request->rowBytes = s->info.bytesPerRow;
	request->width = s->info.width;
	request->height = s->info.height;
	request->pixelFormat = s->info.pixelFormat;
	request->planes = s->info.planeCount;
	if (request->handle == kHakiosDescribe) {
		request->handle = 0;
		memory->release();
		MiniIOSurfaceDrop(s);
		return kIOReturnSuccess;
	}

	IOLockLock(gGPULock);
	for (uint32_t i = 0; i < GPU_BUFFERS; i++) {
		if (fBuffers[i].memory != NULL && fBuffers[i].surface == s) {
			b = &fBuffers[i];
			request->handle = i + 1;
			break;
		}
	}
	if (b != NULL) {
		memory->release();
		MiniIOSurfaceDrop(s);
	} else if ((b = newBuffer(s->memory, PAGE_SIZE, size, 0, &request->handle)) != NULL) {
		b->memory = memory;
		b->kernel = s->kbase + PAGE_SIZE;
		b->surface = s;
		gGPUStat.surfaces++;
	}
	gGPUStat.taking += mach_absolute_time() - began;
	IOLockUnlock(gGPULock);
	if (b == NULL) {
		memory->release();
		MiniIOSurfaceDrop(s);
		return kIOReturnNoMemory;
	}
	return kIOReturnSuccess;
}

/*
 * Pages of the client's own as a buffer, as Linux's userptr: wired for as long as the buffer is there,
 * entered into the GPU's address space, and seen by the kernel through a mapping of its own (for the
 * cache: they are ordinary memory, pushed out of this processor's cache at every submission that
 * names them). Vulkan's VK_EXT_external_memory_host over it (Turnip, tools/mesa/hakios-drm): Metal's
 * no-copy buffers, CoreAnimation's backing stores.
 */
IOReturn
HakiosGPUUserClient::userBuffer(struct hakios_user_memory * request)
{
	IOMemoryDescriptor * memory = NULL;
	struct gpu_buffer * b;
	uint64_t began = mach_absolute_time();

	if ((request->address & PAGE_MASK) || (request->size & PAGE_MASK) || request->size == 0 ||
	    request->size > GPU_BUFFER_MAX) {
		static uint32_t said;

		if (said++ < 8) {
			IOLog("HakiosGPU: pages at %#llx (%llu bytes) of the client's: not whole pages of %u bytes\n",
			    request->address, request->size, (unsigned) PAGE_SIZE);
		}
		return kIOReturnBadArgument;
	}
	/* pages the client may only read (a window server's view of another program's backing store): the GPU reads them only */
	IOReturn prepared = kIOReturnNoMemory;
	for (int pass = 0; pass < 2 && prepared != kIOReturnSuccess; pass++) {
		memory = IOMemoryDescriptor::withAddressRange(request->address, request->size,
		    pass == 0 ? kIODirectionInOut : kIODirectionOut, fTask);
		if (memory == NULL) {
			IOLog("HakiosGPU: pages at %#llx (%llu bytes) of the client's: no descriptor\n", request->address, request->size);
			return kIOReturnNoMemory;
		}
		prepared = memory->prepare();
		if (prepared != kIOReturnSuccess) {
			memory->release();
			memory = NULL;
		} else if (pass == 1) {
			request->flags |= MSM_BO_GPU_READONLY;
		}
	}
	if (memory == NULL) {
		static uint32_t said;

		if (said++ < 8) {
			IOLog("HakiosGPU: pages at %#llx (%llu bytes) of the client's cannot be wired: %#x\n",
			    request->address, request->size, prepared);
		}
		return kIOReturnVMError;
	}
	IOLockLock(gGPULock);
	b = newBuffer(memory, 0, request->size, request->flags, &request->handle);
	if (b != NULL) {
		b->memory = memory;             /* (its pages unwired as it goes: IOGeneralMemoryDescriptor::free) */
		b->kernel = NULL;
		b->client = true;
		b->seen = true;
		gGPUStat.surfaces++;
	}
	gGPUStat.taking += mach_absolute_time() - began;
	IOLockUnlock(gGPULock);
	if (b == NULL) {
		static uint32_t said;

		if (said++ < 8) {
			IOLog("HakiosGPU: pages at %#llx (%llu bytes) of the client's: no room for them in the GPU\n",
			    request->address, request->size);
		}
		memory->complete();
		memory->release();
		return kIOReturnNoMemory;
	}
	return kIOReturnSuccess;
}

IOReturn
HakiosGPUUserClient::gemInfo(struct msm_gem_info * request)
{
	struct gpu_buffer * b;
	IOReturn ret = kIOReturnSuccess;

	IOLockLock(gGPULock);
	b = buffer(request->handle);
	if (b == NULL) {
		ret = kIOReturnBadArgument;
	} else if (request->info == MSM_INFO_GET_OFFSET) {
		request->value = (uint64_t) request->handle << 32;      /* nobody maps by it here */
	} else if (request->info == MSM_INFO_GET_IOVA) {
		request->value = b->address;
	} else if (request->info == MSM_INFO_GET_FLAGS) {
		request->value = b->flags;
	} else if (request->info != MSM_INFO_SET_NAME) {            /* a name for somebody's debugger: not kept */
		ret = kIOReturnUnsupported;
	}
	IOLockUnlock(gGPULock);
	return ret;
}

IOReturn
HakiosGPUUserClient::submit(struct msm_gem_submit * request)
{
	struct msm_gem_submit_bo * bos = NULL;
	struct msm_gem_submit_cmd * cmds = NULL;
	uint64_t addresses[GPU_SUBMIT_CMDS];
	uint32_t words[GPU_SUBMIT_CMDS], count = 0;
	uint64_t began, nanos, again;
	IOReturn ret = kIOReturnBadArgument;

	if (request->nr_cmds == 0 || request->nr_cmds > GPU_SUBMIT_CMDS || request->nr_bos == 0 || request->nr_bos > GPU_SUBMIT_BOS) {
		IOLog("HakiosGPU: a submission of %u command buffers in %u buffers: not taken\n", request->nr_cmds, request->nr_bos);
		return kIOReturnBadArgument;
	}
	bos = IONew(struct msm_gem_submit_bo, request->nr_bos);
	cmds = IONew(struct msm_gem_submit_cmd, request->nr_cmds);
	if (bos == NULL || cmds == NULL) {
		ret = kIOReturnNoMemory;
		goto out;
	}
	if (copyin((user_addr_t) request->bos, bos, request->nr_bos * sizeof(*bos)) != 0 ||
	    copyin((user_addr_t) request->cmds, cmds, request->nr_cmds * sizeof(*cmds)) != 0) {
		ret = kIOReturnVMError;
		goto out;
	}
	IOLockLock(gGPULock);
	{
		uint64_t since = mach_absolute_time(), patience;
		bool waited = false;
		nanoseconds_to_absolutetime(500 * NSEC_PER_MSEC, &patience);
		for (;;) {
			bool reading = false;
			for (uint32_t i = 0; i < request->nr_bos; i++) {
				struct gpu_buffer * b = buffer(bos[i].handle);
				reading |= b != NULL && b->surface != NULL && b->surface->cpuReaders != 0;
			}
			if (!reading) break;
			if (mach_absolute_time() - since >= patience) {
				gCopyTimeouts++;
				IOLockUnlock(gGPULock);
				ret = kIOReturnBusy;
				goto out;
			}
			if (!waited && gCopyWaits++ < 8) IOLog("HakiosGPU: submission waits for the framebuffer's CPU read\n");
			waited = true;
			IOLockUnlock(gGPULock);
			delay_for_interval(GPU_ASK_AGAIN_US, NSEC_PER_USEC);
			IOLockLock(gGPULock);
		}
	}
	/*
	 * The GPU reads memory and writes memory. What this processor's cache holds of an IOSurface
	 * (somebody has written it, or has read it, as the display's driver does, and the GPU is to
	 * draw into it again) goes out of the cache now: when the surface is said to have changed
	 * (its seed: who writes one is to lock it, and so does a program that has had the GPU draw
	 * into one: userland/src/eagl.c), and in any case once in a few milliseconds, which is once
	 * a frame and not once a submission: not everybody who draws into a surface says so (an
	 * application drawing a picture that is on the screen meanwhile). A surface this program
	 * has mapped, every time. The buffers made here need none of that, unless they are
	 * ordinary memory (gpu_cached=1).
	 */
	began = mach_absolute_time();
	nanoseconds_to_absolutetime(GPU_PUSH_AGAIN_MS * NSEC_PER_MSEC, &again);
	for (uint32_t i = 0; i < request->nr_bos; i++) {
		struct gpu_buffer * b = buffer(bos[i].handle);

		if (b == NULL) {
			continue;
		}
		if (b->surface != NULL) {
			uint32_t seed = b->surface->shared()->seed;

			if (seed == b->seed && !b->seen && began - b->pushed < again) {
				continue;
			}
			b->seed = seed;
			b->pushed = began;
		} else if (b->client) {
			/* the client's own pages are ordinary memory: out of the cache every time, by their physical addresses */
			for (uint64_t offset = 0; offset < b->size;) {
				IOByteCount length = 0;
				addr64_t phys = b->memory->getPhysicalSegment(offset, &length, kIOMemoryMapperNone);

				if (phys == 0 || length == 0) {
					break;
				}
				length = MIN(length, b->size - offset);
				flush_dcache64(phys, (unsigned) length, TRUE);
				offset += length;
			}
			gGPUStat.flushed += b->size;
			continue;
		} else if (!b->seen || !gGPUCached) {
			continue;
		}
		hakios_gpu_to_memory(b->kernel, b->size);
		gGPUStat.flushed += b->size;
	}
	hakios_gpu_to_memory(NULL, 0);                 /* (and what was written last is in memory: a barrier) */
	gGPUStat.flushing += mach_absolute_time() - began;
	gGPUStat.buffers += request->nr_bos;
	for (uint32_t i = 0; i < request->nr_cmds; i++) {
		struct gpu_buffer * b = cmds[i].submit_idx < request->nr_bos ? buffer(bos[cmds[i].submit_idx].handle) : NULL;

		if (cmds[i].type != MSM_SUBMIT_CMD_BUF) {
			continue;                               /* the other kinds are for a GPU shared between address spaces */
		}
		if (b == NULL || cmds[i].nr_relocs != 0 || (cmds[i].size & 3) || (cmds[i].submit_offset & 3) ||
		    (uint64_t) cmds[i].submit_offset + cmds[i].size > b->size) {
			IOLog("HakiosGPU: command buffer %u of a submission: buffer %u, offset %u, %u bytes, %u relocations: not taken\n",
			    i, cmds[i].submit_idx, cmds[i].submit_offset, cmds[i].size, cmds[i].nr_relocs);
			IOLockUnlock(gGPULock);
			goto out;
		}
		addresses[count] = b->address + cmds[i].submit_offset;
		words[count++] = cmds[i].size / 4;
	}
	request->fence = hakios_gpu_submit(addresses, words, count);
	if (gpu_log && request->fence != 0) {
		/* "tl: Q FENCE at US: N buffers, B of 4 MB or more, W words of commands": what was given (tl: q: what it took) */
		uint32_t big = 0, all_words = 0;
		uint64_t ns;

		for (uint32_t i = 0; i < request->nr_bos; i++) {
			struct gpu_buffer * b = buffer(bos[i].handle);

			big += b != NULL && b->size >= (4u << 20);
		}
		for (uint32_t i = 0; i < count; i++) {
			all_words += words[i];
		}
		absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
		hakios_tl("tl: Q %u at %llu: %u buffers, %u big, %u words\n", request->fence, ns / 1000, request->nr_bos, big, all_words);
	}
	if (request->flags & MSM_SUBMIT_FENCE_FD_OUT) {
		request->fence_fd = -1;
	}
	/*
	 * The surfaces named are being drawn into, or read, until this submission is done: the
	 * display's driver waits for that before it puts one on the panel (MiniIOSurfaceDrawn()).
	 * (freedreno names every buffer for reading and writing alike, so which it is cannot be
	 * told; the last submission of a frame names the surface the frame is drawn into.)
	 */
	for (uint32_t i = 0; request->fence != 0 && i < request->nr_bos; i++) {
		struct gpu_buffer * b = buffer(bos[i].handle);

		if (b != NULL) {
			b->fence = request->fence;
			if (b->surface != NULL) {
				b->surface->drawn = request->fence;
			}
		}
	}
	reap();
	if (request->fence == 0 && !gGPUReported) {
		gGPUReported = true;
		hakios_gpu_report("the ring is full");
	}
	if (fSubmits++ < 4) {
		/* (addresses and words are only set up to `count`: with no command buffer among the commands, the first would be stack garbage) */
		IOLog("HakiosGPU: submission %u: %u command buffers, the first at %#llx, %u words\n", request->fence, count,
		    count != 0 ? addresses[0] : 0ull, count != 0 ? words[0] : 0u);
	}
	/* every five seconds in which something was submitted: how much, and what it cost here */
	gGPUStat.submissions++;
	gGPUStat.commands += count;
	if (gGPUStat.since == 0) {
		gGPUStat.since = began;
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - gGPUStat.since, &nanos);
	if (nanos >= 5 * NSEC_PER_SEC) {
		uint64_t flushing, waiting, taking, worked;
		uint32_t timed, by[7];

		absolutetime_to_nanoseconds(gGPUStat.flushing, &flushing);
		absolutetime_to_nanoseconds(gGPUStat.waiting, &waiting);
		absolutetime_to_nanoseconds(gGPUStat.taking, &taking);
		hakios_gpu_worked(&worked, &timed, by);
		uint64_t making;

		absolutetime_to_nanoseconds(gGPUStat.making, &making);
		IOLog("HakiosGPU: buffers: %u made in %llu us each, %u kept ones given again (%llu MB kept)\n", gGPUStat.made,
		    gGPUStat.made ? making / NSEC_PER_USEC / gGPUStat.made : 0, gGPUStat.given_again, fKeptBytes >> 20);
		IOLog("HakiosGPU: %u submissions in %llu ms: %u command buffers and %u buffers named, %llu KB out of the cache "
		    "in %llu us a submission; the GPU worked %llu us on each of %u (%u under 0.1 ms, %u under 0.25, %u under 0.5, "
		    "%u under 1, %u under 2, %u under 4, %u longer; %llu ms in all); %u waits of %llu us, %u over before they "
		    "began; %u IOSurfaces taken as buffers in %llu us each; %u buffers, %llu MB\n",
		    gGPUStat.submissions, nanos / NSEC_PER_MSEC, gGPUStat.commands, gGPUStat.buffers,
		    (gGPUStat.flushed >> 10) / gGPUStat.submissions, flushing / NSEC_PER_USEC / gGPUStat.submissions,
		    timed ? worked / timed : 0, timed, by[0], by[1], by[2], by[3], by[4], by[5], by[6], worked / 1000,
		    gGPUStat.waits, gGPUStat.waits ? waiting / NSEC_PER_USEC / gGPUStat.waits : 0, gGPUStat.over,
		    gGPUStat.surfaces, gGPUStat.surfaces ? taking / NSEC_PER_USEC / gGPUStat.surfaces : 0, fBufferCount,
		    fBufferBytes >> 20);
		bzero(&gGPUStat, sizeof(gGPUStat));
	}
	IOLockUnlock(gGPULock);
	ret = request->fence != 0 ? kIOReturnSuccess : kIOReturnTimeout;
out:
	if (bos != NULL) {
		IODelete(bos, struct msm_gem_submit_bo, request->nr_bos);
	}
	if (cmds != NULL) {
		IODelete(cmds, struct msm_gem_submit_cmd, request->nr_cmds);
	}
	return ret;
}

/* Pixels of the caller's (32 bits each, blue first) copied to the panel: for a test to show what it has drawn. */
IOReturn
HakiosGPUUserClient::show(const struct hakios_show * request)
{
	uint64_t size = (uint64_t) request->rowBytes * request->height;
	uint8_t * pixels;
	IOReturn ret = kIOReturnSuccess;

	if (request->width == 0 || request->width > 4096 || request->height == 0 || request->height > 4096 ||
	    request->rowBytes < 4 * request->width) {
		return kIOReturnBadArgument;
	}
	/* rowBytes is the caller's own number: without a limit it asks for terabytes of wired memory (no more than a buffer may be) */
	if (size > GPU_BUFFER_MAX) {
		return kIOReturnBadArgument;
	}
	pixels = (uint8_t *) IOMallocData(size);
	if (pixels == NULL) {
		return kIOReturnNoMemory;
	}
	if (copyin((user_addr_t) request->pixels, pixels, size) != 0) {
		ret = kIOReturnVMError;
	} else {
		MiniIOMobileFramebufferShow(pixels, request->width, request->height, request->rowBytes);
	}
	IOFreeData(pixels, size);
	return ret;
}

IOReturn
HakiosGPUUserClient::clientMemoryForType(UInt32 type, IOOptionBits * options, IOMemoryDescriptor ** memory)
{
	struct gpu_buffer * b;
	IOReturn ret = kIOReturnBadArgument;

	IOLockLock(gGPULock);
	b = buffer(type);
	if (b != NULL) {
		b->memory->retain();
		b->seen = true;
		*memory = b->memory;
		*options = 0;
		ret = kIOReturnSuccess;
	}
	IOLockUnlock(gGPULock);
	return ret;
}

IOReturn
HakiosGPUUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments * args,
    IOExternalMethodDispatch * dispatch, OSObject * target, void * reference)
{
	union {
		struct msm_param param;
		struct msm_gem_new gemNew;
		struct msm_gem_info gemInfo;
		struct msm_gem_cpu_prep cpuPrep;
		struct msm_gem_submit submit;
		struct msm_wait_fence waitFence;
		struct msm_gem_madvise madvise;
		struct msm_submitqueue queue;
		struct msm_submitqueue_query query;
		struct drm_gem_close close;
		struct hakios_surface surface;
		struct hakios_user_memory user;
		struct hakios_show show;
	} r;
	size_t size = args->structureInputSize;
	struct gpu_buffer * b;
	IOReturn ret = kIOReturnSuccess;
	uint64_t zero = 0, patience, waitBegan;

	if (args->structureInput == NULL || size > sizeof(r) || fBuffers == NULL) {
		return kIOReturnBadArgument;
	}
	if (!hakios_gpu_ready() && selector != kHakiosShow) {
		return kIOReturnNotReady;                       /* given back: the run is ending */
	}
	bzero(&r, sizeof(r));
	memcpy(&r, args->structureInput, size);

	switch (selector) {
	case kMsmGetParam:
		ret = hakios_gpu_parameter(r.param.param, &r.param.value) ? kIOReturnSuccess : kIOReturnBadArgument;
		break;
	case kMsmSetParam:                              /* the program's name, for Linux's lists of who uses the GPU */
	case kMsmGemCpuFini:
	case kMsmSubmitQueueClose:
		break;
	case kMsmGemNew:
		ret = gemNew(&r.gemNew);
		break;
	case kMsmGemInfo:
		ret = gemInfo(&r.gemInfo);
		break;
	case kMsmGemCpuPrep:                            /* is the GPU done with a buffer: with all it has been given, here */
		waitBegan = mach_absolute_time();
		IOLockLock(gGPULock);
		patience = (r.cpuPrep.op & MSM_PREP_NOSYNC) ? 0 : millisecondsUntil(&r.cpuPrep.timeout);
		if (!gpuWait(hakios_gpu_last(), patience)) {
			ret = kIOReturnBusy;
			if (patience == 0 && !(r.cpuPrep.op & MSM_PREP_NOSYNC)) {
				waitOver(&r.cpuPrep.timeout, hakios_gpu_last());
			}
		}
		IOLockUnlock(gGPULock);
		slowWait("cpu-prep", waitBegan);
		break;
	case kMsmGemSubmit:
		ret = submit(&r.submit);
		break;
	case kMsmWaitFence:
		waitBegan = mach_absolute_time();
		IOLockLock(gGPULock);
		/* a fence that was never issued is never done: refuse it, so that waiting for it cannot make gpuWait() declare the GPU dead for every client */
		if ((int32_t)(r.waitFence.fence - hakios_gpu_last()) > 0) {
			IOLockUnlock(gGPULock);
			ret = kIOReturnBadArgument;
			break;
		}
		patience = millisecondsUntil(&r.waitFence.timeout);
		if (!gpuWait(r.waitFence.fence, patience)) {
			ret = kIOReturnTimeout;
			if (patience == 0) {
				waitOver(&r.waitFence.timeout, r.waitFence.fence);
			}
		}
		IOLockUnlock(gGPULock);
		slowWait("fence", waitBegan);
		break;
	case kMsmGemMadvise:
		r.madvise.retained = 1;
		break;
	case kMsmSubmitQueueNew:
		r.queue.id = ++fNextQueue;
		break;
	case kMsmSubmitQueueQuery:                      /* how often the GPU has faulted on this client's commands */
		if (r.query.data != 0 && r.query.len >= sizeof(zero)) {
			copyout(&zero, (user_addr_t) r.query.data, sizeof(zero));
		}
		break;
	case kDrmGemClose:
		IOLockLock(gGPULock);
		b = buffer(r.close.handle);
		if (b == NULL) {
			ret = kIOReturnBadArgument;
		} else {
			freeBuffer(b);
			reap();
		}
		IOLockUnlock(gGPULock);
		break;
	case kHakiosSurface:
		ret = surfaceBuffer(&r.surface);
		break;
	case kHakiosUserMemory:
		ret = userBuffer(&r.user);
		break;
	case kHakiosShow:
		ret = show(&r.show);
		break;
	default:
		IOLog("HakiosGPU: request %#x (%zu bytes) is not known here\n", selector, size);
		ret = kIOReturnUnsupported;
		break;
	}
	if (ret == kIOReturnSuccess && args->structureOutput != NULL) {
		if (args->structureOutputSize > size) {
			args->structureOutputSize = (uint32_t) size;
		}
		memcpy(args->structureOutput, &r, args->structureOutputSize);
	}
	return ret;
}
