/*
 * NE2213 Iris2 RAM-boot diagnostic. iris_probe=1 checks power and clocks.
 * iris_probe=2 also authenticates the stock PAS 9 image and exchanges HFI.
 * iris_probe=3 also configures and closes a small HEVC encode session.
 * iris_probe=4 submits a single NV12 test frame and logs the returned HEVC.
 * iris_probe=5 tests TrustZone ownership of a scratch page for secure IOMMU tables.
 * iris_probe=6 encodes a 4032x3024 still as a grid of 512x512 HEVC tiles.
 * Level 7 encodes caller-supplied stills on /dev/hakios-iris, with a full
 * firmware load, encode and stock power-off for each request. It is the
 * default service with cam=5 (iris=0 turns it off; iris_probe=7 also asks for it).
 * The firmware is the phone's own vpu20_4v.mbn, read from fw/ of the boot
 * image (userland/build.sh copies it from fw-video/) and split here into the
 * PAS metadata and the carveout image.
 * Source: OnePlus SM8450 video-driver (Waipio / Iris2) and SM8450 VideoCC.
 */
#include <arm/caches_internal.h>
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <sys/conf.h>
#include <sys/hakios.input.h>
#include <sys/hakios.iris.h>
#include <sys/hakios.mmio.h>
#include <sys/hakios.smmu.h>
#include <sys/param.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <miscfs/devfs/devfs.h>
#include <vm/pmap.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map_unmappable(vm_offset_t, vm_size_t, unsigned);
extern void ml_io_unmap(vm_offset_t, vm_size_t);
extern uint64_t kvtophys(vm_offset_t);
extern void loader_log_line(const char *);
extern unsigned long gPhysBase, gPhysSize;

extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

#define IRIS_FIRMWARE       "vpu20_4v.mbn"

#define GCC_VIDEO_PA        0x00142000UL
#define VIDEO_CC_PA         0x0aaf0000UL
#define IRIS_PA             0x0aa00000UL
#define IRIS_REG_SIZE       0xf0000UL
#define FW_PA               0x9fd00000UL
#define FW_SIZE             0x700000UL

#define GCC_VIDEO_BCR       0x000
#define GCC_AHB_CBCR        0x004
#define GCC_XO_CBCR         0x028
#define GCC_AXI_CBCR        0x018
#define AXI_RESET           (1u << 2)
#define VIDEO_RCG_CMD       0x8000
#define VIDEO_RCG_CFG       0x8004
#define RCG_UPDATE          1u
#define RCG_HW_CONTROL      (1u << 20)
#define RCG_RATE_MASK       (0x371fu | RCG_HW_CONTROL)
#define PLL_MODE            0x00
#define PLL_OPMODE          0x04
#define PLL_L               0x10
#define PLL_ALPHA           0x14
#define PLL_USER            0x18
#define PLL_USER_HI         0x1c
#define PLL_CONFIG          0x20
#define PLL_CONFIG_HI       0x24
#define PLL_CONFIG_HI1      0x28
#define PLL_OUTPUT          1u
#define PLL_RESET_N         (1u << 2)
#define PLL_LOCK            (1u << 31)
#define VIDEO_CTL_GDSCR     0x804c
#define VIDEO_CTL_CBCR      0x8064
#define VIDEO_CORE_GDSCR    0x809c
#define VIDEO_CORE_CBCR     0x80b0
#define VIDEO_AHB_CBCR      0x80e4
#define VIDEO_XO_CBCR       0x8114
#define VIDEO_SLEEP_CBCR    0x8130
#define CLK_ENABLE          1u
#define CLK_OFF             (1u << 31)
#define GDSC_COLLAPSE       1u
#define GDSC_HW_CONTROL     (1u << 1)
#define GDSC_RETAIN         (1u << 11)
#define GDSC_POWERED        (1u << 31)

#define CTRL_INIT           0xa0048
#define CTRL_STATUS         0xa004c
#define QTBL_INFO           0xa0050
#define QTBL_ADDR           0xa0054
#define HFI_VERSION_INFO    0xa0058
#define SFR_ADDR            0xa005c
#define UC_REGION_ADDR      0xa0064
#define UC_REGION_SIZE      0xa0068
#define H2XSOFTINTEN        0xa0148
#define HOST_DOORBELL       0xa0150
#define X2RPMH              0xa0168
#define WRAPPER_INTR_STATUS 0xb000c
#define WRAPPER_INTR_MASK   0xb0010
#define CORE_CLOCK_CONFIG   0xb0088
#define WRAPPER_TZ_CPU_STATUS 0xc0010
/* Stock Iris2 power-off handshake (msm_vidc_iris2.c, HPG 6.1.1 and 6.1.2). */
#define VCODEC_SS_IDLE      0x00070
#define VCODEC_PIPES        4u
#define MVP_NOC_RESET_REQ   0x1f000
#define MVP_NOC_RESET_ACK   0x1f004
#define AHB_BRIDGE_SYNC_RESET 0xa0160
#define DEBUG_BRIDGE_LPI_CONTROL 0xb0054
#define DEBUG_BRIDGE_LPI_STATUS 0xb0058
#define CPU_NOC_LPI_CONTROL 0xb005c
#define CPU_NOC_LPI_STATUS  0xb0060
#define MVP_NOC_LPI_CONTROL 0xe0000
#define MVP_NOC_LPI_STATUS  0xe0004

#define HFI_IOVA            0x25800000u
#define HFI_DMA_SIZE        0x300000u
#define HFI_SFR_OFFSET      (HFI_DMA_SIZE - 0x2000u)
#define HFI_RING_WORDS      (0xc8000u / sizeof(uint32_t))
#define HFI_CMD_INIT        0x01000001u
#define HFI_POWER_COLLAPSE  0x01000002u
#define HFI_CMD_OPEN        0x01000003u
#define HFI_CMD_CLOSE       0x01000004u
#define HFI_CMD_START       0x01000005u
#define HFI_CMD_STOP        0x01000006u
#define HFI_CMD_BUFFER      0x01000009u
#define HFI_IMAGE_VERSION   0x03000001u
#define HFI_DEBUG_LEVEL     0x0300000bu
#define HFI_PROP_CODEC      0x03000100u
#define HFI_PROP_COLOR      0x03000101u
#define HFI_PROP_SECURE     0x03000102u
#define HFI_PROP_BITSTREAM  0x03000103u
#define HFI_PROP_STRIDE     0x03000104u
#define HFI_PROP_CROP       0x03000105u
#define HFI_PROP_PROFILE    0x03000107u
#define HFI_PROP_STAGE      0x0300010au
#define HFI_PROP_PIPE       0x0300010bu
#define HFI_PROP_FRAME_RATE 0x0300010cu
#define HFI_PROP_HOST_COUNT 0x03000123u
#define HFI_PROP_RC         0x0300012au
#define HFI_PROP_LAYER_TYPE 0x03000138u
#define HFI_PROP_LAYER_COUNT 0x03000139u
#define HFI_PROP_BITRATE    0x0300013bu
#define HFI_PROP_CQ         0x03000143u
#define HFI_PROP_GRID       0x03000144u
#define HFI_PROP_GOP        0x03000146u
#define HFI_PROP_B_FRAMES   0x03000147u
#define HFI_PROP_QUALITY    0x03000148u
#define HFI_PROP_SEQ_HEADER 0x03000149u
#define HFI_PROP_SIGNAL_COLOR 0x03000155u
#define HFI_PROP_BLUR       0x0300014eu
#define HFI_PROP_RAW        0x03000178u
#define HFI_HOST_REPLY      3u /* Response and interrupt requested. */
#define HFI_HOST_KEEP       4u
#define HFI_FW_SUCCESS      1u
#define HFI_FW_ERROR        0xcu
#define HFI_PORT_NONE       0u
#define HFI_PORT_BITSTREAM  1u
#define HFI_PORT_RAW        2u
#define HFI_PAYLOAD_NONE    0u
#define HFI_PAYLOAD_U32     1u
#define HFI_PAYLOAD_STRUCT  5u
#define HFI_PAYLOAD_Q16     8u
#define HFI_PAYLOAD_ENUM    9u
#define HFI_PAYLOAD_PACK32  10u
#define HFI_PAYLOAD_PACK64  13u
#define HFI_ENCODE_HEVC     4u
#define HFI_COLOR_NV12      1u
#define IRIS_SESSION_ID     1u
#define IRIS_TEST_WIDTH     256u
#define IRIS_TEST_HEIGHT    128u
#define IRIS_NV12_SIZE      (IRIS_TEST_WIDTH * IRIS_TEST_HEIGHT * 3 / 2)
#define IRIS_MEDIA_IOVA     (HFI_IOVA + HFI_DMA_SIZE)
#define IRIS_MEDIA_SIZE     0x200000u
#define IRIS_PIXEL_IOVA     (IRIS_MEDIA_IOVA + IRIS_MEDIA_SIZE)
#define IRIS_PIXEL_SIZE     0x40000u
#define IRIS_OUTPUT_SIZE    0x20000u
#define IRIS_GRID_WIDTH     HAKIOS_IRIS_WIDTH
#define IRIS_GRID_HEIGHT    HAKIOS_IRIS_HEIGHT
#define IRIS_GRID_TILE      512u
#define IRIS_GRID_STRIDE    4096u
#define IRIS_GRID_LINES     3072u
#define IRIS_GRID_NV12      (IRIS_GRID_STRIDE * IRIS_GRID_LINES * 3 / 2)
#define IRIS_GRID_TILES     (HAKIOS_IRIS_PACKETS - 1)
#define IRIS_GRID_SLOTS     4u
#define IRIS_GRID_OUTPUT    983040u
#define IRIS_GRID_MEDIA     0x880000u
#define IRIS_GRID_PIXELS    0x12c8000u
#define IRIS_ARP_IOVA       0x257c0000u
#define IRIS_ARP_SIZE       204800u
#define HFI_BUF_BITSTREAM   1u
#define HFI_BUF_RAW         2u
#define HFI_BUF_DPB         6u
#define HFI_BUF_BIN         7u
#define HFI_BUF_LINE        8u
#define HFI_BUF_ARP         9u
#define HFI_BUF_COMV        10u
#define HFI_BUF_NON_COMV    11u
#define HFI_BUF_VPSS        13u

#define PAS_ID              9u
#define PAS_INIT_IMAGE      0x0201
#define PAS_MEM_SETUP       0x0202
#define PAS_AUTH_AND_RESET  0x0205
#define PAS_SHUTDOWN        0x0206
#define SCM_VIDEO_PROTECT   0x0c08
#define SCM_REMOTE_STATE    0x010a
#define SCM_MP_ASSIGN       0x0c16
#define SCM_ASSIGN_ARGS     0x1117u
#define VMID_HLOS           3u
#define VMID_VIDEO_NONPIXEL 11u
#define VM_PERM_R           4u
#define VM_PERM_RW          6u
#define VM_PERM_RWX         7u

#define ELF_PT_LOAD         1u
#define MBN_MAX_SEGMENTS    32u
#define MDT_TYPE_MASK       (7u << 24)
#define MDT_TYPE_HASH       (2u << 24)
#define MDT_RELOCATABLE     (1u << 27)
#define HFI_PROP_UBWC       0x03000003u

/* The firmware file, checked once: where PAS metadata and each load segment come from. */
static struct {
	const uint8_t *file;
	uint32_t header_size, hash_offset, hash_size, loads;
	struct {
		uint32_t offset, destination, size;
	} load[MBN_MAX_SEGMENTS];
	bool reported;
} fw;

/* HFI Gen2 system init with Waipio's LPDDR5 UBWC defaults, in hfi_packet_sys_init()'s order. */
#define HFI_U32_PACKET(type, flags, id, value) 36u, (type), (flags), HFI_PAYLOAD_U32, 0u, (id), 0u, 0u, (value)
static const uint32_t hfi_init_message[] = {
	32u + 8u * 36u, 0u, 0u, 0u, 0u, 0u, 0u, 8u,
	HFI_U32_PACKET(HFI_CMD_INIT, 7u, 0u, 1u),
	HFI_U32_PACKET(HFI_PROP_UBWC, 0u, 1u, 8u),
	HFI_U32_PACKET(HFI_PROP_UBWC + 1u, 0u, 2u, 32u),
	HFI_U32_PACKET(HFI_PROP_UBWC + 2u, 0u, 3u, 16u),
	HFI_U32_PACKET(HFI_PROP_UBWC + 3u, 0u, 4u, 0u),
	HFI_U32_PACKET(HFI_PROP_UBWC + 4u, 0u, 5u, 1u),
	HFI_U32_PACKET(HFI_PROP_UBWC + 5u, 0u, 6u, 1u),
	HFI_U32_PACKET(HFI_PROP_UBWC + 6u, 0u, 7u, 1u),
};

/* The image version request, a property read with interrupt and response. */
static const uint32_t hfi_version_message[] = {
	64u, 0u, 1u, 0u, 0u, 0u, 0u, 1u,
	32u, HFI_IMAGE_VERSION, 0xbu, HFI_PAYLOAD_NONE, 0u, 8u, 0u, 0u,
};

struct iris_change {
	vm_offset_t reg;
	uint32_t old, mask, status;
	bool rcg;
};

struct hfi_buffer {
	uint32_t type, index;
	uint64_t address;
	uint32_t address_offset, buffer_size, data_offset, data_size;
	uint64_t timestamp;
	uint32_t flags, reserved[5];
};

struct iris_probe {
	vm_offset_t gcc, cc, regs, backing, queues, metadata;
	vm_offset_t media_backing, media, arp_backing;
	uint64_t arp_pa;
	struct iris_change changes[28];
	unsigned count, codec_mark;
	unsigned command_header, command_packet;
	unsigned media_size, pixel_size;
	unsigned pending_read, pending_count;
	struct hfi_buffer pending[10]; /* At most ten 96-byte packets per 1024-byte reply. */
	struct hakios_iris_still *still;
	struct hakios_cam_power power;
	struct hakios_smmu_context *smmu;
	struct hakios_smmu_context *pixels;
	struct hakios_smmu_context *arp;
	bool mapped, pas_initialized, auth_attempted;
	bool media_mapped, pixels_mapped, arp_assignment_attempted, grid, codec_on;
};

_Static_assert(sizeof(struct hfi_buffer) == 64, "Iris2 buffer ABI");

struct hfi_queue {
	uint32_t status, address, type, words, packet_size, dropped;
	uint32_t rx_watermark, tx_watermark, rx_request, tx_request;
	uint32_t rx_irq, tx_irq, read_index, write_index;
};

struct hfi_table {
	uint32_t version, size, queue_offset, queue_header_size, count, active;
	uint64_t host_cookie;
	char name[256];
	struct hfi_queue queue[3];
};

_Static_assert(sizeof(struct hfi_table) == 0x1c8, "Iris2 queue table ABI");
_Static_assert(sizeof(struct hfi_queue) == 56, "Iris2 queue header ABI");

SYSCTL_DECL(_kern_hakios);
static int iris_result;
SYSCTL_INT(_kern_hakios, OID_AUTO, iris_probe, CTLFLAG_RD | CTLFLAG_LOCKED,
    &iris_result, 0, "Iris2 diagnostic: 0 pending, 1 pass, -1 failed, -2 teardown failed");

static void iris_log(const char *, ...) __printflike(1, 2);

static void
iris_log(const char *format, ...)
{
	char line[256];
	va_list args;
	va_start(args, format);
	vsnprintf(line, sizeof(line), format, args);
	va_end(args);
	loader_log_line(line);
}

static bool
read_reg(vm_offset_t reg, uint32_t *value)
{
	return hakios_mmio_try_read32((const volatile void *)reg, value) == 0 && *value != UINT32_MAX;
}

static bool
write_reg(vm_offset_t reg, uint32_t value)
{
	if (hakios_mmio_try_write32((volatile void *)reg, value) != 0) {
		return false;
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	return true;
}

static bool
poll_reg(vm_offset_t reg, uint32_t mask, uint32_t want, unsigned timeout_us)
{
	for (unsigned elapsed = 0; elapsed < timeout_us; elapsed += 10) {
		uint32_t value;
		if (!read_reg(reg, &value)) {
			return false;
		}
		if ((value & mask) == want) {
			return true;
		}
		delay(10);
	}
	return false;
}

static bool
change_reg(struct iris_probe *p, vm_offset_t reg, uint32_t mask, uint32_t want,
    uint32_t status, uint32_t ready, bool rcg)
{
	uint32_t old;
	if (p->count == sizeof(p->changes) / sizeof(p->changes[0]) || !read_reg(reg, &old)) {
		return false;
	}
	if ((old & mask) != want) {
		p->changes[p->count++] = (struct iris_change){ reg, old, mask, status, rcg };
		if (!write_reg(reg, (old & ~mask) | want)) {
			return false;
		}
		if (rcg && (!write_reg(reg - 4, RCG_UPDATE) || !poll_reg(reg - 4, RCG_UPDATE, 0, 2000))) {
			return false;
		}
	}
	return poll_reg(reg, mask | status, want | ready, 4000);
}

static bool
enable_clock(struct iris_probe *p, vm_offset_t reg, bool wait)
{
	bool ok = change_reg(p, reg, CLK_ENABLE, CLK_ENABLE, wait ? CLK_OFF : 0, 0, false);
	uint32_t value = UINT32_MAX;
	(void)read_reg(reg, &value);
	iris_log("iris_probe: clock %s+%#lx %#x%s\n", reg >= p->cc && reg < p->cc + 0x10000 ? "VideoCC" : "GCC",
	    (unsigned long)(reg >= p->cc && reg < p->cc + 0x10000 ? reg - p->cc : reg - p->gcc),
	    value, ok ? "" : " failed");
	return ok;
}

static void
log_video_cc(const struct iris_probe *p)
{
	const unsigned offsets[] = { 0, VIDEO_RCG_CMD, VIDEO_CTL_GDSCR, VIDEO_CTL_CBCR,
		VIDEO_CORE_GDSCR, VIDEO_CORE_CBCR, VIDEO_AHB_CBCR, VIDEO_XO_CBCR, VIDEO_SLEEP_CBCR };
	for (unsigned i = 0; i < sizeof(offsets) / sizeof(offsets[0]); ++i) {
		uint32_t value = UINT32_MAX;
		bool ok = read_reg(p->cc + offsets[i], &value);
		iris_log("iris_probe: VideoCC+%#x %#x%s\n", offsets[i], value, ok ? "" : " unreadable");
	}
}

static bool
enable_domain(struct iris_probe *p, vm_offset_t reg)
{
	uint32_t old = UINT32_MAX;
	if (!read_reg(reg, &old) || (old & (GDSC_HW_CONTROL | GDSC_POWERED))) {
		iris_log("iris_probe: cannot acquire GDSC %#lx (%#x)\n", (unsigned long)(reg - p->cc), old);
		return false;
	}
	return change_reg(p, reg, GDSC_COLLAPSE | GDSC_RETAIN, GDSC_RETAIN,
	           GDSC_POWERED, GDSC_POWERED, false);
}

static bool
enable_pll(struct iris_probe *p)
{
	uint32_t mode, user;
	if (!read_reg(p->cc + PLL_MODE, &mode) || !read_reg(p->cc + PLL_USER, &user) ||
	    (mode & (PLL_OUTPUT | PLL_LOCK)) || (user & (1u << 25))) {
		return false;
	}
	/*
	 * Stock Waipio Lucid EVO PLL0 settings (cal_l 0x44) at 1332 MHz: L 69 and alpha 0.375 of 19.2 MHz. That is the
	 * top of the stock mvs0 table (576, 720, 1014, 1098, 1332 MHz; the codec's 240..444 MHz in the device tree's
	 * qcom,allowed-clock-rates, a third of it), allowed at the nominal corner, which hakios_cam_power_vote holds on
	 * MMCX and MXC. At 576 MHz (192 MHz in the codec, below stock's lowest) a 7 MB dark-room photograph took
	 * 385 ms on Iris2, bound by its entropy coder.
	 */
	static const struct { unsigned reg; uint32_t value; } config[] = {
		{ PLL_L, 0x00440045 }, { PLL_ALPHA, 0x6000 }, { PLL_CONFIG, 0x20485699 },
		{ PLL_CONFIG_HI, 0x00182261 }, { PLL_CONFIG_HI1, 0x32aa299c },
		{ PLL_USER, 0 }, { PLL_USER_HI, 0x805 },
	};
	for (unsigned i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
		if (!change_reg(p, p->cc + config[i].reg, UINT32_MAX, config[i].value, 0, 0, false)) {
			iris_log("iris_probe: PLL0 configuration failed at %#x\n", config[i].reg);
			return false;
		}
	}
	if (!change_reg(p, p->cc + PLL_OPMODE, 1, 0, 0, 0, false) ||
	    !change_reg(p, p->cc + PLL_MODE, PLL_RESET_N, PLL_RESET_N, 0, 0, false) ||
	    !change_reg(p, p->cc + PLL_OPMODE, 1, 1, 0, 0, false) ||
	    !poll_reg(p->cc + PLL_MODE, PLL_LOCK, PLL_LOCK, 1000) ||
	    !change_reg(p, p->cc + PLL_USER, 7, 7, 0, 0, false) ||
	    !change_reg(p, p->cc + PLL_MODE, PLL_OUTPUT, PLL_OUTPUT, 0, 0, false)) {
		return false;
	}
	iris_log("iris_probe: VideoCC PLL0 locked at 1332 MHz\n");
	return true;
}

static bool
power_up(struct iris_probe *p, unsigned level, const char **step)
{
	*step = "GCC video AHB/XO";
	iris_log("iris_probe: enabling GCC video gates\n");
	if (!enable_clock(p, p->gcc + GCC_AHB_CBCR, true) ||
	    !enable_clock(p, p->gcc + GCC_XO_CBCR, true)) {
		return false;
	}
	*step = "GCC video reset state";
	uint32_t reset = UINT32_MAX;
	if (!read_reg(p->gcc + GCC_VIDEO_BCR, &reset)) {
		return false;
	}
	iris_log("iris_probe: GCC video BCR %#x\n", reset);
	if ((reset & 1u) && !change_reg(p, p->gcc + GCC_VIDEO_BCR, 1, 0, 0, 0, false)) {
		return false;
	}
	*step = "VideoCC AHB/XO/sleep";
	iris_log("iris_probe: enabling VideoCC interface gates\n");
	if (!enable_clock(p, p->cc + VIDEO_AHB_CBCR, true)) {
		uint32_t ahb, gdsc;
		/* On NE2213 the local AHB enable reads zero with GCC AHB on.
		 * Proceed only if the VideoCC control bus is already responding. */
		if (!read_reg(p->cc + VIDEO_AHB_CBCR, &ahb) || ahb != 0 ||
		    !read_reg(p->cc + VIDEO_CTL_GDSCR, &gdsc) || !(gdsc & GDSC_COLLAPSE)) {
			log_video_cc(p);
			return false;
		}
		iris_log("iris_probe: local AHB enable reads zero, controller GDSC accessible (%#x)\n", gdsc);
	}
	if (!enable_clock(p, p->cc + VIDEO_XO_CBCR, true) ||
	    !enable_clock(p, p->cc + VIDEO_SLEEP_CBCR, false)) {
		log_video_cc(p);
		return false;
	}
	*step = "controller GDSC";
	iris_log("iris_probe: enabling controller GDSC\n");
	if (!enable_domain(p, p->cc + VIDEO_CTL_GDSCR)) {
		return false;
	}
	*step = "controller bridge resets and AXI clock";
	uint32_t axi, ctl;
	vm_offset_t axi_reg = p->gcc + GCC_AXI_CBCR;
	vm_offset_t ctl_reg = p->cc + VIDEO_CTL_CBCR;
	if (!read_reg(axi_reg, &axi) || (axi & (CLK_ENABLE | AXI_RESET)) ||
	    !read_reg(ctl_reg, &ctl) || (ctl & (CLK_ENABLE | AXI_RESET)) ||
	    !change_reg(p, axi_reg, AXI_RESET, AXI_RESET, 0, 0, false)) {
		return false;
	}
	delay(1000);
	if (!change_reg(p, ctl_reg, AXI_RESET, AXI_RESET, 0, 0, false)) {
		return false;
	}
	delay(1000);
	if (!write_reg(axi_reg, axi) || !poll_reg(axi_reg, AXI_RESET, 0, 2000) ||
	    !write_reg(ctl_reg, ctl) || !poll_reg(ctl_reg, AXI_RESET, 0, 2000) ||
	    !enable_clock(p, axi_reg, false)) {
		return false;
	}
	*step = "VideoCC PLL0";
	if (level >= 2 && !enable_pll(p)) {
		return false;
	}
	/* Use TCXO for register access alone and the stock PLL for the firmware. */
	*step = "VideoCC source and controller clock";
	iris_log("iris_probe: selecting %s and enabling the controller clock\n", level >= 2 ? "PLL0" : "TCXO");
	uint32_t rate_mask = level >= 2 ? RCG_RATE_MASK : RCG_RATE_MASK & ~RCG_HW_CONTROL;
	if (!poll_reg(p->cc + VIDEO_RCG_CMD, RCG_UPDATE, 0, 2000) ||
	    !change_reg(p, p->cc + VIDEO_RCG_CFG, rate_mask, level >= 2 ? RCG_HW_CONTROL | 0x101 : 0, 0, 0, true) ||
	    !enable_clock(p, p->cc + VIDEO_CTL_CBCR, true)) {
		return false;
	}
	*step = "codec GDSC and clock";
	iris_log("iris_probe: enabling codec GDSC and clock\n");
	p->codec_mark = p->count;
	if (!enable_domain(p, p->cc + VIDEO_CORE_GDSCR) ||
	    !enable_clock(p, p->cc + VIDEO_CORE_CBCR, true)) {
		return false;
	}
	*step = "powered codec registers";
	uint32_t status, wrapper;
	if (!read_reg(p->regs + CTRL_STATUS, &status) || !read_reg(p->regs + CORE_CLOCK_CONFIG, &wrapper)) {
		return false;
	}
	iris_log("iris_probe: power on, control status %#x, wrapper clock %#x\n", status, wrapper);
	p->codec_on = true;
	return true;
}

static bool
restore_changes(struct iris_probe *p, unsigned down_to)
{
	bool ok = true;
	while (p->count > down_to) {
		const struct iris_change *change = &p->changes[--p->count];
		uint32_t value;
		bool restored = read_reg(change->reg, &value) &&
		    write_reg(change->reg, (value & ~change->mask) | (change->old & change->mask));
		if (restored && change->rcg) {
			restored = write_reg(change->reg - 4, RCG_UPDATE) &&
			    poll_reg(change->reg - 4, RCG_UPDATE, 0, 2000);
		}
		if (restored) {
			uint32_t bits = change->mask | change->status;
			restored = poll_reg(change->reg, bits, change->old & bits, 4000);
		}
		if (!restored) {
			iris_log("iris_probe: restore failed for MMIO %#lx mask %#x\n",
			    (unsigned long)change->reg, change->mask);
			ok = false;
		}
	}
	return ok;
}

static void
poll_logged(struct iris_probe *p, unsigned reg, uint32_t mask, uint32_t want, const char *name)
{
	if (!poll_reg(p->regs + reg, mask, want, 2000)) {
		uint32_t value = UINT32_MAX;
		(void)read_reg(p->regs + reg, &value);
		iris_log("iris_probe: power-off %s not acknowledged (%#x)\n", name, value);
	}
}

/* Stock __power_off_iris2_hardware, the codec clock and GDSC, then the NoC
 * low-power requests of __power_off_iris2_controller. The controller clocks
 * and GDSC follow in restore_power. */
static bool
quiesce_codec(struct iris_probe *p)
{
	if (!p->codec_on) {
		return true;
	}
	p->codec_on = false;
	vm_offset_t r = p->regs;
	for (unsigned i = 0; i < VCODEC_PIPES; ++i) {
		poll_logged(p, VCODEC_SS_IDLE + 4 * i, 0x400000, 0x400000, "codec pipe idle");
	}
	bool ok = write_reg(r + MVP_NOC_RESET_REQ, 3);
	poll_logged(p, MVP_NOC_RESET_ACK, 3, 3, "MVP NoC reset assert");
	ok = write_reg(r + MVP_NOC_RESET_REQ, 0) && ok;
	poll_logged(p, MVP_NOC_RESET_ACK, 3, 0, "MVP NoC reset release");
	ok = write_reg(r + AHB_BRIDGE_SYNC_RESET, 3) && write_reg(r + AHB_BRIDGE_SYNC_RESET, 2) &&
	    write_reg(r + AHB_BRIDGE_SYNC_RESET, 0) && ok;
	/* The codec clock and GDSC, and the wrapper settings made after them. */
	ok = restore_changes(p, p->codec_mark) && ok;
	uint32_t value = 0;
	ok = write_reg(r + X2RPMH, 3) && ok;
	ok = read_reg(r + MVP_NOC_LPI_CONTROL, &value) && write_reg(r + MVP_NOC_LPI_CONTROL, value | 1) && ok;
	poll_logged(p, MVP_NOC_LPI_STATUS, 1, 1, "MVP NoC low power");
	ok = read_reg(r + CPU_NOC_LPI_CONTROL, &value) && write_reg(r + CPU_NOC_LPI_CONTROL, value | 1) && ok;
	poll_logged(p, CPU_NOC_LPI_STATUS, 1, 1, "CPU NoC low power");
	ok = write_reg(r + DEBUG_BRIDGE_LPI_CONTROL, 7) && ok;
	poll_logged(p, DEBUG_BRIDGE_LPI_STATUS, 7, 7, "debug bridge low power");
	ok = write_reg(r + DEBUG_BRIDGE_LPI_CONTROL, 0) && ok;
	poll_logged(p, DEBUG_BRIDGE_LPI_STATUS, UINT32_MAX, 0, "debug bridge release");
	iris_log("iris_probe: codec power-off handshake %s\n", ok ? "done" : "failed");
	return ok;
}

static bool
restore_power(struct iris_probe *p)
{
	bool ok = quiesce_codec(p);
	ok = restore_changes(p, 0) && ok;
	return hakios_cam_power_vote(&p->power, HAKIOS_RPMH_IRIS, false) && ok;
}

static bool
scm(const char *name, uint32_t op, uint32_t arginfo, uint64_t a, uint64_t b, uint64_t c, uint64_t d)
{
	uint64_t token = 0;
	for (unsigned i = 0; i < 100; ++i) {
		register uint64_t x0 __asm__("x0") = i ? 1 : 0x42000000u | op;
		register uint64_t x1 __asm__("x1") = arginfo;
		register uint64_t x2 __asm__("x2") = a;
		register uint64_t x3 __asm__("x3") = b;
		register uint64_t x4 __asm__("x4") = c;
		register uint64_t x5 __asm__("x5") = d;
		register uint64_t x6 __asm__("x6") = token;
		register uint64_t x7 __asm__("x7") = 0;
		__asm__ volatile ("smc #0" : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3), "+r"(x4),
		    "+r"(x5), "+r"(x6), "+r"(x7) :
		    : "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
		token = x6;
		if (x0 != 1) {
			iris_log("iris_probe: %s SCM %d, result %#llx\n", name, (int32_t)x0, x1);
			return x0 == 0 && x1 == 0;
		}
	}
	iris_log("iris_probe: %s SCM interrupted repeatedly\n", name);
	return false;
}

static void
hfi_sync(const volatile void *addr, unsigned size)
{
	flush_dcache64((addr64_t)addr, size, FALSE);
	__asm__ volatile ("dsb sy" ::: "memory");
}

/* Stock SMMU tables remain writable by HLOS. The ARP payload is owned by video. */
static bool
video_memory_ownership(uint64_t pa, unsigned long size, bool tables, bool grant)
{
	struct iris_assign_request {
		_Alignas(64) uint64_t region[2];
		_Alignas(64) uint32_t sources[2];
		_Alignas(64) struct {
			uint32_t vmid, permissions;
			uint64_t context;
			uint32_t context_size, reserved;
		} destinations[2];
		_Alignas(64) uint64_t extra_args[4];
	};
	vm_offset_t request = 0;
	const unsigned flags = KMA_KOBJECT | KMA_ZERO | KMA_DATA;
	if (!pa || kmem_alloc(kernel_map, &request, PAGE_SIZE, flags, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return false;
	}
	uint64_t request_pa = kvtophys(request);
	if (!request_pa) {
		kmem_free(kernel_map, request, PAGE_SIZE);
		return false;
	}
	struct iris_assign_request *args = (void *)request;
	uint64_t sources_pa = request_pa + offsetof(struct iris_assign_request, sources);
	uint64_t destinations_pa = request_pa + offsetof(struct iris_assign_request, destinations);
	uint64_t extra_pa = request_pa + offsetof(struct iris_assign_request, extra_args);
	args->region[0] = pa;
	args->region[1] = size;
	args->extra_args[0] = sizeof(args->sources[0]);
	args->extra_args[1] = destinations_pa;
	args->extra_args[2] = sizeof(args->destinations[0]);
	args->sources[0] = grant || tables ? VMID_HLOS : VMID_VIDEO_NONPIXEL;
	args->destinations[0].permissions = VM_PERM_RWX;
	args->destinations[0].vmid = VMID_HLOS;
	if (grant && tables) {
		args->destinations[0].permissions = VM_PERM_RW;
		args->destinations[1].vmid = VMID_VIDEO_NONPIXEL;
		args->destinations[1].permissions = VM_PERM_R;
		args->extra_args[2] = sizeof(args->destinations);
	} else if (grant) {
		args->destinations[0].vmid = VMID_VIDEO_NONPIXEL;
		args->destinations[0].permissions = VM_PERM_RW;
	} else if (tables) {
		args->sources[1] = VMID_VIDEO_NONPIXEL;
		args->extra_args[0] = sizeof(args->sources);
	}
	hfi_sync(args, sizeof(*args));
	const char *name = tables ? (grant ? "secure table assignment" : "secure table return") :
	    (grant ? "secure ARP assignment" : "secure ARP return");
	bool ok = scm(name, SCM_MP_ASSIGN, SCM_ASSIGN_ARGS, request_pa,
	    sizeof(args->region), sources_pa, extra_pa);
	kmem_free(kernel_map, request, PAGE_SIZE);
	return ok;
}

static bool
video_table_ownership(unsigned long long pa, unsigned long size, bool grant)
{
	return video_memory_ownership(pa, size, true, grant);
}

static int
probe_table_assignment(void)
{
	vm_offset_t backing = 0;
	const unsigned flags = KMA_KOBJECT | KMA_ZERO | KMA_DATA;
	if (kmem_alloc(kernel_map, &backing, PAGE_SIZE, flags, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return -1;
	}
	uint64_t pa = kvtophys(backing);
	if (!pa) {
		kmem_free(kernel_map, backing, PAGE_SIZE);
		return -1;
	}
	volatile uint64_t *sentinel = (void *)backing;
	*sentinel = UINT64_C(0x4952495332505445);
	hfi_sync(sentinel, PAGE_SIZE);
	if (!video_table_ownership(pa, PAGE_SIZE, true) ||
	    !video_table_ownership(pa, PAGE_SIZE, false)) {
		iris_log("iris_probe: retaining scratch page after uncertain ownership\n");
		return -2;
	}
	int result = *sentinel == UINT64_C(0x4952495332505445) ? 1 : -1;
	kmem_free(kernel_map, backing, PAGE_SIZE);
	return result;
}

/*
 * The HFI queues' and the secure ARP buffer's physically contiguous memory. /dev/hakios-iris (level 7) takes both
 * once, when it starts and memory is not yet in pieces, and every photograph reuses them: a 3 MiB contiguous
 * allocation made per photograph failed on the third one in a Camera.app session (Live Photo's staging moves 41 MB
 * buffers), and that photograph went to the software encoder. The diagnostic levels allocate per run as before.
 */
static vm_offset_t kept_hfi_backing, kept_arp_backing;
static bool keep_backings;

static bool
contig_backing(vm_offset_t *backing, vm_offset_t *kept, vm_size_t size, const char *what)
{
	if (keep_backings && *kept) {
		*backing = *kept;
		bzero((void *)*backing, size);
		return true;
	}
	kern_return_t kr = kmem_alloc_contig(kernel_map, backing, size, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT);
	if (kr != KERN_SUCCESS) {
		*backing = 0;
		iris_log("iris_probe: %s: %lu contiguous bytes not allocated (%d)\n", what, (unsigned long)size, kr);
		return false;
	}
	if (keep_backings) {
		*kept = *backing;
	}
	return true;
}

/* Free a contiguous backing, or keep it for the next request. */
static void
free_backing(vm_offset_t backing, vm_offset_t kept, vm_size_t size)
{
	if (!keep_backings || backing != kept) {
		kmem_free(kernel_map, backing, size);
	}
}

static bool
setup_queues(struct iris_probe *p, bool encode)
{
	if (!contig_backing(&p->backing, &kept_hfi_backing, HFI_DMA_SIZE, "HFI queues")) {
		return false;
	}
	uint64_t pa = kvtophys(p->backing);
	if (!pa) {
		iris_log("iris_probe: HFI queues: no physical address\n");
		return false;
	}
	/* Use the uncached CPU alias for all queue accesses after clearing the
	 * cached alias. Firmware treats the shared window as uncached. This
	 * also avoids cleaning a line the other side may be updating. */
	hfi_sync((void *)p->backing, HFI_DMA_SIZE);
	p->queues = ml_io_map_unmappable(pa, HFI_DMA_SIZE, VM_WIMG_WCOMB);
	if (!p->queues) {
		iris_log("iris_probe: HFI queues: no uncached mapping\n");
		return false;
	}
	p->smmu = hakios_smmu_claim(0x2180, 0x400, HFI_IOVA,
	    HFI_DMA_SIZE + (encode ? p->media_size : 0), "Iris2 HFI");
	if (!p->smmu) {
		iris_log("iris_probe: HFI queues: SMMU context not claimed\n");
		return false;
	}
	if (hakios_smmu_map(p->smmu, HFI_IOVA, pa, HFI_DMA_SIZE, HAKIOS_SMMU_READ | HAKIOS_SMMU_WRITE)) {
		iris_log("iris_probe: HFI queues: SMMU map failed\n");
		return false;
	}
	p->mapped = true;
	struct hfi_table *table = (struct hfi_table *)p->queues;
	*table = (struct hfi_table){ .size = sizeof(*table), .queue_offset = offsetof(struct hfi_table, queue),
		.queue_header_size = sizeof(struct hfi_queue), .count = 3, .active = 3, .name = "msm_v4l2_vidc" };
	for (unsigned i = 0; i < 3; ++i) {
		table->queue[i] = (struct hfi_queue){ .status = 1,
			.address = HFI_IOVA + sizeof(*table) + i * HFI_RING_WORDS * sizeof(uint32_t),
			.type = 0x01010000u | i, .words = HFI_RING_WORDS,
			.rx_watermark = 1, .tx_watermark = 1, .rx_request = i != 2 };
	}
	*(uint32_t *)(p->queues + HFI_SFR_OFFSET) = 0x1000;
	__asm__ volatile ("dsb sy" ::: "memory");
	return true;
}

static uint32_t
read32(const uint8_t *at)
{
	uint32_t value;
	memcpy(&value, at, sizeof(value));
	return value;
}

static uint16_t
read16(const uint8_t *at)
{
	uint16_t value;
	memcpy(&value, at, sizeof(value));
	return value;
}

/* Split the MBN as the stock mdt loader does: headers plus hash segment, then the load segments by address. */
static bool
parse_firmware(void)
{
	if (fw.loads) {
		return true;
	}
	uint64_t size = 0;
	const uint8_t *file = hakios_image_file("fw", IRIS_FIRMWARE, &size, NULL);
	const char *problem = NULL;
	uint32_t phoff = 0, phnum = 0;
	if (!file) {
		problem = "not in the boot image";
	} else if (size < 52 || size > UINT32_MAX || memcmp(file, "\x7f" "ELF\x01\x01\x01", 7) != 0 ||
	    read16(file + 42) != 32) {
		problem = "not an ELF32 image";
	} else {
		phoff = read32(file + 28);
		phnum = read16(file + 44);
		if (phoff < 52 || phnum < 2 || phnum > MBN_MAX_SEGMENTS || phoff + 32u * phnum > size) {
			problem = "invalid program header table";
		}
	}
	uint32_t header_size = 0, hash_offset = 0, hash_size = 0, hashes = 0, loads = 0;
	uint32_t offset[MBN_MAX_SEGMENTS], address[MBN_MAX_SEGMENTS], file_size[MBN_MAX_SEGMENTS], memory_size[MBN_MAX_SEGMENTS];
	bool relocatable = false;
	if (!problem) {
		header_size = read32(file + phoff + 16);
		if (read32(file + phoff) == ELF_PT_LOAD || header_size < phoff + 32u * phnum || header_size > size) {
			problem = "invalid PAS metadata headers";
		}
	}
	for (uint32_t i = 1; !problem && i < phnum; i++) {
		const uint8_t *ph = file + phoff + 32u * i;
		uint32_t type = read32(ph), flags = read32(ph + 24);
		if ((flags & MDT_TYPE_MASK) == MDT_TYPE_HASH) {
			hash_offset = read32(ph + 4);
			hash_size = read32(ph + 16);
			hashes++;
		} else if (type == ELF_PT_LOAD && read32(ph + 20)) {
			/* Insertion by physical address. */
			uint32_t at = loads++;
			while (at && address[at - 1] > read32(ph + 12)) {
				offset[at] = offset[at - 1];
				address[at] = address[at - 1];
				file_size[at] = file_size[at - 1];
				memory_size[at] = memory_size[at - 1];
				at--;
			}
			offset[at] = read32(ph + 4);
			address[at] = read32(ph + 12);
			file_size[at] = read32(ph + 16);
			memory_size[at] = read32(ph + 20);
			relocatable = relocatable || (flags & MDT_RELOCATABLE);
		}
	}
	if (!problem && (hashes != 1 || hash_offset < header_size || !hash_size ||
	    (uint64_t)hash_offset + hash_size > size || header_size + hash_size > PAGE_SIZE)) {
		problem = "invalid PAS hash segment";
	}
	if (!problem && (!loads || !relocatable)) {
		problem = "no relocatable load segments";
	}
	uint64_t end = 0;
	for (uint32_t i = 0; !problem && i < loads; i++) {
		uint64_t at = address[i] - address[0];
		if (at < end || at + memory_size[i] > FW_SIZE || file_size[i] > memory_size[i] ||
		    (file_size[i] && (uint64_t)offset[i] + file_size[i] > size)) {
			problem = "invalid load segment";
		}
		fw.load[i].offset = offset[i];
		fw.load[i].destination = (uint32_t)at;
		fw.load[i].size = file_size[i];
		end = at + memory_size[i];
	}
	if (!problem && ((end + 0xfff) & ~0xfffull) != FW_SIZE) {
		problem = "span differs from the stock PAS memory";
	}
	if (problem) {
		if (!fw.reported) {
			iris_log("iris_probe: fw/%s %s\n", IRIS_FIRMWARE, problem);
		}
		fw.reported = true;
		return false;
	}
	fw.file = file;
	fw.header_size = header_size;
	fw.hash_offset = hash_offset;
	fw.hash_size = hash_size;
	fw.loads = loads;
	iris_log("iris_probe: fw/%s, %llu bytes: %u load segments, %#x bytes of PAS metadata\n", IRIS_FIRMWARE, size,
	    loads, header_size + hash_size);
	return true;
}

static bool
load_firmware(struct iris_probe *p, const char **step)
{
	*step = "fw/" IRIS_FIRMWARE;
	if (!parse_firmware()) {
		return false;
	}
	*step = "reserved firmware memory";
	if (FW_PA < gPhysBase + gPhysSize && FW_PA + FW_SIZE > gPhysBase) {
		return false;
	}
	*step = "PAS metadata allocation";
	if (kmem_alloc(kernel_map, &p->metadata, PAGE_SIZE, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return false;
	}
	uint64_t pa = kvtophys(p->metadata);
	if (!pa) {
		return false;
	}
	memcpy((void *)p->metadata, fw.file, fw.header_size);
	memcpy((void *)(p->metadata + fw.header_size), fw.file + fw.hash_offset, fw.hash_size);
	hfi_sync((void *)p->metadata, PAGE_SIZE);
	*step = "PAS INIT_IMAGE";
	if (!scm("PAS INIT_IMAGE", PAS_INIT_IMAGE, 2 | (2 << 6), PAS_ID, pa, 0, 0)) {
		return false;
	}
	p->pas_initialized = true;
	*step = "PAS MEM_SETUP";
	if (!scm("PAS MEM_SETUP", PAS_MEM_SETUP, 3, PAS_ID, FW_PA, FW_SIZE, 0)) {
		return false;
	}
	*step = "copy stock firmware";
	vm_offset_t firmware = ml_io_map_unmappable(FW_PA, FW_SIZE, VM_WIMG_WCOMB);
	if (!firmware) {
		return false;
	}
	bzero((void *)firmware, FW_SIZE);
	for (uint32_t i = 0; i < fw.loads; i++) {
		memcpy((void *)(firmware + fw.load[i].destination), fw.file + fw.load[i].offset, fw.load[i].size);
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	ml_io_unmap(firmware, FW_SIZE);
	*step = "PAS AUTH_AND_RESET";
	p->auth_attempted = true;
	return scm("PAS AUTH_AND_RESET", PAS_AUTH_AND_RESET, 1, PAS_ID, 0, 0, 0);
}

static bool
boot_codec(struct iris_probe *p)
{
	vm_offset_t regs = p->regs;
	/* Poll the message queue. Leave the hardware-to-host IRQ masked. */
	if (!change_reg(p, regs + WRAPPER_INTR_MASK, 0xc, 0xc, 0, 0, false) ||
	    !change_reg(p, regs + CORE_CLOCK_CONFIG, 0x11, 0, 0, 0, false) ||
	    !write_reg(regs + UC_REGION_ADDR, HFI_IOVA) ||
	    !write_reg(regs + UC_REGION_SIZE, HFI_DMA_SIZE) ||
	    !write_reg(regs + QTBL_ADDR, HFI_IOVA) || !write_reg(regs + QTBL_INFO, 1) ||
	    !write_reg(regs + SFR_ADDR, HFI_IOVA + HFI_SFR_OFFSET + 1) ||
	    !write_reg(regs + CTRL_INIT, 1) ||
	    !write_reg(regs + HFI_VERSION_INFO, 1)) {
		return false;
	}
	uint32_t status = 0;
	for (unsigned i = 0; i < 2000; ++i) {
		if (!read_reg(regs + CTRL_STATUS, &status) || (status & 0xfe)) {
			iris_log("iris_probe: codec boot control error %#x\n", status);
			return false;
		}
		if (status & 1) {
			iris_log("iris_probe: codec boot control ready %#x\n", status);
			return write_reg(regs + H2XSOFTINTEN, 1) && write_reg(regs + X2RPMH, 0);
		}
		delay(50);
	}
	uint32_t init = UINT32_MAX, cpu = UINT32_MAX, intr = UINT32_MAX;
	(void)read_reg(regs + CTRL_INIT, &init);
	(void)read_reg(regs + WRAPPER_TZ_CPU_STATUS, &cpu);
	(void)read_reg(regs + WRAPPER_INTR_STATUS, &intr);
	iris_log("iris_probe: codec boot control timed out (init %#x status %#x cpu %#x irq %#x)\n",
	    init, status, cpu, intr);
	return false;
}

static bool
hfi_send(struct iris_probe *p, const void *packet, unsigned size)
{
	volatile struct hfi_table *table = (volatile struct hfi_table *)p->queues;
	volatile struct hfi_queue *queue = &table->queue[0];
	unsigned start = queue->write_index;
	/* A single diagnostic session never wraps the command ring. */
	if (start >= HFI_RING_WORDS || size % 4 || size / 4 >= HFI_RING_WORDS - start) {
		return false;
	}
	void *ring = (void *)(p->queues + sizeof(*table) + start * sizeof(uint32_t));
	memcpy(ring, packet, size);
	/* Publish payload before the write index, then ring the MMIO doorbell. */
	__atomic_store_n(&queue->write_index, start + size / sizeof(uint32_t), __ATOMIC_RELEASE);
	__asm__ volatile ("dsb sy" ::: "memory");
	return write_reg(p->regs + HOST_DOORBELL, 1);
}

static int
hfi_response(struct iris_probe *p, const uint32_t *words, unsigned count,
    uint32_t session, uint32_t wanted)
{
	if (count < 8 || (words[1] != 0 && words[1] != session)) {
		return -1;
	}
	int result = 0;
	unsigned at = 8;
	for (unsigned i = 0; i < words[7]; ++i) {
		if (count - at < 8) {
			return -1;
		}
		const uint32_t *pkt = words + at;
		unsigned size = pkt[0];
		if ((size & 3) || size < 32 || size / 4 > count - at) {
			return -1;
		}
		iris_log("iris_probe: HFI packet type %#x flags %#x bytes %u id %u port %u\n",
		    pkt[1], pkt[2], size, pkt[5], pkt[4]);
		if (pkt[2] & HFI_FW_ERROR) {
			return -1;
		}
		if (words[1] == session && pkt[1] == wanted) {
			if (wanted == HFI_CMD_BUFFER &&
			    (pkt[4] != HFI_PORT_BITSTREAM || size != 32 + sizeof(struct hfi_buffer))) {
				at += size / 4;
				continue;
			}
			if (wanted != HFI_IMAGE_VERSION && wanted != HFI_CMD_BUFFER &&
			    !(pkt[2] & HFI_FW_SUCCESS)) {
				return -1;
			}
			if (wanted == HFI_CMD_BUFFER) {
				if (p->pending_count == sizeof(p->pending) / sizeof(p->pending[0])) {
					return -1;
				}
				memcpy(&p->pending[p->pending_count++], pkt + 8, sizeof(p->pending[0]));
			}
			if (wanted == HFI_IMAGE_VERSION) {
				static const char expected[] = "video-firmware.2.4-74704eec8d7d8abb3a3628b38c3ac44b4169b83e";
				char text[129];
				unsigned n = MIN(size - 32, sizeof(text) - 1);
				const uint8_t *payload = (const uint8_t *)(pkt + 8);
				for (unsigned j = 0; j < n; ++j) {
					text[j] = payload[j] >= 32 && payload[j] < 127 ? (char)payload[j] : ' ';
				}
				text[n] = '\0';
				iris_log("iris_probe: firmware version %s\n", text);
				unsigned start = 0;
				while (start + sizeof(expected) - 1 <= n &&
				    memcmp(text + start, expected, sizeof(expected) - 1) != 0) {
					++start;
				}
				if (start + sizeof(expected) - 1 > n) {
					return -1;
				}
			}
			result = 1;
		}
		at += size / 4;
	}
	return at == count ? result : -1;
}

static bool
hfi_wait(struct iris_probe *p, uint32_t session, uint32_t wanted, struct hfi_buffer *output)
{
	if (output && p->pending_read < p->pending_count) {
		*output = p->pending[p->pending_read++];
		return true;
	}
	p->pending_read = p->pending_count = 0;
	volatile struct hfi_table *table = (volatile struct hfi_table *)p->queues;
	volatile struct hfi_queue *queue = &table->queue[1];
	volatile const uint32_t *ring = (volatile const uint32_t *)
	    (p->queues + sizeof(*table) + HFI_RING_WORDS * sizeof(uint32_t));
	unsigned consumed = queue->read_index;
	for (unsigned i = 0; i < 1000; ++i) {
		unsigned end = __atomic_load_n(&queue->write_index, __ATOMIC_ACQUIRE);
		if (end < consumed || end >= HFI_RING_WORDS) {
			return false;
		}
		if (end == consumed) {
			delay(1000);
			continue;
		}
		unsigned size = ring[consumed];
		uint32_t words[256];
		if ((size & 3) || size < 32 || size > sizeof(words) || size / 4 > end - consumed) {
			iris_log("iris_probe: invalid HFI response size %u indices %u/%u\n", size, consumed, end);
			return false;
		}
		memcpy(words, (const void *)(ring + consumed), size);
		consumed += size / 4;
		__atomic_store_n(&queue->read_index, consumed, __ATOMIC_RELEASE);
		int result = hfi_response(p, words, size / 4, session, wanted);
		if (result) {
			if (result > 0 && output) {
				*output = p->pending[p->pending_read++];
			}
			return result > 0;
		}
	}
	uint32_t ctrl = UINT32_MAX, irq = UINT32_MAX, cpu = UINT32_MAX, soft = UINT32_MAX;
	(void)read_reg(p->regs + CTRL_STATUS, &ctrl);
	(void)read_reg(p->regs + WRAPPER_INTR_STATUS, &irq);
	(void)read_reg(p->regs + WRAPPER_TZ_CPU_STATUS, &cpu);
	(void)read_reg(p->regs + H2XSOFTINTEN, &soft);
	iris_log("iris_probe: HFI response %#x timed out (ctrl %#x irq %#x cpu %#x h2x %#x)\n",
	    wanted, ctrl, irq, cpu, soft);
	iris_log("iris_probe: HFI read/write indices cmd %u/%u msg %u/%u dbg %u/%u\n",
	    table->queue[0].read_index, table->queue[0].write_index,
	    queue->read_index, queue->write_index, table->queue[2].read_index, table->queue[2].write_index);
	const volatile uint8_t *sfr = (const volatile uint8_t *)(p->queues + HFI_SFR_OFFSET + 4);
	char text[129];
	unsigned n = 0;
	while (n < sizeof(text) - 1 && sfr[n]) {
		text[n] = sfr[n] >= 32 && sfr[n] < 127 ? (char)sfr[n] : ' ';
		++n;
	}
	text[n] = '\0';
	if (n) {
		iris_log("iris_probe: firmware failure record %s\n", text);
	}
	return false;
}

static bool
hfi_command(struct iris_probe *p, uint32_t session, uint32_t type,
    uint32_t flags, uint32_t port, uint32_t payload_type, const void *payload, unsigned bytes)
{
	unsigned size = 64 + bytes;
	uint32_t message[16 + sizeof(struct hfi_buffer) / sizeof(uint32_t)] = {
		size, session, p->command_header++, 0, 0, 0, 0, 1,
		size - 32, type, flags, payload_type, port, p->command_packet++, 0, 0,
	};
	if (bytes) {
		memcpy(message + 16, payload, bytes);
	}
	return hfi_send(p, message, size);
}

static bool
hfi_u32_command(struct iris_probe *p, uint32_t session, uint32_t type,
    uint32_t flags, uint32_t port, uint32_t payload_type, uint32_t low, uint32_t high)
{
	uint32_t values[] = { low, high };
	unsigned bytes = payload_type == HFI_PAYLOAD_NONE ? 0 :
	    (payload_type == HFI_PAYLOAD_PACK64 ? 8 : 4);
	return hfi_command(p, session, type, flags, port, payload_type, values, bytes);
}

static bool
resume_idle_codec(struct iris_probe *p, const char **step)
{
	vm_offset_t status = p->regs + CTRL_STATUS;
	vm_offset_t wfi = p->regs + WRAPPER_TZ_CPU_STATUS;
	*step = "HFI power-collapse readiness";
	if (!poll_reg(status, 1u << 30, 1u << 30, 30000) ||
	    !poll_reg(wfi, 1, 1, 30000) ||
	    !hfi_command(p, 0, HFI_POWER_COLLAPSE, 0, HFI_PORT_NONE, HFI_PAYLOAD_NONE, NULL, 0) ||
	    !poll_reg(status, 0x100, 0x100, 10000) ||
	    !poll_reg(wfi, 1, 1, 10000)) {
		return false;
	}
	*step = "video TrustZone suspend and resume";
	if (!scm("VIDEO SUSPEND", SCM_REMOTE_STATE, 2, 0, 0, 0, 0) ||
	    !scm("VIDEO RESUME", SCM_REMOTE_STATE, 2, 1, 0, 0, 0)) {
		return false;
	}
	*step = "HFI restart after video resume";
	return boot_codec(p) &&
	    hfi_u32_command(p, 0, HFI_IMAGE_VERSION, HFI_HOST_REPLY, HFI_PORT_NONE, HFI_PAYLOAD_NONE, 0, 0) &&
	    hfi_wait(p, 0, HFI_IMAGE_VERSION, NULL);
}

static bool
setup_arp(struct iris_probe *p)
{
	const unsigned size = round_page(IRIS_ARP_SIZE);
	if (!contig_backing(&p->arp_backing, &kept_arp_backing, size, "secure ARP")) {
		return false;
	}
	p->arp_pa = kvtophys(p->arp_backing);
	if (!p->arp_pa) {
		return false;
	}
	p->arp = hakios_smmu_claim_with_table_ownership(0x2184, 0x400, IRIS_ARP_IOVA,
	    size, "Iris2 secure ARP", video_table_ownership);
	if (!p->arp || hakios_smmu_map(p->arp, IRIS_ARP_IOVA, p->arp_pa, size,
	    HAKIOS_SMMU_READ | HAKIOS_SMMU_WRITE)) {
		return false;
	}
	hfi_sync((void *)p->arp_backing, size);
	p->arp_assignment_attempted = true;
	return video_memory_ownership(p->arp_pa, size, false, true);
}

static bool
setup_media(struct iris_probe *p)
{
	unsigned total = p->media_size + p->pixel_size;
	unsigned pixel_iova = IRIS_MEDIA_IOVA + p->media_size;
	const unsigned flags = KMA_KOBJECT | KMA_ZERO | KMA_DATA;
	kern_return_t allocated = p->still ?
	    kmem_alloc(kernel_map, &p->media_backing, total, flags, VM_KERN_MEMORY_IOKIT) :
	    kmem_alloc_contig(kernel_map, &p->media_backing, total, 0, 0, 0, flags, VM_KERN_MEMORY_IOKIT);
	if (allocated != KERN_SUCCESS) {
		iris_log("iris_probe: media allocation of %u bytes failed (%d)\n", total, allocated);
		return false;
	}
	if (p->still) {
		/* The camera already has a wired ring. Map the still by page through the SMMU. */
		uint8_t *nv12 = (uint8_t *)(p->media_backing + p->media_size);
		const struct hakios_iris_still *still = p->still;
		const user_addr_t planes[2] = { still->input, still->input_uv };
		const unsigned strides[2] = { still->stride, still->stride_uv };
		for (unsigned plane = 0; plane < 2; ++plane) {
			unsigned rows = IRIS_GRID_LINES >> plane;
			unsigned height = IRIS_GRID_HEIGHT >> plane;
			user_addr_t input = planes[plane];
			memset(nv12, plane ? 128 : 16, rows * IRIS_GRID_STRIDE);
			for (unsigned y = 0; y < height; ++y) {
				/* Padding rows repeat the last image row, which keeps the edge tiles' padding cheap. */
				int error = copyin(input + (user_addr_t)y * strides[plane], nv12 + y * IRIS_GRID_STRIDE, IRIS_GRID_WIDTH);
				if (error != 0) {
					iris_log("iris_probe: copyin of NV12 plane %u row %u failed (%d)\n", plane, y, error);
					return false;
				}
			}
			for (unsigned y = height; y < rows; ++y) {
				memcpy(nv12 + y * IRIS_GRID_STRIDE, nv12 + (height - 1) * IRIS_GRID_STRIDE, IRIS_GRID_STRIDE);
			}
			for (unsigned y = 0; y < rows; ++y) {
				uint8_t *row = nv12 + y * IRIS_GRID_STRIDE;
				if (plane == 0) {
					memset(row + IRIS_GRID_WIDTH, row[IRIS_GRID_WIDTH - 1], IRIS_GRID_STRIDE - IRIS_GRID_WIDTH);
				} else {
					for (unsigned x = IRIS_GRID_WIDTH; x < IRIS_GRID_STRIDE; x += 2) {
						row[x] = row[IRIS_GRID_WIDTH - 2];
						row[x + 1] = row[IRIS_GRID_WIDTH - 1];
					}
				}
			}
			nv12 += rows * IRIS_GRID_STRIDE;
		}
	}
	hfi_sync((void *)p->media_backing, total);
	uint64_t pa = p->still ? 0 : kvtophys(p->media_backing);
	p->media = p->still ? p->media_backing : pa ? ml_io_map_unmappable(pa, total, VM_WIMG_WCOMB) : 0;
	const unsigned perms = HAKIOS_SMMU_READ | HAKIOS_SMMU_WRITE;
	if (!p->media) {
		iris_log("iris_probe: uncached media mapping failed\n");
		return false;
	}
	int mapped = p->still ? hakios_smmu_map_kva(p->smmu, IRIS_MEDIA_IOVA, p->media_backing, p->media_size, perms) :
	    hakios_smmu_map(p->smmu, IRIS_MEDIA_IOVA, pa, p->media_size, perms);
	if (mapped) {
		iris_log("iris_probe: media SMMU map failed (%d)\n", mapped);
		return false;
	}
	p->media_mapped = true;
	p->pixels = hakios_smmu_claim(0x2187, 0x400, pixel_iova, p->pixel_size, "Iris2 pixels");
	if (!p->pixels) {
		return false;
	}
	mapped = p->still ? hakios_smmu_map_kva(p->pixels, pixel_iova, p->media_backing + p->media_size, p->pixel_size, perms) :
	    hakios_smmu_map(p->pixels, pixel_iova, pa + p->media_size, p->pixel_size, perms);
	if (mapped) {
		iris_log("iris_probe: pixel SMMU map failed (%d)\n", mapped);
		return false;
	}
	p->pixels_mapped = true;
	if (p->still) {
		return true;
	}

	/* Four flat luma bars and neutral chroma for an independent decode check. */
	uint8_t *nv12 = (uint8_t *)(p->media + p->media_size);
	if (p->grid) {
		for (unsigned y = 0; y < IRIS_GRID_LINES; ++y) {
			for (unsigned x = 0; x < IRIS_GRID_STRIDE; x += IRIS_GRID_TILE) {
				unsigned tile = 3 * (x / IRIS_GRID_TILE) + 5 * (y / IRIS_GRID_TILE);
				memset(nv12 + y * IRIS_GRID_STRIDE + x, 40 + 22 * (tile % 8), IRIS_GRID_TILE);
			}
		}
		memset(nv12 + IRIS_GRID_STRIDE * IRIS_GRID_LINES, 128, IRIS_GRID_NV12 / 3);
		__asm__ volatile ("dsb sy" ::: "memory");
		return true;
	}
	for (unsigned y = 0; y < IRIS_TEST_HEIGHT; ++y) {
		for (unsigned x = 0; x < IRIS_TEST_WIDTH; ++x) {
			nv12[y * IRIS_TEST_WIDTH + x] = 32 + 56 * (x / 64);
		}
	}
	memset(nv12 + IRIS_TEST_WIDTH * IRIS_TEST_HEIGHT, 128, IRIS_NV12_SIZE / 3);
	__asm__ volatile ("dsb sy" ::: "memory");
	return true;
}

static bool
hfi_queue_buffer(struct iris_probe *p, unsigned type, unsigned index, unsigned port,
    unsigned iova, unsigned size, unsigned used)
{
	const struct hfi_buffer buffer = {
		.type = type, .index = index, .address = iova, .buffer_size = size, .data_size = used,
	};
	return hfi_command(p, IRIS_SESSION_ID, HFI_CMD_BUFFER, 1, port, HFI_PAYLOAD_STRUCT,
	    &buffer, sizeof(buffer));
}

static bool
encoder_port(struct iris_probe *p, unsigned cmd, unsigned port)
{
	unsigned flags = HFI_HOST_REPLY | (cmd == HFI_CMD_STOP ? HFI_HOST_KEEP : 0);
	return hfi_u32_command(p, IRIS_SESSION_ID, cmd, flags, port, HFI_PAYLOAD_NONE, 0, 0) &&
	    hfi_wait(p, IRIS_SESSION_ID, cmd, NULL);
}

static void
log_firmware_debug(struct iris_probe *p)
{
	volatile struct hfi_table *table = (volatile struct hfi_table *)p->queues;
	volatile struct hfi_queue *queue = &table->queue[2];
	volatile const uint32_t *ring = (volatile const uint32_t *)
	    (p->queues + sizeof(*table) + 2 * HFI_RING_WORDS * sizeof(uint32_t));
	unsigned end = __atomic_load_n(&queue->write_index, __ATOMIC_ACQUIRE);
	if (end >= HFI_RING_WORDS) {
		return;
	}
	for (unsigned at = queue->read_index; at < end;) {
		unsigned size = ring[at];
		if (size < 16 || size % 4 || size / 4 > end - at) {
			break;
		}
		const volatile uint8_t *data = (const volatile uint8_t *)(ring + at + 4);
		char line[193];
		unsigned n = MIN(size - 16, sizeof(line) - 1);
		for (unsigned i = 0; i < n; ++i) {
			uint8_t byte = data[i];
			if (byte == '\0') {
				n = i;
				break;
			}
			line[i] = byte >= 32 && byte < 127 ? (char)byte : ' ';
		}
		line[n] = '\0';
		iris_log("iris_probe: firmware debug %s\n", line);
		at += size / 4;
	}
}

static bool
encode_test_frame(struct iris_probe *p, const char **step)
{
	*step = "encoder DMA buffers";
	if (!setup_media(p)) {
		return false;
	}
	*step = "secure ARP DMA buffer";
	if (!setup_arp(p) || !hfi_queue_buffer(p, HFI_BUF_ARP, 0, HFI_PORT_NONE,
	    IRIS_ARP_IOVA, IRIS_ARP_SIZE, 0)) {
		return false;
	}
	/* Stock NE2213 non-secure work buffers for 256x128 HEVC at 300 kbit/s.
	 * The stock session also queues ARP in its secure non-pixel context. */
	static const struct {
		unsigned type, index, size, iova;
	} work[] = {
		{ HFI_BUF_BIN, 0, 1293824, IRIS_MEDIA_IOVA + 0x20000 },
		{ HFI_BUF_COMV, 0, 8192, IRIS_MEDIA_IOVA + 0x15c000 },
		{ HFI_BUF_NON_COMV, 0, 265472, IRIS_MEDIA_IOVA + 0x160000 },
		{ HFI_BUF_LINE, 0, 326144, IRIS_MEDIA_IOVA + 0x1a4000 },
		{ HFI_BUF_DPB, 0, 57344, IRIS_PIXEL_IOVA + 0xc000 },
		{ HFI_BUF_DPB, 1, 57344, IRIS_PIXEL_IOVA + 0x1c000 },
	};
	*step = "HFI encoder controls";
	if (!hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_STAGE, 0,
	    HFI_PORT_NONE, HFI_PAYLOAD_U32, 2, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_PIPE, 0,
	    HFI_PORT_NONE, HFI_PAYLOAD_U32, 4, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_PROFILE, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_ENUM, 0, 0) || /* HEVC Main */
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_FRAME_RATE, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_Q16, 30u << 16, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_GOP, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_U32, 0, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_B_FRAMES, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_U32, 0, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_LAYER_TYPE, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_ENUM, 1, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_LAYER_COUNT, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_U32, 1, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_RC, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_ENUM, 0, 0) || /* VBR CFR */
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_BITRATE, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_U32, 300000, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_BLUR, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_ENUM, 0, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_SEQ_HEADER, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_ENUM, 2, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_HOST_COUNT, 0,
	    HFI_PORT_RAW, HFI_PAYLOAD_U32, 64, 0) ||
	    !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_HOST_COUNT, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_U32, 64, 0)) {
		return false;
	}
	*step = "HFI encoder raw START";
	if (!hfi_queue_buffer(p, HFI_BUF_VPSS, 0, HFI_PORT_RAW,
	    IRIS_PIXEL_IOVA + 0x2c000, 57344, 0) ||
	    !encoder_port(p, HFI_CMD_START, HFI_PORT_RAW)) {
		return false;
	}
	*step = "HFI encoder work buffers";
	for (unsigned i = 0; i < sizeof(work) / sizeof(work[0]); ++i) {
		if (!hfi_queue_buffer(p, work[i].type, work[i].index, HFI_PORT_BITSTREAM,
		    work[i].iova, work[i].size, 0)) {
			return false;
		}
	}
	*step = "HFI encoder quality mode";
	if (!hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_QUALITY, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_ENUM, 1, 0)) { /* Maximum quality */
		return false;
	}
	*step = "HFI encoder bitstream START";
	if (!encoder_port(p, HFI_CMD_START, HFI_PORT_BITSTREAM)) {
		return false;
	}
	*step = "HFI encoder frame";
	struct hfi_buffer output = { 0 };
	uint64_t started = mach_absolute_time();
	if (!hfi_queue_buffer(p, HFI_BUF_BITSTREAM, 0, HFI_PORT_BITSTREAM,
	    IRIS_MEDIA_IOVA, IRIS_OUTPUT_SIZE, 0) ||
	    !hfi_queue_buffer(p, HFI_BUF_RAW, 0, HFI_PORT_RAW, IRIS_PIXEL_IOVA,
	    IRIS_NV12_SIZE, IRIS_NV12_SIZE) ||
	    !hfi_wait(p, IRIS_SESSION_ID, HFI_CMD_BUFFER, &output)) {
		return false;
	}
	if (output.type != HFI_BUF_BITSTREAM || output.index != 0 ||
	    output.address != IRIS_MEDIA_IOVA || output.address_offset != 0 ||
	    output.data_offset > IRIS_OUTPUT_SIZE || output.data_size < 5 ||
	    output.data_size > IRIS_OUTPUT_SIZE - output.data_offset || output.data_size > 8192) {
		iris_log("iris_probe: invalid bitstream type %u index %u address %#llx offset %u/%u size %u\n",
		    output.type, output.index, output.address, output.address_offset,
		    output.data_offset, output.data_size);
		return false;
	}
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - started, &ns);
	__asm__ volatile ("dsb sy" ::: "memory");
	const uint8_t *bytes = (const uint8_t *)(p->media + output.data_offset);
	if (bytes[0] || bytes[1] || (bytes[2] != 1 && (bytes[2] || bytes[3] != 1))) {
		iris_log("iris_probe: missing Annex B start code\n");
		return false;
	}
	iris_log("iris_probe: encoded %u bytes in %llu us\n", output.data_size, ns / 1000);
	static const char hex[] = "0123456789abcdef";
	for (unsigned offset = 0; offset < output.data_size; offset += 64) {
		char line[129];
		unsigned count = MIN(64, output.data_size - offset);
		for (unsigned i = 0; i < count; ++i) {
			uint8_t byte = bytes[offset + i];
			line[2 * i] = hex[byte >> 4];
			line[2 * i + 1] = hex[byte & 15];
		}
		line[count * 2] = '\0';
		iris_log("iris_probe: HEVC %u %s\n", offset, line);
	}
	*step = "HFI encoder STOP";
	return encoder_port(p, HFI_CMD_STOP, HFI_PORT_RAW) &&
	    encoder_port(p, HFI_CMD_STOP, HFI_PORT_BITSTREAM);
}

static bool
encode_grid(struct iris_probe *p, const char **step)
{
	const unsigned pixel_iova = IRIS_MEDIA_IOVA + p->media_size;
	*step = "grid DMA buffers";
	if (!setup_media(p) || !setup_arp(p) ||
	    !hfi_queue_buffer(p, HFI_BUF_ARP, 0, HFI_PORT_NONE, IRIS_ARP_IOVA, IRIS_ARP_SIZE, 0)) {
		return false;
	}
	*step = "grid raw START";
	if (!hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_HOST_COUNT, 0,
	    HFI_PORT_RAW, HFI_PAYLOAD_U32, 256, 0) ||
	    (p->still && p->still->color_info && !hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_SIGNAL_COLOR, 0,
	    HFI_PORT_RAW, HFI_PAYLOAD_PACK32, p->still->color_info, 0)) ||
	    !encoder_port(p, HFI_CMD_START, HFI_PORT_RAW)) {
		return false;
	}
	static const struct {
		unsigned type, port, payload, value;
	} properties[] = {
		{ HFI_PROP_HOST_COUNT, HFI_PORT_BITSTREAM, HFI_PAYLOAD_U32, 256 },
		{ HFI_PROP_FRAME_RATE, HFI_PORT_BITSTREAM, HFI_PAYLOAD_Q16, 1u << 16 },
		{ HFI_PROP_SEQ_HEADER, HFI_PORT_BITSTREAM, HFI_PAYLOAD_ENUM, 9 },
		{ HFI_PROP_RC, HFI_PORT_BITSTREAM, HFI_PAYLOAD_ENUM, 2 },
		{ HFI_PROP_GOP, HFI_PORT_BITSTREAM, HFI_PAYLOAD_U32, 0 },
		{ HFI_PROP_B_FRAMES, HFI_PORT_BITSTREAM, HFI_PAYLOAD_U32, 0 },
		{ HFI_PROP_GRID, HFI_PORT_BITSTREAM, HFI_PAYLOAD_U32, 1 },
		{ HFI_PROP_STAGE, HFI_PORT_NONE, HFI_PAYLOAD_U32, 2 },
		{ HFI_PROP_PIPE, HFI_PORT_NONE, HFI_PAYLOAD_U32, 4 },
		{ HFI_PROP_PROFILE, HFI_PORT_BITSTREAM, HFI_PAYLOAD_ENUM, 1 },
		{ HFI_PROP_CQ, HFI_PORT_BITSTREAM, HFI_PAYLOAD_U32, 90 },
	};
	*step = "grid encoder controls";
	for (unsigned i = 0; i < sizeof(properties) / sizeof(properties[0]); ++i) {
		if (!hfi_u32_command(p, IRIS_SESSION_ID, properties[i].type, 0,
		    properties[i].port, properties[i].payload, properties[i].value, 0)) {
			return false;
		}
	}
	static const struct { unsigned type, size; } work[] = {
		{ HFI_BUF_BIN, 4239872 }, { HFI_BUF_COMV, 58368 },
		{ HFI_BUF_NON_COMV, 272128 }, { HFI_BUF_LINE, 381184 },
	};
	*step = "grid encoder work buffers";
	unsigned addr = IRIS_MEDIA_IOVA + IRIS_GRID_SLOTS * IRIS_GRID_OUTPUT;
	for (unsigned i = 0; i < sizeof(work) / sizeof(work[0]); ++i) {
		if (addr + round_page(work[i].size) > pixel_iova ||
		    !hfi_queue_buffer(p, work[i].type, 0, HFI_PORT_BITSTREAM, addr, work[i].size, 0)) {
			return false;
		}
		addr += round_page(work[i].size);
	}
	const unsigned dpb_size = 401408;
	for (unsigned i = 0; i < 2; ++i) {
		addr = pixel_iova + IRIS_GRID_NV12 + i * round_page(dpb_size);
		if (addr + round_page(dpb_size) > pixel_iova + p->pixel_size ||
		    !hfi_queue_buffer(p, HFI_BUF_DPB, i, HFI_PORT_BITSTREAM, addr, dpb_size, 0)) {
			return false;
		}
	}
	*step = "grid bitstream START";
	if (!hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_QUALITY, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_ENUM, 1, 0) ||
	    !encoder_port(p, HFI_CMD_START, HFI_PORT_BITSTREAM)) {
		return false;
	}
	*step = "grid frame submission";
	uint64_t started = mach_absolute_time();
	for (unsigned i = 0; i < IRIS_GRID_SLOTS; ++i) {
		if (!hfi_queue_buffer(p, HFI_BUF_BITSTREAM, i, HFI_PORT_BITSTREAM,
		    IRIS_MEDIA_IOVA + i * IRIS_GRID_OUTPUT, IRIS_GRID_OUTPUT, 0)) {
			return false;
		}
	}
	if (!hfi_queue_buffer(p, HFI_BUF_RAW, 0, HFI_PORT_RAW,
	    pixel_iova, IRIS_GRID_NV12, IRIS_GRID_NV12)) {
		return false;
	}
	*step = "grid output";
	unsigned total = 0;
	unsigned capacity = p->still ? p->still->capacity : 65536;
	for (unsigned i = 0; i <= IRIS_GRID_TILES; ++i) {
		struct hfi_buffer output;
		if (!hfi_wait(p, IRIS_SESSION_ID, HFI_CMD_BUFFER, &output)) {
			return false;
		}
		unsigned slot = output.index;
		addr = IRIS_MEDIA_IOVA + slot * IRIS_GRID_OUTPUT;
		if (slot >= IRIS_GRID_SLOTS || output.type != HFI_BUF_BITSTREAM || output.address != addr ||
		    output.address_offset || output.data_offset > IRIS_GRID_OUTPUT || output.data_size < 5 ||
		    output.data_size > IRIS_GRID_OUTPUT - output.data_offset || output.data_size > capacity - total) {
			iris_log("iris_probe: invalid grid output %u index %u type %u address %#llx size %u\n",
			    i, slot, output.type, output.address, output.data_size);
			return false;
		}
		__asm__ volatile ("dsb sy" ::: "memory");
		const uint8_t *bytes = (const uint8_t *)(p->media + slot * IRIS_GRID_OUTPUT + output.data_offset);
		if (p->still) {
			hfi_sync(bytes, output.data_size);
		}
		if (bytes[0] || bytes[1] || (bytes[2] != 1 && (bytes[2] || bytes[3] != 1))) {
			return false;
		}
		if (p->still) {
			if (copyout(bytes, p->still->output + total, output.data_size) != 0) {
				return false;
			}
			p->still->packet_bytes[i] = output.data_size;
		} else {
			iris_log("iris_probe: grid output %u bytes %u flags %#x\n", i, output.data_size, output.flags);
			static const char hex[] = "0123456789abcdef";
			for (unsigned offset = 0; offset < output.data_size; offset += 64) {
				char line[129];
				unsigned count = MIN(64, output.data_size - offset);
				for (unsigned j = 0; j < count; ++j) {
					uint8_t byte = bytes[offset + j];
					line[2 * j] = hex[byte >> 4];
					line[2 * j + 1] = hex[byte & 15];
				}
				line[count * 2] = '\0';
				iris_log("iris_probe: HEVC %u %s\n", total + offset, line);
			}
		}
		total += output.data_size;
		if (i + IRIS_GRID_SLOTS <= IRIS_GRID_TILES &&
		    !hfi_queue_buffer(p, HFI_BUF_BITSTREAM, slot, HFI_PORT_BITSTREAM, addr, IRIS_GRID_OUTPUT, 0)) {
			return false;
		}
	}
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - started, &ns);
	if (p->still) {
		p->still->bytes = total;
		p->still->elapsed_us = ns / 1000;
	}
	iris_log("iris_probe: encoded %u bytes in %llu us (includes %s)\n", total, ns / 1000,
	    p->still ? "userspace copy" : "grid log export");
	*step = "grid encoder STOP";
	return encoder_port(p, HFI_CMD_STOP, HFI_PORT_RAW) &&
	    encoder_port(p, HFI_CMD_STOP, HFI_PORT_BITSTREAM);
}

static bool
probe_encoder_session(struct iris_probe *p, bool encode, const char **step)
{
	/* 256x128 meets the linear NV12 stride and scanline requirements
	 * without padding the visible frame. This level exercises no pixel DMA. */
	const unsigned width = p->grid ? IRIS_GRID_WIDTH : IRIS_TEST_WIDTH;
	const unsigned height = p->grid ? IRIS_GRID_HEIGHT : IRIS_TEST_HEIGHT;
	const unsigned stride = p->grid ? IRIS_GRID_STRIDE : width;
	const unsigned scanlines = p->grid ? IRIS_GRID_LINES : height;
	const uint32_t size = (width << 16) | height;
	const uint32_t coded = p->grid ? (IRIS_GRID_TILE << 16) | IRIS_GRID_TILE : size;
	*step = "HFI encoder OPEN";
	if (!hfi_u32_command(p, 0, HFI_CMD_OPEN, HFI_HOST_REPLY, HFI_PORT_NONE,
	    HFI_PAYLOAD_U32, IRIS_SESSION_ID, 0) ||
	    !hfi_wait(p, IRIS_SESSION_ID, HFI_CMD_OPEN, NULL)) {
		return false;
	}

	*step = "HFI encoder properties";
	bool configured = hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_CODEC, 0,
	    HFI_PORT_NONE, HFI_PAYLOAD_ENUM, HFI_ENCODE_HEVC, 0) &&
	    hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_SECURE, 0,
	    HFI_PORT_NONE, HFI_PAYLOAD_U32, 0, 0) &&
	    hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_COLOR, 0,
	    HFI_PORT_RAW, HFI_PAYLOAD_ENUM, HFI_COLOR_NV12, 0) &&
	    hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_RAW, 0,
	    HFI_PORT_RAW, HFI_PAYLOAD_PACK32, size, 0) &&
	    hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_STRIDE, 0,
	    HFI_PORT_RAW, HFI_PAYLOAD_PACK64, (stride << 16) | scanlines, (stride << 16) | (scanlines / 2)) &&
	    hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_BITSTREAM, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_PACK32, coded, 0) &&
	    hfi_u32_command(p, IRIS_SESSION_ID, HFI_PROP_CROP, 0,
	    HFI_PORT_BITSTREAM, HFI_PAYLOAD_PACK64, 0, 0);

	if (configured && encode) {
		configured = p->grid ? encode_grid(p, step) : encode_test_frame(p, step);
	}
	if (configured) {
		*step = "HFI encoder CLOSE";
	}
	bool closed = hfi_u32_command(p, IRIS_SESSION_ID, HFI_CMD_CLOSE,
	    HFI_HOST_REPLY | HFI_HOST_KEEP, HFI_PORT_NONE, HFI_PAYLOAD_NONE, 0, 0) &&
	    hfi_wait(p, IRIS_SESSION_ID, HFI_CMD_CLOSE, NULL);
	if (configured && closed) {
		iris_log("iris_probe: HEVC session opened, %ux%u NV12 properties sent, session closed\n", width, height);
	}
	return configured && closed;
}

static bool
cleanup(struct iris_probe *p)
{
	bool ok = true;
	if (p->pas_initialized) {
		bool shutdown = scm("PAS SHUTDOWN", PAS_SHUTDOWN, 1, PAS_ID, 0, 0, 0);
		if (!shutdown && p->auth_attempted) {
			iris_log("iris_probe: preserving power and DMA after uncertain PAS shutdown\n");
			return false;
		}
		ok = shutdown;
		/* Stock order: PAS shutdown, power off, then the queue and buffer memory. */
		ok = quiesce_codec(p) && ok;
	}
	if (p->arp_backing) {
		bool clean = !p->arp || hakios_smmu_faults(p->arp);
		int release = hakios_smmu_release(p->arp);
		iris_log("iris_probe: ARP SMMU %s, release %d\n", clean ? "clean" : "faulted", release);
		bool reclaimed = !release && (!p->arp_assignment_attempted ||
		    video_memory_ownership(p->arp_pa, round_page(IRIS_ARP_SIZE), false, false));
		if (reclaimed) {
			free_backing(p->arp_backing, kept_arp_backing, round_page(IRIS_ARP_SIZE));
		} else {
			if (p->arp_backing == kept_arp_backing) {
				kept_arp_backing = 0;   /* (still the secure side's: never handed out again) */
			}
			iris_log("iris_probe: retaining ARP backing after uncertain teardown\n");
		}
		ok = ok && clean && reclaimed;
	}
	bool keep_media = false;
	if (p->pixels) {
		bool clean = hakios_smmu_faults(p->pixels);
		int unmap = p->pixels_mapped ? hakios_smmu_unmap(p->pixels, IRIS_MEDIA_IOVA + p->media_size, p->pixel_size) : 0;
		int release = hakios_smmu_release(p->pixels);
		iris_log("iris_probe: pixel SMMU %s, unmap %d, release %d\n", clean ? "clean" : "faulted", unmap, release);
		ok = ok && clean && !unmap && !release;
		keep_media = p->pixels_mapped && unmap && release;
	}
	if (p->smmu) {
		bool clean = hakios_smmu_faults(p->smmu);
		int unmap = p->mapped ? hakios_smmu_unmap(p->smmu, HFI_IOVA, HFI_DMA_SIZE) : 0;
		int media_unmap = p->media_mapped ? hakios_smmu_unmap(p->smmu, IRIS_MEDIA_IOVA, p->media_size) : 0;
		int release = hakios_smmu_release(p->smmu);
		iris_log("iris_probe: SMMU %s, unmap %d, release %d\n", clean ? "clean" : "faulted", unmap, release);
		ok = ok && clean && !unmap && !media_unmap && !release;
		if (p->media_mapped) {
			iris_log("iris_probe: media SMMU unmap %d\n", media_unmap);
			keep_media = keep_media || (media_unmap && release);
		}
		if (release && (!p->mapped || unmap)) {
			if (p->backing == kept_hfi_backing) {
				kept_hfi_backing = 0;
			}
			p->backing = 0; /* Translations could still reference the wired backing. */
		}
	}
	if (p->media && p->media != p->media_backing) {
		ml_io_unmap(p->media, p->media_size + p->pixel_size);
	}
	if (p->media_backing && !keep_media) {
		kmem_free(kernel_map, p->media_backing, p->media_size + p->pixel_size);
	}
	if (p->queues) {
		ml_io_unmap(p->queues, HFI_DMA_SIZE);
	}
	if (p->backing) {
		free_backing(p->backing, kept_hfi_backing, HFI_DMA_SIZE);
	}
	if (p->metadata) {
		kmem_free(kernel_map, p->metadata, PAGE_SIZE);
	}
	ok = restore_power(p) && ok;
	if (p->regs) {
		ml_io_unmap(p->regs, IRIS_REG_SIZE);
	}
	if (p->cc) {
		ml_io_unmap(p->cc, 0x10000);
	}
	if (p->gcc) {
		ml_io_unmap(p->gcc - (GCC_VIDEO_PA & PAGE_MASK), PAGE_SIZE);
	}
	return ok;
}

static int
run_probe(unsigned level, struct hakios_iris_still *still)
{
	if (level == 5) {
		int result = probe_table_assignment();
		iris_log("iris_probe: level 5 %s (secure IOMMU table ownership, teardown %s)\n",
		    result == 1 ? "PASS" : "FAIL", result == -2 ? "FAILED" : "OK");
		return result;
	}
	bool grid = level >= 6;
	struct iris_probe p = {
		.grid = grid,
		.media_size = grid ? IRIS_GRID_MEDIA : IRIS_MEDIA_SIZE,
		.pixel_size = grid ? IRIS_GRID_PIXELS : IRIS_PIXEL_SIZE,
		.still = still,
	};
	iris_log("iris_probe: level %u starting, RAM %#lx + %#lx\n", level, gPhysBase, gPhysSize);
	const char *step = "RPMh power and bandwidth";
	bool ok = hakios_cam_power_vote(&p.power, HAKIOS_RPMH_IRIS, true);
	if (ok) {
		step = "map codec registers";
		p.gcc = ml_io_map_unmappable(trunc_page(GCC_VIDEO_PA), PAGE_SIZE, VM_WIMG_IO);
		if (p.gcc) {
			p.gcc += GCC_VIDEO_PA & PAGE_MASK;
		}
		p.cc = ml_io_map_unmappable(VIDEO_CC_PA, 0x10000, VM_WIMG_IO);
		p.regs = ml_io_map_unmappable(IRIS_PA, IRIS_REG_SIZE, VM_WIMG_IO);
		ok = p.gcc && p.cc && p.regs && power_up(&p, level, &step);
	}
	bool encode = level == 4 || grid;
	if (ok && level >= 2) {
		step = "HFI queues and SMMU";
		ok = setup_queues(&p, encode) && load_firmware(&p, &step);
		if (ok) {
			step = "codec control boot";
			ok = scm("VIDEO_PROTECT", SCM_VIDEO_PROTECT, 4, 0, HFI_IOVA, 0x01000000, 0x24800000) &&
			    boot_codec(&p);
		}
		if (ok) {
			step = "HFI INIT";
			ok = hfi_send(&p, hfi_init_message, sizeof(hfi_init_message)) &&
			    hfi_wait(&p, 0, HFI_CMD_INIT, NULL);
		}
		if (ok) {
			step = "HFI IMAGE_VERSION";
			ok = hfi_send(&p, hfi_version_message, sizeof(hfi_version_message)) &&
			    hfi_wait(&p, 0, HFI_IMAGE_VERSION, NULL);
		}
		if (ok && level >= 3) {
			p.command_header = 2;
			p.command_packet = 9;
			if (encode) {
				ok = resume_idle_codec(&p, &step);
				if (ok) {
					step = "HFI firmware debug level";
					ok = hfi_u32_command(&p, 0, HFI_DEBUG_LEVEL, 0,
					    HFI_PORT_NONE, HFI_PAYLOAD_ENUM, 0x3f, 0);
				}
			}
			if (ok) {
				ok = probe_encoder_session(&p, encode, &step);
			}
			if (!ok && encode) {
				log_firmware_debug(&p);
			}
		}
	}
	bool restored = cleanup(&p);
	int result = restored ? (ok ? 1 : -1) : -2;
	iris_log("iris_probe: level %u %s (%s, teardown %s)\n", level,
	    result == 1 ? "PASS" : "FAIL", ok ? "checkpoint reached" : step, restored ? "OK" : "FAILED");
	return result;
}

static void
probe_thread(void *arg, __unused wait_result_t wait)
{
	iris_result = run_probe((unsigned)(uintptr_t)arg, NULL);
	thread_terminate(current_thread());
}

static LCK_GRP_DECLARE(iris_group, "Iris2");
static LCK_MTX_DECLARE(iris_lock, &iris_group);

static int
iris_ioctl(__unused dev_t device, u_long cmd, caddr_t data, __unused int flags, __unused struct proc *proc)
{
	if (cmd != HAKIOS_IRIS_ENCODE_STILL) {
		return ENOTTY;
	}
	struct hakios_iris_still *still = (void *)data;
	if (!still->stride) {
		still->stride = HAKIOS_IRIS_WIDTH;
	}
	if (!still->stride_uv) {
		still->stride_uv = HAKIOS_IRIS_WIDTH;
	}
	if (!still->input_uv && still->input <= UINT64_MAX - (uint64_t)still->stride * HAKIOS_IRIS_HEIGHT) {
		still->input_uv = still->input + (uint64_t)still->stride * HAKIOS_IRIS_HEIGHT;
	}
	const uint64_t luma = (uint64_t)still->stride * (HAKIOS_IRIS_HEIGHT - 1) + HAKIOS_IRIS_WIDTH;
	const uint64_t chroma = (uint64_t)still->stride_uv * (HAKIOS_IRIS_HEIGHT / 2 - 1) + HAKIOS_IRIS_WIDTH;
	if (!still->input || still->input > UINT64_MAX - luma ||
	    !still->input_uv || still->input_uv > UINT64_MAX - chroma ||
	    still->stride < HAKIOS_IRIS_WIDTH || still->stride > HAKIOS_IRIS_MAX_STRIDE ||
	    still->stride_uv < HAKIOS_IRIS_WIDTH || still->stride_uv > HAKIOS_IRIS_MAX_STRIDE ||
	    (still->color_info >> 30) || still->reserved ||
	    !still->capacity || still->capacity > HAKIOS_IRIS_OUTPUT_MAX ||
	    !still->output || still->output > UINT64_MAX - still->capacity) {
		return EINVAL;
	}
	if (!lck_mtx_try_lock(&iris_lock)) {
		return EBUSY;
	}
	/* After a teardown that kept memory or power, leave the codec alone. */
	int error = EIO;
	if (!parse_firmware()) {
		error = ENOENT;
	} else if (iris_result != -2) {
		still->bytes = 0;
		still->elapsed_us = 0;
		memset(still->packet_bytes, 0, sizeof(still->packet_bytes));
		uint64_t started = mach_absolute_time(), ns;
		iris_result = run_probe(7, still);
		error = iris_result == 1 ? 0 : EIO;
		absolutetime_to_nanoseconds(mach_absolute_time() - started, &ns);
		iris_log("iris_probe: request %s in %llu us: %llu us encoding, the rest power, firmware, copy and teardown\n",
		    error ? "failed" : "done", ns / 1000, (unsigned long long)still->elapsed_us);
	}
	lck_mtx_unlock(&iris_lock);
	return error;
}

static const struct cdevsw iris_cdevsw = {
	.d_open = nulldev, .d_close = nulldev, .d_read = eno_rdwrt, .d_write = eno_rdwrt,
	.d_ioctl = iris_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0,
};

void
hakios_iris_probe_init(uint32_t level)
{
	if (!hakios_cam_rails_ready() || level > 7) {
		iris_result = -1;
		iris_log("iris_probe: unavailable level %u\n", level);
		return;
	}
	if (level == 7) {
		/* the contiguous memory every photograph reuses, taken now (see contig_backing) */
		vm_offset_t unused;
		keep_backings = true;
		if (!contig_backing(&unused, &kept_hfi_backing, HFI_DMA_SIZE, "HFI queues") ||
		    !contig_backing(&unused, &kept_arp_backing, round_page(IRIS_ARP_SIZE), "secure ARP")) {
			iris_log("iris_probe: level 7 takes its contiguous memory per request instead\n");
		}
		int major = cdevsw_add(-1, &iris_cdevsw);
		if (major >= 0 && devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "hakios-iris")) {
			iris_log("iris_probe: level 7 serving /dev/hakios-iris\n");
			return;
		}
		if (major >= 0) {
			cdevsw_remove(major, &iris_cdevsw);
		}
		iris_result = -1;
		iris_log("iris_probe: could not create /dev/hakios-iris\n");
		return;
	}
	thread_t thread;
	if (kernel_thread_start(probe_thread, (void *)(uintptr_t)level, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		iris_result = -1;
		iris_log("iris_probe: could not start diagnostic thread\n");
	}
}
