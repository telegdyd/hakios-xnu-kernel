/*
 * The touch screen of the OnePlus 10 Pro (negroni): a Synaptics S3908 ("TouchComm" v1) at I2C address 0x4b,
 * as a source of touches for /dev/hakios-touch (sys/hakios.input.h). Only with boot-arg touchscreen=1.
 *
 * What this relies on, and what it does not do:
 *   power    none: the firmware's NegroniTouchDxe (Mu) has switched the panel's supplies on and leaves them on
 *   bus      the serial engine that serves the chip (QUP0 SE4) cannot be used: its FIFO is off and its DMA
 *            block is locked by the secure world. So the two wires, GPIO16 (SDA) and GPIO17 (SCL), are driven
 *            by hand, open drain, with the pull-ups of the TLMM
 *   chip     reset is GPIO20 (low = in reset), the attention line is GPIO21 (low = a message is waiting)
 *   report   a bit stream described by the chip's own "touch report config" (command 0x25), read once at start
 * No interrupt is used: a thread looks at the attention line every 10 ms.
 * With boot-arg touch_bus=2 the bytes go through the serial engine and the QUP's GPI DMA instead of the wires by hand
 * (see "the serial engine itself" below); touch_bus=1 only looks at that hardware and logs it.
 *
 * Written from slop's negroni-ios patch 0093 (bsd/dev/negroni_touch*), which took it from the OPLUS driver
 * (drivers/input/touchscreen/synaptics_tcm_oncell.c). The wires by hand: in use on the phone since 2026-11-08.
 * With touchscreen=1 and -v a line on the console says how it is doing every few seconds (s3908: ...).
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>
#include <mach/vm_param.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void loader_log_line(const char *line);

/*
 * Each line also into the loader's log, which the live USB log (usb-netlog.py) streams: the console is quiet once
 * user space runs, and a fault of the serial engine's DMA can reset the phone before the kernel's log is fetched.
 */
static int s3908_say(const char *format, ...) __printflike(1, 2);

static int
s3908_say(const char *format, ...)
{
	char line[400];
	va_list list;
	int length;

	va_start(list, format);
	length = vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	loader_log_line(line);
	printf("%s", line);
	return length;
}
#define printf s3908_say

/* ---- pins (TLMM: GPIO n has its configuration at n * 0x1000 and its level at +4) ------------------- */

#define TLMM_PINS_PHYS          0x0f110000UL    /* GPIO16 ... */
#define TLMM_PINS_SIZE          0x8000UL        /* ... GPIO23 */
#define PIN_SDA                 0x0000UL        /* GPIO16 */
#define PIN_SCL                 0x1000UL        /* GPIO17 */
#define PIN_RESET               0x4000UL        /* GPIO20 */
#define PIN_ATTN                0x5000UL        /* GPIO21 */
#define CFG_LINE                3u              /* GPIO function, 2 mA, pull up */
#define CFG_OE                  (1u << 9)
#define IO_OUT                  2u

#define S3908_ADDRESS           0x4b
#define PANEL_MAX_X             5760            /* the chip's coordinates (touchpanel,panel-coords) */
#define PANEL_MAX_Y             12864

static volatile uint8_t *pins;

static inline uint32_t
rd(unsigned long offset)
{
	return *(volatile uint32_t *)(pins + offset);
}

static inline void
wr(unsigned long offset, uint32_t value)
{
	*(volatile uint32_t *)(pins + offset) = value;
}

static void
delay_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

static bool
attention(void)
{
	return rd(PIN_ATTN + 4) & 1;    /* true = high = nothing waiting */
}

/* ---- I2C by hand: a line is let go (pulled high) or driven low, never driven high ----------------- */

static void
line(unsigned long pin, bool release)
{
	wr(pin + 4, 0);
	wr(pin, CFG_LINE | (release ? 0 : CFG_OE));
	(void)rd(pin);
}

static inline bool
sda_get(void)
{
	return rd(PIN_SDA + 4) & 1;
}

static inline bool
scl_get(void)
{
	return rd(PIN_SCL + 4) & 1;
}

/* SCL high, waiting for a chip that holds it low (stretching): 0, or -4 after 1000 tries */
static int
scl_high(void)
{
	line(PIN_SCL, true);
	for (unsigned n = 0; n < 1000; n++) {
		if (scl_get()) {
			delay_us(5);
			return 0;
		}
		delay_us(1);
	}
	return -4;
}

static void
scl_low(void)
{
	line(PIN_SCL, false);
	delay_us(5);
}

static int
i2c_stop(void)
{
	int rc;

	scl_low();
	line(PIN_SDA, false);
	delay_us(5);
	rc = scl_high();
	line(PIN_SDA, true);
	delay_us(5);
	return rc;
}

/* one byte out and the chip's answer: 0, -2 (not acknowledged) or -4 */
static int
i2c_byte_out(uint8_t value)
{
	int rc;

	for (unsigned i = 0; i < 8; i++) {
		line(PIN_SDA, (value & 0x80) != 0);
		delay_us(5);
		if ((rc = scl_high()) != 0) {
			return rc;
		}
		scl_low();
		value <<= 1;
	}
	line(PIN_SDA, true);
	delay_us(5);
	if ((rc = scl_high()) != 0) {
		return rc;
	}
	bool nack = sda_get();
	scl_low();
	return nack ? -2 : 0;
}

static int
i2c_start(uint8_t address, bool read)
{
	int rc;

	line(PIN_SDA, true);
	if ((rc = scl_high()) != 0) {
		return rc;
	}
	if (!sda_get()) {
		return -3;      /* the bus is held */
	}
	line(PIN_SDA, false);
	delay_us(5);
	scl_low();
	return i2c_byte_out((uint8_t)(address << 1 | (read ? 1 : 0)));
}

static int
bb_write(const uint8_t *buffer, uint32_t length)
{
	int rc = i2c_start(S3908_ADDRESS, false);

	for (uint32_t i = 0; rc == 0 && i < length; i++) {
		rc = i2c_byte_out(buffer[i]);
	}
	int stop = i2c_stop();
	return rc ? rc : stop;
}

static int
bb_read(uint8_t *buffer, uint32_t length)
{
	int rc = i2c_start(S3908_ADDRESS, true);

	for (uint32_t i = 0; rc == 0 && i < length; i++) {
		uint8_t value = 0;

		line(PIN_SDA, true);
		for (unsigned bit = 0; bit < 8; bit++) {
			if ((rc = scl_high()) != 0) {
				break;
			}
			value = (uint8_t)(value << 1 | (sda_get() ? 1 : 0));
			scl_low();
		}
		if (rc) {
			break;
		}
		buffer[i] = value;
		line(PIN_SDA, i + 1 == length);         /* acknowledge every byte but the last */
		delay_us(5);
		rc = scl_high();
		scl_low();
		line(PIN_SDA, true);
	}
	int stop = i2c_stop();
	return rc ? rc : stop;
}

/*
 * ---- the serial engine itself, through the QUP's GPI DMA (boot-arg touch_bus) -----------------------------------
 *
 * touch_bus=1  before anything else, a look at the hardware a real I2C would need, written to the log (s3908: bus ...):
 *              the serial engine, its clocks in the GCC, the pins, the GPI engine's channels 1..6, the SMMU. Nothing is
 *              written; the wires are then driven by hand as always.
 * touch_bus=2  the bytes go through the serial engine (QUP0 SE4), as Android's i2c-qcom-geni does it: the firmware has
 *              switched the engine's FIFO off (GENI_IF_DISABLE_RO), so only the QUP's GPI DMA can feed it. GPII 0, the
 *              one bound to SE 4 (qcom,static-gpii-mask 0x1), polled: a transfer is done when its completion is in the
 *              event ring. The GPI reaches memory through the application processors' SMMU as stream 0x5b6 (dt: iommus
 *              of qcom,gpi-dma@900000), which the hypervisor does not let through as it is, so it is given a context of
 *              ours that shows it our block at 0x100000, below 4 GB as Android's addresses are. Anything that fails:
 *              back to the wires by hand.
 *
 * From Linux v6.12: drivers/dma/qcom/gpi.c (rings, commands, the I2C TREs), drivers/i2c/busses/i2c-qcom-geni.c
 * (geni_i2c_gpi_xfer: a write is CONFIG0, GO(write), DMA on the out channel; a read is CONFIG0, GO(read, length) with
 * LINK on the out channel and DMA on the in channel), drivers/soc/qcom/qcom-geni-se.c (geni_se_select_gpi_mode),
 * drivers/clk/qcom/gcc-sm8450.c (the clocks), drivers/pinctrl/qcom/pinctrl-sm8450.c (GPIO16/17: function 1 = qup4).
 */

extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);
extern void FlushPoC_DcacheRegion(vm_offset_t va, size_t length);
extern lck_mtx_t hakios_smmu_lock;

#define SE4_PHYS                0x00990000UL    /* only its first page: the second is the engine's protected program */
#define QUP0_PHYS               0x009c0000UL    /* the wrapper of QUP0 */
#define GPII_PHYS(n)            (0x00910000UL + 0x4000UL * (n))         /* 0x900000 - ee offset 0x10000 + 0x20000 */
#define GCC_SE4_PAGE            0x00127000UL    /* wrap0 m/s ahb 0x27004/8, s4 branch 0x274dc, s4 RCG 0x274e4 */
#define GCC_CORE_PAGE           0x00133000UL    /* wrap0 core 0x33000, core 2x 0x3300c */
#define GCC_VOTE_PAGE           0x00162000UL    /* APCS vote 0x62008 */
#define APPS_SMMU_PHYS          0x15000000UL

/* the serial engine (geni-se.h) */
#define SE_GENI_STATUS          0x040
#define GENI_SER_M_CLK_CFG      0x048
#define GENI_IF_DISABLE_RO      0x064
#define GENI_FW_REVISION_RO     0x068
#define GENI_FW_S_REVISION_RO   0x06c
#define SE_GENI_CLK_SEL         0x07c
#define SE_GENI_DMA_MODE_EN     0x258
#define SE_GENI_M_IRQ_STATUS    0x610
#define SE_GENI_M_IRQ_EN        0x614
#define SE_GENI_M_IRQ_CLEAR     0x618
#define SE_GENI_S_IRQ_STATUS    0x640
#define SE_GENI_S_IRQ_CLEAR     0x648
#define SE_GENI_IOS             0x908
#define SE_GSI_EVENT_EN         0xe18
#define SE_IRQ_EN               0xe1c
#define SE_HW_PARAM_0           0xe24
#define SE_HW_PARAM_1           0xe28

/* the GCC (gcc-sm8450.c) */
#define GCC_M_AHB_CBCR          0x004
#define GCC_S_AHB_CBCR          0x008
#define GCC_S4_CBCR             0x4dc
#define GCC_S4_RCG              0x4e4           /* CMD; CFG +4, M +8, N +0xc, D +0x10, DFSR +0x14, PERF_DFSR(l) +0x1c+4l */
#define GCC_CORE_CBCR           0x000
#define GCC_CORE2X_CBCR         0x00c
#define GCC_VOTE                0x008
#define GCC_VOTE_BITS           (1u << 6 | 1u << 7 | 1u << 8 | 1u << 9 | 1u << 14)     /* m ahb, s ahb, core, core 2x, s4 */
#define CBCR_OFF                (1u << 31)

/* a GPII's registers, from GPII_PHYS(n) (gpi.c, less 0x20000 + 0x4000 n) */
#define G_CH(k)                 (0x0000u + 0x80u * (k))
#define G_EV                    0x1000u
#define G_CH_DB(k)              (0x2000u + 0x8u * (k))
#define G_EV_DB                 0x2100u
#define G_CH_CMD                0x3008u
#define G_EV_CMD                0x3010u
#define G_TYPE_IRQ              0x3080u
#define G_GLOB_IRQ_STTS         0x3100u
#define G_ERROR_LOG             0x3200u
#define G_TYPE_IRQ_MSK          0x3088u
#define G_SRC_CH_IRQ_MSK        0x3098u
#define G_SRC_EV_CH_IRQ_MSK     0x309cu
#define G_SRC_CH_IRQ_CLR        0x30a0u
#define G_SRC_EV_CH_IRQ_CLR     0x30a4u
#define G_SRC_IEOB_IRQ_MSK      0x30b8u
#define G_SRC_IEOB_IRQ_CLR      0x30c0u
#define G_GLOB_IRQ_EN           0x3108u
#define G_GLOB_IRQ_CLR          0x3110u
#define G_GPII_IRQ_EN           0x3120u
#define G_GPII_IRQ_CLR          0x3128u
#define G_INTSET                0x3180u
#define G_MSI_BASE_LSB          0x3188u
#define G_MSI_BASE_MSB          0x318cu
#define G_CNTXT_SCRATCH_0       0x3400u
#define G_CNTXT_SCRATCH_1       0x3404u
#define G_QOS(k)                (0x005cu + 0x80u * (k))
#define G_SCRATCH(k, i)         (0x0060u + 0x80u * (k) + 4u * (i))
#define CNTXT_0                 0x00
#define CNTXT_1_LENGTH          0x04
#define CNTXT_2_BASE_LSB        0x08
#define CNTXT_3_BASE_MSB        0x0c
#define CNTXT_4_RP_LSB          0x10
#define CNTXT_8_INT_MOD         0x20
#define CNTXT_10_MSI_LSB        0x28
#define CNTXT_11_MSI_MSB        0x2c
#define CNTXT_12_RP_UPD_LSB     0x30
#define CNTXT_13_RP_UPD_MSB     0x34
#define G_STATE(v)              (((v) >> 20) & 0xfu)
#define G_NOT_ALLOCATED         0
#define G_ALLOCATED             1
#define G_STARTED               2
#define G_STOPPED               3
#define G_CMD_ALLOCATE          0x00u
#define G_CMD_START             0x01u
#define G_CMD_STOP              0x02u
#define G_CMD_RESET             0x09u
#define G_CMD_DE_ALLOC          0x0au

#define EL                      16u
#define TRES                    64u             /* each channel's ring (CHAN_TRES) */
#define EVS                     (TRES << 2)     /* the event ring (ev-factor 2) */
#define TRE_CHAIN               (1u << 0)
#define TRE_IEOT                (1u << 9)
#define TRE_LINK                (1u << 11)
#define TRE_DMA                 (0x10u << 16)
#define TRE_GO                  (0x20u << 16)
#define TRE_CONFIG0             (0x22u << 16)
#define GO_WRITE                1u
#define GO_READ                 2u
#define EV_XFER_COMPLETE        0x22u
#define TCE_SUCCESS             1u
#define TCE_EOT                 2u
/* 400 kHz from 19.2 MHz (geni_i2c_clk_map): divider 2, high 5, low 12, cycle 24; TX and RX packing */
#define CONFIG0_WORD0           (12u | 5u << 8 | 24u << 16 | 1u << 24 | 1u << 25)
#define CONFIG0_WORD2           2u

/* our block of memory: the event ring, the two channels' rings, the bytes */
#define M_EV                    0x0000u
#define M_TX                    0x1000u
#define M_RX                    0x1400u
#define M_BUF                   0x2000u
#define M_SIZE                  0x4000u

#define GPI_STREAM              0x5b6u          /* dt: iommus of qcom,gpi-dma@900000: the rings */
#define QUP_STREAM              0x5a3u          /* dt: iommus of qcom,qupv3_0_geni_se@9c0000: the bytes */

/* the SMMU (as hakios.mpss.c) */
#define SMMU_SCR0               0x000
#define SMMU_IDR0               0x020
#define SMMU_IDR1               0x024
#define SMMU_SGFSR              0x048
#define SMMU_SMR(n)             (0x800 + 4 * (n))
#define SMMU_S2CR(n)            (0xc00 + 4 * (n))
#define SMMU_CBAR(n)            (4 * (n))
#define SMMU_CBAR_STAGE_1       (1u << 16 | 3u << 8 | 0xfu << 12)
#define SMMU_CBA2R(n)           (0x800 + 4 * (n))
#define SMMU_CB_SCTLR           0x00
#define SMMU_CB_TCR2            0x10
#define SMMU_CB_TTBR0           0x20
#define SMMU_CB_TTBR1           0x28
#define SMMU_CB_TCR             0x30
#define SMMU_CB_MAIR0           0x38
#define SMMU_CB_MAIR1           0x3c
#define SMMU_CB_FSR             0x58
#define SMMU_CB_FAR             0x60
#define SMMU_CB_FSYNR0          0x68
#define SMMU_CB_TLBIASID        0x610
#define SMMU_CB_TLBSYNC         0x7f0
#define SMMU_CB_TLBSTATUS       0x7f4
#define SMMU_SCTLR_ON           (1u << 0 | 1u << 1 | 1u << 2 | 1u << 5 | 1u << 8 | 1u << 12)
#define SMMU_TCR_39_BITS        (25u | 1u << 8 | 1u << 10 | 3u << 12 | 1u << 23)
#define SMMU_TCR2               (1u | 1u << 4 | 7u << 15)
#define SMMU_MAIR0              0xf404ff44      /* 0 uncached, 1 cached, 2 device */
#define SMMU_FSR_ALL            0xc00001fe
#define SMMU_PAGE               (3ull | 0ull << 2 | 1ull << 6 | 2ull << 8 | 1ull << 10 | 1ull << 11)  /* 4K, uncached, RW */
#define GPI_IO                  0x00100000u     /* where the GPI sees our block */

static uint32_t touch_bus;      /* boot-arg touch_bus: 0 wires by hand, 1 look first, 2 the serial engine */

static inline uint32_t
r32(volatile uint8_t *base, uint32_t offset)
{
	return *(volatile uint32_t *)(base + offset);
}

static inline void
w32(volatile uint8_t *base, uint32_t offset, uint32_t value)
{
	*(volatile uint32_t *)(base + offset) = value;
}

static inline void
barrier(void)
{
	__builtin_arm_dsb(0xf);
}

static volatile uint8_t *
map(vm_offset_t phys, vm_size_t size)
{
	volatile uint8_t *at = (volatile uint8_t *)ml_io_map(phys, size);

	if (at == NULL) {
		printf("s3908: bus: cannot map %#lx\n", (unsigned long)phys);
	}
	return at;
}

static struct {
	volatile uint8_t *se, *gcc, *gcc_core, *gcc_vote, *smmu, *smmu_second, *smmu_context, *gpii;
	volatile uint8_t *mem;                  /* our block, uncached */
	uint64_t pa;
	vm_offset_t tables;
	uint32_t n, tx_wp, rx_wp, ev_rp;
	uint32_t match, context, old_smr, old_s2cr, old_sctlr, page, pages;
	uint32_t match_qup, old_smr_qup, old_s2cr_qup;
	bool smmu_on, on;
	uint32_t last_ev[4], last_error, restarts;
} gpi;

/* ---- touch_bus=1: what is there ---- */

static void
bus_look(void)
{
	volatile uint8_t *qup = map(QUP0_PHYS, 0x1000), *se = map(SE4_PHYS, 0x1000);
	volatile uint8_t *gcc = map(GCC_SE4_PAGE, 0x1000), *core = map(GCC_CORE_PAGE, 0x1000), *vote = map(GCC_VOTE_PAGE, 0x1000);
	volatile uint8_t *smmu = map(APPS_SMMU_PHYS, 0x1000);

	if (qup == NULL || se == NULL || gcc == NULL || core == NULL || vote == NULL || smmu == NULL) {
		return;
	}
	printf("s3908: bus: qup0 hw version %#x\n", r32(qup, 0x4));
	printf("s3908: bus: gcc vote %#x (wanted bits %#x), m ahb %#x, s ahb %#x, core %#x, core 2x %#x, s4 %#x\n",
	    r32(vote, GCC_VOTE), GCC_VOTE_BITS, r32(gcc, GCC_M_AHB_CBCR), r32(gcc, GCC_S_AHB_CBCR), r32(core, GCC_CORE_CBCR),
	    r32(core, GCC_CORE2X_CBCR), r32(gcc, GCC_S4_CBCR));
	printf("s3908: bus: s4 rcg cmd %#x cfg %#x m %#x n %#x d %#x dfsr %#x\n", r32(gcc, GCC_S4_RCG), r32(gcc, GCC_S4_RCG + 4),
	    r32(gcc, GCC_S4_RCG + 8), r32(gcc, GCC_S4_RCG + 0xc), r32(gcc, GCC_S4_RCG + 0x10), r32(gcc, GCC_S4_RCG + 0x14));
	for (unsigned l = 0; l < 8; l++) {
		printf("s3908: bus: s4 rcg level %u: perf %#x m %#x n %#x\n", l, r32(gcc, GCC_S4_RCG + 0x1c + 4 * l),
		    r32(gcc, GCC_S4_RCG + 0x5c + 4 * l), r32(gcc, GCC_S4_RCG + 0x9c + 4 * l));
	}
	printf("s3908: bus: se4 fw %#x s fw %#x if-disable %#x status %#x m-clk %#x clk-sel %#x dma-mode %#x\n",
	    r32(se, GENI_FW_REVISION_RO), r32(se, GENI_FW_S_REVISION_RO), r32(se, GENI_IF_DISABLE_RO), r32(se, SE_GENI_STATUS),
	    r32(se, GENI_SER_M_CLK_CFG), r32(se, SE_GENI_CLK_SEL), r32(se, SE_GENI_DMA_MODE_EN));
	printf("s3908: bus: se4 gsi-event %#x irq-en %#x hw0 %#x hw1 %#x m-irq %#x/%#x s-irq %#x ios %#x\n",
	    r32(se, SE_GSI_EVENT_EN), r32(se, SE_IRQ_EN), r32(se, SE_HW_PARAM_0), r32(se, SE_HW_PARAM_1),
	    r32(se, SE_GENI_M_IRQ_STATUS), r32(se, SE_GENI_M_IRQ_EN), r32(se, SE_GENI_S_IRQ_STATUS), r32(se, SE_GENI_IOS));
	printf("s3908: bus: pins: gpio16 %#x/%#x gpio17 %#x/%#x gpio20 %#x/%#x gpio21 %#x/%#x\n", rd(PIN_SDA), rd(PIN_SDA + 4),
	    rd(PIN_SCL), rd(PIN_SCL + 4), rd(PIN_RESET), rd(PIN_RESET + 4), rd(PIN_ATTN), rd(PIN_ATTN + 4));

	uint32_t idr0 = r32(smmu, SMMU_IDR0), idr1 = r32(smmu, SMMU_IDR1), matches = idr0 & 0xff, free = 0;
	printf("s3908: bus: smmu scr0 %#x idr0 %#x idr1 %#x\n", r32(smmu, SMMU_SCR0), idr0, idr1);
	for (uint32_t m = 0; m < matches && SMMU_SMR(m) < 0xc00; m++) {
		uint32_t smr = r32(smmu, SMMU_SMR(m)), id = smr & 0x7fff, mask = (smr >> 16) & 0x7fff;

		if (!(smr >> 31)) {
			free++;
		} else if (((id ^ GPI_STREAM) & ~mask & 0x7fff) == 0 || ((id ^ 0x5a3u) & ~mask & 0x7fff) == 0) {
			printf("s3908: bus: smmu match %u: smr %#x s2cr %#x\n", m, smr, r32(smmu, SMMU_S2CR(m)));
		}
	}
	printf("s3908: bus: smmu: %u match registers free; none above matches stream %#x unless printed\n", free, GPI_STREAM);

	for (uint32_t n = 1; n <= 6; n++) {
		volatile uint8_t *g;

		printf("s3908: bus: gpii %u: mapping %#lx\n", n, GPII_PHYS(n));
		g = map(GPII_PHYS(n), 0x4000);
		if (g == NULL) {
			continue;
		}
		printf("s3908: bus: gpii %u: ch0 %#x ch1 %#x ev %#x scratch %#x %#x, type irq %#x, glob %#x, error %#x\n", n,
		    r32(g, G_CH(0) + CNTXT_0), r32(g, G_CH(1) + CNTXT_0), r32(g, G_EV + CNTXT_0), r32(g, G_SCRATCH(0, 0)),
		    r32(g, G_SCRATCH(1, 0)), r32(g, G_TYPE_IRQ), r32(g, G_GLOB_IRQ_STTS), r32(g, G_ERROR_LOG));
	}
	printf("s3908: bus: look done\n");
}

/*
 * ---- touch_bus=2: the buses on the QUP's way to memory (RPMh BCM votes) ----
 *
 * The device tree's interconnects of qupv3_0_geni_se: qup-core, snoc-llcc, qup-ddr. In Linux's sm8450 interconnect
 * driver the QUP0 master (qhm_qup0) is on aggre2_noc, which reaches the system NoC through qnm_aggre2_noc (BCM SN3),
 * then the gem NoC through qns_gemnoc_sf (BCM SN0); its core clock is BCM QUP0. Memory itself (SH0, MC0) is on for
 * the processors already and is not touched. Through DRV 2's TCS 0, as hakios.adsp.c does on this phone.
 */
#define CMD_DB_PHYS             0x80860000UL
#define CMD_DB_SIZE             0x20000UL
#define RSC_PHYS_8450           0x17a20000UL    /* dt: rsc@17a00000, drv-id 2 */
#define RSC_TCS_BASE            0xd00
#define RSC_IRQ_STATUS          (RSC_TCS_BASE + 0x04)
#define RSC_IRQ_CLEAR           (RSC_TCS_BASE + 0x08)
#define RSC_TCS0                RSC_TCS_BASE
#define TCS_WAIT_FOR_DONE       0x10
#define TCS_CONTROL             0x14
#define TCS_STATUS              0x18            /* not zero: idle */
#define TCS_ENABLED             0x1c
#define TCS_COMMAND0            0x30            /* message id, address, value, status */
#define TCS_ACTIVE_MODE         (1u << 16)
#define TCS_TRIGGER             (1u << 24)
#define MSGID_WRITE_WAITED      (8u | 1u << 8 | 1u << 16)
#define BCM_VOTE(y)             (1u << 30 | 1u << 29 | ((y) & 0x3fffu))   /* commit, valid, peak vote */

extern lck_mtx_t hakios_rsc_lock;
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, unsigned int flags);

static uint32_t
cmd_db_address(const char *name)
{
	static const uint8_t *db;
	char id[8] = { 0 };

	if (db == NULL) {
		db = (const uint8_t *)ml_io_map_wcomb(CMD_DB_PHYS, CMD_DB_SIZE);       /* memory, not registers */
	}
	if (db == NULL || !(db[4] == 0xdb && db[5] == 0x30 && db[6] == 0x03 && db[7] == 0x0c)) {
		printf("s3908: gpi: no command database\n");
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const uint8_t *slave = db + 8 + 16 * s;                 /* id, header offset, data offset, count, ... */
		uint16_t slave_id = slave[0] | slave[1] << 8, header = slave[2] | slave[3] << 8, count = slave[6] | slave[7] << 8;
		const uint8_t *data = db + 8 + 16 * 8 + 8;               /* after the slaves, the checksum and a word */

		if (slave_id == 0 || count > 512) {
			continue;
		}
		for (unsigned i = 0; i < count; i++) {
			const uint8_t *entry = data + header + 24 * i;  /* id[8], priority[2], address, length, offset */

			if (memcmp(entry, id, sizeof(id)) == 0) {
				return entry[16] | entry[17] << 8 | entry[18] << 16 | (uint32_t)entry[19] << 24;
			}
		}
	}
	printf("s3908: gpi: the command database has no \"%s\"\n", name);
	return 0;
}

static bool
bcm_vote(const char *name, uint32_t vote)
{
	static volatile uint8_t *rsc;
	uint32_t address = cmd_db_address(name), done = 0;

	if (address == 0) {
		return false;
	}
	if (rsc == NULL && (rsc = map(RSC_PHYS_8450, 0x10000)) == NULL) {
		return false;
	}
	lck_mtx_lock(&hakios_rsc_lock);
	if (r32(rsc, RSC_TCS0 + TCS_STATUS) == 0) {
		lck_mtx_unlock(&hakios_rsc_lock);
		printf("s3908: gpi: RPMh's TCS 0 is busy: %s not asked\n", name);
		return false;
	}
	w32(rsc, RSC_IRQ_CLEAR, 1);
	w32(rsc, RSC_TCS0 + TCS_COMMAND0, MSGID_WRITE_WAITED);
	w32(rsc, RSC_TCS0 + TCS_COMMAND0 + 4, address);
	w32(rsc, RSC_TCS0 + TCS_COMMAND0 + 8, BCM_VOTE(vote));
	w32(rsc, RSC_TCS0 + TCS_WAIT_FOR_DONE, 1);
	w32(rsc, RSC_TCS0 + TCS_ENABLED, 1);
	barrier();
	w32(rsc, RSC_TCS0 + TCS_CONTROL, r32(rsc, RSC_TCS0 + TCS_CONTROL) & ~TCS_TRIGGER);
	w32(rsc, RSC_TCS0 + TCS_CONTROL, r32(rsc, RSC_TCS0 + TCS_CONTROL) | TCS_ACTIVE_MODE);
	w32(rsc, RSC_TCS0 + TCS_CONTROL, r32(rsc, RSC_TCS0 + TCS_CONTROL) | TCS_TRIGGER);
	for (unsigned i = 0; i < 2000 && !(done = r32(rsc, RSC_IRQ_STATUS) & 1); i++) {
		delay_us(50);
	}
	w32(rsc, RSC_IRQ_CLEAR, 1);
	w32(rsc, RSC_TCS0 + TCS_CONTROL, r32(rsc, RSC_TCS0 + TCS_CONTROL) & ~(TCS_TRIGGER | TCS_ACTIVE_MODE));
	w32(rsc, RSC_TCS0 + TCS_ENABLED, 0);
	lck_mtx_unlock(&hakios_rsc_lock);
	printf("s3908: gpi: RPMh: %s (%#x) = %#x: %s\n", name, address, BCM_VOTE(vote), done ? "done" : "NOT DONE");
	return done != 0;
}

/* ---- touch_bus=2: the SMMU context for the GPI's stream ---- */

static void
gpi_smmu_faults(const char *when)
{
	uint32_t status;

	if (!gpi.smmu_on) {
		return;
	}
	status = r32(gpi.smmu_context, SMMU_CB_FSR);
	if ((status & SMMU_FSR_ALL) == 0) {
		return;
	}
	printf("s3908: gpi: %s: THE SMMU REFUSED THE GPI: status %#x address %#llx syndrome %#x\n", when, status,
	    (uint64_t)r32(gpi.smmu_context, SMMU_CB_FAR + 4) << 32 | r32(gpi.smmu_context, SMMU_CB_FAR),
	    r32(gpi.smmu_context, SMMU_CB_FSYNR0));
	w32(gpi.smmu_context, SMMU_CB_FSR, SMMU_FSR_ALL);
}

static bool
gpi_smmu_on(void)
{
	bool used[256] = { false };
	uint32_t idr0, idr1, matches, contexts, sees = 1u << 31 | GPI_STREAM;
	uint64_t *root, ttbr;

	if (gpi.smmu_on) {
		return true;
	}
	lck_mtx_lock(&hakios_smmu_lock);
	idr0 = r32(gpi.smmu, SMMU_IDR0);
	idr1 = r32(gpi.smmu, SMMU_IDR1);
	matches = idr0 & 0xff;
	contexts = idr1 & 0xff;
	gpi.page = idr1 >> 31 ? 0x10000 : 0x1000;
	gpi.pages = 2u << (idr1 >> 28 & 7);
	if ((r32(gpi.smmu, SMMU_SCR0) & 1u << 3) || gpi.page != 0x1000 || 2 * gpi.pages * gpi.page != 0x100000) {
		lck_mtx_unlock(&hakios_smmu_lock);
		printf("s3908: gpi: the SMMU is not of the shape this knows (idr1 %#x)\n", idr1);
		return false;
	}
	gpi.match = gpi.match_qup = ~0u;
	for (uint32_t m = 0; m < matches && SMMU_SMR(m) < 0xc00; m++) {
		uint32_t smr = r32(gpi.smmu, SMMU_SMR(m)), s2cr = r32(gpi.smmu, SMMU_S2CR(m));

		if (smr >> 31) {
			used[s2cr & 0xff] = true;
			if (((smr ^ GPI_STREAM) & ~(smr >> 16) & 0x7fff) == 0 || ((smr ^ QUP_STREAM) & ~(smr >> 16) & 0x7fff) == 0) {
				lck_mtx_unlock(&hakios_smmu_lock);
				printf("s3908: gpi: stream %#x or %#x is already matched (register %u: smr %#x s2cr %#x): left alone\n",
				    GPI_STREAM, QUP_STREAM, m, smr, s2cr);
				return false;
			}
		} else if (gpi.match == ~0u) {
			gpi.match = m;
			gpi.old_smr = smr;
			gpi.old_s2cr = s2cr;
		} else if (gpi.match_qup == ~0u) {
			gpi.match_qup = m;
			gpi.old_smr_qup = smr;
			gpi.old_s2cr_qup = s2cr;
		}
	}
	for (gpi.context = 0; gpi.context < contexts && used[gpi.context]; gpi.context++) {
	}
	if (gpi.match == ~0u || gpi.match_qup == ~0u || gpi.context >= contexts) {
		lck_mtx_unlock(&hakios_smmu_lock);
		printf("s3908: gpi: the SMMU has no match register or no context left\n");
		return false;
	}
	gpi.smmu_second = map(APPS_SMMU_PHYS + gpi.page, 0x1000);
	gpi.smmu_context = map(APPS_SMMU_PHYS + (gpi.pages + gpi.context) * gpi.page, 0x1000);
	if (gpi.smmu_second == NULL || gpi.smmu_context == NULL) {
		lck_mtx_unlock(&hakios_smmu_lock);
		return false;
	}
	gpi.old_sctlr = r32(gpi.smmu_context, SMMU_CB_SCTLR);
	if (gpi.old_sctlr & 1) {
		lck_mtx_unlock(&hakios_smmu_lock);
		printf("s3908: gpi: context %u of the SMMU is on although no stream uses it: left alone\n", gpi.context);
		return false;
	}
	if (gpi.tables == 0 &&
	    kmem_alloc(kernel_map, &gpi.tables, 3 * 4096, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		lck_mtx_unlock(&hakios_smmu_lock);
		return false;
	}
	/* our block's pages at GPI_IO, below 4 GB as Android gives them (dt: qcom,iommu-dma-addr-pool 0x100000) */
	root = (uint64_t *)gpi.tables;
	uint64_t *level2 = (uint64_t *)(gpi.tables + 4096), *level3 = (uint64_t *)(gpi.tables + 8192);
	bzero(root, 3 * 4096);
	root[GPI_IO >> 30 & 0x1ff] = kvtophys((vm_offset_t)level2) | 3;
	level2[GPI_IO >> 21 & 0x1ff] = kvtophys((vm_offset_t)level3) | 3;
	for (uint32_t page = 0; page < M_SIZE / 4096; page++) {
		level3[(GPI_IO >> 12 & 0x1ff) + page] = (gpi.pa + 4096ull * page) | SMMU_PAGE;
	}
	FlushPoC_DcacheRegion(gpi.tables, 3 * 4096);
	barrier();
	ttbr = kvtophys(gpi.tables) | (uint64_t)gpi.context << 48;
	printf("s3908: gpi: THE SMMU IS ABOUT TO BE WRITTEN: context %u (root %#llx: %#x -> %#llx, %u pages), match register %u "
	    "for stream %#x (before: smr %#x s2cr %#x)\n", gpi.context, ttbr & 0x0000fffffffff000ull, GPI_IO, gpi.pa,
	    M_SIZE / 4096, gpi.match, GPI_STREAM, gpi.old_smr, gpi.old_s2cr);
	w32(gpi.smmu_context, SMMU_CB_SCTLR, 0);
	w32(gpi.smmu_second, SMMU_CBA2R(gpi.context), 1);
	w32(gpi.smmu_second, SMMU_CBAR(gpi.context), SMMU_CBAR_STAGE_1);
	w32(gpi.smmu_context, SMMU_CB_TCR2, SMMU_TCR2);
	w32(gpi.smmu_context, SMMU_CB_TCR, SMMU_TCR_39_BITS);
	w32(gpi.smmu_context, SMMU_CB_TTBR0, (uint32_t)ttbr);
	w32(gpi.smmu_context, SMMU_CB_TTBR0 + 4, (uint32_t)(ttbr >> 32));
	w32(gpi.smmu_context, SMMU_CB_TTBR1, 0);
	w32(gpi.smmu_context, SMMU_CB_TTBR1 + 4, 0);
	w32(gpi.smmu_context, SMMU_CB_MAIR0, SMMU_MAIR0);
	w32(gpi.smmu_context, SMMU_CB_MAIR1, 0);
	w32(gpi.smmu_context, SMMU_CB_FSR, SMMU_FSR_ALL);
	w32(gpi.smmu_context, SMMU_CB_TLBIASID, gpi.context);
	w32(gpi.smmu_context, SMMU_CB_TLBSYNC, 1);
	for (unsigned ms = 0; r32(gpi.smmu_context, SMMU_CB_TLBSTATUS) & 1; ms++) {
		if (ms == 100) {
			lck_mtx_unlock(&hakios_smmu_lock);
			printf("s3908: gpi: the SMMU does not finish forgetting old translations\n");
			return false;
		}
		sleep_ms(1);
	}
	w32(gpi.smmu_context, SMMU_CB_SCTLR, SMMU_SCTLR_ON);
	w32(gpi.smmu, SMMU_S2CR(gpi.match), gpi.context);
	w32(gpi.smmu, SMMU_SMR(gpi.match), sees);
	/* the wrapper's stream too: the serial engine fetches and stores the bytes as the QUP (Android maps them for wrapper_dev) */
	w32(gpi.smmu, SMMU_S2CR(gpi.match_qup), gpi.context);
	w32(gpi.smmu, SMMU_SMR(gpi.match_qup), 1u << 31 | QUP_STREAM);
	gpi.smmu_on = true;
	if (r32(gpi.smmu, SMMU_SMR(gpi.match)) != sees || (r32(gpi.smmu, SMMU_S2CR(gpi.match)) & 0x300ff) != gpi.context ||
	    r32(gpi.smmu, SMMU_SMR(gpi.match_qup)) != (1u << 31 | QUP_STREAM) ||
	    (r32(gpi.smmu, SMMU_S2CR(gpi.match_qup)) & 0x300ff) != gpi.context || !(r32(gpi.smmu_context, SMMU_CB_SCTLR) & 1)) {
		printf("s3908: gpi: THE SMMU DID NOT TAKE IT: smr %#x s2cr %#x, smr %#x s2cr %#x, sctlr %#x\n",
		    r32(gpi.smmu, SMMU_SMR(gpi.match)), r32(gpi.smmu, SMMU_S2CR(gpi.match)), r32(gpi.smmu, SMMU_SMR(gpi.match_qup)),
		    r32(gpi.smmu, SMMU_S2CR(gpi.match_qup)), r32(gpi.smmu_context, SMMU_CB_SCTLR));
		w32(gpi.smmu, SMMU_SMR(gpi.match_qup), gpi.old_smr_qup);
		w32(gpi.smmu, SMMU_S2CR(gpi.match_qup), gpi.old_s2cr_qup);
		w32(gpi.smmu, SMMU_SMR(gpi.match), gpi.old_smr);
		w32(gpi.smmu, SMMU_S2CR(gpi.match), gpi.old_s2cr);
		w32(gpi.smmu_context, SMMU_CB_SCTLR, gpi.old_sctlr);
		gpi.smmu_on = false;
		lck_mtx_unlock(&hakios_smmu_lock);
		return false;
	}
	lck_mtx_unlock(&hakios_smmu_lock);
	printf("s3908: gpi: streams %#x and %#x go through context %u of the SMMU (match registers %u and %u)\n", GPI_STREAM,
	    QUP_STREAM, gpi.context, gpi.match, gpi.match_qup);
	return true;
}

/* ---- touch_bus=2: the GPII ---- */

static int
gpi_state(uint32_t cntxt, uint32_t want)
{
	for (unsigned i = 0; i < 2500; i++) {          /* Linux waits 250 ms for a command */
		if (G_STATE(r32(gpi.gpii, cntxt + CNTXT_0)) == want) {
			return 0;
		}
		delay_us(100);
	}
	return -11;
}

static int
gpi_ch_cmd(uint32_t k, uint32_t cmd, uint32_t want)
{
	w32(gpi.gpii, G_CH_CMD, cmd << 24 | k);
	return gpi_state(G_CH(k), want);
}

static int
gpi_ev_cmd(uint32_t cmd, uint32_t want)
{
	w32(gpi.gpii, G_EV_CMD, cmd << 24);
	return gpi_state(G_EV, want);
}

/* whatever is on the GPII (ours from before a restart, or another's): stopped, reset, given back */
static void
gpi_release(void)
{
	for (uint32_t k = 0; k < 2; k++) {
		uint32_t st = G_STATE(r32(gpi.gpii, G_CH(k) + CNTXT_0));

		if (st == G_STARTED) {
			gpi_ch_cmd(k, G_CMD_STOP, G_STOPPED);
		}
		if (st != G_NOT_ALLOCATED) {
			gpi_ch_cmd(k, G_CMD_RESET, G_ALLOCATED);
			gpi_ch_cmd(k, G_CMD_DE_ALLOC, G_NOT_ALLOCATED);
		}
	}
	if (G_STATE(r32(gpi.gpii, G_EV + CNTXT_0)) != G_NOT_ALLOCATED) {
		gpi_ev_cmd(G_CMD_RESET, G_ALLOCATED);
		gpi_ev_cmd(G_CMD_DE_ALLOC, G_NOT_ALLOCATED);
	}
}

static void
ring(uint32_t cntxt, uint32_t config, uint32_t length, uint64_t pa)
{
	w32(gpi.gpii, cntxt + CNTXT_0, config);
	w32(gpi.gpii, cntxt + CNTXT_1_LENGTH, length);
	w32(gpi.gpii, cntxt + CNTXT_2_BASE_LSB, (uint32_t)pa);
	w32(gpi.gpii, cntxt + CNTXT_3_BASE_MSB, (uint32_t)(pa >> 32));
}

static void gpi_sync(void);

/* the event ring and the two channels (gpi_ch_init): 0 or < 0 */
static int
gpi_start(void)
{
	int rc;

	gpi_release();
	for (uint32_t i = 0; i < M_SIZE; i += 4) {
		w32(gpi.mem, i, 0);
	}
	gpi_sync();
	/*
	 * gpi_config_interrupts(): the GPII signals by a wire (INTSET 1), not by a write to the MSI address -- left as it
	 * was, a random one, which the SMMU refused (seen 2026-11-11: writes of stream 0x5b6 to 0x356456c81). The
	 * interrupts themselves stay masked: this polls, and nothing in this kernel takes the GPII's line.
	 */
	w32(gpi.gpii, G_TYPE_IRQ_MSK, 0);
	w32(gpi.gpii, G_SRC_IEOB_IRQ_MSK, 0);
	w32(gpi.gpii, G_SRC_CH_IRQ_MSK, 0);
	w32(gpi.gpii, G_SRC_EV_CH_IRQ_MSK, 0);
	w32(gpi.gpii, G_GLOB_IRQ_EN, 0);
	w32(gpi.gpii, G_GPII_IRQ_EN, 0);
	w32(gpi.gpii, G_MSI_BASE_LSB, 0);
	w32(gpi.gpii, G_MSI_BASE_MSB, 0);
	w32(gpi.gpii, G_CNTXT_SCRATCH_0, 0);
	w32(gpi.gpii, G_CNTXT_SCRATCH_1, 0);
	w32(gpi.gpii, G_INTSET, 1);
	w32(gpi.gpii, G_ERROR_LOG, 0);
	w32(gpi.gpii, G_SRC_CH_IRQ_CLR, 0xffffffffu);
	w32(gpi.gpii, G_SRC_EV_CH_IRQ_CLR, 0xffffffffu);
	w32(gpi.gpii, G_SRC_IEOB_IRQ_CLR, 0xffffffffu);
	w32(gpi.gpii, G_GLOB_IRQ_CLR, 0xffffffffu);
	w32(gpi.gpii, G_GPII_IRQ_CLR, 0xffffffffu);
	barrier();
	if ((rc = gpi_ev_cmd(G_CMD_ALLOCATE, G_ALLOCATED)) != 0) {
		return rc;
	}
	ring(G_EV, EL << 24 | 1u << 16 | 2u, EVS * EL, GPI_IO + M_EV);    /* IRQ type, GPI event ring */
	w32(gpi.gpii, G_EV_DB + 4, (uint32_t)((GPI_IO + M_EV) >> 32));
	w32(gpi.gpii, G_EV + CNTXT_8_INT_MOD, 0);
	w32(gpi.gpii, G_EV + CNTXT_10_MSI_LSB, 0);
	w32(gpi.gpii, G_EV + CNTXT_11_MSI_MSB, 0);
	w32(gpi.gpii, G_EV + CNTXT_12_RP_UPD_LSB, 0);
	w32(gpi.gpii, G_EV + CNTXT_13_RP_UPD_MSB, 0);
	gpi.ev_rp = 0;
	barrier();
	w32(gpi.gpii, G_EV_DB, (uint32_t)(GPI_IO + M_EV + (EVS - 1) * EL));     /* all the ring the engine's */

	for (uint32_t k = 0; k < 2; k++) {
		uint64_t pa = GPI_IO + (k ? M_RX : M_TX);

		if ((rc = gpi_ch_cmd(k, G_CMD_ALLOCATE, G_ALLOCATED)) != 0) {
			return rc;
		}
		ring(G_CH(k), EL << 24 | (k ? 0u : 1u) << 3 | 2u, TRES * EL, pa);       /* out for 0, in for 1; event ring 0 */
		w32(gpi.gpii, G_CH_DB(k) + 4, (uint32_t)(pa >> 32));
		w32(gpi.gpii, G_SCRATCH(k, 0), (1u - k) << 16 | 3u << 4 | 4u);          /* pair, I2C, SE 4 */
		w32(gpi.gpii, G_SCRATCH(k, 1), 0);
		w32(gpi.gpii, G_SCRATCH(k, 2), 0);
		w32(gpi.gpii, G_SCRATCH(k, 3), 0);
		w32(gpi.gpii, G_QOS(k), 1);
	}
	gpi.tx_wp = gpi.rx_wp = 0;
	for (uint32_t k = 0; k < 2; k++) {
		if ((rc = gpi_ch_cmd(k, G_CMD_START, G_STARTED)) != 0) {
			return rc;
		}
	}
	return 0;
}

static void
put_tre(uint32_t k, uint32_t d0, uint32_t d1, uint32_t d2, uint32_t d3)
{
	uint32_t *wp = k ? &gpi.rx_wp : &gpi.tx_wp, at = (k ? M_RX : M_TX) + *wp * EL;

	w32(gpi.mem, at, d0);
	w32(gpi.mem, at + 4, d1);
	w32(gpi.mem, at + 8, d2);
	w32(gpi.mem, at + 12, d3);
	*wp = (*wp + 1) % TRES;
}

/* our uncached writes out before the engine reads, its writes seen after (a cached block, tried 2026-11-11, changed nothing) */
static void
gpi_sync(void)
{
	barrier();
}

static void
doorbell(uint32_t k)
{
	gpi_sync();
	w32(gpi.gpii, G_CH_DB(k), (uint32_t)(GPI_IO + (k ? M_RX : M_TX) + (k ? gpi.rx_wp : gpi.tx_wp) * EL));
}

/* the completion of channel k: 0, -20 (the engine said no), -21 (nothing came) */
static int
gpi_wait(uint32_t k)
{
	uint32_t ev = (uint32_t)(GPI_IO + M_EV);

	for (unsigned n = 0; n < 20000; n++) {          /* 20 ms, a 512-byte read takes ~13 ms at 400 kHz */
		uint32_t hw = ((r32(gpi.gpii, G_EV + CNTXT_4_RP_LSB) - ev) / EL) % EVS;

		if (gpi.ev_rp != hw) {
			gpi_sync();
		}
		while (gpi.ev_rp != hw) {
			uint32_t at = M_EV + gpi.ev_rp * EL, d2, d3;

			for (unsigned i = 0; i < 4; i++) {
				gpi.last_ev[i] = r32(gpi.mem, at + 4 * i);
			}
			d2 = gpi.last_ev[2];
			d3 = gpi.last_ev[3];
			gpi.ev_rp = (gpi.ev_rp + 1) % EVS;
			w32(gpi.gpii, G_EV_DB, ev + ((gpi.ev_rp + EVS - 1) % EVS) * EL);
			if (((d3 >> 16) & 0xff) != EV_XFER_COMPLETE || (d3 >> 24) != k) {
				continue;
			}
			if ((d2 >> 24) == TCE_EOT || (d2 >> 24) == TCE_SUCCESS) {
				return 0;
			}
			gpi.last_error = d2 >> 24 | (d3 & 0xffff) << 8;
			return -20;
		}
		delay_us(1);
	}
	return -21;
}

static void gpi_off(const char *why, int rc);

static int
gpi_transfer(bool read, uint8_t *buffer, uint32_t length)
{
	int rc;

	if (length == 0 || length > 0x2000) {
		return -2;
	}
	if (!read) {
		for (uint32_t i = 0; i < length; i++) {
			gpi.mem[M_BUF + i] = buffer[i];
		}
	} else {
		put_tre(1, (uint32_t)(GPI_IO + M_BUF), (uint32_t)((GPI_IO + M_BUF) >> 32), length, TRE_DMA | TRE_IEOT);
		doorbell(1);
	}
	put_tre(0, CONFIG0_WORD0, 0, CONFIG0_WORD2, TRE_CONFIG0 | TRE_CHAIN);
	if (read) {
		put_tre(0, GO_READ | (uint32_t)S3908_ADDRESS << 8, 0, length, TRE_GO | TRE_LINK);
	} else {
		put_tre(0, GO_WRITE | (uint32_t)S3908_ADDRESS << 8, 0, 0, TRE_GO | TRE_CHAIN);
		put_tre(0, (uint32_t)(GPI_IO + M_BUF), (uint32_t)((GPI_IO + M_BUF) >> 32), length, TRE_DMA | TRE_IEOT);
	}
	doorbell(0);
	rc = gpi_wait(read ? 1 : 0);
	if (rc == 0 && read) {
		gpi_sync();
		for (uint32_t i = 0; i < length; i++) {
			buffer[i] = gpi.mem[M_BUF + i];
		}
	}
	if (rc != 0) {
		/* the engine's channels may now be in error: start them again (as Linux's terminate_all) */
		if (gpi.restarts++ < 3 || gpi.restarts % 100 == 0) {
			printf("s3908: gpi: %s of %u bytes: rc %d, last event %08x %08x %08x %08x, gpii error %#x, se m-irq %#x\n",
			    read ? "read" : "write", length, rc, gpi.last_ev[0], gpi.last_ev[1], gpi.last_ev[2], gpi.last_ev[3],
			    r32(gpi.gpii, G_ERROR_LOG), r32(gpi.se, SE_GENI_M_IRQ_STATUS));
		}
		gpi_smmu_faults("after a failed transfer");
		w32(gpi.se, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
		w32(gpi.se, SE_GENI_S_IRQ_CLEAR, 0xffffffffu);
		int again = gpi_start();
		if (again != 0) {
			gpi_off("the GPII did not start again", again);
		}
	}
	return rc;
}

/* touch_bus=2: everything, or nothing and the wires by hand */
static bool
gpi_on(void)
{
	vm_offset_t block = 0;
	uint32_t fw, vote;
	int rc;

	gpi.se = map(SE4_PHYS, 0x1000);
	gpi.gcc = map(GCC_SE4_PAGE, 0x1000);
	gpi.gcc_vote = map(GCC_VOTE_PAGE, 0x1000);
	gpi.smmu = map(APPS_SMMU_PHYS, 0x1000);
	if (gpi.se == NULL || gpi.gcc == NULL || gpi.gcc_vote == NULL || gpi.smmu == NULL) {
		return false;
	}
	fw = r32(gpi.se, GENI_FW_REVISION_RO);
	if (((fw >> 8) & 0xff) != 3) {
		printf("s3908: gpi: the serial engine runs protocol %u, not I2C (fw %#x)\n", (fw >> 8) & 0xff, fw);
		return false;
	}
	vote = r32(gpi.gcc_vote, GCC_VOTE);
	if ((vote & GCC_VOTE_BITS) != GCC_VOTE_BITS) {
		printf("s3908: gpi: voting for the QUP0 clocks (vote %#x -> %#x)\n", vote, vote | GCC_VOTE_BITS);
		w32(gpi.gcc_vote, GCC_VOTE, vote | GCC_VOTE_BITS);
		barrier();
	}
	for (unsigned i = 0; r32(gpi.gcc, GCC_S4_CBCR) & CBCR_OFF; i++) {
		if (i == 100) {
			printf("s3908: gpi: the serial engine's clock stays off (cbcr %#x)\n", r32(gpi.gcc, GCC_S4_CBCR));
			return false;
		}
		delay_us(10);
	}
	if (r32(gpi.gcc, GCC_S4_RCG + 0x14) & 1) {
		w32(gpi.se, SE_GENI_CLK_SEL, 0);        /* DFS: level 0, as qcom_geni_i2c_conf */
	}

	if (kmem_alloc_contig(kernel_map, &block, M_SIZE, PAGE_MASK, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("s3908: gpi: no memory\n");
		return false;
	}
	FlushPoC_DcacheRegion(block, M_SIZE);
	gpi.pa = kvtophys(block);
	gpi.mem = (volatile uint8_t *)ml_io_map_wcomb((vm_offset_t)gpi.pa, M_SIZE);    /* used only through this, uncached */
	if (gpi.mem == NULL || (gpi.pa >> 30) != ((gpi.pa + M_SIZE - 1) >> 30)) {
		printf("s3908: gpi: memory at %#llx not usable\n", gpi.pa);
		return false;
	}
	if (!gpi_smmu_on()) {
		return false;
	}

	/* geni_se_select_gpi_mode() */
	w32(gpi.se, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
	w32(gpi.se, SE_GENI_S_IRQ_CLEAR, 0xffffffffu);
	w32(gpi.se, SE_IRQ_EN, 0);
	w32(gpi.se, SE_GENI_M_IRQ_EN, r32(gpi.se, SE_GENI_M_IRQ_EN) & ~(1u << 0 | 1u << 26 | 1u << 27 | 1u << 30));
	w32(gpi.se, SE_GENI_DMA_MODE_EN, 1);
	w32(gpi.se, SE_GSI_EVENT_EN, r32(gpi.se, SE_GSI_EVENT_EN) | 0xfu);

	/* the QUP's way to memory, as Android's interconnect votes (on in every run that worked; not tried without) */
	bcm_vote("QUP0", 1);
	bcm_vote("SN3", 0x3fff);
	bcm_vote("SN0", 0x3fff);
	/*
	 * GPII 0: the one the serial engine is bound to (dt: qcom,static-gpii-mask 0x1). Android's driver takes it for SE 4
	 * ("gpi_of_dma_xlate: client req gpii:0 ... proto:3 SE:4", its ipc log, 2026-11-11); a GO from GPII 1 for SE 4
	 * reset the phone every time.
	 */
	gpi.n = 0;
	gpi.gpii = map(GPII_PHYS(gpi.n), 0x4000);
	if (gpi.gpii == NULL) {
		return false;
	}
	if ((rc = gpi_start()) != 0) {
		printf("s3908: gpi: GPII %u does not start: rc %d (ch0 %#x ch1 %#x ev %#x, error %#x)\n", gpi.n, rc,
		    r32(gpi.gpii, G_CH(0) + CNTXT_0), r32(gpi.gpii, G_CH(1) + CNTXT_0), r32(gpi.gpii, G_EV + CNTXT_0),
		    r32(gpi.gpii, G_ERROR_LOG));
		gpi_release();
		return false;
	}

	/* the pins to the serial engine: function 1 (qup4), 2 mA, pull up */
	wr(PIN_SDA, 3u | 1u << 2);
	wr(PIN_SCL, 3u | 1u << 2);
	(void)rd(PIN_SCL);
	gpi.on = true;
	printf("s3908: gpi: GPII %u started, memory at %#llx, the pins are the serial engine's\n", gpi.n, gpi.pa);
	return true;
}

/* back to the wires by hand */
static void
gpi_off(const char *why, int rc)
{
	if (!gpi.on) {
		return;
	}
	gpi.on = false;
	printf("s3908: gpi: %s (rc %d): back to the wires by hand\n", why, rc);
	gpi_release();
	line(PIN_SDA, true);
	line(PIN_SCL, true);
}

static int
i2c_read(uint8_t *buffer, uint32_t length)
{
	return gpi.on ? gpi_transfer(true, buffer, length) : bb_read(buffer, length);
}

static int
i2c_write(const uint8_t *buffer, uint32_t length)
{
	return gpi.on ? gpi_transfer(false, (uint8_t *)(uintptr_t)buffer, length) : bb_write(buffer, length);
}

/* ---- TouchComm v1 ------------------------------------------------------------------------------- */

#define TCM_MARKER              0xA5
#define TCM_PADDING             0x5A
#define TCM_STATUS_IDLE         0x00
#define TCM_STATUS_OK           0x01
#define TCM_STATUS_BUSY         0x02
#define TCM_STATUS_CONTINUED    0x03
#define TCM_REPORT_IDENTIFY     0x10
#define TCM_REPORT_TOUCH        0x11
#define TCM_CMD_GET_REPORT_CONFIG 0x25
#define TCM_MAX_OBJECTS         10
#define TCM_MAX_PAYLOAD         512
#define TCM_MAX_CONFIG          256

/* the codes of the touch report config */
enum {
	TOUCH_END = 0, TOUCH_FOREACH_ACTIVE_OBJECT, TOUCH_FOREACH_OBJECT, TOUCH_FOREACH_END, TOUCH_PAD_TO_NEXT_BYTE,
	TOUCH_TIMESTAMP, TOUCH_OBJECT_N_INDEX, TOUCH_OBJECT_N_CLASSIFICATION, TOUCH_OBJECT_N_X_POSITION,
	TOUCH_OBJECT_N_Y_POSITION, TOUCH_OBJECT_N_Z, TOUCH_NUM_OF_ACTIVE_OBJECTS = 24,
};

static struct {
	uint8_t config[TCM_MAX_CONFIG];
	uint32_t config_length;
	uint8_t payload[TCM_MAX_PAYLOAD];
	uint32_t payload_length;
	uint8_t code;
	uint8_t header[4];
	int bus_error;
	struct {
		uint8_t status;         /* 0 = no finger */
		uint32_t x, y;
	} object[TCM_MAX_OBJECTS];
} tcm;

/* One message: 0 = it is in tcm.code / tcm.payload, 1 = nothing waiting, < 0 = the bus or the framing failed */
static int
tcm_read_message(void)
{
	static uint8_t buffer[TCM_MAX_PAYLOAD + 3];
	uint32_t length;

	tcm.code = 0;
	tcm.payload_length = 0;
	tcm.header[0] = tcm.header[1] = tcm.header[2] = tcm.header[3] = 0;
	if ((tcm.bus_error = i2c_read(tcm.header, 4)) != 0) {
		return -1;
	}
	if (tcm.header[0] != TCM_MARKER) {
		return 1;
	}
	tcm.code = tcm.header[1];
	if (tcm.code == TCM_STATUS_IDLE || tcm.code == TCM_STATUS_BUSY) {
		return 1;
	}
	if (tcm.code == TCM_STATUS_CONTINUED) {
		return -6;      /* the rest of an answer we did not take whole: read on (tcm_command) */
	}
	length = tcm.header[2] | (uint32_t)tcm.header[3] << 8;
	if (length == 0) {
		return 0;
	}
	if (length > TCM_MAX_PAYLOAD) {
		return -2;
	}
	if ((tcm.bus_error = i2c_read(buffer, length + 3)) != 0) {
		return -1;
	}
	if (buffer[0] != TCM_MARKER || buffer[1] != TCM_STATUS_CONTINUED || buffer[length + 2] != TCM_PADDING) {
		return -3;
	}
	for (uint32_t i = 0; i < length; i++) {
		tcm.payload[i] = buffer[2 + i];
	}
	tcm.payload_length = length;
	return 0;
}

/* a command with no payload, and its answer (reports that come in between are dropped) */
static int
tcm_command(uint8_t command)
{
	uint8_t message[3] = { command, 0, 0 };

	if ((tcm.bus_error = i2c_write(message, sizeof(message))) != 0) {
		return -1;
	}
	for (unsigned tries = 0; tries < 200; tries++) {
		int rc = tcm_read_message();

		if (rc == -6) {
			/* a stray continuation (seen 2026-11-09: the chip answered late, after an earlier read had given up) */
			sleep_ms(1);
			continue;
		}
		if (rc < 0) {
			return rc;
		}
		if (rc == 0 && tcm.code < TCM_REPORT_IDENTIFY) {
			return tcm.code == TCM_STATUS_OK ? 0 : -4;
		}
		sleep_ms(1);
	}
	return -5;
}

static int
tcm_get_report_config(void)
{
	int rc = tcm_command(TCM_CMD_GET_REPORT_CONFIG);

	if (rc) {
		return rc;
	}
	if (tcm.payload_length > TCM_MAX_CONFIG) {
		return -2;
	}
	for (uint32_t i = 0; i < tcm.payload_length; i++) {
		tcm.config[i] = tcm.payload[i];
	}
	tcm.config_length = tcm.payload_length;
	return 0;
}

/* up to 32 bits of the payload at a bit offset, least significant first */
static uint32_t
tcm_bits(uint32_t offset, uint32_t bits)
{
	uint32_t out = 0, done = 0;

	if (bits == 0 || bits > 32 || offset + bits > tcm.payload_length * 8) {
		return 0;
	}
	while (done < bits) {
		uint32_t byte = offset / 8, shift = offset % 8;
		uint32_t take = 8 - shift < bits - done ? 8 - shift : bits - done;
		uint32_t v = ((uint32_t)tcm.payload[byte] >> shift) & (0xffu >> (8 - take));

		out |= v << done;
		done += take;
		offset += take;
	}
	return out;
}

/* a touch report into tcm.object[], as its config describes it: 0 or < 0 */
static int
tcm_parse_touch(void)
{
	uint32_t idx = 0, offset = 0, obj = 0, next = 0, objects = 0, active = 0;
	bool active_only = false, have_active = false;

	for (int i = 0; i < TCM_MAX_OBJECTS; i++) {
		tcm.object[i].status = 0;
	}
	while (idx < tcm.config_length) {
		uint8_t code = tcm.config[idx++];
		uint32_t bits, v;

		switch (code) {
		case TOUCH_END:
			return 0;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = code == TOUCH_FOREACH_ACTIVE_OBJECT;
			break;
		case TOUCH_FOREACH_END:
			if (active_only) {
				if (have_active) {
					if (++objects < active) {
						idx = next;
					}
				} else if (offset < tcm.payload_length * 8) {
					idx = next;
				}
			} else if (++obj < TCM_MAX_OBJECTS) {
				idx = next;
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = (offset + 7) & ~7u;
			break;
		default:
			if (idx >= tcm.config_length) {
				return -1;
			}
			bits = tcm.config[idx++];
			v = tcm_bits(offset, bits);
			offset += bits;
			switch (code) {
			case TOUCH_OBJECT_N_INDEX:
				if (v >= TCM_MAX_OBJECTS) {
					return -1;
				}
				obj = v;
				break;
			case TOUCH_OBJECT_N_CLASSIFICATION:
				tcm.object[obj].status = (uint8_t)v;
				break;
			case TOUCH_OBJECT_N_X_POSITION:
				tcm.object[obj].x = v;
				break;
			case TOUCH_OBJECT_N_Y_POSITION:
				tcm.object[obj].y = v;
				break;
			case TOUCH_NUM_OF_ACTIVE_OBJECTS:
				active = v;
				have_active = true;
				if (active == 0) {
					return 0;
				}
				break;
			default:        /* timestamp, z, widths, gestures: skipped */
				break;
			}
		}
	}
	return 0;
}

/* ---- touches ------------------------------------------------------------------------------------ */

static struct {
	bool down;
	uint32_t x, y;
} contact[TCM_MAX_OBJECTS];
static uint32_t flip;           /* boot-arg touch_flip: 1 = mirror left-right, 2 = mirror top-bottom */
static struct {
	uint32_t polls, messages, touches, errors, last_x, last_y;
	uint64_t report_ns, reports;    /* the time to take one touch report off the chip, header and all */
} seen;

static void
touch_report(void)
{
	bool now[TCM_MAX_OBJECTS];

	for (unsigned id = 0; id < TCM_MAX_OBJECTS; id++) {
		now[id] = false;
		if (tcm.object[id].status == 0) {
			continue;
		}
		uint32_t px = tcm.object[id].x, py = tcm.object[id].y;
		uint32_t x = (px >= PANEL_MAX_X ? PANEL_MAX_X : px) * 65535ull / PANEL_MAX_X;
		uint32_t y = (py >= PANEL_MAX_Y ? PANEL_MAX_Y : py) * 65535ull / PANEL_MAX_Y;

		if (flip & 1) {
			x = 65535 - x;
		}
		if (flip & 2) {
			y = 65535 - y;
		}
		now[id] = true;
		seen.touches++;
		seen.last_x = px;
		seen.last_y = py;
		if (!contact[id].down) {
			hakios_touch_post(HAKIOS_TOUCH_DOWN, id, x, y);
		} else if (contact[id].x != x || contact[id].y != y) {
			hakios_touch_post(HAKIOS_TOUCH_MOVE, id, x, y);
		}
		contact[id].down = true;
		contact[id].x = x;
		contact[id].y = y;
	}
	for (unsigned id = 0; id < TCM_MAX_OBJECTS; id++) {
		if (contact[id].down && !now[id]) {
			contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
	}
}

static void
touch_release_all(void)
{
	for (unsigned id = 0; id < TCM_MAX_OBJECTS; id++) {
		tcm.object[id].status = 0;
	}
	touch_report();
}

/* Drain what is waiting (the identify report after a reset, old touches), then learn the report layout. */
static int
chip_setup(void)
{
	int rc = 1;

	for (unsigned i = 0; i < 50 && !attention(); i++) {
		rc = tcm_read_message();
		if (rc < 0) {
			break;
		}
	}
	printf("s3908: drain rc=%d code=%#x header %02x %02x %02x %02x bus=%d attention=%d\n", rc, tcm.code,
	    tcm.header[0], tcm.header[1], tcm.header[2], tcm.header[3], tcm.bus_error, attention());
	/* a few tries: an answer can come late and leave a stray piece behind (2026-11-09, first boot from darwin) */
	for (unsigned tries = 0; tries < 5; tries++) {
		rc = tcm_get_report_config();
		printf("s3908: report config rc=%d length=%u bus=%d (try %u)\n", rc, tcm.config_length, tcm.bus_error,
		    tries + 1);
		if (rc == 0) {
			break;
		}
		sleep_ms(50);
		for (unsigned i = 0; i < 50 && !attention(); i++) {
			if (tcm_read_message() < 0 && tcm.bus_error) {
				break;
			}
		}
	}
	return rc;
}

static void
s3908_thread(__unused void *argument, __unused wait_result_t wait)
{
	uint32_t beat = 0, since_beat = 0;

	sleep_ms(5000);         /* the system is up by then, and the screen shows what happens */
	pins = (volatile uint8_t *)ml_io_map(TLMM_PINS_PHYS, TLMM_PINS_SIZE);
	if (pins == NULL) {
		printf("s3908: cannot map the pins\n");
		return;
	}
	PE_parse_boot_argn("touch_bus", &touch_bus, sizeof(touch_bus));
	if (touch_bus == 1) {
		bus_look();
	}
	line(PIN_SDA, true);
	line(PIN_SCL, true);
	delay_us(5);
	printf("s3908: reset=%u attention=%d sda=%d scl=%d (the firmware leaves the panel powered)\n",
	    (rd(PIN_RESET + 4) >> 1) & 1, attention(), sda_get(), scl_get());

	if (touch_bus == 2 && gpi_on()) {
		int rc = chip_setup();

		if (rc != 0) {
			gpi_off("no report config through the serial engine", rc);
		}
	}
	if (!gpi.on && chip_setup() != 0) {
		/* reset the chip (low for 10 ms, then 200 ms for its firmware to start) and try once more */
		wr(PIN_RESET + 4, 0);
		sleep_ms(10);
		wr(PIN_RESET + 4, IO_OUT);
		sleep_ms(200);
		if (chip_setup() != 0) {
			printf("s3908: no report config from the chip: stopping\n");
			return;
		}
	}
	printf("s3908: running (%d x %d) over %s, looking at the attention line every 10 ms\n", PANEL_MAX_X, PANEL_MAX_Y,
	    gpi.on ? "the serial engine (GPI DMA)" : "the wires by hand");

	for (;;) {
		for (unsigned n = 0; n < 4 && !attention(); n++) {
			uint64_t start = mach_absolute_time(), ns;
			int rc = tcm_read_message();

			if (rc < 0) {
				seen.errors++;
				touch_release_all();
				break;
			}
			seen.messages++;
			if (rc == 0 && tcm.code == TCM_REPORT_TOUCH) {
				absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
				seen.report_ns += ns;
				seen.reports++;
				if (tcm_parse_touch() == 0) {
					touch_report();
				}
			}
		}
		seen.polls++;
		sleep_ms(10);
		if (++since_beat >= 500) {      /* every 5 s: a line for the console, every minute once it is quiet */
			since_beat = 0;
			if (beat++ < 24 || beat % 12 == 0) {
				printf("s3908: %u looks, %u messages, %u finger positions (last %u, %u), %u errors, attention %s, "
				    "%s, %llu touch reports at %llu us each\n",
				    seen.polls, seen.messages, seen.touches, seen.last_x, seen.last_y, seen.errors,
				    attention() ? "high" : "low", gpi.on ? "gpi" : "by hand", seen.reports,
				    seen.reports ? seen.report_ns / seen.reports / 1000 : 0);
			}
		}
	}
}

void
hakios_s3908_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("touchscreen", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	PE_parse_boot_argn("touch_flip", &flip, sizeof(flip));
	if (kernel_thread_start(s3908_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
