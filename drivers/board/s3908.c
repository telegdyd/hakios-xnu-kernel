/*
 * A touch screen on I2C: a Synaptics S3908 speaking TouchComm v1, as a source of touches for /dev/hakios-touch
 * (sys/board/input.h). Where the chip is, its address, its attention line and reset, how many fingers it reports and
 * the largest coordinates are the board's touch-i2c { } (board/board.h), on a board that lists "touch-s3908".
 *
 * Run on the OnePlus 10 Pro (negroni) since 2026-11-08, as drivers/input/hakios.s3908.c of its own tree; this is that
 * driver's path through the wires by hand, its wiring made data. Written from slop's negroni-ios patch 0093, which took
 * it from the OPLUS driver (drivers/input/touchscreen/synaptics_tcm_oncell.c).
 *
 *   power      none: the firmware (NegroniTouchDxe on negroni) has switched the panel's supplies on and leaves them on
 *   I2C        through the bus's own serial engine fed by the QUP's GPI DMA, where touch-i2c { } describes them (the
 *              engine's FIFO is off on negroni, so only the GPI can feed it); else, or after anything about that fails,
 *              by hand (board/i2c_gpio.h) on the board's two pins, taken for each transaction and given back as found
 *   attention  low while the chip has a message, read until it is high again: an interrupt through the PDC where
 *              touch-i2c { interrupt-pdc } says, with a net of a second; else looked at every POLL_MS
 *   reset      low = in reset: pulsed only when the chip does not answer as the firmware left it
 *   report     a bit stream the chip's own "touch report config" (command 0x25) describes, asked for once at start
 *
 * Nothing is done unless a boot-arg asks for it:
 *   touchscreen=1   the chip's messages read and its touches posted
 *   touchscreen=-1  only looks: the pins, and the chip asked for its report config as the firmware left it, by hand
 *   touch_flip=N    bit 0 mirrors the touches left to right, bit 1 top to bottom (as the other touch drivers)
 *   s3908_trace=1   the first 40 fingers down, as the chip gives them, in the log
 *   touch_bus=0     the wires by hand even where the board has the serial engine (default 2: the engine and the GPI)
 *   touch_irq=0     the attention line looked at every POLL_MS even where the board has its PDC route (default 1)
 *   touch_idle_ms=N the bus votes for the QUP's way to memory let go after N ms without a transfer (default 250; 0:
 *                   held)
 *
 * The GPI's way to memory (from negroni's driver, measured there: a touch report 947 us against 11 ms by hand, the
 * chip's setup 3.6 ms against 54 ms):
 *   - the engine's clocks voted on in the GCC, its rate generator's DFS at level 0, the engine put in GPI mode
 *   - an SMMU context of ours for the GPI's stream (its rings) and the QUP wrapper's (the bytes), which see our 16 KB
 *     block at GPI_IO, below 4 GB as Android's addresses are: the hypervisor does not let either through as it is
 *   - the GPII polled, its interrupts masked and wired (INTSET 1, no MSI address: a random one was refused by the
 *     SMMU, 2026-11-11)
 *   - the bus clock managers on the QUP's way to memory (touch-i2c { bus-votes }: QUP0, SN3, SN0 on SM8450) voted the
 *     least, through RPMh, while transfers are made, as Android's interconnect votes; let go when idle, held for good
 *     after a transfer that failed (the DMA may not have stopped)
 * A transfer that fails restarts the GPII; a restart that fails goes back to the wires by hand.
 *
 * At the run's end (hakios_s3908_quiesce()) the polling stops, the interrupt is masked and the PDC and the pin's
 * interrupt configuration are as found, the GPII is stopped and the SMMU's match registers and context as found, and
 * the bus's pins as the run found them. The chip is left running: the firmware's touch driver reads it again after the
 * run.
 *
 * Not ported from negroni's tree: its read-only look at the bus hardware, and the slower look while the screen is off.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <machine/machine_routines.h>
#include <os/atomic_private.h>
#include <pexpert/pexpert.h>
#include <sys/board/input.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>
#include <vm/vm_kern_xnu.h>  /* xnu-12377: the KMA flags and kmem_alloc moved to the internal header */

#include "kernel.h"
#include "board.h"
#include "log.h"

extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *, uint32_t), void *context);
extern void gic_v3_spi_mask(uint32_t intid);
extern void gic_v3_spi_unmask(uint32_t intid);
extern lck_mtx_t hakios_smmu_lock;
extern void hakios_rpmh_lock(void);
extern void hakios_rpmh_unlock(void);
extern bool hakios_rpmh_bw(enum hakios_rpmh_client client, const char *bcm, uint32_t vote_a, uint32_t vote_b) __attribute__((weak));

static vm_offset_t tlmm;

/* as in board/ft5x06.c: the thread holds `busy' around the chip, `leaving' is the run's end, `on_bus' a transaction */
static bool leaving, busy, on_bus;
static bool started;

HAKIOS_DEFINE_SAY(!__atomic_load_n(&leaving, __ATOMIC_SEQ_CST) && ml_get_interrupts_enabled())

#include "mmio.h"
#include "time.h"

#define wait_ms(ms)             hakios_wait_ms((ms), __atomic_load_n(&leaving, __ATOMIC_SEQ_CST))

#define POLL_MS                 8       /* the attention line is looked at 125 times a second */
#define MESSAGES_A_LOOK         4       /* ... and so many messages read at most while it stays low */
#define RESET_LOW_MS            10      /* negroni's driver: low 10 ms, then 200 ms for the chip's firmware */
#define RESET_WAIT_MS           200
#define DRAIN_MESSAGES          50
#define CONFIG_TRIES            5
#define TROUBLE_ERRORS          250     /* so many failed reads in a row: the chip is reset ... */
#define TROUBLE_RESETS          3       /* ... so many times; the next time the polling ends */

/* ---- pins (TLMM) ------------------------------------------------------------------------------------- */

#define TLMM_PHYS               (hakios_board()->pins.tlmm)
#define HAKIOS_TLMM             tlmm
#include "tlmm.h"

#define WIRED                   (&hakios_board()->touch_i2c)
#define PIN_SDA                 ((unsigned)WIRED->sda)
#define PIN_SCL                 ((unsigned)WIRED->scl)
#define PIN_ATTN                ((unsigned)WIRED->interrupt)    /* low = the chip has a message */
#define PIN_RESET               ((unsigned)WIRED->reset)        /* low = in reset */
#define ADDRESS                 (WIRED->address)

static bool
attention(void)
{
	return !gpio_get(PIN_ATTN);
}

static void
show_pin(unsigned pin, const char *what)
{
	uint32_t cfg = rd(GPIO_CFG(pin)), io = rd(GPIO_IO(pin));

	say("s3908: gpio%u (%s): function %u, pull %u, %u mA, %s, reads %u\n", pin, what, (cfg >> 2) & 15, cfg & 3,
	    (((cfg >> 6) & 7) + 1) * 2, (cfg & CFG_OUTPUT) ? (io & 2 ? "drives high" : "drives low") : "input", io & 1);
}

static void
show_pins(void)
{
	show_pin(PIN_SDA, "I2C data");
	show_pin(PIN_SCL, "I2C clock");
	show_pin(PIN_ATTN, "attention");
	show_pin(PIN_RESET, "reset");
}

/* ---- I2C by hand (board/i2c_gpio.h), the bus's pins as the run's first transaction found them ----------- */

#include "i2c_gpio.h"

static struct i2c_gpio i2c_bus;
static uint32_t bus_found_cfg[2], bus_found_io[2];
static bool bus_found;
static unsigned refusals_said;

static void
bus_record(void)
{
	if (__atomic_load_n(&bus_found, __ATOMIC_SEQ_CST)) {
		return;
	}
	bus_found_cfg[0] = rd(GPIO_CFG(i2c_bus.sda));
	bus_found_cfg[1] = rd(GPIO_CFG(i2c_bus.scl));
	bus_found_io[0] = rd(GPIO_IO(i2c_bus.sda));
	bus_found_io[1] = rd(GPIO_IO(i2c_bus.scl));
	__atomic_store_n(&bus_found, true, __ATOMIC_SEQ_CST);
}

static void
bus_give_back(void)
{
	for (unsigned i = 0; i < 2; i++) {
		i2c_bus.found_cfg[i] = bus_found_cfg[i];
		i2c_bus.found_io[i] = bus_found_io[i];
	}
	i2c_gpio_give_back(&i2c_bus);
	__atomic_store_n(&on_bus, false, __ATOMIC_SEQ_CST);
}

static bool
bus_take(void)
{
	bus_record();
	__atomic_store_n(&on_bus, true, __ATOMIC_SEQ_CST);
	if (i2c_gpio_take(&i2c_bus)) {
		return true;
	}
	bus_give_back();
	if (refusals_said++ < 4) {
		say("s3908: the I2C bus is not at rest (gpio%u reads %u, gpio%u reads %u): not touched\n", PIN_SDA,
		    i2c_gpio_level(PIN_SDA), PIN_SCL, i2c_gpio_level(PIN_SCL));
	}
	return false;
}

/* 0, -2 not acknowledged, -3 the bus is not at rest */
static int
bb_read(uint8_t *buffer, uint32_t length)
{
	bool answered;

	if (!bus_take()) {
		return -3;
	}
	i2c_gpio_start(&i2c_bus);
	answered = i2c_gpio_write(&i2c_bus, (uint8_t)(ADDRESS << 1 | 1));
	for (uint32_t i = 0; answered && i < length; i++) {
		buffer[i] = i2c_gpio_read(&i2c_bus, i + 1 < length);
	}
	i2c_gpio_stop(&i2c_bus);
	bus_give_back();
	return answered ? 0 : -2;
}

static int
bb_write(const uint8_t *buffer, uint32_t length)
{
	bool answered;

	if (!bus_take()) {
		return -3;
	}
	i2c_gpio_start(&i2c_bus);
	answered = i2c_gpio_write(&i2c_bus, (uint8_t)(ADDRESS << 1));
	for (uint32_t i = 0; answered && i < length; i++) {
		answered = i2c_gpio_write(&i2c_bus, buffer[i]);
	}
	i2c_gpio_stop(&i2c_bus);
	bus_give_back();
	return answered ? 0 : -2;
}

/* ---- the attention line as an interrupt, through the PDC (touch-i2c { interrupt-pdc }) ------------------ */

/*
 * Linux pinctrl-msm.c with the SoC's layout: a pin's interrupt config at +8 of its registers, [0] enable; its status at
 * +0xc. The PDC sees the pin by itself (Linux's skip_wake_irqs): the TLMM's own interrupt for it is switched off, the
 * PDC's pin set to level (it turns a low level into the GIC's high), and enabled -- in the bank register before PDC
 * v3.2, in the pin's own config from it. The handler masks the GIC's line and wakes the thread; the thread reads,
 * then unmasks: a line still low rings again at once, so nothing is lost while it reads.
 */
#define INTR_CFG(pin)           (GPIO_CFG(pin) + 8)
#define INTR_STATUS(pin)        (GPIO_CFG(pin) + 0xc)
#define INTR_ENABLE             (1u << 0)
#define PDC_SIZE                0x2000
#define PDC_VERSION             0x1000
#define PDC_V3_2                0x30200u
#define PDC_BANK(pin)           (0x10 + 4 * ((pin) / 32))
#define PDC_BIT(pin)            (1u << ((pin) % 32))
#define PDC_CFG(pin)            (0x110 + 4 * (pin))
#define IRQ_NET_MS              1000    /* a look even without an interrupt, in case one was lost */
#define IRQ_STORM               2000    /* more in a second (the chip reports some hundreds while touched): looked at instead */

static vm_offset_t pdc;
static uint32_t irq_wanted = 1;
static struct {
	bool on, delivered, said_storm;
	uint32_t old_tlmm, old_pdc_cfg, old_pdc_bank;
	volatile uint32_t rang;
	uint32_t count, by_line, nets, period_count;
	uint64_t period, second;
} irq;

#define PDC_PIN                 (WIRED->pdc_pin)
#define PDC_INTID               (WIRED->pdc_intid)

static uint32_t pdc_rd(uint32_t offset) { return *(volatile uint32_t *)(pdc + offset); }
static void pdc_wr(uint32_t offset, uint32_t value) { *(volatile uint32_t *)(pdc + offset) = value; }

/* In interrupt context. */
static void
irq_arrived(__unused void *context, uint32_t intid)
{
	uint64_t now = mach_absolute_time();

	gic_v3_spi_mask(intid);
	irq.count++;
	if (now - irq.period >= irq.second) {
		irq.period = now;
		irq.period_count = 0;
	}
	if (++irq.period_count > IRQ_STORM) {
		irq.on = false;                 /* (left masked; the thread says so and looks instead) */
	}
	os_atomic_store(&irq.rang, 1, release);
	thread_wakeup((event_t)&irq.rang);
}

/* the PDC's pin and the TLMM's interrupt for it as they were found */
static void
irq_restore(void)
{
	if (pdc == 0) {
		return;
	}
	pdc_wr(PDC_CFG(PDC_PIN), irq.old_pdc_cfg);
	if (pdc_rd(PDC_VERSION) < PDC_V3_2) {
		pdc_wr(PDC_BANK(PDC_PIN), (pdc_rd(PDC_BANK(PDC_PIN)) & ~PDC_BIT(PDC_PIN)) | (irq.old_pdc_bank & PDC_BIT(PDC_PIN)));
	}
	wr(INTR_CFG(PIN_ATTN), irq.old_tlmm);
}

static void
irq_start(void)
{
	uint32_t version, level;

	if (!irq_wanted || PDC_PIN == 0 || hakios_board()->pins.pdc == 0) {
		say("s3908: the attention line is looked at every %u ms (%s)\n", POLL_MS,
		    irq_wanted ? "the board names no PDC route for it" : "touch_irq=0");
		return;
	}
	pdc = ml_io_map(hakios_board()->pins.pdc, PDC_SIZE);
	if (pdc == 0) {
		say("s3908: the PDC could not be mapped: looked at every %u ms\n", POLL_MS);
		return;
	}
	version = pdc_rd(PDC_VERSION);
	level = version >= PDC_V3_2 ? 8u : 0u;          /* (v3.2 has the enable in the pin's config, bit 3) */
	irq.old_tlmm = rd(INTR_CFG(PIN_ATTN));
	irq.old_pdc_cfg = pdc_rd(PDC_CFG(PDC_PIN));
	irq.old_pdc_bank = pdc_rd(PDC_BANK(PDC_PIN));
	wr(INTR_CFG(PIN_ATTN), irq.old_tlmm & ~INTR_ENABLE);
	wr(INTR_STATUS(PIN_ATTN), 0);
	pdc_wr(PDC_CFG(PDC_PIN), level);
	if (version < PDC_V3_2) {
		pdc_wr(PDC_BANK(PDC_PIN), irq.old_pdc_bank | PDC_BIT(PDC_PIN));
	}
	if ((pdc_rd(PDC_CFG(PDC_PIN)) & 0xfu) != level ||
	    (version < PDC_V3_2 && (pdc_rd(PDC_BANK(PDC_PIN)) & PDC_BIT(PDC_PIN)) == 0)) {
		say("s3908: the PDC (version 0x%x) did not take pin %u: looked at every %u ms\n", version, PDC_PIN, POLL_MS);
		irq_restore();
		pdc = 0;
		return;
	}
	nanoseconds_to_absolutetime(NSEC_PER_SEC, &irq.second);
	os_atomic_store(&irq.rang, 0, relaxed);
	irq.on = true;
	if (gic_v3_spi_attach(PDC_INTID, false, irq_arrived, NULL) != KERN_SUCCESS) {
		irq.on = false;
		irq_restore();
		pdc = 0;
		say("s3908: GIC %u cannot be taken: looked at every %u ms\n", PDC_INTID, POLL_MS);
		return;
	}
	irq.delivered = true;
	say("s3908: interrupts: PDC version 0x%x, gpio%u -> pin %u -> GIC %u; config 0x%x -> 0x%x, bank 0x%x -> 0x%x; net %u ms\n",
	    version, PIN_ATTN, PDC_PIN, PDC_INTID, irq.old_pdc_cfg, pdc_rd(PDC_CFG(PDC_PIN)), irq.old_pdc_bank,
	    pdc_rd(PDC_BANK(PDC_PIN)), IRQ_NET_MS);
}

/* ---- the bus through its serial engine and the QUP's GPI DMA (touch-i2c { serial-engine ... }) --------------- */

/*
 * From Linux v6.12: drivers/dma/qcom/gpi.c (rings, commands, the I2C TREs), drivers/i2c/busses/i2c-qcom-geni.c
 * (geni_i2c_gpi_xfer: a write is CONFIG0, GO(write), DMA on the out channel; a read is CONFIG0, GO(read, length) with
 * LINK on the out channel and DMA on the in channel), drivers/soc/qcom/qcom-geni-se.c (geni_se_select_gpi_mode).
 * The SMMU's layout: board/smmu.h.
 */
#include "smmu.h"

#define SE_GENI_FW_REVISION     0x068
#define SE_GENI_CLK_SEL         0x07c
#define SE_GENI_DMA_MODE_EN     0x258
#define SE_GENI_M_IRQ_STATUS    0x610
#define SE_GENI_M_IRQ_EN        0x614
#define SE_GENI_M_IRQ_CLEAR     0x618
#define SE_GENI_S_IRQ_CLEAR     0x648
#define SE_GSI_EVENT_EN         0xe18
#define SE_IRQ_EN               0xe1c
#define CBCR_OFF                (1u << 31)
#define RCG_DFSR                0x14    /* from the engine's rate generator (its branch's offset + 8) */

#define G_CH(k)                 (0x0000u + 0x80u * (k))
#define G_EV                    0x1000u
#define G_CH_DB(k)              (0x2000u + 0x8u * (k))
#define G_EV_DB                 0x2100u
#define G_CH_CMD                0x3008u
#define G_EV_CMD                0x3010u
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
#define G_ERROR_LOG             0x3200u
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

#define EL                      16u             /* a ring element */
#define TRES                    64u             /* each channel's ring */
#define EVS                     (TRES << 2)     /* the event ring */
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
#define CONFIG0_WORD0           (12u | 5u << 8 | 24u << 16 | 1u << 24 | 1u << 25)      /* 400 kHz from 19.2 MHz */
#define CONFIG0_WORD2           2u

#define M_EV                    0x0000u         /* our block: the event ring, the two channels' rings, the bytes */
#define M_TX                    0x1000u
#define M_RX                    0x1400u
#define M_BUF                   0x2000u
#define M_SIZE                  0x4000u
#define GPI_IO                  0x00100000u     /* where the GPI and the QUP see it (dt: qcom,iommu-dma-addr-pool) */
#define SMMU_SCTLR_ON           (1u << 0 | 1u << 1 | 1u << 2 | 1u << 5 | 1u << 8 | 1u << 12)
#define SMMU_PAGE               (3ull | 1ull << 6 | 2ull << 8 | 1ull << 10 | 1ull << 11)  /* 4K, uncached, RW */

#define BCM_COMMIT              (1u << 30)
#define BCM_VALID               (1u << 29)
#define BCM_VOTE(x, y)          (BCM_COMMIT | BCM_VALID | ((uint32_t)(x) & 0x3fff) << 14 | ((uint32_t)(y) & 0x3fff))
#define CMD_DB_PHYS             (hakios_board()->rpmh.cmd_db)
#define CMD_DB_SIZE             (hakios_board()->rpmh.cmd_db_size)
#define RSC_SIZE                0x10000UL
#include "rpmh.h"
#include "cmd_db.h"

static uint32_t bus_wanted = 2, idle_ms = 250;

static struct {
	vm_offset_t se, gcc, smmu, smmu_second, smmu_context, gpii, rsc, tables;
	volatile uint8_t *mem;                  /* our block, uncached */
	uint64_t pa;
	uint32_t tx_wp, rx_wp, ev_rp;
	uint32_t match[2], old_smr[2], old_s2cr[2], context, old_sctlr;
	uint32_t bcm[3];
	bool smmu_on, on, votes_on, votes_held;
	uint64_t last_us;
	uint32_t last_ev[4], last_error, restarts, transfers, errors, vote_errors;
} gpi;

static inline uint32_t r32(vm_offset_t base, uint32_t offset) { return *(volatile uint32_t *)(base + offset); }
static inline void w32(vm_offset_t base, uint32_t offset, uint32_t value) { *(volatile uint32_t *)(base + offset) = value; }
static inline void barrier(void) { __builtin_arm_dsb(0xf); }

/* The QUP's way to memory: each BCM the least (QUP0: the peak only, as Android's geni-se asks), or nothing. */
static bool
votes_set(bool on)
{
	uint32_t data[3] = {
		on ? BCM_VOTE(0, 1) : BCM_VOTE(0, 0), on ? BCM_VOTE(1, 1) : BCM_VOTE(0, 0), on ? BCM_VOTE(1, 1) : BCM_VOTE(0, 0),
	};
	const char *problem;
	bool locking = !__atomic_load_n(&leaving, __ATOMIC_SEQ_CST);

	if (gpi.rsc == 0) {
		return false;
	}
	if (locking && hakios_board_has(HAKIOS_HAS_SOC_SLEEP) && hakios_rpmh_bw != NULL) {
		const struct hakios_board *board = hakios_board();
		bool ok = true;

		for (unsigned i = 0; i < 3; i++) {
			if (board->touch_i2c.bus_votes[i] != NULL &&
			    !hakios_rpmh_bw(HAKIOS_RPMH_TOUCH, board->touch_i2c.bus_votes[i], i == 0 ? 0 : (on ? 1 : 0),
			    on ? 1 : 0)) {
				ok = false;
			}
		}
		if (ok) {
			gpi.votes_on = on;
			return true;
		}
		gpi.vote_errors++;
		gpi.votes_held = true;
		return false;
	}
	if (locking) {
		hakios_rpmh_lock();
	}
	problem = hakios_rpmh_send_rsc(gpi.rsc, hakios_board()->rpmh.tcs_offset, gpi.bcm, data, 3);
	if (locking) {
		hakios_rpmh_unlock();
	}
	if (problem != NULL) {
		gpi.vote_errors++;
		gpi.votes_held = true;          /* (perhaps in force: kept on from here) */
		if (gpi.vote_errors <= 4) {
			say("s3908: gpi: RPMh: %s\n", problem);
		}
		return false;
	}
	gpi.votes_on = on;
	return true;
}

static bool
votes_get(void)
{
	if (!gpi.votes_on && !votes_set(true)) {
		return false;
	}
	gpi.last_us = now_us();
	return true;
}

static void
smmu_faults(const char *when)
{
	uint32_t status;

	if (!gpi.smmu_on) {
		return;
	}
	status = r32(gpi.smmu_context, SMMU_CB_FSR);
	if ((status & SMMU_FSR_ALL) == 0) {
		return;
	}
	say("s3908: gpi: %s: THE SMMU REFUSED THE GPI: status 0x%x address 0x%llx syndrome 0x%x\n", when, status,
	    (uint64_t)r32(gpi.smmu_context, SMMU_CB_FAR + 4) << 32 | r32(gpi.smmu_context, SMMU_CB_FAR),
	    r32(gpi.smmu_context, SMMU_CB_FSYNR0));
	w32(gpi.smmu_context, SMMU_CB_FSR, SMMU_FSR_ALL);
}

/* the match registers and the context as found (with the lock held, or on the way out) */
static void
smmu_restore(void)
{
	for (int i = 1; i >= 0; i--) {
		w32(gpi.smmu, SMMU_SMR(gpi.match[i]), gpi.old_smr[i]);
		w32(gpi.smmu, SMMU_S2CR(gpi.match[i]), gpi.old_s2cr[i]);
	}
	w32(gpi.smmu_context, SMMU_CB_SCTLR, gpi.old_sctlr);
	gpi.smmu_on = false;
}

/* Our context for the GPI's stream and the QUP's, which see our block at GPI_IO. */
static bool
smmu_on(void)
{
	static bool used[256];
	uint32_t idr0, idr1, matches, contexts, page, pages, streams[2] = { WIRED->gpi_stream, WIRED->qup_stream };
	uint64_t *root, *level2, *level3, ttbr;
	unsigned found = 0;

	lck_mtx_lock(&hakios_smmu_lock);
	idr0 = r32(gpi.smmu, SMMU_IDR0);
	idr1 = r32(gpi.smmu, SMMU_IDR1);
	matches = idr0 & 0xff;
	contexts = idr1 & 0xff;
	page = idr1 >> 31 ? 0x10000 : 0x1000;
	pages = 2u << (idr1 >> 28 & 7);
	if ((r32(gpi.smmu, SMMU_SCR0) & 1u << 3) || page != 0x1000 || 2 * pages * page != 0x100000) {
		lck_mtx_unlock(&hakios_smmu_lock);
		say("s3908: gpi: the SMMU is not of the shape this knows (idr1 0x%x)\n", idr1);
		return false;
	}
	bzero(used, sizeof(used));
	for (uint32_t m = 0; m < matches && SMMU_SMR(m) < 0xc00; m++) {
		uint32_t smr = r32(gpi.smmu, SMMU_SMR(m)), s2cr = r32(gpi.smmu, SMMU_S2CR(m));

		if (smr >> 31) {
			used[s2cr & 0xff] = true;
			for (unsigned i = 0; i < 2; i++) {
				if (((smr ^ streams[i]) & ~(smr >> 16) & 0x7fff) == 0) {
					lck_mtx_unlock(&hakios_smmu_lock);
					say("s3908: gpi: stream 0x%x is matched already (register %u: smr 0x%x s2cr 0x%x): left alone\n",
					    streams[i], m, smr, s2cr);
					return false;
				}
			}
		} else if (found < 2) {
			gpi.match[found] = m;
			gpi.old_smr[found] = smr;
			gpi.old_s2cr[found] = s2cr;
			found++;
		}
	}
	for (gpi.context = 0; gpi.context < contexts && used[gpi.context]; gpi.context++) {
	}
	if (found < 2 || gpi.context >= contexts) {
		lck_mtx_unlock(&hakios_smmu_lock);
		say("s3908: gpi: the SMMU has no match register or no context left\n");
		return false;
	}
	gpi.smmu_second = ml_io_map(hakios_board()->smmu.apps + page, 0x1000);
	gpi.smmu_context = ml_io_map(hakios_board()->smmu.apps + (pages + gpi.context) * page, 0x1000);
	if (gpi.smmu_second == 0 || gpi.smmu_context == 0) {
		lck_mtx_unlock(&hakios_smmu_lock);
		return false;
	}
	gpi.old_sctlr = r32(gpi.smmu_context, SMMU_CB_SCTLR);
	if (gpi.old_sctlr & 1) {
		lck_mtx_unlock(&hakios_smmu_lock);
		say("s3908: gpi: context %u of the SMMU is on although no stream uses it: left alone\n", gpi.context);
		return false;
	}
	if (gpi.tables == 0 &&
	    kmem_alloc(kernel_map, &gpi.tables, 3 * 4096, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		lck_mtx_unlock(&hakios_smmu_lock);
		return false;
	}
	root = (uint64_t *)gpi.tables;
	level2 = (uint64_t *)(gpi.tables + 4096);
	level3 = (uint64_t *)(gpi.tables + 8192);
	bzero(root, 3 * 4096);
	root[GPI_IO >> 30 & 0x1ff] = kvtophys((vm_offset_t)level2) | 3;
	level2[GPI_IO >> 21 & 0x1ff] = kvtophys((vm_offset_t)level3) | 3;
	for (uint32_t p = 0; p < M_SIZE / 4096; p++) {
		level3[(GPI_IO >> 12 & 0x1ff) + p] = (gpi.pa + 4096ull * p) | SMMU_PAGE;
	}
	FlushPoC_DcacheRegion(gpi.tables, 3 * 4096);
	barrier();
	ttbr = kvtophys(gpi.tables) | (uint64_t)gpi.context << 48;
	say("s3908: gpi: the SMMU: context %u (0x%x -> 0x%llx, %u pages), match registers %u and %u for streams 0x%x and 0x%x\n",
	    gpi.context, GPI_IO, gpi.pa, M_SIZE / 4096, gpi.match[0], gpi.match[1], streams[0], streams[1]);
	w32(gpi.smmu_context, SMMU_CB_SCTLR, 0);
	w32(gpi.smmu_second, SMMU_CBA2R(gpi.context), SMMU_CBA2R_64_BIT);
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
			say("s3908: gpi: the SMMU does not finish forgetting old translations\n");
			return false;
		}
		sleep_ms(1);
	}
	w32(gpi.smmu_context, SMMU_CB_SCTLR, SMMU_SCTLR_ON);
	for (unsigned i = 0; i < 2; i++) {
		w32(gpi.smmu, SMMU_S2CR(gpi.match[i]), gpi.context);
		w32(gpi.smmu, SMMU_SMR(gpi.match[i]), 1u << 31 | streams[i]);
	}
	gpi.smmu_on = true;
	for (unsigned i = 0; i < 2; i++) {
		if (r32(gpi.smmu, SMMU_SMR(gpi.match[i])) != (1u << 31 | streams[i]) ||
		    (r32(gpi.smmu, SMMU_S2CR(gpi.match[i])) & 0x300ff) != gpi.context || !(r32(gpi.smmu_context, SMMU_CB_SCTLR) & 1)) {
			say("s3908: gpi: THE SMMU DID NOT TAKE IT: smr 0x%x s2cr 0x%x, sctlr 0x%x\n", r32(gpi.smmu, SMMU_SMR(gpi.match[i])),
			    r32(gpi.smmu, SMMU_S2CR(gpi.match[i])), r32(gpi.smmu_context, SMMU_CB_SCTLR));
			smmu_restore();
			lck_mtx_unlock(&hakios_smmu_lock);
			return false;
		}
	}
	lck_mtx_unlock(&hakios_smmu_lock);
	return true;
}

static int
gpi_state(uint32_t cntxt, uint32_t want)
{
	for (unsigned i = 0; i < 2500; i++) {          /* Linux waits 250 ms for a command */
		if (G_STATE(r32(gpi.gpii, cntxt + CNTXT_0)) == want) {
			return 0;
		}
		spin_us(100);
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
			(void)gpi_ch_cmd(k, G_CMD_STOP, G_STOPPED);
		}
		if (st != G_NOT_ALLOCATED) {
			(void)gpi_ch_cmd(k, G_CMD_RESET, G_ALLOCATED);
			(void)gpi_ch_cmd(k, G_CMD_DE_ALLOC, G_NOT_ALLOCATED);
		}
	}
	if (G_STATE(r32(gpi.gpii, G_EV + CNTXT_0)) != G_NOT_ALLOCATED) {
		(void)gpi_ev_cmd(G_CMD_RESET, G_ALLOCATED);
		(void)gpi_ev_cmd(G_CMD_DE_ALLOC, G_NOT_ALLOCATED);
	}
}

static void
ring(uint32_t cntxt, uint32_t config, uint32_t length, uint64_t at)
{
	w32(gpi.gpii, cntxt + CNTXT_0, config);
	w32(gpi.gpii, cntxt + CNTXT_1_LENGTH, length);
	w32(gpi.gpii, cntxt + CNTXT_2_BASE_LSB, (uint32_t)at);
	w32(gpi.gpii, cntxt + CNTXT_3_BASE_MSB, (uint32_t)(at >> 32));
}

/* the event ring and the two channels (Linux: gpi_ch_init): 0 or < 0 */
static int
gpi_start(void)
{
	int rc;

	gpi_release();
	for (uint32_t i = 0; i < M_SIZE; i += 4) {
		*(volatile uint32_t *)(gpi.mem + i) = 0;
	}
	barrier();
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
	w32(gpi.gpii, G_EV_DB + 4, (uint32_t)((uint64_t)(GPI_IO + M_EV) >> 32));
	w32(gpi.gpii, G_EV + CNTXT_8_INT_MOD, 0);
	w32(gpi.gpii, G_EV + CNTXT_10_MSI_LSB, 0);
	w32(gpi.gpii, G_EV + CNTXT_11_MSI_MSB, 0);
	w32(gpi.gpii, G_EV + CNTXT_12_RP_UPD_LSB, 0);
	w32(gpi.gpii, G_EV + CNTXT_13_RP_UPD_MSB, 0);
	gpi.ev_rp = 0;
	barrier();
	w32(gpi.gpii, G_EV_DB, (uint32_t)(GPI_IO + M_EV + (EVS - 1) * EL));     /* all the ring the engine's */
	for (uint32_t k = 0; k < 2; k++) {
		uint64_t at = GPI_IO + (k ? M_RX : M_TX);

		if ((rc = gpi_ch_cmd(k, G_CMD_ALLOCATE, G_ALLOCATED)) != 0) {
			return rc;
		}
		ring(G_CH(k), EL << 24 | (k ? 0u : 1u) << 3 | 2u, TRES * EL, at);      /* out for 0, in for 1; event ring 0 */
		w32(gpi.gpii, G_CH_DB(k) + 4, (uint32_t)(at >> 32));
		w32(gpi.gpii, G_SCRATCH(k, 0), (1u - k) << 16 | 3u << 4 | WIRED->se_index);      /* pair, I2C, the engine */
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
	volatile uint32_t *tre = (volatile uint32_t *)(gpi.mem + at);

	tre[0] = d0;
	tre[1] = d1;
	tre[2] = d2;
	tre[3] = d3;
	*wp = (*wp + 1) % TRES;
}

static void
doorbell(uint32_t k)
{
	barrier();
	w32(gpi.gpii, G_CH_DB(k), (uint32_t)(GPI_IO + (k ? M_RX : M_TX) + (k ? gpi.rx_wp : gpi.tx_wp) * EL));
}

/* the completion of channel k: 0, -20 (the engine said no), -21 (nothing came in 20 ms) */
static int
gpi_wait(uint32_t k)
{
	uint32_t ev = (uint32_t)(GPI_IO + M_EV);

	for (unsigned n = 0; n < 20000; n++) {          /* a 512-byte read takes ~13 ms at 400 kHz */
		uint32_t hw = ((r32(gpi.gpii, G_EV + CNTXT_4_RP_LSB) - ev) / EL) % EVS;

		if (gpi.ev_rp != hw) {
			barrier();
		}
		while (gpi.ev_rp != hw) {
			volatile uint32_t *event = (volatile uint32_t *)(gpi.mem + M_EV + gpi.ev_rp * EL);
			uint32_t d2, d3;

			for (unsigned i = 0; i < 4; i++) {
				gpi.last_ev[i] = event[i];
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
		spin_us(1);
	}
	return -21;
}

/* back to the wires by hand: the GPII given back, the pins as the run found them */
static void
gpi_off(const char *why, int rc)
{
	if (!gpi.on) {
		return;
	}
	gpi.on = false;
	say("s3908: gpi: %s (rc %d): back to the wires by hand\n", why, rc);
	gpi_release();
	bus_give_back();
}

static int
gpi_transfer(bool read, uint8_t *buffer, uint32_t length)
{
	int rc;

	if (length == 0 || length > M_SIZE - M_BUF) {
		return -2;
	}
	if (!votes_get()) {
		return -22;             /* the DMA engine is not touched without the QUP's way to memory */
	}
	gpi.transfers++;
	if (!read) {
		for (uint32_t i = 0; i < length; i++) {
			gpi.mem[M_BUF + i] = buffer[i];
		}
	} else {
		put_tre(1, GPI_IO + M_BUF, 0, length, TRE_DMA | TRE_IEOT);
		doorbell(1);
	}
	put_tre(0, CONFIG0_WORD0, 0, CONFIG0_WORD2, TRE_CONFIG0 | TRE_CHAIN);
	if (read) {
		put_tre(0, GO_READ | (uint32_t)ADDRESS << 8, 0, length, TRE_GO | TRE_LINK);
	} else {
		put_tre(0, GO_WRITE | (uint32_t)ADDRESS << 8, 0, 0, TRE_GO | TRE_CHAIN);
		put_tre(0, GPI_IO + M_BUF, 0, length, TRE_DMA | TRE_IEOT);
	}
	doorbell(0);
	rc = gpi_wait(read ? 1 : 0);
	if (rc == 0 && read) {
		barrier();
		for (uint32_t i = 0; i < length; i++) {
			buffer[i] = gpi.mem[M_BUF + i];
		}
	}
	if (rc != 0) {
		gpi.errors++;
		gpi.votes_held = true;          /* the DMA's end not certain: the way to memory kept */
		if (gpi.restarts++ < 3 || gpi.restarts % 100 == 0) {
			say("s3908: gpi: %s of %u bytes: rc %d, last event %08x %08x %08x %08x, gpii error 0x%x, engine m-irq 0x%x\n",
			    read ? "read" : "write", length, rc, gpi.last_ev[0], gpi.last_ev[1], gpi.last_ev[2], gpi.last_ev[3],
			    r32(gpi.gpii, G_ERROR_LOG), r32(gpi.se, SE_GENI_M_IRQ_STATUS));
		}
		smmu_faults("after a failed transfer");
		w32(gpi.se, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
		w32(gpi.se, SE_GENI_S_IRQ_CLEAR, 0xffffffffu);
		int again = gpi_start();
		if (again != 0) {
			gpi_off("the GPII does not start again", again);
		}
	}
	gpi.last_us = now_us();
	return rc;
}

/* everything, or nothing and the wires by hand */
static bool
gpi_on(void)
{
	const struct hakios_board *b = hakios_board();
	vm_offset_t block = 0;
	uint32_t fw, vote, bits = 0;
	int rc;

	if (bus_wanted != 2 || WIRED->serial_engine == 0) {
		say("s3908: the bus by hand (%s)\n", bus_wanted != 2 ? "touch_bus=0" : "the board names no serial engine for it");
		return false;
	}
	if (b->clocks.gcc == 0 || b->smmu.apps == 0 || b->rpmh.rsc == 0 || b->rpmh.cmd_db == 0) {
		say("s3908: gpi: the SoC file does not say where the GCC, the SMMU or RPMh are: the bus by hand\n");
		return false;
	}
	gpi.se = ml_io_map(WIRED->serial_engine, 0x1000);       /* (only its first page: the second is its protected program) */
	gpi.gcc = ml_io_map(b->clocks.gcc, 0x100000);
	gpi.smmu = ml_io_map(b->smmu.apps, 0x1000);
	gpi.rsc = ml_io_map(b->rpmh.rsc, RSC_SIZE);
	if (gpi.se == 0 || gpi.gcc == 0 || gpi.smmu == 0 || gpi.rsc == 0) {
		say("s3908: gpi: registers could not be mapped: the bus by hand\n");
		return false;
	}
	{
		const struct cmd_db_header *db = (const struct cmd_db_header *)ml_io_map_wcomb(b->rpmh.cmd_db, CMD_DB_SIZE);

		for (unsigned i = 0; i < 3; i++) {
			const struct cmd_db_entry *entry = WIRED->bus_votes[i] != NULL && db != NULL ?
			    hakios_cmd_db_find(db, CMD_DB_SIZE, WIRED->bus_votes[i], CMD_DB_BCM, NULL) : NULL;

			gpi.bcm[i] = entry != NULL ? entry->address : 0;
			if (gpi.bcm[i] == 0) {
				say("s3908: gpi: no BCM %s in the command database: the bus by hand\n",
				    WIRED->bus_votes[i] != NULL ? WIRED->bus_votes[i] : "(none)");
				return false;
			}
		}
	}
	fw = r32(gpi.se, SE_GENI_FW_REVISION);
	if (((fw >> 8) & 0xff) != 3) {
		say("s3908: gpi: the serial engine runs protocol %u, not I2C (fw 0x%x): the bus by hand\n", (fw >> 8) & 0xff, fw);
		return false;
	}
	for (unsigned i = 0; i < 5; i++) {
		bits |= 1u << WIRED->votes[i];
	}
	vote = r32(gpi.gcc, WIRED->vote_register);
	if ((vote & bits) != bits) {
		say("s3908: gpi: voting for the engine's clocks (0x%x -> 0x%x)\n", vote, vote | bits);
		w32(gpi.gcc, WIRED->vote_register, vote | bits);
		barrier();
	}
	for (unsigned i = 0; r32(gpi.gcc, WIRED->clocks[2]) & CBCR_OFF; i++) {
		if (i == 100) {
			say("s3908: gpi: the engine's clock stays off (0x%x): the bus by hand\n", r32(gpi.gcc, WIRED->clocks[2]));
			return false;
		}
		spin_us(10);
	}
	if (r32(gpi.gcc, WIRED->clocks[2] + 8 + RCG_DFSR) & 1) {
		w32(gpi.se, SE_GENI_CLK_SEL, 0);        /* DFS: level 0, as qcom_geni_i2c_conf */
	}
	if (kmem_alloc_contig(kernel_map, &block, M_SIZE, PAGE_MASK, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		say("s3908: gpi: no memory: the bus by hand\n");
		return false;
	}
	FlushPoC_DcacheRegion(block, M_SIZE);
	gpi.pa = kvtophys(block);
	gpi.mem = (volatile uint8_t *)ml_io_map_wcomb((vm_offset_t)gpi.pa, M_SIZE);    /* used only through this, uncached */
	if (gpi.mem == NULL || (gpi.pa >> 30) != ((gpi.pa + M_SIZE - 1) >> 30)) {
		say("s3908: gpi: memory at 0x%llx not usable: the bus by hand\n", gpi.pa);
		return false;
	}
	if (!smmu_on()) {
		return false;
	}
	/* geni_se_select_gpi_mode() */
	w32(gpi.se, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
	w32(gpi.se, SE_GENI_S_IRQ_CLEAR, 0xffffffffu);
	w32(gpi.se, SE_IRQ_EN, 0);
	w32(gpi.se, SE_GENI_M_IRQ_EN, r32(gpi.se, SE_GENI_M_IRQ_EN) & ~(1u << 0 | 1u << 26 | 1u << 27 | 1u << 30));
	w32(gpi.se, SE_GENI_DMA_MODE_EN, 1);
	w32(gpi.se, SE_GSI_EVENT_EN, r32(gpi.se, SE_GSI_EVENT_EN) | 0xfu);
	if (!votes_get()) {
		say("s3908: gpi: the QUP's way to memory could not be voted: the bus by hand\n");
		return false;
	}
	say("s3908: gpi: RPMh: %s 0/1, %s 1/1, %s 1/1\n", WIRED->bus_votes[0], WIRED->bus_votes[1], WIRED->bus_votes[2]);
	/*
	 * The GPII the engine is bound to (dt: qcom,static-gpii-mask). Android's driver takes GPII 0 for SE 4 on negroni
	 * ("gpi_of_dma_xlate: client req gpii:0 ... proto:3 SE:4", 2026-11-11); a GO from GPII 1 for SE 4 reset the phone.
	 */
	gpi.gpii = ml_io_map(WIRED->gpii, 0x4000);
	if (gpi.gpii == 0) {
		return false;
	}
	if ((rc = gpi_start()) != 0) {
		say("s3908: gpi: the GPII does not start: rc %d (ch0 0x%x ch1 0x%x ev 0x%x, error 0x%x): the bus by hand\n", rc,
		    r32(gpi.gpii, G_CH(0) + CNTXT_0), r32(gpi.gpii, G_CH(1) + CNTXT_0), r32(gpi.gpii, G_EV + CNTXT_0),
		    r32(gpi.gpii, G_ERROR_LOG));
		gpi_release();
		return false;
	}
	/* the pins to the engine: its function, 2 mA, pulled up (as they were found recorded first, for the way back) */
	bus_record();
	gpio_configure(PIN_SDA, WIRED->se_function, PULL_UP, 2, false);
	gpio_configure(PIN_SCL, WIRED->se_function, PULL_UP, 2, false);
	(void)rd(GPIO_CFG(PIN_SCL));
	gpi.on = true;
	say("s3908: gpi: started, memory at 0x%llx; the pins are the serial engine's\n", gpi.pa);
	return true;
}

/* ---- the bus: through the GPI, or by hand ------------------------------------------------------------- */

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

#include "s3908_report.h"

static struct s3908_tcm tcm;
static uint8_t message[S3908_MAX_PAYLOAD + 3];

/* One message: 0 = it is in tcm.code / tcm.payload, 1 = nothing waiting, < 0 = the bus or the framing failed */
static int
tcm_read_message(void)
{
	uint32_t length;

	tcm.code = 0;
	tcm.payload_length = 0;
	for (unsigned i = 0; i < 4; i++) {
		tcm.header[i] = 0;
	}
	if ((tcm.bus_error = i2c_read(tcm.header, 4)) != 0) {
		return -1;
	}
	if (tcm.header[0] != S3908_MARKER) {
		return 1;
	}
	tcm.code = tcm.header[1];
	if (tcm.code == S3908_STATUS_IDLE || tcm.code == S3908_STATUS_BUSY) {
		return 1;
	}
	if (tcm.code == S3908_STATUS_CONTINUED) {
		return -6;      /* the rest of an answer not taken whole: read on (tcm_command) */
	}
	length = tcm.header[2] | (uint32_t)tcm.header[3] << 8;
	if (length == 0) {
		return 0;
	}
	if (length > S3908_MAX_PAYLOAD) {
		return -2;
	}
	if ((tcm.bus_error = i2c_read(message, length + 3)) != 0) {
		return -1;
	}
	if (message[0] != S3908_MARKER || message[1] != S3908_STATUS_CONTINUED || message[length + 2] != S3908_PADDING) {
		return -3;
	}
	for (uint32_t i = 0; i < length; i++) {
		tcm.payload[i] = message[2 + i];
	}
	tcm.payload_length = length;
	return 0;
}

/* a command with no payload, and its answer (reports that come in between are dropped) */
static int
tcm_command(uint8_t command)
{
	uint8_t out[3] = { command, 0, 0 };

	if ((tcm.bus_error = i2c_write(out, sizeof(out))) != 0) {
		return -1;
	}
	for (unsigned tries = 0; tries < 200 && !__atomic_load_n(&leaving, __ATOMIC_SEQ_CST); tries++) {
		int rc = tcm_read_message();

		if (rc == -6) {
			/* a stray continuation (seen 2026-11-09: the chip answered late, after an earlier read had given up) */
			wait_ms(1);
			continue;
		}
		if (rc < 0) {
			return rc;
		}
		if (rc == 0 && tcm.code < S3908_REPORT_IDENTIFY) {
			return tcm.code == S3908_STATUS_OK ? 0 : -4;
		}
		wait_ms(1);
	}
	return -5;
}

static int
tcm_get_report_config(void)
{
	int rc = tcm_command(S3908_CMD_GET_REPORT_CONFIG);

	if (rc) {
		return rc;
	}
	if (tcm.payload_length > S3908_MAX_CONFIG) {
		return -2;
	}
	for (uint32_t i = 0; i < tcm.payload_length; i++) {
		tcm.config[i] = tcm.payload[i];
	}
	tcm.config_length = tcm.payload_length;
	return 0;
}

/* ---- touches ------------------------------------------------------------------------------------ */

static struct { bool down; uint32_t x, y; } contact[S3908_MAX_OBJECTS];
static uint32_t flip, trace;
static uint32_t panel_x, panel_y;
static unsigned contacts;
static struct {
	uint32_t looks, messages, reports, positions, errors, bad, downs;
	uint64_t report_us;
} seen;

static void
touch_report(void)
{
	for (unsigned id = 0; id < S3908_MAX_OBJECTS; id++) {
		bool now = id < contacts && tcm.object[id].status != 0;

		if (!now) {
			if (contact[id].down) {
				contact[id].down = false;
				hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
			}
			continue;
		}
		uint32_t px = tcm.object[id].x, py = tcm.object[id].y;
		uint32_t x = (uint32_t)((px >= panel_x ? panel_x : px) * 65535ull / panel_x);
		uint32_t y = (uint32_t)((py >= panel_y ? panel_y : py) * 65535ull / panel_y);

		if (flip & 1) {
			x = 65535 - x;
		}
		if (flip & 2) {
			y = 65535 - y;
		}
		seen.positions++;
		if (!contact[id].down) {
			if (trace && seen.downs++ < 40) {
				say("s3908: finger %u down at %u, %u (of %u x %u)\n", id, px, py, panel_x, panel_y);
			}
			hakios_touch_post(HAKIOS_TOUCH_DOWN, id, x, y);
		} else if (contact[id].x != x || contact[id].y != y) {
			hakios_touch_post(HAKIOS_TOUCH_MOVE, id, x, y);
		}
		contact[id].down = true;
		contact[id].x = x;
		contact[id].y = y;
	}
}

static void
release(void)
{
	for (unsigned id = 0; id < S3908_MAX_OBJECTS; id++) {
		tcm.object[id].status = 0;
	}
	touch_report();
}

/* ---- the chip ------------------------------------------------------------------------------------- */

/* Drain what is waiting (the identify report after a reset, old touches), then learn the report layout. */
static int
chip_setup(void)
{
	int rc = 1;

	for (unsigned i = 0; i < DRAIN_MESSAGES && attention(); i++) {
		if ((rc = tcm_read_message()) < 0) {
			break;
		}
	}
	say("s3908: drained: rc %d, code 0x%x, header %02x %02x %02x %02x, bus %d, attention %s\n", rc, tcm.code,
	    tcm.header[0], tcm.header[1], tcm.header[2], tcm.header[3], tcm.bus_error, attention() ? "low" : "high");
	/* a few tries: an answer can come late and leave a stray piece behind (2026-11-09, first boot from darwin) */
	for (unsigned tries = 0; tries < CONFIG_TRIES && !__atomic_load_n(&leaving, __ATOMIC_SEQ_CST); tries++) {
		rc = tcm_get_report_config();
		say("s3908: report config: rc %d, %u bytes, bus %d (try %u)\n", rc, tcm.config_length, tcm.bus_error, tries + 1);
		if (rc == 0) {
			break;
		}
		wait_ms(50);
		for (unsigned i = 0; i < DRAIN_MESSAGES && attention(); i++) {
			if (tcm_read_message() < 0 && tcm.bus_error) {
				break;
			}
		}
	}
	return rc;
}

static void
reset_pulse(void)
{
	say("s3908: reset: gpio%u low %u ms, then %u ms for the chip's firmware\n", PIN_RESET, RESET_LOW_MS, RESET_WAIT_MS);
	gpio_output(PIN_RESET, false, PULL_NONE, 2);
	wait_ms(RESET_LOW_MS);
	gpio_set(PIN_RESET, true);
	wait_ms(RESET_WAIT_MS);
}

static bool
begin(void)
{
	__atomic_store_n(&busy, true, __ATOMIC_SEQ_CST);
	if (__atomic_load_n(&leaving, __ATOMIC_SEQ_CST)) {
		__atomic_store_n(&busy, false, __ATOMIC_SEQ_CST);
		return false;
	}
	return true;
}

static void
done(void)
{
	__atomic_store_n(&busy, false, __ATOMIC_SEQ_CST);
}

static bool looking_only;

/* One look; false: the chip is given up. */
static bool
poll_once(unsigned tick)
{
	static unsigned errors_in_a_row, resets;

	for (unsigned n = 0; n < MESSAGES_A_LOOK && attention(); n++) {
		uint64_t start = now_us();
		int rc = tcm_read_message();

		seen.looks++;
		if (rc < 0) {
			seen.errors++;
			if (seen.errors <= 4) {
				say("s3908: a message not read: rc %d, bus %d, header %02x %02x %02x %02x\n", rc, tcm.bus_error,
				    tcm.header[0], tcm.header[1], tcm.header[2], tcm.header[3]);
			}
			release();
			if (++errors_in_a_row >= TROUBLE_ERRORS) {
				say("s3908: %u reads in a row failed; resets made for that before: %u of %u\n", errors_in_a_row, resets,
				    TROUBLE_RESETS);
				if (resets++ >= TROUBLE_RESETS) {
					return false;
				}
				reset_pulse();
				if (chip_setup() != 0) {
					return false;
				}
				errors_in_a_row = 0;
			}
			break;
		}
		errors_in_a_row = 0;
		seen.messages++;
		if (rc == 0 && tcm.code == S3908_REPORT_TOUCH) {
			seen.report_us += now_us() - start;
			seen.reports++;
			if (s3908_parse_touch(&tcm) == 0) {
				touch_report();
			} else {
				seen.bad++;
			}
		}
	}
	/* every 5 s for the first two minutes, then every minute */
	static uint64_t first_us, next_us;
	uint64_t now = now_us();

	(void)tick;
	if (first_us == 0) {
		first_us = now;
		next_us = now + 5000000;
	}
	if (now >= next_us) {
		next_us = now + (now - first_us < 120000000 ? 5000000 : 60000000);
		say("s3908: %u messages (%u touch reports at %llu us each, %u not parsed), %u finger positions, %u errors; "
		    "attention %s; bus %s (%u transfers, %u failed, votes %s); interrupts %s: %u, %u by the line, %u nets\n",
		    seen.messages, seen.reports, seen.reports ? seen.report_us / seen.reports : 0, seen.bad, seen.positions,
		    seen.errors, attention() ? "low" : "high", gpi.on ? "gpi" : "by hand", gpi.transfers, gpi.errors,
		    gpi.votes_on ? "on" : "off", irq.on ? "on" : pdc != 0 ? "OFF (a storm)" : "none", irq.count, irq.by_line, irq.nets);
	}
	return true;
}

/*
 * Until the chip has something, or the net. The bus votes are let go here, with nothing waiting at the chip and no
 * transfer for idle_ms.
 */
static void
touch_wait(void)
{
	uint32_t ms = irq.on ? IRQ_NET_MS : POLL_MS;

	if (gpi.votes_on && !gpi.votes_held && idle_ms != 0 && !attention()) {
		uint64_t quiet = now_us() - gpi.last_us;

		if (quiet >= idle_ms * 1000ull) {
			(void)votes_set(false);
		} else if (ms > idle_ms - quiet / 1000) {
			ms = (uint32_t)(idle_ms - quiet / 1000) + 1;
		}
	}
	if (!irq.on) {
		if (pdc != 0 && !irq.said_storm) {
			irq.said_storm = true;
			say("s3908: MORE THAN %u INTERRUPTS IN A SECOND: the line is left masked; looked at every %u ms\n", IRQ_STORM,
			    POLL_MS);
		}
		wait_ms(ms);
		return;
	}
	/* re-armed only now that what was waiting has been read: a line still low rings again at once */
	wr(INTR_STATUS(PIN_ATTN), 0);
	gic_v3_spi_unmask(PDC_INTID);
	/* (with the display off the net may go off later, with another wake of its processor: arch/arm64/kernel/psci_smp.c) */
	(void)hakios_assert_wait_net((event_t)&irq.rang, ms);
	if (os_atomic_load(&irq.rang, acquire) != 0) {
		clear_wait(current_thread(), THREAD_AWAKENED);
	} else {
		(void)thread_block(THREAD_CONTINUE_NULL);
	}
	if (os_atomic_xchg(&irq.rang, 0, acquire) == 0) {
		irq.nets++;
	} else if (attention()) {
		irq.by_line++;
	}
}

static void
s3908_thread(__unused void *parameter, __unused wait_result_t result)
{
	int rc;

	tlmm = ml_io_map(TLMM_PHYS, TLMM_SIZE);
	if (tlmm == 0) {
		say("s3908: the pins could not be mapped\n");
		thread_terminate(current_thread());
		return;
	}
	if (!begin()) {
		thread_terminate(current_thread());
		return;
	}
	show_pins();
	if (looking_only) {
		say("s3908: touchscreen=-1: the chip asked for its report config as the firmware left it\n");
		(void)chip_setup();
		say("s3908: touchscreen=-1: nothing reset or configured\n");
		done();
		thread_terminate(current_thread());
		return;
	}
	if (gpi_on()) {
		uint64_t start = now_us();

		rc = chip_setup();
		if (rc != 0) {
			gpi_off("no report config through the serial engine", rc);
		} else {
			say("s3908: gpi: the chip set up in %llu us\n", now_us() - start);
		}
	}
	if (!gpi.on) {
		rc = chip_setup();
		if (rc != 0 && !__atomic_load_n(&leaving, __ATOMIC_SEQ_CST)) {
			reset_pulse();
			rc = chip_setup();
		}
	}
	if (rc != 0) {
		say("s3908: no report config from the chip at 0x%02x: touch screen unavailable\n", ADDRESS);
		done();
		thread_terminate(current_thread());
		return;
	}
	irq_start();
	done();
	say("s3908: running, %u x %u, %u fingers, over %s; the attention line (gpio%u) %s\n", panel_x, panel_y, contacts,
	    gpi.on ? "the serial engine (GPI DMA)" : "the wires by hand", PIN_ATTN,
	    irq.on ? "wakes the thread through the PDC" : "is looked at every 8 ms");
	for (unsigned tick = 1; begin(); tick++) {
		if (!poll_once(tick)) {
			release();
			say("s3908: stopped: the chip does not come back\n");
			done();
			break;
		}
		done();
		touch_wait();
	}
	thread_terminate(current_thread());
}

void
hakios_s3908_init(void)
{
	const struct hakios_board *b = hakios_board();
	int32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("touchscreen", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	looking_only = enabled == -1;
	if (b == NULL || b->pins.tlmm == 0) {
		say("s3908: disabled: the SoC file does not say where the pins are (pins { })\n");
		return;
	}
	if (WIRED->sda == WIRED->scl || WIRED->address == 0 || WIRED->interrupt == WIRED->reset) {
		say("s3908: disabled: the board file does not say how the controller is wired (touch-i2c { })\n");
		return;
	}
	contacts = WIRED->contacts;
	panel_x = WIRED->max_x;
	panel_y = WIRED->max_y;
	if (contacts == 0 || contacts > S3908_MAX_OBJECTS || panel_x == 0 || panel_y == 0) {
		say("s3908: disabled: touch-i2c { } contacts %u and coordinates %u x %u: 1 to %u fingers, coordinates not 0\n",
		    contacts, panel_x, panel_y, S3908_MAX_OBJECTS);
		return;
	}
	i2c_bus.sda = PIN_SDA;
	i2c_bus.scl = PIN_SCL;
	PE_parse_boot_argn("touch_flip", &flip, sizeof(flip));
	PE_parse_boot_argn("s3908_trace", &trace, sizeof(trace));
	PE_parse_boot_argn("touch_bus", &bus_wanted, sizeof(bus_wanted));
	PE_parse_boot_argn("touch_irq", &irq_wanted, sizeof(irq_wanted));
	PE_parse_boot_argn("touch_idle_ms", &idle_ms, sizeof(idle_ms));
	if (idle_ms > 60000) {
		idle_ms = 60000;
	}
	if (looking_only) {
		bus_wanted = 0;                 /* (touchscreen=-1 configures nothing: the bus by hand, polled) */
		irq_wanted = 0;
	}
	__atomic_store_n(&started, true, __ATOMIC_SEQ_CST);
	if (kernel_thread_start(s3908_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		__atomic_store_n(&started, false, __ATOMIC_SEQ_CST);
		say("s3908: cannot start the touch screen's thread\n");
	}
}

/*
 * At the run's end and on a panic's way out (arch/arm64/kernel/loader_return.c): no more polling, a transaction cut
 * off ended and the bus's pins given back as the run found them. The chip keeps running for the firmware.
 */
void
hakios_s3908_quiesce(void)
{
	unsigned clocks = 0;

	if (!__atomic_load_n(&started, __ATOMIC_SEQ_CST)) {
		return;
	}
	__atomic_store_n(&leaving, true, __ATOMIC_SEQ_CST);
	for (unsigned i = 0; i < 100 && __atomic_load_n(&busy, __ATOMIC_SEQ_CST); i++) {
		spin_us(1000);
	}
	if (tlmm == 0) {
		return;
	}
	if (pdc != 0) {
		irq.on = false;
		gic_v3_spi_mask(PDC_INTID);
		irq_restore();
	}
	if (gpi.gpii != 0 && gpi.on) {
		/*
		 * (the other cores are stopped, perhaps one in the SMMU's lock: not taken). The GPII's registers only with the
		 * QUP's way voted on, as for every transfer: let go when idle, a read of them stalls the core for good
		 * (2026-11-24, run-20261124-190926: the way back hung, the watchdog reset the phone).
		 */
		if (!gpi.votes_on && !votes_set(true)) {
			say("s3908: at the run's end: the QUP's way could not be voted: the GPII left as it is\n");
			return;
		}
		gpi_release();
		gpi.on = false;
		if (gpi.smmu_on) {
			smmu_restore();
		}
		bus_give_back();
		if (gpi.votes_on && !gpi.votes_held) {
			(void)votes_set(false);
		}
		say("s3908: at the run's end: %u transfers through the GPI (%u failed); the GPII, the SMMU and the pins as found\n",
		    gpi.transfers, gpi.errors);
	}
	if (!__atomic_load_n(&on_bus, __ATOMIC_SEQ_CST)) {
		return;
	}
	i2c_gpio_stop(&i2c_bus);
	bus_give_back();
	if (!i2c_gpio_level(PIN_SDA) || !i2c_gpio_level(PIN_SCL)) {
		bool came_back = i2c_gpio_unwedge(&i2c_bus, &clocks);

		bus_give_back();
		say("s3908: at the run's end: a transfer cut off: %s after %u clocks\n",
		    came_back ? "the bus came back" : "THE I2C BUS IS NOT AT REST", clocks);
	}
}
