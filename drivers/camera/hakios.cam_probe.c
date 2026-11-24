/*
 * OnePlus 10 Pro (NE2213) rear IMX789 chip-ID diagnostic. The sensor
 * settings are from the stock NE2213 CamX module and live device tree.
 * SM8450 CAMCC, GDSC and CCI transactions follow iOSPlus10 patches 0310
 * and 0311. The CCI runs at 100 kHz from a 19.2 MHz TCXO source.
 *
 * cam_probe=N  Rear IMX789 chip ID at 1, then IFE test-pattern capture at 2, then
 * IMX789 mode 1 (4000 x 3008 RAW10, three-trio C-PHY on CSIPHY1) at 3, then /dev/hakios-camera
 * (sys/hakios.camera.h) streaming that mode at 4; at 5, before serving, IFE0's pixel path (CSID0 IPP, the stock
 * CamX init packet, 640 x 480 NV12 FD output) in three sessions. Level 6 tests native-size FULL NV12.
 * Level 7 runs levels 1-3, then streams mode 0 (8000 x 6016 RAW10) into RDI0 only, and serves nothing.
 * The SM8450 capture
 * sequence follows iOSPlus10 patches 0312, 0313 and 0315; the CSIPHY 2.1.0 tables, CamCC PLLs
 * and CSID 680 RX fields were checked against the OnePlus SM8450 kernel and camera-kernel
 * sources, and the sensor tables come from the NE2213's own CamX module (camsensor.py).
 *
 * cam_ife_preview=1  Serve IFE FULL NV12 alongside RAW10 on ordinary camera boots.
 * cam_ife_preview=2  Serve native-size FULL NV12 for still capture as well as preview.
 * iris=1  Serve Iris2 still encoding on /dev/hakios-iris with cam=5 (hakios.iris_probe.c level 7).
 * iris_probe=N  Iris2 diagnostic levels 1-6 with cam=0 (hakios.iris_probe.c); 7 is the iris=1 service.
 * icp_probe=N  Camera ICP diagnostic with cam=0 (hakios.icp_probe.c): 1 power, 2 firmware, SYS_INIT and ping, 3 start and stop, 4 timed windows.
 *
 * Copyright (c) 2026 iOSPlus10 contributors (MIT, see THIRD_PARTY_LICENSES.md).
 */
#include <sys/param.h>
#include <sys/conf.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/uio.h>
#include <sys/hakios.camera.h>
#include <sys/hakios.input.h>
#include <sys/hakios.smmu.h>
#include <arm/caches_internal.h>
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <machine/machine_routines.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>
#include <pexpert/arm64/board_config.h>
#include <vm/pmap.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys, vm_size_t size, unsigned flags);
extern void ml_io_unmap(vm_offset_t addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t address);
extern void loader_log_line(const char *line);

static void cam_log(const char *format, ...) __printflike(1, 2);

static void
cam_log(const char *format, ...)
{
	char line[256];
	va_list args;
	va_start(args, format);
	vsnprintf(line, sizeof(line), format, args);
	va_end(args);
	loader_log_line(line);
}

#define GCC_PA          0x00136000UL
#define CAMCC_PA        0x0ade0000UL
#define CAMCC_SIZE      0x20000UL
#define CCI_PA          0x0ac15000UL
#define TLMM_PA         0x0f164000UL /* GPIO100 at TLMM base + 0x100000 + 100 * 0x1000 */
#define TLMM_SIZE       0x15000UL
#define GDSC_CLK        0x1320c
#define TITAN_GDSCR     0x131dc

#define CLK_ENABLE      (1u << 0)
#define CLK_HWCG        (1u << 1)
#define CLK_OFF         (1u << 31)
#define RCG_UPDATE      (1u << 0)
#define RCG_ROOT_ENABLE (1u << 1)
#define RCG_RATE_MASK   (0x1fu | (7u << 8) | (3u << 12))
#define GDSC_COLLAPSE   (1u << 0)
#define GDSC_HW_CONTROL (1u << 1)
#define GDSC_RETAIN     (1u << 11)
#define GDSC_POWERED    (1u << 31)

#define CCI_RESET          0x004
#define CCI_START          0x008
#define CCI_HALT           0x034
#define CCI_M0_READ_DATA   0x118
#define CCI_M0_READ_LEVEL  0x11c
#define CCI_M0_Q1_EXEC     0x400
#define CCI_M0_Q1_COUNT    0x404
#define CCI_M0_Q1_LOAD     0x410
#define CCI_IRQ_CLEAR      0xc00
#define CCI_IRQ_MASK_0     0xc04
#define CCI_IRQ_CLEAR_0    0xc08
#define CCI_IRQ_STATUS_0   0xc0c
#define CCI_IRQ_MASK_1     0xc10
#define CCI_IRQ_CLEAR_1    0xc14
#define CCI_MASK_0         0x7fff7ff7u
#define CCI_MASK_1         0x00110000u
#define CCI_RESET_DONE     (1u << 24)
#define CCI_HALT_ACK       (1u << 25)
#define CCI_M0_READ_DONE   1u
#define CCI_M0_ERROR       0x18000ee6u
#define CCI_TIMEOUT_US     100000u

#define GPIO_CFG_MASK      0x3ffu
#define GPIO_OUT           (1u << 9)
#define GPIO_EGPIO_PRESENT (1u << 11)
#define GPIO_EGPIO_ENABLE  (1u << 12)
#define GPIO_IO_OUT        (1u << 1)

/*
 * NE2213 msm_cam_smmu_ife: its eight iommus tuples describe the same
 * group once ignored bits are removed. The 32-bit IOVA starts inside
 * the IFE DT pool [0x00100000, 0xfff00000). Allow a second RAW10 frame
 * of headroom while halting the write master.
 */
#define IFE_SID            0x800u
#define IFE_SID_MASK       0x460u
#define IFE_IOVA           0x10c00000ull
#define TPG_WIDTH          640u
#define TPG_HEIGHT         480u
#define TPG_STRIDE         (TPG_WIDTH * 10u / 8u)
#define TPG_FRAME_BYTES    (TPG_STRIDE * TPG_HEIGHT)
#define TPG_FILL           0xa5

/* SM8450 CPAS 680.110, CSID 680, IFE bus v3 and TPG v1.3. Physical
 * bases and spans match the NE2213 live device tree. */
#define CPAS_PA            0x0ac13000UL
#define CAMNOC_PA          0x0ac19000UL
#define CAMNOC_SIZE        0x9000UL
#define CSID_TOP_PA        0x0acb6000UL
#define CSID_PA            0x0acb7000UL
#define IFE_PA             0x0ac62000UL
#define IFE_SIZE           0xf000UL
#define TPG_PA             0x0acf6000UL
#define IFE_GDSCR          0x11004

#define CSID_RESET_CFG     0x00c
#define CSID_RESET_CMD     0x010
#define CSID_IRQ_CMD       0x014
#define CSID_RUP_AUP       0x018
#define CSID_TOP_STATUS    0x07c
#define CSID_TOP_MASK      0x080
#define CSID_TOP_CLEAR     0x084
#define CSID_BUF_STATUS    0x08c
#define CSID_BUF_MASK      0x090
#define CSID_BUF_CLEAR     0x094
#define CSID_RX_STATUS     0x09c
#define CSID_RX_MASK       0x0a0
#define CSID_RX_CLEAR      0x0a4
#define CSID_RDI_STATUS    0x0ec
#define CSID_RDI_MASK      0x0f0
#define CSID_RDI_CLEAR     0x0f4
#define CSID_RX_CFG0       0x200
#define CSID_RX_CFG1       0x204
#define CSID_RX_CAPTURE    0x208
#define CSID_RDI_CFG0      0x500
#define CSID_RDI_CTRL      0x504
#define CSID_RDI_CFG1      0x510
#define RDI0_BUF_DONE      (1u << 14)
#define RDI_ENABLE         (1u << 31)

#define IFE_WM_RDI0        0x2600
#define WM_CFG             0x00
#define WM_ADDRESS         0x04
#define WM_FRAME_INCR      0x08
#define WM_IMAGE_CFG0      0x0c
#define WM_IMAGE_CFG2      0x14
#define WM_PACKER_CFG      0x18
#define WM_ENABLE          1u

#define TPG_CTRL           0x064
#define TPG_STATUS         0x004
#define TPG_IRQ_STATUS     0x1e0
#define TPG_IRQ_MASK       0x1e4
#define TPG_IRQ_CLEAR      0x1e8
#define TPG_IRQ_CMD        0x1f0
#define TPG_RESET          0x1f4
#define TPG_ENABLE         1u
#define DT_RAW10           0x2bu

static const struct {
	const char *name;
	uint32_t mv;
	int mode;
} rails[] = {
	{ "ldoi3", 2800, -1 }, /* VANA */
	{ "ldoi1", 1104, 7 },  /* VDIG, PMIC5 LDO HPM */
	{ "ldoc2", 1800, 7 },  /* VIO */
	{ "ldoi7", 2800, -1 }, /* VAF */
	{ "ldoi5", 1800, 7 },  /* CUSTOM_REG1 */
};
static const unsigned rail_shutdown[] = { 3, 4, 2, 1, 0 };

static const uint32_t gcc_gates[] = { 0x004, 0x020, 0x010, 0x018 };
/* SM8450 CCI0 clocks: CPAS AHB, CAMNOC AXI and CCI0 (Linux sm8450.dtsi). */
static const uint32_t cam_gates[] = { 0x13164, 0x131ac, 0x13144 };
static const uint32_t cam_sources[] = { 0x10034, 0x13194, 0x1312c };
static const unsigned pins[] = { 100, 110, 111, 120 };
enum { MCLK_PIN, SDA_PIN, SCL_PIN, RESET_PIN };

struct clock_source {
	uint32_t rate;
	bool changed;
};

struct cam_probe {
	vm_offset_t gcc, camcc, cci, tlmm;
	uint32_t gcc_owned, cam_owned;
	uint32_t gdsc_saved, irq0_saved, irq1_saved;
	bool gdsc_gate_owned, gdsc_changed, cci_started, mclk_owned, reset_high;
	struct clock_source sources[sizeof(cam_sources) / sizeof(cam_sources[0])], mclk_source;
	struct hakios_cam_rail rails[sizeof(rails) / sizeof(rails[0])];
	struct {
		uint32_t cfg, io;
		bool changed;
	} pins[sizeof(pins) / sizeof(pins[0])];
};

static uint32_t
read_reg(vm_offset_t reg)
{
	uint32_t value = *(volatile uint32_t *)reg;
	__asm__ volatile("dmb oshld" ::: "memory");
	return value;
}

static void
write_reg(vm_offset_t reg, uint32_t value)
{
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)reg = value;
	__asm__ volatile("dsb osh" ::: "memory");
}

/*
 * Short waits spin on the clock. delay() here costs milliseconds even for 1 us (level 4 boot 3:
 * 4.4 s to bring the sensor path up, nearly all of it in delay(1) polls and short delays).
 */
static void
spin_us(uint32_t us)
{
	uint64_t end;
	nanoseconds_to_absolutetime((uint64_t)us * NSEC_PER_USEC, &end);
	end += mach_absolute_time();
	while (mach_absolute_time() < end) {
	}
}

static bool
poll_reg(vm_offset_t reg, uint32_t mask, uint32_t expected, unsigned timeout)
{
	for (unsigned i = 0; i < timeout; i++) {
		if ((read_reg(reg) & mask) == expected) {
			return true;
		}
		spin_us(10); /* timeouts in tens of microseconds: margin over what delay(1) used to give */
	}
	return (read_reg(reg) & mask) == expected;
}

static bool
gate_set(vm_offset_t reg, bool on, bool wait)
{
	uint32_t initial = read_reg(reg);
	if (initial == UINT32_MAX) {
		return false;
	}
	write_reg(reg, on ? initial | CLK_ENABLE : initial & ~CLK_ENABLE);
	spin_us(10);
	uint32_t mask = CLK_ENABLE;
	if (on && wait && !(initial & CLK_HWCG)) {
		mask |= CLK_OFF;
	}
	bool ok = poll_reg(reg, mask, on ? CLK_ENABLE : 0, 500);
	if (!ok) {
		cam_log("cam_probe: gate readback %#x -> %#x (wanted enable=%u)\n",
		    initial, read_reg(reg), (unsigned)on);
	}
	return ok;
}

static bool
gate_acquire(vm_offset_t reg, bool wait, bool *owned)
{
	uint32_t initial = read_reg(reg);
	if (initial == UINT32_MAX) {
		return false;
	}
	if (initial & CLK_ENABLE) {
		return !wait || (initial & CLK_HWCG) || !(initial & CLK_OFF);
	}
	*owned = true;
	return gate_set(reg, true, wait);
}

static bool
gates_acquire(vm_offset_t base, const uint32_t *regs, unsigned count, unsigned skip_wait, uint32_t *owned)
{
	for (unsigned i = 0; i < count; i++) {
		bool acquired = false;
		bool ok = gate_acquire(base + regs[i], !(skip_wait & (1u << i)), &acquired);
		if (acquired) {
			*owned |= 1u << i;
		}
		if (!ok) {
			cam_log("cam_probe: clock gate %#x did not start\n", regs[i]);
			return false;
		}
	}
	return true;
}

static bool
gates_release(vm_offset_t base, const uint32_t *regs, unsigned count, uint32_t owned)
{
	bool ok = true;
	for (unsigned i = count; i-- > 0;) {
		if ((owned & (1u << i)) && !gate_set(base + regs[i], false, false)) {
			cam_log("cam_probe: clock gate %#x did not stop\n", regs[i]);
			ok = false;
		}
	}
	return ok;
}

static bool
source_set(vm_offset_t cmd, uint32_t rate)
{
	uint32_t initial = read_reg(cmd);
	write_reg(cmd + 4, (read_reg(cmd + 4) & ~RCG_RATE_MASK) | rate);
	write_reg(cmd, initial | RCG_ROOT_ENABLE);
	write_reg(cmd, initial | RCG_ROOT_ENABLE | RCG_UPDATE);
	bool updated = poll_reg(cmd, RCG_UPDATE, 0, 500);
	write_reg(cmd, (read_reg(cmd) & ~(RCG_ROOT_ENABLE | RCG_UPDATE)) | (initial & RCG_ROOT_ENABLE));
	return updated && (read_reg(cmd + 4) & RCG_RATE_MASK) == rate;
}

static bool
source_acquire(vm_offset_t cmd, struct clock_source *saved)
{
	saved->rate = read_reg(cmd + 4) & RCG_RATE_MASK;
	if (saved->rate == 0) {
		return true;
	}
	saved->changed = true;
	return source_set(cmd, 0); /* TCXO, divider and M/N disabled */
}

static bool
cam_clocks_up(struct cam_probe *cam)
{
	cam_log("cam_probe: enabling GCC camera clocks\n");
	if (!gates_acquire(cam->gcc, gcc_gates, 4, 0xc, &cam->gcc_owned)) {
		return false;
	}
	cam_log("cam_probe: enabling CAMCC GDSC clock\n");
	if (!gate_acquire(cam->camcc + GDSC_CLK, false, &cam->gdsc_gate_owned)) {
		return false;
	}
	vm_offset_t reg = cam->camcc + TITAN_GDSCR;
	cam->gdsc_saved = read_reg(reg);
	cam_log("cam_probe: Titan top GDSCR %#x\n", cam->gdsc_saved);
	if (cam->gdsc_saved == UINT32_MAX || (cam->gdsc_saved & GDSC_HW_CONTROL)) {
		return false;
	}
	if (!(cam->gdsc_saved & GDSC_POWERED)) {
		cam->gdsc_changed = true;
		write_reg(reg, cam->gdsc_saved & ~GDSC_COLLAPSE);
		if (!poll_reg(reg, GDSC_POWERED, GDSC_POWERED, 500)) {
			return false;
		}
	}
	if (!(read_reg(reg) & GDSC_RETAIN)) {
		cam->gdsc_changed = true;
		write_reg(reg, read_reg(reg) | GDSC_RETAIN);
	}
	/* CCI reset and a clock-source change require an idle camera path. */
	for (unsigned i = 0; i < sizeof(cam_gates) / sizeof(cam_gates[0]); i++) {
		if (read_reg(cam->camcc + cam_gates[i]) & CLK_ENABLE) {
			cam_log("cam_probe: camera gate %#x already enabled\n", cam_gates[i]);
			return false;
		}
	}
	if (read_reg(cam->camcc + 0x15018) & CLK_ENABLE) {
		cam_log("cam_probe: MCLK0 already enabled\n");
		return false;
	}
	for (unsigned i = 0; i < sizeof(cam_sources) / sizeof(cam_sources[0]); i++) {
		if (!source_acquire(cam->camcc + cam_sources[i], &cam->sources[i])) {
			cam_log("cam_probe: clock source %#x did not update\n", cam_sources[i]);
			return false;
		}
	}
	return gates_acquire(cam->camcc, cam_gates, sizeof(cam_gates) / sizeof(cam_gates[0]), 0, &cam->cam_owned);
}

static void
cci_clear(vm_offset_t cci, uint32_t bits)
{
	write_reg(cci + CCI_IRQ_CLEAR_0, bits);
	write_reg(cci + CCI_IRQ_CLEAR, 1);
}

static bool
cci_reset(vm_offset_t cci, bool all)
{
	cci_clear(cci, CCI_MASK_0);
	write_reg(cci + CCI_RESET, all ? 0x0f73f3f7u : 0x3f1u);
	if (all) {
		write_reg(cci + CCI_RESET, 1);
	}
	bool ok = poll_reg(cci + CCI_IRQ_STATUS_0, CCI_RESET_DONE, CCI_RESET_DONE, CCI_TIMEOUT_US);
	cci_clear(cci, CCI_RESET_DONE);
	return ok;
}

static bool
cci_start(struct cam_probe *cam)
{
	vm_offset_t cci = cam->cci;
	uint32_t version = read_reg(cci);
	cam_log("cam_probe: CCI0 version %#x\n", version);
	if (version == 0 || version == UINT32_MAX) {
		return false;
	}
	cam->irq0_saved = read_reg(cci + CCI_IRQ_MASK_0);
	cam->irq1_saved = read_reg(cci + CCI_IRQ_MASK_1);
	cam->cci_started = true;
	write_reg(cci + CCI_IRQ_MASK_0, CCI_MASK_0);
	if (!cci_reset(cci, true)) {
		return false;
	}
	write_reg(cci + CCI_IRQ_MASK_0, CCI_MASK_0);
	write_reg(cci + CCI_IRQ_MASK_1, CCI_MASK_1);
	write_reg(cci + CCI_M0_READ_LEVEL + 4, 0x30); /* read threshold */
	return true;
}

static void
cci_timing(vm_offset_t cci)
{
	write_reg(cci + 0x100, 78u << 16 | 114u);
	write_reg(cci + 0x104, 28u << 16 | 28u);
	write_reg(cci + 0x108, 10u << 16 | 77u);
	write_reg(cci + 0x10c, 118);
	write_reg(cci + 0x110, 1u << 8 | 6u << 4 | 1u); /* SCL stretch, hold and spike widths */
}

static bool
cci_chip_id(vm_offset_t cci)
{
	const uint32_t commands[] = {
		0x1a1,    /* SET_PARAM, 7-bit slave address 0x1a */
		6,        /* LOCK */
		0x16002b, /* WRITE_DISABLE_P, 16-bit register 0x0016, no STOP */
		0x2a,     /* READ two bytes */
		7,        /* UNLOCK */
	};
	for (unsigned attempt = 0; attempt < 3; attempt++) {
		if (read_reg(cci + CCI_M0_Q1_COUNT)) {
			return false;
		}
		cci_clear(cci, CCI_MASK_0);
		for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
			write_reg(cci + CCI_M0_Q1_LOAD, commands[i]);
		}
		write_reg(cci + CCI_M0_Q1_EXEC, read_reg(cci + CCI_M0_Q1_COUNT));
		write_reg(cci + CCI_START, 1u << 1);
		uint32_t status = 0;
		for (unsigned i = 0; i < CCI_TIMEOUT_US; i++) {
			status = read_reg(cci + CCI_IRQ_STATUS_0);
			if (status & (CCI_M0_READ_DONE | CCI_M0_ERROR)) {
				break;
			}
			spin_us(1);
		}
		if ((status & (CCI_M0_READ_DONE | CCI_M0_ERROR)) == CCI_M0_READ_DONE &&
		    read_reg(cci + CCI_M0_READ_LEVEL) == 1) {
			uint32_t data = read_reg(cci + CCI_M0_READ_DATA);
			cci_clear(cci, status);
			uint32_t id = ((data >> 8) & 0xffu) << 8 | ((data >> 16) & 0xffu);
			cam_log("cam_probe: IMX789 chip ID 0x%04x (expected 0x0789)\n", id);
			return id == 0x0789;
		}
		cam_log("cam_probe: CCI read attempt %u, IRQ status %#x\n", attempt + 1, status);
		cci_clear(cci, status);
		write_reg(cci + CCI_HALT, 1);
		(void)poll_reg(cci + CCI_IRQ_STATUS_0, CCI_HALT_ACK, CCI_HALT_ACK, CCI_TIMEOUT_US);
		if (!cci_reset(cci, false)) {
			return false;
		}
		cci_timing(cci);
	}
	return false;
}

static vm_offset_t
pin_reg(const struct cam_probe *cam, unsigned index)
{
	return cam->tlmm + (pins[index] - 100) * 0x1000u;
}

static void
pin_acquire(struct cam_probe *cam, unsigned index, uint32_t cfg)
{
	vm_offset_t reg = pin_reg(cam, index);
	cam->pins[index].cfg = read_reg(reg);
	cam->pins[index].io = read_reg(reg + 4);
	cam->pins[index].changed = true;
	if (cfg & GPIO_OUT) {
		write_reg(reg + 4, 0);
	}
	uint32_t saved = cam->pins[index].cfg;
	uint32_t egpio = saved & GPIO_EGPIO_PRESENT ? GPIO_EGPIO_ENABLE : 0;
	write_reg(reg, (saved & ~GPIO_CFG_MASK) | egpio | cfg);
}

static bool
pin_restore(struct cam_probe *cam, unsigned index)
{
	if (!cam->pins[index].changed) {
		return true;
	}
	vm_offset_t reg = pin_reg(cam, index);
	write_reg(reg + 4, cam->pins[index].io & GPIO_IO_OUT);
	write_reg(reg, cam->pins[index].cfg);
	const uint32_t mask = GPIO_CFG_MASK | GPIO_EGPIO_ENABLE;
	bool ok = (read_reg(reg) & mask) == (cam->pins[index].cfg & mask) &&
	    (read_reg(reg + 4) & GPIO_IO_OUT) == (cam->pins[index].io & GPIO_IO_OUT);
	if (!ok) {
		cam_log("cam_probe: GPIO%u restore failed\n", pins[index]);
	}
	return ok;
}

static bool
sensor_probe(struct cam_probe *cam)
{
	pin_acquire(cam, RESET_PIN, GPIO_OUT);
	pin_acquire(cam, MCLK_PIN, 1u << 2); /* cam_mclk, no pull, 2 mA */
	for (unsigned i = 0; i < sizeof(rails) / sizeof(rails[0]); i++) {
		uint64_t t0 = mach_absolute_time(), ns;
		int error = hakios_cam_rail_vote(rails[i].name, rails[i].mv, rails[i].mode, &cam->rails[i], true);
		absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
		cam_log("cam_probe: rail %s %u mV acquire %d in %llu ms\n", rails[i].name, rails[i].mv, error, ns / NSEC_PER_MSEC);
		if (error) {
			return false;
		}
	}
	write_reg(pin_reg(cam, RESET_PIN) + 4, GPIO_IO_OUT);
	cam->reset_high = true;
	spin_us(1000);
	if (!source_acquire(cam->camcc + 0x15000, &cam->mclk_source) ||
	    !gate_acquire(cam->camcc + 0x15018, true, &cam->mclk_owned)) {
		return false;
	}
	spin_us(1000);
	pin_acquire(cam, SDA_PIN, 1u << 2 | 3u); /* cci_i2c, pull-up, 2 mA */
	pin_acquire(cam, SCL_PIN, 1u << 2 | 3u);
	if (!cci_reset(cam->cci, false)) {
		return false;
	}
	cci_timing(cam->cci);
	return cci_chip_id(cam->cci);
}

static bool
cam_restore(struct cam_probe *cam)
{
	bool ok = pin_restore(cam, SCL_PIN);
	ok = pin_restore(cam, SDA_PIN) && ok;
	if (cam->mclk_owned) {
		ok = gate_set(cam->camcc + 0x15018, false, true) && ok;
		spin_us(1000);
	}
	if (cam->reset_high) {
		write_reg(pin_reg(cam, RESET_PIN) + 4, 0);
		spin_us(1000);
	}
	for (unsigned j = 0; j < sizeof(rail_shutdown) / sizeof(rail_shutdown[0]); j++) {
		unsigned i = rail_shutdown[j];
		if (cam->rails[i].changed) {
			int error = hakios_cam_rail_vote(rails[i].name, rails[i].mv, rails[i].mode, &cam->rails[i], false);
			cam_log("cam_probe: rail %s restore %d\n", rails[i].name, error);
			ok = error == 0 && ok;
		}
	}
	ok = pin_restore(cam, RESET_PIN) && ok;
	ok = pin_restore(cam, MCLK_PIN) && ok;
	if (cam->cci_started) {
		cci_clear(cam->cci, CCI_MASK_0);
		write_reg(cam->cci + CCI_IRQ_CLEAR_1, CCI_MASK_1);
		write_reg(cam->cci + CCI_IRQ_CLEAR, 1);
		write_reg(cam->cci + CCI_IRQ_MASK_0, cam->irq0_saved);
		write_reg(cam->cci + CCI_IRQ_MASK_1, cam->irq1_saved);
	}
	if (cam->mclk_source.changed) {
		ok = source_set(cam->camcc + 0x15000, cam->mclk_source.rate) && ok;
	}
	ok = gates_release(cam->camcc, cam_gates, sizeof(cam_gates) / sizeof(cam_gates[0]), cam->cam_owned) && ok;
	for (unsigned i = sizeof(cam_sources) / sizeof(cam_sources[0]); i-- > 0;) {
		if (cam->sources[i].changed) {
			ok = source_set(cam->camcc + cam_sources[i], cam->sources[i].rate) && ok;
		}
	}
	if (cam->gdsc_changed) {
		vm_offset_t reg = cam->camcc + TITAN_GDSCR;
		uint32_t mask = GDSC_COLLAPSE | GDSC_RETAIN;
		write_reg(reg, (read_reg(reg) & ~mask) | (cam->gdsc_saved & mask));
		mask |= GDSC_POWERED;
		ok = poll_reg(reg, mask, cam->gdsc_saved & mask, 500) && ok;
	}
	if (cam->gdsc_gate_owned) {
		ok = gate_set(cam->camcc + GDSC_CLK, false, false) && ok;
	}
	ok = gates_release(cam->gcc, gcc_gates, 4, cam->gcc_owned) && ok;
	return ok;
}

static vm_offset_t
map_registers(vm_offset_t physical, vm_size_t size)
{
	vm_offset_t page = trunc_page(physical);
	vm_offset_t offset = physical - page;
	vm_offset_t base = ml_io_map_unmappable(page, round_page(offset + size), VM_WIMG_IO);
	return base ? base + offset : 0;
}

static void
unmap_registers(vm_offset_t address, vm_size_t size)
{
	if (address) {
		vm_offset_t base = trunc_page(address);
		ml_io_unmap(base, round_page(address - base + size));
	}
}

static void
csid_clear_irqs(vm_offset_t csid)
{
	write_reg(csid + CSID_TOP_CLEAR, UINT32_MAX);
	write_reg(csid + CSID_BUF_CLEAR, UINT32_MAX);
	write_reg(csid + CSID_RX_CLEAR, UINT32_MAX);
	write_reg(csid + CSID_RDI_CLEAR, UINT32_MAX);
	write_reg(csid + CSID_IRQ_CMD, 1);
}

static bool
csid_reset(vm_offset_t csid)
{
	csid_clear_irqs(csid);
	write_reg(csid + CSID_TOP_MASK, 1);
	write_reg(csid + CSID_RESET_CFG, 1u | 1u << 4); /* immediate, complete */
	write_reg(csid + CSID_RESET_CMD, 3); /* hardware and software */
	bool ok = poll_reg(csid + CSID_TOP_STATUS, 1, 1, 100000);
	write_reg(csid + CSID_TOP_MASK, 0);
	csid_clear_irqs(csid);
	return ok;
}

/*
 * CSID0 RX link from TPG13 (a CPHY_TPG input in cam_ife_csid_hw_ver2.c).
 * The TPG throttle (0x1cc, CamX skip_pattern) gates output cycles: with 0
 * the lanes reported SOT and nothing more; all ones sends the frame. Lane
 * maps follow the RX cfg0 lane_cfg field.
 */
struct tpg_link {
	const char *name;
	uint32_t cphy, lanes, lane_cfg, throttle;
};
static const struct tpg_link tpg_link = { "D-PHY 4 lanes", 0, 4, 0x3210, UINT32_MAX };

/* RX cfg0 and capture for the TPG13 mux (lane count and map, PHY type, TPG mux enabled, TPG13 selected). */
static uint32_t
tpg_rx_cfg0(const struct tpg_link *link)
{
	return (link->lanes - 1u) | link->lane_cfg << 4 | link->cphy << 24 | 1u << 27 | 1u << 28;
}

/* VC0 long and short packets, plus C-PHY packet headers. */
static uint32_t
rx_capture(uint32_t cphy)
{
	return DT_RAW10 << 4 | 3u | (cphy ? 1u << 2 | DT_RAW10 << 20 : 0);
}

static void
cam_rdi_start(vm_offset_t csid, vm_offset_t wm, uint32_t stride, uint32_t frame_incr, uint32_t iova,
    uint32_t rx_cfg0, uint32_t capture)
{
	write_reg(wm + WM_IMAGE_CFG0, 0xffff);
	write_reg(wm + WM_PACKER_CFG, 0); /* plain 128, preserve packed CSI bytes */
	write_reg(wm + WM_IMAGE_CFG2, stride);
	write_reg(wm + WM_FRAME_INCR, frame_incr);
	write_reg(wm + WM_ADDRESS, iova);
	write_reg(wm + 0x30, 0); /* IRQ subsample period */
	write_reg(wm + 0x34, 1); /* IRQ subsample pattern */
	write_reg(wm + 0x38, 0); /* frame drop period */
	write_reg(wm + 0x3c, 1); /* frame drop pattern */
	write_reg(wm + 0x60, 0); /* do not prefetch beyond the DMA window */
	write_reg(wm + WM_CFG, 1u << 16 | WM_ENABLE); /* frame based */

	write_reg(csid + CSID_BUF_MASK, RDI0_BUF_DONE);
	write_reg(csid + CSID_RDI_MASK, 0x7fff);
	write_reg(csid + CSID_RX_MASK, 0x0fffffff);
	write_reg(csid + CSID_TOP_MASK, UINT32_MAX);
	/* VC0, RAW10, payload only, with the MIPI packer enabled. */
	write_reg(csid + CSID_RDI_CFG0, 0xfu << 12 | DT_RAW10 << 16);
	write_reg(csid + CSID_RDI_CFG1, 1u << 15 | 1u << 4 | 1u << 2 | 1u);
	write_reg(csid + 0x514, 9); /* RDI0 overflow recovery, mode 8 */
	write_reg(csid + 0x544, 0); /* frame drop period */
	write_reg(csid + 0x548, 1); /* IRQ subsample pattern */
	write_reg(csid + 0x54c, 0); /* IRQ subsample period */
	write_reg(csid + CSID_RDI_CTRL, 0);
	write_reg(csid + CSID_RDI_CFG0, RDI_ENABLE | 0xfu << 12 | DT_RAW10 << 16);
	write_reg(csid + CSID_RUP_AUP, 1u << 4 | 1u << 20);
	write_reg(csid + CSID_RX_CFG0, rx_cfg0);
	write_reg(csid + CSID_RX_CFG1, 1u | 1u << 6); /* ECC correction, MISR */
	write_reg(csid + CSID_RX_CAPTURE, capture);
	write_reg(csid + CSID_RDI_CTRL, 1); /* resume at frame boundary */
}

static void
cam_tpg_start(vm_offset_t tpg, const struct tpg_link *link, uint32_t width, uint32_t height)
{
	write_reg(tpg + TPG_IRQ_MASK, 0);
	write_reg(tpg + TPG_IRQ_CLEAR, 1);
	write_reg(tpg + TPG_IRQ_CMD, 1);
	write_reg(tpg + TPG_RESET, 1);
	write_reg(tpg + 0x068, 0); /* VC0, one data type */
	write_reg(tpg + 0x06c, 0x12345678); /* LFSR seed */
	/* HBI as in camss-tpg-gen1: W * 10 bits * 545% / (8 * lanes). */
	write_reg(tpg + 0x070, (width * 10u * 545u + 800u * link->lanes - 1u) / (800u * link->lanes));
	write_reg(tpg + 0x074, height / 10u); /* VBI */
	write_reg(tpg + 0x078, 0xau << 8); /* color bars */
	write_reg(tpg + 0x07c, width << 16 | height);
	write_reg(tpg + 0x080, DT_RAW10);
	write_reg(tpg + 0x084, 2u << 20 | 8u); /* RAW10 color bar payload */
	write_reg(tpg + 0x1cc, link->throttle); /* throttle, CamX skip_pattern */
	write_reg(tpg + TPG_IRQ_MASK, 1);
	/* One VC, frame interleave, lane count and PHY type, enable. */
	write_reg(tpg + TPG_CTRL, (link->lanes - 1u) << 4 | link->cphy << 3 | TPG_ENABLE);
}

static bool
cam_capture_pattern(vm_offset_t csid, vm_offset_t ife, vm_offset_t tpg,
    const struct tpg_link *link, bool *stopped)
{
	vm_offset_t wm = ife + IFE_WM_RDI0;
	cam_rdi_start(csid, wm, TPG_STRIDE, TPG_FRAME_BYTES, (uint32_t)IFE_IOVA, tpg_rx_cfg0(link), rx_capture(link->cphy));
	cam_tpg_start(tpg, link, TPG_WIDTH, TPG_HEIGHT);
	uint32_t tpg_started = read_reg(tpg + TPG_CTRL);
	uint32_t wm_started = read_reg(wm + WM_CFG);
	cam_log("cam_probe: %s: TPG ctrl %#x status %#x, CSID RX %#x RDI %#x, WM cfg %#x IOVA %#x\n",
	    link->name, tpg_started, read_reg(tpg + TPG_STATUS), read_reg(csid + CSID_RX_CFG0),
	    read_reg(csid + CSID_RDI_CFG0), wm_started, read_reg(wm + WM_ADDRESS));
	bool done = false;
	if (tpg_started != UINT32_MAX && (tpg_started & TPG_ENABLE) &&
	    wm_started != UINT32_MAX && (wm_started & WM_ENABLE)) {
		uint64_t deadline;
		clock_interval_to_deadline(500, NSEC_PER_MSEC, &deadline);
		do {
			uint32_t status = read_reg(csid + CSID_BUF_STATUS);
			done = status != UINT32_MAX && (status & RDI0_BUF_DONE);
			if (done || status == UINT32_MAX) {
				break;
			}
			spin_us(10);
		} while (mach_absolute_time() < deadline);
	}
	uint32_t packets = read_reg(csid + 0x240);
	uint32_t tpg_status = read_reg(tpg + TPG_STATUS);
	uint32_t rx_status = read_reg(csid + CSID_RX_STATUS);
	uint32_t misr[4];
	for (unsigned i = 0; i < 4; i++) {
		misr[i] = read_reg(csid + 0x230 + 4 * i);
	}
	/* Stop feeding the receiver immediately, then allow outstanding writes to drain. */
	write_reg(tpg + TPG_CTRL, 0);
	write_reg(csid + CSID_RDI_CTRL, 0);
	delay(40000);
	write_reg(wm + WM_CFG, 0);
	*stopped = poll_reg(tpg + TPG_CTRL, TPG_ENABLE, 0, 500) &&
	    poll_reg(wm + WM_CFG, WM_ENABLE, 0, 500);

	uint32_t violation = read_reg(ife + 0xc64);
	uint32_t overflow = read_reg(ife + 0xc68);
	uint32_t image_error = read_reg(ife + 0xc70);
	uint32_t crc_errors = read_reg(csid + 0x248);
	cam_log("cam_probe: %s, CSID buf %#x RDI %#x RX %#x top %#x CRC %u, TPG IRQ %#x\n",
	    done ? "FRAME DONE" : "NO FRAME", read_reg(csid + CSID_BUF_STATUS),
	    read_reg(csid + CSID_RDI_STATUS), read_reg(csid + CSID_RX_STATUS),
	    read_reg(csid + CSID_TOP_STATUS), crc_errors, read_reg(tpg + TPG_IRQ_STATUS));
	cam_log("cam_probe: IFE bus IRQ %#x/%#x violation %#x overflow %#x image %#x, WM address %#x\n",
	    read_reg(ife + 0xc28), read_reg(ife + 0xc2c), violation, overflow,
	    image_error, read_reg(wm + 0x70));
	/* CSID 680 RX capture registers (cam_ife_csid680.h). */
	cam_log("cam_probe: %s: CSID packets %u RX %#x, long %#x/%#x short %#x/%#x unmapped %#x C-PHY %#x ECC %#x, TPG status %#x\n",
	    link->name, packets, rx_status, read_reg(csid + 0x220), read_reg(csid + 0x224),
	    read_reg(csid + 0x218), read_reg(csid + 0x21c), read_reg(csid + 0x210),
	    read_reg(csid + 0x22c), read_reg(csid + 0x244), tpg_status);
	cam_log("cam_probe: %s: lane MISR %#x %#x %#x %#x\n", link->name, misr[0], misr[1], misr[2], misr[3]);

	write_reg(tpg + TPG_IRQ_MASK, 0);
	write_reg(tpg + TPG_IRQ_CLEAR, 1);
	write_reg(tpg + TPG_IRQ_CMD, 1);
	write_reg(tpg + TPG_RESET, 1);
	write_reg(csid + CSID_RDI_CFG0, 0);
	write_reg(csid + CSID_RX_CFG0, 0);
	write_reg(csid + CSID_RX_CFG1, 0);
	write_reg(csid + CSID_RX_CAPTURE, 0);
	write_reg(csid + CSID_RDI_MASK, 0);
	write_reg(csid + CSID_BUF_MASK, 0);
	write_reg(csid + CSID_RX_MASK, 0);
	return csid_reset(csid) && done && !violation && !overflow && !image_error && !crc_errors;
}

/* CPAS fast AHB first, then the TPG, CSID and IFE clock sources. */
static const uint32_t capture_sources[] = { 0x10018, 0x1104c, 0x13174, 0x11018 };

/* CPAS fast AHB, CSID core/RX, IFE fast AHB/core/CPAS. The separate
 * CPAS core AHB enable at 0x131f0 did not latch on the NE2213. */
static const uint32_t capture_gates[] = { 0x1316c, 0x1318c, 0x15100, 0x11048, 0x11030, 0x11038 };
static const struct {
	uint32_t reg, value;
} rdi_qos[] = {
	{ 0x5230, 0x66665433 }, { 0x5234, 0x66666666 }, { 0x5238, 0x00001e30 },
	{ 0x5240, 0xffffff00 }, { 0x5248, 0x0000000f },
};

static bool
cam_pattern_run(bool *stopped)
{
	struct cam_probe cam = { 0 };
	struct hakios_cam_power power = { 0 };
	struct clock_source sources[sizeof(capture_sources) / sizeof(capture_sources[0])] = { 0 };
	uint32_t gates = 0, gdsc = 0;
	bool gdsc_changed = false;
	bool captured = false;
	const char *step = "MMIO mappings";
	cam.gcc = map_registers(GCC_PA, 0x1000);
	cam.camcc = map_registers(CAMCC_PA, CAMCC_SIZE);
	vm_offset_t cpas = map_registers(CPAS_PA, 0x1000);
	vm_offset_t camnoc = map_registers(CAMNOC_PA, CAMNOC_SIZE);
	vm_offset_t csid = map_registers(CSID_PA, 0x1000);
	vm_offset_t top = map_registers(CSID_TOP_PA, 0x1000);
	vm_offset_t ife = map_registers(IFE_PA, IFE_SIZE);
	vm_offset_t tpg = map_registers(TPG_PA, 0x1000);
	if (!cam.gcc || !cam.camcc || !cpas || !camnoc || !csid || !top || !ife || !tpg) {
		goto out;
	}
	step = "MMCX and camera NoC votes";
	cam_log("cam_probe: requesting %s\n", step);
	if (!hakios_cam_power_vote(&power, HAKIOS_RPMH_CAMERA, true)) {
		goto out;
	}
	step = "Titan camera clocks";
	if (!cam_clocks_up(&cam)) {
		goto out;
	}
	step = "capture clocks busy";
	for (unsigned i = 0; i < sizeof(capture_gates) / sizeof(capture_gates[0]); i++) {
		if (read_reg(cam.camcc + capture_gates[i]) & CLK_ENABLE) {
			goto out;
		}
	}
	step = "capture clock sources";
	cam_log("cam_probe: setting CSID, TPG and IFE sources to TCXO\n");
	for (unsigned i = 0; i < sizeof(capture_sources) / sizeof(capture_sources[0]); i++) {
		if (!source_acquire(cam.camcc + capture_sources[i], &sources[i])) {
			goto out;
		}
	}
	step = "IFE0 power domain";
	cam_log("cam_probe: starting %s\n", step);
	gdsc = read_reg(cam.camcc + IFE_GDSCR);
	if (gdsc == UINT32_MAX || (gdsc & GDSC_HW_CONTROL)) {
		goto out;
	}
	gdsc_changed = true;
	write_reg(cam.camcc + IFE_GDSCR, (gdsc & ~GDSC_COLLAPSE) | GDSC_RETAIN);
	if (!poll_reg(cam.camcc + IFE_GDSCR, GDSC_POWERED, GDSC_POWERED, 500)) {
		goto out;
	}
	step = "capture clock gates";
	cam_log("cam_probe: starting %s\n", step);
	if (!gates_acquire(cam.camcc, capture_gates, sizeof(capture_gates) / sizeof(capture_gates[0]), 0, &gates)) {
		goto out;
	}
	uint32_t camera = read_reg(cpas);
	cam_log("cam_probe: CPAS camera version %#x\n", camera);
	uint32_t csid_version = read_reg(csid);
	cam_log("cam_probe: CSID0 version %#x\n", csid_version);
	uint32_t tpg_version = read_reg(tpg);
	cam_log("cam_probe: TPG13 version %#x\n", tpg_version);
	uint32_t ife_bus = read_reg(ife + 0xc00);
	cam_log("cam_probe: camera %#x CPAS %#x, CSID %#x IFE bus %#x TPG %#x\n",
	    camera, read_reg(cpas + 4), csid_version, ife_bus, tpg_version);
	step = "capture block versions or active DMA";
	if (camera != 0x00060800 || !csid_version || csid_version == UINT32_MAX ||
	    !tpg_version || tpg_version == UINT32_MAX || !ife_bus || ife_bus == UINT32_MAX ||
	    (read_reg(ife + IFE_WM_RDI0 + WM_CFG) & WM_ENABLE) ||
	    (read_reg(tpg + TPG_CTRL) & TPG_ENABLE) || (read_reg(csid + CSID_RDI_CFG0) & RDI_ENABLE)) {
		goto out;
	}
	step = "CSID reset";
	if (!csid_reset(csid)) {
		goto out;
	}
	uint32_t saved_route = read_reg(top);
	write_reg(top, 0x101); /* CSID0 to full IFE0 */
	uint32_t saved_qos[sizeof(rdi_qos) / sizeof(rdi_qos[0])];
	for (unsigned i = 0; i < sizeof(rdi_qos) / sizeof(rdi_qos[0]); i++) {
		saved_qos[i] = read_reg(camnoc + rdi_qos[i].reg);
		write_reg(camnoc + rdi_qos[i].reg, rdi_qos[i].value);
	}
	step = "TPG through CSID0 and IFE0 RDI0";
	captured = cam_capture_pattern(csid, ife, tpg, &tpg_link, stopped);
	uint32_t noc_error = read_reg(camnoc + 0x2010);
	cam_log("cam_probe: CAMNOC fault %#x errvld %#x errlog %#x:%#x\n", read_reg(camnoc + 0x2248),
	    noc_error, read_reg(camnoc + 0x2024), read_reg(camnoc + 0x2020));
	captured = captured && !(noc_error & 1u);
	bool routing_restored = true;
	for (unsigned i = 0; i < sizeof(rdi_qos) / sizeof(rdi_qos[0]); i++) {
		write_reg(camnoc + rdi_qos[i].reg, saved_qos[i]);
		routing_restored = (read_reg(camnoc + rdi_qos[i].reg) == saved_qos[i]) && routing_restored;
	}
	write_reg(top, saved_route);
	captured = captured && routing_restored && read_reg(top) == saved_route;
out:
	if (!captured) {
		cam_log("cam_probe: capture failed at %s\n", step);
	}
	bool restored = gates_release(cam.camcc, capture_gates, sizeof(capture_gates) / sizeof(capture_gates[0]), gates);
	if (gdsc_changed) {
		uint32_t mask = GDSC_COLLAPSE | GDSC_RETAIN;
		write_reg(cam.camcc + IFE_GDSCR, (read_reg(cam.camcc + IFE_GDSCR) & ~mask) | (gdsc & mask));
		mask |= GDSC_POWERED;
		restored = poll_reg(cam.camcc + IFE_GDSCR, mask, gdsc & mask, 500) && restored;
	}
	for (unsigned i = sizeof(capture_sources) / sizeof(capture_sources[0]); i-- > 0;) {
		if (sources[i].changed) {
			restored = source_set(cam.camcc + capture_sources[i], sources[i].rate) && restored;
		}
	}
	restored = cam_restore(&cam) && restored;
	restored = hakios_cam_power_vote(&power, HAKIOS_RPMH_CAMERA, false) && restored;
	cam_log("cam_probe: TPG/WM stopped %u, clocks and power restore %s\n",
	    (unsigned)*stopped, restored ? "OK" : "FAILED");
	unmap_registers(tpg, 0x1000);
	unmap_registers(ife, IFE_SIZE);
	unmap_registers(top, 0x1000);
	unmap_registers(csid, 0x1000);
	unmap_registers(camnoc, CAMNOC_SIZE);
	unmap_registers(cpas, 0x1000);
	unmap_registers(cam.camcc, CAMCC_SIZE);
	unmap_registers(cam.gcc, 0x1000);
	return captured && restored && *stopped;
}

/* NCAMRAW1 is also understood by the reference RAW10 decoder. Source 1 is the TPG, 2 the sensor. */
struct ncam_header {
	char magic[8];
	uint32_t version, width, height, stride, bytes, data_type, bits, packing, source, valid;
	uint8_t padding[16];
};
_Static_assert(sizeof(struct ncam_header) == 64, "NCAMRAW1 header must be 64 bytes");

/*
 * Minor 0 is /dev/hakios-camera-tpg, minors 1-3 are /dev/hakios-camera-raw0-2, minors 4-6 are
 * level 5's /dev/hakios-camera-fd0-2, minors 7-9 its /dev/hakios-camera-full0-2 (both NV12), minors
 * 10-11 level 7's /dev/hakios-camera-hires0-1 (mode 0 RAW10). Each is published once, after the writer
 * has stopped and the DMA mapping is gone.
 */
#define CAM_FRAME_NODES 12
static struct {
	struct ncam_header header;
	vm_offset_t frame;
} cam_frames[CAM_FRAME_NODES];
static int cam_frames_major = -1;

static int
cam_frame_open(dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	unsigned n = (unsigned)minor(dev);
	return n < CAM_FRAME_NODES && __atomic_load_n(&cam_frames[n].frame, __ATOMIC_ACQUIRE) ? 0 : ENXIO;
}

static int
cam_frame_read(dev_t dev, struct uio *uio, __unused int flags)
{
	unsigned n = (unsigned)minor(dev);
	vm_offset_t frame = n < CAM_FRAME_NODES ? __atomic_load_n(&cam_frames[n].frame, __ATOMIC_ACQUIRE) : 0;
	off_t off = uio_offset(uio);
	if (!frame || off < 0) {
		return EINVAL;
	}
	const struct ncam_header *header = &cam_frames[n].header;
	if (off < (off_t)sizeof(*header)) {
		int bytes = (int)MIN(uio_resid(uio), (off_t)sizeof(*header) - off);
		int error = uiomove((caddr_t)header + off, bytes, uio);
		if (error || uio_resid(uio) == 0) {
			return error;
		}
		off = uio_offset(uio);
	}
	off -= (off_t)sizeof(*header);
	if (off >= header->bytes) {
		return 0;
	}
	return uiomove((caddr_t)(frame + off), (int)MIN(uio_resid(uio), (off_t)header->bytes - off), uio);
}

static const struct cdevsw cam_frame_cdevsw = {
	.d_open = cam_frame_open,
	.d_close = nulldev,
	.d_read = cam_frame_read,
	.d_write = eno_rdwrt,
	.d_ioctl = eno_ioctl,
	.d_stop = eno_stop,
	.d_reset = eno_reset,
	.d_select = eno_select,
	.d_mmap = eno_mmap,
	.d_strategy = eno_strat,
};

static bool
cam_frame_publish(unsigned n, const char *name, vm_offset_t kva, uint32_t width, uint32_t height,
    uint32_t stride, uint32_t source)
{
	if (cam_frames_major < 0) {
		cam_frames_major = cdevsw_add(-1, &cam_frame_cdevsw);
		if (cam_frames_major < 0) {
			return false;
		}
	}
	cam_frames[n].header = (struct ncam_header){
		"NCAMRAW1", 1, width, height, stride, stride * height, DT_RAW10, 10, 1, source, 1, { 0 },
	};
	__atomic_store_n(&cam_frames[n].frame, kva, __ATOMIC_RELEASE);
	if (devfs_make_node(makedev(cam_frames_major, n), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0400, "%s", name)) {
		return true;
	}
	__atomic_store_n(&cam_frames[n].frame, 0, __ATOMIC_RELEASE);
	return false;
}

static bool
cam_frame_check(const uint8_t *data)
{
	unsigned changed = 0, distinct = 0;
	bool seen[256] = { 0 };
	for (unsigned i = 0; i < TPG_FRAME_BYTES; i++) {
		changed += data[i] != TPG_FILL;
		if (!seen[data[i]]) {
			seen[data[i]] = true;
			distinct++;
		}
	}
	/*
	 * Saturated RAW10 colour bars pack into only a few byte values, so check
	 * structure: vertical bars repeat every two rows (Bayer period), and a
	 * row holds more than one value.
	 */
	unsigned row_values = 0, mismatched_rows = 0;
	bool row_seen[256] = { 0 };
	for (unsigned i = 0; i < TPG_STRIDE; i++) {
		if (!row_seen[data[i]]) {
			row_seen[data[i]] = true;
			row_values++;
		}
	}
	for (unsigned row = 2; row < TPG_HEIGHT; row++) {
		mismatched_rows += memcmp(data + row * TPG_STRIDE, data + (row % 2) * TPG_STRIDE, TPG_STRIDE) != 0;
	}
	cam_log("cam_probe: TPG %ux%u packed RAW10 stride %u, %u/%u bytes changed, %u distinct, "
	    "%u in row 0, %u rows differ from rows 0/1\n",
	    TPG_WIDTH, TPG_HEIGHT, TPG_STRIDE, changed, TPG_FRAME_BYTES, distinct, row_values, mismatched_rows);
	return changed > TPG_FRAME_BYTES / 2 && row_values > 1 && mismatched_rows == 0;
}

static bool
cam_tpg_capture(void)
{
	vm_size_t size = round_page(2 * TPG_FRAME_BYTES);
	vm_offset_t kva;
	if (kmem_alloc_contig(kernel_map, &kva, size, 0, 0, 0,
	    KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		cam_log("cam_probe: level 2 FAIL (DMA buffer allocation)\n");
		return false;
	}

	uint64_t pa = kvtophys(kva);
	if (!pa) {
		cam_log("cam_probe: level 2 FAIL (DMA buffer physical address)\n");
		kmem_free(kernel_map, kva, size);
		return false;
	}

	struct hakios_smmu_context *ctx = hakios_smmu_claim(IFE_SID, IFE_SID_MASK,
	    IFE_IOVA, size, "camera IFE");
	if (!ctx) {
		cam_log("cam_probe: level 2 FAIL (IFE stream claim)\n");
		kmem_free(kernel_map, kva, size);
		return false;
	}

	int map_error = hakios_smmu_map(ctx, IFE_IOVA, pa, size,
	    HAKIOS_SMMU_READ | HAKIOS_SMMU_WRITE);
	bool stopped = true;
	bool captured = false;
	if (!map_error) {
		memset((void *)kva, TPG_FILL, size);
		flush_dcache64((addr64_t)pa, (unsigned)size, TRUE);
		captured = cam_pattern_run(&stopped);
		captured = hakios_smmu_faults(ctx) && captured;
	}
	int unmap_error = map_error ? 0 : hakios_smmu_unmap(ctx, IFE_IOVA, size);
	int release_error = hakios_smmu_release(ctx);
	cam_log("cam_probe: IFE SID %#x/%#x IOVA %#llx PA %#llx bytes %#lx: map %d, unmap %d, release %d\n",
	    IFE_SID, IFE_SID_MASK, IFE_IOVA, (unsigned long long)pa, (unsigned long)size,
	    map_error, unmap_error, release_error);

	if (captured && !unmap_error && !release_error) {
		flush_dcache64((addr64_t)pa, (unsigned)size, TRUE);
		if (cam_frame_check((const uint8_t *)kva)) {
			if (cam_frame_publish(0, "hakios-camera-tpg", kva, TPG_WIDTH, TPG_HEIGHT, TPG_STRIDE, 1)) {
				cam_log("cam_probe: level 2 PASS (TPG frame sanity and teardown, dump /dev/hakios-camera-tpg)\n");
				return true;
			}
			cam_log("cam_probe: could not publish TPG frame\n");
		}
	}
	/* Retain the backing if the writer or its translations could still be live. */
	if (stopped && (!release_error || (!map_error && !unmap_error))) {
		kmem_free(kernel_map, kva, size);
	} else {
		cam_log("cam_probe: retaining DMA buffer after uncertain SMMU teardown\n");
	}
	cam_log("cam_probe: level 2 FAIL (TPG frame or teardown)\n");
	return false;
}

/* ---- level 3: IMX789 mode 1 over three-trio C-PHY on CSIPHY1, CSID0, IFE0 RDI0 ---- */

/*
 * The sensor tables are the stock module's (vendor data, not in the repository):
 * tools/camsensor.py MODULE.bin --c-header hakios.cam_imx789.h 1 writes them into the kernel
 * tree's bsd/dev/ before the build. Without that header, level 3 reports that it has no tables.
 */
struct cam_reg {
	uint16_t addr;
	uint8_t data;
	uint32_t delay_us;
};
#if __has_include("hakios.cam_imx789.h")
#include "hakios.cam_imx789.h"
#define CAM_HAVE_TABLES 1
_Static_assert(CAM_TABLES_ID == 0x789 && CAM_TABLES_SLAVE == 0x34 && CAM_TABLES_LANE_ASSIGN == 0x210 &&
    CAM_TABLES_LANES == 3 && CAM_TABLES_CPHY == 1 && CAM_TABLES_DATA_TYPE == DT_RAW10,
    "hakios.cam_imx789.h is not the NE2213 IMX789 module's three-trio RAW10 mode");
#define RAW_WIDTH          CAM_TABLES_WIDTH
#define RAW_HEIGHT         CAM_TABLES_HEIGHT
#else
#define CAM_HAVE_TABLES 0
#define RAW_WIDTH          4000u
#define RAW_HEIGHT         3008u
#endif
#define RAW_LINE           (RAW_WIDTH * 10u / 8u)
/*
 * The write master puts each 5000-byte line on a 16-byte boundary: the second boot's frames
 * kept the RAW10 group phase only with a 5008-byte pitch, and a 5000-byte slot let the last
 * frame run 24 KiB past the mapping (a CAMNOC error).
 */
#define RAW_STRIDE         ((RAW_LINE + 15u) & ~15u)
#define RAW_FRAME_BYTES    (RAW_STRIDE * RAW_HEIGHT)
#define RAW_SLOTS          3u /* two consecutive frames, then a short exposure (or a late third frame) */
#define RAW_BYTES          (RAW_SLOTS * RAW_FRAME_BYTES)
#define RAW_BLACK          64u /* 10-bit pedestal, typical for Sony sensors; used only for the exposure ratio */
#define CCI_M0_Q1_REPORT   (1u << 8)
#define IMX789_FRAME_COUNT 0x0005u
#define IMX789_MODE_SELECT 0x0100u
_Static_assert(RAW_FRAME_BYTES % 256 == 0, "RAW10 slots must stay 256-byte aligned for the write master");

/* SM8450 CSIPHY1 (live DT qcom,csiphy1@ace6000, csiphy-v2.1.0) */
#define CSIPHY1_PA         0x0ace6000UL
#define CSIPHY_SIZE        0x2000UL
#define CPHY_LANE_ENABLE   0x2au /* CPHY_LANE_0 | CPHY_LANE_1 | CPHY_LANE_2 (cam_csiphy_dev.h) for lane assign 0x210 */
/* CSID 680 RX cfg0 for CSIPHY1: 3 trios, lane map 0x210, C-PHY, phy_num = CSIPHY index + 1. */
#define SENSOR_RX_CFG0     ((3u - 1u) | 0x210u << 4 | 2u << 20 | 1u << 24)

/*
 * Lucid EVO PLLs (camcc-waipio.c cam_cc_pll0_config / cam_cc_pll3_config, clk-alpha-pll.c
 * clk_lucid_evo_pll_configure and alpha_pll_lucid_evo_enable). PLL0: 19.2 MHz x 62.5 = 1200 MHz,
 * main 1200, even /2 600, odd /3 400. PLL3: 19.2 MHz x 45 = 864 MHz, even /2 432.
 */
#define PLL_MODE           0x00
#define PLL_OPMODE         0x04
#define PLL_L_VAL          0x10
#define PLL_ALPHA_VAL      0x14
#define PLL_USER_CTL       0x18
#define PLL_USER_CTL_U     0x1c
#define PLL_CONFIG_CTL     0x20
#define PLL_CONFIG_CTL_U   0x24
#define PLL_CONFIG_CTL_U1  0x28
#define PLL_OUTCTRL        (1u << 0)
#define PLL_RESET_N        (1u << 2)
#define PLL_LOCK_DET       (1u << 31)
#define PLL_OUT_MASK       7u
#define PLL_VOTE_RUN       (1u << 25)
static const struct cam_pll {
	const char *name;
	uint32_t offset, l, cal_l, alpha, config, config_u, config_u1, user, user_u;
} cam_plls[] = {
	{ "PLL0", 0x0000, 0x3e, 0x44, 0x8000, 0x20485699, 0x00182261, 0x32aa299c, 0x00008400, 0x00000805 },
	{ "PLL3", 0x3000, 0x2d, 0x44, 0x0000, 0x20485699, 0x00182261, 0x32aa299c, 0x00000400, 0x00000805 },
};
/*
 * Level 5's pixel path: PLL3 at 1188 MHz (L 0x3d, alpha 0xe000), so IFE0 runs at 594 MHz from its
 * even output, camcc-waipio's next IFE rate above CamX's 586.5 MHz pixel vote for this mode.
 */
static const struct cam_pll pixel_plls[] = {
	{ "PLL0", 0x0000, 0x3e, 0x44, 0x8000, 0x20485699, 0x00182261, 0x32aa299c, 0x00008400, 0x00000805 },
	{ "PLL3", 0x3000, 0x3d, 0x44, 0xe000, 0x20485699, 0x00182261, 0x32aa299c, 0x00000400, 0x00000805 },
};
_Static_assert(sizeof(pixel_plls) == sizeof(cam_plls), "pixel_plls replaces cam_plls one for one");

/*
 * Sensor-path clock sources, RCG cfg = (2 * divider - 1) | source << 8 (camcc-waipio.c tables):
 * CPAS fast AHB at TCXO as in level 2; CPHY RX 400 MHz = PLL0 main / 3 and CSID 400 MHz = PLL0
 * main / 3 (the DT lowsvs rates); IFE0 432 MHz = PLL3 even; CSI1 PHY timer 400 MHz = PLL0 odd;
 * CAMNOC AXI 400 MHz = PLL0 odd for the 450 MB/s RAW10 stream (it was at TCXO for CCI).
 */
static const struct {
	uint32_t cmd, rate;
} sensor_sources[] = {
	{ 0x10018, 0 }, { 0x1104c, 5u | 1u << 8 }, { 0x13174, 5u | 1u << 8 }, { 0x11018, 1u | 6u << 8 },
	{ 0x15104, 1u | 3u << 8 }, { 0x13194, 1u | 3u << 8 },
};
#define CAMNOC_AXI_SOURCE  5u /* index in sensor_sources: level 1 owns its saved rate */
/* Level 2's gates plus CSIPHY1 and its timer (live DT csiphy1 clocks). */
static const uint32_t sensor_gates[] = { 0x1316c, 0x1318c, 0x15100, 0x11048, 0x11030, 0x11038, 0x15120, 0x1511c };

/* csiphy_3ph_v2_1_0_reg trio 0 (cam_csiphy_2_1_0_hwreg.h); trios 1 and 2 are +0x400 and +0x800. */
#define PHY_SETTLE_LO      0x100u
#define PHY_SETTLE_HI      0x200u
static const struct {
	uint16_t addr;
	uint16_t data; /* or PHY_SETTLE_LO/HI */
	uint16_t delay_us;
} cphy_trio[] = {
	{ 0x268, 0xf1, 100 }, { 0x294, 0x08, 0 }, { 0x2f4, 0x00, 0 }, { 0x2f8, 0x00, 0 }, { 0x2fc, 0x00, 0 },
	{ 0x2f0, 0x00, 2 }, { 0x2f0, 0xef, 100 }, { 0x294, 0x09, 0 }, { 0x204, 0x00, 0 },
	{ 0x20c, PHY_SETTLE_LO, 0 }, { 0x208, PHY_SETTLE_HI, 0 }, { 0x210, 0x00, 0 }, { 0x2e4, 0x00, 0 },
	{ 0x2e8, 0x7f, 0 }, { 0x2ec, 0x7f, 0 }, { 0x218, 0x3e, 0 }, { 0x21c, 0x41, 0 }, { 0x220, 0x41, 0 },
	{ 0x224, 0x7f, 0 }, { 0x228, 0x00, 0 }, { 0x22c, 0x00, 0 }, { 0x264, 0x01, 0 }, { 0x244, 0xb2, 0 },
	{ 0x310, 0x35, 0 }, { 0x2bc, 0xd0, 0 }, { 0x270, 0x00, 0 }, { 0x230, 0x94, 0 }, { 0x234, 0x31, 0 },
	{ 0x238, 0x60, 0 }, { 0x23c, 0xa6, 0 }, { 0x258, 0x10, 0 }, { 0x254, 0x00, 0 }, { 0x25c, 0x04, 0 },
	{ 0x248, 0x00, 0 }, { 0x24c, 0x00, 0 }, { 0x240, 0x00, 0 }, { 0x260, 0xa8, 0 },
};
/*
 * data_rate_delta_table_2_1_0 (cam_csiphy_2_1_0_hwreg.h, the same in every OnePlus SM8450 branch):
 * the vendor driver takes the first entry whose bandwidth covers CamX's data rate, the output pixel
 * clock x 10 bits / 3 trios (the stock preview log: mode 2, "Datarate: 4432466666"). Mode 1 needs
 * 750.45 MHz -> 2.50 Gb/s, entry 0 (up to 2.736 Gb/s per trio, 1.2 GS/s); mode 0 needs 2106.51 MHz ->
 * 7.02 Gb/s, entry 7 (up to 7.524 Gb/s, 3.3 GS/s). Trio 2 also writes 0x108c; the DNP and auxiliary
 * entries are not written (data_rate_aux_mask starts at 0). The RX clock stays at 400 MHz for both
 * (get_clk_voting_dynamic: rate / 32 is under the lowsvs 400 MHz).
 */
struct cphy_rate {
	uint64_t bandwidth;
	uint8_t trio2_108c;
	struct {
		uint16_t addr;
		uint8_t data;
	} regs[7];
};
static const struct cphy_rate cphy_rate0 = { 2736000000ull, 0x0c, {
	{ 0x274, 0x07 }, { 0x278, 0x70 }, { 0x288, 0x00 }, { 0x28c, 0x11 }, { 0x26c, 0x10 }, { 0x290, 0x03 }, { 0x214, 0x10 },
} };
static const struct cphy_rate __unused cphy_rate7 = { 7524000000ull, 0x03, {
	{ 0x274, 0x07 }, { 0x278, 0x1d }, { 0x288, 0x00 }, { 0x28c, 0x01 }, { 0x26c, 0x10 }, { 0x290, 0x02 }, { 0x214, 0x00 },
} };
static const uint8_t cphy_irq[] = { 0xff, 0xff, 0xfb, 0xff, 0x7f, 0xff, 0xff, 0xef, 0xff, 0xff, 0xff };

static bool
pll_up(vm_offset_t camcc, const struct cam_pll *p, bool *ours)
{
	vm_offset_t b = camcc + p->offset;
	uint32_t mode = read_reg(b + PLL_MODE), user = read_reg(b + PLL_USER_CTL);
	cam_log("cam_probe: %s mode %#x opmode %#x user %#x L %#x\n", p->name, mode, read_reg(b + PLL_OPMODE),
	    user, read_reg(b + PLL_L_VAL));
	if (mode == UINT32_MAX || (user & PLL_VOTE_RUN)) {
		return false;
	}
	if ((mode & PLL_OUTCTRL) && (mode & PLL_LOCK_DET)) {
		/* Someone else runs it; check that it runs at our rate before using it. */
		return (read_reg(b + PLL_L_VAL) & 0xffffu) == p->l && read_reg(b + PLL_ALPHA_VAL) == p->alpha &&
		       (user & 0xff00u) == (p->user & 0xff00u);
	}
	*ours = true;
	write_reg(b + PLL_L_VAL, p->cal_l << 16 | p->l);
	write_reg(b + PLL_ALPHA_VAL, p->alpha);
	write_reg(b + PLL_CONFIG_CTL, p->config);
	write_reg(b + PLL_CONFIG_CTL_U, p->config_u);
	write_reg(b + PLL_CONFIG_CTL_U1, p->config_u1);
	write_reg(b + PLL_USER_CTL, p->user);
	write_reg(b + PLL_USER_CTL_U, p->user_u);
	write_reg(b + PLL_MODE, read_reg(b + PLL_MODE) & ~PLL_OUTCTRL);
	write_reg(b + PLL_OPMODE, 0); /* standby */
	write_reg(b + PLL_MODE, read_reg(b + PLL_MODE) | PLL_RESET_N);
	write_reg(b + PLL_OPMODE, 1); /* run */
	if (!poll_reg(b + PLL_MODE, PLL_LOCK_DET, PLL_LOCK_DET, 200)) {
		cam_log("cam_probe: %s did not lock, mode %#x\n", p->name, read_reg(b + PLL_MODE));
		return false;
	}
	write_reg(b + PLL_USER_CTL, read_reg(b + PLL_USER_CTL) | PLL_OUT_MASK);
	write_reg(b + PLL_MODE, read_reg(b + PLL_MODE) | PLL_OUTCTRL);
	cam_log("cam_probe: %s locked, mode %#x user %#x\n", p->name, read_reg(b + PLL_MODE), read_reg(b + PLL_USER_CTL));
	return true;
}

static void
pll_down(vm_offset_t camcc, const struct cam_pll *p)
{
	vm_offset_t b = camcc + p->offset;
	write_reg(b + PLL_MODE, read_reg(b + PLL_MODE) & ~PLL_OUTCTRL);
	write_reg(b + PLL_USER_CTL, read_reg(b + PLL_USER_CTL) & ~PLL_OUT_MASK);
	write_reg(b + PLL_OPMODE, 0);
}

/*
 * cam_csiphy_core.c CAM_START_DEV for one three-trio C-PHY device: reset enter, common data,
 * config_dev (common registers with the lane enable, data rate entry 0, the 37 three-phase
 * registers of each trio), IRQ configuration, release from reset (0x1000 = 0x0e, 3048 us).
 * The vendor writes the common DEFAULT_PARAMS into every CSIPHY (rgltr-enable-sync); only
 * CSIPHY1 is clocked here, so only CSIPHY1 is written. The settle value is the module's
 * count (the vendor driver divides CamX's settle time by 200000000 to get it back).
 */
static __unused void
csiphy_start_rate(vm_offset_t phy, unsigned settle, const struct cphy_rate *rate)
{
	write_reg(phy + 0x1014, 0);
	write_reg(phy + 0x1000, 1);
	spin_us(1);
	write_reg(phy + 0x1084, 0x00);
	write_reg(phy + 0x1018, 0x01);
	write_reg(phy + 0x101c, 0x7a);
	write_reg(phy + 0x1014, CPHY_LANE_ENABLE);
	write_reg(phy + 0x1084, 0x00);
	write_reg(phy + 0x1018, 0x01);
	write_reg(phy + 0x101c, 0x7a);
	for (uint32_t t = 0; t < 3; t++) {
		for (unsigned i = 0; i < sizeof(rate->regs) / sizeof(rate->regs[0]); i++) {
			write_reg(phy + 0x400 * t + rate->regs[i].addr, rate->regs[i].data);
		}
	}
	write_reg(phy + 0x108c, rate->trio2_108c);
	for (uint32_t t = 0; t < 3; t++) {
		for (unsigned i = 0; i < sizeof(cphy_trio) / sizeof(cphy_trio[0]); i++) {
			uint32_t v = cphy_trio[i].data;
			v = v == PHY_SETTLE_LO ? (settle & 0xffu) : v == PHY_SETTLE_HI ? ((settle >> 8) & 0xffu) : v;
			write_reg(phy + 0x400 * t + cphy_trio[i].addr, v);
			unsigned wait = cphy_trio[i].delay_us;
			if (t == 2 && i == sizeof(cphy_trio) / sizeof(cphy_trio[0]) - 1) {
				wait = 100; /* the third trio's last entry carries 0x64 us */
			}
			if (wait) {
				spin_us(wait);
			}
		}
	}
	for (unsigned i = 0; i < sizeof(cphy_irq); i++) {
		write_reg(phy + 0x102c + 4 * i, cphy_irq[i]);
	}
	spin_us(100);
	write_reg(phy + 0x1000, 0x0e);
	spin_us(3048);
	cam_log("cam_probe: CSIPHY1 settle %u, rate entry %llu Mb/s, lanes %#x, trio status %#x %#x %#x (0 or 8 is normal)\n",
	    settle, rate->bandwidth / 1000000, read_reg(phy + 0x1014), read_reg(phy + 0x358) & 0x1f, read_reg(phy + 0x758) & 0x1f,
	    read_reg(phy + 0xb58) & 0x1f);
}

static __unused void
csiphy_start(vm_offset_t phy, unsigned settle)
{
	csiphy_start_rate(phy, settle, &cphy_rate0);
}

/* cam_csiphy_reset, prgm_cmn_data(reset) and cphy_irq_disable. */
static void
csiphy_stop(vm_offset_t phy)
{
	write_reg(phy + 0x1014, 0);
	write_reg(phy + 0x1000, 1);
	spin_us(1);
	write_reg(phy + 0x1084, 0);
	write_reg(phy + 0x1018, 0);
	write_reg(phy + 0x101c, 0);
	for (unsigned i = 0; i < sizeof(cphy_irq); i++) {
		write_reg(phy + 0x102c + 4 * i, 0);
	}
}

static __unused void
csiphy_status(vm_offset_t phy, const char *when)
{
	for (uint32_t t = 0; t < 3; t++) {
		vm_offset_t b = phy + 0x340 + 0x400 * t;
		cam_log("cam_probe: CSIPHY1 trio %u %s: %#x %#x %#x %#x %#x %#x %#x %#x\n", t, when, read_reg(b),
		    read_reg(b + 4), read_reg(b + 8), read_reg(b + 12), read_reg(b + 16), read_reg(b + 20),
		    read_reg(b + 24), read_reg(b + 28));
	}
	cam_log("cam_probe: CSIPHY1 IRQ status %#x %#x %#x %#x\n", read_reg(phy + 0x10b0), read_reg(phy + 0x10b4),
	    read_reg(phy + 0x10b8), read_reg(phy + 0x10bc));
}

/* One queue-1 transaction on CCI0 master 0; the status word, or 0 if the queue was busy. */
static uint32_t
cci_run(vm_offset_t cci, const uint32_t *commands, unsigned count, uint32_t done)
{
	if (read_reg(cci + CCI_M0_Q1_COUNT)) {
		return 0;
	}
	cci_clear(cci, CCI_MASK_0);
	for (unsigned i = 0; i < count; i++) {
		write_reg(cci + CCI_M0_Q1_LOAD, commands[i]);
	}
	write_reg(cci + CCI_M0_Q1_EXEC, read_reg(cci + CCI_M0_Q1_COUNT));
	write_reg(cci + CCI_START, 1u << 1);
	/*
	 * Spin on the clock, not delay(1): each delay(1) took far longer than a microsecond, so a
	 * 0.4 ms write cost about 9 ms (level 4: 486 table writes in 4.6 s).
	 */
	uint32_t status = 0;
	uint64_t deadline;
	nanoseconds_to_absolutetime((uint64_t)CCI_TIMEOUT_US * NSEC_PER_USEC, &deadline);
	deadline += mach_absolute_time();
	do {
		status = read_reg(cci + CCI_IRQ_STATUS_0);
		if (status & (done | CCI_M0_ERROR)) {
			break;
		}
	} while (mach_absolute_time() < deadline);
	return status;
}

static bool
cci_recover(vm_offset_t cci, uint32_t status)
{
	cci_clear(cci, status);
	write_reg(cci + CCI_HALT, 1);
	(void)poll_reg(cci + CCI_IRQ_STATUS_0, CCI_HALT_ACK, CCI_HALT_ACK, CCI_TIMEOUT_US);
	if (!cci_reset(cci, false)) {
		return false;
	}
	cci_timing(cci);
	return true;
}

/* Word address, byte data, as i2c-qcom-cci.c cci_i2c_write: SET_PARAM, WRITE, REPORT. */
static bool
cci_write8(vm_offset_t cci, uint16_t reg, uint8_t value)
{
	const uint32_t commands[] = {
		0x1a1,
		9u | 3u << 4 | (uint32_t)(reg >> 8) << 8 | (uint32_t)(reg & 0xffu) << 16 | (uint32_t)value << 24,
		8u | 1u << 8,
	};
	for (unsigned attempt = 0; attempt < 3; attempt++) {
		uint32_t status = cci_run(cci, commands, 3, CCI_M0_Q1_REPORT);
		if ((status & (CCI_M0_Q1_REPORT | CCI_M0_ERROR)) == CCI_M0_Q1_REPORT) {
			cci_clear(cci, status);
			return true;
		}
		cam_log("cam_probe: CCI write %#06x = %#04x attempt %u, IRQ status %#x\n", reg, value, attempt + 1, status);
		if (!cci_recover(cci, status)) {
			return false;
		}
	}
	return false;
}

static __unused bool
cci_read8(vm_offset_t cci, uint16_t reg, uint8_t *value)
{
	const uint32_t commands[] = {
		0x1a1, 6, 0xbu | 2u << 4 | (uint32_t)(reg >> 8) << 8 | (uint32_t)(reg & 0xffu) << 16, 0xau | 1u << 4, 7,
	};
	for (unsigned attempt = 0; attempt < 3; attempt++) {
		uint32_t status = cci_run(cci, commands, 5, CCI_M0_READ_DONE);
		uint32_t words = read_reg(cci + CCI_M0_READ_LEVEL);
		if ((status & (CCI_M0_READ_DONE | CCI_M0_ERROR)) == CCI_M0_READ_DONE && words >= 1) {
			uint32_t data = read_reg(cci + CCI_M0_READ_DATA);
			for (uint32_t i = 1; i < words && i < 16; i++) {
				(void)read_reg(cci + CCI_M0_READ_DATA);
			}
			cci_clear(cci, status);
			*value = (uint8_t)(data >> 8);
			return true;
		}
		if (!cci_recover(cci, status)) {
			return false;
		}
	}
	return false;
}

static __unused bool
cci_table(vm_offset_t cci, const struct cam_reg *regs, unsigned count, const char *name)
{
	for (unsigned i = 0; i < count; i++) {
		if (!cci_write8(cci, regs[i].addr, regs[i].data)) {
			cam_log("cam_probe: IMX789 %s write %u/%u (%#06x = %#04x) failed\n", name, i + 1, count,
			    regs[i].addr, regs[i].data);
			return false;
		}
		if (regs[i].delay_us) {
			spin_us(regs[i].delay_us);
		}
	}
	return true;
}

/*
 * Other devices on CCI0 master 0 (7-bit addresses): write up to 7 bytes (register address first);
 * read up to 8 bytes after a register address of 1 or 2 bytes. As i2c-qcom-cci.c: the read FIFO's
 * first byte is the slave's, then the data.
 */
static bool
cci_dev_write(vm_offset_t cci, uint8_t slave, const uint8_t *bytes, unsigned n)
{
	uint32_t commands[4] = { 1u | (uint32_t)slave << 4, 9u | n << 4 };
	unsigned count;
	if (n == 0 || n > 7) {
		return false;
	}
	for (unsigned i = 0; i < n; i++) {
		unsigned at = i + 1; /* byte position after the command byte */
		commands[1 + at / 4] |= (uint32_t)bytes[i] << (8 * (at % 4));
	}
	count = n > 3 ? 3 : 2;
	commands[count++] = 8u | 1u << 8; /* REPORT */
	for (unsigned attempt = 0; attempt < 2; attempt++) {
		uint32_t status = cci_run(cci, commands, count, CCI_M0_Q1_REPORT);
		if ((status & (CCI_M0_Q1_REPORT | CCI_M0_ERROR)) == CCI_M0_Q1_REPORT) {
			cci_clear(cci, status);
			return true;
		}
		if (!cci_recover(cci, status)) {
			return false;
		}
	}
	return false;
}

static bool
cci_dev_read(vm_offset_t cci, uint8_t slave, uint16_t reg, unsigned reg_bytes, uint8_t *out, unsigned n)
{
	uint32_t address = reg_bytes == 2 ? (uint32_t)(reg >> 8) << 8 | (uint32_t)(reg & 0xffu) << 16 : (uint32_t)(reg & 0xffu) << 8;
	const uint32_t commands[] = { 1u | (uint32_t)slave << 4, 6, 0xbu | reg_bytes << 4 | address, 0xau | n << 4, 7 };
	if (n == 0 || n > 8 || reg_bytes < 1 || reg_bytes > 2) {
		return false;
	}
	for (unsigned attempt = 0; attempt < 2; attempt++) {
		uint32_t status = cci_run(cci, commands, 5, CCI_M0_READ_DONE);
		uint32_t words = read_reg(cci + CCI_M0_READ_LEVEL);
		if ((status & (CCI_M0_READ_DONE | CCI_M0_ERROR)) == CCI_M0_READ_DONE && words * 4 >= n + 1) {
			unsigned got = 0;
			for (uint32_t w = 0; w < words && w < 16; w++) {
				uint32_t data = read_reg(cci + CCI_M0_READ_DATA);
				for (unsigned b = 0; b < 4; b++) {
					unsigned at = w * 4 + b;
					if (at >= 1 && got < n) {
						out[got++] = (uint8_t)(data >> (8 * b));
					}
				}
			}
			cci_clear(cci, status);
			return got == n;
		}
		if (!cci_recover(cci, status)) {
			return false;
		}
	}
	return false;
}

/*
 * The rear main camera's lens: the AK7377 voice-coil driver at 0x18 (7-bit 0x0c) on CCI0 master 0
 * next to the IMX789. The sequences are the stock module file's (com.qti.sensormodule.waipio.
 * semco_imx789_MP.bin, actuator imx789_ak7377): init writes 0x02 = 0x40, 1 ms, 0x02 = 0x00, 1 ms; a
 * move writes the ten-bit code shifted left by 6 as a big-endian word at register 0x00; its tuning
 * starts the lens at code 400. Its supplies (VIO, VAF) are on with the sensor's. Frames give the
 * code itself as the lens's place (0, the furthest, to 1023); focus is found by contrast
 * (camera_device.c), so no AF calibration is used: CamX takes the codes from the LC898128 OIS's data
 * (its calibration buffer is that chip's 0x1c00 and 0x2400 areas, then the EEPROM), and plain register
 * reads there give the same 8 bytes at every address. The lens is walked up to its start and, at the
 * end, down to 0 in steps (the module has no park table), then the chip is put in standby.
 *
 * The module's P24C128E EEPROM (0xa0, 7-bit 0x50) is read once: its first 256 bytes are in
 * kern.hakios.cam_eeprom (from 0x04, the PDAF gain map's 17 x 13), for the colour work to come.
 */
#define AF_SLAVE        0x0c
#define AF_EEPROM       0x50
#define AF_START_CODE   400u
#define AF_STEP         48u

static bool af_eeprom_read;
static char af_eeprom_hex[2 * 0x100 + 1] = "";
SYSCTL_DECL(_kern_hakios);
SYSCTL_STRING(_kern_hakios, OID_AUTO, cam_eeprom, CTLFLAG_RD | CTLFLAG_LOCKED, af_eeprom_hex, 0,
    "the rear module EEPROM's first 256 bytes, hex");
static uint32_t af_code = HAKIOS_CAMERA_NO_FOCUS; /* the lens's code and place: HAKIOS_CAMERA_NO_FOCUS while it is not driven */

static bool
af_write_code(vm_offset_t cci, uint32_t code)
{
	code = MIN(code, 1023u);
	uint8_t out[3] = { 0x00, (uint8_t)(code << 6 >> 8), (uint8_t)(code << 6) };
	if (!cci_dev_write(cci, AF_SLAVE, out, 3)) {
		return false;
	}
	af_code = code;
	return true;
}

/* Walk the lens to code, a step at a time (thrown across its travel a lens knocks against its stop). */
static bool
af_walk(vm_offset_t cci, uint32_t code, uint32_t pause_us)
{
	while (af_code != code) {
		uint32_t next = af_code < code ? MIN(af_code + AF_STEP, code) : (af_code - code > AF_STEP ? af_code - AF_STEP : code);
		if (!af_write_code(cci, next)) {
			return false;
		}
		if (next != code) {
			spin_us(pause_us);
		}
	}
	return true;
}

static void
af_read_eeprom(vm_offset_t cci)
{
	uint8_t data[0x100];
	if (af_eeprom_read) {
		return;
	}
	af_eeprom_read = true;
	for (unsigned at = 0; at < sizeof(data); at += 8) {
		if (!cci_dev_read(cci, AF_EEPROM, (uint16_t)at, 2, &data[at], 8)) {
			cam_log("cam_probe: focus: the EEPROM does not answer at %#x\n", at);
			return;
		}
	}
	for (unsigned i = 0; i < sizeof(data); i++) {
		snprintf(af_eeprom_hex + 2 * i, 3, "%02x", data[i]);
	}
}

/* With the sensor up: the AK7377 out of standby and the lens at its starting code. */
static __unused void
af_up(vm_offset_t cci)
{
	static const uint8_t standby[2] = { 0x02, 0x40 }, active[2] = { 0x02, 0x00 };
	af_code = HAKIOS_CAMERA_NO_FOCUS;
	af_read_eeprom(cci);
	if (!cci_dev_write(cci, AF_SLAVE, standby, 2)) {
		cam_log("cam_probe: focus: the AK7377 does not answer: no focus in this session\n");
		return;
	}
	spin_us(1000);
	if (!cci_dev_write(cci, AF_SLAVE, active, 2)) {
		cam_log("cam_probe: focus: the AK7377 stopped answering in its init\n");
		return;
	}
	spin_us(1000);
	af_code = 0; /* (at rest) */
	if (!af_walk(cci, AF_START_CODE, 2000)) {
		cam_log("cam_probe: focus: the AK7377 stopped answering on the way to code %u\n", AF_START_CODE);
		af_code = HAKIOS_CAMERA_NO_FOCUS;
		return;
	}
	cam_log("cam_probe: focus: AK7377 up, the lens at code %u\n", af_code);
}

static void
af_move(vm_offset_t cci, uint32_t code)
{
	if (af_code != HAKIOS_CAMERA_NO_FOCUS && MIN(code, 1023u) != af_code) {
		(void)af_write_code(cci, code);
	}
}

static void
af_park(vm_offset_t cci)
{
	static const uint8_t standby[2] = { 0x02, 0x40 };
	if (af_code != HAKIOS_CAMERA_NO_FOCUS) {
		uint32_t from = af_code;
		bool walked = af_walk(cci, 0, 5000);
		bool standby_ok = cci_dev_write(cci, AF_SLAVE, standby, 2);
		cam_log("cam_probe: focus: lens parked from code %u to %u%s, standby %s\n", from, af_code, walked ? "" : " (stopped answering)",
		    standby_ok ? "OK" : "FAILED");
	}
	af_code = HAKIOS_CAMERA_NO_FOCUS;
}

/* Map a wired, page-backed buffer through the IFE context: one PTE clean and one TLB sync for all of it. */
static int
map_buffer(struct hakios_smmu_context *ctx, uint64_t iova, vm_offset_t kva, vm_size_t size, vm_size_t *mapped)
{
	int error = hakios_smmu_map_kva(ctx, iova, kva, size, HAKIOS_SMMU_READ | HAKIOS_SMMU_WRITE);
	*mapped = error ? 0 : size;
	return error;
}

struct raw_stats {
	uint32_t written_permille, lo, hi;
	uint32_t mean[4]; /* 10-bit means of the 2x2 phases: row 0 col 0, row 0 col 1, row 1 col 0, row 1 col 1 */
};

static __unused void
raw_stats_geom(const uint8_t *frame, uint32_t width, uint32_t height, uint32_t stride, struct raw_stats *s)
{
	uint64_t changed = 0, bytes = (uint64_t)stride * height;
	for (uint64_t i = 0; i < bytes; i++) {
		changed += frame[i] != TPG_FILL;
	}
	s->written_permille = (uint32_t)(changed * 1000 / bytes);
	uint64_t sum[4] = { 0 };
	uint32_t n[4] = { 0 };
	s->lo = 1023;
	s->hi = 0;
	/* Every 16th row pair and every 8th five-byte group: about 190,000 samples in mode 1. */
	for (uint32_t row = 0; row + 1 < height; row += 16) {
		for (uint32_t r = 0; r < 2; r++) {
			const uint8_t *line = frame + (row + r) * stride;
			for (uint32_t g = 0; g < width / 4; g += 8) {
				const uint8_t *b = line + 5 * g;
				for (uint32_t k = 0; k < 4; k++) {
					uint32_t v = (uint32_t)b[k] << 2 | ((b[4] >> (2 * k)) & 3u);
					sum[2 * r + (k & 1)] += v;
					n[2 * r + (k & 1)]++;
					s->lo = MIN(s->lo, v);
					s->hi = MAX(s->hi, v);
				}
			}
		}
	}
	for (unsigned p = 0; p < 4; p++) {
		s->mean[p] = n[p] ? (uint32_t)(sum[p] / n[p]) : 0;
	}
}

static __unused void
raw_stats(const uint8_t *frame, struct raw_stats *s)
{
	raw_stats_geom(frame, RAW_WIDTH, RAW_HEIGHT, RAW_STRIDE, s);
}

/* Mean of a 64 x 64 pixel block, in 1/16 counts (all four Bayer phases). */
static __unused uint32_t
block_mean16(const uint8_t *frame, uint32_t row, uint32_t col)
{
	uint64_t sum = 0;
	for (uint32_t r = row; r < row + 64; r++) {
		const uint8_t *line = frame + r * RAW_STRIDE;
		for (uint32_t c = col; c < col + 64; c++) {
			const uint8_t *b = line + 5 * (c / 4);
			uint32_t k = c % 4;
			sum += (uint32_t)b[k] << 2 | ((b[4] >> (2 * k)) & 3u);
		}
	}
	return (uint32_t)(sum * 16 / (64 * 64));
}

static uint32_t
raw_mean(const struct raw_stats *s)
{
	return (s->mean[0] + s->mean[1] + s->mean[2] + s->mean[3]) / 4;
}

struct sensor_hw {
	vm_offset_t csid, ife, phy, cci, camnoc;
};

/* After the write master has stopped: CSID0's RX and RDI0 off, CSIPHY1 in reset, CSID0 reset. */
static bool
rdi_teardown(const struct sensor_hw *hw)
{
	write_reg(hw->csid + CSID_RDI_CFG0, 0);
	write_reg(hw->csid + CSID_RX_CFG0, 0);
	write_reg(hw->csid + CSID_RX_CFG1, 0);
	write_reg(hw->csid + CSID_RX_CAPTURE, 0);
	write_reg(hw->csid + CSID_RDI_MASK, 0);
	write_reg(hw->csid + CSID_BUF_MASK, 0);
	write_reg(hw->csid + CSID_RX_MASK, 0);
	csiphy_stop(hw->phy);
	return csid_reset(hw->csid);
}

/*
 * Arm RDI0 for frames frames into slot onward (each next buffer frame_incr further on: a frame
 * steps through the slots, 0 overwrites the slot), turn the sensor stream on, wait for the
 * buffer-done IRQs, halt RDI0 at the next frame boundary and turn the stream off.
 */
static __unused bool
sensor_capture_geom(const struct sensor_hw *hw, uint32_t iova, uint32_t stride, const struct cphy_rate *rate, unsigned frames,
    uint32_t frame_incr, unsigned settle, uint64_t intervals_ns[2], uint8_t counts[2], unsigned *done_count, bool *streaming)
{
	vm_offset_t wm = hw->ife + IFE_WM_RDI0;
	*done_count = 0;
	if (!csid_reset(hw->csid)) {
		cam_log("cam_probe: CSID reset before capture failed\n");
		return false;
	}
	csiphy_start_rate(hw->phy, settle, rate);
	cam_rdi_start(hw->csid, wm, stride, 0, iova, SENSOR_RX_CFG0, rx_capture(1));
	cam_log("cam_probe: sensor RX %#x RDI %#x, WM cfg %#x IOVA %#x incr %#x\n", read_reg(hw->csid + CSID_RX_CFG0),
	    read_reg(hw->csid + CSID_RDI_CFG0), read_reg(wm + WM_CFG), read_reg(wm + WM_ADDRESS), read_reg(wm + WM_FRAME_INCR));
	*streaming = true;
	bool on = cci_write8(hw->cci, IMX789_MODE_SELECT, 1);
	if (!on) {
		cam_log("cam_probe: IMX789 stream on failed\n");
	}
	uint64_t start = mach_absolute_time(), last = start, deadline;
	clock_interval_to_deadline(1000, NSEC_PER_MSEC, &deadline);
	while (on && *done_count < frames && mach_absolute_time() < deadline) {
		uint32_t status = read_reg(hw->csid + CSID_BUF_STATUS);
		if (status == UINT32_MAX) {
			break;
		}
		if (status & RDI0_BUF_DONE) {
			uint64_t now = mach_absolute_time();
			if (*done_count + 1 == frames) {
				write_reg(hw->csid + CSID_RDI_CTRL, 0); /* halt at the next frame boundary */
			} else {
				/*
				 * The write master takes one buffer per address update: without the next one it
				 * drops the frame (the first boot got one frame of two). Give it in the vertical
				 * blanking that follows buffer done (136 lines, about 1.4 ms in mode 1; 178 lines,
				 * about 0.96 ms in mode 0).
				 */
				write_reg(wm + WM_ADDRESS, iova + (*done_count + 1) * frame_incr);
				write_reg(hw->csid + CSID_RUP_AUP, 1u << 4 | 1u << 20);
			}
			write_reg(hw->csid + CSID_BUF_CLEAR, RDI0_BUF_DONE);
			write_reg(hw->csid + CSID_IRQ_CMD, 1);
			if (*done_count < 2) {
				absolutetime_to_nanoseconds(now - last, &intervals_ns[*done_count]);
			}
			last = now;
			(*done_count)++;
			/* The sensor's frame counter, read while it streams: after the first and the last frame. */
			if (*done_count == 1 || *done_count == frames) {
				(void)cci_read8(hw->cci, IMX789_FRAME_COUNT, &counts[*done_count == 1 ? 0 : 1]);
			}
			continue;
		}
		spin_us(10);
	}
	uint32_t packets = read_reg(hw->csid + 0x240), rx = read_reg(hw->csid + CSID_RX_STATUS);
	csiphy_status(hw->phy, "streaming");
	write_reg(hw->csid + CSID_RDI_CTRL, 0);
	if (cci_write8(hw->cci, IMX789_MODE_SELECT, 0)) {
		*streaming = false;
	}
	delay(40000);
	write_reg(wm + WM_CFG, 0);
	bool stopped = poll_reg(wm + WM_CFG, WM_ENABLE, 0, 500);
	uint32_t violation = read_reg(hw->ife + 0xc64), overflow = read_reg(hw->ife + 0xc68);
	uint32_t image_error = read_reg(hw->ife + 0xc70), crc = read_reg(hw->csid + 0x248);
	cam_log("cam_probe: settle %u: %u/%u frames, CSID buf %#x RDI %#x RX %#x top %#x, packets %u CRC %u ECC %#x\n",
	    settle, *done_count, frames, read_reg(hw->csid + CSID_BUF_STATUS), read_reg(hw->csid + CSID_RDI_STATUS), rx,
	    read_reg(hw->csid + CSID_TOP_STATUS), packets, crc, read_reg(hw->csid + 0x244));
	cam_log("cam_probe: long %#x/%#x short %#x/%#x C-PHY %#x unmapped %#x, IFE violation %#x overflow %#x image %#x, WM %#x\n",
	    read_reg(hw->csid + 0x220), read_reg(hw->csid + 0x224), read_reg(hw->csid + 0x218), read_reg(hw->csid + 0x21c),
	    read_reg(hw->csid + 0x22c), read_reg(hw->csid + 0x210), violation, overflow, image_error, read_reg(wm + 0x70));
	cam_log("cam_probe: CAMNOC fault %#x errvld %#x errlog %#x:%#x\n", read_reg(hw->camnoc + 0x2248),
	    read_reg(hw->camnoc + 0x2010), read_reg(hw->camnoc + 0x2024), read_reg(hw->camnoc + 0x2020));
	bool reset = rdi_teardown(hw);
	return on && stopped && reset && !*streaming && *done_count == frames && !violation && !overflow && !image_error &&
	       !crc && !(read_reg(hw->camnoc + 0x2010) & 1u);
}

static __unused bool
sensor_capture(const struct sensor_hw *hw, uint32_t slot, unsigned frames, uint32_t frame_incr, unsigned settle,
    uint64_t intervals_ns[2], uint8_t counts[2], unsigned *done_count, bool *streaming)
{
	return sensor_capture_geom(hw, (uint32_t)IFE_IOVA + slot * RAW_FRAME_BYTES, RAW_STRIDE, &cphy_rate0, frames, frame_incr,
	    settle, intervals_ns, counts, done_count, streaming);
}

#if CAM_HAVE_TABLES
static bool
find_reg(const struct cam_reg *regs, unsigned count, uint16_t addr, uint8_t *value)
{
	for (unsigned i = count; i-- > 0;) {
		if (regs[i].addr == addr) {
			*value = regs[i].data;
			return true;
		}
	}
	return false;
}
#endif

struct sensor_result {
	bool captured, short_captured, counted, exposure_judged, exposure_follows;
	unsigned settle;
	uint8_t count[2], short_count[2];
	uint64_t interval_ns;
	uint32_t coarse, short_coarse;
	struct raw_stats stats[RAW_SLOTS];
	uint32_t frames_differ;
	uint32_t block_row, block_col, long_block16, short_block16;
};

static bool
sensor_frames(const struct sensor_hw *hw, struct sensor_result *r, bool *streaming, vm_offset_t kva)
{
#if CAM_HAVE_TABLES
	cam_log("cam_probe: IMX789 tables from %s mode %u: init %u, mode %u writes\n", CAM_TABLES_MODULE, CAM_TABLES_MODE,
	    (unsigned)(sizeof(cam_init_regs) / sizeof(cam_init_regs[0])), (unsigned)(sizeof(cam_mode_regs) / sizeof(cam_mode_regs[0])));
	if (!cci_table(hw->cci, cam_init_regs, sizeof(cam_init_regs) / sizeof(cam_init_regs[0]), "init") ||
	    !cci_table(hw->cci, cam_mode_regs, sizeof(cam_mode_regs) / sizeof(cam_mode_regs[0]), "mode")) {
		return false;
	}
	uint8_t hi = 0, lo = 0;
	if (!find_reg(cam_mode_regs, sizeof(cam_mode_regs) / sizeof(cam_mode_regs[0]), 0x0202, &hi) ||
	    !find_reg(cam_mode_regs, sizeof(cam_mode_regs) / sizeof(cam_mode_regs[0]), 0x0203, &lo)) {
		cam_log("cam_probe: mode table has no coarse integration time\n");
		return false;
	}
	r->coarse = (uint32_t)hi << 8 | lo;

	/* Two consecutive frames into slots 0 and 1. Retry the PHY with other settle counts on no frame. */
	static const unsigned settles[] = { CAM_TABLES_SETTLE, 36, 28, 64 };
	uint64_t intervals[2] = { 0, 0 };
	unsigned done = 0;
	for (unsigned i = 0; i < sizeof(settles) / sizeof(settles[0]) && !r->captured; i++) {
		r->settle = settles[i];
		r->captured = sensor_capture(hw, 0, 2, RAW_FRAME_BYTES, settles[i], intervals, r->count, &done, streaming);
		if (*streaming || (!r->captured && done)) {
			break; /* stream off failed, or frames came with errors: a different settle is not the fix */
		}
	}
	r->interval_ns = intervals[1];
	r->counted = r->count[1] != r->count[0];
	if (!r->captured) {
		return false;
	}

	/*
	 * An eighth of the exposure under group hold, then two frames into slot 2 with frame
	 * increment 0: the second overwrites the first, in case the first still had the old exposure.
	 */
	r->short_coarse = MAX(r->coarse / 8, 8u);
	if (!cci_write8(hw->cci, 0x0104, 1) || !cci_write8(hw->cci, 0x0202, (uint8_t)(r->short_coarse >> 8)) ||
	    !cci_write8(hw->cci, 0x0203, (uint8_t)r->short_coarse) || !cci_write8(hw->cci, 0x0104, 0)) {
		return false;
	}
	uint64_t ignore[2];
	r->short_captured = sensor_capture(hw, 2, 2, 0, r->settle, ignore, r->short_count, &done, streaming);
	flush_dcache(kva, (unsigned)RAW_BYTES, FALSE);
	for (unsigned s = 0; s < RAW_SLOTS; s++) {
		raw_stats((const uint8_t *)kva + s * RAW_FRAME_BYTES, &r->stats[s]);
		cam_log("cam_probe: frame %u: %u/1000 written, 10-bit range %u-%u, 2x2 means %u %u / %u %u\n", s,
		    r->stats[s].written_permille, r->stats[s].lo, r->stats[s].hi, r->stats[s].mean[0], r->stats[s].mean[1],
		    r->stats[s].mean[2], r->stats[s].mean[3]);
	}
	const uint8_t *a = (const uint8_t *)kva, *b = a + RAW_FRAME_BYTES;
	for (uint32_t i = 0; i < RAW_FRAME_BYTES; i += 97) {
		r->frames_differ += a[i] != b[i];
	}
	/*
	 * Judge the exposure on the brightest 64 x 64 block of the long frame, the same block in the
	 * short one: a dim room leaves the whole-frame mean a few counts above black (the fourth boot:
	 * mean 67 -> 64, while the lamp's block went 12.5-24.5 -> 1.6-3.1 above black, the 8x asked).
	 */
	const uint8_t *frame0 = (const uint8_t *)kva, *frame2 = frame0 + 2 * RAW_FRAME_BYTES;
	uint32_t best = 0;
	for (uint32_t row = 32; row + 32 < RAW_HEIGHT; row += 64) {
		for (uint32_t col = 32; col + 32 < RAW_WIDTH; col += 64) {
			uint32_t level = block_mean16(frame0, row - 32, col - 32);
			if (level > best) {
				best = level;
				r->block_row = row - 32;
				r->block_col = col - 32;
			}
		}
	}
	r->long_block16 = best;
	r->short_block16 = block_mean16(frame2, r->block_row, r->block_col);
	r->exposure_judged = r->long_block16 >= 16 * (RAW_BLACK + 8) && r->long_block16 < 16 * 900;
	r->exposure_follows = r->short_block16 < 16 * RAW_BLACK + (r->long_block16 - 16 * RAW_BLACK) / 2;
	return r->short_captured;
#else
	(void)hw;
	(void)r;
	(void)streaming;
	(void)kva;
	cam_log("cam_probe: this kernel has no IMX789 tables (tools/camsensor.py --c-header hakios.cam_imx789.h 1)\n");
	return false;
#endif
}

/*
 * Everything between the sensor and memory, without the sensor's tables: the MMIO mappings, the
 * camera votes and clocks, CamCC PLL0 and PLL3, the capture clock sources, IFE0's power domain, the
 * capture and CSIPHY1 gates, CCI0, the sensor's power and chip ID, CSID0 routed to IFE0 and the RDI
 * QoS. sensor_path_down() undoes what sensor_path_up() did, however far it got.
 */
struct sensor_path {
	struct cam_probe cam;
	struct hakios_cam_power power;
	struct clock_source sources[sizeof(sensor_sources) / sizeof(sensor_sources[0])];
	bool pll_ours[sizeof(cam_plls) / sizeof(cam_plls[0])], pll_tried[sizeof(cam_plls) / sizeof(cam_plls[0])];
	uint32_t gates, gdsc, saved_route, saved_qos[sizeof(rdi_qos) / sizeof(rdi_qos[0])];
	bool gdsc_changed, routed;
	vm_offset_t cpas, top;
	struct sensor_hw hw;
	const struct cam_pll *plls; /* cam_plls unless set */
	const char *step;
	uint64_t step_abs;
};

/* The next step of sensor_path_up(); the one before is logged when it took 20 ms or more. */
static void
path_step(struct sensor_path *p, const char *next)
{
	uint64_t now = mach_absolute_time(), ns = 0;
	if (p->step && p->step_abs) {
		absolutetime_to_nanoseconds(now - p->step_abs, &ns);
		if (ns >= 20 * NSEC_PER_MSEC) {
			cam_log("cam_probe: path: %s took %llu ms\n", p->step, ns / NSEC_PER_MSEC);
		}
	}
	p->step = next;
	p->step_abs = now;
}

static bool
sensor_path_up(struct sensor_path *p)
{
	struct cam_probe *cam = &p->cam;
	path_step(p, "MMIO mappings");
	cam->gcc = map_registers(GCC_PA, 0x1000);
	cam->camcc = map_registers(CAMCC_PA, CAMCC_SIZE);
	cam->cci = map_registers(CCI_PA, 0x1000);
	cam->tlmm = map_registers(TLMM_PA, TLMM_SIZE);
	p->cpas = map_registers(CPAS_PA, 0x1000);
	p->hw.camnoc = map_registers(CAMNOC_PA, CAMNOC_SIZE);
	p->hw.csid = map_registers(CSID_PA, 0x1000);
	p->top = map_registers(CSID_TOP_PA, 0x1000);
	p->hw.ife = map_registers(IFE_PA, IFE_SIZE);
	p->hw.phy = map_registers(CSIPHY1_PA, CSIPHY_SIZE);
	p->hw.cci = cam->cci;
	if (!cam->gcc || !cam->camcc || !cam->cci || !cam->tlmm || !p->cpas || !p->hw.camnoc || !p->hw.csid || !p->top ||
	    !p->hw.ife || !p->hw.phy) {
		return false;
	}
	path_step(p, "MMCX, MXC and camera NoC votes");
	cam_log("cam_probe: requesting %s\n", p->step);
	if (!hakios_cam_power_vote(&p->power, HAKIOS_RPMH_CAMERA, true)) {
		return false;
	}
	path_step(p, "Titan camera clocks");
	if (!cam_clocks_up(cam)) {
		return false;
	}
	path_step(p, "capture clocks busy");
	for (unsigned i = 0; i < sizeof(sensor_gates) / sizeof(sensor_gates[0]); i++) {
		if (read_reg(cam->camcc + sensor_gates[i]) & CLK_ENABLE) {
			return false;
		}
	}
	path_step(p, "CamCC PLL0 and PLL3");
	if (!p->plls) {
		p->plls = cam_plls;
	}
	for (unsigned i = 0; i < sizeof(cam_plls) / sizeof(cam_plls[0]); i++) {
		p->pll_tried[i] = true;
		if (!pll_up(cam->camcc, &p->plls[i], &p->pll_ours[i])) {
			return false;
		}
	}
	path_step(p, "sensor clock sources");
	for (unsigned i = 0; i < sizeof(sensor_sources) / sizeof(sensor_sources[0]); i++) {
		vm_offset_t cmd = cam->camcc + sensor_sources[i].cmd;
		if (i != CAMNOC_AXI_SOURCE) {
			p->sources[i].rate = read_reg(cmd + 4) & RCG_RATE_MASK;
		}
		p->sources[i].changed = true;
		if (!source_set(cmd, sensor_sources[i].rate)) {
			cam_log("cam_probe: clock source %#x did not take %#x\n", sensor_sources[i].cmd, sensor_sources[i].rate);
			return false;
		}
	}
	path_step(p, "IFE0 power domain");
	p->gdsc = read_reg(cam->camcc + IFE_GDSCR);
	if (p->gdsc == UINT32_MAX || (p->gdsc & GDSC_HW_CONTROL)) {
		return false;
	}
	p->gdsc_changed = true;
	write_reg(cam->camcc + IFE_GDSCR, (p->gdsc & ~GDSC_COLLAPSE) | GDSC_RETAIN);
	if (!poll_reg(cam->camcc + IFE_GDSCR, GDSC_POWERED, GDSC_POWERED, 500)) {
		return false;
	}
	path_step(p, "capture and CSIPHY1 clock gates");
	if (!gates_acquire(cam->camcc, sensor_gates, sizeof(sensor_gates) / sizeof(sensor_gates[0]), 0, &p->gates)) {
		return false;
	}
	path_step(p, "capture block versions or active DMA");
	cam_log("cam_probe: camera %#x, CSID %#x, IFE bus %#x, CSIPHY1 %#x\n", read_reg(p->cpas), read_reg(p->hw.csid),
	    read_reg(p->hw.ife + 0xc00), read_reg(p->hw.phy + 0x10a4));
	if (read_reg(p->cpas) != 0x00060800 || !read_reg(p->hw.csid) || read_reg(p->hw.csid) == UINT32_MAX ||
	    (read_reg(p->hw.ife + IFE_WM_RDI0 + WM_CFG) & WM_ENABLE) || (read_reg(p->hw.csid + CSID_RDI_CFG0) & RDI_ENABLE)) {
		return false;
	}
	path_step(p, "CCI0 reset");
	if (!cci_start(cam)) {
		return false;
	}
	path_step(p, "sensor power and chip ID");
	if (!sensor_probe(cam)) {
		return false;
	}
	p->saved_route = read_reg(p->top);
	p->routed = true;
	write_reg(p->top, 0x101); /* CSID0 to full IFE0 */
	for (unsigned i = 0; i < sizeof(rdi_qos) / sizeof(rdi_qos[0]); i++) {
		p->saved_qos[i] = read_reg(p->hw.camnoc + rdi_qos[i].reg);
		write_reg(p->hw.camnoc + rdi_qos[i].reg, rdi_qos[i].value);
	}
	path_step(p, "CSID0 routed to IFE0");
	return true;
}

/* Stream off first if *streaming; true if everything went back as it was. */
static bool
sensor_path_down(struct sensor_path *p, bool *streaming)
{
	struct cam_probe *cam = &p->cam;
	if (*streaming) {
		*streaming = !cci_write8(cam->cci, IMX789_MODE_SELECT, 0);
		delay(40000);
	}
	if (p->hw.phy && p->gates) {
		csiphy_stop(p->hw.phy);
	}
	bool restored = true;
	if (p->routed) {
		for (unsigned i = 0; i < sizeof(rdi_qos) / sizeof(rdi_qos[0]); i++) {
			write_reg(p->hw.camnoc + rdi_qos[i].reg, p->saved_qos[i]);
			restored = read_reg(p->hw.camnoc + rdi_qos[i].reg) == p->saved_qos[i] && restored;
		}
		write_reg(p->top, p->saved_route);
		restored = read_reg(p->top) == p->saved_route && restored;
	}
	if (cam->camcc) {
		restored = gates_release(cam->camcc, sensor_gates, sizeof(sensor_gates) / sizeof(sensor_gates[0]), p->gates) && restored;
	}
	if (p->gdsc_changed) {
		uint32_t mask = GDSC_COLLAPSE | GDSC_RETAIN;
		write_reg(cam->camcc + IFE_GDSCR, (read_reg(cam->camcc + IFE_GDSCR) & ~mask) | (p->gdsc & mask));
		mask |= GDSC_POWERED;
		restored = poll_reg(cam->camcc + IFE_GDSCR, mask, p->gdsc & mask, 500) && restored;
	}
	for (unsigned i = sizeof(sensor_sources) / sizeof(sensor_sources[0]); i-- > 0;) {
		if (p->sources[i].changed) {
			/* CAMNOC AXI goes back to the TCXO of level 1; cam_restore() puts back its original rate. */
			restored = source_set(cam->camcc + sensor_sources[i].cmd, p->sources[i].rate) && restored;
		}
	}
	for (unsigned i = sizeof(cam_plls) / sizeof(cam_plls[0]); i-- > 0;) {
		if (p->pll_tried[i] && p->pll_ours[i]) {
			pll_down(cam->camcc, &p->plls[i]);
		}
	}
	if (cam->gcc && cam->camcc && cam->cci && cam->tlmm) {
		restored = cam_restore(cam) && restored;
	}
	restored = hakios_cam_power_vote(&p->power, HAKIOS_RPMH_CAMERA, false) && restored;
	unmap_registers(p->hw.phy, CSIPHY_SIZE);
	unmap_registers(p->hw.ife, IFE_SIZE);
	unmap_registers(p->top, 0x1000);
	unmap_registers(p->hw.csid, 0x1000);
	unmap_registers(p->hw.camnoc, CAMNOC_SIZE);
	unmap_registers(p->cpas, 0x1000);
	unmap_registers(cam->tlmm, TLMM_SIZE);
	unmap_registers(cam->cci, 0x1000);
	unmap_registers(cam->camcc, CAMCC_SIZE);
	unmap_registers(cam->gcc, 0x1000);
	return restored;
}

static bool
cam_sensor_run(vm_offset_t kva, struct sensor_result *r, bool *stopped)
{
	struct sensor_path p = { 0 };
	bool streaming = false;
	bool ok = sensor_path_up(&p);
	if (ok) {
		p.step = "IMX789 through CSIPHY1, CSID0 and IFE0 RDI0";
		ok = sensor_frames(&p.hw, r, &streaming, kva);
	}
	if (!ok) {
		cam_log("cam_probe: sensor capture failed at %s\n", p.step);
	}
	bool restored = sensor_path_down(&p, &streaming);
	*stopped = !streaming;
	cam_log("cam_probe: sensor stream stopped %u, clocks and power restore %s\n", (unsigned)*stopped,
	    restored ? "OK" : "FAILED");
	return ok && restored && *stopped;
}

static bool
cam_sensor_capture(void)
{
	vm_size_t size = round_page(RAW_BYTES);
	vm_offset_t kva;
	if (kmem_alloc(kernel_map, &kva, size, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		cam_log("cam_probe: level 3 FAIL (frame buffer allocation)\n");
		return false;
	}
	struct hakios_smmu_context *ctx = hakios_smmu_claim(IFE_SID, IFE_SID_MASK, IFE_IOVA, size, "camera IFE");
	if (!ctx) {
		cam_log("cam_probe: level 3 FAIL (IFE stream claim)\n");
		kmem_free(kernel_map, kva, size);
		return false;
	}
	memset((void *)kva, TPG_FILL, size);
	flush_dcache(kva, (unsigned)size, FALSE);
	vm_size_t mapped = 0;
	int map_error = map_buffer(ctx, IFE_IOVA, kva, size, &mapped);
	struct sensor_result r = { 0 };
	bool stopped = true, captured = false;
	if (!map_error) {
		captured = cam_sensor_run(kva, &r, &stopped);
		captured = hakios_smmu_faults(ctx) && captured;
	}
	int unmap_error = mapped ? hakios_smmu_unmap(ctx, IFE_IOVA, mapped) : 0;
	int release_error = hakios_smmu_release(ctx);
	cam_log("cam_probe: IFE SID %#x/%#x IOVA %#llx bytes %#lx: map %d, unmap %d, release %d\n", IFE_SID, IFE_SID_MASK,
	    IFE_IOVA, (unsigned long)size, map_error, unmap_error, release_error);
	cam_log("cam_probe: IMX789 frame counter %#x -> %#x (short exposure %#x -> %#x), frame interval %llu us, "
	    "%u/%u sampled bytes differ between frames 0 and 1\n", r.count[0], r.count[1], r.short_count[0], r.short_count[1],
	    r.interval_ns / 1000, r.frames_differ, RAW_FRAME_BYTES / 97 + 1);
	cam_log("cam_probe: exposure %u -> %u lines: mean %u -> %u, brightest block at %u,%u %u.%02u -> %u.%02u (%s)\n",
	    r.coarse, r.short_coarse, raw_mean(&r.stats[0]), raw_mean(&r.stats[2]), r.block_row, r.block_col,
	    r.long_block16 / 16, r.long_block16 % 16 * 100 / 16, r.short_block16 / 16, r.short_block16 % 16 * 100 / 16,
	    !r.exposure_judged ? "scene too dark or saturated to judge" :
	    r.exposure_follows ? "follows the exposure" : "DOES NOT follow the exposure");
	bool frames_ok = captured && r.counted && r.interval_ns > 25000000 && r.interval_ns < 45000000 && r.frames_differ > 0;
	for (unsigned s = 0; s < RAW_SLOTS; s++) {
		/* 0xa5 is also a real RAW10 byte: a dark frame keeps about 0.2 % of them (fourth boot). */
		frames_ok = frames_ok && r.stats[s].written_permille >= 990 && r.stats[s].hi > r.stats[s].lo;
	}
	bool pass = frames_ok && (!r.exposure_judged || r.exposure_follows);
	/* Frames that arrived are worth a look even when a later check failed; valid says which. */
	if (!unmap_error && !release_error && stopped && r.captured) {
		static const char *const names[RAW_SLOTS] = { "hakios-camera-raw0", "hakios-camera-raw1", "hakios-camera-raw2" };
		bool published = true;
		for (unsigned s = 0; s < RAW_SLOTS; s++) {
			published = cam_frame_publish(1 + s, names[s], kva + s * RAW_FRAME_BYTES, RAW_WIDTH, RAW_HEIGHT, RAW_STRIDE, 2) &&
			    published;
			cam_frames[1 + s].header.valid = pass;
		}
		if (published) {
			cam_log("cam_probe: level 3 %s (IMX789 settle %u, dump /dev/hakios-camera-raw0-2)\n", pass ? "PASS" : "FAIL", r.settle);
			return pass;
		}
		cam_log("cam_probe: could not publish the sensor frames\n");
	}
	if (stopped && (!release_error || (!map_error && !unmap_error))) {
		kmem_free(kernel_map, kva, size);
	} else {
		cam_log("cam_probe: retaining frame buffer after uncertain SMMU teardown\n");
	}
	cam_log("cam_probe: level 3 FAIL (sensor frames or teardown)\n");
	return false;
}

/* ---- level 7: IMX789 mode 0 (8000 x 6016 RAW10) into RDI0 only (CAMERA-PLAN section 5.1) ---- */

/*
 * The module's full-resolution mode, from a fresh sensor power-up (the stock init table, then mode 0's
 * own table: mode 1 sets 0x380c and mode 0 does not, so the two are not applied over each other here).
 * Two consecutive frames at the table's exposure into slots 0 and 1, then 16x analog gain under group
 * hold and two frames into slot 1 with frame increment 0 (the second overwrites the first). Nothing goes
 * through the IFE pixel path: RDI0 writes the CSI bytes as they come, 10,000 bytes a line.
 *
 * The colour order is judged from the 4 x 4 phase means of the brighter frame: an ordinary Bayer
 * mosaic repeats every 2 pixels (phases r and r + 2 agree), an unremosaiced quad-Bayer one every 4
 * (phases inside each 2 x 2 block agree). A scene with no colour or no light cannot tell them apart.
 */
#if CAM_HAVE_TABLES && defined(CAM_HIRES_MODE)
_Static_assert(CAM_HIRES_LANES == 3 && CAM_HIRES_CPHY == 1 && CAM_HIRES_DATA_TYPE == DT_RAW10 && CAM_HIRES_WIDTH % 4 == 0,
    "level 7 expects the IMX789's three-trio C-PHY RAW10 mode 0");
_Static_assert((uint64_t)CAM_HIRES_PIXEL_CLOCK * 10u / CAM_HIRES_LANES > 6384000000ull &&
    (uint64_t)CAM_HIRES_PIXEL_CLOCK * 10u / CAM_HIRES_LANES <= 7524000000ull,
    "mode 0's data rate is not in data_rate_delta_table_2_1_0 entry 7");
_Static_assert((uint64_t)CAM_TABLES_PIXEL_CLOCK * 10u / CAM_TABLES_LANES <= 2736000000ull,
    "mode 1's data rate is not in data_rate_delta_table_2_1_0 entry 0");
#define HIRES_LINE         (CAM_HIRES_WIDTH * 10u / 8u)
#define HIRES_STRIDE       ((HIRES_LINE + 15u) & ~15u)
#define HIRES_FRAME_BYTES  (HIRES_STRIDE * CAM_HIRES_HEIGHT)
#define HIRES_SLOTS        2u
#define HIRES_BYTES        ((vm_size_t)HIRES_SLOTS * HIRES_FRAME_BYTES)
#define HIRES_GAIN         16u
_Static_assert(HIRES_FRAME_BYTES % 256 == 0, "RAW10 slots must stay 256-byte aligned for the write master");
_Static_assert(IFE_IOVA + HIRES_BYTES < 0xfff00000ull, "mode 0 slots must fit the IFE IOVA pool");

/* 10-bit means of the 16 phases (row % 4, column % 4), sampled every 32nd row quad and every 4th group. */
static void
hires_phases(const uint8_t *frame, uint32_t m[16])
{
	uint64_t sum[16] = { 0 };
	uint32_t n = 0;
	for (uint32_t row = 0; row + 3 < CAM_HIRES_HEIGHT; row += 32) {
		for (uint32_t g = 0; g < CAM_HIRES_WIDTH / 4; g += 4) {
			for (uint32_t r = 0; r < 4; r++) {
				const uint8_t *b = frame + (row + r) * HIRES_STRIDE + 5 * g;
				for (uint32_t k = 0; k < 4; k++) {
					sum[4 * r + k] += (uint32_t)b[k] << 2 | ((b[4] >> (2 * k)) & 3u);
				}
			}
			n++;
		}
	}
	for (unsigned i = 0; i < 16; i++) {
		m[i] = n ? (uint32_t)(sum[i] / n) : 0;
	}
}

static uint32_t
absdiff(uint32_t a, uint32_t b)
{
	return a > b ? a - b : b - a;
}

/* Sum over phases of the differences that an ordinary (period 2) and a quad (period 4) mosaic would not have. */
static const char *
hires_order(const uint32_t m[16], uint32_t *bayer_var, uint32_t *quad_var)
{
	*bayer_var = *quad_var = 0;
	for (uint32_t r = 0; r < 4; r++) {
		for (uint32_t c = 0; c < 4; c++) {
			*bayer_var += absdiff(m[4 * r + c], m[4 * (r ^ 2) + c]) + absdiff(m[4 * r + c], m[4 * r + (c ^ 2)]);
			*quad_var += absdiff(m[4 * r + c], m[4 * (r ^ 1) + c]) + absdiff(m[4 * r + c], m[4 * r + (c ^ 1)]);
		}
	}
	if (*quad_var < 32 && *bayer_var < 32) {
		return "too flat to judge";
	}
	return *bayer_var * 3 < *quad_var ? "ordinary 2 x 2 Bayer" : *quad_var * 3 < *bayer_var ? "QUAD Bayer (not remosaiced)" :
	       "undetermined";
}

struct hires_result {
	bool captured, gain_captured, counted;
	unsigned settle, done, gain_done;
	uint8_t count[2], gain_count[2];
	uint64_t interval_ns, first_ns;
	struct raw_stats stats[HIRES_SLOTS];
	uint32_t frames_differ, phases[2][16], bayer_var[2], quad_var[2];
	const char *order[2];
};

static bool
hires_frames(const struct sensor_hw *hw, struct hires_result *r, bool *streaming, vm_offset_t kva)
{
	cam_log("cam_probe: hires: IMX789 tables from %s mode %u: init %u, mode %u writes (%u x %u, %u Mpix/s, %u.%03u fps, %s)\n",
	    CAM_TABLES_MODULE, CAM_HIRES_MODE, (unsigned)(sizeof(cam_init_regs) / sizeof(cam_init_regs[0])),
	    (unsigned)(sizeof(cam_hires_regs) / sizeof(cam_hires_regs[0])), CAM_HIRES_WIDTH, CAM_HIRES_HEIGHT,
	    CAM_HIRES_PIXEL_CLOCK / 1000000u, CAM_HIRES_FPS_MILLI / 1000u, CAM_HIRES_FPS_MILLI % 1000u, CAM_HIRES_CFA);
	uint64_t start = mach_absolute_time();
	if (!cci_table(hw->cci, cam_init_regs, sizeof(cam_init_regs) / sizeof(cam_init_regs[0]), "init") ||
	    !cci_table(hw->cci, cam_hires_regs, sizeof(cam_hires_regs) / sizeof(cam_hires_regs[0]), "mode 0")) {
		return false;
	}
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
	cam_log("cam_probe: hires: tables written in %llu ms\n", ns / NSEC_PER_MSEC);

	static const unsigned settles[] = { CAM_HIRES_SETTLE, 36, 44, 52 };
	uint64_t intervals[2] = { 0, 0 };
	for (unsigned i = 0; i < sizeof(settles) / sizeof(settles[0]) && !r->captured; i++) {
		r->settle = settles[i];
		r->captured = sensor_capture_geom(hw, (uint32_t)IFE_IOVA, HIRES_STRIDE, &cphy_rate7, 2, HIRES_FRAME_BYTES, settles[i],
		    intervals, r->count, &r->done, streaming);
		if (*streaming || (!r->captured && r->done)) {
			break; /* stream off failed, or frames came with errors: a different settle is not the fix */
		}
	}
	r->first_ns = intervals[0];
	r->interval_ns = intervals[1];
	r->counted = r->count[1] != r->count[0];
	if (!r->captured) {
		return false;
	}
	flush_dcache(kva, (unsigned)HIRES_BYTES, FALSE);
	const uint8_t *a = (const uint8_t *)kva, *b = a + HIRES_FRAME_BYTES;
	for (uint32_t i = 0; i < HIRES_FRAME_BYTES; i += 97) {
		r->frames_differ += a[i] != b[i];
	}
	raw_stats_geom(a, CAM_HIRES_WIDTH, CAM_HIRES_HEIGHT, HIRES_STRIDE, &r->stats[0]);
	hires_phases(a, r->phases[0]);

	/* 16x analog gain (Sony's code 16384 - 16384 / gain) at the table's coarse time. */
	uint32_t code = 16384u - 16384u / HIRES_GAIN;
	if (!cci_table(hw->cci, cam_holdon_regs, sizeof(cam_holdon_regs) / sizeof(cam_holdon_regs[0]), "group hold") ||
	    !cci_write8(hw->cci, 0x0204, (uint8_t)(code >> 8)) || !cci_write8(hw->cci, 0x0205, (uint8_t)code) ||
	    !cci_table(hw->cci, cam_holdoff_regs, sizeof(cam_holdoff_regs) / sizeof(cam_holdoff_regs[0]), "group release")) {
		return false;
	}
	uint64_t ignore[2];
	r->gain_captured = sensor_capture_geom(hw, (uint32_t)IFE_IOVA + HIRES_FRAME_BYTES, HIRES_STRIDE, &cphy_rate7, 2, 0,
	    r->settle, ignore, r->gain_count, &r->gain_done, streaming);
	flush_dcache(kva + HIRES_FRAME_BYTES, (unsigned)HIRES_FRAME_BYTES, FALSE);
	raw_stats_geom(b, CAM_HIRES_WIDTH, CAM_HIRES_HEIGHT, HIRES_STRIDE, &r->stats[1]);
	hires_phases(b, r->phases[1]);
	for (unsigned s = 0; s < HIRES_SLOTS; s++) {
		const uint32_t *m = r->phases[s];
		r->order[s] = hires_order(m, &r->bayer_var[s], &r->quad_var[s]);
		cam_log("cam_probe: hires: frame %u%s: %u/1000 written, 10-bit range %u-%u, 2x2 means %u %u / %u %u\n", s,
		    s ? " (16x gain)" : "", r->stats[s].written_permille, r->stats[s].lo, r->stats[s].hi, r->stats[s].mean[0],
		    r->stats[s].mean[1], r->stats[s].mean[2], r->stats[s].mean[3]);
		cam_log("cam_probe: hires: frame %u 4x4 phases %u %u %u %u / %u %u %u %u / %u %u %u %u / %u %u %u %u: "
		    "period-2 spread %u, in-block spread %u: %s\n", s, m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9],
		    m[10], m[11], m[12], m[13], m[14], m[15], r->bayer_var[s], r->quad_var[s], r->order[s]);
	}
	return r->gain_captured;
}

static bool
hires_run(vm_offset_t kva, struct hires_result *r, bool *stopped)
{
	struct sensor_path p = { 0 };
	bool streaming = false;
	bool ok = sensor_path_up(&p);
	if (ok) {
		p.step = "IMX789 mode 0 through CSIPHY1, CSID0 and IFE0 RDI0";
		ok = hires_frames(&p.hw, r, &streaming, kva);
	}
	if (!ok) {
		cam_log("cam_probe: hires: capture failed at %s\n", p.step);
	}
	bool restored = sensor_path_down(&p, &streaming);
	*stopped = !streaming;
	cam_log("cam_probe: hires: sensor stream stopped %u, clocks and power restore %s\n", (unsigned)*stopped,
	    restored ? "OK" : "FAILED");
	return ok && restored && *stopped;
}

static bool
cam_hires_capture(void)
{
	vm_size_t size = round_page(HIRES_BYTES);
	vm_offset_t kva;
	if (kmem_alloc(kernel_map, &kva, size, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		cam_log("cam_probe: level 7 FAIL (%lu MB frame buffer allocation)\n", (unsigned long)(size >> 20));
		return false;
	}
	struct hakios_smmu_context *ctx = hakios_smmu_claim(IFE_SID, IFE_SID_MASK, IFE_IOVA, size, "camera IFE");
	if (!ctx) {
		cam_log("cam_probe: level 7 FAIL (IFE stream claim)\n");
		kmem_free(kernel_map, kva, size);
		return false;
	}
	memset((void *)kva, TPG_FILL, size);
	flush_dcache(kva, (unsigned)size, FALSE);
	vm_size_t mapped = 0;
	uint64_t start = mach_absolute_time(), map_ns;
	int map_error = map_buffer(ctx, IFE_IOVA, kva, size, &mapped);
	absolutetime_to_nanoseconds(mach_absolute_time() - start, &map_ns);
	struct hires_result r = { 0 };
	bool stopped = true, captured = false;
	if (!map_error) {
		captured = hires_run(kva, &r, &stopped);
		captured = hakios_smmu_faults(ctx) && captured;
	}
	int unmap_error = mapped ? hakios_smmu_unmap(ctx, IFE_IOVA, mapped) : 0;
	int release_error = hakios_smmu_release(ctx);
	cam_log("cam_probe: hires: IFE SID %#x/%#x IOVA %#llx bytes %#lx (stride %u, %u per frame): map %d in %llu ms, unmap %d, "
	    "release %d\n", IFE_SID, IFE_SID_MASK, IFE_IOVA, (unsigned long)size, HIRES_STRIDE, HIRES_FRAME_BYTES, map_error,
	    map_ns / NSEC_PER_MSEC, unmap_error, release_error);
	uint32_t m0 = raw_mean(&r.stats[0]), m1 = raw_mean(&r.stats[1]);
	cam_log("cam_probe: hires: settle %u, frames %u+%u, frame counter %#x -> %#x (gain %#x -> %#x), first frame after %llu ms, "
	    "interval %llu us, %u/%u sampled bytes differ, mean %u -> %u at %ux gain\n", r.settle, r.done, r.gain_done, r.count[0],
	    r.count[1], r.gain_count[0], r.gain_count[1], r.first_ns / NSEC_PER_MSEC, r.interval_ns / 1000, r.frames_differ,
	    HIRES_FRAME_BYTES / 97 + 1, m0, m1, HIRES_GAIN);
	bool pass = captured && r.counted && r.interval_ns > 25000000 && r.interval_ns < 45000000 && r.frames_differ > 0 &&
	    (m1 > m0 || m0 >= 1000);
	for (unsigned s = 0; s < HIRES_SLOTS; s++) {
		pass = pass && r.stats[s].written_permille >= 990 && r.stats[s].hi > r.stats[s].lo;
	}
	if (!unmap_error && !release_error && stopped && r.captured) {
		static const char *const names[HIRES_SLOTS] = { "hakios-camera-hires0", "hakios-camera-hires1" };
		bool published = true;
		for (unsigned s = 0; s < HIRES_SLOTS; s++) {
			published = cam_frame_publish(10 + s, names[s], kva + s * HIRES_FRAME_BYTES, CAM_HIRES_WIDTH, CAM_HIRES_HEIGHT,
			    HIRES_STRIDE, 2) && published;
			cam_frames[10 + s].header.valid = pass;
		}
		if (published) {
			cam_log("cam_probe: level 7 %s (IMX789 mode 0 %u x %u RAW10, %s; dump /dev/hakios-camera-hires0-1)\n",
			    pass ? "PASS" : "FAIL", CAM_HIRES_WIDTH, CAM_HIRES_HEIGHT, r.order[1] ? r.order[1] : "order not judged");
			return pass;
		}
		cam_log("cam_probe: hires: could not publish the frames\n");
	}
	if (stopped && (!release_error || (!map_error && !unmap_error))) {
		kmem_free(kernel_map, kva, size);
	} else {
		cam_log("cam_probe: hires: retaining frame buffer after uncertain SMMU teardown\n");
	}
	cam_log("cam_probe: level 7 FAIL (mode 0 frames or teardown)\n");
	return false;
}
#else
static bool
cam_hires_capture(void)
{
	cam_log("cam_probe: level 7 FAIL (this kernel has no mode 0 table: tools/camsensor.py --c-header hakios.cam_imx789.h 1 0)\n");
	return false;
}
#endif

/* ---- level 5: IFE0's pixel path, CamX's init packet (CAMERA-PLAN section 4.2) ---- */

/*
 * Stock OxygenOS previews this mode through CSID IPP -> IFE CAMIF -> the CLC pipeline (demosaic,
 * colour correction, gamma, RGB to YUV) -> FD Y/C, 640 x 480 linear NV12 (section 4.1). Level 5 does
 * the same on CSID0/IFE0: the init packet's register writes and DMI tables (tools/camife.py ->
 * hakios.cam_ife_init.h, in bsd/dev/ like the sensor tables), the IFE top words CamX's update packets
 * leave live (IFE2 snapshot), CSID IPP as CSID2 ran it, and the FD write masters (bus clients 8 and
 * 9, completion group 2: CSID buffer-done bit 2). IFE0 runs at 594 MHz (pixel_plls).
 *
 * cam_ife_cdm=1  Run level 5's captured IFE register and DMI writes through RT CDM0.
 *
 * FD_SESSIONS full sessions (path up, init, FD_SLOTS frames, path down), as section 4.2 asks for
 * repeated capture and teardown. The last session's frames are /dev/hakios-camera-fd0-2.
 */
#if CAM_HAVE_TABLES && __has_include("hakios.cam_ife_init.h")
#include "hakios.cam_ife_init.h"
#define CAM_HAVE_IFE_INIT 1
#else
#define CAM_HAVE_IFE_INIT 0
#endif

#define FD_WIDTH         640u
#define FD_HEIGHT        480u
#define FD_Y_BYTES       (FD_WIDTH * FD_HEIGHT)
#define FD_C_BYTES       (FD_Y_BYTES / 2u)
#define FD_FRAME_BYTES   (FD_Y_BYTES + FD_C_BYTES)
#define FD_SLOT_BYTES    0x80000u /* Y at 0, C right after it: one NV12 image, 16 KiB aligned */
/* FULL (bus clients 0 and 1, completion group 0) as linear NV12 instead of CamX's UBWC, 1440 x 1080. */
#define FULL_WIDTH       1440u
#define FULL_HEIGHT      1080u
#define FULL_STRIDE      1536u
#define FULL_SLOT_BYTES  0x280000u
#define NATIVE_FULL_STRIDE 4096u
#define FULL_BASE        0x180000u /* after the FD slots */
#define SCRATCH_BYTES    0x200000u /* DS4, DS16 and the stats outputs, all on one buffer */
/* PIXEL_RAW (bus client 10): the Bayer data inside the IFE, 16 bits a pixel as CamX's packer 0x25, frame 0 only. */
#define PRAW_STRIDE      (RAW_WIDTH * 2u)
#define PRAW_BYTES       (PRAW_STRIDE * RAW_HEIGHT)
#define IFE_WM_PRAW      0x1800
#define IFE_WM_FULL_Y    0x0e00
#define IFE_WM_FULL_C    0x0f00
#define FULL_BUF_DONE    (1u << 0)
#define FD_SLOTS         3u
#define FD_SESSIONS      3u
#define FD_FILL          0xa5
#define IFE_WM_FD_Y      0x1600
#define IFE_WM_FD_C      0x1700
#define FD_BUF_DONE      (1u << 2)
#define CSID_IPP_STATUS  0x0ac
#define CSID_IPP_MASK    0x0b0
#define CSID_IPP_CLEAR   0x0b4
#define CSID_IPP_CFG0    0x300
#define CSID_IPP_CTRL    0x304
#define IPP_RUP_AUP      (1u | 1u << 16)
#define IPP_CFG0         0x802b2000u /* enable, DT RAW10, VC 0, decode format 2 (CSID2) */

struct ife_capture_layout {
	uint32_t width, height, stride, chroma_offset, slot_bytes, scratch_base, praw_base;
	bool native;
};

static struct ife_capture_layout
ife_full_layout(bool native)
{
	struct ife_capture_layout layout = {
		.native = native,
		.width = native ? RAW_WIDTH : FULL_WIDTH,
		.height = native ? RAW_HEIGHT : FULL_HEIGHT,
		.stride = native ? NATIVE_FULL_STRIDE : FULL_STRIDE,
	};
	layout.chroma_offset = layout.stride * layout.height;
	layout.slot_bytes = native ? (uint32_t)round_page(layout.chroma_offset * 3 / 2) : FULL_SLOT_BYTES;
	layout.scratch_base = FULL_BASE + FD_SLOTS * layout.slot_bytes;
	layout.praw_base = layout.scratch_base + SCRATCH_BYTES;
	return layout;
}

/*
 * Words CamX leaves set during preview that neither the init nor the update packet writes (IFE2
 * snapshot): core_cfg_1 and 3-6, stats throttle, epoch, top debug, and CLC 0x6500's enable.
 */
static const struct {
	uint16_t reg;
	uint32_t value;
} ife_top_words[] = {
	{ 0x028, 0x00000010 }, { 0x068, 0x000aaaa0 }, { 0x070, 0x0f150f0f }, { 0x074, 0x0f0f1503 },
	{ 0x078, 0x00000f07 }, { 0x080, 0x000aaaa0 }, { 0x084, 0x00012491 }, { 0x088, 0x00040001 },
	{ 0x094, 0x00000001 }, { 0x098, 0x00000001 }, { 0x09c, 0x000005df }, { 0x0fc, 0x00000003 },
	{ 0x6560, 0x00000001 },
};

/* The stats CLCs' module_cfg words (0xb000-0xc060), switched off. */
static const uint16_t stats_clcs[] = { 0xb060, 0xb260, 0xb460, 0xb660, 0xb860, 0xba60, 0xbc60, 0xbe60, 0xc060 };

/* FULL's stock IPE input keeps 10-bit samples. Use FD's 8-bit rounding and clamps for linear NV12. */
static const uint16_t full_round_clamp[] = { 0x9c74, 0x9c88, 0x9e74, 0x9e7c, 0x9e88, 0x9e8c };

/* CSID2's IPP during preview (cfg1 = crop, pix store, early EOF, timestamps; full 4000 x 3008 crop). */
static const struct {
	uint16_t reg;
	uint32_t value;
} ipp_words[] = {
	{ 0x310, 0x00017242 }, { 0x318, 0x00000009 }, { 0x330, 0x02000000 }, { 0x334, 0x05df0000 },
	{ 0x338, 0x00000001 }, { 0x33c, 0x00000001 }, { 0x35c, 0x0f9f0000 }, { 0x360, 0x0bbf0000 },
	{ 0x364, 0x00000000 }, { 0x368, 0x00000001 }, { 0x36c, 0x00000000 }, { 0x370, 0x00000001 },
	{ 0x374, 0x00000000 }, { 0x378, 0x00000001 }, { 0x37c, 0x00000001 }, { 0x380, 0x00000000 },
	{ 0x384, 0x0000001f }, { 0x388, 0x0bc00fa0 },
};

/*
 * The other outputs CamX ran (IFE2 snapshot): DS4, DS16 and the stats write masters, with CamX's
 * cfg, frame increment, image cfg 0, stride (frame size for the frame-based ones) and packer. They
 * all write the scratch buffer; nothing reads it. Left off, the first boot's pixel path overflowed
 * (CSID IPP recovery overflow) after one frame.
 */
static const struct {
	uint8_t wm;
	uint32_t cfg, incr, image0, stride, packer;
} scratch_wms[] = {
	{ 2, 0x00000001, 0x0003c000, 0x008700b4, 0x00000600, 0xa }, { 3, 0x00000001, 0x00008000, 0x0022002d, 0x00000200, 0xa },
	{ 11, 0x00010001, 0x000a0000, 0, 0x000a0000, 0xa }, { 12, 0x00010001, 0x00008000, 0, 0x00008000, 0xa },
	{ 13, 0x00010001, 0x00048000, 0, 0x00048000, 0xa }, { 14, 0x00010001, 0x00151800, 0, 0x00151800, 0xa },
	{ 15, 0x00020001, 0x00000a00, 0x0005000a, 0x00000050, 0xa }, { 16, 0x00020001, 0x00002d00, 0x00090004, 0x00000020, 0xa },
	{ 17, 0x00010001, 0x00008000, 0, 0x00008000, 0xa }, { 18, 0x00010001, 0x00010000, 0, 0x00010000, 0x8 },
	{ 19, 0x00010001, 0x00001000, 0, 0x00001000, 0x8 },
};
#define IFE_WM(n) (0x0e00u + (n) * 0x100u)

#if CAM_HAVE_IFE_INIT

struct ife_cdm;
#ifdef CAM_IFE_CDM_BYTES
/* NE2213 rt-cdm0 and msm_cam_smmu_cpas_cdm, with CDM 2.1 FIFO 0 register offsets. */
#define CDM_PA          0x0ac25000UL
#define CDM_SIZE        0x400UL
#define CDM_IOVA        0x12000000u
#define CDM_SID_MASK    0x400u
#define CDM_VERSION     0x00u
#define CDM_RESET       0x10u
#define CDM_CORE_CFG    0x18u
#define CDM_CORE_EN     0x1cu
#define CDM_IRQ_MASK    0x30u
#define CDM_IRQ_CLEAR   0x34u
#define CDM_IRQ_CMD     0x38u
#define CDM_IRQ_STATUS  0x44u
#define CDM_FIFO_BASE   0x50u
#define CDM_FIFO_LEN    0x54u
#define CDM_FIFO_STORE  0x58u
#define CDM_FIFO_CFG    0x5cu
#define CDM_PENDING     0x6cu
#define CDM_USER_DATA   0x80u
#define CDM_AHB_ERROR   0xe0u
#define CDM_AHB_BASE    0xf0u
#define CDM_INVALID_BL  0x1d0u
#define CDM_RESET_DONE  0x1u
#define CDM_INLINE_IRQ  0x2u
#define CDM_ERRORS      0x70000u
#define CDM_TAG         0x35u /* also in the generated GEN_IRQ command */

/* Each masked match also covers its 0x24xx alias listed in the stock device tree. */
static const unsigned cdm_sids[] = { 0x20c0, 0x20a0 };

struct ife_cdm {
	struct hakios_smmu_context *ctx[sizeof(cdm_sids) / sizeof(cdm_sids[0])];
	vm_offset_t kva;
	vm_size_t size;
	bool idle;
};

static bool
ife_cdm_prepare(struct ife_cdm *cdm)
{
	cdm->size = round_page(sizeof(cam_ife_cdm));
	if (kmem_alloc(kernel_map, &cdm->kva, cdm->size, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		cdm->kva = 0;
		return false;
	}
	uint32_t *cmd = (uint32_t *)cdm->kva;
	memcpy(cmd, cam_ife_cdm, sizeof(cam_ife_cdm));
	for (unsigned i = 0; i < sizeof(cam_ife_cdm_fixups) / sizeof(cam_ife_cdm_fixups[0]); i++) {
		cmd[cam_ife_cdm_fixups[i]] += CDM_IOVA;
	}
	flush_dcache(cdm->kva, (unsigned)cdm->size, FALSE);
	for (unsigned i = 0; i < sizeof(cdm_sids) / sizeof(cdm_sids[0]); i++) {
		cdm->ctx[i] = hakios_smmu_claim(cdm_sids[i], CDM_SID_MASK, CDM_IOVA, cdm->size, "camera RT CDM");
		if (!cdm->ctx[i] || hakios_smmu_map_kva(cdm->ctx[i], CDM_IOVA, cdm->kva, cdm->size, HAKIOS_SMMU_READ)) {
			return false;
		}
	}
	return true;
}

static bool
ife_cdm_release(struct ife_cdm *cdm)
{
	bool released = cdm->idle;
	if (cdm->idle) {
		for (unsigned i = 0; i < sizeof(cdm_sids) / sizeof(cdm_sids[0]); i++) {
			if (cdm->ctx[i]) {
				released = hakios_smmu_release(cdm->ctx[i]) == 0 && released;
			}
		}
	}
	if (released && cdm->kva) {
		kmem_free(kernel_map, cdm->kva, cdm->size);
	}
	if (!released) {
		cam_log("cam_probe: ife: retaining CDM buffer after uncertain teardown\n");
	}
	return released;
}

static void
ife_cdm_clear(vm_offset_t cdm)
{
	write_reg(cdm + CDM_IRQ_CLEAR, CDM_RESET_DONE | CDM_INLINE_IRQ | 4u | CDM_ERRORS);
	write_reg(cdm + CDM_IRQ_CMD, 1);
}

/* Pause and flush FIFO 0 before the DMA mapping can be released, even after a replay timeout. */
static bool
ife_cdm_reset(vm_offset_t cdm)
{
	write_reg(cdm + CDM_CORE_EN, 3);
	spin_us(1000);
	ife_cdm_clear(cdm);
	write_reg(cdm + CDM_IRQ_MASK, CDM_RESET_DONE | CDM_INLINE_IRQ | CDM_ERRORS);
	write_reg(cdm + CDM_RESET, 9);
	bool idle = poll_reg(cdm + CDM_IRQ_STATUS, CDM_RESET_DONE, CDM_RESET_DONE, 2000) &&
	    poll_reg(cdm + CDM_PENDING, 0x1ff, 0, 2000);
	write_reg(cdm + CDM_CORE_EN, 0);
	ife_cdm_clear(cdm);
	return idle;
}

static bool
ife_cdm_replay(struct ife_cdm *dma)
{
	vm_offset_t cdm = map_registers(CDM_PA, CDM_SIZE);
	if (!cdm) {
		return false;
	}
	uint32_t version = read_reg(cdm + CDM_VERSION);
	uint32_t pending = read_reg(cdm + CDM_PENDING);
	if (version != 0x20010000 || (pending & 0x01ff01ff) || (read_reg(cdm + CDM_PENDING + 4) & 0x01ff01ff)) {
		cam_log("cam_probe: ife: RT CDM0 unavailable (version %#x, pending %#x)\n", version, pending);
		unmap_registers(cdm, CDM_SIZE);
		return false;
	}
	uint32_t en = read_reg(cdm + CDM_CORE_EN);
	uint32_t cfg = read_reg(cdm + CDM_CORE_CFG);
	uint32_t fifo_cfg = read_reg(cdm + CDM_FIFO_CFG);
	uint32_t irq_mask = read_reg(cdm + CDM_IRQ_MASK);
	bool complete = false;
	if (ife_cdm_reset(cdm)) {
		write_reg(cdm + CDM_CORE_CFG, 0x11f); /* stop on AHB error, burst of 16 */
		write_reg(cdm + CDM_FIFO_CFG, 64u << 16);
		write_reg(cdm + CDM_CORE_EN, 1);
		write_reg(cdm + CDM_FIFO_BASE, CDM_IOVA);
		write_reg(cdm + CDM_FIFO_LEN, CDM_TAG << 24 | (CAM_IFE_CDM_BYTES - 1));
		dma->idle = false;
		write_reg(cdm + CDM_FIFO_STORE, 1);
		uint32_t status = 0;
		for (unsigned i = 0; i < 2000; i++) {
			status = read_reg(cdm + CDM_IRQ_STATUS);
			if (status & (CDM_INLINE_IRQ | CDM_ERRORS)) {
				break;
			}
			spin_us(10);
		}
		uint32_t tag = read_reg(cdm + CDM_USER_DATA) & 0xff;
		uint32_t base = read_reg(cdm + CDM_AHB_BASE);
		complete = (status & (CDM_INLINE_IRQ | CDM_ERRORS)) == CDM_INLINE_IRQ &&
		    tag == CDM_TAG && base == 0; /* IFE0 is at the RT wrapper base. */
		cam_log("cam_probe: ife: RT CDM0 %s (%u bytes, status %#x, tag %#x, AHB base %#x, AHB error %#x, invalid BL %#x)\n",
		    complete ? "complete" : "failed", CAM_IFE_CDM_BYTES, status, tag, base,
		    read_reg(cdm + CDM_AHB_ERROR), read_reg(cdm + CDM_INVALID_BL));
		dma->idle = ife_cdm_reset(cdm);
	} else {
		cam_log("cam_probe: ife: RT CDM0 initial reset timed out\n");
	}
	write_reg(cdm + CDM_IRQ_MASK, irq_mask);
	if (dma->idle) {
		write_reg(cdm + CDM_CORE_CFG, cfg);
		write_reg(cdm + CDM_FIFO_CFG, fifo_cfg);
		write_reg(cdm + CDM_CORE_EN, en);
	}
	unmap_registers(cdm, CDM_SIZE);
	for (unsigned i = 0; i < sizeof(cdm_sids) / sizeof(cdm_sids[0]); i++) {
		complete = hakios_smmu_faults(dma->ctx[i]) && complete;
	}
	return complete && dma->idle;
}
#endif
/*
 * Bit 0: data at +0xc (else +8); bit 1: address written per word; bit 2: config is DMISel alone; bit 3:
 * config 0x40 | DMISel, the TFE's (cam_tfe_core.c reads a LUT with 0x40 | n in its DMI config, the
 * address 0 once, then the data-low port again and again), else 0x100 | DMISel (vfe40's).
 */
#define DMI_WAYS 9u
static const char *const dmi_ways[DMI_WAYS] = { "data +8, auto", "data +0xc, auto", "data +8, addressed",
	                                        "data +0xc, addressed", "sel only, data +8, addressed",
	                                        "sel only, data +0xc, addressed", "0x40, data +0xc, auto",
	                                        "0x40, data +8, auto", "0x40, data +0xc, addressed" };
static const uint8_t dmi_way_bits[DMI_WAYS] = { 0, 1, 2, 3, 6, 7, 9, 8, 11 };
#define DMI_CFG(bits, sel) (((bits) & 4) ? (sel) : ((bits) & 8) ? (0x40u | (sel)) : (0x100u | (sel)))

static void
dmi_put(vm_offset_t cfg, uint32_t sel, const uint32_t *v, uint32_t n, unsigned way)
{
	way = dmi_way_bits[way];
	vm_offset_t data = cfg + ((way & 1) ? 0xc : 0x8);
	write_reg(cfg, DMI_CFG(way, sel));
	write_reg(cfg + 4, 0);
	for (uint32_t i = 0; i < n; i++) {
		if (way & 2) {
			write_reg(cfg + 4, i);
		}
		write_reg(data, v[i]);
	}
	write_reg(cfg, 0);
}

/* Words that read back differently; *at and *got describe the first. */
static uint32_t
dmi_check(vm_offset_t cfg, uint32_t sel, const uint32_t *v, uint32_t n, unsigned way, uint32_t *at, uint32_t *got)
{
	way = dmi_way_bits[way];
	vm_offset_t data = cfg + ((way & 1) ? 0xc : 0x8);
	uint32_t bad = 0;
	write_reg(cfg, DMI_CFG(way, sel));
	write_reg(cfg + 4, 0);
	for (uint32_t i = 0; i < n; i++) {
		if (way & 2) {
			write_reg(cfg + 4, i);
		}
		uint32_t r = read_reg(data);
		if (r != v[i] && bad++ == 0) {
			*at = i;
			*got = r;
		}
	}
	write_reg(cfg, 0);
	return bad;
}

struct ife_replay {
	unsigned way, tables_ok, tables_bad, tables_zero, writes;
	bool diagnosed;
};

/*
 * Boot 3: only an address written before each word changes the picture, and then every fourth word
 * reads back 0. Register-only experiments on GLUT's DMI port (config 0x5f08, DMISel 1) to learn the
 * convention: reads repeated and on both data offsets, and which config bit auto-increments.
 */
static void
dmi_lab(vm_offset_t ife)
{
	vm_offset_t cfg = ife + 0x5f08;
	uint32_t r1[8], r2[8], r3[8];
	/* 1: addressed writes of 0x100 + i; addressed reads, each read twice, then +0xc. */
	write_reg(cfg, 0x101);
	for (uint32_t i = 0; i < 16; i++) {
		write_reg(cfg + 4, i);
		write_reg(cfg + 8, 0x100 + i);
	}
	for (uint32_t i = 0; i < 8; i++) {
		write_reg(cfg + 4, i);
		r1[i] = read_reg(cfg + 8);
		r2[i] = read_reg(cfg + 8);
		r3[i] = read_reg(cfg + 0xc);
	}
	write_reg(cfg, 0);
	cam_log("cam_probe: ife: dmi lab 1 (addressed 0x100+i): read %#x %#x %#x %#x %#x %#x %#x %#x | again %#x %#x %#x %#x "
	    "%#x %#x %#x %#x | +0xc %#x %#x %#x %#x %#x %#x %#x %#x\n", r1[0], r1[1], r1[2], r1[3], r1[4], r1[5], r1[6], r1[7],
	    r2[0], r2[1], r2[2], r2[3], r2[4], r2[5], r2[6], r2[7], r3[0], r3[1], r3[2], r3[3], r3[4], r3[5], r3[6], r3[7]);
	/* 2: address 0 once, then 8 data writes under each config; read back addressed (second read). */
	static const uint32_t cfgs[] = { 0x101, 0x201, 0x401, 0x801, 0x1001, 0x10001, 0x80000001, 0x1 };
	for (unsigned k = 0; k < sizeof(cfgs) / sizeof(cfgs[0]); k++) {
		for (unsigned port = 8; port <= 0xc; port += 4) {
			uint32_t base = 0x1000 * (k + 1) + (port << 8);
			write_reg(cfg, 0x101); /* clear rows 0-7 */
			for (uint32_t i = 0; i < 8; i++) {
				write_reg(cfg + 4, i);
				write_reg(cfg + 8, 0);
			}
			write_reg(cfg, cfgs[k]);
			write_reg(cfg + 4, 0);
			for (uint32_t i = 0; i < 8; i++) {
				write_reg(cfg + port, base + i);
			}
			uint32_t addr_after = read_reg(cfg + 4);
			write_reg(cfg, 0x101);
			for (uint32_t i = 0; i < 8; i++) {
				write_reg(cfg + 4, i);
				(void)read_reg(cfg + 8);
				r1[i] = read_reg(cfg + 8);
			}
			write_reg(cfg, 0);
			cam_log("cam_probe: ife: dmi lab 2 cfg %#x data +%#x (wrote %#x+i): addr after %#x, rows %#x %#x %#x %#x %#x %#x "
			    "%#x %#x\n", cfgs[k], port, base, addr_after, r1[0], r1[1], r1[2], r1[3], r1[4], r1[5], r1[6], r1[7]);
		}
	}
}

/* Write a table one way and read it back the same way: exact matches, matches one word late or early. */
static void
dmi_diag(vm_offset_t cfg, uint32_t reg, uint32_t sel, const uint32_t *v, uint32_t n, unsigned way)
{
	uint32_t got[256];
	n = MIN(n, 256u);
	dmi_put(cfg, sel, v, n, way);
	unsigned bits = dmi_way_bits[way];
	vm_offset_t data = cfg + ((bits & 1) ? 0xc : 0x8);
	write_reg(cfg, DMI_CFG(bits, sel));
	write_reg(cfg + 4, 0);
	for (uint32_t i = 0; i < n; i++) {
		if (bits & 2) {
			write_reg(cfg + 4, i);
		}
		got[i] = read_reg(data);
	}
	uint32_t cfg_now = read_reg(cfg), addr_now = read_reg(cfg + 4);
	write_reg(cfg, 0);
	unsigned exact = 0, late = 0, early = 0;
	for (uint32_t i = 0; i < n; i++) {
		exact += got[i] == v[i];
		late += i > 0 && got[i] == v[i - 1];
		early += i + 1 < n && got[i] == v[i + 1];
	}
	cam_log("cam_probe: ife: DMI %#x sel %u %s: %u exact, %u late, %u early of %u; cfg %#x addr %#x; "
	    "read %#x %#x %#x %#x %#x %#x, wrote %#x %#x %#x %#x %#x %#x\n", reg, sel, dmi_ways[way], exact, late, early, n,
	    cfg_now, addr_now, got[0], got[1], got[2], got[3], got[4], got[5], v[0], v[1], v[2], v[3], v[4], v[5]);
}

static bool
ife_replay(vm_offset_t ife, struct ife_replay *r, bool verbose)
{
	const uint32_t *w = cam_ife_init, *end = cam_ife_init + sizeof(cam_ife_init) / sizeof(cam_ife_init[0]);
	unsigned logged = 0;
	while (w + 2 <= end) {
		uint32_t kind = w[0] >> 24, n = w[0] & 0xffffffu, reg = w[1] & 0xffffffu, sel = w[1] >> 24;
		const uint32_t *v = w + 2;
		if (v + n > end || reg + 4u * n > IFE_SIZE || (kind == 2 && reg + 0x10u > IFE_SIZE)) {
			cam_log("cam_probe: ife: bad init op %#x at %#x\n", w[0], reg);
			return false;
		}
		if (kind == 1) {
			for (uint32_t i = 0; i < n; i++) {
				write_reg(ife + reg + 4 * i, v[i]);
			}
			r->writes += n;
		} else if (kind == 2) {
			vm_offset_t cfg = ife + reg;
			uint32_t at = 0, got = 0, bad;
			if (verbose && !r->diagnosed && n >= 16 && v[0] != 0) {
				r->diagnosed = true;
				for (unsigned way = 0; way < DMI_WAYS; way++) {
					dmi_diag(cfg, reg, sel, v, n, way);
				}
			}
			dmi_put(cfg, sel, v, n, r->way);
			bad = dmi_check(cfg, sel, v, n, r->way, &at, &got);
			if (bad) {
				r->tables_bad++;
				if (verbose && logged++ < 4) {
					cam_log("cam_probe: ife: DMI %#x sel %u: %u/%u words differ (first %u: %#x, wrote %#x)\n", reg, sel,
					    bad, n, at, got, v[at]);
				}
			} else {
				r->tables_ok++;
			}
		} else {
			cam_log("cam_probe: ife: unknown init op %#x\n", w[0]);
			return false;
		}
		w = v + n;
	}
	return w == end;
}

/* Init registers that read back differently now (double-buffered ones take their value at the reg update). */
static unsigned
ife_readback(vm_offset_t ife)
{
	const uint32_t *w = cam_ife_init, *end = cam_ife_init + sizeof(cam_ife_init) / sizeof(cam_ife_init[0]);
	unsigned bad = 0;
	while (w + 2 <= end) {
		uint32_t kind = w[0] >> 24, n = w[0] & 0xffffffu, reg = w[1] & 0xffffffu;
		for (uint32_t i = 0; kind == 1 && i < n; i++) {
			uint32_t now = read_reg(ife + reg + 4 * i);
			if (now != w[2 + i] && bad++ < 12) {
				cam_log("cam_probe: ife: init %#x = %#x, reads %#x\n", reg + 4 * i, w[2 + i], now);
			}
		}
		w += 2 + n;
	}
	return bad;
}

static void
wm_start(vm_offset_t wm, uint32_t iova, uint32_t image0, uint32_t stride, uint32_t packer, uint32_t incr, uint32_t cfg)
{
	write_reg(wm + WM_IMAGE_CFG0, image0);
	write_reg(wm + 0x10, 0); /* image_cfg_1: no offset */
	write_reg(wm + WM_IMAGE_CFG2, stride);
	write_reg(wm + WM_PACKER_CFG, packer);
	write_reg(wm + WM_FRAME_INCR, incr);
	write_reg(wm + WM_ADDRESS, iova);
	write_reg(wm + 0x30, 0);
	write_reg(wm + 0x34, 1);
	write_reg(wm + 0x38, 0);
	write_reg(wm + 0x3c, 1);
	write_reg(wm + 0x60, 0); /* no prefetch beyond the window */
	write_reg(wm + WM_CFG, cfg);
}

/* NV12 (packer 3, PLAIN_8_LSB_MSB_10 as CamX's FD), line based. */
static void
nv12_wm_start(vm_offset_t wm, uint32_t iova, uint32_t width, uint32_t height, uint32_t stride, uint32_t incr)
{
	wm_start(wm, iova, height << 16 | width, stride, 3, incr, WM_ENABLE);
}

/* Every output's buffer for frame n. */
static void
outputs_arm(const struct sensor_hw *hw, const struct ife_capture_layout *layout, uint32_t n)
{
	uint32_t fd = (uint32_t)IFE_IOVA + n * FD_SLOT_BYTES, full = (uint32_t)IFE_IOVA + FULL_BASE + n * layout->slot_bytes;
	write_reg(hw->ife + IFE_WM_FD_Y + WM_ADDRESS, fd);
	write_reg(hw->ife + IFE_WM_FD_C + WM_ADDRESS, fd + FD_Y_BYTES);
	write_reg(hw->ife + IFE_WM_FULL_Y + WM_ADDRESS, full);
	write_reg(hw->ife + IFE_WM_FULL_C + WM_ADDRESS, full + layout->chroma_offset);
	for (unsigned i = 0; i < sizeof(scratch_wms) / sizeof(scratch_wms[0]); i++) {
		write_reg(hw->ife + IFE_WM(scratch_wms[i].wm) + WM_ADDRESS, (uint32_t)IFE_IOVA + layout->scratch_base);
	}
}

/* Write masters off; true once all have stopped. */
static bool
outputs_stop(const struct sensor_hw *hw)
{
	vm_offset_t wms[5 + sizeof(scratch_wms) / sizeof(scratch_wms[0])];
	unsigned n = 0;
	wms[n++] = hw->ife + IFE_WM_FD_Y;
	wms[n++] = hw->ife + IFE_WM_FD_C;
	wms[n++] = hw->ife + IFE_WM_FULL_Y;
	wms[n++] = hw->ife + IFE_WM_FULL_C;
	wms[n++] = hw->ife + IFE_WM_PRAW;
	for (unsigned i = 0; i < sizeof(scratch_wms) / sizeof(scratch_wms[0]); i++) {
		wms[n++] = hw->ife + IFE_WM(scratch_wms[i].wm);
	}
	for (unsigned i = 0; i < n; i++) {
		write_reg(wms[i] + WM_CFG, 0);
	}
	bool stopped = true;
	for (unsigned i = 0; i < n; i++) {
		stopped = poll_reg(wms[i] + WM_CFG, WM_ENABLE, 0, 500) && stopped;
	}
	return stopped;
}

struct fd_stats {
	uint32_t written_permille, y_lo, y_hi, y_mean, u_mean, v_mean;
	uint8_t grid[3][4];
};

static void
fd_stats(const uint8_t *frame, uint32_t width, uint32_t height, uint32_t stride, uint32_t c_offset, struct fd_stats *s)
{
	uint32_t written = 0, lo = 255, hi = 0;
	uint64_t sum = 0, us = 0, vs = 0, cells[3][4] = { { 0 } };
	for (uint32_t row = 0; row < height; row++) {
		for (uint32_t col = 0; col < width; col++) {
			uint8_t y = frame[row * stride + col];
			written += y != FD_FILL;
			lo = MIN(lo, y);
			hi = MAX(hi, y);
			sum += y;
			cells[row * 3 / height][col * 4 / width] += y;
		}
	}
	for (uint32_t row = 0; row < height / 2; row++) {
		for (uint32_t col = 0; col < width; col += 2) {
			us += frame[c_offset + row * stride + col];
			vs += frame[c_offset + row * stride + col + 1];
		}
	}
	uint32_t pixels = width * height;
	s->written_permille = (uint32_t)((uint64_t)written * 1000 / pixels);
	s->y_lo = lo;
	s->y_hi = hi;
	s->y_mean = (uint32_t)(sum / pixels);
	s->u_mean = (uint32_t)(us / (pixels / 4));
	s->v_mean = (uint32_t)(vs / (pixels / 4));
	for (unsigned r = 0; r < 3; r++) {
		for (unsigned c = 0; c < 4; c++) {
			s->grid[r][c] = (uint8_t)(cells[r][c] / (pixels / 12));
		}
	}
}

struct fd_session {
	unsigned frames, readback_bad;
	struct ife_replay replay;
	uint64_t interval_ns;
	uint32_t ipp_status, ipp_first, buf_seen, ife_status0, ife_status1, ipp_violation, bus_violation, overflow, image_error,
	    camnoc_err;
	uint32_t measure[3];
	bool stopped, reset, stall_logged, tpg_stopped;
	uint32_t overflow_us; /* after frame 0's buffer done, +1; 0 = not seen */
};

/* While the pixel path waits: the IPP's state, the IFE's frame counters and its per-module busy nibbles. */
static void
ife_stall_log(const struct sensor_hw *hw, const char *when, uint32_t overflow_us, uint32_t ipp, uint32_t buf)
{
	cam_log("cam_probe: ife: %s: IPP status %#x (overflow %s%u us), buf %#x, CAMIF debug %#x %#x, IFE diag %#x %#x "
	    "frame counts %#x %#x, top status %#x/%#x\n", when, ipp, overflow_us ? "at " : "not seen ", overflow_us ? overflow_us - 1 : 0,
	    buf, read_reg(hw->csid + 0x340), read_reg(hw->csid + 0x344), read_reg(hw->ife + 0x54), read_reg(hw->ife + 0x58),
	    read_reg(hw->ife + 0x5c), read_reg(hw->ife + 0x60), read_reg(hw->ife + 0x44), read_reg(hw->ife + 0x48));
	cam_log("cam_probe: ife: %s: top debug a0-e0: %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x, "
	    "bus debug %#x, WM debug FD Y %#x %#x FULL Y %#x %#x\n", when, read_reg(hw->ife + 0xa0), read_reg(hw->ife + 0xa4),
	    read_reg(hw->ife + 0xa8), read_reg(hw->ife + 0xac), read_reg(hw->ife + 0xb0), read_reg(hw->ife + 0xb4),
	    read_reg(hw->ife + 0xb8), read_reg(hw->ife + 0xbc), read_reg(hw->ife + 0xc0), read_reg(hw->ife + 0xc4),
	    read_reg(hw->ife + 0xc8), read_reg(hw->ife + 0xcc), read_reg(hw->ife + 0xd0), read_reg(hw->ife + 0xd4),
	    read_reg(hw->ife + 0xd8), read_reg(hw->ife + 0xdc), read_reg(hw->ife + 0xe0), read_reg(hw->ife + 0xcd8),
	    read_reg(hw->ife + IFE_WM_FD_Y + 0x84), read_reg(hw->ife + IFE_WM_FD_Y + 0x88),
	    read_reg(hw->ife + IFE_WM_FULL_Y + 0x84), read_reg(hw->ife + IFE_WM_FULL_Y + 0x88));
}

/* After the write masters have stopped: IPP and RX off, CSIPHY1 in reset, CSID0 reset, IFE IRQ masks off. */
static bool
ipp_teardown(const struct sensor_hw *hw)
{
	write_reg(hw->csid + CSID_IPP_CFG0, 0);
	write_reg(hw->csid + CSID_IPP_MASK, 0);
	write_reg(hw->csid + CSID_RX_CFG0, 0);
	write_reg(hw->csid + CSID_RX_CFG1, 0);
	write_reg(hw->csid + CSID_RX_CAPTURE, 0);
	write_reg(hw->csid + CSID_BUF_MASK, 0);
	write_reg(hw->csid + CSID_RX_MASK, 0);
	write_reg(hw->csid + CSID_IPP_CLEAR, UINT32_MAX);
	write_reg(hw->ife + 0x34, 0);
	write_reg(hw->ife + 0xc18, 0);
	csiphy_stop(hw->phy);
	return csid_reset(hw->csid);
}

static void
ife_pixel_configure(const struct sensor_hw *hw)
{
	for (unsigned i = 0; i < sizeof(ife_top_words) / sizeof(ife_top_words[0]); i++) {
		write_reg(hw->ife + ife_top_words[i].reg, ife_top_words[i].value);
	}
	for (unsigned i = 0; i < sizeof(full_round_clamp) / sizeof(full_round_clamp[0]); i++) {
		uint16_t reg = full_round_clamp[i];
		write_reg(hw->ife + reg, read_reg(hw->ife + reg - 0x2000));
	}
	/*
	 * The stats CLCs off, and their write masters not started: with them, BAF and BFW stayed busy after
	 * frame 0 and the IFE back-pressured CSID IPP into overflow recovery (boot 4). AE, AWB and AF run on the CPU.
	 */
	{
		for (unsigned i = 0; i < sizeof(stats_clcs) / sizeof(stats_clcs[0]); i++) {
			write_reg(hw->ife + stats_clcs[i], read_reg(hw->ife + stats_clcs[i]) & ~1u);
		}
	}
	write_reg(hw->ife + 0x3c, UINT32_MAX); /* top IRQ clear 0 and 1, then the global clear */
	write_reg(hw->ife + 0x40, UINT32_MAX);
	write_reg(hw->ife + 0x30, 1);
	write_reg(hw->ife + 0x34, 0x7f051);      /* CamX's top mask: SOF, EOF, errors */
	write_reg(hw->ife + 0xc18, 0xd0000000u); /* CamX's bus mask: violations, overflow */
	write_reg(hw->ife + 0xc60, 1);           /* clear the bus overflow status */

}

static void
ife_native_size(vm_offset_t ife)
{
	/* FULL's Y/UV scalers use Q21 phase steps. Keep the high CamX control bits and UV subsampling. */
	for (unsigned c = 0; c < 2; c++) {
		vm_offset_t scale = ife + 0x9800 + 0x200 * c, crop = ife + 0x9c00 + 0x200 * c;
		vm_offset_t downscale = ife + 0xa000 + 0x200 * c;
		uint32_t width = RAW_WIDTH >> c, height = RAW_HEIGHT >> c, phase = (1u << 21) << c;

		write_reg(scale + 0x6c, (read_reg(scale + 0x6c) & 0xff000000u) | phase);
		write_reg(scale + 0x74, (read_reg(scale + 0x74) & 0xff000000u) | phase);
		write_reg(crop + 0x68, height - 1);
		write_reg(crop + 0x6c, width - 1);
		write_reg(downscale + 0x78, ((height / 4) << 16) | (width / 4));
		write_reg(downscale + 0x7c, (height << 16) | width);
	}
}

static void
ipp_configure(const struct sensor_hw *hw)
{
	write_reg(hw->csid + CSID_IPP_CFG0, IPP_CFG0 & ~RDI_ENABLE);
	for (unsigned i = 0; i < sizeof(ipp_words) / sizeof(ipp_words[0]); i++) {
		write_reg(hw->csid + ipp_words[i].reg, ipp_words[i].value);
	}
	write_reg(hw->csid + CSID_IPP_CTRL, 0);
	write_reg(hw->csid + CSID_IPP_CFG0, IPP_CFG0);
	write_reg(hw->csid + CSID_RUP_AUP, IPP_RUP_AUP);
}

static bool
fd_frames(const struct sensor_hw *hw, struct fd_session *r, bool *streaming, bool verbose, unsigned variant,
    struct ife_cdm *cdm, const struct ife_capture_layout *layout)
{
	bool from_tpg = false;
	vm_offset_t tpg = 0;
	if (from_tpg) {
		tpg = map_registers(TPG_PA, 0x1000);
		if (!tpg || (read_reg(tpg + TPG_CTRL) & TPG_ENABLE)) {
			unmap_registers(tpg, 0x1000);
			return false;
		}
	} else if (!cci_table(hw->cci, cam_init_regs, sizeof(cam_init_regs) / sizeof(cam_init_regs[0]), "init") ||
	    !cci_table(hw->cci, cam_mode_regs, sizeof(cam_mode_regs) / sizeof(cam_mode_regs[0]), "mode")) {
		return false;
	} else if (!cdm || variant != 1) {
		/* SMIA test pattern: 0x0600/0x0601 mode (2 colour bars, 1 solid), 0x0602-0x0609 R, Gr, B, Gb (10-bit). */
		bool ok = cci_write8(hw->cci, 0x0600, 0) && cci_write8(hw->cci, 0x0601, 2);
		for (uint16_t reg = 0x0602; ok && reg <= 0x0609; reg += 2) {
			ok = cci_write8(hw->cci, reg, 0x02) && cci_write8(hw->cci, reg + 1, 0x00);
		}
		if (!ok) {
			cam_log("cam_probe: ife: IMX789 test pattern write failed\n");
			return false;
		}
	}
	if (!csid_reset(hw->csid)) {
		unmap_registers(tpg, 0x1000);
		cam_log("cam_probe: CSID reset before the pixel path failed\n");
		return false;
	}
#ifdef CAM_IFE_CDM_BYTES
	if (cdm) {
		if (!ife_cdm_replay(cdm)) {
			unmap_registers(tpg, 0x1000);
			return false;
		}
	} else
#endif
	{
		uint64_t t0 = mach_absolute_time(), ns = 0;
		static const unsigned session_way[FD_SESSIONS] = { 6, 7, 2 };
		r->replay.way = session_way[variant];
		if (!ife_replay(hw->ife, &r->replay, verbose)) {
			unmap_registers(tpg, 0x1000);
			return false;
		}
		absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
		cam_log("cam_probe: ife: %s: %u writes, DMI %s, %u tables read back, %u differ, %u all zero, %llu us\n",
		    CAM_IFE_INIT_SOURCE, r->replay.writes, dmi_ways[r->replay.way], r->replay.tables_ok, r->replay.tables_bad,
		    r->replay.tables_zero, ns / 1000);
	}
	ife_pixel_configure(hw);
	if (layout->native) {
		ife_native_size(hw->ife);
	}

	vm_offset_t wy = hw->ife + IFE_WM_FD_Y, wc = hw->ife + IFE_WM_FD_C;
	uint32_t iova = (uint32_t)IFE_IOVA;
	nv12_wm_start(wy, iova, FD_WIDTH, FD_HEIGHT, FD_WIDTH, FD_SLOT_BYTES);
	nv12_wm_start(wc, iova + FD_Y_BYTES, FD_WIDTH, FD_HEIGHT / 2, FD_WIDTH, FD_SLOT_BYTES);
	write_reg(hw->ife + IFE_WM_FULL_Y + 0x48, 0); /* UBWC mode_cfg off: linear */
	write_reg(hw->ife + IFE_WM_FULL_C + 0x48, 0);
	nv12_wm_start(hw->ife + IFE_WM_FULL_Y, iova + FULL_BASE, layout->width, layout->height, layout->stride, layout->slot_bytes);
	nv12_wm_start(hw->ife + IFE_WM_FULL_C, iova + FULL_BASE + layout->chroma_offset, layout->width, layout->height / 2,
	    layout->stride, layout->slot_bytes);
	wm_start(hw->ife + IFE_WM_PRAW, iova + layout->praw_base, RAW_HEIGHT << 16 | RAW_WIDTH, PRAW_STRIDE, 0x25, 0, WM_ENABLE);
	for (unsigned i = 0; false && i < sizeof(scratch_wms) / sizeof(scratch_wms[0]); i++) {
		wm_start(hw->ife + IFE_WM(scratch_wms[i].wm), iova + layout->scratch_base, scratch_wms[i].image0, scratch_wms[i].stride,
		    scratch_wms[i].packer, 0, scratch_wms[i].cfg);
	}

	write_reg(hw->csid + CSID_BUF_MASK, UINT32_MAX); /* FD's group 2, and the others to see which complete */
	write_reg(hw->csid + CSID_IPP_MASK, 0x10b8601cu); /* CSID2's IPP mask */
	write_reg(hw->csid + CSID_RX_MASK, 0x0fffffff);
	write_reg(hw->csid + CSID_TOP_MASK, UINT32_MAX);
	ipp_configure(hw);
	if (from_tpg) {
		write_reg(hw->csid + CSID_RX_CFG0, tpg_rx_cfg0(&tpg_link));
		write_reg(hw->csid + CSID_RX_CFG1, 1u | 1u << 6);
		write_reg(hw->csid + CSID_RX_CAPTURE, rx_capture(tpg_link.cphy));
	} else {
		csiphy_start(hw->phy, CAM_TABLES_SETTLE);
		write_reg(hw->csid + CSID_RX_CFG0, SENSOR_RX_CFG0);
		write_reg(hw->csid + CSID_RX_CFG1, 1u | 1u << 6);
		write_reg(hw->csid + CSID_RX_CAPTURE, rx_capture(1));
	}
	write_reg(hw->csid + CSID_IPP_CTRL, 1); /* resume at frame boundary, internal start */
	cam_log("cam_probe: ife: IPP cfg0 %#x cfg1 %#x, FD WM Y %#x C %#x, IFE core_cfg_1 %#x\n",
	    read_reg(hw->csid + CSID_IPP_CFG0), read_reg(hw->csid + 0x310), read_reg(wy + WM_CFG), read_reg(wc + WM_CFG),
	    read_reg(hw->ife + 0x28));

	bool on;
	if (from_tpg) {
		struct tpg_link slow = tpg_link;
		slow.throttle = 0x11111111u; /* one cycle in four: full rate overflowed IPP in frame 0 (boot 5) */
		cam_tpg_start(tpg, &slow, RAW_WIDTH, RAW_HEIGHT);
		on = (read_reg(tpg + TPG_CTRL) & TPG_ENABLE) != 0;
	} else {
		*streaming = true;
		on = cci_write8(hw->cci, IMX789_MODE_SELECT, 1);
		if (!on) {
			cam_log("cam_probe: IMX789 stream on failed\n");
		}
	}
	uint64_t last = mach_absolute_time(), deadline;
	uint32_t done = layout->native ? FD_BUF_DONE | FULL_BUF_DONE : FD_BUF_DONE;
	clock_interval_to_deadline(1500, NSEC_PER_MSEC, &deadline);
	while (on && r->frames < FD_SLOTS && mach_absolute_time() < deadline) {
		uint32_t status = read_reg(hw->csid + CSID_BUF_STATUS);
		if (status == UINT32_MAX) {
			break;
		}
		if ((status & done) != done) {
			if (r->frames == 1 && !r->stall_logged) {
				uint64_t since = 0;
				absolutetime_to_nanoseconds(mach_absolute_time() - last, &since);
				uint32_t ipp = read_reg(hw->csid + CSID_IPP_STATUS);
				if (!r->overflow_us && (ipp & (1u << 19))) {
					r->overflow_us = (uint32_t)(since / 1000) + 1;
				}
				if (since > 50 * NSEC_PER_MSEC) {
					r->stall_logged = true;
					ife_stall_log(hw, "50 ms after frame 0", r->overflow_us, ipp, status);
				}
			}
			spin_us(10);
			continue;
		}
		uint64_t now = mach_absolute_time();
		if (r->frames == 0) {
			r->ipp_first = read_reg(hw->csid + CSID_IPP_STATUS);
		}
		if (r->frames + 1 == FD_SLOTS) {
			write_reg(hw->csid + CSID_IPP_CTRL, 0); /* halt at the next frame boundary */
		} else {
			outputs_arm(hw, layout, r->frames + 1);
			write_reg(hw->csid + CSID_RUP_AUP, IPP_RUP_AUP);
		}
		r->buf_seen |= status;
		write_reg(hw->csid + CSID_BUF_CLEAR, status);
		write_reg(hw->csid + CSID_IRQ_CMD, 1);
		if (r->frames == 1) {
			absolutetime_to_nanoseconds(now - last, &r->interval_ns);
		}
		last = now;
		r->frames++;
	}
	write_reg(hw->csid + CSID_IPP_CTRL, 0);
	r->ipp_status = read_reg(hw->csid + CSID_IPP_STATUS);
	r->ife_status0 = read_reg(hw->ife + 0x44);
	r->ife_status1 = read_reg(hw->ife + 0x48);
	r->ipp_violation = read_reg(hw->ife + 0x64);
	/* vfe680_dbg_reg_info: 4 bits of busy state per CLC and bus client, top_debug_cfg 3. */
	cam_log("cam_probe: ife: top debug a0-e0: %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x, "
	    "bus debug %#x, WM FD Y debug %#x %#x, FULL Y %#x %#x\n", read_reg(hw->ife + 0xa0), read_reg(hw->ife + 0xa4),
	    read_reg(hw->ife + 0xa8), read_reg(hw->ife + 0xac), read_reg(hw->ife + 0xb0), read_reg(hw->ife + 0xb4),
	    read_reg(hw->ife + 0xb8), read_reg(hw->ife + 0xbc), read_reg(hw->ife + 0xc0), read_reg(hw->ife + 0xc4),
	    read_reg(hw->ife + 0xc8), read_reg(hw->ife + 0xcc), read_reg(hw->ife + 0xd0), read_reg(hw->ife + 0xd4),
	    read_reg(hw->ife + 0xd8), read_reg(hw->ife + 0xdc), read_reg(hw->ife + 0xe0), read_reg(hw->ife + 0xcd8),
	    read_reg(hw->ife + IFE_WM_FD_Y + 0x84), read_reg(hw->ife + IFE_WM_FD_Y + 0x88),
	    read_reg(hw->ife + IFE_WM_FULL_Y + 0x84), read_reg(hw->ife + IFE_WM_FULL_Y + 0x88));
	for (unsigned i = 0; i < 3; i++) {
		r->measure[i] = read_reg(hw->csid + 0x38c + 4 * i);
	}
	if (verbose) {
		r->readback_bad = ife_readback(hw->ife);
	}
	uint32_t packets = read_reg(hw->csid + 0x240), rx = read_reg(hw->csid + CSID_RX_STATUS);
	if (from_tpg) {
		write_reg(tpg + TPG_CTRL, 0);
		r->tpg_stopped = poll_reg(tpg + TPG_CTRL, TPG_ENABLE, 0, 500);
		write_reg(tpg + TPG_IRQ_MASK, 0);
		write_reg(tpg + TPG_IRQ_CLEAR, 1);
		write_reg(tpg + TPG_IRQ_CMD, 1);
		write_reg(tpg + TPG_RESET, 1);
		unmap_registers(tpg, 0x1000);
	} else if (cci_write8(hw->cci, IMX789_MODE_SELECT, 0)) {
		*streaming = false;
	}
	delay(40000);
	r->stopped = outputs_stop(hw);
	r->bus_violation = read_reg(hw->ife + 0xc64);
	r->overflow = read_reg(hw->ife + 0xc68);
	r->image_error = read_reg(hw->ife + 0xc70);
	r->camnoc_err = read_reg(hw->camnoc + 0x2010);
	cam_log("cam_probe: ife: %u/%u %s frames, interval %llu us, buffer-done groups seen %#x, CSID buf %#x IPP %#x "
	    "(at frame 0 %#x) RX %#x top %#x, packets %u CRC %u\n",
	    r->frames, FD_SLOTS, layout->native ? "FD + FULL" : "FD", r->interval_ns / 1000, r->buf_seen,
	    read_reg(hw->csid + CSID_BUF_STATUS), r->ipp_status,
	    r->ipp_first, rx,
	    read_reg(hw->csid + CSID_TOP_STATUS), packets, read_reg(hw->csid + 0x248));
	cam_log("cam_probe: ife: IFE top %#x/%#x PP violation %#x, bus IRQ %#x/%#x violation %#x overflow %#x image %#x, "
	    "WM Y %#x C %#x, CAMNOC errvld %#x, IPP measure %#x %#x %#x\n",
	    r->ife_status0, r->ife_status1, r->ipp_violation, read_reg(hw->ife + 0xc28), read_reg(hw->ife + 0xc2c),
	    r->bus_violation, r->overflow, r->image_error, read_reg(wy + 0x70), read_reg(wc + 0x70), r->camnoc_err,
	    r->measure[0], r->measure[1], r->measure[2]);
	if (verbose) {
		cam_log("cam_probe: ife: %u of %u init registers read back differently after the frames\n", r->readback_bad,
		    CAM_IFE_INIT_WRITES);
	}
	r->reset = ipp_teardown(hw);
	return on && r->stopped && r->reset && !*streaming && (!from_tpg || r->tpg_stopped) && r->frames == FD_SLOTS &&
	       !r->bus_violation &&
	       !r->overflow && !r->image_error && !r->ipp_violation && !(r->camnoc_err & 1u);
}
#endif

/* Pass, and *safe when nothing could still be writing (the caller may go on to level 4's streaming). */
static bool
cam_ife_capture(unsigned level, bool *safe)
{
	*safe = false;
#if CAM_HAVE_IFE_INIT
	struct ife_capture_layout layout = ife_full_layout(level == 6);
	uint32_t full_bytes = layout.chroma_offset * 3 / 2;
	vm_size_t size = round_page(layout.praw_base + PRAW_BYTES);
	vm_offset_t kva;
	if (kmem_alloc(kernel_map, &kva, size, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		cam_log("cam_probe: level %u FAIL (frame buffer allocation)\n", level);
		*safe = true;
		return false;
	}
	struct hakios_smmu_context *ctx = hakios_smmu_claim(IFE_SID, IFE_SID_MASK, IFE_IOVA, size, "camera IFE");
	if (!ctx) {
		cam_log("cam_probe: level %u FAIL (IFE stream claim)\n", level);
		kmem_free(kernel_map, kva, size);
		*safe = true;
		return false;
	}
	vm_size_t mapped = 0;
	int map_error = map_buffer(ctx, IFE_IOVA, kva, size, &mapped);
	bool pass = !map_error, stopped = true, restored_all = true;
	unsigned use_cdm = 0;
	PE_parse_boot_argn("cam_ife_cdm", &use_cdm, sizeof(use_cdm));
	use_cdm = use_cdm || layout.native;
	struct ife_cdm *cdm = NULL;
	bool cdm_ready = !use_cdm;
#ifdef CAM_IFE_CDM_BYTES
	struct ife_cdm cdm_dma = { .idle = true };
	if (use_cdm && !map_error) {
		cdm = &cdm_dma;
		cdm_ready = ife_cdm_prepare(cdm);
	}
#endif
	if (use_cdm && !cdm_ready) {
		cam_log("cam_probe: ife: RT CDM0 replay buffer unavailable\n");
	}
	struct fd_stats st[FD_SLOTS], fs;
	unsigned sessions = 0;
	for (unsigned s = 0; !map_error && cdm_ready && s < FD_SESSIONS; s++) {
		memset((void *)kva, FD_FILL, size);
		flush_dcache(kva, (unsigned)size, FALSE);
		struct sensor_path p = { .plls = pixel_plls };
		struct fd_session r = { 0 };
		bool streaming = false;
		bool ok = sensor_path_up(&p);
		if (ok) {
			p.step = "IFE0 pixel path";
			/*
			 * Boot 6: clearing a CLC's module_cfg bit 0 does not bypass it (no frame at all), and the TPG
			 * overflowed IPP even at a quarter of its rate, so the sessions are the sensor again.
			 */
			/*
			 * Boot 9: PIXEL_RAW is the room at black (64, noise to 81), so every table's first entries
			 * decide the picture. The IMX789's own test pattern (SMIA 0x0601) gives known input over the
			 * real link: colour bars, then a solid mid grey, then the room.
			 */
			static const char *const variants[FD_SESSIONS] = { "colour bars, DMI 0x40 +0xc auto",
				                                           "colour bars, DMI 0x40 +8 auto",
				                                           "colour bars, DMI 0x100 +8 addressed" };
			const char *mode = variants[s];
			if (cdm) {
				mode = s == 1 ? "RT CDM0, live scene" : "RT CDM0, colour bars";
			}
			cam_log("cam_probe: ife: session %u: %s\n", s + 1, mode);
			ok = fd_frames(&p.hw, &r, &streaming, s == 0, s, cdm, &layout);
		}
		if (!ok) {
			cam_log("cam_probe: ife: session %u failed at %s\n", s + 1, p.step);
		}
		bool restored = sensor_path_down(&p, &streaming);
		bool clean = hakios_smmu_faults(ctx);
		flush_dcache(kva, (unsigned)size, FALSE);
		bool frames_ok = r.frames == FD_SLOTS;
		{
			const uint16_t *pr = (const uint16_t *)(kva + layout.praw_base);
			uint32_t lo = 0xffff, hi = 0, written = 0;
			uint64_t sum[4] = { 0 };
			for (uint32_t y = 0; y < RAW_HEIGHT; y += 2) {
				for (uint32_t x = 0; x < RAW_WIDTH; x += 2) {
					for (unsigned c = 0; c < 4; c++) {
						uint16_t v = pr[(y + c / 2) * RAW_WIDTH + x + c % 2];
						sum[c] += v;
						lo = MIN(lo, v);
						hi = MAX(hi, v);
						written += v != 0xa5a5;
					}
				}
			}
			uint32_t quads = RAW_WIDTH * RAW_HEIGHT / 4;
			const uint16_t *mid = pr + (RAW_HEIGHT / 2) * RAW_WIDTH;
			cam_log("cam_probe: ife: session %u PIXEL_RAW: %u/1000 written, %u-%u, 2x2 means %llu %llu / %llu %llu, mid row "
			    "%u %u %u %u %u %u %u %u\n", s + 1, (uint32_t)((uint64_t)written * 1000 / (quads * 4)), lo, hi,
			    sum[0] / quads, sum[1] / quads, sum[2] / quads, sum[3] / quads, mid[100], mid[101], mid[600], mid[601],
			    mid[2000], mid[2001], mid[3500], mid[3501]);
		}
		for (unsigned f = 0; f < FD_SLOTS; f++) {
			fd_stats((const uint8_t *)kva + f * FD_SLOT_BYTES, FD_WIDTH, FD_HEIGHT, FD_WIDTH, FD_Y_BYTES, &st[f]);
			cam_log("cam_probe: ife: session %u frame %u: Y %u/1000 written, %u-%u mean %u, U %u V %u, "
			    "Y grid %u %u %u %u / %u %u %u %u / %u %u %u %u\n", s + 1, f, st[f].written_permille, st[f].y_lo,
			    st[f].y_hi, st[f].y_mean, st[f].u_mean, st[f].v_mean, st[f].grid[0][0], st[f].grid[0][1],
			    st[f].grid[0][2], st[f].grid[0][3], st[f].grid[1][0], st[f].grid[1][1], st[f].grid[1][2],
			    st[f].grid[1][3], st[f].grid[2][0], st[f].grid[2][1], st[f].grid[2][2], st[f].grid[2][3]);
			frames_ok = frames_ok && st[f].written_permille >= 950 && st[f].y_hi > st[f].y_lo;
			if (f == 0) {
				const uint8_t *row = (const uint8_t *)kva + (FD_HEIGHT / 2) * FD_WIDTH;
				const uint8_t *uv = (const uint8_t *)kva + FD_Y_BYTES + (FD_HEIGHT / 4) * FD_WIDTH;
				cam_log("cam_probe: ife: session %u frame 0 mid row Y: %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u, "
				    "UV: %u,%u %u,%u %u,%u %u,%u %u,%u %u,%u %u,%u %u,%u\n", s + 1, row[20], row[60], row[100], row[140],
				    row[180], row[220], row[260], row[300], row[340], row[380], row[420], row[460], row[500], row[540],
				    row[580], row[620], uv[40], uv[41], uv[120], uv[121], uv[200], uv[201], uv[280], uv[281], uv[360],
				    uv[361], uv[440], uv[441], uv[520], uv[521], uv[600], uv[601]);
			}
			fd_stats((const uint8_t *)kva + FULL_BASE + f * layout.slot_bytes, layout.width, layout.height, layout.stride,
			    layout.chroma_offset, &fs);
			if (cdm) {
				frames_ok = frames_ok && fs.written_permille >= 950 && fs.y_hi > fs.y_lo;
			}
			cam_log("cam_probe: ife: session %u FULL %u: Y %u/1000 written, %u-%u mean %u, U %u V %u, "
			    "Y grid %u %u %u %u / %u %u %u %u / %u %u %u %u\n", s + 1, f, fs.written_permille, fs.y_lo, fs.y_hi,
			    fs.y_mean, fs.u_mean, fs.v_mean, fs.grid[0][0], fs.grid[0][1], fs.grid[0][2], fs.grid[0][3],
			    fs.grid[1][0], fs.grid[1][1], fs.grid[1][2], fs.grid[1][3], fs.grid[2][0], fs.grid[2][1],
			    fs.grid[2][2], fs.grid[2][3]);
		}
		cam_log("cam_probe: ife: session %u %s (frames %s, SMMU %s, stream stopped %u, restore %s)\n", s + 1,
		    ok && frames_ok && clean && restored ? "OK" : "FAILED", frames_ok ? "OK" : "bad", clean ? "clean" : "FAULTED",
		    (unsigned)!streaming, restored ? "OK" : "FAILED");
		pass = pass && ok && frames_ok && clean && restored;
		restored_all = restored_all && restored;
		sessions++;
		if (streaming) {
			stopped = false;
			break; /* the sensor would not stop: no more sessions */
		}
#ifdef CAM_IFE_CDM_BYTES
		if (cdm && !cdm->idle) {
			break;
		}
#endif
	}
	bool cdm_released = true;
#ifdef CAM_IFE_CDM_BYTES
	if (cdm) {
		cdm_released = ife_cdm_release(cdm);
	}
#endif
	int unmap_error = mapped ? hakios_smmu_unmap(ctx, IFE_IOVA, mapped) : 0;
	int release_error = hakios_smmu_release(ctx);
	cam_log("cam_probe: ife: %u sessions, IFE SID %#x IOVA %#llx bytes %#lx: map %d, unmap %d, release %d\n", sessions,
	    IFE_SID, IFE_IOVA, (unsigned long)size, map_error, unmap_error, release_error);
	pass = pass && cdm_released && !unmap_error && !release_error && sessions == FD_SESSIONS;
	*safe = stopped && cdm_released && !unmap_error && !release_error;
	if (*safe && sessions && !map_error) {
		static const char *const names[2][FD_SLOTS] = {
			{ "hakios-camera-fd0", "hakios-camera-fd1", "hakios-camera-fd2" },
			{ "hakios-camera-full0", "hakios-camera-full1", "hakios-camera-full2" },
		};
		bool published = true;
		for (unsigned f = 0; f < FD_SLOTS; f++) {
			for (unsigned k = 0; k < 2; k++) {
				unsigned n = (k ? 7 : 4) + f;
				published = (k ? cam_frame_publish(n, names[k][f], kva + FULL_BASE + f * layout.slot_bytes, layout.width,
				                     layout.height, layout.stride, 3)
				               : cam_frame_publish(n, names[k][f], kva + f * FD_SLOT_BYTES, FD_WIDTH, FD_HEIGHT,
				                     FD_WIDTH, 3)) &&
				    published;
				cam_frames[n].header.bytes = k ? full_bytes : FD_FRAME_BYTES;
				cam_frames[n].header.data_type = 0x3231564e; /* 'NV12' */
				cam_frames[n].header.bits = 8;
				cam_frames[n].header.packing = 0;
				cam_frames[n].header.valid = pass;
			}
		}
		cam_log("cam_probe: level %u %s (IFE0 pixel path, %u x %u and %u x %u NV12%s)\n", level,
		    pass ? "PASS" : "FAIL", FD_WIDTH, FD_HEIGHT, layout.width, layout.height,
		    published ? ", dump /dev/hakios-camera-fd0-2 and -full0-2" : ", frames not published");
		return pass;
	}
	if (*safe || (!map_error && !unmap_error && !release_error)) {
		kmem_free(kernel_map, kva, size);
	} else {
		cam_log("cam_probe: retaining FD buffer after uncertain SMMU teardown\n");
	}
	cam_log("cam_probe: level %u FAIL (pixel path%s)\n", level, restored_all ? "" : ", restore failed");
	return false;
#else
	cam_log("cam_probe: level %u FAIL (this kernel has no hakios.cam_ife_init.h: tools/camife.py)\n", level);
	*safe = true;
	return false;
#endif
}

/* ---- level 4: /dev/hakios-camera, the rear IMX789's mode 1 frames for Darwin ---- */

/*
 * The ABI of sys/hakios.camera.h. The first read after an open starts a session: the sensor path as
 * level 3 brings it up, the stock init and mode 1 tables, then RDI0 into a ring of STREAM_SLOTS
 * frames. After each buffer done the write master gets the next free slot and an address update in
 * the vertical blanking (about 1.4 ms in mode 1; without one it drops the next frame). CSID0's
 * buffer-done interrupt (GIC INTID 496, DT SPI 0x1d0, rising edge) does that switch to a slot the
 * thread armed beforehand, so the thread only has to re-arm within a frame. If no interrupt comes
 * with the first frame, the session falls back to sleeping until 3.5 ms before each frame is due
 * and spinning on the buffer-done status. A session ends at the last close, after STREAM_IDLE_MS without a read, or on a
 * fault, and its end puts back everything level 3 restores. Reads waiting when a session fails get
 * EIO; the next read starts another session. Only a teardown that could leave DMA running (the
 * stream or the write master would not stop, or the SMMU mapping would not go) keeps the camera off
 * for the rest of the run. cam_probe_fail=1 fails the first session after its third frame, to check
 * that recovery.
 *
 * Exposure: SMIA registers under group hold (0x0104): 0x0340 frame length, 0x0202 coarse integration,
 * 0x0204 analog gain. The frame header's gain is linear, STREAM_GAIN_UNITY for 1x; the register code is
 * 16384 - 16384 / gain (Sony's 14-bit global gain). The module file gives mode 1 64x at most. The
 * margin is the mode table's frame length less its coarse time (3144 - 3096 lines).
 */
#define STREAM_SLOTS          4u
#define STREAM_GAIN_UNITY     256u
#define STREAM_GAIN_MAX       (64u * STREAM_GAIN_UNITY)
#define STREAM_MARGIN         48u
#define STREAM_LINES_MIN      16u
#define STREAM_FRAME_LINES_MAX 0xfff0u
#define STREAM_IDLE_MS        1000u
#define STREAM_NO_FRAME_MS    1500u
#define STREAM_START_GAIN     (2u * STREAM_GAIN_UNITY)
#define STREAM_RING_BYTES     ((vm_size_t)STREAM_SLOTS * RAW_FRAME_BYTES)
#define STREAM_FULL_BASE      round_page(STREAM_RING_BYTES)
#define STREAM_IFE_DONE       (RDI0_BUF_DONE | FULL_BUF_DONE | FD_BUF_DONE)
#define RDI0_RUP_AUP          (1u << 4 | 1u << 20)

static bool stream_ife;
static struct ife_capture_layout stream_full;

static uint32_t
stream_full_iova(unsigned slot)
{
	return (uint32_t)IFE_IOVA + (uint32_t)STREAM_FULL_BASE + slot * stream_full.slot_bytes;
}

static void
stream_arm(vm_offset_t csid, vm_offset_t ife, unsigned slot)
{
	write_reg(ife + IFE_WM_RDI0 + WM_ADDRESS, (uint32_t)IFE_IOVA + slot * RAW_FRAME_BYTES);
	if (stream_ife) {
		uint32_t full = stream_full_iova(slot);
		uint32_t fd = stream_full_iova(STREAM_SLOTS);
		write_reg(ife + IFE_WM_FULL_Y + WM_ADDRESS, full);
		write_reg(ife + IFE_WM_FULL_C + WM_ADDRESS, full + stream_full.chroma_offset);
		write_reg(ife + IFE_WM_FD_Y + WM_ADDRESS, fd);
		write_reg(ife + IFE_WM_FD_C + WM_ADDRESS, fd + FD_Y_BYTES);
	}
	write_reg(csid + CSID_RUP_AUP, RDI0_RUP_AUP | (stream_ife ? IPP_RUP_AUP : 0));
}

/* A slot is publishable only after both payloads have finished DMA. A repeated group
 * before its partners means the completions can no longer be paired reliably. */
static bool
stream_frame_done(uint32_t status, uint32_t *pending, bool *fault)
{
	uint32_t mask = stream_ife ? STREAM_IFE_DONE : RDI0_BUF_DONE;
	uint32_t completed = status & mask;
	if (status == UINT32_MAX || (*pending & completed)) {
		*fault = true;
		return false;
	}
	*pending |= completed;
	if (*pending != mask) {
		return false;
	}
	*pending = 0;
	return true;
}

#if CAM_HAVE_TABLES
_Static_assert(CAM_TABLES_FRAME_LENGTH > STREAM_MARGIN + STREAM_LINES_MIN, "mode 1 frame length");
#define STREAM_FRAME_LINES    CAM_TABLES_FRAME_LENGTH
#define STREAM_LINE_NS        ((uint32_t)(1000000000000ull / ((uint64_t)CAM_TABLES_FPS_MILLI * CAM_TABLES_FRAME_LENGTH)))
#else
#define STREAM_FRAME_LINES    3144u
#define STREAM_LINE_NS        10598u
#endif

enum { SLOT_FREE, SLOT_WRITING, SLOT_READY, SLOT_READING };

static LCK_GRP_DECLARE(stream_lck_grp, "hakios-camera");
static LCK_MTX_DECLARE(stream_lock, &stream_lck_grp);
static struct {
	uint8_t state;
	uint64_t sequence, time;
	uint32_t lines, gain, frame_lines, focus;
} stream_slot[STREAM_SLOTS];
static vm_offset_t stream_ring;
/* under stream_lock: open, a read wants frames, frames coming, a fault for good, reads copying now */
static bool stream_open, stream_wanted, stream_streaming, stream_broken;
static unsigned stream_readers;
static uint64_t stream_given, stream_sequence, stream_last_read, stream_failures; /* stream_failures: sessions that failed */
static uint64_t stream_since; /* when the frames of this session began, in ms */
static uint64_t stream_sessions_count, stream_cur_frames, stream_cur_dropped;
static uint64_t stream_last_frames, stream_last_dropped, stream_last_ms;
static unsigned stream_read_errors;

SYSCTL_DECL(_kern_hakios);
static char cam_probe_cameras[64] = "";
SYSCTL_STRING(_kern_hakios, OID_AUTO, cameras, CTLFLAG_RD | CTLFLAG_LOCKED, cam_probe_cameras, 0,
    "cameras served by /dev/hakios-camera (sensor:name, space-separated)");
static struct {
	uint32_t lines, gain, frame_lines;
	bool pending;
	uint32_t focus;
	bool focus_pending;
} stream_ask = { STREAM_FRAME_LINES - STREAM_MARGIN, STREAM_START_GAIN, 0, false, 0, false };
static struct {
	uint32_t lines, gain, frame_lines;
	uint64_t from;
} stream_now, stream_next;
static uint32_t stream_fail_after;

static uint64_t
stream_ms(void)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / NSEC_PER_MSEC;
}

static int
sysctl_cam_stats SYSCTL_HANDLER_ARGS
{
	char text[200];
	lck_mtx_lock(&stream_lock);
	bool active = stream_streaming;
	uint64_t sessions = stream_sessions_count;
	uint64_t cur_f = stream_cur_frames, cur_d = stream_cur_dropped;
	uint64_t cur_ms = active ? stream_ms() - stream_since : 0;
	uint64_t last_f = stream_last_frames, last_d = stream_last_dropped, last_ms = stream_last_ms;
	lck_mtx_unlock(&stream_lock);
	snprintf(text, sizeof(text), "state=%s sessions=%llu cur_frames=%llu cur_dropped=%llu cur_ms=%llu "
	    "last_frames=%llu last_dropped=%llu last_ms=%llu",
	    active ? "streaming" : "idle", sessions, cur_f, cur_d, cur_ms, last_f, last_d, last_ms);
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, cam_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0, sysctl_cam_stats, "A",
    "IMX789 /dev/hakios-camera session frame and drop counters");

static void
stream_sleep_us(uint32_t us)
{
	/* No timer leeway: a plain kernel wait overslept by milliseconds and missed the address update. */
	uint64_t deadline;
	clock_interval_to_deadline(us, NSEC_PER_USEC, &deadline);
	assert_wait_deadline_with_leeway((event_t)&stream_sleep_us, THREAD_UNINT, TIMEOUT_URGENCY_SYS_CRITICAL, deadline, 0);
	thread_block(THREAD_CONTINUE_NULL);
}

static void
stream_limits(uint32_t *lines, uint32_t *gain, uint32_t *frame_lines)
{
	*gain = MIN(MAX(*gain, STREAM_GAIN_UNITY), STREAM_GAIN_MAX);
	*lines = MIN(MAX(*lines, STREAM_LINES_MIN), STREAM_FRAME_LINES_MAX - STREAM_MARGIN);
	if (*frame_lines == 0) {
		*frame_lines = MAX(*lines + STREAM_MARGIN, STREAM_FRAME_LINES);
	}
	*frame_lines = MIN(MAX(*frame_lines, STREAM_FRAME_LINES), STREAM_FRAME_LINES_MAX);
	*lines = MIN(*lines, *frame_lines - STREAM_MARGIN);
}

#if CAM_HAVE_TABLES
static bool
stream_exposure(vm_offset_t cci, uint32_t lines, uint32_t gain, uint32_t frame_lines)
{
	uint32_t code = 16384u - (uint32_t)((16384ull * STREAM_GAIN_UNITY + gain / 2) / gain);
	return cci_table(cci, cam_holdon_regs, sizeof(cam_holdon_regs) / sizeof(cam_holdon_regs[0]), "group hold") &&
	       cci_write8(cci, 0x0340, (uint8_t)(frame_lines >> 8)) && cci_write8(cci, 0x0341, (uint8_t)frame_lines) &&
	       cci_write8(cci, 0x0202, (uint8_t)(lines >> 8)) && cci_write8(cci, 0x0203, (uint8_t)lines) &&
	       cci_write8(cci, 0x0204, (uint8_t)(code >> 8)) && cci_write8(cci, 0x0205, (uint8_t)code) &&
	       cci_table(cci, cam_holdoff_regs, sizeof(cam_holdoff_regs) / sizeof(cam_holdoff_regs[0]), "group release");
}
#endif

/* The oldest slot that is neither being read nor the one just written; -1 if none. Under stream_lock. */
static int
stream_pick(int writing)
{
	int pick = -1;
	for (int i = 0; i < (int)STREAM_SLOTS; i++) {
		if (i != writing && stream_slot[i].state != SLOT_READING && stream_slot[i].state != SLOT_WRITING &&
		    (pick < 0 || stream_slot[i].sequence < stream_slot[pick].sequence)) {
			pick = i;
		}
	}
	return pick;
}

/*
 * CSID0 buffer done by interrupt. The handler switches the write master to the armed slot (if the
 * thread armed one) and counts; the thread does the bookkeeping. Only the RDI0 buffer-done bit is
 * routed to the line (top IRQ bit 13, seen with RX bit 2 and RDI0 bit 8 after every level 2 frame).
 */
#define CSID0_INTID        (32u + 0x1d0u)
#define CSID_TOP_BUF_DONE  (1u << 13)
#define STREAM_IRQ_STRAYS  64u
#if defined(GENERIC_GIC_V3_IRQ)
extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *, uint32_t), void *context);
extern void gic_v3_spi_detach(uint32_t intid);
#endif
static struct {
	vm_offset_t csid, ife;
	int armed;          /* slot for the next frame, -1 none: taken by the handler */
	int switched;       /* slot the handler switched to, -1 none: taken by the thread */
	uint32_t frames;    /* buffer dones */
	uint32_t strays;    /* interrupts without one */
	uint32_t stray_top; /* top status bits besides buffer done that came with interrupts */
	uint64_t time;      /* mach time of the last buffer done */
	uint32_t pending;   /* completion bits since the last paired frame, owned by the handler */
	bool fault;
} stream_irq = { .armed = -1, .switched = -1 };

static __unused void
stream_irq_arrived(__unused void *context, __unused uint32_t intid)
{
	vm_offset_t csid = __atomic_load_n(&stream_irq.csid, __ATOMIC_ACQUIRE);
	if (!csid) {
		return;
	}
	uint32_t top = read_reg(csid + CSID_TOP_STATUS), buf = read_reg(csid + CSID_BUF_STATUS);
	bool fault = false;
	if (stream_frame_done(buf, &stream_irq.pending, &fault)) {
		int next = __atomic_exchange_n(&stream_irq.armed, -1, __ATOMIC_ACQ_REL);
		if (next >= 0) {
			stream_arm(csid, stream_irq.ife, (unsigned)next);
			__atomic_store_n(&stream_irq.switched, next, __ATOMIC_RELEASE);
		}
		__atomic_store_n(&stream_irq.time, mach_absolute_time(), __ATOMIC_RELAXED);
		__atomic_fetch_add(&stream_irq.frames, 1, __ATOMIC_RELEASE);
	} else if (fault) {
		__atomic_store_n(&stream_irq.fault, true, __ATOMIC_RELEASE);
		write_reg(csid + CSID_TOP_MASK, 0);
	} else if (!(buf & (stream_ife ? STREAM_IFE_DONE : RDI0_BUF_DONE)) &&
	    __atomic_add_fetch(&stream_irq.strays, 1, __ATOMIC_RELAXED) >= STREAM_IRQ_STRAYS) {
		write_reg(csid + CSID_TOP_MASK, 0); /* a storm: the thread sees the strays and polls instead */
	}
	write_reg(csid + CSID_BUF_CLEAR, buf);
	if (top & ~CSID_TOP_BUF_DONE) {
		write_reg(csid + CSID_RX_CLEAR, UINT32_MAX);
		write_reg(csid + CSID_RDI_CLEAR, UINT32_MAX);
		if (stream_ife) {
			write_reg(csid + CSID_IPP_CLEAR, UINT32_MAX);
		}
		__atomic_fetch_or(&stream_irq.stray_top, top, __ATOMIC_RELAXED);
	}
	write_reg(csid + CSID_TOP_CLEAR, top);
	write_reg(csid + CSID_IRQ_CMD, 1);
	thread_wakeup((event_t)&stream_irq.frames);
}

/* Wait up to ms for the interrupt count to pass seen; the count now. */
static uint32_t
stream_irq_wait(uint32_t seen, uint32_t ms)
{
	assert_wait_timeout((event_t)&stream_irq.frames, THREAD_UNINT, ms, NSEC_PER_MSEC);
	uint32_t now = __atomic_load_n(&stream_irq.frames, __ATOMIC_ACQUIRE);
	if (now != seen || __atomic_load_n(&stream_irq.fault, __ATOMIC_ACQUIRE)) {
		clear_wait(current_thread(), THREAD_AWAKENED);
		return now;
	}
	thread_block(THREAD_CONTINUE_NULL);
	return __atomic_load_n(&stream_irq.frames, __ATOMIC_ACQUIRE);
}

static bool stream_irq_off; /* the line stormed or stayed silent: later sessions poll */
static bool stream_irq_attached;

/*
 * Hand the frames back to polling: the line goes, the armed slot is free again. Under no lock. The
 * slot the write master was switched to and the thread has not seen yet, or -1.
 */
static int
stream_irq_stop(vm_offset_t csid)
{
#if defined(GENERIC_GIC_V3_IRQ)
	if (stream_irq_attached) {
		gic_v3_spi_detach(CSID0_INTID); /* no handler running when it returns */
		stream_irq_attached = false;
	}
#endif
	__atomic_store_n(&stream_irq.csid, 0, __ATOMIC_RELEASE);
	if (csid) {
		write_reg(csid + CSID_TOP_MASK, UINT32_MAX);
		write_reg(csid + CSID_RDI_MASK, 0x7fff); /* as cam_rdi_start */
		write_reg(csid + CSID_RX_MASK, 0x0fffffff);
	}
	lck_mtx_lock(&stream_lock);
	int armed = __atomic_exchange_n(&stream_irq.armed, -1, __ATOMIC_ACQ_REL);
	if (armed >= 0 && stream_slot[armed].state == SLOT_WRITING) {
		stream_slot[armed].state = SLOT_FREE;
	}
	int switched = __atomic_exchange_n(&stream_irq.switched, -1, __ATOMIC_ACQ_REL);
	lck_mtx_unlock(&stream_lock);
	return switched;
}

/* Route RDI0 buffer done to CSID0's line and arm slot 1 for the first frame; false if it cannot. */
static bool
stream_irq_start(const struct sensor_hw *hw)
{
#if defined(GENERIC_GIC_V3_IRQ)
	if (stream_irq_off) {
		return false;
	}
	lck_mtx_lock(&stream_lock);
	stream_slot[1].state = SLOT_WRITING;
	lck_mtx_unlock(&stream_lock);
	stream_irq.ife = hw->ife;
	stream_irq.pending = 0;
	__atomic_store_n(&stream_irq.fault, false, __ATOMIC_RELAXED);
	__atomic_store_n(&stream_irq.frames, 0, __ATOMIC_RELAXED);
	__atomic_store_n(&stream_irq.strays, 0, __ATOMIC_RELAXED);
	__atomic_store_n(&stream_irq.stray_top, 0, __ATOMIC_RELAXED);
	__atomic_store_n(&stream_irq.switched, -1, __ATOMIC_RELAXED);
	__atomic_store_n(&stream_irq.armed, 1, __ATOMIC_RELAXED);
	/* Boot 6: with RX and RDI0 unmasked (cam_rdi_start) the line stormed on top bit 2, RX, whatever the top mask. */
	write_reg(hw->csid + CSID_RX_MASK, 0);
	write_reg(hw->csid + CSID_RDI_MASK, 0);
	write_reg(hw->csid + CSID_RX_CLEAR, UINT32_MAX);
	write_reg(hw->csid + CSID_RDI_CLEAR, UINT32_MAX);
	if (stream_ife) {
		write_reg(hw->csid + CSID_IPP_MASK, 0);
		write_reg(hw->csid + CSID_IPP_CLEAR, UINT32_MAX);
	}
	write_reg(hw->csid + CSID_TOP_MASK, CSID_TOP_BUF_DONE);
	write_reg(hw->csid + CSID_TOP_CLEAR, UINT32_MAX);
	write_reg(hw->csid + CSID_BUF_CLEAR, UINT32_MAX);
	write_reg(hw->csid + CSID_IRQ_CMD, 1);
	__atomic_store_n(&stream_irq.csid, hw->csid, __ATOMIC_RELEASE);
	if (gic_v3_spi_attach(CSID0_INTID, true, stream_irq_arrived, NULL) != KERN_SUCCESS) {
		cam_log("cam_probe: stream: CSID0 interrupt %u attach failed; polling\n", CSID0_INTID);
		stream_irq_off = true;
		(void)stream_irq_stop(hw->csid);
		return false;
	}
	stream_irq_attached = true;
	return true;
#else
	(void)hw;
	return false;
#endif
}

/* Polling from here: the slot the write master writes now (a switch the thread had not seen loses one frame). */
static int
stream_irq_fallback(vm_offset_t csid, int writing)
{
	int sw = stream_irq_stop(csid);
	if (sw < 0) {
		return writing;
	}
	lck_mtx_lock(&stream_lock);
	if (stream_slot[writing].state == SLOT_WRITING) {
		stream_slot[writing].state = SLOT_FREE;
	}
	stream_slot[sw].state = SLOT_WRITING;
	lck_mtx_unlock(&stream_lock);
	return sw;
}

/*
 * Frames until the session should end; false on a fault. Called with the sensor streaming into slot 0
 * (and slot 1 armed when irq).
 */
static bool
stream_frames(const struct sensor_hw *hw, struct hakios_smmu_context *ctx, bool irq, uint64_t *frames, uint64_t *dropped)
{
	uint32_t pending = 0;
	int writing = 0;
	uint64_t last_abs = mach_absolute_time(), started_ms = stream_ms(), last_ms = started_ms;
	uint32_t crc_seen = read_reg(hw->csid + 0x248), reported = 0, exposure_logs = 0, late_wakes = 0, irq_seen = 0;
	uint64_t worst_late_us = 0, spin_ns = 0, worst_irq_us = 0;
	for (;;) {
		lck_mtx_lock(&stream_lock);
		bool open = stream_open;
		uint64_t idle = stream_ms() - MAX(stream_last_read, stream_since);
		lck_mtx_unlock(&stream_lock);
		if (!open || idle > STREAM_IDLE_MS) {
			uint64_t ms = stream_ms() - started_ms;
			cam_log("cam_probe: stream: ending after %llu frames (%llu dropped): %s; %s, %u wakes over 1 ms late, worst %llu us, "
			    "spun %llu ms of %llu, worst interrupt-to-thread %llu us\n", *frames, *dropped,
			    !open ? "closed" : "no read for a second", irq ? "by interrupt" : "polling", late_wakes, worst_late_us,
			    spin_ns / NSEC_PER_MSEC, ms, worst_irq_us);
			return true;
		}
		uint64_t period_ns = (uint64_t)stream_now.frame_lines * STREAM_LINE_NS;
		uint64_t now, dt_ns, events = 1, lost, done_sequence;
		int done_slot, next;
		if (irq) {
			if (__atomic_load_n(&stream_irq.strays, __ATOMIC_RELAXED) >= STREAM_IRQ_STRAYS) {
				cam_log("cam_probe: stream: %u CSID0 interrupts without a buffer done (top %#x, bits seen %#x); polling from now on\n",
				    __atomic_load_n(&stream_irq.strays, __ATOMIC_RELAXED), read_reg(hw->csid + CSID_TOP_STATUS),
				    __atomic_load_n(&stream_irq.stray_top, __ATOMIC_RELAXED));
				stream_irq_off = true;
				if (stream_ife) {
					return false;
				}
				writing = stream_irq_fallback(hw->csid, writing);
				irq = false;
				continue;
			}
			uint32_t count = stream_irq_wait(irq_seen, 100);
			if (__atomic_load_n(&stream_irq.fault, __ATOMIC_ACQUIRE)) {
				cam_log("cam_probe: stream: buffer completions out of sync\n");
				return false;
			}
			if (count == irq_seen) {
				uint64_t quiet = stream_ms() - last_ms;
				if (*frames == 0 && quiet > 300 && (read_reg(hw->csid + CSID_BUF_STATUS) & RDI0_BUF_DONE)) {
					cam_log("cam_probe: stream: buffer done without a CSID0 interrupt (top %#x); polling from now on\n",
					    read_reg(hw->csid + CSID_TOP_STATUS));
					stream_irq_off = true;
					if (stream_ife) {
						return false;
					}
					writing = stream_irq_fallback(hw->csid, writing);
					irq = false;
					continue;
				}
				if (quiet > STREAM_NO_FRAME_MS + period_ns / NSEC_PER_MSEC) {
					cam_log("cam_probe: stream: no frame interrupt for %llu ms (%llu frames), CSID buf %#x top %#x RDI %#x RX %#x, "
					    "packets %u CRC %u\n", quiet, *frames, read_reg(hw->csid + CSID_BUF_STATUS),
					    read_reg(hw->csid + CSID_TOP_STATUS), read_reg(hw->csid + CSID_RDI_STATUS),
					    read_reg(hw->csid + CSID_RX_STATUS), read_reg(hw->csid + 0x240), read_reg(hw->csid + 0x248));
					return false;
				}
				continue;
			}
			uint64_t woke = mach_absolute_time(), late_ns;
			events = count - irq_seen;
			irq_seen = count;
			now = __atomic_load_n(&stream_irq.time, __ATOMIC_RELAXED);
			absolutetime_to_nanoseconds(now - last_abs, &dt_ns);
			absolutetime_to_nanoseconds(woke > now ? woke - now : 0, &late_ns);
			worst_irq_us = MAX(worst_irq_us, late_ns / 1000);
			lck_mtx_lock(&stream_lock);
			int sw = __atomic_exchange_n(&stream_irq.switched, -1, __ATOMIC_ACQ_REL);
			/* The first interrupt took the armed slot; frames after it went over that slot. */
			done_slot = sw >= 0 ? writing : -1;
			lost = sw >= 0 ? events - 1 : events;
			done_sequence = stream_sequence + 1;
			if (sw >= 0) {
				writing = sw;
			}
			next = __atomic_load_n(&stream_irq.armed, __ATOMIC_ACQUIRE);
			if (next < 0) {
				next = stream_pick(writing);
				if (next >= 0) {
					stream_slot[next].state = SLOT_WRITING;
					__atomic_store_n(&stream_irq.armed, next, __ATOMIC_RELEASE);
				}
			}
		} else {
			uint32_t status = read_reg(hw->csid + CSID_BUF_STATUS);
			if (status == UINT32_MAX) {
				cam_log("cam_probe: stream: CSID0 reads all ones\n");
				return false;
			}
			bool fault = false;
			bool done = stream_frame_done(status, &pending, &fault);
			if (fault) {
				cam_log("cam_probe: stream: buffer completions out of sync (status %#x, pending %#x)\n", status, pending);
				return false;
			}
			if (!done) {
				if (status) {
					write_reg(hw->csid + CSID_BUF_CLEAR, status);
					write_reg(hw->csid + CSID_IRQ_CMD, 1);
				}
				uint64_t since_ns;
				absolutetime_to_nanoseconds(mach_absolute_time() - last_abs, &since_ns);
				if (stream_ms() - last_ms > STREAM_NO_FRAME_MS + period_ns / NSEC_PER_MSEC) {
					cam_log("cam_probe: stream: no frame for %llu ms (%llu frames), CSID buf %#x RDI %#x RX %#x, "
					    "packets %u CRC %u\n", stream_ms() - last_ms, *frames, status, read_reg(hw->csid + CSID_RDI_STATUS),
					    read_reg(hw->csid + CSID_RX_STATUS), read_reg(hw->csid + 0x240), read_reg(hw->csid + 0x248));
					return false;
				}
				/*
				 * Sleep until 3.5 ms before the frame is due, then spin: buffer done to the next SOF is 1.4 ms,
				 * and zero-leeway wakes still came up to 1.9 ms late (boot 4: 8 frames dropped with 1.5 ms).
				 */
				uint64_t wake_ns = period_ns > 3500000 ? period_ns - 3500000 : 0;
				if (since_ns + 200000 < wake_ns) {
					uint64_t asked = (wake_ns - since_ns) / 1000, t0 = mach_absolute_time(), slept;
					stream_sleep_us((uint32_t)asked);
					absolutetime_to_nanoseconds(mach_absolute_time() - t0, &slept);
					if (slept / 1000 > asked + 1000) {
						late_wakes++;
						worst_late_us = MAX(worst_late_us, slept / 1000 - asked);
					}
				} else {
					uint64_t t0 = mach_absolute_time(), t1;
					spin_us(20);
					absolutetime_to_nanoseconds(mach_absolute_time() - t0, &t1);
					spin_ns += t1;
				}
				continue;
			}
			/* The next address first: the vertical blanking is short. */
			now = mach_absolute_time();
			absolutetime_to_nanoseconds(now - last_abs, &dt_ns);
			lck_mtx_lock(&stream_lock);
			next = stream_pick(writing);
			if (next >= 0) {
				stream_arm(hw->csid, hw->ife, (unsigned)next);
			}
			write_reg(hw->csid + CSID_BUF_CLEAR, status);
			write_reg(hw->csid + CSID_IRQ_CMD, 1);
			/* Frames the write master dropped (no address in time) show as a gap in the sequence. */
			events = *frames == 0 ? 1 : MAX(1ull, (dt_ns + period_ns / 2) / period_ns);
			lost = events - 1;
			done_slot = writing;
			done_sequence = stream_sequence + events;
			if (next >= 0) {
				stream_slot[next].state = SLOT_WRITING;
			}
		}
		*dropped += lost;
		stream_sequence += events;
		*frames += irq ? events : 1;
		stream_cur_frames = *frames;
		stream_cur_dropped = *dropped;
		if (stream_next.from && stream_sequence >= stream_next.from) {
			stream_now.lines = stream_next.lines;
			stream_now.gain = stream_next.gain;
			stream_now.frame_lines = stream_next.frame_lines;
			stream_next.from = 0;
		}
		if (done_slot >= 0) {
			stream_slot[done_slot].state = SLOT_READY;
			stream_slot[done_slot].sequence = done_sequence;
			stream_slot[done_slot].time = now;
			stream_slot[done_slot].lines = stream_now.lines;
			stream_slot[done_slot].gain = stream_now.gain;
			stream_slot[done_slot].frame_lines = stream_now.frame_lines;
			stream_slot[done_slot].focus = af_code;
		}
		if (!irq && next >= 0) {
			writing = next;
		}
		bool ask = stream_ask.pending;
		uint32_t lines = stream_ask.lines, gain = stream_ask.gain, frame_lines = stream_ask.frame_lines;
		stream_ask.pending = false;
		bool focus = stream_ask.focus_pending;
		uint32_t position = stream_ask.focus;
		stream_ask.focus_pending = false;
		lck_mtx_unlock(&stream_lock);
		wakeup(&stream_sequence);
		if (focus) {
			af_move(hw->cci, position);
		}
		last_abs = now;
		last_ms = stream_ms();
		if (next < 0) {
			cam_log("cam_probe: stream: no free slot after frame %llu\n", stream_sequence);
			return false;
		}
		if (*frames <= 3 || *frames % 300 == 0) {
			cam_log("cam_probe: stream: frame %llu (%llu done, %llu dropped) into slot %d, %llu us after the last, "
			    "exposure %u lines gain %u/%u frame %u%s\n", stream_sequence, *frames, *dropped, writing, dt_ns / 1000,
			    stream_now.lines, stream_now.gain, STREAM_GAIN_UNITY, stream_now.frame_lines, irq ? ", by interrupt" : "");
		}
#if CAM_HAVE_TABLES
		if (ask) {
			stream_limits(&lines, &gain, &frame_lines);
			if (lines != stream_now.lines || gain != stream_now.gain || frame_lines != stream_now.frame_lines) {
				uint64_t t0 = mach_absolute_time(), took;
				bool written = stream_exposure(hw->cci, lines, gain, frame_lines);
				absolutetime_to_nanoseconds(mach_absolute_time() - t0, &took);
				if (exposure_logs++ < 6) {
					cam_log("cam_probe: stream: exposure %u lines gain %u/%u frame %u written in %llu us\n", lines, gain,
					    STREAM_GAIN_UNITY, frame_lines, took / 1000);
				}
				if (!written) {
					cam_log("cam_probe: stream: exposure write failed\n");
					return false;
				}
				lck_mtx_lock(&stream_lock);
				stream_next.lines = lines;
				stream_next.gain = gain;
				stream_next.frame_lines = frame_lines;
				stream_next.from = stream_sequence + 2;
				lck_mtx_unlock(&stream_lock);
			}
		}
#else
		(void)ask;
		(void)lines;
		(void)gain;
		(void)frame_lines;
#endif
		if (*frames % 30 == 0) {
			uint32_t violation = read_reg(hw->ife + 0xc64), overflow = read_reg(hw->ife + 0xc68);
			uint32_t image = read_reg(hw->ife + 0xc70), crc = read_reg(hw->csid + 0x248);
			uint32_t noc = read_reg(hw->camnoc + 0x2010);
			if ((violation || overflow || image || crc != crc_seen || (noc & 1u)) && reported++ < 8) {
				cam_log("cam_probe: stream: frame %llu: IFE violation %#x overflow %#x image %#x, CRC %u (was %u), "
				    "CAMNOC errvld %#x\n", stream_sequence, violation, overflow, image, crc, crc_seen, noc);
			}
			crc_seen = crc;
			if (noc & 1u) {
				return false;
			}
		}
		if (*frames % 900 == 0 && !hakios_smmu_faults(ctx)) {
			return false;
		}
		if (stream_fail_after && *frames == stream_fail_after) {
			stream_fail_after = 0;
			cam_log("cam_probe: stream: cam_probe_fail: failing this session after frame %llu\n", stream_sequence);
			return false;
		}
	}
}

/* One session. False if the teardown could leave DMA running: the camera then stays off. */
static bool
stream_session(void)
{
	bool ok = false, streaming = false, wm_stopped = true, torn_down = true;
	uint64_t frames = 0, dropped = 0, started = stream_ms();
	const char *step = "frame ring";
	struct sensor_path p = { .plls = stream_ife ? pixel_plls : NULL };
#if CAM_HAVE_IFE_INIT && defined(CAM_IFE_CDM_BYTES)
	struct ife_cdm cdm = { .idle = true };
#endif
	bool cdm_released = true;
	vm_size_t size = round_page(STREAM_RING_BYTES);
	if (stream_ife) {
		size += STREAM_SLOTS * stream_full.slot_bytes + FD_SLOT_BYTES;
	}
	vm_size_t mapped = 0;
	vm_offset_t kva = 0;
	int map_error = 0, unmap_error = 0, release_error = 0;
	struct hakios_smmu_context *ctx = NULL;
	if (kmem_alloc(kernel_map, &kva, size, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		kva = 0;
		goto out;
	}
	flush_dcache(kva, (unsigned)size, FALSE);
	step = "IFE stream claim";
	ctx = hakios_smmu_claim(IFE_SID, IFE_SID_MASK, IFE_IOVA, size, "camera IFE stream");
	if (!ctx) {
		goto out;
	}
	step = "IFE mapping";
	uint64_t ring_ms = stream_ms();
	map_error = map_buffer(ctx, IFE_IOVA, kva, size, &mapped);
	if (map_error) {
		goto out;
	}
	cam_log("cam_probe: stream: ring of %lu KiB allocated and claimed in %llu ms, mapped in %llu ms\n",
	    (unsigned long)(size / 1024), ring_ms - started, stream_ms() - ring_ms);
	if (!sensor_path_up(&p)) {
		step = p.step;
		goto out;
	}
#if CAM_HAVE_TABLES
	uint64_t up_ms = stream_ms();
	step = "IMX789 init and mode 1 tables";
	if (!cci_table(p.hw.cci, cam_init_regs, sizeof(cam_init_regs) / sizeof(cam_init_regs[0]), "init") ||
	    !cci_table(p.hw.cci, cam_mode_regs, sizeof(cam_mode_regs) / sizeof(cam_mode_regs[0]), "mode")) {
		goto out;
	}
	lck_mtx_lock(&stream_lock);
	uint32_t lines = stream_ask.lines, gain = stream_ask.gain, frame_lines = stream_ask.frame_lines;
	stream_ask.pending = false;
	stream_given = stream_sequence = 0;
	for (unsigned i = 0; i < STREAM_SLOTS; i++) {
		stream_slot[i].state = SLOT_FREE;
		stream_slot[i].sequence = 0;
	}
	stream_slot[0].state = SLOT_WRITING;
	stream_ring = kva;
	lck_mtx_unlock(&stream_lock);
	stream_limits(&lines, &gain, &frame_lines);
	cam_log("cam_probe: stream: sensor path up in %llu ms, 486 table writes in %llu ms\n", up_ms - started,
	    stream_ms() - up_ms);
	step = "lens";
	af_up(p.hw.cci);
	lck_mtx_lock(&stream_lock);
	stream_ask.focus_pending = false;
	lck_mtx_unlock(&stream_lock);
	step = "first exposure";
	if (!stream_exposure(p.hw.cci, lines, gain, frame_lines)) {
		goto out;
	}
	stream_now.lines = lines;
	stream_now.gain = gain;
	stream_now.frame_lines = frame_lines;
	stream_next.from = 0;
	step = "CSID0 reset";
	if (!csid_reset(p.hw.csid)) {
		goto out;
	}
#if CAM_HAVE_IFE_INIT && defined(CAM_IFE_CDM_BYTES)
	if (stream_ife) {
		step = "IFE RT CDM replay";
		if (!ife_cdm_prepare(&cdm) || !ife_cdm_replay(&cdm)) {
			goto out;
		}
		ife_pixel_configure(&p.hw);
		if (stream_full.native) {
			ife_native_size(p.hw.ife);
		}
		uint32_t full = stream_full_iova(0);
		uint32_t fd = stream_full_iova(STREAM_SLOTS);
		write_reg(p.hw.ife + IFE_WM_FULL_Y + 0x48, 0);
		write_reg(p.hw.ife + IFE_WM_FULL_C + 0x48, 0);
		nv12_wm_start(p.hw.ife + IFE_WM_FULL_Y, full, stream_full.width, stream_full.height, stream_full.stride, 0);
		nv12_wm_start(p.hw.ife + IFE_WM_FULL_C, full + stream_full.chroma_offset, stream_full.width, stream_full.height / 2,
		    stream_full.stride, 0);
		nv12_wm_start(p.hw.ife + IFE_WM_FD_Y, fd, FD_WIDTH, FD_HEIGHT, FD_WIDTH, 0);
		nv12_wm_start(p.hw.ife + IFE_WM_FD_C, fd + FD_Y_BYTES, FD_WIDTH, FD_HEIGHT / 2, FD_WIDTH, 0);
		ipp_configure(&p.hw);
	}
#endif
	csiphy_start(p.hw.phy, CAM_TABLES_SETTLE);
	cam_rdi_start(p.hw.csid, p.hw.ife + IFE_WM_RDI0, RAW_STRIDE, 0, (uint32_t)IFE_IOVA, SENSOR_RX_CFG0, rx_capture(1));
	if (stream_ife) {
		write_reg(p.hw.csid + CSID_BUF_MASK, STREAM_IFE_DONE);
		write_reg(p.hw.csid + CSID_IPP_CTRL, 1);
	}
	bool irq = stream_irq_start(&p.hw);
	step = "stream on";
	streaming = true;
	if (!cci_write8(p.hw.cci, IMX789_MODE_SELECT, 1)) {
		goto out;
	}
	lck_mtx_lock(&stream_lock);
	stream_streaming = true;
	stream_since = stream_ms();
	stream_cur_frames = 0;
	stream_cur_dropped = 0;
	lck_mtx_unlock(&stream_lock);
	cam_log("cam_probe: stream: IMX789 mode 1 %ux%u stride %u, %u slots, exposure %u lines gain %u/%u frame %u lines of %u ns, "
	    "started in %llu ms, frames %s\n", RAW_WIDTH, RAW_HEIGHT, RAW_STRIDE, STREAM_SLOTS, lines, gain, STREAM_GAIN_UNITY, frame_lines,
	    STREAM_LINE_NS, stream_ms() - started, irq ? "by CSID0 interrupt" : "by polling");
	if (stream_ife) {
		cam_log("cam_probe: stream: IFE FULL %ux%u stride %u NV12 + RAW10\n",
		    stream_full.width, stream_full.height, stream_full.stride);
	}
	step = "frames";
	ok = stream_frames(&p.hw, ctx, irq, &frames, &dropped);
#else
	step = "IMX789 tables (tools/camsensor.py --c-header hakios.cam_imx789.h 1)";
#endif
out:
	lck_mtx_lock(&stream_lock);
	stream_streaming = false;
	stream_last_frames = frames;
	stream_last_dropped = dropped;
	stream_last_ms = stream_ms() - started;
	if (!ok) {
		stream_failures++;
	}
	for (unsigned i = 0; i < STREAM_SLOTS; i++) {
		if (stream_slot[i].state != SLOT_READING) {
			stream_slot[i].state = SLOT_FREE;
		}
	}
	lck_mtx_unlock(&stream_lock);
	wakeup(&stream_sequence);
	(void)stream_irq_stop(p.hw.csid && p.routed ? p.hw.csid : 0);
	if (p.hw.csid && p.routed) {
		vm_offset_t wm = p.hw.ife + IFE_WM_RDI0;
		write_reg(p.hw.csid + CSID_RDI_CTRL, 0); /* halt at the next frame boundary */
		if (stream_ife) {
			write_reg(p.hw.csid + CSID_IPP_CTRL, 0);
		}
		if (streaming) {
			streaming = !cci_write8(p.hw.cci, IMX789_MODE_SELECT, 0);
		}
		delay(40000);
		write_reg(wm + WM_CFG, 0);
		wm_stopped = poll_reg(wm + WM_CFG, WM_ENABLE, 0, 500);
		cam_log("cam_probe: stream: CSID RX %#x RDI %#x top %#x, packets %u CRC %u ECC %#x, IFE violation %#x overflow %#x "
		    "image %#x, CAMNOC errvld %#x\n", read_reg(p.hw.csid + CSID_RX_STATUS), read_reg(p.hw.csid + CSID_RDI_STATUS),
		    read_reg(p.hw.csid + CSID_TOP_STATUS), read_reg(p.hw.csid + 0x240), read_reg(p.hw.csid + 0x248),
		    read_reg(p.hw.csid + 0x244), read_reg(p.hw.ife + 0xc64), read_reg(p.hw.ife + 0xc68), read_reg(p.hw.ife + 0xc70),
		    read_reg(p.hw.camnoc + 0x2010));
#if CAM_HAVE_IFE_INIT
		if (stream_ife) {
			wm_stopped = outputs_stop(&p.hw) && wm_stopped;
			write_reg(p.hw.csid + CSID_RDI_CFG0, 0);
			write_reg(p.hw.csid + CSID_RDI_MASK, 0);
			torn_down = ipp_teardown(&p.hw);
		} else
#endif
		{
			torn_down = rdi_teardown(&p.hw);
		}
	}
#if CAM_HAVE_IFE_INIT && defined(CAM_IFE_CDM_BYTES)
	cdm_released = ife_cdm_release(&cdm);
#endif
	if (p.hw.cci && p.routed) {
		af_park(p.hw.cci);
	}
	bool restored = sensor_path_down(&p, &streaming) && torn_down;
	bool clean = ctx ? hakios_smmu_faults(ctx) : true;
	unmap_error = mapped && wm_stopped && !streaming ? hakios_smmu_unmap(ctx, IFE_IOVA, mapped) : mapped ? EBUSY : 0;
	release_error = ctx && !unmap_error ? hakios_smmu_release(ctx) : ctx ? EBUSY : 0;
	/* No read copies from the ring after this: every slot is FREE or a reader is still in it. */
	lck_mtx_lock(&stream_lock);
	while (stream_readers) {
		(void)msleep(&stream_readers, &stream_lock, PZERO, "hakios-camera-drain", NULL);
	}
	stream_ring = 0;
	lck_mtx_unlock(&stream_lock);
	bool safe = !streaming && wm_stopped && !unmap_error && !release_error && cdm_released;
	if (kva && safe) {
		kmem_free(kernel_map, kva, size);
	} else if (kva) {
		cam_log("cam_probe: stream: retaining the frame ring after uncertain teardown\n");
	}
	cam_log("cam_probe: stream: session %s at %s: %llu frames (%llu dropped) in %llu ms; stream stopped %u, write master stopped %u, "
	    "SMMU %s map %d unmap %d release %d; clocks and power restore %s\n", ok ? "ended" : "FAILED", step, frames, dropped,
	    stream_ms() - started, (unsigned)!streaming, (unsigned)wm_stopped, clean ? "clean" : "FAULTS", map_error, unmap_error,
	    release_error, restored ? "OK" : "FAILED");
	return safe;
}

static int
stream_open_dev(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	char who[32];
	proc_selfname(who, sizeof(who));
	lck_mtx_lock(&stream_lock);
	bool broken = stream_broken;
	stream_open = !broken;
	lck_mtx_unlock(&stream_lock);
	cam_log("cam_probe: stream: opened by %s%s\n", who, broken ? " (camera off: EIO)" : "");
	return broken ? EIO : 0;
}

/* The last close. */
static int
stream_close_dev(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	lck_mtx_lock(&stream_lock);
	stream_open = false;
	stream_wanted = false;
	lck_mtx_unlock(&stream_lock);
	wakeup(&stream_open);
	char who[32];
	proc_selfname(who, sizeof(who));
	cam_log("cam_probe: stream: last close by %s\n", who);
	return 0;
}

static int
stream_read_dev(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	struct hakios_camera_frame f;
	uint64_t since = stream_ms();
	int b = -1, error = 0;
	lck_mtx_lock(&stream_lock);
	uint64_t failures = stream_failures;
	stream_last_read = stream_ms();
	for (;;) {
		if (stream_broken || stream_failures != failures) {
			bool say = stream_read_errors++ < 8;
			lck_mtx_unlock(&stream_lock);
			if (say) {
				cam_log("cam_probe: stream: read EIO (%s)\n", stream_broken ? "camera off" : "the session failed");
			}
			return EIO;
		}
		b = -1;
		for (int i = 0; i < (int)STREAM_SLOTS; i++) {
			if (stream_slot[i].state == SLOT_READY && stream_slot[i].sequence > stream_given &&
			    (b < 0 || stream_slot[i].sequence > stream_slot[b].sequence)) {
				b = i;
			}
		}
		if (b >= 0 && stream_ring) {
			break;
		}
		/* 3 s without a frame once they come; 15 s for a session to start (counted from when it began streaming). */
		uint64_t waited = stream_ms() - (stream_streaming ? MAX(since, stream_since) : since);
		if (waited > (stream_streaming ? 3000u : 15000u)) {
			bool say = stream_read_errors++ < 8;
			bool streaming = stream_streaming;
			lck_mtx_unlock(&stream_lock);
			if (say) {
				cam_log("cam_probe: stream: read EAGAIN after %llu ms (%s)\n", waited, streaming ? "streaming" : "not streaming");
			}
			return EAGAIN;
		}
		if (!stream_streaming && !stream_wanted) {
			stream_wanted = true;
			wakeup(&stream_open);
		}
		struct timespec ts = { 0, 100 * NSEC_PER_MSEC };
		error = msleep(&stream_sequence, &stream_lock, PZERO | PCATCH, "hakios-camera", &ts);
		if (error != 0 && error != EWOULDBLOCK) {
			lck_mtx_unlock(&stream_lock);
			return error;
		}
		stream_last_read = stream_ms();
	}
	stream_slot[b].state = SLOT_READING;
	stream_readers++;
	vm_offset_t at = stream_ring + (vm_offset_t)b * RAW_FRAME_BYTES;
	stream_given = stream_slot[b].sequence;
	bzero(&f, sizeof(f));
	memcpy(f.magic, HAKIOS_CAMERA_MAGIC, 8);
	f.header_bytes = sizeof(f);
	f.width = RAW_WIDTH;
	f.height = RAW_HEIGHT;
	f.stride = RAW_STRIDE;
	f.format = 1;
	f.bayer = 3; /* BGGR, confirmed in level 3 */
	f.black = RAW_BLACK;
	f.sequence = stream_slot[b].sequence;
	f.time = stream_slot[b].time;
	f.exposure_lines = stream_slot[b].lines;
	f.line_ns = STREAM_LINE_NS;
	f.gain = stream_slot[b].gain;
	f.gain_unity = STREAM_GAIN_UNITY;
	f.gain_max = STREAM_GAIN_MAX;
	f.frame_lines = stream_slot[b].frame_lines;
	f.frame_lines_min = STREAM_FRAME_LINES;
	f.frame_lines_max = STREAM_FRAME_LINES_MAX;
	f.margin = STREAM_MARGIN;
	f.exposure_lines_min = STREAM_LINES_MIN;
	f.sensor = 1;
	f.focus = stream_slot[b].focus;
	if (stream_ife) {
		f.preview_width = stream_full.width;
		f.preview_height = stream_full.height;
		f.preview_stride = stream_full.stride;
	}
	vm_offset_t preview = stream_ring + STREAM_FULL_BASE + (vm_offset_t)b * stream_full.slot_bytes;
	lck_mtx_unlock(&stream_lock);

	error = uiomove((caddr_t)&f, (int)MIN(sizeof(f), (size_t)uio_resid(uio)), uio);
	if (error == 0 && uio_resid(uio) > 0) {
		uint32_t n = (uint32_t)MIN((uint64_t)RAW_FRAME_BYTES, (uint64_t)uio_resid(uio));
		uint64_t t0 = mach_absolute_time(), t1, t2;
		/* What the IFE wrote went past the caches; the CPU never writes the ring, so nothing is dirty. */
		flush_dcache(at, n, FALSE);
		t1 = mach_absolute_time();
		error = uiomove((caddr_t)at, (int)n, uio);
		t2 = mach_absolute_time();
		if (f.sequence <= 2 || f.sequence % 300 == 0) {
			uint64_t a, c;
			absolutetime_to_nanoseconds(t1 - t0, &a);
			absolutetime_to_nanoseconds(t2 - t1, &c);
			cam_log("cam_probe: stream: frame %llu read: %u bytes, caches %llu us, copy %llu us\n", f.sequence, n,
			    a / 1000, c / 1000);
		}
	}
	if (error == 0 && stream_ife && uio_resid(uio) > 0) {
		uint32_t n = (uint32_t)MIN((uint64_t)stream_full.chroma_offset * 3 / 2, (uint64_t)uio_resid(uio));
		flush_dcache(preview, n, FALSE);
		error = uiomove((caddr_t)preview, (int)n, uio);
	}
	lck_mtx_lock(&stream_lock);
	if (stream_slot[b].state == SLOT_READING) {
		stream_slot[b].state = stream_streaming ? SLOT_READY : SLOT_FREE;
	}
	stream_last_read = stream_ms();
	stream_readers--;
	lck_mtx_unlock(&stream_lock);
	wakeup(&stream_readers);
	return error;
}

/* HAKIOS_CAMERA_KEEP's picture, readable as /dev/hakios-camera-keep: an NCAMKEP1 header, then the bytes. */
static LCK_MTX_DECLARE(keep_lock, &stream_lck_grp);
static vm_offset_t keep_buffer;
static vm_size_t keep_size;
static struct ncam_header keep_header;

static int
keep_picture(const struct hakios_camera_command *c, struct uio *uio)
{
	if (c->bytes == 0 || c->bytes > 64u << 20 || c->d < 1 || c->d > 3 || uio_resid(uio) < (user_ssize_t)c->bytes) {
		return EINVAL;
	}
	vm_size_t size = round_page(c->bytes);
	vm_offset_t kva;
	if (kmem_alloc(kernel_map, &kva, size, KMA_KOBJECT | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return ENOMEM;
	}
	int error = uiomove((caddr_t)kva, (int)c->bytes, uio);
	if (error) {
		kmem_free(kernel_map, kva, size);
		return error;
	}
	lck_mtx_lock(&keep_lock);
	vm_offset_t old = keep_buffer;
	vm_size_t old_size = keep_size;
	keep_buffer = kva;
	keep_size = size;
	keep_header = (struct ncam_header){
		"NCAMKEP1", 1, c->a, c->b, c->c, c->bytes, c->d, c->d == 1 ? 10u : 8u, c->d == 1 ? 1u : 0u, 3, 1, { 0 },
	};
	lck_mtx_unlock(&keep_lock);
	if (old) {
		kmem_free(kernel_map, old, old_size);
	}
	cam_log("cam_probe: stream: a picture for the host: %ux%u stride %u, format %u, %u bytes (/dev/hakios-camera-keep)\n",
	    c->a, c->b, c->c, c->d, c->bytes);
	return 0;
}

static int
stream_write_dev(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	struct hakios_camera_command c;
	if (uio_resid(uio) < (user_ssize_t)sizeof(c) || uiomove((caddr_t)&c, sizeof(c), uio) != 0) {
		return EINVAL;
	}
	switch (c.command) {
	case HAKIOS_CAMERA_EXPOSURE:
		lck_mtx_lock(&stream_lock);
		stream_ask.lines = c.a;
		stream_ask.gain = c.b;
		stream_ask.frame_lines = c.c;
		stream_ask.pending = true;
		lck_mtx_unlock(&stream_lock);
		return 0;
	case HAKIOS_CAMERA_KEEP:
		return keep_picture(&c, uio);
	case HAKIOS_CAMERA_SENSOR:
		return c.a == 1 ? 0 : ENODEV; /* only the rear main camera so far */
	case HAKIOS_CAMERA_FOCUS:
		lck_mtx_lock(&stream_lock);
		stream_ask.focus = c.a;
		stream_ask.focus_pending = true;
		lck_mtx_unlock(&stream_lock);
		return 0;
	default:
		return EINVAL;
	}
}

static int
keep_open_dev(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	return 0;
}

static int
keep_read_dev(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	off_t off = uio_offset(uio);
	int error = 0;
	if (off < 0) {
		return EINVAL;
	}
	lck_mtx_lock(&keep_lock);
	if (!keep_buffer) {
		lck_mtx_unlock(&keep_lock);
		return 0;
	}
	if (off < (off_t)sizeof(keep_header)) {
		error = uiomove((caddr_t)&keep_header + off, (int)MIN(uio_resid(uio), (off_t)sizeof(keep_header) - off), uio);
		off = uio_offset(uio);
	}
	off -= (off_t)sizeof(keep_header);
	if (!error && uio_resid(uio) > 0 && off >= 0 && off < (off_t)keep_header.bytes) {
		error = uiomove((caddr_t)(keep_buffer + off), (int)MIN(uio_resid(uio), (off_t)keep_header.bytes - off), uio);
	}
	lck_mtx_unlock(&keep_lock);
	return error;
}

static const struct cdevsw stream_cdevsw = {
	.d_open = stream_open_dev, .d_close = stream_close_dev, .d_read = stream_read_dev, .d_write = stream_write_dev,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0,
};
static const struct cdevsw keep_cdevsw = {
	.d_open = keep_open_dev, .d_close = nulldev, .d_read = keep_read_dev, .d_write = eno_rdwrt,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0,
};

/* Made as the probe thread starts, before mediaserverd looks for cameras. */
static bool
stream_make_nodes(void)
{
	int major = cdevsw_add(-1, &stream_cdevsw), keep_major = cdevsw_add(-1, &keep_cdevsw);
	return major >= 0 && keep_major >= 0 &&
	       devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "hakios-camera") != NULL &&
	       devfs_make_node(makedev(keep_major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0444, "hakios-camera-keep") != NULL;
}

/* A session whenever an open device is read, until a teardown that could leave DMA running. */
static void
stream_serve(bool from_probe)
{
	(void)PE_parse_boot_argn("cam_probe_fail", &stream_fail_after, sizeof(stream_fail_after));
	stream_fail_after = stream_fail_after ? 3 : 0;
	cam_log("cam_probe: %s serving /dev/hakios-camera (IMX789 mode 1, %u x %u RAW10 BGGR)%s\n",
	    from_probe ? "level 4" : "cam=5", RAW_WIDTH, RAW_HEIGHT,
	    stream_fail_after ? ", the first session to fail after 3 frames" : "");
	unsigned sessions = 0;
	for (;;) {
		lck_mtx_lock(&stream_lock);
		while (!(stream_open && stream_wanted)) {
			(void)msleep(&stream_open, &stream_lock, PZERO, "hakios-camera-idle", NULL);
		}
		stream_wanted = false;
		stream_sessions_count = ++sessions;
		lck_mtx_unlock(&stream_lock);
		cam_log("cam_probe: stream: session %u starting\n", sessions);
		if (!stream_session()) {
			lck_mtx_lock(&stream_lock);
			stream_broken = true;
			lck_mtx_unlock(&stream_lock);
			wakeup(&stream_sequence);
			cam_log("cam_probe: %s FAIL (session %u teardown uncertain: the camera stays off)\n",
			    from_probe ? "level 4" : "cam=5", sessions);
			return;
		}
	}
}

static void
cam_probe_thread(void *arg, __unused wait_result_t result)
{
	unsigned level = (unsigned)(uintptr_t)arg;
	if (level == 0) {
		stream_serve(false);
		thread_terminate(current_thread());
		return;
	}
	/* Give USB logging time to start before testing the camera power domain. */
	assert_wait_timeout((event_t)&cam_probe_thread, THREAD_UNINT, 12, NSEC_PER_SEC);
	thread_block(THREAD_CONTINUE_NULL);
	cam_log("cam_probe: mapping camera registers\n");
	struct cam_probe cam = { 0 };
	cam.gcc = map_registers(GCC_PA, 0x1000);
	cam.camcc = map_registers(CAMCC_PA, CAMCC_SIZE);
	cam.cci = map_registers(CCI_PA, 0x1000);
	cam.tlmm = map_registers(TLMM_PA, TLMM_SIZE);
	const char *step = "MMIO map";
	bool ok = cam.gcc && cam.camcc && cam.cci && cam.tlmm;
	if (ok) {
		step = "camera clocks";
		cam_log("cam_probe: starting camera clocks\n");
		ok = cam_clocks_up(&cam);
	}
	if (ok) {
		step = "CCI0 reset";
		ok = cci_start(&cam);
	}
	if (ok) {
		step = "sensor power and chip ID";
		ok = sensor_probe(&cam);
	}
	bool restored = cam_restore(&cam);
	unmap_registers(cam.tlmm, TLMM_SIZE);
	unmap_registers(cam.cci, 0x1000);
	unmap_registers(cam.camcc, CAMCC_SIZE);
	unmap_registers(cam.gcc, 0x1000);
	if (ok && restored) {
		cam_log("cam_probe: level 1 PASS (IMX789 0x0789, state restored)\n");
		if (level >= 2) {
			cam_log("cam_probe: level 2 starting test-pattern capture\n");
			if (cam_tpg_capture() && level >= 3) {
				cam_log("cam_probe: level 3 starting IMX789 capture\n");
				bool sensor_ok = cam_sensor_capture();
				if (sensor_ok && level == 7) {
					cam_log("cam_probe: level 7 starting IMX789 mode 0 capture\n");
					(void)cam_hires_capture();
				} else if (sensor_ok && level >= 4) {
					bool safe = true;
					if (level >= 5) {
						cam_log("cam_probe: level %u starting the IFE0 pixel path\n", level);
						(void)cam_ife_capture(level, &safe);
					}
					if (safe) {
						stream_serve(true);
					}
				}
			}
		}
	} else {
		cam_log("cam_probe: level 1 FAIL (%s, restore %s)\n", ok ? "cleanup" : step, restored ? "OK" : "FAILED");
	}
	thread_terminate(current_thread());
}
void
hakios_cam_probe_init(void)
{
	uint32_t level = 0, cam = 0, iris = 0, service = 0, icp = 0;
	PE_parse_boot_argn("cam_probe", &level, sizeof(level));
	PE_parse_boot_argn("cam", &cam, sizeof(cam));
	PE_parse_boot_argn("icp_probe", &icp, sizeof(icp));
	if (icp) {
		if (cam || level) {
			loader_log_line("icp_probe: requires cam=0 and cam_probe=0\n");
		} else {
			hakios_icp_probe_init(icp);
		}
		return;
	}
	PE_parse_boot_argn("iris_probe", &iris, sizeof(iris));
	PE_parse_boot_argn("iris", &service, sizeof(service));
	if (iris && iris != 7 && !cam && !level) {
		hakios_iris_probe_init(iris);
		return;
	}
	if ((iris == 7 || (!iris && service)) && cam == 5 && !level) {
		hakios_iris_probe_init(7);
	} else if (iris == 7) {
		loader_log_line("iris_probe: level 7 requires cam=5 and cam_probe=0\n");
	} else if (iris) {
		loader_log_line("iris_probe: requires cam=0 and cam_probe=0\n");
	}
	/* icp=1 (CONFIG_ICP): Camera.app's stills through the ISP's BPS and IPE, /dev/hakios-icp */
	uint32_t icp_service = 0;
	PE_parse_boot_argn("icp", &icp_service, sizeof(icp_service));
	if (icp_service && cam == 5 && !level && hakios_cam_rails_ready()) {
		hakios_icp_service_init();
	}
	if (level == 0 && cam == 0) {
		return;
	}
	if (level > 7) {
		cam_log("cam_probe: unsupported level %u (available: 1 chip ID, 2 test pattern, 3 IMX789 frames, "
		    "4 /dev/hakios-camera, 5 IFE0 preview, 6 IFE0 native-size NV12, 7 IMX789 mode 0 RAW10)\n", level);
		return;
	}
	if (level == 0 && cam != 5) {
		cam_log("cam_probe: cam=%u ignored (use cam=5 for /dev/hakios-camera, or cam_probe=1..6 for diagnostics)\n", cam);
		return;
	}
	if (!hakios_cam_rails_ready()) {
		cam_log("cam_probe: %s FAIL (SM8450 RPMh controller unavailable)\n", level ? "level 1" : "cam=5");
		return;
	}
#if CAM_HAVE_IFE_INIT && defined(CAM_IFE_CDM_BYTES)
	uint32_t preview = 0;
	PE_parse_boot_argn("cam_ife_preview", &preview, sizeof(preview));
	stream_ife = level == 0 && (preview == 1 || preview == 2);
	if (stream_ife) {
		stream_full = ife_full_layout(preview == 2);
	}
#endif
	if (level == 0 || (level >= 4 && level <= 6)) {
		if (!stream_make_nodes()) {
			cam_log("cam_probe: %s FAIL (cannot make /dev/hakios-camera)\n", level ? "level 4" : "cam=5");
			return;
		}
		strlcpy(cam_probe_cameras, "1:imx789", sizeof(cam_probe_cameras));
	}
	if (level == 0) {
		cam_log("cam_probe: cam=5 scheduled\n");
	} else {
		cam_log("cam_probe: level %u scheduled\n", level);
	}
	thread_t thread;
	if (kernel_thread_start(cam_probe_thread, (void *)(uintptr_t)level, &thread) != KERN_SUCCESS) {
		cam_log("cam_probe: %s FAIL (cannot start thread)\n", level ? "level 1" : "cam=5");
		return;
	}
	thread_deallocate(thread);
}
