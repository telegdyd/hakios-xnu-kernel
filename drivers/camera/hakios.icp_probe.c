/*
 * NE2213 camera ICP (LX7) RAM-boot diagnostic, CAMERA-PLAN section 5 step 2.
 * icp_probe=1 powers the ICP, BPS and IPE (camera RPMh vote, Titan top, PLL0, their GDSCs and clocks), reads
 * the LX7 control registers and restores everything.
 * icp_probe=2 also authenticates the stock CAMERA_ICP.mbn through PAS 33 into camera_region, sets up the HFI
 * queues at the stock IOVAs, and exchanges SYS_INIT and SYS_PING. It ends with stock's power-collapse
 * order (SYS_PC_PREP, TrustZone suspend) and PAS shutdown.
 * icp_probe=3 starts and stops the firmware with no HFI; icp_probe=4 runs timed firmware windows (run_windows).
 * All run with cam=0. The firmware is the phone's own /vendor/firmware/CAMERA_ICP.mbn, read from fw/ of the
 * boot volume (userland/build.sh copies it from fw-icp/).
 * Sources: OnePlus SM8450 camera-kernel cam_icp (lx7_core.c, hfi.c, cam_icp_hw_mgr.c, hfi_reg.h,
 * hfi_sys_defs.h), the NE2213 live device tree (qcom,lx7, msm_cam_smmu_icp, camera_region) and Linux
 * camcc-sm8450.c for the clock and GDSC registers.
 */
#include <arm/caches_internal.h>
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/hakios.mmio.h>
#include <sys/hakios.smmu.h>
#include <sys/param.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <miscfs/devfs/devfs.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <vm/pmap.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map_unmappable(vm_offset_t, vm_size_t, unsigned);
extern void ml_io_unmap(vm_offset_t, vm_size_t);
extern uint64_t kvtophys(vm_offset_t);
extern void loader_log_line(const char *);
extern unsigned long gPhysBase, gPhysSize;
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

/*
 * Undo ml_io_map_unmappable for any VA it returned. io_map maps whole 16 KiB VM pages and returns the VA plus the
 * address's offset in its page, while PAGE_MASK in this file is the 4 KiB hardware page: unmapping that VA directly
 * (the CPAS, LX7 and GCC maps) panics in pmap_remove with "PTE write is unaligned".
 */
static void
io_unmap(vm_offset_t va, vm_size_t size)
{
	const vm_offset_t vm_page = PAGE_MAX_SIZE;
	vm_offset_t base = va & ~(vm_page - 1);
	ml_io_unmap(base, (va - base + size + vm_page - 1) & ~(vm_page - 1));
}

#define ICP_FIRMWARE        "CAMERA_ICP.mbn"

#define GCC_PA              0x00136000UL
#define CAMCC_PA            0x0ade0000UL
#define CAMCC_SIZE          0x20000UL
#define CPAS_TOP_PA         0x0ac13000UL
#define LX7_PA              0x0ac01000UL /* lx7_csr; lx7_cirq at +0x800 */
/* lx7_wd0 (0xac04000) is not touched: any access, with the firmware running or just shut down, reset the SoC. */
#define FW_PA               0x9f500000UL /* camera_region */
#define FW_REGION           0x800000UL

/* Camera clock controller (camcc-sm8450.c). */
#define GDSC_CLK            0x1320c
#define TITAN_GDSCR         0x131dc
#define BPS_GDSCR           0x10004
#define IPE_GDSCR           0x10078
#define CLK_ENABLE          1u
#define CLK_HWCG            (1u << 1)
#define CLK_OFF             (1u << 31)
#define RCG_UPDATE          1u
#define RCG_ROOT_ENABLE     (1u << 1)
#define RCG_RATE_MASK       (0x1fu | (7u << 8) | (3u << 12) | RCG_HW_CTRL)
#define RCG_HW_CTRL         (1u << 20) /* stock sets it on every camera RCG (CFG_HW_CLK_CTRL) */
#define GDSC_COLLAPSE       1u
#define GDSC_HW_CONTROL     (1u << 1)
#define GDSC_RETAIN         (1u << 11)
#define GDSC_POWERED        (1u << 31)
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
#define PLL_VOTE_RUN        (1u << 25)

/* LX7 CSR and CIRQ (lx7_reg.h); the HFI registers are general-purpose CSR registers from +0x20 (hfi_reg.h). */
#define LX7_GP(n)           (0x20u + 4u * (n))
#define HFI_FW_VERSION      LX7_GP(1)
#define HFI_INIT_REQUEST    LX7_GP(2)
#define HFI_INIT_RESPONSE   LX7_GP(3)
#define HFI_SHARED_PTR      LX7_GP(4)
#define HFI_SHARED_SIZE     LX7_GP(5)
#define HFI_QTBL_PTR        LX7_GP(6)
#define HFI_HEAP_PTR        LX7_GP(7)
#define HFI_HEAP_SIZE       LX7_GP(8)
#define HFI_SFR_PTR         LX7_GP(10)
#define HFI_QDSS_IOVA       LX7_GP(11)
#define HFI_QDSS_SIZE       LX7_GP(12)
#define HFI_IO_IOVA         LX7_GP(13)
#define HFI_IO_SIZE         LX7_GP(14)
#define HFI_IO2_IOVA        LX7_GP(15)
#define HFI_IO2_SIZE        LX7_GP(16)
#define HFI_UNCACHED_IOVA   LX7_GP(17)
#define HFI_UNCACHED_SIZE   LX7_GP(18)
#define LX7_DBG_STATUS      0xc0
#define LX7_DBG_CTRL        0xc4
#define CIRQ                0x800
#define CIRQ_OB_MASK        (CIRQ + 0x0)
#define CIRQ_OB_STATUS      (CIRQ + 0xc)
#define CIRQ_IB_STATUS0     (CIRQ + 0x70)
#define CIRQ_IB_STATUS1     (CIRQ + 0x74)
#define CIRQ_HOST2ICP       (CIRQ + 0x124)
#define CIRQ_PFAULT         (CIRQ + 0x128)

/*
 * The ICP context (msm_cam_smmu_icp): its first iommus group, SID 0x2000 with the DT's mask 0x420 (0x2000, 0x2020,
 * 0x2400, 0x2420). The hypervisor did not take a wider mask covering the BPS and IPE groups (readback EIO). The HFI
 * memory is the stock fw_uncached region at its stock IOVA, laid out as cam_icp_allocate_hfi_mem does: secondary
 * heap, queue table, command, message and debug queues 1 MiB apart, the SFR buffer, and the rest of the 7 MiB for
 * the firmware's own uncached heap. IO regions as stock reports them (around the discarded 0xe0000000 window).
 */
#define ICP_STREAM_GROUPS   4u
static const struct { unsigned sid, mask; } icp_streams[ICP_STREAM_GROUPS] = {
	{ 0x2000, 0x420 }, { 0x2040, 0x420 }, { 0x2100, 0x420 }, { 0x2080, 0x400 },
};
#define HFI_IOVA            0x10400000u
/* Stock's QDSS region (msm_cam_smmu_icp iova-mem-qdss-region): physical 0x16790000, 1 MiB at IOVA 0x10b00000, right
 * after the HFI memory; GP11/GP12 point at it. Without it the firmware logs "QDSS mapping failed 2" and takes an
 * exception after the ping. */
#define QDSS_IOVA           0x10b00000u
#define QDSS_PA             0x16790000u
#define QDSS_SIZE           0x100000u
#define HFI_SIZE            0x700000u
#define HFI_HEAP            0x000000u
#define HFI_QTBL            0x100000u
#define HFI_CMDQ            0x200000u
#define HFI_MSGQ            0x300000u
#define HFI_DBGQ            0x400000u
#define HFI_SFR             0x500000u
#define SHARED_IOVA         0x00800000u
#define SHARED_SIZE         0x0fc00000u
#define IO_IOVA             0x10c00000u
#define IO_SIZE             (0xe0000000u - IO_IOVA)
#define IO2_IOVA            0xe0800000u
#define IO2_SIZE            (IO_IOVA + 0xee300000u - IO2_IOVA)
#define CMDQ_WORDS          (4096u / 4u)
#define MSGQ_WORDS          (4096u / 4u)
#define DBGQ_WORDS          (102400u / 4u)
#define SFR_BYTES           4096u

/* struct hfi_q_hdr: each field follows 15 padding words, so field k is word 16 k + 15. */
#define QHDR_WORDS          239u
#define QF(k)               (16u * (k) + 15u)
enum { Q_STATUS, Q_START, Q_TYPE, Q_SIZE, Q_PKT_SIZE, Q_DROPPED, Q_RX_WM, Q_TX_WM, Q_RX_REQ, Q_TX_REQ,
	Q_RX_IRQ, Q_TX_IRQ, Q_READ, Q_WRITE };
#define QTBL_HEADER_WORDS   6u
#define QTBL_WORDS          (QTBL_HEADER_WORDS + 4u * QHDR_WORDS)

#define HFI_CMD_SYS_INIT    0x10001u
#define HFI_CMD_SYS_PC_PREP 0x10002u
#define HFI_CMD_SYS_PING    0x10005u
#define HFI_MSG_INIT_DONE   0x20001u
#define HFI_MSG_PC_PREP_DONE 0x20002u
#define HFI_MSG_DEBUG       0x20003u
#define HFI_MSG_PING_ACK    0x20006u
#define HFI_MSG_EVENT       0x20008u
#define PING_COOKIE         UINT64_C(0x1c9b0ad15e5e0002)

#define PAS_ID              33u
#define PAS_INIT_IMAGE      0x0201
#define PAS_MEM_SETUP       0x0202
#define PAS_AUTH_AND_RESET  0x0205
#define PAS_SHUTDOWN        0x0206
#define SCM_REMOTE_STATE    0x010a

#define ELF_PT_LOAD         1u
#define MBN_MAX_SEGMENTS    32u
#define MDT_TYPE_MASK       (7u << 24)
#define MDT_TYPE_HASH       (2u << 24)
#define MDT_RELOCATABLE     (1u << 27)

static const uint32_t gcc_gates[] = { 0x004, 0x020, 0x010, 0x018 };
#define GCC_GATES_NO_WAIT   0xcu
#define GCC_ALWAYS_ON       2u /* camera AHB (0x004) and XO (0x020): enabled if needed, never switched off */
/* CPAS AHB, CPAS fast AHB, CAMNOC AXI, QDSS debug XO, ICP AHB, ICP (lx7 clocks and CPAS's, camcc-sm8450.c). QDSS
 * debug XO is the one CamCC branch on in stock at auth and off here (all 81 compared, 2026-11-24): without it Titan's
 * register bus stopped answering 1-1.5 ms after the firmware started. */
static const uint32_t icp_gates[] = { 0x13164, 0x1316c, 0x131ac, 0x131d8, 0x13128, 0x13120 };
/* CPAS core AHB (the DT's cpas_core_ahb_clk): BRANCH_HALT_DELAY, so it reports no status; its enable is written
 * and not checked. Without it the firmware hung the phone right after the HFI init request. */
#define CPAS_CORE_AHB       0x131f0u
/* BPS AHB, fast AHB, core, CPAS BPS; then IPE NPS AHB, NPS and PPS fast AHB, NPS, PPS, CPAS IPE (the DT's). */
static const uint32_t bps_gates[] = { 0x1004c, 0x10030, 0x10068, 0x10070 };
static const uint32_t ipe_gates[] = { 0x100c0, 0x100c4, 0x100c8, 0x100a4, 0x100b0, 0x100ac };
/* Stock's sources while the ICP runs (ifesnap icpregs, RCG cfg = (2 * divider - 1) | source << 8): slow AHB 80 MHz
 * (PLL0 even / 7.5), fast AHB 100 MHz (PLL0 even / 6), CAMNOC AXI 300 MHz (PLL0 even / 2), ICP 480 MHz (PLL8 even).
 * BPS and IPE as stock's boot trace has them while the firmware starts (both on until after SYS_INIT): BPS 400 MHz (PLL0
 * odd), IPE NPS 500 MHz (PLL1 even at 1000 MHz, as stock). Before, IPE ran at TCXO and
 * CSR access 2 ms or more after AUTH_AND_RESET reset the SoC. */
static const struct { uint32_t cmd, rate; } sources[] = {
	{ 0x10034, 0x20e | RCG_HW_CTRL }, { 0x10018, 0x20b | RCG_HW_CTRL }, { 0x13194, 0x203 | RCG_HW_CTRL },
	{ 0x13108, 0x501 | RCG_HW_CTRL }, { 0x10050, 0x301 | RCG_HW_CTRL }, { 0x1008c, 0x401 | RCG_HW_CTRL },
};

/* The ELF, checked once: where the PAS metadata and each load segment come from (CAMERA_ICP.mbn, as mdt_loader). */
static struct {
	const uint8_t *file;
	uint32_t header_size, hash_offset, hash_size, span, loads;
	struct {
		uint32_t offset, destination, size;
	} load[MBN_MAX_SEGMENTS];
	bool reported;
} fw;

struct icp_change {
	vm_offset_t reg;
	uint32_t old, mask, status;
	bool rcg;
};

struct icp_probe {
	vm_offset_t gcc, cc, lx7, backing, hfi, metadata;
	struct icp_change changes[64];
	unsigned count;
	struct hakios_cam_power power;
	struct hakios_smmu_context *smmu[ICP_STREAM_GROUPS], *cdm[2];
	bool group_mapped[ICP_STREAM_GROUPS], qdss_mapped[ICP_STREAM_GROUPS];
	uint32_t slow_saved;
	bool pas_initialized, auth_attempted, running;
};

SYSCTL_DECL(_kern_hakios);
static int icp_result;
static vm_offset_t hfi_reserved; /* the HFI memory, allocated when the probe thread starts (setup_queues takes it) */
static uint32_t icp_claim_base, icp_claim_size; /* the ICP SMMU contexts' IOVA range (level 5 widens it) */
static uint32_t icp_slow_ms; /* icp_slow=MS: pause after each line so the live USB log shows the last step before a reset */
/* icp=1 with cam=5 (hakios_icp_service_init): Camera.app's stills, beside a live camera session. It leaves the CDM
 * streams and any clock source the camera already runs alone. */
static bool icp_service;
SYSCTL_INT(_kern_hakios, OID_AUTO, icp_probe, CTLFLAG_RD | CTLFLAG_LOCKED,
    &icp_result, 0, "camera ICP diagnostic: 0 pending, 1 pass, -1 failed, -2 teardown failed");

static void icp_log(const char *, ...) __printflike(1, 2);

static void
icp_log(const char *format, ...)
{
	char line[256];
	va_list args;
	va_start(args, format);
	vsnprintf(line, sizeof(line), format, args);
	va_end(args);
	loader_log_line(line);
	if (icp_slow_ms) {
		delay(icp_slow_ms * 1000);
	}
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

/* An RCG update needs the root enabled while it latches (cam_probe's source_set). */
static bool
rcg_latch(vm_offset_t cmd)
{
	uint32_t initial;
	if (!read_reg(cmd, &initial) || !write_reg(cmd, initial | RCG_ROOT_ENABLE) ||
	    !write_reg(cmd, initial | RCG_ROOT_ENABLE | RCG_UPDATE)) {
		return false;
	}
	bool ok = poll_reg(cmd, RCG_UPDATE, 0, 5000);
	uint32_t now;
	return read_reg(cmd, &now) &&
	    write_reg(cmd, (now & ~(RCG_ROOT_ENABLE | RCG_UPDATE)) | (initial & RCG_ROOT_ENABLE)) && ok;
}

/* Change the masked bits of a register, remembering the old ones; for an RCG's CFG register, latch it. */
static bool
change_reg(struct icp_probe *p, vm_offset_t reg, uint32_t mask, uint32_t want,
    uint32_t status, uint32_t ready, bool rcg)
{
	uint32_t old;
	if (p->count == sizeof(p->changes) / sizeof(p->changes[0]) || !read_reg(reg, &old)) {
		return false;
	}
	if ((old & mask) != want) {
		p->changes[p->count++] = (struct icp_change){ reg, old, mask, status, rcg };
		if (!write_reg(reg, (old & ~mask) | want) || (rcg && !rcg_latch(reg - 4))) {
			return false;
		}
	}
	return poll_reg(reg, mask | status, want | ready, 4000);
}

static bool
restore_changes(struct icp_probe *p)
{
	bool ok = true;
	while (p->count > 0) {
		const struct icp_change *change = &p->changes[--p->count];
		uint32_t value;
		/* (no line per restore: with the slow-trace pause ~70 of them ran into the payload's return timer) */
		bool restored = read_reg(change->reg, &value) &&
		    write_reg(change->reg, (value & ~change->mask) | (change->old & change->mask));
		if (restored && change->rcg) {
			restored = rcg_latch(change->reg - 4);
		}
		if (restored) {
			uint32_t bits = change->mask | change->status;
			restored = poll_reg(change->reg, bits, change->old & bits, 4000);
		}
		if (!restored) {
			icp_log("icp_probe: restore failed for MMIO %#lx mask %#x\n", (unsigned long)change->reg, change->mask);
			ok = false;
		}
	}
	return ok;
}

static bool
gate_on(struct icp_probe *p, vm_offset_t base, uint32_t offset, bool wait)
{
	uint32_t value = UINT32_MAX;
	bool ok = read_reg(base + offset, &value);
	/* A hardware-gated branch may read CLK_OFF while idle. */
	bool check = ok && wait && !(value & CLK_HWCG);
	ok = ok && change_reg(p, base + offset, CLK_ENABLE, CLK_ENABLE, check ? CLK_OFF : 0, 0, false);
	if (!ok) {
		(void)read_reg(base + offset, &value);
		icp_log("icp_probe: clock %s+%#x did not start (%#x)\n", base == p->cc ? "CamCC" : "GCC", offset, value);
	}
	return ok;
}

static bool
gates_on(struct icp_probe *p, const uint32_t *offsets, unsigned count)
{
	for (unsigned i = 0; i < count; ++i) {
		if (!gate_on(p, p->cc, offsets[i], true)) {
			return false;
		}
	}
	return true;
}

static bool
domain_on(struct icp_probe *p, uint32_t offset, const char *name)
{
	uint32_t old = UINT32_MAX;
	if (!read_reg(p->cc + offset, &old) || (old & GDSC_HW_CONTROL)) {
		icp_log("icp_probe: cannot acquire %s GDSC (%#x)\n", name, old);
		return false;
	}
	bool ok = change_reg(p, p->cc + offset, GDSC_COLLAPSE | GDSC_RETAIN, GDSC_RETAIN, GDSC_POWERED, GDSC_POWERED, false);
	uint32_t now = UINT32_MAX;
	(void)read_reg(p->cc + offset, &now);
	icp_log("icp_probe: %s GDSC %#x -> %#x%s\n", name, old, now, ok ? "" : " failed");
	return ok;
}

/*
 * Lucid EVO PLLs as camcc-sm8450.c configures them, at the rates stock runs while the ICP is up (ifesnap icpregs):
 * PLL0 1200 MHz (L 0x3e, alpha 0x8000; even 600, odd 400) and PLL8 960 MHz (L 0x32; even 480 for the ICP).
 */
struct cam_pll {
	const char *name;
	uint32_t offset, l, alpha, user;
};
static const struct cam_pll pll0 = { "PLL0", 0x0000, 0x3e, 0x8000, 0x8400 };
static const struct cam_pll pll8 = { "PLL8", 0x8000, 0x32, 0x0000, 0x0400 };
static const struct cam_pll pll1 = { "PLL1", 0x1000, 0x34, 0x1555, 0x0400 }; /* stock: 1000 MHz, even 500 MHz for IPE */

static bool
pll_on(struct icp_probe *p, const struct cam_pll *pll)
{
	vm_offset_t b = p->cc + pll->offset;
	uint32_t mode, user, l, alpha;
	if (!read_reg(b + PLL_MODE, &mode) || !read_reg(b + PLL_USER, &user) || (user & PLL_VOTE_RUN)) {
		return false;
	}
	if ((mode & PLL_OUTPUT) && (mode & PLL_LOCK)) {
		/* Someone else runs it: use it only at our rate. */
		bool same = read_reg(b + PLL_L, &l) && hakios_mmio_try_read32((const volatile void *)(b + PLL_ALPHA), &alpha) == 0 &&
		    (l & 0xffffu) == pll->l && alpha == pll->alpha && (user & 0xff00u) == pll->user;
		icp_log("icp_probe: %s already running%s\n", pll->name, same ? " at our rate" : " at another rate");
		return same;
	}
	const struct { unsigned reg; uint32_t value; } config[] = {
		{ PLL_L, 0x00440000u | pll->l }, { PLL_ALPHA, pll->alpha }, { PLL_CONFIG, 0x20485699 },
		{ PLL_CONFIG_HI, 0x00182261 }, { PLL_CONFIG_HI1, 0x32aa299c },
		{ PLL_USER, pll->user }, { PLL_USER_HI, 0x805 },
	};
	for (unsigned i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
		if (!change_reg(p, b + config[i].reg, UINT32_MAX, config[i].value, 0, 0, false)) {
			return false;
		}
	}
	if (!change_reg(p, b + PLL_OPMODE, 1, 0, 0, 0, false) ||
	    !change_reg(p, b + PLL_MODE, PLL_RESET_N, PLL_RESET_N, 0, 0, false) ||
	    !change_reg(p, b + PLL_OPMODE, 1, 1, 0, 0, false) ||
	    !poll_reg(b + PLL_MODE, PLL_LOCK, PLL_LOCK, 2000) ||
	    !change_reg(p, b + PLL_USER, 7, 7, 0, 0, false) ||
	    !change_reg(p, b + PLL_MODE, PLL_OUTPUT, PLL_OUTPUT, 0, 0, false)) {
		(void)read_reg(b + PLL_MODE, &mode);
		icp_log("icp_probe: %s did not lock (mode %#x)\n", pll->name, mode);
		return false;
	}
	icp_log("icp_probe: %s locked (L %#x)\n", pll->name, pll->l);
	return true;
}

static bool
power_up(struct icp_probe *p, const char **step)
{
	*step = "GCC camera clocks";
	for (unsigned i = 0; i < sizeof(gcc_gates) / sizeof(gcc_gates[0]); ++i) {
		if (i < GCC_ALWAYS_ON) {
			/* Linux gcc-sm8450 keeps camera AHB and XO on for good; switching AHB off again hung the phone. */
			uint32_t value;
			if (!read_reg(p->gcc + gcc_gates[i], &value) ||
			    (!(value & CLK_ENABLE) && !write_reg(p->gcc + gcc_gates[i], value | CLK_ENABLE)) ||
			    !poll_reg(p->gcc + gcc_gates[i], CLK_ENABLE, CLK_ENABLE, 4000)) {
				icp_log("icp_probe: GCC camera clock %#x not on\n", gcc_gates[i]);
				return false;
			}
		} else if (!gate_on(p, p->gcc, gcc_gates[i], !(GCC_GATES_NO_WAIT & (1u << i)))) {
			return false;
		}
	}
	*step = "Titan top GDSC";
	if (!gate_on(p, p->cc, GDSC_CLK, false) || !domain_on(p, TITAN_GDSCR, "Titan top")) {
		return false;
	}
	*step = "CamCC PLL0, PLL8 and PLL1";
	if (!pll_on(p, &pll0) || !pll_on(p, &pll8) || !pll_on(p, &pll1)) {
		return false;
	}
	*step = "ICP, BPS and IPE clock sources";
	for (unsigned i = 0; i < sizeof(sources) / sizeof(sources[0]); ++i) {
		uint32_t cfg = 0;
		if (icp_service && read_reg(p->cc + sources[i].cmd + 4, &cfg) && (cfg & (7u << 8))) {
			continue; /* the live camera runs this one (CAMNOC AXI at 400 MHz, slow AHB): not ours to change */
		}
		if (!change_reg(p, p->cc + sources[i].cmd + 4, RCG_RATE_MASK, sources[i].rate, 0, 0, true)) {
			icp_log("icp_probe: clock source %#x did not update\n", sources[i].cmd);
			return false;
		}
	}
	*step = "ICP and CPAS clocks";
	if (!gates_on(p, icp_gates, sizeof(icp_gates) / sizeof(icp_gates[0]))) {
		return false;
	}
	uint32_t core = 0, after = 0;
	if (hakios_mmio_try_read32((const volatile void *)(p->cc + CPAS_CORE_AHB), &core) != 0) {
		return false;
	}
	if (!(core & CLK_ENABLE)) {
		if (p->count == sizeof(p->changes) / sizeof(p->changes[0])) {
			return false;
		}
		p->changes[p->count++] = (struct icp_change){ p->cc + CPAS_CORE_AHB, core, CLK_ENABLE, 0, false };
		if (!write_reg(p->cc + CPAS_CORE_AHB, core | CLK_ENABLE)) {
			return false;
		}
		delay(10);
	}
	(void)hakios_mmio_try_read32((const volatile void *)(p->cc + CPAS_CORE_AHB), &after);
	icp_log("icp_probe: CPAS core AHB %#x -> %#x\n", core, after);
	/* Stock powers BPS and IPE before the firmware boots (cam_icp_mgr_device_init); icp_no_ipe_bps=1 leaves them off. */
	uint32_t no_ipe_bps = 0;
	PE_parse_boot_argn("icp_no_ipe_bps", &no_ipe_bps, sizeof(no_ipe_bps));
	if (no_ipe_bps) {
		icp_log("icp_probe: BPS and IPE left off\n");
		return true;
	}
	*step = "BPS power and clocks";
	if (!domain_on(p, BPS_GDSCR, "BPS") || !gates_on(p, bps_gates, sizeof(bps_gates) / sizeof(bps_gates[0]))) {
		return false;
	}
	*step = "IPE power and clocks";
	if (!domain_on(p, IPE_GDSCR, "IPE") || !gates_on(p, ipe_gates, sizeof(ipe_gates) / sizeof(ipe_gates[0]))) {
		return false;
	}
	return true;
}

static void
log_lx7(struct icp_probe *p, const char *when)
{
	static const struct { unsigned offset; const char *name; } regs[] = {
		{ HFI_FW_VERSION, "fw version" }, { HFI_INIT_REQUEST, "init request" },
		{ HFI_INIT_RESPONSE, "init response" }, { LX7_DBG_STATUS, "debug status" },
		{ LX7_DBG_CTRL, "debug control" },
	};
	char line[200];
	unsigned at = (unsigned)snprintf(line, sizeof(line), "icp_probe: LX7 %s:", when);
	for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]) && at < sizeof(line); ++i) {
		uint32_t value = 0;
		bool ok = hakios_mmio_try_read32((const volatile void *)(p->lx7 + regs[i].offset), &value) == 0;
		at += (unsigned)snprintf(line + at, sizeof(line) - at, " %s %s%#x", regs[i].name, ok ? "" : "?", value);
	}
	icp_log("%s\n", line);
}

static void
hfi_sync(const volatile void *addr, unsigned size)
{
	flush_dcache64((addr64_t)addr, size, FALSE);
	__asm__ volatile ("dsb sy" ::: "memory");
}

static bool
scm(const char *name, uint32_t op, uint32_t arginfo, uint64_t a, uint64_t b, uint64_t c)
{
	uint64_t token = 0;
	for (unsigned i = 0; i < 100; ++i) {
		register uint64_t x0 __asm__("x0") = i ? 1 : 0x42000000u | op;
		register uint64_t x1 __asm__("x1") = arginfo;
		register uint64_t x2 __asm__("x2") = a;
		register uint64_t x3 __asm__("x3") = b;
		register uint64_t x4 __asm__("x4") = c;
		register uint64_t x5 __asm__("x5") = 0;
		register uint64_t x6 __asm__("x6") = token;
		register uint64_t x7 __asm__("x7") = 0;
		__asm__ volatile ("smc #0" : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3), "+r"(x4),
		    "+r"(x5), "+r"(x6), "+r"(x7) :
		    : "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
		token = x6;
		if (x0 != 1) {
			icp_log("icp_probe: %s SCM %d, result %#llx\n", name, (int32_t)x0, x1);
			return x0 == 0 && x1 == 0;
		}
	}
	icp_log("icp_probe: %s SCM interrupted repeatedly\n", name);
	return false;
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

/*
 * The MBN as qcom_mdt_load reads it: the ELF and program headers plus the hash segment are the PAS metadata;
 * each load segment lands at its address less the lowest one, and its memory beyond the file bytes is zero. The
 * segments are copied in header order, as mdt_loader does: CAMERA_ICP's segment 17 lies inside segment 12's zeroed
 * memory and must follow it.
 */
static bool
parse_firmware(void)
{
	if (fw.loads) {
		return true;
	}
	uint64_t size = 0;
	const uint8_t *file = hakios_image_file("fw", ICP_FIRMWARE, &size, NULL);
	const char *problem = NULL;
	uint32_t phoff = 0, phnum = 0, header_size = 0, hash_offset = 0, hash_size = 0, hashes = 0, loads = 0;
	uint32_t low = UINT32_MAX, high = 0;
	bool relocatable = false;
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
		} else {
			header_size = read32(file + phoff + 16);
			if (read32(file + phoff) == ELF_PT_LOAD || header_size < phoff + 32u * phnum || header_size > size) {
				problem = "invalid PAS metadata headers";
			}
		}
	}
	for (uint32_t i = 1; !problem && i < phnum; i++) {
		const uint8_t *ph = file + phoff + 32u * i;
		uint32_t type = read32(ph), flags = read32(ph + 24);
		uint32_t address = read32(ph + 12), file_size = read32(ph + 16), memory_size = read32(ph + 20);
		if ((flags & MDT_TYPE_MASK) == MDT_TYPE_HASH) {
			hash_offset = read32(ph + 4);
			hash_size = file_size;
			hashes++;
		} else if (type == ELF_PT_LOAD && memory_size) {
			if (file_size > memory_size || (file_size && (uint64_t)read32(ph + 4) + file_size > size) ||
			    (uint64_t)address + memory_size > UINT32_MAX) {
				problem = "invalid load segment";
				break;
			}
			low = MIN(low, address);
			high = MAX(high, address + memory_size);
			relocatable = relocatable || (flags & MDT_RELOCATABLE);
			fw.load[loads].offset = read32(ph + 4);
			fw.load[loads].destination = address;
			fw.load[loads].size = file_size;
			loads++;
		}
	}
	if (!problem && (hashes != 1 || hash_offset < header_size || !hash_size ||
	    (uint64_t)hash_offset + hash_size > size || header_size + hash_size > PAGE_SIZE)) {
		problem = "invalid PAS hash segment";
	}
	if (!problem && (!loads || !relocatable)) {
		problem = "no relocatable load segments";
	}
	uint32_t span = (high - low + 0xfffu) & ~0xfffu;
	if (!problem && span > FW_REGION) {
		problem = "larger than camera_region";
	}
	if (problem) {
		if (!fw.reported) {
			icp_log("icp_probe: fw/%s %s\n", ICP_FIRMWARE, problem);
		}
		fw.reported = true;
		fw.loads = 0;
		return false;
	}
	for (uint32_t i = 0; i < loads; i++) {
		fw.load[i].destination -= low;
	}
	fw.file = file;
	fw.header_size = header_size;
	fw.hash_offset = hash_offset;
	fw.hash_size = hash_size;
	fw.span = span;
	fw.loads = loads;
	icp_log("icp_probe: fw/%s, %llu bytes: %u load segments over %#x bytes, %#x bytes of PAS metadata\n",
	    ICP_FIRMWARE, size, loads, span, header_size + hash_size);
	return true;
}

/*
 * The state the firmware starts in, taken right before PAS AUTH_AND_RESET in the format of ifesnap's icp_regs_dump
 * (icpregs=3 takes stock's at the same call), so the two diff line by line. Kept in memory, logged after the window.
 */
#define STATE_LINES 96
static char state_text[STATE_LINES][128];
static unsigned state_count;
static bool state_taken;

static void
state_line(const char *format, ...) __printflike(1, 2);

static void
state_line(const char *format, ...)
{
	if (state_count < STATE_LINES) {
		va_list args;
		va_start(args, format);
		vsnprintf(state_text[state_count], sizeof(state_text[0]), format, args);
		va_end(args);
		/* At once, with the slow-trace pause (before auth, so harmless): one of these reads hung the CPU in the
		 * first run, and the last line that reaches the PC names it. */
		icp_log("icp_probe: at auth %s\n", state_text[state_count]);
		state_count++;
	}
}

static uint32_t
rd(vm_offset_t base, uint32_t offset)
{
	uint32_t value = 0xdeadbeef;
	(void)hakios_mmio_try_read32((const volatile void *)(base + offset), &value);
	return value;
}

static void
state_take(struct icp_probe *p)
{
	static const uint32_t rcg[] = { 0x10034, 0x10018, 0x13194, 0x13108, 0x10050, 0x1008c };
	static const uint32_t gate[] = { 0x13164, 0x131f0, 0x13160, 0x1316c, 0x131ac, 0x13128, 0x13120, 0x1004c, 0x10030,
		0x10068, 0x10070, 0x100c0, 0x100c4, 0x100c8, 0x100a4, 0x100b0, 0x100ac, 0x1320c, 0x131dc, 0x10004,
		0x10078 };
	static const uint32_t allcc[] = {
		0x10004, 0x10078, 0x100d0, 0x11004, 0x12004, 0x12050, 0x13050, 0x13098, 0x131dc, 0x10030, 0x1004c, 0x10068,
		0x10070, 0x100a4, 0x100ac, 0x100b0, 0x100c0, 0x100c4, 0x100c8, 0x100e4, 0x100ec, 0x100f0, 0x11030, 0x11038,
		0x1103c, 0x11048, 0x12030, 0x12038, 0x1203c, 0x12048, 0x1207c, 0x12084, 0x12088, 0x12094, 0x13018, 0x13020,
		0x1303c, 0x13044, 0x13048, 0x1307c, 0x13084, 0x13090, 0x130c4, 0x130cc, 0x130d8, 0x130f4, 0x13120, 0x13128,
		0x13144, 0x13160, 0x13164, 0x1316c, 0x1318c, 0x131ac, 0x131b4, 0x131d4, 0x131d8, 0x131f0, 0x1320c, 0x13228,
		0x15018, 0x15034, 0x15050, 0x1506c, 0x15088, 0x150a4, 0x150c0, 0x150dc, 0x150f8, 0x150fc, 0x15100, 0x1511c,
		0x15120, 0x1513c, 0x15140, 0x15164, 0x15168, 0x15184, 0x15188, 0x151a4, 0x151a8
	};
	if (state_taken) {
		return;
	}
	state_taken = true;
	vm_offset_t cpas_page = ml_io_map_unmappable(trunc_page(CPAS_TOP_PA), PAGE_SIZE, VM_WIMG_IO);
	vm_offset_t cpas = cpas_page ? cpas_page + (CPAS_TOP_PA & PAGE_MASK) : 0;
	for (unsigned i = 0; i <= 8; i++) {
		vm_offset_t b = p->cc + i * 0x1000;
		state_line("icpregs pll%u mode %08x opmode %08x L %08x alpha %08x user %08x user_hi %08x", i, rd(b, 0),
		    rd(b, 4), rd(b, 0x10), rd(b, 0x14), rd(b, 0x18), rd(b, 0x1c));
	}
	for (unsigned i = 0; i < sizeof(rcg) / sizeof(rcg[0]); i++) {
		state_line("icpregs rcg %05x cmd %08x cfg %08x m %08x n %08x d %08x", rcg[i], rd(p->cc, rcg[i]),
		    rd(p->cc, rcg[i] + 4), rd(p->cc, rcg[i] + 8), rd(p->cc, rcg[i] + 0xc), rd(p->cc, rcg[i] + 0x10));
	}
	for (unsigned i = 0; i < sizeof(gate) / sizeof(gate[0]); i++) {
		state_line("icpregs camcc %05x %08x", gate[i], rd(p->cc, gate[i]));
	}
	/* Every CamCC GDSC and branch (camcc-sm8450.c), six to a line; the same list as ifesnap's icp_regs_dump. */
	for (unsigned i = 0; i < sizeof(allcc) / sizeof(allcc[0]); i += 6) {
		char line[128];
		int n = 0;
		for (unsigned j = i; j < i + 6 && j < sizeof(allcc) / sizeof(allcc[0]); j++) {
			n += snprintf(line + n, sizeof(line) - n, " %05x:%08x", allcc[j], rd(p->cc, allcc[j]));
		}
		state_line("icpregs cc%s", line);
	}
	for (uint32_t k = 0; k < 0x100; k += 0x20) {
		state_line("icpregs lx7 %03x %08x %08x %08x %08x %08x %08x %08x %08x", k, rd(p->lx7, k), rd(p->lx7, k + 4),
		    rd(p->lx7, k + 8), rd(p->lx7, k + 12), rd(p->lx7, k + 16), rd(p->lx7, k + 20), rd(p->lx7, k + 24),
		    rd(p->lx7, k + 28));
	}
	for (uint32_t k = 0; cpas && k < 0x200; k += 0x20) {
		state_line("icpregs cpas %03x %08x %08x %08x %08x %08x %08x %08x %08x", k, rd(cpas, k), rd(cpas, k + 4),
		    rd(cpas, k + 8), rd(cpas, k + 12), rd(cpas, k + 16), rd(cpas, k + 20), rd(cpas, k + 24), rd(cpas, k + 28));
	}
	for (uint32_t k = 0; k < 0x40; k += 0x20) {
		state_line("icpregs gcc %03x %08x %08x %08x %08x %08x %08x %08x %08x", k, rd(p->gcc, k), rd(p->gcc, k + 4),
		    rd(p->gcc, k + 8), rd(p->gcc, k + 12), rd(p->gcc, k + 16), rd(p->gcc, k + 20), rd(p->gcc, k + 24),
		    rd(p->gcc, k + 28));
	}
	if (cpas_page) {
		io_unmap(cpas_page, PAGE_SIZE);
	}
}

/* lx7_core.c cam_lx7_boot: mdt load into camera_region, then PAS auth and reset. */
static bool
load_firmware(struct icp_probe *p, const char **step)
{
	*step = "fw/" ICP_FIRMWARE;
	if (!parse_firmware()) {
		return false;
	}
	*step = "reserved firmware memory";
	if (FW_PA < gPhysBase + gPhysSize && FW_PA + FW_REGION > gPhysBase) {
		icp_log("icp_probe: camera_region %#lx overlaps the kernel's RAM\n", FW_PA);
		return false;
	}
	*step = "PAS metadata allocation";
	if (!p->metadata && kmem_alloc(kernel_map, &p->metadata, PAGE_SIZE, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		p->metadata = 0;
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
	if (!scm("PAS INIT_IMAGE", PAS_INIT_IMAGE, 2 | (2 << 6), PAS_ID, pa, 0)) {
		return false;
	}
	p->pas_initialized = true;
	*step = "PAS MEM_SETUP";
	if (!scm("PAS MEM_SETUP", PAS_MEM_SETUP, 3, PAS_ID, FW_PA, fw.span)) {
		return false;
	}
	*step = "copy stock firmware";
	vm_offset_t firmware = ml_io_map_unmappable(FW_PA, FW_REGION, VM_WIMG_WCOMB);
	if (!firmware) {
		return false;
	}
	bzero((void *)firmware, fw.span);
	for (uint32_t i = 0; i < fw.loads; i++) {
		memcpy((void *)(firmware + fw.load[i].destination), fw.file + fw.load[i].offset, fw.load[i].size);
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	io_unmap(firmware, FW_REGION);
	*step = "PAS AUTH_AND_RESET";
	state_take(p);
	p->auth_attempted = true;
	p->slow_saved = icp_slow_ms;
	icp_slow_ms = 0; /* no slow-trace pause until the HFI registers are written */
	if (!scm("PAS AUTH_AND_RESET", PAS_AUTH_AND_RESET, 1, PAS_ID, 0, 0)) {
		return false;
	}
	p->running = true;
	/* Straight on to the HFI registers, as cam_icp_mgr_hw_open does: no pause, no CIRQ reads (a register dump and
	 * the slow-trace pause here were followed by a hang). */
	return true;
}

static volatile uint32_t *
hfi_words(struct icp_probe *p, uint32_t offset)
{
	return (volatile uint32_t *)(p->hfi + offset);
}

static volatile uint32_t *
queue_header(struct icp_probe *p, unsigned queue)
{
	return hfi_words(p, HFI_QTBL) + QTBL_HEADER_WORDS + queue * QHDR_WORDS;
}

static bool queue_table_init(struct icp_probe *p, const char **step);

/* The HFI memory, its SMMU mapping and cam_hfi_init's queue table (event-driven mode 0, as stock passes). */
static bool
setup_queues(struct icp_probe *p, const char **step)
{
	*step = "HFI memory";
	kern_return_t kr = KERN_SUCCESS;
	if (hfi_reserved) {
		/* taken at boot, before the start delay (45 s in, iOS has split memory too finely for 7 MiB) */
		p->backing = hfi_reserved;
		hfi_reserved = 0;
		bzero((void *)p->backing, HFI_SIZE);
	} else {
		kr = kmem_alloc_contig(kernel_map, &p->backing, HFI_SIZE, 0, 0, 0,
		    KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT);
	}
	if (kr != KERN_SUCCESS) {
		p->backing = 0;
		icp_log("icp_probe: %u contiguous bytes for HFI not allocated (%d)\n", HFI_SIZE, kr);
		return false;
	}
	uint64_t pa = kvtophys(p->backing);
	if (!pa) {
		return false;
	}
	/* The firmware sees this memory uncached; all later CPU access uses the uncached alias. */
	hfi_sync((void *)p->backing, HFI_SIZE);
	p->hfi = ml_io_map_unmappable(pa, HFI_SIZE, VM_WIMG_WCOMB);
	if (!p->hfi) {
		return false;
	}
	*step = "ICP SMMU context";
	/* Stock routes every ICP group to one context; the probe gives each group a context with the same mapping. The
	 * service shares one, as stock: beside the live camera, Wi-Fi, audio and Iris2 there are not four to spare. */
	for (unsigned g = 0; g < ICP_STREAM_GROUPS; ++g) {
		if (icp_service && g > 0) {
			if (hakios_smmu_add_stream(p->smmu[0], icp_streams[g].sid, icp_streams[g].mask)) {
				return false;
			}
			continue;
		}
		p->smmu[g] = hakios_smmu_claim(icp_streams[g].sid, icp_streams[g].mask, icp_claim_base, icp_claim_size,
		    "camera ICP");
		if (!p->smmu[g]) {
			return false;
		}
		/* Stock's ICP context (cb49, SCTLR 0x0a5f00e7) is dma-coherent: write-back, shareable transactions. */
		if (hakios_smmu_map(p->smmu[g], HFI_IOVA, pa, HFI_SIZE,
		    HAKIOS_SMMU_READ | HAKIOS_SMMU_WRITE | HAKIOS_SMMU_COHERENT)) {
			return false;
		}
		p->group_mapped[g] = true;
		/* stock: iommu_map(READ | WRITE) on its dma-coherent ICP context */
		if (hakios_smmu_map(p->smmu[g], QDSS_IOVA, QDSS_PA, QDSS_SIZE,
		    HAKIOS_SMMU_READ | HAKIOS_SMMU_WRITE | HAKIOS_SMMU_COHERENT)) {
			icp_log("icp_probe: QDSS region not mapped for SMMU %#x\n", icp_streams[g].sid);
			return false;
		}
		p->qdss_mapped[g] = true;
		/* stock's camera ICP context (cb48 at PAS auth, ifesnap icpregs=3): stall on fault, its memory overrides */
		int attributes = hakios_smmu_context_attributes(p->smmu[g], 0x0a5f00e7, 0x1);
		if (g == 0) {
			icp_log("icp_probe: ICP SMMU contexts as stock (SCTLR 0x0a5f00e7 ACTLR 1): %d\n", attributes);
		}
	}
	/* Stock also has the CPAS and RT CDM streams (msm_cam_smmu_cpas_cdm) attached while the firmware starts. The
	 * service leaves them to the live camera (its IFE programs run through RT CDM0); copying them changed nothing. */
	static const unsigned cdm_sids[2] = { 0x20a0, 0x20c0 };
	for (unsigned c = 0; !icp_service && c < 2; ++c) {
		p->cdm[c] = hakios_smmu_claim(cdm_sids[c], 0x400, 0x100000, 0x100000, "camera CDM");
		icp_log("icp_probe: CDM stream %#x %s\n", cdm_sids[c], p->cdm[c] ? "attached" : "not attached");
	}
	return queue_table_init(p, step);
}

/* cam_hfi_init's queue table in fresh HFI memory (event-driven mode 0, as stock passes). */
static bool
queue_table_init(struct icp_probe *p, const char **step)
{
	*step = "HFI queue table";
	volatile uint32_t *table = hfi_words(p, HFI_QTBL);
	table[0] = 0xffffffffu;
	table[1] = QTBL_WORDS * 4u;
	table[2] = QTBL_HEADER_WORDS * 4u;
	table[3] = QHDR_WORDS * 4u;
	table[4] = 3;
	table[5] = 3;
	static const struct { uint32_t offset, words, rx_wm, tx_wm, rx_req; } queues[3] = {
		{ HFI_CMDQ, CMDQ_WORDS, 1, 1, 1 },
		{ HFI_MSGQ, MSGQ_WORDS, 1, 1, 1 },
		{ HFI_DBGQ, DBGQ_WORDS, 1, 1024, 0 },
	};
	for (unsigned i = 0; i < 3; ++i) {
		volatile uint32_t *q = queue_header(p, i);
		q[QF(Q_STATUS)] = 1;
		q[QF(Q_START)] = HFI_IOVA + queues[i].offset;
		q[QF(Q_TYPE)] = i;
		q[QF(Q_SIZE)] = queues[i].words;
		q[QF(Q_RX_WM)] = queues[i].rx_wm;
		q[QF(Q_TX_WM)] = queues[i].tx_wm;
		q[QF(Q_RX_REQ)] = queues[i].rx_req;
	}
	*hfi_words(p, HFI_SFR) = SFR_BYTES;
	__asm__ volatile ("dsb sy" ::: "memory");
	return true;
}

/* cam_hfi_init: the memory map into the general-purpose registers, then the init request and its response. */
static bool
hfi_init(struct icp_probe *p, const char **step)
{
	*step = "HFI registers";
	static const struct { unsigned reg; uint32_t value; } regs[] = {
		{ HFI_QTBL_PTR, HFI_IOVA + HFI_QTBL }, { HFI_SFR_PTR, HFI_IOVA + HFI_SFR },
		{ HFI_SHARED_PTR, SHARED_IOVA }, { HFI_SHARED_SIZE, SHARED_SIZE },
		{ HFI_HEAP_PTR, HFI_IOVA + HFI_HEAP }, { HFI_HEAP_SIZE, HFI_QTBL - HFI_HEAP },
		{ HFI_QDSS_IOVA, QDSS_IOVA }, { HFI_QDSS_SIZE, QDSS_SIZE },
		{ HFI_IO_IOVA, IO_IOVA }, { HFI_IO_SIZE, IO_SIZE },
		{ HFI_IO2_IOVA, IO2_IOVA }, { HFI_IO2_SIZE, IO2_SIZE },
		{ HFI_UNCACHED_IOVA, HFI_IOVA }, { HFI_UNCACHED_SIZE, HFI_SIZE },
		{ HFI_INIT_REQUEST, 1 },
	};
	for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
		if (!write_reg(p->lx7 + regs[i].reg, regs[i].value)) {
			icp_slow_ms = p->slow_saved;
			icp_log("icp_probe: HFI register %#x not written\n", regs[i].reg);
			return false;
		}
	}
	uint64_t start = mach_absolute_time();
	*step = "HFI init response";
	uint32_t response = 0;
	/*
	 * Poll silently, then log with the slow-trace pause so the line reaches the PC: the SoC died 1.5-3 s after the
	 * firmware started in every run, and a busy poll that logs as it goes never got a line out.
	 */
	uint64_t answered_ns = 0;
	for (unsigned phase = 0; phase < 2 && !response; ++phase) {
		uint64_t limit_ns = phase ? 1000000000ull : 200000000ull;
		for (;;) {
			uint64_t now_ns;
			absolutetime_to_nanoseconds(mach_absolute_time() - start, &now_ns);
			if (hakios_mmio_try_read32((const volatile void *)(p->lx7 + HFI_INIT_RESPONSE), &response) != 0) {
				break;
			}
			if (response) {
				answered_ns = now_ns;
				break;
			}
			if (now_ns >= limit_ns) {
				break;
			}
			delay(50);
		}
		icp_slow_ms = p->slow_saved;
		icp_log("icp_probe: init response %u %s %llu us after the request\n", response,
		    response ? "at" : "still 0", (response ? answered_ns : limit_ns) / 1000);
		icp_slow_ms = 0;
	}
	icp_slow_ms = p->slow_saved;
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
	uint32_t version = 0;
	(void)read_reg(p->lx7 + HFI_FW_VERSION, &version);
	icp_log("icp_probe: HFI init response %u after %llu us, firmware version %#x\n", response, ns / 1000, version);
	return response == 1;
}

static bool
hfi_send(struct icp_probe *p, const uint32_t *packet)
{
	volatile uint32_t *q = queue_header(p, 0);
	volatile uint32_t *ring = hfi_words(p, HFI_CMDQ);
	uint32_t words = packet[0] / 4u, read = q[QF(Q_READ)], write = q[QF(Q_WRITE)];
	uint32_t free = write >= read ? CMDQ_WORDS - (write - read) : read - write;
	if (!words || words >= free || write >= CMDQ_WORDS) {
		icp_log("icp_probe: command queue full (read %u, write %u)\n", read, write);
		return false;
	}
	for (uint32_t i = 0; i < words; ++i) {
		ring[(write + i) % CMDQ_WORDS] = packet[i];
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	q[QF(Q_WRITE)] = (write + words) % CMDQ_WORDS;
	__asm__ volatile ("dsb sy" ::: "memory");
	return write_reg(p->lx7 + CIRQ_HOST2ICP, 1);
}

/* One packet from the message or debug queue (hfi_read_message); 0 when there is none. */
static uint32_t
hfi_read(struct icp_probe *p, unsigned queue, uint32_t *packet, uint32_t capacity)
{
	volatile uint32_t *q = queue_header(p, queue);
	volatile uint32_t *ring = hfi_words(p, queue == 1 ? HFI_MSGQ : HFI_DBGQ);
	uint32_t size = queue == 1 ? MSGQ_WORDS : DBGQ_WORDS;
	uint32_t read = q[QF(Q_READ)], write = q[QF(Q_WRITE)];
	if (read == write || read >= size || write >= size) {
		return 0;
	}
	uint32_t words = ring[read] / 4u, available = write > read ? write - read : size - read + write;
	if (!words || words > available || words > capacity) {
		icp_log("icp_probe: queue %u packet of %u words (%u available), dropping\n", queue, words, available);
		q[QF(Q_READ)] = write;
		return 0;
	}
	for (uint32_t i = 0; i < words; ++i) {
		packet[i] = ring[(read + i) % size];
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	q[QF(Q_READ)] = (read + words) % size;
	__asm__ volatile ("dsb sy" ::: "memory");
	return words;
}

/* hfi_msg_debug: type, size, timestamp, then text. */
static void
log_debug_packet(const uint32_t *packet, uint32_t words)
{
	if (words < 6) {
		return;
	}
	char text[160];
	uint32_t length = MIN(packet[3], (words - 6) * 4u);
	length = MIN(length, (uint32_t)sizeof(text) - 1);
	memcpy(text, &packet[6], length);
	text[length] = 0;
	for (uint32_t i = 0; i < length; ++i) {
		if (text[i] == '\n' || text[i] == '\r') {
			text[i] = ' ';
		}
	}
	icp_log("icp_probe: firmware debug %#x: %s\n", packet[2], text);
}

static void
drain_debug(struct icp_probe *p)
{
	uint32_t packet[256];
	for (unsigned n = 0; n < 64; ++n) {
		uint32_t words = hfi_read(p, 2, packet, 256);
		if (!words) {
			return;
		}
		if (packet[1] == HFI_MSG_DEBUG) {
			log_debug_packet(packet, words);
		} else {
			icp_log("icp_probe: debug queue packet %#x, %u words\n", packet[1], words);
		}
	}
}

/* Wait for one message type; log what else arrives. */
static bool
hfi_wait(struct icp_probe *p, uint32_t wanted, uint32_t *packet, uint32_t capacity, unsigned timeout_ms)
{
	uint64_t start = mach_absolute_time();
	for (unsigned waited = 0; waited < timeout_ms * 10; ++waited) {
		uint32_t words;
		while ((words = hfi_read(p, 1, packet, capacity)) != 0) {
			uint64_t ns;
			absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
			if (packet[1] == wanted) {
				icp_log("icp_probe: message %#x, %u words, after %llu us\n", wanted, words, ns / 1000);
				return true;
			}
			if (packet[1] == HFI_MSG_DEBUG) {
				log_debug_packet(packet, words);
			} else {
				icp_log("icp_probe: unexpected message %#x (%u words: %#x %#x %#x)\n", packet[1], words,
				    words > 2 ? packet[2] : 0, words > 3 ? packet[3] : 0, words > 4 ? packet[4] : 0);
				if (packet[1] == HFI_MSG_EVENT) {
					return false;
				}
			}
		}
		delay(100);
	}
	icp_log("icp_probe: no message %#x within %u ms\n", wanted, timeout_ms);
	return false;
}

/*
 * Level 4: timed firmware windows. Each round authenticates the firmware, optionally runs the HFI init, SYS_INIT and a
 * ping, samples the ICP SMMU contexts every ~1 ms and the HFI memory, and ends with PAS shutdown at a fixed deadline.
 * Nothing is logged inside the window (a log line needs ~1.5 s to reach the PC and the SoC has reset within 0.1-1.5 s
 * of the firmware starting); the record is logged afterwards with the slow-trace pause. Later rounds run longer, so
 * the last round logged brackets when the reset comes and the earlier ones show what the firmware did first.
 */
#define WINDOW_TEXTS        6u
#define WINDOW_TEXT_BYTES   96u
#define WINDOW_REGS         1u /* the HFI memory map into the general-purpose registers */
#define WINDOW_INIT         2u /* the init request */
#define WINDOW_SYS_INIT     4u /* SYS_INIT once the init response is 1 */
#define WINDOW_PING         8u /* SYS_PING once INIT_DONE */
#define WINDOW_CIRQ         16u /* CIRQ and debug registers just before shutdown */
#define WINDOW_SAMPLE       32u /* the ICP SMMU fault registers every ~1 ms */
#define WINDOW_CSR          64u /* LX7 CSR reads (firmware version, GP0) just before shutdown */
#define WINDOW_NO_POLL      256u /* the init request without polling its response (no CSR read afterwards) */
#define WINDOW_FAST         512u /* poll the init response for at most 1 ms, SYS_INIT as soon as it is 1 */
#define WINDOW_CSR_EARLY    128u /* one LX7 CSR read (the init response, as stock polls it) 1 ms after auth */
#define WINDOW_HFI          (WINDOW_SAMPLE | WINDOW_REGS | WINDOW_INIT)
/* One read of another camera block instead of the LX7, just before shutdown: is it the LX7 alone that stops answering? */
#define WINDOW_CPAS_LATE    1024u /* CPAS top version (0xac13000), a block stock's host keeps reading */
#define WINDOW_BPS_LATE     2048u /* BPS top (0xac2c000) */
#define WINDOW_IPE_LATE     4096u /* IPE top (0xac42000) */
#define WINDOW_CAMCC_LATE   8192u /* CamCC's Titan top GDSC (0xadf31dc): the clock controller, outside Titan's AHB */
#define WINDOW_GCC_LATE     16384u /* GCC's camera AHB branch (0x136004) */
#define WINDOW_ERRLOG_AFTER 32768u /* after PAS shutdown, still powered: CPAS version, then CAMNOC's error logger */
#define WINDOW_PC           65536u /* after the ping: SYS_PC_PREP, PC_PREP_DONE, TrustZone suspend, as stock at +18 ms */
#define CAMNOC_ERRLOG_PA    0x0ac1b000u /* CAMNOC 0xac19000 + 0x2000 (cpastop_v680_100.h err_logger_offsets) */
/* The init response is polled for at most this long: CSR reads 60 ms after auth without an initialised firmware reset
 * the SoC, while one at 1 ms is answered. */
#define RESPONSE_POLL_US    20000u
struct window_record {
	unsigned hold_ms, flags, late_at_us;
	uint32_t response, init_error, fw_version;
	uint64_t response_us, init_done_us, ping_us, pc_us;
	uint32_t pc_error;
	bool suspend_ok;
	uint32_t fsr[ICP_STREAM_GROUPS], fsynr[ICP_STREAM_GROUPS];
	uint64_t far[ICP_STREAM_GROUPS], fault_us[ICP_STREAM_GROUPS];
	uint32_t ob_status, ib0, ib1, pfault, dbg_status, dbg_ctrl, gp0;
	uint32_t late_pa, late_value;
	uint32_t after_cpas, errlog[10];
	uint32_t msg_count, other_type, debug_count, dbgq_write;
	char sfr[WINDOW_TEXT_BYTES];
	char text[WINDOW_TEXTS][WINDOW_TEXT_BYTES];
	bool shutdown_ok;
};

static uint64_t
since_us(uint64_t start)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
	return ns / 1000;
}

static void
window_sample(struct icp_probe *p, struct window_record *r, uint64_t start)
{
	for (unsigned g = 0; g < ICP_STREAM_GROUPS; ++g) {
		unsigned fsr = 0, syn = 0;
		unsigned long long far = 0;
		if (!r->fault_us[g] && p->smmu[g] && hakios_smmu_fault_state(p->smmu[g], &fsr, &far, &syn) == 0 &&
		    (fsr & 0xc00001feu)) {
			r->fsr[g] = fsr;
			r->far[g] = far;
			r->fsynr[g] = syn;
			r->fault_us[g] = since_us(start) | 1;
		}
	}
}

static void
window_keep_text(struct window_record *r, const uint32_t *packet, uint32_t words)
{
	if (r->debug_count < WINDOW_TEXTS && words >= 6) {
		char *text = r->text[r->debug_count];
		uint32_t length = MIN(MIN(packet[3], (words - 6) * 4u), WINDOW_TEXT_BYTES - 1);
		memcpy(text, &packet[6], length);
		text[length] = 0;
		for (uint32_t i = 0; i < length; ++i) {
			if (text[i] == '\n' || text[i] == '\r' || (unsigned char)text[i] < 0x20) {
				text[i] = text[i] ? ' ' : 0;
			}
		}
	}
	r->debug_count++;
}

/* Drain the message and debug queues into the record; returns the last message type seen on the message queue. */
static uint32_t
window_drain(struct icp_probe *p, struct window_record *r, uint32_t wanted, uint32_t *packet, uint32_t *word2)
{
	uint32_t found = 0, words;
	while ((words = hfi_read(p, 1, packet, 64)) != 0) {
		r->msg_count++;
		if (packet[1] == HFI_MSG_DEBUG) {
			window_keep_text(r, packet, words);
		} else if (packet[1] == wanted && wanted) {
			found = wanted;
			*word2 = words > 2 ? packet[2] : 0;
		} else {
			r->other_type = packet[1];
		}
	}
	while ((words = hfi_read(p, 2, packet, 256)) != 0) {
		if (packet[1] == HFI_MSG_DEBUG) {
			window_keep_text(r, packet, words);
		}
	}
	return found;
}

/* Reset the queue indices, SFR and init registers left by the previous round. */
static bool
window_reset_hfi(struct icp_probe *p)
{
	for (unsigned i = 0; i < 3; ++i) {
		volatile uint32_t *q = queue_header(p, i);
		q[QF(Q_READ)] = 0;
		q[QF(Q_WRITE)] = 0;
	}
	bzero((void *)(p->hfi + HFI_SFR + 4), SFR_BYTES - 4);
	__asm__ volatile ("dsb sy" ::: "memory");
	return write_reg(p->lx7 + HFI_INIT_REQUEST, 0) && write_reg(p->lx7 + HFI_INIT_RESPONSE, 0);
}

static void
window_run(struct icp_probe *p, struct window_record *r)
{
	uint32_t packet[256];
	/* Mapped before auth: no camera access, only page tables. */
	r->late_pa = (r->flags & WINDOW_CPAS_LATE) ? 0x0ac13000u : (r->flags & WINDOW_BPS_LATE) ? 0x0ac2c000u :
	    (r->flags & WINDOW_IPE_LATE) ? 0x0ac42000u : (r->flags & WINDOW_CAMCC_LATE) ? 0x0adf31dcu :
	    (r->flags & WINDOW_GCC_LATE) ? 0x00136004u : 0;
	vm_offset_t late = r->late_pa ? ml_io_map_unmappable(r->late_pa, 0x1000, VM_WIMG_IO) : 0;
	r->late_value = 0xdeadbeef;
	vm_offset_t after = (r->flags & WINDOW_ERRLOG_AFTER) ? ml_io_map_unmappable(CAMNOC_ERRLOG_PA, 0x1000, VM_WIMG_IO) : 0;
	vm_offset_t after_cpas = after ? ml_io_map_unmappable(0x0ac13000u, 0x1000, VM_WIMG_IO) : 0;
	uint64_t start = mach_absolute_time();
	uint64_t deadline_us = (uint64_t)r->hold_ms * 1000;
	enum { WAIT_RESPONSE, WAIT_INIT, WAIT_PING, WAIT_PC, IDLE } state =
	    (r->flags & WINDOW_INIT) && !(r->flags & WINDOW_NO_POLL) ? WAIT_RESPONSE : IDLE;
	if (r->flags & WINDOW_REGS) {
		static const struct { unsigned reg; uint32_t value; } regs[] = {
			{ HFI_QTBL_PTR, HFI_IOVA + HFI_QTBL }, { HFI_SFR_PTR, HFI_IOVA + HFI_SFR },
			{ HFI_SHARED_PTR, SHARED_IOVA }, { HFI_SHARED_SIZE, SHARED_SIZE },
			{ HFI_HEAP_PTR, HFI_IOVA + HFI_HEAP }, { HFI_HEAP_SIZE, HFI_QTBL - HFI_HEAP },
			{ HFI_QDSS_IOVA, QDSS_IOVA }, { HFI_QDSS_SIZE, QDSS_SIZE },
			{ HFI_IO_IOVA, IO_IOVA }, { HFI_IO_SIZE, IO_SIZE },
			{ HFI_IO2_IOVA, IO2_IOVA }, { HFI_IO2_SIZE, IO2_SIZE },
			{ HFI_UNCACHED_IOVA, HFI_IOVA }, { HFI_UNCACHED_SIZE, HFI_SIZE },
		};
		for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
			(void)write_reg(p->lx7 + regs[i].reg, regs[i].value);
		}
	}
	if (r->flags & WINDOW_INIT) {
		(void)write_reg(p->lx7 + HFI_INIT_REQUEST, 1);
	}
	bool blind_init = (r->flags & WINDOW_NO_POLL) && (r->flags & WINDOW_SYS_INIT);
	if (r->flags & WINDOW_CSR_EARLY) {
		delay(1000);
		(void)hakios_mmio_try_read32((const volatile void *)(p->lx7 + HFI_INIT_RESPONSE), &r->response);
		r->response_us = since_us(start);
	}
	uint64_t now;
	if (late && r->late_at_us) {
		/* The one late read at a fixed time after auth, then nothing more on the camera until shutdown. */
		while (since_us(start) < r->late_at_us) {
		}
		(void)hakios_mmio_try_read32((const volatile void *)late, &r->late_value);
		r->response_us = since_us(start);
	}
	while ((now = since_us(start)) < deadline_us) {
		if (r->flags & WINDOW_SAMPLE) {
			window_sample(p, r, start);
		}
		if (state == WAIT_RESPONSE && now > ((r->flags & WINDOW_FAST) ? 1000u : RESPONSE_POLL_US)) {
			state = IDLE;                   /* no more CSR access in this window */
		} else if (state == WAIT_RESPONSE) {
			uint32_t response = 0;
			if (hakios_mmio_try_read32((const volatile void *)(p->lx7 + HFI_INIT_RESPONSE), &response) == 0 &&
			    response) {
				r->response = response;
				r->response_us = now;
				static const uint32_t init[] = { 8, HFI_CMD_SYS_INIT };
				state = response == 1 && (r->flags & WINDOW_SYS_INIT) && hfi_send(p, init) ? WAIT_INIT : IDLE;
			}
		} else if (blind_init && state == IDLE && !r->response_us && now >= 2000) {
			/* No CSR read after the init request (they reset the SoC): SYS_INIT through the queue 2 ms later; the
			 * firmware's answer comes back in memory. response_us marks when it was sent. */
			static const uint32_t init[] = { 8, HFI_CMD_SYS_INIT };
			r->response_us = now;
			state = hfi_send(p, init) ? WAIT_INIT : IDLE;
		} else if (state == WAIT_INIT) {
			if (window_drain(p, r, HFI_MSG_INIT_DONE, packet, &r->init_error) == HFI_MSG_INIT_DONE) {
				r->init_done_us = now;
				const uint32_t ping[] = { 16, HFI_CMD_SYS_PING, (uint32_t)PING_COOKIE, (uint32_t)(PING_COOKIE >> 32) };
				state = !r->init_error && (r->flags & WINDOW_PING) && hfi_send(p, ping) ? WAIT_PING : IDLE;
			}
		} else if (state == WAIT_PING) {
			uint32_t cookie = 0;
			if (window_drain(p, r, HFI_MSG_PING_ACK, packet, &cookie) == HFI_MSG_PING_ACK) {
				r->ping_us = cookie == (uint32_t)PING_COOKIE ? now : 1;
				static const uint32_t prep[] = { 8, HFI_CMD_SYS_PC_PREP };
				state = (r->flags & WINDOW_PC) && hfi_send(p, prep) ? WAIT_PC : IDLE;
			}
		} else if (state == WAIT_PC) {
			if (window_drain(p, r, HFI_MSG_PC_PREP_DONE, packet, &r->pc_error) == HFI_MSG_PC_PREP_DONE) {
				r->pc_us = now;
				/* stock: qcom_scm_set_remote_state(0, 33) right after PC_PREP_DONE */
				r->suspend_ok = scm("ICP SUSPEND", SCM_REMOTE_STATE, 2, 0, PAS_ID, 0);
				state = IDLE;
			}
		} else if (r->flags & WINDOW_SYS_INIT) {
			uint32_t unused;
			(void)window_drain(p, r, 0, packet, &unused);
		}
		delay(state != IDLE ? 50 : 1000);
	}
	if (r->flags & WINDOW_CSR) {
		(void)read_reg(p->lx7 + HFI_FW_VERSION, &r->fw_version);
		(void)hakios_mmio_try_read32((const volatile void *)(p->lx7 + LX7_GP(0)), &r->gp0);
	}
	r->dbgq_write = queue_header(p, 2)[QF(Q_WRITE)];
	volatile const char *sfr = (volatile const char *)(p->hfi + HFI_SFR + 4);
	for (unsigned i = 0; i < WINDOW_TEXT_BYTES - 1; ++i) {
		char c = sfr[i];
		r->sfr[i] = c && (c < 0x20 || c > 0x7e) ? '.' : c;
		if (!c) {
			break;
		}
	}
	if (r->flags & WINDOW_CIRQ) {
		const struct { unsigned offset; uint32_t *field; } cirq[] = {
			{ CIRQ_OB_STATUS, &r->ob_status }, { CIRQ_IB_STATUS0, &r->ib0 }, { CIRQ_IB_STATUS1, &r->ib1 },
			{ CIRQ_PFAULT, &r->pfault }, { LX7_DBG_STATUS, &r->dbg_status }, { LX7_DBG_CTRL, &r->dbg_ctrl },
		};
		for (unsigned i = 0; i < sizeof(cirq) / sizeof(cirq[0]); ++i) {
			(void)hakios_mmio_try_read32((const volatile void *)(p->lx7 + cirq[i].offset), cirq[i].field);
		}
	}
	if (late && !r->late_at_us) {
		(void)hakios_mmio_try_read32((const volatile void *)late, &r->late_value);
	}
	if (r->flags & WINDOW_SAMPLE) {
		window_sample(p, r, start);
	}
	r->shutdown_ok = scm("PAS SHUTDOWN", PAS_SHUTDOWN, 1, PAS_ID, 0, 0);
	if (after && after_cpas && r->shutdown_ok) {
		/* MAINCTL, ERRVLD, then ERRLOG0-3 low/high (0x008, 0x010, 0x020-0x03c) */
		static const uint16_t off[10] = { 0x008, 0x010, 0x020, 0x024, 0x028, 0x02c, 0x030, 0x034, 0x038, 0x03c };
		r->after_cpas = 0xdeadbeef;
		(void)hakios_mmio_try_read32((const volatile void *)after_cpas, &r->after_cpas);
		for (unsigned i = 0; i < 10; ++i) {
			r->errlog[i] = 0xdeadbeef;
			(void)hakios_mmio_try_read32((const volatile void *)(after + off[i]), &r->errlog[i]);
		}
	}
	if (after) {
		io_unmap(after, 0x1000);
	}
	if (after_cpas) {
		io_unmap(after_cpas, 0x1000);
	}
	if (late) {
		io_unmap(late, 0x1000);
	}
}

static void
window_report(const struct window_record *r, unsigned round)
{
	icp_slow_ms = icp_slow_ms ? icp_slow_ms : 1500;
	icp_log("icp_probe: window %u (%u ms, steps %#x): shutdown %s, init response %u at %llu us, INIT_DONE %llu us "
	    "error %#x, ping %llu us, fw version %#x, gp0 %#x\n", round, r->hold_ms, r->flags,
	    r->shutdown_ok ? "OK" : "FAILED", r->response, r->response_us, r->init_done_us, r->init_error, r->ping_us,
	    r->fw_version, r->gp0);
	icp_log("icp_probe: window %u: %u messages (other type %#x), %u debug, debug queue write %u, SFR \"%s\"\n", round,
	    r->msg_count, r->other_type, r->debug_count, r->dbgq_write, r->sfr);
	for (unsigned g = 0; g < ICP_STREAM_GROUPS; ++g) {
		if (r->fault_us[g]) {
			icp_log("icp_probe: window %u: SMMU %#x fault at %llu us: FSR %#x FAR %#llx FSYNR0 %#x\n", round,
			    icp_streams[g].sid, r->fault_us[g], r->fsr[g], r->far[g], r->fsynr[g]);
		}
	}
	if (r->flags & WINDOW_PC) {
		icp_log("icp_probe: window %u: PC_PREP_DONE %llu us error %#x, TrustZone suspend %s\n", round, r->pc_us,
		    r->pc_error, r->suspend_ok ? "OK" : "not done or FAILED");
	}
	if (r->flags & WINDOW_ERRLOG_AFTER) {
		icp_log("icp_probe: window %u: after shutdown CPAS %#x; CAMNOC errlogger mainctl %#x errvld %#x log0 %#x %#x "
		    "log1 %#x %#x log2 %#x %#x log3 %#x %#x\n", round, r->after_cpas, r->errlog[0], r->errlog[1],
		    r->errlog[2], r->errlog[3], r->errlog[4], r->errlog[5], r->errlog[6], r->errlog[7], r->errlog[8], r->errlog[9]);
	}
	if (r->late_pa) {
		icp_log("icp_probe: window %u: late read of %#x %s: %#x\n", round, r->late_pa,
		    r->late_at_us ? "at a fixed time (init response us field)" : "before shutdown", r->late_value);
	}
	if (r->flags & WINDOW_CIRQ) {
		icp_log("icp_probe: window %u: CIRQ OB %#x IB %#x %#x PFAULT %#x, debug status %#x control %#x\n", round,
		    r->ob_status, r->ib0, r->ib1, r->pfault, r->dbg_status, r->dbg_ctrl);
	}
	for (unsigned i = 0; i < MIN(r->debug_count, WINDOW_TEXTS); ++i) {
		icp_log("icp_probe: window %u firmware: %s\n", round, r->text[i]);
	}
}

static bool
run_windows(struct icp_probe *p, const char **step)
{
	/* One step more per window; icp_first=N starts at window N, so any window can be a boot's first firmware start. */
	static const struct { unsigned hold_ms, flags, late_at_us; } rounds[] = {
		{ 60, 0 }, /* level 3 plus the init registers zeroed before auth */
		{ 60, WINDOW_SAMPLE },
		{ 60, WINDOW_SAMPLE | WINDOW_CSR_EARLY },
		{ 60, WINDOW_SAMPLE | WINDOW_CSR },
		{ 60, WINDOW_SAMPLE | WINDOW_REGS },
		{ 60, WINDOW_HFI | WINDOW_NO_POLL },
		{ 60, WINDOW_HFI | WINDOW_NO_POLL | WINDOW_SYS_INIT },
		{ 80, WINDOW_HFI | WINDOW_NO_POLL | WINDOW_SYS_INIT | WINDOW_PING },
		{ 300, WINDOW_HFI | WINDOW_NO_POLL | WINDOW_SYS_INIT | WINDOW_PING },
		/* Stock's pace: init response polled within 1 ms of the request, SYS_INIT at once (CSR access later reset). */
		{ 60, WINDOW_HFI | WINDOW_FAST | WINDOW_SYS_INIT },
		{ 80, WINDOW_HFI | WINDOW_FAST | WINDOW_SYS_INIT | WINDOW_PING },
		/* 12-14: one late read of CPAS, BPS, IPE (no LX7 access), each a window of its own */
		{ 60, WINDOW_SAMPLE | WINDOW_CPAS_LATE },
		{ 60, WINDOW_SAMPLE | WINDOW_BPS_LATE },
		{ 60, WINDOW_SAMPLE | WINDOW_IPE_LATE },
		/* 15-17: a control with no late read, then CamCC and GCC late reads */
		{ 60, WINDOW_SAMPLE },
		{ 60, WINDOW_SAMPLE | WINDOW_CAMCC_LATE },
		{ 60, WINDOW_SAMPLE | WINDOW_GCC_LATE },
		/* 18-22: when the hang starts: one CPAS read at 1, 1.5, 2, 3 and 5 ms after auth (response_us: when it ran) */
		{ 60, WINDOW_CPAS_LATE, 1000 },
		{ 60, WINDOW_CPAS_LATE, 1500 },
		{ 60, WINDOW_CPAS_LATE, 2000 },
		{ 60, WINDOW_CPAS_LATE, 3000 },
		{ 60, WINDOW_CPAS_LATE, 5000 },
		/* 23: the firmware's own failed access, if it was one: CAMNOC's error logger after shutdown */
		{ 60, WINDOW_SAMPLE | WINDOW_ERRLOG_AFTER },
		/* 24-25: stock's boot: SYS_INIT, ping, power collapse and suspend within a few ms, then shutdown */
		{ 60, WINDOW_HFI | WINDOW_NO_POLL | WINDOW_SYS_INIT | WINDOW_PING | WINDOW_PC },
		{ 300, WINDOW_HFI | WINDOW_NO_POLL | WINDOW_SYS_INIT | WINDOW_PING | WINDOW_PC },
	};
	static struct window_record record;
	uint32_t first = 1;
	PE_parse_boot_argn("icp_first", &first, sizeof(first));
	if (!setup_queues(p, step)) {
		return false;
	}
	for (unsigned i = first ? first - 1 : 0; i < sizeof(rounds) / sizeof(rounds[0]); ++i) {
		bzero(&record, sizeof(record));
		record.hold_ms = rounds[i].hold_ms;
		record.flags = rounds[i].flags;
		record.late_at_us = rounds[i].late_at_us;
		/* Each window powers the ICP up and, right after PAS shutdown, back down, as stock gates its clocks then. */
		*step = "ICP power for a window";
		if (i != (first ? first - 1 : 0) && !power_up(p, step)) {
			return false;
		}
		*step = "HFI reset between windows";
		if (!window_reset_hfi(p)) {
			return false;
		}
		icp_log("icp_probe: window %u: %u ms, steps %#x\n", i + 1, record.hold_ms, record.flags);
		if (!load_firmware(p, step)) {
			icp_slow_ms = p->slow_saved;
			return false;
		}
		window_run(p, &record);
		p->pas_initialized = p->running = p->auth_attempted = false;
		bool powered_down = record.shutdown_ok && restore_changes(p);
		icp_slow_ms = p->slow_saved;
		if (record.shutdown_ok && !powered_down) {
			window_report(&record, i + 1);
			*step = "ICP power-down after a window";
			return false;
		}
		window_report(&record, i + 1);
		for (unsigned line = 0; line < state_count; line++) {
			icp_log("icp_probe: at auth %s\n", state_text[line]);
		}
		state_count = 0;
		if (!record.shutdown_ok) {
			*step = "PAS shutdown after a window";
			return false;
		}
	}
	return true;
}

static void
log_failure_state(struct icp_probe *p)
{
	log_lx7(p, "after failure");
	drain_debug(p);
	char text[121];
	volatile const char *sfr = (volatile const char *)(p->hfi + HFI_SFR + 4);
	unsigned length = 0;
	while (length < sizeof(text) - 1 && sfr[length]) {
		text[length] = sfr[length];
		length++;
	}
	text[length] = 0;
	if (length) {
		icp_log("icp_probe: firmware failure reason: %s\n", text);
	}
}

static bool
exchange(struct icp_probe *p, const char **step)
{
	uint32_t packet[64];
	*step = "SYS_INIT";
	static const uint32_t init[] = { 8, HFI_CMD_SYS_INIT };
	if (!hfi_send(p, init) || !hfi_wait(p, HFI_MSG_INIT_DONE, packet, 64, 5000)) {
		return false;
	}
	icp_log("icp_probe: SYS_INIT_DONE error %#x, %u properties%s%#x\n", packet[2], packet[3],
	    packet[3] ? ", first " : " ", packet[3] && packet[0] >= 20 ? packet[4] : 0);
	if (packet[2]) {
		return false;
	}
	*step = "SYS_PING";
	const uint32_t ping[] = { 16, HFI_CMD_SYS_PING, (uint32_t)PING_COOKIE, (uint32_t)(PING_COOKIE >> 32) };
	if (!hfi_send(p, ping) || !hfi_wait(p, HFI_MSG_PING_ACK, packet, 64, 5000)) {
		return false;
	}
	if (packet[2] != (uint32_t)PING_COOKIE || packet[3] != (uint32_t)(PING_COOKIE >> 32)) {
		icp_log("icp_probe: ping answered with %#x %#x\n", packet[2], packet[3]);
		return false;
	}
	icp_log("icp_probe: ping answered\n");
	drain_debug(p);
	return true;
}

/* Stock power collapse with the firmware authenticated (__power_collapse, cam_lx7_core_control): SYS_PC_PREP,
 * PC_PREP_DONE, TrustZone suspend. The PAS shutdown in cleanup follows, as cam_icp_mgr_hw_close does. */
static bool
power_collapse(struct icp_probe *p)
{
	uint32_t packet[64];
	static const uint32_t prep[] = { 8, HFI_CMD_SYS_PC_PREP };
	bool ok = hfi_send(p, prep) && hfi_wait(p, HFI_MSG_PC_PREP_DONE, packet, 64, 5000);
	if (ok) {
		icp_log("icp_probe: PC_PREP_DONE error %#x\n", packet[2]);
	}
	return scm("ICP SUSPEND", SCM_REMOTE_STATE, 2, 0, PAS_ID, 0) && ok;
}

static bool
cleanup(struct icp_probe *p)
{
	bool ok = true;
	if (p->pas_initialized) {
		bool shutdown = scm("PAS SHUTDOWN", PAS_SHUTDOWN, 1, PAS_ID, 0, 0);
		if (!shutdown && p->auth_attempted) {
			icp_log("icp_probe: preserving power and DMA after uncertain PAS shutdown\n");
			return false;
		}
		ok = shutdown;
	}
	bool keep_backing = false;
	for (unsigned g = 0; g < ICP_STREAM_GROUPS; ++g) {
		if (!p->smmu[g]) {
			continue;
		}
		bool clean = hakios_smmu_faults(p->smmu[g]);
		int unmap = p->group_mapped[g] ? hakios_smmu_unmap(p->smmu[g], HFI_IOVA, HFI_SIZE) : 0;
		if (p->qdss_mapped[g]) {
			unmap = hakios_smmu_unmap(p->smmu[g], QDSS_IOVA, QDSS_SIZE) || unmap;
		}
		int release = hakios_smmu_release(p->smmu[g]);
		icp_log("icp_probe: SMMU %#x %s, unmap %d, release %d\n", icp_streams[g].sid, clean ? "clean" : "faulted",
		    unmap, release);
		ok = ok && clean && !unmap && !release;
		keep_backing = keep_backing || (release && (!p->group_mapped[g] || unmap)); /* translations could reach it */
	}
	for (unsigned c = 0; c < 2; ++c) {
		if (p->cdm[c]) {
			bool clean = hakios_smmu_faults(p->cdm[c]);
			int release = hakios_smmu_release(p->cdm[c]);
			icp_log("icp_probe: CDM SMMU %s, release %d\n", clean ? "clean" : "faulted", release);
			ok = ok && clean && !release;
		}
	}
	/* The HFI memory and the register mappings stay in place: freeing or unmapping them at the end panicked twice in
	 * kmem_free (an "atomicity mismatch", then "no map entry", 2026-11-23), and the probe runs once per boot. */
	(void)keep_backing;
	ok = restore_changes(p) && ok;
	ok = hakios_cam_power_vote(&p->power, HAKIOS_RPMH_CAMERA, false) && ok;
	return ok;
}

/*
 * Level 5 (CAMERA-PLAN section 5 steps 3-4): one stock BPS frame on our raw, then stock's final IPE pass on the BPS
 * output. fw/BPS_REPLAY.bin (make-bps-replay.py) holds the nonzero pages of CamX's command buffers from the stock
 * Hi-res shot, the zeroed output regions, our raw frame and stock's HFI packets. Everything sits at stock's IOVAs; the
 * only patches are the IPE pass's input addresses (made on the PC) and the firmware's handles (word 8 of the async
 * commands, by each packet's handle index). The outputs are checked and published: the BPS 1920 x 1440 NV12
 * thumbnail as /dev/hakios-icp-nv12, the IPE 4000 x 3008 NV12 as /dev/hakios-icp-nv12-full.
 */
#define REPLAY_FILE         "BPS_REPLAY.bin"
#define REPLAY_CLAIM_BASE   0x00800000u
#define REPLAY_CLAIM_END    0xfef00000u /* the end of stock's IO2 region */
#define REPLAY_REGIONS      16u
#define REPLAY_HANDLES      4u
#define REPLAY_CREATE       1u          /* packet flags (make-bps-replay.py) */
#define REPLAY_FRAME        2u
#define HFI_CMD_CREATE      0x01010008u
#define HFI_MSG_CREATE_ACK  0x01020008u
#define HFI_CMD_ASYNC_DIR   0x0101000au
#define HFI_CMD_ASYNC_IND   0x0101000eu

static struct {
	uint32_t npages, nzero, npackets, raw_iova, raw_size, span_lo, span_hi;
	/* [0] the BPS 1920 x 1440 thumbnail, [1] the IPE 4000 x 3008 image: NV12 planes by IOVA */
	uint32_t luma_iova[2], luma_size[2], chroma_iova[2], chroma_size[2], width[2], height[2], stride[2];
	const uint32_t *zero, *page_iova;
	const uint8_t *pages, *raw, *packets, *end;
	/* kernel buffers by IOVA: the span, the raw input, the zeroed regions */
	uint32_t iova[REPLAY_REGIONS], size[REPLAY_REGIONS];
	vm_offset_t kva[REPLAY_REGIONS];
	unsigned regions;
	bool ready[2];
} replay;

static vm_offset_t
replay_kva(uint32_t iova, uint32_t size)
{
	for (unsigned i = 0; i < replay.regions; ++i) {
		if (iova >= replay.iova[i] && iova + size <= replay.iova[i] + replay.size[i]) {
			return replay.kva[i] + (iova - replay.iova[i]);
		}
	}
	return 0;
}

static bool
replay_region(uint32_t iova, uint32_t size, const uint8_t *data, uint32_t data_size)
{
	vm_offset_t kva = 0;
	size = (size + 0x3fffu) & ~0x3fffu;
	if (replay.regions == REPLAY_REGIONS || kmem_alloc(kernel_map, &kva, size, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		icp_log("icp_probe: replay: %u bytes for IOVA %#x not allocated\n", size, iova);
		return false;
	}
	if (data) {
		memcpy((void *)kva, data, data_size);
	}
	flush_dcache64((addr64_t)kva, size, FALSE);
	replay.iova[replay.regions] = iova;
	replay.size[replay.regions] = size;
	replay.kva[replay.regions++] = kva;
	return true;
}

static bool
replay_load(const char **step)
{
	*step = "fw/" REPLAY_FILE;
	uint64_t size = 0;
	const uint8_t *file = hakios_image_file("fw", REPLAY_FILE, &size, NULL);
	if (!file || size < 80 || memcmp(file, "BPSR", 4)) {
		icp_log("icp_probe: replay: fw/%s missing or not a replay bundle\n", REPLAY_FILE);
		return false;
	}
	const uint32_t *h = (const uint32_t *)(file + 4);
	/* version 3 (make-bps-replay.py --no-raw, the still service's): the raw size only, each photograph brings its own */
	if (h[0] != 2 && h[0] != 3) {
		icp_log("icp_probe: replay: bundle version %u, want 2 or 3\n", h[0]);
		return false;
	}
	bool raw_included = h[0] == 2;
	replay.npages = h[1];
	replay.nzero = h[2];
	replay.npackets = h[3];
	replay.raw_iova = h[4];
	replay.raw_size = h[5];
	replay.luma_iova[0] = h[6];
	replay.luma_size[0] = h[7];
	replay.chroma_iova[0] = h[8];
	replay.chroma_size[0] = h[9];
	replay.span_lo = h[10];
	replay.span_hi = h[11];
	replay.luma_iova[1] = h[12];
	replay.luma_size[1] = h[13];
	replay.chroma_iova[1] = h[14];
	replay.chroma_size[1] = h[15];
	replay.width[1] = h[16];
	replay.height[1] = h[17];
	replay.stride[1] = h[18];
	replay.width[0] = 1920;
	replay.height[0] = 1440;
	replay.stride[0] = 1920;
	replay.zero = h + 19;
	replay.page_iova = replay.zero + 2 * replay.nzero;
	replay.pages = (const uint8_t *)(replay.page_iova + replay.npages);
	replay.raw = replay.pages + (uint64_t)replay.npages * 4096;
	replay.packets = replay.raw + (raw_included ? replay.raw_size : 0);
	if (!raw_included) {
		replay.raw = NULL;
	}
	replay.end = file + size;
	if (replay.packets > replay.end || replay.nzero > REPLAY_REGIONS - 2) {
		icp_log("icp_probe: replay: bundle truncated (%llu bytes)\n", size);
		return false;
	}
	*step = "replay buffers";
	if (!replay_region(replay.span_lo, replay.span_hi - replay.span_lo, NULL, 0)) {
		return false;
	}
	if (!replay_region(replay.raw_iova, replay.raw_size, replay.raw, replay.raw_size)) {
		return false;
	}
	uint64_t zeroed = 0;
	for (uint32_t i = 0; i < replay.nzero; ++i) {
		if (!replay_region(replay.zero[2 * i], replay.zero[2 * i + 1], NULL, 0)) {
			return false;
		}
		zeroed += replay.zero[2 * i + 1];
	}
	/* Command pages: the span, and the captured buffers the programs read inside the zeroed regions (the CDM
	 * programs' command lists, buffers 220 and 329, and their DMI tables, 217 and 315). */
	for (uint32_t i = 0; i < replay.npages; ++i) {
		vm_offset_t kva = replay_kva(replay.page_iova[i], 4096);
		if (!kva) {
			icp_log("icp_probe: replay: page at %#x is in no region\n", replay.page_iova[i]);
			return false;
		}
		memcpy((void *)kva, replay.pages + (uint64_t)i * 4096, 4096);
		flush_dcache64((addr64_t)kva, 4096, FALSE);
	}
	icp_log("icp_probe: replay: %u command pages in the %u KiB span at %#x, raw %u bytes at %#x, %u zeroed regions "
	    "(%llu KiB), %u packets\n", replay.npages, (replay.span_hi - replay.span_lo) / 1024, replay.span_lo,
	    replay.raw_size, replay.raw_iova, replay.nzero, zeroed / 1024, replay.npackets);
	icp_claim_base = REPLAY_CLAIM_BASE;
	icp_claim_size = REPLAY_CLAIM_END - REPLAY_CLAIM_BASE;
	return true;
}

/* Between sessions: zero the output regions again and put back the command pages that live inside them. */
static void
replay_reset(void)
{
	for (unsigned i = 2; i < replay.regions; ++i) {
		bzero((void *)replay.kva[i], replay.size[i]);
		flush_dcache64((addr64_t)replay.kva[i], replay.size[i], FALSE);
	}
	for (uint32_t i = 0; i < replay.npages; ++i) {
		uint32_t iova = replay.page_iova[i];
		if (iova >= replay.span_lo && iova < replay.span_hi) {
			continue;
		}
		vm_offset_t kva = replay_kva(iova, 4096);
		if (kva) {
			memcpy((void *)kva, replay.pages + (uint64_t)i * 4096, 4096);
			flush_dcache64((addr64_t)kva, 4096, FALSE);
		}
	}
}

static bool
replay_map(struct icp_probe *p, const char **step)
{
	*step = "replay SMMU mappings";
	for (unsigned g = 0; g < ICP_STREAM_GROUPS; ++g) {
		for (unsigned i = 0; p->smmu[g] && i < replay.regions; ++i) {
			int error = hakios_smmu_map_kva(p->smmu[g], replay.iova[i], replay.kva[i], replay.size[i],
			    HAKIOS_SMMU_READ | HAKIOS_SMMU_WRITE);
			if (error) {
				icp_log("icp_probe: replay: IOVA %#x (%u bytes) not mapped for SMMU %#x: %d\n", replay.iova[i],
				    replay.size[i], icp_streams[g].sid, error);
				return false;
			}
		}
	}
	return true;
}

/* An ACK of one type, with its words logged; debug messages and anything else are logged on the way. */
static bool
replay_ack(struct icp_probe *p, uint32_t wanted, uint32_t *packet, unsigned timeout_ms, uint64_t *us)
{
	uint64_t start = mach_absolute_time();
	bool got = hfi_wait(p, wanted, packet, 256, timeout_ms);
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
	*us = ns / 1000;
	if (got) {
		icp_log("icp_probe: replay: ack %#x after %llu us: %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x\n", wanted, *us,
		    packet[2], packet[3], packet[4], packet[5], packet[6], packet[7], packet[8], packet[9], packet[10],
		    packet[11]);
	}
	drain_debug(p);
	return got;
}

static void
replay_plane_stats(const char *name, uint32_t iova, uint32_t size)
{
	vm_offset_t kva = replay_kva(iova, size);
	if (!kva) {
		icp_log("icp_probe: replay: %s at %#x: not in a region\n", name, iova);
		return;
	}
	flush_dcache64((addr64_t)kva, size, FALSE);
	const uint8_t *b = (const uint8_t *)kva;
	uint64_t sum = 0, nonzero = 0;
	uint8_t lo = 255, hi = 0;
	for (uint32_t i = 0; i < size; ++i) {
		sum += b[i];
		nonzero += b[i] != 0;
		lo = MIN(lo, b[i]);
		hi = MAX(hi, b[i]);
	}
	icp_log("icp_probe: replay: %s at %#x: %u bytes, mean %llu, min %u, max %u, %llu nonzero; first %02x %02x %02x "
	    "%02x\n", name, iova, size, sum / size, lo, hi, nonzero, b[0], b[1], b[2], b[3]);
}

/*
 * /dev/hakios-icp-nv12 (minor 0, BPS thumbnail) and /dev/hakios-icp-nv12-full (minor 1, IPE image): "NV12BPS1",
 * width, height, stride, luma bytes, chroma bytes, three reserved words, then the two planes.
 */
static int icp_nv12_major = -1;
struct icp_nv12_header {
	char magic[8];
	uint32_t width, height, stride, luma, chroma, reserved[3];
};

static int
icp_nv12_open(dev_t dev, __unused int flags, __unused int type, __unused struct proc *proc)
{
	unsigned n = (unsigned)minor(dev);
	return n < 2 && replay.ready[n] ? 0 : ENXIO;
}

static int
icp_nv12_read(dev_t dev, struct uio *uio, __unused int flags)
{
	unsigned n = (unsigned)minor(dev);
	if (n >= 2 || !replay.ready[n]) {
		return ENXIO;
	}
	uint32_t luma = replay.stride[n] * replay.height[n], chroma = replay.stride[n] * (replay.height[n] / 2);
	struct icp_nv12_header header = { "NV12BPS1", replay.width[n], replay.height[n], replay.stride[n], luma, chroma,
		                          { 0 } };
	struct { const void *base; uint64_t bytes; } part[3] = {
		{ &header, sizeof(header) },
		{ (const void *)replay_kva(replay.luma_iova[n], luma), luma },
		{ (const void *)replay_kva(replay.chroma_iova[n], chroma), chroma },
	};
	off_t off = uio_offset(uio);
	for (unsigned i = 0; i < 3 && uio_resid(uio) > 0; ++i) {
		if (off >= (off_t)part[i].bytes) {
			off -= (off_t)part[i].bytes;
			continue;
		}
		if (!part[i].base) {
			return EIO;
		}
		int error = uiomove((caddr_t)part[i].base + off, (int)MIN(uio_resid(uio), (off_t)part[i].bytes - off), uio);
		if (error) {
			return error;
		}
		off = 0;
	}
	return 0;
}

static const struct cdevsw icp_nv12_cdevsw = {
	.d_open = icp_nv12_open,
	.d_close = nulldev,
	.d_read = icp_nv12_read,
	.d_write = eno_rdwrt,
	.d_ioctl = eno_ioctl,
	.d_stop = eno_stop,
	.d_reset = eno_reset,
	.d_select = eno_select,
	.d_mmap = eno_mmap,
	.d_strategy = eno_strat,
};

static void
replay_publish(unsigned n)
{
	static bool made[2];
	replay.ready[n] = true;
	if (made[n]) {
		return;     /* the node serves the newest session's planes */
	}
	if (icp_nv12_major < 0) {
		icp_nv12_major = cdevsw_add(-1, &icp_nv12_cdevsw);
	}
	const char *name = n ? "hakios-icp-nv12-full" : "hakios-icp-nv12";
	if (icp_nv12_major >= 0 && devfs_make_node(makedev(icp_nv12_major, n), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0400,
	    "%s", name)) {
		made[n] = true;
		icp_log("icp_probe: replay: %u x %u NV12 published as /dev/%s\n", replay.width[n], replay.height[n], name);
	}
}

/*
 * Stock's BPS and IPE resume (bps_core.c / ipe_core.c cam_*_handle_resume): clear the CPAS collapse bit (CPAS top 0x48
 * BPS, 0x40 IPE), then regulator_set_mode(FAST), the GDSC's HW_CONTROL bit, so the firmware switches their power
 * itself. Without it the firmware logged "Power Off Failed for BPS/IPE". Back under software control before teardown,
 * as stock does at power collapse.
 */
#define GDSC_HW_CONTROL     (1u << 1)
static bool replay_gdsc_hw_on;

static void
replay_gdsc_hw(struct icp_probe *p, bool on)
{
	vm_offset_t cpas_page = ml_io_map_unmappable(CPAS_TOP_PA, PAGE_SIZE, VM_WIMG_IO);
	static const struct { uint32_t gdscr, pwr_ctrl, pwr_status; } dom[2] = {
		{ BPS_GDSCR, 0x48, 0x44 }, { IPE_GDSCR, 0x40, 0x3c },
	};
	for (unsigned i = 0; i < 2; ++i) {
		uint32_t gdsc = 0, ctrl = 0, status = 0;
		(void)read_reg(p->cc + dom[i].gdscr, &gdsc);
		if (cpas_page && on) {
			(void)read_reg(cpas_page + dom[i].pwr_ctrl, &ctrl);
			if (ctrl & 1u) {
				(void)write_reg(cpas_page + dom[i].pwr_ctrl, 0);
			}
		}
		(void)write_reg(p->cc + dom[i].gdscr, on ? gdsc | GDSC_HW_CONTROL : gdsc & ~GDSC_HW_CONTROL);
		delay(10);
		uint32_t after = 0;
		(void)read_reg(p->cc + dom[i].gdscr, &after);
		if (cpas_page) {
			(void)read_reg(cpas_page + dom[i].pwr_ctrl, &ctrl);
			(void)read_reg(cpas_page + dom[i].pwr_status, &status);
		}
		icp_log("icp_probe: replay: %s GDSC %#x -> %#x (%s control), CPAS pwr_ctrl %#x pwr_status %#x\n",
		    i ? "IPE" : "BPS", gdsc, after, on ? "hardware" : "software", ctrl, status);
	}
	if (cpas_page) {
		io_unmap(cpas_page, PAGE_SIZE);
	}
	replay_gdsc_hw_on = on;
}

static bool
replay_run(struct icp_probe *p, const char **step)
{
	replay_gdsc_hw(p, true);
	uint32_t packet[64], msg[256];
	uint32_t handles[REPLAY_HANDLES] = { 0 }, created = 0, frames = 0, good = 0;
	const uint8_t *q = replay.packets;
	for (uint32_t n = 0; n < replay.npackets; ++n) {
		uint32_t length, flags;
		if (q + 8 > replay.end) {
			return false;
		}
		memcpy(&length, q, 4);
		memcpy(&flags, q + 4, 4);
		if (length < 8 || length > sizeof(packet) || q + 8 + length > replay.end) {
			return false;
		}
		memcpy(packet, q + 8, length);
		q += 8 + ((length + 3) & ~3u);
		uint32_t type = packet[1], index = (flags >> 8) & 0xffu;
		uint64_t us;
		if (index) {
			if (index > created || index >= REPLAY_HANDLES || length < 40) {
				icp_log("icp_probe: replay: packet %u wants handle %u of %u\n", n, index, created);
				return false;
			}
			packet[8] = handles[index];
		}
		*step = "replay command";
		if (!hfi_send(p, packet)) {
			return false;
		}
		if (type == HFI_CMD_CREATE) {
			*step = "CREATE_HANDLE";
			if (!replay_ack(p, HFI_MSG_CREATE_ACK, msg, 2000, &us) || msg[2] || created + 1 >= REPLAY_HANDLES) {
				return false;
			}
			handles[++created] = msg[3];
			icp_log("icp_probe: replay: handle %u (type %u) is %#x\n", created, packet[2], msg[3]);
		} else if (type == HFI_CMD_ASYNC_DIR || type == HFI_CMD_ASYNC_IND) {
			bool frame = flags & REPLAY_FRAME;
			*step = frame ? "FRAME_PROCESS" : type == HFI_CMD_ASYNC_IND ? "CONFIG_IO" : "MEM_MAP";
			if (!replay_ack(p, type + 0x10000u, msg, frame ? 5000 : 2000, &us)) {
				return false;
			}
			if (frame) {
				/* ACK word 7: CAMERAICP_* (hfi_sys_defs.h); 12 is ECDMERROR, words 8-10 its CDM details */
				icp_log("icp_probe: replay: FRAME_PROCESS opcode %u (handle %u) answered after %llu us, error %u%s\n",
				    packet[2], index, us, msg[7], msg[7] == 12 ? " (CDM error)" : "");
				frames++;
				good += msg[7] == 0;
				/* opcode 2 is BPS_FRAME_PROCESS, 6 IPE_FRAME_PROCESS */
				unsigned out = packet[2] == 2 ? 0 : 1;
				replay_plane_stats(out ? "IPE NV12 luma" : "BPS NV12 luma", replay.luma_iova[out],
				    replay.luma_size[out]);
				replay_plane_stats(out ? "IPE NV12 chroma" : "BPS NV12 chroma", replay.chroma_iova[out],
				    replay.chroma_size[out]);
				if (msg[7] == 0) {
					replay_publish(out);
				}
			}
		} else {
			delay(1000); /* SET_PROPERTY: no answer, as stock */
		}
	}
	*step = "frame errors";
	icp_log("icp_probe: replay: %u of %u frames without error\n", good, frames);
	return frames && good == frames;
}

static int
run_probe(unsigned level)
{
	struct icp_probe p = { 0 };
	icp_log("icp_probe: level %u starting, RAM %#lx + %#lx\n", level, gPhysBase, gPhysSize);
	const char *step = "RPMh power and bandwidth";
	icp_claim_base = HFI_IOVA;
	icp_claim_size = HFI_SIZE + QDSS_SIZE;
	bool ok = level != 5 || replay_load(&step);
	if (ok) {
		step = "RPMh power and bandwidth";
		ok = hakios_cam_power_vote(&p.power, HAKIOS_RPMH_CAMERA, true);
	}
	if (ok) {
		step = "map registers";
		p.gcc = ml_io_map_unmappable(trunc_page(GCC_PA), PAGE_SIZE, VM_WIMG_IO);
		if (p.gcc) {
			p.gcc += GCC_PA & PAGE_MASK;
		}
		p.cc = ml_io_map_unmappable(CAMCC_PA, CAMCC_SIZE, VM_WIMG_IO);
		p.lx7 = ml_io_map_unmappable(trunc_page(LX7_PA), PAGE_SIZE, VM_WIMG_IO);
		if (p.lx7) {
			p.lx7 += LX7_PA & PAGE_MASK;
		}
		ok = p.gcc && p.cc && p.lx7 && power_up(&p, &step);
	}
	if (ok) {
		log_lx7(&p, "powered");
	}
	if (ok && level == 3) {
		/* Firmware start and stop only, no HFI: run it 100 ms, then 4 s, each ended by PAS shutdown. */
		ok = setup_queues(&p, &step);
		static const unsigned hold_ms[] = { 100, 4000 };
		for (unsigned i = 0; ok && i < 2; ++i) {
			ok = load_firmware(&p, &step);
			icp_slow_ms = 0;
			delay(hold_ms[i] * 1000);
			bool stopped = scm("PAS SHUTDOWN", PAS_SHUTDOWN, 1, PAS_ID, 0, 0);
			icp_slow_ms = p.slow_saved;
			icp_log("icp_probe: firmware ran %u ms, PAS shutdown %s\n", hold_ms[i], stopped ? "OK" : "FAILED");
			p.pas_initialized = p.running = p.auth_attempted = false;
			ok = ok && stopped;
			step = "firmware start and stop";
		}
	}
	if (ok && level == 4) {
		ok = run_windows(&p, &step);
	}
	if (ok && (level == 2 || level == 5)) {
		ok = setup_queues(&p, &step) && (level != 5 || replay_map(&p, &step)) && load_firmware(&p, &step) &&
		    hfi_init(&p, &step);
		if (ok) {
			log_lx7(&p, "booted");
			ok = exchange(&p, &step);
		}
		/* Level 5 runs the BPS and IPE sessions twice on one firmware boot, outputs zeroed in between (repeated runs). */
		for (unsigned round = 0; ok && level == 5 && round < 2; ++round) {
			if (round) {
				replay_reset();
			}
			icp_log("icp_probe: replay: round %u\n", round + 1);
			ok = replay_run(&p, &step);
		}
		if (!ok && p.auth_attempted && p.hfi) {
			log_failure_state(&p);
		}
		if (p.running) {
			bool collapsed = power_collapse(&p);
			if (replay_gdsc_hw_on) {
				replay_gdsc_hw(&p, false);
			}
			if (!collapsed && ok) {
				step = "power collapse";
				ok = false;
			}
		}
	}
	bool restored = cleanup(&p);
	int result = restored ? (ok ? 1 : -1) : -2;
	icp_log("icp_probe: level %u %s (%s, teardown %s)\n", level, result == 1 ? "PASS" : "FAIL",
	    ok ? "checkpoint reached" : step, restored ? "OK" : "FAILED");
	return result;
}

static void
probe_thread(void *arg, __unused wait_result_t wait)
{
	uint32_t start_s = 0;
	PE_parse_boot_argn("icp_slow", &icp_slow_ms, sizeof(icp_slow_ms));
	PE_parse_boot_argn("icp_start", &start_s, sizeof(start_s));
	if ((uintptr_t)arg != 1 && kmem_alloc_contig(kernel_map, &hfi_reserved, HFI_SIZE, 0, 0, 0,
	    KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		hfi_reserved = 0;
		icp_log("icp_probe: HFI memory not reserved at boot; trying again when the probe starts\n");
	}
	if (start_s) {
		icp_log("icp_probe: starting in %u s\n", start_s);
		delay((uint64_t)start_s * 1000000);
	}
	icp_result = run_probe((unsigned)(uintptr_t)arg);
	thread_terminate(current_thread());
}

void
hakios_icp_probe_init(uint32_t level)
{
	if (level < 1 || level > 5) {
		icp_log("icp_probe: unsupported level %u (1 power, 2 firmware, SYS_INIT and ping, 3 firmware start and stop, "
		    "4 timed firmware windows, 5 one stock BPS frame)\n", level);
		return;
	}
	thread_t thread;
	if (kernel_thread_start(probe_thread, (void *)(uintptr_t)level, &thread) != KERN_SUCCESS) {
		icp_log("icp_probe: cannot start thread\n");
		return;
	}
	thread_deallocate(thread);
}

/*
 * The still service (CONFIG_ICP, icp=1 with cam=5; CAMERA-PLAN section 5 step 4): Camera.app's 12 MP photographs
 * through BPS and IPE, beside the live camera session. /dev/hakios-icp: write() a struct icp_raw_header and the
 * shutter frame's RAW10 lines; the firmware boots, the stock Hi-res BPS pass and final IPE pass (fw/BPS_REPLAY.bin,
 * stock's processing settings) run on that frame, and the firmware is shut down again. read() then returns the IPE
 * 4000 x 3008 NV12 as /dev/hakios-icp-nv12-full does. The SMMU contexts, the HFI memory and the replay's buffers are
 * set up on the first photograph and kept for the boot, as stock keeps its; power, clocks and firmware come and go
 * with each one. Any failure is EIO, and the adapter gives the IFE's own still instead.
 */
struct icp_raw_header {
	char magic[8];          /* "ICPRAW10" */
	uint32_t width, height, stride, reserved;
};
#define ICP_RAW_WIDTH       4000u
#define ICP_RAW_HEIGHT      3008u
#define ICP_RAW_LINE        (ICP_RAW_WIDTH * 5u / 4u)

static LCK_GRP_DECLARE(icp_service_group, "camera ICP");
static LCK_MTX_DECLARE(icp_service_lock, &icp_service_group);
static struct icp_probe svc;
static bool svc_setup, svc_dead;
static uint32_t svc_photos;

static bool
svc_prepare(const char **step)
{
	if (svc_setup) {
		return true;
	}
	if (!replay_load(step)) {
		return false;
	}
	*step = "map registers";
	svc.gcc = ml_io_map_unmappable(trunc_page(GCC_PA), PAGE_SIZE, VM_WIMG_IO);
	svc.cc = ml_io_map_unmappable(CAMCC_PA, CAMCC_SIZE, VM_WIMG_IO);
	svc.lx7 = ml_io_map_unmappable(trunc_page(LX7_PA), PAGE_SIZE, VM_WIMG_IO);
	if (!svc.gcc || !svc.cc || !svc.lx7) {
		return false;
	}
	svc.gcc += GCC_PA & PAGE_MASK;
	svc.lx7 += LX7_PA & PAGE_MASK;
	state_taken = true; /* the full register dump at auth is the probe's */
	if (!setup_queues(&svc, step) || !replay_map(&svc, step)) {
		return false;
	}
	svc_setup = true;
	return true;
}

static bool
svc_photo(const char **step)
{
	struct icp_probe *p = &svc;
	p->count = 0;
	p->pas_initialized = p->auth_attempted = p->running = false;
	*step = "RPMh power and bandwidth";
	bool ok = hakios_cam_power_vote(&p->power, HAKIOS_RPMH_ICP, true) && power_up(p, step);
	if (ok) {
		bzero((void *)p->backing, HFI_SIZE);
		hfi_sync((void *)p->backing, HFI_SIZE);
		ok = queue_table_init(p, step) && load_firmware(p, step) && hfi_init(p, step) && exchange(p, step);
	}
	if (ok) {
		replay_reset();
		ok = replay_run(p, step);
	}
	if (!ok && p->auth_attempted) {
		log_failure_state(p);
	}
	if (p->running) {
		bool collapsed = power_collapse(p);
		if (!collapsed && ok) {
			*step = "power collapse";
			ok = false;
		}
	}
	if (replay_gdsc_hw_on) {
		replay_gdsc_hw(p, false);
	}
	if (p->pas_initialized) {
		bool down = scm("PAS SHUTDOWN", PAS_SHUTDOWN, 1, PAS_ID, 0, 0);
		p->pas_initialized = p->running = false;
		if (!down) {
			icp_log("icp: PAS shutdown failed; power and DMA kept, no more photographs this boot\n");
			svc_dead = true;
			return false;
		}
	}
	for (unsigned g = 0; g < ICP_STREAM_GROUPS; ++g) {
		if (p->smmu[g] && !hakios_smmu_faults(p->smmu[g])) {
			icp_log("icp: SMMU %#x faulted\n", icp_streams[g].sid);
			ok = false;
		}
	}
	bool restored = restore_changes(p);
	restored = hakios_cam_power_vote(&p->power, HAKIOS_RPMH_ICP, false) && restored;
	if (!restored) {
		icp_log("icp: clocks or power not restored; no more photographs this boot\n");
		svc_dead = true;
	}
	return ok && restored;
}

static int
svc_open(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *proc)
{
	return svc_dead ? ENXIO : 0;
}

static int
svc_read(__unused dev_t dev, struct uio *uio, int flags)
{
	return icp_nv12_read(makedev(0, 1), uio, flags);
}

static int
svc_write(__unused dev_t dev, struct uio *uio, __unused int flags)
{
	struct icp_raw_header h;
	if (uio_resid(uio) < (user_ssize_t)sizeof(h)) {
		return EINVAL;
	}
	int error = uiomove((caddr_t)&h, sizeof(h), uio);
	if (error) {
		return error;
	}
	if (memcmp(h.magic, "ICPRAW10", 8) || h.width != ICP_RAW_WIDTH || h.height != ICP_RAW_HEIGHT ||
	    h.stride < ICP_RAW_LINE || h.stride > 8192 || uio_resid(uio) < (user_ssize_t)h.stride * h.height) {
		return EINVAL;
	}
	if (!lck_mtx_try_lock(&icp_service_lock)) {
		return EBUSY;
	}
	uint64_t started = mach_absolute_time(), ns;
	const char *step = "first-photograph setup";
	bool ok = !svc_dead && svc_prepare(&step);
	if (!ok && !svc_dead) {
		svc_dead = true;
		icp_log("icp: setup failed at %s; no more photographs this boot\n", step);
		/* give back what it claimed, so Iris2 and the camera keep their contexts */
		for (unsigned g = 0; g < ICP_STREAM_GROUPS; ++g) {
			if (svc.smmu[g]) {
				int unmapped = svc.group_mapped[g] ? hakios_smmu_unmap(svc.smmu[g], HFI_IOVA, HFI_SIZE) : 0;
				int released = hakios_smmu_release(svc.smmu[g]);
				icp_log("icp: SMMU %#x given back (unmap %d, release %d)\n", icp_streams[g].sid, unmapped, released);
				svc.smmu[g] = NULL;
			}
		}
	}
	replay.ready[1] = false;
	/* The BPS reads MIPI RAW10 at stock's 5040-byte pitch: each line copied in, the frame's own padding skipped. */
	uint32_t pitch = ok ? replay.raw_size / ICP_RAW_HEIGHT : 0;
	vm_offset_t raw = ok ? replay_kva(replay.raw_iova, replay.raw_size) : 0;
	if (ok && (pitch < ICP_RAW_LINE || !raw)) {
		step = "raw input region";
		ok = false;
	}
	uint8_t skip[256];
	for (uint32_t y = 0; ok && y < ICP_RAW_HEIGHT; ++y) {
		error = uiomove((caddr_t)(raw + (vm_offset_t)y * pitch), ICP_RAW_LINE, uio);
		for (uint32_t left = h.stride - ICP_RAW_LINE; !error && left;) {
			uint32_t n = MIN(left, (uint32_t)sizeof(skip));
			error = uiomove((caddr_t)skip, n, uio);
			left -= n;
		}
		if (error) {
			step = "frame copy";
			ok = false;
		}
	}
	if (ok) {
		flush_dcache64((addr64_t)raw, replay.raw_size, FALSE);
		ok = svc_photo(&step);
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - started, &ns);
	++svc_photos;
	icp_log("icp: photograph %u %s in %llu ms%s%s\n", svc_photos, ok ? "through BPS and IPE" : "FAILED",
	    ns / 1000000, ok ? "" : " at ", ok ? "" : step);
	lck_mtx_unlock(&icp_service_lock);
	return ok ? 0 : EIO;
}

static const struct cdevsw icp_service_cdevsw = {
	.d_open = svc_open,
	.d_close = nulldev,
	.d_read = svc_read,
	.d_write = svc_write,
	.d_ioctl = eno_ioctl,
	.d_stop = eno_stop,
	.d_reset = eno_reset,
	.d_select = eno_select,
	.d_mmap = eno_mmap,
	.d_strategy = eno_strat,
};

void
hakios_icp_service_init(void)
{
	icp_service = true;
	/* the 7 MiB of contiguous HFI memory, taken now while it is still to be had (later, iOS has split memory too
	 * finely); everything else waits for the first photograph */
	if (kmem_alloc_contig(kernel_map, &hfi_reserved, HFI_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		hfi_reserved = 0;
		icp_log("icp: no contiguous HFI memory: stills stay on the IFE\n");
		return;
	}
	int major = cdevsw_add(-1, &icp_service_cdevsw);
	if (major >= 0 && devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "hakios-icp")) {
		icp_log("icp: serving /dev/hakios-icp (Camera.app stills through BPS and IPE)\n");
		return;
	}
	if (major >= 0) {
		cdevsw_remove(major, &icp_service_cdevsw);
	}
	icp_log("icp: could not create /dev/hakios-icp\n");
}
