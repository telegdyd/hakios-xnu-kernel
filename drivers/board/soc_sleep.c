/*
 * SoC sleep and active power management for boards listing "soc-sleep" (SM8450):
 *   - APSS DRV (DRV 2) RPMh BCM vote keeper and DDR/LLCC floor following CPU clocks
 *   - Sleep/wake TCS sets 3 and 5 (dropping CPU active-only & USB votes in cluster sleep)
 *   - HLOS ARC sleep & CX/MMCX collapse in TCS sets 4 and 6 (xo.lvl, mmcx.lvl, cx.lvl)
 *   - AOP sleep statistics (aosd, cxsd), DDR statistics, CX vote monitor, and 30 s power log
 *
 * All hardware base addresses come from hakios_board() (board/board.h). Drivers outside
 * here connect through weak-compatible APIs declared in <sys/board/input.h>.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <machine/machine_routines.h>
#include <os/atomic_private.h>
#include <pexpert/pexpert.h>
#include <sys/board/input.h>
#include <sys/errno.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>
#include <stdarg.h>

#include "kernel.h"
#include "board.h"
#include "cmd_db.h"
#include "gcc8450.h"
#include "log.h"
#include "mmio.h"
#include "rpmh.h"

static inline void
ss_barrier(void)
{
#if defined(HAKIOS_MMIO_TEST)
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
#else
	__asm__ volatile ("dsb sy" ::: "memory");
#endif
}

HAKIOS_DEFINE_SAY(ml_get_interrupts_enabled())
#define tell say

#define BCM_COMMIT              (1u << 30)
#define BCM_VALID               (1u << 29)
#define BCM_VOTE(x, y)          (BCM_COMMIT | BCM_VALID | ((uint32_t)(x) & 0x3fffu) << 14 | ((uint32_t)(y) & 0x3fffu))
#define BCM_ACV_APPS            8u

#define KEEP_BCMS               16
#define KEEP_CLIENTS            HAKIOS_RPMH_CLIENTS
#define KEEP_SLEEP_TCS          3u
#define KEEP_WAKE_TCS           5u
#define KEEP_ACTIVE_ONLY        ((1u << HAKIOS_RPMH_CPU) | (1u << HAKIOS_RPMH_USB))
#define KEEP_ALL                ((1u << KEEP_CLIENTS) - 1u)

#define ARC_SLEEP_TCS           4u
#define ARC_WAKE_TCS            6u
#define ARC_MAX                 8u
#define ARC_GATES               16u

#define RSC_TCS_COUNT           8u
#define RSC_CMDS                16u
#define RSC_STRIDE              0x2a0u
#define SET_REG(t, reg)         (rsc_base + rsc_tcs_off + RSC_STRIDE * (t) + (reg))

#define GCC_SIZE_8450           0x1f4200UL
#define DISPCC_SIZE             0x10000UL
#define DISPCC_MDSS_INT2_GDSCR  0xb000u

#define SS_WINDOW               0x10000UL
#define SS_RECORDS              3
#define SS_RECORD               32
#define DDR_MAGIC               0xa1157a75u
#define DDR_MAX_MODES           0x14u
#define DDR_DRVS                18u
#define DDR_SIZE                (8u + 16u * DDR_MAX_MODES + 4u * DDR_DRVS)
#define VX_SIZE                 0x400u
#define VX_DRVS                 20u
#define VX_WORDS                ((VX_DRVS + 3u) / 4u)
#define AOP_TICKS_PER_MS        19200ull
#define LOG_SECONDS             30u
#define LOG_SAMPLES             480u
#define LOG_FREQS               12u

extern bool hakios_display_off(void) __attribute__((weak));
extern bool hakios_screen_off(void) __attribute__((weak));
extern bool hakios_ufs_power_down(void) __attribute__((weak));
__attribute__((weak)) bool
hakios_pcie_drv_cx_ready(void)
{
	return false;
}
extern bool hakios_aop_message(const char *text) __attribute__((weak));
extern bool hakios_battery_get(int *soc, int *present, int *charger) __attribute__((weak));
extern bool hakios_battery_power(int *ma, int *mv, int *charger) __attribute__((weak));
extern int hakios_mmio_try_read32(const volatile void *addr, unsigned *out) __attribute__((weak));
extern void hakios_rpmh_lock(void) __attribute__((weak));
extern void hakios_rpmh_unlock(void) __attribute__((weak));

struct cmd_db_bcm {
	uint32_t unit;
	uint16_t width;
	uint8_t vcd, reserved;
};

static LCK_GRP_DECLARE(soc_sleep_lck_grp, "hakios-soc-sleep");
static LCK_MTX_DECLARE(keep_lock, &soc_sleep_lck_grp);
static LCK_MTX_DECLARE(ss_lock, &soc_sleep_lck_grp);

static vm_offset_t rsc_base;
static uint32_t rsc_tcs_off;

static void
rsc_lock(void)
{
	if (hakios_rpmh_lock != NULL) {
		hakios_rpmh_lock();
	}
}

static void
rsc_unlock(void)
{
	if (hakios_rpmh_unlock != NULL) {
		hakios_rpmh_unlock();
	}
}

static const struct cmd_db_header *
cmd_db_map(uint32_t *size_out)
{
	static const struct cmd_db_header *db;
	static uint32_t db_size;
	const struct hakios_board *board = hakios_board();

	if (db == NULL && board != NULL && board->rpmh.cmd_db != 0 && board->rpmh.cmd_db_size >= sizeof(*db)) {
		db = (const struct cmd_db_header *)ml_io_map_wcomb(board->rpmh.cmd_db, board->rpmh.cmd_db_size);
		db_size = board->rpmh.cmd_db_size;
	}
	if (size_out != NULL) {
		*size_out = db_size;
	}
	return db;
}

static uint32_t
cmd_db_addr(const char *name)
{
	uint32_t db_size = 0;
	const struct cmd_db_header *db = cmd_db_map(&db_size);
	const struct cmd_db_entry *e = db != NULL ? hakios_cmd_db_find(db, db_size, name, CMD_DB_BCM, NULL) : NULL;

	return e != NULL ? e->address : 0;
}

static bool
bcm_aux(const char *name, uint32_t *unit, uint32_t *width)
{
	uint32_t db_size = 0;
	const struct cmd_db_header *db = cmd_db_map(&db_size);
	unsigned slave = 0;
	const struct cmd_db_entry *e;
	const void *data;
	unsigned len = 0;
	struct cmd_db_bcm aux;

	if (db == NULL) {
		return false;
	}
	e = hakios_cmd_db_find(db, db_size, name, CMD_DB_BCM, &slave);
	if (e == NULL) {
		return false;
	}
	data = hakios_cmd_db_data(db, db_size, slave, e, &len);
	if (data == NULL || len < sizeof(aux)) {
		return false;
	}
	memcpy(&aux, data, sizeof(aux));
	*unit = aux.unit;
	*width = aux.width;
	return aux.unit != 0;
}

static uint32_t
rpmh_db_find(const char *name, uint16_t *levels, unsigned *nlevels, unsigned *kind)
{
	uint32_t db_size = 0;
	const struct cmd_db_header *db = cmd_db_map(&db_size);
	unsigned slave = 0;
	const struct cmd_db_entry *e;

	*nlevels = 0;
	*kind = 0;
	if (db == NULL) {
		return 0;
	}
	e = hakios_cmd_db_find(db, db_size, name, CMD_DB_ANY, &slave);
	if (e == NULL) {
		return 0;
	}
	*kind = db->slave[slave].id;
	if (*kind == CMD_DB_ARC) {
		(void)hakios_cmd_db_levels(db, db_size, name, levels, 16, nlevels);
	}
	return e->address;
}

/* ---- APSS DRV BCM vote keeper, sleep/wake TCS 3/5, and DDR/LLCC floor ------------------------- */

static struct {
	char            name[8];
	uint32_t        address, unit, width;
	uint32_t        ab[KEEP_CLIENTS], ib[KEEP_CLIENTS];
	uint32_t        sent;
} keep[KEEP_BCMS];
static unsigned keep_n, keep_writes, keep_failures;
static bool keep_on, keep_sets;
static unsigned keep_sets_n, keep_set_writes;
static uint32_t keep_slept[KEEP_BCMS], keep_woke[KEEP_BCMS];
static uint32_t keep_probe_mhz;
static int keep_drop;

static const struct {
	uint32_t small_khz, big_khz, ddr_mhz, llcc_mhz;
} floor_rows[] = {
	{ 1100000,    1100000,    547,  300 },
	{ 1400000,    1500000,    1017, 466 },
	{ 1800000,    2000000,    1555, 600 },
	{ UINT32_MAX, 2600000,    2092, 806 },
	{ UINT32_MAX, UINT32_MAX, 3196, 1066 },
};
#define FLOOR_OFF_DDR           200u
#define FLOOR_OFF_LLCC          300u

static const struct {
	const char      *bcm;
	unsigned        client;
	uint32_t        ab, ib;
} keep_loader[] = {
	{ "MM0", HAKIOS_RPMH_DISPLAY, 2180, 4255 },
	{ "MM1", HAKIOS_RPMH_DISPLAY, 0, 1 },
	{ "CN0", HAKIOS_RPMH_CPU, 0, 1 },
	{ "SN2", HAKIOS_RPMH_USB, 1265, 1264 },
};
#define KEEP_LOADER             (sizeof(keep_loader) / sizeof(keep_loader[0]))
static int keep_loader_slot[KEEP_LOADER] = { -1, -1, -1, -1 };

static int keep_mc0 = -1, keep_sh0 = -1, keep_acv = -1;
static uint32_t floor_ddr, floor_llcc, floor_changes;
static bool floor_display, floor_display_known, floor_usb, floor_usb_known;
static bool floor_scanout;
static bool pcie_bw_needed;
static int pcie_bw_slot = -1;
static char floor_line[200];

SYSCTL_DECL(_kern_hakios);

static int
ddr_floor_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	if (req->oldptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, floor_line, sizeof(floor_line));
	}
	return SYSCTL_OUT(req, floor_line, strlen(floor_line) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, ddr_floor, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, ddr_floor_sysctl, "A",
    "the processors' DDR/LLCC floor now (MHz, MC0/SH0 votes), its changes, the APSS BCM writes and failures");

static int
keep_slot(const char *name)
{
	char id[8] = { 0 };
	uint32_t address;

	strlcpy(id, name, sizeof(id));
	for (unsigned k = 0; k < keep_n; k++) {
		if (memcmp(keep[k].name, id, sizeof(id)) == 0) {
			return (int)k;
		}
	}
	address = cmd_db_addr(name);
	if (keep_n == KEEP_BCMS || address == 0) {
		return -1;
	}
	memcpy(keep[keep_n].name, id, sizeof(id));
	keep[keep_n].address = address;
	(void)bcm_aux(name, &keep[keep_n].unit, &keep[keep_n].width);
	return (int)keep_n++;
}

static uint32_t
keep_sum(unsigned k, uint32_t clients)
{
	uint32_t ab = 0, ib = 0;
	bool mask = strncmp(keep[k].name, "ACV", 8) == 0;

	for (unsigned c = 0; c < KEEP_CLIENTS; c++) {
		if (!(clients & (1u << c))) {
			continue;
		}
		ab = MIN(ab + keep[k].ab[c], 0x3fffu);
		ib = mask ? (ib | keep[k].ib[c]) : MAX(ib, keep[k].ib[c]);
	}
	return (ab == 0 && ib == 0) ? BCM_COMMIT : BCM_VOTE(ab, MIN(ib, 0x3fffu));
}

static uint32_t
keep_mhz(int k, uint32_t mhz)
{
	uint64_t vote;

	if (k < 0 || keep[k].unit == 0) {
		return 0;
	}
	vote = ((uint64_t)mhz * keep[k].width * 1000000ull + keep[k].unit - 1ull) / keep[k].unit;
	return (uint32_t)MIN(vote, 0x3fffu);
}

static uint32_t
keep_sleep_cmd(unsigned k)
{
	if (keep[k].sent == 0) {
		return 0;
	}
	if (keep_probe_mhz != 0 && (int)k == keep_mc0) {
		return BCM_VOTE(0, keep_mhz((int)k, keep_probe_mhz));
	}
	return keep_sum(k, KEEP_ALL & ~KEEP_ACTIVE_ONLY & ~(uint32_t)keep_drop);
}

static void
keep_sets_write(void)
{
	uint32_t address[KEEP_BCMS], sleep[KEEP_BCMS], wake[KEEP_BCMS];
	unsigned n = 0;
	bool same = true;

	if (!keep_sets || rsc_base == 0) {
		return;
	}
	for (unsigned k = 0; k < keep_n; k++) {
		uint32_t s = keep_sleep_cmd(k);

		same = same && keep_slept[k] == s && keep_woke[k] == keep[k].sent;
		if (keep[k].sent == 0) {
			continue;
		}
		address[n] = keep[k].address;
		sleep[n] = s;
		wake[n++] = keep[k].sent;
	}
	if (same && n == keep_sets_n) {
		return;
	}
	rsc_lock();
	for (unsigned t = KEEP_SLEEP_TCS; t <= KEEP_WAKE_TCS; t += KEEP_WAKE_TCS - KEEP_SLEEP_TCS) {
		wr(SET_REG(t, 0x1cu), 0);
		for (unsigned i = 0; i < n; i++) {
			wr(SET_REG(t, 0x30u + 20u * i), 0x10008u | 0x100u);
			wr(SET_REG(t, 0x34u + 20u * i), address[i]);
			wr(SET_REG(t, 0x38u + 20u * i), t == KEEP_SLEEP_TCS ? sleep[i] : wake[i]);
		}
		wr(SET_REG(t, 0x1cu), n ? (1u << n) - 1u : 0);
	}
	rsc_unlock();
	for (unsigned k = 0; k < keep_n; k++) {
		keep_slept[k] = keep_sleep_cmd(k);
		keep_woke[k] = keep[k].sent;
	}
	keep_sets_n = n;
	keep_set_writes++;
}

static bool
keep_flush_active(void)
{
	uint32_t address[KEEP_BCMS], data[KEEP_BCMS];
	unsigned n = 0, which[KEEP_BCMS];
	const char *problem;

	for (unsigned k = 0; k < keep_n; k++) {
		uint32_t command = keep_sum(k, KEEP_ALL);

		if (command != keep[k].sent) {
			address[n] = keep[k].address;
			data[n] = command;
			which[n++] = k;
		}
	}
	if (n == 0) {
		return true;
	}
	rsc_lock();
	problem = hakios_rpmh_send_rsc(rsc_base, rsc_tcs_off, address, data, n);
	rsc_unlock();
	keep_writes++;
	if (problem != NULL) {
		keep_failures++;
		return false;
	}
	for (unsigned i = 0; i < n; i++) {
		keep[which[i]].sent = data[i];
	}
	return true;
}

static bool
keep_flush(void)
{
	bool done = keep_flush_active();

	keep_sets_write();
	return done;
}

static void
floor_describe(void)
{
	snprintf(floor_line, sizeof(floor_line),
	    "ddr %u MHz (MC0 %u) llcc %u MHz (SH0 %u) changes %u writes %u failed %u%s sets %s %u/%u display %s probe %u usb %s drop %#x",
	    floor_ddr, keep_mhz(keep_mc0, floor_ddr), floor_llcc, keep_mhz(keep_sh0, floor_llcc), floor_changes,
	    keep_writes, keep_failures, keep_on ? "" : " (off)", keep_sets ? "on" : "off", keep_sets_n, keep_set_writes,
	    floor_display ? "on" : "off", keep_probe_mhz, floor_usb ? "in" : "out", keep_drop);
}

static void
keep_loader_set(unsigned client, bool on)
{
	for (unsigned i = 0; i < KEEP_LOADER; i++) {
		int k = keep_loader_slot[i];

		if (k >= 0 && keep_loader[i].client == client) {
			keep[k].ab[client] = on ? keep_loader[i].ab : 0;
			keep[k].ib[client] = on ? keep_loader[i].ib : 0;
		}
	}
}

bool
hakios_rpmh_bw(enum hakios_rpmh_client client, const char *bcm, uint32_t vote_a, uint32_t vote_b)
{
	bool done = false;
	int k;

	if (client >= KEEP_CLIENTS || rsc_base == 0 || bcm == NULL) {
		return false;
	}
	lck_mtx_lock(&keep_lock);
	if ((k = keep_slot(bcm)) >= 0) {
		keep[k].ab[client] = vote_a;
		keep[k].ib[client] = vote_b;
		done = keep_flush();
	}
	lck_mtx_unlock(&keep_lock);
	return done;
}

static void
floor_display_set(bool on)
{
	if (!keep_on) {
		return;
	}
	lck_mtx_lock(&keep_lock);
	if (floor_display_known && on == floor_display) {
		lck_mtx_unlock(&keep_lock);
		return;
	}
	if (keep_mc0 >= 0) {
		keep[keep_mc0].ib[HAKIOS_RPMH_DISPLAY] = on ? keep_mhz(keep_mc0, floor_rows[0].ddr_mhz) : 0;
	}
	if (keep_sh0 >= 0) {
		keep[keep_sh0].ib[HAKIOS_RPMH_DISPLAY] = on ? keep_mhz(keep_sh0, floor_rows[0].llcc_mhz) : 0;
	}
	if (keep_acv >= 0) {
		keep[keep_acv].ib[HAKIOS_RPMH_DISPLAY] = on ? BCM_ACV_APPS : 0;
	}
	keep_loader_set(HAKIOS_RPMH_DISPLAY, on);
	(void)keep_flush();
	floor_display = on;
	floor_display_known = true;
	lck_mtx_unlock(&keep_lock);
	floor_describe();
}

void
hakios_display_bandwidth(bool needed)
{
	bool screen_off = hakios_screen_off != NULL && hakios_screen_off();

	__atomic_store_n(&floor_scanout, needed, __ATOMIC_RELEASE);
	floor_display_set(needed || !screen_off);
}

static void
floor_usb_set(void)
{
	int soc = -1, present = 0, charger = 1;
	bool in;

	if (hakios_battery_get != NULL) {
		(void)hakios_battery_get(&soc, &present, &charger);
	}
	in = charger != 0;
	if (!keep_on || (floor_usb_known && in == floor_usb)) {
		return;
	}
	lck_mtx_lock(&keep_lock);
	keep_loader_set(HAKIOS_RPMH_USB, in);
	(void)keep_flush();
	lck_mtx_unlock(&keep_lock);
	floor_usb = in;
	floor_usb_known = true;
	floor_describe();
}

static bool
floor_pcie_set(void)
{
	bool ok = false, on;

	if (!keep_on) {
		return !__atomic_load_n(&pcie_bw_needed, __ATOMIC_ACQUIRE);
	}
	lck_mtx_lock(&keep_lock);
	on = __atomic_load_n(&pcie_bw_needed, __ATOMIC_ACQUIRE);
	if (on && pcie_bw_slot < 0) {
		pcie_bw_slot = keep_slot("SN0");
	}
	if (pcie_bw_slot >= 0) {
		keep[pcie_bw_slot].ab[HAKIOS_RPMH_PCIE] = on ? 1 : 0;
		keep[pcie_bw_slot].ib[HAKIOS_RPMH_PCIE] = on ? 1 : 0;
		ok = keep_flush();
	} else {
		ok = !on;
	}
	lck_mtx_unlock(&keep_lock);
	return ok;
}

bool
hakios_pcie_bandwidth(bool needed)
{
	__atomic_store_n(&pcie_bw_needed, needed, __ATOMIC_RELEASE);
	return !needed || floor_pcie_set();
}

static void
floor_set(uint32_t ddr_mhz, uint32_t llcc_mhz)
{
	if (!keep_on || (ddr_mhz == floor_ddr && llcc_mhz == floor_llcc)) {
		return;
	}
	lck_mtx_lock(&keep_lock);
	if (keep_mc0 >= 0) {
		keep[keep_mc0].ab[HAKIOS_RPMH_CPU] = 0;
		keep[keep_mc0].ib[HAKIOS_RPMH_CPU] = keep_mhz(keep_mc0, ddr_mhz);
	}
	if (keep_sh0 >= 0) {
		keep[keep_sh0].ab[HAKIOS_RPMH_CPU] = 0;
		keep[keep_sh0].ib[HAKIOS_RPMH_CPU] = keep_mhz(keep_sh0, llcc_mhz);
	}
	(void)keep_flush();
	lck_mtx_unlock(&keep_lock);
	floor_ddr = ddr_mhz;
	floor_llcc = llcc_mhz;
	floor_changes++;
	floor_describe();
}

static int
sysctl_rpmh_sleep_drop SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int value = keep_drop, changed = 0, error;

	error = sysctl_io_number(req, value, sizeof(value), &value, &changed);
	if (error != 0 || !changed) {
		return error;
	}
	lck_mtx_lock(&keep_lock);
	keep_drop = value & (int)KEEP_ALL;
	keep_sets_write();
	lck_mtx_unlock(&keep_lock);
	floor_describe();
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_sleep_drop, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    sysctl_rpmh_sleep_drop, "I", "diag: a bit per client (sys/board/input.h HAKIOS_RPMH_*) left out of the APSS sleep set");

void
hakios_soc_sleep_follow(const uint32_t *khz, uint32_t groups)
{
	uint32_t k0 = groups > 0 ? khz[0] : 0;
	uint32_t k1 = groups > 1 ? khz[1] : 0;
	uint32_t k2 = groups > 2 ? khz[2] : 0;
	uint32_t big = MAX(k1, k2);
	bool screen_off = hakios_screen_off != NULL && hakios_screen_off();
	unsigned r = 0;

	if (!keep_on) {
		return;
	}
	floor_display_set(!screen_off || __atomic_load_n(&floor_scanout, __ATOMIC_ACQUIRE));
	floor_usb_set();
	(void)floor_pcie_set();
	while (r < sizeof(floor_rows) / sizeof(floor_rows[0]) - 1u &&
	    (k0 > floor_rows[r].small_khz || big > floor_rows[r].big_khz)) {
		r++;
	}
	if (r == 0 && screen_off) {
		floor_set(FLOOR_OFF_DDR, FLOOR_OFF_LLCC);
	} else {
		floor_set(floor_rows[r].ddr_mhz, floor_rows[r].llcc_mhz);
	}
}

/* ---- HLOS ARC sleep & CX/MMCX collapse in TCS sets 4 and 6 ------------------------------------- */

static const struct {
	uint32_t offset;
	const char *name;
} gcc8450_gdscs[] = {
	{ 0x7b004, "pcie_0_gdsc" },
	{ 0x9d004, "pcie_1_gdsc" },
	{ 0x87004, "ufs_phy_gdsc" },
	{ 0x49004, "usb30_prim_gdsc" },
};
static const uint32_t arc_collapse_gdsc[] = { 0x49004, 0x7b004, 0x9d004, 0x87004 };
#define ARC_PCIE0_GDSC          0x7b004u
#define ARC_COLLAPSE_GDSCS      (sizeof(arc_collapse_gdsc) / sizeof(arc_collapse_gdsc[0]))

static vm_offset_t arc_gcc, arc_dispcc, arc_camcc, arc_videocc[2];
static uint32_t arc_camcc_off, arc_videocc_off[2];
static bool arc_mm_readable;

static struct {
	char name[12];
	uint32_t address, sleep, wake;
	bool sleep_only;
} arc[ARC_MAX];
static uint32_t arc_n, arc_mask, arc_wake_mask, arc_live, arc_gate[ARC_GATES], arc_gates;
static bool arc_ready;
static struct {
	uint64_t asks, armed, screen, usb, pcie, changes, gate[ARC_GATES];
	uint64_t collapse, collapse_gdsc, collapse_raised, ufs_off, gdsc_held[4], display_held, mmcx_held;
	uint64_t pcie0_retained;
} arc_count;
static uint32_t arc_cx = UINT32_MAX, arc_mmcx = UINT32_MAX, arc_collapse_on;
static uint32_t arc_sleep_cfg[ARC_MAX], arc_floor[ARC_MAX];
static char arc_text[4096];

static bool
safe_rd32(vm_offset_t addr, uint32_t *out)
{
	unsigned v = 0;

	if (addr == 0) {
		return false;
	}
	if (hakios_mmio_try_read32 != NULL) {
		if (hakios_mmio_try_read32((const volatile void *)addr, &v) != 0) {
			return false;
		}
		*out = v;
		return true;
	}
#ifdef HAKIOS_MMIO_TEST
	*out = rd(addr);
	return true;
#else
	return false;
#endif
}

static bool
arc_mm_off(void)
{
	const struct hakios_board *board = hakios_board();
	uint32_t bits = 0;

	if (!arc_mm_readable || hakios_display_off == NULL || !hakios_display_off()) {
		return false;
	}
	if (arc_dispcc != 0 && board != NULL && board->clocks.dispcc_mdss_gdscr != 0) {
		bits |= rd(arc_dispcc + board->clocks.dispcc_mdss_gdscr);
		if (board->clocks.dispcc_mdss_gdscr == 0x9000u) {
			bits |= rd(arc_dispcc + DISPCC_MDSS_INT2_GDSCR);
		}
	}
	if (arc_camcc != 0) {
		bits |= rd(arc_camcc + arc_camcc_off);
	}
	for (unsigned i = 0; i < 2; i++) {
		if (arc_videocc[i] != 0) {
			bits |= rd(arc_videocc[i] + arc_videocc_off[i]);
		}
	}
	return (bits & (1u << 31)) == 0;
}

static bool
rpmh_read(uint32_t address, uint32_t *value, uint32_t *status)
{
	struct hakios_rpmh_layout l = RSC_MAJOR(rd(rsc_base + RSC_DRV_ID)) >= 3 ? RPMH_LAYOUT_V3 : RPMH_LAYOUT_V2;
	uint32_t irq_enable = rd(SET_REG(0, 0x00u));
	bool done = false;

	*value = 0;
	*status = 0;
	for (int i = 0; rd(SET_REG(0, l.status)) == 0; i++) {
		if (i > 2000) {
			return false;
		}
		delay_for_interval(100, NSEC_PER_USEC);
	}
	wr(SET_REG(0, l.control), rd(SET_REG(0, l.control)) & ~(1u << 24));
	wr(SET_REG(0, l.control), rd(SET_REG(0, l.control)) & ~(1u << 16));
	wr(SET_REG(0, 0x08u), 1u);
	wr(SET_REG(0, l.msgid), 8u | 0x100u);
	wr(SET_REG(0, l.msgid + 4u), address);
	wr(SET_REG(0, l.msgid + 8u), 0);
	wr(SET_REG(0, l.msgid + 0x10u), 0);
	wr(SET_REG(0, l.wait), 1u);
	wr(SET_REG(0, l.cmd_enable), 1u);
	wr(SET_REG(0, 0x00u), irq_enable | 1u);
	wr(SET_REG(0, l.control), 1u << 16);
	wr(SET_REG(0, l.control), (1u << 16) | (1u << 24));
	for (int i = 0; i < 200 && !done; i++) {
		done = (rd(SET_REG(0, 0x04u)) & 1u) != 0;
		if (!done) {
			delay_for_interval(100, NSEC_PER_USEC);
		}
	}
	*status = rd(SET_REG(0, l.msgid + 0x0cu));
	*value = rd(SET_REG(0, l.msgid + 0x10u));
	wr(SET_REG(0, l.control), rd(SET_REG(0, l.control)) & ~(1u << 24));
	wr(SET_REG(0, l.control), rd(SET_REG(0, l.control)) & ~(1u << 16));
	wr(SET_REG(0, l.cmd_enable), 0);
	wr(SET_REG(0, l.wait), 0);
	wr(SET_REG(0, 0x08u), 1u);
	wr(SET_REG(0, 0x00u), irq_enable);
	return done && (*status & 0x10100u) == 0x10100u;
}

static const char *const rpmh_vote_names[] = {
	"cx.lvl", "mx.lvl", "mxc.lvl", "mmcx.lvl", "ebi.lvl", "gfx.lvl", "lcx.lvl", "lmx.lvl", "mss.lvl", "nsp.lvl",
	"xo.lvl", "qphy.lvl", "lnbclka1", "lnbclka2", "lnbclka3", "rfclka1", "rfclka2", "rfclka3", "rfclka4", "rfclka5",
	"rfclkd1", "rfclkd2", "rfclkd3", "rfclkd4", "clka1", "clka2", "clka3", "clka4", "clka5",
};

static int
rpmh_votes_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	static char text[4096];
	int at = 0;

	if (req->newptr != USER_ADDR_NULL) {
		return EPERM;
	}
	if (req->oldptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, text, sizeof(text));
	}
	if (rsc_base == 0) {
		return ENODEV;
	}
	at += snprintf(text + at, sizeof(text) - (size_t)at,
	    "this DRV's (APSS, 2) votes read back through TCS 0 (VRM clocks: +4, enable):\n");
	for (unsigned i = 0; i < sizeof(rpmh_vote_names) / sizeof(rpmh_vote_names[0]) && at < (int)sizeof(text) - 200; i++) {
		uint16_t levels[16];
		unsigned n, kind = 0;
		uint32_t address = rpmh_db_find(rpmh_vote_names[i], levels, &n, &kind), value, status;
		bool ok;

		if (address == 0) {
			at += snprintf(text + at, sizeof(text) - (size_t)at, "%s: not in cmd-db\n", rpmh_vote_names[i]);
			continue;
		}
		if (kind == CMD_DB_VRM) {
			address += 4u;
		}
		rsc_lock();
		ok = rpmh_read(address, &value, &status);
		rsc_unlock();
		at += snprintf(text + at, sizeof(text) - (size_t)at, "%s %#x (kind %u): %s %#x status %#x",
		    rpmh_vote_names[i], address, kind, ok ? "vote" : "NO ANSWER, read", value, status);
		for (unsigned l = 0; l < n && at < (int)sizeof(text) - 20; l++) {
			at += snprintf(text + at, sizeof(text) - (size_t)at, "%s%u", l ? " " : "; levels ", levels[l]);
		}
		at += snprintf(text + at, sizeof(text) - (size_t)at, "\n");
	}
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_votes, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, rpmh_votes_sysctl, "A",
    "diag: this DRV's ARC and RPMh clock votes read back from RPMh, and each ARC's levels");

static bool
gcc_map(void)
{
	const struct hakios_board *board = hakios_board();

	if (arc_gcc == 0 && board != NULL && board->clocks.gcc != 0) {
		arc_gcc = ml_io_map(board->clocks.gcc, GCC_SIZE_8450);
	}
	return arc_gcc != 0;
}

static int
gcc_on_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	static char text[8192];
	const struct hakios_board *board = hakios_board();
	unsigned on = 0, off = 0, unread = 0;
	int at = 0;

	if (req->newptr != USER_ADDR_NULL) {
		return EPERM;
	}
	if (req->oldptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, text, sizeof(text));
	}
	if (!gcc_map()) {
		return ENODEV;
	}
	at += snprintf(text + at, sizeof(text) - (size_t)at, "GCC branches running (halt register bit 31 clear):\n");
	for (unsigned i = 0; i < sizeof(gcc8450_branches) / sizeof(gcc8450_branches[0]); i++) {
		uint32_t v = 0;

		if (!safe_rd32(arc_gcc + gcc8450_branches[i].offset, &v)) {
			unread++;
			continue;
		}
		if (v & (1u << 31)) {
			off++;
			continue;
		}
		on++;
		if (at < (int)sizeof(text) - 100) {
			at += snprintf(text + at, sizeof(text) - (size_t)at, "%#07x %s %#x\n",
			    gcc8450_branches[i].offset, gcc8450_branches[i].name, v);
		}
	}
	at += snprintf(text + at, sizeof(text) - (size_t)at, "%u running, %u stopped, %u not readable\n", on, off, unread);
	for (unsigned i = 0; i < sizeof(gcc8450_gdscs) / sizeof(gcc8450_gdscs[0]); i++) {
		uint32_t v = rd(arc_gcc + gcc8450_gdscs[i].offset);

		at += snprintf(text + at, sizeof(text) - (size_t)at, "gdsc %#07x %s %s (%#x)\n", gcc8450_gdscs[i].offset,
		    gcc8450_gdscs[i].name, (v & (1u << 31)) ? "ON" : "off", v);
	}
	if (arc_dispcc != 0 && board != NULL && board->clocks.dispcc_mdss_gdscr != 0) {
		uint32_t v = rd(arc_dispcc + board->clocks.dispcc_mdss_gdscr);

		at += snprintf(text + at, sizeof(text) - (size_t)at, "gdsc dispcc %#x mdss_gdsc %s (%#x)\n",
		    board->clocks.dispcc_mdss_gdscr, (v & (1u << 31)) ? "ON" : "off", v);
		if (board->clocks.dispcc_mdss_gdscr == 0x9000u) {
			uint32_t w = rd(arc_dispcc + DISPCC_MDSS_INT2_GDSCR);
			at += snprintf(text + at, sizeof(text) - (size_t)at, "gdsc dispcc 0xb000 mdss_int2_gdsc %s (%#x)\n",
			    (w & (1u << 31)) ? "ON" : "off", w);
		}
	}
	if (arc_mm_readable) {
		if (arc_camcc != 0) {
			uint32_t c = rd(arc_camcc + arc_camcc_off);
			at += snprintf(text + at, sizeof(text) - (size_t)at, "gdsc camcc %#x titan_top_gdsc %s (%#x)\n",
			    board != NULL ? board->rpmh.camcc_gdsc : 0, (c & (1u << 31)) ? "ON" : "off", c);
		}
		for (unsigned i = 0; i < 2; i++) {
			if (arc_videocc[i] != 0) {
				uint32_t v = rd(arc_videocc[i] + arc_videocc_off[i]);
				at += snprintf(text + at, sizeof(text) - (size_t)at, "gdsc videocc[%u] %#x %s (%#x)\n",
				    i, board != NULL ? board->rpmh.videocc_gdsc[i] : 0, (v & (1u << 31)) ? "ON" : "off", v);
			}
		}
	}
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, gcc_on, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, gcc_on_sysctl, "A",
    "diag: the GCC's branch clocks that run now (gcc-sm8450.c's list)");

static void
arc_enable(uint32_t mask)
{
	wr(SET_REG(ARC_SLEEP_TCS, 0x1cu), mask);
	wr(SET_REG(ARC_WAKE_TCS, 0x1cu), mask & arc_wake_mask);
	ss_barrier();
	arc_live = mask;
}

void hakios_rpmh_llcc_down(void);

void
hakios_rpmh_llcc_down(void)
{
	uint32_t want = arc_mask;

	if (!__atomic_load_n(&arc_ready, __ATOMIC_ACQUIRE) || rsc_base == 0) {
		return;
	}
	arc_count.asks++;
	if (hakios_screen_off == NULL || !hakios_screen_off()) {
		want = 0;
		arc_count.screen++;
	} else if (floor_usb || !floor_usb_known) {
		want = 0;
		arc_count.usb++;
	} else if (__atomic_load_n(&pcie_bw_needed, __ATOMIC_RELAXED)) {
		want = 0;
		arc_count.pcie++;
	} else {
		for (uint32_t g = 0; g < arc_gates; g++) {
			if (arc_gcc != 0 && !(rd(arc_gcc + arc_gate[g]) & (1u << 31))) {
				want = 0;
				arc_count.gate[g]++;
				break;
			}
		}
	}
	if (want != 0 && arc_collapse_on && arc_cx < arc_n) {
		uint32_t v = 0;
		vm_offset_t data_reg = SET_REG(ARC_SLEEP_TCS, 0x38u + 20u * arc_cx);

		if (arc[arc_cx].sleep != arc_sleep_cfg[arc_cx]) {
			v = arc[arc_cx].sleep;
			arc_count.collapse_raised++;
		} else if (arc_mmcx >= arc_n || arc[arc_mmcx].wake == 0 || arc[arc_mmcx].wake != arc_floor[arc_mmcx] ||
		    arc[arc_mmcx].sleep != arc_sleep_cfg[arc_mmcx]) {
			v = arc[arc_cx].sleep;
			arc_count.mmcx_held++;
		} else {
			if (!arc_mm_off()) {
				v = arc[arc_cx].sleep;
				arc_count.display_held++;
			} else if (hakios_ufs_power_down != NULL && hakios_ufs_power_down()) {
				arc_count.ufs_off++;
			}
			for (uint32_t g = 0; g < ARC_COLLAPSE_GDSCS && v == 0; g++) {
				if (arc_gcc != 0 && (rd(arc_gcc + arc_collapse_gdsc[g]) & (1u << 31))) {
					if (arc_collapse_gdsc[g] == ARC_PCIE0_GDSC &&
					    hakios_pcie_drv_cx_ready != NULL && hakios_pcie_drv_cx_ready()) {
						arc_count.pcie0_retained++;
						continue;
					}
					v = arc[arc_cx].sleep;
					arc_count.collapse_gdsc++;
					arc_count.gdsc_held[g]++;
					break;
				}
			}
		}
		if (v == 0) {
			arc_count.collapse++;
		}
		if (rd(data_reg) != v) {
			wr(data_reg, v);
			ss_barrier();
		}
		if (arc_mmcx < arc_n) {
			vm_offset_t mm_reg = SET_REG(ARC_SLEEP_TCS, 0x38u + 20u * arc_mmcx);
			uint32_t m = v == 0 ? 0 : arc[arc_mmcx].sleep;

			if (rd(mm_reg) != m) {
				wr(mm_reg, m);
				ss_barrier();
			}
		}
	}
	if (want != arc_live) {
		arc_enable(want);
		arc_count.changes++;
	}
	if (want != 0) {
		arc_count.armed++;
	}
}

static void
arc_disarm(void)
{
	__atomic_store_n(&arc_ready, false, __ATOMIC_RELEASE);
	if (rsc_base != 0) {
		arc_enable(0);
	}
}

static uint32_t
arc_number(const char *s, const char **end)
{
	uint32_t v = 0, base = 10;

	if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
		base = 16;
		s += 2;
	}
	for (;; s++) {
		uint32_t d = (*s >= '0' && *s <= '9') ? (uint32_t)(*s - '0') :
		    (*s >= 'a' && *s <= 'f') ? (uint32_t)(*s - 'a' + 10) :
		    (*s >= 'A' && *s <= 'F') ? (uint32_t)(*s - 'A' + 10) : 99u;

		if (d >= base) {
			break;
		}
		v = v * base + d;
	}
	*end = s;
	return v;
}

static void
arc_describe(void)
{
	int at = 0;

	at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at,
	    "ready %u entries %u live %#x (sleep TCS %u, wake TCS %u)\n",
	    arc_ready, arc_n, arc_live, ARC_SLEEP_TCS, ARC_WAKE_TCS);
	for (uint32_t i = 0; i < arc_n; i++) {
		at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at, "entry %s %#x sleep %#x wake %#x%s\n",
		    arc[i].name, arc[i].address, arc[i].sleep, arc[i].wake,
		    arc[i].sleep_only ? " (sleep only: not restored)" : "");
	}
	at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at,
	    "llcc-off asks %llu armed %llu changes %llu; held by screen %llu usb %llu pcie %llu\n",
	    arc_count.asks, arc_count.armed, arc_count.changes, arc_count.screen, arc_count.usb, arc_count.pcie);
	at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at,
	    "cx collapse %s: armed %llu; held by a power domain on %llu, a driver's vote %llu, MMCX not droppable %llu, "
	    "the display's/camera's/video's hardware on %llu; UFS's domain off %llu; held by usb %llu pcie0 %llu pcie1 %llu ufs %llu; "
	    "MMCX entry %s, its blocks readable %u\n",
	    (arc_collapse_on && arc_cx < arc_n) ? "on" : "off", arc_count.collapse, arc_count.collapse_gdsc,
	    arc_count.collapse_raised, arc_count.mmcx_held, arc_count.display_held, arc_count.ufs_off,
	    arc_count.gdsc_held[0], arc_count.gdsc_held[1], arc_count.gdsc_held[2], arc_count.gdsc_held[3],
	    arc_mmcx < arc_n ? "in the sleep set" : "NONE", arc_mm_readable);
	at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at, "pcie0 retained in DRV %llu\n",
	    arc_count.pcie0_retained);
	for (uint32_t g = 0; g < arc_gates && at < (int)sizeof(arc_text) - 120; g++) {
		const char *name = "?";

		for (unsigned i = 0; i < sizeof(gcc8450_branches) / sizeof(gcc8450_branches[0]); i++) {
			if (gcc8450_branches[i].offset == arc_gate[g]) {
				name = gcc8450_branches[i].name;
			}
		}
		at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at, "gate %#07x %s: held %llu, now %s\n",
		    arc_gate[g], name, arc_count.gate[g],
		    (arc_gcc != 0 && !(rd(arc_gcc + arc_gate[g]) & (1u << 31))) ? "RUNNING" : "stopped");
	}
}

static int
arc_input(struct sysctl_req *req, char *in, size_t size)
{
	size_t len = req->newlen;
	int error;

	if (len >= size) {
		return EINVAL;
	}
	error = SYSCTL_IN(req, in, len);
	in[error == 0 ? len : 0] = 0;
	return error;
}

static int
arc_configure(const char *spec)
{
	const char *s = spec;
	uint32_t n = 0;
	int error = 0;

	if (rsc_base == 0 || !gcc_map()) {
		return ENODEV;
	}
	arc_disarm();
	rsc_lock();
	arc_cx = UINT32_MAX;
	arc_mmcx = UINT32_MAX;
	while (*s != 0 && error == 0) {
		char name[12] = { 0 };
		uint16_t levels[16];
		unsigned nlevels, kind = 0, k = 0;
		uint32_t address, value, status;

		while (*s == ',' || *s == ' ' || *s == '\n') {
			s++;
		}
		if (*s == 0) {
			break;
		}
		while (*s != '=' && *s != 0 && k < sizeof(name) - 1u) {
			name[k++] = *s++;
		}
		if (*s++ != '=' || n == ARC_MAX || (address = rpmh_db_find(name, levels, &nlevels, &kind)) == 0) {
			error = EINVAL;
			break;
		}
		if (kind == CMD_DB_VRM) {
			address += 4u;
		}
		if (*s == '@') {
			uint32_t corner = arc_number(s + 1, &s);
			unsigned l = 0;

			while (l < nlevels && levels[l] < corner) {
				l++;
			}
			if (l == nlevels) {
				error = ERANGE;
				break;
			}
			arc[n].sleep = l;
		} else {
			arc[n].sleep = arc_number(s, &s);
		}
		arc[n].sleep_only = *s == '!';
		if (*s == '!') {
			s++;
		}
		if (!rpmh_read(address, &value, &status)) {
			tell("rpmh: %s (%#x): its vote cannot be read back (status %#x): not put in the sleep set\n",
			    name, address, status);
			error = EIO;
			break;
		}
		strlcpy(arc[n].name, name, sizeof(arc[n].name));
		arc[n].address = address;
		arc[n].wake = value;
		arc_floor[n] = value;
		arc_sleep_cfg[n] = arc[n].sleep;
		if (strcmp(name, "cx.lvl") == 0 && !arc[n].sleep_only && arc[n].sleep != 0) {
			arc_cx = n;
		}
		if (strcmp(name, "mmcx.lvl") == 0 && !arc[n].sleep_only) {
			arc[n].sleep = arc_sleep_cfg[n] = value;
			arc_mmcx = n;
		}
		n++;
	}
	if (error == 0) {
		for (unsigned t = ARC_SLEEP_TCS; t <= ARC_WAKE_TCS; t += ARC_WAKE_TCS - ARC_SLEEP_TCS) {
			for (uint32_t i = 0; i < n; i++) {
				wr(SET_REG(t, 0x30u + 20u * i), 0x10008u | 0x100u);
				wr(SET_REG(t, 0x34u + 20u * i), arc[i].address);
				wr(SET_REG(t, 0x38u + 20u * i), t == ARC_SLEEP_TCS ? arc[i].sleep : arc[i].wake);
			}
		}
		arc_n = n;
		arc_mask = n ? (1u << n) - 1u : 0;
		arc_wake_mask = 0;
		for (uint32_t i = 0; i < n; i++) {
			arc_wake_mask |= arc[i].sleep_only ? 0 : (1u << i);
		}
		bzero(&arc_count, sizeof(arc_count));
	} else {
		arc_n = 0;
		arc_mask = 0;
	}
	rsc_unlock();
	if (error == 0 && n != 0) {
		__atomic_store_n(&arc_ready, true, __ATOMIC_RELEASE);
	}
	tell("rpmh: ARC sleep entries: %u (%s)\n", arc_n, error ? "refused" : n ? "armed at llcc-off" : "cleared");
	return error;
}

static int
rpmh_sleep_arc_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char in[256];
	int error;

	if (req->newptr != USER_ADDR_NULL) {
		if ((error = arc_input(req, in, sizeof(in))) != 0 || (error = arc_configure(in)) != 0) {
			return error;
		}
	}
	arc_describe();
	return SYSCTL_OUT(req, arc_text, strlen(arc_text) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_sleep_arc, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED, 0, 0,
    rpmh_sleep_arc_sysctl, "A",
    "diag: name=value,... ARC/clock commands in sleep TCS 4 (wake TCS 6: the vote read back), armed at llcc-off; \"\" clears");

static int
rpmh_sleep_gate_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char in[256];
	int error;

	if (req->newptr != USER_ADDR_NULL) {
		const char *s = in;
		uint32_t gates[ARC_GATES], n = 0;
		bool was = __atomic_load_n(&arc_ready, __ATOMIC_ACQUIRE);

		if ((error = arc_input(req, in, sizeof(in))) != 0) {
			return error;
		}
		if (!gcc_map()) {
			return ENODEV;
		}
		while (*s != 0) {
			uint32_t v = 0, off;

			while (*s == ',' || *s == ' ' || *s == '\n') {
				s++;
			}
			if (*s == 0) {
				break;
			}
			off = arc_number(s, &s);
			if (n == ARC_GATES || off >= GCC_SIZE_8450 || (off & 3u) != 0 || !safe_rd32(arc_gcc + off, &v)) {
				return EINVAL;
			}
			gates[n++] = off;
		}
		arc_disarm();
		memcpy(arc_gate, gates, sizeof(gates[0]) * n);
		arc_gates = n;
		bzero(arc_count.gate, sizeof(arc_count.gate));
		if (was && arc_n != 0) {
			__atomic_store_n(&arc_ready, true, __ATOMIC_RELEASE);
		}
	}
	arc_describe();
	return SYSCTL_OUT(req, arc_text, strlen(arc_text) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_sleep_gate, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED, 0, 0,
    rpmh_sleep_gate_sysctl, "A",
    "diag: GCC branch offsets that must be stopped for the ARC sleep entries to be armed");

void hakios_rpmh_arc_active(uint32_t address, uint32_t value);

void
hakios_rpmh_arc_active(uint32_t address, uint32_t value)
{
	for (uint32_t i = 0; i < arc_n; i++) {
		if (arc[i].address == address) {
			arc[i].wake = MAX(arc_floor[i], value);
			arc[i].sleep = (value > arc_floor[i]) ? value : arc_sleep_cfg[i];
			wr(SET_REG(ARC_WAKE_TCS, 0x38u + 20u * i), arc[i].wake);
			wr(SET_REG(ARC_SLEEP_TCS, 0x38u + 20u * i), arc[i].sleep);
			ss_barrier();
		}
	}
}

#define ARC_DEFAULT             "xo.lvl=0,mmcx.lvl=0,cx.lvl=@16"
#define ARC_DEFAULT_NO_MMCX     "xo.lvl=0,cx.lvl=@16"
#define ARC_DEFAULT_GATES       "0x87018,0x87064,0x870d4,0x49018,0x7b03c,0x7b028,0x24004,0x3300c,0x3314c,0x3328c"

static void
arc_default_start(void)
{
	const struct hakios_board *board = hakios_board();
	uint32_t on = 1;
	const char *s = ARC_DEFAULT_GATES;
	int error;

	PE_parse_boot_argn("rpmh_sleep_arc", &on, sizeof(on));
	arc_collapse_on = 1;
	PE_parse_boot_argn("rpmh_cx_collapse", &arc_collapse_on, sizeof(arc_collapse_on));
	if (arc_collapse_on && board != NULL) {
		uint32_t r = 0;
		bool ok = true;

		if (board->clocks.dispcc != 0) {
			arc_dispcc = ml_io_map(board->clocks.dispcc, DISPCC_SIZE);
		}
		if (board->rpmh.camcc_gdsc != 0) {
			arc_camcc = ml_io_map(board->rpmh.camcc_gdsc & ~0xfffu, 0x1000);
			arc_camcc_off = board->rpmh.camcc_gdsc & 0xfffu;
		}
		for (unsigned i = 0; i < 2; i++) {
			uint32_t g = board->rpmh.videocc_gdsc[i];

			if (g != 0) {
				if (i == 1 && (g & ~0xfffu) == (board->rpmh.videocc_gdsc[0] & ~0xfffu)) {
					arc_videocc[1] = arc_videocc[0];
				} else {
					arc_videocc[i] = ml_io_map(g & ~0xfffu, 0x1000);
				}
				arc_videocc_off[i] = g & 0xfffu;
			}
		}
		if (arc_dispcc == 0 || board->clocks.dispcc_mdss_gdscr == 0 ||
		    !safe_rd32(arc_dispcc + board->clocks.dispcc_mdss_gdscr, &r)) {
			ok = false;
		}
		if (board->rpmh.camcc_gdsc != 0 && (arc_camcc == 0 || !safe_rd32(arc_camcc + arc_camcc_off, &r))) {
			ok = false;
		}
		for (unsigned i = 0; i < 2; i++) {
			if (board->rpmh.videocc_gdsc[i] != 0 &&
			    (arc_videocc[i] == 0 || !safe_rd32(arc_videocc[i] + arc_videocc_off[i], &r))) {
				ok = false;
			}
		}
		arc_mm_readable = ok;
	}
	if (on == 0 || !gcc_map()) {
		tell("rpmh: ARC sleep entries: off (rpmh_sleep_arc=%u)\n", on);
		return;
	}
	arc_gates = 0;
	while (*s != 0 && arc_gates < ARC_GATES) {
		arc_gate[arc_gates++] = arc_number(s, &s);
		while (*s == ',') {
			s++;
		}
	}
	error = arc_configure(ARC_DEFAULT);
	if (error != 0) {
		tell("rpmh: ARC sleep entries with mmcx.lvl refused (%d): without it, and so no CX collapse\n", error);
		error = arc_configure(ARC_DEFAULT_NO_MMCX);
	}
	tell("rpmh: ARC sleep entries by default (%s, %u gates): %s; CX collapse with MMCX while the display's hardware is off: %s "
	    "(MMCX vote %#x; its blocks' domains readable %u)\n",
	    arc_mmcx < arc_n ? ARC_DEFAULT : ARC_DEFAULT_NO_MMCX, arc_gates,
	    error ? "REFUSED" : "on", (arc_collapse_on && arc_mmcx < arc_n) ? "on" : "off",
	    arc_mmcx < arc_n ? arc[arc_mmcx].wake : 0, arc_mm_readable);
}

/* ---- AOP sleep statistics, DDR statistics, CX monitor, and 30 s power log ---------------------- */

static const char *const vx_drv[VX_DRVS] = {
	"TZ", "HYP", "HLOS", "L3", "SECPROC", "AUDIO", "SENSOR", "AOP", "DEBUG", "GPU",
	"DISPLAY", "COMPUTE_DSP", "TME_SW", "TME_HW", "MDM_SW", "MDM_HW", "WLAN_RF", "WLAN_BB", "DDR_AUX", "ARC_CPRF",
};

static vm_offset_t ss_window, ss_vx;
static uint32_t ss_phys_base, ss_rec_off, ss_ddr_off, ss_ddr_n;
static bool ss_started, ss_vx_asked;
static char ss_text[8192];
static int ss_at;

static struct ss_sample {
	uint32_t t;
	int32_t ma, mv, charger;
	uint32_t count[2];
	uint64_t ticks[2];
	uint64_t freq_ticks[LOG_FREQS];
} ss_log[LOG_SAMPLES];
static uint32_t ss_logged, ss_freq_mhz[LOG_FREQS], ss_freqs;
static char ss_log_text[LOG_SAMPLES * 96 + 512];

static bool
ss_rd(vm_offset_t base, uint32_t offset, uint32_t *value)
{
	return base != 0 && safe_rd32(base + offset, value);
}

static bool
ss_rd64(vm_offset_t base, uint32_t offset, uint64_t *value)
{
	uint32_t lo = 0, hi = 0;

	if (!ss_rd(base, offset, &lo) || !ss_rd(base, offset + 4u, &hi)) {
		return false;
	}
	*value = ((uint64_t)hi << 32) | lo;
	return true;
}

static void ss_say(const char *format, ...) __printflike(1, 2);

static void
ss_say(const char *format, ...)
{
	va_list list;

	if (ss_at >= (int)sizeof(ss_text) - 1) {
		return;
	}
	va_start(list, format);
	ss_at += vsnprintf(ss_text + ss_at, sizeof(ss_text) - (size_t)ss_at, format, list);
	va_end(list);
	if (ss_at > (int)sizeof(ss_text) - 1) {
		ss_at = (int)sizeof(ss_text) - 1;
	}
}

static uint32_t
ss_region(uint32_t word_at, uint32_t size)
{
	uint32_t word = 0;

	if (!ss_rd(ss_window, word_at, &word) || ss_phys_base == 0) {
		return 0;
	}
	word = (ss_phys_base | word) - ss_phys_base;
	if (word == 0 || (word & 3u) != 0 || word > SS_WINDOW - size) {
		return 0;
	}
	return word;
}

static void
ss_map(void)
{
	const struct hakios_board *board = hakios_board();
	uint32_t magic = 0, n = 0;

	if (ss_started || board == NULL) {
		return;
	}
	ss_started = true;
	ss_phys_base = board->rpmh.sleep_stats;
	if (ss_phys_base != 0) {
		ss_window = ml_io_map(ss_phys_base, SS_WINDOW);
	}
	if (board->rpmh.cx_monitor != 0) {
		ss_vx = ml_io_map(board->rpmh.cx_monitor, PAGE_SIZE);
	}
	if (rsc_base == 0 && board->rpmh.rsc != 0) {
		rsc_base = ml_io_map(board->rpmh.rsc, 0x10000);
		rsc_tcs_off = board->rpmh.tcs_offset;
	}
	ss_rec_off = ss_region(0x4u, SS_RECORDS * SS_RECORD);
	ss_ddr_off = ss_region(0x1cu, DDR_SIZE);
	if (ss_ddr_off != 0 && ss_rd(ss_window, ss_ddr_off, &magic) && ss_rd(ss_window, ss_ddr_off + 4u, &n) &&
	    magic == DDR_MAGIC && n > 0 && n <= DDR_MAX_MODES) {
		ss_ddr_n = n;
	} else {
		ss_ddr_off = 0;
	}
	tell("rpmh-stats: sleep records at %#x%s, DDR statistics at %#x%s (%u entries)\n",
	    ss_phys_base + ss_rec_off, ss_rec_off ? "" : " NOT READABLE",
	    ss_phys_base + ss_ddr_off, ss_ddr_off ? "" : " NOT READABLE", ss_ddr_n);
}

static void
ss_resource(uint32_t address, char *out, size_t size)
{
	uint32_t db_size = 0;
	const struct cmd_db_header *db = cmd_db_map(&db_size);

	out[0] = 0;
	if (db == NULL || !hakios_cmd_db_valid(db)) {
		return;
	}
	for (unsigned pass = 0; pass < 2; pass++) {
		uint32_t best = 0;
		const struct cmd_db_entry *best_entry = NULL;

		for (unsigned s = 0; s < CMD_DB_SLAVES; s++) {
			const struct cmd_db_entry *entry;

			for (unsigned i = 0; (entry = hakios_cmd_db_entry(db, db_size, s, i)) != NULL; i++) {
				uint32_t base = entry->address;

				if (pass == 0 ? (base == address) : (base < address && address - base < 0x10u && base > best)) {
					best = base;
					best_entry = entry;
					if (pass == 0) {
						break;
					}
				}
			}
			if (pass == 0 && best_entry != NULL) {
				break;
			}
		}
		if (best_entry != NULL) {
			char id[9] = { 0 };

			memcpy(id, best_entry->id, 8);
			if (address == best) {
				snprintf(out, size, "%s", id);
			} else {
				snprintf(out, size, "%s+%u", id, (unsigned)((address - best) & 0xfu));
			}
			return;
		}
	}
}

static bool
ss_record(int i, char name[5], uint32_t *count, uint64_t *ticks, bool *in)
{
	uint32_t b = ss_rec_off + (uint32_t)i * SS_RECORD, type = 0;
	uint64_t entered = 0, exited = 0;

	if (ss_rec_off == 0 || !ss_rd(ss_window, b, &type) || !ss_rd(ss_window, b + 4u, count) ||
	    !ss_rd64(ss_window, b + 0x8u, &entered) || !ss_rd64(ss_window, b + 0x10u, &exited) ||
	    !ss_rd64(ss_window, b + 0x18u, ticks)) {
		return false;
	}
	for (int k = 0; k < 4; k++) {
		char c = (char)(type >> (8 * k));
		name[k] = (c >= 0x20 && c < 0x7f) ? c : '.';
	}
	name[4] = 0;
	*in = entered > exited;
	return true;
}

static bool
ss_ddr_entry(uint32_t i, uint32_t *name, uint32_t *count, uint64_t *ticks)
{
	uint32_t b = ss_ddr_off + 8u + 16u * i;

	return ss_ddr_off != 0 && ss_rd(ss_window, b, name) && ss_rd(ss_window, b + 4u, count) &&
	       ss_rd64(ss_window, b + 8u, ticks);
}

static void
ss_aop_say(const char *text, uint32_t settle_ms)
{
	if (hakios_aop_message != NULL && hakios_aop_message(text)) {
		delay_for_interval(settle_ms, NSEC_PER_MSEC);
	} else {
		ss_say("(the AOP did not take \"%s\")\n", text);
	}
}

static void
ss_snapshot(void)
{
	uint64_t now_ns = 0, ticks = 0, total = 0;
	char name[5], res[16];
	uint32_t count = 0, word = 0, w0 = 0, w1 = 0;
	bool in = false;

	ss_at = 0;
	ss_text[0] = 0;
	ss_map();
	absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
	ss_say("uptime %llu ms\n", now_ns / 1000000ull);

	for (int i = 0; i < SS_RECORDS; i++) {
		if (!ss_record(i, name, &count, &ticks, &in)) {
			ss_say("sleep record %d: not readable\n", i);
			continue;
		}
		ss_say("sleep %s: count %u, time %llu ms%s\n", name, count, ticks / AOP_TICKS_PER_MS, in ? " (in it now)" : "");
	}

	if (ss_ddr_off != 0) {
		ss_aop_say("{class: ddr, action: freqsync}", 20);
		for (uint32_t i = 0; i < ss_ddr_n; i++) {
			uint32_t n = 0;
			if (ss_ddr_entry(i, &n, &count, &ticks) && ((n >> 8) & 0xffu) == 1u) {
				total += ticks;
			}
		}
		for (uint32_t i = 0; i < ss_ddr_n; i++) {
			uint32_t n = 0;
			if (!ss_ddr_entry(i, &n, &count, &ticks)) {
				ss_say("ddr entry %u: not readable\n", i);
			} else if (ticks == 0 && ((n >> 8) & 0xffu) != 0) {
				continue;
			} else if (((n >> 8) & 0xffu) == 0) {
				ss_say("ddr lpm %#x: count %u, time %llu ms\n", n & 0xffu, count, ticks / AOP_TICKS_PER_MS);
			} else if (((n >> 8) & 0xffu) == 1u && (n >> 16) != 0) {
				ss_say("ddr freq %u MHz (cp %u): count %u, time %llu ms (%llu%%)\n",
				    n >> 16, n & 0x1fu, count, ticks / AOP_TICKS_PER_MS, total ? ticks * 100ull / total : 0ull);
			} else {
				ss_say("ddr entry %u: name %#x, count %u, time %llu ms\n", i, n, count, ticks / AOP_TICKS_PER_MS);
			}
		}
		ss_aop_say("{class: ddr, res: drvs_ddr_votes}", 20);
		ss_say("ddr votes by DRV (ab/ib; names as sys_pm_vx's [I]):");
		for (uint32_t i = 0; i < DDR_DRVS; i++) {
			if (!ss_rd(ss_window, ss_ddr_off + 8u + 16u * ss_ddr_n + 4u * i, &word)) {
				ss_say(" %u:?", i);
			} else if (word == 0xdeaddeadU) {
				ss_say(" %u(%s):absent", i, vx_drv[i]);
			} else if (word == 0xffffdeadU) {
				ss_say(" %u(%s):invalid", i, vx_drv[i]);
			} else {
				ss_say(" %u(%s):%u/%u", i, vx_drv[i], (word >> 14) & 0x3fffu, word & 0x3fffu);
			}
		}
		ss_say("\n");
	}

	if (!ss_vx_asked && ss_vx != 0) {
		ss_vx_asked = true;
		ss_aop_say("{class: lpm_mon, type: cxpc, dur: 1000, flush: 5, ts_adj: 1}", 20);
		ss_say("cx monitor: started now; read again later\n");
	}
	if (!ss_rd(ss_vx, 0, &w0) || !ss_rd(ss_vx, 4u, &w1)) {
		ss_say("cx monitor: not readable\n");
	} else {
		uint32_t logsize = (w0 >> 8) & 0xffu, shift = (w1 >> 16) & 0xffu, lines = 0, marks = 0;
		uint32_t held[VX_DRVS] = { 0 }, top[VX_DRVS] = { 0 }, at = 8u;

		ss_say("cx monitor: mode %#x, %u ms a line, time shift %u, flush %u, up to %u lines\n",
		    w0 & 0xffu, w1 & 0xffffu, shift, w1 >> 24, logsize);
		for (uint32_t l = 0; l < logsize && at + 4u * (1u + VX_WORDS) <= VX_SIZE; l++) {
			uint32_t ts = 0, v[VX_DRVS], any = 0;

			if (!ss_rd(ss_vx, at, &ts) || ts == 0) {
				break;
			}
			at += 4u;
			for (uint32_t k = 0; k < VX_WORDS; k++, at += 4u) {
				uint32_t word4 = 0;
				(void)ss_rd(ss_vx, at, &word4);
				for (uint32_t j = 0; j < 4u && 4u * k + j < VX_DRVS; j++) {
					v[4u * k + j] = (word4 >> (8u * j)) & 0xffu;
					any |= v[4u * k + j];
				}
			}
			lines++;
			if (!any) {
				marks++;
			}
			for (uint32_t j = 0; j < VX_DRVS; j++) {
				held[j] += v[j] != 0;
				top[j] = MAX(top[j], v[j]);
			}
			if (l < 6u) {
				ss_say("cx line %u at %#x:", l, ts << shift);
				for (uint32_t j = 0; j < VX_DRVS; j++) {
					if (v[j]) {
						ss_say(" %s=%u", vx_drv[j], v[j]);
					}
				}
				ss_say(any ? "\n" : " (collapse entry/exit)\n");
			}
		}
		ss_say("cx monitor: %u lines, %u all-zero; held CX in (lines, most):", lines, marks);
		for (uint32_t j = 0; j < VX_DRVS; j++) {
			if (held[j]) {
				ss_say(" %s %u/%u%s", vx_drv[j], held[j], top[j], held[j] == lines ? " (every line)" : "");
			}
		}
		ss_say("\n");
	}

	ss_say("floor: %s\n", floor_line);
	lck_mtx_lock(&keep_lock);
	for (unsigned k = 0; k < keep_n; k++) {
		static const char *const owners[KEEP_CLIENTS] = { "cpu", "touch", "display", "usb", "pcie" };

		ss_say("keep %.8s %#x (unit %u width %u): sent %#x;",
		    keep[k].name, keep[k].address, keep[k].unit, keep[k].width, keep[k].sent);
		for (unsigned c = 0; c < KEEP_CLIENTS; c++) {
			if (keep[k].ab[c] || keep[k].ib[c]) {
				ss_say(" client %u(%s) %u/%u", c, owners[c], keep[k].ab[c], keep[k].ib[c]);
			}
		}
		ss_say("\n");
	}
	lck_mtx_unlock(&keep_lock);

	ss_say("rsc descriptors (not live votes; active may be stale):\n");
	for (unsigned t = 0; rsc_base != 0 && t < RSC_TCS_COUNT; t++) {
		uint32_t enabled = rd(SET_REG(t, 0x1cu));

		for (unsigned c = 0; c < RSC_CMDS; c++) {
			uint32_t msgid = rd(SET_REG(t, 0x30u + 20u * c));
			uint32_t address = rd(SET_REG(t, 0x34u + 20u * c));
			uint32_t data = rd(SET_REG(t, 0x38u + 20u * c));

			if ((address == 0 && data == 0) || (t < 3u && !(enabled & (1u << c)))) {
				continue;
			}
			ss_resource(address, res, sizeof(res));
			if (msgid == 0x10108u && t >= 3u) {
				ss_say("rsc tcs %u (%s) cmd %u%s: %s %#x = %#x\n",
				    t, t < 5u ? "sleep" : t < 7u ? "wake" : "fast", c,
				    (enabled & (1u << c)) ? " on" : "", res[0] ? res : "?", address, data);
				continue;
			}
			ss_say("rsc tcs %u (%s) cmd %u%s: %s %#x = %#x (msg %#x)\n", t,
			    t < 3u ? "active" : t < 5u ? "sleep" : t < 7u ? "wake" : "fast", c,
			    (enabled & (1u << c)) ? " on" : "", res[0] ? res : "?", address, data, msgid);
		}
	}
	ss_say("rsc: end (slots not listed are empty)\n");
}

static void
ss_sample(void)
{
	struct ss_sample *s = &ss_log[ss_logged % LOG_SAMPLES];
	uint64_t now_ns = 0;
	char name[5];
	bool in = false;

	ss_map();
	bzero(s, sizeof(*s));
	absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
	s->t = (uint32_t)(now_ns / NSEC_PER_SEC);
	if (hakios_battery_power != NULL) {
		(void)hakios_battery_power(&s->ma, &s->mv, &s->charger);
	}
	for (int i = 0; i < 2; i++) {
		(void)ss_record(i, name, &s->count[i], &s->ticks[i], &in);
	}
	if (ss_ddr_off != 0 && hakios_aop_message != NULL && hakios_aop_message("{class: ddr, action: freqsync}")) {
		delay_for_interval(20, NSEC_PER_MSEC);
		for (uint32_t i = 0; i < ss_ddr_n; i++) {
			uint32_t n = 0, count = 0, f;
			uint64_t ticks = 0;

			if (!ss_ddr_entry(i, &n, &count, &ticks) || ((n >> 8) & 0xffu) != 1u || (n >> 16) == 0) {
				continue;
			}
			for (f = 0; f < ss_freqs && ss_freq_mhz[f] != (n >> 16); f++) {
			}
			if (f == ss_freqs && ss_freqs < LOG_FREQS) {
				ss_freq_mhz[ss_freqs++] = n >> 16;
			}
			if (f < LOG_FREQS) {
				s->freq_ticks[f] = ticks;
			}
		}
	}
	ss_logged++;
}

static void
ss_thread(__unused void *arg, __unused wait_result_t wr)
{
	thread_set_thread_name(current_thread(), "rpmh-stats");
	for (;;) {
		lck_mtx_lock(&ss_lock);
		ss_sample();
		lck_mtx_unlock(&ss_lock);
		assert_wait_timeout((event_t)&ss_logged, THREAD_UNINT, LOG_SECONDS, NSEC_PER_SEC);
		thread_block(THREAD_CONTINUE_NULL);
	}
}

static int
rpmh_stats_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int error;

	if (!hakios_board_has(HAKIOS_HAS_SOC_SLEEP)) {
		return ENODEV;
	}
	if (req->oldptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, ss_text, sizeof(ss_text));
	}
	lck_mtx_lock(&ss_lock);
	ss_snapshot();
	error = SYSCTL_OUT(req, ss_text, strlen(ss_text) + 1);
	lck_mtx_unlock(&ss_lock);
	return error;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, rpmh_stats_sysctl, "A",
    "the AOP's sleep records (aosd, cxsd), DDR's time per frequency and votes per DRV, who held CX, the APSS RSC's slots");

static void
ss_log_describe(void)
{
	uint32_t first = ss_logged > LOG_SAMPLES ? ss_logged - LOG_SAMPLES : 0;
	int at = 0;
	int64_t off_ma = 0;
	uint32_t off_n = 0;

	at += snprintf(ss_log_text + at, sizeof(ss_log_text) - (size_t)at,
	    "every %u s: t(s) mA mV charger | aosd +n +ms | cxsd +n +ms | ddr: MHz of most time (%% of it)\n", LOG_SECONDS);
	for (uint32_t k = first + 1; k < ss_logged && at < (int)sizeof(ss_log_text) - 100; k++) {
		const struct ss_sample *a = &ss_log[(k - 1u) % LOG_SAMPLES], *b = &ss_log[k % LOG_SAMPLES];
		uint64_t grew = 0, best = 0;
		uint32_t best_mhz = 0;

		for (uint32_t f = 0; f < ss_freqs; f++) {
			uint64_t d = b->freq_ticks[f] >= a->freq_ticks[f] ? b->freq_ticks[f] - a->freq_ticks[f] : 0;
			grew += d;
			if (d > best) {
				best = d;
				best_mhz = ss_freq_mhz[f];
			}
		}
		at += snprintf(ss_log_text + at, sizeof(ss_log_text) - (size_t)at,
		    "%u %d %d %d | %u %llu | %u %llu | %u (%llu%%)\n",
		    b->t, b->ma, b->mv, b->charger, b->count[0] - a->count[0], (b->ticks[0] - a->ticks[0]) / AOP_TICKS_PER_MS,
		    b->count[1] - a->count[1], (b->ticks[1] - a->ticks[1]) / AOP_TICKS_PER_MS, best_mhz,
		    grew ? best * 100ull / grew : 0ull);
		if (!b->charger) {
			off_ma += b->ma;
			off_n++;
		}
	}
	if (at < (int)sizeof(ss_log_text) - 100) {
		snprintf(ss_log_text + at, sizeof(ss_log_text) - (size_t)at,
		    "%u samples; without a charger: %u, mean %lld mA\n",
		    ss_logged > first ? ss_logged - first : 0u, off_n, off_n ? off_ma / (int64_t)off_n : 0ll);
	}
}

static int
rpmh_log_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int error;

	if (!hakios_board_has(HAKIOS_HAS_SOC_SLEEP)) {
		return ENODEV;
	}
	if (req->oldptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, ss_log_text, sizeof(ss_log_text));
	}
	lck_mtx_lock(&ss_lock);
	ss_log_describe();
	error = SYSCTL_OUT(req, ss_log_text, strlen(ss_log_text) + 1);
	lck_mtx_unlock(&ss_lock);
	return error;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_log, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, rpmh_log_sysctl, "A",
    "every 30 s: battery mA/mV/charger, aosd and cxsd growth, the DDR frequency the time went to");

static void
ss_start(void)
{
	thread_t thread;

	if (kernel_thread_start(ss_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

static void
keep_start(void)
{
	const struct hakios_board *board = hakios_board();
	uint32_t on = 1, sets = 1;

	PE_parse_boot_argn("ddr_floor", &on, sizeof(on));
	PE_parse_boot_argn("rpmh_sets", &sets, sizeof(sets));
	PE_parse_boot_argn("rpmh_sleep_probe", &keep_probe_mhz, sizeof(keep_probe_mhz));
	keep_probe_mhz = MIN(keep_probe_mhz, 3196u);
	rsc_base = ml_io_map(board->rpmh.rsc, 0x10000);
	rsc_tcs_off = board->rpmh.tcs_offset;
	if (rsc_base == 0) {
		return;
	}
	lck_mtx_lock(&keep_lock);
	keep_mc0 = keep_slot("MC0");
	keep_sh0 = keep_slot("SH0");
	keep_acv = keep_slot("ACV");
	for (unsigned i = 0; i < KEEP_LOADER; i++) {
		keep_loader_slot[i] = keep_slot(keep_loader[i].bcm);
	}
	lck_mtx_unlock(&keep_lock);
	tell("ddr: rsc drv 2 config %#x (tcs %u, cmds %u), solver %#x\n",
	    rd(rsc_base + 0x0cu), (rd(rsc_base + 0x0cu) >> 12) & 0x3fu,
	    (rd(rsc_base + 0x0cu) >> 27) & 0x1fu, rd(rsc_base + 0x04u));
	tell("ddr: MC0 unit %u width %u, SH0 unit %u width %u: DDR 547/2092/3196 MHz = %u/%u/%u, LLCC 300/806 = %u/%u\n",
	    keep_mc0 >= 0 ? keep[keep_mc0].unit : 0, keep_mc0 >= 0 ? keep[keep_mc0].width : 0,
	    keep_sh0 >= 0 ? keep[keep_sh0].unit : 0, keep_sh0 >= 0 ? keep[keep_sh0].width : 0,
	    keep_mhz(keep_mc0, 547), keep_mhz(keep_mc0, 2092), keep_mhz(keep_mc0, 3196),
	    keep_mhz(keep_sh0, 300), keep_mhz(keep_sh0, 806));
	if (!on || keep_mc0 < 0 || keep_sh0 < 0 || keep[keep_mc0].unit == 0 || keep[keep_sh0].unit == 0) {
		floor_describe();
		tell("ddr: no floor (ddr_floor=%u, MC0 %d, SH0 %d): the boot loader's votes stay\n", on, keep_mc0, keep_sh0);
		return;
	}
	keep_on = true;
	keep_sets = sets != 0;
	(void)hakios_rpmh_bw(HAKIOS_RPMH_CPU, "ACV", 0, BCM_ACV_APPS);
	(void)hakios_rpmh_bw(HAKIOS_RPMH_CPU, "SN0", 1, 1);
	(void)hakios_rpmh_bw(HAKIOS_RPMH_CPU, "SH1", 0, 1);
	lck_mtx_lock(&keep_lock);
	keep_loader_set(HAKIOS_RPMH_CPU, true);
	(void)keep_flush();
	lck_mtx_unlock(&keep_lock);
	floor_display_set(true);
	floor_usb_set();
	floor_set(2092, 806);
	tell("ddr: floor %s\n", floor_line);
	if (keep_sets) {
		arc_default_start();
	}
}

bool
hakios_soc_sleep_quiesce(void)
{
	uint32_t a[2 + KEEP_LOADER], v[2 + KEEP_LOADER];
	unsigned n = 2;
	const char *problem;

	if (!keep_on && !__atomic_load_n(&arc_ready, __ATOMIC_ACQUIRE)) {
		return false;
	}
	arc_disarm();
	if (!keep_on || rsc_base == 0 || keep_mc0 < 0 || keep_sh0 < 0) {
		return false;
	}
	a[0] = keep[keep_mc0].address;
	v[0] = BCM_VOTE(210, 9936);
	a[1] = keep[keep_sh0].address;
	v[1] = BCM_VOTE(160, 7570);
	for (unsigned i = 0; i < KEEP_LOADER; i++) {
		if (keep_loader_slot[i] >= 0) {
			a[n] = keep[keep_loader_slot[i]].address;
			v[n++] = BCM_VOTE(keep_loader[i].ab, keep_loader[i].ib);
		}
	}
	keep_on = false;
	if (keep_sets) {
		wr(SET_REG(KEEP_SLEEP_TCS, 0x1cu), 0);
		wr(SET_REG(KEEP_WAKE_TCS, 0x1cu), 0);
		keep_sets = false;
	}
	problem = hakios_rpmh_send_rsc(rsc_base, rsc_tcs_off, a, v, n);
	tell("ddr: the boot loader's memory and NoC votes back for the loader (%u): %s\n", n,
	    problem == NULL ? "done" : problem);
	return true;
}

void
hakios_soc_sleep_init(void)
{
	const struct hakios_board *board = hakios_board();
	uint32_t sleep_on = 1;

	if (!hakios_board_has(HAKIOS_HAS_SOC_SLEEP) || board == NULL ||
	    board->rpmh.rsc == 0 || board->rpmh.cmd_db == 0 || board->rpmh.cmd_db_size == 0) {
		return;
	}
	PE_parse_boot_argn("rpmh_sleep", &sleep_on, sizeof(sleep_on));
	if (sleep_on != 0) {
		keep_start();
	} else {
		floor_describe();
		tell("rpmh: soc-sleep disabled (rpmh_sleep=0)\n");
	}
	if (board->rpmh.sleep_stats != 0) {
		ss_start();
	}
}
