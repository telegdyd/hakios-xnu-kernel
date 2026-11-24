/*
 * ---- HLOS's own ARC and VRM votes, the GCC's running branches, and ARC sleep entries (POWER-COLLAPSE-PLAN.md 5e) ----
 *
 * Included by hakios.cpufreq.c (it uses ddr_rsc, the command database's structs, hakios_rsc_lock, floor_usb and
 * pcie_bw_needed from there).
 *
 * Why: with PCIe down and USB out the AOP's CX monitor still said HLOS held CX in every line, and the APSS sleep set
 * (TCS 3) held only BCM commands. An ARC keeps a DRV's last vote unless the sleep set writes another, so whatever the
 * boot loader (or anything since) voted on cx.lvl, xo.lvl ... through DRV 2 stays through every llcc-off. Linux writes
 * a sleep-set vote for each of them (rpmhpd: the sleep corner, 0 with only active-only users; clk-rpmh: bi_tcxo off in
 * sleep once its last GCC child is off).
 *
 * kern.hakios.rpmh_votes (read): each ARC's and RPMh clock's vote of this DRV, read back through TCS 0 with a read
 * command (message id without the write bit; the answer in the command's RESP_DATA, rpmh-rsc.c's register 0x40), and
 * each ARC's level table from the command database. A read that does not complete says so: then reads are not
 * something this RPMh answers, and the votes stay unknown.
 *
 * kern.hakios.gcc_on (read): the GCC's branch clocks (Linux gcc-sm8450.c, hakios.gcc8450.h) whose halt register says
 * running (bit 31 CLK_OFF clear). A branch that runs while no core does needs CX at its level through the sleep.
 *
 * kern.hakios.rpmh_sleep_arc (write "name=value,..."; "" clears; read: the state): ARC/RPMh-clock commands in the second
 * sleep TCS (4) and the second wake TCS (6); the keeper's 3 and 5 are untouched. "value" is the raw vote, or "@corner":
 * the first level position at or above that corner (cmd-db). The wake command is the vote read back at the write; a
 * name whose vote cannot be read is refused. The entries start disabled. kern.hakios.rpmh_sleep_gate (write
 * "0xoffset,..." of GCC branches; read: the list): branches that must be stopped.
 *
 * Arming happens only in hakios_rpmh_llcc_down(), called with interrupts off by the last core up as it asks for llcc-off
 * (psci_smp.c), the moment Linux flushes its sleep set (rpmh_flush at the cluster's power-down): every other core is down
 * then, so nothing changes under the check. Enabled (both TCSs' command masks) only while the display is off, no USB
 * cable is in (the keeper's floor_usb), PCIe's bandwidth is not needed, and every gate branch is stopped; otherwise both
 * masks are cleared and the reason counted. If the firmware then refuses llcc-off the sets stay as written; they run
 * only on a sleep, and the next llcc-off checks again. On the way back to the loader both are cleared.
 */
#include "hakios.gcc8450.h"

#define ARC_SLEEP_TCS           4
#define ARC_WAKE_TCS            6
#define ARC_MAX                 8
#define ARC_GATES               16
#define GCC_PHYS_8450           0x00100000UL
#define GCC_SIZE_8450           0x1f4200UL
#define CMD_DB_ARC              3
#define CMD_DB_VRM              4
#define TCS_CMD_STATUS          0x3c            /* per command, after 0x30 + 20 * n: issued bit 8, completed bit 16 */
#define TCS_CMD_RESP            0x40

/* The GCC's power domains (gcc-sm8450.c; GDSCR: SW_COLLAPSE bit 0, PWR_ON bit 31) */
static const struct {
	uint32_t        offset;
	const char      *name;
} gcc8450_gdscs[] = {
	{ 0x7b004, "pcie_0_gdsc" }, { 0x9d004, "pcie_1_gdsc" }, { 0x87004, "ufs_phy_gdsc" }, { 0x49004, "usb30_prim_gdsc" },
};
/*
 * CX collapse (boot-arg rpmh_cx_collapse=0: never): with the entries armed, cx.lvl's sleep command becomes 0 only while
 * each of these domains is off; otherwise its configured retention. run5: collapses with the parked USB controller's
 * domain on ended in a frozen phone at the replug (its logic unpowered under a domain that says on); the USB driver
 * turns that domain off while parked (usb_park=2). PCIe's and UFS's too: run8 (USB's only) and run9 (USB's and PCIe's)
 * collapsed 36 and 42 times with Wi-Fi off and froze some minutes after Wi-Fi was back, with no collapse in between in
 * run9; from the first collapses the UFS clock stayed on in most llcc-off asks (run7, none: 48 in 4000). So UFS's
 * domain must be off too: the UFS driver turns it off here, just before the check, when its link is in Hibern8 and
 * nothing is in flight (hakios_ufs_power_down, hakios.ufshc-power.h: flop retention, back on at the next doorbell).
 */
static const uint32_t arc_collapse_gdsc[] = { 0x49004, 0x7b004, 0x9d004, 0x87004 };
#define ARC_PCIE0_GDSC 0x7b004u
extern bool hakios_ufs_power_down(void) __attribute__((weak));
extern bool hakios_pcie_drv_cx_ready(void) __attribute__((weak));
/*
 * And MMCX, the rail of the display, the camera and the video blocks, which Linux's rpmhpd gives CX as its parent on
 * sm8450 (mmcx_w_cx_parent: CX at least MMCX's level). Screen off only put the panel to sleep (hakios.backlight.cpp,
 * sleepPanel); MDSS, the DSI controller and PHY and its PLL stayed powered and clocked, and the sleep set dropped CX to 0
 * under MMCX at the boot loader's vote. Every run with a CX collapse (run5, run8, run9, run14: cxsd 15..1937) froze
 * minutes later; every run without one (run2-4, 6, 7, 10) did not, with the same branches running. So CX collapses only
 * with MMCX: mmcx.lvl is an entry of the sleep set, held at its vote, and 0 with cx.lvl only while the display's hardware
 * is all off (hakios_display_off(), hakios.display-power.h: from after the panel's sleep to before its wake; the
 * dispcc GDSCs, mdss_gdsc 0x9000 and mdss_int2_gdsc 0xb000, read off as well) and the camera's and video's domains are
 * off (camcc titan_top_gdsc 0x131dc, videocc mvs0c 0x804c and mvs1c 0x8074). Not while this DRV's MMCX vote is 0 (the
 * rail is somebody else's then) or a driver raised it. (Their register buses, disp/camera/video_ahb_clk, are kept on.)
 */
#define DISPCC_PHYS_8450        0x0af00000UL
#define DISPCC_MDSS_GDSCR       0x9000
#define DISPCC_MDSS_INT2_GDSCR  0xb000
#define CAMCC_TITAN_PAGE_8450   0x0adf3000UL    /* camcc 0xade0000 + 0x13000: titan_top_gdsc at +0x1dc */
#define VIDEOCC_PAGE_8450       0x0aaf8000UL    /* videocc 0xaaf0000 + 0x8000: mvs0c_gdsc +0x4c, mvs1c_gdsc +0x74 */
static vm_offset_t arc_dispcc, arc_camcc, arc_videocc;
static uint32_t arc_mmcx = UINT32_MAX;                  /* mmcx.lvl's entry */
static bool arc_mm_readable;
extern bool hakios_display_off(void) __attribute__((weak));

/* MMCX's blocks all off (the last core up, interrupts off) */
static bool
arc_mm_off(void)
{
	return arc_mm_readable && hakios_display_off != NULL && hakios_display_off() &&
	       !((*(volatile uint32_t *)(arc_dispcc + DISPCC_MDSS_GDSCR) | *(volatile uint32_t *)(arc_dispcc + DISPCC_MDSS_INT2_GDSCR) |
	       *(volatile uint32_t *)(arc_camcc + 0x1dc) | *(volatile uint32_t *)(arc_videocc + 0x4c) |
	       *(volatile uint32_t *)(arc_videocc + 0x74)) & (1u << 31));
}
#define ARC_COLLAPSE_GDSCS      (sizeof(arc_collapse_gdsc) / sizeof(arc_collapse_gdsc[0]))

static struct {
	char            name[12];
	uint32_t        address, sleep, wake;
	bool            sleep_only;     /* "name=value!": no wake command, a sentinel read back afterwards */
} arc[ARC_MAX];
static uint32_t arc_n, arc_mask, arc_wake_mask, arc_live, arc_gate[ARC_GATES], arc_gates;
static bool arc_ready;
static vm_offset_t arc_gcc;
static struct {
	uint64_t        asks, armed, screen, usb, pcie, changes, gate[ARC_GATES];
	uint64_t        collapse, collapse_gdsc, collapse_raised, ufs_off, gdsc_held[4], display_held, mmcx_held;
	uint64_t        pcie0_retained;
} arc_count;
static uint32_t arc_cx = UINT32_MAX, arc_collapse_on;      /* cx.lvl's entry */
static uint32_t arc_sleep_cfg[ARC_MAX];
static uint32_t arc_floor[ARC_MAX];     /* (below: what it is) */
static char arc_text[4096];

/* A resource of any kind in the command database: its address, and an ARC's level table (up to 16) */
static uint32_t
rpmh_db_find(const char *name, uint16_t *levels, unsigned *nlevels, unsigned *kind)
{
	static const struct cmd_db_header *db;          /* mapped once: every read of the sysctl comes here */
	char id[8] = { 0 };

	*nlevels = 0;
	if (db == NULL) {
		db = cmd_db_mapped();
	}
	if (db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		return 0;
	}
	memcpy(id, name, strlen(name) < sizeof(id) ? strlen(name) : sizeof(id));     /* (8 bytes, padded, no terminator: "mmcx.lvl" fills all 8) */
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &db->slave[s];
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);

		if (slave->id == 0 || slave->header_offset >= CMD_DB_SIZE - 0x1000 || slave->count > 512) {
			continue;
		}
		for (unsigned i = 0; i < slave->count; i++) {
			if (memcmp(entry[i].id, id, sizeof(id)) != 0) {
				continue;
			}
			*kind = slave->id;
			if (slave->id == CMD_DB_ARC && (unsigned)slave->data_offset + entry[i].offset + entry[i].length <= CMD_DB_SIZE - 0x1000) {
				const uint8_t *d = db->data + slave->data_offset + entry[i].offset;

				for (unsigned l = 0; l < entry[i].length / 2u && l < 16; l++) {
					uint16_t v = (uint16_t)(d[2 * l] | d[2 * l + 1] << 8);

					if (l > 0 && v == 0) {
						break;
					}
					levels[(*nlevels)++] = v;
				}
			}
			return entry[i].address;
		}
	}
	return 0;
}

/* One read command through TCS 0 (as ddr_rpmh_write, a read): true when it completed. With hakios_rsc_lock. */
static bool
rpmh_read(uint32_t address, uint32_t *value, uint32_t *status)
{
	uint32_t irq_enable = *(volatile uint32_t *)TCS_REG(0x00);
	bool done = false;

	*value = 0;
	*status = 0;
	for (int i = 0; *(volatile uint32_t *)TCS_REG(0x18) == 0; i++) {
		if (i > 2000) {
			return false;
		}
		delay_for_interval(100, NSEC_PER_USEC);
	}
	*(volatile uint32_t *)TCS_REG(0x14) &= ~(1u << 24);
	*(volatile uint32_t *)TCS_REG(0x14) &= ~(1u << 16);
	*(volatile uint32_t *)TCS_REG(0x08) = 1;
	*(volatile uint32_t *)TCS_REG(0x30) = 8 | 0x100;               /* four bytes, an answer wanted, no write bit: a read */
	*(volatile uint32_t *)TCS_REG(0x34) = address;
	*(volatile uint32_t *)TCS_REG(0x38) = 0;
	*(volatile uint32_t *)TCS_REG(TCS_CMD_RESP) = 0;
	*(volatile uint32_t *)TCS_REG(0x10) = 1;
	*(volatile uint32_t *)TCS_REG(0x1c) = 1;
	*(volatile uint32_t *)TCS_REG(0x00) = irq_enable | 1;
	*(volatile uint32_t *)TCS_REG(0x14) = 1u << 16;
	*(volatile uint32_t *)TCS_REG(0x14) = 1u << 16 | 1u << 24;
	for (int i = 0; i < 200 && !done; i++) {
		done = (*(volatile uint32_t *)TCS_REG(0x04) & 1) != 0;
		if (!done) {
			delay_for_interval(100, NSEC_PER_USEC);
		}
	}
	*status = *(volatile uint32_t *)TCS_REG(TCS_CMD_STATUS);
	*value = *(volatile uint32_t *)TCS_REG(TCS_CMD_RESP);
	*(volatile uint32_t *)TCS_REG(0x14) &= ~(1u << 24);
	*(volatile uint32_t *)TCS_REG(0x14) &= ~(1u << 16);
	*(volatile uint32_t *)TCS_REG(0x1c) = 0;
	*(volatile uint32_t *)TCS_REG(0x10) = 0;
	*(volatile uint32_t *)TCS_REG(0x08) = 1;
	*(volatile uint32_t *)TCS_REG(0x00) = irq_enable;
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
	int at = 0, error;

	if (req->newptr != USER_ADDR_NULL) {
		return EPERM;
	}
	if (req->oldptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, text, sizeof(text));
	}
	if (ddr_rsc == 0) {
		return ENODEV;
	}
	at += snprintf(text + at, sizeof(text) - (size_t)at, "this DRV's (APSS, 2) votes read back through TCS 0 (VRM clocks: +4, enable):\n");
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
			address += 4;
		}
		lck_mtx_lock(&hakios_rsc_lock);
		ok = rpmh_read(address, &value, &status);
		lck_mtx_unlock(&hakios_rsc_lock);
		at += snprintf(text + at, sizeof(text) - (size_t)at, "%s %#x (kind %u): %s %#x status %#x", rpmh_vote_names[i], address, kind,
		    ok ? "vote" : "NO ANSWER, read", value, status);
		for (unsigned l = 0; l < n && at < (int)sizeof(text) - 20; l++) {
			at += snprintf(text + at, sizeof(text) - (size_t)at, "%s%u", l ? " " : "; levels ", levels[l]);
		}
		at += snprintf(text + at, sizeof(text) - (size_t)at, "\n");
	}
	error = SYSCTL_OUT(req, text, strlen(text) + 1);
	return error;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_votes, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, rpmh_votes_sysctl, "A",
    "diag: this DRV's ARC and RPMh clock votes read back from RPMh, and each ARC's levels");

static bool
gcc_map(void)
{
	if (arc_gcc == 0) {
		arc_gcc = ml_io_map(GCC_PHYS_8450, GCC_SIZE_8450);
	}
	return arc_gcc != 0;
}

static int
gcc_on_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	static char text[8192];
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
		unsigned v;

		if (hakios_mmio_try_read32((const volatile void *)(arc_gcc + gcc8450_branches[i].offset), &v) != 0) {
			unread++;
			continue;
		}
		if (v & (1u << 31)) {
			off++;
			continue;
		}
		on++;
		if (at < (int)sizeof(text) - 100) {
			at += snprintf(text + at, sizeof(text) - (size_t)at, "%#07x %s %#x\n", gcc8450_branches[i].offset,
			    gcc8450_branches[i].name, v);
		}
	}
	at += snprintf(text + at, sizeof(text) - (size_t)at, "%u running, %u stopped, %u not readable\n", on, off, unread);
	for (unsigned i = 0; i < sizeof(gcc8450_gdscs) / sizeof(gcc8450_gdscs[0]); i++) {
		unsigned v = *(volatile uint32_t *)(arc_gcc + gcc8450_gdscs[i].offset);

		at += snprintf(text + at, sizeof(text) - (size_t)at, "gdsc %#07x %s %s (%#x)\n", gcc8450_gdscs[i].offset,
		    gcc8450_gdscs[i].name, v & (1u << 31) ? "ON" : "off", v);
	}
	if (arc_dispcc != 0) {
		unsigned v = *(volatile uint32_t *)(arc_dispcc + DISPCC_MDSS_GDSCR);
		unsigned w = *(volatile uint32_t *)(arc_dispcc + DISPCC_MDSS_INT2_GDSCR);

		at += snprintf(text + at, sizeof(text) - (size_t)at, "gdsc dispcc 0x9000 mdss_gdsc %s (%#x)\n"
		    "gdsc dispcc 0xb000 mdss_int2_gdsc %s (%#x)\n", v & (1u << 31) ? "ON" : "off", v, w & (1u << 31) ? "ON" : "off", w);
	}
	if (arc_mm_readable) {
		unsigned c = *(volatile uint32_t *)(arc_camcc + 0x1dc), v0 = *(volatile uint32_t *)(arc_videocc + 0x4c);
		unsigned v1 = *(volatile uint32_t *)(arc_videocc + 0x74);

		at += snprintf(text + at, sizeof(text) - (size_t)at, "gdsc camcc 0x131dc titan_top_gdsc %s (%#x)\n"
		    "gdsc videocc 0x804c mvs0c_gdsc %s (%#x)\ngdsc videocc 0x8074 mvs1c_gdsc %s (%#x)\n", c & (1u << 31) ? "ON" : "off", c,
		    v0 & (1u << 31) ? "ON" : "off", v0, v1 & (1u << 31) ? "ON" : "off", v1);
	}
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, gcc_on, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, gcc_on_sysctl, "A",
    "diag: the GCC's branch clocks that run now (gcc-sm8450.c's list)");

/* both experiment TCSs' command masks; no lock: the last core up (or a writer that disarmed first) */
static void
arc_enable(uint32_t mask)
{
	*(volatile uint32_t *)SET_REG(ARC_SLEEP_TCS, 0x1c) = mask;
	*(volatile uint32_t *)SET_REG(ARC_WAKE_TCS, 0x1c) = mask & arc_wake_mask;
	__asm__ volatile ("dsb sy" ::: "memory");
	arc_live = mask;
}

void hakios_rpmh_llcc_down(void);

void
hakios_rpmh_llcc_down(void)
{
	uint32_t want = arc_mask;

	if (!os_atomic_load(&arc_ready, acquire) || ddr_rsc == 0) {
		return;
	}
	arc_count.asks++;
	if (!hakios_screen_off()) {
		want = 0;
		arc_count.screen++;
	} else if (floor_usb || !floor_usb_known) {
		want = 0;
		arc_count.usb++;
	} else if (os_atomic_load(&pcie_bw_needed, relaxed)) {
		want = 0;
		arc_count.pcie++;
	} else {
		for (uint32_t g = 0; g < arc_gates; g++) {
			if (!(*(volatile uint32_t *)(arc_gcc + arc_gate[g]) & (1u << 31))) {
				want = 0;
				arc_count.gate[g]++;
				break;
			}
		}
	}
	if (want != 0 && arc_collapse_on && arc_cx < arc_n) {
		uint32_t v = 0;
		volatile uint32_t *data = (volatile uint32_t *)SET_REG(ARC_SLEEP_TCS, 0x38 + 20 * arc_cx);

		if (arc[arc_cx].sleep != arc_sleep_cfg[arc_cx]) {
			v = arc[arc_cx].sleep;          /* (a driver's vote above the floor holds it) */
			arc_count.collapse_raised++;
		} else if (arc_mmcx >= arc_n || arc[arc_mmcx].wake == 0 || arc[arc_mmcx].wake != arc_floor[arc_mmcx] ||
		    arc[arc_mmcx].sleep != arc_sleep_cfg[arc_mmcx]) {
			v = arc[arc_cx].sleep;          /* (MMCX not this DRV's to drop, or raised: CX stays at retention) */
			arc_count.mmcx_held++;
		} else {
			if (!arc_mm_off()) {
				v = arc[arc_cx].sleep;  /* (the display's hardware on, or the camera's or video's: CX stays at retention) */
				arc_count.display_held++;
			} else if (hakios_ufs_power_down != NULL && hakios_ufs_power_down()) {
				arc_count.ufs_off++;
			}
			for (uint32_t g = 0; g < ARC_COLLAPSE_GDSCS && v == 0; g++) {
				if (*(volatile uint32_t *)(arc_gcc + arc_collapse_gdsc[g]) & (1u << 31)) {
					/* SM8450 PCIe0 is RET_ON. The hardware retains it when CX drops after DRV handoff. */
					if (arc_collapse_gdsc[g] == ARC_PCIE0_GDSC &&
					    hakios_pcie_drv_cx_ready && hakios_pcie_drv_cx_ready()) {
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
		if (*data != v) {
			*data = v;
			__asm__ volatile ("dsb sy" ::: "memory");
		}
		if (arc_mmcx < arc_n) {
			/* MMCX with CX: 0 in the collapse, otherwise held at its vote */
			volatile uint32_t *mm = (volatile uint32_t *)SET_REG(ARC_SLEEP_TCS, 0x38 + 20 * arc_mmcx);
			uint32_t m = v == 0 ? 0 : arc[arc_mmcx].sleep;

			if (*mm != m) {
				*mm = m;
				__asm__ volatile ("dsb sy" ::: "memory");
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
	os_atomic_store(&arc_ready, false, release);
	if (ddr_rsc != 0) {
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
		uint32_t d = *s >= '0' && *s <= '9' ? (uint32_t)(*s - '0') : *s >= 'a' && *s <= 'f' ? (uint32_t)(*s - 'a' + 10) :
		    *s >= 'A' && *s <= 'F' ? (uint32_t)(*s - 'A' + 10) : 99;

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

	at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at, "ready %u entries %u live %#x (sleep TCS %u, wake TCS %u)\n",
	    arc_ready, arc_n, arc_live, ARC_SLEEP_TCS, ARC_WAKE_TCS);
	for (uint32_t i = 0; i < arc_n; i++) {
		at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at, "entry %s %#x sleep %#x wake %#x%s\n", arc[i].name,
		    arc[i].address, arc[i].sleep, arc[i].wake, arc[i].sleep_only ? " (sleep only: not restored)" : "");
	}
	at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at, "llcc-off asks %llu armed %llu changes %llu; held by screen %llu "
	    "usb %llu pcie %llu\n", arc_count.asks, arc_count.armed, arc_count.changes, arc_count.screen, arc_count.usb, arc_count.pcie);
	at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at, "cx collapse %s: armed %llu; held by a power domain on %llu, "
	    "a driver's vote %llu, MMCX not droppable %llu, the display's/camera's/video's hardware on %llu; UFS's domain off %llu; "
	    "held by usb %llu pcie0 %llu pcie1 %llu ufs %llu; MMCX entry %s, its blocks readable %u\n",
	    arc_collapse_on && arc_cx < arc_n ? "on" : "off", arc_count.collapse, arc_count.collapse_gdsc, arc_count.collapse_raised,
	    arc_count.mmcx_held, arc_count.display_held, arc_count.ufs_off, arc_count.gdsc_held[0], arc_count.gdsc_held[1],
	    arc_count.gdsc_held[2], arc_count.gdsc_held[3], arc_mmcx < arc_n ? "in the sleep set" : "NONE", arc_mm_readable);
	at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at, "pcie0 retained in DRV %llu\n",
	    arc_count.pcie0_retained);
	for (uint32_t g = 0; g < arc_gates && at < (int)sizeof(arc_text) - 120; g++) {
		const char *name = "?";

		for (unsigned i = 0; i < sizeof(gcc8450_branches) / sizeof(gcc8450_branches[0]); i++) {
			if (gcc8450_branches[i].offset == arc_gate[g]) {
				name = gcc8450_branches[i].name;
			}
		}
		at += snprintf(arc_text + at, sizeof(arc_text) - (size_t)at, "gate %#07x %s: held %llu, now %s\n", arc_gate[g], name,
		    arc_count.gate[g], arc_gcc && !(*(volatile uint32_t *)(arc_gcc + arc_gate[g]) & (1u << 31)) ? "RUNNING" : "stopped");
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

/* each entry's wake floor: the vote read back when it was put in (the boot loader's, at the default's start); and the
 * sleep value configured (arc_floor, arc_sleep_cfg: declared above) */

/* The entries from "name=value,..." ("" clears), disarmed first, armed again when there are any. */
static int
arc_configure(const char *spec)
{
	const char *s = spec;
	uint32_t n = 0;
	int error = 0;

	{
		if (ddr_rsc == 0 || !gcc_map()) {
			return ENODEV;
		}
		arc_disarm();
		lck_mtx_lock(&hakios_rsc_lock);
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
			while (*s != '=' && *s != 0 && k < sizeof(name) - 1) {
				name[k++] = *s++;
			}
			if (*s++ != '=' || n == ARC_MAX || (address = rpmh_db_find(name, levels, &nlevels, &kind)) == 0) {
				error = EINVAL;
				break;
			}
			if (kind == CMD_DB_VRM) {
				address += 4;
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
				tell("rpmh: %s (%#x): its vote cannot be read back (status %#x): not put in the sleep set\n", name, address, status);
				error = EIO;
				break;
			}
			strlcpy(arc[n].name, name, sizeof(arc[n].name));
			arc[n].address = address;
			arc[n].wake = value;
			arc_floor[n] = value;
			arc_sleep_cfg[n] = arc[n].sleep;
			if (strcmp(name, "cx.lvl") == 0 && !arc[n].sleep_only && arc[n].sleep != 0) {
				arc_cx = n;     /* (the retention the collapse falls back to) */
			}
			if (strcmp(name, "mmcx.lvl") == 0 && !arc[n].sleep_only) {
				arc[n].sleep = arc_sleep_cfg[n] = value;        /* (held at its vote; 0 only with CX's collapse) */
				arc_mmcx = n;
			}
			n++;
		}
		if (error == 0) {
			for (unsigned t = ARC_SLEEP_TCS; t <= ARC_WAKE_TCS; t += ARC_WAKE_TCS - ARC_SLEEP_TCS) {
				for (uint32_t i = 0; i < n; i++) {
					*(volatile uint32_t *)SET_REG(t, 0x30 + 20 * i) = 0x10008 | 0x100;
					*(volatile uint32_t *)SET_REG(t, 0x34 + 20 * i) = arc[i].address;
					*(volatile uint32_t *)SET_REG(t, 0x38 + 20 * i) = t == ARC_SLEEP_TCS ? arc[i].sleep : arc[i].wake;
				}
			}
			arc_n = n;
			arc_mask = n ? (1u << n) - 1 : 0;
			arc_wake_mask = 0;
			for (uint32_t i = 0; i < n; i++) {
				arc_wake_mask |= arc[i].sleep_only ? 0 : 1u << i;
			}
			bzero(&arc_count, sizeof(arc_count));
		} else {
			arc_n = 0;
			arc_mask = 0;
		}
		lck_mtx_unlock(&hakios_rsc_lock);
		if (error == 0 && n != 0) {
			os_atomic_store(&arc_ready, true, release);
		}
		tell("rpmh: ARC sleep entries: %u (%s)\n", arc_n, error ? "refused" : n ? "armed at llcc-off" : "cleared");
	}
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

/*
 * A driver's own write of an ARC through this DRV's TCS 0 (the modem's and the camera's cx.lvl/mx.lvl while they start),
 * with hakios_rsc_lock held: that entry's wake command becomes the higher of its floor and the vote now, so the wake set
 * never takes a rail below what a running block asked for; and while the vote is above the floor (a proxy request: the
 * modem booting until it votes itself) the sleep command holds it too, so no core's idle lowers it under that block.
 * A core runs this, so the sets are idle.
 */
void hakios_rpmh_arc_active(uint32_t address, uint32_t value);

void
hakios_rpmh_arc_active(uint32_t address, uint32_t value)
{
	for (uint32_t i = 0; i < arc_n; i++) {
		if (arc[i].address == address) {
			arc[i].wake = MAX(arc_floor[i], value);
			arc[i].sleep = value > arc_floor[i] ? value : arc_sleep_cfg[i];
			*(volatile uint32_t *)SET_REG(ARC_WAKE_TCS, 0x38 + 20 * i) = arc[i].wake;
			*(volatile uint32_t *)SET_REG(ARC_SLEEP_TCS, 0x38 + 20 * i) = arc[i].sleep;
			__asm__ volatile ("dsb sy" ::: "memory");
		}
	}
}

/*
 * The default (boot-arg rpmh_sleep_arc=0: not): XO off and CX at retention in the sleep set, CX 0 instead only while
 * the collapse domains above are off (run5's replug after collapses with USB's on froze the phone; run6, retention
 * only, slept and replugged fine), with the DMA masters' clocks as gates. From the keeper's start, while the boot loader's votes are the ones read back.
 */
#define ARC_DEFAULT             "xo.lvl=0,mmcx.lvl=0,cx.lvl=@16"  /* (MMCX before CX: down first in the sleep set) */
#define ARC_DEFAULT_NO_MMCX     "xo.lvl=0,cx.lvl=@16"
#define ARC_DEFAULT_GATES       "0x87018,0x87064,0x870d4,0x49018,0x7b03c,0x7b028,0x24004,0x3300c,0x3314c,0x3328c"

static void
arc_default_start(void)
{
	uint32_t on = 1;
	const char *s = ARC_DEFAULT_GATES;
	int error;

	PE_parse_boot_argn("rpmh_sleep_arc", &on, sizeof(on));
	arc_collapse_on = 1;
	PE_parse_boot_argn("rpmh_cx_collapse", &arc_collapse_on, sizeof(arc_collapse_on));
	if (arc_collapse_on) {
		unsigned r;

		arc_dispcc = ml_io_map(DISPCC_PHYS_8450, 0x10000);
		arc_camcc = ml_io_map(CAMCC_TITAN_PAGE_8450, 0x1000);
		arc_videocc = ml_io_map(VIDEOCC_PAGE_8450, 0x1000);
		arc_mm_readable = arc_dispcc != 0 && arc_camcc != 0 && arc_videocc != 0 && hakios_mmio_try_read32 != NULL &&
		    hakios_mmio_try_read32((const volatile void *)(arc_dispcc + DISPCC_MDSS_GDSCR), &r) == 0 &&
		    hakios_mmio_try_read32((const volatile void *)(arc_camcc + 0x1dc), &r) == 0 &&
		    hakios_mmio_try_read32((const volatile void *)(arc_videocc + 0x4c), &r) == 0 &&
		    hakios_mmio_try_read32((const volatile void *)(arc_videocc + 0x74), &r) == 0;
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
	    "(MMCX vote %#x; its blocks' domains readable %u)\n", arc_mmcx < arc_n ? ARC_DEFAULT : ARC_DEFAULT_NO_MMCX, arc_gates,
	    error ? "REFUSED" : "on", arc_collapse_on && arc_mmcx < arc_n ? "on" : "off", arc_mmcx < arc_n ? arc[arc_mmcx].wake : 0,
	    arc_mm_readable);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_sleep_arc, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED, 0, 0, rpmh_sleep_arc_sysctl, "A",
    "diag: name=value,... ARC/clock commands in sleep TCS 4 (wake TCS 6: the vote read back), armed at llcc-off; \"\" clears");

static int
rpmh_sleep_gate_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char in[256];
	int error;

	if (req->newptr != USER_ADDR_NULL) {
		const char *s = in;
		uint32_t gates[ARC_GATES], n = 0;
		bool was = os_atomic_load(&arc_ready, acquire);

		if ((error = arc_input(req, in, sizeof(in))) != 0) {
			return error;
		}
		if (!gcc_map()) {
			return ENODEV;
		}
		while (*s != 0) {
			unsigned v;
			uint32_t off;

			while (*s == ',' || *s == ' ' || *s == '\n') {
				s++;
			}
			if (*s == 0) {
				break;
			}
			off = arc_number(s, &s);
			if (n == ARC_GATES || off >= GCC_SIZE_8450 || (off & 3) ||
			    hakios_mmio_try_read32((const volatile void *)(arc_gcc + off), &v) != 0) {
				return EINVAL;
			}
			gates[n++] = off;
		}
		/* no llcc-off check may read the list while it changes: disarmed around it, armed again as it was */
		arc_disarm();
		memcpy(arc_gate, gates, sizeof(gates[0]) * n);
		arc_gates = n;
		bzero(arc_count.gate, sizeof(arc_count.gate));
		if (was && arc_n != 0) {
			os_atomic_store(&arc_ready, true, release);
		}
	}
	arc_describe();
	return SYSCTL_OUT(req, arc_text, strlen(arc_text) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, rpmh_sleep_gate, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED, 0, 0, rpmh_sleep_gate_sysctl, "A",
    "diag: GCC branch offsets that must be stopped for the ARC sleep entries to be armed");
