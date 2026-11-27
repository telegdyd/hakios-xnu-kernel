/*
 * Host test for drivers/board/soc_sleep.c: BCM vote summing (including ACV OR-masking,
 * MC0/SH0 MHz conversion, active-only exclusion from sleep TCS 3 vs wake TCS 5),
 * floor_follow row selection & screen-off 200 MHz drop, and hakios_rpmh_llcc_down gating
 * (screen, USB cable, PCIe bandwidth, GCC branch gates, display/UFS/GDSC collapse).
 *
 *   soc_sleep_test SCENARIO
 */
#include <unistd.h>

#include "fakekernel.h"

#define KERNEL 1
#define KERNEL_PRIVATE 1
#define HAKIOS_MMIO_TEST 1

#ifndef SYSCTL_IN
#define SYSCTL_IN(req, p, l) copyin((user_addr_t)(uintptr_t)(req)->newptr, (p), (l))
#endif

static unsigned failures;
#define CHECK(cond, ...) do { if (!(cond)) { fprintf(stderr, "FAIL %s:%d: ", __func__, __LINE__); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); failures++; } } while (0)

/* Forward mocks used inside drivers/board/soc_sleep.c */
#include "board/board.h"
#include <sys/board/input.h>

static struct hakios_board test_board;
static uint64_t test_features;
static bool mock_screen_off, mock_display_off, mock_ufs_down;
static int mock_charger = 1;
static uint32_t mock_locks, mock_unlocks;

const struct hakios_board *hakios_board(void) { return &test_board; }
bool hakios_board_has(uint64_t f) { return (test_features & f) != 0; }
void hakios_rpmh_lock(void) { mock_locks++; }
void hakios_rpmh_unlock(void) { mock_unlocks++; }
#define hakios_screen_off mock_hakios_screen_off
bool mock_hakios_screen_off(void) { return mock_screen_off; }
bool hakios_display_off(void) { return mock_display_off; }
bool hakios_ufs_power_down(void) { return mock_ufs_down; }
bool hakios_battery_get(int *soc, int *present, int *charger)
{
	*soc = 80;
	*present = 1;
	*charger = mock_charger;
	return true;
}
bool hakios_battery_power(int *ma, int *mv, int *charger)
{
	*ma = -300;
	*mv = 4000;
	*charger = mock_charger;
	return true;
}
bool hakios_aop_message(const char *text __unused) { return false; }

uint32_t hakios_test_rd(vm_offset_t address);
void hakios_test_wr(vm_offset_t address, uint32_t value);
int hakios_mmio_try_read32(const volatile void *addr, unsigned *out);
vm_offset_t ml_io_map(vm_offset_t phys, vm_size_t size __unused);
vm_offset_t ml_io_map_wcomb(vm_offset_t phys, vm_size_t size __unused);

#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic ignored "-Waddress"
#include "board/soc_sleep.c"

#define MOCK_CMD_DB_PA  0x80860000u
#define MOCK_CMD_DB_SZ  0x4000u
#define MOCK_RSC_PA     0x17a20000u
#define MOCK_TCS_OFF    0xd00u
#define MOCK_GCC_PA     0x00100000u
#define MOCK_DISPCC_PA  0x0af00000u
#define MOCK_CAMCC_GDSC 0x0adf31dcu
#define MOCK_VCC0_GDSC  0x0aaf804cu
#define MOCK_VCC1_GDSC  0x0aaf8074u

static uint8_t mock_cmd_db[MOCK_CMD_DB_SZ];

struct mock_tcs {
	uint32_t irq_enable, irq_status, irq_clear;
	uint32_t amc_mode, control, idle, wait_ack, enable;
	struct {
		uint32_t msgid, address, data, status, resp;
	} cmd[16];
};
static struct mock_tcs mock_tcs[8];

static uint32_t mock_branches[16];
static uint32_t mock_gdsc_usb, mock_gdsc_pcie0, mock_gdsc_pcie1, mock_gdsc_ufs;
static uint32_t mock_gdsc_disp0, mock_gdsc_disp2, mock_gdsc_cam, mock_gdsc_v0, mock_gdsc_v1;
static uint32_t mock_arc_read_cx = 3, mock_arc_read_mmcx = 2, mock_arc_read_xo = 1;

static const uint32_t gate_offs[] = {
	0x87018, 0x87064, 0x870d4, 0x49018, 0x7b03c, 0x7b028, 0x24004, 0x3300c, 0x3314c, 0x3328c
};

static void
build_cmd_db(void)
{
	struct cmd_db_header *hdr = (struct cmd_db_header *)mock_cmd_db;
	uint16_t h_off = 0, d_off = 0x1000;

	memset(mock_cmd_db, 0, sizeof(mock_cmd_db));
	hdr->magic[0] = 0xdb;
	hdr->magic[1] = 0x30;
	hdr->magic[2] = 0x03;
	hdr->magic[3] = 0x0c;

	static const struct {
		const char *name;
		uint32_t addr, unit;
		uint16_t width;
	} bcms[] = {
		{ "MC0", 0x50000, 3200000, 16 },
		{ "SH0", 0x50004, 4200000, 64 },
		{ "ACV", 0x50008, 1, 1 },
		{ "SN0", 0x5000c, 1, 1 },
		{ "SN2", 0x50010, 1, 1 },
		{ "SN3", 0x50014, 1, 1 },
		{ "QUP0", 0x50018, 1, 1 },
		{ "MM0", 0x5001c, 1, 1 },
		{ "MM1", 0x50020, 1, 1 },
		{ "CN0", 0x50024, 1, 1 },
		{ "SH1", 0x50028, 1, 1 },
	};
	hdr->slave[0].id = CMD_DB_BCM;
	hdr->slave[0].header_offset = h_off;
	hdr->slave[0].data_offset = d_off;
	hdr->slave[0].count = sizeof(bcms) / sizeof(bcms[0]);
	for (unsigned i = 0; i < hdr->slave[0].count; i++) {
		struct cmd_db_entry *e = (struct cmd_db_entry *)(hdr->data + h_off + i * sizeof(*e));
		uint32_t aux[2];

		memcpy(e->id, bcms[i].name, strlen(bcms[i].name));
		e->address = bcms[i].addr;
		e->offset = (uint16_t)(i * 8);
		e->length = 8;
		aux[0] = bcms[i].unit;
		aux[1] = bcms[i].width;
		memcpy(hdr->data + d_off + e->offset, aux, sizeof(aux));
	}

	h_off = 0x400;
	d_off = 0x1400;
	static const struct {
		const char *name;
		uint32_t addr;
		uint16_t levels[4];
		uint16_t len;
	} arcs[] = {
		{ "xo.lvl",   0x30000, { 0, 48, 0, 0 }, 4 },
		{ "mmcx.lvl", 0x30004, { 0, 16, 48, 64 }, 8 },
		{ "cx.lvl",   0x30008, { 0, 16, 48, 64 }, 8 },
	};
	hdr->slave[1].id = CMD_DB_ARC;
	hdr->slave[1].header_offset = h_off;
	hdr->slave[1].data_offset = d_off;
	hdr->slave[1].count = sizeof(arcs) / sizeof(arcs[0]);
	for (unsigned i = 0; i < hdr->slave[1].count; i++) {
		struct cmd_db_entry *e = (struct cmd_db_entry *)(hdr->data + h_off + i * sizeof(*e));

		memcpy(e->id, arcs[i].name, strlen(arcs[i].name));
		e->address = arcs[i].addr;
		e->offset = (uint16_t)(i * 8);
		e->length = arcs[i].len;
		memcpy(hdr->data + d_off + e->offset, arcs[i].levels, arcs[i].len);
	}
}

static uint32_t *
find_gcc_reg(vm_offset_t address)
{
	uint32_t off = (uint32_t)(address - MOCK_GCC_PA);

	if (off == 0x49004) return &mock_gdsc_usb;
	if (off == 0x7b004) return &mock_gdsc_pcie0;
	if (off == 0x9d004) return &mock_gdsc_pcie1;
	if (off == 0x87004) return &mock_gdsc_ufs;
	for (unsigned i = 0; i < sizeof(gate_offs) / sizeof(gate_offs[0]); i++) {
		if (off == gate_offs[i]) return &mock_branches[i];
	}
	return NULL;
}

uint32_t
hakios_test_rd(vm_offset_t address)
{
	if (address >= MOCK_RSC_PA && address < MOCK_RSC_PA + 0x10000u) {
		uint32_t off = (uint32_t)(address - MOCK_RSC_PA);
		if (off == 0x0) return 0x00020000u;
		if (off >= MOCK_TCS_OFF) {
			uint32_t rel = off - MOCK_TCS_OFF;
			uint32_t t = rel / 0x2a0u, r = rel % 0x2a0u;
			if (t < 8) {
				if (r == 0x00) return mock_tcs[t].irq_enable;
				if (r == 0x04) return mock_tcs[t].irq_status;
				if (r == 0x08) return mock_tcs[t].irq_clear;
				if (r == 0x10) return mock_tcs[t].wait_ack;
				if (r == 0x14) return mock_tcs[t].control;
				if (r == 0x18) return mock_tcs[t].idle;
				if (r == 0x1c) return mock_tcs[t].enable;
				if (r >= 0x30) {
					uint32_t c = (r - 0x30) / 20, f = (r - 0x30) % 20;
					if (c < 16) {
						if (f == 0x00) return mock_tcs[t].cmd[c].msgid;
						if (f == 0x04) return mock_tcs[t].cmd[c].address;
						if (f == 0x08) return mock_tcs[t].cmd[c].data;
						if (f == 0x0c) return mock_tcs[t].cmd[c].status;
						if (f == 0x10) return mock_tcs[t].cmd[c].resp;
					}
				}
			}
		}
		return 0;
	}
	if (address >= MOCK_GCC_PA && address < MOCK_GCC_PA + 0x200000u) {
		uint32_t *p = find_gcc_reg(address);
		return p ? *p : (1u << 31);
	}
	if (address == MOCK_DISPCC_PA + 0x9000u) return mock_gdsc_disp0;
	if (address == MOCK_DISPCC_PA + 0xb000u) return mock_gdsc_disp2;
	if (address == MOCK_CAMCC_GDSC) return mock_gdsc_cam;
	if (address == MOCK_VCC0_GDSC) return mock_gdsc_v0;
	if (address == MOCK_VCC1_GDSC) return mock_gdsc_v1;
	return 0;
}

void
hakios_test_wr(vm_offset_t address, uint32_t value)
{
	if (address >= MOCK_RSC_PA && address < MOCK_RSC_PA + 0x10000u) {
		uint32_t off = (uint32_t)(address - MOCK_RSC_PA);
		if (off >= MOCK_TCS_OFF) {
			uint32_t rel = off - MOCK_TCS_OFF;
			uint32_t t = rel / 0x2a0u, r = rel % 0x2a0u;
			if (t < 8) {
				if (r == 0x00) mock_tcs[t].irq_enable = value;
				else if (r == 0x08) mock_tcs[t].irq_status &= ~value;
				else if (r == 0x10) mock_tcs[t].wait_ack = value;
				else if (r == 0x14) {
					mock_tcs[t].control = value;
					if ((value & (1u << 24)) != 0) {
						mock_tcs[t].irq_status |= 1u;
						for (unsigned c = 0; c < 16; c++) {
							if ((mock_tcs[t].enable & (1u << c)) == 0) continue;
							mock_tcs[t].cmd[c].status = 0x10100u;
							if ((mock_tcs[t].cmd[c].msgid & 0x10000u) == 0) {
								uint32_t a = mock_tcs[t].cmd[c].address;
								mock_tcs[t].cmd[c].resp =
								    a == 0x30000 ? mock_arc_read_xo :
								    a == 0x30004 ? mock_arc_read_mmcx :
								    a == 0x30008 ? mock_arc_read_cx : 0;
							}
						}
					}
				} else if (r == 0x1c) mock_tcs[t].enable = value;
				else if (r >= 0x30) {
					uint32_t c = (r - 0x30) / 20, f = (r - 0x30) % 20;
					if (c < 16) {
						if (f == 0x00) mock_tcs[t].cmd[c].msgid = value;
						else if (f == 0x04) mock_tcs[t].cmd[c].address = value;
						else if (f == 0x08) mock_tcs[t].cmd[c].data = value;
						else if (f == 0x0c) mock_tcs[t].cmd[c].status = value;
						else if (f == 0x10) mock_tcs[t].cmd[c].resp = value;
					}
				}
			}
		}
	}
}

int hakios_mmio_try_read32(const volatile void *addr, unsigned *out)
{
	*out = hakios_test_rd((vm_offset_t)addr);
	return 0;
}

vm_offset_t ml_io_map(vm_offset_t phys, vm_size_t size __unused) { return phys; }
vm_offset_t ml_io_map_wcomb(vm_offset_t phys, vm_size_t size __unused)
{
	return phys == MOCK_CMD_DB_PA ? (vm_offset_t)mock_cmd_db : phys;
}

static int
tcs_find(unsigned t, uint32_t address, uint32_t *data_out)
{
	for (unsigned c = 0; c < 16; c++) {
		if ((mock_tcs[t].enable & (1u << c)) && mock_tcs[t].cmd[c].address == address) {
			if (data_out) *data_out = mock_tcs[t].cmd[c].data;
			return (int)c;
		}
	}
	return -1;
}

static void
setup_board(void)
{
	memset(&test_board, 0, sizeof(test_board));
	memset(mock_tcs, 0, sizeof(mock_tcs));
	for (unsigned t = 0; t < 8; t++) mock_tcs[t].idle = 1;
	for (unsigned i = 0; i < sizeof(gate_offs) / sizeof(gate_offs[0]); i++) mock_branches[i] = 1u << 31;
	mock_gdsc_usb = mock_gdsc_pcie0 = mock_gdsc_pcie1 = mock_gdsc_ufs = 0;
	mock_gdsc_disp0 = mock_gdsc_disp2 = mock_gdsc_cam = mock_gdsc_v0 = mock_gdsc_v1 = 0;
	mock_screen_off = false;
	mock_display_off = false;
	mock_ufs_down = true;
	mock_charger = 1;
	test_features = HAKIOS_HAS_SOC_SLEEP;
	test_board.rpmh.cmd_db = MOCK_CMD_DB_PA;
	test_board.rpmh.cmd_db_size = MOCK_CMD_DB_SZ;
	test_board.rpmh.rsc = MOCK_RSC_PA;
	test_board.rpmh.tcs_offset = MOCK_TCS_OFF;
	test_board.rpmh.camcc_gdsc = MOCK_CAMCC_GDSC;
	test_board.rpmh.videocc_gdsc[0] = MOCK_VCC0_GDSC;
	test_board.rpmh.videocc_gdsc[1] = MOCK_VCC1_GDSC;
	test_board.clocks.gcc = MOCK_GCC_PA;
	test_board.clocks.dispcc = MOCK_DISPCC_PA;
	test_board.clocks.dispcc_mdss_gdscr = 0x9000;
	build_cmd_db();
	boot_args = "";
}

static void
bcm_sum_and_sets(void)
{
	uint32_t data = 0;
	int slot;

	setup_board();
	hakios_soc_sleep_init();
	CHECK(keep_on && keep_sets, "keeper & sets started");

	CHECK(tcs_find(5, 0x50000, &data) >= 0 && (data & 0x3fffu) == 10460u, "MC0 wake 2092 MHz: %#x", data);
	CHECK(tcs_find(3, 0x50000, &data) >= 0 && (data & 0x3fffu) == 2735u, "MC0 sleep holds display 547 MHz: %#x", data);

	CHECK(tcs_find(5, 0x50010, &data) >= 0 && data == BCM_VOTE(1265, 1264), "SN2 wake has loader vote: %#x", data);
	CHECK(tcs_find(3, 0x50010, &data) >= 0 && data == BCM_COMMIT, "SN2 sleep is no-request: %#x", data);

	CHECK(hakios_rpmh_bw(HAKIOS_RPMH_TOUCH, "SN0", 1, 1), "touch SN0 ok");
	CHECK(tcs_find(5, 0x5000c, &data) >= 0 && data == BCM_VOTE(2, 1), "SN0 wake summed cpu(1/1)+touch(1/1): %#x", data);
	CHECK(tcs_find(3, 0x5000c, &data) >= 0 && data == BCM_VOTE(1, 1), "SN0 sleep kept touch(1/1): %#x", data);

	CHECK(hakios_rpmh_bw(HAKIOS_RPMH_TOUCH, "SN0", 0, 0), "touch SN0 release ok");
	CHECK(tcs_find(3, 0x5000c, &data) >= 0 && data == BCM_COMMIT, "SN0 sleep released: %#x", data);
	slot = tcs_find(3, 0x50008, &data);
	CHECK(slot >= 0 && (data & 0x3fffu) == BCM_ACV_APPS, "ACV held by display while screen on: %#x", data);
	CHECK(hakios_soc_sleep_quiesce(), "quiesce returns true");
}

static void
floor_follow_and_screen_off(void)
{
	uint32_t data = 0;
	uint32_t low[3] = { 800000, 800000, 800000 };
	uint32_t mid[3] = { 1200000, 1400000, 1400000 };
	uint32_t high[3] = { 1900000, 2700000, 2700000 };

	setup_board();
	hakios_soc_sleep_init();

	hakios_soc_sleep_follow(low, 3);
	CHECK(floor_ddr == 547 && floor_llcc == 300, "low row 547/300: %u/%u", floor_ddr, floor_llcc);
	hakios_soc_sleep_follow(mid, 3);
	CHECK(floor_ddr == 1017 && floor_llcc == 466, "mid row 1017/466: %u/%u", floor_ddr, floor_llcc);
	hakios_soc_sleep_follow(high, 3);
	CHECK(floor_ddr == 3196 && floor_llcc == 1066, "top row 3196/1066: %u/%u", floor_ddr, floor_llcc);

	mock_screen_off = true;
	hakios_soc_sleep_follow(low, 3);
	CHECK(floor_ddr == 200 && floor_llcc == 300, "screen-off floor 200/300: %u/%u", floor_ddr, floor_llcc);
	CHECK(tcs_find(3, 0x50000, &data) >= 0 && data == BCM_COMMIT, "MC0 sleep drops to no-request: %#x", data);
	CHECK(tcs_find(3, 0x50008, &data) >= 0 && data == BCM_COMMIT, "ACV sleep drops to no-request: %#x", data);

	hakios_display_bandwidth(true);
	CHECK(tcs_find(3, 0x50000, &data) >= 0 && (data & 0x3fffu) == 2735u, "scanout holds MC0 in sleep: %#x", data);
	hakios_display_bandwidth(false);
	CHECK(tcs_find(3, 0x50000, &data) >= 0 && data == BCM_COMMIT, "scanout end releases MC0: %#x", data);

	CHECK(hakios_soc_sleep_quiesce(), "quiesce ok");
	CHECK(mock_tcs[3].enable == 0 && mock_tcs[5].enable == 0, "TCS 3/5 cleared");
	CHECK(mock_tcs[4].enable == 0 && mock_tcs[6].enable == 0, "ARC TCS 4/6 disarmed");
}

static void
llcc_down_gating_and_collapse(void)
{
	uint32_t low[3] = { 800000, 800000, 800000 };

	setup_board();
	hakios_soc_sleep_init();
	CHECK(arc_ready, "ARC default entries ready");

	hakios_rpmh_llcc_down();
	CHECK(mock_tcs[4].enable == 0 && arc_count.screen == 1, "screen on holds ARC off");

	mock_screen_off = true;
	hakios_rpmh_llcc_down();
	CHECK(mock_tcs[4].enable == 0 && arc_count.usb == 1, "USB in holds ARC off");

	mock_charger = 0;
	hakios_soc_sleep_follow(low, 3);

	CHECK(hakios_pcie_bandwidth(true), "PCIe bw acquired");
	hakios_rpmh_llcc_down();
	CHECK(mock_tcs[4].enable == 0 && arc_count.pcie == 1, "PCIe holds ARC off");
	CHECK(hakios_pcie_bandwidth(false), "PCIe bw released");

	mock_branches[0] = 0;
	hakios_rpmh_llcc_down();
	CHECK(mock_tcs[4].enable == 0 && arc_count.gate[0] == 1, "gate clock holds ARC off");
	mock_branches[0] = 1u << 31;

	mock_display_off = false;
	hakios_rpmh_llcc_down();
	CHECK(mock_tcs[4].enable == 0x7u && mock_tcs[6].enable == 0x7u, "ARC TCS 4/6 armed: %#x/%#x",
	    mock_tcs[4].enable, mock_tcs[6].enable);
	CHECK(mock_tcs[4].cmd[1].data == 2u && mock_tcs[4].cmd[2].data == 1u,
	    "display on keeps CX at retention (mmcx=%u cx=%u)", mock_tcs[4].cmd[1].data, mock_tcs[4].cmd[2].data);

	mock_display_off = true;
	mock_gdsc_usb = 1u << 31;
	hakios_rpmh_llcc_down();
	CHECK(mock_tcs[4].cmd[2].data == 1u && arc_count.gdsc_held[0] == 1, "USB GDSC holds CX at retention");

	mock_gdsc_usb = 0;
	hakios_rpmh_llcc_down();
	CHECK(mock_tcs[4].cmd[1].data == 0u && mock_tcs[4].cmd[2].data == 0u && arc_count.collapse == 1,
	    "CX and MMCX collapsed cleanly (mmcx=%u cx=%u collapse=%llu)",
	    mock_tcs[4].cmd[1].data, mock_tcs[4].cmd[2].data, (unsigned long long)arc_count.collapse);

	CHECK(hakios_soc_sleep_quiesce(), "quiesce ok");
}

int
main(int argc, char **argv)
{
	static const struct { const char *name; void (*run)(void); } scenarios[] = {
		{ "bcm_sum_and_sets", bcm_sum_and_sets },
		{ "floor_follow_and_screen_off", floor_follow_and_screen_off },
		{ "llcc_down_gating_and_collapse", llcc_down_gating_and_collapse },
	};

	if (argc < 2) {
		for (unsigned i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
			printf("%s\n", scenarios[i].name);
		}
		return 0;
	}
	for (unsigned i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
		if (strcmp(argv[1], scenarios[i].name) == 0) {
			if (freopen("/dev/null", "w", stdout) == NULL) {
				return 2;
			}
			scenarios[i].run();
			fprintf(stderr, "%-20s %s\n", argv[1], failures ? "FAILED" : "ok");
			return failures ? 1 : 0;
		}
	}
	fprintf(stderr, "no scenario %s\n", argv[1]);
	return 2;
}
