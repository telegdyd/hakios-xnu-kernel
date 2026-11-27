/*
 * Dual NXP TFA9874 driver (drivers/board/tfa.c) tested against a TLMM + two-chip 16-bit I2C model
 * and constructed PM3 containers.
 *
 *   tfa_driver_test SCENARIO
 */
#include <unistd.h>

#include "fakekernel.h"
#include "tlmm_model.h"

extern char log_text[];

#define PIN_SDA         48
#define PIN_SCL         49
#define PIN_RST0        50
#define PIN_RST1        51
#define ADDR0           0x34
#define ADDR1           0x35

static struct {
	uint16_t reg[2][256];
	uint32_t written[2][256];
	unsigned mtp_writes;
	unsigned reset_high[2], reset_pulses[2];
	bool reset_was_high[2];
	bool absent[2];
	bool clks_stuck_low;
	/* I2C open-drain bus state machine (16-bit big-endian registers, 0x00..0xff) */
	bool active, reading, first, expect_reg, has_hi, pending_read, master_acked;
	int chip;
	uint8_t cur_reg, hi_byte, shift, out;
	unsigned bit;
	bool sda_low, scl_low, was_sda, was_scl;
	unsigned driven_high;
} tfa_hw;

static bool soc_low(unsigned pin) { return (model.cfg[pin] & 1u << 9) && !(model.io[pin] & 2); }

static void
sync_lines(void)
{
	model.input[PIN_SDA] = !(soc_low(PIN_SDA) || tfa_hw.sda_low);
	model.input[PIN_SCL] = !(soc_low(PIN_SCL) || tfa_hw.scl_low);
}

static void
tfa_reg_written(int chip, uint8_t reg, uint16_t val)
{
	if (reg >= 0xf0) {
		tfa_hw.mtp_writes++;
	}
	tfa_hw.reg[chip][reg] = val;
	tfa_hw.written[chip][reg]++;
	/* Update CLKS and MANSTATE based on PWDN (0x00 bit 0), AMPE (0x00 bit 3), MANSCONF (0x01 bit 2) */
	if (reg == 0x00 || reg == 0x01) {
		bool pwdn = (tfa_hw.reg[chip][0x00] & 1u) != 0;
		bool ampe = (tfa_hw.reg[chip][0x00] & 8u) != 0;
		bool mansconf = (tfa_hw.reg[chip][0x01] & 4u) != 0;

		if (!pwdn && !tfa_hw.clks_stuck_low) {
			tfa_hw.reg[chip][0x11] |= (1u << 7);
		} else {
			tfa_hw.reg[chip][0x11] &= ~(1u << 7);
		}
		if (!pwdn && ampe && mansconf && !tfa_hw.clks_stuck_low) {
			tfa_hw.reg[chip][0x14] = (uint16_t)((tfa_hw.reg[chip][0x14] & ~(0xfu << 3)) | (9u << 3));
		} else if (pwdn) {
			tfa_hw.reg[chip][0x14] &= ~(0xfu << 3);
		}
	}
}

static void
bus_changed(void)
{
	bool sda, scl;

	for (unsigned i = 0; i < 2; i++) {
		unsigned rpin = i == 0 ? PIN_RST0 : PIN_RST1;
		bool high = (model.cfg[rpin] & 1u << 9) && (model.io[rpin] & 2);

		if (high) {
			tfa_hw.reset_high[i]++;
		}
		if (tfa_hw.reset_was_high[i] && !high && (model.cfg[rpin] & 1u << 9)) {
			tfa_hw.reset_pulses[i]++;
		}
		tfa_hw.reset_was_high[i] = high;
	}
	for (unsigned pin = PIN_SDA; pin <= PIN_SCL; pin++) {
		tfa_hw.driven_high += (model.cfg[pin] & 1u << 9) && (model.io[pin] & 2);
	}
	sync_lines();
	sda = model.input[PIN_SDA];
	scl = model.input[PIN_SCL];
	if (scl && tfa_hw.was_scl && tfa_hw.was_sda && !sda) {
		tfa_hw.active = true;
		tfa_hw.first = true;
		tfa_hw.reading = tfa_hw.pending_read = false;
		tfa_hw.expect_reg = false;
		tfa_hw.has_hi = false;
		tfa_hw.bit = 0;
		tfa_hw.shift = 0;
		tfa_hw.sda_low = false;
	} else if (scl && tfa_hw.was_scl && !tfa_hw.was_sda && sda) {
		tfa_hw.active = false;
		tfa_hw.sda_low = false;
	} else if (scl && !tfa_hw.was_scl && tfa_hw.active) {
		if (tfa_hw.bit < 8) {
			if (!tfa_hw.reading) {
				tfa_hw.shift = (uint8_t)(tfa_hw.shift << 1 | sda);
			}
		} else if (tfa_hw.reading) {
			tfa_hw.master_acked = !sda;
		}
		tfa_hw.bit++;
	} else if (!scl && tfa_hw.was_scl && tfa_hw.active) {
		if (tfa_hw.bit == 8) {
			if (!tfa_hw.reading) {
				bool ours = true;

				if (tfa_hw.first) {
					uint8_t addr = tfa_hw.shift >> 1;

					tfa_hw.chip = addr == ADDR0 ? 0 : (addr == ADDR1 ? 1 : -1);
					ours = tfa_hw.chip >= 0 && !tfa_hw.absent[tfa_hw.chip];
					tfa_hw.pending_read = tfa_hw.shift & 1;
					tfa_hw.expect_reg = !tfa_hw.pending_read;
					tfa_hw.has_hi = false;
					tfa_hw.first = false;
				} else if (tfa_hw.expect_reg) {
					tfa_hw.cur_reg = tfa_hw.shift;
					tfa_hw.expect_reg = false;
				} else if (!tfa_hw.has_hi) {
					tfa_hw.hi_byte = tfa_hw.shift;
					tfa_hw.has_hi = true;
				} else {
					uint16_t val = (uint16_t)((tfa_hw.hi_byte << 8) | tfa_hw.shift);

					tfa_reg_written(tfa_hw.chip, tfa_hw.cur_reg++, val);
					tfa_hw.has_hi = false;
				}
				tfa_hw.active = ours;
				tfa_hw.sda_low = ours;
			} else {
				tfa_hw.sda_low = false;
			}
		} else if (tfa_hw.bit == 9) {
			tfa_hw.bit = 0;
			tfa_hw.shift = 0;
			tfa_hw.sda_low = false;
			if (tfa_hw.pending_read) {
				tfa_hw.reading = true;
				tfa_hw.pending_read = false;
				tfa_hw.has_hi = false;
				tfa_hw.master_acked = true;
			}
			if (tfa_hw.reading) {
				if (tfa_hw.master_acked && tfa_hw.chip >= 0) {
					uint16_t val = tfa_hw.reg[tfa_hw.chip][tfa_hw.cur_reg];

					if (!tfa_hw.has_hi) {
						tfa_hw.out = (uint8_t)(val >> 8);
						tfa_hw.has_hi = true;
					} else {
						tfa_hw.out = (uint8_t)val;
						tfa_hw.has_hi = false;
						tfa_hw.cur_reg++;
					}
					tfa_hw.sda_low = !(tfa_hw.out & 0x80);
				} else {
					tfa_hw.active = false;
				}
			}
		} else if (tfa_hw.reading) {
			tfa_hw.sda_low = !(tfa_hw.out & (0x80 >> tfa_hw.bit));
		}
		sync_lines();
	}
	tfa_hw.was_sda = model.input[PIN_SDA];
	tfa_hw.was_scl = model.input[PIN_SCL];
}

uint32_t hakios_test_rd(vm_offset_t address) { return tlmm_model_rd(address); }
void hakios_test_wr(vm_offset_t address, uint32_t value) { tlmm_model_wr(address, value); }
vm_offset_t ml_io_map(vm_offset_t phys, vm_size_t size __unused) { return phys; }

static uint32_t soft_below = UINT32_MAX;
static bool soft_muted = true;
static unsigned soft_calls;
void
hakios_sound_soft_level(uint32_t below, bool muted)
{
	soft_below = below;
	soft_muted = muted;
	soft_calls++;
}

#pragma GCC diagnostic ignored "-Wformat"
#define KERNEL 1
#define HAKIOS_MMIO_TEST 1
#include "board/board.h"
static struct hakios_board test_board;
const struct hakios_board *hakios_board(void) { return &test_board; }
bool hakios_board_has(uint64_t feature) { return (test_board.features & feature) != 0; }
#include "board/tfa.c"
#undef printf

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; fprintf(stderr, "FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond); \
	fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } } while (0)

static void
put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static void
put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static size_t
build_pm3(uint8_t *buf, size_t cap, bool with_zero_mask)
{
	size_t off;
	uint32_t str_speaker, bf_item[2], reg_item[2], zero_item, prof[2], dev[2];

	memset(buf, 0, cap);
	buf[0] = 'P'; buf[1] = 'M'; buf[2] = '3';
	memcpy(buf + 0x20, "9874", 4);
	put16(buf + 0x28, 2);

	off = 0x40;
	str_speaker = (uint32_t)off;
	memcpy(buf + off, "speaker", 8);
	off += 8;

	for (unsigned d = 0; d < 2; d++) {
		bf_item[d] = (uint32_t)off;
		put16(buf + off, 0x0001);               /* value 1 */
		put16(buf + off + 2, BF(0x04, 2, 2));   /* register 0x04 bits 2..3 */
		off += 4;

		reg_item[d] = (uint32_t)off;
		buf[off] = 0x05;                        /* reg 0x05 */
		put16(buf + off + 1, (uint16_t)(0x1230 + d));
		put16(buf + off + 3, 0xffff);
		off += 5;
	}
	zero_item = (uint32_t)off;
	buf[off] = 0x06;
	put16(buf + off + 1, 0x9999);
	put16(buf + off + 3, 0x0000);
	off += 5;

	for (unsigned d = 0; d < 2; d++) {
		uint32_t count = with_zero_mask ? 3 : 2;

		prof[d] = (uint32_t)off;
		put32(buf + off, count);
		put32(buf + off + 4, (DSC_STRING << 24) | str_speaker);
		put32(buf + off + 8, (DSC_REGISTER << 24) | reg_item[d]);
		if (with_zero_mask) {
			put32(buf + off + 12, (DSC_REGISTER << 24) | zero_item);
		}
		off += 8 + 4 * (count - 1);

		dev[d] = (uint32_t)off;
		buf[off] = 2;                           /* length */
		buf[off + 2] = d == 0 ? ADDR0 : ADDR1;
		put32(buf + off + 12, (DSC_BITFIELD << 24) | bf_item[d]);
		put32(buf + off + 16, (DSC_PROFILE << 24) | prof[d]);
		off += 12 + 8;

		put32(buf + 0x2e + 4 * d, (DSC_DEVICE << 24) | dev[d]);
	}
	return off;
}

static void
setup_board(void)
{
	test_board = (struct hakios_board){
		.name = "tfa-test",
		.features = HAKIOS_HAS_SPEAKER_TFA9874,
		.pins = { .tlmm = 0x0f100000 },
		.speaker = {
			.sda = PIN_SDA,
			.scl = PIN_SCL,
			.address = { ADDR0, ADDR1 },
			.reset = { PIN_RST0, PIN_RST1 },
			.has_resets = 1,
			.sound_pin = { 121, 123, 122, 124 },
			.sound_function = 1,
			.tdm = 2,
		},
	};
	tlmm = 0;
	broken = false;
	resets_released = false;
	memset(&tfa_hw, 0, sizeof(tfa_hw));
	for (unsigned d = 0; d < 2; d++) {
		tfa_hw.reg[d][REG_REVISION] = TFA_REVISION;
		tfa_hw.reg[d][0x02] = 0x0108;   /* AUDFS=8, FRACTDEL=8 */
		tfa_hw.reg[d][0xfb] = 0x1234;   /* MTP key seed */
	}
	memset(&model, 0, sizeof(model));
	tlmm_model_at((uint32_t)hakios_board()->pins.tlmm);
	model.input[PIN_SDA] = true;
	model.input[PIN_SCL] = true;
	model.after_write = bus_changed;
}

static void
prepare_and_mtp_guard(void)
{
	uint8_t cnt[512];
	size_t size = build_pm3(cnt, sizeof(cnt), true);

	setup_board();
	CHECK(hakios_tfa_wanted(), "negroni wants TFA9874");
	CHECK(hakios_tfa_prepare(cnt, size), "prepared both TFA9874s: %s", log_text);
	CHECK(tfa_hw.reset_pulses[0] == 1 && tfa_hw.reset_pulses[1] == 1, "both resets pulsed");
	CHECK(tfa_hw.mtp_writes == 0, "no MTP writes during prepare: %u", tfa_hw.mtp_writes);
	for (unsigned d = 0; d < 2; d++) {
		CHECK(tfa_hw.reg[d][0x05] == (uint16_t)(0x1230 + d), "profile reg 0x05 applied (%#x)", tfa_hw.reg[d][0x05]);
		CHECK(tfa_hw.written[d][0x06] == 0, "zero-mask register 0x06 skipped");
		CHECK((tfa_hw.reg[d][0x04] & 0x0c) == 0x04, "device bitfield applied (%#x)", tfa_hw.reg[d][0x04]);
		CHECK(tfa_hw.reg[d][0xa0] == (0x1234 ^ 0x005a) && tfa_hw.reg[d][0xa1] == 0x005a, "keys unlocked properly");
	}
	CHECK(!reg_put(ADDR0, 0xf5, 0x1234) && tfa_hw.mtp_writes == 0, "write >= 0xf0 refused cleanly");
	CHECK(tfa_hw.driven_high == 0, "I2C lines never driven high");
}

static void
container_rejected_when_malformed(void)
{
	uint8_t cnt[512];
	size_t size = build_pm3(cnt, sizeof(cnt), false);

	setup_board();
	CHECK(!hakios_tfa_prepare(NULL, 0), "NULL container rejected");
	CHECK(!hakios_tfa_prepare(cnt, 0x20), "short container rejected");
	cnt[0] = 'X';
	CHECK(!hakios_tfa_prepare(cnt, size), "bad magic rejected");
	cnt[0] = 'P';
	cnt[0x20] = '7';
	CHECK(!hakios_tfa_prepare(cnt, size), "non-9874 container rejected");
	CHECK(tfa_hw.reset_pulses[0] == 0, "hardware untouched on invalid container");
}

static void
start_volume_stop(void)
{
	uint8_t cnt[512];
	size_t size = build_pm3(cnt, sizeof(cnt), false);

	setup_board();
	CHECK(hakios_tfa_prepare(cnt, size), "prepared");
	CHECK(hakios_tfa_start(), "started: %s", log_text);
	CHECK((tfa_hw.reg[0][REG_STATUS_FLAGS4] >> 3 & 0xf) == 9 &&
	    (tfa_hw.reg[1][REG_STATUS_FLAGS4] >> 3 & 0xf) == 9, "both in MANSTATE 9");
	CHECK(hakios_tfa_volume(48, 0) && soft_calls == 1 && soft_below == 48 && !soft_muted,
	    "volume forwarded to hakios_sound_soft_level");
	CHECK(hakios_tfa_healthy(), "healthy when no alarm");
	hakios_tfa_stop();
	CHECK((tfa_hw.reg[0][REG_STATUS_FLAGS4] >> 3 & 0xf) == 0 &&
	    (tfa_hw.reg[1][REG_STATUS_FLAGS4] >> 3 & 0xf) == 0, "stopped to MANSTATE 0");

	/* stuck clocks prevent start */
	tfa_hw.clks_stuck_low = true;
	tfa_hw.reg[0][0x01] = 0;
	CHECK(!hakios_tfa_start(), "start fails when CLKS stays 0");
}

static void
healthy_alarm_and_quiesce(void)
{
	uint8_t cnt[512];
	size_t size = build_pm3(cnt, sizeof(cnt), false);

	setup_board();
	CHECK(hakios_tfa_prepare(cnt, size) && hakios_tfa_start(), "started");
	tfa_hw.reg[1][REG_STATUS_FLAGS0] = 0x28;        /* MANALARM | OCDS */
	CHECK(!hakios_tfa_healthy(), "alarm triggers unhealthy");
	CHECK((tfa_hw.reg[0][REG_STATUS_FLAGS4] >> 3 & 0xf) == 0 &&
	    (tfa_hw.reg[1][REG_STATUS_FLAGS4] >> 3 & 0xf) == 0, "both powered down on alarm");

	/* missing I2C response triggers quiesce resetting both GPIOs high without I2C */
	setup_board();
	CHECK(hakios_tfa_prepare(cnt, size) && hakios_tfa_start(), "started again");
	tfa_hw.absent[0] = true;
	CHECK(!hakios_tfa_healthy(), "absent I2C triggers unhealthy");
	CHECK((model.io[PIN_RST0] & 2) != 0 && (model.io[PIN_RST1] & 2) != 0,
	    "quiesce drove both active-high resets high");

	setup_board();
	tfa_hw.absent[1] = true;
	CHECK(!hakios_tfa_prepare(cnt, size), "absent second amp fails prepare");
}

int
main(int argc, char **argv)
{
	static const struct {
		const char *name;
		void (*run)(void);
	} scenarios[] = {
		{ "prepare_and_mtp_guard", prepare_and_mtp_guard },
		{ "container_rejected_when_malformed", container_rejected_when_malformed },
		{ "start_volume_stop", start_volume_stop },
		{ "healthy_alarm_and_quiesce", healthy_alarm_and_quiesce },
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
			fprintf(stderr, "%-36s %s\n", argv[1], failures ? "FAILED" : "ok");
			return failures ? 1 : 0;
		}
	}
	fprintf(stderr, "no scenario %s\n", argv[1]);
	return 2;
}
