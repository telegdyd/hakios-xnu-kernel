/*
 * Dual NXP TFA9874 speaker amplifiers (feature "speaker-tfa9874"), handed the speaker calls
 * from board/speaker.c when present on the board.
 *
 * The two amplifiers sit on a board-defined I2C bus (e.g. QUP SE13 GPIO48/49 on OnePlus 10 Pro:
 * 0x34 channel 0 / top speaker, 0x35 channel 1 / bottom speaker) with active-high reset GPIOs
 * (50 and 51) and TDM / tertiary MI2S audio pins. Android's revision register reads 0x0c74 for
 * both ("TFA9874 detected").
 *
 * A TFA9874 has no DSP of its own ("probus"): Android's NXP algorithm runs on the audio DSP, and
 * its kernel driver only writes registers. What it writes, and in which order, follows
 * vendor/oplus/kernel/audio/asoc/codecs/tfa98xx-v6 (tfa_dsp_v6.c, tfa_init_v6.c,
 * tfa_container_v6.c), with the values of /fw/tfa98xx.cnt ("speaker" profile):
 *
 *   prepare  per amplifier: reset pulsed (1, 5 ms, 0, 5 ms) and revision checked; sample rate
 *            and V/I delay fields saved; I2C reset (register 0, bit 1); DSP reset bit (RST,
 *            0x90 bit 0); keys unlocked and chip "optimal settings" for 0x0c74; saved fields
 *            restored; container device list (up to its patch) and "speaker" profile (up to its
 *            default section). Left powered down.
 *   start    only while the bit clock runs: if cold (MANSCONF 0), init state (MANSCONF 1, PWDN 0,
 *            wait CLKS); then operating (PWDN 0, MANSCONF 1, AMPE 1); wait MANSTATE 9 and check
 *            the alarm flag.
 *   volume   amplifiers stay at the container's gain; attenuation is applied to the samples in
 *            board/sound.c after the guard (hakios_sound_soft_level()).
 *   stop     PWDN 1 and AMPE 0, then wait for MANSTATE 0 (powerdown).
 *   healthy  register 0x10: alarm bit 5 (and OCP bit 3 set, OTP bit 2 clear, UVP bit 4 clear);
 *            on any fault or I2C failure, power down both.
 *   quiesce  both active-high reset GPIOs driven high immediately, without touching I2C.
 *
 * Nothing at 0xF0 or above (the MTP and its calibration) is ever written: reg_put() refuses it.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/board/audio.h>

#include "kernel.h"
#include "board.h"
#include "log.h"
#include "mmio.h"
#include "time.h"

static vm_offset_t tlmm;
static bool leaving;                    /* on a panic's way out: nothing sleeps, nothing is said but to the loader */
HAKIOS_DEFINE_SAY(!leaving)

#define wait_ms(ms)             hakios_wait_ms((ms), leaving)

#define TLMM_PHYS               (hakios_board()->pins.tlmm)
#define HAKIOS_TLMM             tlmm
#include "tlmm.h"
#include "i2c_gpio.h"

/* ---- I2C and pins ------------------------------------------------------------------------------------ */

static struct i2c_gpio i2c_bus;
static bool powering_down;

static bool
map(void)
{
	if (TLMM_PHYS == 0) {
		return false;
	}
	if (tlmm == 0) {
		tlmm = ml_io_map(TLMM_PHYS, TLMM_SIZE);
	}
	return tlmm != 0;
}

/* A reset pin as a GPIO output, 2 mA, no pull, at `high'. */
static void
reset_pin(unsigned pin, bool high)
{
	gpio_output(pin, high, PULL_NONE, 2);
}

static bool
bus_take(void)
{
	unsigned clocks;

	if (i2c_gpio_take(&i2c_bus)) {
		return true;
	}
	if (powering_down && i2c_gpio_unwedge(&i2c_bus, &clocks)) {
		say("tfa: the I2C bus was held low: an amplifier let go of the data line after %u clocks\n", clocks);
		return true;
	}
	say("tfa: the I2C bus is not at rest (gpio%u reads %u, gpio%u reads %u): %s\n", i2c_bus.sda, i2c_gpio_level(i2c_bus.sda),
	    i2c_bus.scl, i2c_gpio_level(i2c_bus.scl), powering_down ? "nine clocks did not free it" : "not touched");
	return false;
}

/* A 16-bit register, most significant byte first (tfa98xx_read_register16_v6). False if nobody answers. */
static bool
reg_read(uint8_t address, uint8_t reg, uint16_t *value)
{
	bool answered;

	if (!bus_take()) {
		return false;
	}
	i2c_gpio_start(&i2c_bus);
	answered = i2c_gpio_write(&i2c_bus, (uint8_t)(address << 1)) && i2c_gpio_write(&i2c_bus, reg);
	if (answered) {
		i2c_gpio_start(&i2c_bus);
		answered = i2c_gpio_write(&i2c_bus, (uint8_t)(address << 1 | 1));
	}
	if (answered) {
		*value = (uint16_t)(i2c_gpio_read(&i2c_bus, true) << 8);
		*value |= i2c_gpio_read(&i2c_bus, false);
	}
	i2c_gpio_stop(&i2c_bus);
	i2c_gpio_give_back(&i2c_bus);
	return answered;
}

static bool
reg_put(uint8_t address, uint8_t reg, uint16_t value)
{
	bool answered;

	if (reg >= 0xf0) {
		say("tfa: %#x: A WRITE OF %#06x TO REGISTER %#04x (THE MTP) IS REFUSED\n", address, value, reg);
		return false;
	}
	if (!bus_take()) {
		return false;
	}
	i2c_gpio_start(&i2c_bus);
	answered = i2c_gpio_write(&i2c_bus, (uint8_t)(address << 1)) && i2c_gpio_write(&i2c_bus, reg) &&
	    i2c_gpio_write(&i2c_bus, (uint8_t)(value >> 8)) && i2c_gpio_write(&i2c_bus, (uint8_t)value);
	i2c_gpio_stop(&i2c_bus);
	i2c_gpio_give_back(&i2c_bus);
	return answered;
}

/* ---- the amplifiers ---------------------------------------------------------------------------------- */

#define TFA_REVISION            0x0c74
#define REG_SYS_CONTROL0        0x00    /* PWDN bit 0, I2CR bit 1, AMPE bit 3, DCA bit 4 */
#define REG_SYS_CONTROL1        0x01    /* MANSCONF bit 2 */
#define REG_REVISION            0x03
#define REG_STATUS_FLAGS0       0x10    /* OTDS 2 (0: alarm), OCDS 3, UVDS 4 (0: alarm), MANALARM 5, NOCLK 7 */
#define REG_STATUS_FLAGS1       0x11    /* PLLS 6, CLKS 7, SWS 13, AMPS 14 */
#define REG_STATUS_FLAGS4       0x14    /* MANSTATE bits 3..6 */
#define BF(reg, pos, len)       ((uint16_t)((reg) << 8 | (pos) << 4 | ((len) - 1)))
#define BF_PWDN                 BF(0x00, 0, 1)
#define BF_AMPE                 BF(0x00, 3, 1)
#define BF_MANSCONF             BF(0x01, 2, 1)
#define BF_AUDFS                BF(0x02, 0, 4)
#define BF_FRACTDEL             BF(0x02, 5, 6)
#define BF_RST                  BF(0x90, 0, 1)  /* the generic TFA2 RST the driver writes on every family-2 chip */
#define BF_CLKS                 BF(0x11, 7, 1)
#define BF_MANSTATE             BF(0x14, 3, 4)

/* revision 0x0c74's "optimal settings" (tfa9874_specific(), generated code) */
static const uint16_t optimal[][2] = {
	{ 0x02, 0x22c8 }, { 0x52, 0x57dc }, { 0x53, 0x003e }, { 0x56, 0x0400 }, { 0x61, 0x0110 }, { 0x6f, 0x00a5 },
	{ 0x70, 0x07f8 }, { 0x73, 0x0047 }, { 0x74, 0x5098 }, { 0x75, 0x8d28 }, { 0x80, 0x0000 }, { 0x83, 0x0799 },
	{ 0x84, 0x0081 },
};

#define MAX_WRITES              96

struct write {
	uint16_t field_or_reg;          /* a bitfield (BF()), or a register patch */
	uint16_t value, mask;           /* mask 0: a bitfield */
};

static struct amp {
	uint8_t address;
	unsigned reset;
	const char *where;
	struct write device[MAX_WRITES], profile[MAX_WRITES];
	unsigned devices, profiles;
	bool found, prepared, on, faulted;
} amps[2] = {
	{ .where = "top (channel 0)" },
	{ .where = "bottom (channel 1)" },
};

static bool broken;                     /* an amplifier did not answer: nothing more is done to either */
static bool resets_released;

static bool
wired(void)
{
	const struct hakios_board *board = hakios_board();

	if (board->pins.tlmm == 0 || !board->speaker.has_resets || board->speaker.sda == board->speaker.scl ||
	    board->speaker.address[0] == 0 || board->speaker.address[1] == 0) {
		return false;
	}
	i2c_bus.sda = board->speaker.sda;
	i2c_bus.scl = board->speaker.scl;
	amps[0].address = board->speaker.address[0];
	amps[0].reset = board->speaker.reset[0];
	amps[1].address = board->speaker.address[1];
	amps[1].reset = board->speaker.reset[1];
	return true;
}

static void
show_pin(unsigned pin, const char *what)
{
	uint32_t cfg = rd(GPIO_CFG(pin)), io = rd(GPIO_IO(pin));

	say("tfa: gpio%u (%s): function %u, pull %u, %u mA, %s, reads %u\n", pin, what, (cfg >> 2) & 15,
	    cfg & 3, (((cfg >> 6) & 7) + 1) * 2, (cfg & CFG_OUTPUT) ? (io & 2 ? "drives high" : "drives low") : "input", io & 1);
}

static void
show_pins(void)
{
	static const char *const sound[4] = { "I2S bit clock", "I2S word select", "I2S data in", "I2S data out" };

	show_pin(i2c_bus.sda, "I2C data");
	show_pin(i2c_bus.scl, "I2C clock");
	for (unsigned i = 0; i < 2; i++) {
		show_pin(amps[i].reset, amps[i].where);
	}
	for (unsigned i = 0; i < 4; i++) {
		if (hakios_board()->speaker.sound_pin[i] != 0) {
			show_pin(hakios_board()->speaker.sound_pin[i], sound[i]);
		}
	}
}

static bool
bf_get(struct amp *a, uint16_t field, uint16_t *value)
{
	uint16_t reg, len = (field & 15) + 1, pos = field >> 4 & 15;

	if (!reg_read(a->address, (uint8_t)(field >> 8), &reg)) {
		say("tfa: %#x: NO ANSWER reading register %#04x\n", a->address, field >> 8);
		broken = true;
		return false;
	}
	*value = (uint16_t)(reg >> pos & ((1u << len) - 1));
	return true;
}

/* A bitfield written (read, modified, written: tfa_set_bf_v6_volatile()); `check': read back and said if it differs. */
static bool
bf_put(struct amp *a, uint16_t field, uint16_t value, bool check, const char *what)
{
	uint16_t reg, len = (field & 15) + 1, pos = field >> 4 & 15, mask = (uint16_t)(((1u << len) - 1) << pos), back;
	uint8_t address = (uint8_t)(field >> 8);

	if (broken) {
		return false;
	}
	if (!reg_read(a->address, address, &reg)) {
		say("tfa: %#x: NO ANSWER reading register %#04x\n", a->address, address);
		broken = true;
		return false;
	}
	reg = (uint16_t)((reg & ~mask) | ((value << pos) & mask));
	say("tfa: %#x: %s: register %#04x bits %u..%u = %#x (the register %#06x)\n", a->address, what, address, pos, pos + len - 1,
	    value, reg);
	if (!reg_put(a->address, address, reg)) {
		say("tfa: %#x: THE WRITE TO %#04x WAS NOT TAKEN\n", a->address, address);
		broken = true;
		return false;
	}
	if (check && reg_read(a->address, address, &back) && (back & mask) != (reg & mask)) {
		say("tfa: %#x: register %#04x reads back %#06x (bits %u..%u: %#x, not %#x)\n", a->address, address, back, pos,
		    pos + len - 1, (back & mask) >> pos, value);
	}
	return true;
}

/* A register written whole (or under a mask: a container's register patch, tfaRunWriteRegister()). */
static bool
reg_write(struct amp *a, uint8_t address, uint16_t value, uint16_t mask, bool check, const char *what)
{
	uint16_t reg = 0, back;

	if (broken) {
		return false;
	}
	if (mask != 0xffff) {
		if (!reg_read(a->address, address, &reg)) {
			say("tfa: %#x: NO ANSWER reading register %#04x\n", a->address, address);
			broken = true;
			return false;
		}
	}
	reg = (uint16_t)((reg & ~mask) | (value & mask));
	say("tfa: %#x: %s: register %#04x = %#06x\n", a->address, what, address, reg);
	if (!reg_put(a->address, address, reg)) {
		say("tfa: %#x: THE WRITE TO %#04x WAS NOT TAKEN\n", a->address, address);
		broken = true;
		return false;
	}
	if (check && reg_read(a->address, address, &back) && back != reg) {
		say("tfa: %#x: register %#04x reads back %#06x, not %#06x\n", a->address, address, back, reg);
	}
	return true;
}

static void
apply(struct amp *a, const struct write *w, unsigned count, const char *what)
{
	for (unsigned i = 0; i < count && !broken; i++) {
		if (w[i].mask == 0) {
			bf_put(a, w[i].field_or_reg, w[i].value, true, what);
		} else {
			reg_write(a, (uint8_t)w[i].field_or_reg, w[i].value, w[i].mask, true, what);
		}
	}
}

/* Both resets pulsed as Android's driver pulses them (1, 5 ms, 0, 5 ms), and both revisions read. */
static bool
find(void)
{
	bool all = true;

	if (!wired() || !map()) {
		return false;
	}
	say("tfa: the resets (gpio%u, gpio%u) pulsed: high 5 ms, then low\n", amps[0].reset, amps[1].reset);
	reset_pin(amps[0].reset, true);
	reset_pin(amps[1].reset, true);
	wait_ms(5);
	reset_pin(amps[0].reset, false);
	reset_pin(amps[1].reset, false);
	resets_released = true;
	wait_ms(5);
	for (unsigned i = 0; i < 2; i++) {
		uint16_t revision = 0;

		amps[i].found = reg_read(amps[i].address, REG_REVISION, &revision) && revision == TFA_REVISION;
		say("tfa: %#x, the %s: %s (revision %#06x)\n", amps[i].address, amps[i].where, amps[i].found ? "A TFA9874" :
		    "NOT FOUND or not the TFA9874 Android has", revision);
		all &= amps[i].found;
	}
	return all;
}

/* ---- the container (tfa98xx.cnt) ---------------------------------------------------------------------- */

enum {
	DSC_DEVICE = 0, DSC_PROFILE = 1, DSC_REGISTER = 2, DSC_STRING = 3, DSC_FILE = 4, DSC_PATCH = 5,
	DSC_BITFIELD = 16, DSC_DEFAULT = 17,
};

static uint32_t
le32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static uint16_t
le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | p[1] << 8);
}

static bool
string_is(const uint8_t *cnt, uint64_t size, uint32_t offset, const char *what)
{
	size_t n = strlen(what);

	return (uint64_t)offset + n + 1 <= size && memcmp(cnt + offset, what, n) == 0 && cnt[offset + n] == 0;
}

/* One item of a list: a register patch or a bitfield into `w'; anything else ends the list (false). */
static bool
item(const uint8_t *cnt, uint64_t size, uint32_t word, struct write *w, unsigned *count)
{
	uint32_t offset = word & 0xffffff, type = word >> 24;

	if (type == DSC_BITFIELD) {
		if ((uint64_t)offset + 4 > size || *count >= MAX_WRITES) {
			return false;
		}
		w[*count].value = le16(cnt + offset);
		w[*count].field_or_reg = le16(cnt + offset + 2);
		w[*count].mask = 0;
		(*count)++;
		return true;
	}
	if (type == DSC_REGISTER) {
		if ((uint64_t)offset + 5 > size || *count >= MAX_WRITES) {
			return false;
		}
		w[*count].field_or_reg = cnt[offset];
		w[*count].value = le16(cnt + offset + 1);
		w[*count].mask = le16(cnt + offset + 3);
		if (w[*count].mask != 0) {
			(*count)++;                     /* (a zero mask writes nothing: skipped) */
		}
		return true;
	}
	return type != DSC_PATCH && type != DSC_FILE && type != DSC_PROFILE && type != DSC_DEFAULT;
}

/* The container's device lists for both board addresses and their "speaker" profiles, read into amps[]. */
static bool
container(const uint8_t *cnt, uint64_t size)
{
	unsigned ndev;

	if (cnt == NULL || size < 0x2e || cnt[0] != 'P' || cnt[1] != 'M' || cnt[2] != '3' ||
	    memcmp(cnt + 0x20, "9874", 4) != 0) {
		say("tfa: the container is not a TFA9874's PM3 container (it begins %02x %02x %02x)\n",
		    cnt != NULL && size ? cnt[0] : 0, cnt != NULL && size > 1 ? cnt[1] : 0,
		    cnt != NULL && size > 2 ? cnt[2] : 0);
		return false;
	}
	amps[0].devices = amps[0].profiles = 0;
	amps[1].devices = amps[1].profiles = 0;
	ndev = le16(cnt + 0x28);
	for (unsigned d = 0; d < ndev && d < 8 && (uint64_t)0x2e + 4 * d + 4 <= size; d++) {
		uint32_t dsc = le32(cnt + 0x2e + 4 * d), offset = dsc & 0xffffff, length;
		struct amp *a = NULL;
		bool writing = true;

		if (dsc >> 24 != DSC_DEVICE || (uint64_t)offset + 12 > size) {
			continue;
		}
		length = cnt[offset];
		for (unsigned i = 0; i < 2; i++) {
			if (amps[i].address == cnt[offset + 2]) {
				a = &amps[i];
			}
		}
		if (a == NULL || (uint64_t)offset + 12 + (uint64_t)4 * length > size) {
			continue;
		}
		a->devices = a->profiles = 0;
		for (uint32_t i = 0; i < length; i++) {
			uint32_t word = le32(cnt + offset + 12 + 4 * i), o = word & 0xffffff;

			/* the device list's registers and bitfields, up to its patch, file or first profile */
			if (writing && !item(cnt, size, word, a->device, &a->devices)) {
				writing = false;
			}
			if (word >> 24 == DSC_PROFILE && (uint64_t)o + 8 <= size) {
				uint32_t head = le32(cnt + o), count = head & 0xff, name = le32(cnt + o + 4) & 0xffffff;

				if (count == 0 || !string_is(cnt, size, name, "speaker") ||
				    (uint64_t)o + 8 + (uint64_t)4 * (count - 1) > size) {
					continue;
				}
				for (uint32_t j = 0; j + 1 < count; j++) {
					if (!item(cnt, size, le32(cnt + o + 8 + 4 * j), a->profile, &a->profiles)) {
						break;          /* (its default section, or a file: what is written when the profile is left) */
					}
				}
			}
		}
		say("tfa: the container for %#x: %u writes in its device list, %u in its \"speaker\" profile\n", a->address,
		    a->devices, a->profiles);
	}
	return amps[0].devices && amps[1].devices && amps[0].profiles && amps[1].profiles;
}

/* ---- what board/speaker.c hands over ----------------------------------------------------------------- */

bool
hakios_tfa_wanted(void)
{
	return hakios_board_has(HAKIOS_HAS_SPEAKER_TFA9874);
}

/* tfaRunStartup_v6() for one amplifier, up to (not including) the init state: needs no clock. */
static bool
prepare_one(struct amp *a)
{
	uint16_t audfs = 8, fractdel = 0, mtp = 0;

	if (!bf_get(a, BF_AUDFS, &audfs) || !bf_get(a, BF_FRACTDEL, &fractdel)) {
		return false;
	}
	/* tfa98xx_init_v6(): I2C reset (the register written whole), the DSP reset, then tfa9874_specific() */
	reg_write(a, REG_SYS_CONTROL0, 0x0002, 0xffff, false, "I2C reset");
	bf_put(a, BF_RST, 1, false, "the DSP reset (RST)");
	reg_write(a, 0x0f, 0x5a6b, 0xffff, false, "the keys' registers shown");
	if (!reg_read(a->address, 0xfb, &mtp)) {
		broken = true;
		return false;
	}
	reg_write(a, 0xa0, (uint16_t)(mtp ^ 0x005a), 0xffff, false, "key 1");
	reg_write(a, 0x0f, 0x5a6b, 0xffff, false, "the keys' registers shown");
	reg_write(a, 0xa1, 0x005a, 0xffff, false, "key 2 (MTPK)");
	reg_write(a, 0x0f, 0x0000, 0xffff, false, "the keys' registers hidden");
	for (unsigned i = 0; i < sizeof(optimal) / sizeof(optimal[0]) && !broken; i++) {
		reg_write(a, (uint8_t)optimal[i][0], optimal[i][1], 0xffff, true, "revision 0x0c74's settings");
	}
	bf_put(a, BF_AUDFS, audfs, true, "the sample rate, put back");
	bf_put(a, BF_FRACTDEL, fractdel, true, "the V/I delay, put back");
	apply(a, a->device, a->devices, "the container's device list");
	apply(a, a->profile, a->profiles, "the \"speaker\" profile");
	a->prepared = !broken;
	return a->prepared;
}

bool
hakios_tfa_prepare(const uint8_t *cnt, uint64_t size)
{
	broken = false;
	amps[0].found = amps[0].prepared = amps[0].on = amps[0].faulted = false;
	amps[1].found = amps[1].prepared = amps[1].on = amps[1].faulted = false;
	if (!wired()) {
		say("tfa: not wired: no pins or addresses in board.dts\n");
		return false;
	}
	if (!container(cnt, size)) {
		say("tfa: the container has no device list or \"speaker\" profile for %#x and %#x: the amplifiers are left alone\n",
		    amps[0].address, amps[1].address);
		return false;
	}
	if (!find()) {
		return false;
	}
	for (unsigned i = 0; i < 2 && !broken; i++) {
		prepare_one(&amps[i]);
	}
	if (broken) {
		hakios_tfa_quiesce();
		return false;
	}
	say("tfa: both amplifiers set up as Android sets them up (the \"speaker\" profile), powered down\n");
	return true;
}

/* tfa_dev_start() of a prepared amplifier: the init state if cold, then operating. Only while the bit clock runs. */
static bool
start_one(struct amp *a)
{
	uint16_t mansconf = 0, clks = 0, state = 0, flags0 = 0;

	if (!bf_get(a, BF_MANSCONF, &mansconf)) {
		return false;
	}
	if (mansconf == 0) {
		bf_put(a, BF_MANSCONF, 1, false, "the init state: I2C configured (MANSCONF)");
		bf_put(a, BF_PWDN, 0, false, "the init state: powered up (PWDN 0)");
		for (unsigned ms = 0; ms < 100 && !broken; ms += 2) {
			if (bf_get(a, BF_CLKS, &clks) && clks) {
				break;
			}
			wait_ms(2);
		}
		if (!clks) {
			say("tfa: %#x: THE CLOCKS ARE NOT STABLE (CLKS 0) 100 ms after power-up: is the bit clock running?\n", a->address);
			return false;
		}
	}
	bf_put(a, BF_PWDN, 0, false, "operating: PWDN 0");
	bf_put(a, BF_MANSCONF, 1, false, "operating: MANSCONF 1");
	bf_put(a, BF_AMPE, 1, false, "operating: the amplifier on (AMPE 1)");
	for (unsigned ms = 0; ms < 200 && !broken; ms += 5) {
		if (bf_get(a, BF_MANSTATE, &state) && state == 9) {
			break;
		}
		wait_ms(5);
	}
	reg_read(a->address, REG_STATUS_FLAGS0, &flags0);
	say("tfa: %#x: the manager's state %u%s, status %#06x\n", a->address, state, state == 9 ? " (operating)" : " (NOT OPERATING)",
	    flags0);
	a->on = state == 9 && !(flags0 & 0x20);
	return a->on;
}

static void
stop_one(struct amp *a)
{
	uint16_t state = 0;

	if (!a->prepared) {
		return;
	}
	bf_put(a, BF_PWDN, 1, false, "powered down (PWDN 1)");
	bf_put(a, BF_AMPE, 0, false, "the amplifier off (AMPE 0)");
	for (unsigned i = 0; i < 20; i++) {
		if (!bf_get(a, BF_MANSTATE, &state) || state == 0) {
			break;
		}
		wait_ms(5);
	}
	say("tfa: %#x: %s (the manager's state %u)\n", a->address, state == 0 ? "in powerdown" : "NOT IN POWERDOWN", state);
	a->on = false;
}

bool
hakios_tfa_start(void)
{
	if (broken || !amps[0].prepared || !amps[1].prepared) {
		return false;
	}
	for (unsigned i = 0; i < 2; i++) {
		if (!start_one(&amps[i])) {
			hakios_tfa_stop();
			return false;
		}
	}
	say("tfa: BOTH AMPLIFIERS OPERATE, at the container's gain; the level under it is the samples' (sound.c)\n");
	return true;
}

bool
hakios_tfa_volume(uint32_t below, __unused uint32_t step_ms)
{
	hakios_sound_soft_level(below, false);
	return !broken;
}

/* OnePlus's check (tfa98xx_v6.c): register 0x10, bit 5 the alarm; on one, both powered down. */
bool
hakios_tfa_healthy(void)
{
	for (unsigned i = 0; i < 2; i++) {
		uint16_t flags0 = 0;

		if (!amps[i].on) {
			continue;
		}
		if (!reg_read(amps[i].address, REG_STATUS_FLAGS0, &flags0)) {
			say("tfa: %#x: NO ANSWER to the status read: both are powered down\n", amps[i].address);
			broken = true;
			hakios_tfa_quiesce();
			return false;
		}
		if (flags0 & 0x20) {
			say("tfa: %#x: IN ALARM (status %#06x:%s%s%s%s): both are powered down\n", amps[i].address, flags0,
			    flags0 & 0x08 ? " over-current" : "", !(flags0 & 0x04) ? " over-temperature" : "",
			    !(flags0 & 0x10) ? " under-voltage" : "", flags0 & 0x80 ? " no clock" : "");
			amps[i].faulted = true;
			hakios_tfa_stop();
			return false;
		}
	}
	return true;
}

uint32_t
hakios_tfa_monitor(bool tell)
{
	if (tell) {
		for (unsigned i = 0; i < 2; i++) {
			uint16_t f0 = 0, f1 = 0, f4 = 0;

			if (amps[i].on && reg_read(amps[i].address, REG_STATUS_FLAGS0, &f0) &&
			    reg_read(amps[i].address, REG_STATUS_FLAGS1, &f1) && reg_read(amps[i].address, REG_STATUS_FLAGS4, &f4)) {
				say("tfa: %#x: status %#06x %#06x, the manager's state %u\n", amps[i].address, f0, f1, (f4 >> 3) & 15);
			}
		}
	}
	return 0;                       /* (no table of reductions for these: the guard alone) */
}

void
hakios_tfa_stop(void)
{
	powering_down = true;
	for (unsigned i = 0; i < 2; i++) {
		stop_one(&amps[i]);
	}
	powering_down = false;
}

void
hakios_tfa_quiesce(void)
{
	leaving = true;
	if (resets_released && wired() && map()) {
		reset_pin(amps[0].reset, true);
		reset_pin(amps[1].reset, true);
		resets_released = false;
	}
	for (unsigned i = 0; i < 2; i++) {
		amps[i].found = false;
		amps[i].on = false;
		amps[i].prepared = false;
	}
	leaving = false;
}

static void
probe_thread(void *parameter, __unused wait_result_t result)
{
	uint32_t probe = (uint32_t)(uintptr_t)parameter, wait_s = 0;

	/* boot-arg audio_wait=S (described at adsp.c): look S seconds after start, when the USB log is up */
	if (PE_parse_boot_argn("audio_wait", &wait_s, sizeof(wait_s)) && wait_s != 0) {
		delay_for_interval(wait_s * 1000, NSEC_PER_MSEC);
		say("tfa: audio_wait=%u: the amplifiers' look now\n", wait_s);
	}
	if (!wired() || !map()) {
		thread_terminate(current_thread());
		return;
	}
	say("tfa: speaker_probe=%u: the amplifiers' pins%s\n", probe, probe >= 2 ? ", and who they are" : "");
	show_pins();
	if (probe >= 2) {
		find();
		show_pins();
	}
	thread_terminate(current_thread());
}

void
hakios_tfa_probe(uint32_t probe)
{
	thread_t thread;

	if (kernel_thread_start(probe_thread, (void *)(uintptr_t)probe, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
