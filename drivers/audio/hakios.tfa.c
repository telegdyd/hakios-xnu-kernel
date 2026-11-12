/*
 * The speakers of the OnePlus 10 Pro (negroni, boot-arg speaker_soc=8450): two NXP TFA9874 amplifiers.
 * Every address below belongs to this phone (its live device tree), and nothing is touched unless a
 * boot-arg asks for it. hakios.speaker.c hands its public functions here (sys/hakios.audio.h).
 *
 * The two amplifiers sit alone on I2C of QUP serial engine 13 (i2c@a94000: GPIO48 data, GPIO49 clock,
 * function qup13): 0x34 (tfa_channel 0, the top speaker, reset GPIO50) and 0x35 (channel 1, the bottom
 * one, reset GPIO51). Their resets are active-high. Both are fed by the tertiary MI2S interface,
 * which the audio DSP drives (hakios.sound.c, sound_soc=8450): 0x34 takes slot 0, 0x35 slot 1.
 * Android's revision register reads 0x0c74 for both ("TFA9874 detected").
 *
 * A TFA9874 has no DSP of its own ("probus"): Android's NXP algorithm runs on the audio DSP, and its
 * kernel driver only writes registers. What it writes, and in which order, is the phone's own kernel's
 * (OnePlusOSS android_kernel_modules_and_devicetree_oneplus_sm8450, vendor/oplus/kernel/audio/asoc/
 * codecs/tfa98xx-v6: tfa_dsp_v6.c tfa_dev_start(), tfaRunStartup_v6(), tfa_dev_set_state(),
 * tfa_dev_stop(); tfa_init_v6.c tfa9874_specific(); tfa_container_v6.c), with the values of the
 * phone's container /odm/firmware/tfa98xx.cnt ("speaker" profile), which is not in this repository
 * (sweet-sh.py --put puts it into /private/var/fw):
 *
 *   prepare  per amplifier: the reset pulsed (1, 5 ms, 0, 5 ms) and the revision checked; the
 *            sample rate and V/I delay fields read; I2C reset (register 0, bit 1); the DSP
 *            reset bit (generic TFA2 field RST, 0x90 bit 0, as the driver sets it); the keys
 *            unlocked and the chip's "optimal settings" for revision 0x0c74; the two fields put
 *            back; then the container's device list (registers and bitfields up to its patch) and
 *            the "speaker" profile up to its default section. Left powered down.
 *   start    only while the bit clock runs: if cold (MANSCONF 0), the init state (MANSCONF 1, PWDN
 *            0, then CLKS waited for); then operating (PWDN 0, MANSCONF 1, AMPE 1); the manager's
 *            state looked at (9: operating) and the alarm flag
 *   volume   the amplifiers stay at the container's gain (Android's own level); what is to be under
 *            it is taken off the samples, after the speakers' guard (hakios_sound_soft_level()):
 *            nothing here depends on which way TDMSPKG counts
 *   stop     PWDN 1 and AMPE 0, then the manager waited for to reach powerdown (state 0)
 *   healthy  register 0x10: the alarm bit (5), and under it OCP (bit 3 set), OTP (bit 2 clear), UVP
 *            (bit 4 clear), as OnePlus's monitor reads it; on one both amplifiers are powered down
 *   quiesce  both resets driven high: silent at once, without I2C
 *
 * Nothing at 0xF0 or above (the MTP and its calibration) is ever written: a write there is refused,
 * whatever the container says. Every write is said; container and init writes are read back.
 *
 *   speaker_soc=8450 speaker_probe=1   the pins only, read
 *   speaker_soc=8450 speaker_probe=2   ... and the resets pulsed and both revisions read, then left as
 *                                      Android leaves them (out of reset, powered down)
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/hakios.audio.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern void loader_log_putc(char c);

static vm_offset_t tlmm;
static bool leaving;                    /* on a panic's way out: nothing sleeps */

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[300];
	va_list list;
	int length;

	va_start(list, format);
	length = vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	if (!leaving) {
		printf("%s", line);
	}
	return length;
}

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
}

static void
spin_us(uint32_t us)
{
	uint64_t until;

	nanoseconds_to_absolutetime((uint64_t)us * NSEC_PER_USEC, &until);
	until += mach_absolute_time();
	while (mach_absolute_time() < until) {
	}
}

static void
wait_ms(uint32_t ms)
{
	if (leaving) {
		spin_us(ms * 1000);
	} else {
		delay_for_interval(ms, NSEC_PER_MSEC);
	}
}

/* ---- pins ------------------------------------------------------------------------------------------ */

#define TLMM_PHYS               0x0f100000UL    /* tlmm@f100000 */
#define TLMM_SIZE               0x100000UL
#define GPIO_CFG(n)             (tlmm + 0x1000UL * (n))         /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(n)              (tlmm + 0x1000UL * (n) + 4)     /* bit 0 in, bit 1 out */
#define CFG_PULL_UP             3
#define CFG_OUTPUT              (1u << 9)

#define PIN_SDA                 48
#define PIN_SCL                 49

static const struct {
	unsigned pin;
	const char *what;
} pins[] = {
	{ 48, "I2C data (SE13)" }, { 49, "I2C clock (SE13)" }, { 50, "0x34's reset" }, { 51, "0x35's reset" },
	{ 121, "MI2S 2 bit clock" }, { 123, "MI2S 2 word select" }, { 122, "MI2S 2 data 0" }, { 124, "MI2S 2 data 1" },
};

static bool
map(void)
{
	if (tlmm == 0) {
		tlmm = ml_io_map(TLMM_PHYS, TLMM_SIZE);
	}
	return tlmm != 0;
}

static void
show_pins(void)
{
	for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
		uint32_t cfg = rd(GPIO_CFG(pins[i].pin)), io = rd(GPIO_IO(pins[i].pin));

		say("tfa: gpio%u (%s): function %u, pull %u, %u mA, %s, reads %u\n", pins[i].pin, pins[i].what, (cfg >> 2) & 15,
		    cfg & 3, (((cfg >> 6) & 7) + 1) * 2, (cfg & CFG_OUTPUT) ? (io & 2 ? "drives high" : "drives low") : "input", io & 1);
	}
}

/* A reset pin as a GPIO output, 2 mA, no pull, at `high'. */
static void
reset_pin(unsigned pin, bool high)
{
	wr(GPIO_IO(pin), high ? 2 : 0);
	wr(GPIO_CFG(pin), CFG_OUTPUT);
}

/* ---- I2C by hand (as hakios.speaker.c's) ----------------------------------------------------------- */

static void
half_bit(void)
{
	spin_us(5);                     /* 100 kHz at most */
}

static void
line(unsigned pin, bool high)
{
	wr(GPIO_IO(pin), 0);
	wr(GPIO_CFG(pin), CFG_PULL_UP | (high ? 0 : CFG_OUTPUT));
}

static bool
level(unsigned pin)
{
	return rd(GPIO_IO(pin)) & 1;
}

static void
i2c_start(void)
{
	line(PIN_SDA, true);
	line(PIN_SCL, true);
	half_bit();
	line(PIN_SDA, false);
	half_bit();
	line(PIN_SCL, false);
	half_bit();
}

static void
i2c_stop(void)
{
	line(PIN_SDA, false);
	half_bit();
	line(PIN_SCL, true);
	half_bit();
	line(PIN_SDA, true);
	half_bit();
}

static bool
i2c_bit(bool out)
{
	bool in;

	line(PIN_SDA, out);
	half_bit();
	line(PIN_SCL, true);
	for (unsigned stretched = 0; !level(PIN_SCL) && stretched < 100; stretched++) {
		half_bit();
	}
	half_bit();
	in = level(PIN_SDA);
	line(PIN_SCL, false);
	return in;
}

static bool
i2c_write(uint8_t byte)
{
	for (int bit = 7; bit >= 0; bit--) {
		i2c_bit((byte >> bit) & 1);
	}
	return !i2c_bit(true);
}

static uint8_t
i2c_read(bool more)
{
	uint8_t byte = 0;

	for (int bit = 7; bit >= 0; bit--) {
		byte = (uint8_t)(byte << 1 | i2c_bit(true));
	}
	i2c_bit(!more);
	return byte;
}

static uint32_t taken_cfg[2], taken_io[2];

static bool
bus_take(void)
{
	taken_cfg[0] = rd(GPIO_CFG(PIN_SDA));
	taken_cfg[1] = rd(GPIO_CFG(PIN_SCL));
	taken_io[0] = rd(GPIO_IO(PIN_SDA));
	taken_io[1] = rd(GPIO_IO(PIN_SCL));
	line(PIN_SDA, true);
	line(PIN_SCL, true);
	half_bit();
	if (!level(PIN_SDA) || !level(PIN_SCL)) {
		wr(GPIO_IO(PIN_SDA), taken_io[0] & 2);
		wr(GPIO_IO(PIN_SCL), taken_io[1] & 2);
		wr(GPIO_CFG(PIN_SDA), taken_cfg[0]);
		wr(GPIO_CFG(PIN_SCL), taken_cfg[1]);
		return false;
	}
	return true;
}

static void
bus_give_back(void)
{
	wr(GPIO_IO(PIN_SDA), taken_io[0] & 2);
	wr(GPIO_IO(PIN_SCL), taken_io[1] & 2);
	wr(GPIO_CFG(PIN_SDA), taken_cfg[0]);
	wr(GPIO_CFG(PIN_SCL), taken_cfg[1]);
}

/* A 16-bit register, most significant byte first (tfa98xx_read_register16_v6). False if nobody answers. */
static bool
reg_read(uint8_t address, uint8_t reg, uint16_t *value)
{
	bool answered;

	if (!bus_take()) {
		say("tfa: the I2C bus is not at rest (gpio%u reads %u, gpio%u reads %u): not touched\n", PIN_SDA, level(PIN_SDA),
		    PIN_SCL, level(PIN_SCL));
		return false;
	}
	i2c_start();
	answered = i2c_write((uint8_t)(address << 1)) && i2c_write(reg);
	if (answered) {
		i2c_start();
		answered = i2c_write((uint8_t)(address << 1 | 1));
	}
	if (answered) {
		*value = (uint16_t)(i2c_read(true) << 8);
		*value |= i2c_read(false);
	}
	i2c_stop();
	bus_give_back();
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
		say("tfa: the I2C bus is not at rest: not touched\n");
		return false;
	}
	i2c_start();
	answered = i2c_write((uint8_t)(address << 1)) && i2c_write(reg) && i2c_write((uint8_t)(value >> 8)) &&
	    i2c_write((uint8_t)value);
	i2c_stop();
	bus_give_back();
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
	{ .address = 0x34, .reset = 50, .where = "top (tfa_channel 0)" },
	{ .address = 0x35, .reset = 51, .where = "bottom (tfa_channel 1)" },
};

static bool broken;                     /* an amplifier did not answer: nothing more is done to either */

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

	if (!map()) {
		return false;
	}
	say("tfa: the resets (gpio50, gpio51) pulsed: high 5 ms, then low\n");
	reset_pin(amps[0].reset, true);
	reset_pin(amps[1].reset, true);
	wait_ms(5);
	reset_pin(amps[0].reset, false);
	reset_pin(amps[1].reset, false);
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

enum { DSC_DEVICE = 0, DSC_PROFILE = 1, DSC_REGISTER = 2, DSC_STRING = 3, DSC_FILE = 4, DSC_PATCH = 5, DSC_BITFIELD = 16,
       DSC_DEFAULT = 17 };

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

	return offset + n + 1 <= size && memcmp(cnt + offset, what, n) == 0 && cnt[offset + n] == 0;
}

/* One item of a list: a register patch or a bitfield into `w'; anything else ends the list (false). */
static bool
item(const uint8_t *cnt, uint64_t size, uint32_t word, struct write *w, unsigned *count)
{
	uint32_t offset = word & 0xffffff, type = word >> 24;

	if (type == DSC_BITFIELD) {
		if (offset + 4 > size || *count >= MAX_WRITES) {
			return false;
		}
		w[*count].value = le16(cnt + offset);
		w[*count].field_or_reg = le16(cnt + offset + 2);
		w[*count].mask = 0;
		(*count)++;
		return true;
	}
	if (type == DSC_REGISTER) {
		if (offset + 5 > size || *count >= MAX_WRITES) {
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

/* The container's device lists for 0x34 and 0x35, and their "speaker" profiles, read into amps[]. */
static bool
container(const uint8_t *cnt, uint64_t size)
{
	unsigned ndev;

	if (size < 0x2e || cnt[0] != 'P' || cnt[1] != 'M' || cnt[2] != '3' || memcmp(cnt + 0x20, "9874", 4) != 0) {
		say("tfa: the container is not a TFA9874's PM3 container (it begins %02x %02x %02x)\n", size ? cnt[0] : 0,
		    size > 1 ? cnt[1] : 0, size > 2 ? cnt[2] : 0);
		return false;
	}
	ndev = le16(cnt + 0x28);
	for (unsigned d = 0; d < ndev && d < 8 && 0x2e + 4 * d + 4 <= size; d++) {
		uint32_t dsc = le32(cnt + 0x2e + 4 * d), offset = dsc & 0xffffff, length;
		struct amp *a = NULL;
		bool writing = true;

		if (dsc >> 24 != DSC_DEVICE || offset + 12 > size) {
			continue;
		}
		length = cnt[offset];
		for (unsigned i = 0; i < 2; i++) {
			if (amps[i].address == cnt[offset + 2]) {
				a = &amps[i];
			}
		}
		if (a == NULL || offset + 12 + 4 * length > size) {
			continue;
		}
		a->devices = a->profiles = 0;
		for (uint32_t i = 0; i < length; i++) {
			uint32_t word = le32(cnt + offset + 12 + 4 * i), o = word & 0xffffff;

			/* the device list's registers and bitfields, up to its patch, file or first profile (tfaContWriteRegsDev_v6_v6()) */
			if (writing && !item(cnt, size, word, a->device, &a->devices)) {
				writing = false;
			}
			if (word >> 24 == DSC_PROFILE && o + 8 <= size) {
				uint32_t head = le32(cnt + o), count = head & 0xff, name = le32(cnt + o + 4) & 0xffffff;

				if (!string_is(cnt, size, name, "speaker") || o + 8 + 4 * (count - 1) > size || count == 0) {
					continue;
				}
				for (uint32_t j = 0; j + 1 < count; j++) {
					if (!item(cnt, size, le32(cnt + o + 8 + 4 * j), a->profile, &a->profiles)) {
						break;          /* (its default section, or a file: what is written when the profile is left) */
					}
				}
			}
		}
		say("tfa: the container for %#x: %u writes in its device list, %u in its \"speaker\" profile\n", a->address, a->devices,
		    a->profiles);
	}
	return amps[0].devices && amps[1].devices && amps[0].profiles && amps[1].profiles;
}

/* ---- what hakios.speaker.c hands over -------------------------------------------------------------------- */

bool
hakios_tfa_wanted(void)
{
	uint32_t soc = 0;

	return PE_parse_boot_argn("speaker_soc", &soc, sizeof(soc)) && soc == 8450;
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
	if (!container(cnt, size)) {
		say("tfa: the container has no device list or \"speaker\" profile for 0x34 and 0x35: the amplifiers are left alone\n");
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
	for (unsigned i = 0; i < 2; i++) {
		stop_one(&amps[i]);
	}
}

void
hakios_tfa_quiesce(void)
{
	leaving = true;
	if (map()) {
		reset_pin(amps[0].reset, true);
		reset_pin(amps[1].reset, true);
	}
	for (unsigned i = 0; i < 2; i++) {
		amps[i].on = false;
		amps[i].prepared = false;
	}
	leaving = false;
}

static void
probe_thread(void *parameter, __unused wait_result_t result)
{
	uint32_t probe = (uint32_t)(uintptr_t)parameter, wait_s = 0;

	/* boot-arg audio_wait=S (a test, hakios.adsp.c): the look S seconds after the system's start, when the USB log is up */
	if (PE_parse_boot_argn("audio_wait", &wait_s, sizeof(wait_s)) && wait_s != 0) {
		delay_for_interval(wait_s * 1000, NSEC_PER_MSEC);
		say("tfa: audio_wait=%u: the amplifiers' look now\n", wait_s);
	}
	if (!map()) {
		thread_terminate(current_thread());
	}
	say("tfa: speaker_probe=%u on the OnePlus 10 Pro: the amplifiers' pins%s\n", probe, probe >= 2 ? ", and who they are" : "");
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
