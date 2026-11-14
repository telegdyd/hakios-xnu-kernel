/*
 * The buttons of the POCO X5 Pro (redwood) and of the OnePlus 10 Pro (negroni) -- power, volume up,
 * volume down -- as a source of button records for /dev/hakios-touch (sys/hakios.input.h). Only with
 * boot-arg buttons=1 (or touchscreen=1). The addresses are the X5 Pro's (SM7325) unless boot-arg
 * buttons_soc=8450 says the OnePlus 10 Pro's (SM8450): see "the PMIC arbiter" below.
 *
 * None of the three is a pin of the SoC. They are inputs of two of the power management chips, the
 * same on both phones but for the second chip's name:
 *   power        KPDPWR_N of the power-on block ("PON") of the PMK8350
 *   RESIN_N of the same block            volume down on the X5 Pro, volume UP on the 10 Pro
 *   GPIO6 of the PM7325 / of the PM8350  volume up on the X5 Pro, volume DOWN on the 10 Pro;
 *                                        pulled up, the key pulls it low
 * (The 10 Pro's stock tree labels gpio6 "volume_up" but gives it KEY_VOLUMEDOWN, 0x72, and RESIN
 * KEY_VOLUMEUP, 0x73; a hand on the phone confirmed the codes, not the label, 2026-11-09.)
 * Both chips sit on the SPMI bus (slaves 0 and 1), and the way to that bus is the SoC's "PMIC
 * arbiter": it owns the bus and gives each of the systems on the chip (this one, the modem, the
 * always-on processor, TrustZone...) a set of registers per PMIC peripheral -- a "channel" -- to
 * ask for a transfer with. Reads go through the "observer" channels, which every system has for
 * every peripheral, whoever owns it.
 *
 * What is written. The configuration of the inputs never: the PMICs make the phone's supplies, and
 * a wrong write there can switch one off. The firmware reads the same keys itself (volume down and
 * volume up choose between fastboot and recovery) and has left both inputs as they are needed; the
 * driver checks that before it believes them, and where it finds something else it says so and
 * leaves that key alone. On the 10 Pro the keys' own interrupt registers are written (type,
 * polarity, enable, latch clear: 0x11 to 0x15 of the power-on block and of the GPIO, the bits of
 * the keys only, as Linux's pwrkey and GPIO drivers set them), and the arbiter's per-channel
 * interrupt enable: see "interrupts" below. Nothing else.
 *
 * What a press is: the PON's "real time status" has a bit per key, set while the key is held
 * (after the block's own debouncing); the GPIO's has the level of the pin. A thread reads the
 * two registers, and a key has changed when two looks in a row say so. A key
 * that reads as held before it has ever been seen released is not believed (a pin that is not
 * what it was taken for must not become a volume key held down for the whole session).
 * When: on the X5 Pro 60 times a second. On the 10 Pro an edge of any key raises the arbiter's
 * interrupt, which wakes the thread; it looks every 16 ms only until the keys are steady again,
 * and once a second as a net (16 ms until the line has delivered once). buttons_irq=0: looks only.
 *
 * The mapping: the phone has what the iPhone X has, whose user space runs here -- a side button
 * and two volume keys, no home button. Power is the side button (HAKIOS_BUTTON_LOCK).
 *
 * The bring-up is made in steps, each said in the log before it is made. Boot-arg
 * buttons_steps=N stops after the first N:
 *   1  map the arbiter and read its version             (the first access to that block)
 *   2  find the channels of the two peripherals         (reads of the arbiter's tables)
 *   3  ask each peripheral what it is                   (the first transfers on the bus)
 *   4  read how the keys' inputs are set up, and what they say now
 *   5  watch them                                       (the default)
 *
 * The other boot-arg:
 *   buttons_soc=8450   the OnePlus 10 Pro's (SM8450) PMIC arbiter instead of the X5 Pro's
 *
 * Register layouts: Linux drivers/spmi/spmi-pmic-arb.c (arbiter versions 5 and 7; v6.12 read for 7),
 * drivers/input/misc/pm8941-pwrkey.c (the "pmk8350" kind), drivers/pinctrl/qcom/pinctrl-spmi-gpio.c,
 * and each phone's own device tree (X5 Pro: qcom,spmi@c440000; 10 Pro: qcom,spmi@c42d000, its bus 0;
 * both: pon_hlos@1300 of slave 0, gpio_keys -> gpio6 of pinctrl@8800 of slave 1).
 *
 * Status: X5 Pro, run on the phone (all three keys). 10 Pro: see the head of "the PMIC arbiter".
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <os/atomic_private.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/hakios.input.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern void hakios_screen_pause(uint32_t on_ms, uint32_t off_ms);
extern bool hakios_screen_off(void);
extern void hakios_spmi_lock(void);
extern void hakios_spmi_unlock(void);
extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *, uint32_t), void *context);
extern void gic_v3_spi_mask(uint32_t intid);
extern void gic_v3_spi_unmask(uint32_t intid);
extern void thread_set_thread_name(thread_t thread, const char *name);

/*
 * Every line goes to the loader's log directly as well: without -v the kernel's own messages
 * reach that log only at the end of a run (with the kernel's message buffer), and "xl log"
 * during a run is to show a press when it happens.
 */
extern void loader_log_putc(char c);

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[400];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	return printf("%s", line);
}
#define printf say

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

/* ---- the PMIC arbiter ---------------------------------------------------------------------------- */

/*
 * Two layouts of it. What stays the same: the version at "core" + 0, the number of channels in "core" + 4
 * ([10:0]), one word per channel in "core" saying which peripheral it is ([19:8] slave and peripheral,
 * [24] interrupts go to the owner), one per channel in "cnfg" saying who may write through it ([2:0]),
 * and the observer channel's own registers (command, status, data: below).
 *
 *   version 5, SM7325 (X5 Pro): the table at core + 0x900, owners at cnfg + 0x700, observer channels
 *       0x80 apart, 0x10000 per system.
 *   version 7, SM8450 (10 Pro): the table at core + 0x2000, owners at cnfg + 0, observer channels 0x20
 *       apart, 0x8000 per system. Version 7 has two SPMI buses behind one arbiter; the first "features"
 *       count is bus 0's channels and they come first in the table, so looking at that many looks at
 *       bus 0 only. Both PMICs of the keys are on bus 0 (the stock tree's qcom,spmi@c42d000, qcom,bus-id 0:
 *       pmk8350@0, pm8350@1). Bus 1's channels follow, with peripheral numbers of their own that could
 *       match ours, which is why the count matters. The SM8450 addresses are the stock tree's reg/reg-names
 *       of that node (cnfg 0xc42d000, core 0xc400000, obsrvr 0xc440000).
 *       Status: written from Linux v6.12's spmi-pmic-arb.c and the phone's stock tree (2026-11-09);
 *       QEMU has no arbiter, so the first run on the phone is the first test of this layout.
 */
struct arbiter_layout {
	const char *    soc;
	const char *    pin_name;       /* the GPIO key's pin, as the log says it */
	bool            resin_is_up;    /* RESIN is volume up and the GPIO volume down (the 10 Pro) */
	vm_offset_t     core_phys, cnfg_phys, observer_phys;
	vm_size_t       core_size, cnfg_size, observer_size;    /* the observer: our system's part only */
	uint32_t        version_from, version_to;               /* [from, to) */
	uint32_t        table, owners;                          /* where channel 0's words are in core and cnfg */
	uint32_t        observer_system, observer_channel;      /* the strides */
	uint32_t        max_channels;
};

#define ARB_VERSION_5           0x50000000u
#define ARB_VERSION_7           0x70000000u
#define ARB_VERSION_8           0x80000000u

static const struct arbiter_layout arb_sm7325 = {
	.soc = "SM7325 (POCO X5 Pro)", .pin_name = "GPIO6 of the PM7325",
	.core_phys = 0x0c440000UL, .core_size = 0x1100UL,
	.cnfg_phys = 0x0c40a000UL, .cnfg_size = 0x1000UL,
	.observer_phys = 0x0e600000UL, .observer_size = 0x10000UL,
	.version_from = ARB_VERSION_5, .version_to = ARB_VERSION_7,
	.table = 0x900, .owners = 0x700, .observer_system = 0x10000, .observer_channel = 0x80, .max_channels = 512,
};
static const struct arbiter_layout arb_sm8450 = {
	.soc = "SM8450 (OnePlus 10 Pro)", .pin_name = "GPIO6 of the PM8350", .resin_is_up = true,
	.core_phys = 0x0c400000UL, .core_size = 0x3000UL,
	.cnfg_phys = 0x0c42d000UL, .cnfg_size = 0x1000UL,
	.observer_phys = 0x0c440000UL, .observer_size = 0x8000UL,
	.version_from = ARB_VERSION_7, .version_to = ARB_VERSION_8,
	.table = 0x2000, .owners = 0x000, .observer_system = 0x8000, .observer_channel = 0x20, .max_channels = 1024,
};
static const struct arbiter_layout *arb = &arb_sm7325;

#define ARB_SYSTEM              0               /* who we are to the arbiter: the device tree's "qcom,ee" (0 on both) */
#define ARB_VERSION             0x000
#define ARB_FEATURES            0x004           /* [10:0] the number of channels (version 7: of bus 0) */
#define ARB_CHANNEL(n)          (arb->table + 4 * (n))
#define ARB_CHANNEL_INTERRUPTS  (1u << 24)
#define ARB_OWNER(n)            (arb->owners + 4 * (n))

#define OBSERVER_COMMAND        0x00
#define OBSERVER_STATUS         0x08
#define OBSERVER_DATA0          0x18            /* the bytes read, the first in the lowest byte */
#define OBSERVER_DATA1          0x1c
#define STATUS_DONE             0x1
#define STATUS_FAILURE          0x2
#define STATUS_DENIED           0x4
#define STATUS_DROPPED          0x8
#define COMMAND_READ            (1u << 27)      /* "extended register read long"; [11:4] register, [2:0] bytes - 1 */
#define READ_TIMEOUT_US         2000            /* a read takes some tens of microseconds */

static vm_offset_t arb_core, arb_cnfg, arb_observer;

/* one transfer at a time through the arbiter: hakios_spmi_lock() (drivers/core/hakios.platform.c) */

struct peripheral {
	const char *    name;
	uint16_t        id;             /* slave << 8 | peripheral: the high byte of its register addresses */
	uint16_t        channel;
	bool            found;
};
static struct peripheral pon = { .name = "the PMK8350's power-on block", .id = 0x013 };          /* registers 0x1300 of slave 0 */
static struct peripheral pin = { .name = "GPIO6 of the PM7325", .id = 0x18d };                   /* registers 0x8d00 of slave 1; named by the layout */
static struct peripheral rtc = { .name = "the PMK8350's real-time clock", .id = 0x061 };         /* registers 0x6100 of slave 0 */

/* How the reads have gone: said once a minute. */
static struct {
	uint64_t reads, failed, total_ns, longest_ns, last_ns;
} bus;

/*
 * Read `count` (1 to 8) registers of a peripheral, from register `reg` on. Returns NULL, or what
 * went wrong.
 */
static const char *
spmi_read(const struct peripheral *p, uint8_t reg, uint8_t *bytes, uint32_t count)
{
	/* arb_observer is our system's part already (ARB_SYSTEM's: see buttons_thread) */
	vm_offset_t channel = arb_observer + (vm_offset_t)arb->observer_channel * p->channel;
	uint64_t start = mach_absolute_time(), now, interval, taken;
	uint32_t status, data[2];
	const char *problem = NULL;

	nanoseconds_to_absolutetime(READ_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	hakios_spmi_lock();
	wr(channel + OBSERVER_COMMAND, COMMAND_READ | (uint32_t)reg << 4 | (count - 1));
	do {
		status = rd(channel + OBSERVER_STATUS);
		now = mach_absolute_time();
	} while (!(status & STATUS_DONE) && now - start < interval);
	data[0] = rd(channel + OBSERVER_DATA0);
	data[1] = count > 4 ? rd(channel + OBSERVER_DATA1) : 0;
	hakios_spmi_unlock();

	if (!(status & STATUS_DONE)) {
		problem = "no answer from the arbiter";
	} else if (status & STATUS_DENIED) {
		problem = "the arbiter refuses";
	} else if (status & STATUS_FAILURE) {
		problem = "the transfer failed (nobody at that address?)";
	} else if (status & STATUS_DROPPED) {
		problem = "the transfer was dropped";
	} else {
		for (uint32_t i = 0; i < count; i++) {
			bytes[i] = (uint8_t)(data[i / 4] >> (8 * (i % 4)));
		}
	}
	absolutetime_to_nanoseconds(now - start, &taken);
	bus.reads++;
	bus.total_ns += taken;
	bus.last_ns = taken;
	if (taken > bus.longest_ns) {
		bus.longest_ns = taken;
	}
	if (problem != NULL) {
		bus.failed++;
	}
	return problem;
}

/*
 * Which channel is that peripheral's. Several can be: one per system that may write to it. As
 * Linux chooses: the first, unless a later one is ours to write through.
 */
static void
find_channel(struct peripheral *p, uint32_t channels)
{
	for (uint32_t n = 0; n < channels; n++) {
		uint32_t map = rd(arb_core + ARB_CHANNEL(n)), owner;

		if (map == 0 || ((map >> 8) & 0xfff) != p->id) {
			continue;
		}
		owner = rd(arb_cnfg + ARB_OWNER(n)) & 7;
		printf("buttons: channel %u is %s (%#x); written by system %u%s\n", n, p->name, map, owner,
		    (map & ARB_CHANNEL_INTERRUPTS) ? ", which also gets its interrupts" : "");
		if (!p->found || owner == ARB_SYSTEM) {
			p->found = true;
			p->channel = (uint16_t)n;
		}
	}
	if (!p->found) {
		printf("buttons: no channel of the arbiter is %s (slave %u, peripheral %#x)\n", p->name, p->id >> 8, p->id & 0xff);
	}
}

/* ---- the two peripherals ------------------------------------------------------------------------- */

/* every peripheral of these PMICs */
#define REG_TYPE                0x04
#define REG_SUBTYPE             0x05
#define REG_RT_STATUS           0x10            /* the inputs as they are now */
#define REG_INT_SET_TYPE        0x11            /* interrupts: a bit per input in each of these */
#define REG_INT_POLARITY_HIGH   0x12
#define REG_INT_POLARITY_LOW    0x13
#define REG_INT_ENABLED         0x15
#define REG_INT_LATCHED         0x18
#define REG_INT_PENDING         0x19

/* the power-on block, of the kind that is split in two ("generation 3"): this is its half for the operating system */
#define PON_TYPE                0x01
#define PON_SUBTYPE_HLOS        0x09
#define PON_KPDPWR              0x80            /* in the status and in the interrupt registers */
#define PON_RESIN               0x40

/* a GPIO of the low or medium voltage kind */
#define GPIO_TYPE               0x10
#define GPIO_SUBTYPE_LV         0x10
#define GPIO_SUBTYPE_MV_VIN3    0x13            /* 0x10 to 0x13: one layout */
#define GPIO_LEVEL              0x01            /* in the status */
#define GPIO_REG_MODE           0x40            /* [1:0]: 0 input, 1 output, 2 both, 3 analog */
#define GPIO_REG_SUPPLY         0x41
#define GPIO_REG_PULL           0x42            /* [2:0]: 0 to 3 up (30, 1.5, 31.5 uA, 1.5 with a boost), 4 down, 5 none */
#define GPIO_REG_ENABLE         0x46            /* [7] */

#define POLL_MS                 16
#define MINUTE_TICKS            (60 * 1000 / POLL_MS)

static struct button {
	const char *    name;
	uint32_t        record;         /* HAKIOS_BUTTON_* */
	bool            usable;         /* its input is what it was taken for */
	bool            live;           /* it has been seen released */
	bool            last;           /* the look before this one */
	bool            down;           /* as reported */
	uint64_t        since;          /* when it went down */
	uint32_t        presses;
} power = { .name = "power", .record = HAKIOS_BUTTON_LOCK },
    volume_up = { .name = "volume up", .record = HAKIOS_BUTTON_VOLUME_UP },
    volume_down = { .name = "volume down", .record = HAKIOS_BUTTON_VOLUME_DOWN };

/* which volume key each input is: the X5 Pro's way unless the layout says resin_is_up (buttons_thread) */
static struct button *resin_key = &volume_down, *pin_key = &volume_up;

/* "12.345 s (08:13:05.918 UTC)": since boot as in the log's own stamps, and as the daemons' lines have it */
static void
now_text(char *text, size_t size, uint64_t now)
{
	clock_sec_t seconds;
	clock_usec_t micro;
	uint64_t up;

	absolutetime_to_nanoseconds(now, &up);
	clock_get_calendar_microtime(&seconds, &micro);
	snprintf(text, size, "%llu.%03llu s (%02u:%02u:%02u.%03u UTC)", up / NSEC_PER_SEC, up / NSEC_PER_MSEC % 1000,
	    (unsigned)(seconds / 3600 % 24), (unsigned)(seconds / 60 % 60), (unsigned)(seconds % 60), (unsigned)(micro / 1000));
}

static void
button_look(struct button *b, bool pressed)
{
	uint64_t now = mach_absolute_time(), held;
	bool steady = pressed == b->last;
	char when[64];

	b->last = pressed;
	if (!b->usable || !steady) {
		return;
	}
	if (!b->live) {
		if (!pressed) {
			b->live = true;
			printf("buttons: %s is released now: believed from here on\n", b->name);
		}
		return;
	}
	if (pressed == b->down) {
		return;
	}
	b->down = pressed;
	now_text(when, sizeof(when), now);
	if (pressed) {
		b->since = now;
		b->presses++;
		printf("buttons: %s PRESSED at %s\n", b->name, when);
	} else {
		absolutetime_to_nanoseconds(now - b->since, &held);
		printf("buttons: %s released at %s, after %llu ms\n", b->name, when, held / NSEC_PER_MSEC);
	}
	hakios_touch_post(pressed ? HAKIOS_BUTTON_DOWN : HAKIOS_BUTTON_UP, b->record, 0, 0);
}

/* One look at both peripherals. */
static void
buttons_poll(void)
{
	static unsigned said;
	const char *problem;
	uint8_t status;

	if (power.usable || resin_key->usable) {
		problem = spmi_read(&pon, REG_RT_STATUS, &status, 1);
		if (problem == NULL) {
			button_look(&power, status & PON_KPDPWR);
			button_look(resin_key, status & PON_RESIN);
		} else if (said < 8) {
			said++;
			printf("buttons: reading %s: %s\n", pon.name, problem);
		}
	}
	if (pin_key->usable) {
		problem = spmi_read(&pin, REG_RT_STATUS, &status, 1);
		if (problem == NULL) {
			button_look(pin_key, !(status & GPIO_LEVEL));
		} else if (said < 8) {
			said++;
			printf("buttons: reading %s: %s\n", pin.name, problem);
		}
	}
}

/* Step 3: is each peripheral what the registers above are those of? */
static bool
identify(const struct peripheral *p, uint8_t type, uint8_t subtype_from, uint8_t subtype_to)
{
	const char *problem;
	uint8_t is[2];

	if (!p->found) {
		return false;
	}
	printf("buttons: next: read the type of %s through channel %u (2 registers, 0x%02x04)\n", p->name, p->channel, p->id & 0xff);
	problem = spmi_read(p, REG_TYPE, is, 2);
	if (problem != NULL) {
		printf("buttons: %s: %s\n", p->name, problem);
		return false;
	}
	printf("buttons: %s says type %#x, subtype %#x (the read took %llu us)\n", p->name, is[0], is[1], bus.last_ns / 1000);
	if (is[0] != type || is[1] < subtype_from || is[1] > subtype_to) {
		printf("buttons: ... which is not what this driver knows (type %#x, subtype %#x to %#x): left alone\n", type,
		    subtype_from, subtype_to);
		return false;
	}
	return true;
}

/*
 * kern.hakios.rtc_counter: the PMK8350's real-time clock as it reads now (four bytes at 0x6148, little-endian, as
 * Linux's rtc-pm8xxx.c reads them), or -1 when the clock was not found (step 3) or the read fails. It is a counter
 * of seconds that goes on while the phone is off, not a date (drivers/power/hakios.pmic.c: on the X5 Pro it read
 * as 1971): Android keeps the offset to real time in a file of its own, and so does hakios (userland/src/
 * clock_keep.c, WIFI-PLAN.md I8). Read here because this driver has the arbiter of either phone mapped already;
 * like the keys, only read: nothing is written to the clock.
 */
static bool rtc_known;

static int
rtc_counter_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int64_t value = -1;
	uint8_t b[4];

	if (req->newptr != USER_ADDR_NULL) {
		return EPERM;
	}
	if (rtc_known && spmi_read(&rtc, 0x48, b, 4) == NULL) {
		value = (int64_t)(b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24);
	}
	return SYSCTL_OUT(req, &value, sizeof(value));
}
SYSCTL_DECL(_kern_hakios);
SYSCTL_PROC(_kern_hakios, OID_AUTO, rtc_counter, CTLTYPE_QUAD | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, rtc_counter_sysctl, "Q",
    "the PMIC real-time clock's seconds counter as it reads now (not a date); -1: not found or not read");

/* Step 4 for the power-on block: nothing to check but what it says now; its interrupt setup goes to the log. */
static void
pon_inputs(void)
{
	const char *problem;
	uint8_t r[8], more[2];

	printf("buttons: next: read the status and the interrupt setup of %s (registers 0x1310 to 0x1319)\n", pon.name);
	problem = spmi_read(&pon, REG_RT_STATUS, r, 8);
	if (problem == NULL) {
		problem = spmi_read(&pon, REG_INT_LATCHED, more, 2);
	}
	if (problem != NULL) {
		printf("buttons: %s: %s\n", pon.name, problem);
		return;
	}
	printf("buttons: power-on block: status %#x (power %s, %s %s); interrupts: edge %#x, high %#x, low %#x, "
	    "enabled %#x, latched %#x, pending %#x\n", r[0], (r[0] & PON_KPDPWR) ? "HELD" : "released", resin_key->name,
	    (r[0] & PON_RESIN) ? "HELD" : "released", r[REG_INT_SET_TYPE - REG_RT_STATUS], r[REG_INT_POLARITY_HIGH - REG_RT_STATUS],
	    r[REG_INT_POLARITY_LOW - REG_RT_STATUS], r[REG_INT_ENABLED - REG_RT_STATUS], more[0], more[1]);
	power.usable = resin_key->usable = true;
	power.last = r[0] & PON_KPDPWR;
	resin_key->last = r[0] & PON_RESIN;
}

/* Step 4 for the pin: it is a key only if it is an input with a pull-up, and switched on. */
static void
pin_input(void)
{
	const char *problem;
	uint8_t c[8], status = 0;
	unsigned mode, pull;

	printf("buttons: next: read the setup and the level of %s (registers 0x8d40 to 0x8d47, 0x8d10)\n", pin.name);
	problem = spmi_read(&pin, GPIO_REG_MODE, c, 8);
	if (problem == NULL) {
		problem = spmi_read(&pin, REG_RT_STATUS, &status, 1);
	}
	if (problem != NULL) {
		printf("buttons: %s: %s\n", pin.name, problem);
		return;
	}
	mode = c[0] & 3;
	pull = c[GPIO_REG_PULL - GPIO_REG_MODE] & 7;
	printf("buttons: %s's pin: mode %u (%s), supply %u, pull %u (%s), %s; registers 0x40.. = %02x %02x %02x %02x %02x %02x "
	    "%02x %02x; level %s\n", pin_key->name, mode, mode == 0 ? "input" : mode == 1 ? "output" : mode == 2 ? "input and output" : "analog",
	    c[GPIO_REG_SUPPLY - GPIO_REG_MODE] & 7, pull, pull < 4 ? "up" : pull == 4 ? "down" : "none",
	    (c[GPIO_REG_ENABLE - GPIO_REG_MODE] & 0x80) ? "enabled" : "DISABLED", c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
	    (status & GPIO_LEVEL) ? "high (released)" : "LOW (held)");
	if (mode != 0 || pull >= 4 || !(c[GPIO_REG_ENABLE - GPIO_REG_MODE] & 0x80)) {
		printf("buttons: %s's pin is not an enabled input with a pull-up. Making it one is a write to the PMIC, "
		    "which this driver does not do: %s is left alone\n", pin_key->name, pin_key->name);
		return;
	}
	pin_key->usable = true;
	pin_key->last = !(status & GPIO_LEVEL);
}

/* the 10 Pro's interrupt (see "interrupts" below) */
static struct {
	bool            on;             /* attached and believed */
	bool            delivered;      /* it has brought a key's edge at least once: the net can be slow */
	volatile uint32_t rang;
	uint32_t        count, edges, spurious, nets, period_count;
	uint64_t        period, second;
} keys_irq;

static void
say_summary(void)
{
	printf("buttons: so far %u presses of power, %u of volume up, %u of volume down; now%s%s%s%s; %llu reads of the bus, "
	    "%llu us each (the longest %llu), %llu failed\n", power.presses, volume_up.presses, volume_down.presses,
	    power.down ? " power" : "", volume_up.down ? " volume-up" : "", volume_down.down ? " volume-down" : "",
	    (power.down || volume_up.down || volume_down.down) ? " held" : " none held", bus.reads,
	    bus.reads ? bus.total_ns / bus.reads / 1000 : 0, bus.longest_ns / 1000, bus.failed);
	if (arb == &arb_sm8450) {
		printf("buttons: interrupts %s: %u taken, %u with a key's edge, %u of others, %u nets\n", keys_irq.on ? "on" : "OFF",
		    keys_irq.count, keys_irq.edges, keys_irq.spurious, keys_irq.nets);
	}
}

/* ---- interrupts (the 10 Pro) ---------------------------------------------------------------------- */

/*
 * The way an edge of a key comes in (Linux v6.12 spmi-pmic-arb.c, version 7; qcom-pdc.c; the phone's tree):
 *   the peripheral   latches the edge (both edges of the keys' bits: type, polarity high and low) and, enabled,
 *                    sends an interrupt message on the bus; it raises no other until the latch is cleared (0x14)
 *   the arbiter      sets that channel's status ("chnls" + 0x1000 per channel: 0x100 enable, 0x104 status, 0x108
 *                    clear) and its bit in our system's accumulated status ("intr" + 4 per 32 channels), and raises
 *                    its one line, "periph_irq": PDC pin 1, level high
 *   the PDC          pin 1 is the GIC's SPI 481 (qcom,pdc-ranges: pins 0-11 are SPIs 480-491), INTID 513; the pin
 *                    passes only while enabled in the PDC (bank 0 bit 1 before version 3.2, its config's bit 3 after)
 * The handler masks the GIC line and wakes the thread; the thread clears the arbiter's status and the peripheral's
 * latch (SPMI writes take a spin lock and tens of microseconds: not in interrupt context), looks, and unmasks.
 */
#define ARB8450_CHNLS_PHYS      0x0c500000UL    /* reg-names "chnls" of qcom,spmi@c42d000 */
#define ARB8450_INTR_PHYS       0x0c4c0000UL    /* "intr"; our system's (0) words first */
#define CHNL_STRIDE             0x1000UL
#define CHNL_COMMAND            0x00
#define CHNL_STATUS             0x08
#define CHNL_WDATA0             0x10
#define CHNL_ACC_ENABLE         0x100
#define CHNL_IRQ_STATUS         0x104
#define CHNL_IRQ_CLEAR          0x108
#define COMMAND_WRITE           (0u << 27)      /* "extended register write long" */
#define REG_INT_LATCHED_CLR     0x14
#define REG_INT_EN_SET          0x15
#define PDC_PHYS                0x0b220000UL
#define PDC_ENABLE_BANK0        0x010
#define PDC_CFG(pin)            (0x110 + 4 * (pin))
#define PDC_VERSION             0x1000
#define PDC_VERSION_3_2         0x30200u
#define PDC_CFG_ENABLE          (1u << 3)
#define PDC_LEVEL_HIGH          4u
#define KEYS_PDC_PIN            1
#define KEYS_INTID              513             /* SPI 481 */
#define KEYS_NET_MS             1000
#define KEYS_STORM              200             /* interrupts in a second: past that, the line is given up */
#define GPIO_IRQ_BIT            0x01            /* a GPIO's one interrupt */

static vm_offset_t pon_chnl, pin_chnl, arb_intr, keys_pdc;
static uint32_t arb_channels, pdc_found_bank, pdc_found_cfg;


SYSCTL_UINT(_kern_hakios, OID_AUTO, buttons_irq_count, CTLFLAG_RD | CTLFLAG_LOCKED, &keys_irq.count, 0,
    "the PMIC arbiter's interrupts taken by the buttons");
SYSCTL_UINT(_kern_hakios, OID_AUTO, buttons_irq_edges, CTLFLAG_RD | CTLFLAG_LOCKED, &keys_irq.edges, 0,
    "of them, ones that brought a key's edge");
SYSCTL_UINT(_kern_hakios, OID_AUTO, buttons_irq_spurious, CTLFLAG_RD | CTLFLAG_LOCKED, &keys_irq.spurious, 0,
    "another channel's interrupt found and switched off");
SYSCTL_UINT(_kern_hakios, OID_AUTO, buttons_irq_nets, CTLFLAG_RD | CTLFLAG_LOCKED, &keys_irq.nets, 0,
    "waits ended by the net (no interrupt)");
SYSCTL_UINT(_kern_hakios, OID_AUTO, buttons_presses, CTLFLAG_RD | CTLFLAG_LOCKED, &power.presses, 0, "presses of power");
SYSCTL_UINT(_kern_hakios, OID_AUTO, buttons_presses_up, CTLFLAG_RD | CTLFLAG_LOCKED, &volume_up.presses, 0,
    "presses of volume up");
SYSCTL_UINT(_kern_hakios, OID_AUTO, buttons_presses_down, CTLFLAG_RD | CTLFLAG_LOCKED, &volume_down.presses, 0,
    "presses of volume down");

/* Write one register of a peripheral through our own (write) channel. NULL, or what went wrong. */
static const char *
spmi_write(vm_offset_t channel, uint8_t reg, uint8_t value)
{
	uint64_t start = mach_absolute_time(), interval;
	uint32_t status;

	nanoseconds_to_absolutetime(READ_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	hakios_spmi_lock();
	wr(channel + CHNL_WDATA0, value);
	wr(channel + CHNL_COMMAND, COMMAND_WRITE | (uint32_t)reg << 4);
	do {
		status = rd(channel + CHNL_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	hakios_spmi_unlock();
	return !(status & STATUS_DONE) ? "no answer from the arbiter" : (status & STATUS_DENIED) ? "the arbiter refuses" :
	       (status & STATUS_FAILURE) ? "the transfer failed" : (status & STATUS_DROPPED) ? "the transfer was dropped" : NULL;
}

/* In interrupt context. */
static void
keys_irq_arrived(__unused void *context, uint32_t intid)
{
	uint64_t now = mach_absolute_time();

	gic_v3_spi_mask(intid);
	keys_irq.count++;
	if (now - keys_irq.period >= keys_irq.second) {
		keys_irq.period = now;
		keys_irq.period_count = 0;
	}
	if (++keys_irq.period_count > KEYS_STORM) {
		keys_irq.on = false;            /* (left masked; the thread says so) */
	}
	os_atomic_store(&keys_irq.rang, 1, release);
	thread_wakeup((event_t)&keys_irq.rang);
}

/* The interrupt's peripheral bits that came in since the last time, per key peripheral; their latches cleared. */
static void
keys_irq_service(uint8_t *pon_bits, uint8_t *pin_bits)
{
	uint32_t words = (arb_channels + 31) / 32;

	*pon_bits = *pin_bits = 0;
	for (uint32_t w = 0; w < words; w++) {
		uint32_t pending = rd(arb_intr + 4 * w);

		while (pending != 0) {
			uint32_t apid = 32 * w + (uint32_t)__builtin_ctz(pending);
			vm_offset_t ch;
			uint32_t status;

			pending &= pending - 1;
			ch = (pon.found && apid == pon.channel) ? pon_chnl : (pin.found && apid == pin.channel) ? pin_chnl : 0;
			if (ch == 0) {
				/* someone else's interrupt left enabled for us: off, as Linux does with one nobody asked for */
				ch = ml_io_map(ARB8450_CHNLS_PHYS + CHNL_STRIDE * apid, CHNL_STRIDE);
				status = rd(ch + CHNL_IRQ_STATUS);
				wr(ch + CHNL_ACC_ENABLE, 0);
				wr(ch + CHNL_IRQ_CLEAR, status);
				if (keys_irq.spurious++ < 8) {
					printf("buttons: interrupt of channel %u (%#x), not a key's: switched off\n", apid,
					    rd(arb_core + ARB_CHANNEL(apid)));
				}
				continue;
			}
			status = rd(ch + CHNL_IRQ_STATUS);
			wr(ch + CHNL_IRQ_CLEAR, status);
			*(ch == pon_chnl ? pon_bits : pin_bits) |= (uint8_t)status;
		}
	}
	if (*pon_bits != 0) {
		(void)spmi_write(pon_chnl, REG_INT_LATCHED_CLR, *pon_bits);
	}
	if (*pin_bits != 0) {
		(void)spmi_write(pin_chnl, REG_INT_LATCHED_CLR, *pin_bits);
	}
}

/* type, polarity high, polarity low as the firmware left them: put back before a reset (hakios_buttons_quiesce) */
static uint8_t pon_found[3], pin_found[3];
static bool pon_written, pin_written;

/* One peripheral's interrupt for `bits`, on both edges: the arbiter's channel enabled, then the peripheral. */
static bool
keys_irq_peripheral(const struct peripheral *p, vm_offset_t ch, uint8_t bits, uint8_t found[3], bool *written)
{
	static const uint8_t regs[3] = { REG_INT_SET_TYPE, REG_INT_POLARITY_HIGH, REG_INT_POLARITY_LOW };
	const char *problem = NULL;
	uint8_t r[8];

	wr(ch + CHNL_IRQ_CLEAR, 0xff);
	wr(ch + CHNL_ACC_ENABLE, 1);
	problem = spmi_read(p, REG_RT_STATUS, r, 8);
	if (problem == NULL) {
		found[0] = r[1];
		found[1] = r[2];
		found[2] = r[3];
		*written = true;
	}
	for (unsigned i = 0; problem == NULL && i < 3; i++) {
		problem = spmi_write(ch, regs[i], r[regs[i] - REG_RT_STATUS] | bits);
	}
	if (problem == NULL) {
		problem = spmi_write(ch, REG_INT_LATCHED_CLR, bits);
	}
	if (problem == NULL) {
		problem = spmi_write(ch, REG_INT_EN_SET, bits);
	}
	if (problem == NULL) {
		problem = spmi_read(p, REG_RT_STATUS, r, 8);
	}
	if (problem != NULL) {
		printf("buttons: interrupts of %s: %s\n", p->name, problem);
		return false;
	}
	printf("buttons: interrupts of %s (bits %#x): edge %#x, high %#x, low %#x, enabled %#x; arbiter channel %u: enable %#x\n",
	    p->name, bits, r[1], r[2], r[3], r[5], p->channel, rd(ch + CHNL_ACC_ENABLE));
	return (r[5] & bits) == bits && (r[1] & bits) == bits;
}

static void
keys_irq_start(void)
{
	uint32_t wanted = 1, version, bank, cfg;
	uint8_t pon_bits = (power.usable ? PON_KPDPWR : 0) | (resin_key->usable ? PON_RESIN : 0), a, b;
	vm_offset_t pdc;
	bool ok = true;

	PE_parse_boot_argn("buttons_irq", &wanted, sizeof(wanted));
	if (arb != &arb_sm8450 || !wanted) {
		printf("buttons: no interrupt (%s): the keys are looked at every %u ms\n", wanted ? "not the 10 Pro" : "buttons_irq=0",
		    POLL_MS);
		return;
	}
	printf("buttons: interrupts: the arbiter's line (PDC pin %u, GIC %u), channels %u and %u\n", KEYS_PDC_PIN, KEYS_INTID,
	    pon.channel, pin.channel);
	nanoseconds_to_absolutetime(NSEC_PER_SEC, &keys_irq.second);
	arb_intr = ml_io_map(ARB8450_INTR_PHYS, 0x1000);
	pon_chnl = ml_io_map(ARB8450_CHNLS_PHYS + CHNL_STRIDE * pon.channel, CHNL_STRIDE);
	pin_chnl = ml_io_map(ARB8450_CHNLS_PHYS + CHNL_STRIDE * pin.channel, CHNL_STRIDE);

	pdc = ml_io_map(PDC_PHYS, 0x2000);
	version = rd(pdc + PDC_VERSION);
	bank = rd(pdc + PDC_ENABLE_BANK0);
	cfg = rd(pdc + PDC_CFG(KEYS_PDC_PIN));
	keys_pdc = pdc;
	pdc_found_bank = bank;
	pdc_found_cfg = cfg;
	if (version >= PDC_VERSION_3_2) {
		wr(pdc + PDC_CFG(KEYS_PDC_PIN), PDC_LEVEL_HIGH | PDC_CFG_ENABLE);
	} else {
		wr(pdc + PDC_CFG(KEYS_PDC_PIN), PDC_LEVEL_HIGH);
		wr(pdc + PDC_ENABLE_BANK0, bank | 1u << KEYS_PDC_PIN);
	}
	printf("buttons: PDC version %#x: pin %u config %#x -> %#x, bank 0 %#x -> %#x\n", version, KEYS_PDC_PIN, cfg,
	    rd(pdc + PDC_CFG(KEYS_PDC_PIN)), bank, rd(pdc + PDC_ENABLE_BANK0));

	os_atomic_store(&keys_irq.rang, 0, relaxed);
	keys_irq.on = true;
	/* (attaching enables the line: anything already pending only wakes the thread) */
	if (gic_v3_spi_attach(KEYS_INTID, false, keys_irq_arrived, NULL) != KERN_SUCCESS) {
		keys_irq.on = false;
		printf("buttons: THE ARBITER'S INTERRUPT (GIC %u) CANNOT BE TAKEN: looking every %u ms\n", KEYS_INTID, POLL_MS);
		return;
	}
	if (pon_bits != 0) {
		ok = keys_irq_peripheral(&pon, pon_chnl, pon_bits, pon_found, &pon_written);
	}
	if (pin_key->usable) {
		ok = keys_irq_peripheral(&pin, pin_chnl, GPIO_IRQ_BIT, pin_found, &pin_written) && ok;
	}
	keys_irq_service(&a, &b);       /* (whatever was latched before) */
	gic_v3_spi_unmask(KEYS_INTID);
	if (!ok) {
		/* (left attached: an edge still wakes the thread early) */
		printf("buttons: an interrupt is not set as asked: looking every %u ms as well\n", POLL_MS);
	}
	keys_irq.delivered = false;
	printf("buttons: interrupts on: %u taken while setting up; the net is %u ms until one brings an edge, then %u ms\n",
	    keys_irq.count, POLL_MS, KEYS_NET_MS);
}

/*
 * Before a reset (drivers/core/hakios.platform.c, kern.hakios.loader_request): the keys' interrupts off again in the
 * PMICs, latches cleared, as the firmware left them: PMIC registers live through a warm reset, and the boot loader and
 * the next kernel find the keys as the firmware gave them. The PDC pin's enable is put back as found too: the PDC keeps
 * it through the reset. (Not what broke sweet-sh.py reboot after step 2: that was XNU's I/O map running full,
 * REBOOT-FAULT.md.)
 */
#define REG_INT_EN_CLR          0x16

void hakios_buttons_quiesce(void);

void
hakios_buttons_quiesce(void)
{
	uint8_t pon_bits = PON_KPDPWR | PON_RESIN;

	if (arb != &arb_sm8450 || pon_chnl == 0) {
		return;
	}
	keys_irq.on = false;
	gic_v3_spi_mask(KEYS_INTID);
	if (pon.found && pon_written) {
		(void)spmi_write(pon_chnl, REG_INT_EN_CLR, pon_bits);
		(void)spmi_write(pon_chnl, REG_INT_LATCHED_CLR, pon_bits);
		(void)spmi_write(pon_chnl, REG_INT_SET_TYPE, pon_found[0]);
		(void)spmi_write(pon_chnl, REG_INT_POLARITY_HIGH, pon_found[1]);
		(void)spmi_write(pon_chnl, REG_INT_POLARITY_LOW, pon_found[2]);
	}
	if (pon.found) {
		wr(pon_chnl + CHNL_ACC_ENABLE, 0);
		wr(pon_chnl + CHNL_IRQ_CLEAR, 0xff);
	}
	if (pin.found && pin_chnl != 0 && pin_written) {
		(void)spmi_write(pin_chnl, REG_INT_EN_CLR, GPIO_IRQ_BIT);
		(void)spmi_write(pin_chnl, REG_INT_LATCHED_CLR, GPIO_IRQ_BIT);
		(void)spmi_write(pin_chnl, REG_INT_SET_TYPE, pin_found[0]);
		(void)spmi_write(pin_chnl, REG_INT_POLARITY_HIGH, pin_found[1]);
		(void)spmi_write(pin_chnl, REG_INT_POLARITY_LOW, pin_found[2]);
	}
	if (pin.found && pin_chnl != 0) {
		wr(pin_chnl + CHNL_ACC_ENABLE, 0);
		wr(pin_chnl + CHNL_IRQ_CLEAR, 0xff);
	}
	if (keys_pdc != 0) {
		/* the PDC is in the always-on block: what is left in it is there for the boot loader after a warm reset */
		wr(keys_pdc + PDC_ENABLE_BANK0, pdc_found_bank);
		wr(keys_pdc + PDC_CFG(KEYS_PDC_PIN), pdc_found_cfg);
	}
	printf("buttons: the keys' interrupts off in the PMICs, PDC pin %u as found (bank 0 %#x) (before a reset)\n", KEYS_PDC_PIN,
	    rd(keys_pdc + PDC_ENABLE_BANK0));
}

/* Every usable key's last look is what was reported: nothing in flux, an edge will ring. */
static bool
keys_steady(void)
{
	struct button *all[3] = { &power, &volume_up, &volume_down };

	for (unsigned i = 0; i < 3; i++) {
		if (all[i]->usable && all[i]->last != all[i]->down) {
			return false;
		}
	}
	return true;
}

/* A press and its release both over before the look (the latch says there was an edge; the key is up again). */
static void
keys_tap(struct button *b, bool edge)
{
	if (edge && b->usable && b->live && !b->down && !b->last) {
		printf("buttons: %s was pressed and released between two looks\n", b->name);
		b->last = true;
		button_look(b, true);
		b->last = false;
		button_look(b, false);
	}
}

/* Until an edge rings (or the net). Then the latches, and what they say about taps the next look cannot see. */
static void
keys_wait(void)
{
	uint8_t pon_bits, pin_bits;
	bool rang;

	assert_wait_timeout((event_t)&keys_irq.rang, THREAD_UNINT,
	    keys_irq.delivered ? KEYS_NET_MS : hakios_screen_off() ? 3 * POLL_MS : POLL_MS, NSEC_PER_MSEC);
	if (os_atomic_load(&keys_irq.rang, acquire) != 0) {
		clear_wait(current_thread(), THREAD_AWAKENED);
	} else {
		(void)thread_block(THREAD_CONTINUE_NULL);
	}
	rang = os_atomic_xchg(&keys_irq.rang, 0, acquire) != 0;
	if (!rang) {
		keys_irq.nets++;
		return;
	}
	keys_irq_service(&pon_bits, &pin_bits);
	if (keys_irq.on) {
		gic_v3_spi_unmask(KEYS_INTID);
	} else {
		printf("buttons: MORE THAN %u INTERRUPTS IN A SECOND: the line is left masked; looking every %u ms\n", KEYS_STORM,
		    POLL_MS);
	}
	if ((pon_bits | pin_bits) == 0) {
		return;
	}
	keys_irq.edges++;
	keys_irq.delivered = true;
	buttons_poll();
	keys_tap(&power, pon_bits & PON_KPDPWR);
	keys_tap(resin_key, pon_bits & PON_RESIN);
	keys_tap(pin_key, pin_bits & GPIO_IRQ_BIT);
}

static void
buttons_thread(void *parameter, __unused wait_result_t result)
{
	uint32_t steps = (uint32_t)(uintptr_t)parameter, version, channels;
	bool pon_known, pin_known;

	pin.name = arb->pin_name;
	if (arb->resin_is_up) {
		resin_key = &volume_up;
		pin_key = &volume_down;
	}
	printf("buttons: step 1: map the PMIC arbiter of the %s (core %#lx, cnfg %#lx, observer %#lx) and read its version\n",
	    arb->soc, (unsigned long)arb->core_phys, (unsigned long)arb->cnfg_phys,
	    (unsigned long)(arb->observer_phys + (vm_offset_t)arb->observer_system * ARB_SYSTEM));
	arb_core = ml_io_map(arb->core_phys, arb->core_size);
	arb_cnfg = ml_io_map(arb->cnfg_phys, arb->cnfg_size);
	arb_observer = ml_io_map(arb->observer_phys + (vm_offset_t)arb->observer_system * ARB_SYSTEM, arb->observer_size);
	version = rd(arb_core + ARB_VERSION);
	channels = rd(arb_core + ARB_FEATURES) & 0x7ff;
	printf("buttons: PMIC arbiter version %#x, %u channels\n", version, channels);
	if (version < arb->version_from || version >= arb->version_to) {
		printf("buttons: the %s's layout is that of arbiter version %u (%#x to %#x): no buttons (wrong buttons_soc=?)\n",
		    arb->soc, arb->version_from >> 28, arb->version_from, arb->version_to - 1);
		goto out;
	}
	if (channels == 0 || channels > arb->max_channels) {
		printf("buttons: ... which is not a channel count this layout has room for: looking at all %u\n", arb->max_channels);
		channels = arb->max_channels;
	}
	if (steps < 2) {
		goto out;
	}

	printf("buttons: step 2: look through the arbiter's table for the channels of the two peripherals\n");
	find_channel(&pon, channels);
	find_channel(&pin, channels);
	find_channel(&rtc, channels);
	if (steps < 3) {
		goto out;
	}

	printf("buttons: step 3: the first transfers on the SPMI bus\n");
	pon_known = identify(&pon, PON_TYPE, PON_SUBTYPE_HLOS, PON_SUBTYPE_HLOS);
	pin_known = identify(&pin, GPIO_TYPE, GPIO_SUBTYPE_LV, GPIO_SUBTYPE_MV_VIN3);
	rtc_known = identify(&rtc, 0x07, 0x01, 0x01);   /* the clock's "write" block (pmic.c: type 0x07, subtype 0x01) */
	if (rtc_known) {
		uint8_t b[4];

		if (spmi_read(&rtc, 0x48, b, 4) == NULL) {
			printf("buttons: %s counts %u s (kern.hakios.rtc_counter)\n", rtc.name,
			    b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24);
		}
	}
	if (steps < 4) {
		goto out;
	}

	printf("buttons: step 4: how the keys' inputs are set up\n");
	if (pon_known) {
		pon_inputs();
	}
	if (pin_known) {
		pin_input();
	}
	if (!power.usable && !pin_key->usable) {
		printf("buttons: nothing that can be read: no buttons\n");
		goto out;
	}
	if (power.last || volume_down.last || volume_up.last) {
		printf("buttons: a key reads as held before anything was watched:%s%s%s. It is not reported until it has been "
		    "seen released\n", power.last ? " power" : "", volume_up.last ? " volume-up" : "",
		    volume_down.last ? " volume-down" : "");
	}
	/* (a key that is released now is believed at once) */
	power.live = power.usable && !power.last;
	volume_down.live = volume_down.usable && !volume_down.last;
	volume_up.live = volume_up.usable && !volume_up.last;
	if (steps < 5) {
		goto out;
	}

	printf("buttons: step 5: watching%s%s%s: power is the side button\n", power.usable ? " power" : "",
	    volume_up.usable ? " volume-up" : "", volume_down.usable ? " volume-down" : "");
	arb_channels = channels;
	keys_irq_start();
	uint64_t minute, next_summary = mach_absolute_time();

	nanoseconds_to_absolutetime(60 * NSEC_PER_SEC, &minute);
	next_summary += minute / 6;     /* once early on as well (10 s): a short run is to say what the reads cost too */
	for (;;) {
		buttons_poll();
		if (mach_absolute_time() >= next_summary) {
			say_summary();
			next_summary = mach_absolute_time() + minute;
		}
		if (bus.failed >= 1000 && bus.failed * 2 > bus.reads) {
			printf("buttons: more than half of %llu reads have failed: giving up\n", bus.reads);
			break;
		}
		if (keys_irq.on && keys_steady()) {
			keys_wait();
		} else {
			hakios_screen_pause(POLL_MS, 3 * POLL_MS);   /* (48 ms with the screen off: a press still lasts longer) */
		}
	}
out:
	if (steps < 5) {
		printf("buttons: stopping here (buttons_steps=%u)\n", steps);
	}
	thread_terminate(current_thread());
}

/*
 * boot-arg press_power=<seconds>: a short press of the power key that many seconds after boot, and another ten
 * seconds later, given to Darwin as the key's own are (hakios_touch_post): the lock, and the wake, tried with nobody
 * at the phone. press_power_ms=<ms> is how long each is held (default 150: a tap; 3000 and up is a long press).
 */
static uint32_t press_ms = 150, press_left;
static thread_call_t press_call;

static void
press_step(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	static bool down;
	uint64_t deadline;
	char when[64];

	down = !down;
	now_text(when, sizeof(when), mach_absolute_time());
	printf("buttons: press_power: power %s at %s\n", down ? "PRESSED (by the test)" : "released (by the test)", when);
	hakios_touch_post(down ? HAKIOS_BUTTON_DOWN : HAKIOS_BUTTON_UP, HAKIOS_BUTTON_LOCK, 0, 0);
	if (down) {
		clock_interval_to_deadline(press_ms, NSEC_PER_MSEC, &deadline);
	} else if (--press_left > 0) {
		clock_interval_to_deadline(10, NSEC_PER_SEC, &deadline);
	} else {
		return;
	}
	thread_call_enter_delayed(press_call, deadline);
}

static void
press_power_init(void)
{
	uint32_t seconds = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("press_power", &seconds, sizeof(seconds)) || seconds == 0) {
		return;
	}
	PE_parse_boot_argn("press_power_ms", &press_ms, sizeof(press_ms));
	press_left = 2;
	press_call = thread_call_allocate(press_step, NULL);
	clock_interval_to_deadline(seconds, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(press_call, deadline);
	printf("buttons: press_power: the power key will be pressed for %u ms at %u s and at %u s\n", press_ms, seconds,
	    seconds + 10);
}

void
hakios_buttons_init(void)
{
	uint32_t enabled = 0, steps = 5;
	thread_t thread;

	press_power_init();

	/*
	 * On with the touch screen (the phone, somebody at it) unless buttons=0 says otherwise: a hand
	 * has pressed all three keys and each press and release was read as it was made.
	 */
	if (!PE_parse_boot_argn("buttons", &enabled, sizeof(enabled))) {
		uint32_t touch_screen = 0;

		enabled = PE_parse_boot_argn("touchscreen", &touch_screen, sizeof(touch_screen)) && touch_screen != 0;
	}
	if (!enabled) {
		return;
	}
	PE_parse_boot_argn("buttons_steps", &steps, sizeof(steps));
	if (steps == 0) {
		return;
	}
	/* boot-arg buttons_soc=8450: the OnePlus 10 Pro's arbiter (version 7) instead of the X5 Pro's */
	uint32_t soc = 0;
	if (PE_parse_boot_argn("buttons_soc", &soc, sizeof(soc)) && soc == 8450) {
		arb = &arb_sm8450;
	}
	if (kernel_thread_start(buttons_thread, (void *)(uintptr_t)steps, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "buttons");
		thread_deallocate(thread);
	}
}

/*
 * Interrupts on the X5 Pro would take the same as on the 10 Pro (see "interrupts" above) with version 5's layout:
 * "chnls" 0x10000 per channel, "intr" 0x10000 per system, and that phone's PDC pin and GIC number from its tree.
 * Firmware on the 10 Pro leaves the keys' interrupts off (step 4: enabled 0); our system owns both channels and
 * gets their interrupts (step 2).
 */
