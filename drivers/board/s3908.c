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
 *   I2C        done by hand (board/i2c_gpio.h) on the board's two pins, taken for each transaction and given back as
 *              found. The serial engine of that bus has its FIFO off and its DMA locked by the secure world on negroni
 *   attention  low while the chip has a message: looked at every POLL_MS, and read until it is high again
 *   reset      low = in reset: pulsed only when the chip does not answer as the firmware left it
 *   report     a bit stream the chip's own "touch report config" (command 0x25) describes, asked for once at start
 *
 * Nothing is done unless a boot-arg asks for it:
 *   touchscreen=1   the chip's messages read and its touches posted
 *   touchscreen=-1  only looks: the pins, and the chip asked for its report config as the firmware left it
 *   touch_flip=N    bit 0 mirrors the touches left to right, bit 1 top to bottom (as the other touch drivers)
 *   s3908_trace=1   the first 40 fingers down, as the chip gives them, in the log
 *
 * At the run's end (hakios_s3908_quiesce()) the polling stops and a transaction cut off is ended, the bus's pins as the
 * run found them. The chip is left running: the firmware's touch driver reads it again after the run.
 *
 * Not ported yet from negroni's tree: the attention line as an interrupt (through the PDC: GPIO21 -> pin 132 -> GIC
 * 754 on SM8450), the bus through the serial engine and the QUP's GPI DMA (touch_bus=2), and the slower look while the
 * screen is off.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <sys/board/input.h>
#include <sys/systm.h>

#include "kernel.h"
#include "board.h"
#include "log.h"

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
i2c_read(uint8_t *buffer, uint32_t length)
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
i2c_write(const uint8_t *buffer, uint32_t length)
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
	uint32_t looks, messages, reports, positions, errors, bad;
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
			if (trace && seen.positions < 40) {
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
	if (tick % (5000 / POLL_MS) == 0 && (tick <= 120000 / POLL_MS || tick % (60000 / POLL_MS) == 0)) {
		say("s3908: %u messages (%u touch reports at %llu us each, %u not parsed), %u finger positions, %u errors; "
		    "attention %s\n", seen.messages, seen.reports, seen.reports ? seen.report_us / seen.reports : 0, seen.bad,
		    seen.positions, seen.errors, attention() ? "low" : "high");
	}
	return true;
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
	rc = chip_setup();
	if (rc != 0 && !__atomic_load_n(&leaving, __ATOMIC_SEQ_CST)) {
		reset_pulse();
		rc = chip_setup();
	}
	if (rc != 0) {
		say("s3908: no report config from the chip at 0x%02x: touch screen unavailable\n", ADDRESS);
		done();
		thread_terminate(current_thread());
		return;
	}
	done();
	say("s3908: running, %u x %u, %u fingers; the attention line (gpio%u) is looked at every %u ms\n", panel_x, panel_y,
	    contacts, PIN_ATTN, POLL_MS);
	for (unsigned tick = 1; begin(); tick++) {
		if (!poll_once(tick)) {
			release();
			say("s3908: stopped: the chip does not come back\n");
			done();
			break;
		}
		done();
		wait_ms(POLL_MS);
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
	if (tlmm == 0 || !__atomic_load_n(&on_bus, __ATOMIC_SEQ_CST)) {
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
