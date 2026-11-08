/*
 * The touch screen of the OnePlus 10 Pro (negroni): a Synaptics S3908 ("TouchComm" v1) at I2C address 0x4b,
 * as a source of touches for /dev/hakios-touch (sys/hakios.input.h). Only with boot-arg touchscreen=1.
 *
 * What this relies on, and what it does not do:
 *   power    none: the firmware's NegroniTouchDxe (Mu) has switched the panel's supplies on and leaves them on
 *   bus      the serial engine that serves the chip (QUP0 SE4) cannot be used: its FIFO is off and its DMA
 *            block is locked by the secure world. So the two wires, GPIO16 (SDA) and GPIO17 (SCL), are driven
 *            by hand, open drain, with the pull-ups of the TLMM
 *   chip     reset is GPIO20 (low = in reset), the attention line is GPIO21 (low = a message is waiting)
 *   report   a bit stream described by the chip's own "touch report config" (command 0x25), read once at start
 * No interrupt is used: a thread looks at the attention line every 10 ms.
 *
 * Written from slop's negroni-ios patch 0093 (bsd/dev/negroni_touch*), which took it from the OPLUS driver
 * (drivers/input/touchscreen/synaptics_tcm_oncell.c). Status: compiled; not yet run on the phone.
 * With touchscreen=1 and -v a line on the console says how it is doing every few seconds (s3908: ...).
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* ---- pins (TLMM: GPIO n has its configuration at n * 0x1000 and its level at +4) ------------------- */

#define TLMM_PINS_PHYS          0x0f110000UL    /* GPIO16 ... */
#define TLMM_PINS_SIZE          0x8000UL        /* ... GPIO23 */
#define PIN_SDA                 0x0000UL        /* GPIO16 */
#define PIN_SCL                 0x1000UL        /* GPIO17 */
#define PIN_RESET               0x4000UL        /* GPIO20 */
#define PIN_ATTN                0x5000UL        /* GPIO21 */
#define CFG_LINE                3u              /* GPIO function, 2 mA, pull up */
#define CFG_OE                  (1u << 9)
#define IO_OUT                  2u

#define S3908_ADDRESS           0x4b
#define PANEL_MAX_X             5760            /* the chip's coordinates (touchpanel,panel-coords) */
#define PANEL_MAX_Y             12864

static volatile uint8_t *pins;

static inline uint32_t
rd(unsigned long offset)
{
	return *(volatile uint32_t *)(pins + offset);
}

static inline void
wr(unsigned long offset, uint32_t value)
{
	*(volatile uint32_t *)(pins + offset) = value;
}

static void
delay_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

static bool
attention(void)
{
	return rd(PIN_ATTN + 4) & 1;    /* true = high = nothing waiting */
}

/* ---- I2C by hand: a line is let go (pulled high) or driven low, never driven high ----------------- */

static void
line(unsigned long pin, bool release)
{
	wr(pin + 4, 0);
	wr(pin, CFG_LINE | (release ? 0 : CFG_OE));
	(void)rd(pin);
}

static inline bool
sda_get(void)
{
	return rd(PIN_SDA + 4) & 1;
}

static inline bool
scl_get(void)
{
	return rd(PIN_SCL + 4) & 1;
}

/* SCL high, waiting for a chip that holds it low (stretching): 0, or -4 after 1000 tries */
static int
scl_high(void)
{
	line(PIN_SCL, true);
	for (unsigned n = 0; n < 1000; n++) {
		if (scl_get()) {
			delay_us(5);
			return 0;
		}
		delay_us(1);
	}
	return -4;
}

static void
scl_low(void)
{
	line(PIN_SCL, false);
	delay_us(5);
}

static int
i2c_stop(void)
{
	int rc;

	scl_low();
	line(PIN_SDA, false);
	delay_us(5);
	rc = scl_high();
	line(PIN_SDA, true);
	delay_us(5);
	return rc;
}

/* one byte out and the chip's answer: 0, -2 (not acknowledged) or -4 */
static int
i2c_byte_out(uint8_t value)
{
	int rc;

	for (unsigned i = 0; i < 8; i++) {
		line(PIN_SDA, (value & 0x80) != 0);
		delay_us(5);
		if ((rc = scl_high()) != 0) {
			return rc;
		}
		scl_low();
		value <<= 1;
	}
	line(PIN_SDA, true);
	delay_us(5);
	if ((rc = scl_high()) != 0) {
		return rc;
	}
	bool nack = sda_get();
	scl_low();
	return nack ? -2 : 0;
}

static int
i2c_start(uint8_t address, bool read)
{
	int rc;

	line(PIN_SDA, true);
	if ((rc = scl_high()) != 0) {
		return rc;
	}
	if (!sda_get()) {
		return -3;      /* the bus is held */
	}
	line(PIN_SDA, false);
	delay_us(5);
	scl_low();
	return i2c_byte_out((uint8_t)(address << 1 | (read ? 1 : 0)));
}

static int
i2c_write(const uint8_t *buffer, uint32_t length)
{
	int rc = i2c_start(S3908_ADDRESS, false);

	for (uint32_t i = 0; rc == 0 && i < length; i++) {
		rc = i2c_byte_out(buffer[i]);
	}
	int stop = i2c_stop();
	return rc ? rc : stop;
}

static int
i2c_read(uint8_t *buffer, uint32_t length)
{
	int rc = i2c_start(S3908_ADDRESS, true);

	for (uint32_t i = 0; rc == 0 && i < length; i++) {
		uint8_t value = 0;

		line(PIN_SDA, true);
		for (unsigned bit = 0; bit < 8; bit++) {
			if ((rc = scl_high()) != 0) {
				break;
			}
			value = (uint8_t)(value << 1 | (sda_get() ? 1 : 0));
			scl_low();
		}
		if (rc) {
			break;
		}
		buffer[i] = value;
		line(PIN_SDA, i + 1 == length);         /* acknowledge every byte but the last */
		delay_us(5);
		rc = scl_high();
		scl_low();
		line(PIN_SDA, true);
	}
	int stop = i2c_stop();
	return rc ? rc : stop;
}

/* ---- TouchComm v1 ------------------------------------------------------------------------------- */

#define TCM_MARKER              0xA5
#define TCM_PADDING             0x5A
#define TCM_STATUS_IDLE         0x00
#define TCM_STATUS_OK           0x01
#define TCM_STATUS_BUSY         0x02
#define TCM_STATUS_CONTINUED    0x03
#define TCM_REPORT_IDENTIFY     0x10
#define TCM_REPORT_TOUCH        0x11
#define TCM_CMD_GET_REPORT_CONFIG 0x25
#define TCM_MAX_OBJECTS         10
#define TCM_MAX_PAYLOAD         512
#define TCM_MAX_CONFIG          256

/* the codes of the touch report config */
enum {
	TOUCH_END = 0, TOUCH_FOREACH_ACTIVE_OBJECT, TOUCH_FOREACH_OBJECT, TOUCH_FOREACH_END, TOUCH_PAD_TO_NEXT_BYTE,
	TOUCH_TIMESTAMP, TOUCH_OBJECT_N_INDEX, TOUCH_OBJECT_N_CLASSIFICATION, TOUCH_OBJECT_N_X_POSITION,
	TOUCH_OBJECT_N_Y_POSITION, TOUCH_OBJECT_N_Z, TOUCH_NUM_OF_ACTIVE_OBJECTS = 24,
};

static struct {
	uint8_t config[TCM_MAX_CONFIG];
	uint32_t config_length;
	uint8_t payload[TCM_MAX_PAYLOAD];
	uint32_t payload_length;
	uint8_t code;
	uint8_t header[4];
	int bus_error;
	struct {
		uint8_t status;         /* 0 = no finger */
		uint32_t x, y;
	} object[TCM_MAX_OBJECTS];
} tcm;

/* One message: 0 = it is in tcm.code / tcm.payload, 1 = nothing waiting, < 0 = the bus or the framing failed */
static int
tcm_read_message(void)
{
	static uint8_t buffer[TCM_MAX_PAYLOAD + 3];
	uint32_t length;

	tcm.code = 0;
	tcm.payload_length = 0;
	tcm.header[0] = tcm.header[1] = tcm.header[2] = tcm.header[3] = 0;
	if ((tcm.bus_error = i2c_read(tcm.header, 4)) != 0) {
		return -1;
	}
	if (tcm.header[0] != TCM_MARKER) {
		return 1;
	}
	tcm.code = tcm.header[1];
	if (tcm.code == TCM_STATUS_IDLE || tcm.code == TCM_STATUS_BUSY) {
		return 1;
	}
	if (tcm.code == TCM_STATUS_CONTINUED) {
		return -3;
	}
	length = tcm.header[2] | (uint32_t)tcm.header[3] << 8;
	if (length == 0) {
		return 0;
	}
	if (length > TCM_MAX_PAYLOAD) {
		return -2;
	}
	if ((tcm.bus_error = i2c_read(buffer, length + 3)) != 0) {
		return -1;
	}
	if (buffer[0] != TCM_MARKER || buffer[1] != TCM_STATUS_CONTINUED || buffer[length + 2] != TCM_PADDING) {
		return -3;
	}
	for (uint32_t i = 0; i < length; i++) {
		tcm.payload[i] = buffer[2 + i];
	}
	tcm.payload_length = length;
	return 0;
}

/* a command with no payload, and its answer (reports that come in between are dropped) */
static int
tcm_command(uint8_t command)
{
	uint8_t message[3] = { command, 0, 0 };

	if ((tcm.bus_error = i2c_write(message, sizeof(message))) != 0) {
		return -1;
	}
	for (unsigned tries = 0; tries < 200; tries++) {
		int rc = tcm_read_message();

		if (rc < 0) {
			return rc;
		}
		if (rc == 0 && tcm.code < TCM_REPORT_IDENTIFY) {
			return tcm.code == TCM_STATUS_OK ? 0 : -4;
		}
		sleep_ms(1);
	}
	return -5;
}

static int
tcm_get_report_config(void)
{
	int rc = tcm_command(TCM_CMD_GET_REPORT_CONFIG);

	if (rc) {
		return rc;
	}
	if (tcm.payload_length > TCM_MAX_CONFIG) {
		return -2;
	}
	for (uint32_t i = 0; i < tcm.payload_length; i++) {
		tcm.config[i] = tcm.payload[i];
	}
	tcm.config_length = tcm.payload_length;
	return 0;
}

/* up to 32 bits of the payload at a bit offset, least significant first */
static uint32_t
tcm_bits(uint32_t offset, uint32_t bits)
{
	uint32_t out = 0, done = 0;

	if (bits == 0 || bits > 32 || offset + bits > tcm.payload_length * 8) {
		return 0;
	}
	while (done < bits) {
		uint32_t byte = offset / 8, shift = offset % 8;
		uint32_t take = 8 - shift < bits - done ? 8 - shift : bits - done;
		uint32_t v = ((uint32_t)tcm.payload[byte] >> shift) & (0xffu >> (8 - take));

		out |= v << done;
		done += take;
		offset += take;
	}
	return out;
}

/* a touch report into tcm.object[], as its config describes it: 0 or < 0 */
static int
tcm_parse_touch(void)
{
	uint32_t idx = 0, offset = 0, obj = 0, next = 0, objects = 0, active = 0;
	bool active_only = false, have_active = false;

	for (int i = 0; i < TCM_MAX_OBJECTS; i++) {
		tcm.object[i].status = 0;
	}
	while (idx < tcm.config_length) {
		uint8_t code = tcm.config[idx++];
		uint32_t bits, v;

		switch (code) {
		case TOUCH_END:
			return 0;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = code == TOUCH_FOREACH_ACTIVE_OBJECT;
			break;
		case TOUCH_FOREACH_END:
			if (active_only) {
				if (have_active) {
					if (++objects < active) {
						idx = next;
					}
				} else if (offset < tcm.payload_length * 8) {
					idx = next;
				}
			} else if (++obj < TCM_MAX_OBJECTS) {
				idx = next;
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = (offset + 7) & ~7u;
			break;
		default:
			if (idx >= tcm.config_length) {
				return -1;
			}
			bits = tcm.config[idx++];
			v = tcm_bits(offset, bits);
			offset += bits;
			switch (code) {
			case TOUCH_OBJECT_N_INDEX:
				if (v >= TCM_MAX_OBJECTS) {
					return -1;
				}
				obj = v;
				break;
			case TOUCH_OBJECT_N_CLASSIFICATION:
				tcm.object[obj].status = (uint8_t)v;
				break;
			case TOUCH_OBJECT_N_X_POSITION:
				tcm.object[obj].x = v;
				break;
			case TOUCH_OBJECT_N_Y_POSITION:
				tcm.object[obj].y = v;
				break;
			case TOUCH_NUM_OF_ACTIVE_OBJECTS:
				active = v;
				have_active = true;
				if (active == 0) {
					return 0;
				}
				break;
			default:        /* timestamp, z, widths, gestures: skipped */
				break;
			}
		}
	}
	return 0;
}

/* ---- touches ------------------------------------------------------------------------------------ */

static struct {
	bool down;
	uint32_t x, y;
} contact[TCM_MAX_OBJECTS];
static uint32_t flip;           /* boot-arg touch_flip: 1 = mirror left-right, 2 = mirror top-bottom */
static struct {
	uint32_t polls, messages, touches, errors, last_x, last_y;
} seen;

static void
touch_report(void)
{
	bool now[TCM_MAX_OBJECTS];

	for (unsigned id = 0; id < TCM_MAX_OBJECTS; id++) {
		now[id] = false;
		if (tcm.object[id].status == 0) {
			continue;
		}
		uint32_t px = tcm.object[id].x, py = tcm.object[id].y;
		uint32_t x = (px >= PANEL_MAX_X ? PANEL_MAX_X : px) * 65535ull / PANEL_MAX_X;
		uint32_t y = (py >= PANEL_MAX_Y ? PANEL_MAX_Y : py) * 65535ull / PANEL_MAX_Y;

		if (flip & 1) {
			x = 65535 - x;
		}
		if (flip & 2) {
			y = 65535 - y;
		}
		now[id] = true;
		seen.touches++;
		seen.last_x = px;
		seen.last_y = py;
		if (!contact[id].down) {
			hakios_touch_post(HAKIOS_TOUCH_DOWN, id, x, y);
		} else if (contact[id].x != x || contact[id].y != y) {
			hakios_touch_post(HAKIOS_TOUCH_MOVE, id, x, y);
		}
		contact[id].down = true;
		contact[id].x = x;
		contact[id].y = y;
	}
	for (unsigned id = 0; id < TCM_MAX_OBJECTS; id++) {
		if (contact[id].down && !now[id]) {
			contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
	}
}

static void
touch_release_all(void)
{
	for (unsigned id = 0; id < TCM_MAX_OBJECTS; id++) {
		tcm.object[id].status = 0;
	}
	touch_report();
}

/* Drain what is waiting (the identify report after a reset, old touches), then learn the report layout. */
static int
chip_setup(void)
{
	int rc = 1;

	for (unsigned i = 0; i < 50 && !attention(); i++) {
		rc = tcm_read_message();
		if (rc < 0) {
			break;
		}
	}
	printf("s3908: drain rc=%d code=%#x header %02x %02x %02x %02x bus=%d attention=%d\n", rc, tcm.code,
	    tcm.header[0], tcm.header[1], tcm.header[2], tcm.header[3], tcm.bus_error, attention());
	rc = tcm_get_report_config();
	printf("s3908: report config rc=%d length=%u bus=%d\n", rc, tcm.config_length, tcm.bus_error);
	return rc;
}

static void
s3908_thread(__unused void *argument, __unused wait_result_t wait)
{
	uint32_t beat = 0, since_beat = 0;

	sleep_ms(5000);         /* the system is up by then, and the screen shows what happens */
	pins = (volatile uint8_t *)ml_io_map(TLMM_PINS_PHYS, TLMM_PINS_SIZE);
	if (pins == NULL) {
		printf("s3908: cannot map the pins\n");
		return;
	}
	line(PIN_SDA, true);
	line(PIN_SCL, true);
	delay_us(5);
	printf("s3908: reset=%u attention=%d sda=%d scl=%d (the firmware leaves the panel powered)\n",
	    (rd(PIN_RESET + 4) >> 1) & 1, attention(), sda_get(), scl_get());

	if (chip_setup() != 0) {
		/* reset the chip (low for 10 ms, then 200 ms for its firmware to start) and try once more */
		wr(PIN_RESET + 4, 0);
		sleep_ms(10);
		wr(PIN_RESET + 4, IO_OUT);
		sleep_ms(200);
		if (chip_setup() != 0) {
			printf("s3908: no report config from the chip: stopping\n");
			return;
		}
	}
	printf("s3908: running (%d x %d), looking at the attention line every 10 ms\n", PANEL_MAX_X, PANEL_MAX_Y);

	for (;;) {
		for (unsigned n = 0; n < 4 && !attention(); n++) {
			int rc = tcm_read_message();

			if (rc < 0) {
				seen.errors++;
				touch_release_all();
				break;
			}
			seen.messages++;
			if (rc == 0 && tcm.code == TCM_REPORT_TOUCH && tcm_parse_touch() == 0) {
				touch_report();
			}
		}
		seen.polls++;
		sleep_ms(10);
		if (++since_beat >= 500) {      /* every 5 s: a line for the console, every minute once it is quiet */
			since_beat = 0;
			if (beat++ < 24 || beat % 12 == 0) {
				printf("s3908: %u looks, %u messages, %u finger positions (last %u, %u), %u errors, attention %s\n",
				    seen.polls, seen.messages, seen.touches, seen.last_x, seen.last_y, seen.errors,
				    attention() ? "high" : "low");
			}
		}
	}
}

void
hakios_s3908_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("touchscreen", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	PE_parse_boot_argn("touch_flip", &flip, sizeof(flip));
	if (kernel_thread_start(s3908_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
