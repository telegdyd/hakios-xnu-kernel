/*
 * Synaptics TouchComm v1 as the S3908 speaks it (board/s3908.c): the framing of a message and the touch report, a bit
 * stream laid out by the chip's own "touch report config" (command 0x25). From the OPLUS driver
 * (synaptics_tcm_oncell.c) by way of negroni's drivers/input/hakios.s3908.c. Compiled by the host tests as well
 * (workbench/s3908_report_test.c).
 */
#ifndef _HAKIOS_S3908_REPORT_H
#define _HAKIOS_S3908_REPORT_H

#include <stdbool.h>
#include <stdint.h>

#define S3908_MARKER                    0xA5
#define S3908_PADDING                   0x5A
#define S3908_STATUS_IDLE               0x00
#define S3908_STATUS_OK                 0x01
#define S3908_STATUS_BUSY               0x02
#define S3908_STATUS_CONTINUED          0x03
#define S3908_REPORT_IDENTIFY           0x10
#define S3908_REPORT_TOUCH              0x11
#define S3908_CMD_GET_REPORT_CONFIG     0x25
#define S3908_MAX_OBJECTS               10
#define S3908_MAX_PAYLOAD               512
#define S3908_MAX_CONFIG                256

/* the codes of the touch report config */
enum {
	S3908_TOUCH_END = 0, S3908_TOUCH_FOREACH_ACTIVE_OBJECT, S3908_TOUCH_FOREACH_OBJECT, S3908_TOUCH_FOREACH_END,
	S3908_TOUCH_PAD_TO_NEXT_BYTE, S3908_TOUCH_TIMESTAMP, S3908_TOUCH_OBJECT_N_INDEX, S3908_TOUCH_OBJECT_N_CLASSIFICATION,
	S3908_TOUCH_OBJECT_N_X_POSITION, S3908_TOUCH_OBJECT_N_Y_POSITION, S3908_TOUCH_OBJECT_N_Z,
	S3908_TOUCH_NUM_OF_ACTIVE_OBJECTS = 24,
};

struct s3908_tcm {
	uint8_t config[S3908_MAX_CONFIG];
	uint32_t config_length;
	uint8_t payload[S3908_MAX_PAYLOAD];
	uint32_t payload_length;
	uint8_t code;
	uint8_t header[4];
	int bus_error;
	struct {
		uint8_t status;         /* 0 = no finger */
		uint32_t x, y;
	} object[S3908_MAX_OBJECTS];
};

/* up to 32 bits of the payload at a bit offset, least significant first */
static inline uint32_t
s3908_bits(const struct s3908_tcm *tcm, uint32_t offset, uint32_t bits)
{
	uint32_t out = 0, done = 0;

	if (bits == 0 || bits > 32 || offset + bits > tcm->payload_length * 8) {
		return 0;
	}
	while (done < bits) {
		uint32_t byte = offset / 8, shift = offset % 8;
		uint32_t take = 8 - shift < bits - done ? 8 - shift : bits - done;
		uint32_t v = ((uint32_t)tcm->payload[byte] >> shift) & (0xffu >> (8 - take));

		out |= v << done;
		done += take;
		offset += take;
	}
	return out;
}

/* a touch report into tcm->object[], as its config describes it: 0, or < 0 for one that cannot be */
static inline int
s3908_parse_touch(struct s3908_tcm *tcm)
{
	uint32_t idx = 0, offset = 0, obj = 0, next = 0, objects = 0, active = 0;
	bool active_only = false, have_active = false;

	for (int i = 0; i < S3908_MAX_OBJECTS; i++) {
		tcm->object[i].status = 0;
	}
	while (idx < tcm->config_length) {
		uint8_t code = tcm->config[idx++];
		uint32_t bits, v;

		switch (code) {
		case S3908_TOUCH_END:
			return 0;
		case S3908_TOUCH_FOREACH_ACTIVE_OBJECT:
		case S3908_TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = code == S3908_TOUCH_FOREACH_ACTIVE_OBJECT;
			break;
		case S3908_TOUCH_FOREACH_END:
			if (active_only) {
				if (have_active) {
					if (++objects < active) {
						idx = next;
					}
				} else if (offset < tcm->payload_length * 8) {
					idx = next;
				}
			} else if (++obj < S3908_MAX_OBJECTS) {
				idx = next;
			}
			break;
		case S3908_TOUCH_PAD_TO_NEXT_BYTE:
			offset = (offset + 7) & ~7u;
			break;
		default:
			if (idx >= tcm->config_length) {
				return -1;
			}
			bits = tcm->config[idx++];
			v = s3908_bits(tcm, offset, bits);
			offset += bits;
			switch (code) {
			case S3908_TOUCH_OBJECT_N_INDEX:
				if (v >= S3908_MAX_OBJECTS) {
					return -1;
				}
				obj = v;
				break;
			case S3908_TOUCH_OBJECT_N_CLASSIFICATION:
				if (obj < S3908_MAX_OBJECTS) {
					tcm->object[obj].status = (uint8_t)v;
				}
				break;
			case S3908_TOUCH_OBJECT_N_X_POSITION:
				if (obj < S3908_MAX_OBJECTS) {
					tcm->object[obj].x = v;
				}
				break;
			case S3908_TOUCH_OBJECT_N_Y_POSITION:
				if (obj < S3908_MAX_OBJECTS) {
					tcm->object[obj].y = v;
				}
				break;
			case S3908_TOUCH_NUM_OF_ACTIVE_OBJECTS:
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

#endif /* _HAKIOS_S3908_REPORT_H */
