/* Native UFS write commands. No controller access; the window starts disabled. */
#ifndef _HAKIOS_UFSHC_WRITE_H_
#define _HAKIOS_UFSHC_WRITE_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

struct ufshc_write_window {
	uint64_t first, end;       /* absolute 4096-byte LBAs, end exclusive */
	bool enabled;
};

/* WRITE(10) only, ordinary LUN 0 only. No arithmetic may wrap. */
static inline bool
ufshc_write_range(const struct ufshc_write_window *w, uint8_t lun, uint64_t first, uint64_t blocks)
{
	return w->enabled && lun == 0 && w->first < w->end && w->end <= 0x100000000ULL &&
	    blocks != 0 && blocks <= 0xffff && first >= w->first && first < w->end &&
	    blocks <= w->end - first;
}

static inline void
ufshc_write_command(uint8_t upiu[32], uint8_t tag, uint32_t first, uint16_t blocks)
{
	uint32_t bytes = (uint32_t)blocks * 4096;

	memset(upiu, 0, 32);
	upiu[0] = 0x01;           /* COMMAND */
	upiu[1] = 0x20;           /* W, simple task */
	upiu[3] = tag;
	upiu[12] = (uint8_t)(bytes >> 24);
	upiu[13] = (uint8_t)(bytes >> 16);
	upiu[14] = (uint8_t)(bytes >> 8);
	upiu[15] = (uint8_t)bytes;
	upiu[16] = 0x2a;          /* WRITE(10), no FUA or other flags */
	upiu[18] = (uint8_t)(first >> 24);
	upiu[19] = (uint8_t)(first >> 16);
	upiu[20] = (uint8_t)(first >> 8);
	upiu[21] = (uint8_t)first;
	upiu[23] = (uint8_t)(blocks >> 8);
	upiu[24] = (uint8_t)blocks;
}

static inline void
ufshc_flush_command(uint8_t upiu[32], uint8_t tag)
{
	memset(upiu, 0, 32);
	upiu[0] = 0x01;
	upiu[3] = tag;
	/* SYNCHRONIZE CACHE(10), IMMED=0, LBA=0, count=0: wait for the whole LUN. */
	upiu[16] = 0x35;
}

/* Require the exact supported encoding, including flags, lengths and reserved bytes. */
static inline bool
ufshc_write_permitted(const struct ufshc_write_window *w, const uint8_t upiu[32], uint32_t dw0)
{
	uint8_t expected[32];
	uint64_t first = (uint64_t)upiu[18] << 24 | (uint64_t)upiu[19] << 16 |
	    (uint64_t)upiu[20] << 8 | upiu[21];
	uint32_t blocks = (uint32_t)upiu[23] << 8 | upiu[24];

	if (!w->enabled || w->first >= w->end || w->end > 0x100000000ULL) {
		return false;
	}
	if (upiu[16] == 0x2a) {
		if (dw0 != ((1u << 28) | (1u << 25)) || !ufshc_write_range(w, upiu[2], first, blocks)) {
			return false;
		}
		ufshc_write_command(expected, upiu[3], (uint32_t)first, (uint16_t)blocks);
	} else if (upiu[16] == 0x35) {
		if (dw0 != (1u << 28)) {
			return false;
		}
		ufshc_flush_command(expected, upiu[3]);
	} else {
		return false;
	}
	return memcmp(upiu, expected, 32) == 0;
}

#endif
