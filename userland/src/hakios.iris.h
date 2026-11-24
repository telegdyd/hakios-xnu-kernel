#ifndef HAKIOS_IRIS_H
/* (a copy of the kernel's bsd/sys/hakios.iris.h, for /dev/hakios-iris) */
#define HAKIOS_IRIS_H

#include <stdint.h>
#include <sys/ioccom.h>

#define HAKIOS_IRIS_DEVICE "/dev/hakios-iris"
#define HAKIOS_IRIS_WIDTH 4032u
#define HAKIOS_IRIS_HEIGHT 3024u
#define HAKIOS_IRIS_TILE 512u
#define HAKIOS_IRIS_INPUT_BYTES (HAKIOS_IRIS_WIDTH * HAKIOS_IRIS_HEIGHT * 3u / 2u)
#define HAKIOS_IRIS_PACKETS 49u /* VPS/SPS/PPS, followed by 48 raster-order tiles */
#define HAKIOS_IRIS_OUTPUT_MAX (HAKIOS_IRIS_PACKETS * 983040u)
#define HAKIOS_IRIS_MAX_STRIDE 16384u

/*
 * NV12 to an Annex B stream of 512x512 HEVC tiles. All addresses are in the calling process.
 * input is the luma plane. input_uv is the interleaved chroma plane, or 0 when it follows
 * packed luma. A stride of 0 means the image width. color_info is HFI_PROP_SIGNAL_COLOR_INFO
 * (matrix | transfer << 8 | primaries << 16 | description present << 24 | full range << 25 |
 * video format << 26 | signal type present << 29), or 0 for no colour signalling.
 */
struct hakios_iris_still {
	uint64_t input;
	uint64_t output;
	uint32_t capacity;
	uint32_t bytes;
	uint32_t packet_bytes[HAKIOS_IRIS_PACKETS];
	uint64_t elapsed_us;
	uint64_t input_uv;
	uint32_t stride;
	uint32_t stride_uv;
	uint32_t color_info;
	uint32_t reserved;
};

#define HAKIOS_IRIS_ENCODE_STILL _IOWR('I', 2, struct hakios_iris_still)

#endif
