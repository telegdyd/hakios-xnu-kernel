/*
 * iris_nv12: encode an NV12 file on the phone's video encoder (Iris2, /dev/hakios-iris) as 48 HEVC tiles.
 *
 *     iris_nv12 in.nv12 out.hevc
 *
 * in.nv12 is what icp_probe=5 publishes (/dev/hakios-icp-nv12-full: a 40-byte "NV12BPS1" header with width, height,
 * stride, luma bytes and chroma bytes, then the two planes), e.g. the ISP's 4000 x 3008 image with stride 4032. The
 * image is placed top-left in Iris2's 4032 x 3024 frame, the rest black, and the Annex B stream (VPS/SPS/PPS, then
 * 48 raster-order 512 x 512 tiles) is written to out.hevc. Prints the size of each packet and the encode time.
 * CAMERA-PLAN section 5 step 4.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include "hakios.iris.h"

struct nv12_header {
	char magic[8];
	uint32_t width, height, stride, luma, chroma, reserved[3];
};

int
main(int argc, char **argv)
{
	if (argc != 3) {
		fprintf(stderr, "usage: %s in.nv12 out.hevc\n", argv[0]);
		return 2;
	}
	int in = open(argv[1], O_RDONLY);
	struct stat st;
	if (in < 0 || fstat(in, &st) != 0) {
		perror(argv[1]);
		return 1;
	}
	uint8_t *file = malloc((size_t)st.st_size);
	if (!file || read(in, file, (size_t)st.st_size) != st.st_size) {
		perror("read");
		return 1;
	}
	close(in);
	struct nv12_header h;
	memcpy(&h, file, sizeof(h));
	if (memcmp(h.magic, "NV12BPS1", 8) || (uint64_t)sizeof(h) + h.luma + h.chroma > (uint64_t)st.st_size ||
	    h.width > HAKIOS_IRIS_WIDTH || h.height > HAKIOS_IRIS_HEIGHT || h.stride < h.width) {
		fprintf(stderr, "%s: not an NV12BPS1 image that fits %ux%u\n", argv[1], HAKIOS_IRIS_WIDTH, HAKIOS_IRIS_HEIGHT);
		return 1;
	}
	const uint32_t W = HAKIOS_IRIS_WIDTH, H = HAKIOS_IRIS_HEIGHT;
	uint8_t *frame = malloc(HAKIOS_IRIS_INPUT_BYTES);
	uint8_t *out = malloc(HAKIOS_IRIS_OUTPUT_MAX);
	if (!frame || !out) {
		perror("malloc");
		return 1;
	}
	memset(frame, 16, (size_t)W * H);                       /* black: Y 16, U = V = 128 */
	memset(frame + (size_t)W * H, 128, (size_t)W * H / 2);
	const uint8_t *luma = file + sizeof(h), *chroma = luma + h.luma;
	for (uint32_t y = 0; y < h.height; ++y) {
		memcpy(frame + (size_t)y * W, luma + (size_t)y * h.stride, h.width);
	}
	for (uint32_t y = 0; y < h.height / 2; ++y) {
		memcpy(frame + (size_t)W * H + (size_t)y * W, chroma + (size_t)y * h.stride, h.width);
	}
	int dev = open(HAKIOS_IRIS_DEVICE, O_RDWR);
	if (dev < 0) {
		perror(HAKIOS_IRIS_DEVICE);
		return 1;
	}
	struct hakios_iris_still s = {
		.input = (uint64_t)(uintptr_t)frame,
		.output = (uint64_t)(uintptr_t)out,
		.capacity = HAKIOS_IRIS_OUTPUT_MAX,
	};
	if (ioctl(dev, HAKIOS_IRIS_ENCODE_STILL, &s) != 0) {
		fprintf(stderr, "HAKIOS_IRIS_ENCODE_STILL: %s\n", strerror(errno));
		return 1;
	}
	close(dev);
	printf("%ux%u (stride %u) in a %ux%u frame: %u bytes in %llu us\n", h.width, h.height, h.stride, W, H, s.bytes,
	    (unsigned long long)s.elapsed_us);
	uint32_t smallest = UINT32_MAX, largest = 0;
	for (unsigned i = 1; i < HAKIOS_IRIS_PACKETS; ++i) {
		smallest = s.packet_bytes[i] < smallest ? s.packet_bytes[i] : smallest;
		largest = s.packet_bytes[i] > largest ? s.packet_bytes[i] : largest;
	}
	printf("parameter sets %u bytes; tiles %u to %u bytes\n", s.packet_bytes[0], smallest, largest);
	for (unsigned row = 0; row < 6; ++row) {
		printf("  row %u:", row);
		for (unsigned col = 0; col < 8; ++col) {
			printf(" %6u", s.packet_bytes[1 + row * 8 + col]);
		}
		printf("\n");
	}
	FILE *f = fopen(argv[2], "wb");
	if (!f || fwrite(out, 1, s.bytes, f) != s.bytes || fclose(f) != 0) {
		perror(argv[2]);
		return 1;
	}
	return 0;
}
