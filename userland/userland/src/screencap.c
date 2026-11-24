/*
 * screencap: what the panel shows, as a PNG (the kernel's kern.hakios.screen, MiniIOMobileFramebuffer.cpp).
 *
 * usage: screencap [FILE]
 * Without FILE the PNG goes to standard output, for sweet-sh.py on the PC:
 *   sweet-sh.py /hakios/bin/screencap > screen.png    (sweet-sh adds a newline at the end; a PNG reader ignores it)
 * To standard output it writes the PNG alone (sweetd merges standard error into it: a failure's line then shows instead
 * of a picture); with FILE a line says the size and how long the read and the encoding took. Root only, as the sysctl is.
 */
#include <CoreFoundation/CoreFoundation.h>
#include <CoreGraphics/CoreGraphics.h>
#include <ImageIO/ImageIO.h>
#include <sys/sysctl.h>
#include <mach/mach_time.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

struct screen_header {
	char magic[8];
	uint32_t width, height, rowBytes, format;
};

static double
ms_since(uint64_t from)
{
	static mach_timebase_info_data_t tb;

	if (tb.denom == 0) {
		mach_timebase_info(&tb);
	}
	return (double) (mach_absolute_time() - from) * tb.numer / tb.denom / 1e6;
}

int
main(int argc, char **argv)
{
	size_t size = 0;
	uint64_t began = mach_absolute_time();

	if (argc > 2 || (argc == 2 && argv[1][0] == '-')) {
		fprintf(stderr, "usage: screencap [FILE]   (no FILE: the PNG on standard output)\n");
		return 2;
	}
	if (sysctlbyname("kern.hakios.screen", NULL, &size, NULL, 0) != 0 || size <= sizeof(struct screen_header)) {
		fprintf(stderr, "screencap: kern.hakios.screen: %s\n", size ? "too small" : strerror(errno));
		return 1;
	}
	uint8_t *buf = malloc(size);
	if (buf == NULL || sysctlbyname("kern.hakios.screen", buf, &size, NULL, 0) != 0) {
		fprintf(stderr, "screencap: reading kern.hakios.screen (%zu bytes): %s\n", size, strerror(errno));
		return 1;
	}
	double readMs = ms_since(began);
	struct screen_header h;
	memcpy(&h, buf, sizeof(h));
	if (memcmp(h.magic, "HKSCREEN", 8) != 0 || h.format != 0 || h.width == 0 || h.height == 0 ||
	    size < sizeof(h) + (size_t) h.rowBytes * h.height) {
		fprintf(stderr, "screencap: not a screen this understands (%zu bytes)\n", size);
		return 1;
	}

	/* BGRA in memory, the fourth byte not alpha: little-endian words of xRGB */
	CGColorSpaceRef space = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
	CFDataRef pixels = CFDataCreateWithBytesNoCopy(NULL, buf + sizeof(h), (CFIndex) h.rowBytes * h.height, kCFAllocatorNull);
	CGDataProviderRef provider = CGDataProviderCreateWithCFData(pixels);
	CGImageRef image = CGImageCreate(h.width, h.height, 8, 32, h.rowBytes, space,
	    kCGBitmapByteOrder32Little | (CGBitmapInfo) kCGImageAlphaNoneSkipFirst, provider, NULL, false,
	    kCGRenderingIntentDefault);
	CFMutableDataRef png = CFDataCreateMutable(NULL, 0);
	CGImageDestinationRef dest = CGImageDestinationCreateWithData(png, CFSTR("public.png"), 1, NULL);
	if (image == NULL || dest == NULL) {
		fprintf(stderr, "screencap: no image or no PNG encoder\n");
		return 1;
	}
	CGImageDestinationAddImage(dest, image, NULL);
	if (!CGImageDestinationFinalize(dest)) {
		fprintf(stderr, "screencap: the PNG was not made\n");
		return 1;
	}

	FILE *out = argc == 2 ? fopen(argv[1], "wb") : stdout;
	if (out == NULL) {
		fprintf(stderr, "screencap: %s: %s\n", argv[1], strerror(errno));
		return 1;
	}
	size_t len = (size_t) CFDataGetLength(png);
	if (fwrite(CFDataGetBytePtr(png), 1, len, out) != len || fflush(out) != 0) {
		fprintf(stderr, "screencap: writing the PNG: %s\n", strerror(errno));
		return 1;
	}
	if (out == stdout) {
		return 0;       /* (nothing more: sweetd merges standard error into the PNG's stream) */
	}
	fclose(out);
	printf("screencap: %ux%u, %zu bytes of PNG; read in %.1f ms, all in %.1f ms\n", h.width, h.height, len, readMs,
	    ms_since(began));
	return 0;
}
