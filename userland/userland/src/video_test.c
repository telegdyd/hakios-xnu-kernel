/*
 * video_test: a video layer for the display controller without a video player (MiniIOMobileFramebuffer.cpp, fb_video).
 *
 *   video_test [SECONDS] [f]   a 1280 x 720 '420v' surface ('420f' with f) of colour bars, a white bar moving across
 *                              them, handed to the kernel as kern.hakios.fb_video_test for SECONDS (10), then taken back
 *
 * The kernel shows it under every frame the GPU puts into a buffer of fb_flip's, 960 x 540 at 60, 800 of the panel,
 * with a hole cut into the frame over it: what a player's layer and CoreAnimation's hole would be. The bars are the
 * 75 % ones of BT.601 (white, yellow, cyan, green, magenta, red, blue, black), so the colours on the panel say whether
 * the conversion is right; the bar says whether the frames follow the surface.
 */
#include <CoreFoundation/CoreFoundation.h>
#include <IOSurface/IOSurfaceRef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/sysctl.h>
#include <unistd.h>

#define WIDTH   1280
#define HEIGHT  720

/* BT.601 75 % bars, video range (Y, Cb, Cr); full range is worked out from them */
static const unsigned char kBars[8][3] = {
	{ 180, 128, 128 }, { 162, 44, 142 }, { 131, 156, 44 }, { 112, 72, 58 },
	{ 84, 184, 198 }, { 65, 100, 212 }, { 35, 212, 114 }, { 16, 128, 128 },
};

static CFNumberRef
num(int value)
{
	return CFNumberCreate(NULL, kCFNumberIntType, &value);
}

static unsigned char
full_luma(int y)
{
	int v = (y - 16) * 255 / 219;
	return (unsigned char) (v < 0 ? 0 : v > 255 ? 255 : v);
}

static unsigned char
full_chroma(int c)
{
	int v = 128 + (c - 128) * 255 / 224;
	return (unsigned char) (v < 0 ? 0 : v > 255 ? 255 : v);
}

static void
draw(IOSurfaceRef s, int frame, int full)
{
	unsigned char *luma = IOSurfaceGetBaseAddressOfPlane(s, 0), *chroma = IOSurfaceGetBaseAddressOfPlane(s, 1);
	size_t lrow = IOSurfaceGetBytesPerRowOfPlane(s, 0), crow = IOSurfaceGetBytesPerRowOfPlane(s, 1);
	int bar = (frame * 8) % WIDTH;

	IOSurfaceLock(s, 0, NULL);
	for (int y = 0; y < HEIGHT; y++) {
		for (int x = 0; x < WIDTH; x++) {
			int moving = x >= bar && x < bar + 32;
			int yv = moving ? 235 : kBars[x * 8 / WIDTH][0];
			luma[(size_t) y * lrow + x] = full ? full_luma(yv) : (unsigned char) yv;
		}
	}
	for (int y = 0; y < HEIGHT / 2; y++) {
		for (int x = 0; x < WIDTH / 2; x++) {
			int moving = 2 * x >= bar && 2 * x < bar + 32;
			int cb = moving ? 128 : kBars[2 * x * 8 / WIDTH][1], cr = moving ? 128 : kBars[2 * x * 8 / WIDTH][2];
			chroma[(size_t) y * crow + 2 * x] = full ? full_chroma(cb) : (unsigned char) cb;
			chroma[(size_t) y * crow + 2 * x + 1] = full ? full_chroma(cr) : (unsigned char) cr;
		}
	}
	IOSurfaceUnlock(s, 0, NULL);
}

int
main(int argc, char **argv)
{
	int seconds = argc > 1 ? atoi(argv[1]) : 10, full = argc > 2 && argv[2][0] == 'f';
	CFMutableDictionaryRef props = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks,
	    &kCFTypeDictionaryValueCallBacks);
	CFNumberRef n;
	IOSurfaceRef s;
	unsigned sid, off = 0;

	n = num(WIDTH); CFDictionarySetValue(props, kIOSurfaceWidth, n); CFRelease(n);
	n = num(HEIGHT); CFDictionarySetValue(props, kIOSurfaceHeight, n); CFRelease(n);
	n = num(full ? '420f' : '420v'); CFDictionarySetValue(props, kIOSurfacePixelFormat, n); CFRelease(n);
	s = IOSurfaceCreate(props);
	CFRelease(props);
	if (s == NULL || IOSurfaceGetPlaneCount(s) != 2) {
		printf("video_test: no two-plane surface\n");
		return 1;
	}
	sid = IOSurfaceGetID(s);
	printf("video_test: surface %u '420%c' %zux%zu, planes: %zu a row at %p, %zu a row at %p\n", sid, full ? 'f' : 'v',
	    IOSurfaceGetWidth(s), IOSurfaceGetHeight(s), IOSurfaceGetBytesPerRowOfPlane(s, 0), IOSurfaceGetBaseAddressOfPlane(s, 0),
	    IOSurfaceGetBytesPerRowOfPlane(s, 1), IOSurfaceGetBaseAddressOfPlane(s, 1));
	draw(s, 0, full);
	if (sysctlbyname("kern.hakios.fb_video_test", NULL, NULL, &sid, sizeof(sid)) != 0) {
		perror("video_test: kern.hakios.fb_video_test");
		return 1;
	}
	for (int frame = 1; frame < seconds * 30; frame++) {
		usleep(33333);
		draw(s, frame, full);
	}
	sysctlbyname("kern.hakios.fb_video_test", NULL, NULL, &off, sizeof(off));
	printf("video_test: %d frames drawn in %d seconds\n", seconds * 30, seconds);
	CFRelease(s);
	return 0;
}
