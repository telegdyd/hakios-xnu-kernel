/*
 * iris_tiles.c: Camera.app's HEIC on the phone's video encoder (Iris2, /dev/hakios-iris), inside mediaserverd.
 *
 * CMPhoto encodes a photo's 4032x3024 primary image through a VideoToolbox tile compression session:
 * 48 tiles of 512x512, each one encodeTile call on the encoder VideoToolbox picked, which is the
 * software HEVC encoder of VCPHEVC.videocodec (see register_hevc_encoder in inject_media.c). That takes
 * about two seconds of the shutter. Iris2 encodes the same grid in about 0.4 s plus its firmware start.
 *
 * The software encoder stays the encoder. Its class (the CMBaseObject vtable's VTVideoEncoderClass) is
 * copied with two entries replaced: prepareToEncodeTiles notes the session and its size, and encodeTile,
 * at the first tile of a 4032x3024 NV12 image, gives the whole image to Iris2 once, then answers each tile
 * from that stream with VTEncoderSessionEmitEncodedTile. Each tile's sample carries a format description
 * made from Iris2's own VPS/SPS/PPS. Anything else (other sizes, the 320x240 thumbnail, other formats,
 * an Iris2 failure at the image's first tile) goes to the software encoder, image by image.
 *
 * VCPHEVC's class, unslid 20F66: +0x88 prepareToEncodeTiles(encoder, session, ?, tile dimensions,
 * specification), +0x98 encodeTile(encoder, request, pixels, origin, dimensions, properties, flags), with
 * each CMVideoDimensions-like pair as one 64-bit argument (width or x low, height or y high).
 */
#include <CoreFoundation/CoreFoundation.h>
#include <CoreMedia/CoreMedia.h>
#include <CoreVideo/CoreVideo.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <mach/mach_time.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "hakios.iris.h"

#define CLASS_BYTES     0xb0    /* VCPHEVC's VTVideoEncoderClass, version 3, through completeTiles */
#define PREPARE_TILES   0x88
#define ENCODE_TILE     0x98
#define TILES_X         ((HAKIOS_IRIS_WIDTH + HAKIOS_IRIS_TILE - 1) / HAKIOS_IRIS_TILE)
#define TILES_Y         ((HAKIOS_IRIS_HEIGHT + HAKIOS_IRIS_TILE - 1) / HAKIOS_IRIS_TILE)
#define TILES           (TILES_X * TILES_Y)
#define SESSIONS        8
#define ENCODER_MALFUNCTION (-12911) /* kVTVideoEncoderMalfunctionErr */

_Static_assert(TILES + 1 == HAKIOS_IRIS_PACKETS, "Iris2 grid");

typedef OSStatus (*prepare_fn)(void *, void *, uint64_t, uint64_t, CFTypeRef);
typedef OSStatus (*encode_fn)(void *, void *, CVPixelBufferRef, uint64_t, uint64_t, CFDictionaryRef, uint32_t *);
typedef OSStatus (*emit_fn)(void *, void *, OSStatus, uint32_t, CMSampleBufferRef);
typedef int (*code_point_fn)(CFStringRef);

struct vtable {
	void *reserved;
	const void *base;
	const void *derived;
};

/* An encoder we wrapped, and the image its session is on. */
struct session {
	void *encoder, *session;
	bool eligible;
	CVPixelBufferRef image;         /* the 4032x3024 image whose tiles are being answered, retained */
	bool hardware;
	unsigned emitted;
	bool done[TILES];
};

static void (*say)(const char *, ...);
extern bool hakios_save_trace;          /* inject_media.c: cam_save_trace=1 */
static prepare_fn software_prepare;
static encode_fn software_encode;
static emit_fn emit_tile;
static code_point_fn primaries_code, transfer_code, matrix_code;
static struct vtable *wrapped_vtable;
static const void *software_vtable;
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static struct session sessions[SESSIONS];
static unsigned next_session;
static uint8_t *stream;                 /* the last image's Annex B stream, HAKIOS_IRIS_OUTPUT_MAX */
static struct hakios_iris_still still;
static uint32_t offsets[HAKIOS_IRIS_PACKETS];
static CMVideoFormatDescriptionRef format;
static uint8_t format_header[256];
static size_t format_header_bytes;
static uint32_t format_color;

static struct session *
find(void *encoder)
{
	for (unsigned i = 0; i < SESSIONS; i++) {
		if (sessions[i].encoder == encoder) {
			return &sessions[i];
		}
	}
	return NULL;
}

static OSStatus
wrapped_prepare(void *encoder, void *session, uint64_t codec, uint64_t dimensions, CFTypeRef specification)
{
	OSStatus status = software_prepare(encoder, session, codec, dimensions, specification);
	uint32_t width = (uint32_t)dimensions, height = (uint32_t)(dimensions >> 32);

	pthread_mutex_lock(&lock);
	struct session *s = find(encoder);
	if (s == NULL) {
		s = &sessions[next_session++ % SESSIONS];
	}
	if (s->image != NULL) {
		CFRelease(s->image);
	}
	memset(s, 0, sizeof(*s));
	s->encoder = encoder;
	s->session = session;
	/* The session's dimensions are its tiles'. The image's size is checked at its first tile. */
	s->eligible = status == noErr && width == HAKIOS_IRIS_TILE && height == HAKIOS_IRIS_TILE;
	pthread_mutex_unlock(&lock);
	say("hakios: iris2: tile session of %ux%u tiles: %s (%d)\n", width, height,
	    s->eligible ? "Iris2 when the image allows" : "software", (int)status);
	(void)codec;
	return status;
}

static int
attachment_code(CVPixelBufferRef pixels, CFStringRef key, code_point_fn convert, int otherwise)
{
	CFTypeRef value = CVBufferGetAttachment(pixels, key, NULL);
	int code = value != NULL && CFGetTypeID(value) == CFStringGetTypeID() && convert != NULL ? convert(value) : 0;
	return code > 0 && code < 256 ? code : otherwise;
}

/* Split Annex B into NAL units: start and length of each, at most max. */
static unsigned
nal_units(const uint8_t *data, size_t bytes, const uint8_t **starts, size_t *lengths, unsigned max)
{
	unsigned count = 0;
	size_t i = 0;

	while (i + 3 <= bytes) {
		if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
			if (count > 0) {
				size_t end = i;
				while (end > (size_t)(starts[count - 1] - data) && data[end - 1] == 0) {
					end--;
				}
				lengths[count - 1] = end - (size_t)(starts[count - 1] - data);
			}
			if (count == max) {
				return 0;
			}
			starts[count++] = data + i + 3;
			i += 3;
		} else {
			i++;
		}
	}
	if (count > 0) {
		lengths[count - 1] = bytes - (size_t)(starts[count - 1] - data);
	}
	return count;
}

static bool
make_format(CVPixelBufferRef pixels, uint32_t color)
{
	const uint8_t *header = stream;
	size_t bytes = still.packet_bytes[0];

	if (format != NULL && bytes == format_header_bytes && color == format_color &&
	    memcmp(header, format_header, bytes) == 0) {
		return true;
	}
	const uint8_t *units[3];
	size_t lengths[3];
	if (bytes > sizeof(format_header) || nal_units(header, bytes, units, lengths, 3) != 3) {
		say("hakios: iris2: unexpected parameter sets (%zu bytes)\n", bytes);
		return false;
	}
	const void *keys[4];
	const void *values[4];
	CFIndex n = 0;
	CFStringRef names[][2] = {
		{ kCVImageBufferColorPrimariesKey, kCMFormatDescriptionExtension_ColorPrimaries },
		{ kCVImageBufferTransferFunctionKey, kCMFormatDescriptionExtension_TransferFunction },
		{ kCVImageBufferYCbCrMatrixKey, kCMFormatDescriptionExtension_YCbCrMatrix },
	};
	for (unsigned i = 0; i < 3; i++) {
		CFTypeRef value = CVBufferGetAttachment(pixels, names[i][0], NULL);
		if (value != NULL) {
			keys[n] = names[i][1];
			values[n++] = value;
		}
	}
	keys[n] = kCMFormatDescriptionExtension_FullRangeVideo;
	values[n++] = (color >> 25) & 1 ? kCFBooleanTrue : kCFBooleanFalse;
	CFDictionaryRef extensions = CFDictionaryCreate(NULL, keys, values, n, &kCFTypeDictionaryKeyCallBacks,
	    &kCFTypeDictionaryValueCallBacks);
	CMVideoFormatDescriptionRef made = NULL;
	OSStatus status = CMVideoFormatDescriptionCreateFromHEVCParameterSets(NULL, 3, units, lengths, 4, extensions, &made);
	CFRelease(extensions);
	if (status != noErr) {
		say("hakios: iris2: no format description from its parameter sets (%d)\n", (int)status);
		return false;
	}
	if (format != NULL) {
		CFRelease(format);
	}
	format = made;
	memcpy(format_header, header, bytes);
	format_header_bytes = bytes;
	format_color = color;
	return true;
}

/* The whole image to Iris2. Called with the lock held. */
static bool
encode_image(CVPixelBufferRef pixels)
{
	static int device = -1;
	OSType type = CVPixelBufferGetPixelFormatType(pixels);
	bool full = type == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange;

	if ((!full && type != kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange) ||
	    CVPixelBufferGetPlaneCount(pixels) != 2 ||
	    CVPixelBufferGetWidth(pixels) != HAKIOS_IRIS_WIDTH || CVPixelBufferGetHeight(pixels) != HAKIOS_IRIS_HEIGHT) {
		say("hakios: iris2: software for a %zux%zu '%c%c%c%c' image\n", CVPixelBufferGetWidth(pixels),
		    CVPixelBufferGetHeight(pixels), (char)(type >> 24), (char)(type >> 16), (char)(type >> 8), (char)type);
		return false;
	}
	if (stream == NULL && (stream = malloc(HAKIOS_IRIS_OUTPUT_MAX)) == NULL) {
		return false;
	}
	if (device < 0 && (device = open(HAKIOS_IRIS_DEVICE, O_RDWR)) < 0) {
		say("hakios: iris2: software, no %s (%d)\n", HAKIOS_IRIS_DEVICE, errno);
		return false;
	}
	/* Iris2 signals the same colour as the HEIF's colr box: full range or not, and the buffer's code points. */
	uint32_t color = (uint32_t)attachment_code(pixels, kCVImageBufferYCbCrMatrixKey, matrix_code, 2) |
	    (uint32_t)attachment_code(pixels, kCVImageBufferTransferFunctionKey, transfer_code, 2) << 8 |
	    (uint32_t)attachment_code(pixels, kCVImageBufferColorPrimariesKey, primaries_code, 2) << 16 |
	    1u << 24 | (full ? 1u : 0u) << 25 | 5u << 26 | 1u << 29;
	if (CVPixelBufferLockBaseAddress(pixels, kCVPixelBufferLock_ReadOnly) != kCVReturnSuccess) {
		return false;
	}
	memset(&still, 0, sizeof(still));
	still.input = (uintptr_t)CVPixelBufferGetBaseAddressOfPlane(pixels, 0);
	still.input_uv = (uintptr_t)CVPixelBufferGetBaseAddressOfPlane(pixels, 1);
	still.stride = (uint32_t)CVPixelBufferGetBytesPerRowOfPlane(pixels, 0);
	still.stride_uv = (uint32_t)CVPixelBufferGetBytesPerRowOfPlane(pixels, 1);
	still.output = (uintptr_t)stream;
	still.capacity = HAKIOS_IRIS_OUTPUT_MAX;
	still.color_info = color;
	uint64_t started = mach_absolute_time();
	int result = ioctl(device, HAKIOS_IRIS_ENCODE_STILL, &still);
	int error = errno;
	uint64_t ended = mach_absolute_time();
	CVPixelBufferUnlockBaseAddress(pixels, kCVPixelBufferLock_ReadOnly);
	mach_timebase_info_data_t base;
	mach_timebase_info(&base);
	unsigned ms = (unsigned)((ended - started) * base.numer / base.denom / 1000000);
	if (result != 0) {
		say("hakios: iris2: software, the encode failed (%d) after %u ms\n", error, ms);
		return false;
	}
	uint32_t total = 0;
	for (unsigned i = 0; i < HAKIOS_IRIS_PACKETS; i++) {
		if (still.packet_bytes[i] == 0 || still.packet_bytes[i] > still.bytes - total) {
			say("hakios: iris2: software, packet %u is invalid\n", i);
			return false;
		}
		offsets[i] = total;
		total += still.packet_bytes[i];
	}
	if (total != still.bytes || !make_format(pixels, color)) {
		return false;
	}
	say("hakios: iris2: %u-byte stream in %u ms (%llu us on Iris2), colour %#x, strides %u/%u\n", total, ms,
	    (unsigned long long)still.elapsed_us, color, still.stride, still.stride_uv);
	return true;
}

/* One tile of the stream as a sample with four-byte NAL lengths. */
static CMSampleBufferRef
tile_sample(unsigned tile)
{
	const uint8_t *data = stream + offsets[tile + 1];
	size_t bytes = still.packet_bytes[tile + 1];
	const uint8_t *units[16];
	size_t lengths[16];
	unsigned count = nal_units(data, bytes, units, lengths, 16);
	size_t size = 0;

	for (unsigned i = 0; i < count; i++) {
		size += 4 + lengths[i];
	}
	CMBlockBufferRef block = NULL;
	CMSampleBufferRef sample = NULL;
	if (count == 0 || CMBlockBufferCreateWithMemoryBlock(NULL, NULL, size, NULL, NULL, 0, size,
	    kCMBlockBufferAssureMemoryNowFlag, &block) != kCMBlockBufferNoErr) {
		return NULL;
	}
	size_t at = 0;
	for (unsigned i = 0; i < count; i++) {
		uint8_t prefix[4] = { (uint8_t)(lengths[i] >> 24), (uint8_t)(lengths[i] >> 16), (uint8_t)(lengths[i] >> 8),
			(uint8_t)lengths[i] };
		CMBlockBufferReplaceDataBytes(prefix, block, at, 4);
		CMBlockBufferReplaceDataBytes(units[i], block, at + 4, lengths[i]);
		at += 4 + lengths[i];
	}
	OSStatus status = CMSampleBufferCreateReady(NULL, block, format, 1, 0, NULL, 1, &size, &sample);
	CFRelease(block);
	return status == noErr ? sample : NULL;
}

/* The valid part of tile t in the image: where it starts and how much of it the image covers. */
static void
tile_area(unsigned t, unsigned *x, unsigned *y, unsigned *width, unsigned *height)
{
	*x = t % TILES_X * HAKIOS_IRIS_TILE;
	*y = t / TILES_X * HAKIOS_IRIS_TILE;
	*width = HAKIOS_IRIS_WIDTH - *x < HAKIOS_IRIS_TILE ? HAKIOS_IRIS_WIDTH - *x : HAKIOS_IRIS_TILE;
	*height = HAKIOS_IRIS_HEIGHT - *y < HAKIOS_IRIS_TILE ? HAKIOS_IRIS_HEIGHT - *y : HAKIOS_IRIS_TILE;
}

static bool
same_rows(const uint8_t *a, size_t a_stride, const uint8_t *b, size_t b_stride, unsigned width, unsigned height)
{
	const unsigned rows[] = { 0, height / 3, height / 2, height - 1 };
	for (unsigned i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
		if (memcmp(a + rows[i] * a_stride, b + rows[i] * b_stride, width) != 0) {
			return false;
		}
	}
	return true;
}

/*
 * CMPhoto gives an edge tile, which the image covers only in part, as its own padded tile-sized buffer.
 * Which tile it is: the outstanding one whose part of the image it holds. Called with the lock held.
 */
static int
find_padded_tile(struct session *s, CVPixelBufferRef tile)
{
	int found = -1;

	if (CVPixelBufferGetPixelFormatType(tile) != CVPixelBufferGetPixelFormatType(s->image) ||
	    CVPixelBufferGetPlaneCount(tile) != 2 ||
	    CVPixelBufferLockBaseAddress(tile, kCVPixelBufferLock_ReadOnly) != kCVReturnSuccess) {
		return -1;
	}
	if (CVPixelBufferLockBaseAddress(s->image, kCVPixelBufferLock_ReadOnly) == kCVReturnSuccess) {
		const uint8_t *image[2], *small[2];
		size_t image_stride[2], small_stride[2];
		for (unsigned p = 0; p < 2; p++) {
			image[p] = CVPixelBufferGetBaseAddressOfPlane(s->image, p);
			image_stride[p] = CVPixelBufferGetBytesPerRowOfPlane(s->image, p);
			small[p] = CVPixelBufferGetBaseAddressOfPlane(tile, p);
			small_stride[p] = CVPixelBufferGetBytesPerRowOfPlane(tile, p);
		}
		for (unsigned t = 0; t < TILES && found < 0; t++) {
			unsigned x, y, width, height;
			tile_area(t, &x, &y, &width, &height);
			if (s->done[t] || (width == HAKIOS_IRIS_TILE && height == HAKIOS_IRIS_TILE)) {
				continue;
			}
			if (same_rows(image[0] + y * image_stride[0] + x, image_stride[0], small[0], small_stride[0], width, height) &&
			    same_rows(image[1] + y / 2 * image_stride[1] + x, image_stride[1], small[1], small_stride[1], width,
			    height / 2)) {
				found = (int)t;
			}
		}
		CVPixelBufferUnlockBaseAddress(s->image, kCVPixelBufferLock_ReadOnly);
	}
	CVPixelBufferUnlockBaseAddress(tile, kCVPixelBufferLock_ReadOnly);
	return found;
}

static void
end_image(struct session *s)
{
	if (s->image != NULL) {
		CFRelease(s->image);
	}
	s->image = NULL;
	s->hardware = false;
	s->emitted = 0;
	memset(s->done, 0, sizeof(s->done));
}

static OSStatus
wrapped_encode(void *encoder, void *request, CVPixelBufferRef pixels, uint64_t origin, uint64_t dimensions,
    CFDictionaryRef properties, uint32_t *flags)
{
	uint32_t x = (uint32_t)origin, y = (uint32_t)(origin >> 32);
	uint32_t width = (uint32_t)dimensions, height = (uint32_t)(dimensions >> 32);
	int tile = -1;

	pthread_mutex_lock(&lock);
	struct session *s = find(encoder);
	if (s == NULL || !s->eligible || pixels == NULL) {
		pthread_mutex_unlock(&lock);
		if (!hakios_save_trace) {
			return software_encode(encoder, request, pixels, origin, dimensions, properties, flags);
		}
		/* cam_save_trace=1: how long software takes over what is left to it (the thumbnail) */
		uint64_t started = mach_absolute_time();
		OSStatus status = software_encode(encoder, request, pixels, origin, dimensions, properties, flags);
		mach_timebase_info_data_t base;
		mach_timebase_info(&base);
		say("hakios: iris2: software tile at %u,%u size %ux%u of a %zux%zu buffer in %llu us (%d)\n", x, y, width, height,
		    pixels != NULL ? CVPixelBufferGetWidth(pixels) : 0, pixels != NULL ? CVPixelBufferGetHeight(pixels) : 0,
		    (unsigned long long)((mach_absolute_time() - started) * base.numer / base.denom / 1000), (int)status);
		return status;
	}
	bool whole = CVPixelBufferGetWidth(pixels) == HAKIOS_IRIS_WIDTH && CVPixelBufferGetHeight(pixels) == HAKIOS_IRIS_HEIGHT;
	if (whole && pixels != s->image) {
		end_image(s);
		s->image = (CVPixelBufferRef)CFRetain(pixels);
		s->hardware = encode_image(pixels);
	}
	if (s->hardware && pixels == s->image) {
		/* A tile inside the image. */
		unsigned t = y / HAKIOS_IRIS_TILE * TILES_X + x / HAKIOS_IRIS_TILE;
		if (x % HAKIOS_IRIS_TILE == 0 && y % HAKIOS_IRIS_TILE == 0 && x < HAKIOS_IRIS_WIDTH && y < HAKIOS_IRIS_HEIGHT &&
		    width == HAKIOS_IRIS_TILE && height == HAKIOS_IRIS_TILE && !s->done[t]) {
			tile = (int)t;
		}
	} else if (s->hardware && x == 0 && y == 0 && width == HAKIOS_IRIS_TILE && height == HAKIOS_IRIS_TILE &&
	    CVPixelBufferGetWidth(pixels) == HAKIOS_IRIS_TILE && CVPixelBufferGetHeight(pixels) == HAKIOS_IRIS_TILE) {
		tile = find_padded_tile(s, pixels);
	}
	if (tile < 0) {
		if (s->hardware) {
			say("hakios: iris2: software for a tile at %u,%u size %ux%u of a %zux%zu buffer\n", x, y, width, height,
			    CVPixelBufferGetWidth(pixels), CVPixelBufferGetHeight(pixels));
		}
		pthread_mutex_unlock(&lock);
		return software_encode(encoder, request, pixels, origin, dimensions, properties, flags);
	}
	CMSampleBufferRef sample = tile_sample((unsigned)tile);
	s->done[tile] = true;
	if (++s->emitted == TILES) {
		say("hakios: iris2: all %u tiles of image %p from Iris2\n", TILES, (void *)s->image);
		end_image(s);
	}
	void *session = s->session;
	pthread_mutex_unlock(&lock);
	if (sample == NULL) {
		say("hakios: iris2: no sample for tile %d\n", tile);
		emit_tile(session, request, ENCODER_MALFUNCTION, 0, NULL);
		return noErr;
	}
	if (flags != NULL) {
		*flags = 0;
	}
	OSStatus status = emit_tile(session, request, noErr, 0, sample);
	CFRelease(sample);
	return status;
}

/*
 * Wrap a software encoder from VCPHEVC's factory before VideoToolbox sees it. Anything unexpected
 * leaves the encoder as it is.
 */
void
hakios_iris_wrap_encoder(void *encoder, void (*log)(const char *, ...))
{
	struct vtable **slot = (struct vtable **)((uint8_t *)encoder + 0x18);
	static bool ready, failed;

	pthread_mutex_lock(&lock);
	if (!ready && !failed) {
		say = log;
		failed = true;
		void *vt = dlopen("/System/Library/Frameworks/VideoToolbox.framework/VideoToolbox", RTLD_NOW | RTLD_GLOBAL);
		void *cv = dlopen("/System/Library/Frameworks/CoreVideo.framework/CoreVideo", RTLD_NOW | RTLD_GLOBAL);
		emit_tile = vt != NULL ? (emit_fn)dlsym(vt, "VTEncoderSessionEmitEncodedTile") : NULL;
		primaries_code = cv != NULL ? (code_point_fn)dlsym(cv, "CVColorPrimariesGetIntegerCodePointForString") : NULL;
		transfer_code = cv != NULL ? (code_point_fn)dlsym(cv, "CVTransferFunctionGetIntegerCodePointForString") : NULL;
		matrix_code = cv != NULL ? (code_point_fn)dlsym(cv, "CVYCbCrMatrixGetIntegerCodePointForString") : NULL;
		const struct vtable *original = *slot;
		const uint64_t *derived = original != NULL ? original->derived : NULL;
		if (emit_tile != NULL && derived != NULL && derived[0] == 3 && derived[PREPARE_TILES / 8] != 0 &&
		    derived[ENCODE_TILE / 8] != 0 && access(HAKIOS_IRIS_DEVICE, F_OK) == 0) {
			uint64_t *copy = malloc(CLASS_BYTES);
			wrapped_vtable = malloc(sizeof(*wrapped_vtable));
			if (copy != NULL && wrapped_vtable != NULL) {
				memcpy(copy, derived, CLASS_BYTES);
				software_prepare = (prepare_fn)derived[PREPARE_TILES / 8];
				software_encode = (encode_fn)derived[ENCODE_TILE / 8];
				copy[PREPARE_TILES / 8] = (uint64_t)(uintptr_t)wrapped_prepare;
				copy[ENCODE_TILE / 8] = (uint64_t)(uintptr_t)wrapped_encode;
				*wrapped_vtable = *original;
				wrapped_vtable->derived = copy;
				software_vtable = original;
				ready = true;
				failed = false;
			}
		}
		say("hakios: iris2: HEIC tiles %s\n", ready ? "on Iris2, software as the fallback" :
		    "software only (no encoder class, emit function or /dev/hakios-iris)");
	}
	if (ready && *slot == software_vtable) {
		*slot = wrapped_vtable;
	}
	pthread_mutex_unlock(&lock);
}
