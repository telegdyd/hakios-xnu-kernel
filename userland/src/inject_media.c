/*
 * libhakios.media.dylib: code of ours inside mediaserverd (see inject.c for how it gets there).
 *
 * mediaserverd is the audio server, and also the camera's: the capture stack of CMCapture lives
 * in the same process. Soon after it starts, that stack "prewarms" the Core Image filters of the
 * portrait mode, for which Core Image wants an OpenGL ES context; there is no GPU, the context
 * has no functions behind it, and the daemon jumps to address 0 -- every time launchd restarts
 * it. Nobody is going to take a portrait here: the prewarming is skipped.
 *
 * What the daemon is wanted for meanwhile is neither sound nor the camera: it is where
 * VideoToolbox decodes for everybody else (the service com.apple.coremedia.decompressionsession),
 * and a HEIC picture is HEVC inside. Without the daemon ImageIO has "no way to decode a HEIC",
 * and the stock wallpaper of Darwin userland 16, whose layers are four such pictures, is its gradient alone.
 * There is no hardware decoder here; VideoToolbox takes its software one (VCPHEVC.videocodec).
 *
 * The flashlight (only where the kernel has one: torch=3, /dev/hakios-torch): Control Center's
 * button and AVFlashlight reach FigFlashlight in here, which asks the capture stack's device vendor
 * (+[BWFigCaptureDeviceVendor sharedCaptureDeviceVendor]) for the camera device and its back
 * camera's stream, and sets the stream's "TorchLevel" (0 to 1). Without a camera there is no
 * vendor (it is nil), every message to it does nothing, and FigFlashlight says it has set the level
 * when nothing has been. So where there is no vendor, FigFlashlight -- and only FigFlashlight,
 * by where it calls from -- is given one of ours, with a device and a stream whose torch level goes
 * to the kernel (see flashlight_*() below). Everybody else still gets none, as before.
 *
 * The camera (only where the kernel has one: cam=5, /dev/hakios-camera): the vendor is made for
 * everybody, with the create function of our capture device (camera_device.c) in place of the
 * one H10ISP.mediacapture would give for the iPhone's ISP; the capture graph is built around it and
 * its frames reach Camera.app's preview (see camera_device.c). Its stream takes the torch level too,
 * so FigFlashlight would need nothing of its own then (not tried yet).
 *
 * What the capture stack asks of hardware this phone has none of is refused here: there is no ISP
 * processing session, so there can be no video stabilization (see video_stabilization_method below).
 *
 * CMCapture is not loaded yet when this library is: the methods are replaced when its image
 * arrives.
 */
#include <CoreFoundation/CoreFoundation.h>
#include <CoreMedia/CoreMedia.h>
#include <CoreVideo/CoreVideo.h>
#include <dlfcn.h>
#include <execinfo.h>
#include <fcntl.h>
#include <mach-o/dyld.h>
#include <mach/mach_time.h>
#include <objc/message.h>
#include <objc/runtime.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/sysctl.h>
#include <sys/ucontext.h>
#include <unistd.h>

/*
 * cam_save_trace=1: every line said here and by camera_device.c (hakios_camera_say) starts with the time since
 * boot, "[t=1234.567] ", on mach_absolute_time's clock, which savetrace (src/savetrace.c) and the other programs
 * share; and each step of a photograph through the capture graph is said (trace_photo_step below). What a
 * photograph's two seconds between the shutter and DCIM were spent on.
 */
bool hakios_save_trace;

int
hakios_trace_stamp(char *line, size_t size)
{
	static mach_timebase_info_data_t base;

	if (!hakios_save_trace) {
		return 0;
	}
	if (base.denom == 0) {
		mach_timebase_info(&base);
	}
	uint64_t us = mach_absolute_time() * base.numer / base.denom / 1000;
	return snprintf(line, size, "[t=%llu.%03llu] ", (unsigned long long)(us / 1000), (unsigned long long)(us % 1000));
}

static void
say(const char *format, ...)
{
	char line[320];
	va_list ap;
	int n = hakios_trace_stamp(line, sizeof(line));

	va_start(ap, format);
	n += vsnprintf(line + n, sizeof(line) - (size_t)n, format, ap);
	va_end(ap);
	if (n > (int)sizeof(line) - 1) {
		n = sizeof(line) - 1;
	}
	write(2, line, (size_t)n);
}

extern const void *_dyld_get_shared_cache_range(size_t *length);

#define CACHE_BASE 0x180000000UL                /* where the cache was built to be, before the slide */

static uintptr_t slide;

/* +[BWCoreImageFilterRenderer prewarmPortraitFilterVersion:semanticStyleFilters:] */
static void
prewarm_portrait_filter(__unused id self, __unused SEL selector, __unused int version, __unused BOOL styles)
{
}

/* ---- the flashlight ------------------------------------------------------------------------------- */

static int torch = -1;                  /* /dev/hakios-torch */
static bool camera;                     /* /dev/hakios-camera is there */
extern int hakios_camera_device_create(const void *allocator, const void *options, void **out);
static Class vendor_class, device_class, stream_class;
static IMP original_shared_vendor;
static unsigned torch_said;

/*
 * FigFlashlight's functions that ask for the vendor, unslid (20F66): flashlight_IsAvailable,
 * _NotifyForCurrentState and _setupDevice lie together, _teardownDevice apart.
 */
static const struct { uintptr_t from, to; } flashlight_code[] = {
	{ 0x1a1fb0830UL, 0x1a1fb0c5cUL }, { 0x1a1eee5c0UL, 0x1a1eee6e8UL },
};

/* +[BWFigCaptureDeviceVendor sharedCaptureDeviceVendor] */
static id
shared_vendor(id self, SEL selector)
{
	uintptr_t caller = (uintptr_t)__builtin_return_address(0) - slide;
	id vendor = ((id (*)(id, SEL))original_shared_vendor)(self, selector);
	static id ours, camera_vendor;
	static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

	if (vendor != NULL) {
		return vendor;
	}
	if (!camera && access("/dev/hakios-camera", F_OK) == 0) {
		camera = true;          /* (should the kernel have made it after mediaserverd started) */
		say("hakios: mediaserverd: the kernel has a camera (/dev/hakios-camera)\n");
	}
	if (camera) {
		pthread_mutex_lock(&lock);
		if (camera_vendor == NULL) {
			id made = ((id (*)(Class, SEL))objc_msgSend)(objc_getClass("BWFigCaptureDeviceVendor"), sel_registerName("alloc"));

			camera_vendor = ((id (*)(id, SEL, void *))objc_msgSend)(made, sel_registerName("initWithDeviceCreateFunction:"),
			    (void *)hakios_camera_device_create);
			say("hakios: mediaserverd: the camera's device vendor %s\n", camera_vendor ? "made (camera_device.c)" :
			    "CANNOT BE MADE");
		}
		pthread_mutex_unlock(&lock);
		return camera_vendor;
	}
	for (size_t i = 0; torch >= 0 && i < sizeof(flashlight_code) / sizeof(flashlight_code[0]); i++) {
		if (caller >= flashlight_code[i].from && caller < flashlight_code[i].to) {
			if (ours == NULL) {
				ours = class_createInstance(vendor_class, 0);
				say("hakios: mediaserverd: no camera, so no device vendor: FigFlashlight gets the flashlight's own\n");
			}
			return ours;
		}
	}
	return NULL;
}

static void
flashlight_set(uint32_t level)
{
	ssize_t written = write(torch, &level, sizeof(level));

	if (written != (ssize_t)sizeof(level) || torch_said < 40) {
		torch_said++;
		say("hakios: mediaserverd: the flashlight at %u of 1000%s\n", level, written == (ssize_t)sizeof(level) ? "" :
		    " (NOT TAKEN by /dev/hakios-torch)");
	}
}

/* the vendor's: one client, one device, always available to it */
static int
flashlight_register(__unused id self, __unused SEL selector, __unused int pid, __unused id description, __unused int priority,
    __unused BOOL steal, __unused BOOL share, __unused id handler)
{
	return 1;
}

static id
flashlight_copy_device(__unused id self, __unused SEL selector, __unused int client, __unused BOOL inform, int *error)
{
	if (error != NULL) {
		*error = 0;
	}
	return class_createInstance(device_class, 0);
}

static id
flashlight_copy_stream(__unused id self, __unused SEL selector, __unused int position, __unused int type, id device)
{
	return device != NULL && object_getClass(device) == device_class ? class_createInstance(stream_class, 0) : NULL;
}

static void
flashlight_take_back(__unused id self, __unused SEL selector, __unused id device, __unused int client, __unused BOOL inform)
{
	flashlight_set(0);
}

static BOOL
flashlight_active_equals(__unused id self, __unused SEL selector, id device)
{
	return device == NULL || object_getClass(device) == device_class;
}

/* the stream's -supportedProperties: none that FigFlashlight looks for ("HighCurrentTorchEnabled") */
static id
flashlight_stream_properties(__unused id self, __unused SEL selector)
{
	static CFDictionaryRef none;

	if (none == NULL) {
		none = CFDictionaryCreate(NULL, NULL, NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
	}
	return (id)none;
}

/* the stream's -setProperty:value: -- "TorchLevel", 0 to 1, as thousandths to the kernel */
static int
flashlight_stream_set(__unused id self, __unused SEL selector, id key, id value)
{
	float level = 0;

	if (key != NULL && CFEqual((CFTypeRef)key, CFSTR("TorchLevel"))) {
		if (value != NULL && CFGetTypeID((CFTypeRef)value) == CFNumberGetTypeID()) {
			CFNumberGetValue((CFNumberRef)value, kCFNumberFloatType, &level);
		}
		level = level < 0 ? 0 : level > 1 ? 1 : level;
		flashlight_set((uint32_t)(level * 1000 + 0.5f));
	}
	return 0;
}

/*
 * Video stabilization, of which this hardware has none: the capture graph's VIS node
 * (-[FigCaptureVISPipeline _newVISNodeWithUpstreamOutput:...]) is made with the ISP's processing
 * session, and this device has no such session, so the node comes back nil, the VIS pipeline with
 * it, the movie file sink's tail pipeline with that, and captureSession_buildGraphWithConfiguration
 * tears the whole graph down -- the preview too, for all that the preview has nothing to do with
 * stabilizing a recording. The connection's stabilization method is where the whole stack reads the
 * wish from (+[FigVideoCaptureConnectionConfiguration videoStabilizationMethods:includeIris:] ->
 * cspc_getVideoStabilizationType -> the connections' FigCaptureVideoStabilizationType), and 0 there
 * means none: the graph is then built without a VIS pipeline anywhere
 * (+[FigCaptureVISPipeline visPipelineIsRequiredForFigCaptureVideoStabilizationType:] says no to
 * every type but 2 and 3), and recordings are as steady as the hand that holds the phone.
 */
static int
video_stabilization_method(id self, SEL selector)
{
	(void)self;
	(void)selector;
	return 0;
}

/*
 * The type the stack acts on. The connection configuration arrives from the application's
 * AVFoundation with the type in it, which is read straight out of the object and not computed from
 * the method above (-[FigCaptureSessionParsedConfiguration _computeVideoStabilizationTypes] only
 * fills in what the client left out), so the wish has to be refused here as well.
 */
static int
video_stabilization_type(id self, SEL selector)
{
	(void)self;
	(void)selector;
	return 0;                       /* kFigCaptureVideoStabilizationType_None */
}

/*
 * The microphone, which the capture stack has none of here. A capture session's sound comes through
 * CoreMedia's own capture audio session -- BWAudioSourceNode's start sets its properties and reads
 * its pull format, all of it through a brokered plug-in of the audio HAL -- and that plug-in is not
 * on this phone: the node's start comes back with 'nope', the HAL's word for "no such thing".
 *
 * Camera.app's session has an audio input in it, because it is a session for photographs and for
 * video both. The graph starts its source nodes late (startDeferredSourceNodesIfNeeded) and takes
 * the first error it gets for the whole graph's, so the session reported AVCaptureSessionRuntimeError
 * to the application, which took the session down and built it again, two seconds later, for ever:
 * the camera's preview came in bursts of five frames with the sensor powering down in between.
 *
 * The node is told that it started. It then produces no sound -- which is all a phone with no
 * capture audio has to offer -- and the rest of the graph, the preview and the photograph, runs.
 * Only that one error is swallowed: a microphone that is there and fails for another reason still
 * takes the session down, as it should. (The kernel's own microphone, adsp=1 mic=4, is a different
 * path: AVAudioRecorder and Voice Memos record through it, and nothing here touches it.)
 */
#define ERROR_NO_SUCH_THING     1852797029      /* 'nope' */

static IMP original_audio_source_start;

static BOOL
audio_source_start(id self, SEL selector, id *error)
{
	BOOL started = ((BOOL (*)(id, SEL, id *))original_audio_source_start)(self, selector, error);
	id made = error != NULL ? *error : NULL;
	static bool said;

	if (started || made == NULL || ((long (*)(id, SEL))objc_msgSend)(made, sel_registerName("code")) != ERROR_NO_SUCH_THING) {
		return started;
	}
	*error = NULL;                  /* (autoreleased by the capture stack: not ours to give back) */
	if (!said) {
		said = true;
		say("hakios: mediaserverd: there is no capture audio on this phone: the session's audio source gives nothing, "
		    "and the camera runs without it\n");
	}
	return true;
}

/*
 * A photograph's quality prioritization, "speed" at the most. Asked for "balanced" (Camera.app's) or
 * "quality", the capture device takes a scene for a low-light one whenever a frame was exposed at
 * least as long as the sensor's still image stabilization threshold
 * (-[BWFigVideoCaptureDevice _isLowLightSceneUsingFrameStatisticsByPortType:]) -- a threshold from
 * the sensor's tuning in CameraSetup.plist, 0 for a sensor it has no tuning for, as ours -- and
 * then asks the stream for a bracket of four frames for SIS fusion (captureType 4, BWSISNode), which
 * this port has nothing to fuse with: the photograph never came. With "speed" it asks for one frame
 * (-_resolveStillImageCaptureTypeFromStatisticsByPortType:..., captureType 1), which the stream
 * gives (camera_device.c, StillImageCaptureNow).
 */
static IMP original_quality_prioritization;

static int
quality_prioritization(id self, SEL selector)
{
	int asked = ((int (*)(id, SEL))original_quality_prioritization)(self, selector);
	static bool said;

	if (asked > 1 && !said) {
		said = true;
		say("hakios: mediaserverd: a photograph's quality prioritization %d taken as 1 (speed): one frame, no SIS bracket\n",
		    asked);
	}
	return asked > 1 ? 1 : asked;
}

/*
 * A photograph's noise reduction (BWGNRNode, between the still image coordinator and the photo
 * encoder) is a FigSampleBufferProcessor of the iPhone's, which is handed the picture and never
 * hands one back here: the photograph stopped there, with no error, and the request with it (the
 * shutter's second press asked for nothing more). The picture goes past it to the node's output as
 * it came, as with no noise reduction to do.
 */
static void
gnr_render(id self, SEL selector, void *sample_buffer, id input)
{
	id output = ((id (*)(id, SEL))objc_msgSend)(self, sel_registerName("output"));
	static bool said;

	(void)selector, (void)input;
	if (!said) {
		said = true;
		say("hakios: mediaserverd: a photograph goes past the noise reduction, which has nothing to do it with here\n");
	}
	if (output != nil) {
		((void (*)(id, SEL, void *))objc_msgSend)(output, sel_registerName("emitSampleBuffer:"), sample_buffer);
	}
}

/*
 * The filter picker's live thumbnails (BWMultiFilterThumbnailNode): its colour cube filter is a
 * FigColorCubeMetalFilter, which is not made here (nil), and the node writes into it anyway --
 * mediaserverd died of SIGSEGV at 0x8 (CMCapture+0x61370) a moment after Camera.app's first preview
 * picture, every time, and the preview "restarted" with each new mediaserverd. Without the filter
 * the node makes no thumbnails: its frames are dropped, and the method that sets the filter up
 * (v16@0:8 at CMCapture+0x61248, its name not in this cache) does nothing.
 */
static void
thumbnail_render(__unused id self, __unused SEL selector, __unused void *sample_buffer, __unused id input)
{
	static bool said;

	if (!said) {
		said = true;
		say("hakios: mediaserverd: the filter thumbnails are not made (no colour cube filter here)\n");
	}
}

static void
thumbnail_nothing(__unused id self, SEL selector)
{
	static bool said;

	if (!said) {
		said = true;
		say("hakios: mediaserverd: -[BWMultiFilterThumbnailNode %s] does nothing (no colour cube filter here)\n",
		    sel_getName(selector));
	}
}

static bool
is_node(id node, const char *name)
{
	return node != nil && strcmp(object_getClassName(node), name) == 0;
}

/*
 * How long a photograph's encoding took, from the picture handed to the photo encoder node to the one
 * it hands on (HEVC in software, see register_hevc_encoder below: the slowest step of a photograph).
 */
static void
time_photo_encoder(id output, CMSampleBufferRef sample_buffer)
{
	CVImageBufferRef image = CMSampleBufferGetImageBuffer(sample_buffer);
	static uint64_t began;
	static mach_timebase_info_data_t timebase;
	id connection, input;

	if (image == NULL || CVPixelBufferGetWidth(image) < 2000) {
		return;                 /* not a photograph */
	}
	if (is_node(((id (*)(id, SEL))objc_msgSend)(output, sel_registerName("node")), "BWPhotoEncoderNode")) {
		if (began != 0) {
			if (timebase.denom == 0) {
				mach_timebase_info(&timebase);
			}
			say("hakios: mediaserverd: a photograph of %zux%zu encoded in %.0f ms\n", CVPixelBufferGetWidth(image),
			    CVPixelBufferGetHeight(image), (double)(mach_absolute_time() - began) * timebase.numer / timebase.denom / 1e6);
		}
		began = 0;
		return;
	}
	connection = ((id (*)(id, SEL))objc_msgSend)(output, sel_registerName("connection"));
	input = connection != nil ? ((id (*)(id, SEL))objc_msgSend)(connection, sel_registerName("input")) : nil;
	if (input != nil && is_node(((id (*)(id, SEL))objc_msgSend)(input, sel_registerName("node")), "BWPhotoEncoderNode")) {
		began = mach_absolute_time();
	}
}

/*
 * cam_save_trace=1: a photograph's steps. Every sample buffer a node emits that holds a picture 2000 or more wide,
 * and for four seconds after one, every one that holds no picture at all (an encoded photograph, its metadata),
 * by the node that emitted it and the node it goes to. The preview's buffers are left out: they hold pictures, and
 * small ones.
 */
static void
trace_photo_step(id output, CMSampleBufferRef sample_buffer)
{
	static uint64_t last_photo;
	static unsigned said;
	static mach_timebase_info_data_t base;
	CVImageBufferRef image = CMSampleBufferGetImageBuffer(sample_buffer);
	uint64_t now = mach_absolute_time();
	size_t width = image != NULL ? CVPixelBufferGetWidth(image) : 0, height = image != NULL ? CVPixelBufferGetHeight(image) : 0;
	id node, connection, input, to;

	if (base.denom == 0) {
		mach_timebase_info(&base);
	}
	if (image != NULL && width < 2000) {
		return;
	}
	if (image == NULL && (last_photo == 0 || (now - last_photo) * base.numer / base.denom > 4000000000ull)) {
		return;
	}
	node = ((id (*)(id, SEL))objc_msgSend)(output, sel_registerName("node"));
	if (node != nil && strncmp(object_getClassName(node), "BWAudio", 7) == 0) {
		return;
	}
	if (image != NULL) {
		if (last_photo == 0 || (now - last_photo) * base.numer / base.denom > 4000000000ull) {
			said = 0;
		}
		last_photo = now;
	}
	if (said++ >= 80) {
		return;
	}
	connection = ((id (*)(id, SEL))objc_msgSend)(output, sel_registerName("connection"));
	input = connection != nil ? ((id (*)(id, SEL))objc_msgSend)(connection, sel_registerName("input")) : nil;
	to = input != nil ? ((id (*)(id, SEL))objc_msgSend)(input, sel_registerName("node")) : nil;
	CMBlockBufferRef data = CMSampleBufferGetDataBuffer(sample_buffer);
	say("hakios: save trace: %s -> %s: %zux%zu%s, %zu bytes of data\n", node != nil ? object_getClassName(node) : "?",
	    to != nil ? object_getClassName(to) : "(nothing)", width, height, image != NULL ? "" : " (no picture)",
	    data != NULL ? CMBlockBufferGetDataLength(data) : 0);
}

/*
 * A node's output given no sample buffer at all, which no node takes: the session's audio source
 * (-[BWAudioSourceNode _generateSamples]), whose audio unit never started here (see audio_source_start
 * above), fills the silence by a timer with buffers it fails to make and emits them all the same --
 * NULL. Live Photo's staging node added that to an array (-[BWIrisStagingNode renderSampleBuffer:forInput:])
 * and mediaserverd died of the NSInvalidArgumentException, every half a minute with Live Photo on, the
 * preview black for a second each time. A NULL is not passed on.
 */
static IMP original_emit_sample_buffer;

/*
 * No Live Photo (Iris) with the kernel's camera. D22's AVCaptureSession.plist gives the 12MP format IrisSupported,
 * and the plist's word wins over the stream's own format. There is no capture audio here (see audio_source_start),
 * so Live Photo's staging node never became ready: every photograph's _captureSession_captureStillImageNow waited out
 * -[BWIrisStagingNode waitUntilReadyToReceiveRequestsWithTimeout:], a second, and no movie was ever written. The
 * staging node also took every preview frame. The format's dictionary says no instead, before the format keeps it:
 * the applications' copies come from it (by XPC), so Camera offers no Live Photo and asks for none.
 */
static IMP original_format_init, original_format_init_override;

static CFDictionaryRef
without_iris(CFDictionaryRef dictionary)
{
	if (!camera || dictionary == NULL || CFGetTypeID(dictionary) != CFDictionaryGetTypeID() ||
	    CFDictionaryGetValue(dictionary, CFSTR("IrisSupported")) != kCFBooleanTrue) {
		return dictionary != NULL ? CFRetain(dictionary) : NULL;
	}
	CFMutableDictionaryRef copy = CFDictionaryCreateMutableCopy(NULL, 0, dictionary);
	CFDictionarySetValue(copy, CFSTR("IrisSupported"), kCFBooleanFalse);
	CFDictionarySetValue(copy, CFSTR("IrisVideoStabilizationSupported"), kCFBooleanFalse);
	return copy;
}

static id
format_init(id self, SEL selector, CFDictionaryRef dictionary)
{
	CFDictionaryRef changed = without_iris(dictionary);
	id format = ((id (*)(id, SEL, CFDictionaryRef))original_format_init)(self, selector, changed);

	if (changed != NULL) {
		CFRelease(changed);
	}
	return format;
}

static id
format_init_override(id self, SEL selector, CFDictionaryRef dictionary, uint32_t pixel_format)
{
	CFDictionaryRef changed = without_iris(dictionary);
	id format = ((id (*)(id, SEL, CFDictionaryRef, uint32_t))original_format_init_override)(self, selector, changed,
	    pixel_format);

	if (changed != NULL) {
		CFRelease(changed);
	}
	return format;
}

static void
emit_sample_buffer(id self, SEL selector, CMSampleBufferRef sample_buffer)
{
	static bool said;

	if (sample_buffer == NULL) {
		if (!said) {
			said = true;
			say("hakios: mediaserverd: a node's output was given no sample buffer (the silent audio source's): not passed on\n");
		}
		return;
	}
	time_photo_encoder(self, sample_buffer);
	if (hakios_save_trace) {
		trace_photo_step(self, sample_buffer);
	}
	((void (*)(id, SEL, CMSampleBufferRef))original_emit_sample_buffer)(self, selector, sample_buffer);
}

/*
 * The photograph's encoder. Camera.app asks for HEIC -- HEVC inside a HEIF file, which CMPhoto makes
 * through VideoToolbox -- and VideoToolbox here has no HEVC encoder but the iPhone's hardware ones
 * (com.apple.videotoolbox.videoencoder.hevc and .hevc.gva, behind its AVE): the photo encoder node
 * failed with kVTCouldNotFindVideoEncoderErr (-12908). Apple's software HEVC codec is in the system
 * all the same, VCPHEVC.videocodec, whose decoder VideoToolbox takes here for every HEIC picture (see
 * the top), and there is an encoder in it, HEVCVideoEncoder_CreateInstance, that Darwin never registers:
 * the codec's VCPHEVCRegisterEncoder is an empty function. It is registered here, as the codec
 * registers its decoder (VTRegisterVideoDecoderWithInfo, not hardware accelerated).
 */
typedef OSStatus (*hevc_create_fn)(const void *, CFAllocatorRef, void **);
static hevc_create_fn software_hevc_create;

/* Record the callers of the existing encoder without changing the codec selected by VideoToolbox. */
static OSStatus
trace_hevc_create(const void *specification, CFAllocatorRef allocator, void **encoder)
{
	void *frames[10];
	OSStatus status = software_hevc_create(specification, allocator, encoder);
	int count = backtrace(frames, sizeof(frames) / sizeof(frames[0]));
	char stack[168];
	size_t used = 0;

	stack[0] = '\0';
	for (int i = 1; i < count && sizeof(stack) - used >= 20; i++) {
		used += (size_t)snprintf(stack + used, sizeof(stack) - used, " %#lx", (uintptr_t)frames[i] - slide);
	}
	say("hakios: heic trace: software factory status=%d, unslid callers%s\n", status, stack);
	return status;
}

/* The software encoder with Iris2 in front of its HEIC tiles (iris_tiles.c). */
extern void hakios_iris_wrap_encoder(void *encoder, void (*log)(const char *, ...));
static hevc_create_fn traced_or_software_create;

static OSStatus
iris_hevc_create(const void *specification, CFAllocatorRef allocator, void **encoder)
{
	OSStatus status = traced_or_software_create(specification, allocator, encoder);

	if (status == noErr && encoder != NULL && *encoder != NULL) {
		hakios_iris_wrap_encoder(*encoder, say);
	}
	return status;
}

static bool boot_word(const char *wanted);

static bool
heic_trace_enabled(void)
{
	return boot_word("cam_heic_trace=1");
}

static bool
boot_word(const char *wanted)
{
	char arguments[2048];
	size_t length = sizeof(arguments);
	char *state;

	if (sysctlbyname("kern.bootargs", arguments, &length, NULL, 0) != 0) {
		return false;
	}
	for (char *word = strtok_r(arguments, " ", &state); word != NULL; word = strtok_r(NULL, " ", &state)) {
		if (strcmp(word, wanted) == 0) {
			return true;
		}
	}
	return false;
}

static void
register_hevc_encoder(void)
{
	void *codec = dlopen("/System/Library/VideoCodecs/VCPHEVC.videocodec", RTLD_NOW | RTLD_LOCAL);
	void *vt = dlopen("/System/Library/Frameworks/VideoToolbox.framework/VideoToolbox", RTLD_NOW | RTLD_GLOBAL);
	hevc_create_fn create = codec != NULL ? dlsym(codec, "HEVCVideoEncoder_CreateInstance") : NULL;
	int (*register_encoder)(uint32_t, CFDictionaryRef, hevc_create_fn) =
	    vt != NULL ? dlsym(vt, "VTRegisterVideoEncoderWithInfo") : NULL;
	CFStringRef *id_key = vt != NULL ? dlsym(vt, "kVTVideoEncoderList_EncoderID") : NULL;
	CFStringRef *name_key = vt != NULL ? dlsym(vt, "kVTVideoEncoderList_EncoderName") : NULL;
	CFStringRef *hardware_key = vt != NULL ? dlsym(vt, "kVTVideoEncoderList_IsHardwareAccelerated") : NULL;
	int status;

	if (create == NULL || register_encoder == NULL || id_key == NULL || name_key == NULL || hardware_key == NULL) {
		say("hakios: mediaserverd: no software HEVC encoder to register (%s)\n", codec == NULL ? "no VCPHEVC.videocodec" :
		    create == NULL ? "no HEVCVideoEncoder_CreateInstance" : "VideoToolbox without VTRegisterVideoEncoderWithInfo");
		return;
	}
	const void *keys[] = { *id_key, *name_key, *hardware_key };
	const void *values[] = { CFSTR("com.apple.videotoolbox.videoencoder.hevc.vcp"), CFSTR("HEVC (VCPHEVC, software)"),
		kCFBooleanFalse };
	CFDictionaryRef info = CFDictionaryCreate(NULL, keys, values, 3, &kCFTypeDictionaryKeyCallBacks,
	    &kCFTypeDictionaryValueCallBacks);

	software_hevc_create = create;
	traced_or_software_create = heic_trace_enabled() ? trace_hevc_create : create;
	status = register_encoder('hvc1', info,
	    access("/dev/hakios-iris", F_OK) == 0 ? iris_hevc_create : traced_or_software_create);
	CFRelease(info);
	say("hakios: mediaserverd: the software HEVC encoder of VCPHEVC.videocodec registered for 'hvc1' (%d)\n", status);
}

struct method_spec {
	const char *selector, *types;
	IMP imp;
};

static Class
make_class(const char *name, const struct method_spec *methods, size_t count)
{
	Class cls = objc_allocateClassPair(objc_getClass("NSObject"), name, 0);

	for (size_t i = 0; i < count; i++) {
		class_addMethod(cls, sel_registerName(methods[i].selector), methods[i].imp, methods[i].types);
	}
	objc_registerClassPair(cls);
	return cls;
}

static void
flashlight_classes(void)
{
	static const struct method_spec vendor[] = {
		{ "registerClientWithPID:clientDescription:clientPriority:canStealFromClientsWithSamePriority:"
		  "deviceSharingWithOtherClientsAllowed:deviceAvailabilityChangedHandler:", "i@:i@iBB@", (IMP)flashlight_register },
		{ "copyDeviceForClient:informClientWhenDeviceAvailableAgain:error:", "@@:iB^i", (IMP)flashlight_copy_device },
		{ "copyStreamForFlashlightWithPosition:deviceType:forDevice:", "@@:ii@", (IMP)flashlight_copy_stream },
		{ "takeBackDevice:forClient:informClientWhenDeviceAvailableAgain:", "v@:@iB", (IMP)flashlight_take_back },
		{ "activeDeviceEquals:", "B@:@", (IMP)flashlight_active_equals },
	}, stream[] = {
		{ "supportedProperties", "@@:", (IMP)flashlight_stream_properties },
		{ "setProperty:value:", "i@:@@", (IMP)flashlight_stream_set },
	};

	vendor_class = make_class("HakiosFlashlightVendor", vendor, sizeof(vendor) / sizeof(vendor[0]));
	device_class = make_class("HakiosFlashlightDevice", NULL, 0);
	stream_class = make_class("HakiosFlashlightStream", stream, sizeof(stream) / sizeof(stream[0]));
}

/*
 * The class methods replaced, each as soon as its class exists. `expected` is the unslid address
 * of the code in Darwin userland 16.5 (20F66): a method that is not what it was is left alone. `original`, where
 * there is one, keeps what was there (the vendor's: given as it is wherever there is one, and where
 * there is none, ours only with the kernel's camera or flashlight).
 */
static struct replacement {
	const char *class_name, *selector;
	uintptr_t expected;
	IMP replacement, *original;
	bool instance;                  /* an instance method, not a class method */
	bool done;
} replacements[] = {
	{ "BWCoreImageFilterRenderer", "prewarmPortraitFilterVersion:semanticStyleFilters:", 0x1a2115c0cUL,
	  (IMP)prewarm_portrait_filter, NULL, false, false },
	{ "BWFigCaptureDeviceVendor", "sharedCaptureDeviceVendor", 0x1a1ef0d04UL,
	  (IMP)shared_vendor, &original_shared_vendor, false, false },
	{ "BWAudioSourceNode", "start:", 0x1a1eefa18UL, (IMP)audio_source_start, &original_audio_source_start, true, false },
	{ "FigVideoCaptureConnectionConfiguration", "videoStabilizationMethod", 0x1a1e99ef8UL,
	  (IMP)video_stabilization_method, NULL, true, false },
	{ "FigVideoCaptureConnectionConfiguration", "videoStabilizationType", 0x1a20d8f48UL,
	  (IMP)video_stabilization_type, NULL, true, false },
	{ "FigCaptureStillImageSettings", "qualityPrioritization", 0x1a1f533dcUL,
	  (IMP)quality_prioritization, &original_quality_prioritization, true, false },
	{ "BWGNRNode", "renderSampleBuffer:forInput:", 0x1a1ec83b8UL, (IMP)gnr_render, NULL, true, false },
	{ "BWMultiFilterThumbnailNode", "renderSampleBuffer:forInput:", 0x1a203d680UL, (IMP)thumbnail_render, NULL, true, false },
	/* (no selector: the instance method whose code is at `expected`) */
	{ "BWMultiFilterThumbnailNode", NULL, 0x1a1ef7248UL, (IMP)thumbnail_nothing, NULL, true, false },
	{ "BWNodeOutput", "emitSampleBuffer:", 0x1a1ea37e0UL, (IMP)emit_sample_buffer, &original_emit_sample_buffer, true, false },
	{ "FigCaptureSourceVideoFormat", "initWithFigCaptureStreamFormatDictionary:", 0x1a1e99518UL, (IMP)format_init,
	  &original_format_init, true, false },
	{ "FigCaptureSourceVideoFormat", "initWithFigCaptureStreamFormatDictionary:pixelFormatOverride:", 0x1a2035f4cUL,
	  (IMP)format_init_override, &original_format_init_override, true, false },
};

static void
image_added(__unused const struct mach_header *header, __unused intptr_t image_slide)
{
	for (size_t i = 0; i < sizeof(replacements) / sizeof(replacements[0]); i++) {
		struct replacement *r = &replacements[i];
		Class cls;
		Method method;
		IMP original;

		if (r->done || (cls = objc_getClass(r->class_name)) == NULL) {
			continue;
		}
		r->done = true;
		if (r->selector == NULL) {
			unsigned count = 0;
			Method *list = class_copyMethodList(cls, &count);

			method = NULL;
			for (unsigned j = 0; list != NULL && j < count; j++) {
				if ((uintptr_t)method_getImplementation(list[j]) == r->expected + slide) {
					method = list[j];
				}
			}
			free(list);
			if (method == NULL) {
				say("hakios: -[%s (code at %#lx)]: not found; left alone\n", r->class_name, r->expected);
				continue;
			}
			method_setImplementation(method, r->replacement);
			say("hakios: mediaserverd: -[%s %s] (at %#lx) replaced\n", r->class_name, sel_getName(method_getName(method)),
			    r->expected);
			continue;
		}
		method = r->instance ? class_getInstanceMethod(cls, sel_registerName(r->selector)) :
		    class_getClassMethod(cls, sel_registerName(r->selector));
		original = method ? method_getImplementation(method) : NULL;
		if (original == NULL || (uintptr_t)original != r->expected + slide) {
			say("hakios: %c[%s %s]: found %#lx, expected %#lx; left alone\n", r->instance ? '-' : '+', r->class_name,
			    r->selector,
			    original ? (uintptr_t)original - slide : 0, r->expected);
			continue;
		}
		if (r->original != NULL) {
			*r->original = original;
		}
		method_setImplementation(method, r->replacement);
		say("hakios: mediaserverd: %c[%s %s] replaced\n", r->instance ? '-' : '+', r->class_name, r->selector);
	}
}

/* An address as "image+offset" (our dylib's own code is not in the shared cache) or unslid in the cache. */
static void
where(uintptr_t address, char *out, size_t size)
{
	Dl_info info;

	if (address != 0 && dladdr((const void *)address, &info) && info.dli_fname != NULL && info.dli_fbase != NULL) {
		const char *base = strrchr(info.dli_fname, '/');
		snprintf(out, size, "%s+%#lx", base ? base + 1 : info.dli_fname, address - (uintptr_t)info.dli_fbase);
	} else {
		snprintf(out, size, "%#lx", address - slide);
	}
}

/*
 * Where mediaserverd died (as inject.c does for the window server): its crash reports never arrive
 * ("Failed to send exception EXC_CORPSE_NOTIFY"), and Camera.app's preview restarting every few
 * seconds was mediaserverd dying of SIGSEGV with nothing to go on. One line, then the signal kills it.
 */
static void
say_the_fault(int signal_number, siginfo_t *info, void *context_in)
{
	ucontext_t *context = context_in;
	uintptr_t pc = 0, lr = 0, fp = 0;
	char line[512], spot[96];
	size_t used;

	if (context != NULL && context->uc_mcontext != NULL) {
		pc = (uintptr_t)context->uc_mcontext->__ss.__pc;
		lr = (uintptr_t)context->uc_mcontext->__ss.__lr;
		fp = (uintptr_t)context->uc_mcontext->__ss.__fp;
	}
	used = (size_t)snprintf(line, sizeof(line), "hakios: mediaserverd: signal %d at %p, pc ", signal_number,
	    info != NULL ? info->si_addr : NULL);
	where(pc, spot, sizeof(spot));
	used += (size_t)snprintf(line + used, sizeof(line) - used, "%s, lr ", spot);
	where(lr, spot, sizeof(spot));
	used += (size_t)snprintf(line + used, sizeof(line) - used, "%s, called from", spot);
	for (unsigned i = 0; i < 6 && fp != 0 && (fp & 7) == 0 && used < sizeof(line) - 100; i++) {
		uintptr_t next = ((uintptr_t *)fp)[0], ret = ((uintptr_t *)fp)[1];

		if (ret == 0) {
			break;
		}
		where(ret, spot, sizeof(spot));
		used += (size_t)snprintf(line + used, sizeof(line) - used, " %s", spot);
		fp = next > fp ? next : 0;
	}
	say("%s (cache addresses unslid)\n", line);
	signal(signal_number, SIG_DFL);
	raise(signal_number);
}

static void
watch_for_faults(void)
{
	struct sigaction action;

	memset(&action, 0, sizeof(action));
	action.sa_sigaction = say_the_fault;
	action.sa_flags = SA_SIGINFO | SA_RESETHAND;
	sigaction(SIGSEGV, &action, NULL);
	sigaction(SIGBUS, &action, NULL);
	sigaction(SIGABRT, &action, NULL);
	sigaction(SIGILL, &action, NULL);
	sigaction(SIGTRAP, &action, NULL);
}

__attribute__((constructor))
static void
hakios_media_init(void)
{
	const char *name = getprogname();
	size_t length = 0;

	if (name == NULL || strcmp(name, "mediaserverd") != 0) {
		return;
	}
	hakios_save_trace = boot_word("cam_save_trace=1");
	slide = (uintptr_t)_dyld_get_shared_cache_range(&length) - CACHE_BASE;
	if (hakios_save_trace) {
		say("hakios: save trace: mediaserverd pid %d, the shared cache's slide %#lx\n", getpid(), (unsigned long)slide);
	}
	torch = open("/dev/hakios-torch", O_WRONLY);
	camera = access("/dev/hakios-camera", F_OK) == 0;
	if (torch >= 0) {
		flashlight_classes();
		say("hakios: mediaserverd: the kernel has a flashlight (/dev/hakios-torch)\n");
	}
	if (camera) {
		watch_for_faults();
		say("hakios: mediaserverd: the kernel has a camera (/dev/hakios-camera)\n");
		register_hevc_encoder();
	}
	_dyld_register_func_for_add_image(image_added);         /* also called for every image already there */
}
