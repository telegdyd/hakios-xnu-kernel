/*
 * The phone's cameras as Darwin's capture stack wants them: a FigCaptureDevice with two FigCaptureStreams,
 * the wide camera as the back camera and the front one as the front camera, made for mediaserverd's
 * BWFigCaptureDeviceVendor in place of the device Apple's H10ISP.mediacapture makes of the iPhone's ISP
 * (part of libhakios.media.dylib; inject_media.c gives the vendor our device's create function when
 * the kernel has a camera: cam=5).
 *
 * Both are CoreMedia "derived objects" (CMDerivedObjectCreate): a table of functions for the base
 * class (properties) and one for the device or stream class, as H10ISP's are laid out in Darwin userland 16.5:
 *   device  RequestStreams, RelinquishStreams (and a processing session and Pearl calibration, not here)
 *   stream  Start, Stop, AddAttachmentsToSampleBuffer
 * A stream's frames go to the capture graph through the blocks it is given in its property
 * "VideoOutputHandlers" ({ output: { PixelBufferHandler: ^(CVPixelBufferRef, CMTime) } }), for the
 * outputs "VideoOutputConfigurations" describes (the graph's own buffer pools) and "VideoOutputsEnabled"
 * switches on. A frame: read from the kernel's /dev/hakios-camera (RAW10), made into a picture by
 * camera_isp.c (exposure, white balance kept to the sensor's reference lights, the colour matrix and
 * lens shading of the camera's own tuning, gamma, zoom), one pixel buffer for each output, with the
 * metadata the graph keeps its frame statistics and its zoom by. The kernel gives one sensor's frames
 * at a time: a stream that starts stops the other's frames and has the kernel change sensors.
 *
 * The formats the streams offer are named as the iPhone X's cameras' are in CMCapture's
 * D22/AVCaptureSession.plist (FigCaptureStreamFormats, by the sensor sizes and frame rates the plist
 * matches them by), so that their AVCaptureDevice formats and session presets resolve: the pictures
 * are scaled from the sensor's frames to whatever size is asked for.
 *
 * The shutter: StillImageCaptureNow asks for one frame of the whole sensor on the StillImage output
 * (the time machine's TimeMachineSuspendNow/ResumeNow are taken and ignored). Focus, where there is
 * a lens motor (the wide camera's): by contrast, continuous, and on the window a tap gives (af_frame()).
 *
 * Everything asked of the device and the streams that is not answered here is said in mediaserverd's
 * log ("hakios: camera: ..."), once per property.
 */
#include <CoreFoundation/CoreFoundation.h>
#include <CoreMedia/CoreMedia.h>
#include <CoreVideo/CoreVideo.h>
#include <IOSurface/IOSurfaceRef.h>
#include <dispatch/dispatch.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <mach/mach_time.h>
#include <math.h>
#include <notify.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/sysctl.h>
#include <sys/uio.h>
#include <unistd.h>
#include "camera_isp.h"
#include "camera_tap.h"

void hakios_camera_say(const char *format, ...) __attribute__((format(printf, 1, 2)));
int hakios_trace_stamp(char *line, size_t size);         /* inject_media.c: "[t=...] " with cam_save_trace=1 */

void
hakios_camera_say(const char *format, ...)
{
	char line[440];
	va_list ap;
	int n = hakios_trace_stamp(line, sizeof(line));

	n += snprintf(line + n, sizeof(line) - (size_t)n, "hakios: camera: ");

	va_start(ap, format);
	n += vsnprintf(line + n, sizeof(line) - (size_t)n, format, ap);
	va_end(ap);
	if (n > (int)sizeof(line) - 2) {
		n = sizeof(line) - 2;
	}
	line[n++] = '\n';
	write(2, line, (size_t)n);
}
#define say hakios_camera_say

/* ---- CoreMedia's derived objects, as Darwin userland 16.5 lays them out ---------------------------------------- */

typedef struct OpaqueCMBaseObject *CMBaseObjectRef;
typedef const void *CMBaseClassID;

struct base_class {
	uint64_t        version;
	uint64_t        storage_size;
	Boolean         (*equal)(CMBaseObjectRef, CMBaseObjectRef);
	OSStatus        (*invalidate)(CMBaseObjectRef);
	void            (*finalize)(CMBaseObjectRef);
	CFStringRef     (*copy_debug_description)(CMBaseObjectRef);
	OSStatus        (*copy_property)(CMBaseObjectRef, CFStringRef, CFAllocatorRef, void *);
	OSStatus        (*set_property)(CMBaseObjectRef, CFStringRef, CFTypeRef);
	OSStatus        (*notification_barrier)(CMBaseObjectRef);
	const void *    protocols;
	const void *    reserved;
};
struct device_class {
	uint64_t        version;                /* 3 */
	void *          processing_session_create;
	OSStatus        (*request_streams)(CMBaseObjectRef, CFArrayRef, CFDictionaryRef);
	OSStatus        (*relinquish_streams)(CMBaseObjectRef, CFArrayRef, CFDictionaryRef);
	void *          pearl_calibration;
};
struct stream_class {
	uint64_t        version;                /* 2 */
	OSStatus        (*start)(CMBaseObjectRef);
	OSStatus        (*stop)(CMBaseObjectRef);
	OSStatus        (*add_attachments)(CMBaseObjectRef, CMSampleBufferRef, CFDictionaryRef);
};
struct vtable {
	const void *    reserved;
	const struct base_class *base;
	const void *    derived;
};

#define ERR_PARAM               (-12780)
#define ERR_NOT_AVAILABLE       (-12783)
#define ERR_UNSUPPORTED         (-12782)

static OSStatus (*derived_create)(CFAllocatorRef, const struct vtable *, CMBaseClassID, CMBaseObjectRef *);
static void *(*derived_storage)(CMBaseObjectRef);
static CMBaseClassID (*device_class_id)(void);
static CMBaseClassID (*stream_class_id)(void);
static CFStringRef port_back;           /* kFigCapturePortType_BackFacingCamera */
static CFStringRef port_front;          /* kFigCapturePortType_FrontFacingCamera */

/*
 * The frames' metadata: what H10ISP's stream attaches to every frame (a dictionary under
 * kFigCaptureSampleBufferAttachmentKey_MetadataDictionary, through the stream's
 * AddAttachmentsToSampleBuffer when the graph defers it), and what the capture device keeps its frame
 * statistics of. -[BWFrameStatisticsByPortType updateWithFrameMetadata:updateFocusDistance:] counts
 * a frame only for a buffer that has it, and -[BWFigVideoCaptureDevice
 * _stillImageCaptureInProgressSubstateForFrameStatisticsByPortType:...] puts a standard photograph
 * off until more than four frames have been counted (then only while AFStatus is 2, a coarse focus
 * scan): without it the shutter's capture was put off on every frame (the retry in
 * _updateStateUsingVideoSampleBuffer:fromCaptureStream:), for ever, and StillImageCaptureNow never
 * came. The dictionary goes on the pixel buffer too, and mutable, as the ISP's does: FigImageControl_Focus
 * (the device's camera controls, on every frame once the sample buffer has metadata) reads it from
 * the pixel buffer with no check that it is there -- CFDictionaryGetValue(NULL, ...): mediaserverd
 * died of SIGSEGV -- and adds its own keys to it. The keys are CoreMedia's; the values come from the
 * frame's header and our ISP's levels, in Apple's units where they are known (the exposure in seconds,
 * gains with 256 for 1x); lux and SNR are rough.
 */
enum {
	MD_PORT, MD_PTS, MD_EXPOSURE, MD_AGC, MD_ISP_DGAIN, MD_SENSOR_DGAIN, MD_ISO_BASE, MD_FRAME_RATE, MD_LUX, MD_SNR,
	MD_AF_STATUS, MD_FOCUSING, MD_AE_STABLE, MD_AE_LOCKED, MD_AWB_STABLE, MD_AWB_LOCKED, MD_AF_LOCKED, MD_ZOOM, MD_EXPOSURE_BIAS, MD_EXPOSURE_BIAS_ID, MD_KEYS
};
static const char *const md_names[MD_KEYS] = {
	"kFigCaptureStreamMetadata_PortType", "kFigCaptureSampleBufferAttachmentKey_OriginalPresentationTimeStamp",
	"kFigCaptureStreamMetadata_ExposureTime", "kFigCaptureStreamMetadata_AGC", "kFigCaptureStreamMetadata_ispDGain",
	"kFigCaptureStreamMetadata_sensorDGain", "kFigCaptureStreamMetadata_ISOBase", "kFigCaptureStreamMetadata_CurrentFrameRate",
	"kFigCaptureStreamMetadata_LuxLevel", "kFigCaptureStreamMetadata_NormalizedSNR", "kFigCaptureStreamMetadata_AFStatus",
	"kFigCaptureStreamMetadata_FocusingMethod", "kFigCaptureStreamMetadata_AEStable", "kFigCaptureStreamMetadata_AELocked",
	"kFigCaptureStreamMetadata_AWBStable", "kFigCaptureStreamMetadata_AWBLocked", "kFigCaptureStreamMetadata_AFLocked",
	"kFigCaptureStreamMetadata_ZoomFactor", "kFigCaptureStreamMetadata_ExposureBias",
	"kFigCaptureStreamMetadata_ExposureBiasCommandID",
};
static CFStringRef md_key[MD_KEYS];
static CFStringRef md_attachment;       /* kFigCaptureSampleBufferAttachmentKey_MetadataDictionary */

static bool
find_functions(void)
{
	CFStringRef *port;

	derived_create = dlsym(RTLD_DEFAULT, "CMDerivedObjectCreate");
	derived_storage = dlsym(RTLD_DEFAULT, "CMBaseObjectGetDerivedStorage");
	device_class_id = dlsym(RTLD_DEFAULT, "FigCaptureDeviceGetClassID");
	stream_class_id = dlsym(RTLD_DEFAULT, "FigCaptureStreamGetClassID");
	port = dlsym(RTLD_DEFAULT, "kFigCapturePortType_BackFacingCamera");
	port_back = port != NULL ? *port : NULL;
	port = dlsym(RTLD_DEFAULT, "kFigCapturePortType_FrontFacingCamera");
	port_front = port != NULL ? *port : NULL;
	port = dlsym(RTLD_DEFAULT, "kFigCaptureSampleBufferAttachmentKey_MetadataDictionary");
	md_attachment = port != NULL ? *port : NULL;
	for (int i = 0; i < MD_KEYS; i++) {
		port = dlsym(RTLD_DEFAULT, md_names[i]);
		md_key[i] = port != NULL ? *port : NULL;
		if (md_key[i] == NULL) {
			say("%s is not there: the frames' metadata go without it", md_names[i]);
		}
	}
	if (md_attachment == NULL || md_key[MD_PORT] == NULL) {
		say("kFigCaptureSampleBufferAttachmentKey_MetadataDictionary or the port's key is not there: frames without "
		    "metadata, and the shutter will wait for ever");
		md_attachment = NULL;
	}
	if (derived_create == NULL || derived_storage == NULL || device_class_id == NULL || stream_class_id == NULL || port_back == NULL ||
	    port_front == NULL) {
		say("CoreMedia's or CMCapture's functions are not there: no camera");
		return false;
	}
	return true;
}

/* ---- what is asked and not answered: said once per name ------------------------------------------- */

static CFMutableSetRef said_names;
static pthread_mutex_t said_lock = PTHREAD_MUTEX_INITIALIZER;

static void
say_value(const char *what, CFStringRef key, CFTypeRef value)
{
	char name[160] = "?", text[300] = "";

	CFStringGetCString(key, name, sizeof(name), kCFStringEncodingUTF8);
	if (value != NULL) {
		CFStringRef d = CFCopyDescription(value);

		CFStringGetCString(d, text, sizeof(text), kCFStringEncodingUTF8);
		CFRelease(d);
		for (char *c = text; *c; c++) {
			*c = *c == '\n' ? ' ' : *c;
		}
	}
	say("%s %s%s%s", what, name, value ? " = " : "", text);
}

static void
say_once(const char *what, CFStringRef key, CFTypeRef value)
{
	CFStringRef tag = CFStringCreateWithFormat(NULL, NULL, CFSTR("%s %@"), what, key);

	pthread_mutex_lock(&said_lock);
	if (said_names == NULL) {
		said_names = CFSetCreateMutable(NULL, 0, &kCFTypeSetCallBacks);
	}
	bool first = !CFSetContainsValue(said_names, tag);

	CFSetAddValue(said_names, tag);
	pthread_mutex_unlock(&said_lock);
	CFRelease(tag);
	if (first) {
		say_value(what, key, value);
	}
}

/* ---- numbers and dictionaries ---------------------------------------------------------------------- */

static CFNumberRef
number_i(int64_t v)
{
	return CFNumberCreate(NULL, kCFNumberSInt64Type, &v);
}

static CFNumberRef
number_f(double v)
{
	return CFNumberCreate(NULL, kCFNumberDoubleType, &v);
}

static void
put(CFMutableDictionaryRef d, const char *key, CFTypeRef value)
{
	CFStringRef k = CFStringCreateWithCString(NULL, key, kCFStringEncodingUTF8);

	CFDictionarySetValue(d, k, value);
	CFRelease(k);
	CFRelease(value);
}

static CFDictionaryRef
new_dict(void)
{
	return CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
}

static bool
key_is(CFStringRef key, const char *name)
{
	char buffer[128];

	return CFStringGetCString(key, buffer, sizeof(buffer), kCFStringEncodingUTF8) && strcmp(buffer, name) == 0;
}

/* ---- what the device and the stream say they have ---------------------------------------------------- */

/*
 * The capture stack asks only for the properties an object lists in its "SupportedPropertiesDictionary"
 * ({ name: { PropertyType, ReadWriteStatus } }). Listed here: what is answered, and what is taken
 * (and, for now, mostly not acted on) without failing the graph that sets it.
 */
struct prop {
	const char *name, *type, *access;
};

static const struct prop stream_props[] = {
	{ "PortType", "String", "ReadOnly" }, { "UniqueID", "String", "ReadOnly" }, { "NonLocalizedName", "String", "ReadOnly" },
	{ "SupportedFormatsArray", "Collection", "ReadOnly" }, { "FocalLength", "Number", "ReadOnly" },
	{ "LensFNumber", "Number", "ReadOnly" }, { "PixelSize", "Number", "ReadOnly" },
	{ "MaxZoomFactor", "Number", "ReadOnly" }, { "SupportedColorSpaces", "Collection", "ReadOnly" },
	{ "HasAutoFocusPositionSensor", "Boolean", "ReadOnly" }, { "Clock", "Clock", "ReadOnly" },
	{ "FormatIndex", "Number", "ReadWrite" }, { "MinimumFrameRate", "Number", "ReadWrite" }, { "MaximumFrameRate", "Number", "ReadWrite" },
	{ "MaximumAllowedFrameRate", "Number", "ReadWrite" }, { "VideoOutputConfigurations", "Collection", "ReadWrite" },
	{ "VideoOutputsEnabled", "Collection", "ReadWrite" }, { "VideoOutputHandlers", "Collection", "WriteOnly" },
	{ "DeferAdditionOfAttachments", "Boolean", "ReadWrite" }, { "TorchLevel", "Number", "ReadWrite" },
	{ "HighCurrentTorchEnabled", "Boolean", "ReadWrite" }, { "ColorSpace", "Number", "ReadWrite" }, { "ZoomFactor", "Number", "ReadWrite" },
	{ "ExposureBias", "Number", "ReadWrite" }, { "DeviceOrientation", "Number", "ReadWrite" }, { "FlashMode", "Number", "ReadWrite" },
	{ "StillImageCaptureNow", "Collection", "WriteOnly" }, { "LockAENow", "Boolean", "WriteOnly" }, { "UnlockAENow", "Boolean", "WriteOnly" },
	{ "LockAWBNow", "Boolean", "WriteOnly" }, { "UnlockAWBNow", "Boolean", "WriteOnly" }, { "MaxIntegrationTime", "Number", "ReadWrite" },
	{ "AEMaxGain", "Number", "ReadWrite" }, { "AEMinGain", "Number", "ReadWrite" }, { "FaceDetectionEnabled", "Boolean", "ReadWrite" },
	{ "VideoRecordingInProgress", "Boolean", "ReadWrite" }, { "MemoryPoolID", "Number", "ReadWrite" },
	{ "ChromaNoiseReductionEnabled", "Boolean", "ReadWrite" }, { "TemporalNoiseReductionEnabled", "Boolean", "ReadWrite" },
	{ "MultipleOutputSupportEnabled", "Boolean", "ReadWrite" }, { "VideoOutputsColorInfo", "Collection", "ReadWrite" },
	{ "RegionOfInterestForCameraControls", "Collection", "ReadWrite" }, { "AEMeteringMode", "Number", "ReadWrite" },
	{ "MaxFrameCoalescingDuration", "Number", "ReadWrite" }, { "LimitAERange", "Boolean", "ReadWrite" },
	{ "TimeMachineSuspendNow", "Boolean", "WriteOnly" }, { "TimeMachineResumeNow", "Boolean", "WriteOnly" },
	/* focus (the wide camera's lens motor; see af_frame()) */
	{ "AutoFocusNow", "Collection", "WriteOnly" }, { "ContinuousAutoFocusNow", "Collection", "WriteOnly" },
	{ "FocusNow", "Collection", "WriteOnly" }, { "PeakTrackingFocusNow", "Collection", "WriteOnly" },
	{ "LockAFNow", "Boolean", "WriteOnly" }, { "ManualFocusNow", "Collection", "WriteOnly" },
	{ "AFWindowParams", "Collection", "ReadWrite" }, { "AutoFocusMode", "Number", "ReadWrite" },
}, device_props[] = {
	{ "StreamArray", "Collection", "ReadOnly" }, { "Clock", "Clock", "ReadOnly" }, { "ClientIsActive", "Boolean", "ReadWrite" },
	{ "SupportedSynchronizedStreamsGroups", "Collection", "ReadOnly" }, { "MemoryPoolID", "Number", "ReadWrite" },
};

static CFDictionaryRef
supported(const struct prop *p, size_t n)
{
	CFMutableDictionaryRef all = (CFMutableDictionaryRef)new_dict();

	for (size_t i = 0; i < n; i++) {
		CFMutableDictionaryRef one = (CFMutableDictionaryRef)new_dict();

		put(one, "PropertyType", CFStringCreateWithCString(NULL, p[i].type, kCFStringEncodingUTF8));
		put(one, "ReadWriteStatus", CFStringCreateWithCString(NULL, p[i].access, kCFStringEncodingUTF8));
		put(all, p[i].name, one);
	}
	return all;
}

/* ---- the formats ----------------------------------------------------------------------------------- */

#define PF_420F         0x34323066      /* '420f' */
#define PF_420V         0x34323076      /* '420v' */
#define PF_BGRA         0x42475241      /* 'BGRA' */

static CFDictionaryRef
output_info(int width, int height)
{
	CFMutableDictionaryRef o = (CFMutableDictionaryRef)new_dict();
	CFNumberRef pf[3] = { number_i(PF_420F), number_i(PF_420V), number_i(PF_BGRA) };
	CFArrayRef pfs = CFArrayCreate(NULL, (const void **)pf, 3, &kCFTypeArrayCallBacks);

	for (int i = 0; i < 3; i++) {
		CFRelease(pf[i]);
	}
	put(o, "SupportedPixelFormats", pfs);
	put(o, "NativeWidth", number_i(width));
	put(o, "NativeHeight", number_i(height));
	put(o, "MaxDownscaleFactor", number_f(32));
	put(o, "MaxScaleFactor", number_f(1));
	put(o, "MinScaleFactor", number_f(1.0 / 32));
	put(o, "OutputWidthAlignmentRequirement", number_i(16));
	return o;
}

static CFDictionaryRef
format(int sensor_width, int sensor_height, int max_fps, bool binned, double fov)
{
	CFMutableDictionaryRef f = (CFMutableDictionaryRef)new_dict(), outputs = (CFMutableDictionaryRef)new_dict();
	CMVideoFormatDescriptionRef fd = NULL;

	CMVideoFormatDescriptionCreate(NULL, PF_420F, sensor_width, sensor_height, NULL, &fd);
	if (fd != NULL) {
		put(f, "FormatDescription", fd);
	}
	put(f, "VideoMinFrameRate", number_f(2));
	put(f, "VideoMaxFrameRate", number_f(max_fps));
	put(f, "VideoIsBinned", CFRetain(binned ? kCFBooleanTrue : kCFBooleanFalse));
	put(f, "VideoBinningFactorHorizontal", number_i(binned ? 2 : 1));
	put(f, "VideoBinningFactorVertical", number_i(binned ? 2 : 1));
	put(f, "VideoFieldOfView", number_f(fov));
	put(f, "VideoMinIntegrationTime", number_i(100));
	put(f, "VideoMaxIntegrationTime", number_i(1000000 / 15));
	put(f, "VideoRawBitDepth", number_i(10));
	put(f, "VideoMaxWidth", number_i(sensor_width));
	put(f, "VideoMaxHeight", number_i(sensor_height));
	put(f, "Experimental", CFRetain(kCFBooleanFalse));
	put(f, "SensorWidth", number_i(sensor_width));
	put(f, "SensorHeight", number_i(sensor_height));
	put(f, "ISOBase", number_f(32));
	put(f, "DefaultAEMaxGain", number_f(64));

	/*
	 * The sensor crop geometry the capture-graph builder reads off this format to resolve non-zero
	 * output dimensions (confirmed by a hook: -[FigCaptureSourceVideoFormat sensorCropDimensions] /
	 * postGDCCropDimensions] return these). Everything spans the full sensor: no GDC crop.
	 */
	put(f, "SensorCropWidth", number_i(sensor_width));
	put(f, "SensorCropHeight", number_i(sensor_height));
	put(f, "PreGDCSensorCropWidth", number_i(sensor_width));
	put(f, "PreGDCSensorCropHeight", number_i(sensor_height));
	put(f, "PostGDCCropWidth", number_i(sensor_width));
	put(f, "PostGDCCropHeight", number_i(sensor_height));
	put(f, "GeometricDistortionCorrectedNativeWidth", number_i(sensor_width));
	put(f, "GeometricDistortionCorrectedNativeHeight", number_i(sensor_height));

	put(outputs, "PrimaryScaler", output_info(sensor_width, sensor_height));
	put(outputs, "SecondaryScaler", output_info(sensor_width, sensor_height));
	put(outputs, "StillImage", output_info(sensor_width, sensor_height));
	put(f, "SupportedOutputs", outputs);
	return f;
}

/* ---- the stream ------------------------------------------------------------------------------------- */

#define HELD    3               /* the moving pictures kept from the pool after they are given (see deliver) */

struct output {
	const char *    name;
	CFDictionaryRef config;                 /* from VideoOutputConfigurations */
	bool            enabled;
	void *          handler;                /* a block: ^(CVPixelBufferRef, CMTime) */
	bool            said_shape;
	float           measured_bias;  /* most recent preview compensation sampled at the output */
	uint64_t        delivered, refused;
	uint64_t        render_ns, render_count;
	CVPixelBufferRef held[HELD];            /* the pictures given last, kept from the pool (see deliver) */
	unsigned        held_next;
	bool            said_pool;
};

static void let_go(struct output *out);

#define OUTPUTS 3
static const char *const output_names[OUTPUTS] = { "PrimaryScaler", "SecondaryScaler", "StillImage" };

#define MAX_ZOOM        16              /* (the most the iPhone X's wide camera zooms digitally) */

/*
 * The device's cameras, a stream each, as the iPhone X's are in CMCapture's D22/AVCaptureSession.plist
 * (its devices' uniqueIDs, and the stream formats its formats and presets are matched to by sensor size
 * and frame rate): the wide camera as the back camera, the front one as the front camera. The kernel
 * gives one sensor's frames at a time; a stream's frames start with its own sensor's (see worker()).
 */
static const struct camera {
	uint32_t        sensor;         /* the kernel's (the frames' sensor, HAKIOS_CAMERA_SENSOR) */
	const char      *name, *unique_id;
	bool            turned;         /* its sensor the other way up from the iPhone's: the pictures turned half round */
	double          focal_length, f_number, pixel_size, fov;
	int             formats[2][3];  /* sensor width, height, frames a second of "Photo" and of a video format */
} cameras[2] = {
	{ 1, "Back Camera", "com.apple.avfoundation.avcapturedevice.built-in_video:0", false, 4.0, 1.8, 1.22, 65.0,
	  { { 4032, 3024, 30 }, { 4224, 2376, 60 } } },        /* "Photo", "Video-60" */
	{ 2, "Front Camera", "com.apple.avfoundation.avcapturedevice.built-in_video:1", true, 2.87, 2.2, 1.0, 70.0,
	  { { 3088, 2320, 30 }, { 3840, 2160, 30 } } },        /* "Photo", "Video-30" */
};

/* ExposureBias is an unsigned 8.8 gain. The optional command ID is echoed on an applied frame. */
struct exposure_bias {
	uint16_t gain;
	int32_t command_id;
};

struct stream {
	CFArrayRef      formats;
	CFDictionaryRef supported;
	CFDictionaryRef colour_info;    /* from VideoOutputsColorInfo */
	CFDictionaryRef metadata;       /* the last frame's (frame_metadata()), for a buffer that has none of its own */
	pthread_mutex_t metadata_lock;  /* (not `lock': AddAttachmentsToSampleBuffer may come while a frame is given out) */
	unsigned        stills_wanted;  /* photographs asked for by StillImageCaptureNow, not yet given */
	int             format_index;
	double          min_fps, max_fps;
	float           zoom;           /* ZoomFactor: the picture is the middle 1/zoom of the sensor each way */
	const struct camera *camera;
	CFStringRef     port;
	pthread_mutex_t controls_lock;
	bool            ae_spot, ae_locked;
	struct exposure_bias exposure_bias;
	bool            af_now;         /* a focus asked for (AutoFocusNow and the like): a search at the next frame */
	bool            af_locked;      /* LockAFNow: no search but one asked for */
	float           af_window[4];   /* AFWindowParams: where to focus, as fractions of the frame (x0, y0, x1, y1) */
	struct output   out[OUTPUTS];
	pthread_mutex_t lock;
	pthread_t       worker;
	volatile bool   running;
	volatile bool   graph_running;  /* the capture graph has started this stream: frames may be given out */
	bool            worker_started;
	int             torch;
	uint64_t        frames, stills_given;
	uint64_t        graph_started_at, graph_first_seq, graph_last_seq;
	uint64_t        graph_frames, graph_delivered_at_start, graph_refused_at_start;
	uint64_t        graph_render_ns_at_start, graph_render_count_at_start;
};

static struct stream *
stream_of(CMBaseObjectRef o)
{
	return derived_storage(o);
}

static int
output_index(CFStringRef name)
{
	for (int i = 0; i < OUTPUTS; i++) {
		if (key_is(name, output_names[i])) {
			return i;
		}
	}
	return -1;
}

static OSStatus
stream_copy_property(CMBaseObjectRef o, CFStringRef key, CFAllocatorRef allocator, void *out)
{
	struct stream *s = stream_of(o);
	CFTypeRef value = NULL;

	(void)allocator;
	if (key_is(key, "PortType")) {
		value = CFRetain(s->port);
	} else if (key_is(key, "UniqueID") || key_is(key, "ModelID")) {
		value = CFStringCreateWithCString(NULL, s->camera->unique_id, kCFStringEncodingUTF8);
	} else if (key_is(key, "NonLocalizedName") || key_is(key, "LocalizedName")) {
		value = CFStringCreateWithCString(NULL, s->camera->name, kCFStringEncodingUTF8);
	} else if (key_is(key, "Manufacturer")) {
		value = CFRetain(CFSTR("Apple Inc."));
	} else if (key_is(key, "SupportedFormatsArray")) {
		value = CFRetain(s->formats);
	} else if (key_is(key, "FormatIndex")) {
		value = number_i(s->format_index);
	} else if (key_is(key, "MinimumFrameRate")) {
		value = number_f(s->min_fps);
	} else if (key_is(key, "MaximumFrameRate")) {
		value = number_f(s->max_fps);
	} else if (key_is(key, "Clock")) {
		value = CFRetain(CMClockGetHostTimeClock());
	} else if (key_is(key, "TorchLevel")) {
		value = number_f(0);
	} else if (key_is(key, "ExposureBias")) {
		pthread_mutex_lock(&s->controls_lock);
		value = number_i(s->exposure_bias.gain);
		pthread_mutex_unlock(&s->controls_lock);
	} else if (key_is(key, "VideoOutputConfigurations") || key_is(key, "VideoOutputsEnabled")) {
		/*
		 * ReadWrite, but the capture-graph builder READS these back while it builds the graph
		 * (BWMultiStreamCameraSourceNode's _updateOutputsStorage does a getProperty: for
		 * VideoOutputConfigurations, not IfSupported:), and a failed read tears the graph down. Return
		 * what each output holds now.
		 */
		bool configs = key_is(key, "VideoOutputConfigurations");
		CFMutableDictionaryRef d = (CFMutableDictionaryRef)new_dict();

		pthread_mutex_lock(&s->lock);
		for (int i = 0; i < OUTPUTS; i++) {
			CFStringRef name = CFStringCreateWithCString(NULL, output_names[i], kCFStringEncodingUTF8);

			if (configs) {
				if (s->out[i].config != NULL) {
					CFDictionarySetValue(d, name, s->out[i].config);
				}
			} else {
				CFDictionarySetValue(d, name, s->out[i].enabled ? kCFBooleanTrue : kCFBooleanFalse);
			}
			CFRelease(name);
		}
		pthread_mutex_unlock(&s->lock);
		value = d;
	} else if (key_is(key, "VideoOutputsColorInfo")) {
		/*
		 * ReadWrite, and the capture graph READS it before it starts the session:
		 * -[BWMultiStreamCameraSourceNode _verifyColorSpacePropertiesForAllOutputs] does a
		 * getProperty:error: for it, and an error there fails the session's transition to running
		 * and tears the whole graph down again. What it holds matters only for an output whose
		 * colour space is not sRGB (_verifyColorSpacePropertiesForOutputID: returns at once when
		 * the output's colour space is 0), and sRGB is the only one this stream offers, so what
		 * the graph set is given back as it is.
		 */
		pthread_mutex_lock(&s->lock);
		value = s->colour_info != NULL ? CFRetain(s->colour_info) : new_dict();
		pthread_mutex_unlock(&s->lock);
	} else if (key_is(key, "SupportedPropertiesDictionary")) {
		value = CFRetain(s->supported);
	} else if (key_is(key, "FocalLength")) {
		value = number_f(s->camera->focal_length);     /* (the iPhone X's cameras', in millimetres) */
	} else if (key_is(key, "LensFNumber")) {
		value = number_f(s->camera->f_number);
	} else if (key_is(key, "PixelSize")) {
		/*
		 * The capture-graph builder REQUIRES this of the stream: -[BWFigVideoCaptureStream
		 * initWithCaptureStream:...] does a getProperty: (not IfSupported:) for it, and without it the
		 * init fails, so FigVideoCaptureSourcesActivateAndCreateDevices errors and the graph is never
		 * built. The sensor's pixel pitch in micrometres (the iPhone X cameras'); any value in
		 * (0, 100) is accepted.
		 */
		value = number_f(s->camera->pixel_size);
	} else if (key_is(key, "MaxZoomFactor")) {
		value = number_f(MAX_ZOOM);
	} else if (key_is(key, "ZoomFactor")) {
		value = number_f(s->zoom);
	} else if (key_is(key, "SupportedColorSpaces")) {
		CFNumberRef srgb = number_i(0);

		value = CFArrayCreate(NULL, (const void **)&srgb, 1, &kCFTypeArrayCallBacks);
		CFRelease(srgb);
	} else if (key_is(key, "ColorSpace") || key_is(key, "DeviceOrientation") || key_is(key, "FlashMode")) {
		value = number_i(0);
	} else if (key_is(key, "HasAutoFocusPositionSensor") || key_is(key, "HighCurrentTorchEnabled") ||
	    key_is(key, "DeferAdditionOfAttachments")) {
		value = CFRetain(kCFBooleanFalse);
	} else {
		say_once("stream: no answer for", key, NULL);
		return ERR_NOT_AVAILABLE;
	}
	*(CFTypeRef *)out = value;
	return 0;
}

static void torch_set(struct stream *s, CFTypeRef value);

static double
dict_number(CFDictionaryRef d, const char *name, double otherwise)
{
	CFStringRef k = CFStringCreateWithCString(NULL, name, kCFStringEncodingUTF8);
	CFTypeRef v = CFDictionaryGetValue(d, k);
	double n = otherwise;

	CFRelease(k);
	if (v != NULL && CFGetTypeID(v) == CFNumberGetTypeID()) {
		CFNumberGetValue(v, kCFNumberDoubleType, &n);
	}
	return n;
}

/* AFWindowParams: the first window, as fractions of the frame; none (or not one) is the middle */
static void
af_window(struct stream *s, CFTypeRef value)
{
	CFDictionaryRef w = NULL;
	double x0, y0, width, height;

	if (value != NULL && CFGetTypeID(value) == CFArrayGetTypeID() && CFArrayGetCount(value) > 0) {
		w = CFArrayGetValueAtIndex(value, 0);
	}
	if (w == NULL || CFGetTypeID(w) != CFDictionaryGetTypeID() || (width = dict_number(w, "width", 0)) <= 0 ||
	    (height = dict_number(w, "height", 0)) <= 0) {
		s->af_window[0] = s->af_window[1] = 0.3f;
		s->af_window[2] = s->af_window[3] = 0.7f;
		return;
	}
	x0 = dict_number(w, "hStart", 0) / 1000;
	y0 = dict_number(w, "vStart", 0) / 1000;
	width = fmin(width / 1000, 1);
	height = fmin(height / 1000, 1);
	x0 = isfinite(x0) ? x0 : 0;
	y0 = isfinite(y0) ? y0 : 0;
	/* (at least a tenth of the frame each way, about the middle of what was asked) */
	if (width < 0.1) {
		x0 -= (0.1 - width) / 2;
		width = 0.1;
	}
	if (height < 0.1) {
		y0 -= (0.1 - height) / 2;
		height = 0.1;
	}
	x0 = x0 < 0 ? 0 : x0 + width > 1 ? 1 - width : x0;
	y0 = y0 < 0 ? 0 : y0 + height > 1 ? 1 - height : y0;
	s->af_window[0] = (float)x0;
	s->af_window[1] = (float)y0;
	s->af_window[2] = (float)(x0 + width);
	s->af_window[3] = (float)(y0 + height);
}

static OSStatus
stream_set_property(CMBaseObjectRef o, CFStringRef key, CFTypeRef value)
{
	struct stream *s = stream_of(o);

	if (key_is(key, "FormatIndex")) {
		int64_t i = 0;

		if (value != NULL && CFGetTypeID(value) == CFNumberGetTypeID()) {
			CFNumberGetValue(value, kCFNumberSInt64Type, &i);
		}
		if (i < 0 || i >= CFArrayGetCount(s->formats)) {
			return ERR_PARAM;
		}
		s->format_index = (int)i;
		say("stream: format %lld", (long long)i);
		return 0;
	}
	if (key_is(key, "ZoomFactor")) {
		/*
		 * The zoom: the camera's pictures -- moving and still alike -- made of the middle of the sensor
		 * (src/camera_isp.c, crop()), as the iPhone's ISP crops for it. Digital: no more detail than
		 * that part of the sensor has.
		 */
		double zoom = 1;

		if (value != NULL && CFGetTypeID(value) == CFNumberGetTypeID()) {
			CFNumberGetValue(value, kCFNumberDoubleType, &zoom);
		}
		zoom = zoom < 1 ? 1 : zoom > MAX_ZOOM ? MAX_ZOOM : zoom;
		if (fabs(zoom - s->zoom) >= 0.25 || (zoom == 1) != (s->zoom == 1)) {
			say("stream: zoom %.2fx", zoom);
		}
		s->zoom = (float)zoom;
		return 0;
	}
	if (key_is(key, "MinimumFrameRate") || key_is(key, "MaximumFrameRate")) {
		double fps = 0;

		if (value != NULL && CFGetTypeID(value) == CFNumberGetTypeID()) {
			CFNumberGetValue(value, kCFNumberDoubleType, &fps);
		}
		*(key_is(key, "MinimumFrameRate") ? &s->min_fps : &s->max_fps) = fps;
		say_once("stream: set", key, value);
		return 0;
	}
	if (key_is(key, "VideoOutputConfigurations") || key_is(key, "VideoOutputsEnabled") || key_is(key, "VideoOutputHandlers")) {
		bool configs = key_is(key, "VideoOutputConfigurations"), enabled = key_is(key, "VideoOutputsEnabled");

		say_once("stream: set", key, value);
		if (value == NULL || CFGetTypeID(value) != CFDictionaryGetTypeID()) {
			return 0;
		}
		pthread_mutex_lock(&s->lock);
		CFIndex n = CFDictionaryGetCount(value);
		const void **keys = calloc((size_t)n + 1, sizeof(void *)), **values = calloc((size_t)n + 1, sizeof(void *));

		CFDictionaryGetKeysAndValues(value, keys, values);
		for (CFIndex i = 0; i < n; i++) {
			int k = CFGetTypeID(keys[i]) == CFStringGetTypeID() ? output_index(keys[i]) : -1;
			struct output *out;

			if (k < 0) {
				say_once("stream: an output not here:", keys[i], NULL);
				continue;
			}
			out = &s->out[k];
			if (configs) {
				if (out->config != NULL) {
					CFRelease(out->config);
				}
				let_go(out);
				out->config = CFGetTypeID(values[i]) == CFDictionaryGetTypeID() ? CFRetain(values[i]) : NULL;
			} else if (enabled) {
				int64_t on = 1;

				if (CFGetTypeID(values[i]) == CFBooleanGetTypeID()) {
					on = CFBooleanGetValue(values[i]);
				} else if (CFGetTypeID(values[i]) == CFNumberGetTypeID()) {
					CFNumberGetValue(values[i], kCFNumberSInt64Type, &on);
				}
				out->enabled = on != 0;
			} else {
				CFTypeRef h = CFGetTypeID(values[i]) == CFDictionaryGetTypeID() ?
				    CFDictionaryGetValue(values[i], CFSTR("PixelBufferHandler")) : NULL;

				if (out->handler != NULL) {
					CFRelease(out->handler);
				}
				out->handler = h != NULL ? (void *)CFRetain(h) : NULL;
			}
		}
		free(keys);
		free(values);
		pthread_mutex_unlock(&s->lock);
		return 0;
	}
	if (key_is(key, "StillImageCaptureNow")) {
		/*
		 * The shutter: the capture graph's still image coordinator asks for a photograph by writing
		 * this, with what it wants of it (how many pictures, the flash, the settings' number by which
		 * it will know the frames again). One frame of the sensor goes to the StillImage output for
		 * each ask.
		 */
		pthread_mutex_lock(&s->lock);
		s->stills_wanted++;
		pthread_mutex_unlock(&s->lock);
		say_once("stream: a photograph asked for", key, value);
		return 0;
	}
	if (key_is(key, "VideoOutputsColorInfo")) {
		pthread_mutex_lock(&s->lock);
		if (s->colour_info != NULL) {
			CFRelease(s->colour_info);
		}
		s->colour_info = value != NULL && CFGetTypeID(value) == CFDictionaryGetTypeID() ? CFRetain(value) : NULL;
		pthread_mutex_unlock(&s->lock);
		return 0;
	}
	if (key_is(key, "TorchLevel")) {
		torch_set(s, value);
		return 0;
	}
	if (key_is(key, "AEMeteringMode") || key_is(key, "ExposureBias") ||
	    key_is(key, "LockAENow") || key_is(key, "UnlockAENow")) {
		pthread_mutex_lock(&s->controls_lock);
		if (key_is(key, "AEMeteringMode")) {
			s->ae_spot = value != NULL && CFGetTypeID(value) == CFStringGetTypeID() &&
			    CFEqual(value, CFSTR("MeteringModeSpot"));
		} else if (key_is(key, "ExposureBias")) {
			CFTypeRef gain_value = value, id_value = NULL;
			if (value != NULL && CFGetTypeID(value) == CFDictionaryGetTypeID()) {
				gain_value = CFDictionaryGetValue(value, CFSTR("ExposureBias"));
				id_value = CFDictionaryGetValue(value, CFSTR("CommandID"));
			}
			int32_t gain = 0, command_id = 0;
			if (id_value != NULL && CFGetTypeID(id_value) == CFNumberGetTypeID()) {
				(void)CFNumberGetValue(id_value, kCFNumberSInt32Type, &command_id);
			}
			if (gain_value == NULL || CFGetTypeID(gain_value) != CFNumberGetTypeID() ||
			    !CFNumberGetValue(gain_value, kCFNumberSInt32Type, &gain) || gain < 1 || gain > UINT16_MAX) {
				pthread_mutex_unlock(&s->controls_lock);
				return ERR_PARAM;
			}
			if (s->exposure_bias.gain != gain || s->exposure_bias.command_id != command_id) {
				say("exposure bias: %+.2f EV (gain %d/256, command %d)", log2(gain / 256.0), gain, command_id);
			}
			s->exposure_bias = (struct exposure_bias){ (uint16_t)gain, command_id };
		} else {
			s->ae_locked = key_is(key, "LockAENow") && value != kCFBooleanFalse;
		}
		pthread_mutex_unlock(&s->controls_lock);
		return 0;
	}
	if (key_is(key, "AutoFocusNow") || key_is(key, "ContinuousAutoFocusNow") || key_is(key, "FocusNow") ||
	    key_is(key, "PeakTrackingFocusNow") || key_is(key, "LockAFNow") || key_is(key, "ManualFocusNow") ||
	    key_is(key, "AFWindowParams") || key_is(key, "AutoFocusMode")) {
		/*
		 * Camera.app's tap: AutoFocusMode "AutoFocusModeWeightedSum", AFWindowParams an array of
		 * { hStart, vStart, width, height, weight } in thousandths of the frame (the first window is
		 * the one taken here; an empty value is the middle), then AutoFocusNow (true). LockAFNow
		 * keeps the lens where it is until focus is asked for again.
		 */
		if (!key_is(key, "AFWindowParams") || (value != NULL && CFGetTypeID(value) == CFArrayGetTypeID() &&
		    CFArrayGetCount(value) > 0)) {
			say_value("stream: focus:", key, value);        /* (an empty window, the middle: often, unsaid) */
		}
		pthread_mutex_lock(&s->controls_lock);
		if (key_is(key, "AFWindowParams")) {
			af_window(s, value);
		} else if (key_is(key, "LockAFNow")) {
			s->af_locked = value != kCFBooleanFalse;
			if (s->af_locked) {
				s->af_now = false;
			}
		} else if (key_is(key, "AutoFocusNow") || key_is(key, "ContinuousAutoFocusNow") || key_is(key, "FocusNow") ||
		    key_is(key, "PeakTrackingFocusNow")) {
			s->af_locked = false;
			if (value != kCFBooleanFalse) {
				s->af_now = true;
			}
		}
		pthread_mutex_unlock(&s->controls_lock);
		return 0;
	}
	say_once("stream: set (not used)", key, value);
	return 0;
}

/* the flashlight: TorchLevel (0 to 1) as thousandths to the kernel's /dev/hakios-torch (torch=3) */
static void
torch_set(struct stream *s, CFTypeRef value)
{
	float level = 0;
	uint32_t thousandths;

	if (value != NULL && CFGetTypeID(value) == CFNumberGetTypeID()) {
		CFNumberGetValue(value, kCFNumberFloatType, &level);
	}
	level = level < 0 ? 0 : level > 1 ? 1 : level;
	thousandths = (uint32_t)(level * 1000 + 0.5f);
	if (s->torch < 0) {
		s->torch = open("/dev/hakios-torch", O_WRONLY);
	}
	if (s->torch < 0 || write(s->torch, &thousandths, sizeof(thousandths)) != (ssize_t)sizeof(thousandths)) {
		say("the flashlight at %u of 1000 NOT TAKEN (no /dev/hakios-torch)", thousandths);
	} else {
		say("the flashlight at %u of 1000", thousandths);
	}
}

/* ---- the frames ------------------------------------------------------------------------------------- */

/*
 * The photograph through the camera ISP's BPS and IPE (/dev/hakios-icp, CONFIG_ICP): the shutter frame's RAW10 in,
 * the IPE's 4000 x 3008 NV12 back, with stock's processing settings (stock's white balance too, for now). NULL
 * when the kernel has no such service or this one failed: the IFE's own still is given then.
 */
struct icp_raw_header {
	char magic[8];
	uint32_t width, height, stride, reserved;
};
struct icp_nv12_header {
	char magic[8];
	uint32_t width, height, stride, luma, chroma, reserved[3];
};
static bool icp_absent, still_from_icp;

static const uint8_t *
icp_still(const struct hakios_camera_frame *f, const uint8_t *raw, struct hakios_camera_frame *icp_f)
{
	static uint8_t *image;
	static size_t capacity;
	if (icp_absent || f->format != 1 || f->width != 4000 || f->height != 3008) {
		return NULL;
	}
	int fd = open("/dev/hakios-icp", O_RDWR);
	if (fd < 0) {
		icp_absent = errno == ENOENT;
		say("the photograph: no ICP (%s), the IFE's still instead", strerror(errno));
		return NULL;
	}
	uint64_t began = mach_absolute_time();
	struct icp_raw_header h = { "ICPRAW10", f->width, f->height, f->stride, 0 };
	struct iovec iov[2] = { { &h, sizeof(h) }, { (void *)raw, (size_t)f->stride * f->height } };
	ssize_t wrote = writev(fd, iov, 2);
	const uint8_t *planes = NULL;
	if (wrote < 0) {
		say("the photograph: ICP failed (%s), the IFE's still instead", strerror(errno));
	} else {
		size_t want = sizeof(struct icp_nv12_header) + (size_t)4032 * 3008 * 3 / 2;
		if (capacity < want) {
			free(image);
			image = malloc(want);
			capacity = image ? want : 0;
		}
		ssize_t got = image ? pread(fd, image, capacity, 0) : -1;
		const struct icp_nv12_header *n = (const void *)image;
		if (got >= (ssize_t)sizeof(*n) && memcmp(n->magic, "NV12BPS1", 8) == 0 && n->width == f->width &&
		    n->height == f->height && n->stride >= n->width &&
		    (size_t)got >= sizeof(*n) + (size_t)n->luma + n->chroma && n->luma == n->stride * n->height &&
		    n->chroma == n->stride * (n->height / 2)) {
			*icp_f = *f;
			icp_f->preview_width = n->width;
			icp_f->preview_height = n->height;
			icp_f->preview_stride = n->stride;
			planes = image + sizeof(*n);
			static mach_timebase_info_data_t tb;
			if (tb.denom == 0) {
				mach_timebase_info(&tb);
			}
			say("the photograph: ICP BPS and IPE in %.0f ms", (double)(mach_absolute_time() - began) * tb.numer / tb.denom / 1e6);
		} else {
			say("the photograph: ICP image unreadable (%zd bytes), the IFE's still instead", got);
		}
	}
	close(fd);
	return planes;
}

struct render_job {
	const struct isp *isp;
	const struct hakios_camera_frame *f;
	const uint8_t *raw;
	CVPixelBufferRef pb;
	uint32_t format, width, height, pieces;
	bool still;                     /* the photograph: every pixel of the sensor (isp_render_still_nv12) */
	const uint8_t *preview;
	const uint8_t *preview_luma;
};

/* Resize the paired IFE NV12 frame into the graph's own buffer. Chroma is sampled as UV pairs. */
static void
render_preview(const struct render_job *j, const uint8_t *preview, uint32_t first, uint32_t end)
{
	const struct hakios_camera_frame *f = j->f;
	uint32_t sw = f->preview_width, sh = f->preview_height, stride = f->preview_stride;
	uint32_t cw = sw, ch = sh;
	if ((uint64_t)j->width * sh > (uint64_t)j->height * sw) {
		ch = (uint32_t)((uint64_t)sw * j->height / j->width);
	} else {
		cw = (uint32_t)((uint64_t)sh * j->width / j->height);
	}
	float zoom = fmaxf(1.0f, j->isp->zoom);
	cw = (uint32_t)fmaxf(2.0f, cw / zoom) & ~1u;
	ch = (uint32_t)fmaxf(2.0f, ch / zoom) & ~1u;
	uint32_t x0 = ((sw - cw) / 2) & ~1u, y0 = ((sh - ch) / 2) & ~1u;
	bool limited = j->format == PF_420V;

	for (unsigned plane = 0; plane < 2; plane++) {
		uint32_t scale = 1u << plane;
		uint32_t w = j->width / scale, h = j->height / scale;
		uint32_t src_w = cw / scale, src_h = ch / scale;
		uint32_t dx = (src_w << 16) / w;
		uint8_t *dst = CVPixelBufferGetBaseAddressOfPlane(j->pb, plane);
		size_t dst_stride = CVPixelBufferGetBytesPerRowOfPlane(j->pb, plane);
		const uint8_t *src = preview + (plane ? (size_t)stride * sh : 0) + (size_t)(y0 / scale) * stride + x0;
		for (uint32_t y = first / scale; y < end / scale; y++) {
			uint32_t sy = y * src_h / h;
			if (j->isp->turned) {
				sy = src_h - 1 - sy;
			}
			const uint8_t *row = src + (size_t)sy * stride;
			uint8_t *out = dst + (size_t)y * dst_stride;
			if (src_w == w && !j->isp->turned && (plane ? !limited : !j->preview_luma)) {
				memcpy(out, row, j->width);
				continue;
			}
			for (uint32_t x = 0, sx_fp = dx / 2; x < w; x++, sx_fp += dx) {
				uint32_t sx = sx_fp >> 16;
				if (j->isp->turned) {
					sx = src_w - 1 - sx;
				}
				if (plane == 0) {
					unsigned v = row[sx];
					out[x] = j->preview_luma ? j->preview_luma[v] : (uint8_t)v;
				} else {
					unsigned u = row[2 * sx], v = row[2 * sx + 1];
					out[2 * x] = limited ? (uint8_t)(16 + (224 * u + 127) / 255) : (uint8_t)u;
					out[2 * x + 1] = limited ? (uint8_t)(16 + (224 * v + 127) / 255) : (uint8_t)v;
				}
			}
		}
	}
}

static void
render_piece(void *context, size_t i)
{
	struct render_job *j = context;
	uint32_t rows = (j->height + j->pieces - 1) / j->pieces, first = (uint32_t)i * rows, end = first + rows;

	first &= ~1u;
	end = i + 1 == j->pieces ? j->height : end & ~1u;
	if (j->preview) {
		render_preview(j, j->preview, first, end);
	} else if (j->format == PF_BGRA) {
		isp_render_bgra(j->isp, j->f, j->raw, CVPixelBufferGetBaseAddress(j->pb), CVPixelBufferGetBytesPerRow(j->pb), j->width,
		    j->height, first, end);
	} else if (j->still) {
		isp_render_still_nv12(j->isp, j->f, j->raw, CVPixelBufferGetBaseAddressOfPlane(j->pb, 0),
		    CVPixelBufferGetBytesPerRowOfPlane(j->pb, 0), CVPixelBufferGetBaseAddressOfPlane(j->pb, 1),
		    CVPixelBufferGetBytesPerRowOfPlane(j->pb, 1), j->width, j->height, first, end, j->format == PF_420V);
	} else {
		isp_render_nv12(j->isp, j->f, j->raw, CVPixelBufferGetBaseAddressOfPlane(j->pb, 0), CVPixelBufferGetBytesPerRowOfPlane(j->pb, 0),
		    CVPixelBufferGetBaseAddressOfPlane(j->pb, 1), CVPixelBufferGetBytesPerRowOfPlane(j->pb, 1), j->width, j->height, first, end,
		    j->format == PF_420V);
	}
}

static CVPixelBufferRef
output_buffer(struct output *out)
{
	CVPixelBufferPoolRef pool = NULL;
	CVPixelBufferRef pb = NULL;
	CFTypeRef value = out->config != NULL ? CFDictionaryGetValue(out->config, CFSTR("BufferPool")) : NULL;

	/*
	 * The graph makes the pool itself and hands it over in the output's configuration
	 * (kFigCaptureStreamVideoOutputConfigurationKey_BufferPool), sized and laid out as the node
	 * behind the output wants it (an IOSurface each, extra rows at the bottom where a scaler wants
	 * them, as few buffers as the node holds at once) -- there is nothing for this side to choose,
	 * and a buffer of our own making is not one the graph would take. The configuration carries no
	 * width or height: the pool's buffers have them.
	 */
	if (value != NULL && CFGetTypeID(value) == CVPixelBufferPoolGetTypeID()) {
		pool = (CVPixelBufferPoolRef)value;
	}
	if (pool == NULL) {
		if (!out->said_shape) {
			out->said_shape = true;
			say("%s: no pool of the graph's in the configuration: no pictures", out->name);
		}
		return NULL;
	}
	for (unsigned i = 0; CVPixelBufferPoolCreatePixelBuffer(NULL, pool, &pb) != kCVReturnSuccess && i < HELD; i++) {
		/* a pool that will not grow that far: the oldest picture kept is the one to have */
		unsigned oldest = (out->held_next + i) % HELD;

		if (out->held[oldest] != NULL) {
			CVPixelBufferRelease(out->held[oldest]);
			out->held[oldest] = NULL;
		}
		if (!out->said_pool) {
			out->said_pool = true;
			say("%s: the graph's pool will not grow by %u: fewer pictures kept", out->name, HELD);
		}
	}
	if (pb == NULL) {
		/*
		 * The preview pool holds one buffer (MinimumBufferCount 1): while the graph still has it,
		 * there is none to be had, and this frame is not given out. Said at most every 300th frame.
		 */
		if (out->refused++ % 300 == 0) {
			say("%s: the graph's pool has no buffer to spare (%llu frames so)", out->name,
			    (unsigned long long)out->refused);
		}
		return NULL;
	}
	return pb;
}

typedef void (^pixel_buffer_handler)(CVPixelBufferRef, CMTime);

static void
let_go(struct output *out)
{
	for (unsigned i = 0; i < HELD; i++) {
		if (out->held[i] != NULL) {
			CVPixelBufferRelease(out->held[i]);
			out->held[i] = NULL;
		}
	}
}

static void
md_put(CFMutableDictionaryRef m, int key, CFTypeRef value)
{
	if (md_key[key] != NULL && value != NULL) {
		CFDictionarySetValue(m, md_key[key], value);
	}
	if (value != NULL) {
		CFRelease(value);
	}
}

/* the frame's metadata, as H10ISP's stream gives them (see md_names) */
static CFDictionaryRef
frame_metadata(const struct isp *isp, const struct hakios_camera_frame *f, const struct isp_stats *st, CFStringRef port,
    bool ae_locked, bool af_locked, bool focusing, struct exposure_bias bias)
{
	CFMutableDictionaryRef m;
	double exposure = f->exposure_lines * (double)f->line_ns / 1e9, analog = f->gain_unity ? (double)f->gain / f->gain_unity : 1;
	double total = analog * isp->digital, fps = f->frame_lines && f->line_ns ? 1e9 / ((double)f->frame_lines * f->line_ns) : 30;
	/* (green's level for a second at 1x, a quarter of it: a room's 70 or so, daylight's tens of thousands) */
	double lux = exposure > 0 && total > 0 ? st->mean[1] / (exposure * total) / 4 : 0;

	if (md_attachment == NULL) {
		return NULL;
	}
	m = (CFMutableDictionaryRef)new_dict();
	md_put(m, MD_PORT, CFRetain(port));
	md_put(m, MD_PTS, CMTimeCopyAsDictionary(CMClockMakeHostTimeFromSystemUnits(f->time), NULL));
	md_put(m, MD_EXPOSURE, number_f(exposure));
	md_put(m, MD_AGC, number_i((int64_t)(analog * 256 + 0.5)));
	md_put(m, MD_ISP_DGAIN, number_i((int64_t)(isp->digital * 256 + 0.5)));
	md_put(m, MD_SENSOR_DGAIN, number_i(256));
	md_put(m, MD_ISO_BASE, number_f(32));
	md_put(m, MD_FRAME_RATE, number_f(fps));
	md_put(m, MD_LUX, number_i((int64_t)(lux + 0.5)));
	md_put(m, MD_SNR, number_f(40 - 10 * log10(total < 1 ? 1 : total)));
	/* Frame statistics use AFStatus 2 for an active scan, then 1 for the settled lens. */
	md_put(m, MD_AF_STATUS, number_i(f->focus == HAKIOS_CAMERA_NO_FOCUS ? 0 : focusing ? 2 : 1));
	md_put(m, MD_FOCUSING, number_i(0));
	md_put(m, MD_AE_STABLE, CFRetain(ae_locked || !isp->asked ? kCFBooleanTrue : kCFBooleanFalse));
	md_put(m, MD_AE_LOCKED, CFRetain(ae_locked ? kCFBooleanTrue : kCFBooleanFalse));
	md_put(m, MD_EXPOSURE_BIAS, number_i(bias.gain));
	if (bias.command_id != 0) {
		md_put(m, MD_EXPOSURE_BIAS_ID, number_i(bias.command_id));
	}
	md_put(m, MD_AWB_STABLE, CFRetain(kCFBooleanTrue));
	md_put(m, MD_AWB_LOCKED, CFRetain(kCFBooleanFalse));
	md_put(m, MD_AF_LOCKED, CFRetain(af_locked ? kCFBooleanTrue : kCFBooleanFalse));
	/*
	 * The zoom the picture was made with: the source node takes it for what the ISP applied
	 * (-[BWMultiStreamCameraSourceNode _ispAppliedZoomFactorFromSampleBufferMetadataDictionary:outputIndex:])
	 * and would make up the rest of what was asked for itself; without it, a zoomed preview stood still.
	 */
	md_put(m, MD_ZOOM, number_f(isp->zoom));
	return m;
}

static bool
deliver(struct stream *s, struct output *out, const struct isp *isp, const struct hakios_camera_frame *f, const uint8_t *raw,
    const uint8_t *preview, CFDictionaryRef metadata)
{
	CVPixelBufferRef pb = output_buffer(out);
	uint32_t pf, width, height;

	if (pb == NULL) {
		return false;
	}
	pf = CVPixelBufferGetPixelFormatType(pb);
	width = (uint32_t)CVPixelBufferGetWidth(pb);
	height = (uint32_t)CVPixelBufferGetHeight(pb);
	if (width == 0 || height == 0 || (pf != PF_420F && pf != PF_420V && pf != PF_BGRA)) {
		if (out->delivered == 0 && out->said_shape == false) {
			out->said_shape = true;
			say("%s: %ux%u %c%c%c%c buffers are not for this ISP", out->name, width, height, pf >> 24, pf >> 16 & 255,
			    pf >> 8 & 255, pf & 255);
		}
		CVPixelBufferRelease(pb);
		return false;
	}
	CVPixelBufferLockBaseAddress(pb, 0);

	const bool still = strcmp(out->name, "StillImage") == 0;
	const bool use_preview = preview && pf != PF_BGRA && width >= 2 && height >= 2 && !(width & 1) && !(height & 1);
	uint8_t luma[256];
	bool remap = use_preview && (isp->digital != 1 || pf == PF_420V);
	if (remap) {
		/* IFE luma is already gamma encoded. Approximate digital gain in linear light. */
		for (unsigned i = 0; i < sizeof(luma); i++) {
			unsigned v = i;
			if (isp->digital != 1) {
				float y = i / 255.0f;
				float linear = y <= 0.04045f ? y / 12.92f : powf((y + 0.055f) / 1.055f, 2.4f);
				linear = fminf(1, linear * isp->digital);
				y = linear <= 0.0031308f ? 12.92f * linear : 1.055f * powf(linear, 1 / 2.4f) - 0.055f;
				v = (unsigned)(y * 255 + 0.5f);
			}
			luma[i] = pf == PF_420V ? (uint8_t)(16 + (219 * v + 127) / 255) : (uint8_t)v;
		}
	}
	struct render_job job = { isp, f, raw, pb, pf, width, height, 8,
	    still && pf != PF_BGRA, use_preview ? preview : NULL, remap ? luma : NULL };
	uint64_t began = mach_absolute_time();

	dispatch_apply_f(job.pieces, dispatch_get_global_queue(QOS_CLASS_USER_INTERACTIVE, 0), &job, render_piece);
	{
		static mach_timebase_info_data_t tb;
		uint64_t elapsed_ns;

		if (tb.denom == 0) {
			mach_timebase_info(&tb);
		}
		elapsed_ns = (uint64_t)((double)(mach_absolute_time() - began) * tb.numer / tb.denom);
		out->render_ns += elapsed_ns;
		out->render_count++;
		if (still) {
			say("%s: %ux%u via %s in %.0f ms", out->name, width, height,
			    !use_preview ? "CPU ISP" : still_from_icp ? "ICP NV12" : "native IFE NV12", (double)elapsed_ns / 1e6);
		}
	}
	if (use_preview && out == &s->out[0] && (out->delivered == 0 || isp->exposure_bias != out->measured_bias)) {
		const uint8_t *luma_plane = CVPixelBufferGetBaseAddressOfPlane(pb, 0);
		size_t stride = CVPixelBufferGetBytesPerRowOfPlane(pb, 0);
		uint64_t sum = 0, count = 0;
		for (uint32_t y = 0; y < height; y += 32) {
			const uint8_t *row = luma_plane + (size_t)y * stride;
			for (uint32_t x = 0; x < width; x += 32) {
				sum += row[x];
				count++;
			}
		}
		out->measured_bias = isp->exposure_bias;
		say("preview output: mean Y %.1f/255 at %+.2f EV (digital gain %.2fx)", (double)sum / count,
		    isp->exposure_bias, isp->digital);
	}
	CVPixelBufferUnlockBaseAddress(pb, 0);
	CVBufferSetAttachment(pb, kCVImageBufferYCbCrMatrixKey, kCVImageBufferYCbCrMatrix_ITU_R_601_4, kCVAttachmentMode_ShouldPropagate);
	CVBufferSetAttachment(pb, kCVImageBufferColorPrimariesKey, kCVImageBufferColorPrimaries_ITU_R_709_2, kCVAttachmentMode_ShouldPropagate);
	CVBufferSetAttachment(pb, kCVImageBufferTransferFunctionKey, kCVImageBufferTransferFunction_ITU_R_709_2,
	    kCVAttachmentMode_ShouldPropagate);

	if (metadata != NULL) {
		CFMutableDictionaryRef m = CFDictionaryCreateMutableCopy(NULL, 0, metadata);

		CVBufferSetAttachment(pb, md_attachment, m, kCVAttachmentMode_ShouldPropagate);
		CFRelease(m);
	}

	CMTime pts = CMClockMakeHostTimeFromSystemUnits(f->time);

	((pixel_buffer_handler)out->handler)(pb, pts);
	/*
	 * The moving picture's buffers are kept for HELD frames after they are given. The preview's pool
	 * would otherwise hand a buffer straight back -- the graph lets go of it as soon as it has passed
	 * it on -- while it is still on its way to the screen (the preview sink, Camera.app's image queue,
	 * the window server copying it), and the next frame drawn into it showed as bands of two frames,
	 * torn across. Kept, they make the pool take others. (One kept was not enough.)
	 */
	if (!still) {
		if (out->held[out->held_next] != NULL) {
			CVPixelBufferRelease(out->held[out->held_next]);
		}
		out->held[out->held_next] = pb;
		out->held_next = (out->held_next + 1) % HELD;
	} else {
		CVPixelBufferRelease(pb);
	}
	if (out->delivered++ == 0) {
		say("%s: the first picture given (%ux%u %c%c%c%c, %s)", out->name, width, height, pf >> 24, pf >> 16 & 255,
		    pf >> 8 & 255, pf & 255, use_preview ? "IFE NV12" : "CPU ISP");
	}
	return true;
}

/*
 * The scene's light as the cameras' tuning measures it (Qualcomm's lux index, higher is darker): about
 * 46 a doubling of exposure times gain, 197 at a 33 ms exposure and 1x -- the black level's ranges in
 * the tuning of the wide camera pair lux indexes with gains, 195-200 with 1-1.1x and 380-385 with
 * 15.9-16x, at the longest exposure a frame at 30 a second allows.
 */
static float
lux_index(const struct isp *isp, const struct hakios_camera_frame *f)
{
	double exposure = f->exposure_lines * (double)f->line_ns / 1e9 * f->gain / f->gain_unity * isp->digital;

	return (float)(197 + 46 * log2(exposure > 1e-6 ? exposure / 0.0333 : 1e-6 / 0.0333));
}

/*
 * The light's colour temperature: where the white balance is on the line of the tuning's reference
 * lights; without them, guessed from how much more the blue is raised than the red, by what is typical of a phone's sensor --
 * blue's gain over red's about 0.8 in daylight (6500 K), 1.3 under a fluorescent tube (4000 K), 2 under
 * a filament lamp (2850 K) -- between those on the logarithm of the ratio, and no further.
 */
static float
colour_temperature(const struct isp *isp)
{
	static const float ratio[3] = { 0.8f, 1.3f, 2.0f }, kelvin[3] = { 6500, 4000, 2850 };
	float r = isp->wb[0] > 0 ? isp->wb[2] / isp->wb[0] : 1, cct = isp_cct(isp);

	if (cct > 0) {
		return cct;     /* (by the tuning's reference lights) */
	}

	if (r <= ratio[0]) {
		return kelvin[0];
	}
	for (int i = 1; i < 3; i++) {
		if (r <= ratio[i]) {
			float t = logf(r / ratio[i - 1]) / logf(ratio[i] / ratio[i - 1]);

			return kelvin[i - 1] + t * (kelvin[i] - kelvin[i - 1]);
		}
	}
	return kelvin[2];
}

/*
 * Focus, where the camera has a lens motor (the frames' focus is a position, not HAKIOS_CAMERA_NO_FOCUS:
 * the wide camera's): by contrast, as there is nothing else to go by -- the sensor's phase-detection
 * pixels are not read. A search climbs from where the lens is: a step of AF_STEP one way, on that way
 * while the middle of the picture gets sharper, back the other way if the first step was not, until
 * two steps in a row are less sharp than the sharpest (or the travel, AF_NEAR to AF_FAR, ends); then
 * around the sharpest in steps of AF_FINE, and there it stays. A position is measured once the frame
 * metadata confirms the lens got there and it had time to settle. The lens is only ever moved a step at a
 * time: thrown across its travel it knocked, audibly, against its stop at 0. In focus, it stays until
 * the middle's contrast or brightness changes for about half a second and stays steady for another
 * quarter second: no searching while the phone is being moved.
 * A focus asked for (a tap: AutoFocusNow, on the window AFWindowParams gave) sweeps instead: the
 * lens is walked to the nearer end of its travel, then measured towards the other in steps of
 * AF_STEP until it is past the sharpest (two steps in a row well under it), then around the sharpest
 * -- it finds what was tapped however far out of focus it was, where the climb gave up on the flat
 * of the curve.
 */
enum { AF_WATCH, AF_STEADY, AF_CLIMB, AF_SWEEP, AF_FINE_SEARCH, AF_SETTLE };
/*
 * The travel: the whole of 0..1023 (the IMX789's AK7377 codes; it focuses far at about 200 and near
 * towards 900), except on the POCO's s5khm2sd lens: 16 (off the stop at 0) to 768 (its module file's actuator region
 * goes to 700; near 1023 the PM8008i's LDO5 said over-current).
 */
static int AF_NEAR = 0, AF_FAR = 1023;
static const char *sensor_name_for(uint32_t sensor);
#define AF_STEP         48
#define AF_FINE         12

struct af {
	int             state;
	bool            known;          /* at has been taken from the frames */
	uint32_t        at, target;     /* confirmed position and final target */
	uint32_t        commanded;
	unsigned        waiting;        /* frame reads since a position was requested */
	bool            pending;
	int             way;            /* +1 nearer, -1 further */
	bool            turned;
	unsigned        worse;          /* steps in a row less sharp than the sharpest */
	uint32_t        start, best, fine_to;
	float           best_sharpness;
	float           sharpness, level;       /* in focus: the middle's, to watch */
	float           last_sharpness, last_level;
	uint64_t        unlike_ns, steady_ns;
	unsigned        steps;
	bool            now;            /* asked for: no waiting for the picture to keep still */
	bool            sweep;          /* ... and the whole travel measured */
	float           sum_sharpness, sum_level;
	unsigned        measured;
};

static void
af_go(int device, struct af *af, uint32_t target)
{
	struct hakios_camera_command c = { .command = HAKIOS_CAMERA_FOCUS };

	af->target = target < AF_NEAR ? AF_NEAR : target > AF_FAR ? AF_FAR : target;
	c.a = af->at < af->target ? (af->target - af->at > AF_STEP ? af->at + AF_STEP : af->target) :
	    (af->at - af->target > AF_STEP ? af->at - AF_STEP : af->target);
	af->commanded = c.a;
	af->pending = true;
	af->waiting = 0;
	af->steps++;
	af->sum_sharpness = af->sum_level = 0;
	af->measured = 0;
	(void)write(device, &c, sizeof(c));
}

static void
af_frame(int device, struct af *af, const struct hakios_camera_frame *f, const uint8_t *raw, const float window[4], bool locked,
    bool requested)
{
	float level, sharpness;
	uint64_t frame_ns = (uint64_t)f->frame_lines * f->line_ns;
	unsigned needed = f->gain >= 16 * f->gain_unity ? 2 : 1;      /* (in the dark, two frames to a measurement) */

	if (f->focus == HAKIOS_CAMERA_NO_FOCUS) {
		return;
	}
	{
		const char *name = sensor_name_for(f->sensor);
		bool full = name == NULL || strcmp(name, "s5khm2sd") != 0;
		static int said_travel = -1;

		AF_NEAR = full ? 0 : 16;
		AF_FAR = full ? 1023 : 768;
		if (said_travel != (int)full) {
			said_travel = full;
			say("focus: the %s lens (sensor %u): travel %d..%d", name ? name : "unnamed", f->sensor, AF_NEAR, AF_FAR);
		}
	}
	af->at = f->focus;
	if (locked) {
		af->known = false;
		af->pending = false;
		af->state = AF_WATCH;
		return;
	}
	if (!af->known || requested) {
		af->known = true;
		af->target = af->at;
		af->pending = false;
		af->sum_sharpness = af->sum_level = 0;
		af->measured = 0;
	}
	if (requested) {
		af->state = AF_STEADY;
		af->steady_ns = 0;
		af->now = true;
		af->sweep = true;
	}
	if (af->pending) {
		if (af->at != af->commanded) {
			if (++af->waiting < 5) {
				return;
			}
			say("focus: lens stayed at %u when asked for %u", af->at, af->commanded);
			af->target = af->at;
			af->state = AF_WATCH;
			af->pending = false;
			return;
		}
		af->pending = false;
		if (af->waiting == 0 && af->at == af->target) {
			return; /* give the lens a frame to settle after the move */
		}
	}
	if (af->at != af->target) {
		af_go(device, af, af->target);          /* (the next step of a longer way) */
		return;
	}
	sharpness = isp_sharpness(f, raw, window[0], window[1], window[2], window[3], &level);
	af->sum_sharpness += sharpness;
	af->sum_level += level;
	if (++af->measured < needed) {
		return;
	}
	sharpness = af->sum_sharpness / af->measured;
	level = af->sum_level / af->measured;
	af->sum_sharpness = af->sum_level = 0;
	af->measured = 0;
	switch (af->state) {
	case AF_WATCH:
		if (af->sharpness > 0 && sharpness > af->sharpness * 0.72f && sharpness < af->sharpness * 1.6f &&
		    fabsf(level - af->level) < fmaxf(4.0f, af->level * 0.35f)) {
			af->unlike_ns = 0;
			return;
		}
		af->unlike_ns += frame_ns * needed;
		if (af->sharpness > 0 && af->unlike_ns < 550000000) {
			return;
		}
		af->state = AF_STEADY;
		af->steady_ns = 0;
		af->last_sharpness = sharpness;
		af->last_level = level;
		return;
	case AF_STEADY:
		if (fabsf(sharpness - af->last_sharpness) < fmaxf(0.01f, 0.20f * af->last_sharpness) &&
		    fabsf(level - af->last_level) < fmaxf(3.0f, 0.15f * af->last_level)) {
			af->steady_ns += frame_ns * needed;
		} else {
			af->steady_ns = 0;
		}
		af->last_sharpness = sharpness;
		af->last_level = level;
		if (af->steady_ns < 280000000 && !af->now) {
			return;
		}
		af->now = false;
		if (af->sweep) {
			af->sweep = false;
			af->state = AF_SWEEP;
			af->best_sharpness = -1;
			af->worse = 0;
			af->steps = 0;
			af->way = af->at - AF_NEAR < AF_FAR - af->at ? 1 : -1;
			af_go(device, af, af->way > 0 ? AF_NEAR : AF_FAR);     /* (walked there a step at a time) */
			return;
		}
		af->state = AF_CLIMB;
		say("focus: continuous scan from %u (sharpness %.3f)", af->at, sharpness);
		af->best = af->start = af->at;
		af->best_sharpness = sharpness;
		af->way = af->at + AF_STEP > AF_FAR ? -1 : 1;
		af->turned = false;
		af->worse = 0;
		af->steps = 0;
		af_go(device, af, (uint32_t)((int)af->at + af->way * AF_STEP));
		return;
	case AF_CLIMB: {
		int next = (int)af->at + af->way * AF_STEP;
		bool end = next < AF_NEAR || next > AF_FAR;

		if (sharpness > af->best_sharpness * 1.02f) {
			af->best = af->at;
			af->best_sharpness = sharpness;
			af->worse = 0;
		} else {
			af->worse++;
		}
		if (af->worse < 2 && !end) {
			af_go(device, af, (uint32_t)next);
			return;
		}
		next = (int)af->best - af->way * AF_STEP;
		if (!af->turned && af->best == af->start && next >= AF_NEAR && next <= AF_FAR) {
			/* not sharper that way at all: the other way, from where it started */
			af->turned = true;
			af->way = -af->way;
			af->worse = 0;
			af_go(device, af, (uint32_t)next);
			return;
		}
		af->state = AF_FINE_SEARCH;
		af->fine_to = af->best + 2 * AF_FINE > AF_FAR ? AF_FAR : af->best + 2 * AF_FINE;
		af_go(device, af, af->best > 2 * AF_FINE + AF_NEAR ? af->best - 2 * AF_FINE : AF_NEAR);
		return;
	}
	case AF_SWEEP:
		say("focus: sweep: %u %.4f", af->at, sharpness);
		if (sharpness > af->best_sharpness) {
			af->best = af->at;
			af->best_sharpness = sharpness;
			af->worse = 0;
		} else if (sharpness < af->best_sharpness * 0.85f) {
			af->worse++;
		}
		if (af->worse < 2 && (int)af->at + af->way * AF_STEP >= AF_NEAR && (int)af->at + af->way * AF_STEP <= AF_FAR) {
			af_go(device, af, (uint32_t)((int)af->at + af->way * AF_STEP));
			return;
		}
		af->state = AF_FINE_SEARCH;
		af->fine_to = af->best + 2 * AF_FINE > AF_FAR ? AF_FAR : af->best + 2 * AF_FINE;
		af_go(device, af, af->best > 2 * AF_FINE + AF_NEAR ? af->best - 2 * AF_FINE : AF_NEAR);
		return;
	case AF_FINE_SEARCH:
		if (sharpness > af->best_sharpness) {
			af->best = af->at;
			af->best_sharpness = sharpness;
		}
		if (af->at + AF_FINE <= af->fine_to) {
			af_go(device, af, af->at + AF_FINE);
			return;
		}
		af->state = AF_SETTLE;
		af_go(device, af, af->best);
		return;
	case AF_SETTLE:
		af->state = AF_WATCH;
		af->sharpness = sharpness;
		af->level = level;
		af->unlike_ns = 0;
		say("focus: the lens at %u after %u moves (sharpness %.3f)", af->at, af->steps, sharpness);
		return;
	}
}

/*
 * Which cameras the kernel actually serves and their sensor names, from kern.hakios.cameras
 * ("sensor:name ..." e.g. "1:imx789"). When the sysctl is absent (older/redwood kernels), keep the
 * original POCO X5 Pro defaults (back s5khm2sd and front ov16a1q).
 */
static uint32_t kernel_sensor_mask;
static char kernel_sensor_names[4][32];
static char kernel_device_desc[128];

static void
discover_cameras(void)
{
	char buf[128];
	size_t len = sizeof(buf) - 1;

	memset(kernel_sensor_names, 0, sizeof(kernel_sensor_names));
	kernel_sensor_mask = 0;
	if (sysctlbyname("kern.hakios.cameras", buf, &len, NULL, 0) == 0 && len > 0) {
		buf[len] = '\0';
		char *p = buf;
		while (*p != '\0') {
			while (*p == ' ' || *p == '\t' || *p == '\n') {
				p++;
			}
			if (*p == '\0') {
				break;
			}
			char *end = NULL;
			unsigned long id = strtoul(p, &end, 10);
			if (end != p && *end == ':' && id < 4) {
				const char *name = end + 1;
				size_t n = 0;
				while (name[n] != '\0' && name[n] != ' ' && name[n] != '\t' && name[n] != '\n' && n + 1 < sizeof(kernel_sensor_names[0])) {
					kernel_sensor_names[id][n] = name[n];
					n++;
				}
				kernel_sensor_names[id][n] = '\0';
				if (n > 0) {
					kernel_sensor_mask |= (1u << id);
				}
				p = (char *)(name + n);
			}
			while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\n') {
				p++;
			}
		}
	} else {
		static const char *const fallback[4] = { "ov02b10", "s5khm2sd", "ov16a1q", "s5k4h7" };
		for (int i = 0; i < 4; i++) {
			strlcpy(kernel_sensor_names[i], fallback[i], sizeof(kernel_sensor_names[i]));
		}
		kernel_sensor_mask = (1u << 1) | (1u << 2);
	}
	size_t pos = (size_t)snprintf(kernel_device_desc, sizeof(kernel_device_desc), "<HakiosCaptureDevice:");
	bool any = false;
	for (int i = 0; i < 4; i++) {
		if ((kernel_sensor_mask & (1u << i)) != 0 && kernel_sensor_names[i][0] != '\0' && pos + 2 < sizeof(kernel_device_desc)) {
			pos += (size_t)snprintf(kernel_device_desc + pos, sizeof(kernel_device_desc) - pos, "%s %s", any ? "," : "",
			    kernel_sensor_names[i]);
			any = true;
		}
	}
	snprintf(kernel_device_desc + pos, sizeof(kernel_device_desc) - pos, "%s>", any ? "" : " none");
}

static const char *
sensor_name_for(uint32_t sensor)
{
	if (sensor < 4 && kernel_sensor_names[sensor][0] != '\0') {
		return kernel_sensor_names[sensor];
	}
	return NULL;
}

/* the sensor's colour tuning, when the image has one (fw/camera-<sensor>.isp, tools/camtuning.py) */
static void
load_tuning(struct isp *isp, uint32_t sensor)
{
	const char *name = sensor_name_for(sensor);
	char path[96];

	if (name == NULL) {
		return;
	}
	snprintf(path, sizeof(path), "/redwood/fw/camera-%s.isp", name);
	if (isp_load_tuning(isp, path)) {
		say("the %s's colour tuning from %s: %u colour matrices' lights, %u lens shadings'", name, path,
		    isp->tuning->ccm.lights, isp->tuning->lsc.lights);
	} else {
		say("no colour tuning for the %s (%s): no colour matrix, no lens shading", name, path);
		isp_set_ccm(isp, (const float[9]){ 1, 0, 0, 0, 1, 0, 0, 0, 1 });
		isp->lsc.on = false;
	}
}

static void *
worker(void *context)
{
	CMBaseObjectRef o = context;
	struct stream *s = stream_of(o);
	struct hakios_camera_frame head;
	struct isp isp;
	struct isp_stats st;
	uint8_t *buffer = NULL;
	size_t bytes = 0;
	uint32_t tuned_for = ~0u;
	bool switched = false, has_preview = false, preview_using_raw = false;
	float lux = -1, cct = -1;
	struct af af = { 0 };
	struct exposure_bias current_bias = { .gain = 256 }, applied_bias = current_bias;
	uint64_t bias_sequence = 0;
	float locked_base_bias = 0;
	bool was_ae_locked = false;
	int device = open(HAKIOS_CAMERA_DEVICE, O_RDWR), failures = 0;

	pthread_setname_np("hakios-camera");
	if (device < 0) {
		say("no %s (%s): no frames", HAKIOS_CAMERA_DEVICE, strerror(errno));
		return NULL;
	}
	int tap_token = -1, tap_changed;
	const char *sensor_name = sensor_name_for(s->camera->sensor);
	if (sensor_name != NULL && strcmp(sensor_name, "imx789") == 0 &&
	    notify_register_check(HAKIOS_CAMERA_TAP_NAME, &tap_token) == NOTIFY_STATUS_OK) {
		(void)notify_check(tap_token, &tap_changed); /* drain the initial notification */
	}
	isp_init(&isp);
	while (s->running && failures < 5) {
		ssize_t n;

		if (buffer == NULL) {
			n = read(device, &head, sizeof(head));
			if (n != (ssize_t)sizeof(head)) {
				failures += n < 0 && errno == EAGAIN ? 0 : 1;
				if (n < 0 && errno == EIO) {
					say("the kernel's camera has had a fault: no frames");
					break;
				}
				continue;
			}
			if (head.sensor != s->camera->sensor) {
				/* another camera's frames (the kernel's last sensor): changed, and the next ones are this one's */
				if (!switched) {
					struct hakios_camera_command c = { .command = HAKIOS_CAMERA_SENSOR, .a = s->camera->sensor, .b = ~0u };

					switched = true;
					say("the kernel's frames are camera %u's: to %u, the %s", head.sensor, s->camera->sensor, s->camera->name);
					(void)write(device, &c, sizeof(c));
				}
				continue;
			}
			bytes = sizeof(head) + (size_t)head.stride * head.height;
			if (head.preview_width >= 2 && !(head.preview_width & 1) && head.preview_height >= 2 &&
			    !(head.preview_height & 1) && head.preview_stride >= head.preview_width) {
				bytes += (size_t)head.preview_stride * head.preview_height * 3 / 2;
				has_preview = true;
			}
			buffer = malloc(bytes);
			if (buffer == NULL) {
				say("could not allocate %zu bytes for a camera frame", bytes);
				break;
			}
			say("frames of %ux%u from the kernel (the %s)", head.width, head.height, s->camera->name);
			continue;
		}
		n = read(device, buffer, bytes);
		if (n != (ssize_t)bytes) {
			if (n < 0 && errno == EIO) {
				say("the kernel's camera has had a fault: no more frames");
				break;
			}
			failures += n < 0 && errno == EAGAIN ? 0 : 1;
			continue;
		}
		failures = 0;

		const struct hakios_camera_frame *f = (const void *)buffer;
		const uint8_t *raw = buffer + f->header_bytes;
		const uint8_t *preview = has_preview ? raw + (size_t)f->stride * f->height : NULL;
		bool native_ife = preview != NULL && f->preview_width == f->width && f->preview_height == f->height;
		struct hakios_camera_command c;

		float window[4];
		pthread_mutex_lock(&s->controls_lock);
		uint64_t tap;
		if (tap_token >= 0 && notify_check(tap_token, &tap_changed) == NOTIFY_STATUS_OK && tap_changed &&
		    notify_get_state(tap_token, &tap) == NOTIFY_STATUS_OK) {
			float x = (float)(tap >> 16) / HAKIOS_CAMERA_TAP_SCALE;
			float y = (float)(tap & HAKIOS_CAMERA_TAP_SCALE) / HAKIOS_CAMERA_TAP_SCALE;
			if (x <= 1) {
				/* The same 187 x 250 thousandths window that the capture service uses for a tap. */
				s->af_window[0] = fminf(0.813f, fmaxf(0, x - 0.0935f));
				s->af_window[1] = fminf(0.75f, fmaxf(0, y - 0.125f));
				s->af_window[2] = s->af_window[0] + 0.187f;
				s->af_window[3] = s->af_window[1] + 0.25f;
				s->af_now = true;
				s->af_locked = s->ae_locked = false;
				s->ae_spot = true;
				say("focus: repeated tap at %.3f %.3f", x, y);
			}
		}
		memcpy(window, s->af_window, sizeof(window));
		bool af_now = s->af_now, af_locked = s->af_locked, ae_locked = s->ae_locked, ae_spot = s->ae_spot;
		s->af_now = false;
		struct exposure_bias requested_bias = s->exposure_bias;
		pthread_mutex_unlock(&s->controls_lock);

		isp_measure(&isp, f, raw, ae_spot ? window : NULL, &st);
		struct isp_stats preview_stats = { 0 };
		if (preview != NULL) {
			isp_measure_preview(&isp, f, preview, ae_spot ? window : NULL, &preview_stats);
			bool overbright = isp_preview_overbright(&st, &preview_stats, preview_using_raw);
			if (overbright != preview_using_raw) {
				say("preview: switching to %s (IFE/RAW10 mean %.0f/%.0f, near white %.1f%%/%.1f%%)",
				    overbright ? "RAW10" : "IFE", preview_stats.mean[1], st.mean[1], 100 * preview_stats.near_white,
				    100 * st.near_white);
				preview_using_raw = overbright;
			}
		}
		isp_awb(&isp, &st);
		if (f->sensor != tuned_for) {
			tuned_for = f->sensor;
			load_tuning(&isp, f->sensor);
		}
		/*
		 * Slowly, a second or so: the colour temperature is guessed from the white balance, the tuning it
		 * picks changes what the white balance sees, and the colour matrix makes every swing of it plain.
		 */
		lux = lux < 0 ? lux_index(&isp, f) : lux * 0.9f + lux_index(&isp, f) * 0.1f;
		cct = cct < 0 ? colour_temperature(&isp) : cct * 0.97f + colour_temperature(&isp) * 0.03f;
		isp_tune(&isp, lux, cct);
		isp.zoom = s->zoom;
		isp.turned = s->camera->turned;
		float bias = log2f(requested_bias.gain / 256.0f);
		if (ae_locked && !was_ae_locked) {
			locked_base_bias = isp.exposure_bias;
		}
		was_ae_locked = ae_locked;
		isp.exposure_bias = bias;
		if (requested_bias.gain != current_bias.gain || requested_bias.command_id != current_bias.command_id) {
			current_bias = requested_bias;
			bias_sequence = f->sequence;
		}
		if (af_now) {
			say("tap controls: focus %u, exposure %u lines gain %u/%u, spot %s (green %.0f of frame mean %.0f), bias %+.2f EV",
			    f->focus, f->exposure_lines, f->gain, f->gain_unity, ae_spot ? "on" : "off", st.spot_green, st.mean[1], bias);
		}
		bool exposure_arrived = isp.asked && f->sequence > isp.asked_sequence &&
		    f->exposure_lines == isp.asked_lines && f->gain == isp.asked_gain && f->frame_lines == isp.asked_frame_lines;
		/* Both preview paths share the RAW10 exposure used by still capture. */
		if (!ae_locked && isp_ae(&isp, f, &st, &c) && write(device, &c, sizeof(c)) != (ssize_t)sizeof(c)) {
			isp.asked = false;
		}
		if (ae_locked || !isp.asked || (exposure_arrived && f->sequence > bias_sequence)) {
			if (applied_bias.gain != current_bias.gain || applied_bias.command_id != current_bias.command_id) {
				say("exposure bias applied: %+.2f EV at frame %llu (meter level %.1f)", bias,
				    (unsigned long long)f->sequence, st.mean[1]);
			}
			applied_bias = current_bias;
		}
		float auto_digital = isp.digital;
		if (ae_locked) {
			/* A slider movement can trim the locked exposure without re-metering the scene. */
			isp.digital = fminf(16, fmaxf(1.0f / 16, auto_digital * exp2f(bias - locked_base_bias)));
		}
		af_frame(device, &af, f, raw, window, af_locked, af_now);
		s->frames++;

		CFDictionaryRef metadata = frame_metadata(&isp, f, &st, s->port, ae_locked, af_locked, af.state >= AF_CLIMB, applied_bias);

		/*
		 * Only once the graph has started the stream. Its PixelBufferHandler blocks hand the buffer
		 * to multiStreamCameraSourceNode_sendBufferToBufferServicingThread, which reaches into the
		 * source node's innards: called before the graph is started (or after it has been torn down)
		 * it walks a pointer that is not there yet and mediaserverd dies of SIGSEGV. The frames
		 * before that are read and thrown away, which is also what settles the exposure.
		 */
		if (metadata != NULL) {
			pthread_mutex_lock(&s->metadata_lock);
			if (s->metadata != NULL) {
				CFRelease(s->metadata);
			}
			s->metadata = CFRetain(metadata);
			pthread_mutex_unlock(&s->metadata_lock);
		}
		/* The photograph's BPS and IPE pass, before the stream lock: it takes a few hundred milliseconds. */
		struct hakios_camera_frame icp_f;
		const uint8_t *icp = s->graph_running && s->stills_wanted > 0 ? icp_still(f, raw, &icp_f) : NULL;
		pthread_mutex_lock(&s->lock);
		if (s->graph_running) {
			if (s->graph_first_seq == 0) {
				s->graph_first_seq = f->sequence;
				s->graph_started_at = mach_absolute_time();
			}
			s->graph_last_seq = f->sequence;
			s->graph_frames++;
		}
		for (int i = 0; s->graph_running && i < OUTPUTS; i++) {
			struct output *out = &s->out[i];

			/*
			 * The scalers' outputs are the moving picture and are given every frame. The still image
			 * output is the whole sensor (4032x3024) and is the photograph: a frame goes to it only
			 * when the graph has asked for one, which is not something to make thirty times a second.
			 */
			bool still = strcmp(out->name, "StillImage") == 0;

			if (out->handler == NULL || out->config == NULL || !(out->enabled || i == 0)) {
				continue;
			}
			if (still && s->stills_wanted == 0) {
				continue;
			}
			/* Keep CPU stills when the IFE would blow out the same frame's highlights. */
			const uint8_t *processed = preview_using_raw || (still && !native_ife) ? NULL : preview;
			still_from_icp = still && icp != NULL;
			bool delivered = still_from_icp ? deliver(s, out, &isp, &icp_f, raw, icp, metadata) :
			    deliver(s, out, &isp, f, raw, processed, metadata);
			still_from_icp = false;
			if (still && delivered) {
				s->stills_wanted--;
				say("the photograph is given (%llu so far)", (unsigned long long)++s->stills_given);
			}
		}
		if (s->graph_running && s->graph_frames > 0 && s->graph_frames % 150 == 0) {
			static mach_timebase_info_data_t tb;
			if (tb.denom == 0) {
				mach_timebase_info(&tb);
			}
			double secs = (double)(mach_absolute_time() - s->graph_started_at) * tb.numer / tb.denom / 1e9;
			uint64_t deliv = s->out[0].delivered - s->graph_delivered_at_start;
			uint64_t ref = s->out[0].refused - s->graph_refused_at_start;
			uint64_t rns = s->out[0].render_ns - s->graph_render_ns_at_start;
			uint64_t rcnt = s->out[0].render_count - s->graph_render_count_at_start;
			uint64_t span = s->graph_last_seq >= s->graph_first_seq ? s->graph_last_seq - s->graph_first_seq + 1 : s->graph_frames;
			uint64_t skipped = span > s->graph_frames ? span - s->graph_frames : 0;
			say("preview: %.2f fps over %.2f s (%llu delivered, %llu skipped, %llu refused, seq %llu..%llu, render avg %.1f ms)",
			    secs > 0 ? (double)deliv / secs : 0.0, secs, (unsigned long long)deliv, (unsigned long long)skipped,
			    (unsigned long long)ref, (unsigned long long)s->graph_first_seq, (unsigned long long)s->graph_last_seq,
			    rcnt > 0 ? (double)rns / (double)rcnt / 1e6 : 0.0);
		}
		pthread_mutex_unlock(&s->lock);
		if (metadata != NULL) {
			CFRelease(metadata);
		}
		if (s->frames == 1 || s->frames % 300 == 0) {
			say("frame %llu: exposure %.1f ms, gain %.1fx, digital %.2fx, green %.0f above black; white balance R %.2f "
			    "B %.2f, lux index %.0f, about %.0f K", f->sequence, f->exposure_lines * (double)f->line_ns / 1e6,
			    (double)f->gain / f->gain_unity, isp.digital, st.mean[1], isp.wb[0], isp.wb[2], lux, cct);
			if (preview != NULL) {
				say("IFE preview stats: linearized luma %.0f, spot %.0f, highlight %.0f / 959, near white %.1f%%, bias %+.2f EV",
				    preview_stats.mean[1], preview_stats.spot_green, preview_stats.bright, 100 * preview_stats.near_white, bias);
			}
		}
		isp.digital = auto_digital;
	}
	if (tap_token >= 0) {
		notify_cancel(tap_token);
	}
	free(buffer);
	close(device);
	say("the frames stopped (%llu)", s->frames);
	return NULL;
}

/*
 * Reading frames from the kernel and giving them to the graph are two things: the first begins when
 * the capture graph reserves the stream (`graph` false, from the device's RequestStreams: the sensor
 * takes a moment to settle and the exposure longer), the second only when the graph starts the
 * stream itself (`graph` true, the FigCaptureStream start method).
 */
static CMBaseObjectRef all_streams[2];  /* the device's, back and front */
static OSStatus stream_stop(CMBaseObjectRef o);

static OSStatus
stream_begin(CMBaseObjectRef o, bool graph)
{
	struct stream *s = stream_of(o);

	if (graph) {
		pthread_mutex_lock(&s->lock);
		s->graph_running = true;
		s->graph_started_at = 0;
		s->graph_first_seq = 0;
		s->graph_last_seq = 0;
		s->graph_frames = 0;
		s->graph_delivered_at_start = s->out[0].delivered;
		s->graph_refused_at_start = s->out[0].refused;
		s->graph_render_ns_at_start = s->out[0].render_ns;
		s->graph_render_count_at_start = s->out[0].render_count;
		pthread_mutex_unlock(&s->lock);
		say("stream: the graph starts the stream (format %d)", s->format_index);
	}
	if (s->running) {
		return 0;
	}
	/* one camera at a time: the kernel gives one sensor's frames (the other stream's worker stopped first) */
	for (int i = 0; i < 2; i++) {
		if (all_streams[i] != NULL && all_streams[i] != o && stream_of(all_streams[i])->running) {
			say("stream: the %s's frames stop for the %s's", stream_of(all_streams[i])->camera->name, s->camera->name);
			(void)stream_stop(all_streams[i]);
		}
	}
	s->running = true;
	s->frames = 0;
	say("stream: frames begin (format %d)", s->format_index);
	if (pthread_create(&s->worker, NULL, worker, o) != 0) {
		s->running = false;
		return ERR_UNSUPPORTED;
	}
	s->worker_started = true;
	return 0;
}

static OSStatus
stream_start(CMBaseObjectRef o)
{
	return stream_begin(o, true);
}

static OSStatus
stream_stop(CMBaseObjectRef o)
{
	struct stream *s = stream_of(o);

	pthread_mutex_lock(&s->lock);
	if (s->graph_frames > 0 && s->graph_started_at != 0) {
		static mach_timebase_info_data_t tb;
		if (tb.denom == 0) {
			mach_timebase_info(&tb);
		}
		double secs = (double)(mach_absolute_time() - s->graph_started_at) * tb.numer / tb.denom / 1e9;
		uint64_t deliv = s->out[0].delivered - s->graph_delivered_at_start;
		uint64_t ref = s->out[0].refused - s->graph_refused_at_start;
		uint64_t rns = s->out[0].render_ns - s->graph_render_ns_at_start;
		uint64_t rcnt = s->out[0].render_count - s->graph_render_count_at_start;
		uint64_t span = s->graph_last_seq >= s->graph_first_seq ? s->graph_last_seq - s->graph_first_seq + 1 : s->graph_frames;
		uint64_t skipped = span > s->graph_frames ? span - s->graph_frames : 0;
		say("stream: stop (%llu frames, %llu pictures given; session %.2f fps over %.2f s, %llu delivered, %llu skipped, "
		    "%llu refused, render avg %.1f ms)", (unsigned long long)s->frames, (unsigned long long)s->out[0].delivered,
		    secs > 0 ? (double)deliv / secs : 0.0, secs, (unsigned long long)deliv, (unsigned long long)skipped,
		    (unsigned long long)ref, rcnt > 0 ? (double)rns / (double)rcnt / 1e6 : 0.0);
	} else {
		say("stream: stop (%llu frames, %llu pictures given)", (unsigned long long)s->frames,
		    (unsigned long long)s->out[0].delivered);
	}
	s->graph_running = false;
	for (int i = 0; i < OUTPUTS; i++) {
		let_go(&s->out[i]);
	}
	pthread_mutex_unlock(&s->lock);
	s->running = false;
	if (s->worker_started) {
		pthread_join(s->worker, NULL);
		s->worker_started = false;
	}
	return 0;
}

static void
add_absent(const void *key, const void *value, void *context)
{
	if (!CFDictionaryContainsKey(context, key)) {
		CFDictionarySetValue(context, key, value);
	}
}

/*
 * The frame's metadata into the sample buffer the graph made of our pixel buffer: the pixel buffer's
 * own dictionary (deliver()), the same object, else the last frame's; whatever the graph has put in a
 * dictionary of the sample buffer's already stays as it is.
 */
static OSStatus
stream_add_attachments(CMBaseObjectRef o, CMSampleBufferRef sbuf, CFDictionaryRef options)
{
	struct stream *s = stream_of(o);
	static bool said;
	CVImageBufferRef image = sbuf != NULL ? CMSampleBufferGetImageBuffer(sbuf) : NULL;
	CFTypeRef ours = image != NULL && md_attachment != NULL ? CVBufferGetAttachment(image, md_attachment, NULL) : NULL, theirs;
	CFMutableDictionaryRef m;

	if (!said) {
		said = true;
		say("stream: AddAttachmentsToSampleBuffer (options %s): the frames' metadata %s", options ? "given" : "none",
		    md_attachment != NULL ? "attached" : "NOT THERE");
	}
	if (md_attachment == NULL || sbuf == NULL) {
		return 0;
	}
	if (ours != NULL && CFGetTypeID(ours) == CFDictionaryGetTypeID()) {
		CFRetain(ours);
	} else {
		pthread_mutex_lock(&s->metadata_lock);
		ours = s->metadata != NULL ? CFDictionaryCreateMutableCopy(NULL, 0, s->metadata) : NULL;
		pthread_mutex_unlock(&s->metadata_lock);
	}
	if (ours == NULL) {
		return 0;
	}
	theirs = CMGetAttachment(sbuf, md_attachment, NULL);
	if (theirs == ours) {
		CFRelease(ours);
		return 0;
	}
	if (theirs != NULL && CFGetTypeID(theirs) == CFDictionaryGetTypeID()) {
		m = CFDictionaryCreateMutableCopy(NULL, 0, theirs);
		CFDictionaryApplyFunction(ours, add_absent, m);
	} else {
		m = (CFMutableDictionaryRef)CFRetain(ours);
	}
	CMSetAttachment(sbuf, md_attachment, m, kCMAttachmentMode_ShouldPropagate);
	CFRelease(m);
	CFRelease(ours);
	return 0;
}

static OSStatus
stream_invalidate(CMBaseObjectRef o)
{
	(void)stream_stop(o);
	return 0;
}

static void
stream_finalize(CMBaseObjectRef o)
{
	struct stream *s = stream_of(o);

	(void)stream_stop(o);
	for (int i = 0; i < OUTPUTS; i++) {
		if (s->out[i].config) {
			CFRelease(s->out[i].config);
		}
		if (s->out[i].handler) {
			CFRelease(s->out[i].handler);
		}
	}
	if (s->formats) {
		CFRelease(s->formats);
	}
	if (s->supported) {
		CFRelease(s->supported);
	}
	if (s->colour_info) {
		CFRelease(s->colour_info);
	}
	if (s->metadata) {
		CFRelease(s->metadata);
	}
	if (s->torch >= 0) {
		close(s->torch);
	}
}

static CFStringRef
stream_description(CMBaseObjectRef o)
{
	struct stream *s = stream_of(o);
	const char *sensor = s && s->camera ? sensor_name_for(s->camera->sensor) : NULL;

	if (s && s->camera && sensor != NULL) {
		return CFStringCreateWithFormat(NULL, NULL, CFSTR("<HakiosCaptureStream: %s (%s)>"), s->camera->name, sensor);
	}
	return CFRetain(CFSTR("<HakiosCaptureStream: camera>"));
}

static const struct base_class stream_base = {
	1, sizeof(struct stream), NULL, stream_invalidate, stream_finalize, stream_description, stream_copy_property, stream_set_property,
	NULL, NULL, NULL
};
static const struct stream_class stream_derived = { 2, stream_start, stream_stop, stream_add_attachments };
static const struct vtable stream_vtable = { NULL, &stream_base, &stream_derived };

static CMBaseObjectRef
stream_create(CFAllocatorRef allocator, const struct camera *camera, CFStringRef port)
{
	CMBaseObjectRef o = NULL;
	struct stream *s;

	if (derived_create(allocator, &stream_vtable, stream_class_id(), &o) != 0 || o == NULL) {
		say("the stream cannot be made");
		return NULL;
	}
	s = stream_of(o);
	memset(s, 0, sizeof(*s));
	pthread_mutex_init(&s->lock, NULL);
	pthread_mutex_init(&s->metadata_lock, NULL);
	pthread_mutex_init(&s->controls_lock, NULL);
	s->torch = -1;
	s->exposure_bias.gain = 256;
	s->min_fps = 2;
	s->zoom = 1;
	s->af_window[0] = s->af_window[1] = 0.3f;
	s->af_window[2] = s->af_window[3] = 0.7f;
	s->max_fps = 30;
	s->camera = camera;
	s->port = port;
	for (int i = 0; i < OUTPUTS; i++) {
		s->out[i].name = output_names[i];
	}

	CFDictionaryRef f[2] = { format(camera->formats[0][0], camera->formats[0][1], camera->formats[0][2], false, camera->fov),
		format(camera->formats[1][0], camera->formats[1][1], camera->formats[1][2], false, camera->fov) };

	s->formats = CFArrayCreate(NULL, (const void **)f, 2, &kCFTypeArrayCallBacks);
	s->supported = supported(stream_props, sizeof(stream_props) / sizeof(stream_props[0]));
	CFRelease(f[0]);
	CFRelease(f[1]);
	return o;
}

/* ---- the device ------------------------------------------------------------------------------------- */

struct device {
	CFArrayRef      streams;
	CFDictionaryRef supported;
};

static OSStatus
device_copy_property(CMBaseObjectRef o, CFStringRef key, CFAllocatorRef allocator, void *out)
{
	struct device *d = derived_storage(o);
	CFTypeRef value = NULL;

	(void)allocator;
	if (key_is(key, "StreamArray")) {
		value = CFRetain(d->streams);
	} else if (key_is(key, "Clock")) {
		value = CFRetain(CMClockGetHostTimeClock());
	} else if (key_is(key, "SupportedPropertiesDictionary")) {
		value = CFRetain(d->supported);
	} else if (key_is(key, "SupportedSynchronizedStreamsGroups")) {
		value = CFArrayCreate(NULL, NULL, 0, &kCFTypeArrayCallBacks);
	} else {
		say_once("device: no answer for", key, NULL);
		return ERR_NOT_AVAILABLE;
	}
	*(CFTypeRef *)out = value;
	return 0;
}

static OSStatus
device_set_property(CMBaseObjectRef o, CFStringRef key, CFTypeRef value)
{
	(void)o;
	say_once("device: set (not used)", key, value);
	return 0;
}

static unsigned requests, relinquishes;

static OSStatus
device_request_streams(CMBaseObjectRef o, CFArrayRef streams, CFDictionaryRef options)
{
	(void)o;
	if (requests++ < 3) {
		say("device: streams asked for (%ld)%s", streams ? (long)CFArrayGetCount(streams) : 0L, options ? ", with options" : "");
		if (options != NULL) {
			say_once("device: the request's options", CFSTR("RequestStreams"), options);
		}
	}
	/*
	 * Requesting a stream is this device's cue to begin producing: mediaserverd's capture graph
	 * configures the stream (it installs the PrimaryScaler PixelBufferHandler) and then reserves the
	 * streams through here, but it never calls the FigCaptureStream start method, so start the stream's
	 * frame worker from the reservation. stream_start is idempotent (it no-ops when already running),
	 * and relinquish does not stop the worker, so the reservation thrash does not tear it down.
	 */
	if (streams != NULL) {
		CFIndex n = CFArrayGetCount(streams);

		for (CFIndex i = 0; i < n; i++) {
			CMBaseObjectRef st = (CMBaseObjectRef)CFArrayGetValueAtIndex(streams, i);

			if (st != NULL) {
				(void)stream_begin(st, false);
			}
		}
	}
	return 0;
}

static OSStatus
device_relinquish_streams(CMBaseObjectRef o, CFArrayRef streams, CFDictionaryRef options)
{
	(void)o;
	(void)options;
	if (relinquishes++ < 3) {
		say("device: streams given back (%ld)", streams ? (long)CFArrayGetCount(streams) : 0L);
	} else if (relinquishes % 1000 == 0) {
		say("device: streams given back %u times", relinquishes);
	}
	return 0;
}

static OSStatus
device_invalidate(CMBaseObjectRef o)
{
	(void)o;
	return 0;
}

static void
device_finalize(CMBaseObjectRef o)
{
	struct device *d = derived_storage(o);

	if (d->streams) {
		CFRelease(d->streams);
	}
	if (d->supported) {
		CFRelease(d->supported);
	}
}

static CFStringRef
device_description(CMBaseObjectRef o)
{
	(void)o;
	return CFStringCreateWithCString(NULL, kernel_device_desc[0] ? kernel_device_desc : "<HakiosCaptureDevice>",
	    kCFStringEncodingUTF8);
}

static const struct base_class device_base = {
	1, sizeof(struct device), NULL, device_invalidate, device_finalize, device_description, device_copy_property, device_set_property,
	NULL, NULL, NULL
};
static const struct device_class device_derived = { 3, NULL, device_request_streams, device_relinquish_streams, NULL };
static const struct vtable device_vtable = { NULL, &device_base, &device_derived };

/* The vendor's device create function (as H10ISPCaptureDeviceCreate is called). */
OSStatus hakios_camera_device_create(CFAllocatorRef allocator, CFDictionaryRef options, CMBaseObjectRef *out);

OSStatus
hakios_camera_device_create(CFAllocatorRef allocator, CFDictionaryRef options, CMBaseObjectRef *out)
{
	static bool found, usable;
	CMBaseObjectRef device = NULL, stream[2] = { NULL, NULL };
	CFStringRef ports[2];
	struct device *d;
	CFIndex count = 0;
	char summary[160] = "";
	size_t spos = 0;

	(void)options;
	if (!found) {
		found = true;
		usable = find_functions();
		discover_cameras();
	}
	if (!usable || out == NULL) {
		return ERR_UNSUPPORTED;
	}
	ports[0] = port_back;
	ports[1] = port_front;
	for (int i = 0; i < 2; i++) {
		if ((kernel_sensor_mask & (1u << cameras[i].sensor)) == 0) {
			continue;
		}
		stream[count] = stream_create(allocator, &cameras[i], ports[i]);
		if (stream[count] == NULL) {
			for (CFIndex j = 0; j < count; j++) {
				CFRelease(stream[j]);
			}
			return ERR_UNSUPPORTED;
		}
		const char *sname = sensor_name_for(cameras[i].sensor);
		spos += (size_t)snprintf(summary + spos, sizeof(summary) - spos, "%s%s: %s", count > 0 ? ", " : "",
		    cameras[i].name, sname ? sname : "unknown");
		count++;
	}
	if (count == 0) {
		say("the kernel advertises no supported cameras: device not made");
		return ERR_UNSUPPORTED;
	}
	if (derived_create(allocator, &device_vtable, device_class_id(), &device) != 0 || device == NULL) {
		for (CFIndex j = 0; j < count; j++) {
			CFRelease(stream[j]);
		}
		say("the device cannot be made");
		return ERR_UNSUPPORTED;
	}
	d = derived_storage(device);
	memset(d, 0, sizeof(*d));
	d->streams = CFArrayCreate(NULL, (const void **)stream, count, &kCFTypeArrayCallBacks);
	all_streams[0] = count > 0 ? stream[0] : NULL;
	all_streams[1] = count > 1 ? stream[1] : NULL;
	d->supported = supported(device_props, sizeof(device_props) / sizeof(device_props[0]));
	for (CFIndex j = 0; j < count; j++) {
		CFRelease(stream[j]);
	}
	*out = device;
	say("the device made: %ld stream%s (%s)", (long)count, count == 1 ? "" : "s", summary);
	return 0;
}
