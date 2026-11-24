/*
 * libhakios.app.dylib: code of ours inside an application, to find out why it dies (see inject.c
 * for how it gets there; build.sh's INJECT names the programs).
 *
 * An application that is launched and gone again in a third of a second leaves nothing behind here:
 * there is no crash reporter in this system, and what it writes on its standard error goes nowhere.
 * This gives it a standard error -- the console -- so that the Swift runtime's last words are read,
 * and says the frames of the stack when a signal kills it, under the dyld shared cache's slide,
 * which `ipsw dyld a2s <cache> <address>` turns back into names.
 */
#include <Block.h>
#include <CoreGraphics/CGGeometry.h>
#include <dispatch/dispatch.h>
#include <mach/mach_time.h>
#include <math.h>
#include <notify.h>
#include <sys/sysctl.h>
#include <spawn.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <objc/message.h>
#include <objc/runtime.h>
#include <execinfo.h>
#include <fcntl.h>
#include <mach-o/dyld.h>
#include <signal.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "camera_tap.h"

extern const void *_dyld_get_shared_cache_range(size_t *length);

#define CACHE_BASE 0x180000000UL

static uintptr_t slide;
static int console = -1;
static const char *who = "?";

static void
say(const char *text, size_t length)
{
	if (console < 0) {
		console = open("/dev/console", O_WRONLY);
	}
	(void)write(console >= 0 ? console : 2, text, length);
}

static void
killed(int number, siginfo_t *info, void *context)
{
	void *frames[32];
	char line[900];
	int n = backtrace(frames, 32), k;

	(void)context;
	k = snprintf(line, sizeof(line), "hakios: %s: SIGNAL %d at %p (the cache's slide %#lx), the stack:", who, number,
	    info != NULL ? info->si_addr : NULL, (unsigned long)slide);
	for (int i = 0; i < n && k < (int)sizeof(line) - 24; i++) {
		k += snprintf(line + k, sizeof(line) - (size_t)k, " %#lx", (unsigned long)((uintptr_t)frames[i] - slide));
	}
	line[k++] = '\n';
	say(line, (size_t)k);
	signal(number, SIG_DFL);
	raise(number);
}

/*
 * Two things an application does on its way to a download, said out loud. Sileo queues a package and
 * nothing happens: no connection is made and no error is shown, so the question is how far it gets --
 * does it ask UIKit for a background task (a round trip to another daemon, which can wait for ever if
 * that daemon does not answer), and does it ever start a URL session task, for what address.
 */
static unsigned long (*begin_task)(id, SEL, id);
static void (*resume_task)(id, SEL);

static unsigned long
watched_begin_task(id self, SEL cmd, id handler)
{
	unsigned long task;
	char line[160];
	int n;

	n = snprintf(line, sizeof(line), "hakios: %s: asking UIKit for a background task\n", who);
	say(line, (size_t)(n > 0 ? n : 0));
	task = begin_task(self, cmd, handler);
	n = snprintf(line, sizeof(line), "hakios: %s: the background task is %lu\n", who, task);
	say(line, (size_t)(n > 0 ? n : 0));
	return task;
}

static void
watched_resume_task(id self, SEL cmd)
{
	const char *(*string)(id, SEL) = (const char *(*)(id, SEL))objc_msgSend;
	id (*get)(id, SEL) = (id (*)(id, SEL))objc_msgSend;
	const char *url = "?";
	char line[700];
	id request, address, text;
	int n;

	request = get(self, sel_registerName("originalRequest"));
	address = request != NULL ? get(request, sel_registerName("URL")) : NULL;
	text = address != NULL ? get(address, sel_registerName("absoluteString")) : NULL;
	if (text != NULL) {
		url = string(text, sel_registerName("UTF8String"));
	}
	n = snprintf(line, sizeof(line), "hakios: %s: starting a session task for %s\n", who,
	    url != NULL ? url : "?");
	say(line, (size_t)(n > 0 ? n : 0));
	resume_task(self, cmd);
}

/*
 * One setting of Sileo's, set for it at every start: "Show Packages From Canister" (ShowProvisional,
 * on by default). Canister is the search service Sileo asks about packages, and where its servers
 * cannot be reached the asking never stops -- 3810 session tasks in two minutes in one run here,
 * and the download of a package never gets its turn among them. With the setting off Sileo keeps to
 * the repositories themselves, which work. The prefix lives in memory and is new at every boot, so
 * there is nowhere to keep this but here. It is done on the main queue, not here in the library's
 * own initializer: at that point Foundation is loaded but has not run its own initializers yet, and
 * asking it for the defaults then kills the application before it says a word.
 */
static void
keep_to_the_repositories(void)
{
	id (*get)(id, SEL) = (id (*)(id, SEL))objc_msgSend;
	id (*from_text)(id, SEL, const char *) = (id (*)(id, SEL, const char *))objc_msgSend;
	void (*set_bool)(id, SEL, signed char, id) = (void (*)(id, SEL, signed char, id))objc_msgSend;
	id defaults = (id)objc_getClass("NSUserDefaults");
	id key = from_text((id)objc_getClass("NSString"), sel_registerName("stringWithUTF8String:"), "ShowProvisional");

	if (defaults == NULL || key == NULL) {
		return;
	}
	defaults = get(defaults, sel_registerName("standardUserDefaults"));
	if (defaults != NULL) {
		set_bool(defaults, sel_registerName("setBool:forKey:"), 0, key);
		say("hakios: Sileo: its Canister search is turned off (it is not reachable here)\n", 73);
	}
}

/*
 * apt-get install, not its simulation (-s...): the one that ends in "Unable to fetch some archives"
 * when Sileo runs it and installs the package when a terminal runs the same command.
 */
static int
apt_install(char *const argv[])
{
	int install = 0;

	if (argv == NULL || argv[0] == NULL || strcmp(argv[0], "apt-get") != 0) {
		return 0;
	}
	for (int i = 1; argv[i] != NULL; i++) {
		if (argv[i][0] == '-' && argv[i][1] == 's') {
			return 0;
		}
		install |= strcmp(argv[i], "install") == 0;
	}
	return install;
}

/*
 * What apt says goes to Sileo, which shows a line of it at most. Run the same apt-get, with the same
 * descriptors, credential and environment, under a shell that first says on the console who it runs
 * as, what lies in apt's archives and what Sileo's index says of the package, and afterwards how apt
 * ended and what the archives hold then. This is how "Unable to fetch some archives" was found: the
 * archives were empty, Sileo's /bin/mv having never run.
 */
static int
spawn_apt_said(pid_t *pid, const posix_spawn_file_actions_t *actions, const posix_spawnattr_t *attr,
    char *const argv[], char *const envp[])
{
	static char script[] =
	    "exec 7>/dev/console; J=/var/jb\n"
	    "echo \"hakios: apt-get runs as $($J/usr/bin/id), in $(pwd)\" >&7\n"
	    "$J/bin/ls -ln $J/var/cache/apt/archives $J/var/lib/apt/sileolists >&7 2>&1\n"
	    "for a in \"$@\"; do case $a in -*|install) ;; *) p=${a%%=*}\n"
	    "  echo \"hakios: $p in sileolists:\" >&7\n"
	    "  $J/usr/bin/grep -h -A30 \"^Package: $p\\$\" $J/var/lib/apt/sileolists/*Packages 2>&1 | $J/usr/bin/head -40 >&7 ;; esac; done\n"
	    "$J/usr/bin/apt-get \"$@\"\n"
	    "s=$?; echo \"hakios: apt-get exits $s; its archives after:\" >&7\n"
	    "$J/bin/ls -ln $J/var/cache/apt/archives >&7 2>&1; exit $s\n";
	char *wrapped[96];
	int k = 0;

	for (int i = 0; envp != NULL && envp[i] != NULL; i++) {
		char line[600];
		int n = snprintf(line, sizeof(line), "hakios: %s: apt-get's environment: %s\n", who, envp[i]);

		say(line, (size_t)(n < (int)sizeof(line) ? n : (int)sizeof(line) - 1));
	}
	wrapped[k++] = "sh";
	wrapped[k++] = "-c";
	wrapped[k++] = script;
	wrapped[k++] = "apt-get";
	for (int i = 1; argv[i] != NULL && k < 95; i++) {
		wrapped[k++] = argv[i];
	}
	wrapped[k] = NULL;
	return posix_spawn(pid, "/var/jb/bin/sh", actions, attr, wrapped, envp);
}

/*
 * Every program the application starts, with its arguments. Sileo does everything to the system by
 * spawning its set-user-ID helper, which execs apt or dpkg; the arguments are the whole question
 * when apt's answer to Sileo differs from apt's answer on the command line.
 */
static int
watched_spawn(pid_t *pid, const char *path, const posix_spawn_file_actions_t *actions,
    const posix_spawnattr_t *attr, char *const argv[], char *const envp[])
{
	char line[1200];
	int n = snprintf(line, sizeof(line), "hakios: %s: (uid %d) spawns %s", who, (int)getuid(),
	    path != NULL ? path : "?");

	for (int i = 0; argv != NULL && argv[i] != NULL && n < (int)sizeof(line) - 40; i++) {
		n += snprintf(line + n, sizeof(line) - (size_t)n, " %s", argv[i]);
	}
	/*
	 * The size of what it is about to move, while the file is still there: apt refuses the
	 * package Sileo puts in its archives ("Unable to fetch some archives") and takes the very
	 * same package when apt itself fetched it, so the first thing to know is whether the file
	 * is whole.
	 */
	if (argv != NULL && argv[0] != NULL && strcmp(argv[0], "mv") == 0 && argv[1] != NULL) {
		struct stat st;

		if (stat(argv[1], &st) == 0) {
			n += snprintf(line + n, sizeof(line) - (size_t)n, " [%lld bytes]", (long long)st.st_size);
		} else {
			n += snprintf(line + n, sizeof(line) - (size_t)n, " [no such file]");
		}
	}
	n += snprintf(line + n, sizeof(line) - (size_t)n, "\n");
	say(line, (size_t)n);
	if (apt_install(argv)) {
		return spawn_apt_said(pid, actions, attr, argv, envp);
	}
	n = posix_spawn(pid, path, actions, attr, argv, envp);
	if (n != 0) {
		char said[300];
		int k = snprintf(said, sizeof(said), "hakios: %s: ... and that spawn failed: %s\n", who, strerror(n));

		say(said, (size_t)(k < (int)sizeof(said) ? k : (int)sizeof(said) - 1));
	}
	return n;
}

/*
 * A file the application moves or copies itself (NSFileManager, under Swift's FileManager), when it
 * fails: from where, to where, and what the error says. Sileo moves each download into place this
 * way, and says only "The operation couldn't be completed. No such file or directory".
 */
static const char *
text_of(id object)
{
	id (*get)(id, SEL) = (id (*)(id, SEL))objc_msgSend;
	const char *(*utf8)(id, SEL) = (const char *(*)(id, SEL))objc_msgSend;
	id text;

	if (object == NULL) {
		return "(nil)";
	}
	text = (class_respondsToSelector(object_getClass(object), sel_registerName("path")) ?
	    get(object, sel_registerName("path")) : get(object, sel_registerName("description")));
	return text != NULL ? utf8(text, sel_registerName("UTF8String")) : "?";
}

static signed char (*move_url)(id, SEL, id, id, id *), (*copy_url)(id, SEL, id, id, id *);
static signed char (*move_path)(id, SEL, id, id, id *), (*copy_path)(id, SEL, id, id, id *);

static signed char
file_said(const char *what, signed char ok, id from, id to, id error)
{
	if (!ok) {
		char line[1400];
		int n = snprintf(line, sizeof(line), "hakios: %s: %s %s -> %s failed: %s\n", who, what, text_of(from),
		    text_of(to), error != NULL ? text_of(error) : "(no error given)");

		say(line, (size_t)(n < (int)sizeof(line) ? n : (int)sizeof(line) - 1));
	}
	return ok;
}

static signed char
watched_move_url(id self, SEL cmd, id from, id to, id *error)
{
	id mine = NULL;
	signed char ok = move_url(self, cmd, from, to, error != NULL ? error : &mine);

	return file_said("moveItemAtURL", ok, from, to, error != NULL ? *error : mine);
}

static signed char
watched_copy_url(id self, SEL cmd, id from, id to, id *error)
{
	id mine = NULL;
	signed char ok = copy_url(self, cmd, from, to, error != NULL ? error : &mine);

	return file_said("copyItemAtURL", ok, from, to, error != NULL ? *error : mine);
}

static signed char
watched_move_path(id self, SEL cmd, id from, id to, id *error)
{
	id mine = NULL;
	signed char ok = move_path(self, cmd, from, to, error != NULL ? error : &mine);

	return file_said("moveItemAtPath", ok, from, to, error != NULL ? *error : mine);
}

static signed char
watched_copy_path(id self, SEL cmd, id from, id to, id *error)
{
	id mine = NULL;
	signed char ok = copy_path(self, cmd, from, to, error != NULL ? error : &mine);

	return file_said("copyItemAtPath", ok, from, to, error != NULL ? *error : mine);
}

__attribute__((used)) static struct {
	const void *ours, *theirs;
} spawn_interpose __attribute__((section("__DATA,__interpose"))) = {
	(const void *)watched_spawn, (const void *)posix_spawn
};

/* Put ours in the place of a method, keeping the one that was there. */
static void
watch(const char *name, const char *selector, IMP ours, void *theirs)
{
	Class class = objc_getClass(name);
	Method method = class != NULL ? class_getInstanceMethod(class, sel_registerName(selector)) : NULL;

	if (method != NULL && method_getImplementation(method) != ours) {
		*(IMP *)theirs = method_setImplementation(method, ours);
	}
}

/*
 * The iOS 16.5 capture service submits the first single autofocus operation but can suppress
 * repeats with new points while it stays in that mode. Camera has already converted each
 * tap to a normalized capture point. Relay repeated requests to the active HakiOS stream.
 */
enum { CAMERA_FOCUS_SINGLE = 1 };
static int (*camera_set_focus)(id, SEL, long, float, int);
static char camera_single_focus_key;
static int camera_tap_token = -1;

static int
camera_repeat_focus(id device, SEL cmd, long mode, float position, int request)
{
	bool repeated = mode == CAMERA_FOCUS_SINGLE && objc_getAssociatedObject(device, &camera_single_focus_key) != NULL;
	int result = camera_set_focus(device, cmd, mode, position, request);
	if (result != 0) {
		return result;
	}
	objc_setAssociatedObject(device, &camera_single_focus_key, mode == CAMERA_FOCUS_SINGLE ? device : nil,
	    OBJC_ASSOCIATION_ASSIGN);
	if (repeated) {
		CGPoint (*get_point)(id, SEL) = (CGPoint (*)(id, SEL))objc_msgSend;
		CGPoint point = get_point(device, sel_registerName("focusPointOfInterest"));
		if (point.x >= 0 && point.x <= 1 && point.y >= 0 && point.y <= 1) {
			uint64_t x = (uint64_t)lround(point.x * HAKIOS_CAMERA_TAP_SCALE);
			uint64_t y = (uint64_t)lround(point.y * HAKIOS_CAMERA_TAP_SCALE);
			if (notify_set_state(camera_tap_token, x << 16 | y) == NOTIFY_STATUS_OK) {
				(void)notify_post(HAKIOS_CAMERA_TAP_NAME);
			}
		}
	}
	return result;
}

static void
camera_enable_repeat_focus(void)
{
	if (notify_register_check(HAKIOS_CAMERA_TAP_NAME, &camera_tap_token) != NOTIFY_STATUS_OK) {
		return;
	}
	watch("AVCaptureFigVideoDevice", "_setFocusWithMode:lensPosition:requestID:", (IMP)camera_repeat_focus,
	    &camera_set_focus);
	if (camera_set_focus != NULL) {
		static const char line[] = "hakios: Camera: repeat tap-to-focus enabled\n";

		say(line, sizeof(line) - 1);
	}
}

/*
 * cam_save_trace=1: Camera's side of a photograph, on mach_absolute_time's clock like mediaserverd's lines and
 * savetrace's ("[t=...]"): when a touch reaches the application, the shutter button's action, the requests to
 * AVCapturePhotoOutput and each of the photograph's delegate calls.
 */
static void
trace_say(const char *format, ...)
{
	static mach_timebase_info_data_t base;
	char line[300];
	va_list ap;

	if (base.denom == 0) {
		mach_timebase_info(&base);
	}
	uint64_t us = mach_absolute_time() * base.numer / base.denom / 1000;
	int n = snprintf(line, sizeof(line), "[t=%llu.%03llu] hakios: Camera: ", (unsigned long long)(us / 1000),
	    (unsigned long long)(us % 1000));
	va_start(ap, format);
	n += vsnprintf(line + n, sizeof(line) - (size_t)n - 1, format, ap);
	va_end(ap);
	if (n > (int)sizeof(line) - 2) {
		n = sizeof(line) - 2;
	}
	line[n++] = '\n';
	say(line, (size_t)n);
}

static void (*send_event)(id, SEL, id);
static void (*send_action)(id, SEL, SEL, id, id);
static void (*capture_photo)(id, SEL, id, id);
static void (*prepare_photos)(id, SEL, id, id);

static void
traced_send_event(id app, SEL cmd, id event)
{
	long type = ((long (*)(id, SEL))objc_msgSend)(event, sel_registerName("type"));
	if (type == 0) {        /* UIEventTypeTouches */
		id touches = ((id (*)(id, SEL))objc_msgSend)(event, sel_registerName("allTouches"));
		id touch = ((id (*)(id, SEL))objc_msgSend)(touches, sel_registerName("anyObject"));
		long phase = touch != nil ? ((long (*)(id, SEL))objc_msgSend)(touch, sel_registerName("phase")) : -1;
		if (phase == 0 || phase == 3 || phase == 4) {
			trace_say("touch %s reaches the application", phase == 0 ? "began" : phase == 3 ? "ended" : "cancelled");
		}
	}
	send_event(app, cmd, event);
	if (type == 0) {
		trace_say("touch event handled");
	}
}

static void
traced_send_action(id control, SEL cmd, SEL action, id target, id event)
{
	trace_say("%s sends %s to %s", object_getClassName(control), sel_getName(action),
	    target != nil ? object_getClassName(target) : "(the responder chain)");
	send_action(control, cmd, action, target, event);
	trace_say("%s returned", sel_getName(action));
}

/* The photograph's delegate's methods, each class's own once: (output, settings or photo[, error]). */
#define DELEGATE_SELECTORS 5
static const char *const delegate_selectors[DELEGATE_SELECTORS] = {
	"captureOutput:willBeginCaptureForResolvedSettings:",
	"captureOutput:willCapturePhotoForResolvedSettings:",
	"captureOutput:didCapturePhotoForResolvedSettings:",
	"captureOutput:didFinishProcessingPhoto:error:",
	"captureOutput:didFinishCaptureForResolvedSettings:error:",
};

static void
hook_delegate(Class class)
{
	static Class hooked[8];
	static unsigned count;

	for (unsigned i = 0; i < count; i++) {
		if (hooked[i] == class) {
			return;
		}
	}
	if (count == sizeof(hooked) / sizeof(hooked[0])) {
		return;
	}
	hooked[count++] = class;
	for (unsigned i = 0; i < DELEGATE_SELECTORS; i++) {
		SEL selector = sel_registerName(delegate_selectors[i]);
		Method method = class_getInstanceMethod(class, selector);
		if (method == NULL) {
			continue;
		}
		void (*original)(id, SEL, id, id, id) = (void (*)(id, SEL, id, id, id))method_getImplementation(method);
		const char *name = delegate_selectors[i];
		void (^body)(id, id, id, id) = ^(id self, id output, id a, id b) {
			trace_say("delegate %s", name);
			original(self, selector, output, a, b);
			trace_say("delegate %s returned", name);
		};
		IMP traced = imp_implementationWithBlock((id)body);
		method_setImplementation(method, traced);
	}
	trace_say("photograph delegate %s traced", class_getName(class));
}

static void
traced_capture_photo(id output, SEL cmd, id settings, id delegate)
{
	static unsigned shots;
	SEL url = sel_registerName("livePhotoMovieFileURL");
	id movie = settings != nil && class_respondsToSelector(object_getClass(settings), url) ?
	    ((id (*)(id, SEL))objc_msgSend)(settings, url) : nil;
	long flash = ((long (*)(id, SEL))objc_msgSend)(settings, sel_registerName("flashMode"));

	trace_say("photograph %u: flash mode %ld, %s", ++shots, flash, movie != nil ? "with a Live Photo movie" : "no Live Photo");
	trace_say("capturePhotoWithSettings: asked (delegate %s)", delegate != nil ? object_getClassName(delegate) : "nil");
	if (delegate != nil) {
		hook_delegate(object_getClass(delegate));
	}
	capture_photo(output, cmd, settings, delegate);
	trace_say("capturePhotoWithSettings: returned");
}

static void
traced_prepare_photos(id output, SEL cmd, id settings, id completion)
{
	trace_say("setPreparedPhotoSettingsArray: asked");
	if (completion == nil) {
		prepare_photos(output, cmd, settings, completion);
		return;
	}
	void (^handler)(BOOL, id) = (void (^)(BOOL, id))completion;
	void (^wrapped)(BOOL, id) = Block_copy(^(BOOL prepared, id error) {
		trace_say("setPreparedPhotoSettingsArray: completed (%s)", prepared ? "prepared" : "not prepared");
		handler(prepared, error);
	});
	prepare_photos(output, cmd, settings, (id)wrapped);
}

static bool
save_trace_enabled(void)
{
	char arguments[2048];
	size_t length = sizeof(arguments);
	char *state;

	if (sysctlbyname("kern.bootargs", arguments, &length, NULL, 0) != 0) {
		return false;
	}
	for (char *word = strtok_r(arguments, " ", &state); word != NULL; word = strtok_r(NULL, " ", &state)) {
		if (strcmp(word, "cam_save_trace=1") == 0) {
			return true;
		}
	}
	return false;
}

static void
camera_enable_save_trace(void)
{
	watch("UIApplication", "sendEvent:", (IMP)traced_send_event, &send_event);
	watch("UIControl", "sendAction:to:forEvent:", (IMP)traced_send_action, &send_action);
	watch("AVCapturePhotoOutput", "capturePhotoWithSettings:delegate:", (IMP)traced_capture_photo, &capture_photo);
	watch("AVCapturePhotoOutput", "setPreparedPhotoSettingsArray:completionHandler:", (IMP)traced_prepare_photos,
	    &prepare_photos);
	trace_say("save trace on (sendEvent %s, sendAction %s, capturePhoto %s, prepare %s)", send_event ? "yes" : "no",
	    send_action ? "yes" : "no", capture_photo ? "yes" : "no", prepare_photos ? "yes" : "no");
}

__attribute__((constructor))
static void
hakios_app_init(void)
{
	static const int signals[] = { SIGTRAP, SIGABRT, SIGILL, SIGSEGV, SIGBUS, SIGSYS };
	struct sigaction action = { 0 };
	const char *name = getprogname();
	char line[200];
	size_t length = 0;
	int n;

	if (name == NULL) {
		return;
	}
	if (strcmp(name, "Camera") == 0) {
		bool trace = save_trace_enabled();
		dispatch_async(dispatch_get_main_queue(), ^{
			camera_enable_repeat_focus();
			if (trace) {
				camera_enable_save_trace();
			}
		});
		return;
	}
	who = name;
	slide = (uintptr_t)_dyld_get_shared_cache_range(&length) - CACHE_BASE;
	/*
	 * An application's standard error goes nowhere here, and that is where the Swift runtime says
	 * what it is about to die of ("Fatal error: ..."), so it is pointed at the console.
	 */
	console = open("/dev/console", O_WRONLY);
	if (console >= 0) {
		dup2(console, 2);
	}
	action.sa_sigaction = killed;
	action.sa_flags = SA_SIGINFO | SA_NODEFER | SA_RESETHAND;
	sigemptyset(&action.sa_mask);
	for (unsigned i = 0; i < sizeof(signals) / sizeof(signals[0]); i++) {
		sigaction(signals[i], &action, NULL);
	}
	if (strcmp(name, "Sileo") == 0) {
		dispatch_async(dispatch_get_main_queue(), ^{ keep_to_the_repositories(); });
	}
	watch("UIApplication", "beginBackgroundTaskWithExpirationHandler:", (IMP)watched_begin_task, &begin_task);
	/*
	 * NSURLSessionTask alone: its subclasses inherit the very same method, and watching it twice
	 * would leave ours as the "original" of itself -- a call that never returns.
	 */
	watch("NSURLSessionTask", "resume", (IMP)watched_resume_task, &resume_task);
	watch("NSFileManager", "moveItemAtURL:toURL:error:", (IMP)watched_move_url, &move_url);
	watch("NSFileManager", "copyItemAtURL:toURL:error:", (IMP)watched_copy_url, &copy_url);
	watch("NSFileManager", "moveItemAtPath:toPath:error:", (IMP)watched_move_path, &move_path);
	watch("NSFileManager", "copyItemAtPath:toPath:error:", (IMP)watched_copy_path, &copy_path);
	n = snprintf(line, sizeof(line), "hakios: %s: started as uid %d (effective %d), watching for what kills it\n",
	    name, (int)getuid(), (int)geteuid());
	say(line, (size_t)(n > 0 ? n : 0));
}
