/*
 * oslogshow: the system log for the named processes (what `log show` and `log stream` print on a Mac; iOS has no log
 * tool). Through LoggingSupport, the framework under both: OSLogEventStore + OSLogEventStream for what logd has kept,
 * OSLogEventLiveStore + OSLogEventLiveStream for what is said from now on (through diagnosticd; info and debug lines
 * too, which logd does not keep). OSLog's OSLogStore has no system scope on iOS: it answered nil with no error.
 * Entitlements: oslogshow.entitlements (ldid -S).
 *
 *   oslogshow show SECONDS [PROCESS ...]             what was kept of the last SECONDS
 *   oslogshow stream SECONDS FILE [PROCESS ...]      in the background: SECONDS of what is said, into FILE
 *
 * Every process if none is named. Why it is here (COMMON-PORT.md "System sleep"): after a sleep woken by the power key,
 * SpringBoard took no notice of the first press; its own lines say why.
 */
#include <CoreFoundation/CoreFoundation.h>
#include <dispatch/dispatch.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <objc/message.h>
#include <objc/runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

extern void *objc_autoreleasePoolPush(void);
extern void objc_autoreleasePoolPop(void *pool);

static int nwanted;
static char **wanted;
static FILE *out;
static unsigned shown, seen;
static bool dump;     /* (oslogshow dump: show, and the first entries field by field) */

static SEL
sel(const char *name)
{
	return sel_registerName(name);
}

static bool
answers(id o, const char *name)
{
	/* (by the class: OSLogEventProxy's own respondsToSelector: answers NO for its accessors) */
	return o != nil && class_respondsToSelector(object_getClass(o), sel(name));
}

static id
get(id o, const char *name)
{
	return answers(o, name) ? ((id (*)(id, SEL))objc_msgSend)(o, sel(name)) : nil;
}

static const char *
text(id s, char *buffer, size_t size)
{
	buffer[0] = '\0';
	if (s != nil && CFGetTypeID((CFTypeRef)s) == CFStringGetTypeID()) {
		CFStringGetCString((CFStringRef)s, buffer, (CFIndex)size, kCFStringEncodingUTF8);
	}
	return buffer;
}

static void
say_error(const char *what, id error)
{
	char line[1024];

	fprintf(out, "%s: %s\n", what, error != nil ? text(get(error, "description"), line, sizeof(line)) : "(no error)");
	fflush(out);
}

/* one event (an OSLogEventProxy), if its process is wanted */
static void
event(id e)
{
	static char message[16384], process[256], cat[128];
	void *pool = objc_autoreleasePoolPush();
	bool want = nwanted == 0;

	seen++;
	if (seen <= 3 && dump) {
		static const char *const fields[] = { "process", "processImagePath", "processIdentifier", "sender",
			"senderImagePath", "subsystem", "category", "composedMessage", "formatString", "type", "logType" };

		fprintf(out, "entry %u: class %s\n", seen, object_getClassName(e));
		if (seen == 1) {
			for (Class c = object_getClass(e); c != Nil; c = class_getSuperclass(c)) {
				unsigned n = 0;
				Method *m = class_copyMethodList(c, &n);

				fprintf(out, " methods of %s (%u):", class_getName(c), n);
				for (unsigned i = 0; i < n && i < 200; i++) {
					fprintf(out, " %s", sel_getName(method_getName(m[i])));
				}
				fprintf(out, "\n");
				free(m);
				if (strcmp(class_getName(c), "NSObject") == 0) {
					break;
				}
			}
		}
		for (unsigned i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
			bool has = answers(e, fields[i]);
			id v = has && (strcmp(fields[i], "processIdentifier") != 0 && strcmp(fields[i], "type") != 0 &&
			    strcmp(fields[i], "logType") != 0) ? get(e, fields[i]) : nil;

			fprintf(out, "  %s: %s %s\n", fields[i], has ? "answers" : "-",
			    v != nil ? text(get(v, "description"), message, 512) : "");
		}
	}
	/* (the process by its image's name: "process" was empty in every entry, run-20261126-071123: 0 of 179974) */
	text(get(e, "processImagePath"), process, sizeof(process));
	if (strrchr(process, '/') != NULL) {
		memmove(process, strrchr(process, '/') + 1, strlen(strrchr(process, '/')));
	}
	if (process[0] == '\0') {
		text(get(e, "process"), process, sizeof(process));
	}
	for (int i = 0; i < nwanted && !want; i++) {
		want = strcmp(process, wanted[i]) == 0;
	}
	if (want) {
		id when = get(e, "date");
		double t = when != nil ? ((double (*)(id, SEL))objc_msgSend)(when, sel("timeIntervalSince1970")) : 0;
		time_t secs = (time_t)t;
		struct tm tm;
		unsigned type = answers(e, "logType") ? ((unsigned char (*)(id, SEL))objc_msgSend)(e, sel("logType")) : 255;

		text(get(e, "category"), cat, sizeof(cat));
		gmtime_r(&secs, &tm);
		text(get(e, "composedMessage"), message, sizeof(message));
		/*
		 * (without logd's table of the shared cache's strings, /private/var/db/uuidtext/dsc, every message is a
		 *  "<compose failure [shared UUID]>": the place it was said from is printed instead, image and offset, and
		 *  the PC's copy of the cache gives its format string)
		 */
		char where[320] = "";
		if (strncmp(message, "<compose failure", 16) == 0) {
			char image[256];
			unsigned long long offset = answers(e, "senderImageOffset") ?
			    ((unsigned long long (*)(id, SEL))objc_msgSend)(e, sel("senderImageOffset")) : 0;

			text(get(e, "senderImagePath"), image, sizeof(image));
			snprintf(where, sizeof(where), " @ %s+%#llx", image, offset);
		}
		fprintf(out, "%02d:%02d:%02d.%03d %s %#x %s: %s%s\n", tm.tm_hour, tm.tm_min, tm.tm_sec,
		    (int)((t - (double)secs) * 1000), process, type, cat, message, where);
		fflush(out);
		shown++;
	}
	objc_autoreleasePoolPop(pool);
}

/* a store's source, by its prepareWithCompletionHandler: */
static id
prepare(id store)
{
	dispatch_semaphore_t done = dispatch_semaphore_create(0);
	__block id source = nil;

	((void (*)(id, SEL, void (^)(id, id)))objc_msgSend)(store, sel("prepareWithCompletionHandler:"), ^(id s, id error) {
		source = s;
		if (s == nil) {
			say_error("prepare", error);
		} else {
			CFRetain((CFTypeRef)s);
		}
		dispatch_semaphore_signal(done);
	});
	if (dispatch_semaphore_wait(done, dispatch_time(DISPATCH_TIME_NOW, 30 * NSEC_PER_SEC)) != 0) {
		fprintf(out, "prepare: no answer in 30 s\n");
		return nil;
	}
	return source;
}

static int
show(double seconds)
{
	Class store_class = objc_getClass("OSLogEventStore"), stream_class = objc_getClass("OSLogEventStream");
	Class date_class = objc_getClass("NSDate");
	dispatch_semaphore_t done = dispatch_semaphore_create(0);
	id store, source, stream, from, to;

	if (store_class == Nil || stream_class == Nil) {
		fprintf(out, "no OSLogEventStore or OSLogEventStream\n");
		return 1;
	}
	store = ((id (*)(Class, SEL))objc_msgSend)(store_class, sel("localStore"));
	if (store == nil || (source = prepare(store)) == nil) {
		fprintf(out, "no local store\n");
		return 1;
	}
	stream = ((id (*)(id, SEL, id))objc_msgSend)(((id (*)(Class, SEL))objc_msgSend)(stream_class, sel("alloc")),
	    sel("initWithSource:"), source);
	((void (*)(id, SEL, void (^)(id)))objc_msgSend)(stream, sel("setEventHandler:"), ^(id e) {
		event(e);
	});
	((void (*)(id, SEL, void (^)(unsigned long, id)))objc_msgSend)(stream, sel("setInvalidationHandler:"),
	    ^(unsigned long reason, __unused id position) {
		fprintf(out, "--- the stream ends (%lu)\n", reason);
		dispatch_semaphore_signal(done);
	});
	from = ((id (*)(Class, SEL, double))objc_msgSend)(date_class, sel("dateWithTimeIntervalSinceNow:"), -seconds);
	to = ((id (*)(Class, SEL))objc_msgSend)(date_class, sel("date"));
	((void (*)(id, SEL, id, id))objc_msgSend)(stream, sel("activateStreamFromDate:toDate:"), from, to);
	if (dispatch_semaphore_wait(done, dispatch_time(DISPATCH_TIME_NOW, 120 * NSEC_PER_SEC)) != 0) {
		fprintf(out, "--- not finished in 120 s\n");
	}
	fprintf(out, "--- %u of %u entries kept of the last %.0f s\n", shown, seen, seconds);
	return 0;
}

static int
stream_live(double seconds)
{
	Class store_class = objc_getClass("OSLogEventLiveStore"), stream_class = objc_getClass("OSLogEventLiveStream");
	id store, source, stream;

	if (store_class == Nil || stream_class == Nil) {
		fprintf(out, "no OSLogEventLiveStore or OSLogEventLiveStream\n");
		return 1;
	}
	store = ((id (*)(Class, SEL))objc_msgSend)(store_class, sel("liveLocalStore"));
	if (store == nil || (source = prepare(store)) == nil) {
		fprintf(out, "no live store\n");
		return 1;
	}
	stream = ((id (*)(id, SEL, id))objc_msgSend)(((id (*)(Class, SEL))objc_msgSend)(stream_class, sel("alloc")),
	    sel("initWithLiveSource:"), source);
	if (answers(stream, "setFlags:")) {
		/* (as `log stream --level debug`: info and debug messages too) */
		unsigned long flags = getenv("OSLOG_FLAGS") != NULL ? strtoul(getenv("OSLOG_FLAGS"), NULL, 0) : 0x3;

		((void (*)(id, SEL, unsigned long))objc_msgSend)(stream, sel("setFlags:"), flags);
		fprintf(out, "--- flags %#lx\n", flags);
	}
	((void (*)(id, SEL, void (^)(id)))objc_msgSend)(stream, sel("setEventHandler:"), ^(id e) {
		event(e);
	});
	((void (*)(id, SEL, void (^)(unsigned long)))objc_msgSend)(stream, sel("setInvalidationHandler:"), ^(unsigned long r) {
		fprintf(out, "--- the live stream ends (%lu)\n", r);
		fflush(out);
	});
	((void (*)(id, SEL))objc_msgSend)(stream, sel("activate"));
	fprintf(out, "--- streaming for %.0f s\n", seconds);
	fflush(out);
	sleep((unsigned)seconds);
	fprintf(out, "--- %u of %u events in %.0f s\n", shown, seen, seconds);
	return 0;
}

int
main(int argc, char **argv)
{
	out = stdout;
	if (argc < 3 || (strcmp(argv[1], "stream") == 0 && argc < 4)) {
		fprintf(stderr, "usage: oslogshow show SECONDS [PROCESS ...] | stream SECONDS FILE [PROCESS ...]\n");
		return 2;
	}
	dlopen("/System/Library/Frameworks/Foundation.framework/Foundation", RTLD_NOW);
	if (dlopen("/System/Library/PrivateFrameworks/LoggingSupport.framework/LoggingSupport", RTLD_NOW) == NULL) {
		printf("no LoggingSupport.framework: %s\n", dlerror());
		return 1;
	}
	if (strcmp(argv[1], "stream") == 0) {
		int fd = open(argv[3], O_WRONLY | O_CREAT | O_TRUNC, 0644);

		if (fd < 0) {
			perror(argv[3]);
			return 1;
		}
		wanted = argv + 4;
		nwanted = argc - 4;
		/* (in the background, all of its descriptors its own: the caller's connection ends at once) */
		if (fork() != 0) {
			printf("streaming into %s\n", argv[3]);
			return 0;
		}
		setsid();
		dup2(fd, 1);
		dup2(fd, 2);
		close(fd);
		int null = open("/dev/null", O_RDONLY);
		if (null >= 0) {
			dup2(null, 0);
			close(null);
		}
		out = fdopen(1, "w");
		return stream_live(atof(argv[2]));
	}
	wanted = argv + 3;
	nwanted = argc - 3;
	dump = strcmp(argv[1], "dump") == 0;
	return show(atof(argv[2]));
}
