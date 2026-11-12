/*
 * clock_keep: the phone's date from one boot to the next (WIFI-PLAN.md I8).
 *
 * The kernel starts at 1970: nothing gives it the date on this phone. The PMK8350's real-time clock is a counter
 * of seconds that goes on while the phone is off, not a date (kern.hakios.rtc_counter, drivers/input/
 * hakios.buttons.c), so, as Android does, the difference between it and real time is kept in a file. timed sets
 * the real time once the network is up; this job then writes "offset last" to OFFSET_FILE (offset = the date
 * minus the counter, last = the date when it was written). At the next boot, before anything has set the clock,
 * it sets it to counter + offset -- or to "last" if that is later, which means the counter started again from
 * nothing (the battery ran flat) and the offset no longer holds; timed corrects it when the network is up.
 *
 * It matters to Wi-Fi: wifid stamps a join with the date, and a network last joined "in 1970" is one CoreWiFi's
 * auto-join leaves out once the date is right ("Known network profile unused for 18 months").
 *
 * A launchd job (com.hakios.clock.plist), run at load and kept alive: the date at start, then a save whenever the
 * clock is set (the notification com.apple.system.clock_set) and once an hour. Root: settimeofday() is allowed to
 * root. Writes a line to stdout (the job's log) for each thing it does.
 */
#include <dispatch/dispatch.h>
#include <errno.h>
#include <fcntl.h>
#include <notify.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define OFFSET_FILE     "/private/var/db/hakios-clock"
#define REAL_DATE       1735689600L     /* 2025-01-01: a clock at or after this has been set */

static void
say(const char *format, ...) __attribute__((format(printf, 1, 2)));

static void
say(const char *format, ...)
{
	char stamp[32];
	time_t now = time(NULL);
	struct tm tm;
	va_list list;

	gmtime_r(&now, &tm);
	strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &tm);
	printf("%s UTC clock_keep: ", stamp);
	va_start(list, format);
	vprintf(format, list);
	va_end(list);
	putchar('\n');
	fflush(stdout);
}

static int64_t
counter(void)
{
	int64_t value = -1;
	size_t size = sizeof(value);

	if (sysctlbyname("kern.hakios.rtc_counter", &value, &size, NULL, 0) != 0) {
		return -1;
	}
	return value;
}

static int64_t saved_offset, saved_last;
static int have_saved;

static void
load(void)
{
	FILE *f = fopen(OFFSET_FILE, "r");
	long long offset, last;

	if (f == NULL) {
		return;
	}
	if (fscanf(f, "%lld %lld", &offset, &last) == 2) {
		saved_offset = offset;
		saved_last = last;
		have_saved = 1;
	}
	fclose(f);
}

/* Write "offset last" if the clock is a real date and the offset moved, or an hour has gone by since "last". */
static void
save(const char *why)
{
	int64_t c = counter(), now = (int64_t)time(NULL), offset;
	char line[64];
	int fd, n;

	if (c < 0 || now < REAL_DATE) {
		return;
	}
	offset = now - c;
	if (have_saved && llabs((long long)(offset - saved_offset)) <= 2 && now - saved_last < 3600) {
		return;
	}
	n = snprintf(line, sizeof(line), "%lld %lld\n", (long long)offset, (long long)now);
	fd = open(OFFSET_FILE ".new", O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (fd < 0 || write(fd, line, (size_t)n) != n || fsync(fd) != 0 || close(fd) != 0 ||
	    rename(OFFSET_FILE ".new", OFFSET_FILE) != 0) {
		say("cannot write %s: %s", OFFSET_FILE, strerror(errno));
		if (fd >= 0) {
			close(fd);
		}
		return;
	}
	if (!have_saved || llabs((long long)(offset - saved_offset)) > 2) {
		say("%s: saved offset %lld (counter %lld)%s", why, (long long)offset, (long long)c,
		    have_saved ? "" : ", the first");
	}
	saved_offset = offset;
	saved_last = now;
	have_saved = 1;
}

static void
set_at_boot(void)
{
	int64_t c = counter(), now = (int64_t)time(NULL), date;
	struct timeval tv = { 0, 0 };

	if (c < 0) {
		say("no kern.hakios.rtc_counter (the PMIC clock was not read): the date is left to timed");
		return;
	}
	if (now >= REAL_DATE) {
		say("the clock is set already (counter %lld)", (long long)c);
		return;
	}
	if (!have_saved) {
		say("no %s yet: the date is left to timed (counter %lld)", OFFSET_FILE, (long long)c);
		return;
	}
	date = c + saved_offset;
	if (date < saved_last) {
		say("counter %lld + offset %lld is before the last saved date: the counter started again; using that date",
		    (long long)c, (long long)saved_offset);
		date = saved_last;
	}
	tv.tv_sec = (time_t)date;
	if (settimeofday(&tv, NULL) != 0) {
		say("settimeofday: %s", strerror(errno));
		return;
	}
	say("set the date from counter %lld + offset %lld", (long long)c, (long long)saved_offset);
}

int
main(void)
{
	int token;
	dispatch_source_t hourly;

	load();
	set_at_boot();
	save("at start");
	if (notify_register_dispatch("com.apple.system.clock_set", &token, dispatch_get_main_queue(), ^(__unused int t) {
		save("the clock was set");
	}) != NOTIFY_STATUS_OK) {
		say("cannot register for com.apple.system.clock_set: saving hourly only");
	}
	hourly = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, dispatch_get_main_queue());
	dispatch_source_set_timer(hourly, dispatch_time(DISPATCH_TIME_NOW, 60 * NSEC_PER_SEC), 3600 * NSEC_PER_SEC,
	    60 * NSEC_PER_SEC);
	dispatch_source_set_event_handler(hourly, ^{
		save("hourly");
	});
	dispatch_resume(hourly);
	dispatch_main();
}
