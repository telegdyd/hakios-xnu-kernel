/*
 * savetrace: where a photograph's time between the shutter and DCIM goes, measured on the phone.
 *
 * It presses the shutter itself (through /dev/hakios-touch, as rwtouch's tap: down, 60 ms, up), then every 10 ms
 * looks at DCIM's folders and the times of the Photos library's database files, and takes every process's
 * processor time. It says, on its standard output and on the console (where mediaserverd's lines are, with
 * cam_save_trace=1 on the same clock):
 *
 *   savetrace: [t=1234.567] +0 ms touch down         each event, as time since boot and since the touch
 *   savetrace: [t=...] +1830 ms new IMG_0060.HEIC 0 bytes
 *   savetrace: +0..100 ms: 212% mediaserverd 150% Camera 40% ...   who had the processors, every 100 ms
 *   savetrace: in all: mediaserverd 812 ms, assetsd 430 ms, ...    and over the whole run
 *
 * The time a process is given while the others wait on it shows as a busy process; a stretch with nothing much
 * busy is a wait (a timer, an animation, a reply).
 *
 *   usage: savetrace [X Y [SECONDS]]     the shutter at X Y (fractions of the screen, default 0.5 0.885);
 *                                        watch until SECONDS after the touch (default 6)
 */
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <mach/mach_time.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/qos.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "hakios.input.h"

/* libproc's (see cputop.c) */
#define PROC_ALL_PIDS           1
#define PROC_PIDTASKINFO        4
struct proc_taskinfo {
	uint64_t pti_virtual_size, pti_resident_size, pti_total_user, pti_total_system, pti_threads_user,
	    pti_threads_system;
	int32_t pti_policy, pti_faults, pti_pageins, pti_cow_faults, pti_messages_sent, pti_messages_received,
	    pti_syscalls_mach, pti_syscalls_unix, pti_csw, pti_threadnum, pti_numrunning, pti_priority;
};
int proc_listpids(uint32_t type, uint32_t typeinfo, void *buffer, int buffersize);
int proc_pidinfo(int pid, int flavor, uint64_t arg, void *buffer, int buffersize);
int proc_name(int pid, void *buffer, uint32_t buffersize);

#define DCIM            "/var/mobile/Media/DCIM"
#define PHOTO_DATA      "/var/mobile/Media/PhotoData"
#define LOOK_MS         10
#define BUCKET_MS       100
#define MAX_BUCKETS     200
#define MAX_PROCS       1024
#define MAX_FILES       4096
#define SHOWN           6

static mach_timebase_info_data_t base;
static uint64_t touched;                /* mach_absolute_time of the touch */
static int console = -1;

static uint64_t
us_of(uint64_t ticks)
{
	return ticks * base.numer / base.denom / 1000;
}

static void
event(const char *format, ...)
{
	char line[400];
	va_list ap;
	uint64_t now = mach_absolute_time(), us = us_of(now);
	long since = touched != 0 ? (long)((int64_t)us - (int64_t)us_of(touched)) / 1000 : 0;
	int n = snprintf(line, sizeof(line), "savetrace: [t=%llu.%03llu] +%ld ms ", (unsigned long long)(us / 1000),
	    (unsigned long long)(us % 1000), since);

	va_start(ap, format);
	n += vsnprintf(line + n, sizeof(line) - (size_t)n - 1, format, ap);
	va_end(ap);
	if (n > (int)sizeof(line) - 2) {
		n = sizeof(line) - 2;
	}
	line[n++] = '\n';
	fwrite(line, 1, (size_t)n, stdout);
	if (console >= 0) {
		write(console, line, (size_t)n);
	}
}

/* ---- DCIM's files ---------------------------------------------------------------------------------------- */

struct file {
	char name[96];
	off_t size;
	uint64_t stable_since;          /* when its size last changed */
	bool reported_stable;
};
static struct file files[MAX_FILES];
static int file_count;

static struct file *
known(const char *name)
{
	for (int i = 0; i < file_count; i++) {
		if (strcmp(files[i].name, name) == 0) {
			return &files[i];
		}
	}
	return NULL;
}

/* Every file under DCIM's folders; `report` says what is new or changed. */
static void
look_at_dcim(bool report)
{
	DIR *top = opendir(DCIM);
	struct dirent *folder;

	if (top == NULL) {
		return;
	}
	while ((folder = readdir(top)) != NULL) {
		char path[160];
		DIR *inside;
		struct dirent *entry;

		if (folder->d_name[0] == '.') {
			continue;
		}
		snprintf(path, sizeof(path), DCIM "/%s", folder->d_name);
		if ((inside = opendir(path)) == NULL) {
			continue;
		}
		while ((entry = readdir(inside)) != NULL) {
			char name[96], full[300];
			struct stat st;
			struct file *f;

			if (entry->d_name[0] == '.' && (entry->d_name[1] == 0 || entry->d_name[1] == '.')) {
				continue;
			}
			snprintf(name, sizeof(name), "%s/%s", folder->d_name, entry->d_name);
			snprintf(full, sizeof(full), "%s/%s", path, entry->d_name);
			if (stat(full, &st) != 0) {
				continue;
			}
			if ((f = known(name)) == NULL) {
				if (file_count == MAX_FILES) {
					continue;
				}
				f = &files[file_count++];
				snprintf(f->name, sizeof(f->name), "%s", name);
				f->size = st.st_size;
				f->stable_since = mach_absolute_time();
				f->reported_stable = !report;
				if (report) {
					event("new %s %lld bytes", name, (long long)st.st_size);
				}
			} else if (st.st_size != f->size) {
				if (report) {
					event("%s now %lld bytes", name, (long long)st.st_size);
				}
				f->size = st.st_size;
				f->stable_since = mach_absolute_time();
				f->reported_stable = false;
			} else if (report && !f->reported_stable && us_of(mach_absolute_time() - f->stable_since) >= 300000) {
				f->reported_stable = true;
				event("%s unchanged at %lld bytes for 300 ms (since the size above)", name, (long long)f->size);
			}
		}
		closedir(inside);
	}
	closedir(top);
}

/* ---- the Photos library's database ------------------------------------------------------------------------ */

static const char *const watched[] = {
	PHOTO_DATA "/Photos.sqlite", PHOTO_DATA "/Photos.sqlite-wal", PHOTO_DATA "/Photos.sqlite-shm",
};
static struct timespec watched_time[sizeof(watched) / sizeof(watched[0])];
static off_t watched_size[sizeof(watched) / sizeof(watched[0])];

static void
look_at_library(bool report)
{
	for (unsigned i = 0; i < sizeof(watched) / sizeof(watched[0]); i++) {
		struct stat st;

		if (stat(watched[i], &st) != 0) {
			continue;
		}
		if (report && (st.st_mtimespec.tv_sec != watched_time[i].tv_sec ||
		    st.st_mtimespec.tv_nsec != watched_time[i].tv_nsec || st.st_size != watched_size[i])) {
			event("%s written (%lld bytes)", strrchr(watched[i], '/') + 1, (long long)st.st_size);
		}
		watched_time[i] = st.st_mtimespec;
		watched_size[i] = st.st_size;
	}
}

/* ---- processor time ---------------------------------------------------------------------------------------- */

struct proc {
	int pid;
	char name[32];
	uint64_t last;                  /* user + system at the last look, mach_absolute_time's units */
	uint64_t bucket[MAX_BUCKETS];
	uint64_t total;
	bool seen;
};
static struct proc procs[MAX_PROCS];
static int proc_count;

static struct proc *
proc_of(int pid)
{
	for (int i = 0; i < proc_count; i++) {
		if (procs[i].pid == pid) {
			return &procs[i];
		}
	}
	if (proc_count == MAX_PROCS) {
		return NULL;
	}
	struct proc *p = &procs[proc_count++];
	memset(p, 0, sizeof(*p));
	p->pid = pid;
	proc_name(pid, p->name, sizeof(p->name));
	return p;
}

static void
look_at_processors(int bucket)
{
	static int pids[MAX_PROCS];
	int bytes = proc_listpids(PROC_ALL_PIDS, 0, pids, sizeof(pids));

	for (int i = 0; i < bytes / (int)sizeof(int); i++) {
		struct proc_taskinfo info;
		struct proc *p;
		bool born;

		if (pids[i] <= 0 || proc_pidinfo(pids[i], PROC_PIDTASKINFO, 0, &info, sizeof(info)) != sizeof(info)) {
			continue;
		}
		born = false;
		for (int k = 0; k < proc_count; k++) {
			if (procs[k].pid == pids[i]) {
				born = true;
				break;
			}
		}
		born = !born;
		if ((p = proc_of(pids[i])) == NULL) {
			continue;
		}
		uint64_t now = info.pti_total_user + info.pti_total_system;
		if (bucket >= 0 && now > p->last) {
			uint64_t given = born ? now : now - p->last;    /* (one born since: all it has had, under 10 ms old) */
			if (born) {
				event("process %s[%d] started", p->name, p->pid);
			}
			p->bucket[bucket < MAX_BUCKETS ? bucket : MAX_BUCKETS - 1] += given;
			p->total += given;
		}
		p->last = now;
	}
}

static int
by_bucket_cmp(const void *a, const void *b, int bucket)
{
	const struct proc *x = *(const struct proc *const *)a, *y = *(const struct proc *const *)b;
	uint64_t u = bucket < 0 ? x->total : x->bucket[bucket], v = bucket < 0 ? y->total : y->bucket[bucket];

	return u < v ? 1 : u > v ? -1 : 0;
}

static int sort_bucket;

static int
by_bucket(const void *a, const void *b)
{
	return by_bucket_cmp(a, b, sort_bucket);
}

static void
say_shares(int buckets)
{
	static struct proc *order[MAX_PROCS];
	char line[600];

	for (int i = 0; i < proc_count; i++) {
		order[i] = &procs[i];
	}
	for (int b = 0; b <= buckets; b++) {
		bool all = b == buckets;
		uint64_t sum = 0;
		int n = 0;

		sort_bucket = all ? -1 : b;
		qsort(order, (size_t)proc_count, sizeof(order[0]), by_bucket);
		for (int i = 0; i < proc_count; i++) {
			sum += all ? order[i]->total : order[i]->bucket[b];
		}
		if (all) {
			n = snprintf(line, sizeof(line), "savetrace: in all (%d ms):", buckets * BUCKET_MS);
		} else {
			n = snprintf(line, sizeof(line), "savetrace: +%d..%d ms: %3.0f%%", b * BUCKET_MS, (b + 1) * BUCKET_MS,
			    100.0 * (double)us_of(sum) / (BUCKET_MS * 1000.0));
		}
		for (int i = 0; i < proc_count && i < (all ? 14 : SHOWN) && n < (int)sizeof(line) - 60; i++) {
			uint64_t t = all ? order[i]->total : order[i]->bucket[b];
			if (us_of(t) < (all ? 5000 : 2000)) {
				break;
			}
			if (all) {
				n += snprintf(line + n, sizeof(line) - (size_t)n, " %s %llu ms", order[i]->name,
				    (unsigned long long)(us_of(t) / 1000));
			} else {
				n += snprintf(line + n, sizeof(line) - (size_t)n, " %s %.0f%%", order[i]->name,
				    100.0 * (double)us_of(t) / (BUCKET_MS * 1000.0));
			}
		}
		printf("%s\n", line);
		if (console >= 0) {
			line[n++] = '\n';
			write(console, line, (size_t)n);
		}
	}
}

/* ---- the shutter -------------------------------------------------------------------------------------------- */

static void
touch(int device, uint32_t type, double x, double y)
{
	struct hakios_touch t = { .type = type, .finger = 0, .x = (uint32_t)(x * 65535.0 + 0.5), .y = (uint32_t)(y * 65535.0 + 0.5) };

	if (write(device, &t, sizeof(t)) != (ssize_t)sizeof(t)) {
		perror("savetrace: /dev/hakios-touch");
		exit(1);
	}
}

/* ---- a stackshot of one process (as kstacks.c, with its user stacks) ------------------------------------------- */

typedef struct {
	int             sc_pid;
	uint64_t        sc_flags;
	uint64_t        sc_delta_timestamp;
	uint64_t        sc_buffer;
	uint32_t        sc_size;
	uint64_t        sc_out_buffer_addr;
	uint64_t        sc_out_size_addr;
	uint64_t        sc_pagetable_mask;
} stackshot_config_t;
#define SYS_stack_snapshot_with_config  491

static int
pid_named(const char *wanted)
{
	static int pids[MAX_PROCS];
	int bytes = proc_listpids(PROC_ALL_PIDS, 0, pids, sizeof(pids));

	for (int i = 0; i < bytes / (int)sizeof(int); i++) {
		char name[64] = "";

		if (pids[i] > 0 && proc_name(pids[i], name, sizeof(name)) > 0 && strcmp(name, wanted) == 0) {
			return pids[i];
		}
	}
	return -1;
}

static void
stackshot(int pid, const char *path)
{
	stackshot_config_t config;
	uint64_t buffer = 0;
	uint32_t size = 0;

	memset(&config, 0, sizeof(config));
	config.sc_pid = pid;
	/* kcdata, thread wait info, no I/O statistics or latency, kext and dyld load info */
	config.sc_flags = 0x10000ull | 0x1000000ull | 0x800000ull | 0x40000000ull | 0x08ull | 0x02ull;
	config.sc_out_buffer_addr = (uint64_t)(uintptr_t)&buffer;
	config.sc_out_size_addr = (uint64_t)(uintptr_t)&size;
	if (syscall(SYS_stack_snapshot_with_config, 1, &config, sizeof(config)) != 0) {
		event("stackshot of %d failed (%d)", pid, errno);
		return;
	}
	int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (fd < 0 || write(fd, (const void *)(uintptr_t)buffer, size) != (ssize_t)size) {
		event("stackshot not written to %s (%d)", path, errno);
	} else {
		event("stackshot of %d: %u bytes in %s", pid, size, path);
	}
	if (fd >= 0) {
		close(fd);
	}
}

int
main(int argc, char **argv)
{
	double x = argc > 2 ? atof(argv[1]) : 0.5, y = argc > 2 ? atof(argv[2]) : 0.885;
	unsigned seconds = argc > 3 ? (unsigned)atoi(argv[3]) : 6;
	/* savetrace X Y SECONDS stackshot NAME MS FILE: a stackshot of NAME MS after the touch */
	bool shoot = argc > 7 && strcmp(argv[4], "stackshot") == 0;
	int shot_pid = shoot ? pid_named(argv[5]) : -1;
	unsigned shot_ms = shoot ? (unsigned)atoi(argv[6]) : 0;
	int device = open("/dev/hakios-touch", O_WRONLY);
	bool up = false;

	if (device < 0) {
		perror("savetrace: /dev/hakios-touch");
		return 1;
	}
	pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
	mach_timebase_info(&base);
	setvbuf(stdout, NULL, _IOLBF, 0);
	console = open("/dev/console", O_WRONLY);
	look_at_dcim(false);
	look_at_library(false);
	look_at_processors(-1);

	touched = mach_absolute_time();
	touch(device, HAKIOS_TOUCH_DOWN, x, y);
	event("touch down at %.3f %.3f", x, y);
	for (uint64_t next = touched;;) {
		uint64_t now = mach_absolute_time(), since_ms = us_of(now - touched) / 1000;

		if (!up && since_ms >= 60) {
			touch(device, HAKIOS_TOUCH_UP, x, y);
			up = true;
			event("touch up");
		}
		if (shot_pid > 0 && since_ms >= shot_ms) {
			stackshot(shot_pid, argv[7]);
			shot_pid = -1;
		}
		look_at_processors((int)(since_ms / BUCKET_MS));
		look_at_dcim(true);
		look_at_library(true);
		if (since_ms >= seconds * 1000ull || since_ms / BUCKET_MS >= MAX_BUCKETS) {
			break;
		}
		next += (uint64_t)LOOK_MS * 1000000ull * base.denom / base.numer;
		now = mach_absolute_time();
		if (next > now) {
			usleep((useconds_t)us_of(next - now));
		}
	}
	event("done");
	say_shares((int)(us_of(mach_absolute_time() - touched) / 1000 / BUCKET_MS));
	return 0;
}
