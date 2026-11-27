/*
 * /dev/hakios-touch: a queue of touch samples (see sys/board/input.h).
 *
 * The node is open to every user, as the window server reading it (inject26.c, in backboardd) is
 * not root; but what goes through it is every touch on the screen, a passcode's among them, and
 * what is written to it is taken for touches. So:
 *   - only root writes (rwtouch, from the run's jobs or a shell): any other program's samples
 *     would be taps on whatever the screen shows -- an "Allow", a purchase -- refused with EPERM;
 *   - one program reads, the first to open the node for reading (the window server, at its
 *     start): a second gets EBUSY while the first is alive. Two readers would share the queue,
 *     each taking samples from the other -- touches lost to the screen -- and a second could be
 *     any application, watching the passcode typed. Root is let through (a look with rwtouch
 *     read, knowing that it takes them).
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <miscfs/devfs/devfs.h>
#include <sys/conf.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/kauth.h>
#include <sys/proc.h>
#include <sys/board/input.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/uio.h>
#include <pexpert/pexpert.h>

#include "kernel.h"

#define TOUCH_QUEUE     512             /* records; a power of two */

/*
 * Boot-arg touch_lat=1: for measuring how long a touch takes to reach the panel, a line for every
 * sample posted ("tl: p TYPE FINGER X Y at US", the time in microseconds of mach_absolute_time),
 * to be matched with the window server's lines about reading it (userland/src/inject.c), drawing
 * (eagl.c) and the display driver's about the frame (MiniIOMobileFramebuffer.cpp): see
 * tools/latency.py.
 */
static uint32_t touch_lat = 2;          /* 2: not yet looked up */

extern void loader_log_line(const char *line);

/*
 * A "tl:" line, for the kernel's drivers: into the loader's log as it is written -- the kernel's own
 * log keeps only the last of a run, and a live look at the loader's (BENCH_FOLLOW) then misses nothing.
 */
void hakios_tl(const char *format, ...) __printflike(1, 2);

void
hakios_tl(const char *format, ...)
{
	char line[192];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	loader_log_line(line);
}

static bool
latency_lines(void)
{
	if (touch_lat == 2) {
		uint32_t on = 0;

		touch_lat = PE_parse_boot_argn("touch_lat", &on, sizeof(on)) && on != 0;
	}
	return touch_lat != 0;
}

static LCK_GRP_DECLARE(touch_lck_grp, "hakios-touch");
static LCK_MTX_DECLARE(touch_lock, &touch_lck_grp);

/*
 * the application processor's RPMh command slots (DRV2, TCS0): the touch driver's, the Wi-Fi's, the memory vote's -- and the
 * modem's rails (mpss.c), which take the SoC's lock for the same slots: one lock for all of them, soc.c's (two locks let
 * the modem and the others write one slot at once)
 */
extern lck_mtx_t hakios_rsc_lock;
void hakios_rpmh_lock(void) { lck_mtx_lock(&hakios_rsc_lock); }
void hakios_rpmh_unlock(void) { lck_mtx_unlock(&hakios_rsc_lock); }
static struct hakios_touch touch_queue[TOUCH_QUEUE];
static uint32_t touch_head, touch_tail;        /* read from head, add at tail */

/*
 * The way out when nothing else is at hand: five fingers held on the screen for a second and a
 * half end the run -- control goes back to the loader, as when loader_return_after expires.
 * Only a driver's fingers count, not samples a program writes to the device.
 */
#define EXIT_FINGERS    5
#define EXIT_HOLD_MS    1500

static thread_call_t exit_call;
static uint32_t fingers_down;           /* one bit for each finger a driver says is down */

static void
exit_hold_elapsed(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	printf("\n--- %d fingers held on the screen for %d ms: back to the loader ---\n", EXIT_FINGERS, EXIT_HOLD_MS);
	hakios_adreno_stop();
	loader_return();
}

/* With touch_lock held. */
static void
count_fingers(uint32_t type, uint32_t finger)
{
	int before = __builtin_popcount(fingers_down), now;
	uint64_t deadline;

	if (finger >= 32 || exit_call == NULL || (type != HAKIOS_TOUCH_DOWN && type != HAKIOS_TOUCH_UP)) {
		return;
	}
	if (type == HAKIOS_TOUCH_DOWN) {
		fingers_down |= 1u << finger;
	} else {
		fingers_down &= ~(1u << finger);
	}
	now = __builtin_popcount(fingers_down);
	if (now >= EXIT_FINGERS && before < EXIT_FINGERS) {
		clock_interval_to_deadline(EXIT_HOLD_MS, NSEC_PER_MSEC, &deadline);
		thread_call_enter_delayed(exit_call, deadline);
	} else if (now < EXIT_FINGERS && before >= EXIT_FINGERS) {
		thread_call_cancel(exit_call);
	}
}

/* With touch_lock held. `when`: the sample's time (mach_absolute_time), 0 for now. */
static void
touch_post_locked(uint32_t type, uint32_t finger, uint32_t x, uint32_t y, bool from_driver, uint64_t when)
{
	struct hakios_touch *t;

	if (touch_tail - touch_head == TOUCH_QUEUE) {
		touch_head++;                   /* nobody reads: drop the oldest */
	}
	t = &touch_queue[touch_tail++ % TOUCH_QUEUE];
	t->time = when != 0 ? when : mach_absolute_time();
	if (latency_lines()) {
		uint64_t ns;

		absolutetime_to_nanoseconds(t->time, &ns);
		hakios_tl("tl: p %u %u %u %u at %llu\n", type, finger, x, y, ns / 1000);
	}
	t->type = type;
	t->finger = finger;
	t->x = x > 65535 ? 65535 : x;
	t->y = y > 65535 ? 65535 : y;
	if (from_driver) {
		count_fingers(type, finger);
	}
}

static void
touch_post(uint32_t type, uint32_t finger, uint32_t x, uint32_t y, bool from_driver)
{
	lck_mtx_lock(&touch_lock);
	touch_post_locked(type, finger, x, y, from_driver, 0);
	lck_mtx_unlock(&touch_lock);
	wakeup(&touch_queue);
}

/*
 * Samples written with a time (struct hakios_touch.time, mach_absolute_time; 0: at once) are posted
 * by the kernel when that time comes, with that time as theirs: a gesture a program writes in one go
 * (uibench) has its samples exactly as far apart as it says, however late the writer or this thread
 * call runs -- the window server takes a touch's speed from the samples' times. They wait here, in
 * the order written; a time earlier than the one before it is taken as that one.
 */
#define TOUCH_LATER     256             /* records; a power of two */

static struct hakios_touch touch_later[TOUCH_LATER];
static uint32_t later_head, later_tail;
static thread_call_t later_call;

/* With touch_lock held: the thread call set for the first sample that waits, if any. */
static void
later_arm(void)
{
	if (later_head != later_tail) {
		thread_call_enter_delayed_with_leeway(later_call, NULL, touch_later[later_head % TOUCH_LATER].time, 0,
		    THREAD_CALL_DELAY_LEEWAY);
	}
}

static void
later_due(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint64_t now = mach_absolute_time();
	bool posted = false;

	lck_mtx_lock(&touch_lock);
	while (later_head != later_tail && touch_later[later_head % TOUCH_LATER].time <= now) {
		struct hakios_touch *t = &touch_later[later_head++ % TOUCH_LATER];

		touch_post_locked(t->type, t->finger, t->x, t->y, false, t->time);
		posted = true;
	}
	later_arm();
	lck_mtx_unlock(&touch_lock);
	if (posted) {
		wakeup(&touch_queue);
	}
}

/* sysctl kern.hakios.touch_later: how many timed samples still wait (a writer waits for 0 before it looks) */
SYSCTL_DECL(_kern_hakios);

static int
sysctl_touch_later SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int waiting;

	lck_mtx_lock(&touch_lock);
	waiting = (int)(later_tail - later_head);
	lck_mtx_unlock(&touch_lock);
	return SYSCTL_OUT(req, &waiting, sizeof(waiting));
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, touch_later, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0, sysctl_touch_later,
    "I", "hakios-touch: timed samples written that wait for their time");

/* when a finger last touched the screen, in absolute time */
uint64_t hakios_touch_last;

void
hakios_touch_post(uint32_t type, uint32_t finger, uint32_t x, uint32_t y)
{
	hakios_touch_last = mach_absolute_time();
	touch_post(type, finger, x, y, true);
}

/* the program reading the queue (its process ID), 0: none yet; with touch_lock held */
static int touch_reader;

static bool
touch_root(void)
{
	return suser(kauth_cred_get(), NULL) == 0;
}

static int
touch_open(__unused dev_t dev, int flags, __unused int devtype, __unused struct proc *p)
{
	int me = proc_selfpid(), held = 0, error = 0;

	if ((flags & FWRITE) != 0 && !touch_root()) {
		return EPERM;
	}
	if ((flags & FREAD) == 0 || touch_root()) {
		return 0;
	}
	lck_mtx_lock(&touch_lock);
	if (touch_reader != 0 && touch_reader != me) {
		proc_t reader = proc_find(touch_reader);

		if (reader != NULL) {
			proc_rele(reader);
			held = touch_reader;
			error = EBUSY;
		}
	}
	if (error == 0) {
		touch_reader = me;      /* (the one before gone: its process ended without the node's last close) */
	}
	lck_mtx_unlock(&touch_lock);
	if (error != 0) {
		printf("hakios-touch: pid %d may not read the touches: pid %d does\n", me, held);
	}
	return error;
}

/* (the node's last close: nobody has it open any more) */
static int
touch_close(__unused dev_t dev, __unused int flags, __unused int devtype, __unused struct proc *p)
{
	lck_mtx_lock(&touch_lock);
	touch_reader = 0;
	lck_mtx_unlock(&touch_lock);
	return 0;
}

static int
touch_read(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	struct hakios_touch t;
	int error = 0, moved = 0;

	if (uio_resid(uio) < (user_ssize_t)sizeof(t)) {
		/* (room for no record: refused at once, not slept on until a touch and then answered as the end of the file) */
		return EINVAL;
	}
	lck_mtx_lock(&touch_lock);
	while (touch_head == touch_tail) {
		error = msleep(&touch_queue, &touch_lock, PZERO | PCATCH, "hakios-touch", NULL);
		if (error) {
			lck_mtx_unlock(&touch_lock);
			return error;
		}
	}
	while (touch_head != touch_tail && uio_resid(uio) >= (user_ssize_t)sizeof(t)) {
		t = touch_queue[touch_head++ % TOUCH_QUEUE];
		lck_mtx_unlock(&touch_lock);
		error = uiomove((caddr_t)&t, sizeof(t), uio);
		lck_mtx_lock(&touch_lock);
		if (error) {
			break;
		}
		moved++;
	}
	lck_mtx_unlock(&touch_lock);
	return moved ? 0 : error;
}

static int
touch_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	struct hakios_touch t;
	int error = 0;

	if (!touch_root()) {
		return EPERM;           /* (also for a descriptor root opened and handed on) */
	}
	while (uio_resid(uio) >= (user_ssize_t)sizeof(t)) {
		error = uiomove((caddr_t)&t, sizeof(t), uio);
		if (error) {
			break;
		}
		if (t.type < HAKIOS_TOUCH_DOWN || t.type > HAKIOS_BUTTON_UP) {
			return EINVAL;
		}
		if (t.time == 0) {
			touch_post(t.type, t.finger, t.x, t.y, false);
			continue;
		}
		lck_mtx_lock(&touch_lock);
		if (later_tail - later_head == TOUCH_LATER) {
			lck_mtx_unlock(&touch_lock);
			return ENOSPC;          /* (a longer gesture is written in parts, each once the one before is posted) */
		}
		if (later_head != later_tail && t.time < touch_later[(later_tail - 1) % TOUCH_LATER].time) {
			t.time = touch_later[(later_tail - 1) % TOUCH_LATER].time;
		}
		touch_later[later_tail++ % TOUCH_LATER] = t;
		if (later_tail - later_head == 1) {
			later_arm();
		}
		lck_mtx_unlock(&touch_lock);
	}
	return error;
}

static const struct cdevsw touch_cdevsw = {
	.d_open = touch_open, .d_close = touch_close, .d_read = touch_read, .d_write = touch_write,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

void
hakios_input_init(void)
{
	int major = cdevsw_add(-1, &touch_cdevsw);

	if (major < 0 || devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "hakios-touch") == NULL) {
		printf("hakios-touch: cannot create the device\n");
	}
	exit_call = thread_call_allocate(exit_hold_elapsed, NULL);
	later_call = thread_call_allocate_with_priority(later_due, NULL, THREAD_CALL_PRIORITY_KERNEL_HIGH);
}
