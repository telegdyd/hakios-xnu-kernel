/*
 * uibench: drive the screen through a list of steps, checking each one landed, and measure it.
 *
 *   usage: uibench run STEPS [quiet MS] [settle MS] [idle S] [prof]    the steps of the file STEPS, in order
 *                                                   (first waiting at most idle S, 30, for the system to be quiet;
 *                                                   prof: backboardd's CPU profile of the steps after the last line)
 *          uibench state                                what the screen is now (SpringBoard's answer)
 *          uibench do ACTION...                         one action (as in a step), then the state
 *
 * A step is a line "NAME | BEFORE | ACTION | AFTER" ('#' starts a comment):
 *   BEFORE, AFTER    conditions, all of which must hold: locked=0|1, cc=0|1, switcher=0|1, front=ID|- (the
 *                    application in front, - the home screen), page=N|+N|-N (the home screen's page, or so many
 *                    from where it was before the action), icon=ID (that icon on the screen). Empty: none.
 *                    BEFORE is brought about if it does not hold (unlock, close Control Center, go home, turn the
 *                    page); a step whose BEFORE cannot be had is not run and fails. A condition written ?KEY=VALUE
 *                    is only looked at: when it does not hold the step is skipped (SKIP, counted as passed).
 *   ACTION           swipe X0 Y0 X1 Y1 MS | tap X Y | tap icon:ID | button home|lock | wait MS | none
 *                    (positions in fractions of the screen, 0 0 top left, as rwtouch takes them)
 * After the action uibench waits for the screen to be still (no frame for `quiet` ms, 120; at most `settle` ms,
 * 3000), asks SpringBoard what it shows (com.hakios.uistate, inject_sb.c) and checks AFTER, asking again for a
 * second while it does not hold. Each step's line:
 *   STEP NAME PASS|FAIL [why] | still after MS | screen moved D | frames N, X% within 18 ms, longest MS, 95% under MS |
 *       NAME MS ...
 * how far the picture on the panel moved (a step whose state changed fails below SCREEN_MIN: the screen did not
 * follow), the frames from the start of the action until the screen was still (kern.hakios.fb_step), and the
 * processes that took the most CPU time meanwhile. A display that a tap does not wake fails the step. The last
 * line: "UIBENCH N steps, P passed, F failed, S s".
 *
 * Gestures are written to /dev/hakios-touch in one go with each sample's time: the kernel posts them then
 * (board/input.c), with those times, so a swipe is the same swipe however busy the system is.
 */
#include <fcntl.h>
#include <mach/mach_time.h>
#include <notify.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/qos.h>
#include <sys/sysctl.h>
#include <unistd.h>

#include "board/input.h"

/* (libproc, in libSystem; the SDK has neither its header nor sys/proc_info.h) */
#define PROC_PIDTASKINFO        4
struct proc_taskinfo {
	uint64_t pti_virtual_size, pti_resident_size;
	uint64_t pti_total_user, pti_total_system;      /* mach_absolute_time units */
	uint64_t pti_threads_user, pti_threads_system;
	int32_t pti_policy, pti_faults, pti_pageins, pti_cow_faults, pti_messages_sent, pti_messages_received;
	int32_t pti_syscalls_mach, pti_syscalls_unix, pti_csw, pti_threadnum, pti_numrunning, pti_priority;
};
extern int proc_listallpids(void *buffer, int size);
extern int proc_pidinfo(int pid, int flavor, uint64_t arg, void *buffer, int size);
extern int proc_name(int pid, void *buffer, uint32_t size);

#define STATE_FILE      "/private/var/tmp/hakios-uistate"
#define SAMPLE_MS       8               /* a touch sample every so many milliseconds, as the touch screen gives them */
#define ICONS           96
#define CONDITIONS      8
#define PROCS           1024

static int device = -1;
static mach_timebase_info_data_t timebase;
static unsigned quiet_ms = 120, settle_ms = 3000;

static uint64_t
ms_to_abs(double ms)
{
	return (uint64_t)(ms * 1e6 * timebase.denom / timebase.numer);
}

static double
abs_to_ms(uint64_t t)
{
	return (double)t * timebase.numer / timebase.denom / 1e6;
}

static double
now_ms(void)
{
	return abs_to_ms(mach_absolute_time());
}

static void
sleep_ms(double ms)
{
	if (ms > 0) {
		usleep((useconds_t)(ms * 1000));
	}
}

static int
sysctl_int(const char *name)
{
	int value = -1;
	size_t size = sizeof(value);

	return sysctlbyname(name, &value, &size, NULL, 0) == 0 ? value : -1;
}

/* --- the screen as SpringBoard says it is ---------------------------------------------------------- */

struct icon {
	char id[96];
	double x, y;
};

struct state {
	int locked, coversheet, cc, switcher, page, pages;     /* -1: not known */
	char front[128];
	struct icon icons[ICONS];
	int icon_count;
	char missing[256];
	bool ok;
};

static int done_token = -1;

static bool
ask_state(struct state *s)
{
	char line[256];
	FILE *f;
	int changed = 0;
	double until;

	memset(s, 0, sizeof(*s));
	s->locked = s->coversheet = s->cc = s->switcher = s->page = s->pages = -1;
	strcpy(s->front, "?");
	if (done_token < 0 && notify_register_check("com.hakios.uistate.done", &done_token) != NOTIFY_STATUS_OK) {
		return false;
	}
	notify_check(done_token, &changed);     /* (what was posted before is not this answer) */
	notify_post("com.hakios.uistate");
	for (until = now_ms() + 2000, changed = 0; !changed && now_ms() < until; sleep_ms(2)) {
		notify_check(done_token, &changed);
	}
	if (!changed || (f = fopen(STATE_FILE, "r")) == NULL) {
		return false;
	}
	while (fgets(line, sizeof(line), f) != NULL) {
		struct icon *icon = &s->icons[s->icon_count];
		char word[128];

		line[strcspn(line, "\n")] = 0;
		if (sscanf(line, "locked %d", &s->locked) == 1 || sscanf(line, "coversheet %d", &s->coversheet) == 1 ||
		    sscanf(line, "cc %d", &s->cc) == 1 || sscanf(line, "switcher %d", &s->switcher) == 1 ||
		    sscanf(line, "page %d of %d", &s->page, &s->pages) == 2) {
			continue;
		}
		if (sscanf(line, "front %127s", word) == 1) {
			strcpy(s->front, word);
		} else if (s->icon_count < ICONS && sscanf(line, "icon %95s %lf %lf", icon->id, &icon->x, &icon->y) == 3) {
			s->icon_count++;
		} else if (strncmp(line, "missing ", 8) == 0 && strlen(s->missing) + strlen(line) + 2 < sizeof(s->missing)) {
			strcat(s->missing, line + 8);
			strcat(s->missing, "; ");
		} else if (strcmp(line, "end") == 0) {
			s->ok = true;
		}
	}
	fclose(f);
	return s->ok;
}

static const struct icon *
find_icon(const struct state *s, const char *id)
{
	for (int i = 0; i < s->icon_count; i++) {
		if (strcmp(s->icons[i].id, id) == 0) {
			return &s->icons[i];
		}
	}
	return NULL;
}

static void
print_state(const char *prefix, const struct state *s)
{
	printf("%slocked %d coversheet %d cc %d switcher %d front %s page %d of %d, %d icons%s%s\n", prefix, s->locked,
	    s->coversheet, s->cc, s->switcher, s->front, s->page, s->pages, s->icon_count, s->missing[0] ? "; missing: " : "",
	    s->missing);
	if (s->icon_count > 0) {
		printf("%sicons:", prefix);
		for (int i = 0; i < s->icon_count; i++) {
			printf(" %s", s->icons[i].id);
		}
		printf("\n");
	}
}

/* --- the screen being still ---------------------------------------------------------------------- */

static uint64_t
frame_last(void)
{
	uint64_t last = 0;
	size_t size = sizeof(last);

	sysctlbyname("kern.hakios.fb_frame_last", &last, &size, NULL, 0);
	return last;
}

/*
 * Until no frame for quiet_ms (true), or settle_ms since `from` (false); the time it was still at in *still.
 * need_after (0: none): still only once a frame has come after that time, the end of the action -- what a
 * gesture moves goes on after the finger is up, and a window server that is behind shows those frames late:
 * the quiet before them is not the screen at rest.
 */
static bool
wait_still_after(double from, double *still, double need_after)
{
	for (;;) {
		double now = now_ms(), last = abs_to_ms(frame_last()), since = now - (last > from ? last : from);

		if (since >= quiet_ms && (need_after == 0 || last > need_after)) {
			*still = (last > from ? last : from);
			return true;
		}
		if (now - from >= settle_ms) {
			*still = now;
			return false;
		}
		sleep_ms(10);
	}
}

static bool
wait_still(double from, double *still)
{
	return wait_still_after(from, still, 0);
}

static void
frame_step(char *text, size_t size)
{
	text[0] = 0;
	if (sysctlbyname("kern.hakios.fb_step", text, &size, NULL, 0) != 0) {
		text[0] = 0;
	}
}

/* --- touches --------------------------------------------------------------------------------------- */

static void
put_samples(const struct hakios_touch *t, int n)
{
	if (write(device, t, sizeof(*t) * (size_t)n) != (ssize_t)(sizeof(*t) * (size_t)n)) {
		perror("uibench: write");
	}
}

static struct hakios_touch
sample(uint64_t at, uint32_t type, double x, double y)
{
	struct hakios_touch t = { .time = at, .type = type, .finger = 0 };

	t.x = (uint32_t)((x < 0 ? 0 : x > 1 ? 1 : x) * 65535.0 + 0.5);
	t.y = (uint32_t)((y < 0 ? 0 : y > 1 ? 1 : y) * 65535.0 + 0.5);
	return t;
}

/* a finger from one point to the other in ms, a sample every SAMPLE_MS, each with its time; back once it is up */
static void
swipe(double x0, double y0, double x1, double y1, double ms)
{
	struct hakios_touch t[200];
	uint64_t start = mach_absolute_time() + ms_to_abs(4);
	int n = 0;

	if (ms < 1) {
		ms = 1;
	}
	t[n++] = sample(start, HAKIOS_TOUCH_DOWN, x0, y0);
	for (double at = SAMPLE_MS;; at += SAMPLE_MS) {
		double part;

		if (at > ms) {
			at = ms;
		}
		part = at / ms;
		t[n++] = sample(start + ms_to_abs(at), HAKIOS_TOUCH_MOVE, x0 + (x1 - x0) * part, y0 + (y1 - y0) * part);
		if (at >= ms || n >= 190) {
			break;
		}
	}
	t[n++] = sample(start + ms_to_abs(ms + 16), HAKIOS_TOUCH_UP, x1, y1);
	put_samples(t, n);
	sleep_ms(ms + 24);
	for (int i = 0; i < 100 && sysctl_int("kern.hakios.touch_later") > 0; i++) {
		sleep_ms(2);
	}
}

static void
tap(double x, double y)
{
	struct hakios_touch t[2];
	uint64_t start = mach_absolute_time() + ms_to_abs(4);

	t[0] = sample(start, HAKIOS_TOUCH_DOWN, x, y);
	t[1] = sample(start + ms_to_abs(60), HAKIOS_TOUCH_UP, x, y);
	put_samples(t, 2);
	sleep_ms(70);
}

static void
button(uint32_t which)
{
	struct hakios_touch t = { .type = HAKIOS_BUTTON_DOWN, .finger = which };

	put_samples(&t, 1);
	sleep_ms(80);
	t.type = HAKIOS_BUTTON_UP;
	put_samples(&t, 1);
}

static bool
display_off(void)
{
	char text[512];
	size_t size = sizeof(text) - 1;

	if (sysctlbyname("kern.hakios.display_power_stats", text, &size, NULL, 0) != 0) {
		return false;
	}
	text[size] = 0;
	return strstr(text, "off now 1") != NULL;
}

/*
 * The display on: a tap if it is off (backboardd's library turns it into a press of the lock button while the window
 * server has the touch screen waking), else the lock button itself, as the owner would; false when it stays off.
 */
static bool
display_wake(void)
{
	double still;

	for (int how = 0; how < 2 && display_off(); how++) {
		if (how == 0) {
			tap(0.5, 0.5);
		} else {
			button(HAKIOS_BUTTON_LOCK);
		}
		wait_still_after(now_ms(), &still, now_ms());
		for (double until = now_ms() + 1000; display_off() && now_ms() < until; sleep_ms(50)) {
		}
	}
	return !display_off();
}

/*
 * --- what the panel shows, coarsely -----------------------------------------------------------------
 * The image the display is given (kern.hakios.screen, as screencap reads it) as a grid of mean brightness, 40 by 40
 * pixels a cell. A step whose state changed (unlocked, another page, Control Center, another application in front)
 * must also change the picture: on 2026-11-27 backboardd went on showing a lock screen while SpringBoard unlocked and
 * turned pages behind it, and every step still passed. How far the grid moved (the mean of the cells' differences,
 * 0 to 255) is on each step's line.
 */
#define GRID_CELL       40
#define GRID_W          27
#define GRID_H          61
#define SCREEN_MIN      2.0             /* a step whose state changed moves the grid at least this much */

struct grid {
	bool ok;
	float cell[GRID_H][GRID_W];
};

static uint8_t *screen_buf;
static size_t screen_size;

static void
screen_grid(struct grid *g)
{
	struct { char magic[8]; uint32_t width, height, rowBytes, format; } h;
	size_t size = 0;

	g->ok = false;
	if (sysctlbyname("kern.hakios.screen", NULL, &size, NULL, 0) != 0 || size <= sizeof(h)) {
		return;
	}
	if (size > screen_size) {
		free(screen_buf);
		screen_buf = malloc(size);
		screen_size = screen_buf != NULL ? size : 0;
		if (screen_buf == NULL) {
			return;
		}
	}
	if (sysctlbyname("kern.hakios.screen", screen_buf, &size, NULL, 0) != 0 || size <= sizeof(h)) {
		return;
	}
	memcpy(&h, screen_buf, sizeof(h));
	if (memcmp(h.magic, "HKSCREEN", 8) != 0 || h.format != 0 || h.width == 0 || h.height == 0 ||
	    size < sizeof(h) + (size_t)h.rowBytes * h.height) {
		return;
	}
	for (unsigned gy = 0; gy < GRID_H; gy++) {
		for (unsigned gx = 0; gx < GRID_W; gx++) {
			unsigned sum = 0, n = 0;

			/* (every fourth pixel each way: BGRx, the brightness of each) */
			for (unsigned y = gy * GRID_CELL; y < (gy + 1) * GRID_CELL && y < h.height; y += 4) {
				const uint8_t *row = screen_buf + sizeof(h) + (size_t)y * h.rowBytes;

				for (unsigned x = gx * GRID_CELL; x < (gx + 1) * GRID_CELL && x < h.width; x += 4) {
					sum += (row[x * 4 + 2] * 77u + row[x * 4 + 1] * 150u + row[x * 4] * 29u) >> 8;
					n++;
				}
			}
			g->cell[gy][gx] = n != 0 ? (float)sum / (float)n : 0;
		}
	}
	g->ok = true;
}

/* how far the picture moved from a to b: the mean difference of the cells; -1 when either was not read */
static double
grid_moved(const struct grid *a, const struct grid *b)
{
	double sum = 0;

	if (!a->ok || !b->ok) {
		return -1;
	}
	for (unsigned gy = 0; gy < GRID_H; gy++) {
		for (unsigned gx = 0; gx < GRID_W; gx++) {
			float d = a->cell[gy][gx] - b->cell[gy][gx];

			sum += d < 0 ? -d : d;
		}
	}
	return sum / (GRID_W * GRID_H);
}

/* what SpringBoard shows differently in b than in a, for the screen check ("" when nothing) */
static const char *
state_changed(const struct state *a, const struct state *b)
{
	if (a->locked != b->locked) return "locked";
	if (a->coversheet != b->coversheet) return "coversheet";
	if (a->cc != b->cc) return "cc";
	if (a->switcher != b->switcher) return "switcher";
	if (strcmp(a->front, b->front) != 0) return "front";
	if (a->page != b->page) return "page";
	return "";
}

/* --- actions and conditions ------------------------------------------------------------------------ */

/* an action's words; false (and why in `why`) when it cannot be done */
static bool
act(char **w, int n, const struct state *before, char *why, size_t why_size)
{
	if (n == 0 || strcmp(w[0], "none") == 0) {
		return true;
	}
	if (strcmp(w[0], "swipe") == 0 && n >= 6) {
		swipe(atof(w[1]), atof(w[2]), atof(w[3]), atof(w[4]), atof(w[5]));
	} else if (strcmp(w[0], "tap") == 0 && n >= 2 && strncmp(w[1], "icon:", 5) == 0) {
		const struct icon *icon = before != NULL ? find_icon(before, w[1] + 5) : NULL;

		if (icon == NULL) {
			snprintf(why, why_size, "no icon %s on the screen", w[1] + 5);
			return false;
		}
		tap(icon->x, icon->y);
	} else if (strcmp(w[0], "tap") == 0 && n >= 3) {
		tap(atof(w[1]), atof(w[2]));
	} else if (strcmp(w[0], "button") == 0 && n >= 2) {
		button(strcmp(w[1], "lock") == 0 ? HAKIOS_BUTTON_LOCK : HAKIOS_BUTTON_HOME);
	} else if (strcmp(w[0], "wait") == 0 && n >= 2) {
		sleep_ms(atof(w[1]));
	} else {
		snprintf(why, why_size, "the action \"%s\" is not known (or lacks words)", w[0]);
		return false;
	}
	return true;
}

struct condition {
	char key[16], value[128];
	bool only_if;           /* ?KEY=VALUE: the step is skipped when it does not hold */
};

static int
parse_conditions(char *text, struct condition *c)
{
	int n = 0;

	for (char *word = strtok(text, " \t"); word != NULL && n < CONDITIONS; word = strtok(NULL, " \t")) {
		char *eq = strchr(word, '=');

		if (eq == NULL) {
			continue;
		}
		*eq = 0;
		c[n].only_if = word[0] == '?';
		snprintf(c[n].key, sizeof(c[n].key), "%s", word + c[n].only_if);
		snprintf(c[n].value, sizeof(c[n].value), "%s", eq + 1);
		n++;
	}
	return n;
}

/* whether c holds in s (page relative to `page_before`); if not, what is there in `is` */
static bool
holds(const struct condition *c, const struct state *s, int page_before, char *is, size_t is_size)
{
	int want;

	if (strcmp(c->key, "front") == 0) {
		snprintf(is, is_size, "front is %s", s->front);
		return strcmp(s->front, c->value) == 0;
	}
	if (strcmp(c->key, "icon") == 0) {
		snprintf(is, is_size, "no icon %s on the screen", c->value);
		return find_icon(s, c->value) != NULL;
	}
	if (strcmp(c->key, "page") == 0) {
		want = c->value[0] == '+' || c->value[0] == '-' ? page_before + atoi(c->value) : atoi(c->value);
		snprintf(is, is_size, "page is %d, wanted %d", s->page, want);
		return s->page >= 0 && s->page == want;
	}
	want = atoi(c->value);
	for (const char *const *k = (const char *const[]){ "locked", "cc", "switcher", "coversheet", NULL }; *k; k++) {
		if (strcmp(c->key, *k) == 0) {
			int v = strcmp(*k, "locked") == 0 ? s->locked : strcmp(*k, "cc") == 0 ? s->cc :
			    strcmp(*k, "switcher") == 0 ? s->switcher : s->coversheet;

			snprintf(is, is_size, "%s is %d, wanted %d", *k, v, want);
			return v == want;
		}
	}
	snprintf(is, is_size, "the condition %s is not known", c->key);
	return false;
}

static bool
all_hold(const struct condition *c, int n, const struct state *s, int page_before, char *is, size_t is_size)
{
	for (int i = 0; i < n; i++) {
		if (!holds(&c[i], s, page_before, is, is_size)) {
			return false;
		}
	}
	return true;
}

static bool icon_search_back;

/* the way to a condition that does not hold: one try, false when there is none */
static bool
fix(const struct condition *c, const struct state *s)
{
	if (strcmp(c->key, "locked") == 0 && atoi(c->value) == 0) {
		swipe(0.5, 0.99, 0.5, 0.4, 250);
	} else if (strcmp(c->key, "cc") == 0 && atoi(c->value) == 0) {
		swipe(0.5, 0.99, 0.5, 0.3, 250);
	} else if (strcmp(c->key, "cc") == 0 && atoi(c->value) == 1) {
		swipe(0.92, 0.003, 0.92, 0.55, 300);
	} else if (strcmp(c->key, "front") == 0 && strcmp(c->value, "-") == 0) {
		button(HAKIOS_BUTTON_HOME);
	} else if (strcmp(c->key, "front") == 0 && find_icon(s, c->value) != NULL) {
		const struct icon *icon = find_icon(s, c->value);

		tap(icon->x, icon->y);
	} else if (strcmp(c->key, "icon") == 0 && s->page >= 0 && s->pages > 1) {
		/* (through the pages to the last, then back to the first, until it is on the screen) */
		if (s->page >= s->pages - 1) {
			icon_search_back = true;
		} else if (s->page <= 0) {
			icon_search_back = false;
		}
		if (icon_search_back) {
			swipe(0.15, 0.5, 0.85, 0.5, 200);
		} else {
			swipe(0.85, 0.5, 0.15, 0.5, 200);
		}
	} else if (strcmp(c->key, "page") == 0 && c->value[0] != '+' && c->value[0] != '-' && s->page >= 0) {
		if (atoi(c->value) > s->page) {
			swipe(0.85, 0.5, 0.15, 0.5, 200);
		} else {
			swipe(0.15, 0.5, 0.85, 0.5, 200);
		}
	} else {
		return false;
	}
	return true;
}

/* BEFORE brought about: false (what is there in `is`) when it cannot be */
static bool
reach(const struct condition *c, int n, struct state *s, char *is, size_t is_size)
{
	double still;

	icon_search_back = false;
	for (int tries = 0; tries < 14; tries++) {
		int i;

		if (!display_wake()) {
			snprintf(is, is_size, "the display stays off after a tap and the lock button");
			return false;
		}
		if (!ask_state(s)) {
			snprintf(is, is_size, "SpringBoard did not answer");
			return false;
		}
		/* (what is in the way first: the lock screen, Control Center, an application) */
		if (s->locked == 1 && tries >= 3) {
			notify_post("com.hakios.unlock");
			wait_still_after(now_ms(), &still, now_ms());
			continue;
		}
		for (i = 0; i < n && holds(&c[i], s, s->page, is, is_size); i++) {
			;
		}
		if (i == n) {
			return true;
		}
		if (s->locked == 1 && strcmp(c[i].key, "locked") != 0) {
			swipe(0.5, 0.99, 0.5, 0.4, 250);
		} else if (s->cc == 1 && !(strcmp(c[i].key, "cc") == 0 && atoi(c[i].value) == 1)) {
			swipe(0.5, 0.99, 0.5, 0.3, 250);
		} else if (strcmp(s->front, "-") != 0 && strcmp(c[i].key, "front") != 0 && strcmp(c[i].key, "cc") != 0) {
			button(HAKIOS_BUTTON_HOME);
		} else if (!fix(&c[i], s)) {
			return false;
		}
		/* (its frames first: a step taken again before the screen has followed the last would go too far) */
		wait_still_after(now_ms() - 400, &still, now_ms() - 10);
	}
	ask_state(s);
	return all_hold(c, n, s, s->page, is, is_size);
}

/* --- CPU time by process --------------------------------------------------------------------------- */

struct cpu {
	int pid;
	uint64_t total;
	char name[32];
};

static struct cpu cpu_before[PROCS], cpu_after[PROCS];
static int cpu_before_n, cpu_after_n;

static int
cpu_look(struct cpu *list)
{
	static int pids[PROCS];
	int n = proc_listallpids(pids, sizeof(pids)), count = 0;

	for (int i = 0; i < n && count < PROCS; i++) {
		struct proc_taskinfo ti;

		if (pids[i] <= 0 || proc_pidinfo(pids[i], PROC_PIDTASKINFO, 0, &ti, sizeof(ti)) != (int)sizeof(ti)) {
			continue;
		}
		list[count].pid = pids[i];
		list[count].total = ti.pti_total_user + ti.pti_total_system;
		if (proc_name(pids[i], list[count].name, sizeof(list[count].name)) <= 0) {
			snprintf(list[count].name, sizeof(list[count].name), "%d", pids[i]);
		}
		count++;
	}
	return count;
}

/* the processes that took the most CPU time between the two looks, into text */
static void
cpu_top(char *text, size_t size)
{
	struct { int i; uint64_t took; } top[4] = { { -1, 0 }, { -1, 0 }, { -1, 0 }, { -1, 0 } };
	size_t used = 0;

	for (int i = 0; i < cpu_after_n; i++) {
		uint64_t took = cpu_after[i].total;

		for (int j = 0; j < cpu_before_n; j++) {
			if (cpu_before[j].pid == cpu_after[i].pid) {
				took = cpu_after[i].total >= cpu_before[j].total ? cpu_after[i].total - cpu_before[j].total : 0;
				break;
			}
		}
		for (int k = 0; k < 4; k++) {
			if (took > top[k].took) {
				memmove(&top[k + 1], &top[k], sizeof(top[0]) * (size_t)(3 - k));
				top[k].i = i;
				top[k].took = took;
				break;
			}
		}
	}
	text[0] = 0;
	for (int k = 0; k < 4 && top[k].i >= 0 && used < size; k++) {
		used += (size_t)snprintf(text + used, size - used, " %s %.0f", cpu_after[top[k].i].name, abs_to_ms(top[k].took));
	}
}

/* all the processes' CPU time since the last look, in milliseconds (cpu_before: the last look) */
static double
cpu_all_since(void)
{
	uint64_t took = 0;

	cpu_after_n = cpu_look(cpu_after);
	for (int i = 0; i < cpu_after_n; i++) {
		for (int j = 0; j < cpu_before_n; j++) {
			if (cpu_before[j].pid == cpu_after[i].pid) {
				took += cpu_after[i].total >= cpu_before[j].total ? cpu_after[i].total - cpu_before[j].total : 0;
				break;
			}
		}
	}
	memcpy(cpu_before, cpu_after, sizeof(cpu_before[0]) * (size_t)cpu_after_n);
	cpu_before_n = cpu_after_n;
	return abs_to_ms(took);
}

/*
 * Before the first step: the system done with what it does after a boot (icons drawn, widgets, caches), at most
 * idle_s seconds -- what is measured meanwhile is that, not the gestures. Quiet: all processes together under
 * IDLE_CORES of a core for two half seconds in a row.
 */
#define IDLE_CORES      0.35
static unsigned idle_s = 30;

static void
wait_idle(void)
{
	double began = now_ms(), last = began, load = 0;
	int calm = 0;

	if (idle_s == 0) {
		return;
	}
	cpu_all_since();
	while (now_ms() - began < idle_s * 1000.0) {
		double now;

		sleep_ms(500);
		now = now_ms();
		load = cpu_all_since() / (now - last);
		last = now;
		calm = load < IDLE_CORES ? calm + 1 : 0;
		if (calm >= 2) {
			printf("UIBENCH quiet after %.1f s (%.0f%% of a core)\n", (now - began) / 1000, load * 100);
			return;
		}
	}
	{
		char top[192];

		/* (who keeps it busy: CPU milliseconds of the last second, cpu_before being the last look) */
		sleep_ms(1000);
		cpu_after_n = cpu_look(cpu_after);
		cpu_top(top, sizeof(top));
		printf("UIBENCH still busy after %u s (%.0f%% of a core), the last second:%s; measuring all the same\n", idle_s,
		    load * 100, top);
	}
}

/* --- the steps -------------------------------------------------------------------------------------- */

static char *
trim(char *s)
{
	char *end;

	while (*s == ' ' || *s == '\t') {
		s++;
	}
	end = s + strlen(s);
	while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) {
		*--end = 0;
	}
	return s;
}

/*
 * --- the window server's CPU profile (run ... prof) ---------------------------------------------------
 * backboardd's library (inject.c, cp_sampler) samples its running threads from the first step to the last when the
 * notification com.hakios.prof has the state 1, and when it goes back to 0 writes PROF_FILE and posts
 * com.hakios.prof.done; its lines (each beginning "PROF") follow the last step's.
 */
#define PROF_FILE       "/private/var/tmp/hakios-prof.txt"
static bool prof_on;

static void
prof_set(uint64_t on)
{
	static int token = -1;

	if (token < 0 && notify_register_check("com.hakios.prof", &token) != NOTIFY_STATUS_OK) {
		return;
	}
	notify_set_state(token, on);
	notify_post("com.hakios.prof");
}

static void
prof_finish(void)
{
	int done = -1, posted = 0;
	char line[1024];
	FILE *f;

	if (notify_register_check("com.hakios.prof.done", &done) == NOTIFY_STATUS_OK) {
		notify_check(done, &posted);    /* (an old answer is not this one) */
	}
	posted = 0;
	prof_set(0);
	for (double until = now_ms() + 5000; !posted && now_ms() < until; sleep_ms(20)) {
		if (done >= 0) {
			notify_check(done, &posted);
		}
	}
	f = posted ? fopen(PROF_FILE, "r") : NULL;
	if (f == NULL) {
		printf("PROF none: backboardd wrote no profile (its library too old, or not in it)\n");
		return;
	}
	while (fgets(line, sizeof(line), f) != NULL) {
		fputs(line, stdout);
	}
	fclose(f);
}

/* "frames N under10 A under18 B ... longest L half H most M pauses P" as a short summary */
static void
frames_summary(const char *step, char *text, size_t size)
{
	unsigned frames = 0, u10 = 0, u18 = 0, u26 = 0, u35 = 0, longer = 0, longest = 0, half = 0, most = 0, pauses = 0;

	if (sscanf(step, "frames %u under10 %u under18 %u under26 %u under35 %u longer %u longest %u half %u most %u pauses %u",
	    &frames, &u10, &u18, &u26, &u35, &longer, &longest, &half, &most, &pauses) != 10 || frames == 0) {
		snprintf(text, size, "frames 0");
		return;
	}
	snprintf(text, size, "frames %u, %u%% within 18 ms, longest %u ms, 95%% under %u ms", frames,
	    (u10 + u18) * 100 / frames, longest, most);
}

static int
run(const char *path)
{
	FILE *f = fopen(path, "r");
	char line[512];
	int steps = 0, passed = 0;
	double began = now_ms();

	if (f == NULL) {
		perror(path);
		return 2;
	}
	wait_idle();
	if (prof_on) {
		unlink(PROF_FILE);
		prof_set(1);
	}
	began = now_ms();
	while (fgets(line, sizeof(line), f) != NULL) {
		char *hash = strchr(line, '#'), *field[4] = { NULL }, *w[12], why[256] = "", is[256] = "", frames[192], fsum[160],
		    cpu[192] = "", actcopy[256];
		struct condition pre[CONDITIONS], post[CONDITIONS];
		int fields = 0, npre, npost, nw = 0, page_before;
		struct state s, before;
		static struct grid grid_before, grid_after;
		char screen[64];
		const char *changed;
		double start, still, moved;
		bool ok = false, was_still;

		if (hash != NULL) {
			*hash = 0;
		}
		for (char *p = line, *bar; fields < 4; p = bar + 1) {
			bar = strchr(p, '|');
			if (bar != NULL) {
				*bar = 0;
			}
			field[fields++] = trim(p);
			if (bar == NULL) {
				break;
			}
		}
		if (fields < 3 || field[0][0] == 0) {
			continue;
		}
		steps++;
		npre = parse_conditions(field[1], pre);
		npost = fields > 3 ? parse_conditions(field[3], post) : 0;
		snprintf(actcopy, sizeof(actcopy), "%s", field[2]);
		for (char *word = strtok(actcopy, " \t"); word != NULL && nw < 12; word = strtok(NULL, " \t")) {
			w[nw++] = word;
		}
		{
			struct condition looks[CONDITIONS];
			int nlooks = 0, kept = 0;
			bool skip = false;

			for (int i = 0; i < npre; i++) {
				if (pre[i].only_if) {
					looks[nlooks++] = pre[i];
				} else {
					pre[kept++] = pre[i];
				}
			}
			npre = kept;
			if (nlooks > 0) {
				/* (a display that stays off fails the step below, in reach) */
				skip = display_wake() && ask_state(&s) && !all_hold(looks, nlooks, &s, s.page, is, sizeof(is));
			}
			if (skip) {
				printf("STEP %s SKIP (%s)\n", field[0], is);
				passed++;
				continue;
			}
		}
		memset(&s, 0, sizeof(s));
		s.locked = s.coversheet = s.cc = s.switcher = s.page = s.pages = -1;
		strcpy(s.front, "?");
		if (!reach(pre, npre, &s, is, sizeof(is))) {
			printf("STEP %s FAIL before: %s\n", field[0], is);
			print_state("  state: ", &s);
			fflush(stdout);
			if (display_off()) {
				/* (every step after would only wait for it again) */
				printf("UIBENCH stopped: the display cannot be woken\n");
				break;
			}
			continue;
		}
		page_before = s.page;
		before = s;
		screen_grid(&grid_before);              /* (before anything is timed: a read and its sums take some 50 ms) */
		frame_step(frames, sizeof(frames));     /* (counted afresh from here) */
		cpu_before_n = cpu_look(cpu_before);
		start = now_ms();
		if (!act(w, nw, &s, why, sizeof(why))) {
			printf("STEP %s FAIL %s\n", field[0], why);
			fflush(stdout);
			continue;
		}
		/* (a frame after the finger is up, for what moves the screen) */
		was_still = wait_still_after(start, &still,
		    nw > 0 && strcmp(w[0], "none") != 0 && strcmp(w[0], "wait") != 0 ? now_ms() - 10 : 0);
		frame_step(frames, sizeof(frames));
		cpu_after_n = cpu_look(cpu_after);
		cpu_top(cpu, sizeof(cpu));
		frames_summary(frames, fsum, sizeof(fsum));
		for (double until = now_ms() + 1000;;) {
			if (!ask_state(&s)) {
				snprintf(is, sizeof(is), "SpringBoard did not answer");
			} else if (all_hold(post, npost, &s, page_before, is, sizeof(is))) {
				ok = true;
				break;
			}
			if (now_ms() >= until) {
				break;
			}
			sleep_ms(100);
		}
		/* the picture followed what SpringBoard says changed (the screen check above) */
		screen_grid(&grid_after);
		moved = grid_moved(&grid_before, &grid_after);
		changed = state_changed(&before, &s);
		/* (an application's first picture may come after the screen was still: Settings opened 95 ms after the tap
		 * and drew nothing yet; a picture that has not moved in 1.5 s is a screen that does not follow) */
		for (double until = now_ms() + 1500; changed[0] != 0 && moved >= 0 && moved < SCREEN_MIN && now_ms() < until;) {
			sleep_ms(100);
			screen_grid(&grid_after);
			moved = grid_moved(&grid_before, &grid_after);
		}
		if (moved < 0) {
			snprintf(screen, sizeof(screen), "screen not read");
		} else {
			snprintf(screen, sizeof(screen), "screen moved %.1f", moved);
		}
		if (ok && moved >= 0 && changed[0] != 0 && moved < SCREEN_MIN) {
			ok = false;
			snprintf(is, sizeof(is), "the screen did not follow (%s changed, the picture moved %.1f)", changed, moved);
		}
		passed += ok;
		printf("STEP %s %s%s%s | %s %.0f ms | %s | %s |%s\n", field[0], ok ? "PASS" : "FAIL", ok ? "" : " ", ok ? "" : is,
		    was_still ? "still after" : "never still, gave up after", still - start, screen, fsum, cpu);
		if (!ok) {
			print_state("  state: ", &s);
		}
		fflush(stdout);
	}
	fclose(f);
	printf("UIBENCH %d steps, %d passed, %d failed, %.1f s\n", steps, passed, steps - passed, (now_ms() - began) / 1000);
	if (prof_on) {
		prof_finish();
	}
	return passed == steps ? 0 : 1;
}

int
main(int argc, char **argv)
{
	struct state s;

	pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
	mach_timebase_info(&timebase);
	setvbuf(stdout, NULL, _IOLBF, 0);
	if (argc >= 2 && strcmp(argv[1], "state") == 0) {
		if (!ask_state(&s)) {
			fprintf(stderr, "uibench: SpringBoard did not answer\n");
			return 1;
		}
		print_state("", &s);
		for (int i = 0; i < s.icon_count; i++) {
			printf("  icon %s %.3f %.3f\n", s.icons[i].id, s.icons[i].x, s.icons[i].y);
		}
		return 0;
	}
	device = open(HAKIOS_TOUCH_DEVICE, O_RDWR);
	if (device < 0) {
		perror("uibench: " HAKIOS_TOUCH_DEVICE);
		return 1;
	}
	if (argc >= 3 && strcmp(argv[1], "run") == 0) {
		for (int i = 3; i < argc; i++) {
			if (strcmp(argv[i], "prof") == 0) {
				prof_on = true;
			} else if (i + 1 >= argc) {
				break;
			} else if (strcmp(argv[i], "quiet") == 0) {
				quiet_ms = (unsigned)atoi(argv[++i]);
			} else if (strcmp(argv[i], "settle") == 0) {
				settle_ms = (unsigned)atoi(argv[++i]);
			} else if (strcmp(argv[i], "idle") == 0) {
				idle_s = (unsigned)atoi(argv[++i]);
			}
		}
		return run(argv[2]);
	}
	if (argc >= 3 && strcmp(argv[1], "do") == 0) {
		char why[256] = "";
		double still;

		ask_state(&s);
		if (!act(argv + 2, argc - 2, &s, why, sizeof(why))) {
			fprintf(stderr, "uibench: %s\n", why);
			return 1;
		}
		wait_still(now_ms(), &still);
		ask_state(&s);
		print_state("", &s);
		return 0;
	}
	fprintf(stderr, "usage: uibench run STEPS [quiet MS] [settle MS] [idle S] [prof] | uibench state | uibench do ACTION...\n");
	return 2;
}
