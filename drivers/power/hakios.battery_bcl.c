/*
 * The OnePlus 10 Pro's (SM8450) battery without its audio DSP: an estimate from what the application
 * processor can read itself. Only with boot-arg battery_soc=8450.
 *
 * On this phone the charger and the fuel gauge belong to the audio DSP (the stock tree's qcom,pmic_glink,
 * qcom,battery_charger with qcom,external_gauge): the gauge is a chip on a bus only the DSP reaches, and
 * the charger's registers have no channel of ours in the PMIC arbiter. What we can read is the PM8350B's
 * battery current limiter ("BCL", slave 3, peripheral 0x47, the stock tree's bcl@4700, Linux's
 * drivers/thermal/qcom/bcl_pmic5.c): it measures the battery's voltage and current all the time, for its
 * own alarms. Its registers, as bcl_pmic5.c has them:
 *   0x46  MONITOR_EN     [7] the monitor runs (Linux sets it at probe; the firmware leaves it on here)
 *   0x72  VBAT_CONV_REQ  not 0: the voltage is converted
 *   0x76  VBAT_READ      49.827 mV a step; 0x7f = no reading. On this phone the 1S side of the 2S pack:
 *                        one cell's voltage, the pack's half
 *   0x86  IBAT_READ      signed, 78.127 mA a step, + while the battery gives charge; 0 = no new reading
 * (iOSPlus10 read these on this phone in 2026-10: `73% 4085mV -78mA`, MONITOR_EN 0x80.)
 *
 * THIS DRIVER ONLY READS, through the arbiter's observer channel of that peripheral, the way
 * drivers/input/hakios.buttons.c reads the keys (same layout, same lock: hakios_spmi_lock()). The only
 * thing written is the command word that asks the arbiter for a read. Nothing is written to a PMIC.
 *
 * The percentage is an ESTIMATE, and said to be one (CapacityEstimated): the open-circuit voltage
 * (the voltage read, less the current times 80 mOhm, at most 150 mV either way) through a table for a
 * 4.45 V class cell (iOSPlus10 0271/0276), filtered, and shown moving at most 1 % per 30 s: never
 * down while the battery takes charge, never up while it gives it. 50 mV steps and a generic curve: expect
 * a few percent off at rest and more under load. The audio DSP's gauge (adsp=1, hakios.adsp.c's battery
 * manager) is the real value; when it reports, its values win (hakios.battery.c).
 *
 * Whether a charger is on the cable cannot be read here (that is the charger's, or the PMK8350 ADC's,
 * which needs writes): it is taken from the current's direction, with a margin (+50 mA charging: on a
 * charger; -50 mA: not). Under a load bigger than what the cable gives, a plugged phone reads as not on
 * a charger -- which is also the truth that matters: the battery is emptying.
 *
 * The percentage is kept across boots in /private/var/db/hakios_battery (a line of text) and used at the
 * next start when it is within 15 % of the voltage's estimate: the estimate alone would start a few percent
 * off with every boot's load.
 *
 * Boot-args:
 *   battery_soc=8450      do it
 *   battery_poll=<s>      how often (2 to 120; 10)
 *   battery_log=1         a line with every reading, not only when something changes
 *   battery_store=0       neither read nor write the file
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/fcntl.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>
#include <sys/time.h>
#include <sys/vnode.h>
#include <sys/mount.h>
#include <sys/vnode_internal.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void loader_log_putc(char c);

/* A line of the log, kept in the loader's log as well: the kernel's buffer had lost the first ones by the end of a run. */
static int say(const char *format, ...) __printflike(1, 2);
static int
say(const char *format, ...)
{
	char line[256];
	va_list arguments;

	va_start(arguments, format);
	vsnprintf(line, sizeof(line), format, arguments);
	va_end(arguments);
	for (const char *c = "battery: "; *c != 0; c++) {
		loader_log_putc(*c);
	}
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	return printf("battery: %s", line);
}

/* ---- the PMIC arbiter, version 7 (hakios.buttons.c's arb_sm8450: the stock tree's qcom,spmi@c42d000) ---- */

#define ARB_CORE_PHYS           0x0c400000UL
#define ARB_CORE_SIZE           0x3000UL
#define ARB_OBSERVER_PHYS       0x0c440000UL    /* system 0's part */
#define ARB_VERSION             0x000
#define ARB_FEATURES            0x004           /* [10:0] channels of bus 0 */
#define ARB_TABLE               0x2000          /* a word per channel: [19:8] slave and peripheral */
#define ARB_MAX_CHANNELS        1024
#define ARB_OBSERVER_CHANNEL    0x20
#define ARB_VERSION_7           0x70000000u
#define ARB_VERSION_8           0x80000000u
#define PAGE_16K                0x4000UL

#define OBSERVER_COMMAND        0x00
#define OBSERVER_STATUS         0x08
#define OBSERVER_DATA0          0x18
#define STATUS_DONE             0x1
#define STATUS_FAILURE          0x2
#define STATUS_DENIED           0x4
#define STATUS_DROPPED          0x8
#define COMMAND_READ            (1u << 27)      /* [11:4] register, [2:0] bytes - 1 */
#define READ_TIMEOUT_US         2000

#define BCL_ID                  0x347           /* slave 3 (PM8350B), peripheral 0x47 */
#define BCL_TYPE                0x04
#define BCL_SUBTYPE             0x05
#define BCL_MONITOR_EN          0x46
#define BCL_VBAT_CONV_REQ       0x72
#define BCL_VBAT_READ           0x76
#define BCL_IBAT_READ           0x86
#define BCL_VBAT_UV             49827
#define BCL_VBAT_NONE           0x7f
#define BCL_IBAT_UA             78127
#define BCL_SAMPLES             8

static vm_offset_t observer;            /* our channel's registers */
static uint64_t reads, failed_reads;

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

/* One register of the BCL, or false (and nothing in *value). */
static bool
bcl_read(uint8_t reg, uint8_t *value)
{
	uint64_t start = mach_absolute_time(), interval;
	uint32_t status, data;

	nanoseconds_to_absolutetime(READ_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	hakios_spmi_lock();
	*(volatile uint32_t *)(observer + OBSERVER_COMMAND) = COMMAND_READ | (uint32_t)reg << 4;
	do {
		status = rd(observer + OBSERVER_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	data = rd(observer + OBSERVER_DATA0);
	hakios_spmi_unlock();
	reads++;
	if (!(status & STATUS_DONE) || (status & (STATUS_FAILURE | STATUS_DENIED | STATUS_DROPPED))) {
		failed_reads++;
		return false;
	}
	*value = (uint8_t)data;
	return true;
}

/* Find the BCL's channel and map its observer registers. */
static bool
bcl_map(void)
{
	vm_offset_t core = ml_io_map(ARB_CORE_PHYS, ARB_CORE_SIZE), page;
	uint32_t version = rd(core + ARB_VERSION), channels = rd(core + ARB_FEATURES) & 0x7ff, channel = ARB_MAX_CHANNELS;

	if (version < ARB_VERSION_7 || version >= ARB_VERSION_8) {
		say("the PMIC arbiter is version %#x, not 7: this is not the SM8450's layout; no battery\n", version);
		return false;
	}
	if (channels == 0 || channels > ARB_MAX_CHANNELS) {
		channels = ARB_MAX_CHANNELS;
	}
	/* (the first channel of that peripheral: we only read, so whose it is to write does not matter) */
	for (uint32_t n = 0; n < channels; n++) {
		if (((rd(core + ARB_TABLE + 4 * n) >> 8) & 0xfff) == BCL_ID) {
			channel = n;
			break;
		}
	}
	if (channel == ARB_MAX_CHANNELS) {
		say("no channel of the arbiter (version %#x, %u channels) is the PM8350B's BCL: no battery\n", version, channels);
		return false;
	}
	page = ARB_OBSERVER_PHYS + (vm_offset_t)channel * ARB_OBSERVER_CHANNEL;
	observer = ml_io_map(page & ~(PAGE_16K - 1), PAGE_16K) + (page & (PAGE_16K - 1));
	say("the PM8350B's BCL is channel %u of the arbiter (version %#x)\n", channel, version);
	return true;
}

/* ---- one reading ----------------------------------------------------------------------------------- */

struct reading {
	bool voltage_ok, current_ok, monitor_on;
	uint8_t monitor, conversion;
	int32_t mv;                     /* one cell */
	int32_t ma;                     /* + while the battery takes charge */
	unsigned voltages, currents;
};

static void
bcl_sample(struct reading *r)
{
	int32_t vsum = 0, isum = 0;
	unsigned vn = 0, in = 0, nonzero = 0;
	uint8_t v;

	bzero(r, sizeof(*r));
	if (!bcl_read(BCL_MONITOR_EN, &r->monitor) || !bcl_read(BCL_VBAT_CONV_REQ, &r->conversion)) {
		return;
	}
	r->monitor_on = (r->monitor & 0x80) != 0;
	if (!r->monitor_on) {
		return;
	}
	for (unsigned i = 0; i < BCL_SAMPLES; i++) {
		if (r->conversion != 0 && bcl_read(BCL_VBAT_READ, &v) && v != BCL_VBAT_NONE && v != 0) {
			vsum += v;
			vn++;
		}
		if (bcl_read(BCL_IBAT_READ, &v)) {
			in++;
			if (v != 0) {
				isum += (int8_t)v;
				nonzero++;
			}
		}
		delay(1000);
	}
	if (vn >= BCL_SAMPLES / 2) {
		int32_t mv = (int32_t)((int64_t)vsum * BCL_VBAT_UV / ((int64_t)vn * 1000));

		if (mv > 5000) {
			mv /= 2;        /* (the pack after all: 2S) */
		}
		r->mv = mv;
		r->voltages = vn;
		r->voltage_ok = mv >= 2500 && mv <= 4700;
	}
	if (in >= BCL_SAMPLES / 2) {
		/* + is discharge in the BCL; Darwin wants + while charging */
		r->ma = nonzero ? -(int32_t)((int64_t)isum * BCL_IBAT_UA / ((int64_t)nonzero * 1000)) : 0;
		r->currents = nonzero;
		r->current_ok = true;
	}
}

/* ---- the estimate ------------------------------------------------------------------------------------ */

#define R_MOHM                  80              /* the cell's resistance, roughly */
#define IR_MAX_MV               150
#define OUTLIER_MV              250
#define REST_MA                 100
#define DIRECTION_MA            50
#define RATE_HUNDREDTHS_30S     100             /* 1 % per 30 s at most */
#define STORE_TRUST             1500            /* a saved value within 15 % of the estimate is believed */
#define DESIGN_UAH              5000000         /* the stock tree's qcom,batt_capacity_mah 5000 */

/* open-circuit voltage of one cell (mV) -> charge (hundredths of a percent); a 4.45 V class cell, generic */
static const struct { int32_t mv, soc; } ocv_table[] = {
	{3300, 0}, {3450, 200}, {3550, 400}, {3620, 600}, {3680, 1000}, {3720, 1500},
	{3750, 2000}, {3770, 2500}, {3790, 3000}, {3810, 3500}, {3840, 4000}, {3870, 4500},
	{3910, 5000}, {3950, 5500}, {4000, 6000}, {4050, 6500}, {4100, 7000}, {4150, 7500},
	{4200, 8000}, {4250, 8500}, {4300, 9000}, {4350, 9500}, {4400, 10000},
};

static int32_t
ocv_to_soc(int32_t mv)
{
	const unsigned n = sizeof(ocv_table) / sizeof(ocv_table[0]);

	if (mv <= ocv_table[0].mv) {
		return 0;
	}
	for (unsigned i = 1; i < n; i++) {
		if (mv < ocv_table[i].mv) {
			return ocv_table[i - 1].soc + (mv - ocv_table[i - 1].mv) * (ocv_table[i].soc - ocv_table[i - 1].soc) /
			       (ocv_table[i].mv - ocv_table[i - 1].mv);
		}
	}
	return 10000;
}

static struct {
	bool started;                   /* soc has a value */
	int32_t ocv_mv;                 /* filtered */
	int32_t avg_ma;                 /* filtered current, + charging */
	int32_t soc;                    /* shown, hundredths */
	int32_t target;
	unsigned outliers;
	bool external;
	int32_t status;
	uint32_t poll_s;
	bool log_all, store;
	int32_t restored;               /* -1: nothing restored (yet) */
	int32_t saved;                  /* the value last written, -1 none */
	uint64_t saved_at;              /* mach_absolute_time() */
	unsigned store_tries;
	bool store_failed;
} est = { .restored = -1, .saved = -1 };

/* ---- the file -------------------------------------------------------------------------------------- */

#define STORE_PATH              "/private/var/db/hakios_battery"

/* /private/var/db is on the data volume (init mounts it: an HFS volume, writable), not on Apple's sealed root. */
static bool
var_ready(void)
{
	vfs_context_t context = vfs_context_kernel();
	vnode_t vnode = NULLVP;
	mount_t mount;
	bool ready = false;

	if (rootvnode == NULLVP || vnode_lookup("/private/var/db", 0, &vnode, context) != 0) {
		return false;
	}
	mount = vnode_mount(vnode);
	if (mount != NULL && !vfs_isrdonly(mount) && strncmp(vfs_statfs(mount)->f_fstypename, "hfs", 3) == 0) {
		ready = true;
	}
	vnode_put(vnode);
	return ready;
}

static bool
store_read(int32_t *soc)
{
	vfs_context_t context = vfs_context_kernel();
	vnode_t vnode = NULLVP;
	char text[64];
	int left = 0, error;
	const char *p;
	int32_t value = 0;
	bool digits = false;

	if (rootvnode == NULLVP || vnode_lookup(STORE_PATH, 0, &vnode, context) != 0) {
		return false;
	}
	bzero(text, sizeof(text));
	error = vn_rdwr(UIO_READ, vnode, (caddr_t)text, sizeof(text) - 1, 0, UIO_SYSSPACE, 0,
	    vfs_context_ucred(context), &left, vfs_context_proc(context));
	vnode_put(vnode);
	if (error != 0) {
		return false;
	}
	/* "hakios-battery 1 soc <hundredths>" */
	if (strncmp(text, "hakios-battery 1 soc ", 21) != 0) {
		return false;
	}
	for (p = text + 21; *p >= '0' && *p <= '9' && value <= 10000; p++) {
		value = value * 10 + (*p - '0');
		digits = true;
	}
	if (!digits || value > 10000) {
		return false;
	}
	*soc = value;
	return true;
}

static int
store_write(int32_t soc)
{
	vfs_context_t context = vfs_context_kernel();
	vnode_t vnode = NULLVP;
	char text[64];
	int length, left = 0, error;

	if (rootvnode == NULLVP) {
		return ENOENT;
	}
	length = snprintf(text, sizeof(text), "hakios-battery 1 soc %d\n", soc);
	error = vnode_open(STORE_PATH, O_CREAT | O_TRUNC | FWRITE | O_NOFOLLOW, 0644, 0, &vnode, context);
	if (error != 0) {
		return error;
	}
	error = vn_rdwr(UIO_WRITE, vnode, (caddr_t)text, length, 0, UIO_SYSSPACE, IO_SYNC,
	    vfs_context_ucred(context), &left, vfs_context_proc(context));
	if (error == 0) {
		error = VNOP_FSYNC(vnode, MNT_WAIT, context);
	}
	vnode_close(vnode, FWRITE, context);
	return error != 0 ? error : left != 0 ? EIO : 0;
}

/* The saved value, once, at the first estimate (or soon after, while the root is not there yet). */
static void
store_restore(void)
{
	int32_t saved;

	if (!est.store || est.restored >= 0) {
		return;
	}
	if (!var_ready()) {
		/* (init mounts the data volume a little after the kernel's start: wait for it, ten minutes at most) */
		if (++est.store_tries * est.poll_s >= 600) {
			say("store: /private/var is not a writable HFS volume after 10 minutes: nothing restored or saved\n");
			est.store = false;
		}
		return;
	}
	if (!store_read(&saved)) {
		say("store: no saved percentage in " STORE_PATH ": the estimate %d.%02d%% stays\n", est.soc / 100, est.soc % 100);
		est.restored = est.soc;
		return;
	}
	if (saved - est.target > STORE_TRUST || est.target - saved > STORE_TRUST) {
		say("store: the saved %d.%02d%% is more than 15 %% from the estimate %d.%02d%% (charged elsewhere?): the estimate wins\n",
		    saved / 100, saved % 100, est.target / 100, est.target % 100);
		est.restored = est.soc;
		return;
	}
	say("store: %d.%02d%% restored (estimate %d.%02d%%)\n", saved / 100, saved % 100, est.target / 100, est.target % 100);
	est.soc = saved;
	est.restored = saved;
}

/* After a change of a whole percent, at most once a minute. */
static void
store_save(void)
{
	uint64_t now = mach_absolute_time(), minute;
	int error;

	if (!est.store || est.restored < 0 || (est.saved >= 0 && est.saved / 100 == est.soc / 100)) {
		return;
	}
	nanoseconds_to_absolutetime(60 * NSEC_PER_SEC, &minute);
	if (est.saved_at != 0 && now - est.saved_at < minute) {
		return;
	}
	est.saved_at = now;
	error = var_ready() ? store_write(est.soc) : EROFS;
	if (error == 0) {
		if (est.saved < 0) {
			say("store: %d%% saved to " STORE_PATH " (then with each change of a percent, once a minute at most)\n", est.soc / 100);
		}
		est.saved = est.soc;
		est.store_failed = false;
	} else if (!est.store_failed) {
		say("store: " STORE_PATH " not written (error %d): tried again each minute, said once\n", error);
		est.store_failed = true;
	}
}

/* ---- the loop --------------------------------------------------------------------------------------- */

static void
estimate(const struct reading *r)
{
	int32_t ir = r->ma * R_MOHM / 1000, ocv, step;

	ir = ir > IR_MAX_MV ? IR_MAX_MV : ir < -IR_MAX_MV ? -IR_MAX_MV : ir;
	ocv = r->mv - ir;               /* (charging lifts the terminal voltage above the open circuit's) */
	if (!est.started) {
		est.ocv_mv = ocv;
		est.avg_ma = r->ma;
	} else {
		est.avg_ma += (r->ma - est.avg_ma) / 4;
		if (ocv - est.ocv_mv > OUTLIER_MV || est.ocv_mv - ocv > OUTLIER_MV) {
			if (++est.outliers < 3) {
				return;
			}
		}
		est.outliers = 0;
		est.ocv_mv += (ocv - est.ocv_mv) / ((est.avg_ma <= REST_MA && est.avg_ma >= -REST_MA) ? 4 : 8);
	}
	est.target = ocv_to_soc(est.ocv_mv);
	if (!est.started) {
		est.soc = est.target;
		est.started = true;
		return;
	}
	step = (int32_t)(RATE_HUNDREDTHS_30S * est.poll_s / 30);
	if (est.target > est.soc && est.avg_ma > -DIRECTION_MA) {
		est.soc += est.target - est.soc < step ? est.target - est.soc : step;
	} else if (est.target < est.soc && est.avg_ma < DIRECTION_MA) {
		est.soc -= est.soc - est.target < step ? est.soc - est.target : step;
	}
}

static void
publish(const struct reading *r)
{
	struct hakios_battery_state b = { .size = sizeof(b) };

	if (est.avg_ma >= DIRECTION_MA) {
		est.external = true;
	} else if (est.avg_ma <= -DIRECTION_MA) {
		est.external = false;
	}
	/* Linux's POWER_SUPPLY_STATUS_*: 1 charging, 2 discharging, 3 not charging, 4 full */
	if (!est.external) {
		est.status = 2;
	} else if (est.soc >= 9900 && est.avg_ma < REST_MA) {
		est.status = 4;
	} else {
		est.status = est.avg_ma >= DIRECTION_MA ? 1 : 3;
	}
	b.percent_hundredths = est.soc;
	b.voltage_uv = r->mv * 1000;
	b.current_ua = est.avg_ma * 1000;
	b.temperature_hundredths = 0;
	b.present = 1;
	b.external = est.external;
	b.status = est.status;
	b.charge_uah = (int32_t)((int64_t)DESIGN_UAH * est.soc / 10000);
	b.design_uah = DESIGN_UAH;
	b.estimated = HAKIOS_BATTERY_ESTIMATED | HAKIOS_BATTERY_NO_TEMPERATURE;
	hakios_battery_report(&b);
}

static void
battery_bcl_thread(__unused void *parameter, __unused wait_result_t result)
{
	struct reading r;
	uint8_t type = 0, subtype = 0;
	int32_t last_percent = -1, last_status = -1;
	unsigned bad = 0;

	if (!bcl_map()) {
		return;
	}
	(void)bcl_read(BCL_TYPE, &type);
	(void)bcl_read(BCL_SUBTYPE, &subtype);
	bcl_sample(&r);
	say("BCL type %#x subtype %#x, MONITOR_EN %#x, VBAT_CONV_REQ %#x; reading every %u s (read only)\n", type, subtype,
	    r.monitor, r.conversion, est.poll_s);
	for (uint64_t tick = 0;; tick++) {
		if (tick != 0) {
			bcl_sample(&r);
		}
		if (!r.voltage_ok || !r.current_ok) {
			if (bad++ == 3) {
				say("no reading for 4 polls: MONITOR_EN %#x VBAT_CONV_REQ %#x, %u voltages %u currents (%llu reads, %llu "
				    "failed): no percentage until there is one\n", r.monitor, r.conversion, r.voltages, r.currents, reads,
				    failed_reads);
			}
		} else {
			bad = 0;
			estimate(&r);
			store_restore();
			publish(&r);
			store_save();
			if (est.log_all || est.soc / 100 != last_percent || est.status != last_status || tick % (600 / est.poll_s) == 0) {
				say("%d%% (estimate; target %d.%02d%%) %d mV/cell, ocv %d mV, %d mA (now %d), %s, status %d\n",
				    est.soc / 100, est.target / 100, est.target % 100, r.mv, est.ocv_mv, est.avg_ma, r.ma,
				    est.external ? "on a charger" : "on battery", est.status);
				last_percent = est.soc / 100;
				last_status = est.status;
			}
		}
		delay_for_interval(est.poll_s * 1000, NSEC_PER_MSEC);
	}
}

/* From hakios_battery_init(): true if it has started (battery_soc=8450). */
bool
hakios_battery_bcl_init(void)
{
	uint32_t soc = 0, value;
	thread_t thread;

	if (!PE_parse_boot_argn("battery_soc", &soc, sizeof(soc)) || soc != 8450) {
		return false;
	}
	est.poll_s = 10;
	if (PE_parse_boot_argn("battery_poll", &value, sizeof(value)) && value >= 2 && value <= 120) {
		est.poll_s = value;
	}
	est.log_all = PE_parse_boot_argn("battery_log", &value, sizeof(value)) && value != 0;
	est.store = !(PE_parse_boot_argn("battery_store", &value, sizeof(value)) && value == 0);
	if (kernel_thread_start(battery_bcl_thread, NULL, &thread) != KERN_SUCCESS) {
		say("no thread\n");
		return false;
	}
	thread_deallocate(thread);
	return true;
}
