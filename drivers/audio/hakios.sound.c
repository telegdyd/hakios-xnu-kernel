/*
 * The phone's sound, step 3: what the audio DSP's services are told so that the speakers play.
 * Every address and number below belongs to the POCO X5 Pro (redwood), and nothing here happens
 * unless boot-arg sound=N asks for it, with adsp=1 (hakios.adsp.c), which starts the DSP and
 * calls hakios_sound_dsp_up() once its audio services are up.
 *
 * Under Android the speakers' sound goes from the audio server through three of the DSP's
 * services: ASM, a stream the samples are written into; ADM, the matrix that routes a stream to
 * a device path ("COPP"); and AFE, the port at the SoC's interface. The two amplifiers
 * (hakios.speaker.c) sit on the primary TDM interface, which the DSP runs: Android's
 * PRI_TDM_RX_0, AFE port 0x9000 in group 0x9100 (with 0x9002, unused). The samples themselves
 * lie in memory of ours that the DSP reads through the application processors' SMMU, as
 * stream 0x1801 (Android: qcom,msm-audio-ion, iommus <&apps_smmu 0x1801 0>, its addresses from
 * 0x10000000, and the stream's low four bits in bits 32 to 35 of every address the DSP is given:
 * qcom,smmu-sid-mask 0xf, msm_audio_ion.c).
 *
 *   sound=1  the port: the TDM pins to the interface, the interface's bit clock, the group, the
 *            port, started; whether the bit clock and the frame sync then toggle at the pins is
 *            looked at; held for sound_secs (default 3), then everything stopped and put back
 *   sound=2  ... and a stream: our memory shown to the DSP through the SMMU, a device path
 *            (ADM) to the port, an ASM stream routed to it, and a test tone -- 1 kHz at -20 dBFS,
 *            both channels -- written into it for sound_secs (at most 3); the DSP's "written"
 *            events say whether it takes the samples at 48 kHz. The amplifiers stay off.
 *   sound=3  ... and the amplifiers, with the tone playing: set up as Android's driver sets them
 *            up, started once the clock runs, faded in to sound_below eighths of a dB under
 *            Android's own level (default 160: 20 dB under, the least it may be is 0), faded
 *            out and powered down at the end. Audible.
 *   sound=4  what Darwin plays (/dev/hakios-audio, hakios.audio.c), until the run ends: the port
 *            and the stream as above, kept running (silence when Darwin writes nothing); the
 *            amplifiers set up once and powered down, started and faded in (the driver's quick
 *            fade) whenever a write is not silence, stopped after three seconds of silence; their
 *            level is what is set on the plug-in's volume control (ioctl HAKIOS_AUDIO_SET_VOLUME):
 *            so far under sound_below, which stays the loudest, and at 0 they stay off. (Darwin sets
 *            that control only once, to the top, and applies its volume to the samples itself --
 *            see userland/src/audio_plugin.c --; at its volume 0 it writes silence, and the
 *            amplifiers stay off for that too.) On their way the samples go through the speakers'
 *            guard (hakios.guard.c), in place of the DSP's speaker protection that Android has:
 *            sound_gain eighths of a dB of gain (default and most 96, 12 dB: Darwin's own processing
 *            keeps its top that far under full scale), a high-pass filter, three averages of the
 *            power at the speakers and a peak limiter. sound_guard_as=N, for a test: the guard
 *            reckons the amplifiers N eighths of a dB under Android's level (louder than they are)
 *            and holds back as it would there -- with sound_below=720 (the amplifiers at -90 dB)
 *            what it does at the loudest can be seen in the log and the recording without a sound.
 *
 * The numbers are Android's own, from the phone's live device tree (qcom,msm-dai-tdm-pri-rx and
 * its port, the pins pri_tdm_*_active), its mixer paths (the speaker: PRI_TDM_RX_0 at 48 kHz,
 * two channels, S32_LE) and its kernel (Xiaomi's hakios-s-oss, techpack/audio: asoc/lahaina.c,
 * asoc/msm-dai-q6-v2.c, dsp/q6afe.c, dsp/q6adm.c, dsp/q6asm.c):
 *   - the bit clock: the interface's internal one (clock 0x200, "PRI_TDM_IBIT"), attribute 4
 *     ("invert, couple no", msm-dai-q6-v2.c's default; the tree names none), at 48000 x 32 bits x
 *     2 slots = 3.072 MHz -- the machine driver sets that rate itself (lahaina.c, which on redwood
 *     halves its four slots to two: "Using default tdm max slot: 4" in Android's log), not the
 *     tree's 1.536 MHz
 *   - the group: ports 0x9000 and 0x9002, two 32-bit slots, mask 3, 48 kHz
 *   - the port: two channels of 32 bits at 48 kHz, linear PCM; sync mode 0, sync source 1, data
 *     out 0, inverted sync 1, data delay 1; slot mapping: channels at byte offsets 0 and 4 (slot
 *     0, the amplifier at 0x35; slot 1, the one at 0x34), MSB-aligned
 *   - the pins: GPIO 97 (bit clock) and 100 (frame sync) function mi2s0, 8 mA, no pull, output
 *     high; 98 and 99 (data in and out) function mi2s0, 8 mA, no pull (pinctrl-yupik.c: function
 *     1 for all four)
 *   - the path: ADM's device open (version 5) for playback to port 0x9000 with no processing
 *     (topology 0x10312), two channels, 24 bits, 48 kHz; the stream: ASM session 1, stream 1,
 *     opened for writing linear PCM of 16 bits, no processing (0x10c68), routed to that path
 *
 * What is started is stopped, and what is written is put back, when the time is over -- or, if
 * the DSP is to be shut down earlier (the run's end), before that (hakios_sound_dsp_down()).
 * The amplifiers are powered down on a panic's way out (hakios_sound_quiesce()).
 *
 * Message layouts: the files above, and Linux's sound/soc/qcom/qdsp6/ (q6afe.c, q6adm.c,
 * q6asm.c); the SMMU: hakios.wifi.c, which does the same for the Wi-Fi's stream.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <libkern/OSAtomic.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/hakios.audio.h>
#include <sys/hakios.smmu.h>
#include <sys/hakios.mmio.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);
extern kern_return_t thread_terminate(thread_t thread);
extern void loader_log_putc(char c);

/* The SMMU's match registers and contexts are found free and taken by the Wi-Fi's start as well (hakios.wifi.c). */
static LCK_GRP_DECLARE(audio_space_group, "hakios-audio-space");
static LCK_MTX_DECLARE(audio_space_lock, &audio_space_group);

static LCK_GRP_DECLARE(sound_lock_group, "hakios-sound");
static LCK_MTX_DECLARE(sound_lock, &sound_lock_group);  /* the path being set up or taken down */

static uint32_t level, seconds = 3, below = 240, pause_ms = 1000, gain = HAKIOS_GUARD_MAX_GAIN;
static uint32_t guard_as = UINT32_MAX;  /* boot-arg sound_guard_as=N: the guard reckons the amplifiers at most N under (a test) */
static char last_write[96] = "nothing written";

/* ---- what is said: to the loader's log as well, as it happens ---------------------------------------- */

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[400];
	va_list list;
	int length;

	va_start(list, format);
	length = vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	printf("%s", line);
	return length;
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

static uint64_t
now_us(void)
{
	uint64_t nanoseconds;

	absolutetime_to_nanoseconds(mach_absolute_time(), &nanoseconds);
	return nanoseconds / 1000;
}

/* The time of day (UTC, as the host gave it: boot-arg epoch) and since the kernel began, for whoever listens. */
static void
say_when(const char *what)
{
	clock_sec_t secs;
	clock_usec_t usecs;
	uint64_t up = now_us();

	clock_get_calendar_microtime(&secs, &usecs);
	say("sound: %s at %02lu:%02lu:%02lu.%01u UTC (%llu.%01llu s after the kernel began)\n", what, (unsigned long)(secs / 3600 % 24),
	    (unsigned long)(secs / 60 % 60), (unsigned long)(secs % 60), (unsigned)(usecs / 100000), up / 1000000, up / 100000 % 10);
}

/* Time for the loader to note what has just been said, before something not yet seen to be harmless. */
static void
announce(const char *what)
{
	say("sound: %s\n", what);
	if (pause_ms) {
		sleep_ms(pause_ms);
	}
}

static uint32_t __attribute__((noinline))
rd(vm_offset_t address)
{
	uint32_t value;

	__asm__ volatile ("ldr %w0, [%1]" : "=r" (value) : "r" (address) : "memory");
	return value;
}

static void __attribute__((noinline))
wr(vm_offset_t address, uint32_t value)
{
	__asm__ volatile ("str %w0, [%1]" : : "r" (value), "r" (address) : "memory");
}

struct block {
	const char *name;
	uint32_t phys, size;
	vm_offset_t base;
};

static uint32_t
get(struct block *b, const char *name, uint32_t offset)
{
	uint32_t value;

	if (b->base == 0) {
		b->base = ml_io_map(b->phys, b->size);
	}
	value = rd(b->base + offset);
	say("sound: %s %s (%#x) = %#x\n", b->name, name, b->phys + offset, value);
	return value;
}

static void
put(struct block *b, const char *name, uint32_t offset, uint32_t value)
{
	if (b->base == 0) {
		b->base = ml_io_map(b->phys, b->size);
	}
	say("sound: about to write %s %s at %#x: %#x\n", b->name, name, b->phys + offset, value);
	snprintf(last_write, sizeof(last_write), "wrote %#x to %s %s at %#x", value, b->name, name, b->phys + offset);
	wr(b->base + offset, value);
}

/* ---- the TDM pins ------------------------------------------------------------------------------------- */

static struct block tlmm = { "pin controller", 0x0f100000, 0x100000 };
#define GPIO_CFG(n)             (0x1000u * (n))         /* [1:0] pull, [5:2] function, [8:6] drive (mA / 2 - 1), [9] output */
#define GPIO_IO(n)              (0x1000u * (n) + 4)     /* bit 0 in, bit 1 out */
#define CFG_MI2S0               (1u << 2)
#define CFG_8MA                 (3u << 6)
#define CFG_OUTPUT              (1u << 9)

struct pin {
	unsigned pin;
	const char *name;
	uint32_t cfg, io;               /* Android's active state */
};
static const struct pin redwood_pins[] = {
	{ 97, "gpio97 (bit clock)", CFG_MI2S0 | CFG_8MA | CFG_OUTPUT, 2 },
	{ 100, "gpio100 (frame sync)", CFG_MI2S0 | CFG_8MA | CFG_OUTPUT, 2 },
	{ 98, "gpio98 (data in)", CFG_MI2S0 | CFG_8MA, 0 },
	{ 99, "gpio99 (data out)", CFG_MI2S0 | CFG_8MA, 0 },
};
/*
 * The OnePlus 10 Pro (sound_soc=8450): the tertiary MI2S interface, Android's tert_aux_pcm_*_active states
 * (function mi2s2_sck/ws/data0/data1, function 1 of pinctrl-sm8450.c; 8 mA, no pull; the clock and the word
 * select output-high): the speakers' data goes out on SD1 (GPIO124), their feedback would come in on SD0.
 */
static const struct pin negroni_pins[] = {
	{ 121, "gpio121 (bit clock)", CFG_MI2S0 | CFG_8MA | CFG_OUTPUT, 2 },
	{ 123, "gpio123 (word select)", CFG_MI2S0 | CFG_8MA | CFG_OUTPUT, 2 },
	{ 122, "gpio122 (data 0, in)", CFG_MI2S0 | CFG_8MA, 0 },
	{ 124, "gpio124 (data 1, out)", CFG_MI2S0 | CFG_8MA, 0 },
};
#define TDM_PINS                4
static const struct pin *tdm_pins = redwood_pins;
static uint32_t found_cfg[TDM_PINS], found_io[TDM_PINS];
static bool pins_taken;

static void
pins_on(void)
{
	for (unsigned i = 0; i < TDM_PINS; i++) {
		found_cfg[i] = get(&tlmm, tdm_pins[i].name, GPIO_CFG(tdm_pins[i].pin));
		found_io[i] = get(&tlmm, tdm_pins[i].name, GPIO_IO(tdm_pins[i].pin));
	}
	pins_taken = true;
	for (unsigned i = 0; i < TDM_PINS; i++) {
		put(&tlmm, tdm_pins[i].name, GPIO_IO(tdm_pins[i].pin), tdm_pins[i].io);
		put(&tlmm, tdm_pins[i].name, GPIO_CFG(tdm_pins[i].pin), tdm_pins[i].cfg);
	}
}

static void
pins_off(void)
{
	if (!pins_taken) {
		return;
	}
	for (unsigned i = 0; i < TDM_PINS; i++) {
		put(&tlmm, tdm_pins[i].name, GPIO_CFG(tdm_pins[i].pin), found_cfg[i]);
		put(&tlmm, tdm_pins[i].name, GPIO_IO(tdm_pins[i].pin), found_io[i] & 2);
		get(&tlmm, tdm_pins[i].name, GPIO_CFG(tdm_pins[i].pin));
	}
	pins_taken = false;
}

/* Whether a pin toggles: read many times as fast as it goes. */
static void
pin_activity(unsigned pin, const char *name, uint32_t *changes, uint32_t *ones)
{
	uint32_t last = rd(tlmm.base + GPIO_IO(pin)) & 1, reads = 20000;
	uint64_t began = now_us();

	*changes = *ones = 0;
	for (uint32_t i = 0; i < reads; i++) {
		uint32_t now = rd(tlmm.base + GPIO_IO(pin)) & 1;

		*changes += now != last;
		*ones += now;
		last = now;
	}
	say("sound: %s: %u changes and %u ones in %u reads (%llu us)\n", name, *changes, *ones, reads, now_us() - began);
}

/* ---- APR ------------------------------------------------------------------------------------------- */

#define APR_SVC_AFE             4
#define APR_SVC_ASM             7
#define APR_SVC_ADM             8
#define APR_BASIC_RSP_RESULT    0x000110e8u
#define APR_WAIT_MS             1000

static const char *
dsp_error(uint32_t status)
{
	static const char *const names[] = { "ok", "failed", "bad parameter", "unsupported", "version", "unexpected", "panic",
		                             "no resource", "bad handle", "already", "not ready", "pending", "busy", "aborted",
		                             "preempted", "continue", "immediate", "not implemented", "need more", "no memory",
		                             "not there" };

	return status < sizeof(names) / sizeof(names[0]) ? names[status] : "an error this does not know";
}

/*
 * One command, its answer waited for: true if the answer is the basic one with status 0, or the
 * opcode `wanted' (then in *answer).
 */
static bool
command(const char *what, uint8_t service, uint16_t port, uint32_t opcode, const void *payload, uint32_t length, uint32_t wanted,
    struct hakios_apr_answer *answer)
{
	struct hakios_apr_answer local;

	if (answer == NULL) {
		answer = &local;
	}
	snprintf(last_write, sizeof(last_write), "sent %s (opcode %#x)", what, opcode);
	if (!hakios_adsp_apr(service, port, 0, opcode, payload, length, APR_WAIT_MS, answer)) {
		say("sound: %s (opcode %#x to service %u, port %#x): NO ANSWER\n", what, opcode, service, port);
		return false;
	}
	if (answer->opcode == APR_BASIC_RSP_RESULT && answer->words[0] == opcode) {
		say("sound: %s: %s%s\n", what, answer->words[1] == 0 ? "done" : "REFUSED: ", answer->words[1] == 0 ? "" :
		    dsp_error(answer->words[1]));
		return answer->words[1] == 0 && wanted == 0;
	}
	if (wanted != 0 && answer->opcode == wanted) {
		return true;
	}
	say("sound: %s: an answer not expected: opcode %#x, %u bytes: %#x %#x\n", what, answer->opcode, answer->length,
	    answer->words[0], answer->words[1]);
	return false;
}

/* ---- AFE: the port ---------------------------------------------------------------------------------- */

#define AFE_PORT_CMD_DEVICE_START               0x000100e5u
#define AFE_PORT_CMD_DEVICE_STOP                0x000100e6u
#define AFE_PORT_CMD_SET_PARAM_V2               0x000100efu
#define AFE_SVC_CMD_SET_PARAM                   0x000100f3u
#define AFE_MODULE_AUDIO_DEV_INTERFACE          0x0001020cu
#define AFE_PARAM_ID_TDM_CONFIG                 0x0001029du
#define AFE_MODULE_TDM                          0x0001028au
#define AFE_PARAM_ID_PORT_SLOT_MAPPING_CONFIG   0x00010297u
#define AFE_MODULE_GROUP_DEVICE                 0x00010254u
#define AFE_PARAM_ID_GROUP_DEVICE_ENABLE        0x00010256u
#define AFE_PARAM_ID_GROUP_DEVICE_TDM_CONFIG    0x0001029eu
#define AFE_MODULE_CLOCK_SET                    0x0001028fu
#define AFE_PARAM_ID_CLOCK_SET                  0x00010290u

#define PORT                    0x9000          /* PRI_TDM_RX_0 */
#define GROUP                   0x9100          /* PRI_TDM_RX */
#define GROUP_OTHER_PORT        0x9002
#define PORT_INVALID            0xffff
#define CLOCK_PRI_TDM_IBIT      0x200
#define CLOCK_INVERT_COUPLE_NO  4
#define RATE                    48000
#define SLOTS                   2
#define SLOT_BITS               32
#define PORT_BITS               32              /* the mixer path's S32_LE */

struct afe_param {
	uint32_t module, param;
	uint16_t size, reserved;
} __attribute__((packed));

struct afe_clock_set {
	uint32_t version, id, hz;
	uint16_t attribute, root;
	uint32_t enable;
} __attribute__((packed));

struct afe_group_tdm {
	uint32_t version;
	uint16_t group, reserved;
	uint16_t ports[8];
	uint32_t channels, rate, bits;
	uint16_t slots, slot_bits;
	uint32_t slot_mask;
} __attribute__((packed));

struct afe_group_enable {
	uint16_t group, enable;
} __attribute__((packed));

struct afe_tdm {
	uint32_t version, channels, rate, bits;
	uint16_t data_format, sync_mode, sync_source, slots, data_out, invert_sync, data_delay, slot_bits;
	uint32_t slot_mask;
} __attribute__((packed));

struct afe_slot_mapping {
	uint32_t version;
	uint16_t channels, bits;
	uint32_t align;
	uint16_t offsets[8];
} __attribute__((packed));

/* A parameter for the AFE as a whole (its clocks, its groups): in the packet, not in shared memory. */
static bool
afe_service_param(const char *what, uint32_t module, uint32_t param, const void *data, uint16_t size)
{
	uint8_t packet[16 + sizeof(struct afe_param) + 64];
	uint32_t *head = (uint32_t *)packet;
	struct afe_param *p = (struct afe_param *)(packet + 16);

	bzero(packet, sizeof(packet));
	head[0] = (uint32_t)sizeof(*p) + size;          /* payload size; address 0 and handle 0: in band */
	p->module = module;
	p->param = param;
	p->size = size;
	memcpy(packet + 16 + sizeof(*p), data, size);
	return command(what, APR_SVC_AFE, 0, AFE_SVC_CMD_SET_PARAM, packet, 16 + (uint32_t)sizeof(*p) + size, 0, NULL);
}

/* A parameter of the port. */
static bool
afe_port_param(const char *what, uint32_t module, uint32_t param, const void *data, uint16_t size)
{
	uint8_t packet[16 + sizeof(struct afe_param) + 64];
	struct afe_param *p = (struct afe_param *)(packet + 16);

	bzero(packet, sizeof(packet));
	*(uint16_t *)packet = PORT;
	*(uint16_t *)(packet + 2) = (uint16_t)(sizeof(*p) + size);
	p->module = module;
	p->param = param;
	p->size = size;
	memcpy(packet + 16 + sizeof(*p), data, size);
	return command(what, APR_SVC_AFE, 0, AFE_PORT_CMD_SET_PARAM_V2, packet, 16 + (uint32_t)sizeof(*p) + size, 0, NULL);
}

static bool clock_on, group_on, port_on;

static bool
afe_clock(bool on)
{
	struct afe_clock_set clock = { 1, CLOCK_PRI_TDM_IBIT, RATE * SLOT_BITS * SLOTS, CLOCK_INVERT_COUPLE_NO, 0, on };

	return afe_service_param(on ? "the TDM bit clock (0x200) at 3.072 MHz, on" : "the TDM bit clock, off", AFE_MODULE_CLOCK_SET,
	           AFE_PARAM_ID_CLOCK_SET, &clock, sizeof(clock));
}

static bool
afe_on(void)
{
	struct afe_group_tdm group = {
		.version = 1, .group = GROUP, .ports = { PORT, GROUP_OTHER_PORT, PORT_INVALID, PORT_INVALID, PORT_INVALID,
		                                         PORT_INVALID, PORT_INVALID, PORT_INVALID },
		.channels = SLOTS, .rate = RATE, .bits = SLOT_BITS, .slots = SLOTS, .slot_bits = SLOT_BITS, .slot_mask = 3,
	};
	struct afe_group_enable enable = { GROUP, 1 };
	struct afe_tdm tdm = {
		.version = 1, .channels = 2, .rate = RATE, .bits = PORT_BITS, .data_format = 0, .sync_mode = 0, .sync_source = 1,
		.slots = SLOTS, .data_out = 0, .invert_sync = 1, .data_delay = 1, .slot_bits = SLOT_BITS, .slot_mask = 3,
	};
	struct afe_slot_mapping mapping = { .version = 1, .channels = 2, .bits = PORT_BITS, .align = 0,
		                            .offsets = { 0, 4, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff } };
	uint16_t start[2] = { PORT, 0 };

	announce("the DSP is asked for the TDM interface's bit clock, the group, the port and its start (AFE)");
	clock_on = afe_clock(true);
	if (!clock_on) {
		return false;
	}
	group_on = afe_service_param("the group 0x9100: ports 0x9000 0x9002, 2 slots of 32 bits, 48 kHz", AFE_MODULE_GROUP_DEVICE,
	    AFE_PARAM_ID_GROUP_DEVICE_TDM_CONFIG, &group, sizeof(group)) &&
	    afe_service_param("the group 0x9100, on", AFE_MODULE_GROUP_DEVICE, AFE_PARAM_ID_GROUP_DEVICE_ENABLE, &enable,
	    sizeof(enable));
	if (!group_on) {
		return false;
	}
	if (!afe_port_param("the port 0x9000: 2 channels of 32 bits at 48 kHz in 2 slots, sync 0 1, inverted, delay 1",
	    AFE_MODULE_AUDIO_DEV_INTERFACE, AFE_PARAM_ID_TDM_CONFIG, &tdm, sizeof(tdm)) ||
	    !afe_port_param("the port's slots: channels at bytes 0 and 4", AFE_MODULE_TDM, AFE_PARAM_ID_PORT_SLOT_MAPPING_CONFIG,
	    &mapping, sizeof(mapping))) {
		return false;
	}
	port_on = command("the port 0x9000, started", APR_SVC_AFE, 0, AFE_PORT_CMD_DEVICE_START, start, sizeof(start), 0, NULL);
	return port_on;
}

static void
afe_off(void)
{
	uint16_t stop[2] = { PORT, 0 };
	struct afe_group_enable disable = { GROUP, 0 };

	if (port_on) {
		command("the port 0x9000, stopped", APR_SVC_AFE, 0, AFE_PORT_CMD_DEVICE_STOP, stop, sizeof(stop), 0, NULL);
		port_on = false;
	}
	if (group_on) {
		afe_service_param("the group 0x9100, off", AFE_MODULE_GROUP_DEVICE, AFE_PARAM_ID_GROUP_DEVICE_ENABLE, &disable,
		    sizeof(disable));
		group_on = false;
	}
	if (clock_on) {
		afe_clock(false);
		clock_on = false;
	}
}

/* ---- the DSP's view of our memory: stream 0x1801 of the SMMU ------------------------------------------- */

#define AUDIO_STREAM            0x1801
#define AUDIO_ADDRESSES         0x10000000ull   /* dt: qcom,iommu-dma-addr-pool of qcom,msm-audio-ion */
#define AUDIO_STREAM_BITS       (1ull << 32)    /* the stream's low bits (0x1801 & 0xf), where the DSP takes them from */

#define PERIODS                 8
#define PERIOD_FRAMES           960             /* 20 ms */
#define PERIOD_BYTES            (PERIOD_FRAMES * 4)
#define BUFFER_BYTES            (PERIODS * 4096)        /* a page each */
#define COMMAND_BYTES           4096                    /* behind them: AudioReach's commands too big for GLINK (ar_command()) */
#define MIC_BYTES               (16 * 4096)             /* the microphones' (hakios.mic.c), behind those */
#define SHARED_BYTES            (BUFFER_BYTES + COMMAND_BYTES + MIC_BYTES)

/* C3 owns hardware and tables; audio_space_lock owns this client/user count.
 * Never hold hakios_smmu_lock while calling C3 (the helper takes it itself).
 */
static struct { bool passed, on; struct hakios_smmu_context *ctx; } space;
static vm_offset_t buffer;
static uint64_t buffer_address;
extern lck_mtx_t hakios_smmu_lock;

/* Preserve the old redwood boot-chain bypass case without rewriting its SMR.
 * Unknown/refused reads fail closed; C3 then rejects an overlapping owned SMR.
 */
static bool
space_is_bypassed(void)
{
    vm_offset_t regs; unsigned id0, scr0; bool passed=false;
    lck_mtx_lock(&hakios_smmu_lock);
    regs=ml_io_map(0x15000000,0x2000);
    if (!regs || hakios_mmio_try_read32((void *)regs,&scr0) || (scr0&(1u<<3)) ||
        hakios_mmio_try_read32((void *)(regs+0x20),&id0)) goto out;
    for (unsigned n=0;n<(id0&255);n++) {
        unsigned smr,s2cr;
        if (hakios_mmio_try_read32((void *)(regs+0x800+4*n),&smr) ||
            hakios_mmio_try_read32((void *)(regs+0xc00+4*n),&s2cr)) break;
        if ((smr>>31) && !((smr^AUDIO_STREAM)&0x7fff&~(smr>>16))) {
            passed=((s2cr>>16)&3)==1; break;
        }
    }
out:
    lck_mtx_unlock(&hakios_smmu_lock);
    return passed;
}

static void
space_faults(void)
{
    if (space.ctx) hakios_smmu_faults(space.ctx);
}

static bool
space_on(void)
{
    space.passed=space_is_bypassed();
    if (space.passed) return true;
    space.ctx=hakios_smmu_claim(AUDIO_STREAM,0,AUDIO_ADDRESSES,SHARED_BYTES,"sound");
    if (!space.ctx) return false;
    for (unsigned page=0;page<SHARED_BYTES/4096;page++) {
        if (hakios_smmu_map(space.ctx,AUDIO_ADDRESSES+4096*page,kvtophys(buffer+4096*page),
            4096,HAKIOS_SMMU_READ|HAKIOS_SMMU_WRITE)) {
            hakios_smmu_release(space.ctx); space.ctx=NULL; return false;
        }
    }
    space.on=true;
    return true;
}

static void
space_off(void)
{
    space_faults();
    hakios_smmu_release(space.ctx);
    space.ctx=NULL; space.on=false; space.passed=false;
}

/*
 * The DSP's view of our memory is one: the sound's periods and the microphones' buffers behind them
 * (hakios.mic.c), through the same stream and context. Whoever comes first sets it up, whoever
 * leaves last takes it down. user: 0 the sound, 1 the microphones. With audio_space_lock held.
 */
static uint32_t space_users;

static bool
shared_get(unsigned user)
{
	if (space_users == 0) {
		if (buffer == 0 && kmem_alloc(kernel_map, &buffer, SHARED_BYTES, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
		    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
			buffer = 0;
			say("sound: no memory for the DSP's buffers\n");
			return false;
		}
		if (!space.passed && !space.on && !space_on()) {
			return false;
		}
		buffer_address = space.passed ? (kvtophys(buffer) | AUDIO_STREAM_BITS) : (AUDIO_ADDRESSES | AUDIO_STREAM_BITS);
		if (space.passed) {
			for (unsigned page = 1; page < SHARED_BYTES / 4096; page++) {
				if (kvtophys(buffer + 4096 * page) != kvtophys(buffer) + 4096 * page) {
					say("sound: the DSP's buffers are not one piece of memory, and the stream is not translated\n");
					return false;
				}
			}
		}
	}
	space_users |= 1u << user;
	return true;
}

static void
shared_put(unsigned user)
{
	if (!(space_users & 1u << user)) {
		return;
	}
	space_users &= ~(1u << user);
	if (space_users == 0) {
		space_off();
	}
}

bool
hakios_audio_dsp_memory(uintptr_t *memory, uint64_t *dsp_address, uint32_t *bytes)
{
	bool ok;

	lck_mtx_lock(&audio_space_lock);
	ok = shared_get(1);
	lck_mtx_unlock(&audio_space_lock);
	if (ok) {
		*memory = buffer + BUFFER_BYTES + COMMAND_BYTES;
		*dsp_address = buffer_address + BUFFER_BYTES + COMMAND_BYTES;
		*bytes = MIC_BYTES;
	}
	return ok;
}

void
hakios_audio_dsp_memory_put(void)
{
	lck_mtx_lock(&audio_space_lock);
	shared_put(1);
	lck_mtx_unlock(&audio_space_lock);
}

/* ---- ADM and ASM: the path and the stream --------------------------------------------------------------- */

#define ADM_CMD_DEVICE_OPEN_V5                  0x00010326u
#define ADM_CMDRSP_DEVICE_OPEN_V5               0x00010329u
#define ADM_CMD_DEVICE_CLOSE_V5                 0x00010327u
#define ADM_CMD_MATRIX_MAP_ROUTINGS_V5          0x00010325u
#define ADM_PATH_PLAYBACK                       1
#define ADM_MATRIX_AUDIO_RX                     0
#define ADM_TOPOLOGY_NONE                       0x00010312u
#define ADM_BITS                                24

#define ASM_CMD_SHARED_MEM_MAP_REGIONS          0x00010d92u
#define ASM_CMDRSP_SHARED_MEM_MAP_REGIONS       0x00010d93u
#define ASM_CMD_SHARED_MEM_UNMAP_REGIONS        0x00010d94u
#define ASM_STREAM_CMD_OPEN_WRITE_V3            0x00010db3u
#define ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2        0x00010d98u
#define ASM_SESSION_CMD_RUN_V2                  0x00010daau
#define ASM_DATA_CMD_WRITE_V2                   0x00010dabu
#define ASM_DATA_EVENT_WRITE_DONE_V2            0x00010d99u
#define ASM_SESSION_EVENT_RX_UNDERFLOW          0x00010c17u
#define ASM_STREAM_CMD_CLOSE                    0x00010bcdu
#define ASM_TOPOLOGY_NONE                       0x00010c68u
#define ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2      0x00010da5u
#define ASM_SHMEM_4K_POOL                       3
#define SESSION                                 1
#define STREAM_PORT                             (SESSION << 8 | 1)
#define WRITE_TOKEN                             0x80000000u     /* | the period: what a write's "done" carries back */
#define PCM_CHANNEL_FL                          1
#define PCM_CHANNEL_FR                          2

static volatile bool stopping;          /* the run is ending: whatever plays stops */
static bool mapped, copp_open, stream_open;
static uint32_t map_handle, copp;
static volatile uint32_t returned;      /* the periods the DSP has said it is done with (a bit each) */
static volatile uint32_t written_done, underflows, write_errors;

/* The DSP's events: the "done" of every write, and underflows. On the link's thread. */
static bool
events(uint8_t service, __unused uint16_t port, uint32_t opcode, uint32_t token, const uint32_t *payload, uint32_t length)
{
	if (service != APR_SVC_ASM) {
		return false;
	}
	if (opcode == ASM_DATA_EVENT_WRITE_DONE_V2 && (token & WRITE_TOKEN)) {
		if (length >= 16 && payload[3] != 0) {
			OSIncrementAtomic((volatile SInt32 *)&write_errors);
		}
		OSIncrementAtomic((volatile SInt32 *)&written_done);
		OSBitOrAtomic(1u << (token & 31), &returned);
		return true;
	}
	if (opcode == APR_BASIC_RSP_RESULT && (token & WRITE_TOKEN)) {
		/* (a write the DSP refused outright: it answers it as a command) */
		OSIncrementAtomic((volatile SInt32 *)&write_errors);
		OSIncrementAtomic((volatile SInt32 *)&written_done);
		OSBitOrAtomic(1u << (token & 31), &returned);
		return true;
	}
	if (opcode == ASM_SESSION_EVENT_RX_UNDERFLOW) {
		OSIncrementAtomic((volatile SInt32 *)&underflows);
		return true;
	}
	return false;
}

static bool
stream_on(void)
{
	struct {
		uint16_t pool, regions;
		uint32_t property;
		uint32_t low, high, size;
	} __attribute__((packed)) map = { ASM_SHMEM_4K_POOL, 1, 0, (uint32_t)buffer_address, (uint32_t)(buffer_address >> 32),
		                          BUFFER_BYTES };
	struct {
		uint16_t flags, mode, endpoint1, endpoint2;
		uint32_t topology;
		uint16_t channels, bits;
		uint32_t rate;
		uint8_t map[8];
	} __attribute__((packed)) open = { 0, ADM_PATH_PLAYBACK, PORT, 0xffff, ADM_TOPOLOGY_NONE, 2, ADM_BITS, RATE,
		                           { PCM_CHANNEL_FL, PCM_CHANNEL_FR } };
	struct {
		uint32_t matrix, sessions;
		uint16_t session, copps;
		uint16_t copp[2];
	} __attribute__((packed)) route = { ADM_MATRIX_AUDIO_RX, 1, SESSION, 1, { 0, 0 } };
	struct {
		uint32_t flags;
		uint16_t sink, bits;
		uint32_t topology, format;
	} __attribute__((packed)) write = { 0, 0, 16, ASM_TOPOLOGY_NONE, ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2 };
	struct {
		uint32_t size;
		uint16_t channels, bits;
		uint32_t rate;
		uint16_t is_signed, reserved;
		uint8_t map[8];
	} __attribute__((packed)) format = { 20, 2, 16, RATE, 1, 0, { PCM_CHANNEL_FL, PCM_CHANNEL_FR } };
	uint32_t run[3] = { 0, 0, 0 };
	struct hakios_apr_answer answer;

	announce("the DSP is given our periods (ASM), a device path to the port (ADM) and a stream routed to it (ASM)");
	if (!command("our periods, mapped for the DSP", APR_SVC_ASM, 0, ASM_CMD_SHARED_MEM_MAP_REGIONS, &map, sizeof(map),
	    ASM_CMDRSP_SHARED_MEM_MAP_REGIONS, &answer)) {
		return false;
	}
	mapped = true;
	map_handle = answer.words[0];
	say("sound: our %u KB at %#llx are the DSP's handle %#x\n", BUFFER_BYTES / 1024, buffer_address, map_handle);
	if (!command("a device path to the port: playback, no processing, 2 channels of 24 bits at 48 kHz", APR_SVC_ADM, PORT,
	    ADM_CMD_DEVICE_OPEN_V5, &open, sizeof(open), ADM_CMDRSP_DEVICE_OPEN_V5, &answer)) {
		return false;
	}
	if (answer.words[0] != 0) {
		say("sound: the device path: REFUSED: %s\n", dsp_error(answer.words[0]));
		return false;
	}
	copp_open = true;
	copp = answer.words[1] & 0xffff;        /* (the upper half is reserved) */
	route.copp[0] = (uint16_t)copp;
	say("sound: the device path is open: COPP %u\n", copp);
	if (!command("the stream, opened for writing: 16-bit linear PCM, no processing", APR_SVC_ASM, STREAM_PORT,
	    ASM_STREAM_CMD_OPEN_WRITE_V3, &write, sizeof(write), 0, NULL)) {
		return false;
	}
	stream_open = true;
	return command("session 1 routed to the device path", APR_SVC_ADM, 0, ADM_CMD_MATRIX_MAP_ROUTINGS_V5, &route, sizeof(route),
	           0, NULL) &&
	       command("the stream's format: 2 channels of 16 bits at 48 kHz, left and right", APR_SVC_ASM, STREAM_PORT,
	           ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2, &format, sizeof(format), 0, NULL) &&
	       command("the session, run", APR_SVC_ASM, STREAM_PORT, ASM_SESSION_CMD_RUN_V2, run, sizeof(run), 0, NULL);
}

static void
stream_off(void)
{
	uint32_t handle = map_handle;

	if (stream_open) {
		command("the stream, closed", APR_SVC_ASM, STREAM_PORT, ASM_STREAM_CMD_CLOSE, NULL, 0, 0, NULL);
		stream_open = false;
	}
	if (copp_open) {
		command("the device path, closed", APR_SVC_ADM, (uint16_t)copp, ADM_CMD_DEVICE_CLOSE_V5, NULL, 0, 0, NULL);
		copp_open = false;
	}
	if (mapped) {
		command("our periods, unmapped", APR_SVC_ASM, 0, ASM_CMD_SHARED_MEM_UNMAP_REGIONS, &handle, sizeof(handle), 0, NULL);
		mapped = false;
	}
}

/* ---- AudioReach: the OnePlus 10 Pro's DSP (sound_soc=8450) ----------------------------------------------------------- */

/*
 * On SM8450 the DSP's audio framework is AudioReach (SPF), spoken to in GPR packets (hakios_adsp_gpr()),
 * and what Android's AFE, ADM and ASM did is one graph of modules there. What is said and in which
 * order is Linux's (sound/soc/qcom/qdsp6/audioreach.c, q6apm.c, q6prm.c; the topology of
 * linux-msm/audioreach-topology's SM8450-HDK.m4 and audioreach/subgraph-*.m4 for the shapes):
 *
 *   the clock    PRM (instance 2) is asked for the tertiary MI2S interface's internal bit clock (clock
 *                0x104, TER_MI2S_IBIT) at 48000 x 2 x 32 = 3.072 MHz, attribute 1 ("couple no"), root 0
 *                -- Android's speaker back end MI2S-LPAIF-RX-TERTIARY, 2 channels of 24 bits in 32-bit
 *                slots (resourcemanager.xml, AK msm_common.c), the SoC the master
 *   the graph    one GRAPH_OPEN with two subgraphs, as the topology's stream and device subgraphs are:
 *                  0x4001 stream (low latency, playback): container 0x5001 (capability "CD", stack
 *                         8192, the ADSP) with WR_SHARED_MEM_EP 0x6001 -> PCM_DEC 0x6002 -> PCM_CNV
 *                         0x6003 -> MFC 0x6004
 *                  0x4002 device: container 0x5002 ("EP") with I2S_SINK 0x6005 (tertiary, SD1, the
 *                         word select ours)
 *                MFC's output to I2S_SINK's input; output ports are 1, input ports 2
 *   its setup    SET_CFG to each module as q6apm_graph_media_format_shmem()/_pcm() send them: the
 *                shared-memory end point takes 16-bit PCM, 2 channels, 48 kHz (PARAM_ID_MEDIA_FORMAT);
 *                PCM_DEC and PCM_CNV give 16 bits out (PARAM_ID_PCM_OUTPUT_FORMAT_CFG, interleaved and
 *                de-interleaved unpacked); MFC gives 24 bits (PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT), the
 *                I2S end point is set to 48 kHz, 24 bits, 2 channels (PARAM_ID_HW_EP_MF_CFG), its
 *                interface (PARAM_ID_I2S_INTF_CFG) and a frame-size factor of 1
 *   the memory   our periods mapped for the DSP (APM_CMD_SHARED_MEM_MAP_REGIONS, the 4K pool), through
 *                the SMMU as on the X5 Pro (stream 0x1801, the address's bits 32 on the stream's low bits)
 *   then         GRAPH_PREPARE and GRAPH_START of both subgraphs; each period written with
 *                DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2 to the end point, its "done" (0x05001004) coming
 *                back to our port with the period in the token
 *   at the end   GRAPH_STOP, the memory unmapped, GRAPH_CLOSE, the clock released
 */
#define AR_PORT_APM                     1
#define AR_PORT_PRM                     2
#define AR_OURS                         3               /* our port for APM's commands (Android's spf_core: 3) */
#define AR_OURS_PRM                     7               /* and for PRM's (Android's q6prm: 7) */
#define AR_OURS_DATA                    0x10000001u     /* the graph's data commands (Linux: GPR_DYNAMIC_PORT_START on) */
#define AR_WAIT_MS                      5000            /* (Linux's q6apm and q6prm wait 5 s for an answer) */
#define GPR_BASIC_RSP_RESULT            0x02001005u
#define APM_CMD_GRAPH_OPEN              0x01001000u
#define APM_CMD_GRAPH_PREPARE           0x01001001u
#define APM_CMD_GRAPH_START             0x01001002u
#define APM_CMD_GRAPH_STOP              0x01001003u
#define APM_CMD_GRAPH_CLOSE             0x01001004u
#define APM_CMD_SET_CFG                 0x01001006u
#define APM_CMD_SHARED_MEM_MAP_REGIONS  0x0100100cu
#define APM_CMD_SHARED_MEM_UNMAP_REGIONS 0x0100100du
#define APM_CMD_RSP_SHARED_MEM_MAP_REGIONS 0x02001001u
#define PRM_CMD_REQUEST_HW_RSC          0x0100100fu
#define PRM_CMD_RSP_REQUEST_HW_RSC      0x02001002u
#define PRM_CMD_RELEASE_HW_RSC          0x01001010u
#define PRM_CMD_RSP_RELEASE_HW_RSC      0x02001003u
#define PARAM_ID_RSC_AUDIO_HW_CLK       0x0800102cu
#define APM_PARAM_ID_CONTAINER_CONFIG   0x08001000u
#define APM_PARAM_ID_SUB_GRAPH_CONFIG   0x08001001u
#define APM_PARAM_ID_MODULE_LIST        0x08001002u
#define APM_PARAM_ID_MODULE_PROP        0x08001003u
#define APM_PARAM_ID_MODULE_CONN        0x08001004u
#define APM_PARAM_ID_SUB_GRAPH_LIST     0x08001005u
#define PARAM_ID_PCM_OUTPUT_FORMAT_CFG  0x08001008u
#define PARAM_ID_MEDIA_FORMAT           0x0800100cu
#define APM_SUB_GRAPH_PROP_ID_PERF_MODE 0x0800100eu
#define APM_SUB_GRAPH_PROP_ID_DIRECTION 0x0800100fu
#define APM_SUB_GRAPH_PROP_ID_SCENARIO_ID 0x08001010u
#define APM_CONTAINER_PROP_ID_CAPABILITY_LIST 0x08001011u
#define APM_CONTAINER_PROP_ID_GRAPH_POS 0x08001012u
#define APM_CONTAINER_PROP_ID_STACK_SIZE 0x08001013u
#define APM_CONTAINER_PROP_ID_PROC_DOMAIN 0x08001014u
#define APM_MODULE_PROP_ID_PORT_INFO    0x08001015u
#define PARAM_ID_HW_EP_MF_CFG           0x08001017u
#define PARAM_ID_HW_EP_FRAME_SIZE_FACTOR 0x08001018u
#define PARAM_ID_I2S_INTF_CFG           0x08001019u
#define PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT 0x08001024u
#define DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2 0x0400100au
#define DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE_V2 0x05001004u
#define MODULE_ID_WR_SHARED_MEM_EP      0x07001000u
#define MODULE_ID_PCM_CNV               0x07001003u
#define MODULE_ID_PCM_DEC               0x07001005u
#define MODULE_ID_I2S_SINK              0x0700100au
#define MODULE_ID_MFC                   0x07001015u
#define MEDIA_FMT_ID_PCM                0x09001000u
#define CLOCK_TER_MI2S_IBIT             0x104
#define AR_BCLK                         (RATE * 2 * 32)
#define AR_SG_STREAM                    0x4001
#define AR_SG_DEVICE                    0x4002
#define AR_CONT_STREAM                  0x5001
#define AR_CONT_DEVICE                  0x5002
#define AR_WR_EP                        0x6001
#define AR_PCM_DEC                      0x6002
#define AR_PCM_CNV                      0x6003
#define AR_MFC                          0x6004
#define AR_I2S                          0x6005
#define AR_DEVICE_BITS                  24

static uint32_t sound_soc;              /* boot-arg sound_soc: 8450 the OnePlus 10 Pro (AudioReach, the TFA9874 amplifiers) */
#define AR                              (sound_soc == 8450)
static bool ar_clock_on, ar_open, ar_started;

static const char *
ar_error(uint32_t status)
{
	/* (AudioReach's AR_E* are the same numbers as APR's ADSP_E*: ar_error_codes.h) */
	return dsp_error(status);
}

/* A packet's words built up: what each of the commands below takes, little-endian 32-bit words. */
struct words {
	uint32_t w[256];
	unsigned n;
	bool full;
};

static void
word(struct words *p, uint32_t value)
{
	if (p->n >= sizeof(p->w) / sizeof(p->w[0])) {
		p->full = true;
		return;
	}
	p->w[p->n++] = value;
}

static void
halves(struct words *p, uint16_t low, uint16_t high)
{
	word(p, (uint32_t)low | (uint32_t)high << 16);
}

/*
 * One command to a module of the DSP, its answer waited for: true if it is the basic answer with status 0 for this
 * opcode, or the opcode `wanted' (then in *answer). `apm_header': the command's payload is behind APM's command
 * header (in band: address 0, handle 0, and the payload's size), as audioreach_alloc_cmd_pkt() makes it.
 */
static bool
ar_command(const char *what, uint32_t to_port, uint32_t from_port, uint32_t opcode, bool apm_header, const struct words *p,
    uint32_t wanted, struct hakios_apr_answer *answer)
{
	struct hakios_apr_answer local;
	uint32_t packet[4 + 256];
	uint32_t length = 0;

	if (answer == NULL) {
		answer = &local;
	}
	if (p != NULL && p->full) {
		say("sound: %s: THE PACKET IS TOO BIG to be built here: not sent\n", what);
		return false;
	}
	bzero(packet, sizeof(packet));
	if (apm_header) {
		packet[3] = p != NULL ? p->n * 4 : 0;
		length = 16;
	}
	if (p != NULL) {
		memcpy((uint8_t *)packet + length, p->w, p->n * 4);
		length += p->n * 4;
	}
	snprintf(last_write, sizeof(last_write), "sent %s (opcode %#x to %#x)", what, opcode, to_port);
	if (!hakios_adsp_gpr(to_port, from_port, 0, opcode, packet, length, AR_WAIT_MS, answer)) {
		uint32_t bytes = p != NULL ? p->n * 4 : 0;
		uint64_t at = buffer_address + BUFFER_BYTES;

		/*
		 * Not sent (no intent of the DSP's that big): APM's commands take their payload out of band too, from memory the
		 * DSP has mapped -- the header's address, the map's handle and the size instead of the payload behind it
		 */
		if (hakios_adsp_gpr_error == 0 || !apm_header || bytes == 0 || bytes > COMMAND_BYTES || !mapped) {
			say("sound: %s (opcode %#x to port %#x, %u bytes): %s\n", what, opcode, to_port, length,
			    hakios_adsp_gpr_error == 0 ? "NO ANSWER" : "NOT SENT");
			return false;
		}
		memcpy((void *)(buffer + BUFFER_BYTES), p->w, bytes);
		for (vm_offset_t line = buffer + BUFFER_BYTES; line < buffer + BUFFER_BYTES + bytes; line += 64) {
			__asm__ volatile ("dc civac, %0" : : "r" (line));
		}
		__asm__ volatile ("dsb sy");
		packet[0] = (uint32_t)at;
		packet[1] = (uint32_t)(at >> 32);
		packet[2] = map_handle;
		packet[3] = bytes;
		say("sound: %s: too big for the link (%u bytes), sent out of band: its %u bytes at %#llx, handle %#x\n", what, length,
		    bytes, at, map_handle);
		if (!hakios_adsp_gpr(to_port, from_port, 0, opcode, packet, 16, AR_WAIT_MS, answer)) {
			say("sound: %s, out of band (opcode %#x to port %#x): %s\n", what, opcode, to_port,
			    hakios_adsp_gpr_error == 0 ? "NO ANSWER" : "NOT SENT");
			return false;
		}
	}
	if (answer->opcode == GPR_BASIC_RSP_RESULT && answer->words[0] == opcode) {
		say("sound: %s: %s%s\n", what, answer->words[1] == 0 ? "done" : "REFUSED: ", answer->words[1] == 0 ? "" :
		    ar_error(answer->words[1]));
		return answer->words[1] == 0 && wanted == 0;
	}
	if (wanted != 0 && answer->opcode == wanted) {
		return true;
	}
	say("sound: %s: an answer not expected: opcode %#x, %u bytes: %#x %#x\n", what, answer->opcode, answer->length,
	    answer->words[0], answer->words[1]);
	return false;
}

/* A module's parameter in a SET_CFG: its instance, the parameter, the size, error 0; then the data (padded to 8). */
static void
param(struct words *p, uint32_t instance, uint32_t id, uint32_t bytes)
{
	word(p, instance);
	word(p, id);
	word(p, bytes);
	word(p, 0);
}

static void
pad8(struct words *p)
{
	if (p->n & 1) {
		word(p, 0);
	}
}

/* PRM: the bit clock of the tertiary MI2S interface, asked for or released. */
static bool
ar_clock(bool on)
{
	struct words p = { .n = 0 };
	struct hakios_apr_answer answer;
	bool ok;

	param(&p, AR_PORT_PRM, PARAM_ID_RSC_AUDIO_HW_CLK, on ? 20 : 8);
	word(&p, 1);                                    /* one clock */
	word(&p, CLOCK_TER_MI2S_IBIT);
	if (on) {
		word(&p, AR_BCLK);
		word(&p, 1);                            /* Q6PRM_LPASS_CLK_ATTRIBUTE_COUPLE_NO */
		word(&p, 0);                            /* the default root */
	}
	ok = ar_command(on ? "PRM: the tertiary MI2S bit clock (0x104) at 3.072 MHz" : "PRM: the tertiary MI2S bit clock, released",
	    AR_PORT_PRM, AR_OURS_PRM, on ? PRM_CMD_REQUEST_HW_RSC : PRM_CMD_RELEASE_HW_RSC, true, &p,
	    on ? PRM_CMD_RSP_REQUEST_HW_RSC : PRM_CMD_RSP_RELEASE_HW_RSC, &answer);
	if (ok) {
		/* (q6prm.c's callback: the answer's payload is the basic result, opcode then status) */
		say("sound: %s: %s%s\n", on ? "the bit clock" : "the bit clock's release", answer.words[1] == 0 ? "GRANTED" : "REFUSED: ",
		    answer.words[1] == 0 ? "" : ar_error(answer.words[1]));
		ok = answer.words[1] == 0;
	}
	return ok;
}

/* The graph: two subgraphs, two containers, five modules, four connections (see above). */
static bool
ar_graph_open(void)
{
	static const struct {
		uint32_t subgraph, container, module, instance, inputs, outputs;
	} modules[] = {
		{ AR_SG_STREAM, AR_CONT_STREAM, MODULE_ID_WR_SHARED_MEM_EP, AR_WR_EP, 0, 1 },
		{ AR_SG_STREAM, AR_CONT_STREAM, MODULE_ID_PCM_DEC, AR_PCM_DEC, 1, 1 },
		{ AR_SG_STREAM, AR_CONT_STREAM, MODULE_ID_PCM_CNV, AR_PCM_CNV, 1, 1 },
		{ AR_SG_STREAM, AR_CONT_STREAM, MODULE_ID_MFC, AR_MFC, 1, 1 },
		{ AR_SG_DEVICE, AR_CONT_DEVICE, MODULE_ID_I2S_SINK, AR_I2S, 1, 0 },
	};
	static const uint32_t connections[][2] = {
		{ AR_WR_EP, AR_PCM_DEC }, { AR_PCM_DEC, AR_PCM_CNV }, { AR_PCM_CNV, AR_MFC }, { AR_MFC, AR_I2S },
	};
	struct words p = { .n = 0 };
	unsigned at;

	/* the subgraphs: perf mode 2 (low latency), direction (the stream 2, the device 1, as the topology has them), playback */
	at = p.n;
	param(&p, AR_PORT_APM, APM_PARAM_ID_SUB_GRAPH_CONFIG, 0);
	word(&p, 2);
	for (unsigned s = 0; s < 2; s++) {
		word(&p, s == 0 ? AR_SG_STREAM : AR_SG_DEVICE);
		word(&p, 3);
		word(&p, APM_SUB_GRAPH_PROP_ID_PERF_MODE);
		word(&p, 4);
		word(&p, 2);
		word(&p, APM_SUB_GRAPH_PROP_ID_DIRECTION);
		word(&p, 4);
		word(&p, s == 0 ? 2 : 1);
		word(&p, APM_SUB_GRAPH_PROP_ID_SCENARIO_ID);
		word(&p, 4);
		word(&p, 1);
	}
	pad8(&p);
	p.w[at + 2] = (p.n - at - 4) * 4;
	/* the containers: capability CD (2) for the stream, EP (3) for the device; graph position "stream", stack 8192, the ADSP */
	at = p.n;
	param(&p, AR_PORT_APM, APM_PARAM_ID_CONTAINER_CONFIG, 0);
	word(&p, 2);
	for (unsigned c = 0; c < 2; c++) {
		word(&p, c == 0 ? AR_CONT_STREAM : AR_CONT_DEVICE);
		word(&p, 4);
		word(&p, APM_CONTAINER_PROP_ID_CAPABILITY_LIST);
		word(&p, 8);
		word(&p, 1);
		word(&p, c == 0 ? 2 : 3);
		word(&p, APM_CONTAINER_PROP_ID_GRAPH_POS);
		word(&p, 4);
		word(&p, 1);
		word(&p, APM_CONTAINER_PROP_ID_STACK_SIZE);
		word(&p, 4);
		word(&p, 8192);
		word(&p, APM_CONTAINER_PROP_ID_PROC_DOMAIN);
		word(&p, 4);
		word(&p, 2);
	}
	pad8(&p);
	p.w[at + 2] = (p.n - at - 4) * 4;
	/* the modules of each container */
	at = p.n;
	param(&p, AR_PORT_APM, APM_PARAM_ID_MODULE_LIST, 0);
	word(&p, 2);
	for (unsigned c = 0; c < 2; c++) {
		uint32_t container = c == 0 ? AR_CONT_STREAM : AR_CONT_DEVICE;
		unsigned count = 0;

		for (unsigned m = 0; m < sizeof(modules) / sizeof(modules[0]); m++) {
			count += modules[m].container == container;
		}
		word(&p, c == 0 ? AR_SG_STREAM : AR_SG_DEVICE);
		word(&p, container);
		word(&p, count);
		for (unsigned m = 0; m < sizeof(modules) / sizeof(modules[0]); m++) {
			if (modules[m].container == container) {
				word(&p, modules[m].module);
				word(&p, modules[m].instance);
			}
		}
	}
	pad8(&p);
	p.w[at + 2] = (p.n - at - 4) * 4;
	/* their ports */
	at = p.n;
	param(&p, AR_PORT_APM, APM_PARAM_ID_MODULE_PROP, 0);
	word(&p, sizeof(modules) / sizeof(modules[0]));
	for (unsigned m = 0; m < sizeof(modules) / sizeof(modules[0]); m++) {
		word(&p, modules[m].instance);
		word(&p, 1);
		word(&p, APM_MODULE_PROP_ID_PORT_INFO);
		word(&p, 8);
		word(&p, modules[m].inputs);
		word(&p, modules[m].outputs);
	}
	pad8(&p);
	p.w[at + 2] = (p.n - at - 4) * 4;
	/* the connections: output port 1 to input port 2 */
	at = p.n;
	param(&p, AR_PORT_APM, APM_PARAM_ID_MODULE_CONN, 0);
	word(&p, sizeof(connections) / sizeof(connections[0]));
	for (unsigned c = 0; c < sizeof(connections) / sizeof(connections[0]); c++) {
		word(&p, connections[c][0]);
		word(&p, 1);
		word(&p, connections[c][1]);
		word(&p, 2);
	}
	pad8(&p);
	p.w[at + 2] = (p.n - at - 4) * 4;
	say("sound: the graph: %u bytes of GRAPH_OPEN\n", p.n * 4 + 16);
	ar_open = ar_command("APM: GRAPH_OPEN (the stream 0x4001: WR_SHARED_MEM_EP, PCM_DEC, PCM_CNV, MFC; the device 0x4002: "
	    "I2S_SINK)", AR_PORT_APM, AR_OURS, APM_CMD_GRAPH_OPEN, true, &p, 0, NULL);
	return ar_open;
}

/* The modules' formats, each a SET_CFG of its own (as audioreach_set_media_format() sends them). */
static bool
ar_graph_formats(void)
{
	struct words p;

	/* the end point's input: 16-bit PCM, 2 channels, 48 kHz (Linux's payload_size for the format, kept as it is) */
	p.n = 0;
	p.full = false;
	param(&p, AR_WR_EP, PARAM_ID_MEDIA_FORMAT, 32);
	word(&p, 1);                                    /* DATA_FORMAT_FIXED_POINT */
	word(&p, MEDIA_FMT_ID_PCM);
	word(&p, 48 - 12);
	word(&p, RATE);
	halves(&p, 16, 1);                              /* bit width, alignment: LSB */
	halves(&p, 16, 15);                             /* bits per sample, Q factor */
	halves(&p, 1, 2);                               /* little-endian, channels */
	halves(&p, PCM_CHANNEL_FL | PCM_CHANNEL_FR << 8, 0);
	if (!ar_command("the end point 0x6001's input: 16-bit PCM, 2 channels, 48 kHz", AR_WR_EP, AR_OURS_DATA, APM_CMD_SET_CFG, true,
	    &p, 0, NULL)) {
		return false;
	}
	/* PCM_DEC (interleaved) and PCM_CNV (de-interleaved unpacked): 16 bits out */
	for (unsigned i = 0; i < 2; i++) {
		p.n = 0;
		p.full = false;
		param(&p, i == 0 ? AR_PCM_DEC : AR_PCM_CNV, PARAM_ID_PCM_OUTPUT_FORMAT_CFG, 32);
		word(&p, 1);
		word(&p, MEDIA_FMT_ID_PCM);
		word(&p, 20);
		halves(&p, 16, 1);                      /* bit width, alignment */
		halves(&p, 16, 15);                     /* bits per sample, Q factor */
		halves(&p, 1, i == 0 ? 1 : 3);          /* endianness, interleaving */
		halves(&p, 0, 2);                       /* reserved, channels */
		halves(&p, PCM_CHANNEL_FL | PCM_CHANNEL_FR << 8, 0);
		if (!ar_command(i == 0 ? "PCM_DEC 0x6002: 16 bits out, interleaved" : "PCM_CNV 0x6003: 16 bits out, de-interleaved",
		    AR_PORT_APM, AR_OURS, APM_CMD_SET_CFG, true, &p, 0, NULL)) {
			return false;
		}
	}
	/* MFC: 48 kHz, 24 bits, 2 channels (left, right) out */
	p.n = 0;
	p.full = false;
	param(&p, AR_MFC, PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT, 12);
	word(&p, RATE);
	halves(&p, AR_DEVICE_BITS, 2);
	halves(&p, PCM_CHANNEL_FL, PCM_CHANNEL_FR);
	if (!ar_command("MFC 0x6004: 48 kHz, 24 bits, 2 channels out", AR_PORT_APM, AR_OURS, APM_CMD_SET_CFG, true, &p, 0, NULL)) {
		return false;
	}
	/* I2S_SINK: the interface (tertiary, SD1, the word select internal), its format, a frame-size factor of 1 */
	p.n = 0;
	p.full = false;
	param(&p, AR_I2S, PARAM_ID_I2S_INTF_CFG, 16);
	word(&p, 0);                                    /* LPAIF */
	word(&p, 2);                                    /* I2S_INTF_TYPE_TERTIARY */
	halves(&p, 2, 1);                               /* I2S_SD1; CONFIG_I2S_WS_SRC_INTERNAL */
	word(&p, 0);
	param(&p, AR_I2S, PARAM_ID_HW_EP_MF_CFG, 16);
	word(&p, RATE);
	halves(&p, AR_DEVICE_BITS, 2);
	word(&p, 1);                                    /* fixed point */
	word(&p, 0);
	param(&p, AR_I2S, PARAM_ID_HW_EP_FRAME_SIZE_FACTOR, 8);
	word(&p, 1);
	word(&p, 0);
	return ar_command("I2S_SINK 0x6005: tertiary MI2S, SD1, 48 kHz, 24 bits, 2 channels", AR_PORT_APM, AR_OURS, APM_CMD_SET_CFG,
	           true, &p, 0, NULL);
}

static bool
ar_graph_command(const char *what, uint32_t opcode)
{
	struct words p = { .n = 0 };

	param(&p, AR_PORT_APM, APM_PARAM_ID_SUB_GRAPH_LIST, 16);
	word(&p, 2);
	word(&p, AR_SG_STREAM);
	word(&p, AR_SG_DEVICE);
	pad8(&p);
	return ar_command(what, AR_PORT_APM, AR_OURS, opcode, true, &p, 0, NULL);
}

/* The DSP's events: the "done" of every write. On the link's thread. */
static bool
ar_events(uint8_t service, uint16_t port, uint32_t opcode, uint32_t token, const uint32_t *payload, uint32_t length)
{
	if (service != HAKIOS_GPR_SERVICE || port != AR_WR_EP || !(token & WRITE_TOKEN)) {
		return false;
	}
	if (opcode == DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE_V2) {
		if (length >= 16 && payload[3] != 0) {
			OSIncrementAtomic((volatile SInt32 *)&write_errors);
		}
	} else if (opcode == GPR_BASIC_RSP_RESULT) {
		/* (a write the DSP refused outright: it answers it as a command) */
		OSIncrementAtomic((volatile SInt32 *)&write_errors);
	} else {
		return false;
	}
	OSIncrementAtomic((volatile SInt32 *)&written_done);
	OSBitOrAtomic(1u << (token & 31), &returned);
	return true;
}

static bool
ar_stream_on(void)
{
	struct words p = { .n = 0 };
	struct hakios_apr_answer answer;

	announce("the DSP is given the graph (APM: GRAPH_OPEN), its formats (SET_CFG), our periods (SHARED_MEM_MAP), and the graph "
	    "is prepared and started");
	/* (the memory first: a GRAPH_OPEN too big for the link goes out of band, from the page behind the periods) */
	halves(&p, 3, 1);                               /* APM_MEMORY_MAP_SHMEM8_4K_POOL, one region */
	word(&p, 0);
	word(&p, (uint32_t)buffer_address);
	word(&p, (uint32_t)(buffer_address >> 32));
	word(&p, BUFFER_BYTES + COMMAND_BYTES);
	if (!ar_command("our periods, mapped for the DSP", AR_PORT_APM, AR_OURS_DATA, APM_CMD_SHARED_MEM_MAP_REGIONS, false, &p,
	    APM_CMD_RSP_SHARED_MEM_MAP_REGIONS, &answer)) {
		return false;
	}
	mapped = true;
	map_handle = answer.words[0];
	say("sound: our %u KB at %#llx are the DSP's handle %#x\n", (BUFFER_BYTES + COMMAND_BYTES) / 1024, buffer_address,
	    map_handle);
	if (!ar_graph_open() || !ar_graph_formats()) {
		return false;
	}
	if (!ar_graph_command("APM: GRAPH_PREPARE", APM_CMD_GRAPH_PREPARE)) {
		return false;
	}
	ar_started = ar_graph_command("APM: GRAPH_START", APM_CMD_GRAPH_START);
	return ar_started;
}

static void
ar_stream_off(void)
{
	if (ar_started) {
		ar_graph_command("APM: GRAPH_STOP", APM_CMD_GRAPH_STOP);
		ar_started = false;
	}
	if (mapped) {
		struct words p = { .n = 0 };

		word(&p, map_handle);
		ar_command("our periods, unmapped", AR_PORT_APM, AR_OURS_DATA, APM_CMD_SHARED_MEM_UNMAP_REGIONS, false, &p, 0, NULL);
		mapped = false;
	}
	if (ar_open) {
		ar_graph_command("APM: GRAPH_CLOSE", APM_CMD_GRAPH_CLOSE);
		ar_open = false;
	}
}

static bool
ar_period_write(unsigned period)
{
	uint64_t at = buffer_address + 4096 * period;
	uint32_t write[11] = { (uint32_t)at, (uint32_t)(at >> 32), map_handle, PERIOD_BYTES, 0, 0, 0, 0, 0, 0, 0 };

	return hakios_adsp_gpr(AR_WR_EP, AR_OURS_DATA, WRITE_TOKEN | period, DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2, write,
	           sizeof(write), 0, NULL);
}

/*
 * The speakers' level in software, on the OnePlus 10 Pro: the TFA9874s play at Android's own gain (the
 * container's), and everything under it is taken off the samples here, after the guard (hakios.tfa.c).
 * In Q15, from eighths of a dB (10^(-1/160) an eighth).
 */
static volatile uint32_t soft_q15 = 0;  /* 0: silence until a level is set */

static uint32_t
q15_of(uint32_t below)
{
	uint64_t g = 1ull << 30;

	if (below >= 90 * 8) {
		return 0;
	}
	for (uint32_t db = 0; db < below / 8; db++) {
		g = g * 957037441ull >> 30;             /* -1 dB */
	}
	for (uint32_t e = 0; e < below % 8; e++) {
		g = g * 1058396570ull >> 30;            /* -1/8 dB */
	}
	return (uint32_t)(g >> 15);
}

void
hakios_sound_soft_level(uint32_t below, bool muted)
{
	soft_q15 = muted ? 0 : q15_of(below);
}

static void
soft_apply(int16_t *samples, uint32_t count)
{
	uint32_t g = soft_q15;

	for (uint32_t i = 0; i < count; i++) {
		samples[i] = (int16_t)(((int32_t)samples[i] * (int32_t)g) >> 15);
	}
}

/* ---- the test tone ----------------------------------------------------------------------------------- */

/* 1 kHz at 48 kHz, -20 dBFS (a peak of 3277): one cycle */
static const int16_t tone[48] = {
	0, 428, 848, 1254, 1638, 1995, 2317, 2600, 2838, 3028, 3165, 3249,
	3277, 3249, 3165, 3028, 2838, 2600, 2317, 1995, 1638, 1254, 848, 428,
	0, -428, -848, -1254, -1638, -1995, -2317, -2600, -2838, -3028, -3165, -3249,
	-3277, -3249, -3165, -3028, -2838, -2600, -2317, -1995, -1638, -1254, -848, -428,
};
static uint32_t tone_at;

static void
period_fill(unsigned period, bool silent)
{
	int16_t *samples = (int16_t *)(buffer + 4096 * period);

	for (unsigned frame = 0; frame < PERIOD_FRAMES; frame++) {
		int16_t sample = silent ? 0 : tone[tone_at++ % 48];

		samples[2 * frame] = sample;
		samples[2 * frame + 1] = sample;
	}
	if (AR) {
		soft_apply(samples, 2 * PERIOD_FRAMES);
	}
	for (vm_offset_t line = (vm_offset_t)samples; line < (vm_offset_t)samples + PERIOD_BYTES; line += 64) {
		__asm__ volatile ("dc cvac, %0" : : "r" (line));        /* the DSP reads memory, not our caches */
	}
	__asm__ volatile ("dsb sy");
}

static bool
period_write(unsigned period)
{
	uint32_t write[8] = { (uint32_t)(buffer_address + 4096 * period), (uint32_t)((buffer_address + 4096 * period) >> 32),
		              map_handle, PERIOD_BYTES, period, 0, 0, 0 };

	if (AR) {
		return ar_period_write(period);
	}
	return hakios_adsp_apr(APR_SVC_ASM, STREAM_PORT, WRITE_TOKEN | period, ASM_DATA_CMD_WRITE_V2, write, sizeof(write), 0, NULL);
}

/*
 * The tone, written period after period as the DSP gives them back, for `ms'; then silence until
 * the DSP has given every period back. What came back and how fast is said.
 */
static bool
play(uint32_t ms, bool amplifiers)
{
	uint64_t began, last_look, ended;
	uint32_t sent = 0, done_before = written_done, healthy_looks = 0;
	bool ok = true, tone_over = false;

	returned = 0;
	tone_at = 0;
	for (unsigned p = 0; p < PERIODS; p++) {
		period_fill(p, false);
	}
	began = last_look = now_us();
	for (unsigned p = 0; p < PERIODS; p++) {
		if (!period_write(p)) {
			say("sound: the write of period %u could not be sent\n", p);
			return false;
		}
		sent++;
	}
	for (;;) {
		uint32_t back = returned;
		uint64_t now = now_us();

		if (back != 0) {
			OSBitAndAtomic(~back, &returned);
			for (unsigned p = 0; p < PERIODS; p++) {
				if (!(back & 1u << p) || tone_over) {
					continue;
				}
				period_fill(p, false);
				if (!period_write(p)) {
					say("sound: the write of period %u could not be sent\n", p);
					ok = false;
					tone_over = true;
					break;
				}
				sent++;
			}
		}
		if (!tone_over && (now - began >= ms * 1000ull || stopping)) {
			tone_over = true;
		}
		if (tone_over && written_done - done_before >= sent) {
			break;
		}
		if (now - began > ms * 1000ull + 2000000) {
			say("sound: THE DSP HAS NOT GIVEN EVERY PERIOD BACK 2 s after the end (%u of %u)\n", written_done - done_before,
			    sent);
			ok = false;
			break;
		}
		if (written_done == done_before && now - began > 1000000) {
			say("sound: THE DSP HAS GIVEN NO PERIOD BACK IN A SECOND\n");
			ok = false;
			break;
		}
		if (amplifiers && now - last_look >= 200000) {
			last_look = now;
			healthy_looks++;
			if (!hakios_speaker_healthy()) {
				ok = false;
				tone_over = true;
			}
		}
		sleep_ms(2);
	}
	ended = now_us();
	{
		uint32_t done = written_done - done_before;
		uint64_t frames = (uint64_t)done * PERIOD_FRAMES, us = ended - began;

		say("sound: %u periods written, %u given back (%u with an error), %u underflows, in %llu ms: the DSP took %llu frames a "
		    "second%s\n", sent, done, write_errors, underflows, us / 1000, us ? frames * 1000000 / us : 0,
		    amplifiers ? "" : " (the amplifiers off: nothing to hear)");
		if (amplifiers) {
			say("sound: the amplifiers' status looked at %u times while it played\n", healthy_looks);
		}
	}
	return ok;
}

/* ---- sound=4: what Darwin plays ------------------------------------------------------------------------------ */

/*
 * /dev/hakios-audio's writer (the audio server's real-time thread, hakios.audio.c) puts frames
 * into a ring and does not wait; the feeder, a thread of ours, takes them from there into each
 * period the DSP gives back (and silence when there is nothing), so the stream never stops
 * while the run lasts. The amplifiers are only on while there is sound: the watcher starts them
 * (and fades them in to sound_below) when a write is not silence, and stops them after three
 * seconds of silence or no writes.
 */
#define RING_FRAMES             32768           /* 0.68 s; a power of two, the indices run on and wrap */
#define SILENCE                 8               /* samples no larger than this are silence (as hakios.audio.c's) */
#define QUIET_BEFORE_OFF_US     3000000ull

static int16_t ring[RING_FRAMES * 2];
static volatile uint32_t ring_head, ring_tail;  /* frames: written by the sink, taken by the feeder */
static volatile uint64_t last_sound_us;         /* when the latest write that was not silence came */
static volatile bool feeder_runs, watcher_runs;
static uint64_t fed_frames, silent_frames, darwin_frames;
static volatile uint32_t short_periods, dropped_frames, primings;
static bool primed;                             /* the ring holds its cushion (period_fill_darwin()) */
#define CUSHION_FRAMES          (2 * PERIOD_FRAMES)   /* (since the last 5 s line) a period cut short in sound; frames the ring had no room for */
static volatile uint32_t darwin_peak;              /* the largest sample since the amplifiers last started */

/* From the writer's thread, under hakios.audio.c's lock: never waits. */
static uint32_t
darwin_sink(const int16_t *frames, uint32_t count)
{
	uint32_t head = ring_head, room = RING_FRAMES - (head - ring_tail), taken = count < room ? count : room;
	bool sound = false;

	for (uint32_t i = 0; i < taken; i++) {
		int16_t left = frames[2 * i], right = frames[2 * i + 1];
		uint32_t l = left < 0 ? (uint32_t)-(int32_t)left : (uint32_t)left, r = right < 0 ? (uint32_t)-(int32_t)right : (uint32_t)right;

		ring[2 * ((head + i) % RING_FRAMES)] = left;
		ring[2 * ((head + i) % RING_FRAMES) + 1] = right;
		sound |= l > SILENCE || r > SILENCE;
		if (l > darwin_peak || r > darwin_peak) {
			darwin_peak = l > r ? l : r;
		}
	}
	OSMemoryBarrier();
	ring_head = head + taken;
	darwin_frames += taken;
	dropped_frames += count - taken;
	if (sound) {
		last_sound_us = now_us();
	}
	return taken;
}

static uint32_t guard_periods;                  /* periods through the guard */
static volatile bool guard_on;                  /* the amplifiers are on: what the guard lets through is heard */

/* tenths of a dB as text: "-5.7" */
static const char *
db_text(char *text, int tenths)
{
	snprintf(text, 16, "%s%d.%d", tenths < 0 ? "-" : "", (tenths < 0 ? -tenths : tenths) / 10, (tenths < 0 ? -tenths : tenths) % 10);
	return text;
}

/* What the guard has done since it was last said (hakios.guard.c), as one line; nothing if `quiet'. */
static void
guard_say(const char *when, bool quiet)
{
	struct hakios_guard_report r;
	char t[6][16];

	hakios_guard_take_report(&r);
	if (r.blocks == 0 || quiet) {
		return;
	}
	/* (levels at the speakers: dB under a full-scale square wave at Android's level; gains in dB) */
	say("sound: the guard%s: peaks in %u, out %u (of 32768); gain now %s dB, least %s dB, held down by the 0.2 s/3 s/60 s "
	    "averages in %u/%u/%u of %u periods; peaks limited in %u frames (by %s dB at most); averages %s/%s/%s dB\n",
	    when, r.peak_in, r.peak_out, db_text(t[0], hakios_guard_decibels(r.gain * r.gain >> 30)),
	    db_text(t[1], hakios_guard_decibels(r.least_gain * r.least_gain >> 30)), r.rms_blocks, r.fast_blocks, r.slow_blocks,
	    r.blocks, r.limited_frames, db_text(t[2], hakios_guard_decibels((uint64_t)r.least_peak_gain * r.least_peak_gain >> 2)),
	    db_text(t[3], hakios_guard_decibels(r.rms)), db_text(t[4], hakios_guard_decibels(r.fast)),
	    db_text(t[5], hakios_guard_decibels(r.slow)));
}

/* A period from the ring, the rest of it silence. */
static void
period_fill_darwin(unsigned period)
{
	int16_t *samples = (int16_t *)(buffer + 4096 * period);
	uint32_t tail, available, n;

	/*
	 * In the middle of sound a period is not cut short: the rest of it silence was a click each time Darwin's writes
	 * came a little after the DSP's period (round 2). Up to 10 ms are waited for the rest -- the period is played
	 * behind the seven queued at the DSP, 140 ms on -- and only then is it filled with what there is.
	 */
	for (unsigned waited = 0; guard_on && primed && ring_head - ring_tail < PERIOD_FRAMES && ring_head != ring_tail && waited < 10;
	    waited++) {
		sleep_ms(1);
	}
	tail = ring_tail;
	available = ring_head - tail;
	/*
	 * Darwin keeps the ring at about a period, just in time (round 2: 0 to 960 frames, and periods cut short even so): when
	 * sound begins, or after a period had to be cut short, the ring is let fill to a cushion of two periods behind silence
	 * first, and played from then on 40 ms later -- the DSP and Darwin keep the same rate, so the cushion stays (what is
	 * left when the sound has ended, 0.2 s on, is played as it is).
	 */
	if (!primed && available != 0 && available < CUSHION_FRAMES && now_us() - last_sound_us < 200000) {
		available = 0;
	} else if (!primed && available != 0) {
		primed = true;
		primings++;
	}
	n = available < PERIOD_FRAMES ? available : PERIOD_FRAMES;
	if (n < PERIOD_FRAMES && primed) {
		if (n != 0 && guard_on) {
			short_periods++;
		}
		primed = false;
	}
	OSMemoryBarrier();
	for (uint32_t i = 0; i < n; i++) {
		samples[2 * i] = ring[2 * ((tail + i) % RING_FRAMES)];
		samples[2 * i + 1] = ring[2 * ((tail + i) % RING_FRAMES) + 1];
	}
	bzero(samples + 2 * n, (PERIOD_FRAMES - n) * 4);
	OSMemoryBarrier();
	ring_tail = tail + n;
	fed_frames += n;
	silent_frames += PERIOD_FRAMES - n;
	/* the guard, on every period, silence too: its filter and its averages go on (hakios.guard.c) */
	hakios_guard_process(samples, PERIOD_FRAMES);
	/* (whole periods while the amplifiers are on, the silence a short one was filled up with too: the recording is what is played) */
	if ((n != 0 || guard_on) && hakios_audio_recording_fed()) {
		hakios_audio_record_fed(samples, PERIOD_FRAMES);
	}
	if (AR) {
		soft_apply(samples, 2 * PERIOD_FRAMES);       /* (the level under Android's, after the guard: see soft_q15) */
	}
	guard_periods++;
	for (vm_offset_t line = (vm_offset_t)samples; line < (vm_offset_t)samples + PERIOD_BYTES; line += 64) {
		__asm__ volatile ("dc cvac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy");
}

static void
feeder_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t sent = 0, failed = 0;

	returned = 0;
	for (unsigned p = 0; p < PERIODS; p++) {
		period_fill_darwin(p);
		sent += period_write(p);
	}
	while (!stopping) {
		uint32_t back = returned;
		static bool was_on;

		if (guard_on && !was_on) {
			guard_say("", true);    /* (what it did to the silence meanwhile is of no interest) */
			guard_periods = 0;
		} else if (was_on && !guard_on) {
			guard_say(" until the amplifiers stopped", false);
		} else if (guard_on && guard_periods >= 250) {
			guard_say(", 5 s", false);
			say("sound: Darwin's, 5 s: %u periods cut short, %u frames dropped (the ring full), %u frames in the ring now, "
			    "%u starts behind a cushion\n", short_periods, dropped_frames, ring_head - ring_tail, primings);
			short_periods = 0;
			primings = 0;
			dropped_frames = 0;
			guard_periods = 0;
		}
		was_on = guard_on;
		if (back != 0) {
			OSBitAndAtomic(~back, &returned);
			for (unsigned p = 0; p < PERIODS; p++) {
				if (back & 1u << p) {
					period_fill_darwin(p);
					if (period_write(p)) {
						sent++;
					} else {
						failed++;
					}
				}
			}
		}
		sleep_ms(2);
	}
	say("sound: the feeder stops: %u periods written (%u could not be sent), %u given back (%u with an error), %u underflows; "
	    "from Darwin %llu frames came, %llu went to the DSP, %llu of silence filled in\n", sent, failed, written_done, write_errors,
	    underflows, darwin_frames, fed_frames, silent_frames);
	feeder_runs = false;
	thread_terminate(current_thread());
}

/* The amplifiers' level as the guard is to reckon with it (sound_guard_as: louder than it is, for a test). */
static void
guard_level(uint32_t level_below)
{
	uint32_t as = level_below < guard_as ? level_below : guard_as;

	/* (HAKIOS_GUARD_SILENT means "the amplifiers are off": a level must never be taken for it, it would turn the power limits off) */
	hakios_guard_attenuation(as >= HAKIOS_GUARD_SILENT ? HAKIOS_GUARD_SILENT - 1 : as);
}

/*
 * sound_below and the volume Darwin has set (`extra', eighths of a dB under it), added up: saturating, so that a
 * huge sound_below or `extra' cannot wrap the sum round to a small number, that is to a loud level.
 */
static uint32_t
below_with(uint32_t extra)
{
	uint64_t total = (uint64_t)below + extra;

	return total > HAKIOS_GUARD_SILENT - 1 ? HAKIOS_GUARD_SILENT - 1 : (uint32_t)total;
}

/*
 * The amplifiers are off. The guard goes on at their level all the same: what it lets through now reaches them
 * 160 ms later, behind the periods queued at the DSP, and if that is sound, they will have been started by then
 * (70 ms after it comes). While they are off nothing but silence comes, and the averages fall as a speaker cools.
 * Only when nothing can be heard at all -- Darwin's volume at 0, the amplifiers given up on -- is the guard told so.
 */
static void
guard_off(bool unheard)
{
	guard_on = false;
	if (unheard) {
		hakios_guard_attenuation(HAKIOS_GUARD_SILENT);
	}
}

/*
 * The amplifiers on while there is sound, at the volume Darwin has set (hakios_audio_volume(): how
 * far under the loudest, sound_below under Android's level), and off while Darwin's volume is 0;
 * their status looked at every 200 ms while they are on. On the sound's thread.
 */
static void
watch(void)
{
	bool on = false, given_up = false, muted = false;
	uint64_t last_look = 0, last_monitor = 0;
	uint32_t starts = 0, changes = 0, generation = 0, extra = 0, monitor = 0, monitor_looks = 0;

	guard_level(below);                     /* (from the first sample on: see guard_off()) */

	while (!stopping) {
		uint64_t now = now_us(), sounded = last_sound_us;
		uint32_t extra_now, generation_now;
		bool muted_now;

		if (hakios_audio_volume(&extra_now, &muted_now, &generation_now) && generation_now != generation) {
			generation = generation_now;
			extra = extra_now;
			muted = muted_now;
			changes++;
			say("sound: Darwin sets the volume: %s%u.%03u dB under the loudest, which is %u.%03u dB under Android's level%s\n",
			    muted ? "muted (0), " : "", extra / 8, extra % 8 * 125, below / 8, below % 8 * 125,
			    on ? (muted ? ": the amplifiers are stopped" : ": the amplifiers follow") : "");
			if (muted) {
				guard_off(true);
				if (on) {
					hakios_speaker_stop();
					on = false;
				}
			} else if (on && !hakios_speaker_volume(below_with(extra), 1)) {
				guard_off(true);
				on = false;
				given_up = true;
			} else if (!given_up) {
				guard_level(below_with(extra));
			}
		}
		if (!on && !given_up && !muted && sounded != 0 && now - sounded < 100000) {
			say_when("Darwin plays: the amplifiers are started");
			starts++;
			darwin_peak = 0;
			if (!hakios_speaker_start() || !hakios_speaker_volume(below_with(extra), 1)) {
				say("sound: THE AMPLIFIERS DID NOT START; Darwin's sound goes on to the DSP, unheard\n");
				guard_off(true);
				given_up = true;
			} else {
				on = true;
				guard_on = true;
				last_look = now_us();
				last_monitor = 0;
				monitor_looks = 0;
				/* (what is written now is heard after the periods already queued at the DSP: PERIODS x 20 ms) */
				say("sound: the amplifiers are up %llu ms after the first write that was not silence; that write reaches "
				    "them %u ms after it was made, behind the periods queued at the DSP\n", (last_look - sounded) / 1000,
				    PERIODS * PERIOD_FRAMES / 48);
			}
		} else if (on && now - sounded > QUIET_BEFORE_OFF_US) {
			say_when("three seconds of silence: the amplifiers are stopped");
			say("sound: what Darwin wrote while they were on peaked at %u%% of full scale (%u)\n", darwin_peak * 100 / 32768,
			    darwin_peak);
			guard_off(false);
			hakios_speaker_stop();
			on = false;
		} else if (on && now - last_look >= 200000) {
			last_look = now;
			if (!hakios_speaker_healthy()) {
				guard_off(true);
				on = false;
				given_up = true;
			} else if (now - last_monitor >= 1000000) {
				/* the amplifiers' temperature and the battery as they see it, every second: a reduction if Android's table asks for one */
				uint32_t wanted;

				last_monitor = now;
				wanted = hakios_speaker_monitor(monitor_looks++ % 10 == 0);
				if (wanted != monitor) {
					say("sound: the amplifiers' watch asks for %u.%03u dB less (it was %u.%03u)\n", wanted / 8, wanted % 8 * 125,
					    monitor / 8, monitor % 8 * 125);
					monitor = wanted;
					hakios_guard_monitor(monitor);
				}
			}
		}
		sleep_ms(5);
	}
	guard_off(true);
	say("sound: the watcher stops; the amplifiers were started %u times, Darwin set the volume %u times%s\n", starts, changes,
	    given_up ? ", and the amplifiers were then given up on" : "");
	watcher_runs = false;
}

/* ---- the whole ---------------------------------------------------------------------------------------- */

static bool running, taken_down;

static void
take_down(void)
{
	stopping = true;
	if (level >= 4) {
		hakios_audio_set_sink("nothing", NULL);
	}
	for (unsigned ms = 0; ms < 3000 && (feeder_runs || watcher_runs); ms++) {
		sleep_ms(1);
	}
	lck_mtx_lock(&sound_lock);
	if (running && !taken_down) {
		taken_down = true;
		hakios_speaker_stop();
		if (AR) {
			ar_stream_off();
			if (ar_clock_on) {
				ar_clock(false);
				ar_clock_on = false;
			}
		} else {
			stream_off();
			afe_off();
		}
		pins_off();
		lck_mtx_lock(&audio_space_lock);
		shared_put(0);
		lck_mtx_unlock(&audio_space_lock);
		hakios_adsp_apr_events(NULL);
		say("sound: everything of the sound's is stopped and put back\n");
	}
	lck_mtx_unlock(&sound_lock);
}

static void
sound_thread(__unused void *parameter, __unused wait_result_t result)
{
	const uint8_t *acf = NULL;
	uint64_t acf_size = 0;
	uint32_t changes, ones, sync_changes, sync_ones;
	bool ok = false;

	lck_mtx_lock(&sound_lock);
	if (taken_down) {
		lck_mtx_unlock(&sound_lock);
		thread_terminate(current_thread());
	}
	if (level == 4) {
		say("sound: level 4: what Darwin plays, through the port, a stream and the amplifiers, until the run ends\n");
	} else {
		say("sound: level %u: %s, for %u s\n", level, level == 1 ? "the speakers' TDM port" : level == 2 ?
		    "the port and a stream with a test tone, the amplifiers off" :
		    "the port, a stream with a test tone, and the amplifiers", seconds);
	}
	if (level >= 3) {
		const char *file = AR ? "tfa98xx.cnt" : "aw882xx_acf.bin";

		acf = hakios_adsp_fw_file(file, &acf_size);
		if (acf == NULL) {
			say("sound: no /fw/%s nor /private/var/fw/%s: put the phone's %s there (fw-audio/ and SYSTEM=1 userland/build.sh, "
			    "or sweet-sh.py --put)\n", file, file, AR ? "/odm/firmware/tfa98xx.cnt" : "/vendor/etc/aw882xx_acf.bin");
			goto out;
		}
	}
	if (level >= 2 || AR) {
		/* (the DSP's graph on SM8450 has the shared-memory end point in it at every level: our memory is mapped at level 1 too) */
		lck_mtx_lock(&audio_space_lock);
		ok = shared_get(0);
		lck_mtx_unlock(&audio_space_lock);
		if (!ok) {
			goto out;
		}
		ok = false;
		hakios_adsp_apr_events(AR ? ar_events : events);
	}
	announce(AR ? "the tertiary MI2S pins (GPIO 121 to 124) to the interface, as Android's active state has them" :
	    "the TDM pins (GPIO 97 to 100) to the interface, as Android's active state has them");
	pins_on();
	if (AR) {
		/* the bit clock (PRM), then the whole graph started: the interface's clocks run at the pins from its start */
		announce("the DSP is asked for the speakers' bit clock (PRM)");
		ar_clock_on = ar_clock(true);
		if (!ar_clock_on || !ar_stream_on()) {
			goto out;
		}
	} else if (!afe_on()) {
		goto out;
	}
	sleep_ms(20);
	pin_activity(tdm_pins[0].pin, tdm_pins[0].name, &changes, &ones);
	pin_activity(tdm_pins[1].pin, tdm_pins[1].name, &sync_changes, &sync_ones);
	say("sound: %s\n", changes > 100 && sync_changes > 4 ? "THE BIT CLOCK AND THE FRAME SYNC RUN at the pins" :
	    "THE PINS DO NOT SHOW A RUNNING BIT CLOCK");
	if (level == 1) {
		sleep_ms(seconds * 1000);
		ok = true;
		goto out;
	}
	if (!AR && !stream_on()) {
		goto out;
	}
	if (level == 4) {
		thread_t feeder;

		announce("the amplifiers, set up from Android's Music profile, powered down until Darwin plays");
		if (!hakios_speaker_prepare(acf, acf_size)) {
			goto out;
		}
		hakios_adsp_fw_free(acf, acf_size);
		acf = NULL;
		feeder_runs = true;
		if (kernel_thread_start(feeder_thread, NULL, &feeder) != KERN_SUCCESS) {
			feeder_runs = false;
			say("sound: no thread for the feeder\n");
			goto out;
		}
		thread_deallocate(feeder);
		watcher_runs = true;
		hakios_audio_set_sink("the speakers (the audio DSP, hakios.sound.c)", darwin_sink);
		say("sound: WHAT DARWIN PLAYS GOES TO THE SPEAKERS from here on, %u.%03u dB under Android's level, until the run ends\n",
		    below / 8, below % 8 * 125);
		say("sound: through the guard (hakios.guard.c): %u.%03u dB of gain first (sound_gain), then high-pass at 200 Hz; peaks "
		    "3 dB, the 0.2 s/3 s/60 s power 6/9/12 dB under full scale at Android's level\n", gain / 8, gain % 8 * 125);
		if (guard_as < below) {
			say("sound: sound_guard_as=%u: FOR A TEST the guard reckons the amplifiers %u.%03u dB under Android's level, louder "
			    "than they are, and holds back as much as it would there\n", guard_as, guard_as / 8, guard_as % 8 * 125);
		}
		lck_mtx_unlock(&sound_lock);
		watch();
		thread_terminate(current_thread());
	}
	if (level >= 3) {
		/* the clock runs (the port is started, the stream as well): the amplifiers can lock to it */
		announce("the amplifiers, set up from Android's Music profile, then started at the lowest volume");
		if (!hakios_speaker_prepare(acf, acf_size) || !hakios_speaker_start()) {
			goto out;
		}
	}
	/* (the amplifiers are faded in before the tone begins: they fade in on silence, and the tone starts at that level) */
	if (level >= 3 && !hakios_speaker_volume(below, 50)) {
		goto out;
	}
	if (level >= 3) {
		say("sound: THE TONE PLAYS NOW for %u s: 1 kHz at -20 dBFS, both speakers, %u.%03u dB under Android's level\n",
		    seconds, below / 8, below % 8 * 125);
	}
	say_when("the tone begins");
	ok = play(seconds * 1000, level >= 3);
	say_when("the tone has ended");
	space_faults();
out:
	if (acf != NULL) {
		hakios_adsp_fw_free(acf, acf_size);
	}
	lck_mtx_unlock(&sound_lock);
	take_down();
	say("sound: %s; last: %s\n", ok ? "LEVEL DONE" : "STOPPED SHORT", last_write);
	thread_terminate(current_thread());
}

void
hakios_sound_dsp_up(void)
{
	thread_t thread;

	if (!PE_parse_boot_argn("sound", &level, sizeof(level)) || level == 0) {
		return;
	}
	if (level > 4) {
		level = 4;
	}
	PE_parse_boot_argn("sound_soc", &sound_soc, sizeof(sound_soc));
	if (AR) {
		if (!hakios_adsp_is_gpr()) {
			say("sound: sound_soc=8450 is AudioReach's way, and the DSP does not speak GPR (adsp_soc=8450 is needed)\n");
			return;
		}
		tdm_pins = negroni_pins;
	}
	PE_parse_boot_argn("sound_secs", &seconds, sizeof(seconds));
	PE_parse_boot_argn("sound_below", &below, sizeof(below));
	PE_parse_boot_argn("sound_pause", &pause_ms, sizeof(pause_ms));
	PE_parse_boot_argn("sound_guard_as", &guard_as, sizeof(guard_as));
	PE_parse_boot_argn("sound_gain", &gain, sizeof(gain));
	if (gain > HAKIOS_GUARD_MAX_GAIN) {
		gain = HAKIOS_GUARD_MAX_GAIN;
	}
	hakios_guard_configure(true, gain);
	if (level >= 2 && level <= 3 && seconds > 3) {
		seconds = 3;            /* the test tone: three seconds at most */
	}
	running = true;
	if (kernel_thread_start(sound_thread, NULL, &thread) != KERN_SUCCESS) {
		running = false;
		say("sound: no thread\n");
		return;
	}
	thread_deallocate(thread);
}

void
hakios_sound_dsp_down(void)
{
	if (running) {
		take_down();
	}
}

void
hakios_sound_quiesce(void)
{
	hakios_speaker_quiesce();
}
