/*
 * Host tests of drivers/wifi/hakios.apple80211.c (the request map, P3a): the source the kernel is built from, behind a
 * fake station boundary, driven through hakios_apple80211_ioctl() as wifid drives it.
 */
#include "shim/shim.h"
#include <sys/hakios.sta.h>

errno_t hakios_apple80211_ioctl(u_long command, void *data);

#define SIOCSA80211 0x802869c8u
#define SIOCGA80211 0xc02869c9u

struct apple80211req {
	char req_if_name[16];
	int32_t req_type;
	int32_t req_val;
	uint32_t req_len;
	user_addr_t req_data __attribute__((aligned(8)));
};

static unsigned checks, failures;

#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); \
	printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- the fake station ---- */

static struct hakios_sta_state now;
static struct hakios_sta_bss heard[3];
static unsigned heard_count;

/* the events posted, in order: their codes, the name they carry, and their data */
static struct {
	uint32_t code;
	uint32_t length;
	uint8_t data[256];
	bool named;
} posted[16];
static unsigned posted_count;

int
kev_post_msg(struct kev_msg *message)
{
	if (posted_count < sizeof(posted) / sizeof(posted[0])) {
		posted[posted_count].code = message->event_code;
		posted[posted_count].named = message->dv[0].data_length == 16 && strcmp(message->dv[0].data_ptr, "en0") == 0 &&
		    message->vendor_code == KEV_VENDOR_APPLE && message->kev_class == KEV_IEEE80211_CLASS && message->kev_subclass == 1;
		posted[posted_count].length = message->dv[1].data_length;
		if (message->dv[1].data_length != 0 && message->dv[1].data_length <= sizeof(posted[0].data)) {
			memcpy(posted[posted_count].data, message->dv[1].data_ptr, message->dv[1].data_length);
		}
	}
	posted_count++;
	return 0;
}
void proc_selfname(char *name, int size) { snprintf(name, (size_t)size, "wifid"); }
bool hakios_sta_present(void) { return true; }
bool hakios_sta_powered(void) { return true; }
int hakios_sta_ask_power(bool on) { (void)on; return 0; }
int hakios_sta_ask_scan(const struct hakios_sta_scan *what) { (void)what; return 0; }
static struct hakios_sta_join asked_join;
static unsigned join_calls;
int hakios_sta_ask_join(const struct hakios_sta_join *what) { asked_join=*what;join_calls++;return 0; }
int hakios_sta_ask_leave(void) { return 0; }
int hakios_sta_ask_country(const char *code) { (void)code; return 0; }
bool hakios_sta_country(char *code) { memcpy(code, "DE", 2); return true; }
void hakios_sta_state(struct hakios_sta_state *state) { *state = now; }
unsigned hakios_sta_channels(struct hakios_sta_channel *channels, unsigned most) { (void)channels; (void)most; return 0; }

bool
hakios_sta_heard(unsigned index, struct hakios_sta_bss *bss)
{
	if (index >= heard_count) {
		return false;
	}
	*bss = heard[index];
	return true;
}

/* ---- requests ---- */

static int
ask(bool set, int32_t type, int32_t *value, void *data, uint32_t length)
{
	struct apple80211req request = { .req_if_name = "en0", .req_type = type, .req_val = value ? *value : 0,
		                         .req_len = length, .req_data = (user_addr_t)(uintptr_t)data };
	int error = hakios_apple80211_ioctl(set ? SIOCSA80211 : SIOCGA80211, &request);

	if (value != NULL) {
		*value = request.req_val;
	}
	return error;
}

static int
get_value(int32_t type, int32_t *value)
{
	*value = -12345;
	return ask(false, type, value, NULL, 0);
}

/* ---- networks ---- */

static void
add(struct hakios_sta_bss *bss, const uint8_t *element, unsigned length)
{
	memcpy(bss->elements + bss->elements_length, element, length);
	bss->elements_length = (uint16_t)(bss->elements_length + length);
}

static const uint8_t rates_ofdm[] = { 1, 8, 0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c };     /* 6 ... 54 */
static const uint8_t rates_b[] = { 1, 4, 0x82, 0x84, 0x8b, 0x96 };                               /* 1, 2, 5.5, 11 */

static void
ht_cap(struct hakios_sta_bss *bss, bool two)
{
	uint8_t e[28] = { 45, 26 };

	e[2 + 3] = 0xff;                        /* rx_mask[0] */
	e[2 + 4] = two ? 0xff : 0;              /* rx_mask[1] */
	add(bss, e, sizeof(e));
}

static void
ht_op(struct hakios_sta_bss *bss, uint8_t info)
{
	uint8_t e[24] = { 61, 22, 36, info };

	add(bss, e, sizeof(e));
}

static void
vht(struct hakios_sta_bss *bss, uint16_t map, uint8_t width)
{
	uint8_t cap[14] = { 191, 12 }, op[7] = { 192, 5, width, 42 };

	cap[2 + 4] = (uint8_t)map;
	cap[2 + 5] = (uint8_t)(map >> 8);
	add(bss, cap, sizeof(cap));
	add(bss, op, sizeof(op));
}

/* VHT operation of 160 MHz: width 1, CCFS0 the 80 with the primary (42), CCFS1 the 160's center (50) */
static void
vht160(struct hakios_sta_bss *bss, uint16_t map)
{
	uint8_t cap[14] = { 191, 12 }, op[7] = { 192, 5, 1, 42, 50 };

	cap[2] = 0x04;                          /* (supported widths: 160) */
	cap[2 + 4] = (uint8_t)map;
	cap[2 + 5] = (uint8_t)(map >> 8);
	add(bss, cap, sizeof(cap));
	add(bss, op, sizeof(op));
}

/* HE capabilities (extension 35): PHY B0 as given (0x08: 160 MHz, with its map after the <= 80 one) */
static void
he_cap(struct hakios_sta_bss *bss, uint8_t phy0, uint16_t map80, uint16_t map160)
{
	uint8_t e[2 + 1 + 17 + 8] = { 255, 0, 35 };
	unsigned maps = phy0 & 0x08 ? 8 : 4;

	e[1] = (uint8_t)(1 + 17 + maps);
	e[3 + 6] = phy0;
	e[20] = (uint8_t)map80;                 /* rx */
	e[21] = (uint8_t)(map80 >> 8);
	e[22] = (uint8_t)map80;                 /* tx */
	e[23] = (uint8_t)(map80 >> 8);
	if (maps == 8) {
		e[24] = (uint8_t)map160;
		e[25] = (uint8_t)(map160 >> 8);
		e[26] = (uint8_t)map160;
		e[27] = (uint8_t)(map160 >> 8);
	}
	add(bss, e, 2 + e[1]);
}

static void
network(uint16_t frequency)
{
	bzero(&now, sizeof(now));
	now.phase = HAKIOS_STA_JOINED;
	now.bss.frequency = frequency;
	now.bss.rssi = -60;
	memcpy(now.bss.bssid, "\x02\x11\x22\x33\x44\x55", 6);
}

static void
expect_link(const char *what, uint32_t mode, int32_t rate, int32_t mcs, int32_t streams)
{
	uint32_t phy[3] = { 0 };
	int32_t value;
	int error;

	CHECK(ask(false, 14, NULL, phy, sizeof(phy)) == 0 && phy[0] == 1 && phy[1] == 0x19e && phy[2] == mode,
	    "%s: PHY_MODE {%u, %#x, %#x}, want active %#x", what, phy[0], phy[1], phy[2], mode);
	CHECK(get_value(8, &value) == 0 && value == rate, "%s: RATE %d, want %d", what, value, rate);
	error = get_value(57, &value);
	if (mcs < 0) {
		CHECK(error == ENXIO, "%s: MCS of a legacy link: error %d, want ENXIO", what, error);
	} else {
		CHECK(error == 0 && value == mcs, "%s: MCS %d (error %d), want %d", what, value, error, mcs);
	}
	CHECK(get_value(353, &value) == 0 && value == streams, "%s: NSS %d, want %d", what, value, streams);
}

/* ---- events (P3b) ---- */

static int32_t
word(const uint8_t *data, unsigned at)
{
	int32_t value;

	memcpy(&value, data + at, sizeof(value));
	return value;
}

static int16_t
half(const uint8_t *data, unsigned at)
{
	int16_t value;

	memcpy(&value, data + at, sizeof(value));
	return value;
}

/* the codes posted since the last call, as a string of numbers ("2 3 4 39") */
static const char *
codes(void)
{
	static char line[128];

	line[0] = 0;
	for (unsigned i = 0; i < posted_count && i < sizeof(posted) / sizeof(posted[0]); i++) {
		CHECK(posted[i].named, "event %u does not carry en0's name", posted[i].code);
		snprintf(line + strlen(line), sizeof(line) - strlen(line), "%s%u", i ? " " : "", posted[i].code);
	}
	return line;
}

static unsigned
find(uint32_t code)
{
	for (unsigned i = 0; i < posted_count; i++) {
		if (posted[i].code == code) {
			return i;
		}
	}
	return ~0u;
}

static void
signal_says(int dbm, bool sent, const char *what)
{
	unsigned at;

	posted_count = 0;
	hakios_apple80211_event(HAKIOS_STA_SIGNAL, dbm);
	at = find(39);
	CHECK(sent ? posted_count == 1 && at == 0 && word(posted[0].data, 4) == dbm : posted_count == 0,
	    "%s: %d dBm: posted \"%s\", want %s", what, dbm, codes(), sent ? "39" : "nothing");
}

static void
events(void)
{
	unsigned at;

	/* joined: the name, the access point, the link up with its signal, then the first RSSI_CHANGED at once */
	network(5180);
	posted_count = 0;
	hakios_apple80211_event(HAKIOS_STA_LINK, 1);
	CHECK(strcmp(codes(), "2 3 4 39") == 0, "joined: posted \"%s\"", codes());
	at = find(4);
	CHECK(at != ~0u && posted[at].length == 32 && posted[at].data[0] == 0 && word(posted[at].data, 4) == -60 &&
	    half(posted[at].data, 8) == 35 && half(posted[at].data, 10) == -95 && posted[at].data[0x10] == 0 &&
	    word(posted[at].data, 0x14) == 0, "joined: LINK_CHANGED");
	at = find(39);
	CHECK(at != ~0u && posted[at].length == 180 && posted[at].data[0] == 1 && word(posted[at].data, 4) == -60 &&
	    posted[at].data[8] == 0 && posted[at].data[0xb] == 1 && half(posted[at].data, 0xc) == 35 &&
	    posted[at].data[0xe] == 1 && half(posted[at].data, 0x10) == -95 && posted[at].data[0x12] == 0 &&
	    posted[at].data[0x30] == 0 && posted[at].data[0x31] == 0 && word(posted[at].data, 0x1c) == 0,
	    "joined: RSSI_CHANGED");

	/* the signal: by 3 dB, across the threshold, and every 5 s at least */
	signal_says(-61, false, "1 dB");
	signal_says(-62, false, "2 dB");
	signal_says(-63, true, "3 dB");
	signal_says(-63, false, "quiet 1");
	signal_says(-63, false, "quiet 2");
	signal_says(-62, false, "quiet 3");
	signal_says(-64, false, "quiet 4");
	signal_says(-64, true, "quiet 5");
	signal_says(-79, true, "-15 dB");
	signal_says(-80, false, "at the threshold (1 dB)");
	signal_says(-81, true, "below the threshold (2 dB)");
	signal_says(-79, true, "back above the threshold (2 dB)");

	/* the country */
	posted_count = 0;
	hakios_apple80211_event(HAKIOS_STA_COUNTRY, 0);
	CHECK(strcmp(codes(), "11") == 0 && posted[0].length == 0, "country: posted \"%s\"", codes());

	/* lost: the access point deauthenticates us with reason 7 */
	now.phase = HAKIOS_STA_IDLE;
	now.last_error = ECONNRESET;
	now.loss = HAKIOS_STA_LOSS_DEAUTH;
	now.reason = 7;
	posted_count = 0;
	hakios_apple80211_event(HAKIOS_STA_LINK, ECONNRESET);
	CHECK(strcmp(codes(), "34 2 3 4") == 0, "deauthenticated: posted \"%s\"", codes());
	CHECK(posted[0].length == 8 && word(posted[0].data, 0) == 0 && word(posted[0].data, 4) == 7, "DEAUTH_RECEIVED");
	at = find(4);
	CHECK(at != ~0u && posted[at].data[0] == 1 && posted[at].data[0x10] == 1 && word(posted[at].data, 0x14) == 7 &&
	    word(posted[at].data, 4) == 0, "deauthenticated: LINK_CHANGED down, involuntary, reason %d", word(posted[at].data, 0x14));
	{
		struct {
			uint32_t version, reason;
			uint8_t address[6], unused[2];
		} deauth;

		CHECK(ask(false, 29, NULL, &deauth, sizeof(deauth)) == 0 && deauth.reason == 7, "DEAUTH after one: reason %u",
		    deauth.reason);
	}
	signal_says(-60, false, "after the loss");

	/* ... disassociates us with reason 8 */
	now.loss = HAKIOS_STA_LOSS_DISASSOC;
	now.reason = 8;
	posted_count = 0;
	hakios_apple80211_event(HAKIOS_STA_LINK, ECONNRESET);
	CHECK(strcmp(codes(), "72 2 3 4") == 0 && word(posted[0].data, 4) == 8 && word(posted[3].data, 0x14) == 8,
	    "disassociated: posted \"%s\"", codes());

	/* ... its beacons stop: inactivity, no frame to tell of */
	now.loss = HAKIOS_STA_LOSS_BEACONS;
	now.reason = 0;
	now.last_error = ETIMEDOUT;
	posted_count = 0;
	hakios_apple80211_event(HAKIOS_STA_LINK, ETIMEDOUT);
	CHECK(strcmp(codes(), "2 3 4") == 0 && posted[2].data[0x10] == 1 && word(posted[2].data, 0x14) == 4,
	    "beacons lost: posted \"%s\", reason %d", codes(), word(posted[2].data, 0x14));

	/* left when asked: down, not involuntary, no reason; no signal after */
	network(5180);
	hakios_apple80211_event(HAKIOS_STA_LINK, 1);
	now.phase = HAKIOS_STA_IDLE;
	now.loss = HAKIOS_STA_LOSS_NONE;
	posted_count = 0;
	hakios_apple80211_event(HAKIOS_STA_LINK, 0);
	CHECK(strcmp(codes(), "2 3 4") == 0 && posted[2].data[0] == 1 && posted[2].data[0x10] == 0 &&
	    word(posted[2].data, 0x14) == 0, "left: posted \"%s\"", codes());
	signal_says(-60, false, "after leaving");

	/* scans, joins, power: no data */
	posted_count = 0;
	hakios_apple80211_event(HAKIOS_STA_SCAN_DONE, 0);
	hakios_apple80211_event(HAKIOS_STA_JOIN_DONE, 0);
	hakios_apple80211_event(HAKIOS_STA_POWER, 1);
	CHECK(strcmp(codes(), "10 9 1") == 0 && posted[0].length == 0 && posted[1].length == 0 && posted[2].length == 0,
	    "scan, join, power: posted \"%s\"", codes());
}

int
main(void)
{
	int32_t value;
	int error;

	/* not joined */
	bzero(&now, sizeof(now));
	now.phase = HAKIOS_STA_IDLE;
	{
		uint32_t phy[3] = { 7, 7, 7 }, prot[3] = { 7, 7, 7 };
		uint8_t deauth[16];

		CHECK(ask(false, 14, NULL, phy, sizeof(phy)) == 0 && phy[0] == 1 && phy[1] == 0x19e && phy[2] == 0,
		    "PHY_MODE not joined {%u, %#x, %#x}", phy[0], phy[1], phy[2]);
		CHECK(get_value(8, &value) == ENXIO, "RATE not joined");
		CHECK(get_value(57, &value) == ENXIO, "MCS not joined");
		CHECK(get_value(353, &value) == ENXIO, "NSS not joined");
		CHECK(get_value(5, &value) == 0 && value == 0, "POWERSAVE %d", value);
		CHECK(get_value(18, &value) == 0 && value == 0, "INT_MIT %d", value);
		CHECK(ask(false, 6, NULL, prot, sizeof(prot)) == 0 && prot[0] == 1 && prot[1] == 0 && prot[2] == 0, "PROTMODE");
		memset(deauth, 0xaa, sizeof(deauth));
		CHECK(ask(false, 29, NULL, deauth, sizeof(deauth)) == 0 && deauth[0] == 1 && deauth[4] == 0 && deauth[8] == 0,
		    "DEAUTH with nothing lost");
		{
			int32_t noise[13];

			CHECK(ask(false, 17, NULL, noise, sizeof(noise)) == ENXIO, "NOISE not joined");
		}
	}

	/* lost: DEAUTH says it, and from whom */
	now.phase = HAKIOS_STA_IDLE;
	now.last_error = ETIMEDOUT;
	memcpy(now.bss.bssid, "\x02\x11\x22\x33\x44\x55", 6);
	{
		struct {
			uint32_t version, reason;
			uint8_t address[6], unused[2];
		} deauth;

		CHECK(ask(false, 29, NULL, &deauth, sizeof(deauth)) == 0 && deauth.version == 1 && deauth.reason == 4 &&
		    memcmp(deauth.address, "\x02\x11\x22\x33\x44\x55", 6) == 0, "DEAUTH after a loss: reason %u", deauth.reason);
	}

	/* VHT, 80 MHz, two streams up to MCS 9 (5 GHz) */
	network(5180);
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	ht_cap(&now.bss, true);
	ht_op(&now.bss, 0x05);
	vht(&now.bss, 0xfffa, 1);
	expect_link("VHT80 2x2", 0x80, 780, 9, 2);

	/* the owner's access point (P1c): 20 MHz (no "any width" bit), VHT 2x2 -> MCS 8 at most at 20 MHz */
	network(5180);
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	ht_cap(&now.bss, true);
	ht_op(&now.bss, 0x08);
	vht(&now.bss, 0xfffa, 0);
	expect_link("VHT20 2x2", 0x80, 156, 8, 2);

	/* VHT with only one stream in its map, up to MCS 7 */
	network(5180);
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	ht_cap(&now.bss, true);
	ht_op(&now.bss, 0x05);
	vht(&now.bss, 0xfffc, 1);
	expect_link("VHT80 1x1 MCS 7", 0x80, 292, 7, 1);

	/* VHT 160 MHz (CCFS1 8 channels from CCFS0), two streams up to MCS 9 */
	network(5180);
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	ht_cap(&now.bss, true);
	ht_op(&now.bss, 0x05);
	vht160(&now.bss, 0xfffa);
	expect_link("VHT160 2x2", 0x80, 1560, 9, 2);

	/* HE (Wi-Fi 6): 80 MHz 2x2 up to MCS 11; 160 MHz with its own map (up to MCS 9); 2.4 GHz 20 MHz 2x2 MCS 11 */
	network(5180);
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	ht_cap(&now.bss, true);
	ht_op(&now.bss, 0x05);
	vht(&now.bss, 0xfffa, 1);
	he_cap(&now.bss, 0x04, 0xfffa, 0);
	expect_link("HE80 2x2", 0x100, 1201, 11, 2);
	network(5180);
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	ht_cap(&now.bss, true);
	ht_op(&now.bss, 0x05);
	vht160(&now.bss, 0xfffa);
	he_cap(&now.bss, 0x0c, 0xfffa, 0xfff5);
	expect_link("HE160 2x2 MCS 9", 0x100, 1921, 9, 2);
	network(2412);
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	ht_cap(&now.bss, true);
	ht_op(&now.bss, 0x00);
	he_cap(&now.bss, 0x02, 0xfffa, 0);
	expect_link("HE20 2x2 2.4 GHz", 0x100, 286, 11, 2);

	/* HT on 2.4 GHz: one stream at 20 MHz, two streams at 40 MHz (HT counts the streams into the index) */
	network(2412);
	add(&now.bss, rates_b, sizeof(rates_b));
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	ht_cap(&now.bss, false);
	ht_op(&now.bss, 0x00);
	vht(&now.bss, 0xfffa, 1);               /* (VHT on 2.4 GHz is not used) */
	expect_link("HT20 1x1", 0x10, 65, 7, 1);
	network(2412);
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	ht_cap(&now.bss, true);
	ht_op(&now.bss, 0x05);
	expect_link("HT40 2x2", 0x10, 270, 15, 2);

	/* legacy: 802.11b only, 802.11g, 802.11a */
	network(2437);
	add(&now.bss, rates_b, sizeof(rates_b));
	expect_link("11b", 0x4, 11, -1, 1);
	network(2437);
	add(&now.bss, rates_b, sizeof(rates_b));
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	expect_link("11g", 0x8, 54, -1, 1);
	network(5500);
	add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
	expect_link("11a", 0x2, 54, -1, 1);

	/* a truncated element list: no reading past it */
	network(5180);
	{
		uint8_t broken[] = { 45, 26, 0, 0 };

		add(&now.bss, rates_ofdm, sizeof(rates_ofdm));
		add(&now.bss, broken, sizeof(broken));
	}
	expect_link("broken HT element", 0x2, 54, -1, 1);

	/* NOISE when joined */
	{
		int32_t noise[13];

		CHECK(ask(false, 17, NULL, noise, sizeof(noise)) == 0 && noise[0] == 1 && noise[1] == 1 && noise[7] == -95 &&
		    noise[3] == -95 && noise[12] == -95, "NOISE joined");
	}

	/* a short buffer gets only what fits */
	{
		uint8_t phy[6];

		memset(phy, 0xee, sizeof(phy));
		CHECK(ask(false, 14, NULL, phy, 4) == 0 && phy[0] == 1 && phy[4] == 0xee, "PHY_MODE into 4 bytes");
	}

	/* the beacon cache: one network a request, EIO after the last, then from the first again */
	heard_count = 3;
	for (unsigned i = 0; i < heard_count; i++) {
		bzero(&heard[i], sizeof(heard[i]));
		heard[i].frequency = 2412;
		heard[i].ssid_length = 3;
		heard[i].ssid[0] = 'a' + (char)i;
	}
	for (unsigned pass = 0; pass < 2; pass++) {
		uint8_t result[1164];

		for (unsigned i = 0; i < heard_count; i++) {
			memset(result, 0, sizeof(result));
			result[0x8a] = 0x00;
			result[0x8b] = 0x04;    /* (elements_length: the room, as IO80211 sets it) */
			error = ask(false, 390, NULL, result, sizeof(result));
			CHECK(error == 0 && result[0x61] == 'a' + i, "pass %u, network %u: error %d, name %c", pass, i, error,
			    result[0x61]);
		}
		CHECK(ask(false, 390, NULL, result, sizeof(result)) == EIO, "pass %u ends in EIO", pass);
	}
	/* ... and does not move SCAN_RESULT's place */
	{
		uint8_t result[1164] = { 0 };

		CHECK(ask(false, 11, NULL, result, sizeof(result)) == 0 && result[0x61] == 'a', "SCAN_RESULT starts at the first");
	}

	/* SET_PROPERTY taken */
	{
		static char plist[4104];
		const char *text = "<?xml version=\"1.0\"?><plist><dict><key>IO80211InterfaceHeartBeat</key><true/></dict></plist>";

		memcpy(plist + 8, text, strlen(text));
		CHECK(ask(true, 383, NULL, plist, sizeof(plist)) == 0, "SET_PROPERTY taken");
	}

	/* refused the Broadcom way: EOPNOTSUPP */
	{
		static uint8_t room[4104];
		static const int32_t gets[] = { 96, 97, 115, 346, 386, 387, 423, 7, 47, 49, 79, 80, 107 };
		static const int32_t sets[] = { 25, 422, 47, 86, 90, 94, 95, 382 };

		for (unsigned i = 0; i < sizeof(gets) / sizeof(gets[0]); i++) {
			CHECK(ask(false, gets[i], NULL, room, sizeof(room)) == EOPNOTSUPP, "get %d not refused", gets[i]);
		}
		for (unsigned i = 0; i < sizeof(sets) / sizeof(sets[0]); i++) {
			CHECK(ask(true, sets[i], NULL, room, sizeof(room)) == EOPNOTSUPP, "set %d not refused", sets[i]);
		}
	}

	/* WPA3-only requests preserve their policy; SAE needs the password itself.
	 * Transition requests remain automatic and select SAE from the AP RSN. */
	{
		uint8_t assoc[592]={0};uint32_t n=4,cipher=10;uint16_t auth=0x1000;
		memcpy(assoc+8,&auth,2);memcpy(assoc+12,&n,4);memcpy(assoc+16,"test",4);
		n=3;memcpy(assoc+60,&n,4);memcpy(assoc+64,&cipher,4);memcpy(assoc+72,"sae",3);
		CHECK(ask(true,20,NULL,assoc,sizeof(assoc))==0 && asked_join.security==HAKIOS_STA_WPA3_SAE && asked_join.passphrase_length==3,
		    "WPA3-only password and policy cross the station boundary");
		auth=0x1008;memcpy(assoc+8,&auth,2);
		CHECK(ask(true,20,NULL,assoc,sizeof(assoc))==0 && asked_join.security==HAKIOS_STA_WPA2_PSK,"transition request remains automatic");
		n=32;cipher=6;memcpy(assoc+60,&n,4);memcpy(assoc+64,&cipher,4);unsigned before=join_calls;
		CHECK(ask(true,20,NULL,assoc,sizeof(assoc))==EINVAL && join_calls==before,"WPA3 refuses WPA2-derived PMK");
		now.phase=HAKIOS_STA_JOINED;now.security=HAKIOS_STA_WPA3_SAE;uint32_t answer[4]={0};
		CHECK(ask(false,2,NULL,answer,sizeof(answer))==0 && answer[2]==0x1000,"GET AUTH_TYPE reports negotiated SAE");
		now.security=HAKIOS_STA_WPA2_PSK;
		CHECK(ask(false,2,NULL,answer,sizeof(answer))==0 && answer[2]==8,"GET AUTH_TYPE retains WPA2");
	}

    { /* OWE uses the existing passwordless request and reports Enhanced Open as open. */
        uint8_t assoc[592]={0};uint32_t n=4;
        memcpy(assoc+12,&n,4);memcpy(assoc+16,"test",4);
        CHECK(ask(true,20,NULL,assoc,sizeof(assoc))==0 && asked_join.security==HAKIOS_STA_OPEN &&
            !asked_join.passphrase_length,"passwordless OWE request crosses the station boundary");
        now.phase=HAKIOS_STA_JOINED;now.security=HAKIOS_STA_OWE;uint32_t answer[4]={0};
        CHECK(ask(false,2,NULL,answer,sizeof(answer))==0 && answer[2]==0,"OWE reports passwordless upper authentication");
    }
	events();

	printf("apple80211-test: %u checks, %u failed\n", checks, failures);
	return failures != 0;
}
