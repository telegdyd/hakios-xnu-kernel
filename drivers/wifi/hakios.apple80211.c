/*
 * Darwin-on-hakios: Apple80211, answered in the kernel.
 *
 * Darwin's Wi-Fi side -- wifid, and through it Settings and the status bar -- talks to a Wi-Fi
 * driver in requests that were once ioctls and are, where the driver offers no user client,
 * ioctls still: SIOCGA80211 and SIOCSA80211 on any socket, with a struct apple80211req that
 * names the interface, a request type, a value, and a buffer in the caller's memory
 * (IO80211.framework: Apple80211Get/Set and their kin all end there). XNU hands an interface
 * ioctl it does not know to the interface, so they arrive in drivers/net/hakios.net.c, which
 * passes them here when its link is a radio.
 *
 * What is asked is answered from the station boundary (sys/hakios.sta.h): the radio's power,
 * its channels, a scan and what it heard, the network joined. What happens there by itself is
 * told to Darwin the way a driver tells it: kernel events of class 6 (IEEE 802.11), subclass 1,
 * whose code is Apple80211's own event number and whose data begins with the interface's name
 * in 16 bytes (IO80211.framework reads them from a PF_SYSTEM socket and compares the name).
 *
 * Every request is still written to the log -- who asks, get or set, the type, the value, the
 * length -- because which of them wifid needs is found that way. Never what a request carries:
 * a network's name or its key may be in there.
 *
 * The request types and the layouts are Apple's. The old ones are as the headers of its open
 * IO80211 sources once described them (apple80211_ioctl.h, apple80211_var.h); where Darwin userland 16.5
 * differs the layout was read out of IO80211.framework of 16.5, which fills and reads the same
 * structures on the other side (Apple80211GetWithIOCTL, _getScanData, _addScanResultToList,
 * _performAssociation, _dispatchAsyncAssocEvent, _dispatchLinkChangedEvent).
 *
 * A join: ASSOCIATE (20) carries the network's name, perhaps its access point, and the
 * passphrase as typed (the framework does not make the key of it), and is answered at once;
 * when the station has joined or failed, event 9 says so, and ASSOCIATE_RESULT (21) gives the
 * outcome as one of Apple80211's result numbers, which the framework turns into its errors:
 * 1 success, 21 "invalid PMK" (a wrong passphrase), 9 a timeout, 2 anything else.
 *
 * How a refusal reads: Apple80211GetWithIOCTL (IO80211.framework 16.5) turns both EOPNOTSUPP (102) and ENXIO (6)
 * into the same "unsupported" error (-3903), so either is a Broadcom driver's "not here"; none of the refusals
 * below is retried by wifid (counted over every saved run). ENXIO is kept for "no network joined".
 *
 * The request map (P3a): every request seen on the phone (device-info/.../wifi-runs, all runs to 2026-11-13),
 * plus the ones answered without being seen. Layouts: [F] read in IO80211.framework 16.5 (GetWithIOCTL's case for
 * the type), [H] apple80211_var.h, [O] observed on the phone. Who: w wifid, c configd.
 *
 *   type                       who  answer                                                         layout
 *   GET  1 SSID                w c  the name joined; ENXIO when not joined                         [H]
 *   GET  2 AUTH_TYPE           w c  {1, 1 open, 8 WPA2-PSK, 0}; ENXIO                               [H]
 *   GET  4 CHANNEL             w    the channel joined; ENXIO                                       [H]
 *   GET  5 POWERSAVE           w    value 0: off (the station has no power save yet)                [F] value
 *   GET  6 PROTMODE            w    {1, 0 off, 0 threshold}                                          [F] 12 bytes
 *   GET  8 RATE                w    value: Mbit/s the link was set up for (not measured); ENXIO      [F] value
 *   GET  9 BSSID               w c  the access point joined; ENXIO                                   [H]
 *   GET 11 SCAN_RESULT         w    one network of the last scan a request; EIO after the last       [O]
 *   GET 12 CARD_CAPABILITIES   w c  the capability bits                                              [O]
 *   GET 13 STATE               w    INIT/SCAN/AUTH/RUN from the station's phase                      [H]
 *   GET 14 PHY_MODE            w    {1, supported a/b/g/n/ac, active: the link's (0 not joined)}     [F] 12 bytes
 *   GET 15 OP_MODE             w    value 1: a station                                               [H]
 *   GET 16 RSSI                w    the network joined, dBm; ENXIO                                   [F] 52 bytes
 *   GET 17 NOISE               w    -95 dBm (ath11k's own noise floor); ENXIO                         [F] RSSI's layout
 *   GET 18 INT_MIT             w    value 0: off                                                     [F] value
 *   GET 19 POWER               w    {1, 1 radio, on/off}                                             [O]
 *   GET 21 ASSOCIATE_RESULT    w    value: 0 running, 1/21/9/2 at the end                            [O]
 *   GET 27, 254 channels       w    the radio's channels; ENXIO before it is up                      [O]
 *   GET 29 DEAUTH              w    {1, reason of the last loss (0: none), the access point}         [F] 16 bytes
 *   GET 43, 44 names           -    the driver's name (not seen asked)                               [H]
 *   GET 51 COUNTRY_CODE        w    the country in force; ENXIO while there is none                  [O]
 *   GET 57 MCS                 w    value: the link's highest MCS (not measured); ENXIO if none      [F] value
 *   GET 96, 97 VIRTUAL_IF_*    w    EOPNOTSUPP: no virtual interfaces (asked once at open)           [F]
 *   GET 103 CURRENT_NETWORK    w    the network joined as a scan result; ENXIO                       [O]
 *   GET 106 RSSI_BOUNDS        w    {1, -90, -40, -80}                                                [O]
 *   GET 115 POWER_DEBUG_INFO   w    EOPNOTSUPP: 1288 bytes of the chip's power counters, none here   [F]
 *   GET 207 CHANNELS_INFO      w    the channels' rows; ENXIO before the radio is up                 [O]
 *   GET 346 BEACON_INFO        w    EOPNOTSUPP: 1032 bytes of beacon statistics, none kept           [F]
 *   GET 353 NSS                w    value: the link's spatial streams; ENXIO                          [F] value
 *   GET 386 LQM_SUMMARY        w    EOPNOTSUPP: 4000 bytes of link-quality counters, none kept       [F]
 *   GET 387 COLOCATED_..._ID   c    EOPNOTSUPP: no 6 GHz co-located networks                         [F] 37 bytes
 *   GET 390 BEACON_SCAN_CACHE  w    the last scan's networks, one a request, EIO after the last;     [F] scan result,
 *                                   read in a loop like 11 (its own place in the list)               a loop
 *   GET 423 COUNTRY_BAND_SUP.  w    EOPNOTSUPP (not in GetWithIOCTL; layout unknown)
 *   SET 10 SCAN_REQ            w    a scan                                                           [O]
 *   SET 19 POWER               w    the radio on or off                                              [O]
 *   SET 20 ASSOCIATE           w    a join                                                           [O]
 *   SET 22 DISASSOCIATE        w    leave                                                            [O]
 *   SET 25 HOST_AP_MODE        w    EOPNOTSUPP: no access point mode (P8)
 *   SET 51 COUNTRY_CODE        w    the radio into that country                                      [O]
 *   SET 365 DISPLAY_STATE      w    taken: the station has no power save to change                   value
 *   SET 383 SET_PROPERTY       w    taken: hints (ExternelPowerState, HeartBeat, IPConfigurationBusy,
 *                                   WiFiPrimary, IPV4Params, POWER_PROFILE, FastlaneProfileState,
 *                                   MIMOPowerSaveParams, ApInfoParams, IsMovingNetwork, RoamParams, ...)
 *                                   for firmware tuning this driver does not do; the key is logged
 *   SET 422 6G_MODE            w    EOPNOTSUPP: no 6 GHz
 *
 * Not seen yet, refused (EOPNOTSUPP) until a run shows them: 7 TXPOWER, 28 LOCALE, 47 BACKGROUND_SCAN,
 * 49/79 STATS, 80 ROAM_THRESH, 86 SCAN_REQ_MULTIPLE, 87 BTCOEX_MODE, 90 SCANCACHE_CLEAR, 94/95
 * VIRTUAL_IF_CREATE/DELETE, 107 ROAM, 382 ABORT_SCAN.
 */
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <sys/errno.h>
#include <sys/kern_event.h>
#include <sys/kernel.h>
#include <sys/proc.h>
#include <sys/hakios.input.h>
#include <sys/hakios.net.h>
#include <sys/hakios.sta.h>
#include <sys/sockio.h>
#include <sys/systm.h>

#define SIOCSA80211             0x802869c8u     /* _IOW('i', 200, struct apple80211req) */
#define SIOCGA80211             0xc02869c9u     /* _IOWR('i', 201, struct apple80211req) */

struct apple80211req {
	char req_if_name[16];
	int32_t req_type;
	int32_t req_val;
	uint32_t req_len;
	user_addr_t req_data __attribute__((aligned(8)));
};

/* (as far as they have come by: the names help reading the log; the newer ones from IO80211's kIoctlNameTable) */
static const char *
type_name(int32_t type)
{
	static const struct {
		int32_t type;
		const char *name;
	} names[] = {
		{ 1, "SSID" }, { 2, "AUTH_TYPE" }, { 3, "CIPHER_KEY" }, { 4, "CHANNEL" }, { 5, "POWERSAVE" }, { 6, "PROTMODE" },
		{ 7, "TXPOWER" }, { 8, "RATE" }, { 9, "BSSID" }, { 10, "SCAN_REQ" }, { 11, "SCAN_RESULT" }, { 12, "CARD_CAPABILITIES" },
		{ 13, "STATE" }, { 14, "PHY_MODE" }, { 15, "OP_MODE" }, { 16, "RSSI" }, { 17, "NOISE" }, { 18, "INT_MIT" }, { 19, "POWER" },
		{ 20, "ASSOCIATE" }, { 21, "ASSOCIATE_RESULT" }, { 22, "DISASSOCIATE" }, { 23, "STATUS_DEV_NAME" }, { 24, "IBSS_MODE" },
		{ 25, "HOST_AP_MODE" }, { 26, "AP_MODE" }, { 27, "SUPPORTED_CHANNELS" }, { 28, "LOCALE" }, { 29, "DEAUTH" },
		{ 30, "COUNTERMEASURES" }, { 32, "RATE_SET" }, { 42, "STATION_LIST" }, { 43, "DRIVER_VERSION" },
		{ 44, "HARDWARE_VERSION" }, { 46, "RSN_IE" }, { 47, "BACKGROUND_SCAN" }, { 48, "AP_IE_LIST" }, { 49, "STATS" },
		{ 50, "ASSOCIATION_STATUS" }, { 51, "COUNTRY_CODE" }, { 54, "RADIO_INFO" }, { 57, "MCS" }, { 66, "MCS_INDEX_SET" },
		{ 79, "STA_STATS" }, { 80, "ROAM_THRESH" }, { 85, "IE" }, { 86, "SCAN_REQ_MULTIPLE" }, { 87, "BTCOEX_MODE" },
		{ 90, "SCANCACHE_CLEAR" }, { 94, "VIRTUAL_IF_CREATE" }, { 95, "VIRTUAL_IF_DELETE" }, { 103, "CURRENT_NETWORK" },
		{ 96, "VIRTUAL_IF_ROLE" }, { 97, "VIRTUAL_IF_PARENT" }, { 106, "RSSI_BOUNDS" }, { 107, "ROAM" },
		{ 115, "POWER_DEBUG_INFO" }, { 207, "CHANNELS_INFO" }, { 254, "HW_SUPPORTED_CHANNELS" }, { 267, "SUPPRESS_SCANS" },
		{ 346, "BEACON_INFO" }, { 353, "NSS" }, { 365, "DISPLAY_STATE" }, { 382, "ABORT_SCAN" }, { 383, "SET_PROPERTY" },
		{ 386, "LQM_SUMMARY" }, { 387, "COLOCATED_NETWORK_SCOPE_ID" }, { 390, "BEACON_SCAN_CACHE_REQ" },
		{ 419, "LTE_RESTRICTED_CHANNELS" }, { 422, "6G_MODE" }, { 423, "COUNTRY_BAND_SUPPORT" },
	};

	for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
		if (names[i].type == type) {
			return names[i].name;
		}
	}
	return "?";
}

/* ---- the layouts ------------------------------------------------------------------------------------------ */

#define CHANNEL_20MHZ           0x2u
#define CHANNEL_2GHZ            0x8u
#define CHANNEL_5GHZ            0x10u
#define CHANNEL_ACTIVE          0x80u           /* the radio may send there */
#define CHANNEL_DFS             0x100u

struct apple80211_channel {
	uint32_t version, channel, flags;
};

/* SUPPORTED_CHANNELS (27) and HW_SUPPORTED_CHANNELS (254): 4824 bytes */
struct apple80211_channels {
	uint32_t version, count;
	struct apple80211_channel channels[401];
	uint32_t unused;
};

/* CHANNELS_INFO (207): 10016 bytes; of each channel a byte in each of several rows, and a word of flags */
struct apple80211_channels_info {
	uint8_t unknown_0[12];
	uint16_t count;
	uint8_t unknown_1[800];
	uint8_t number[400], indoor[400], dfs[400], passive[400], wide_40[400], wide_80[400];
	uint8_t unknown_2[1602];
	uint32_t flags[400];
	uint8_t unknown_3[3600];
} __attribute__((packed));

/* SCAN_REQ (10): 5456 bytes */
struct apple80211_scan_request {
	uint32_t version, bss_type;
	uint8_t bssid[6], unused_0[2];
	uint32_t ssid_length;
	uint8_t ssid[32];
	uint32_t scan_type;                     /* 1: asking, 2: listening only */
	uint32_t phy_mode;
	uint16_t dwell_ms, unused_1;
	uint32_t rest_ms;
	uint32_t channel_count;
	struct apple80211_channel channels[400];
	uint64_t flags;
	uint32_t unused_2;
	uint32_t name_count;                    /* networks asked for by name, each in 56 bytes */
	uint8_t names[10][56];
	uint32_t short_ssid, unused_3;
};

/* SCAN_RESULT (11) and CURRENT_NETWORK (103): 1164 bytes */
struct apple80211_scan_result {
	uint32_t version;
	struct apple80211_channel channel;
	uint16_t noise_known;
	int16_t noise, snr, rssi;
	uint16_t beacon_interval, capability;
	uint8_t bssid[6];
	uint8_t rate_count, unused_0;
	uint32_t rates[15];                     /* Mbit/s */
	uint8_t ssid_length, ssid[32], unused_1[3];
	uint32_t age_ms;
	uint8_t flags, unused_2;
	uint16_t elements_length;               /* comes in as the room there is */
	uint8_t elements[1024];
};

/* ASSOCIATE (20): 592 bytes; of what follows the key (the RSN element wifid built, flags) nothing is used */
struct apple80211_assoc {
	uint32_t version;
	uint16_t mode, auth_lower, auth_upper, unused_0;
	uint32_t ssid_length;
	uint8_t ssid[32];
	uint8_t bssid[6], unused_1[2];
	uint32_t key_version, key_length, key_cipher;   /* cipher 10: a passphrase as typed; 6: the key (from 64 hex digits) */
	uint16_t key_flags, key_index;
	uint8_t key[64];
	uint8_t rest[0x250 - 0x88];
};

#define CIPHER_PMK              6
#define CIPHER_PASSPHRASE       10

/* the link event's data (at least 32 bytes, or the framework drops the event) */
struct apple80211_link_changed {
	uint8_t down, unused_0[3];
	int32_t rssi;
	uint16_t snr, noise;
	uint8_t unused_1[4];
	uint8_t involuntary, unused_2[3];
	uint32_t reason, subreason;
	uint8_t unused_3[4];
};

_Static_assert(sizeof(struct apple80211_assoc) == 592 && offsetof(struct apple80211_assoc, key) == 0x48,
    "the association request as Darwin userland 16.5 has it");
_Static_assert(sizeof(struct apple80211_link_changed) == 32 && offsetof(struct apple80211_link_changed, reason) == 0x14,
    "the link event as Darwin userland 16.5 reads it");
_Static_assert(sizeof(struct apple80211_channels) == 4824, "the channel list as Darwin userland 16.5 has it");
_Static_assert(sizeof(struct apple80211_channels_info) == 10016 && offsetof(struct apple80211_channels_info, number) == 0x32e &&
    offsetof(struct apple80211_channels_info, flags) == 0x12d0, "the channels' description as Darwin userland 16.5 has it");
_Static_assert(sizeof(struct apple80211_scan_request) == 5456 && offsetof(struct apple80211_scan_request, channels) == 0x48 &&
    offsetof(struct apple80211_scan_request, flags) == 0x1308 && offsetof(struct apple80211_scan_request, names) == 0x1318,
    "the scan request as Darwin userland 16.5 has it");
_Static_assert(sizeof(struct apple80211_scan_result) == 1164 && offsetof(struct apple80211_scan_result, ssid_length) == 0x60 &&
    offsetof(struct apple80211_scan_result, age_ms) == 0x84 && offsetof(struct apple80211_scan_result, elements) == 0x8c,
    "a scan's result as Darwin userland 16.5 has it");

/* Apple80211's events (APPLE80211_M_...) */
enum { EVENT_POWER_CHANGED = 1, EVENT_SSID_CHANGED = 2, EVENT_BSSID_CHANGED = 3, EVENT_LINK_CHANGED = 4, EVENT_ASSOC_DONE = 9,
       EVENT_SCAN_DONE = 10 };

static const char interface_name[16] = "en0";   /* (drivers/net/hakios.net.c makes one interface, and that is its name) */

static LCK_GRP_DECLARE(apple80211_lock_group, "hakios-apple80211");
static LCK_MTX_DECLARE(apple80211_lock, &apple80211_lock_group);
static unsigned result_next;                    /* which of what the last scan heard a SCAN_RESULT request gets next */
static unsigned cache_next;                     /* ... and a BEACON_SCAN_CACHE_REQ (390) request */
static uint32_t join_result;                    /* of the last join asked for: 0 while it runs, then Apple80211's number */

/* ---- channels ---------------------------------------------------------------------------------------- */

static unsigned
channel_number(unsigned frequency)
{
	return frequency == 2484 ? 14 : frequency < 4000 ? (frequency - 2407) / 5 : (frequency - 5000) / 5;
}

static unsigned
channel_frequency(const struct apple80211_channel *channel)
{
	bool low = channel->flags & CHANNEL_2GHZ ? true : channel->flags & CHANNEL_5GHZ ? false : channel->channel <= 14;

	if (channel->channel == 0 || channel->channel > 196 || (channel->flags & 0x2000)) {     /* (0x2000: 6 GHz) */
		return 0;
	}
	return low ? (channel->channel == 14 ? 2484 : 2407 + 5 * channel->channel) : 5000 + 5 * channel->channel;
}

static void
channel_fill(struct apple80211_channel *channel, unsigned frequency, unsigned flags)
{
	channel->version = 1;
	channel->channel = channel_number(frequency);
	channel->flags = CHANNEL_20MHZ | (frequency < 4000 ? CHANNEL_2GHZ : CHANNEL_5GHZ) | (flags & HAKIOS_STA_CHANNEL_PASSIVE ?
	    0 : CHANNEL_ACTIVE) | (flags & HAKIOS_STA_CHANNEL_RADAR ? CHANNEL_DFS : 0);
}

/* ---- the link joined, as its elements and ours make it ------------------------------------------------ */

/* PHY modes (APPLE80211_MODE_*) */
#define MODE_11A                0x2u
#define MODE_11B                0x4u
#define MODE_11G                0x8u
#define MODE_11N                0x10u
#define MODE_11AC               0x80u
#define MODES_OURS              (MODE_11A | MODE_11B | MODE_11G | MODE_11N | MODE_11AC)   /* (no HE in our association yet) */

#define NOISE_FLOOR             (-95)           /* dBm: ath11k's ATH11K_DEFAULT_NOISE_FLOOR; the firmware reports none */

struct joined_link {
	unsigned mode;                          /* one MODE_ */
	unsigned streams;                       /* 1 or 2 */
	unsigned width;                         /* MHz */
	int mcs;                                /* highest index the link was set up for; -1: legacy rates only */
	unsigned rate;                          /* Mbit/s: the highest the link was set up for, long guard interval */
};

static const uint8_t *
bss_element(const struct hakios_sta_bss *bss, unsigned id)
{
	for (unsigned at = 0; at + 2 <= bss->elements_length && at + 2 + bss->elements[at + 1] <= bss->elements_length;
	    at += 2 + bss->elements[at + 1]) {
		if (bss->elements[at] == id) {
			return &bss->elements[at];
		}
	}
	return NULL;
}

/*
 * What the station set the link up with, from the access point's elements and the rules of wifi/lx/src/lx_80211.c
 * (station_width, the peer's streams): HT when it has HT capabilities, VHT on 5 GHz when it also has VHT; two
 * streams when its HT MCS set has the second; 80 MHz when its VHT operation says 80 (5 GHz), else 40 when its HT
 * operation allows a second 20, else 20 (wifi_wide is on by default; the channels' own limits are not checked
 * here). The rate is the top of the 802.11 rate tables for that, not a measurement.
 */
static void
link_of(const struct hakios_sta_bss *bss, struct joined_link *link)
{
	/* tenths of Mbit/s, one stream, long guard interval, MCS 0-9 */
	static const uint16_t rates[3][10] = {
		{ 65, 130, 195, 260, 390, 520, 585, 650, 780, 867 },
		{ 135, 270, 405, 540, 810, 1080, 1215, 1350, 1620, 1800 },
		{ 293, 585, 878, 1170, 1755, 2340, 2633, 2925, 3510, 3900 },
	};
	const uint8_t *ht = bss_element(bss, 45), *ht_op = bss_element(bss, 61);
	const uint8_t *vht = bss_element(bss, 191), *vht_op = bss_element(bss, 192);
	bool low = bss->frequency < 4000;
	unsigned best = 0;

	bzero(link, sizeof(*link));
	link->streams = 1;
	link->width = 20;
	link->mcs = -1;
	for (unsigned at = 0; at + 2 <= bss->elements_length && at + 2 + bss->elements[at + 1] <= bss->elements_length;
	    at += 2 + bss->elements[at + 1]) {
		for (unsigned i = 0; (bss->elements[at] == 1 || bss->elements[at] == 50) && i < bss->elements[at + 1]; i++) {
			unsigned rate = (bss->elements[at + 2 + i] & 0x7f) / 2;

			best = rate <= 54 && rate > best ? rate : best;
		}
	}
	link->mode = !low ? MODE_11A : best > 11 ? MODE_11G : MODE_11B;
	link->rate = best;
	if (ht == NULL || ht[1] < 26) {
		return;
	}
	link->mode = MODE_11N;
	link->streams = ht[2 + 3 + 1] != 0 ? 2 : 1;     /* (capabilities 2, A-MPDU 1, then the RX MCS bitmask) */
	link->mcs = 7;
	if (ht_op != NULL && ht_op[1] >= 22 && (ht_op[3] & 4) != 0) {
		if (!low && vht != NULL && vht[1] >= 12 && vht_op != NULL && vht_op[1] >= 3 && vht_op[2] == 1) {
			link->width = 80;
		} else if ((ht_op[3] & 3) == 1 || (ht_op[3] & 3) == 3) {
			link->width = 40;
		}
	}
	if (!low && vht != NULL && vht[1] >= 12) {
		/* its RX MCS map (after 4 bytes of capabilities), two bits a stream: 0 up to 7, 1 up to 8, 2 up to 9 */
		unsigned map = vht[2 + 4] | vht[2 + 5] << 8, most = 9;

		link->mode = MODE_11AC;
		for (unsigned s = 0; s < link->streams; s++) {
			unsigned m = (map >> (2 * s)) & 3;

			if (m == 3) {
				link->streams = s == 0 ? 1 : s;
				break;
			}
			most = MIN(most, 7 + m);
		}
		link->mcs = (int)(link->width == 20 && most == 9 ? 8 : most);  /* (VHT MCS 9 is not defined at 20 MHz, 1-2 streams) */
	}
	link->rate = rates[link->width == 80 ? 2 : link->width == 40 ? 1 : 0][link->mcs] * link->streams / 10;
}

/* ---- what the caller is given back ------------------------------------------------------------------- */

/* At most as many bytes as it has room for. */
static int
give(const struct apple80211req *request, const void *data, size_t length)
{
	return request->req_data == USER_ADDR_NULL ? 0 : copyout(data, request->req_data, length < request->req_len ? length :
	       request->req_len);
}

/* An access point in the form a scan's result has; the caller has said how much room for elements there is. */
static int
give_network(const struct apple80211req *request, const struct hakios_sta_bss *bss)
{
	struct apple80211_scan_result *result = kalloc_data(sizeof(*result), Z_WAITOK | Z_ZERO);
	unsigned room = sizeof(result->elements);
	int error;

	if (result == NULL) {
		return ENOMEM;
	}
	if (request->req_len >= sizeof(*result) && copyin(request->req_data, result, offsetof(struct apple80211_scan_result,
	    elements)) == 0 && result->elements_length < room) {
		room = result->elements_length;
	}
	bzero(result, sizeof(*result));
	result->version = 1;
	channel_fill(&result->channel, bss->frequency, 0);
	result->rssi = bss->rssi;
	result->beacon_interval = bss->beacon_interval;
	result->capability = bss->capability;
	memcpy(result->bssid, bss->bssid, sizeof(result->bssid));
	result->ssid_length = bss->ssid_length;
	memcpy(result->ssid, bss->ssid, bss->ssid_length);
	result->age_ms = bss->age_ms;
	result->elements_length = (uint16_t)(bss->elements_length <= room ? bss->elements_length : 0);
	memcpy(result->elements, bss->elements, result->elements_length);
	/* the rates it says it has (elements 1 and 50: units of 500 kbit/s, the top bit marks the ones everybody must have) */
	for (unsigned at = 0; at + 2 <= bss->elements_length && at + 2 + bss->elements[at + 1] <= bss->elements_length;
	    at += 2 + bss->elements[at + 1]) {
		for (unsigned i = 0; (bss->elements[at] == 1 || bss->elements[at] == 50) && i < bss->elements[at + 1]; i++) {
			unsigned rate = (bss->elements[at + 2 + i] & 0x7f) / 2;

			if (rate != 0 && rate <= 54 && result->rate_count < 15) {
				result->rates[result->rate_count++] = rate;
			}
		}
	}
	error = give(request, result, sizeof(*result));
	kfree_data(result, sizeof(*result));
	return error;
}

/* ---- get ---------------------------------------------------------------------------------------------- */

static int
get(struct apple80211req *request)
{
	struct hakios_sta_state *state;
	int error = 0;

	switch (request->req_type) {
	case 43:
	case 44: {                              /* the driver's and the hardware's name: the text alone, in 256 bytes */
		static const char name[256] = "hakios: ath11k";        /* (WCN6750 on redwood, QCA6490 on negroni) */

		return give(request, name, sizeof(name));
	}
	case 12: {
		/*
		 * What the card can do, a bit each (apple80211_var.h: APPLE80211_CAP_*): the 13 bytes are the
		 * bits alone, from the first (the framework hands in a buffer that begins after the old
		 * header's version word). TKIP with Michael MIC (for the WPA2 group key), AES, AES-CCM,
		 * power management, short slot and preamble, WPA2, WME,
		 * short guard interval at 20 MHz. Read wrong, the framework sees no WPA2 and builds no join
		 * for a WPA2 network ("Unable to join").
		 */
		uint8_t bits[13] = { 0 };
		/* APPLE80211_CAP_TKIP=1, TKIPMIC=12, WPA2=14 (apple80211_var.h).
		 * Without TKIP, wifid can reject a TKIP-group RSN before ASSOCIATE,
		 * even though the station can now install that group key. */
		static const uint8_t has[] = { 1, 2, 3, 6, 9, 10, 12, 14, 17, 19 };

		for (unsigned i = 0; i < sizeof(has); i++) {
			bits[has[i] / 8] |= (uint8_t)(1u << (has[i] % 8));
		}
		return give(request, bits, sizeof(bits));
	}
	case 19: {                              /* the radio's power: one radio. Until the driver is up it counts as on: it */
		struct {                        /* is being brought up, and Darwin asks this once, early */
			uint32_t version, radios, state[4];
		} answer = { .version = 1, .radios = 1, .state = { !hakios_sta_present() || hakios_sta_powered() } };

		return give(request, &answer, sizeof(answer));
	}
	case 15:                                /* the mode: a station (APPLE80211_M_STA) */
		request->req_val = 1;
		return 0;
	case 5:                                 /* power save: off (APPLE80211_POWERSAVE_MODE_DISABLED); none yet */
	case 18:                                /* interference mitigation: off */
		request->req_val = 0;
		return 0;
	case 6: {                               /* protection (RTS/CTS for older stations): off, no threshold */
		static const uint32_t answer[3] = { 1, 0, 0 };

		return give(request, answer, sizeof(answer));
	}
	case 390: {
		/*
		 * The beacon cache: IO80211 asks again and again, one network in a scan's result form each time
		 * (elements_length set to the room), until a request fails. Its own place in the last scan's list;
		 * after the failure that ends a pass the next pass starts from the first again.
		 */
		struct hakios_sta_bss *bss = kalloc_data(sizeof(*bss), Z_WAITOK);
		unsigned index;
		bool there;

		if (bss == NULL) {
			return ENOMEM;
		}
		lck_mtx_lock(&apple80211_lock);
		index = cache_next++;
		lck_mtx_unlock(&apple80211_lock);
		there = hakios_sta_heard(index, bss);
		if (!there) {
			lck_mtx_lock(&apple80211_lock);
			cache_next = 0;
			lck_mtx_unlock(&apple80211_lock);
		}
		error = there ? give_network(request, bss) : EIO;
		kfree_data(bss, sizeof(*bss));
		return error;
	}
	case 27:
	case 254: {                             /* the channels the radio may tune to */
		struct apple80211_channels *answer;
		struct hakios_sta_channel channels[96];
		unsigned count = hakios_sta_channels(channels, 96);

		if (count == 0 || (answer = kalloc_data(sizeof(*answer), Z_WAITOK | Z_ZERO)) == NULL) {
			return count == 0 ? ENXIO : ENOMEM;
		}
		answer->version = 1;
		answer->count = count;
		for (unsigned i = 0; i < count; i++) {
			channel_fill(&answer->channels[i], channels[i].frequency, channels[i].flags);
		}
		error = give(request, answer, sizeof(*answer));
		kfree_data(answer, sizeof(*answer));
		return error;
	}
	case 207: {                             /* ... and more of each */
		struct apple80211_channels_info *answer;
		struct hakios_sta_channel channels[96];
		unsigned count = hakios_sta_channels(channels, 96);

		if (count == 0 || (answer = kalloc_data(sizeof(*answer), Z_WAITOK | Z_ZERO)) == NULL) {
			return count == 0 ? ENXIO : ENOMEM;
		}
		answer->count = (uint16_t)count;
		for (unsigned i = 0; i < count; i++) {
			struct apple80211_channel channel;

			channel_fill(&channel, channels[i].frequency, channels[i].flags);
			answer->number[i] = (uint8_t)channel.channel;
			answer->dfs[i] = (channels[i].flags & HAKIOS_STA_CHANNEL_RADAR) != 0;
			answer->passive[i] = (channels[i].flags & HAKIOS_STA_CHANNEL_PASSIVE) != 0;
			answer->flags[i] = channel.flags;
		}
		error = give(request, answer, sizeof(*answer));
		kfree_data(answer, sizeof(*answer));
		return error;
	}
	case 106: {
		/*
		 * The signal range, in dBm: a version, then RSSI_MIN, RSSI_MAX and RSSI_THRESHOLD (int32 each; the
		 * layout as iOSPlus10 read it from IO80211.framework of 20H392). wifid takes RSSI_THRESHOLD as the
		 * weakest network auto-join may pick (its own stricter -70/-65 for moving and crowded networks
		 * apply on top); refused, it stays 0 and every network is "Below minimum RSSI threshold".
		 * The values are not measured: min < threshold < max, -80 the usual auto-join floor.
		 */
		static const int32_t bounds[4] = { 1, -90, -40, -80 };

		return give(request, bounds, sizeof(bounds));
	}
	case 51: {                              /* the country the radio is in: its two letters and a zero */
		char code[3] = { 0 };

		return hakios_sta_country(code) ? give(request, code, sizeof(code)) : ENXIO;
	}
	case 21:                                /* how the last join went (0: not over yet) */
		lck_mtx_lock(&apple80211_lock);
		request->req_val = (int32_t)join_result;
		lck_mtx_unlock(&apple80211_lock);
		return 0;
	case 11: {                              /* what the last scan heard, one access point a request; EIO: that was all */
		struct hakios_sta_bss *bss = kalloc_data(sizeof(*bss), Z_WAITOK);
		unsigned index;
		bool there;

		if (bss == NULL) {
			return ENOMEM;
		}
		lck_mtx_lock(&apple80211_lock);
		index = result_next++;
		lck_mtx_unlock(&apple80211_lock);
		there = hakios_sta_heard(index, bss);
		error = there ? give_network(request, bss) : EIO;
		kfree_data(bss, sizeof(*bss));
		return error;
	}
	default:
		break;
	}

	/* the rest is about the network joined, and there is none unless the station says so */
	if ((state = kalloc_data(sizeof(*state), Z_WAITOK)) == NULL) {
		return ENOMEM;
	}
	hakios_sta_state(state);
	switch (request->req_type) {
	case 13:                                /* the state: APPLE80211_S_INIT 0, _SCAN 1, _AUTH 2, _ASSOC 3, _RUN 4 */
		request->req_val = state->phase == HAKIOS_STA_JOINED ? 4 : state->phase == HAKIOS_STA_JOINING ? 2 :
		    state->phase == HAKIOS_STA_SCANNING ? 1 : 0;
		break;
	case 1:                                 /* its name; the length goes back in the request */
		if (state->phase != HAKIOS_STA_JOINED) {
			error = ENXIO;
		} else if ((error = give(request, state->bss.ssid, state->bss.ssid_length)) == 0) {
			request->req_len = state->bss.ssid_length;
		}
		break;
	case 9:                                 /* its access point's address */
		error = state->phase != HAKIOS_STA_JOINED ? ENXIO : give(request, state->bss.bssid, 6);
		break;
	case 4: {                               /* its channel */
		struct {
			uint32_t version;
			struct apple80211_channel channel;
		} answer = { .version = 1 };

		if (state->phase != HAKIOS_STA_JOINED) {
			error = ENXIO;
		} else {
			channel_fill(&answer.channel, state->bss.frequency, 0);
			error = give(request, &answer, sizeof(answer));
		}
		break;
	}
	case 16: {                              /* how strongly it is heard: dBm, per radio and all together, twice over */
		struct {
			uint32_t version, radios, unit;
			int32_t each[4], all, each_ext[4], all_ext;
		} answer = { .version = 1, .radios = 1, .each = { state->bss.rssi }, .all = state->bss.rssi,
			     .each_ext = { state->bss.rssi }, .all_ext = state->bss.rssi };

		error = state->phase != HAKIOS_STA_JOINED ? ENXIO : give(request, &answer, sizeof(answer));
		break;
	}
	case 103:                               /* all of it, as a scan's result */
		error = state->phase != HAKIOS_STA_JOINED ? ENXIO : give_network(request, &state->bss);
		break;
	case 2: {                               /* how it was joined: open system, then WPA2 with a passphrase */
		struct {
			uint32_t version, lower, upper, extended;
		} answer = { .version = 1, .lower = 1, .upper = 8 };

		error = state->phase != HAKIOS_STA_JOINED ? ENXIO : give(request, &answer, sizeof(answer));
		break;
	}
	case 14: {                              /* the PHY modes: those the radio has, and the link's (0: none) */
		struct joined_link link;
		uint32_t answer[3] = { 1, MODES_OURS, 0 };

		if (state->phase == HAKIOS_STA_JOINED) {
			link_of(&state->bss, &link);
			answer[2] = link.mode;
		}
		error = give(request, answer, sizeof(answer));
		break;
	}
	case 17: {                              /* the noise, in RSSI's layout */
		struct {
			uint32_t version, radios, unit;
			int32_t each[4], all, each_ext[4], all_ext;
		} answer = { .version = 1, .radios = 1, .each = { NOISE_FLOOR }, .all = NOISE_FLOOR, .each_ext = { NOISE_FLOOR },
			     .all_ext = NOISE_FLOOR };

		error = state->phase != HAKIOS_STA_JOINED ? ENXIO : give(request, &answer, sizeof(answer));
		break;
	}
	case 8:                                 /* the rate, Mbit/s */
	case 57:                                /* the MCS: HT counts the streams into the index (0-15), VHT does not */
	case 353: {                             /* the spatial streams */
		struct joined_link link;

		if (state->phase != HAKIOS_STA_JOINED) {
			error = ENXIO;
			break;
		}
		link_of(&state->bss, &link);
		if (request->req_type == 57 && link.mcs < 0) {
			error = ENXIO;          /* (legacy rates only: no MCS) */
			break;
		}
		request->req_val = (int32_t)(request->req_type == 8 ? link.rate : request->req_type == 353 ? link.streams :
		    (unsigned)link.mcs + (link.mode == MODE_11N ? 8 * (link.streams - 1) : 0));
		break;
	}
	case 29: {                              /* why the last network was lost: an 802.11 reason, and from whom */
		struct {
			uint32_t version, reason;
			uint8_t address[6], unused[2];
		} answer = { .version = 1 };

		if (state->phase != HAKIOS_STA_JOINED && state->last_error != 0) {
			answer.reason = 4;      /* (as the link event says it: gone, not asked) */
			memcpy(answer.address, state->bss.bssid, sizeof(answer.address));
		}
		error = give(request, &answer, sizeof(answer));
		break;
	}
	default:
		error = EOPNOTSUPP;
		break;
	}
	kfree_data(state, sizeof(*state));
	return error;
}

/* ---- set ---------------------------------------------------------------------------------------------- */

static int
set_scan(const struct apple80211req *request, char *said, size_t said_size)
{
	struct apple80211_scan_request *asked;
	struct hakios_sta_scan *what;
	unsigned unknown = 0;
	int error;

	if (request->req_len != sizeof(*asked)) {
		return EINVAL;
	}
	if (!hakios_sta_present()) {
		return EBUSY;                   /* (the radio is still being brought up: the framework tries again) */
	}
	asked = kalloc_data(sizeof(*asked), Z_WAITOK);
	what = kalloc_data(sizeof(*what), Z_WAITOK | Z_ZERO);
	if (asked == NULL || what == NULL || (error = copyin(request->req_data, asked, sizeof(*asked))) != 0) {
		error = asked == NULL || what == NULL ? ENOMEM : error;
		goto out;
	}
	if (asked->ssid_length <= sizeof(what->ssid)) {
		what->ssid_length = (uint8_t)asked->ssid_length;
		memcpy(what->ssid, asked->ssid, asked->ssid_length);
	}
	what->passive = asked->scan_type == 2;
	what->dwell_ms = asked->dwell_ms;
	/* (more channels than there is room to name: every channel) */
	for (unsigned i = 0; i < asked->channel_count && i < 400 && asked->channel_count <= 64; i++) {
		unsigned frequency = channel_frequency(&asked->channels[i]);

		if (frequency != 0) {
			what->frequencies[what->frequency_count++] = (uint16_t)frequency;
		} else {
			unknown++;
		}
	}
	snprintf(said, said_size, " (a scan: type %u, %u channels of which %u are not the radio's, %u ms on each, a name of %u bytes, "
	    "%u more names, flags %#llx)", asked->scan_type, asked->channel_count, unknown, asked->dwell_ms, asked->ssid_length,
	    asked->name_count, asked->flags);
	if (asked->channel_count != 0 && asked->channel_count <= 64 && what->frequency_count == 0) {
		error = EINVAL;                 /* (nothing of what is asked for is a channel of this radio's) */
	} else {
		lck_mtx_lock(&apple80211_lock);
		result_next = 0;
		lck_mtx_unlock(&apple80211_lock);
		error = hakios_sta_ask_scan(what);
	}
out:
	if (asked != NULL) {
		kfree_data(asked, sizeof(*asked));
	}
	if (what != NULL) {
		kfree_data(what, sizeof(*what));
	}
	return error;
}

static int
set_join(const struct apple80211req *request, char *said, size_t said_size)
{
	struct apple80211_assoc *asked;
	struct hakios_sta_join *what;
	int error;

	if (request->req_len < offsetof(struct apple80211_assoc, rest)) {
		return EINVAL;
	}
	asked = kalloc_data(sizeof(*asked), Z_WAITOK | Z_ZERO);
	what = kalloc_data(sizeof(*what), Z_WAITOK | Z_ZERO);
	if (asked == NULL || what == NULL || (error = copyin(request->req_data, asked, offsetof(struct apple80211_assoc, rest))) != 0) {
		error = asked == NULL || what == NULL ? ENOMEM : error;
		goto out;
	}
	/* (lengths and kinds only: the name, the access point and the key stay out of the log) */
	snprintf(said, said_size, " (a join: a name of %u bytes, %s access point, authentication %#x/%#x, a key of %u bytes of kind %u)",
	    asked->ssid_length, asked->bssid[0] | asked->bssid[1] | asked->bssid[2] | asked->bssid[3] | asked->bssid[4] |
	    asked->bssid[5] ? "a given" : "any", asked->auth_lower, asked->auth_upper, asked->key_length, asked->key_cipher);
	if (asked->ssid_length == 0 || asked->ssid_length > sizeof(what->ssid)) {
		error = EINVAL;
		goto out;
	}
	what->ssid_length = (uint8_t)asked->ssid_length;
	memcpy(what->ssid, asked->ssid, asked->ssid_length);
	memcpy(what->bssid, asked->bssid, sizeof(what->bssid));
	if (asked->key_length == 0) {
		what->security = HAKIOS_STA_OPEN;
	} else if (asked->key_cipher == CIPHER_PASSPHRASE && asked->key_length >= 8 && asked->key_length <= 63) {
		what->security = HAKIOS_STA_WPA2_PSK;
		what->passphrase_length = (uint8_t)asked->key_length;
		memcpy(what->passphrase, asked->key, asked->key_length);
	} else if (asked->key_cipher == CIPHER_PMK && asked->key_length == sizeof(what->pmk)) {
		what->security = HAKIOS_STA_WPA2_PSK;
		memcpy(what->pmk, asked->key, sizeof(what->pmk));
	} else {
		error = EINVAL;
		goto out;
	}
	lck_mtx_lock(&apple80211_lock);
	join_result = 0;
	lck_mtx_unlock(&apple80211_lock);
	error = hakios_sta_ask_join(what);
out:
	if (asked != NULL) {
		memset_s(asked, sizeof(*asked), 0, sizeof(*asked));
		kfree_data(asked, sizeof(*asked));
	}
	if (what != NULL) {
		memset_s(what, sizeof(*what), 0, sizeof(*what));
		kfree_data(what, sizeof(*what));
	}
	return error;
}

static int
set(const struct apple80211req *request, char *said, size_t said_size)
{
	switch (request->req_type) {
	case 20:
		return set_join(request, said, said_size);
	case 22:                                /* leave the network */
		return hakios_sta_ask_leave();
	case 19: {                              /* the radio's power */
		struct {
			uint32_t version, radios, state[4];
		} asked;
		int error;

		if (request->req_len < sizeof(asked) || (error = copyin(request->req_data, &asked, sizeof(asked))) != 0) {
			return request->req_len < sizeof(asked) ? EINVAL : error;
		}
		snprintf(said, said_size, " (power: %s)", asked.state[0] ? "on" : "off");
		return hakios_sta_ask_power(asked.state[0] != 0);
	}
	case 10:
		return set_scan(request, said, said_size);
	case 365:                               /* the display is on or off (the value): a hint for power save, which */
		snprintf(said, said_size, " (the display: %s)", request->req_val ? "on" : "off");
		return 0;                       /* the station has none; taken, nothing to do */
	case 383: {                             /* Apple80211SetProperty: { key: value } as an XML property list, after a */
		char data[384];                 /* version and its length; hints for firmware tuning: taken, the key said */
		const char *key, *end;          /* (after the XML header and DOCTYPE, ~175 bytes) */

		if (request->req_len >= 8 + 16 && copyin(request->req_data + 8, data, MIN(sizeof(data) - 1, request->req_len - 8)) == 0) {
			data[sizeof(data) - 1] = 0;
			if ((key = strnstr(data, "<key>", sizeof(data))) != NULL && (end = strnstr(key, "</key>", sizeof(data) -
			    (size_t)(key - data))) != NULL) {
				snprintf(said, said_size, " (a property: %.*s)", (int)MIN(end - key - 5, 60), key + 5);
			}
		}
		return 0;
	}
	case 51: {                              /* the country (wifid: from the beacons' country elements); a version, then */
		struct {                        /* the letters (APPLE80211_MAX_CC_LEN 3) */
			uint32_t version;
			char code[4];
		} asked;
		int error;

		if (request->req_len < 6 || request->req_len > sizeof(asked)) {
			return EINVAL;
		}
		bzero(&asked, sizeof(asked));
		if ((error = copyin(request->req_data, &asked, request->req_len)) != 0) {
			return error;
		}
		snprintf(said, said_size, " (the country: version %u, \"%c%c\")", asked.version, asked.code[0] >= ' ' && asked.code[0] < 127 ?
		    asked.code[0] : '?', asked.code[1] >= ' ' && asked.code[1] < 127 ? asked.code[1] : '?');
		return hakios_sta_ask_country(asked.code);
	}
	default:
		return EOPNOTSUPP;
	}
}

/* ---- events -------------------------------------------------------------------------------------------- */

static void
post(uint32_t code, const void *data, uint32_t length)
{
	struct kev_msg message = { .vendor_code = KEV_VENDOR_APPLE, .kev_class = KEV_IEEE80211_CLASS, .kev_subclass = 1,
		                   .event_code = code };
	char name[16];

	memcpy(name, interface_name, sizeof(name));
	message.dv[0].data_length = sizeof(name);
	message.dv[0].data_ptr = name;
	if (length != 0) {
		message.dv[1].data_length = length;
		message.dv[1].data_ptr = __DECONST(void *, data);
	}
	kev_post_msg(&message);
}

/* What the station side says has happened (drivers/wifi/hakios.sta.c calls this, on its thread or the station's). */
void
hakios_apple80211_event(enum hakios_sta_event event, int status)
{
	static unsigned said;

	if (said < 200) {
		said++;
		printf("apple80211: the station says: %s, %d\n", event == HAKIOS_STA_POWER ? "power" : event == HAKIOS_STA_SCAN_DONE ?
		    "a scan is over" : event == HAKIOS_STA_JOIN_DONE ? "a join is over" : "the link", status);
	}
	switch (event) {
	case HAKIOS_STA_POWER:
		post(EVENT_POWER_CHANGED, NULL, 0);
		break;
	case HAKIOS_STA_SCAN_DONE:
		lck_mtx_lock(&apple80211_lock);
		result_next = 0;
		lck_mtx_unlock(&apple80211_lock);
		post(EVENT_SCAN_DONE, NULL, 0);
		break;
	case HAKIOS_STA_JOIN_DONE:
		lck_mtx_lock(&apple80211_lock);
		join_result = status == 0 ? 1 : status == EAUTH ? 21 : status == ETIMEDOUT ? 9 : 2;
		lck_mtx_unlock(&apple80211_lock);
		post(EVENT_ASSOC_DONE, NULL, 0);
		break;
	case HAKIOS_STA_LINK: {
		struct apple80211_link_changed link = { .down = status != 1, .involuntary = status != 1 && status != 0,
			                                .reason = status != 1 && status != 0 ? 4 : 0 };      /* (4: gone, not asked) */
		struct hakios_sta_state *state = kalloc_data(sizeof(*state), Z_WAITOK | Z_ZERO);

		if (state != NULL) {
			hakios_sta_state(state);
			link.rssi = status == 1 ? state->bss.rssi : 0;
			kfree_data(state, sizeof(*state));
		}
		post(EVENT_SSID_CHANGED, NULL, 0);
		post(EVENT_BSSID_CHANGED, NULL, 0);
		post(EVENT_LINK_CHANGED, &link, sizeof(link));
		break;
	}
	}
}

/* ---- the requests as they arrive --------------------------------------------------------------------- */

/*
 * How often each kind of request came and how it fared: said when the run ends, because the
 * lines said along the way can be lost with the middle of a long log.
 */
static struct {
	int32_t type;
	bool set;
	unsigned times, answered;
	int last_error;
} kinds[96];
static unsigned kind_count;

/* true: the first of its kind, or the first answer of a kind refused so far, or the first refusal of one answered */
static bool
note(const struct apple80211req *request, bool is_set, int error)
{
	bool first = false;
	unsigned i;

	lck_mtx_lock(&apple80211_lock);
	for (i = 0; i < kind_count && !(kinds[i].type == request->req_type && kinds[i].set == is_set); i++) {
	}
	if (i < sizeof(kinds) / sizeof(kinds[0])) {
		if (i == kind_count) {
			kinds[kind_count++] = (typeof(kinds[0])) { .type = request->req_type, .set = is_set };
		}
		first = kinds[i].times == 0 || (error == 0 && kinds[i].answered == 0) ||
		    (error != 0 && kinds[i].last_error == 0);
		kinds[i].times++;
		kinds[i].answered += error == 0;
		kinds[i].last_error = error != 0 ? error : kinds[i].last_error;
	}
	lck_mtx_unlock(&apple80211_lock);
	return first;
}

void
hakios_apple80211_summary(void)
{
	static bool said;

	if (said) {
		return;
	}
	said = true;
	for (unsigned i = 0; i < kind_count; i++) {
		printf("apple80211: in this run: %s %d (%s) %u times, answered %u times%s", kinds[i].set ? "set" : "get", kinds[i].type,
		    type_name(kinds[i].type), kinds[i].times, kinds[i].answered, kinds[i].last_error == 0 ? "\n" : "");
		if (kinds[i].last_error != 0) {
			printf("; the last refusal: %d%s\n", kinds[i].last_error, kinds[i].last_error == EOPNOTSUPP ? " (not built)" : "");
		}
	}
}

/* An Apple80211 request for the radio's interface, from the process that is running now. */
errno_t
hakios_apple80211_ioctl(u_long command, void *data)
{
	static unsigned said;
	struct apple80211req *request = data;
	char who[32] = "?", what[200] = "";
	bool is_set = command == SIOCSA80211;
	int error;

	if (command != SIOCSA80211 && command != SIOCGA80211) {
		return EOPNOTSUPP;
	}
	proc_selfname(who, sizeof(who));
	error = is_set ? set(request, what, sizeof(what)) : get(request);
	/* (a request that is repeated for ever is said a few hundred times; after that only what is new of a kind, joins,
	 * leaves and the country) */
	if (note(request, is_set, error) || said < 800 || (is_set && (request->req_type == 20 || request->req_type == 22 ||
	    request->req_type == 51))) {
		said++;
		printf("apple80211: %s %s %d (%s), value %d, %u bytes%s: %s", who, is_set ? "sets" : "gets", request->req_type,
		    type_name(request->req_type), request->req_val, request->req_len, what, error == 0 ? "answered\n" : error ==
		    EOPNOTSUPP ? "REFUSED\n" : "");
		if (error != 0 && error != EOPNOTSUPP) {
			printf("error %d\n", error);
		}
	}
	return error;
}
