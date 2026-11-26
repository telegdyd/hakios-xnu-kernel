/*
 * Where the driver meets 802.11 itself: under Linux that is mac80211 and cfg80211, under FreeBSD
 * LinuxKPI's stand-in for them on top of net80211, and here it is ours -- the functions that
 * LinuxKPI's headers (net/mac80211.h, net/cfg80211.h) declare under linuxkpi_ names, as far as
 * the driver calls them, and a station: the layer that asks the driver for things, in
 * mac80211's order (FreeBSD's linux_80211.c shows it).
 *
 * What is here: the radio as the driver describes it when it registers; the regulatory domain
 * the driver hands over, applied to the radio's channels the way cfg80211 applies it; one
 * interface in station mode; a scan of every channel the domain allows; and the joining of one
 * network that is protected with WPA2 and a passphrase -- found by its name, authenticated to
 * (open system), associated with, the keys agreed in the four-way handshake (lx_wpa.c) and
 * given to the driver --, after which Ethernet frames pass between the kernel and the driver
 * (the firmware puts them into 802.11 and ciphers them: the driver's "frame mode" Ethernet).
 *
 * What is not: more than one network, roaming, power saving, 40 MHz and wider channels, HE,
 * and the check of the packet numbers of the group's data frames that
 * the driver leaves to this layer.
 *
 * On the phone, 13 October, this scans -- 26 channels in 4.1 s, the access points around heard
 * with their channels and strengths -- and joins: the handshake is through 130 ms after the
 * association, and Darwin has the network (board/wifi.c, step_driver(), has the
 * account). What is said of other people's networks names them: it is for the log of the run.
 * The name of the network to join and its passphrase are never said, only their lengths.
 */
#include <linux/kernel.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <net/cfg80211.h>
#include <net/mac80211.h>

#include <lx_os.h>
#include "lx.h"
#include "lx_wpa.h"
#include "lx_sae.h"
#include "lx_owe.h"

int linuxkpi_debug_80211;			/* (which of the headers' own remarks are printed: none) */

#define HEARD_MOST	128
#define ELEMENTS_MOST	HAKIOS_STA_ELEMENTS_MOST

/* A network, as its beacons describe it. */
struct lx_heard {
	uint8_t sender[ETH_ALEN];		/* the access point's address (BSSID) */
	uint8_t name[32], name_length;
	uint16_t frequency;			/* MHz, of the channel it was heard on at its strongest */
	uint8_t says_channel;			/* the channel it says it is on (0: it does not say) */
	int8_t signal;				/* dBm, the strongest */
	uint16_t capability, interval;
	bool protected;				/* it announces WPA2 or later (an RSN element) */
	unsigned times;
	unsigned long long last;		/* when it was last heard: nanoseconds */
	uint16_t elements_length;		/* all it says of itself, as last heard (nothing, if that was too much to keep) */
	uint8_t elements[ELEMENTS_MOST];
};

/* The access point of the wanted network that was heard best, with all it says of itself. */
struct lx_target {
	bool found;
	uint8_t address[ETH_ALEN];
	uint16_t frequency, capability, interval;
	int8_t signal;
	unsigned elements_length;
	uint8_t elements[ELEMENTS_MOST];
};

/* What a management frame from the access point has answered (join() waits for these). */
enum { ANSWER_NONE, ANSWER_AUTHENTICATED, ANSWER_ASSOCIATED, ANSWER_REFUSED, ANSWER_GONE };

/* A radio: what stands behind the driver's "hw" and "wiphy". There is one. */
struct lx_radio {
	void *memory;				/* all of this was allocated as one piece, which begins here */
	struct wiphy *wiphy;
	struct ieee80211_hw hw;
	const struct ieee80211_ops *ops;
	bool registered;
	bool kept;				/* the channels' flags and powers as the driver set them have been put by */
	int applying;				/* a regulatory domain is being applied to the channels right now */
	struct workqueue_struct *work;		/* for what the driver has "the stack" run for it */
	struct linuxkpi_ieee80211_regdomain *regd;
	struct ieee80211_vif *vif;		/* the one interface, while it is up */
	void *vif_memory;
	bool started;

	/* a scan */
	volatile int scan;			/* 0: none, 1: running, 2: over, 3: given up by the driver */
	unsigned frames, others;		/* received in all, and of them neither beacon nor probe response */
	unsigned heard_count, heard_lost;
	struct lx_heard heard[HEARD_MOST];
	uint8_t want[32], want_length;		/* the name of the network that is wanted ... */
	uint8_t want_bssid[6];			/* ... and, if not all zero, the access point of that name that is wanted */
	bool target_fixed;			/* the join has chosen: heard() leaves `target` as it is */
	unsigned width;				/* MHz of the channel joined: 20, or with wifi_wide=1 what the access point offers */
	struct lx_target target;		/* ... and the best of it */

	/* the network joined */
	struct ieee80211_sta *sta;		/* its access point, as the driver knows it */
	void *sta_memory;
	struct ieee80211_chanctx_conf *channel;
	struct cfg80211_bss bss;		/* what the driver may ask of the access point's beacon */
	struct cfg80211_bss_ies bss_elements;
	volatile int answer;
	uint16_t answer_status, aid;
	unsigned answer_length;
	uint8_t answer_elements[ELEMENTS_MOST];	/* of the association response */
	uint8_t rsn[47];                 /* RSN, optional cached SAE PMKID and H2E RSNXE */
	struct lx_sae sae;
	struct lx_owe owe;
	struct lx_sae_cache sae_cache;
	unsigned sae_h2e, sae_cached;
	uint8_t sae_pmkid[16];
	bool sae_listening;
	unsigned sae_rx_length;
	uint8_t sae_rx[400];             /* bounded mailbox: crypto runs on the station thread */
	uint64_t management_tx_pn;		/* CCMP management nonce space is separate from data */
	struct lx_wpa wpa;
	volatile int handshake;			/* the last thing lx_wpa_message() said */
	const char *handshake_why;
	struct work_struct group_work;		/* a new group key, later on */
	/* block ack: the access point's requests to send us frames in aggregates, by traffic class (TID) */
	struct work_struct ba_work;
	volatile uint8_t ba_asked;		/* TIDs with a request (ADDBA or DELBA) not yet dealt with */
	struct {
		bool start;			/* ADDBA (else DELBA) */
		bool active;			/* the driver takes aggregates of it */
		bool amsdu;
		uint8_t token;
		uint16_t ssn, size, timeout;
	} ba[8];
	unsigned ba_sessions, ba_refused;
	struct ieee80211_key_conf *pairwise_key, *group_key;
	volatile bool joined;			/* keys set: frames pass */
	volatile bool joining;
	int reached;				/* how far the access point has come as the driver's station (IEEE80211_STA_...) */
	int8_t signal;				/* dBm, of the last beacon of the network joined (or the driver's measure) */
	unsigned long long signal_rest_until;	/* the driver's measure failed or was slow: not asked again until then */
	unsigned signal_said;
	int last_error;
	enum hakios_sta_loss loss;		/* how the network was lost, if it was ... */
	uint16_t loss_reason;			/* ... and the access point's reason code */
	unsigned sent, sent_lost, received, received_group, eapol, rekeys;
	unsigned sent_offloaded, received_checked, sent_classed;	/* (P1d: checksums the radio did, WMM classes used) */
	/*
	 * The receive and send paths' use of the station and the interface (a reply to the access point, its address, a frame
	 * Darwin sends), against station_leave() and station_down() freeing them: users counts the frames being acted on,
	 * closing keeps new ones off.
	 */
	int station_users;
	bool station_closing;
	unsigned unciphered;			/* frames dropped for coming unciphered once the keys were set */
	unsigned long long joined_at;		/* nanoseconds */
};

static struct lx_radio *the_radio;
extern unsigned int ath11k_debug_mask;	/* (core.c) */
void lx_mhi_m3_irqs(unsigned *ev, unsigned *ev_pending, unsigned *bhi, unsigned *kicks, unsigned *net);
volatile bool lx_doze_in_call;	/* inside the doze's suspend or resume (lx_bus.c keeps their WMI lines out of the log) */
bool lx_mhi_suspend_ready(void);	/* lx_mhi.c */
bool lx_mhi_in_fast_m3(void);
unsigned lx_mhi_suspend_calls(void);
int lx_bus_wow_undo(bool mhi_was_asked);	/* lx_bus.c */
int lx_os_wifi_ipv6_addresses(uint8_t addresses[][16], unsigned capacity);
void lx_bus_wow_ns(struct ieee80211_vif *vif, const uint8_t addresses[][16], unsigned count);

/*
 * Doze (WIFI-PLAN.md P4c/P4d): while the screen is off, a network is joined and nothing has been sent or received for
 * wifi_doze_idle_ms, the radio is put in WoW (ath11k_wow_op_suspend: wake events, ARP offload, keepalive; MHI M3), the
 * PCIe link off (hakios_pcie_sleep), and WAKE# (GPIO 96, through the PDC) armed. A frame to send, a call of the station
 * boundary, WAKE#, the screen on or wifi_doze=0 bring it back (hakios_pcie_wake, ath11k_wow_op_resume). Frames sent while
 * dozing wait in a small queue and go once awake. One thread (wifi-doze) does all of it; the station thread's calls wait
 * for it. Also station power save (wifi_ps: WMI STA_POWERSAVE, the driver's BSS_CHANGED_PS), applied by the same thread.
 */
#define DOZE_QUEUE 32
#define DOZE_FRAME 1600
#define DOZE_SOLICITED_MAX 4
static struct {
	volatile int state;			/* 0 awake, 1 changing, 2 dozing */
	volatile bool wanted;			/* back, asked for */
	volatile unsigned calls, sending;
	volatile unsigned long long active_at;	/* the last frame sent or received, or call */
	bool thread, ps_applied, ps_known;
	unsigned long long backoff_until, since, dozed_ns, longest_ns, last_wake_ns;
	unsigned queued, queue_lost, by_line, by_send, by_call, by_other, by_msi;
	int lost_by;	/* a WoW wake reason that says the network is gone, plus 1 (lx_doze_reason); 0: none */
	unsigned lost_wakes, reason_unspecified, instant;	/* instant: dozes that ended within 5 ms */
	unsigned dozes, wakes, failed_suspend, failed_sleep, failed_wake, failed_resume, ps_changes;
	unsigned not_ready, failed_undo;	/* MHI busy when a doze was due; a failed suspend's undo that failed too */
	bool given_up;				/* a wake failed: no more dozes this boot */
	unsigned lengths[DOZE_QUEUE], flags[DOZE_QUEUE];
	unsigned reason_last[8], reason_n, reason_count[64];
	char reason_name[8][24];
	unsigned dozed_last_ms[8], dozed_n, patterns_used;
	int solicited_patterns;			/* negative when the multicast prefix is used as a fallback */
	uint8_t *queue;				/* DOZE_QUEUE * DOZE_FRAME */
	struct cfg80211_wowlan wowlan;
	struct cfg80211_pkt_pattern patterns[1 + DOZE_SOLICITED_MAX];
	uint8_t pattern_bytes[1 + DOZE_SOLICITED_MAX][ETH_HLEN];
} doze;
static const uint8_t no_network[ETH_ALEN];	/* the address of the network an interface has joined, while it has joined none */

static struct lx_radio *
radio_of(struct ieee80211_hw *hw)
{
	return (__containerof(hw, struct lx_radio, hw));
}

/* ---- the radio, as the driver makes and registers it -------------------------------------------- */

struct ieee80211_hw *
linuxkpi_ieee80211_alloc_hw(size_t private, const struct ieee80211_ops *ops)
{
	size_t ours = roundup2(sizeof(struct lx_radio), 64);
	uint8_t *memory = kzalloc(64 + offsetof(struct wiphy, priv) + ours + private, GFP_KERNEL);
	struct lx_radio *radio;
	struct wiphy *wiphy;

	if (memory == NULL) {
		return (NULL);
	}
	wiphy = (struct wiphy *)roundup2((uintptr_t)memory, 64);
	radio = (struct lx_radio *)wiphy->priv;
	radio->memory = memory;
	radio->wiphy = wiphy;
	radio->ops = ops;
	radio->hw.wiphy = wiphy;
	radio->hw.priv = (uint8_t *)radio + ours;
	radio->work = alloc_ordered_workqueue("wifi-80211", 0);
	mutex_init(&wiphy->mtx);
	return (&radio->hw);
}

struct ieee80211_hw *
linuxkpi_wiphy_to_ieee80211_hw(struct wiphy *wiphy)
{
	return (&((struct lx_radio *)wiphy->priv)->hw);
}

void
linuxkpi_set_ieee80211_dev(struct ieee80211_hw *hw)
{
}

int
linuxkpi_80211_wiphy_register(struct wiphy *wiphy)
{
	return (0);
}

/* ieee80211_register_hw(): the driver has described its radio, and from here on may be asked for things. */
int
linuxkpi_ieee80211_ifattach(struct ieee80211_hw *hw)
{
	struct lx_radio *radio = radio_of(hw);
	char bands[96] = "";

	for (unsigned band = 0; band < NUM_NL80211_BANDS; band++) {
		if (hw->wiphy->bands[band] != NULL) {
			snprintf(bands + strlen(bands), sizeof(bands) - strlen(bands), " %d in band %u,", hw->wiphy->bands[band]->n_channels,
			    band);
		}
	}
	printf("lx: the driver registers a radio: channels:%s (0: 2.4 GHz, 1: 5 GHz, 3: 6 GHz); %s; up to %d names and %u "
	    "bytes of extras in a scan\n", bands, is_zero_ether_addr(hw->wiphy->perm_addr) ? "IT HAS NO ADDRESS" :
	    "it has an address", hw->wiphy->max_scan_ssids, hw->wiphy->max_scan_ie_len);
	radio->registered = true;
	the_radio = radio;
	/*
	 * WIFI-PLAN stage 9 (PCIe path): the firmware has said "unified ready" (core.c waits for it before it registers the
	 * radio) and mac.c has the radio's address, from the source named here (lx_bus.c; never the address itself).
	 */
	if (lx_os_radio_model() == 1) {
		if (is_zero_ether_addr(hw->wiphy->perm_addr)) {
			lx_os_stage(9, "wmi", 0, "wmi: unified ready, but no address for the radio");
		} else {
			lx_os_stage(9, "wmi", 1, "wmi: unified ready, mac registered, address from %s", lx_mac_source());
		}
	}
	return (0);
}

/* The driver takes its radio back, and frees the channels next: nobody may be at them when this returns. */
void
linuxkpi_ieee80211_unregister_hw(struct ieee80211_hw *hw)
{
	struct lx_radio *radio = radio_of(hw);

	lx_os_lock();
	radio->registered = false;
	while (radio->applying != 0) {
		lx_os_sleep(&radio->applying, 5 * 1000000ull);
	}
	if (the_radio == radio) {
		the_radio = NULL;
	}
	lx_os_unlock();
}

void
linuxkpi_ieee80211_iffree(struct ieee80211_hw *hw)
{
	struct lx_radio *radio = radio_of(hw);

	if (radio->work != NULL) {
		destroy_workqueue(radio->work);
		radio->work = NULL;
	}
}

void
linuxkpi_wiphy_free(struct wiphy *wiphy)
{
	struct lx_radio *radio = (struct lx_radio *)wiphy->priv;

	kfree(radio->regd);
	lx_sae_clear(&radio->sae);
	lx_owe_clear(&radio->owe);
	lx_crypto_wipe(&radio->sae_cache,sizeof(radio->sae_cache));
	lx_crypto_wipe(&radio->wpa,sizeof(radio->wpa));
	kfree(radio->memory);
}

/* ---- the regulatory domain ------------------------------------------------------------------------ */

/*
 * What cfg80211 does with a domain (Linux: net/wireless/reg.c, handle_channel()): a channel
 * may be used if a rule covers its 20 MHz, and takes the rule's limits; otherwise it is
 * disabled. The driver reads the result off the channels when it tells the firmware which it
 * may use (reg.c: ath11k_reg_update_chan_list()), and so does the scan below.
 */
static void
regd_apply(struct lx_radio *radio)
{
	const struct linuxkpi_ieee80211_regdomain *regd = radio->regd;
	char said[160] = "";

	for (unsigned band = 0; band < NUM_NL80211_BANDS; band++) {
		struct ieee80211_supported_band *has = radio->wiphy->bands[band];
		int allowed = 0;

		for (int i = 0; has != NULL && i < has->n_channels; i++) {
			struct linuxkpi_ieee80211_channel *channel = &has->channels[i];
			const struct ieee80211_reg_rule *rule = NULL;

			if (!radio->kept) {
				channel->orig_flags = channel->flags;
				channel->orig_mpwr = channel->max_power;
			}
			for (int r = 0; r < regd->n_reg_rules && rule == NULL; r++) {
				if (regd->reg_rules[r].freq_range.start_freq_khz <= ((int)channel->center_freq - 10) * 1000 &&
				    regd->reg_rules[r].freq_range.end_freq_khz >= ((int)channel->center_freq + 10) * 1000) {
					rule = &regd->reg_rules[r];
				}
			}
			channel->flags = channel->orig_flags & ~(IEEE80211_CHAN_DISABLED | IEEE80211_CHAN_NO_IR | IEEE80211_CHAN_RADAR);
			if (rule == NULL) {
				channel->flags |= IEEE80211_CHAN_DISABLED;
				continue;
			}
			if (rule->flags & NL80211_RRF_NO_IR) {
				channel->flags |= IEEE80211_CHAN_NO_IR;
			}
			if (rule->flags & NL80211_RRF_DFS) {
				channel->flags |= IEEE80211_CHAN_RADAR;
			}
			channel->max_reg_power = rule->power_rule.max_eirp / 100;	/* (hundredths of a dBm) */
			channel->max_antenna_gain = rule->power_rule.max_antenna_gain / 100;
			channel->max_power = channel->orig_mpwr != 0 && channel->orig_mpwr < channel->max_reg_power ?
			    channel->orig_mpwr : channel->max_reg_power;
			allowed++;
		}
		if (has != NULL) {
			snprintf(said + strlen(said), sizeof(said) - strlen(said), " %d of %d in band %u,", allowed, has->n_channels, band);
		}
	}
	radio->kept = true;
	lx_tell("lx: the regulatory domain \"%c%c\" (%d rules) allows the radio's channels:%s\n", regd->alpha2[0], regd->alpha2[1],
	    regd->n_reg_rules, said);
	for (int r = 0; r < regd->n_reg_rules; r++) {
		lx_tell("lx:   rule %d: %d..%d MHz, up to %d MHz wide, %d dBm, flags 0x%x\n", r,
		    regd->reg_rules[r].freq_range.start_freq_khz / 1000, regd->reg_rules[r].freq_range.end_freq_khz / 1000,
		    regd->reg_rules[r].freq_range.max_bandwidth_khz / 1000, regd->reg_rules[r].power_rule.max_eirp / 100,
		    regd->reg_rules[r].flags);
	}
}

/*
 * The driver's domain for its radio (which it has from the firmware). It keeps its own copy and
 * may free it. The driver calls this from work of its own, one call at a time, and may be
 * waiting for that work to end while it holds the radio's lock: so that lock is not taken here.
 * A domain that comes when the radio is no longer registered is not applied (the firmware sends
 * one whenever the country changes, also while the driver is on its way down).
 */
int
linuxkpi_regulatory_set_wiphy_regd_sync(struct wiphy *wiphy, struct linuxkpi_ieee80211_regdomain *regd)
{
	struct lx_radio *radio = (struct lx_radio *)wiphy->priv;
	struct linuxkpi_ieee80211_regdomain *copy, *old;
	size_t size;

	if (regd == NULL) {
		return (-EINVAL);
	}
	size = sizeof(*regd) + regd->n_reg_rules * sizeof(regd->reg_rules[0]);
	copy = kmalloc(size, GFP_KERNEL);
	if (copy == NULL) {
		return (-ENOMEM);
	}
	memcpy(copy, regd, size);
	lx_os_lock();
	if (!radio->registered) {
		lx_os_unlock();
		kfree(copy);
		return (-ENODEV);
	}
	radio->applying++;
	lx_os_unlock();
	old = radio->regd;
	radio->regd = copy;
	wiphy->regd = copy;
	regd_apply(radio);
	lx_os_lock();
	radio->applying--;
	lx_os_wakeup(&radio->applying);
	lx_os_wakeup(&radio->regd);		/* (boundary_country() waits for the firmware's new domain) */
	lx_os_unlock();
	kfree(old);
	return (0);
}

/* IEEE 802.11-2020, Annex E: a channel's centre, in MHz */
uint32_t
linuxkpi_ieee80211_channel_to_frequency(uint32_t channel, enum nl80211_band band)
{
	switch (band) {
	case NL80211_BAND_2GHZ:
		return (channel == 14 ? 2484 : 2407 + 5 * channel);
	case NL80211_BAND_5GHZ:
		return (channel >= 182 && channel <= 196 ? 4000 + 5 * channel : 5000 + 5 * channel);
	case NL80211_BAND_6GHZ:
		return (channel == 2 ? 5935 : 5950 + 5 * channel);
	default:
		return (0);
	}
}

static unsigned
channel_of(unsigned frequency)
{
	return (frequency == 2484 ? 14 : frequency < 2484 ? (frequency - 2407) / 5 : frequency < 5950 ? (frequency - 5000) / 5 :
	       (frequency - 5950) / 5);
}

uint32_t
linuxkpi_ieee80211_frequency_to_channel(uint32_t frequency, uint32_t flags)
{
	return (channel_of(frequency));
}

/* The channel of that frequency among those the radio was registered with. */
struct linuxkpi_ieee80211_channel *
linuxkpi_ieee80211_get_channel(struct wiphy *wiphy, uint32_t frequency)
{
	for (unsigned band = 0; band < NUM_NL80211_BANDS; band++) {
		struct ieee80211_supported_band *has = wiphy->bands[band];

		for (int i = 0; has != NULL && i < has->n_channels; i++) {
			if (has->channels[i].center_freq == frequency) {
				return (&has->channels[i]);
			}
		}
	}
	return (NULL);
}

/* ---- what the driver has the layer above do for it ------------------------------------------------- */

void
linuxkpi_ieee80211_queue_work(struct ieee80211_hw *hw, struct work_struct *work)
{
	queue_work(radio_of(hw)->work, work);
}

void
linuxkpi_ieee80211_queue_delayed_work(struct ieee80211_hw *hw, struct delayed_work *work, int delay)
{
	queue_delayed_work(radio_of(hw)->work, work, delay);
}

/* (flags: all interfaces, or those that are up; here the one there is is up for as long as it is there) */
void
linuxkpi_ieee80211_iterate_interfaces(struct ieee80211_hw *hw, enum ieee80211_iface_iter flags,
    void (*function)(void *, uint8_t *, struct ieee80211_vif *), void *argument)
{
	struct ieee80211_vif *vif = radio_of(hw)->vif;

	if (vif != NULL) {
		function(argument, vif->addr, vif);
	}
}

/* (no network has been joined: there are no stations and no channel is held) */
void
linuxkpi_ieee80211_iterate_stations_atomic(struct ieee80211_hw *hw, void (*function)(void *, struct ieee80211_sta *), void *argument)
{
}

void
linuxkpi_ieee80211_iterate_chan_contexts(struct ieee80211_hw *hw,
    void (*function)(struct ieee80211_hw *, struct ieee80211_chanctx_conf *, void *), void *argument)
{
}

/* The one station there can be: the access point of the network being joined. */
struct ieee80211_sta *
linuxkpi_ieee80211_find_sta(struct ieee80211_vif *vif, const u8 *address)
{
	struct lx_radio *radio = the_radio;

	return (radio != NULL && radio->sta != NULL && memcmp(radio->sta->addr, address, ETH_ALEN) == 0 ? radio->sta : NULL);
}

struct ieee80211_sta *
linuxkpi_ieee80211_find_sta_by_ifaddr(struct ieee80211_hw *hw, const uint8_t *address, const uint8_t *own)
{
	struct lx_radio *radio = radio_of(hw);

	return (radio->sta != NULL && memcmp(radio->sta->addr, address, ETH_ALEN) == 0 ? radio->sta : NULL);
}

/*
 * What a scan has learnt of an access point. The driver asks this of the one it is associating
 * with, to see whether that announces RSN: it then tells the firmware to let nothing but the
 * handshake through until the keys are set (mac.c: ath11k_peer_assoc_h_crypto()).
 */
struct cfg80211_bss *
linuxkpi_cfg80211_get_bss(struct wiphy *wiphy, struct linuxkpi_ieee80211_channel *channel, const uint8_t *bssid,
    const uint8_t *ssid, size_t ssid_length, enum ieee80211_bss_type type, enum ieee80211_privacy privacy)
{
	struct lx_radio *radio = (struct lx_radio *)wiphy->priv;

	if (!radio->target.found || bssid == NULL || memcmp(radio->target.address, bssid, ETH_ALEN) != 0) {
		return (NULL);
	}
	radio->bss_elements.data = radio->target.elements;
	radio->bss_elements.len = radio->target.elements_length;
	radio->bss.ies = radio->bss.beacon_ies = &radio->bss_elements;
	radio->bss.signal = radio->target.signal * 100;
	return (&radio->bss);
}

void
linuxkpi_cfg80211_put_bss(struct wiphy *wiphy, struct cfg80211_bss *bss)
{
}

/* (nothing is queued for sending: there is nothing to wake, and nothing sent to account for) */
void
linuxkpi_ieee80211_wake_queues(struct ieee80211_hw *hw)
{
}

void
linuxkpi_ieee80211_stop_queues(struct ieee80211_hw *hw)
{
}

void
linuxkpi_ieee80211_handle_wake_tx_queue(struct ieee80211_hw *hw, struct ieee80211_txq *queue)
{
}

void
linuxkpi_ieee80211_tx_status(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	dev_kfree_skb_any(skb);
}

void
linuxkpi_ieee80211_tx_status_ext(struct ieee80211_hw *hw, struct ieee80211_tx_status *status)
{
	if (status->skb != NULL) {
		dev_kfree_skb_any(status->skb);
	}
}

void
linuxkpi_ieee80211_free_txskb(struct ieee80211_hw *hw, struct sk_buff *skb, int status)
{
	dev_kfree_skb_any(skb);
}

/*
 * The firmware has missed the access point's beacons (WMI_ROAM_REASON_BEACON_MISS). ath11k then waits
 * ATH11K_CONNECTION_LOSS_HZ for one to come again (a beacon received cancels the wait); if none does,
 * connection_loss() below. mac80211 would probe the access point meanwhile; the firmware does that itself
 * (CONNECTION_MONITOR), so this only says it.
 */
void
linuxkpi_ieee80211_beacon_loss(struct ieee80211_vif *vif)
{
	lx_tell("lx: the driver reports beacons lost\n");
}

/* ... and none came again: the network is gone, said as a loss (WIFI-PLAN.md P3b); Darwin decides what next. */
void
linuxkpi_ieee80211_connection_loss(struct ieee80211_vif *vif)
{
	struct lx_radio *radio = the_radio;

	lx_tell("lx: the driver reports the connection lost%s\n", radio != NULL && radio->joined ? ": THE NETWORK IS LOST" : "");
	if (radio != NULL && radio->joined) {
		radio->joined = false;
		radio->last_error = ETIMEDOUT;
		radio->loss = HAKIOS_STA_LOSS_BEACONS;
		radio->loss_reason = 0;
		lx_os_net_link(0);
		lx_os_station_lost(ETIMEDOUT);
	}
}

/* (the driver's recovery after its firmware has died, which nothing here goes through) */
void
linuxkpi_ieee80211_restart_hw(struct ieee80211_hw *hw)
{
}

/* ---- elements --------------------------------------------------------------------------------------- */

/* The element of that number among `length` bytes of elements (number, length, contents; one after another), or NULL. */
static const uint8_t *
element(const uint8_t *elements, unsigned length, unsigned number)
{
	for (unsigned at = 0; at + 2 <= length && at + 2 + elements[at + 1] <= length; at += 2 + elements[at + 1]) {
		if (elements[at] == number) {
			return (elements + at);
		}
	}
	return (NULL);
}

static bool adopt_heard_country(struct lx_radio *radio);

/* A vendor's element (221) that begins with these four bytes. */
static const uint8_t *
vendor_element(const uint8_t *elements, unsigned length, const uint8_t *begins)
{
	for (unsigned at = 0; at + 2 <= length && at + 2 + elements[at + 1] <= length; at += 2 + elements[at + 1]) {
		if (elements[at] == 221 && elements[at + 1] >= 4 && memcmp(elements + at + 2, begins, 4) == 0) {
			return (elements + at);
		}
	}
	return (NULL);
}

/* An extension element (255) of that extension number (35: HE capabilities, 36: HE operation). */
static const uint8_t *
extension_element(const uint8_t *elements, unsigned length, unsigned number)
{
	for (unsigned at = 0; at + 2 <= length && at + 2 + elements[at + 1] <= length; at += 2 + elements[at + 1]) {
		if (elements[at] == 255 && elements[at + 1] >= 1 && elements[at + 2] == number) {
			return (elements + at);
		}
	}
	return (NULL);
}

/*
 * An HE capabilities element after its 17 fixed bytes (802.11ax-2021 9.4.2.248, as mac80211's util.c sizes it): the
 * HE-MCS/NSS set is 4 bytes, 4 more with 160 MHz (PHY B0 bit 3), 4 more with 80+80 (bit 4); then, if PHY B6 bit 7
 * says so, the PPE thresholds: a 7-bit header (NSS-1, RU index bitmask) and 6 bits per NSS per RU, rounded up to bytes.
 */
static unsigned
he_mcs_size(const uint8_t *phy)
{
	return (4 + (phy[0] & 0x08 ? 4 : 0) + (phy[0] & 0x10 ? 4 : 0));
}

static unsigned
he_ppe_size(uint8_t header, const uint8_t *phy)
{
	unsigned bits;

	if (!(phy[6] & 0x80)) {
		return (0);
	}
	bits = 7 + (unsigned)__builtin_popcount(header & 0x78) * (1 + (header & 7)) * 6;
	return ((bits + 7) / 8);
}

static const uint8_t wmm[4] = { 0x00, 0x50, 0xf2, 0x02 };	/* Microsoft's, which is how WMM is announced */

/* ---- what the radio hears -------------------------------------------------------------------------- */

/*
 * A beacon or an answer to a probe (IEEE 802.11-2020, 9.3.3): after the 24 bytes every
 * management frame begins with -- the sender's network is the third address -- a time stamp, the
 * beacon interval, the capabilities, and then elements, of which the name (0), the channel the
 * sender says it is on (3) and whether it announces RSN (48) are taken. Of the network that is
 * wanted, the access point heard best is kept whole.
 */
static void
heard(struct lx_radio *radio, const uint8_t *frame, unsigned length, unsigned frequency, int signal)
{
	struct lx_heard *network = NULL;
	const uint8_t *elements = frame + 36, *name = element(elements, length - 36, 0);

	lx_os_lock();
	if (name != NULL && radio->want_length != 0 && name[1] == radio->want_length &&
	    memcmp(name + 2, radio->want, radio->want_length) == 0 && (!radio->target.found || signal > radio->target.signal) &&
	    length - 36 <= ELEMENTS_MOST && radio->sta == NULL && !radio->target_fixed &&
	    (is_zero_ether_addr(radio->want_bssid) || memcmp(radio->want_bssid, frame + 16, ETH_ALEN) == 0)) {
		/*
		 * (Only the access point that was asked for, if one was, and only until the join has chosen: a stronger beacon of
		 * the same name must not replace the address, capability and elements of the one the join is made with.)
		 */
		radio->target.found = true;
		memcpy(radio->target.address, frame + 16, ETH_ALEN);
		radio->target.frequency = (uint16_t)frequency;
		radio->target.interval = (uint16_t)(frame[32] | frame[33] << 8);
		radio->target.capability = (uint16_t)(frame[34] | frame[35] << 8);
		radio->target.signal = (int8_t)signal;
		radio->target.elements_length = length - 36;
		memcpy(radio->target.elements, elements, length - 36);
	}
	for (unsigned i = 0; i < radio->heard_count && network == NULL; i++) {
		if (memcmp(radio->heard[i].sender, frame + 16, ETH_ALEN) == 0) {
			network = &radio->heard[i];
		}
	}
	if (network == NULL) {
		if (radio->heard_count == HEARD_MOST) {
			radio->heard_lost++;
			lx_os_unlock();
			return;
		}
		network = &radio->heard[radio->heard_count++];
		bzero(network, sizeof(*network));
		memcpy(network->sender, frame + 16, ETH_ALEN);
		network->signal = -128;
	}
	network->times++;
	network->last = lx_os_nanoseconds();
	network->interval = (uint16_t)(frame[32] | frame[33] << 8);
	network->capability = (uint16_t)(frame[34] | frame[35] << 8);
	if (length - 36 <= ELEMENTS_MOST) {
		network->elements_length = (uint16_t)(length - 36);
		memcpy(network->elements, elements, length - 36);
	}
	if (radio->sta != NULL && memcmp(radio->target.address, frame + 16, ETH_ALEN) == 0) {
		radio->signal = (int8_t)signal;
	}
	if (signal > network->signal) {
		network->signal = (int8_t)signal;
		network->frequency = (uint16_t)frequency;
	}
	if (name != NULL && name[1] <= sizeof(network->name) && (name[1] != 0 || network->name_length == 0)) {
		network->name_length = name[1];
		memcpy(network->name, name + 2, name[1]);
	}
	if (element(elements, length - 36, 3) != NULL && element(elements, length - 36, 3)[1] == 1) {
		network->says_channel = element(elements, length - 36, 3)[2];
	}
	network->protected = element(elements, length - 36, 48) != NULL;
	lx_os_unlock();
}

/* The driver's word that the scan it was asked for is over. */
void
linuxkpi_ieee80211_scan_completed(struct ieee80211_hw *hw, struct cfg80211_scan_info *info)
{
	struct lx_radio *radio = radio_of(hw);

	lx_os_lock();
	radio->scan = info != NULL && info->aborted ? 3 : 2;
	lx_os_wakeup(&radio->scan);
	lx_os_unlock();
}

/* What was heard, the strongest first. Names and addresses of other people's networks: for the log of the run only. */
static void
say_heard(struct lx_radio *radio)
{
	unsigned on_channel[200] = { 0 }, low = 0, high = 0, open = 0, nameless = 0;
	int strongest = -128, weakest = 127;
	char line[400] = "";
	bool said[HEARD_MOST] = { false };

	for (unsigned n = 0; n < radio->heard_count; n++) {
		const struct lx_heard *network = NULL;
		char name[33], address[18];
		unsigned which = 0, channel;
		bool wanted;

		for (unsigned i = 0; i < radio->heard_count; i++) {
			if (!said[i] && (network == NULL || radio->heard[i].signal > network->signal)) {
				network = &radio->heard[i];
				which = i;
			}
		}
		said[which] = true;
		for (unsigned i = 0; i < network->name_length; i++) {
			name[i] = network->name[i] >= 0x20 && network->name[i] < 0x7f ? (char)network->name[i] : '?';
		}
		name[network->name_length] = 0;
		channel = channel_of(network->frequency);
		if (channel < nitems(on_channel)) {
			on_channel[channel]++;
		}
		*(network->frequency < 4000 ? &low : &high) += 1;
		open += !(network->capability & 0x10);
		nameless += network->name_length == 0 || network->name[0] == 0;
		strongest = network->signal > strongest ? network->signal : strongest;
		weakest = network->signal < weakest ? network->signal : weakest;
		/* (the network that is wanted is the user's own: its name is not said even here) */
		wanted = radio->want_length != 0 && network->name_length == radio->want_length &&
		    memcmp(network->name, radio->want, radio->want_length) == 0;
		/* (nor is its address said: the BSSID of the user's own access point says where the user is) */
		snprintf(address, sizeof(address), "%02x:%02x:%02x:%02x:%02x:%02x", network->sender[0], network->sender[1],
		    network->sender[2], network->sender[3], network->sender[4], network->sender[5]);
		printf("lx: heard: channel %3u (%u MHz) %4d dBm  %s  %s, %u times  \"%s\"\n", channel,
		    network->frequency, network->signal, wanted ? "xx:xx:xx:xx:xx:xx" : address, !(network->capability & 0x10) ? "open" :
		    network->protected ? "RSN" : "protected", network->times, wanted ? "(the network that is wanted)" : name);
	}
	for (unsigned channel = 0; channel < nitems(on_channel); channel++) {
		if (on_channel[channel] != 0) {
			snprintf(line + strlen(line), sizeof(line) - strlen(line), " %u on %u,", on_channel[channel], channel);
		}
	}
	printf("lx: THE SCAN HEARD %u NETWORKS (access points): %u at 2.4 GHz, %u at 5 GHz; by channel:%s the strongest at %d dBm, "
	    "the weakest at %d; %u open, %u without a name; %u frames in all, %u of them neither beacon nor answer to a probe%s\n",
	    radio->heard_count, low, high, line, radio->heard_count ? strongest : 0, radio->heard_count ? weakest : 0, open, nameless,
	    radio->frames, radio->others, radio->heard_lost ? "; MORE WERE HEARD THAN THERE WAS ROOM FOR" : "");
}

/* ---- frames ------------------------------------------------------------------------------------------ */

/* A frame for the driver to send: `length` bytes at the returned place, in a buffer the radio can read. */
static struct sk_buff *
frame_new(struct lx_radio *radio, unsigned length, uint8_t **bytes)
{
	struct sk_buff *skb = dev_alloc_skb(length + 256);
	struct ieee80211_tx_info *info;

	if (skb == NULL) {
		return (NULL);
	}
	skb_reserve(skb, 128);			/* (room in front for what the driver puts there) */
	*bytes = skb_put(skb, length);
	info = IEEE80211_SKB_CB(skb);
	bzero(info, sizeof(*info));
	info->control.vif = radio->vif;
	info->band = radio->channel != NULL ? radio->channel->def.chan->band : NL80211_BAND_2GHZ;
	return (skb);
}

static void
frame_send(struct lx_radio *radio, struct sk_buff *skb)
{
	struct ieee80211_tx_control control = { .sta = radio->sta };
	/* ath11k encrypts robust unicast management, but mac80211 must supply its CCMP IV.
	 * Its WMI path appends the MIC space. Management nonces use priority 16, separate from data. */
	if (radio->wpa.pmf && radio->pairwise_key != NULL &&
	    !(IEEE80211_SKB_CB(skb)->flags & IEEE80211_TX_CTL_HW_80211_ENCAP) && lx_wpa_robust(skb->data, skb->len)) {
		uint64_t pn;
		lx_os_lock();
		pn = ++radio->management_tx_pn;
		lx_os_unlock();
		if (pn >= (1ull << 48)) {
			dev_kfree_skb_any(skb);
			return;
		}
		unsigned length = skb->len;
		skb_put(skb, 8);
		memmove(skb->data + 32, skb->data + 24, length - 24);
		uint8_t *iv = skb->data + 24;
		iv[0] = (uint8_t)pn;
		iv[1] = (uint8_t)(pn >> 8);
		iv[2] = 0;
		iv[3] = 0x20;
		for (unsigned i = 0; i < 4; i++) iv[4+i] = (uint8_t)(pn >> (16 + 8*i));
		skb->data[1] |= 0x40;
		IEEE80211_SKB_CB(skb)->control.hw_key = radio->pairwise_key;
	}

	radio->ops->tx(&radio->hw, &control, skb);
}

/* The 24 bytes a management frame to the access point begins with (the kind: 0xb0 authentication, 0x00 association request, ...). */
static uint8_t *
management(struct lx_radio *radio, uint8_t *frame, unsigned kind)
{
	bzero(frame, 24);
	frame[0] = (uint8_t)kind;
	memcpy(frame + 4, radio->target.address, ETH_ALEN);
	memcpy(frame + 10, radio->vif->addr, ETH_ALEN);
	memcpy(frame + 16, radio->target.address, ETH_ALEN);
	return (frame + 24);
}

/*
 * An Ethernet frame, through the driver's data path: the firmware makes 802.11 of it, and ciphers it once there is a key.
 * flags (board/lx.h): the traffic identifier the firmware queues it under (its WMM access class: dp_tx.c takes it from
 * skb->priority), and LX_TX_CSUM: the radio fills in the IPv4 header and TCP/UDP checksums (CHECKSUM_PARTIAL).
 */
static bool
ethernet_send(struct lx_radio *radio, const uint8_t *frame, unsigned length, unsigned flags)
{
	struct sk_buff *skb;
	uint8_t *bytes;

	if (radio->vif == NULL || radio->sta == NULL || (skb = frame_new(radio, length, &bytes)) == NULL) {
		radio->sent_lost++;
		return (false);
	}
	memcpy(bytes, frame, length);
	skb->priority = LX_TX_TID(flags);
	if (flags & LX_TX_CSUM) {
		skb->ip_summed = CHECKSUM_PARTIAL;
		radio->sent_offloaded++;
	}
	if (LX_TX_TID(flags) != 0) {
		radio->sent_classed++;
	}
	IEEE80211_SKB_CB(skb)->flags |= IEEE80211_TX_CTL_HW_80211_ENCAP;
	IEEE80211_SKB_CB(skb)->control.hw_key = radio->pairwise_key;
	radio->sent++;
	frame_send(radio, skb);
	return (true);
}

/*
 * An 802.11 data frame made into the Ethernet frame it carries, in place: the driver hands two
 * kinds over in 802.11 although the firmware had them as Ethernet -- the handshake's, and what
 * was sent to everybody and deciphered (dp_rx.c: ath11k_dp_rx_h_undecap()). The header is 24
 * bytes, more with a fourth address, the QoS field, the HT field and, if the driver left it
 * there, the cipher's 8 bytes; then 8 bytes of LLC and SNAP whose last two are the type.
 */
static bool
to_ethernet(struct sk_buff *skb, bool cipher_header_gone)
{
	const uint8_t *f = skb->data;
	unsigned control = f[0] | f[1] << 8, header = 24;
	uint8_t to[ETH_ALEN], from[ETH_ALEN], type[2], *e;

	if ((control & 0x000c) != 0x0008 || skb->len < 32) {
		return (false);
	}
	memcpy(to, control & 0x0100 ? f + 16 : f + 4, ETH_ALEN);
	memcpy(from, (control & 0x0300) == 0x0300 ? f + 24 : control & 0x0200 ? f + 16 : f + 10, ETH_ALEN);
	header += (control & 0x0300) == 0x0300 ? 6 : 0;
	header += control & 0x0080 ? 2 : 0;
	header += (control & 0x8080) == 0x8080 ? 4 : 0;
	header += (control & 0x4000) && !cipher_header_gone ? 8 : 0;
	if (skb->len < header + 8 || f[header] != 0xaa || f[header + 1] != 0xaa || f[header + 2] != 0x03) {
		return (false);
	}
	memcpy(type, f + header + 6, 2);
	e = skb_pull(skb, header + 8 - 14);
	memcpy(e, to, ETH_ALEN);
	memcpy(e + 6, from, ETH_ALEN);
	memcpy(e + 12, type, 2);
	return (true);
}

/* A message of the handshake from the access point: answered at once; the keys are for join() to give to the driver. */
static void
handshake_message(struct lx_radio *radio, const uint8_t *frame, unsigned length)
{
	uint8_t reply[14 + LX_WPA_REPLY_MOST];
	unsigned reply_length = 0;
	const char *why = "";
	int said;

	radio->eapol++;
	if (radio->sta == NULL || memcmp(frame + 6, radio->target.address, ETH_ALEN) != 0) {
		return;
	}
	lx_os_lock();
	said = lx_wpa_message(&radio->wpa, frame + 14, length - 14, reply + 14, &reply_length, &why);
	lx_os_unlock();
	if (reply_length != 0) {
		memcpy(reply, radio->target.address, ETH_ALEN);
		memcpy(reply + 6, radio->vif->addr, ETH_ALEN);
		reply[12] = 0x88;
		reply[13] = 0x8e;
		ethernet_send(radio, reply, 14 + reply_length, 0);
	}
	printf("lx: the handshake: a message of %u bytes from the access point: %s%s\n", length - 14, said == LX_WPA_ANSWER ?
	    "its first; answered with ours (message 2)" : said == LX_WPA_KEYS ? "its second, SIGNED WITH THE KEY WE HAVE TOO, and "
	    "with the group's key; answered (message 4)" : said == LX_WPA_GROUP ? "a new key for the group; answered" :
	    said == LX_WPA_BAD ? "NOT GOOD: " : "not one to answer", why);
	if (said == LX_WPA_GROUP && radio->joined && radio->wpa.group_updated) {
		queue_work(radio->work, &radio->group_work);
	}
	if (said != LX_WPA_NOTHING) {
		/*
		 * The result of the handshake is not last-writer-wins: once message 3 has passed its MIC (KEYS) a later frame -- an
		 * unsigned one from anyone in range can say BAD or ANSWER -- must not hide it, or the join would time out although
		 * the keys are agreed (IEEE 802.11i, 8.5.3: only a verified message counts).
		 */
		lx_os_lock();
		if (radio->handshake != LX_WPA_KEYS) {
			radio->handshake = said;
			radio->handshake_why = why;
		}
		lx_os_wakeup(&radio->handshake);
		lx_os_unlock();
	}
}

static unsigned rx_broadcast;	/* (frames to ff:ff:ff:ff:ff:ff among those to all) */
/*
 * What the host received in the 500 ms after a wake (run73: 75 of 75 wakes "unspecified", most dozes ending ~80 ms
 * after D3hot, whatever the delay before it -- if the firmware wakes us to hand frames over, they are the reason), and
 * how long the doze's entry took. Said at the next doze's entry, for the wakes doze_seen_say() says.
 */
static struct {
	unsigned long long left_at, enter_ns;
	unsigned wake, n, more;
	char what[5][44];
} after;
static void
after_note(const uint8_t *f, unsigned length)
{
	unsigned long long now;
	unsigned type;
	int n;

	if (after.left_at == 0 || length < 14) return;
	now = lx_os_nanoseconds();
	if (now - after.left_at > 500000000ull) return;
	if (after.n >= 5) { after.more++; return; }
	type = (unsigned)f[12] << 8 | f[13];
	n = snprintf(after.what[after.n], sizeof(after.what[0]), "+%llums %s %04x", (now - after.left_at) / 1000000ull,
	    !(f[0] & 1) ? "us" : f[0] == 0xff ? "bc" : "mc", type);
	if (type == 0x0800 && length >= 38)
		snprintf(after.what[after.n] + n, sizeof(after.what[0]) - (size_t)n, " p%u %u.%u.%u.%u:%u", f[23], f[30], f[31], f[32],
		    f[33], (unsigned)f[36] << 8 | f[37]);
	else if (type == 0x0806 && length >= 42)
		snprintf(after.what[after.n] + n, sizeof(after.what[0]) - (size_t)n, " for %u.%u.%u.%u", f[38], f[39], f[40], f[41]);
	after.n++;
}
static void
after_say(void)
{
	char line[300];
	int at;

	if (after.left_at == 0 || (after.wake > 16 && (after.wake & (after.wake - 1)) != 0)) return;
	at = snprintf(line, sizeof(line), "lx: after wake %u the host received:", after.wake);
	for (unsigned i = 0; i < after.n && at < (int)sizeof(line) - 48; i++)
		at += snprintf(line + at, sizeof(line) - (size_t)at, " [%s]", after.what[i]);
	if (after.n == 0) at += snprintf(line + at, sizeof(line) - (size_t)at, " nothing in 500 ms");
	if (after.more) at += snprintf(line + at, sizeof(line) - (size_t)at, " (+%u more)", after.more);
	snprintf(line + at, sizeof(line) - (size_t)at, "; the next doze took %llu ms to enter", after.enter_ns / 1000000ull);
	lx_tell("%s\n", line);
}

/* The station and the interface taken for a frame received or sent, or false while they are being taken down */
static bool
station_use(struct lx_radio *radio)
{
	__atomic_add_fetch(&radio->station_users, 1, __ATOMIC_SEQ_CST);
	if (__atomic_load_n(&radio->station_closing, __ATOMIC_SEQ_CST)) {
		__atomic_sub_fetch(&radio->station_users, 1, __ATOMIC_SEQ_CST);
		return false;
	}
	return true;
}

static void
station_done(struct lx_radio *radio)
{
	__atomic_sub_fetch(&radio->station_users, 1, __ATOMIC_SEQ_CST);
}

/* Before the station or the interface is freed: no frame starts acting on them, and those that had, have finished */
static void
station_close(struct lx_radio *radio)
{
	__atomic_store_n(&radio->station_closing, true, __ATOMIC_SEQ_CST);
	for (unsigned ms = 0; __atomic_load_n(&radio->station_users, __ATOMIC_SEQ_CST) != 0 && ms < 1000; ms++) {
		lx_pause_ms(1);
	}
}

static void
station_open(struct lx_radio *radio)
{
	__atomic_store_n(&radio->station_closing, false, __ATOMIC_SEQ_CST);
}

/* An Ethernet frame the radio has received; FLAGS: the driver's description of it (RX_FLAG_...). */
static void
ethernet_received(struct lx_radio *radio, struct sk_buff *skb, unsigned flags)
{
	if (skb->len >= 14 && skb->data[12] == 0x88 && skb->data[13] == 0x8e) {
		if (station_use(radio)) {
			handshake_message(radio, skb->data, skb->len);
			station_done(radio);
		}
	} else if (radio->joined && skb->len >= 14) {
		/*
		 * Once the keys are set the network's frames come ciphered, and the driver says it deciphered them (ath11k: dp_rx.c,
		 * ath11k_dp_rx_h_mpdu()); one that came in the clear can have been sent by anyone in range. Only the handshake's,
		 * above, may (mac80211 drops the others the same way: ieee80211_drop_unencrypted()). An open network has no keys.
		 */
		if (radio->pairwise_key != NULL && !(flags & RX_FLAG_DECRYPTED)) {
			radio->unciphered++;
			return;
		}
		*(skb->data[0] & 1 ? &radio->received_group : &radio->received) += 1;
		after_note(skb->data, skb->len);
		if (memcmp(skb->data, "\xff\xff\xff\xff\xff\xff", 6) == 0) rx_broadcast++;
		/* (only a frame for us keeps the radio awake: the LAN's broadcasts and multicasts -- mDNS, SSDP, other hosts' ARP --
		 * arrive several a second and would hold off every doze; one that needs an answer makes us send, which counts) */
		if (!(skb->data[0] & 1)) doze.active_at = lx_os_nanoseconds();
		/* (dp_rx.c's attention bits: CHECKSUM_UNNECESSARY when neither the IP nor the TCP/UDP check failed) */
		radio->received_checked += skb->ip_summed == CHECKSUM_UNNECESSARY;
		lx_os_net_input(skb->data, skb->len, skb->ip_summed == CHECKSUM_UNNECESSARY);
	}
}

/* A management frame from the access point of the network being joined. */
static void
management_received(struct lx_radio *radio, uint8_t *f, unsigned length, const struct ieee80211_rx_status *rx)
{
	int answer = ANSWER_NONE;
	unsigned status = 0;

	if (radio->sta == NULL || length < 26 || memcmp(f + 10, radio->target.address, ETH_ALEN) != 0 ||
	    memcmp(f + 16, radio->target.address, ETH_ALEN) != 0 ||
	    (memcmp(f + 4, radio->vif->addr, ETH_ALEN) != 0 && !(f[4] & 1))) {
		return;
	}
	if (radio->wpa.pmf && radio->wpa.replay_known &&
	    (lx_wpa_robust(f, length) || (f[0] == 0xd0 && (f[1] & 0x40)))) {
		int verified;
		lx_os_lock();
		verified = lx_wpa_pmf_rx(&radio->wpa, f, &length, rx->flag & RX_FLAG_DECRYPTED,
		    rx->flag & RX_FLAG_IV_STRIPPED, rx->flag & RX_FLAG_MIC_STRIPPED, rx->flag & RX_FLAG_PN_VALIDATED);
		lx_os_unlock();
		if (verified != 0) return; /* spoofed, replayed or unprotected robust frame */
	}
	if ((f[4] & 1) && (!radio->wpa.pmf || !radio->wpa.replay_known || !lx_wpa_robust(f, length))) return;
	if (f[0] == 0xd0 && length >= 28 && f[24] == 8 && radio->wpa.pmf &&
	    radio->pairwise_key != NULL && !(f[4] & 1)) {
		/* SA Query is host MLME work, not firmware offload. Echo only an authenticated request. */
		if (f[25] == 0) {
			uint8_t *body;
			struct sk_buff *reply = frame_new(radio, 28, &body);
			if (reply != NULL) {
				body = management(radio, body, 0xd0);
				body[0] = 8;
				body[1] = 1;
				memcpy(body + 2, f + 26, 2);
				frame_send(radio, reply);
			}
		}
		return;
	}
	if (f[0] == 0xb0 && length >= 30 && radio->sae_listening) {
		if (f[24] != 3 || f[25] != 0 || (f[4] & 1) || length - 24 > sizeof(radio->sae_rx)) return;
		lx_os_lock();
		if (radio->sae_listening && radio->sae_rx_length == 0) {
			memcpy(radio->sae_rx, f + 24, length - 24);
			radio->sae_rx_length = length - 24;
			lx_os_wakeup(&radio->sae_rx_length);
		}
		lx_os_unlock();
		return;
	}
	if (f[0] == 0xb0 && length >= 30) {		/* open-system authentication only */
		if (f[24] != 0 || f[25] != 0 || (f[4] & 1)) return;
		status = f[28] | f[29] << 8;
		answer = (f[26] | f[27] << 8) != 2 ? ANSWER_NONE : status == 0 ? ANSWER_AUTHENTICATED : ANSWER_REFUSED;
	} else if (f[0] == 0x10 && length >= 30) {	/* association response: capabilities, status, our number, elements */
		status = f[26] | f[27] << 8;
		answer = status == 0 ? ANSWER_ASSOCIATED : ANSWER_REFUSED;
		lx_os_lock();
		radio->aid = (uint16_t)((f[28] | f[29] << 8) & 0x3fff);
		radio->answer_length = length - 30 < ELEMENTS_MOST ? length - 30 : ELEMENTS_MOST;
		memcpy(radio->answer_elements, f + 30, radio->answer_length);
		lx_os_unlock();
	} else if (f[0] == 0xd0 && length >= 28 && f[24] == 3 && radio->reached >= IEEE80211_STA_ASSOC && lx_os_option("wifi_ba")) {
		/* an action frame, block ack's: ADDBA request (0) or DELBA (2); done on the driver's terms in ba_handle() */
		unsigned tid;

		lx_os_lock();
		if (f[25] == 0 && length >= 33) {
			unsigned params = f[27] | f[28] << 8;

			tid = params >> 2 & 7;
			radio->ba[tid].start = true;
			radio->ba[tid].amsdu = params & 1;
			radio->ba[tid].size = (uint16_t)(params >> 6);
			radio->ba[tid].token = f[26];
			radio->ba[tid].timeout = (uint16_t)(f[29] | f[30] << 8);
			radio->ba[tid].ssn = (uint16_t)((f[31] | f[32] << 8) >> 4);
			radio->ba_asked |= (uint8_t)(1u << tid);
		} else if (f[25] == 2) {
			tid = (f[26] | f[27] << 8) >> 12 & 7;
			radio->ba[tid].start = false;
			radio->ba_asked |= (uint8_t)(1u << tid);
		}
		lx_os_unlock();
		queue_work(radio->work, &radio->ba_work);
		return;
	} else if (f[0] == 0xc0 || f[0] == 0xa0) {	/* the access point shows us the door: with a reason */
		status = f[24] | f[25] << 8;
		answer = ANSWER_GONE;
		lx_tell("lx: THE ACCESS POINT %s US, reason %u\n", f[0] == 0xc0 ? "DEAUTHENTICATES" : "DISASSOCIATES", status);
		if (radio->joined) {
			radio->joined = false;
			radio->last_error = ECONNRESET;
			radio->loss = f[0] == 0xc0 ? HAKIOS_STA_LOSS_DEAUTH : HAKIOS_STA_LOSS_DISASSOC;
			radio->loss_reason = (uint16_t)status;
			lx_os_net_link(0);
			lx_os_station_lost(ECONNRESET);
		}
	}
	if (answer != ANSWER_NONE) {
		lx_os_lock();
		radio->answer = answer;
		radio->answer_status = (uint16_t)status;
		lx_os_wakeup(&radio->answer);
		lx_os_unlock();
	}
}

/*
 * A frame the radio has received, with the driver's description behind it (where, how strong,
 * and in which form: an Ethernet frame, or 802.11 as it was on the air).
 */
/*
 * What the data frames come in at, as the firmware reports each (WIFI-PLAN.md P1c): every RX_RATES_EVERY of them one line
 * with the three commonest rates (encoding, MCS or legacy index, streams), then counted afresh (the
 * Ethernet-decap path carries no signal: that is the station's RSSI, polled).
 */
#define RX_RATES_EVERY	20000u
static struct {
	unsigned count[4][2][12];		/* [legacy, HT, VHT, HE][1 or 2 streams][MCS / legacy rate index] */
	unsigned frames, width[4];
} rx_rates;

static void
rx_rate_note(const struct ieee80211_rx_status *status)
{
	static const char *const kind[4] = { "legacy", "HT", "VHT", "HE" };
	unsigned e = status->encoding < 4 ? status->encoding : 0, s = status->nss >= 2 ? 1 : 0, m = status->rate_idx < 12 ?
	    status->rate_idx : 11;
	char line[160] = "";

	rx_rates.count[e][s][m]++;
	rx_rates.width[status->bw < 4 ? status->bw : 3]++;
	if (++rx_rates.frames < RX_RATES_EVERY) {
		return;
	}
	for (unsigned pick = 0; pick < 3; pick++) {
		unsigned best = 0, be = 0, bs = 0, bm = 0;

		for (unsigned i = 0; i < 4; i++) {
			for (unsigned j = 0; j < 2; j++) {
				for (unsigned k = 0; k < 12; k++) {
					if (rx_rates.count[i][j][k] > best) {
						best = rx_rates.count[i][j][k], be = i, bs = j, bm = k;
					}
				}
			}
		}
		if (best == 0) {
			break;
		}
		snprintf(line + strlen(line), sizeof(line) - strlen(line), " %s %s%u x%u %u%%,", kind[be], be ? "MCS" : "#", bm, bs + 1,
		    best * 100 / rx_rates.frames);
		rx_rates.count[be][bs][bm] = 0;
	}
	lx_tell("lx: the last %u data frames came in at:%s widths 20/40/80/160: %u/%u/%u/%u%%\n",
	    rx_rates.frames, line, rx_rates.width[0] * 100 / rx_rates.frames, rx_rates.width[1] * 100 / rx_rates.frames,
	    rx_rates.width[2] * 100 / rx_rates.frames, rx_rates.width[3] * 100 / rx_rates.frames);
	bzero(&rx_rates, sizeof(rx_rates));
}

/*
 * Data frames given as 802.11 (not 802.3), by what was on the air: addr1 a group (the AP's group-addressed frames, under
 * the group key) or ours; for ours, the DA a group all the same (an AP's multicast-to-unicast: A-MSDU subframes or 4
 * addresses); A-MSDU; 4 addresses; decrypted by the driver. run69/70: multicast reached us, broadcast never -- were those
 * multicasts group-addressed at all?
 */
static unsigned rx_8023_group, rx_air_group, rx_air_group_bc, rx_air_ours, rx_air_ours_group_da, rx_air_amsdu, rx_air_4addr, rx_air_decrypted;
static void
rx_air_note(const uint8_t *f, const struct ieee80211_rx_status *status)
{
	unsigned control = f[0] | f[1] << 8;
	bool four = (control & 0x0300) == 0x0300;
	const uint8_t *da = control & 0x0100 ? f + 16 : f + 4;

	if (f[4] & 1) {
		rx_air_group++;
		if (memcmp(f + 4, "\xff\xff\xff\xff\xff\xff", 6) == 0) rx_air_group_bc++;
	} else {
		rx_air_ours++;
		if (da[0] & 1) rx_air_ours_group_da++;
	}
	if (four) rx_air_4addr++;
	if ((control & 0x0080) && (f[24 + (four ? 6 : 0)] & 0x80)) rx_air_amsdu++;
	if (status->flag & RX_FLAG_DECRYPTED) rx_air_decrypted++;
}
static unsigned rx_8021_ok, rx_dropped_data, rx_dropped_len;
static uint32_t rx_dropped_flags;
static uint8_t rx_dropped_head[40];
void
linuxkpi_ieee80211_rx(struct ieee80211_hw *hw, struct sk_buff *skb, struct ieee80211_sta *sta, struct napi_struct *napi,
    struct list_head *list)
{
	struct lx_radio *radio = radio_of(hw);
	const struct ieee80211_rx_status *status = IEEE80211_SKB_RXCB(skb);

	radio->frames++;
	if (status->flag & RX_FLAG_8023) {
		rx_rate_note(status);
	}
	if (status->flag & RX_FLAG_MMIC_ERROR) {
		/* ath11k detected a Michael MIC failure. Never deliver the corrupt packet or
		 * implement a second countermeasure state machine beside the firmware. */
		lx_tell("lx: MICHAEL_MIC_FAILURE: firmware RX status, packet dropped\n");
		dev_kfree_skb_any(skb);
		return;
	}
	if (status->flag & RX_FLAG_8023) {
		if (skb->len >= 14 && (skb->data[0] & 1)) rx_8023_group++;
		ethernet_received(radio, skb, status->flag);
	} else if (skb->len >= 36 && (skb->data[0] == 0x80 || skb->data[0] == 0x50)) {	/* a beacon, an answer to a probe */
		heard(radio, skb->data, skb->len, status->freq, status->signal);
	} else if (skb->len >= 24 && (skb->data[0] & 0x0c) == 0x00) {
		radio->others++;
		if (station_use(radio)) {
			management_received(radio, skb->data, skb->len, status);
			station_done(radio);
		}
	} else if ((skb->len >= 24 && (skb->data[0] & 0x0c) == 0x08 && (rx_air_note(skb->data, status), false)) ||
	    to_ethernet(skb, (status->flag & RX_FLAG_IV_STRIPPED) != 0)) {
		rx_8021_ok++;
		ethernet_received(radio, skb, status->flag);
	} else {
		radio->others++;
		/* (diagnosis: a data frame to_ethernet() would not take; the first one's header and flags) */
		if ((skb->data[0] & 0x0c) == 0x08 && rx_dropped_data++ == 0) {
			rx_dropped_flags = status->flag;
			rx_dropped_len = skb->len;
			memcpy(rx_dropped_head, skb->data, skb->len < sizeof(rx_dropped_head) ? skb->len : sizeof(rx_dropped_head));
		}
	}
	dev_kfree_skb_any(skb);
}

/* ---- the station: an interface, a scan ---------------------------------------------------------------- */

/* The radio started and an interface in station mode on it, as mac80211 brings one up. 0, or an errno. */
static int
station_up(struct lx_radio *radio)
{
	struct ieee80211_hw *hw = &radio->hw;
	struct ieee80211_vif *vif;
	int ret;

	if (radio->vif != NULL) {
		return (0);
	}
	if (is_zero_ether_addr(hw->wiphy->perm_addr)) {
		lx_tell("lx: THE RADIO HAS NO ADDRESS (the kernel was given none, and the firmware has none): no interface is made\n");
		return (EADDRNOTAVAIL);
	}
	radio->vif_memory = kzalloc(64 + sizeof(*vif) + hw->vif_data_size, GFP_KERNEL);
	if (radio->vif_memory == NULL) {
		return (ENOMEM);
	}
	vif = (struct ieee80211_vif *)roundup2((uintptr_t)radio->vif_memory, 64);
	vif->type = NL80211_IFTYPE_STATION;
	memcpy(vif->addr, hw->wiphy->perm_addr, ETH_ALEN);
	memcpy(vif->bss_conf.addr, hw->wiphy->perm_addr, ETH_ALEN);
	vif->bss_conf.vif = vif;
	vif->bss_conf.bssid = no_network;	/* (the driver compares every beacon's sender with it) */
	vif->link_conf[0] = &vif->bss_conf;
	/* the firmware is to take Ethernet frames and to give them, if the driver offers that (its frame mode) */
	if (ieee80211_hw_check(hw, SUPPORTS_TX_ENCAP_OFFLOAD)) {
		vif->offload_flags |= IEEE80211_OFFLOAD_ENCAP_ENABLED;
	}
	/*
	 * Received frames: 802.11 (native Wi-Fi, upstream ath11k's default and what every Linux station runs), unless
	 * wifi_rx_eth. run69..72: with the firmware taking them out of 802.11 (RX_DECAP_TYPE Ethernet) no broadcast frame
	 * ever reached the host -- multicast did, the access point sending both alike (no proxy ARP) and ath11k dropping none.
	 */
	if (ieee80211_hw_check(hw, SUPPORTS_RX_DECAP_OFFLOAD) && lx_os_option("wifi_rx_eth")) {
		vif->offload_flags |= IEEE80211_OFFLOAD_DECAP_ENABLED;
	}

	mutex_lock(&hw->wiphy->mtx);
	ret = radio->ops->start(hw);
	printf("lx: the radio is started: %d\n", ret);
	if (ret == 0) {
		radio->started = true;
		ret = radio->ops->add_interface(hw, vif);
		printf("lx: an interface in station mode is added to it: %d (Ethernet frames %s the firmware, %s it)\n", ret,
		    vif->offload_flags & IEEE80211_OFFLOAD_ENCAP_ENABLED ? "to" : "NOT to", vif->offload_flags &
		    IEEE80211_OFFLOAD_DECAP_ENABLED ? "from" : "NOT from");
		if (ret == 0) {
			radio->vif = vif;
		}
	}
	mutex_unlock(&hw->wiphy->mtx);
	if (ret == 0 && lx_os_radio_model() == 1) lx_mhi_idle_start();
	return (-ret);
}

static void
station_down(struct lx_radio *radio)
{
	struct ieee80211_hw *hw = &radio->hw;

	station_close(radio);           /* (the receive path reads the interface's address) */
	mutex_lock(&hw->wiphy->mtx);
	if (radio->vif != NULL) {
		radio->ops->remove_interface(hw, radio->vif);
		radio->vif = NULL;
	}
	if (radio->started) {
		radio->ops->stop(hw, false);
		radio->started = false;
	}
	mutex_unlock(&hw->wiphy->mtx);
	kfree(radio->vif_memory);
	radio->vif_memory = NULL;
	station_open(radio);
	printf("lx: the interface is removed and the radio stopped\n");
}

/*
 * A scan of every channel the regulatory domain allows, or of those `what` names. With a name
 * the radio asks for that network on every channel where it may send (a probe request; and the
 * access points that hear it answer); without one it asks for any network there, or, if it
 * is to be `quiet`, only listens. 0 when the driver has said the scan is over.
 */
static int
station_scan(struct lx_radio *radio, const uint8_t *name, unsigned name_length, bool quiet, const struct hakios_sta_scan *what)
{
	struct ieee80211_hw *hw = &radio->hw;
	struct ieee80211_scan_request *request;
	struct cfg80211_ssid asked;
	unsigned long long began;
	int channels = 0, ret;

	for (unsigned band = 0; band < NUM_NL80211_BANDS; band++) {
		channels += hw->wiphy->bands[band] != NULL ? hw->wiphy->bands[band]->n_channels : 0;
	}
	request = kzalloc(sizeof(*request) + channels * sizeof(request->req.channels[0]), GFP_KERNEL);
	if (request == NULL) {
		return (ENOMEM);
	}
	request->req.wiphy = hw->wiphy;
	for (unsigned band = 0; band < NUM_NL80211_BANDS; band++) {
		struct ieee80211_supported_band *has = hw->wiphy->bands[band];

		for (int i = 0; has != NULL && i < has->n_channels; i++) {
			bool wanted = what == NULL || what->frequency_count == 0;

			for (unsigned k = 0; what != NULL && k < what->frequency_count && !wanted; k++) {
				wanted = what->frequencies[k] == has->channels[i].center_freq;
			}
			if (wanted && !(has->channels[i].flags & IEEE80211_CHAN_DISABLED)) {
				request->req.channels[request->req.n_channels++] = &has->channels[i];
			}
		}
	}
	if (request->req.n_channels == 0) {
		kfree(request);
		return (EINVAL);
	}
	if (!quiet) {				/* (a name of no length asks for every network) */
		bzero(&asked, sizeof(asked));
		memcpy(asked.ssid, name, name_length);
		asked.ssid_len = (uint8_t)name_length;
		request->req.ssids = &asked;
		request->req.n_ssids = 1;
	}
	if (what != NULL && what->dwell_ms != 0) {
		request->req.duration = what->dwell_ms;
	}
	radio->scan = 1;
	radio->frames = radio->others = radio->heard_count = radio->heard_lost = 0;
	began = lx_os_nanoseconds();
	mutex_lock(&hw->wiphy->mtx);
	ret = radio->ops->hw_scan(hw, radio->vif, request);
	mutex_unlock(&hw->wiphy->mtx);
	printf("lx: THE DRIVER IS ASKED TO SCAN %d channels, %s: %d\n", request->req.n_channels, quiet ? "listening only" :
	    name_length != 0 ? "asking for one network by name where it may send" : "asking for any network where it may send", ret);
	if (ret == 0) {
		lx_os_lock();
		while (radio->scan == 1 && lx_os_nanoseconds() - began < 40000000000ull) {
			lx_os_sleep(&radio->scan, 100 * 1000000ull);
		}
		lx_os_unlock();
		printf("lx: %llu ms later the scan is %s\n", (lx_os_nanoseconds() - began) / 1000000ull, radio->scan == 2 ? "OVER" :
		    radio->scan == 3 ? "GIVEN UP by the driver" : "NOT OVER: the driver has not said so");
		say_heard(radio);
		if (radio->scan == 1 && radio->ops->cancel_hw_scan != NULL) {
			mutex_lock(&hw->wiphy->mtx);
			radio->ops->cancel_hw_scan(hw, radio->vif);
			mutex_unlock(&hw->wiphy->mtx);
		}
	}
	kfree(request);
	return (ret != 0 ? -ret : radio->scan == 2 ? 0 : ETIMEDOUT);
}

/* An interface, a scan that only listens, and down again: what the radio hears is said. 0, or an errno. */
int
lx_station_scan(void)
{
	struct lx_radio *radio = the_radio;
	int error;

	if (radio == NULL || !radio->registered) {
		printf("lx: no radio is registered: nothing to scan with\n");
		return (ENXIO);
	}
	if ((error = station_up(radio)) == 0) {
		error = station_scan(radio, NULL, 0, true, NULL);
	}
	station_down(radio);
	return (error);
}

/* ---- the station: block ack -------------------------------------------------------------------------- */

/*
 * The access point's requests to send us frames in aggregates (A-MPDU) for a traffic class, and to
 * stop doing so, as mac80211 handles them (net/mac80211/agg-rx.c): the driver told (its
 * ampdu_action, which sets up the firmware's reorder queue for the class), then the answer, ADDBA
 * response, with the window size the driver can take. Without this every frame goes on its own
 * and the link carries about a third of what it could. On the driver's work queue: the driver's
 * call sends WMI commands, which the thread that brought the request must be free to see answered.
 */
static void
ba_handle(struct work_struct *work)
{
	struct lx_radio *radio = __containerof(work, struct lx_radio, ba_work);
	struct ieee80211_hw *hw = &radio->hw;

	for (unsigned tid = 0; tid < 8; tid++) {
		struct ieee80211_ampdu_params params = { .sta = radio->sta, .tid = (uint8_t)tid };
		struct sk_buff *skb;
		uint8_t *frame;
		bool asked;
		int ret = 0;

		lx_os_lock();
		asked = (radio->ba_asked & 1u << tid) != 0;
		radio->ba_asked &= (uint8_t)~(1u << tid);
		params.ssn = radio->ba[tid].ssn;
		params.timeout = radio->ba[tid].timeout;
		params.amsdu = radio->ba[tid].amsdu;
		params.buf_size = radio->ba[tid].size == 0 || radio->ba[tid].size > hw->max_rx_aggregation_subframes ?
		    hw->max_rx_aggregation_subframes : radio->ba[tid].size;
		lx_os_unlock();
		if (!asked || radio->sta == NULL || radio->reached < IEEE80211_STA_ASSOC) {
			continue;
		}
		mutex_lock(&hw->wiphy->mtx);
		if (radio->ba[tid].active) {			/* a new request for a class that has one: the old one goes first */
			params.action = IEEE80211_AMPDU_RX_STOP;
			radio->ops->ampdu_action(hw, radio->vif, &params);
			radio->ba[tid].active = false;
		}
		if (radio->ba[tid].start) {
			params.action = IEEE80211_AMPDU_RX_START;
			ret = radio->ops->ampdu_action(hw, radio->vif, &params);
			radio->ba[tid].active = ret == 0;
		}
		mutex_unlock(&hw->wiphy->mtx);
		if (!radio->ba[tid].start) {
			continue;
		}
		radio->ba_sessions += ret == 0;
		radio->ba_refused += ret != 0;
		lx_tell("lx: BLOCK ACK for traffic class %u: the access point asks to send aggregates of up to %u frames; the driver "
		    "takes %u: %d\n", tid, radio->ba[tid].size, params.buf_size, ret);
		if ((skb = frame_new(radio, 24 + 9, &frame)) == NULL) {
			continue;
		}
		frame = management(radio, frame, 0xd0);
		frame[0] = 3;				/* block ack */
		frame[1] = 1;				/* ADDBA response */
		frame[2] = radio->ba[tid].token;
		frame[3] = ret == 0 ? 0 : 37;		/* success, or "the request has been declined" */
		frame[4] = 0;
		{
			unsigned answer = (params.amsdu ? 1u : 0u) | 2u | tid << 2 | (unsigned)params.buf_size << 6;

			frame[5] = (uint8_t)answer;
			frame[6] = (uint8_t)(answer >> 8);
		}
		frame[7] = (uint8_t)params.timeout;
		frame[8] = (uint8_t)(params.timeout >> 8);
		frame_send(radio, skb);
	}
}

/* ---- the station: joining a network -------------------------------------------------------------------- */

/* Wait for the access point's answer to a management frame: one of the ANSWER_s, or ANSWER_NONE when the time is up. */
static int
answer_wait(struct lx_radio *radio, unsigned ms)
{
	unsigned long long began = lx_os_nanoseconds();
	int answer;

	lx_os_lock();
	while (radio->answer == ANSWER_NONE && lx_os_nanoseconds() - began < ms * 1000000ull) {
		lx_os_sleep(&radio->answer, 20 * 1000000ull);
	}
	answer = radio->answer;
	lx_os_unlock();
	return (answer);
}

static int
sae_entropy(void *context, uint8_t *bytes, size_t length)
{
	(void)context;
	get_random_bytes(bytes, length);
	return 0;
}

/* Sleep/RX only in the mailbox critical section; ECC never blocks the poll
 * thread or the global OS lock. Token retries and retransmits have one bound. */
static int
sae_authenticate(struct lx_radio *radio, const struct hakios_sta_join *what)
{
	uint8_t body[400], received[400];
	unsigned length, transaction=1, token_tries=0;
	int error=ETIMEDOUT;
	if (lx_sae_start(&radio->sae,radio->vif->addr,radio->target.address,
	    what->ssid,what->ssid_length,what->passphrase,what->passphrase_length,
	    radio->sae_h2e,sae_entropy,NULL)) return EAUTH;
	lx_os_lock(); radio->answer=ANSWER_NONE; radio->sae_rx_length=0; radio->sae_listening=true; lx_os_unlock();
	length=lx_sae_write_commit(&radio->sae,body);
	for (unsigned attempt=0;attempt<8;attempt++) {
		struct sk_buff *skb; uint8_t *frame;
		if ((skb=frame_new(radio,30+length,&frame))==NULL) { error=ENOMEM;break; }
		frame=management(radio,frame,0xb0);
		frame[0]=3;frame[1]=0;frame[2]=(uint8_t)transaction;frame[3]=0;
		frame[4]=(transaction==1&&radio->sae_h2e)?126:0;frame[5]=0;memcpy(frame+6,body,length);
		frame_send(radio,skb);
		uint64_t deadline=lx_os_nanoseconds()+1000000000ull;
		lx_os_lock();
		while(!radio->sae_rx_length && lx_os_nanoseconds()<deadline && radio->answer!=ANSWER_GONE)
			lx_os_sleep(&radio->sae_rx_length,20*1000000ull);
		unsigned n=radio->sae_rx_length;
		if(n) memcpy(received,radio->sae_rx,n);
		radio->sae_rx_length=0;lx_os_unlock();
		if(radio->answer==ANSWER_GONE) { error=ECONNRESET;break; }
		if(!n) continue;
		unsigned seq=received[2]|received[3]<<8, status=received[4]|received[5]<<8;
		if(seq==1 && status==76 && transaction==1) {
			if(++token_tries>3 || lx_sae_token(&radio->sae,received+6,n-6)) { error=EAUTH;break; }
			length=lx_sae_write_commit(&radio->sae,body);continue;
		}
		if(seq==1 && status==(radio->sae_h2e?126u:0u)) {
			if(lx_sae_peer_commit(&radio->sae,received+6,n-6)) continue;
			if(lx_sae_write_confirm(&radio->sae,body)) { error=EAUTH;break; }
			transaction=2;length=LX_SAE_CONFIRM;continue;
		}
		if(seq==2 && status==0 && transaction==2) {
			if(lx_sae_check_confirm(&radio->sae,received+6,n-6)) continue;
			memcpy(radio->wpa.pmk,radio->sae.pmk,32);
			/* Cache only after the 4-way handshake and driver key install. */
			error=0;break;
		}
		if(status!=0 && status!=126) { error=ECONNREFUSED;break; }
	}
	lx_os_lock(); radio->sae_listening=false; radio->sae_rx_length=0; lx_os_unlock();
	lx_crypto_wipe(body,sizeof(body));lx_crypto_wipe(received,sizeof(received));
	if(error) lx_sae_clear(&radio->sae);
	return error;
}

/* The rates among `count` bytes of a rates element that the band has, as bits in the band's own order; and the "basic" ones. */
static uint32_t
rates_of(const struct ieee80211_supported_band *band, const uint8_t *rates, unsigned count, uint32_t *basic)
{
	uint32_t has = 0;

	for (unsigned i = 0; i < count; i++) {
		for (int r = 0; r < band->n_bitrates; r++) {
			if (band->bitrates[r].bitrate == (rates[i] & 0x7f) * 5u) {	/* (half megabits there, tenths here) */
				has |= 1u << r;
				*basic |= rates[i] & 0x80 ? 1u << r : 0;
			}
		}
	}
	return (has);
}

/*
 * What the access point says it can do, into the form the driver reads it in when it tells the
 * firmware about the peer (mac.c: ath11k_peer_assoc_prepare()): rates, the HT and VHT
 * capabilities, WMM. `elements` are the association response's, or the beacon's before that.
 */
/* The elements as they are, one line each: number (extension number), length, bytes (vendor ones by their OUI and type). */
static void
elements_say(const char *whose, const uint8_t *elements, unsigned length)
{
	for (unsigned at = 0; at + 2 <= length && at + 2 + elements[at + 1] <= length; at += 2 + elements[at + 1]) {
		char line[220];
		int n = snprintf(line, sizeof(line), "lx: %s: element %u length %u:", whose, elements[at], elements[at + 1]);
		if (elements[at] == 0) {	/* (the name: not said) */
			lx_tell("%s (the name)\n", line);
			continue;
		}
		for (unsigned i = 0; i < elements[at + 1] && n < (int)sizeof(line) - 4; i++)
			n += snprintf(line + n, sizeof(line) - (size_t)n, " %02x", elements[at + 2 + i]);
		lx_tell("%s\n", line);
	}
}

static void
station_abilities(struct lx_radio *radio, const uint8_t *elements, unsigned length)
{
	struct ieee80211_supported_band *band = radio->wiphy->bands[radio->channel->def.chan->band];
	struct ieee80211_sta *sta = radio->sta;
	struct ieee80211_bss_conf *conf = &radio->vif->bss_conf;
	const uint8_t *e;
	uint32_t rates = 0, basic = 0;

	if ((e = element(elements, length, 1)) != NULL) {
		rates |= rates_of(band, e + 2, e[1], &basic);
	}
	if ((e = element(elements, length, 50)) != NULL) {
		rates |= rates_of(band, e + 2, e[1], &basic);
	}
	sta->deflink.supp_rates[band->band] = rates;
	conf->basic_rates = basic;
	sta->wme = conf->qos = vendor_element(elements, length, wmm) != NULL;
	sta->deflink.bandwidth = radio->width == 160 ? IEEE80211_STA_RX_BW_160 : radio->width == 80 ? IEEE80211_STA_RX_BW_80 :
	    radio->width == 40 ? IEEE80211_STA_RX_BW_40 : IEEE80211_STA_RX_BW_20;
	sta->deflink.rx_nss = 1;
	sta->deflink.smps_mode = IEEE80211_SMPS_OFF;
	if ((e = element(elements, length, 45)) != NULL && e[1] >= 26 && band->ht_cap.ht_supported) {	/* HT capabilities */
		struct ieee80211_sta_ht_cap *ht = &sta->deflink.ht_cap;

		ht->ht_supported = true;
		/* (without the bits for 40 MHz where the channel is 20 MHz wide) */
		ht->cap = (uint16_t)((e[2] | e[3] << 8) & (radio->width > 20 ? 0xffff : ~(IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		    IEEE80211_HT_CAP_SGI_40)));
		ht->ampdu_factor = e[4] & 3;
		ht->ampdu_density = e[4] >> 2 & 7;
		memcpy(&ht->mcs, e + 5, sizeof(ht->mcs));
		sta->deflink.rx_nss = ht->mcs.rx_mask[1] != 0 ? 2 : 1;
	}
	if ((e = element(elements, length, 191)) != NULL && e[1] >= 12 && band->vht_cap.vht_supported) {	/* VHT capabilities */
		struct ieee80211_sta_vht_cap *vht = &sta->deflink.vht_cap;

		vht->vht_supported = true;
		vht->cap = e[2] | e[3] << 8 | e[4] << 16 | (uint32_t)e[5] << 24;
		memcpy(&vht->vht_mcs, e + 6, sizeof(vht->vht_mcs));
	}
	/*
	 * HE capabilities (extension 35): the access point's MAC and PHY capabilities, HE-MCS/NSS sets and PPE thresholds,
	 * which ath11k_peer_assoc_h_he() intersects with ours; and HE operation (36): its parameters and BSS color, which
	 * it gives the firmware (peer_he_ops, the vdev's BSS color), as mac80211 does in ieee80211_he_cap_ie_to_sta_he_cap()
	 * and ieee80211_assoc_success().
	 */
	sta->deflink.he_cap.has_he = false;
	conf->he_support = false;
	if ((e = extension_element(elements, length, 35)) != NULL && e[1] >= 1 + 17 + 4 &&
	    ieee80211_get_he_iftype_cap(band, NL80211_IFTYPE_STATION) != NULL) {
		struct ieee80211_sta_he_cap *he = &sta->deflink.he_cap;
		unsigned mcs = he_mcs_size(e + 9), have = e[1] - 18u, ppe;

		if (have >= mcs) {
			bzero(he, sizeof(*he));
			memcpy(he->he_cap_elem.mac_cap_info, e + 3, sizeof(he->he_cap_elem.mac_cap_info));
			memcpy(he->he_cap_elem.phy_cap_info, e + 9, sizeof(he->he_cap_elem.phy_cap_info));
			memcpy(&he->he_mcs_nss_supp, e + 20, mcs);
			ppe = have > mcs ? he_ppe_size(e[20 + mcs], e + 9) : 0;
			if (ppe != 0 && have >= mcs + ppe && ppe <= sizeof(he->ppe_thres)) {
				memcpy(he->ppe_thres, e + 20 + mcs, ppe);
			} else {
				he->he_cap_elem.phy_cap_info[6] &= ~0x80;	/* (no thresholds given: none said) */
			}
			he->has_he = true;
			conf->he_support = true;
		}
	}
	if (conf->he_support && (e = extension_element(elements, length, 36)) != NULL && e[1] >= 1 + 6) {
		conf->he_oper.params = e[3] | e[4] << 8 | e[5] << 16 | (uint32_t)e[6] << 24;	/* (top byte: BSS color) */
		conf->he_bss_color.color = e[6] & 0x3f;
		conf->he_bss_color.enabled = !(e[6] & 0x80);
	}
	if ((e = element(elements, length, 61)) != NULL && e[1] >= 22) {	/* HT operation: the protection in use */
		conf->ht_operation_mode = (uint16_t)(e[4] | e[5] << 8);
	}
	if ((e = element(elements, length, 5)) != NULL && e[1] >= 2 && e[3] != 0) {	/* the TIM: every how many beacons */
		conf->dtim_period = e[3];
	}
}

/*
 * Our association request's elements, after the two fixed fields: the name, our rates, what
 * 802.11h asks of a station where the access point uses it, RSN (what we agree to of what it
 * offers: CCMP between the two, its cipher for the group, a passphrase's key), our HT and VHT
 * capabilities if it has them, and WMM. Returns their length.
 */
static void
personal_elements(struct lx_radio *radio, unsigned group_cipher)
{
	radio->rsn[0] = 48;
	radio->rsn[1] = 20;
	memcpy(radio->rsn + 2, "\x01\x00\x00\x0f\xac\x04\x01\x00\x00\x0f\xac\x04\x01\x00\x00\x0f\xac\x02\x00\x00", 20);
	radio->rsn[7] = (uint8_t)group_cipher;
	if (radio->wpa.sae) radio->rsn[19] = 8;
	if (radio->wpa.owe) radio->rsn[19] = 18;
	if (radio->wpa.pmf) {
		radio->rsn[1] = 26;
		radio->rsn[20] = 0x80; /* MFPC; PMF is negotiated whenever the AP is capable */
		memcpy(radio->rsn + 22, "\x00\x00\x00\x0f\xac\x06", 6);
		if (radio->wpa.sae) radio->rsn[20] |= 0x40; /* SAE requires PMF */
		if (radio->sae_cached) {
			radio->rsn[1] = 42; radio->rsn[22] = 1;
			memcpy(radio->rsn + 24, radio->sae_pmkid, 16);
			memcpy(radio->rsn + 40, "\x00\x0f\xac\x06", 4);
		}
	}
	radio->wpa.rsn_length = radio->rsn[1] + 2;
	if (radio->wpa.sae && radio->sae_h2e) {
		memcpy(radio->rsn+radio->wpa.rsn_length,"\xf4\x01\x20",3);
		radio->wpa.rsn_length+=3;
	}
}

static unsigned
request_elements(struct lx_radio *radio, uint8_t *out, unsigned group_cipher)
{
	const struct lx_target *target = &radio->target;
	struct linuxkpi_ieee80211_channel *channel = radio->channel->def.chan;
	struct ieee80211_supported_band *band = radio->wiphy->bands[channel->band];
	const struct ieee80211_sta_he_cap *he;
	uint8_t *at = out;
	int count = band->n_bitrates;

	*at++ = 0;
	*at++ = radio->want_length;
	memcpy(at, radio->want, radio->want_length);
	at += radio->want_length;
	*at++ = 1;
	*at++ = (uint8_t)(count < 8 ? count : 8);
	for (int r = 0; r < count; r++) {
		if (r == 8) {
			*at++ = 50;
			*at++ = (uint8_t)(count - 8);
		}
		*at++ = (uint8_t)(band->bitrates[r].bitrate / 5 | (radio->vif->bss_conf.basic_rates & 1u << r ? 0x80 : 0));
	}
	if (target->capability & 0x0100) {	/* spectrum management: the power we can send with, the channels we have */
		*at++ = 33;
		*at++ = 2;
		*at++ = 0;
		*at++ = (uint8_t)channel->max_power;
		*at++ = 36;
		*at++ = 2;
		*at++ = (uint8_t)channel_of(channel->center_freq);
		*at++ = 1;
	}
	personal_elements(radio, group_cipher);
	memcpy(at, radio->rsn, radio->wpa.rsn_length);
	at += radio->wpa.rsn_length;
	if (radio->wpa.owe) { lx_owe_request(&radio->owe, at); at += 37; }
	if (element(target->elements, target->elements_length, 45) != NULL && band->ht_cap.ht_supported) {
		uint16_t cap = (uint16_t)(band->ht_cap.cap & (radio->width > 20 ? 0xffff : ~(IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		    IEEE80211_HT_CAP_SGI_40)));

		*at++ = 45;
		*at++ = 26;
		bzero(at, 26);
		at[0] = (uint8_t)cap;
		at[1] = (uint8_t)(cap >> 8);
		at[2] = (uint8_t)(band->ht_cap.ampdu_factor | band->ht_cap.ampdu_density << 2);
		memcpy(at + 3, &band->ht_cap.mcs, sizeof(band->ht_cap.mcs));
		at += 26;
	}
	if (element(target->elements, target->elements_length, 191) != NULL && band->vht_cap.vht_supported) {
		*at++ = 191;
		*at++ = 12;
		at[0] = (uint8_t)band->vht_cap.cap;
		at[1] = (uint8_t)(band->vht_cap.cap >> 8);
		at[2] = (uint8_t)(band->vht_cap.cap >> 16);
		at[3] = (uint8_t)(band->vht_cap.cap >> 24);
		memcpy(at + 4, &band->vht_cap.vht_mcs, sizeof(band->vht_cap.vht_mcs));
		at += 12;
	}
	/*
	 * Our HE capabilities, if the access point has HE: ath11k's band data for a station, with the channel widths the
	 * join does not use taken out of PHY B0 (mac80211's ieee80211_get_adjusted_he_cap()), so the HE-MCS/NSS sets
	 * shrink with them; then the PPE thresholds. Before WMM, as mac80211 orders them.
	 */
	if (extension_element(target->elements, target->elements_length, 35) != NULL &&
	    (he = ieee80211_get_he_iftype_cap(band, NL80211_IFTYPE_STATION)) != NULL) {
		uint8_t phy[sizeof(he->he_cap_elem.phy_cap_info)];
		unsigned mcs, ppe;

		memcpy(phy, he->he_cap_elem.phy_cap_info, sizeof(phy));
		if (radio->width < 160) {
			phy[0] &= ~(0x08 | 0x10);	/* 160 MHz, 80+80 in 5 GHz */
		}
		if (radio->width < 80) {
			phy[0] &= ~0x04;		/* 40 and 80 MHz in 5 GHz */
		}
		if (radio->width < 40) {
			phy[0] &= ~0x02;		/* 40 MHz in 2.4 GHz */
		}
		mcs = he_mcs_size(phy);
		ppe = he_ppe_size(he->ppe_thres[0], phy);
		*at++ = 255;
		*at++ = (uint8_t)(1 + 6 + 11 + mcs + ppe);
		*at++ = 35;
		memcpy(at, he->he_cap_elem.mac_cap_info, 6);
		memcpy(at + 6, phy, 11);
		memcpy(at + 17, &he->he_mcs_nss_supp, mcs);
		memcpy(at + 17 + mcs, he->ppe_thres, ppe);
		at += 17 + mcs + ppe;
	}
	if (vendor_element(target->elements, target->elements_length, wmm) != NULL) {
		memcpy(at, "\xdd\x07\x00\x50\xf2\x02\x00\x01\x00", 9);
		at += 9;
	}
	return ((unsigned)(at - out));
}

/*
 * What kind of protection the access point offers (its RSN element: version, the group's
 * cipher, the ciphers and the kinds of key management it takes, capabilities). NULL if we can
 * join it -- CCMP pairwise, CCMP or TKIP group, a passphrase's key (kind 2), optional/required BIP-CMAC-128 PMF --, and
 * *group_cipher is then what it ciphers frames to everybody with; else what stands in the way.
 */
static const char *
protection(const struct lx_target *target, unsigned *group_cipher, unsigned *pmf, unsigned *sae)
{
	const uint8_t *rsn = element(target->elements, target->elements_length, 48), *at, *end;
	bool ccmp = false, passphrase = false;
	bool owe = rsn && lx_owe_rsn(rsn, rsn[1]+2);
	unsigned count;
	*pmf = 0;
	*sae = 0;

	if (rsn == NULL) {
		return (target->capability & 0x10 ? "it is protected by something older than WPA2" : "it is not protected at all: an "
		       "open network is not joined by this");
	}
	at = rsn + 2;
	end = at + rsn[1];
	if (end - at < 8 || at[0] != 1 || at[1] != 0 || at[2] != 0x00 || at[3] != 0x0f || at[4] != 0xac) {
		return ("its RSN element is not one this reads");
	}
	*group_cipher = at[5];
	count = at[6] | at[7] << 8;
	at += 8;
	if (count == 0 || count > (unsigned)(end - at) / 4) {
		return ("its RSN pairwise cipher list is truncated or empty");
	}
	for (; count != 0 && end - at >= 4; count--, at += 4) {
		ccmp |= at[0] == 0x00 && at[1] == 0x0f && at[2] == 0xac && at[3] == 4;
	}
	if (end - at < 2) {
		return ("its RSN element names no kind of key management");
	}
	count = at[0] | at[1] << 8;
	at += 2;
	if (count == 0 || count > (unsigned)(end - at) / 4) {
		return ("its RSN key management list is truncated or empty");
	}
	for (; count != 0 && end - at >= 4; count--, at += 4) {
		passphrase |= at[0] == 0x00 && at[1] == 0x0f && at[2] == 0xac && at[3] == 2;
		*sae |= at[0] == 0x00 && at[1] == 0x0f && at[2] == 0xac && at[3] == 8;
	}
	if (*group_cipher != 4 && *group_cipher != 2) {
		return ("its group cipher is neither CCMP nor TKIP");
	}
	if (!ccmp) {
		return ("it does not offer CCMP");
	}
	if (!passphrase && !*sae && !owe) {
		return ("it offers neither PSK nor SAE key management");
	}
	if (at == end) return (*sae || owe) ? "SAE/OWE requires PMF capabilities" : NULL;
	if (end - at < 2) return "its RSN capabilities are truncated";
	unsigned caps = at[0] | at[1] << 8;
	at += 2;
	if ((caps & 0x40) && !(caps & 0x80)) return "its RSN requires PMF without being PMF capable";
	if (at != end) {
		if (end - at < 2) return "its PMKID count is truncated";
		count = at[0] | at[1] << 8;
		at += 2;
		if (count > (unsigned)(end - at) / 16) return "its PMKID list is truncated";
		at += 16 * count;
		if (at != end && (end - at != 4 || memcmp(at, "\x00\x0f\xac\x06", 4) != 0))
			return "its group management cipher is not BIP-CMAC-128";
	}
	if (caps & 0x80) {
		if (*group_cipher != 4) return "PMF requires a CCMP group cipher";
		*pmf = 1;
	}
	if ((*sae || owe) && !*pmf) return "SAE/OWE requires PMF";
	return (NULL);
}

/* Pairwise stays CCMP; the negotiated group key can also be TKIP. */
static struct ieee80211_key_conf *
key_set(struct lx_radio *radio, const uint8_t *key, unsigned index, bool pairwise, int *error)
{
	bool tkip = !pairwise && radio->wpa.group_cipher == 2;
	unsigned length = tkip ? 32 : 16;
	struct ieee80211_key_conf *conf = kzalloc(sizeof(*conf) + length, GFP_KERNEL);

	if (conf == NULL) {
		*error = -ENOMEM;
		return (NULL);
	}
	conf->cipher = tkip ? WLAN_CIPHER_SUITE_TKIP : WLAN_CIPHER_SUITE_CCMP;
	conf->keyidx = (uint8_t)index;
	conf->keylen = length;
	conf->iv_len = 8;
	conf->icv_len = tkip ? 4 : 8;
	conf->flags = pairwise ? IEEE80211_KEY_FLAG_PAIRWISE : 0;
	conf->link_id = -1;
	memcpy(conf->key, key, length);
	mutex_lock(&radio->wiphy->mtx);
	*error = radio->ops->set_key(&radio->hw, SET_KEY, radio->vif, pairwise ? radio->sta : NULL, conf);
	mutex_unlock(&radio->wiphy->mtx);
	return (conf);
}

static void
key_drop(struct lx_radio *radio, struct ieee80211_key_conf **conf, bool pairwise)
{
	if (*conf != NULL) {
		mutex_lock(&radio->wiphy->mtx);
		radio->ops->set_key(&radio->hw, DISABLE_KEY, radio->vif, pairwise ? radio->sta : NULL, *conf);
		mutex_unlock(&radio->wiphy->mtx);
		explicit_bzero((*conf)->key, (*conf)->keylen);
		kfree(*conf);
		*conf = NULL;
	}
}

/* The access point has changed the group's key (it does every so often): the new one to the driver, the old one gone. */
static void
group_rekey(struct work_struct *work)
{
	struct lx_radio *radio = __containerof(work, struct lx_radio, group_work);
	struct ieee80211_key_conf *old = radio->group_key, *fresh;
	int error = 0;

	radio->rekeys++;
	/* An authenticated rekey must not reinstall identical GTK material and reset firmware PNs. */
	if (old != NULL && old->keyidx == radio->wpa.group_index && old->keylen == radio->wpa.group_length &&
	    memcmp(old->key, radio->wpa.group, old->keylen) == 0) return;
	fresh = key_set(radio, radio->wpa.group, radio->wpa.group_index, false, &error);
	lx_tell("lx: the group's new key (number %u) is given to the driver: %d\n", radio->wpa.group_index, error);
	if (error != 0) {
		/*
		 * The firmware did not take the new key: the old one stays (it is not dropped for a key that is not there), and the
		 * new configuration, which key_set() returns even then, is wiped and freed; nothing was installed to disable.
		 */
		if (fresh != NULL) {
			explicit_bzero(fresh->key, fresh->keylen);
			kfree(fresh);
		}
		return;
	}
	radio->group_key = fresh;
	if (old != NULL && old->keyidx == fresh->keyidx) {
		/* SET_KEY already replaced this firmware slot. DISABLE_KEY would erase the fresh GTK. */
		explicit_bzero(old->key, old->keylen);
		kfree(old);
	} else {
		key_drop(radio, &old, false);
	}
}

/* One step of the station's state as the driver is told of it (mac80211: drv_sta_state()). */
static int
station_state(struct lx_radio *radio, enum ieee80211_sta_state from, enum ieee80211_sta_state to)
{
	int ret;

	mutex_lock(&radio->wiphy->mtx);
	ret = radio->ops->sta_state(&radio->hw, radio->vif, radio->sta, from, to);
	mutex_unlock(&radio->wiphy->mtx);
	return (ret);
}

static void
bss_changed(struct lx_radio *radio, u64 what)
{
	mutex_lock(&radio->wiphy->mtx);
	radio->ops->bss_info_changed(&radio->hw, radio->vif, &radio->vif->bss_conf, what);
	mutex_unlock(&radio->wiphy->mtx);
}

/* Everything a join has made, undone in mac80211's order; as far as it got. */
static void
station_leave(struct lx_radio *radio)
{
	struct ieee80211_hw *hw = &radio->hw;
	struct ieee80211_vif *vif = radio->vif;
	int state = radio->reached;

	/* (a reply the receive path was making to the access point finished, and no other begun, before the handshake's state
	 * is cleared and the station freed) */
	station_close(radio);
	if (radio->joined) {
		radio->joined = false;
		lx_os_net_link(0);
	}
	if (radio->sta != NULL && state >= IEEE80211_STA_AUTH && radio->answer != ANSWER_GONE) {
		struct sk_buff *skb;
		uint8_t *frame;

		if ((skb = frame_new(radio, 26, &frame)) != NULL) {	/* deauthentication, reason 3: we are leaving */
			frame = management(radio, frame, 0xc0);
			frame[0] = 3;
			frame[1] = 0;
			frame_send(radio, skb);
			lx_pause_ms(50);
		}
	}
	cancel_work_sync(&radio->group_work);
	cancel_work_sync(&radio->ba_work);
	radio->ba_asked = 0;
	bzero(radio->ba, sizeof(radio->ba));		/* (the driver forgets the classes' queues with the station) */
	key_drop(radio, &radio->group_key, false);
	key_drop(radio, &radio->pairwise_key, true);
	explicit_bzero(&radio->wpa, sizeof(radio->wpa));
	lx_os_lock();radio->sae_listening=false;radio->sae_rx_length=0;lx_os_unlock();
	lx_sae_clear(&radio->sae);
	lx_owe_clear(&radio->owe);
	lx_crypto_wipe(radio->sae_rx,sizeof(radio->sae_rx));
	lx_crypto_wipe(radio->sae_pmkid,sizeof(radio->sae_pmkid));
	radio->sae_cached=0;
	for (; radio->sta != NULL && state > IEEE80211_STA_NOTEXIST; state--) {
		station_state(radio, state, state - 1);
	}
	if (vif != NULL && vif->cfg.assoc) {
		vif->cfg.assoc = false;
		vif->cfg.aid = 0;
		bss_changed(radio, BSS_CHANGED_ASSOC);
	}
	if (radio->channel != NULL) {
		mutex_lock(&hw->wiphy->mtx);
		radio->ops->unassign_vif_chanctx(hw, vif, &vif->bss_conf, radio->channel);
		vif->bss_conf.chanctx_conf = NULL;
		radio->ops->remove_chanctx(hw, radio->channel);
		mutex_unlock(&hw->wiphy->mtx);
		kfree(radio->channel);
		radio->channel = NULL;
	}
	if (vif != NULL) {
		vif->bss_conf.bssid = no_network;
		bss_changed(radio, BSS_CHANGED_BSSID);
	}
	kfree(radio->sta_memory);
	radio->sta_memory = radio->sta = NULL;
	radio->reached = IEEE80211_STA_NOTEXIST;
	station_open(radio);
}

/*
 * How wide the channel is to be: with wifi_wide=1 as wide as the access point says it works (its
 * HT operation element: the second 20 MHz above or below; its VHT operation element: 80 MHz around
 * a centre) and as the radio may tune (every 20 MHz of it allowed here); else 20 MHz. 80 MHz
 * only at 5 GHz. mac80211 does this in ieee80211_determine_chan_mode().
 */
static void
station_width(struct lx_radio *radio, struct linuxkpi_ieee80211_channel *primary, struct cfg80211_chan_def *def)
{
	const struct lx_target *target = &radio->target;
	struct ieee80211_supported_band *band = radio->wiphy->bands[primary->band];
	const uint8_t *ht = element(target->elements, target->elements_length, 61);
	const uint8_t *vht = element(target->elements, target->elements_length, 192);
	unsigned offset;

	def->chan = primary;
	def->width = NL80211_CHAN_WIDTH_20;
	def->center_freq1 = primary->center_freq;
	radio->width = 20;
	if (!lx_os_option("wifi_wide") || ht == NULL || ht[1] < 22 || !band->ht_cap.ht_supported ||
	    !(band->ht_cap.cap & IEEE80211_HT_CAP_SUP_WIDTH_20_40) || !(ht[3] & 4)) {
		lx_tell("lx: 20 MHz: wifi_wide %u, HT operation %s (length %u, byte %#x: offset %u, any width %u), our HT %s, HT40 %s\n",
		    lx_os_option("wifi_wide"), ht ? "there" : "MISSING", ht ? ht[1] : 0, ht && ht[1] >= 2 ? ht[3] : 0,
		    ht && ht[1] >= 2 ? ht[3] & 3 : 0, ht && ht[1] >= 2 ? !!(ht[3] & 4) : 0, band->ht_cap.ht_supported ? "yes" : "NO",
		    band->ht_cap.cap & IEEE80211_HT_CAP_SUP_WIDTH_20_40 ? "yes" : "NO");
		return;
	}
	if (vht != NULL && vht[1] >= 3 && (vht[2] == 1 || vht[2] == 2) && primary->band == NL80211_BAND_5GHZ &&
	    band->vht_cap.vht_supported) {
		/*
		 * 160 MHz (802.11-2020 Table 9-274): width 1 with CCFS1 the 160's center, 8 channels from CCFS0 (the 80 with
		 * the primary); or the old width 2 with CCFS0 the 160's center. Where we can (the band's VHT supported widths,
		 * or extended NSS bandwidth) and all eight 20s may be used for it.
		 */
		unsigned wide = vht[2] == 2 ? 5000 + 5 * vht[3] : vht[4] != 0 && (vht[4] == vht[3] + 8 || vht[3] == vht[4] + 8) ?
		    5000 + 5 * vht[4] : 0;
		unsigned center = vht[2] == 2 ? (primary->center_freq < wide ? wide - 40 : wide + 40) : 5000 + 5 * vht[3];
		bool all = true;

		if (wide != 0 && (band->vht_cap.cap & (IEEE80211_VHT_CAP_SUPP_CHAN_WIDTH_MASK | IEEE80211_VHT_CAP_EXT_NSS_BW_MASK))) {
			bool all160 = primary->center_freq >= wide - 70 && primary->center_freq <= wide + 70;

			for (unsigned frequency = wide - 70; frequency <= wide + 70; frequency += 20) {
				struct linuxkpi_ieee80211_channel *c = linuxkpi_ieee80211_get_channel(radio->wiphy, frequency);

				all160 &= c != NULL && !(c->flags & (IEEE80211_CHAN_DISABLED | IEEE80211_CHAN_NO_80MHZ |
				    IEEE80211_CHAN_NO_160MHZ));
			}
			if (all160) {
				def->width = NL80211_CHAN_WIDTH_160;
				def->center_freq1 = wide;
				radio->width = 160;
				return;
			}
			lx_tell("lx: not 160 MHz around %u: the eight 20s or the primary %u MHz not all usable for it\n", wide,
			    primary->center_freq);
		}

		for (unsigned frequency = center - 30; frequency <= center + 30; frequency += 20) {
			struct linuxkpi_ieee80211_channel *c = linuxkpi_ieee80211_get_channel(radio->wiphy, frequency);

			all &= c != NULL && !(c->flags & (IEEE80211_CHAN_DISABLED | IEEE80211_CHAN_NO_80MHZ));
		}
		if (all && primary->center_freq >= center - 30 && primary->center_freq <= center + 30) {
			def->width = NL80211_CHAN_WIDTH_80;
			def->center_freq1 = center;
			radio->width = 80;
			return;
		}
		lx_tell("lx: not 80 MHz around %u: the four 20s %s, the primary %u MHz %s them\n", center, all ? "allowed" :
		    "NOT ALL ALLOWED (disabled or no 80 MHz)", primary->center_freq, primary->center_freq + 30 >= center &&
		    primary->center_freq <= center + 30 ? "inside" : "NOT INSIDE");
	} else {
		lx_tell("lx: not 80 MHz: VHT operation %s (length %u, width %u), %s, our VHT %s\n", vht ? "there" : "missing",
		    vht ? vht[1] : 0, vht && vht[1] >= 1 ? vht[2] : 0, primary->band == NL80211_BAND_5GHZ ? "5 GHz" : "2.4 GHz",
		    band->vht_cap.vht_supported ? "yes" : "no");
	}
	offset = ht[3] & 3;		/* the second 20 MHz: 1 above, 3 below, 0 none */
	if (offset == 1 || offset == 3) {
		struct linuxkpi_ieee80211_channel *second = linuxkpi_ieee80211_get_channel(radio->wiphy, offset == 1 ?
		    primary->center_freq + 20 : primary->center_freq - 20);

		if (second != NULL && !(second->flags & IEEE80211_CHAN_DISABLED) && !(primary->flags & (offset == 1 ?
		    IEEE80211_CHAN_NO_HT40PLUS : IEEE80211_CHAN_NO_HT40MINUS))) {
			def->width = NL80211_CHAN_WIDTH_40;
			def->center_freq1 = offset == 1 ? primary->center_freq + 10 : primary->center_freq - 10;
			radio->width = 40;
			return;
		}
	}
	lx_tell("lx: 20 MHz: no usable second 20 (offset %u, primary flags %#x)\n", offset, primary->flags);
}

/*
 * Join the network of that name with that passphrase, in the order mac80211 goes through
 * (net/mac80211/mlme.c of Linux: ieee80211_mgd_auth(), ieee80211_mgd_assoc(), and what the
 * supplicant does after them): the access point found by a scan that asks for the network; the
 * channel taken; the access point made a station the driver knows; authentication; association;
 * the handshake; the keys. 0 when frames can pass, or an errno with the reason said. A network
 * that was joined before is left first. The access point is the one a scan has heard best --
 * the last scan, if that heard the network, or one made now.
 */
/* Association response validation and ECDH run on the station thread.
 * The RX thread discards early EAPOL until the complete PMK is published. */
static int
owe_associate(struct lx_radio *radio)
{
    struct lx_target response={0};unsigned gc=0,mp=0,sa=0;
    uint8_t pmk[32]={0},pmkid[16]={0};int error=EAUTH;
    lx_os_lock();
    if(radio->answer_length>sizeof(response.elements)) { lx_os_unlock();goto out; }
    response.elements_length=radio->answer_length;
    memcpy(response.elements,radio->answer_elements,response.elements_length);
    lx_os_unlock();
    const uint8_t *rs=element(response.elements,response.elements_length,48);
    if(!rs || !lx_owe_rsn(rs,rs[1]+2) || protection(&response,&gc,&mp,&sa) ||
        gc!=radio->wpa.group_cipher || mp!=radio->wpa.pmf ||
        lx_owe_finish(&radio->owe,response.elements,response.elements_length,pmk,pmkid)) goto out;
    lx_os_lock();memcpy(radio->wpa.pmk,pmk,32);radio->wpa.owe_ready=1;lx_os_unlock();error=0;
out:
    lx_owe_clear(&radio->owe);
    lx_crypto_wipe(pmk,sizeof(pmk));lx_crypto_wipe(pmkid,sizeof(pmkid));return error;
}

static int
station_join(struct lx_radio *radio, const struct hakios_sta_join *what)
{
	const uint8_t *name = what->ssid;
	unsigned name_length = what->ssid_length;
	struct ieee80211_hw *hw = &radio->hw;
	struct ieee80211_vif *vif;
	struct linuxkpi_ieee80211_channel *channel;
	struct lx_target *target = &radio->target;
	struct sk_buff *skb;
	uint8_t *frame;
	const uint8_t *e;
	const char *obstacle;
	unsigned long long began;
	unsigned group_cipher = 0, pmf = 0, sae = 0, owe = 0, length;
	bool used_cache=false;
	int ret, error = 0, answer = ANSWER_NONE;

	if (name_length == 0 || name_length > sizeof(radio->want)) {
		return (EINVAL);
	}
	if (what->passphrase_length > 63) return EINVAL;
	if (what->security != HAKIOS_STA_WPA2_PSK && what->security != HAKIOS_STA_WPA3_SAE &&
	    what->security != HAKIOS_STA_OPEN && what->security != HAKIOS_STA_OWE) {
		lx_tell("lx: a network without Personal authentication is asked for: that is not built\n");
		return (ENOTSUP);
	}
	if (radio->sta != NULL) {
		station_leave(radio);
		lx_tell("lx: the network joined before is left\n");
	}
	INIT_WORK(&radio->group_work, group_rekey);
	INIT_WORK(&radio->ba_work, ba_handle);
	radio->ba_asked = 0;
	bzero(radio->ba, sizeof(radio->ba));
	memcpy(radio->want, name, name_length);
	radio->want_length = (uint8_t)name_length;
	/* (under the lock: heard() writes the target under it, from the receive thread) */
	lx_os_lock();
	bzero(target, sizeof(*target));
	memcpy(radio->want_bssid, what->bssid, ETH_ALEN);
	radio->target_fixed = false;
	lx_os_unlock();
	if ((ret = station_up(radio)) != 0) {
		return (ret);
	}
	vif = radio->vif;
	lx_os_lock();
	for (unsigned i = 0; i < radio->heard_count; i++) {
		const struct lx_heard *network = &radio->heard[i];

		if (network->name_length == name_length && memcmp(network->name, name, name_length) == 0 && network->elements_length != 0 &&
		    (is_zero_ether_addr(what->bssid) || memcmp(what->bssid, network->sender, ETH_ALEN) == 0) &&
		    (!target->found || network->signal > target->signal)) {
			target->found = true;
			memcpy(target->address, network->sender, ETH_ALEN);
			target->frequency = network->frequency;
			target->interval = network->interval;
			target->capability = network->capability;
			target->signal = network->signal;
			target->elements_length = network->elements_length;
			memcpy(target->elements, network->elements, network->elements_length);
		}
	}
	lx_os_unlock();
	if (!target->found) {
		adopt_heard_country(radio);	/* (the network may be on a channel only the country allows) */
	}
	for (unsigned attempt = 0; attempt < 3 && !target->found; attempt++) {
		station_scan(radio, name, name_length, false, NULL);
		adopt_heard_country(radio);
	}
	/* the choice is made: a beacon heard from here on does not change it (the join goes on with this one's address and elements) */
	lx_os_lock();
	radio->target_fixed = true;
	lx_os_unlock();
	if (!target->found) {
		lx_tell("lx: THE NETWORK THAT IS WANTED WAS NOT HEARD (a name of %u bytes), in three scans\n", name_length);
		return (ENOENT);
	}
	channel = linuxkpi_ieee80211_get_channel(hw->wiphy, target->frequency);
	if ((e = element(target->elements, target->elements_length, 3)) != NULL && e[1] == 1 && channel != NULL &&
	    linuxkpi_ieee80211_get_channel(hw->wiphy, linuxkpi_ieee80211_channel_to_frequency(e[2], channel->band)) != NULL) {
		/* (a beacon can be heard on the channel next to its own: where it says it is counts) */
		channel = linuxkpi_ieee80211_get_channel(hw->wiphy, linuxkpi_ieee80211_channel_to_frequency(e[2], channel->band));
	}
	obstacle = protection(target, &group_cipher, &pmf, &sae);
    e = element(target->elements, target->elements_length, 48);
    owe = e && lx_owe_rsn(e, e[1]+2);
    if ((what->security==HAKIOS_STA_OPEN || what->security==HAKIOS_STA_OWE) && !owe)
        obstacle="this passwordless request requires an OWE BSS";
    if (owe && (what->passphrase_length || (what->security!=HAKIOS_STA_OPEN && what->security!=HAKIOS_STA_OWE)))
        obstacle="OWE cannot satisfy a Personal authentication request";
	if(what->security==HAKIOS_STA_WPA3_SAE && !sae) obstacle="WPA3 was required but this BSS offers no SAE";
	if(sae && !what->passphrase_length) obstacle="SAE needs the password, not a WPA2-derived PMK";
	if(!sae && what->passphrase_length && what->passphrase_length<8) obstacle="WPA2 requires at least eight password bytes";
	lx_tell("lx: the network that is wanted: an access point on channel %u (%u MHz) at %d dBm, capabilities %#x, beacons every "
	    "%u units; HT %s, VHT %s, HE %s, WMM %s; %s%s\n", channel != NULL ? channel_of(channel->center_freq) : 0, channel != NULL ?
	    channel->center_freq : 0, target->signal, target->capability, target->interval, element(target->elements,
	    target->elements_length, 45) ? "yes" : "no", element(target->elements, target->elements_length, 191) ? "yes" : "no",
	    extension_element(target->elements, target->elements_length, 35) ? "yes" : "no",
	    vendor_element(target->elements, target->elements_length, wmm) ? "yes" : "no", obstacle == NULL ?
	    (owe ? "OWE with CCMP and PMF: it can be joined" : sae ? "WPA3-SAE with CCMP and PMF: it can be joined" : "WPA2-PSK with CCMP: it can be joined") :
	    "IT CANNOT BE JOINED: ", obstacle == NULL ? "" : obstacle);
	if (obstacle != NULL || channel == NULL || (channel->flags & IEEE80211_CHAN_DISABLED)) {
		return (ENOTSUP);
	}

	/* the key of the name and the passphrase, and what the handshake needs besides */
	began = lx_os_nanoseconds();
	bzero(&radio->wpa, sizeof(radio->wpa));
	radio->sae_cached=0;
	e=element(target->elements,target->elements_length,244);
	if(sae && e && (!e[1] || (e[2]&15)+1u!=e[1])) return EINVAL;
	radio->sae_h2e=sae && e && e[1]>=1 && (e[2]&0x20);
	/* 6 GHz requires H2E; the remaining 6 GHz PHY work belongs to P6. */
	if(sae && target->frequency>=5925 && !radio->sae_h2e) return ENOTSUP;
	if(owe) {
        if(lx_owe_start(&radio->owe,sae_entropy,NULL)) return EAUTH;
    } else if(sae) {
		used_cache=radio->sae_cached=lx_sae_cache_get(&radio->sae_cache,vif->addr,target->address,
		    name,name_length,what->passphrase,what->passphrase_length,radio->sae_h2e,lx_os_nanoseconds(),radio->wpa.pmk,radio->sae_pmkid);
	} else if (what->passphrase_length != 0) {
		lx_wpa_pmk(what->passphrase, what->passphrase_length, name, name_length, radio->wpa.pmk);
	} else {
		memcpy(radio->wpa.pmk, what->pmk, sizeof(radio->wpa.pmk));
	}
	memcpy(radio->wpa.own, vif->addr, ETH_ALEN);
	memcpy(radio->wpa.peer, target->address, ETH_ALEN);
	get_random_bytes(radio->wpa.own_nonce, sizeof(radio->wpa.own_nonce));
	radio->wpa.rsn = radio->rsn;
	radio->wpa.rsn_length = 22;
	radio->wpa.group_cipher = group_cipher;
	radio->wpa.pmf = pmf;
	radio->wpa.sae = sae;
	radio->wpa.owe = owe;
	personal_elements(radio, group_cipher);
	radio->wpa.ap_rsn = element(target->elements, target->elements_length, 48);
	radio->wpa.ap_rsn_length = radio->wpa.ap_rsn[1] + 2;
	if(sae && e) { radio->wpa.ap_rsnxe=e;radio->wpa.ap_rsnxe_length=e[1]+2; }
	radio->management_tx_pn = 0;
	lx_tell("lx: protected management frames: %s (BIP-CMAC-128 in host, CCMP in firmware)\n", pmf ? "negotiated" : "off");
	radio->handshake = LX_WPA_NOTHING;
	printf("lx: Personal key state prepared (name %u bytes, password %u bytes; 0: PMK supplied): %llu ms\n",
	    name_length, what->passphrase_length, (lx_os_nanoseconds() - began) / 1000000ull);

	/* the access point's address, and its channel, 20 MHz wide */
	vif->bss_conf.bssid = target->address;
	vif->bss_conf.beacon_int = target->interval;
	vif->bss_conf.assoc_capability = target->capability;
	vif->bss_conf.dtim_period = 1;
	vif->bss_conf.use_short_slot = (target->capability & 0x0400) != 0 || channel->band != NL80211_BAND_2GHZ;
	vif->bss_conf.use_short_preamble = (target->capability & 0x0020) != 0;
	memcpy(vif->cfg.ap_addr, target->address, ETH_ALEN);
	memcpy(vif->cfg.ssid, name, name_length);
	vif->cfg.ssid_len = name_length;
	bss_changed(radio, BSS_CHANGED_BSSID);
	radio->channel = kzalloc(sizeof(*radio->channel) + hw->chanctx_data_size + 64, GFP_KERNEL);
	radio->sta_memory = kzalloc(64 + sizeof(*radio->sta) + hw->sta_data_size, GFP_KERNEL);
	if (radio->channel == NULL || radio->sta_memory == NULL) {
		error = ENOMEM;
		goto out;
	}
	station_width(radio, channel, &radio->channel->def);
	radio->channel->min_def = radio->channel->def;
	radio->channel->rx_chains_static = radio->channel->rx_chains_dynamic = 2;
	vif->bss_conf.chanreq.oper = radio->channel->def;
	mutex_lock(&hw->wiphy->mtx);
	ret = radio->ops->add_chanctx(hw, radio->channel);
	if (ret == 0) {
		vif->bss_conf.chanctx_conf = radio->channel;
		ret = radio->ops->assign_vif_chanctx(hw, vif, &vif->bss_conf, radio->channel);
	}
	mutex_unlock(&hw->wiphy->mtx);
	lx_tell("lx: the channel is taken, %u MHz wide around %u MHz: %d\n", radio->width, radio->channel->def.center_freq1, ret);
	if (ret != 0) {
		error = -ret;
		goto out;
	}

	/* the access point as a station of the driver's: the firmware learns of it, and the radio goes to its channel */
	radio->sta = (struct ieee80211_sta *)roundup2((uintptr_t)radio->sta_memory, 64);
	memcpy(radio->sta->addr, target->address, ETH_ALEN);
	memcpy(radio->sta->deflink.addr, target->address, ETH_ALEN);
	radio->sta->deflink.sta = radio->sta;
	radio->sta->mfp = pmf != 0;
	radio->sta->link[0] = &radio->sta->deflink;
	station_abilities(radio, target->elements, target->elements_length);
	ret = station_state(radio, IEEE80211_STA_NOTEXIST, IEEE80211_STA_NONE);
	printf("lx: the access point is made known to the driver: %d\n", ret);
	if (ret != 0) {
		kfree(radio->sta_memory);
		radio->sta_memory = radio->sta = NULL;
		error = -ret;
		goto out;
	}
	radio->reached = IEEE80211_STA_NONE;

	if(sae && !used_cache) {
		error=sae_authenticate(radio,what);
		if(error) goto out;
		answer=ANSWER_AUTHENTICATED;
	}
	/* A cached SAE PMKSA uses open authentication, with its PMKID in association. */
	for (unsigned attempt = 0; attempt < 3 && answer == ANSWER_NONE; attempt++) {
		radio->answer = ANSWER_NONE;
		if ((skb = frame_new(radio, 30, &frame)) == NULL) {
			break;
		}
		frame = management(radio, frame, 0xb0);
		memcpy(frame, "\x00\x00\x01\x00\x00\x00", 6);	/* algorithm 0, step 1, status 0 */
		frame_send(radio, skb);
		answer = answer_wait(radio, 500);
	}
	lx_tell("lx: AUTHENTICATION: %s (status %u)\n", answer == ANSWER_AUTHENTICATED ? "the access point accepts" : answer ==
	    ANSWER_NONE ? "NO ANSWER from the access point to three requests" : "THE ACCESS POINT REFUSES", radio->answer_status);
	if (answer != ANSWER_AUTHENTICATED || (ret = station_state(radio, IEEE80211_STA_NONE, IEEE80211_STA_AUTH)) != 0) {
		error = answer == ANSWER_NONE ? ETIMEDOUT : ECONNREFUSED;
		goto out;
	}
	radio->reached = IEEE80211_STA_AUTH;

	/* association: what we can do, and that we want its protection */
	answer = ANSWER_NONE;
	for (unsigned attempt = 0; attempt < 3 && answer == ANSWER_NONE; attempt++) {
		radio->answer = ANSWER_NONE;
		if ((skb = frame_new(radio, 28 + 400, &frame)) == NULL) {
			break;
		}
		frame = management(radio, frame, 0x00);
		/* capabilities: an infrastructure network, protected, and of the access point's own what a station repeats */
		frame[0] = (uint8_t)(0x01 | 0x10 | (target->capability & 0x20));
		frame[1] = (uint8_t)((target->capability & 0x0500) >> 8);
		frame[2] = 10;				/* we listen at least every tenth beacon */
		frame[3] = 0;
		length = request_elements(radio, frame + 4, group_cipher);
		skb_trim(skb, 28 + length);
		frame_send(radio, skb);
		answer = answer_wait(radio, 500);
	}
	lx_tell("lx: ASSOCIATION: %s (status %u); we are its station number %u; its answer has %u bytes of elements\n", answer ==
	    ANSWER_ASSOCIATED ? "THE ACCESS POINT ACCEPTS" : answer == ANSWER_NONE ? "NO ANSWER from the access point to three "
	    "requests" : "THE ACCESS POINT REFUSES", radio->answer_status, radio->aid, radio->answer_length);
	/* (diagnosis, run71: no broadcast ever reached us while multicast did -- what the access point says of itself:
	 * Extended Capabilities 127 (bit 12 proxy ARP), Interworking 107, Hotspot 2.0 50-6f-9a-10 and the rest, as they are) */
	elements_say("its beacon", target->elements, target->elements_length);
	elements_say("its association answer", radio->answer_elements, radio->answer_length);
	if (answer != ANSWER_ASSOCIATED) {
		error = answer == ANSWER_NONE ? ETIMEDOUT : ECONNREFUSED;
		goto out;
	}
    if (owe && (error=owe_associate(radio)) != 0) goto out;
	station_abilities(radio, radio->answer_elements, radio->answer_length);
	radio->sta->aid = radio->aid;
	if ((ret = station_state(radio, IEEE80211_STA_AUTH, IEEE80211_STA_ASSOC)) != 0) {
		error = -ret;
		goto out;
	}
	radio->reached = IEEE80211_STA_ASSOC;
	vif->cfg.assoc = true;
	vif->cfg.aid = radio->aid;
	bss_changed(radio, BSS_CHANGED_ASSOC | BSS_CHANGED_BEACON_INT | BSS_CHANGED_ERP_SLOT | BSS_CHANGED_ERP_PREAMBLE |
	    BSS_CHANGED_QOS);
	if ((e = vendor_element(radio->answer_elements, radio->answer_length, wmm)) != NULL && e[1] >= 24 && e[6] == 1) {
		/* WMM's parameters: for each of the four classes how long to wait and how long to hold the air */
		mutex_lock(&hw->wiphy->mtx);
		for (unsigned i = 0; i < 4; i++) {
			const uint8_t *record = e + 10 + 4 * i;
			static const uint8_t classes[4] = { IEEE80211_AC_BE, IEEE80211_AC_BK, IEEE80211_AC_VI, IEEE80211_AC_VO };
			struct ieee80211_tx_queue_params params = { .aifs = record[0] & 0x0f, .cw_min = (uint16_t)((1u << (record[1] &
				                                    0x0f)) - 1), .cw_max = (uint16_t)((1u << (record[1] >> 4)) - 1),
				                                    .txop = (uint16_t)(record[2] | record[3] << 8), .acm = record[0] >> 4 &
				                                    1 };

			radio->ops->conf_tx(hw, vif, 0, classes[record[0] >> 5 & 3], &params);
		}
		mutex_unlock(&hw->wiphy->mtx);
	}
	lx_tell("lx: the driver is told of the association (rates %#x, HT %s, VHT %s, HE %s, %u streams, %u MHz)\n",
	    radio->sta->deflink.supp_rates[channel->band], radio->sta->deflink.ht_cap.ht_supported ? "yes" : "no",
	    radio->sta->deflink.vht_cap.vht_supported ? "yes" : "no", radio->sta->deflink.he_cap.has_he ? "yes" : "no",
	    radio->sta->deflink.rx_nss, radio->width);

	/* the handshake: the access point begins it; its messages come in as frames and are answered where they come in */
	began = lx_os_nanoseconds();
	lx_os_lock();
	/*
	 * (A message that is not good does not end the wait: it may be a forged frame, and a frame that fails its check is discarded
	 * -- IEEE 802.11i, 8.5.3 -- while the genuine messages can still come. The reason is still said below if the time runs out.)
	 */
	while (radio->handshake != LX_WPA_KEYS && radio->answer != ANSWER_GONE &&
	    lx_os_nanoseconds() - began < 8000000000ull) {
		lx_os_sleep(&radio->handshake, 20 * 1000000ull);
	}
	lx_os_unlock();
	if (radio->handshake != LX_WPA_KEYS) {
		lx_tell("lx: THE HANDSHAKE DID NOT COME THROUGH in %llu ms: %s%s (%u of its messages came)\n", (lx_os_nanoseconds() - began) /
		    1000000ull, radio->answer == ANSWER_GONE ? "the access point has shown us the door (a wrong passphrase gives this)" :
		    radio->handshake == LX_WPA_BAD ? "a message was not good: " : radio->handshake == LX_WPA_ANSWER ? "our message 2 was "
		    "not answered (a wrong passphrase gives this)" : "the access point did not begin it", radio->handshake == LX_WPA_BAD ?
		    radio->handshake_why : "", radio->eapol);
		error = EAUTH;
		goto out;
	}
	lx_pause_ms(30);			/* (for message 4 to have left unciphered, before there is a key to cipher with) */
	radio->pairwise_key = key_set(radio, radio->wpa.pairwise, 0, true, &ret);
	if (ret == 0) {
		radio->group_key = key_set(radio, radio->wpa.group, radio->wpa.group_index, false, &ret);
	}
	lx_tell("lx: THE KEYS ARE GIVEN TO THE DRIVER, %llu ms after the association: ours with the access point, and the group's "
	    "(number %u, %u bytes): %d\n", (lx_os_nanoseconds() - began) / 1000000ull, radio->wpa.group_index,
	    radio->wpa.group_length, ret);
	if (ret != 0 || (ret = station_state(radio, IEEE80211_STA_ASSOC, IEEE80211_STA_AUTHORIZED)) != 0) {
		error = -ret;
		goto out;
	}
	radio->reached = IEEE80211_STA_AUTHORIZED;
	radio->sent = radio->sent_lost = radio->received = radio->received_group = radio->rekeys = 0;
	radio->sent_offloaded = radio->received_checked = radio->sent_classed = 0;
	radio->unciphered = 0;
	radio->joined_at = lx_os_nanoseconds();
	radio->signal = target->signal;
	radio->joined = true;
	if(sae && !used_cache) lx_sae_cache_store(&radio->sae_cache,&radio->sae,vif->addr,target->address,
	    name,name_length,what->passphrase,what->passphrase_length,lx_os_nanoseconds());
	lx_sae_clear(&radio->sae);
	lx_owe_clear(&radio->owe);
	lx_os_net_link(1);
	lx_tell("lx: THE NETWORK IS JOINED: frames can pass\n");
	return (0);
out:
	station_leave(radio);
	if(used_cache) {
		/* AP reboot/expiry: discard the entry and make exactly one fresh SAE attempt.
		 * The selected AKM stays SAE; failure cannot downgrade a transition BSS. */
		struct hakios_sta_join fresh=*what;
		fresh.security=HAKIOS_STA_WPA3_SAE;
		memcpy(fresh.bssid,target->address,ETH_ALEN);
		lx_crypto_wipe(&radio->sae_cache,sizeof(radio->sae_cache));
		int retry=station_join(radio,&fresh);
		lx_crypto_wipe(&fresh,sizeof(fresh));
		return retry;
	}
	return (error);
}

/* ---- for the rest of the layer ---------------------------------------------------------------------------- */

/* Join the network the kernel was given the name and passphrase of. 0 when frames can pass, or an errno. */
int
lx_station_join(void)
{
	static struct hakios_sta_join what;
	struct lx_radio *radio = the_radio;
	unsigned name_length = sizeof(what.ssid), passphrase_length = sizeof(what.passphrase);
	int error;

	if (radio == NULL || !radio->registered) {
		lx_tell("lx: no radio is registered: nothing to join with\n");
		return (ENXIO);
	}
	bzero(&what, sizeof(what));
	if (!lx_os_network(what.ssid, &name_length, what.passphrase, &passphrase_length)) {
		return (ENOENT);
	}
	what.ssid_length = (uint8_t)name_length;
	what.passphrase_length = (uint8_t)passphrase_length;
	what.security = HAKIOS_STA_WPA2_PSK;
	lx_os_net_attach(radio->wiphy->perm_addr);
	error = station_join(radio, &what);
	explicit_bzero(&what, sizeof(what));
	return (error);
}

/* What has passed since the network was joined. */
void
lx_station_say(void)
{
	struct lx_radio *radio = the_radio;

	if (radio != NULL) {
		lx_tell("lx: the network is %s, %llu s after it was joined: %u frames sent (%u could not be), %u received for us and "
		    "%u sent to everybody, %u dropped for coming unciphered; %u messages of the handshake, the group's key changed %u times; block ack: %u classes "
		    "taken, %u refused; checksums: %u sent left to the radio, %u received checked by it; %u sent in a WMM class other "
		    "than best effort\n", radio->joined ? "joined" : "NOT joined", radio->joined_at != 0 ? (lx_os_nanoseconds() -
		    radio->joined_at) / 1000000000ull : 0, radio->sent, radio->sent_lost, radio->received, radio->received_group, radio->unciphered,
		    radio->eapol, radio->rekeys, radio->ba_sessions, radio->ba_refused, radio->sent_offloaded, radio->received_checked,
		    radio->sent_classed);
	}
}

/* Leave the network, if one is joined, and take the interface down. */
void
lx_station_leave(void)
{
	struct lx_radio *radio = the_radio;

	if (radio == NULL || radio->vif == NULL) {
		return;
	}
	lx_station_say();
	if (radio->sta != NULL) {
		station_leave(radio);
		lx_tell("lx: the network is left\n");
	}
	station_down(radio);
}

/* What a frame that ended a doze was: ARP, or IP protocol, destination and port (the last 4, in doze_say). */
static char doze_sent_what[4][48];
static unsigned doze_sent_n;
static void
doze_note_send(const unsigned char *f, unsigned long n)
{
	char *w = doze_sent_what[doze_sent_n++ % 4];
	unsigned type = n >= 14 ? (unsigned)f[12] << 8 | f[13] : 0;

	if (type == 0x0806 && n >= 42) {
		snprintf(w, 48, "arp op %u for %u.%u.%u.%u", (unsigned)f[20] << 8 | f[21], f[38], f[39], f[40], f[41]);
	} else if (type == 0x0800 && n >= 34) {
		unsigned ihl = (f[14] & 15) * 4u, port = n >= 14 + ihl + 4 ? (unsigned)f[14 + ihl + 2] << 8 | f[14 + ihl + 3] : 0;
		snprintf(w, 48, "ip%s %u to %u.%u.%u.%u:%u", f[0] & 1 ? " (group)" : "", f[23], f[30], f[31], f[32], f[33], port);
	} else if (type == 0x86dd && n >= 58) {
		snprintf(w, 48, "ip6 %u to %02x%02x::%02x%02x:%u", f[20], f[38], f[39], f[52], f[53], (unsigned)f[56] << 8 | f[57]);
	} else {
		snprintf(w, 48, "type %#x len %lu", type, n);
	}
}

/* An Ethernet frame from the kernel for the network. 0, or an errno. */
static int
station_send(const unsigned char *frame, unsigned long length, unsigned flags)
{
	struct lx_radio *radio = the_radio;
	bool sent;

	if (radio == NULL || !radio->joined) {
		return (ENETDOWN);
	}
	lx_os_lock();
	if (doze.state != 0) {
		if (doze.queue != NULL && doze.queued < DOZE_QUEUE && length <= DOZE_FRAME) {
			memcpy(doze.queue + doze.queued * DOZE_FRAME, frame, length);
			doze.lengths[doze.queued] = (unsigned)length;
			doze.flags[doze.queued++] = flags;
		} else {
			doze.queue_lost++;
		}
		if (!doze.wanted) {
			doze.by_send++;
			doze_note_send(frame, length);
		}
		doze.wanted = true;
		lx_os_wakeup(&doze);
		lx_os_unlock();
		return (0);
	}
	doze.sending++;
	lx_os_unlock();
	sent = ethernet_send(radio, frame, (unsigned)length, flags);
	lx_os_lock();
	doze.sending--;
	doze.active_at = lx_os_nanoseconds();
	lx_os_unlock();
	return (sent ? 0 : ENOBUFS);
}

int
lx_station_send(const unsigned char *frame, unsigned long length, unsigned flags)
{
	struct lx_radio *radio = the_radio;
	int error;

	/*
	 * (counted as the receive path's frames are, so that station_leave() and station_down() do not free the station, the
	 * interface or the key under it; the driver's tx takes only its spinlocks, so the leaving's bounded wait is not held up)
	 */
	if (radio == NULL || !station_use(radio)) {
		return (ENETDOWN);
	}
	error = station_send(frame, length, flags);
	station_done(radio);
	return (error);
}

/* ---- doze ---- */
static void
doze_say(void)
{
	char text[2048];
	unsigned long long dozed = doze.dozed_ns + (doze.state == 2 ? lx_os_nanoseconds() - doze.since : 0);

	char extra[1750] = "";
	int at = 0;

	snprintf(text, sizeof(text), "state %d dozes %u wakes %u (line %u msi %u send %u call %u other %u) failed: suspend %u sleep %u "
	    "wake %u resume %u%s; queue lost %u; dozed_ms %llu longest_ms %llu last_wake_us %llu; ps %u (changes %u)", doze.state,
	    doze.dozes, doze.wakes, doze.by_line, doze.by_msi, doze.by_send, doze.by_call, doze.by_other, doze.failed_suspend, doze.failed_sleep,
	    doze.failed_wake, doze.failed_resume, doze.given_up ? " (given up)" : "", doze.queue_lost, dozed / 1000000ull, doze.longest_ns / 1000000ull,
	    doze.last_wake_ns / 1000ull, doze.ps_applied, doze.ps_changes);
	{
		unsigned ev, evp, bhi, kicks, net;
		lx_mhi_m3_irqs(&ev, &evp, &bhi, &kicks, &net);
		at += snprintf(extra + at, sizeof(extra) - (size_t)at, "; M3 irqs: event %u (with new events %u) bhi %u, kicks %u; "
		    "sweeps %u", ev, evp, bhi, kicks, net);
	}
	if (the_radio != NULL) {
		struct lx_radio *r = the_radio;
		at += snprintf(extra + at, sizeof(extra) - (size_t)at, "; rx: for us %u, to all %u, 802.11 made ethernet %u, dropped data %u",
		    r->received, r->received_group, rx_8021_ok, rx_dropped_data);
		at += snprintf(extra + at, sizeof(extra) - (size_t)at, ", of them broadcast %u; as 802.11: addr1 a group %u "
		    "(broadcast %u), ours %u (DA a group %u), A-MSDU %u, 4 addresses %u, decrypted %u; as 802.3 to a group %u",
		    rx_broadcast, rx_air_group, rx_air_group_bc, rx_air_ours, rx_air_ours_group_da, rx_air_amsdu, rx_air_4addr,
		    rx_air_decrypted, rx_8023_group);
		{
			int lx_bus_rx_errors(char *text, size_t size);
			at += lx_bus_rx_errors(extra + at, sizeof(extra) - (size_t)at);
		}
		if (rx_dropped_data) {
			at += snprintf(extra + at, sizeof(extra) - (size_t)at, " (first: len %u flags %#x head", rx_dropped_len, rx_dropped_flags);
			for (unsigned i = 0; i < 36 && at < (int)sizeof(extra) - 4; i++)
				at += snprintf(extra + at, sizeof(extra) - (size_t)at, "%s%02x", i % 4 ? "" : " ", rx_dropped_head[i]);
			at += snprintf(extra + at, sizeof(extra) - (size_t)at, ")");
		}
		at += snprintf(extra + at, sizeof(extra) - (size_t)at, "; rekeys %u", r->rekeys);
	}
	at += snprintf(extra + at, sizeof(extra) - (size_t)at, "; MHI busy %u, undo failed %u", doze.not_ready, doze.failed_undo);
	at += snprintf(extra + at, sizeof(extra) - (size_t)at, "; woken by sending");
	for (unsigned k = doze_sent_n > 4 ? doze_sent_n - 4 : 0; k < doze_sent_n && at < (int)sizeof(extra) - 52; k++)
		at += snprintf(extra + at, sizeof(extra) - (size_t)at, " [%s]", doze_sent_what[k % 4]);
	{
		extern unsigned lx_wow_flags_sent, lx_wake_msi_vector;
		at += snprintf(extra + at, sizeof(extra) - (size_t)at, "; wow flags %#x; wake msi vector %d: %llu", lx_wow_flags_sent,
		    (int)lx_wake_msi_vector, lx_wake_msi_vector < 32 ? lx_os_pci_msi_count(lx_wake_msi_vector) : 0ull);
	}
	at += snprintf(extra + at, sizeof(extra) - (size_t)at, "; networks lost while dozing %u; instant %u; reasons in all: unspecified %u",
	    doze.lost_wakes, doze.instant, doze.reason_unspecified);
	for (unsigned r = 0; r < 64 && at < (int)sizeof(extra) - 16; r++)
		if (doze.reason_count[r]) at += snprintf(extra + at, sizeof(extra) - (size_t)at, ", %u x%u", r, doze.reason_count[r]);
	at += snprintf(extra + at, sizeof(extra) - (size_t)at, "; patterns %#x; solicited patterns %d; reasons",
	    doze.patterns_used, doze.solicited_patterns);
	for (unsigned k = doze.reason_n > 8 ? doze.reason_n - 8 : 0; k < doze.reason_n && at < (int)sizeof(extra) - 32; k++)
		at += snprintf(extra + at, sizeof(extra) - (size_t)at, " %u:%s", doze.reason_last[k % 8], doze.reason_name[k % 8]);
	at += snprintf(extra + at, sizeof(extra) - (size_t)at, "; dozed last ms");
	for (unsigned k = doze.dozed_n > 8 ? doze.dozed_n - 8 : 0; k < doze.dozed_n && at < (int)sizeof(extra) - 12; k++)
		at += snprintf(extra + at, sizeof(extra) - (size_t)at, " %u", doze.dozed_last_ms[k % 8]);
	strlcat(text, extra, sizeof(text));
	lx_os_doze_stats(text);
}

void lx_doze_reason(int reason, const char *name);
void
lx_doze_reason(int reason, const char *name)
{
	unsigned i = doze.reason_n++ % 8;

	doze.reason_last[i] = (unsigned)reason;
	strlcpy(doze.reason_name[i], name != NULL ? name : "?", sizeof(doze.reason_name[i]));
	if (reason >= 0 && reason < 64) doze.reason_count[reason]++;
	if (reason < 0) doze.reason_unspecified++;
	/* the firmware took the frame that ended the association while we dozed (run64: DEAUTH_RECVD, then nothing told
	 * Darwin, which kept a network that was gone): AP_ASSOC_LOST 1 (beacons), DEAUTH_RECVD 3, DISASSOC_RECVD 4,
	 * CSA_EVENT 13 (a channel switch WoW cannot follow) */
	if (reason == 1 || reason == 3 || reason == 4 || reason == 13) doze.lost_by = reason + 1;
}

/* After a doze: the network lost while dozing, said as the frame itself would have (Darwin rejoins). */
static void
doze_lost(struct lx_radio *radio)
{
	int reason = doze.lost_by - 1;

	doze.lost_by = 0;
	if (reason < 0 || radio == NULL || !radio->joined) return;
	doze.lost_wakes++;
	lx_tell("lx: THE NETWORK WAS LOST WHILE DOZING (the radio woke us for %s)\n", reason == 3 ? "a deauthentication" :
	    reason == 4 ? "a disassociation" : reason == 13 ? "a channel switch" : "beacons lost");
	radio->joined = false;
	radio->last_error = reason == 1 || reason == 13 ? ETIMEDOUT : ECONNRESET;
	radio->loss = reason == 3 ? HAKIOS_STA_LOSS_DEAUTH : reason == 4 ? HAKIOS_STA_LOSS_DISASSOC : HAKIOS_STA_LOSS_BEACONS;
	radio->loss_reason = 0;
	lx_os_net_link(0);
	lx_os_station_lost((int)radio->last_error);
}

/* The radio's MHI interrupt while in M3 (lx_mhi.c): with the link handed to the DSP that is how it asks for the host. */
void lx_mhi_m3_irqs(unsigned *ev, unsigned *ev_pending, unsigned *bhi, unsigned *kicks, unsigned *net);
/* the counts as they are now (an interrupt in M3: rare, and the line is what the run reads) */
void lx_doze_say_live(void);
void lx_doze_say_live(void) { doze_say(); }
void lx_doze_kick(void);
void
lx_doze_kick(void)
{
	lx_os_lock();
	if (doze.state != 0) {
		if (!doze.wanted) doze.by_msi++;
		doze.wanted = true;
		lx_os_wakeup(&doze);
	}
	lx_os_unlock();
}

/* In interrupt context (the PDC's line of WAKE#, masked by then). */
static void
doze_line(void)
{
	if (!doze.wanted) doze.by_line++;
	doze.wanted = true;
	lx_os_wakeup(&doze);
}

/* The radio as the station thread's calls need it: awake. Under no lock; may wait for the doze thread. */
static void
doze_call_begin(const char *operation)
{
	lx_os_lock();
	doze.calls++;
	while (doze.state != 0) {
		if (!doze.wanted) {
			doze.by_call++;
			lx_tell("lx: standby host wake: %s (state %u)\n", operation, doze.state);
		}
		doze.wanted = true;
		lx_os_wakeup(&doze);
		(void)lx_os_sleep(&doze.state, 0);
	}
	lx_os_unlock();
}

static void
doze_call_end(void)
{
	lx_os_lock();
	doze.calls--;
	doze.active_at = lx_os_nanoseconds();
	lx_os_unlock();
}

static void
doze_add_pattern(const uint8_t *address, unsigned length)
{
	static const uint8_t address_mask[1] = { 0x3f }, prefix_mask[1] = { 0x07 };
	unsigned index = doze.wowlan.n_patterns++;

	/* Pad the backing buffer for ath11k's 802.3 to 802.11 conversion. */
	memset(doze.pattern_bytes[index], 0, sizeof(doze.pattern_bytes[index]));
	memcpy(doze.pattern_bytes[index], address, length);
	doze.patterns[index] = (struct cfg80211_pkt_pattern){
		.mask = length == 6 ? address_mask : prefix_mask,
		.pattern = doze.pattern_bytes[index],
		.pattern_len = length,
	};
}

static void
doze_patterns(struct lx_radio *radio)
{
	unsigned mask = lx_os_doze_patterns();
	uint8_t ipv6[DOZE_SOLICITED_MAX][16];
	int count = (mask & 2) ? lx_os_wifi_ipv6_addresses(ipv6, DOZE_SOLICITED_MAX) : 0;

	/* Unicast for us and neighbor discovery for our IPv6 addresses. ARP is offloaded. */
	bzero(&doze.wowlan, sizeof(doze.wowlan));
	doze.wowlan.patterns = doze.patterns;
	doze.wowlan.disconnect = (mask & 4) != 0;
	doze.wowlan.gtk_rekey_failure = (mask & 8) != 0;
	doze.patterns_used = mask;
	doze.solicited_patterns = count;
	lx_bus_wow_ns(radio->vif, ipv6, count > 0 ? (unsigned)count : 0);
	if (mask & 1) doze_add_pattern(radio->vif->addr, 6);
	if (mask & 2) {
		uint8_t address[6] = { 0x33, 0x33, 0xff };

		if (count < 0) {
			/* Preserve reachability if the address list could not fit or be read. */
			doze_add_pattern(address, 3);
		} else {
			for (int i = 0; i < count; i++) {
				memcpy(address + 3, ipv6[i] + 13, 3);
				doze_add_pattern(address, 6);
			}
		}
	}
	if (doze.wowlan.n_patterns == 0) doze.wowlan.patterns = NULL;
}

/* awake -> dozing. state is 1. 0 when dozing. */
static int
doze_enter(struct lx_radio *radio)
{
	unsigned ip;
	int ret, error;

	if (lx_os_wifi_ipv4(&ip) == 0 && (radio->vif->cfg.arp_addr_cnt != 1 || radio->vif->cfg.arp_addr_list[0] != ip)) {
		radio->vif->cfg.arp_addr_list[0] = ip;
		radio->vif->cfg.arp_addr_cnt = 1;
		bss_changed(radio, BSS_CHANGED_ARP_FILTER);
	}
	doze_patterns(radio);
	ath11k_debug_mask |= 0x2u; /* ATH11K_DBG_WMI */	/* (its wake reason line: lx_doze_reason) */
	if (!lx_mhi_suspend_ready()) {
		doze.not_ready++;
		return (EBUSY);		/* (MHI busy: the doze is tried again later, nothing of the driver's touched) */
	}
	unsigned asked = lx_mhi_suspend_calls();
	mutex_lock(&radio->wiphy->mtx);
	lx_doze_in_call = true;
	ret = radio->ops->suspend(&radio->hw, &doze.wowlan);
	lx_doze_in_call = false;
	mutex_unlock(&radio->wiphy->mtx);
	if (ret != 0) {
		/*
		 * ath11k_wow_op_suspend() failing late (hif_suspend) undoes only the WoW enable: the data path's interrupts stay
		 * off, the hw filter, ARP/NS offload and keepalive stay set (run56: transmit dead after "failed: suspend 1");
		 * mac80211 would rebuild the hardware (its return of 1: ieee80211_reconfig). lx_bus_wow_undo() undoes what the
		 * failed step left: the interrupts only if MHI was asked for M3 (hif_suspend, after they were turned off).
		 */
		bool mhi_asked = lx_mhi_suspend_calls() != asked;
		int undo = lx_bus_wow_undo(mhi_asked);

		doze.failed_suspend++;
		if (undo != 0) doze.failed_undo++;
		lx_tell("lx: doze: the driver's suspend failed (%d, %s); undone: %d\n", ret, mhi_asked ? "in hif_suspend" :
		    "before hif_suspend", undo);
		return (EIO);
	}
	error = lx_os_pcie_sleep(lx_mhi_in_fast_m3());
	if (error != 0) {
		doze.failed_sleep++;
		lx_tell("lx: doze: the link's sleep failed (%d); the radio resumed\n", error);
		(void)lx_os_pcie_wake();
		mutex_lock(&radio->wiphy->mtx);
		(void)radio->ops->resume(&radio->hw);
		mutex_unlock(&radio->wiphy->mtx);
		return (error);
	}
	return (0);
}

/* dozing -> awake. state is 1. */
/*
 * What the firmware sent while a doze was being left (the driver's WMI lines, which lx_bus.c keeps out of the log during
 * the doze's calls): each line's format counted, and said with the wake for the first 16 wakes and each power of two
 * after. run67: 104 of 117 wakes were the radio's own PME, most ~80 ms after D3hot, nearly all "unspecified" -- what the
 * firmware delivers once the host is back is what it woke us for.
 */
static struct { const char *format; unsigned n; } doze_seen[10];
static unsigned doze_seen_n, doze_seen_more;
static volatile bool doze_seeing;
void lx_doze_note(const char *format);
void
lx_doze_note(const char *format)
{
	if (!doze_seeing) return;
	for (unsigned i = 0; i < doze_seen_n; i++) {
		if (doze_seen[i].format == format) { doze_seen[i].n++; return; }
	}
	if (doze_seen_n < 10) { doze_seen[doze_seen_n].format = format; doze_seen[doze_seen_n++].n = 1; }
	else doze_seen_more++;
}
static void
doze_seen_say(unsigned wake, unsigned long long dozed_ms, unsigned reasons_before)
{
	char line[600];
	int at;

	if (wake > 16 && (wake & (wake - 1)) != 0) return;
	at = snprintf(line, sizeof(line), "lx: wake %u after %llu ms dozing; reason %s; the driver heard:", wake, dozed_ms,
	    doze.reason_n != reasons_before ? doze.reason_name[(doze.reason_n - 1) % 8] : "(none)");
	for (unsigned i = 0; i < doze_seen_n && at < (int)sizeof(line) - 48; i++) {
		char head[30];
		unsigned k = 0;
		for (; k < sizeof(head) - 1 && doze_seen[i].format[k] && doze_seen[i].format[k] != '\n'; k++) head[k] = doze_seen[i].format[k];
		head[k] = 0;
		at += snprintf(line + at, sizeof(line) - (size_t)at, " [%s x%u]", head, doze_seen[i].n);
	}
	if (doze_seen_more) at += snprintf(line + at, sizeof(line) - (size_t)at, " (+%u more)", doze_seen_more);
	lx_tell("%s\n", line);
}

static void
doze_leave(struct lx_radio *radio)
{
	unsigned long long began = lx_os_nanoseconds();
	int error, ret = 0;

	if (radio == NULL) {
		(void)lx_os_pcie_wake();
		return;
	}
	error = lx_os_pcie_wake();
	if (error != 0) {
		/* (once more: a link that does not train the first time) */
		lx_pause_ms(20);
		error = lx_os_pcie_wake();
	}
	if (error != 0) {
		doze.failed_wake++;
		lx_tell("lx: doze: THE LINK DID NOT COME BACK (%d)\n", error);
		return;
	}
	unsigned reasons_before = doze.reason_n;
	unsigned long long dozed_ms = (began - doze.since) / 1000000ull;
	doze_seen_n = 0;
	doze_seen_more = 0;
	doze_seeing = true;
	mutex_lock(&radio->wiphy->mtx);
	lx_doze_in_call = true;
	ret = radio->ops->resume(&radio->hw);
	lx_doze_in_call = false;
	mutex_unlock(&radio->wiphy->mtx);
	doze_seeing = false;
	if (ret != 0) {
		doze.failed_resume++;
		doze.given_up = true;
		lx_tell("lx: doze: the driver's resume said %d: no more dozes this boot, the radio stays awake\n", ret);
	}
	doze.last_wake_ns = lx_os_nanoseconds() - began;
	doze_seen_say(doze.wakes + 1, dozed_ms, reasons_before);
	after.wake = doze.wakes + 1;
	after.n = after.more = 0;
	after.left_at = lx_os_nanoseconds();
}

static void
doze_ps(struct lx_radio *radio, bool on)
{
	if (doze.ps_known && doze.ps_applied == on) return;
	radio->vif->cfg.ps = on;
	bss_changed(radio, BSS_CHANGED_PS);
	doze.ps_applied = on;
	doze.ps_known = true;
	doze.ps_changes++;
	doze_say();
}

static void
doze_runs(void *argument)
{
	(void)argument;
	lx_os_lock();
	for (;;) {
		struct lx_radio *radio = the_radio;
		unsigned on = 0, idle_ms = 3000, ps = 0;
		unsigned long long now = lx_os_nanoseconds(), wait;
		bool joined = radio != NULL && radio->joined && radio->vif != NULL;

		lx_os_doze_config(&on, &idle_ms, &ps);
		if (doze.given_up) on = 0;
		{
			unsigned mode;
			if (joined && doze.state == 0 && lx_os_rx_filter_asked(&mode)) {
				int lx_bus_rx_filter(unsigned mode);
				int r;
				lx_os_unlock();
				r = lx_bus_rx_filter(mode);
				lx_tell("lx: the firmware's receive data filter set to %u: %d\n", mode, r);
				lx_os_lock();
			}
		}
		if (doze.state != 2) doze_say();	/* (the counts fresh each pass; a write of wifi_doze refreshes them) */
		if (doze.state == 2) {
			if (doze.wanted || !on || !joined || !lx_os_screen_off() || lx_os_pcie_wake_line()) {
				if (!doze.wanted) doze.by_other++;
				doze.state = 1;
				lx_os_pcie_wake_arm(0);
				lx_os_unlock();
				doze_leave(radio);
				doze_lost(radio);
				lx_os_lock();
				now = lx_os_nanoseconds();
				doze.dozed_last_ms[doze.dozed_n++ % 8] = (unsigned)((now - doze.since) / 1000000ull);
				doze.dozed_ns += now - doze.since;
				if (now - doze.since < 5000000ull) doze.instant++;
				if (now - doze.since > doze.longest_ns) doze.longest_ns = now - doze.since;
				doze.wakes++;
				doze.wanted = false;
				doze.state = 0;
				doze.active_at = now;
				lx_os_wakeup(&doze.state);
				/* the frames that waited: sent now, in order (outside the lock: sending may sleep) */
				for (unsigned i = 0; i < doze.queued; i++) {
					doze.sending++;
					lx_os_unlock();
					(void)ethernet_send(radio, doze.queue + i * DOZE_FRAME, doze.lengths[i], doze.flags[i]);
					lx_os_lock();
					doze.sending--;
				}
				doze.queued = 0;
				doze_say();
				continue;
			}
			(void)lx_os_sleep(&doze, 60ull * 1000000000ull);	/* (WAKE# wakes it; this only catches a lost wakeup) */
			continue;
		}
		if (joined && doze.calls == 0 && doze.sending == 0 && (bool)ps != doze.ps_applied) {
			doze.calls++;	/* (the station's calls wait meanwhile) */
			lx_os_unlock();
			doze_ps(radio, ps != 0);
			lx_os_lock();
			doze.calls--;
		}
		if (on && joined && lx_os_screen_off() && doze.calls == 0 && doze.sending == 0 && doze.queued == 0 &&
		    now >= doze.backoff_until && now - doze.active_at >= (unsigned long long)idle_ms * 1000000ull) {
			int error;

			unsigned long long entering = lx_os_nanoseconds();
			doze.state = 1;
			doze.wanted = false;
			lx_os_unlock();
			error = doze_enter(radio);
			lx_os_lock();
			after.enter_ns = lx_os_nanoseconds() - entering;
			if (error == 0) after_say();
			if (error != 0) {
				doze.state = 0;
				/* (MHI busy: soon again; a failure: half a minute) */
				doze.backoff_until = lx_os_nanoseconds() + (error == EBUSY ? 1ull : 30ull) * 1000000000ull;
				lx_os_wakeup(&doze.state);
			} else {
				doze.state = 2;
				doze.since = lx_os_nanoseconds();
				doze.dozes++;
				lx_os_pcie_wake_arm(1);
			}
			doze_say();
			continue;
		}
		if (!on || !joined || !lx_os_screen_off()) {
			wait = 5ull * 1000000000ull;
		} else {
			unsigned long long idle = now - doze.active_at, need = (unsigned long long)idle_ms * 1000000ull;
			wait = idle >= need ? 1000000000ull : need - idle;
			if (wait < 100000000ull) wait = 100000000ull;
		}
		(void)lx_os_sleep(&doze, wait);
	}
}

/* Once, at the first join. */
static void
doze_start(void)
{
	if (doze.thread) return;
	doze.queue = kmalloc((size_t)DOZE_QUEUE * DOZE_FRAME, GFP_KERNEL);
	if (doze.queue == NULL) return;
	(void)lx_os_pcie_wake_irq(doze_line);
	doze.active_at = lx_os_nanoseconds();
	lx_os_doze_event(&doze);	/* (a change of wifi_doze wakes the thread at once) */
	if (lx_os_thread("wifi-doze", doze_runs, NULL) == 0) doze.thread = true;
	doze_say();
}

/* ---- the station boundary: what the kernel asks through sys/board/sta.h --------------------------------- */

static int boundary_power_awake(bool on);
static int
boundary_power(bool on)
{
	int error;
	doze_call_begin("power change");
	error = boundary_power_awake(on);
	doze_call_end();
	return error;
}
static int
boundary_power_awake(bool on)
{
	struct lx_radio *radio = the_radio;
	if (lx_os_radio_model() == 1 && lx_os_option("wifi_pcie_off")) {
		int error = lx_pci_runtime_power(on);
		if (error) lx_os_runtime_state(5, error);
		if (error || !on) return error;
		radio = the_radio; /* a fresh radio after full PCIe reprobe */
	}

	if (radio == NULL || !radio->registered) {
		return (ENXIO);
	}
	if (on) {
		int error = station_up(radio);
		if (error && lx_os_radio_model() == 1 && lx_os_option("wifi_pcie_off"))
			(void)lx_pci_runtime_power(false);
		if (lx_os_radio_model() == 1) lx_os_runtime_state(error ? 5 : 4, error);
		return error;
	}
	if (radio->sta != NULL) {
		station_leave(radio);
		lx_tell("lx: the network is left\n");
	}
	station_down(radio);
	return (0);
}

static int boundary_country(const char *asked, char *now);
static bool country_by_user;		/* wifid (or whoever) set a country: the beacons no longer choose it */

/*
 * 802.11d, as Linux's cfg80211 does with the country elements (7) of the access points it hears: the
 * country most of them name becomes the radio's, unless one was set from above. Without it the board
 * data's domain stays (US on the NE2213), and a network on a channel that domain leaves out (13 at
 * 2.4 GHz in Europe) is never heard. True when the domain changed: the caller scans again.
 */
static bool
adopt_heard_country(struct lx_radio *radio)
{
	char codes[8][2], now[2];
	unsigned votes[8] = { 0 }, kinds = 0, best = 0;

	if (country_by_user) {
		return (false);
	}
	lx_os_lock();
	for (unsigned i = 0; i < radio->heard_count; i++) {
		const struct lx_heard *network = &radio->heard[i];
		const uint8_t *e = element(network->elements, network->elements_length, 7);
		unsigned k;

		if (e == NULL || e[1] < 3 || e[2] < 'A' || e[2] > 'Z' || e[3] < 'A' || e[3] > 'Z' || (e[2] == 'X' && e[3] == 'X')) {
			continue;
		}
		for (k = 0; k < kinds && memcmp(codes[k], e + 2, 2) != 0; k++) {
		}
		if (k == kinds) {
			if (kinds == nitems(codes)) {
				continue;
			}
			memcpy(codes[kinds++], e + 2, 2);
		}
		votes[k]++;
	}
	now[0] = radio->regd != NULL ? radio->regd->alpha2[0] : 0;
	now[1] = radio->regd != NULL ? radio->regd->alpha2[1] : 0;
	lx_os_unlock();
	for (unsigned k = 1; k < kinds; k++) {
		best = votes[k] > votes[best] ? k : best;
	}
	if (kinds == 0 || memcmp(codes[best], now, 2) == 0) {
		return (false);
	}
	lx_tell("lx: %u of the access points heard name the country %c%c (the domain in force: %c%c): it is taken\n", votes[best],
	    codes[best][0], codes[best][1], now[0] ? now[0] : '-', now[1] ? now[1] : '-');
	return (boundary_country(codes[best], NULL) == 0);
}

static int
boundary_scan(const struct hakios_sta_scan *what)
{
	static unsigned said;
	struct lx_radio *radio = the_radio;
	unsigned long long began;
	int error;

	if (radio == NULL || !radio->registered) {
		return (ENXIO);
	}
	if ((error = station_up(radio)) != 0) {
		return (error);
	}
	began = lx_os_nanoseconds();
	error = station_scan(radio, what->ssid, what->ssid_length <= sizeof(what->ssid) ? what->ssid_length : 0, what->passive, what);
	if (error == 0 && adopt_heard_country(radio)) {
		/*
		 * (the new domain's channels: once more, so what is on them is in the answer. The driver sends the firmware
		 * its new channel list from work of its own: a scan begun meanwhile hears nothing, so a moment first, and
		 * again if it still heard nothing)
		 */
		for (unsigned again = 0; again < 3; again++) {
			lx_os_lock();
			(void)lx_os_sleep(&again, 500 * 1000000ull);	/* (nobody wakes it: the time only) */
			lx_os_unlock();
			error = station_scan(radio, what->ssid, what->ssid_length <= sizeof(what->ssid) ? what->ssid_length : 0,
			    what->passive, what);
			if (error == 0 && radio->heard_count != 0) {
				break;
			}
		}
	}
	if (said < 24) {			/* (the first few are told: whoever asks goes on asking as long as the system runs) */
		said++;
		lx_tell("lx: a scan for the kernel: %u channels named (0: all), %s, %u ms on each (0: the radio's own): %d; %u access "
		    "points heard, %u frames, in %llu ms\n", what->frequency_count, what->passive ? "listening" : "asking",
		    what->dwell_ms, error, radio->heard_count, radio->frames, (lx_os_nanoseconds() - began) / 1000000ull);
	}
	return (error);
}

static void
boundary_heard(void (*each)(const struct hakios_sta_bss *, void *), void *context)
{
	static struct hakios_sta_bss bss;	/* (one thread calls this) */
	struct lx_radio *radio = the_radio;

	for (unsigned i = 0; radio != NULL && i < radio->heard_count; i++) {
		const struct lx_heard *network = &radio->heard[i];

		lx_os_lock();
		bzero(&bss, sizeof(bss));
		memcpy(bss.bssid, network->sender, ETH_ALEN);
		memcpy(bss.ssid, network->name, network->name_length);
		bss.ssid_length = network->name_length;
		/* (a beacon can be heard on the channel next to its own: where it says it is counts) */
		bss.frequency = network->says_channel != 0 ? (uint16_t)linuxkpi_ieee80211_channel_to_frequency(network->says_channel,
		    network->frequency < 4000 ? NL80211_BAND_2GHZ : NL80211_BAND_5GHZ) : network->frequency;
		bss.rssi = network->signal;
		bss.beacon_interval = network->interval;
		bss.capability = network->capability;
		bss.age_ms = (uint32_t)((lx_os_nanoseconds() - network->last) / 1000000ull);
		bss.elements_length = network->elements_length;
		memcpy(bss.elements, network->elements, network->elements_length);
		lx_os_unlock();
		each(&bss, context);
	}
}

static int
boundary_join(const struct hakios_sta_join *what)
{
	struct lx_radio *radio = the_radio;
	int error;

	if (radio == NULL || !radio->registered) {
		return (ENXIO);
	}
	radio->joining = true;
	radio->loss = HAKIOS_STA_LOSS_NONE;
	radio->loss_reason = 0;
	radio->signal_rest_until = 0;
	error = station_join(radio, what);
	radio->last_error = error;
	radio->joining = false;
	return (error);
}

static int
boundary_leave(void)
{
	struct lx_radio *radio = the_radio;

	if (radio != NULL && radio->sta != NULL) {
		lx_station_say();
		station_leave(radio);
		radio->last_error = 0;
		radio->loss = HAKIOS_STA_LOSS_NONE;
		radio->loss_reason = 0;
		lx_tell("lx: the network is left\n");
	}
	return (0);
}

/* (on any thread: nothing here but reading) */
static void
boundary_state(struct hakios_sta_state *state)
{
	struct lx_radio *radio = the_radio;

	bzero(state, sizeof(*state));
	if (radio == NULL || radio->vif == NULL) {
		return;
	}
	lx_os_lock();
	state->phase = radio->joined ? HAKIOS_STA_JOINED : radio->joining ? HAKIOS_STA_JOINING : radio->scan == 1 ?
	    HAKIOS_STA_SCANNING : HAKIOS_STA_IDLE;
	state->last_error = radio->last_error;
	state->security = radio->wpa.owe ? HAKIOS_STA_OWE : radio->wpa.sae ? HAKIOS_STA_WPA3_SAE : HAKIOS_STA_WPA2_PSK;
	state->loss = radio->loss;
	state->reason = radio->loss_reason;
	if ((radio->joined || radio->joining) && radio->target.found) {
		memcpy(state->bss.bssid, radio->target.address, ETH_ALEN);
		memcpy(state->bss.ssid, radio->want, radio->want_length);
		state->bss.ssid_length = radio->want_length;
		state->bss.frequency = radio->channel != NULL && radio->channel->def.chan != NULL ?
		    radio->channel->def.chan->center_freq : radio->target.frequency;
		state->bss.rssi = radio->joined ? radio->signal : radio->target.signal;
		state->bss.beacon_interval = radio->target.interval;
		state->bss.capability = radio->target.capability;
		state->bss.elements_length = (uint16_t)radio->target.elements_length;
		memcpy(state->bss.elements, radio->target.elements, radio->target.elements_length);
	}
	lx_os_unlock();
}

/*
 * How strongly the network joined is heard now, as the driver measures it: its sta_statistics(), as mac80211
 * asks it for station info -- the combined RSSI of the last frames, else the firmware's beacon RSSI (a
 * WMI_REQUEST_VDEV_STAT round trip). On the station's thread (board/sta.c), once a second while joined. When
 * the driver has no number, or takes more than half a second for it, it is left alone for 30 s and the last
 * beacon's signal is taken instead (ENODATA).
 */
static int
boundary_signal(int *dbm)
{
	struct lx_radio *radio = the_radio;
	struct station_info *info;
	unsigned long long began = lx_os_nanoseconds();
	int error = ENODATA;

	if (radio == NULL || !radio->joined || radio->sta == NULL || radio->vif == NULL) {
		return (ENOTCONN);
	}
	if (radio->ops->sta_statistics == NULL || began < radio->signal_rest_until ||
	    (info = kzalloc(sizeof(*info), GFP_KERNEL)) == NULL) {
		return (ENODATA);
	}
	mutex_lock(&radio->wiphy->mtx);
	if (radio->joined && radio->sta != NULL) {
		radio->ops->sta_statistics(&radio->hw, radio->vif, radio->sta, info);
	}
	mutex_unlock(&radio->wiphy->mtx);
	if ((info->filled & BIT_ULL(NL80211_STA_INFO_SIGNAL)) && info->signal < 0 && info->signal > -128) {
		*dbm = info->signal;
		lx_os_lock();
		radio->signal = info->signal;
		lx_os_unlock();
		error = 0;
	}
	if (error != 0 || lx_os_nanoseconds() - began > 500000000ull) {
		radio->signal_rest_until = lx_os_nanoseconds() + 30000000000ull;
	}
	if (radio->signal_said < 8 || (error != 0 && radio->signal_said < 40)) {
		radio->signal_said++;
		lx_tell("lx: the driver measures the network's signal: %s%d dBm (the last beacon: %d) in %llu ms\n", error == 0 ? "" :
		    "NONE, ", error == 0 ? *dbm : 0, radio->target.signal, (lx_os_nanoseconds() - began) / 1000000ull);
	}
	kfree(info);
	return (error);
}

static unsigned
boundary_channels(struct hakios_sta_channel *channels, unsigned most)
{
	struct lx_radio *radio = the_radio;
	unsigned count = 0;

	for (unsigned band = 0; radio != NULL && band < NUM_NL80211_BANDS; band++) {
		const struct ieee80211_supported_band *has = radio->wiphy->bands[band];

		for (int i = 0; has != NULL && i < has->n_channels && count < most; i++) {
			if (!(has->channels[i].flags & IEEE80211_CHAN_DISABLED)) {
				channels[count].frequency = (uint16_t)has->channels[i].center_freq;
				channels[count++].flags = (uint16_t)((has->channels[i].flags & IEEE80211_CHAN_NO_IR ? HAKIOS_STA_CHANNEL_PASSIVE :
				    0) | (has->channels[i].flags & IEEE80211_CHAN_RADAR ? HAKIOS_STA_CHANNEL_RADAR : 0));
			}
		}
	}
	return (count);
}

/*
 * The country the radio is in (two letters, ISO 3166-1), as iOS has worked it out (wifid: from the
 * country elements of the beacons it hears). Asked of the driver the way Linux's cfg80211 asks it
 * for a user's hint: its reg_notifier, which hands the firmware the country (WMI
 * SET_CURRENT_COUNTRY on WCN6855, INIT_COUNTRY on the others); the firmware answers with the domain
 * of that country, which comes to linuxkpi_regulatory_set_wiphy_regd_sync() above and changes
 * the channels. Waits for that domain: 0 once the radio is in the country, ETIMEDOUT if the
 * firmware never says so, EINVAL for a code that is not two letters, ENXIO without a radio.
 * `now` (may be NULL) gets the domain's two letters in either case.
 */
static int
boundary_country(const char *asked, char *now)
{
	struct lx_radio *radio = the_radio;
	struct regulatory_request request;
	unsigned long long began;
	char want[2];
	int error = 0;

	if (radio == NULL || !radio->registered) {
		return (ENXIO);
	}
	if (asked != NULL) {
		for (int i = 0; i < 2; i++) {
			if (!((asked[i] >= 'A' && asked[i] <= 'Z') || (asked[i] >= 'a' && asked[i] <= 'z'))) {
				return (EINVAL);
			}
			want[i] = asked[i] & ~0x20;	/* (upper case, as the firmware's domains have it) */
		}
		lx_os_lock();
		error = radio->regd != NULL && memcmp(radio->regd->alpha2, want, 2) == 0 ? -1 : 0;
		lx_os_unlock();
		if (error == 0 && radio->wiphy->reg_notifier == NULL) {
			error = EOPNOTSUPP;
		} else if (error == 0) {
			bzero(&request, sizeof(request));
			memcpy(request.alpha2, want, 2);
			request.initiator = NL80211_REGDOM_SET_BY_USER;
			request.user_reg_hint_type = NL80211_USER_REG_HINT_USER;
			began = lx_os_nanoseconds();
			radio->wiphy->reg_notifier(radio->wiphy, &request);
			lx_os_lock();
			while ((radio->regd == NULL || memcmp(radio->regd->alpha2, want, 2) != 0) &&
			    lx_os_nanoseconds() - began < 5000000000ull) {
				lx_os_sleep(&radio->regd, 100 * 1000000ull);
			}
			error = radio->regd != NULL && memcmp(radio->regd->alpha2, want, 2) == 0 ? 0 : ETIMEDOUT;
			lx_os_unlock();
			if (error == 0) {
				/*
				 * What cfg80211 does once a self-managed radio's new domain is set (reg.c,
				 * reg_process_self_managed_hint): the notifier again, as the driver's. ath11k then
				 * sends the firmware the new channel list (ath11k_reg_update_chan_list); without it
				 * the firmware goes on scanning the old country's channels only.
				 */
				bzero(&request, sizeof(request));
				memcpy(request.alpha2, want, 2);
				request.initiator = NL80211_REGDOM_SET_BY_DRIVER;
				radio->wiphy->reg_notifier(radio->wiphy, &request);
			}
			lx_tell("lx: the country %c%c is asked of the firmware: %s in %llu ms\n", want[0], want[1], error == 0 ?
			    "its domain has come" : "NO DOMAIN of it has come", (lx_os_nanoseconds() - began) / 1000000ull);
		} else {
			error = 0;		/* (the radio is in it already) */
		}
	}
	if (now != NULL) {
		lx_os_lock();
		now[0] = radio->regd != NULL ? radio->regd->alpha2[0] : 0;
		now[1] = radio->regd != NULL ? radio->regd->alpha2[1] : 0;
		lx_os_unlock();
	}
	return (error);
}

/* the calls that need the radio awake: woken first (doze) */
static int boundary_scan_awake(const struct hakios_sta_scan *what)
{ int e; doze_call_begin("scan"); e = boundary_scan(what); doze_call_end(); return e; }
static int boundary_join_awake(const struct hakios_sta_join *what)
{ int e; doze_call_begin("join"); e = boundary_join(what); doze_call_end(); if (e == 0) doze_start(); return e; }
static int boundary_leave_awake(void)
{ int e; doze_call_begin("leave"); e = boundary_leave(); doze_call_end(); return e; }
static int boundary_country_awake(const char *asked, char *now)
{
	int error;
	if (asked == NULL) return boundary_country(NULL, now);
	country_by_user = true;
	doze_call_begin("country change");
	error = boundary_country(asked, now);
	doze_call_end();
	return error;
}
/* dozing: no measure (the station goes by the last beacon's); not woken for it */
static int boundary_signal_awake(int *dbm)
{
	int error;
	lx_os_lock();
	if (doze.state != 0) {
		lx_os_unlock();
		return EAGAIN;
	}
	doze.calls++;
	lx_os_unlock();
	error = boundary_signal(dbm);
	lx_os_lock();
	doze.calls--;
	lx_os_unlock();
	return error;
}

static struct hakios_sta_ops boundary = {
	.country = boundary_country_awake,
	.power = boundary_power,
	.scan = boundary_scan_awake,
	.heard = boundary_heard,
	.join = boundary_join_awake,
	.leave = boundary_leave_awake,
	.state = boundary_state,
	.signal = boundary_signal_awake,
	.channels = boundary_channels,
};

/*
 * The radio is the kernel's to ask from now on: an interface is brought up on it, and the
 * kernel is given the calls above. 0, or an errno.
 */
int
lx_station_offer(void)
{
	struct lx_radio *radio = the_radio;
	int error;

	if (radio == NULL || !radio->registered) {
		lx_tell("lx: no radio is registered: there is no station to offer\n");
		return (ENXIO);
	}
	lx_os_net_attach(radio->wiphy->perm_addr);
	if ((error = station_up(radio)) != 0) {
		return (error);
	}
	memcpy(boundary.address, radio->wiphy->perm_addr, ETH_ALEN);
	lx_os_station(&boundary);
	if (lx_os_radio_model() == 1) lx_os_runtime_state(4, 0);
	return (0);
}

/* ... and no longer: returns when no call of the kernel's is under way. */
void
lx_station_withdraw(void)
{
	lx_os_station(NULL);
}
