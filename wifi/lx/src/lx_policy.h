/*
 * What the Linux layer decides for the PCIe path (wifi_soc=8450), in plain C with no kernel in it, so that
 * tools/wifi-int-test/ runs these very functions on the host. Included by lx_bus.c and lx_base.c; the includer
 * brings uint8_t, uint32_t and bool.
 *
 * The radio's address. ath11k's own order (Linux 7.0.14, mac.c: ath11k_mac_register(), wmi.c: the READY event):
 *   1. the per-radio addresses the firmware reports when it says they are valid (ab->pdevs_macaddr_valid): ath11k
 *      then does not even ask for another;
 *   2. what device_get_mac_address() answers: the kernel's wifi_mac, if one was given;
 *   3. the address of WMI's READY event (ab->mac_addr).
 * ath11k does all of that itself. What is left for us is to answer device_get_mac_address() so that its own
 * order stays what it is, to know which of the three it ended with (the stage-9 line names the SOURCE, never
 * the address), and to give the radio an address when all three are zero: a locally administered one made of
 * the chip's identity, the same on every start of the same chip.
 */
#ifndef _LX_POLICY_H_
#define _LX_POLICY_H_

enum lx_mac_source {
	LX_MAC_FIRMWARE_RADIO,	/* the firmware's per-radio address */
	LX_MAC_WIFI_MAC,	/* the kernel's wifi_mac */
	LX_MAC_WMI_READY,	/* the address in WMI's READY event */
	LX_MAC_DERIVED		/* none of them had one: locally administered, from the chip's identity */
};

static inline const char *
lx_mac_source_name(enum lx_mac_source source)
{
	return (source == LX_MAC_FIRMWARE_RADIO ? "firmware per-radio address" : source == LX_MAC_WIFI_MAC ? "wifi_mac" :
	    source == LX_MAC_WMI_READY ? "wmi ready address" : "derived locally administered address");
}

struct lx_mac_inputs {
	bool radio_valid;		/* ab->pdevs_macaddr_valid */
	bool platform_valid;		/* the kernel has a usable wifi_mac (non-zero, unicast: it checked) */
	uint8_t platform[6];
	uint8_t ready[6];		/* ab->mac_addr, from WMI READY */
	uint32_t seed[4];		/* the chip's identity: MHI serial number, QMI chip id, soc id, board id */
};

static inline bool
lx_mac_zero(const uint8_t *a)
{
	return ((a[0] | a[1] | a[2] | a[3] | a[4] | a[5]) == 0);
}

/* 48 bits of FNV-1a (64) over a fixed label and the seed's words (little-endian), with the "locally administered" bit
 * set and the "group" bit clear: never zero, never multicast. */
static inline void
lx_mac_derive(const uint32_t *seed, uint8_t *out)
{
	static const char label[] = "hakios-wifi-mac-v1";
	uint64_t h = 0xcbf29ce484222325ull;

	for (unsigned i = 0; i < sizeof(label) - 1; i++) {
		h = (h ^ (uint8_t)label[i]) * 0x100000001b3ull;
	}
	for (unsigned i = 0; i < 4; i++) {
		for (unsigned b = 0; b < 4; b++) {
			h = (h ^ (uint8_t)(seed[i] >> (8 * b))) * 0x100000001b3ull;
		}
	}
	for (unsigned i = 0; i < 6; i++) {
		out[i] = (uint8_t)(h >> (8 * i));
	}
	out[0] = (uint8_t)((out[0] | 0x02) & 0xfe);
}

/*
 * The source ath11k ends up with, and what device_get_mac_address() is to answer to get it there:
 * *answer_valid false means "answer ENOENT" (ath11k then uses its own: the per-radio one or the READY one), true
 * means "answer *answer". Called when ath11k asks, which is after WMI's READY event.
 */
static inline enum lx_mac_source
lx_mac_choose(const struct lx_mac_inputs *in, uint8_t *answer, bool *answer_valid)
{
	*answer_valid = false;
	if (in->radio_valid) {
		return (LX_MAC_FIRMWARE_RADIO);
	}
	if (in->platform_valid && !lx_mac_zero(in->platform) && !(in->platform[0] & 1)) {
		for (unsigned i = 0; i < 6; i++) {
			answer[i] = in->platform[i];
		}
		*answer_valid = true;
		return (LX_MAC_WIFI_MAC);
	}
	if (!lx_mac_zero(in->ready)) {
		return (LX_MAC_WMI_READY);
	}
	lx_mac_derive(in->seed, answer);
	*answer_valid = true;
	return (LX_MAC_DERIVED);
}

/*
 * The board data file (board.bin of ath11k's request) when no wifi_board was given: cnss2's rule for this chip family
 * (QMI board id; 0xff: the ELF without a number; a chip id with bit 0x10 is the GF foundry's, and its files carry a "g").
 * [I] the GF bit is cnss2's CHIP_ID_GF_MASK as remembered, not seen in this tree: this phone's chip id is 0x12 and its
 * Android loads a bdwlang file, which agrees; and WP0 found the file Android actually used (bdwlang.b0e) is not what this
 * rule gives for board id 0xff, so round 1 names it with wifi_board. Returns the length, 0 if it does not fit.
 */
static inline unsigned
lx_board_auto_name(uint32_t board_id, uint32_t chip_id, char *out, unsigned size)
{
	static const char hex[] = "0123456789abcdef";
	bool gf = (chip_id & 0x10) != 0;
	unsigned n = 0;
	const char *prefix = gf ? "bdwlang." : "bdwlan.";
	char name[24];

	while (*prefix) {
		name[n++] = *prefix++;
	}
	if (board_id >= 0xff) {
		name[n++] = 'e'; name[n++] = 'l'; name[n++] = 'f';
	} else {
		name[n++] = 'e'; name[n++] = hex[(board_id >> 4) & 15]; name[n++] = hex[board_id & 15];
	}
	name[n] = 0;
	if (n + 1 > size) {
		return (0);
	}
	for (unsigned i = 0; i <= n; i++) {
		out[i] = name[i];
	}
	return (n);
}

#endif
