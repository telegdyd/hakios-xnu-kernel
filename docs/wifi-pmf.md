# Protected management frames (P2a)

Implemented 2026-11-13; software validation only. No phone boot or installation.

The existing WPA2-PSK/CCMP path now negotiates PMF when the AP advertises MFPC,
including APs that require it (MFPR). The association RSN IE advertises MFPC,
zero PMKIDs and group management cipher BIP-CMAC-128; `sta->mfp` tells ath11k
to enable PMF in its peer association. Non-PMF CCMP/TKIP networks retain their
existing negotiation. Unsupported management ciphers and malformed RSN tails
are refused. SAE's SHA256 PTK and AKM-defined descriptor-version-0 handshake
are implemented separately in [wifi-sae.md](wifi-sae.md) (P2b).

## Key and frame handling

- An authenticated, AES-unwrapped message 3 must contain a GTK and, for PMF,
  a 16-byte IGTK KDE (00:0f:ac:9), key ID 4 or 5, and its six-byte little-endian
  IPN. All KDEs are checked before publishing any key. Message 3 must also authenticate
  the AP's advertised RSN IE, including its PMF capabilities. Truncated, duplicate or
  malformed KDEs leave the old keys and EAPOL replay counter unchanged.
- IGTK-only group rekeys are accepted without reinstalling the GTK. Both IGTK
  slots are retained during rotation. Receiving identical IGTK material never
  lowers its IPN, including when that material moves between slots. Identical
  same-slot GTK material is not reinstalled in firmware. Station teardown wipes
  the WPA state, including both IGTK slots.
- BIP-CMAC-128 is installed in the host's WPA state, **not in firmware**.
  The pinned Linux 7.0.14 ath11k `ath11k_mac_op_set_key()` explicitly returns 1
  for BIP ciphers (software crypto). The host validates the final 18-byte MMIE,
  key ID, strictly increasing IPN and 64-bit CMAC before removing the MMIE and
  dispatching the management frame. Only authenticated frames advance the IPN.
- Firmware authenticates/decrypts unicast CCMP management frames. The host checks
  their separate management PN and removes the retained CCMP header and MIC.
  A stripped IV is trusted only when the driver reports PN validation.
  Fragmented management frames are refused on this path.
- After the authenticated initial key message, unprotected robust management
  frames cannot disconnect the station. Sender, BSSID and destination are checked
  before cryptographic state changes. Verified group deauth/disassoc frames may
  reach the normal link-down path.
- The host answers authenticated unicast SA Query requests (category 8, action 0)
  with the same transaction ID and action 1. Replies and local deauthentication
  use a protected CCMP frame, a fresh management TX PN and the installed pairwise
  key. ath11k's WMI path encrypts it and supplies MIC space. Public/unprotected
  action categories and Ethernet traffic do not enter this path.

The plan's assumption that firmware answers SA Query was corrected: this is
host MLME work. This change implements the responder; initiating SA Query on
suspect deauth and association comeback timers remains follow-up MLME work.

## Validation

Run `CC=clang tools/wpa-test/run.sh [seed]` on Linux with Python's `cryptography`
package. The runner includes the production source in sanitizer-enabled tests:

- RFC 4493's four AES-CMAC vectors; independent Python CMAC/BIP and AES-wrapped
  WPA2 handshakes, including PMF initial keys and rotation.
- Malformed/truncated KDEs, missing initial IGTK, duplicate/invalid IDs,
  IGTK-only updates, IPN preservation and frame replay/tampering.
- Production RSN parser and GTK allocation/replacement/failure cleanup.
- Production management RX/TX with a recording driver: authenticated SA Query,
  protected local deauth, forged/replayed input, authenticated unicast and group
  deauth, legacy unprotected deauth, Ethernet separation and PN exhaustion.

Seeds 1 and 7: 6126 arithmetic/handshake/frame checks, zero failures, plus both
station harnesses under ASan/UBSan. The complete PCI and AHB arm64 driver objects
build with zero stubs (65 and 52 kernel imports). Both objects also link against
the cached negroni kernel objects. These are cached-object link checks, not fresh
full builds of negroni and redwood or hardware regression results.

Local build logs: `device-info/wifi-p2a/` in the outer phone workspace.

## Next phone round

Use WPA2-PSK/CCMP APs with PMF disabled, optional and required. Confirm association,
DHCP and HTTPS; capture the negotiated RSN IE and the `protected management frames`
log line. Rotate GTK/IGTK and test authenticated versus forged/replayed unicast
and broadcast deauth/disassoc. Send SA Query and verify the protected response
over the air. Recheck the owner's non-PMF mixed-TKIP network and redwood.

No PMF-required join, real firmware CCMP RX/TX layout, over-the-air SA Query or
redwood join is claimed until that round passes.

## Sources

RFC 4493 for CMAC; IEEE 802.11 RSN/IGTK/MMIE wire formats. Driver contracts were
checked in the pinned local ath11k sources and against upstream
[ath11k key handling](https://github.com/torvalds/linux/blob/master/drivers/net/wireless/ath/ath11k/mac.c),
[management RX](https://github.com/torvalds/linux/blob/master/drivers/net/wireless/ath/ath11k/wmi.c),
[CCMP receive handling](https://github.com/torvalds/linux/blob/master/net/mac80211/wpa.c),
and [SA Query response handling](https://github.com/torvalds/linux/blob/master/net/mac80211/rx.c).
No upstream implementation was copied into the production files.
