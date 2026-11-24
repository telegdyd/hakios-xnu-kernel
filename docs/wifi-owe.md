# Enhanced Open / OWE (P2c)

Implemented 2026-11-13. Software validation only. No phone interaction, boot,
flash, installation, or firmware changes.

## Behavior

- RFC 8110 group 19 (P-256), compact 32-byte x-coordinate public keys in
  association extension element 32. Open-system authentication precedes the
  exchange. A fresh private scalar is generated for each join; association
  retransmissions reuse that exchange. Random scalars are rejection-sampled
  into 1..n-1, with bounded retries and fail-closed entropy errors.
- The AP must advertise the OWE AKM (00:0f:ac:18). Passwordless Apple80211
  requests select it; a Personal/password request cannot silently become OWE.
  The association RSN selects OWE with CCMP and negotiated BIP-CMAC-128 PMF.
  This implementation requires PMF capability and a CCMP group cipher.
- The association response must contain a valid OWE RSN and exactly one
  group-19 DH element, with the selected cipher/PMF policy preserved.
  Truncated, duplicate, missing, unsupported-group, out-of-field and off-curve
  inputs fail the join. Duplicate RSNs and malformed trailing TLVs also fail.
- Shared ECDH x is extracted with salt `C | A | LE16(19)`, then expanded with
  `OWE Key Generation` to a 32-byte PMK. PMKID is the first 16 bytes of
  SHA256(C | A). Private scalar, intermediate shared secret, PRK and temporary
  derived keys are wiped. Failure never publishes a partial PMK.
- The existing four-way/group-key handshake uses the SHA256 PTK KDF,
  AKM-defined descriptor version 0, HMAC-SHA256 truncated to 16 bytes for the
  MIC, AES key wrap, and the existing GTK/IGTK installation/replay checks.
  SAE retains its different AES-CMAC MIC; WPA2 retains HMAC-SHA1/version 2.
  Message 3 authenticates the AP's advertised RSN.
- ECDH runs on the station thread outside the RX lock. The PMK is published
  under that lock. An early EAPOL message is ignored until it is ready; the
  AP's retransmission can start the handshake within the existing timeout.
  Keys must install successfully before the station becomes authorized.
- Station state reports `HAKIOS_STA_OWE`. Apple80211 GET AUTH_TYPE reports
  passwordless upper authentication, matching the existing no-password join
  interface. A failed OWE join does not fall back to plaintext.

## Sources and validation

Protocol: [RFC 8110 sections 4.3-4.4](https://www.rfc-editor.org/rfc/rfc8110.html#section-4.3).
The OWE implementation is local code, reusing the P-256/SHA256/HKDF backend
introduced for SAE; its provenance and timing-audit limits remain documented
in [wifi-sae.md](wifi-sae.md). Hostap 2.11's `src/rsn_supp/wpa.c` OWE exchange
and `src/common/wpa_common.c` AKM-defined MIC were checked for interoperability
rules. No additional third-party implementation was copied.

Run on Linux with Clang, Python and `cryptography`; no test accesses hardware:

```
bash tools/owe-test/run.sh 1
bash tools/owe-test/run.sh 7
CC=clang bash tools/wpa-test/run.sh 1
CC=clang bash tools/wpa-test/run.sh 7
bash tools/sae-test/run.sh 1
bash tools/sae-test/run.sh 7
bash tools/apple80211-test/run.sh
```

OWE tests compile the production association-response validator, RSN parser,
RSN serializer and ECDH/HKDF implementation. Each seed checks 20 independent
OpenSSL-backed Python ECDH/PMK/PMKID vectors and 340 association/entropy cases,
including failure cleanup, one-shot completion, malformed and duplicate
parameters, downgrade attempts, PMF/cipher changes and RSN truncations.
The WPA oracle adds six byte-for-byte OWE handshakes and group rekeys,
including early-M1 gating, tampering, wrong keys and replay regressions.
Both seeds (1 and 7) pass: OWE 340 association/entropy cases and 20 ECDH vectors
per seed; WPA 6336 checks per seed; SAE 1096 checks per seed and its production
MLME/RX harness; Apple80211 134 checks. The RSN/key-lifecycle and PMF station
harnesses also pass. Both PCI and AHB driver objects compile with zero stubs
(65/52 kernel imports). Negroni and redwood kernel executables both rebuild
in isolated copies of the cached XNU trees. Generated Makefiles/dependencies/
filelists are relocated to those copies; redwood's older cache also receives
the existing shared ITS driver and guarded 64-bit MMIO prerequisite from the
current port. No placeholder implementations are used. All host suites use ASan/UBSan. Evidence and isolated build scripts are in the
outer workspace's `device-info/wifi-p2c/` (final host logs in `final/`).

## Remaining hardware gates and scope

No over-the-air interoperability result is claimed. A later authorized round
must check a pure Enhanced Open AP and its Settings presentation, association
retries/early-M1 timing, DHCP/HTTPS, GTK/IGTK rekeys, saved-profile auto-join,
off/on and redwood WPA2 regression. The Apple interface's actual passwordless
OWE request behavior and firmware handling remain unverified.

Only group 19, CCMP and BIP-CMAC-128 are implemented. There is no OWE PMKSA
cache (every reconnect makes a fresh ECDH exchange), no group 20/21 retry, and
no OWE Transition Mode vendor-element redirection from the legacy open BSS.
The OWE BSS itself can be selected directly. Plain open-network support is
unchanged. OWE provides encryption without authenticating the AP; applications
still need their usual end-to-end security.
