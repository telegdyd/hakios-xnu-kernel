# WPA3-Personal / SAE (P2b)

Implemented 2026-11-13. Software validation only; no phone interaction, boot,
installation or firmware changes.

## Implemented

- Group 19 (P-256), SHA-256, SAE commit/confirm over algorithm-3 authentication
  frames. Both hunting-and-pecking and hash-to-element (H2E) are supported.
  H2E is selected when the AP advertises RSNXE SAE-H2E. A 6 GHz SAE BSS without
  it is refused; this does not implement the remaining 6 GHz PHY/channel work.
- Legacy anti-clogging tokens and H2E token-container extension 93; retries
  preserve the original commit and random scalar. All authentication exchange
  work has a bound (eight transmissions, one-second response waits, at most
  three token requests). Duplicate commits resend a confirm. A first peer
  confirm counter of zero, as sent by hostap, is accepted; later replays are not.
- Peer scalar range, on-curve/finite point, reflection and infinite shared-key
  checks. Confirm comparisons are constant-time. A forged confirm never
  publishes the PMK. Invalid lengths, unsupported groups and extensions fail
  closed. The receive thread only copies bounded authentication messages into
  a mailbox; the station thread performs the expensive crypto without the
  global OS lock.
- Transition BSSs offering both PSK and SAE select SAE. SAE requires CCMP and
  PMF; failures cannot retry PSK. An explicitly WPA3-only request also rejects
  PSK-only BSSs. WPA2-PSK/CCMP and TKIP-group operation remain supported.
- SAE's four-way and group-key handshakes use the SHA-256 PTK KDF and descriptor
  version 0 / AES-CMAC MIC (AKM-defined), with the existing AES key wrap and
  PMF key handling. This is not descriptor version 3. The association and
  message 2 include the selected RSN and H2E RSNXE; message 3 must authenticate
  the AP's advertised RSN and RSNXE.
- A bounded, one-entry PMKSA cache binds own MAC, BSSID, SSID, a credential tag,
  SAE/H2E mode and a one-hour monotonic lifetime. It is populated only after
  SAE confirmation, the four-way handshake and successful driver key install.
  A cache hit uses open-system authentication and includes the cached PMKID in
  association. Rejection/timeout invalidates it and makes one fresh SAE attempt,
  pinned to the BSSID and SAE policy. Password changes cannot reuse an old key.
  Teardown wipes ephemeral SAE state; radio disposal also wipes the cache.
- Apple80211 passes a 1–63-byte SAE password, preserves a WPA3-only request
  (upper-auth bit 12), refuses a WPA2-derived PMK as an SAE credential, and
  reports the negotiated SAE AKM through GET AUTH_TYPE. A transition request
  remains automatic and prefers SAE according to the AP's RSN. WPA2's minimum
  password length is enforced after selecting the AP.

## Crypto and provenance

`lx_sae.c` specializes the algorithms in hostap 2.11 `src/common/sae.c` to
group 19. `lx_sae_crypto.c` adapts its BSD-licensed SHA-256 implementation;
HMAC, HKDF and the IEEE KDF have small local interfaces. Full attribution and
license: `wifi/lx/src/sae-crypto/HOSTAP-LICENSE`.

`lx_sae_math.c` is a local, fixed-size 256-bit field/scalar and Jacobian backend.
It uses fixed loop counts and masked selection for reduction, multiplication,
exponentiation and scalar multiplication, including exceptional point cases.
Inversion and square roots use fixed public exponents. Hunting-and-pecking
always performs 40 full candidate tests and selects the first valid result
without an early exit; exhaustion fails closed (probability below 2^-40).
There is no general-purpose big-number allocator or external ECC dependency.
These source-level properties have not had a machine-code timing/power audit.

Pinned source: [hostapd 2.11 release](https://w1.fi/releases/hostapd-2.11.tar.gz).
SHA-256 (original release files):

```
2b3facb632fd4f65e32f4bf82a76b4b72c501f995a4f62e330219fe7aed1747a  hostapd-2.11.tar.gz
9207d4be78db235d99c37ddcd10e7754a069354b50cdcc43ea4281f546ed5c8d  src/common/sae.c
a08d395f20f2b6efa0548e27d1396ed15ab8b313fc63611c418c8497713f58ec  src/crypto/sha256-internal.c
12841a2f0144206f8ff5e1c785bc33063ea610caa8f1854c88a0b9cb251f3099  src/common/common_module_tests.c
```

The test fixtures retain hostap's BSD notice. The SAE known-answer values are
IEEE 802.11-2020 Annex J.10; the H2E point is hostap's published group-19 vector
(SSID/password/password identifier combined as the identical HKDF input in the
test only). The independent ECDH point comes from
[RFC 5903 §8.1](https://www.rfc-editor.org/rfc/rfc5903.html#section-8.1).
Apple auth values were checked against the
[Apple80211 ABI header](https://github.com/OpenIntelWireless/itlwm/blob/master/include/Airport/apple80211_var.h).

## Validation

Run on Linux with Clang and Python; the WPA suite additionally needs Python
`cryptography`. No test connects to a radio.

```
bash tools/sae-test/run.sh 1
bash tools/sae-test/run.sh 7
CC=clang bash tools/wpa-test/run.sh 1
CC=clang bash tools/wpa-test/run.sh 7
bash tools/apple80211-test/run.sh
```

- SAE: RFC 5903 ECDH and IEEE commit/KCK/PMK/PMKID known answers, hostap H2E
  point, independent Python affine/integer H&P/H2E exchanges, 160 randomized
  field/scalar arithmetic cases, invalid scalar/point/length/reflection cases,
  token layouts, confirm replay/forgery, entropy failure and cache binding/expiry.
- Production MLME/RX against a simulated AP: successful H&P/H2E, token retry,
  duplicate commit, first-zero confirm, wrong algorithm/destination, AP refusal,
  timeout and forged-confirm rejection; real RSN/PMKID/RSNXE serialization.
- WPA: 18 independent byte-for-byte handshakes, including six SAE cases (three
  H2E), group rekeys, CMAC, PMF, tampering, replay and all earlier WPA2 tests.
- Apple80211: WPA3-only and transition requests, short SAE passwords, raw-PMK
  refusal and negotiated GET AUTH_TYPE, plus the existing request/event tests.

All host suites run under ASan/UBSan. Local results and isolated build scripts
are in the outer workspace's `device-info/wifi-p2b/`: SAE 1096 checks per seed
(1 and 7), WPA 6228 checks per seed (1 and 7), both production station harnesses,
and Apple80211 132 checks, all with zero failures. Both PCI and AHB driver
objects build with zero stubs (65/52 imports). Negroni and redwood kernel
executables rebuild in separate copies of their cached source/build trees,
including the changed station-state ABI and its kernel consumers. The kernel
`build` target is used: the broader `all` target also checks unrelated kext
export lists and fails on missing existing IOKit/Libkern exports.

## Remaining hardware gates and limits

No over-the-air WPA3 result is claimed. Check pure SAE and transition APs,
H&P/H2E, wrong password, AP token challenges, PMKSA reconnect and AP forgetting
its cache, group/management-key rotation, Settings save/auto-join, DHCP/HTTPS,
Wi-Fi off/on and redwood WPA2 regression on a later authorized phone round.
In particular, verify that wifid supplies the password for saved SAE profiles;
a WPA2-derived PMK is insufficient. Firmware management-frame handling is
also unverified. The full cache-rejection/rejoin path needs that round.

Only group 19, ordinary SAE AKM 8 and BIP-CMAC-128 are supported. Other groups,
SAE-PK, FT-SAE, SAE extended-key AKMs and password-identifier/rejected-group
extensions are refused. There is no downgrade to hunting-and-pecking after
an H2E failure, and no silent fallback to PSK. Timing/performance on the
phone and the independent crypto/security review remain open.
