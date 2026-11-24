# Bluetooth Wave 0 contract

2026-11-13. Interfaces only; Bluetooth is still disabled and unported. Canonical
headers are `xnu/files/bsd/sys/hakios.radio.h` and `hakios.bt.h`, copied into XNU
by the normal files overlay. No stubs, driver registration or hardware writes.
The next package is WP1, shared radio power, on its own branch/worktree from
`negroni-dev`. WP1-7 retain the plan's two-kernel build and package checks.

## Ownership and port boundary

WP1 implements radio power; WP2 implements UART, clock/BCM setup, stage logging
and chip power entry points; WP3 implements the full HCI run; WP4 implements H4
and traces; WP5 implements the shim. WP6 owns the one provisioned address in
`/chosen` and registry; WP3 uses those same bytes for EDL and readback, never a
new identity on reconnect. See the plan's R6 byte-order/provisioning contract.

Signatures follow iOSPlus10 0330-0338, including writable shim TX (0337) and
`h4_send_cmd(..., from_rx)` (0338). The one deliberate signature change beyond
renaming is `hci_run(void)`: `bt=1 bt_soc=8450` runs the complete pipeline,
replacing reference levels 2/3/4. Keep reference step messages inside the fixed
stage numbers. Redwood's read-only driver remains separate; WP2 must dispatch
SM8450 without letting the redwood path access its pins. No default enable
changes before WP8/installed validation.

UART ownership transfers from bring-up to H4 only after HCI success. The H4
power worker stops RX before taking the UART for off/on/full HCI reinit.
Never wait on RX from inside the RX callback. Define lock order in WP4 and
exercise simultaneous TX/RX/client-close paths; header declarations do not
establish concurrency safety by themselves.

WP1 retains R4's cold WLAN write baseline, delay rules, shared retained rails,
bounded RSC transaction lock and uncertain-vote quarantine. Normal transitions
are serialized separately from the RSC transaction lock. Panic quiesce is a
terminal handoff, and must prevent an in-flight normal enable write afterward.
WP2 provides the QUP2 BCM seam from the audited static helper as necessary;
it must use the same RSC transaction lock, never a second independent lock.

## Boot words

These are parser contracts for the owning packages, not implemented options
yet. Numeric helper takes a suffix (`"baud"` -> `bt_baud`); strings use bounded
`PE_parse_boot_argn` handling. Explicit words override installed defaults.

| Word | Bring-up default | Meaning / validation | Owner |
|---|---:|---|---|
| `bt` | 0 | 0 disabled, 1 full pipeline | WP2/WP8 |
| `bt_soc` | no implicit SM8450 selection | 8450 selects this port; preserve redwood dispatch | WP2 |
| `bt_delay` | 3 | seconds before initial hardware setup; reference clamps to 120 | WP2 |
| `bt_baud` | 3000000 | supported QCA + SE clock rate; 115200 skips switch, reject unsupported rates | WP3 |
| `bt_noflow` | 0 | ignore CTS for diagnosis, boolean | WP2 |
| `bt_rfropen` | 0 | vendor RX/RFR configuration, boolean | WP2 |
| `bt_to` | 2000 | HCI wait ms; reference bounds 50..30000 | WP3 |
| `bt_nvm` | board/SoC selection | filename only; reject empty override, slash, backslash, `..`, truncation; must exist in image | WP3 |
| `bt_shim` | 1 | 0 raw pass, 1 known Apple/Broadcom handling, 2 all VSCs; reject outside 0..2 | WP5 |
| `bt_lext` | 1 | legacy LE translation, boolean | WP5 |
| `bt_vscdata` | 8 | VSC return data bytes; outside 2..32 falls back to 8, as reference | WP5 |
| `bt_mfr` | 0 | 0 preserves Qualcomm; otherwise 16-bit manufacturer override | WP5 |
| `bt_dead` | 1 | 0 EIO, 1 hardware-failure answers, 2 refuse client; clamp max 2 | WP4 |
| `bt_pwrfollow` | 1 | power follows last client, boolean | WP4 |
| `bt_off_ms` | 3000 | last-close delay; clamp 100..600000 ms | WP4 |
| `bt_wait_ms` | 30000 | client ready timeout; clamp max 120000 ms | WP4 |
| `bt_h4log` | 300 | first N packets/direction; redaction still mandatory | WP4 |

Keep these reference diagnostics while porting: `bt_shimfix=1` (0336/0337
fixes, WP5), `bt_bcm=1` and `bt_bcm_kbps=5000` (WP2). `bt_bcm=0` is diagnostic
only and cannot satisfy UART acceptance. `bt_ibs=0` (WP3) keeps initial NVM
sleep modifications off; setting NVM bits alone cannot pass P3 power checks.
Do not port `negronibt_aon_ms` or `negronibt_off`: discharge timing is now
owned by the shared radio's R4 rules, and failed bring-up always cleans up.

## Stage and evidence contract

`HAKIOS_BT_STAGES(X)` is the canonical number/name table for WP2-7. Lines are
`bt: STAGE <n> <name>: ok <detail>` or `bt: STAGE <n> <name>: FAILED <reason>`,
through both console and loader log. Keep reference substeps under these
numbers. Failure stops dependent stages, releases only the failing half and
continues boot to iOS; do not invent successful later lines.

| # | Name | Required evidence / owner |
|---|---|---|
| 1 | power | BT rails/enable sequence, SW_CTRL 1 / WP1-2 |
| 2 | uart | QUP2 vote, branches, protocol 2, 115200 RTS/CTS / WP2 |
| 3 | rom | reset + product/SoC/ROM response / WP3 |
| 4 | baud | selected baud + version response / WP3 |
| 5 | patch | rampatch segments + ack count / WP3 |
| 6 | nvm | board response + selected NVM / WP3 |
| 7 | fw | reset + build label / WP3 |
| 8 | addr | selected source + matching HCI readback, no address value / WP3/6 |
| 9 | h4 | control registered + bluetoothd connected / WP4/6 |
| 10 | init | actual command/event trace, STACK_READY, zero restarts for 60 s / WP4/5/7 |
| 11 | lext | translated legacy reports delivered / WP5 |
| 12 | settings | owner sees nearby devices / WP7 + owner |
| 13 | switch | owner off/on, BT_EN low then full ready reinit / WP4/7 + owner |
| 14 | pairing | BLE + classic input, keychain storage, reboot/saved-key reconnect / WP6/7 + owner |
| 15 | coexistence | Wi-Fi stages 1-10 and scan comparison with BT on / WP1/7 |

WP7 also records `mg_test bluetooth bluetooth-le` before discovery/pairing;
both must be true. Each R3 follow-up is PASS/FAIL/NOT RUN with evidence. WP6
must resolve applicable sandbox policy before integration, preserve daemon
identity/access groups/securityd, and provide writable SCPreferences/configd
persistence. Saved switch states across reboot and Airplane Mode are installed
D1 checks. Nothing in Wave 0 closes these runtime or policy gaps.

WP4's trace ring is 512 packets, timestamp/direction/type plus at most 32 bytes;
redact before insertion as well as before text logging. Treat address-bearing
commands/events, NVM segments, ACL pairing/key material and unknown payloads
conservatively: metadata-only when a safe field layout is not established.
WP7's btsnoop export must preserve redaction and actual captured lengths,
never imply a full packet or reconstruct secrets. Keep SCO framing and
separate IBS TX/RX/clock state seams. Inspect hardware-error/vendor-diagnostic
events before shim vendor discard; bounded recovery is P4, not implemented here.

## Wave 0 validation (2026-11-13)

Clang `--target=aarch64-none-elf -ffreestanding -Wall -Wextra -Werror`
compiled a translation unit including both headers twice in C and C++, for
both `ARM64_BOARD_CONFIG_NEGRONI` and `ARM64_BOARD_CONFIG_REDWOOD`: all four
passed. The unit expanded the stage table (15 entries), checked the radio,
HCI, writable shim-TX and `from_rx` function pointer signatures, and called
the formatted stage declaration. `nm -u` on the C++ object confirmed C linkage
for all five referenced functions. Cross-checked defaults against 0330/0331,
0335 and 0337; `bt_to` minimum is 50 ms and invalid `bt_vscdata` falls back
to 8. Git whitespace validation passed.

These are header compile checks, not full kernel builds or a link of the
unimplemented functions. No kernel/userland image was built or enabled, and
no phone was accessed. WP1 onward must still build both kernels and run their
package-specific checks. Unrelated local edits were preserved.
