# Bluetooth WP1: shared radio power

2026-11-13. Branch `bt/wp1-radio`, based on `negroni-dev` at
`b4c39cc65b1297b317fda7be0ae0b19b56659858`. This package implements shared
SM8450 radio power. It does not enable Bluetooth or claim phone validation.

`hakios.radio.c` owns the retained six WLAN/four BT rail votes, enable GPIOs,
XO pulse serialization, ready owners, uncertain enables and per-rail vote
state. A failed XO cleanup is quarantined separately for the boot; an ordinary
enable-down cannot clear that uncertainty and permit a conflicting retry.
Normal transitions use one sleepable mutex; each RPMh transaction uses
the existing bounded shared RSC lock. An uncertain slot is not recycled;
the audited TCS2-to-TCS0 ON-only fallback remains. Rails are never voted off.
BT uses the conservative 500 ms WLAN-off discharge when temperature is unknown,
50 ms enable-low, a 200 us XO lead, 6 ms hold, 50 ms settle and SW_CTRL check.
Normal BT down waits 100 ms. Repeated requests are idempotent and failures
preserve the other owner; a combined request retains an earlier successful half.

PCIe now requests WLAN through the shared owner and retains its own PERST,
CLKREQ, clocks, PHY and register undo. A callback under the transition lock
keeps CLKREQ configuration between enable-low and rail votes, preserving the
cold write baseline. The radio source is a `standard` registration so a later
BT-only configuration can use it independently of CONFIG_HAKIOS_WIFI. It maps
and writes nothing until explicitly called; the Redwood BT driver is unchanged.

Panic is a separate terminal path: cached enable-output-low writes only, with
no radio locks, waits, XO changes or rail votes. Loader handoff first stops other
CPUs, confirms their terminal CPU_OFF state, masks local interrupts and invokes
the radio hook even when Wi-Fi was never selected. This confirmation is needed
because the existing CPU-stop routine may return after a timeout. If a CPU is
still executing, handoff is refused and the returning CPU parks; the loader
must not reuse memory/hardware while a radio writer can still run. CPU_OFF is
terminal even if firmware refuses the power-off call: that CPU stays in WFI.
The normal code on a stopped CPU never resumes, closing the late-enable race.
PCIe panic undo has its own entry and cannot enter the normal radio mutex.

The small map/config, PCIe header, Wi-Fi panic dispatch and loader/PSCI changes
are required integration seams for this package. The inherited Wave 0 radio
header was uncommitted in the source checkout and is included here; unrelated
source edits and the other Wave 0 files remain in that checkout.

## Checks

Run `python3 tools/radio-test/test.py` with Clang, pthreads and ASan/UBSan on
Linux. The tests compile the actual driver with mocked XNU/MMIO services.
The frozen `cold-reference.inc` is the prefix through `power()` of the
pre-WP1 `hakios.pcie.c` (Git blob
`78bcb12786a9a859a0836ac2ff3e8b51c13c630c`), not a rewritten sequence model.

- Cold WLAN: 128 identical MMIO-write/delay events before and after, retained
  in `tools/radio-test/cold-before.txt` and `cold-after.txt`. Trace SHA256:
  `64ead89f950676c818575b70ced94d0cdf180150647d82b0890c450f021f2884`.
- Both start orders, BT-only discharge, combined requests, repeated calls,
  retained votes, failure isolation in both directions and concurrent starts pass.
- All 128 combined-request write positions fail safely both before and after
  the simulated store (256 injections); memory and undefined-behavior checks pass.
- TCS fallback, two-slot timeout/quarantine, bounded busy RSC lock, SW_CTRL
  failure, panic with/without mappings and rejection after terminal handoff pass.
- Failed normal down and persistent XO-cleanup failure are covered. Assertions
  verify RSC writes hold the shared lock and discharge/settle delays do not.
- The actual CPU-stop confirmation function passes absent/running/stopped/
  timeout cases. This is not a live multicore panic test.
- Driver registration and Git whitespace checks pass.

## Kernel validation

Both complete kernel builds pass. Final kernel SHA256 values:

| Device | SHA256 |
|---|---|
| Negroni | `345b53c06eed8c9b1251d53e17817c1fd92d62ab4d64ace54929ff13a834a7aa` |
| Redwood | `2447c100a3093404cfbc304b7e32c280ad210f28cfce3f84a95aeac97f36933f` |

Build logs, configurations, linked-symbol lists, kernels, test log, reproduction
script and input/artifact hashes are saved in the workspace's ignored
`device-info/2026-11-08-initial/bt/validation/wp1/`. Both kernels contain
`hakios_radio_up/down/panic_quiesce`, `hakios_pcie_up/panic_quiesce` and
`psci_smp_others_quiesced`. Isolated build directory:
`/home/teleg/bt-wp1-build`. The main Linux checkout and installed defaults are
not modified. Each configuration rebuilds its corresponding PCI/AHB ath11k
object with zero stubs, with WIFI and QMI enabled in both configurations.
The initial template had Negroni Wi-Fi disabled; final acceptance replaces its
Negroni config/SoC inputs with the current device repository and checks linked
radio, PCIe and CPU-stop symbols. Redwood uses the saved Linux device inputs
(its device repository is absent from the Windows checkout).
The repository's build judge permits only the known
kext symbol-set export-check failures; a compile or link failure is not waived.

No phone, firmware download to the controller, RAM boot or flash was performed.
Wi-Fi stages 1–10 and coexistence remain NOT RUN until the combined phone round.
R3's sandbox/runtime gaps remain open. Next package: WP2 UART and BT power entry.
