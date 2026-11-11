# A730 GPU power: Step 5 validation evidence

2026-11-11. RAM tests on the OnePlus 10 Pro. HakiosGPU and its ABI are unchanged.
No partition was flashed, no installed default changed, and each completed run restored and verified the installed iOS baseline.

Final restoration audit: all 11 RAM boot runs, including two preserved host/helper attempts, restored installed iOS. After the last restoration, 3,000 more exact submissions and all four Metal suites passed on the installed kernel; IRQ and DPU flip stayed enabled, unsafe/unexpected IRQs and native UFS errors were zero, and the experimental power sysctl was absent.

## Power comparison

The controlled screen-on idle comparison recorded **3.002 W inhibited versus 2.735 W enabled**, an observed **0.268 W (8.9%) lower USB input** with DCVS, IFPC and slumber together. This is a telemetry result under the conditions below, not a battery-life or isolated GPU-rail power claim.

| Workload | All inhibited | All enabled | Observed input difference | Readings per mode |
|---|---:|---:|---:|---:|
| Controlled idle | 3.002 W | 2.735 W | -0.268 W (-8.9%) | 16 |
| Idle, normal background daemons | 3.295 W | 3.161 W | -0.134 W (-4.1%) | 8 |
| Sustained CP writes | 3.970 W | 3.761 W | -0.209 W (-5.3%) | 4 |

### Unplugged battery comparison

The owner physically unplugged USB for a second controlled test. A self-contained phone-side recorder ran four 75 s off/on/on/off blocks (five minutes total), with 15 s settling and six gauge readings per block. SpringBoard and vmd were paused automatically and resumed at completion. All 24 readings reported USB charger absent, a DSP gauge source and no estimated SOC; diagnostics confirmed active state for inhibited blocks and actual deeper sleep for enabled blocks.

| Block | Controls | Battery current mean | Reported voltage mean | GPU sensor 0 mean | CPU busy, all cores |
|---|---|---:|---:|---:|---:|
| 1 | Inhibited | -344.3 mA | 3958.8 mV | 43.70 C | 0.48% |
| 2 | Enabled | -306.3 mA | 3955.8 mV | 41.07 C | 0.38% |
| 3 | Enabled | -319.0 mA | 3947.7 mV | 40.95 C | 0.58% |
| 4 | Inhibited | -340.5 mA | 3938.7 mV | 41.80 C | 0.43% |

Pooled battery current: **-342.4 mA inhibited, -312.7 mA enabled** (negative means discharging). The observed drain-magnitude reduction is **8.7%**. Each mode has 12 readings. These are steady screen-on idle samples with those background processes paused; no battery-life extrapolation or pack-power normalization is made. Battery temperature was 35–36 C. GPU temperature was lower during enabled blocks, but the initial cool-down and short duration prevent a steady-state temperature-savings claim.


The initial forward idle pass suggested a saving; the reverse pass did not reproduce its direction. Combined uncontrolled averages must not be treated as a repeatable saving. Sustained work has only four readings per mode and overlapping ranges; its difference is exploratory.

### Controlled idle method

Same RAM kernel and screen image, same USB supply/cable, same root filesystem and CPU/display configuration. SpringBoard was paused to remove animations. The Apple Visual Voicemail daemon (`com.apple.voicemail.vmd`, executable `/System/Library/PrivateFrameworks/VisualVoicemail.framework/vmd`) was also paused temporarily after it was observed using about one CPU core. It was resumed during cleanup; its CPU-loop cause was not diagnosed.

Off/on/on/off blocks: 20 s settling after each change, then eight readings roughly 10 s apart. Battery-manager polling is 10 s. Each block also verified 100 exact short submissions. The prepared boot supports all three features; runtime off uses the documented IFPC OOB inhibit, and the 50 ms worker remains present. This is not a comparison with those features absent from the boot arguments.

| Block | Controls | Mean USB input | GPU sensor 0 mean | CPU sample |
|---|---|---:|---:|---|
| 1 | Inhibited | 3.041 W | 38.02 C | 11% of one core |
| 2 | Enabled | 2.814 W | 36.00 C | 6% of one core |
| 3 | Enabled | 2.655 W | 35.54 C | 6% of one core |
| 4 | Inhibited | 2.963 W | 35.92 C | 12% of one core |

All 32 controlled readings reported 0 mA battery current. Battery temperature was 29–30 C. USB ranges overlap: inhibited 2.293–4.957 W; enabled 2.229–3.383 W. Temperature fell after pausing vmd, so the temperature difference cannot be attributed to GPU policy alone.

Power is `AdapterDetails.AdapterVoltage * AdapterDetails.Current / 1e6`, using millivolts and milliamps from the firmware-backed IOPMPowerSource. It includes the display, CPU, memory, conversion losses and other hardware. Battery current is recorded separately, positive charging and negative discharging. While connected, USB supplies some or all of the load, so battery current alone is not total consumption. No pack-power conversion was made from the reported roughly 4 V battery voltage, and no external-meter calibration was performed. Raw high readings were retained, not discarded as outliers.

### Independent runtime controls

Each of eight DCVS/IFPC/slumber combinations passed a short exact probe in forward and reverse order. The following are uncontrolled-background averages (eight input readings per combination). These data demonstrate switch coverage, not reliable per-feature power attribution.

| DCVS | IFPC | Slumber | USB mean | Range |
|---:|---:|---:|---:|---:|
| 0 | 0 | 0 | 3.295 W | 2.859–3.900 W |
| 0 | 0 | 1 | 3.365 W | 2.971–4.934 W |
| 0 | 1 | 0 | 3.222 W | 2.690–4.080 W |
| 0 | 1 | 1 | 3.198 W | 2.754–3.901 W |
| 1 | 0 | 0 | 3.384 W | 2.752–4.047 W |
| 1 | 0 | 1 | 3.153 W | 2.648–3.816 W |
| 1 | 1 | 0 | 3.340 W | 2.882–3.898 W |
| 1 | 1 | 1 | 3.161 W | 2.815–3.997 W |

### Sustained command-processor workload

Each submission writes 65,536 words and verifies the final word. Four 50 s phases use a fixed 492 MHz request with all inhibited, auto DCVS only, IFPC only, and all enabled. This is command-processor work, not a shader/rendering benchmark. Vmd was running in this pass. All samples remained near 98–99% busy and requested 492 MHz.

| Controls | USB mean | Battery mean | Correct submissions in 50 s |
|---|---:|---:|---:|
| 000 | 3.970 W | -12.2 mA | 1788 |
| 100 | 4.236 W | 0.0 mA | 1789 |
| 010 | 4.051 W | -1.8 mA | 1787 |
| 111 | 3.761 W | 0.0 mA | 1784 |

## Correctness, latency and interface checks

Installed baseline Metal end-only/AIR compute, texture and draw suites were checked before each normal RAM run. Normal RAM runs retain DPU flip and IRQ completion. No unexpected HFI/transition/unsafe-IRQ/unexpected-IRQ/UFS errors or timing loss were accepted.

Timed IRQ fallback completions remain visible: the final UI boot recorded 1 and the battery-only boot recorded 3 cumulatively. The four-mode correctness phase and the controlled/uncontrolled matrix and sustained-work final samples recorded none. All work completed; these counts neither prove physical IRQ loss nor identify a policy as their cause. The isolated missing-notification test is reported separately below.

Submission plus fence-wait latency after 1.5 s idle, using the same retained buffers (12 samples per mode). These values are not touch-to-photon latency:

| DCVS/IFPC/slumber | Median | Range |
|---|---:|---:|
| 000 | 0.074 ms | 0.041–0.116 ms |
| 010 | 0.382 ms | 0.337–0.563 ms |
| 001 | 28.897 ms | 27.304–29.693 ms |
| 111 | 29.060 ms | 28.238–29.782 ms |

The four-mode correctness run passed 12,000 short exact submissions, 48 retained-buffer idle submissions, 80 sustained submissions with four waiters each, and Metal end-only/AIR/47 texture/172 draw checks. All four waiter calls overlapped in 80/80 recorded submissions.

The correctness phase finished before that runner stopped at a refused private URL launch. Its original error is retained; `phase-verification.json` verifies the completed correctness phase independently. The signed private-URL helper also refused on installed iOS, so final app opens used the existing touch-replay tool.

A later full-text boot-argument verification replaced the transport’s truncated hexadecimal display. The full recording arguments fit by omitting the cosmetic version tag from the capture payload. The earlier failed verification attempt and its restored-system evidence remain preserved.

The final capture retained 2102 frames. Sampled home-page swipes, Control Center/blur, Settings and Calendar launch images showed no visible corruption. Calendar reached its first-launch onboarding sheet. The off-mode Settings tap occurred while Control Center was still open, so its Settings/scroll segment is excluded from comparisons. Settings rendering and scrolling were separately verified on installed iOS and on the all-enabled RAM boot before battery recording.

| Recorded action | Median gap, inhibited | Median gap, enabled |
|---|---:|---:|
| Home-page swipes | 16.69 ms | 16.78 ms |
| Control Center | 16.72 ms | 16.85 ms |
| Calendar launch | 32.89 ms | 33.13 ms |

These are recorded frame intervals over actions that include deliberate waits; they are not an end-to-end touch-to-photon or statistical no-regression claim. The native-loader log fetch did not retain the per-touch/per-frame event trace, so the full pipeline latency could not be calculated. Capture instrumentation was separate from every power run. Physical panel tearing and final owner visual sign-off remain outside this capture review.

## Isolated fault injection

Separate RAM-only test images were derived from the unchanged production driver. The test flag is explicitly armed through existing runtime writes after normal startup; it is absent from the normal kernel and is not a supported production interface. Build source and the original kernel were restored and verified after preparing the images.

| Injection | Result |
|---|---|
| Ignore matching runtime HFI acknowledgement after sending the request | Expected failure returned in 1.421 s; later work rejected; no repeated HFI/transition attempt; installed system restored |
| Fail GMU firmware restoration during an armed resume | Expected failure returned in 0.109 s; later work rejected; no repeated HFI/transition attempt; installed system restored |
| Acknowledge hardware completion IRQs but suppress completion wake notifications | Exact work completed, including retained buffers; timed fallback count increased by 80; installed system restored |

These injections validate selected software failure paths and missed-notification fallback. They do not establish physical hardware-fault coverage, interrupt-storm handling, context replay or hang recovery.

## Rollout boundary and reproducibility

Keep all three features independently opt-in. No installation or default-policy change is part of this validation. Hardware thermal threshold/sensor-loss transitions remain untested; host policy checks cover the thresholds, recovery, missing sensors and ceilings. External power-meter calibration, realistic repeated-use discharge measurements and direct visual sign-off remain necessary before a general battery-life or fully validated rollout claim. The 50 ms worker still wakes the CPU during slumber.

Boot-argument catalogue/regressions, driver inventory and DCVS/thermal host policy tests passed. The configuration regression skipped because the redwood reference checkout is absent. The original Task 4 GPU source/build hashes matched after fault-image preparation. The normal kernel hash is `68a572e7e57b2a7d962e14c6d767dba790a5a56e08ce3f23e1a402fa7cb7e278`; normal power-test payload is `b1235ca33165ca4316810bc20fc4d2af791fe77cf8a93d9fc346039796180993`.

Helpers are under `device-info/2026-11-08-initial`: `test-gpu-rollout.py`, `prepare/build-gpu-rollout-probes`, capture builder/analyzer, fault preparer/builder/runner, and `analyze-gpu-rollout.py`. Evidence: `gpu-rollout/*/results.json`, `commands.log`, `samples.csv`, `summary.json`, capture files and pinned payload hashes. No push or commit was requested.
