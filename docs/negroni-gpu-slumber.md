# Negroni A730 deeper idle slumber

Experimental Step 4 on `negroni-dev`, 11 November 2026. Keeps HakiosGPU and
its userspace ABI. Defaults remain off; use RAM payloads before any installation.

## Controls and lifetime

`gpu_slumber=1` prepares the SM8450-only idle worker independently of
`gpu_power_control` and `gpu_ifpc`. `kern.hakios.gpu_slumber` is a root-writable
0/1 control. Invalid writes return EINVAL; absent preparation, final stop,
panic inhibit or a latched transition failure returns ENXIO. Disabling while
asleep resumes hardware before returning. Read-only diagnostics never resume it.

The existing 50 ms thread-call worker and submissions share `gGPULock`. One
second with all fences complete and no new submissions permits suspend. Every
submission restarts that idle interval, including manual-DCVS mode. An unsafe
or storm-disabled IRQ prevents automatic suspend from rearming that IRQ.
Ongoing interface animation counts as work and prevents deeper slumber.

Logical service readiness persists during sleep. Client buffers, virtual addresses,
page-table allocations, fence numbering and completed fence words survive.
Mapping changes during sleep update retained tables; unmap skips powered-off
SMMU registers. Resume installs and invalidates the tables before CP starts.
The empty ring restarts at offset zero; completed fences and timing history do not.

## Suspend and resume

Runtime suspend drains the ring with a bounded wait and acquires GX access,
collects timing records, disables busy sampling, then masks/detaches and drains
completion IRQ handlers. The shared low-level hardware suspend operation releases
OOB/keepalive, waits for the configured IFPC idle state and GX-off, requests
PREPARE_SLUMBER, resets GMU, sleeps RSCC, disables translation streams and
collapses CX. Runtime retains the authenticated zap shader; final stop unloads it.
CX and GX power checks establish hardware collapse, rather than an HFI ACK alone.

On submission, resume restores CX/clocks, retained GMU mapping, firmware and
fresh HFI queues; replays performance/BW/IFPC boot handshakes; installs retained
GPU/LPAC mappings; resets the empty ring/shadows; rebuilds the CP restore list;
initializes CP and leaves secure mode; then reattaches interrupts and counters.
Thermal/manual/automatic policy and an IFPC inhibit survive. Work is published
only after this succeeds. No boot allocation routines or final-stop entry point
are used for runtime resume/suspend. The uncached HFI mapping is reused.

Final stop cancels/drains the worker through the existing wrapper. If hardware
is already asleep, it unloads secure firmware without waking GX/CX. Panic sets
the atomic inhibit and uses the existing nonblocking quiesce path.

## Failure boundaries

Polling remains bounded (including GX wake, ring drain, HFI, RSCC and CX).
One failed suspend or resume latches `slumber_failed`, increments transition
failures and rejects later submissions. Partial resume is torn down with IRQs
detached and translations removed. There is no automatic retry, context replay
or hang recovery. Existing platform secure calls are synchronous; no hard
wall-clock upper bound for the entire transition is claimed.

The worker continues its 50 ms checks while asleep without reading CX/GMU/GX.
This does not establish net platform power savings. Secure firmware retention,
CPU wake overhead and resumed frame latency require rollout measurements.

## Diagnostics

`kern.hakios.gpu_power` remains v=1 with additive fields:

| Field | Meaning |
|---|---|
| `state` | Existing 0..4; 5 suspending, 6 asleep, 7 resuming, 8 runtime failure |
| `slumber_config`, `slumber` | Boot preparation and current idle policy permission |
| `slumber_failed` | Latched transition failure; no retry |
| `slumber_suspends`, `slumber_resumes` | Successful runtime transitions |
| `slumber_suspend_us`, `slumber_resume_us` | Last transition elapsed host microseconds, not frame latency |
| `cx_off` | Driver's last observed CX power-off state |

Shared-memory fence/timing reads and TSENS stay available asleep. IFPC hardware
count/status are suppressed outside active state; `ifpc_count_valid=0` then.
The hardware IFPC counter resets across deeper collapse, so compare within a
single active epoch. Busy samples invalidate and rebase after resume. Requested
kHz/accepted vote fields retain requested values asleep, not measured clocks.
State transitions hold the GPU lock, so an ordinary reader normally sees 1, 6
or 8, not intermediate 5/7.

## Protocol references

The shared shutdown was audited against [Linux v6.12 a6xx_gmu.c](https://github.com/torvalds/linux/blob/v6.12/drivers/gpu/drm/msm/adreno/a6xx_gmu.c)
and the phone-generation [Qualcomm gen7 GMU sequence](https://android.googlesource.com/kernel/msm-modules/graphics/+/6daff35b39e089f6dafec2bc7c69d7f9592d1dd6/adreno_gen7_gmu.c)
with [HFI layout](https://android.googlesource.com/kernel/msm-modules/graphics/+/6daff35b39e089f6dafec2bc7c69d7f9592d1dd6/adreno_hfi.h).
Local revision-pinned copies and their manifest are in the preceding IFPC
stage's `gpu-ifpc/reference/` evidence directory.

PREPARE_SLUMBER uses `{header, bw=2, freq=1}` for the A730's lowest supported
pair. Gen7 requires the configured lowest idle state before this request. Two
preliminary attempts received firmware verdict 1002, including the nonzero
payload without the idle-state wait. Only adding the idle-state sequencing
established successful suspend/resume; the payload change alone was insufficient.
A third attempt completed two transitions, then its runner timed out waiting for
idle while interface animation continued. These are retained as incomplete runs;
installed iOS was restored after each.

## Validation

The full final RAM run passed and installed iOS was restored/verified (IRQ 1,
DPU flip 1, native UFS errors 0, experimental sysctl absent). No partitions flashed.

- Kernel SHA256: `68a572e7e57b2a7d962e14c6d767dba790a5a56e08ce3f23e1a402fa7cb7e278`.
- Main RAM payload SHA256: `b1235ca33165ca4316810bc20fc4d2af791fe77cf8a93d9fc346039796180993`.
- 12 controlled idle/wake cycles: 36,000 short exact submissions. Manual 220,
  317 and 492 MHz after sleep: 9,000 more. Auto mode with IFPC inhibited and
  permitted: 6,000 more. Zero wrong words or failed submissions.
- Same client/command/data buffers reused across 16 sleeps (8 submissions in
  each of two open/close rounds), exact words and strictly advancing fences.
- Metal end-only and AIR compute (three rounds each), 47 texture checks and
  172 draw checks all
  pass after observed deeper slumber, zero probe failures/faults.
- 42 suspend/resume pairs including early startup and resumed interface work;
  51,890 lifetime submissions/timing records in the final active sample.
  Zero lost timings, HFI failures/no-acks, DCVS/IFPC/slumber failures, rejected
  busy deltas, unsafe/unexpected IRQs, IRQ fallbacks or native UFS errors.
- Sampled transition times: suspend 1,217..1,567 us, resume 25,748..29,279 us.
  These are transition samples, not a frame-latency or power-savings measurement.
- Passive reads during observed CX-off neither resumed hardware nor sent HFI.
  Disable resumed once and held active beyond the idle interval. Manual levels,
  auto mode and the OOB IFPC inhibit survived subsequent resumes.
- Host boot catalogue/regressions, driver inventory and DCVS policy tests pass.
  Configuration regression skips for the absent redwood checkout. Source hashes
  match the built GPU sources; `gpu-slumber/host-checks.txt` records checks.

A fourth preliminary run passed repeated cycles/retained buffers but exposed
manual policy being reset during resume; fixed by separating counter restart
from one-time policy initialization. The complete final run above includes the
fix. Attempts 1..4 are retained with restoration evidence.

Independent slumber-only boot passed with both `gpu_ifpc` and
`gpu_power_control` absent: 12,000 short exact submissions, eight retained-buffer
sleeps, 47 texture checks and 172 draw checks. It completed 18 suspend/resume
pairs including startup and interface restart with zero recorded failures,
lost timings or IRQ fallbacks, and restored/verified installed iOS. Evidence:
`gpu-slumber-only/verified.json` and `test.log`; payload SHA256
`5a5a328acd144f6ee0826ce033d06912cb6ce1bd650e6fa4a4b4d37690fb44a5`.

The IFPC+slumber boot without `gpu_power_control` also passed: 12,000 short
exact submissions, eight retained-buffer sleeps, 47 texture checks and 172 draw
checks; 18 suspend/resume pairs including startup/interface restart. All 12,928
submissions have timing records, zero lost; no HFI/transition/IFPC errors,
unsafe/unexpected IRQs or IRQ fallbacks. Installed iOS restored and verified.
Evidence: `gpu-slumber-ifpc/verified.json` and `test.log`; payload SHA256
`c6f289466a11428fa81f5227fe437636a4072a26ae188558f8b350b1c476b3a5`.

Across the three final boots: **75,000 short exact submissions**, 32 additional
retained-buffer submissions, repeated Metal suites, and **78 suspend/resume
pairs**. All three boots passed and restored installed iOS. The IFPC-without-DCVS
run also closes the preceding IFPC stage's separate-worker validation gap.

Helpers in parent `device-info/2026-11-08-initial`:
`build-gpu-slumber.sh`, `test-gpu-slumber.py`, `gpu-slumber-retain.c` and
`gpu-slumber-quiet.c`, with build helpers. The retained-buffer probe pauses for
1.5 seconds before each submission on the same client and mappings. The quiet
helper temporarily stops and resumes SpringBoard only during the RAM test.
Raw results, build log and hashes: `gpu-slumber/`.

Remaining rollout work includes visual scroll/Control Center/blur/app-open
sign-off, matched battery/temperature measurements, frame-latency comparison,
hardware thermal transitions, injected wake/IRQ faults and shared-fence waiters.
No installation, power-savings or recovery claim is made.
