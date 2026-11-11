# Negroni A730 inter-frame power collapse

Implemented and RAM-tested 11 November 2026 on `negroni-dev`. Experimental and
opt-in; the installed kernel and its defaults remain unchanged. Keeps HakiosGPU
and the existing userspace ABI. Deeper idle slumber is covered separately in [the Step 4 notes](negroni-gpu-slumber.md).

## Controls

Boot with `gpu_ifpc=1` to prepare and enable IFPC on SM8450. Without it, the
feature handshake, restore allocation, clock-gating setup and IFPC counter
selection are absent. It does not require `gpu_power_control=1` or enable auto
DCVS. The final RAM run used both boot controls; DCVS auto started off.
The IFPC-only boot combination has not been separately RAM-tested.

`kern.hakios.gpu_ifpc` is a root-writable 0/1 control under the existing GPU
mutex. Writes without boot preparation or after a latched IFPC failure return
ENXIO; invalid values return EINVAL. An identical write does nothing.

The phone firmware acknowledges a runtime FEATURE_CTRL disable but continues
collapsing. Consequently this control keeps the boot feature configured:
writing 0 requests the GMU's OOB GPU power hold and waits for its acknowledgement;
writing 1 releases that hold. The off state was verified by a stationary hardware
collapse counter and powered-on GX, not by a message acknowledgement alone.

## Firmware and restore audit

Hardware facts and firmware layout were audited against Qualcomm's gen7_0_0
path, which selects `a730_sqe.fw` and `gmu_gen70000.bin`:

- [Feature sequence and IFPC request](https://android.googlesource.com/kernel/msm-modules/graphics/+/6daff35b39e089f6dafec2bc7c69d7f9592d1dd6/adreno_gen7_hfi.c).
- [Protocol IDs and feature payload](https://android.googlesource.com/kernel/msm-modules/graphics/+/6daff35b39e089f6dafec2bc7c69d7f9592d1dd6/adreno_hfi.h).
- [CP initialization, restore lists and clock-gating sequence](https://android.googlesource.com/kernel/msm-modules/graphics/+/6daff35b39e089f6dafec2bc7c69d7f9592d1dd6/adreno_gen7.c).
- [A730 clock-gating tables](https://android.googlesource.com/kernel/msm-modules/graphics/+/6daff35b39e089f6dafec2bc7c69d7f9592d1dd6/adreno-gpulist.h)
  and [shared list header](https://android.googlesource.com/kernel/msm-modules/graphics/+/6daff35b39e089f6dafec2bc7c69d7f9592d1dd6/kgsl_util.h).
- [Fenced ring publication and submission markers](https://android.googlesource.com/kernel/msm-modules/graphics/+/6daff35b39e089f6dafec2bc7c69d7f9592d1dd6/adreno_gen7_ringbuffer.c).

The audit also compared upstream Linux v6.12's `a6xx_gpu.c`, `a6xx_gmu.c` and
`a6xx_hfi.c`. That version deliberately leaves its A7xx CP restore list absent;
copying its initialization alone would not establish working IFPC here.
Reference copies, revision metadata and SHA256 manifest live in the parent
workspace's `device-info/2026-11-08-initial/gpu-ifpc/` directory.

The boot HFI request is message 11, feature 9, enable 1, hysteresis 0x1680,
sent after perf/BW tables and before CORE_FW_START. The boot OOB GPU hold and a
keepalive reference protect initialization. Both are released only once runtime
state, the CP restore buffer and IRQ setup are ready.

The driver programs 53 A730 GX clock-gating values and three AO values, and
captures 52 IFPC-only plus 17 shared register/value pairs. Restore storage is a
separate mapped 4 KiB block at GPU VA 0x01300000; scratch VA 0x01200000, including
submission timings at offset 0x400, remains separate. The 16-byte firmware header
contains three zero lock words and packed u8 IFPC/u8 shared/u16 dynamic counts.
The dynamic count is zero. CP_ME_INIT bit 8 supplies the list with its spinlock;
its final word has bit 31 set for the gen7 dynamic-triplet format. The captured
list is immutable while CP runs. This does not enable preemption.

## Access and failure boundaries

With IFPC prepared, ring-space accounting uses the BR RPTR shadow at scratch
+0x80; CP_RB_CNTL permits the hardware to write it. Runtime publication writes
only the fenced WPTR doorbell, checks WRITEDROPPED0 and retries for at most
10,000 one-microsecond delays. Exhaustion latches IFPC failure and rejects new
submissions and subsequent DCVS control. No hang recovery is claimed.

Read-only power diagnostics inspect shared memory and CX/GMU registers, never GX
and never issue firmware messages. Hang reporting suppresses GX reads on the
IFPC path, retaining shared fence/ring and GMU/SMMU diagnostics.

IRQ handling takes a short spin lock with local interrupts disabled only to
update the shared keepalive reference and register. It never takes the sleeping
GPU mutex, sleeps or sends HFI. Before GX access it waits at most 100 one-us
polls for an open AHB fence and GX power/clocks, retaining the existing GDSCR
check. Failure disables the interrupt and leaves bounded fence polling available.
The shared reference prevents an IRQ's release from clearing a thread's hold.

Orderly final stop drains fences, obtains a bounded GX-access reference, detaches
IRQ and performs the existing shutdown. A failed GX wake skips GX stop accesses;
the GMU/CX teardown remains bounded. The hold is released before PREPARE_SLUMBER.
Panic quiesce inhibits new control, masks IRQ, resets GMU and avoids the GX-side
bus-halt accesses that would be unsafe while collapsed. None of this is a
runtime suspend/resume operation.

## Additive `gpu_power` diagnostics

The version remains 1. New fields:

| Field | Meaning |
|---|---|
| `ifpc_config` | Boot preparation selected on SM8450 |
| `ifpc` | Runtime permission to collapse; 0 holds the OOB GPU power request |
| `ifpc_failed` | Latched IFPC publication/control/wake failure |
| `ifpc_count_valid`, `ifpc_count` | Bounded coherent read of the GMU hardware collapse counter, XOCLK4 countable 4; not an HFI acknowledgement count |
| `gx_status` | GMU GFX power/clock status; bit 0 power off, bit 1 clock off |
| `rpmh_state` | GMU power state; 3 is IFPC |
| `ifpc_wakes` | Successful explicit thread GX-access acquisitions, including already-on GX; currently used at final stop. Does **not** count ordinary submission wakes |
| `ifpc_wake_failures` | Failed bounded WPTR publication, explicit GX wake, or OOB power-hold acknowledgement |

Status fields are sampled separately and can straddle a transition. Actual wake
is established by successful subsequent GPU work after observed collapse.

## Validation

The final kernel and RAM payload built successfully. The build wrapper accepted
only its existing kext symbol-set failures, with no compile/link failure.

- Kernel SHA256: `0bdb1afbf8a2f5ca36c12beadc92d1296264855a0200f34f6bf4087238a9f791`.
- RAM payload SHA256: `6876af8a6f602f52acebe1aff1e26e9917b370f3f92cab154be86aa1927ca752`.
- Runtime sequence 1/0/1/0/1: 3,000 exact submissions per phase. Disabled sample
  intervals held the counter exactly at 288 and 333; all 28 disabled samples
  showed GX powered on. Enabled intervals resumed counter progress.
- Manual 220/317/492 MHz: another 3,000 exact submissions each. Four automatic
  phases: another 3,000 each. Total **36,000** short exact submissions.
- Metal `end_only` and `fill_id_air`: three rounds each; **47 texture checks**
  and **172 draw checks** pass, zero probe failures/faults.
- Sustained CP probe: 600 verified submissions, 65,536 CP writes each. Revalidated
  the busy counter at fixed 220 MHz before selecting it: idle mean **1.33%**,
  sustained load mean **99.25%**, all selected samples valid. Auto returned to
  220 MHz and bandwidth index 2 after idle.
- 115 diagnostic samples: first collapse count 130, final 749; **72** snapshots
  had GX power off and GMU state 3. Subsequent work and IRQ completions advanced.
  Lifetime submissions reached 38,598 including rendering and CoreAnimation;
  all timing records were collected, none lost.
- Zero HFI failures/no-acks, IFPC/DCVS failures, rejected busy deltas,
  unsafe/unexpected IRQs or native UFS errors. **29 timed IRQ fallback
  completions** remain visible; this does not prove lost interrupts. Stage 2
  also observed fallbacks during sustained probes. No comparative latency claim.
- Installed iOS restored and verified: IRQ state 1, DPU flip state 1, native UFS
  errors 0, new power sysctl absent as expected. No partitions flashed.
- Boot-argument catalogue/regression, driver inventory and existing DCVS policy
  scenarios pass. Configuration regression skips because the redwood reference
  checkout is absent. Tested build-source hashes match this working tree.

Attempt 1 passed 3,000 exact submissions but showed no collapse: the boot OOB
power hold had not been released. Attempt 2 established real collapse and work
progress but exposed the ineffective firmware disable acknowledgement; its
second 3,000-submission phase passed while collapse continued. Both attempts
restored installed iOS. They are retained as `attempt-{1,2}-*` evidence.

Helpers: parent `device-info/2026-11-08-initial/build-gpu-ifpc.sh` and
`test-gpu-ifpc.py`. Evidence: `gpu-ifpc/verified.json`, `test.log`, `build.log`,
`hashes.txt`, `source-hashes.json`, `reference-hashes.json`, `host-checks.txt`.

Remaining rollout work: visual scroll/Control Center/blur/app-open sign-off,
controlled battery and temperature comparisons, thermal hardware transitions,
injected firmware/wake/lost-IRQ failures and shared-fence multiwaiters. No
power-savings, hang-recovery, deeper-slumber or installation claim. IFPC remains
opt-in. Changes are uncommitted; no push was requested.
