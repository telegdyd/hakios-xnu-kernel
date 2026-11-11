# Changes on negroni-dev

- A730 GPU power rollout measurements and validation (RAM only): independently
  exercise all eight runtime switch combinations, retained-buffer wake latency,
  shared-fence waiters, Metal and interface rendering, and isolated timeout,
  resume-failure and missing-notification paths. Controlled screen-on idle:
  3.002 -> 2.735 W reported USB input (8.9% lower); unplugged off/on/on/off
  comparison: 342.4 -> 312.7 mA discharge (8.7% lower). SpringBoard and vmd were
  paused during controlled idle and resumed afterward; no battery-life claim.
  Deeper-slumber first-submission latency is about 29 ms. Evidence, conditions,
  remaining thermal/visual/long-run gates: `docs/negroni-gpu-rollout.md`.
  Driver/ABI and installed defaults unchanged; no partition flashed.

- A730 deeper idle slumber (experimental `gpu_slumber=1`, default off): a
  one-second completed-work idle interval uses explicit runtime suspend/resume
  under the existing GPU mutex. Preserve client allocations, mappings, completed
  fences and numbering; restore GMU firmware/HFI, SMMU streams, empty ring/CP,
  interrupts and policy before publishing work. Share hardware shutdown primitives
  with final stop while retaining secure firmware during runtime sleep. Diagnostics
  avoid powered-off register blocks; mapping changes defer hardware invalidation.
  A failed transition latches submissions off, without retry or hang recovery.
  Independent of DCVS/IFPC, with root `kern.hakios.gpu_slumber` control and additive
  power telemetry. Audit and RAM evidence: `docs/negroni-gpu-slumber.md`.
  Three final RAM configurations pass 75,000 short exact submissions, retained
  buffers across 32 additional sleeps, repeated Metal suites and 78 suspend/resume
  pairs; zero transition/HFI failures, timing loss or IRQ fallbacks. Independent
  slumber-only and IFPC+slumber boots work without DCVS. Installed iOS restored
  and verified after each.
  No installed defaults changed and no partitions flashed; no power-savings claim.


- A730 completion interrupts (enabled by negroni's device configuration): `gpu_irq=1` attaches GIC INTID 332
  (the phone's DT SPI 300, level high) and enables CACHE_CLEAN_TS only. The final BR
  EVENT_WRITE7 requests an IRQ after the existing BV fence wait; BV never requests one.
  The handler checks GMU AHB/GX reachability, acknowledges completion, and wakes waiters
  without taking the GPU mutex. Unsafe, unexpected or storm interrupts mask the line and
  retain polling. `gpu_irq_wait=1` registers a scheduler wait before rechecking the fence,
  drops the GPU mutex while sleeping, and keeps a 10 ms timed fallback bounded by the
  original deadline. The memory fence remains authoritative. Normal shutdown masks and
  detaches/drains before GX teardown; panic quiesce only masks, without waiting for CPUs.
  Read-only `kern.hakios.gpu_irq_*` counters expose delivery and wait behaviour. Negroni's
  defaults set both switches; A6xx and boots without these arguments retain polling.
  Kernel build and boot-argument checks pass;
  OnePlus 10 Pro RAM boots: count-only and sleeping waits each pass 9,000 exact submissions
  with repeated client open/close. Sleeping run: 9,024 completion IRQs, 319 sleeps,
  294 wakeups and one completion found on a timed fallback during the measured interval;
  no unsafe/unexpected IRQs. Forty consecutive idle flip-buffer pixel checks pass 256/256
  within 4. Host requests were launched concurrently, but overlapping GPU client execution
  was not established; this is not a dedicated multiple-waiters-on-one-fence test. No
  motion sign-off, hang/storm injection or power-collapse validation is claimed.
  Installed on the OnePlus 10 Pro (11 November): the already RAM-tested kernel, menu image
  `3f3a30f8…`, boot_a only, preflight and exact readback passed. Ordinary installed startup:
  IRQ state 1, 795 completions, 655 sleeps / 647 wakeups, 0 fallback completions,
  0 unsafe/unexpected IRQs; DPU flip active, native UFS errors 0. Device defaults enable
  `gpu_irq=1 gpu_irq_wait=1`; manifest pins the device configuration commit.

- A730 Metal activation: device-aware C API compile entry points and CoreAnimation plug-in callers
  target HakiosGPU's chip; specialization retains the base program's chip. The A635 legacy API
  default is preserved. Admit FD730 with GMEM still off. Expose the RAM image's Metal bundle on a
  read-only flash root with a synthetic mount point and bindfs. The sweep accepts `--chip`, and
  ca-metallib accepts an already extracted AIR library as well as a universal one.
  The A730 draw shadow is disabled: 200 joined draws corrupt pixels with register deduplication,
  but all 4096 pixels are exact without it. Also preserve HLSQ_INVALIDATE_CMD at the selected
  chip's offset, since repeated values execute work. Regression fails on the old stream (two
  invalidates instead of four). Host: 282 / 283 tests; phone with Metal CoreAnimation active:
  168 draw checks, textures and AIR compute pass. The owner confirms the checkerboard corruption
  is gone; first-use transient flicker is being tested separately from framebuffer fetch.
  C API buffers retain their logical length with 64-byte-aligned backing for ir3's padded
  constant pushes (a 16-byte float4 otherwise refused a 64-byte push). Both chip paths are
  tested through the C API; low-level short-buffer bounds checks remain intact.
  CoreAnimation's colour-filter fragment functions (`_I…N<n>O…`) no longer take a stand-in that
  reads the filter rows in another layout (another op list, or half4 versus float4 rows): with no
  variant of the same inputs and precision, the function's own module is compiled. The wrong
  stand-ins added a flat garbage colour to the snapshot on cold app launches (boots 12-20).

- `fb_copy_guard=1` (off by default): while the framebuffer driver copies a surface to the panel
  with the CPU, HakiosGPU holds back submissions that name it (500 ms at most, then EBUSY), and
  `settle()` waits for the GPU before its redraw. Cures the A730's black bars; counted in
  `kern.hakios.fb_copy_*`. Remove it with the CPU copy (GPU-PLAN phase 5).

- `xnu-deps/host/bin/xnu-clang`: preserve Clang 14's warning behavior for
  implicit declarations and incompatible pointer conversions on Clang 16+.
  Adapted from ios-vayu commit `852fd53ed2a03a6ae56a323bdd835d1fdeb7c5ae`.
  The current upstream wrapper already drops the obsolete auto-init gate, and
  its host compatibility header no longer redefines glibc's string functions;
  the older `bb653c1` and `bab9b97` fixes therefore need no further port here.
  The Mu-specific `2a197eb` change belongs to the later UEFI build work.

- `xnu/hakios.patch`: remove unnecessary escapes around the syscall cast in
  `bsd/kern/makesyscalls.sh`. Debian 13's mawk preserves those escapes in the
  generated C file, causing every syscall initializer to fail compilation.
- `xnu-deps/setup.sh`: provide a local `python` alias for Python 3 on Linux,
  as XNU's debugger-file packaging uses the unversioned executable name.

- `tools/boot_config.py`, `mkxnuimg.py`, `xnuloader/build.sh`, `xl`: read
  non-executable per-device boot settings. Negroni uses its own FD, packaging
  recipe and payload address; the original defaults remain for redwood/QEMU.
  `xl enter` refuses a negroni loader whose generated build settings differ.
- `xnuloader/loader.c`, `mkpayload.py`: explicit named platform selection,
  measured SM8450 GIC/CPU geometry, configurable payload and memory limits,
  a read-only build option, and an optional bounded loader session. Negroni
  disables persistent logs and storage writes and initially limits sessions
  to 30 seconds. Device-owned UEFI probe, hook and guarded copy shim live
  in the private device repository.
- `xnu/hakios.patch`: reserve the worst-case physical-map block-alignment
  padding in both the aperture bound and L1 sizing for non-PPL large-memory
  kernels. Adapted from slop `negroni-ios/xnu/patches/0011` at
  `aeb9d51b527955f204c2fc0f208feaee149f57e3`. The rebuilt kernel reaches launchd
  under QEMU; hardware bring-up is tracked separately in the device facts.
- `xnuloader/loader.c`: per-platform `usb_restart_after_run` (negroni only)
  restarts the USB controller after a kernel has had the core that serves it;
  negroni's controller does not answer the host again otherwise after a run of
  a minute with userland. The console echo of a returned kernel log is limited
  to its last 3000 bytes; the whole log is still fetched over USB.
- `xnu/files/osfmk/arm64/gic_v3_irq.c`: redistributor frames are found with
  the right stride: the loader's `redistributor-stride` in `/arm-io/gic` if
  given, else four 64 KB pages when `GICR_TYPER.VLPIS` is set, else two; a
  core's own frame (by Aff1, then Aff0) is tried before the walk. SM8450's
  frames are 0x40000 apart while VLPIS reads 0 under Gunyah; with 0x20000 the
  second core read a hole (external abort). `xnuloader/loader.c` passes the
  stride for negroni. QEMU reaches launchd with 1 and 4 cores.
- `drivers/power/hakios.cpufreq.c`: `cpufreq_epss=<pa>` for a phone other
  than redwood (the RPMh memory vote stays redwood-only by default), the
  frequency table and LMh limit in the log and in `kern.hakios.cpufreq`, and
  `hakios_cpufreq_cap()` for the thermal driver; with `cpufreq_epss=` a
  load-based governor (`cpufreq_gov`, `cpufreq_gov_ms`, sysctl
  `kern.hakios.cpufreq_gov`), and the rows put back before the loader runs.
  `drivers/power/hakios.thermal.c`:
  `thermal_soc=8450` (SM8450 sensor table from the stock tree) and a stepped
  CPU cap from `thermal_slow=` degrees (default 85). Redwood's behaviour is
  unchanged without the new arguments.
- `drivers/input/hakios.buttons.c`: `buttons_soc=8450` selects SM8450's PMIC
  arbiter (version 7: core 0xc400000, cnfg 0xc42d000, observer 0xc440000;
  channel table at core + 0x2000, observer channels 0x20 apart, bus 0's count
  only) and the 10 Pro's key wiring (RESIN is volume up, PM8350 GPIO6 volume
  down, as the stock tree's key codes say). Run on the phone: all three keys.
  Redwood's layout and mapping are unchanged without the argument.
  `configs/common_defconfig` regenerated (it lacked earlier negroni symbols).
- `xnu/files/osfmk/arm64/loader_return.c`: the return reason (3, "reset the
  phone") is kept where the boot processor finds it. With more than one core,
  a request handled on another core was handed to the boot processor, which
  returned with reason 0, so `sweet-sh.py reboot` left the phone in the loader.
- `tools/hakios-config.py args --set name=value`: one argument changed for a
  run, replaced where it stands (XNU and the loader read the first occurrence),
  `name=` leaves it out; the name is checked against `configs/boot-args.lst`.
  Negroni's payload command line now comes from its configuration
  (`devices/negroni/tools/make-payload.sh`) instead of being typed per build.
- `xnuloader/loader.c`: platform flag `firmware_quiet_at_once` (negroni): when
  the kernel runs on another core (`bootcpu=N`, needed for `ufs=2`), core 0's
  interrupts are masked from the kernel's start instead of after NOTE_SECONDS.
  Mu's NegroniTouchDxe and NegroniKeyInputDxe otherwise poll the touch chip
  and keys under the kernel (I2C bus errors, no touch). Redwood unchanged.
- `drivers/input/hakios.s3908.c`: a stray continuation packet is read past
  instead of failing the command, and the report config is asked for up to
  five times. Seen on the first boot with the root on flash.
- `drivers/input/hakios.s3908.c`: boot-arg `touch_bus=` (0 default: the wires
  by hand as before; 1: a read-only log of the bus hardware; 2: the bytes
  through QUP0 SE4 and the QUP's GPI DMA, falling back to the wires on any
  failure). Needs: GPII 0 (the one bound to SE 4), wired-IRQ mode (INTSET=1,
  MSI base 0), an SMMU context of ours for both the GPI's stream 0x5b6 (rings)
  and the QUP wrapper's 0x5a3 (the bytes), RPMh BCM votes QUP0/SN3/SN0, pins
  16/17 to function 1. Measured on negroni (same kernel, RAM boots): a touch
  report 947 us against 11,120 us by hand, 0 errors; chip setup 3.6 ms
  against 54 ms. Lines also go to the loader's log (live USB log).
- `xnuloader/loader.c`: a run with `usb_net=1` on another core (`bootcpu=N`)
  leaves the USB controller to the kernel: `run_on_other_cpu()` no longer
  polls it, and the loader restarts USB after the run (returned, hung or not
  started). Before, both drove DWC3 and the live log and sweetd were unusable
  on flash boots. Runs without usb_net unchanged.
- `tools/boot_config.py`, `xnuloader/loader.c`: boot setting `DARWIN_WRITE`
  (default: the opposite of `READ_ONLY`, so existing ports are unchanged). A
  loader built `READ_ONLY=1 DARWIN_WRITE=1` writes no log to logdump but still
  serves the kernel's `ufs_write=1` writes to darwin past the cryptexes, so
  `/private/var` can be kept on a phone whose loader is otherwise read only.
- `tools/mkdatavol.py`: `--blocks N` for a darwin of another size than the
  first phone's (the volume must end where darwin does), and `--runs DIR` for
  the written stretches as raw files, for writing darwin from Android.
- Native UFS read/write mode (`ufs_native=1 ufs_native_write=1 ufs_write=1`):
  WRITE(10) is bounded to the loader-verified LUN-0 darwin data region, and
  SYNCHRONIZE CACHE(10) is ordered with writes. Raw writes flush/read back;
  HFS journal flushes go directly to the controller. Errors never replay
  through the loader in this mode. It is opt-in for other device configs.
- `xnuloader/loader.c`: immutable, versioned GPT-header snapshots occupy a
  compatible extension of the native storage offer. Startup compares native
  reads with these snapshots instead of asking a resident loader core. XNU
  can boot on core 0 (no bootcpu argument), bringing all eight negroni cores
  online. The loader remains in memory for return, but runs on no core while
  XNU owns the device. See `docs/negroni-native-ufs.md` for validation.
- Battery without the audio DSP (`drivers/power/hakios.battery_bcl.c`, `battery_soc=8450`):
  the OnePlus 10 Pro's percentage estimated from the PM8350B BCL's voltage and
  current (observer reads only), kept in `/private/var/db/hakios_battery`.
  `struct hakios_battery_state` gains `estimated` (CapacityEstimated, no made-up
  Temperature); `hakios_battery_get()` lets battguard read the driver directly
  instead of through `kernel_sysctlbyname()`, which read -1 there.
- `xnuloader/loader.c`: boot-arg `loader_stay=1` for a one-shot boot (a payload from
  the boot partition, no host): a kernel that returns is followed by the loader's USB
  commands instead of a reset, so the host can fetch the log (`fastboot get_staged`).
- `tools/mesa/freedreno-darwin.patch` (`fd6_draw.cc`): on the A7xx, every draw whose
  fragment shader reads the framebuffer (EXT_shader_framebuffer_fetch) gets a CCU colour
  clean, a UCHE invalidate and a CP_WAIT_FOR_IDLE before it. The A6xx rule (only the first
  such draw after one that does not read, no wait) left CoreAnimation's blur and vibrancy on
  the a730 as half-drawn quads, black or in stale colours, with GMEM and sysmem, with UBWC and
  without. FD_MESA_DEBUG=flush and glfetch=0 both hid it; this does, at full speed
  (OnePlus 10 Pro, 10 November). The A6xx path is unchanged.
- `hakios-metal`: A730 host preparation (10 November): shared chip facts and generated
  register-offset/magic-state tables; GPU-targeted command streams; gen7 batch start,
  sysmem CCU split, compute state, explicit-value completion events and 2D copies/clears.
  The fake validates those paths, including negative stream tests, with separate A730
  shader fixtures and per-chip ir3 compilers/caches. The fixture compiler lowers uclz
  as the driver's compiler already does. The A730 Metal plug-in gate remains closed;
  its draws/GMEM wait for their later ladder steps. See the parent workspace's
  GPU-PLAN.md, section 7, for checks and the next phone-probe milestone.
- `hakios-metal/tools/probe.cpp`: isolated `--case` hardware ladder with per-chip
  A635/A730 fixture selection; refuses the historical A635-only broad suite on
  other chips. Each case checks results/guards and faults across three queue
  reopenings, stopping at its first failure. Probe build dependencies include
  the A730 fixtures. The float-register helper uses a constexpr bit-cast for
  the sign (including negative zero), which works in the iOS Clang C++17 build.
- A730 compute follows `fd6_emit_shader(A7XX)` for instruction fetch (no A6xx
  instruction-preload packet), and resets `SP_CS_VGPR_CONFIG` through the shared
  generated table. The fake checks both requirements. Fresh-phone assembly and
  AIR buffer compute pass; typed texture stores still leave memory untouched
  and can affect subsequent compute until restart. This is not Metal UI enablement.
- A730 compute IBOs are bindless set 0, as Gallium (`ir3_nir_lower_io_to_bindless`) and turnip
  have them on every gen 6+. The non-bindless IBO path, which no Mesa driver runs there, is
  unreliable on the A730: after one non-bindless typed store, or any bindless dispatch, every
  later non-bindless store wrote nothing until a reboot (OnePlus 10 Pro, 10 November). The
  same instructions with only the bindless bit set worked in that state. `compiler.cpp`'s
  `lowerIboToBindless` and the fixture tool's `lower_ibo_to_bindless` keep the IBO slots (SSBO i
  at i, image i at num_ssbos + i). `tools/ir3-kernel` adds `.base0` to gen 7 assembly's IBO
  instructions. `compute.cpp` points SP_CS_BINDLESS_BASE[0] at the dispatch's table and
  prefetches with SS6_BINDLESS. The fake refuses the non-bindless form on the A730. A6xx
  unchanged. Fixes the "typed texture stores" item above: `tex_fill` and `tex_copy` pass.
- `hakios-metal/tools/probe.cpp`: the suite's texture section (7a-7g and 9mc: half stores, 2D
  arrays and 3D, views of a level and a layer, nearest/linear sampling, six integer formats,
  the address modes and border colours) is now `textureSuite()`, every kernel taken by the chip,
  and runs on its own as `--case textures`; 7d (compiling them on the phone) stays A635 only.
  `--metallib` is `metallibSuite()`, compiling for the chip it runs on, and is the one part of
  the broad suite an A730 runs. On the OnePlus 10 Pro (10 November) all 47 texture checks pass
  twice in one boot, and eight PortraitFilters kernels compile on the phone and complete, the
  same words in two runs. No behaviour change for the A635.
- `hakios-metal/src/render.cpp`, `gen7.cpp`: the A730's first sysmem draw (colour targets, no depth,
  no GMEM), its batch start and pass begin taken from freedreno's own FD730 stream. Every stream of a
  pass (`CommandBuffer::draw`'s head and body, `closePass`'s tiles and tail, OpenPass's) is now built
  with the command buffer's chip: they took `pm4::Stream`'s default A6XX id, so the A730 got the A6XX
  path and hung. `a730_probe_submissions_as_rd` checks the whole submission (and with
  `HAKIOS_A730_SUBMIT_RD` writes it as `.rd` for cffdump). On the OnePlus 10 Pro (10 November)
  `tri_flat` passes all three shapes exactly, three rounds each; compute and textures still pass
  after it. Depth, GMEM, indirect draws and 3D IBOs stay refused on the A730; the A6xx path is unchanged.
- `hakios-metal`: the A730's draw ladder (section 6 step 4). The A730 fake GPU validates draws
  (`gen7DrawReady`: the gen7 batch, freedreno's fixed FD730 pass state, no A6XX-only register, no
  preload, BIN_CONTROL 0, the prefetch's and the depth planes' A7XX forms) instead of refusing them.
  `render.cpp`: the FS texture prefetch in its A7XX form (the command's own field positions, the
  slots in `SP_FS_BINDLESS_PREFETCH_CMD`); depth and stencil attachments (`RB_DEPTH_BUFFER_INFO` /
  `RB_STENCIL_INFO` TILEMODE TILE6_3); indirect draws; occlusion queries (each ZPASS_DONE followed by
  CCU_CLEAN_DEPTH, turnip's A7XX gen1 path); the vertex and fragment stages' IBOs as bindless set 0
  (`SP_BINDLESS_BASE[0]`, GFX_BINDLESS invalidate, SB6_IBO from set 0; `compiler.cpp` and the
  fixture tool lower every gen-7 stage's IBOs, not only compute's). The probe's section 9 is
  `drawSuite()`, every kernel by the chip, `--case draws [--sections 9a,9b,...]`. On the OnePlus 10
  Pro (10 November): 167 checks of section 9 pass, 0 fail; GMEM stays refused on the A730 (its GMEM
  lines are printed, not judged). The A6xx path is unchanged.
- GPU power preparation (11 November): read-only `kern.hakios.gpu_power` snapshot
  under HakiosGPU's existing sleeping lock, shared with submit and orderly stop;
  runtime HFI asserts the same lock. Lifetime submission timing is separate from
  worked()'s interval counters, with overwritten timing slots counted; it is not
  hardware busy utilization. Record accepted vote indices, requested clock, HFI
  acknowledgement failures/latency and final-stop state/failures. Correct SM8450
  GPU temperatures to TSENS0 sensors 14/15 and route minute logging through the
  same lock. No frequency/governor/collapse change. Parent GPU-POWER-PLAN.md records
  the staged plan and validation status. Kernel build and OnePlus 10 Pro RAM
  test pass: 3,000 exact submissions, six snapshots, both GPU sensors valid,
  no HFI/unsafe-IRQ/unexpected-IRQ/UFS errors; installed iOS restored. Hardware
  busy utilization remains explicitly unavailable pending its DCVS-stage audit.
- A730 DCVS (11 November): opt-in `gpu_power_control=1`, manual runtime
  levels 220/317/492 MHz and captured-DT bandwidth indices 2/2/6; root sysctls
  select manual/auto, an upper level, and validated busy-counter use. A 50 ms
  worker shares the submission/stop mutex; burst/backlog boosts precede ring
  publication, low-load downshifts use 200 ms hysteresis, and GPU TSENS sensors
  impose 75/85 C ceilings with 70/80 C recovery (missing sensors: lowest level).
  GMU XO busy deltas follow upstream Linux's counter selection, with bounded
  coherent reads and invalid-delta rejection; submission elapsed time remains
  separate. Firmware failure latches control/submissions off, no recovery
  claimed. Normal stop cancels/drains the worker without holding its mutex;
  panic quiesce inhibits subsequent control without taking a sleeping lock.
  Manual votes were RAM-verified before adding auto. Host policy scenarios and
  the kernel build pass; parent GPU-POWER-PLAN.md records final RAM evidence and
  limitations. No frequency-range expansion, IFPC/slumber, installed-default
  changes or flashing.

- A730 IFPC (11 November): independent opt-in `gpu_ifpc=1`, separate CP restore
  storage at GPU VA 0x01300000, audited gen7_0_0 clock gating and firmware
  feature handshake before CORE_FW_START. Release the boot OOB GPU hold once
  runtime setup is ready. Shadow RPTR accounting and bounded fenced WPTR
  publication permit sleeping GX; shared IRQ-safe keepalive ownership protects
  register access without taking the sleeping GPU mutex in IRQ. Power and hang
  diagnostics avoid GX; final shutdown wakes before GX accesses and panic
  quiesce avoids the sleeping GX bus side. Root `kern.hakios.gpu_ifpc` uses a
  verified OOB power hold to inhibit collapse: this firmware acknowledges a
  FEATURE_CTRL disable without actually stopping it. RAM validation observes
  749 collapses, GX power-off/state-3 snapshots, stable counters while inhibited,
  36,000 short exact submissions, sustained CP load, 47 texture and 172 draw
  checks, and auto DCVS. Installed iOS restored, no flashing/default changes.
  See `docs/negroni-gpu-ifpc.md` for references, evidence and limitations;
  deeper idle slumber remains separate.
