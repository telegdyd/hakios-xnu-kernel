# Changes on negroni-dev

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
