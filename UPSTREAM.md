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
