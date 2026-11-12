# PCI compatibility checks

Run `python3 tools/pci-test/check.py` with clang or a C compiler. The test includes
production `wifi/lx/src/lx_pci.c` and replaces Linux types and C4 hardware calls
with fixtures. It checks MSI32/single programming, enable-before-probe-return,
allocation/config/readback/thread-start failures, optional per-vector MSI masks
(32/64-bit capability layouts), descriptor/config consistency, failed-probe retry,
shared handler removal,
masking, deferred CE/DP work, guarded BAR faults and teardown. On Linux it runs
with address/undefined-behavior sanitizers. It does not emulate MHI or prove
LinuxKPI ABI compatibility: the pinned-source build supplies that separate check.

The PCI build is `tools/mk-ath11k.sh`; `ATH11K_PCI=0 tools/mk-ath11k.sh` selects
the AHB-only regression. `ATH11K_BUILD_DIR` can select an isolated source/object
cache and `ATH11K_LD` an existing Darwin linker. The kernel headers are selected
by `XNU_SRC` as before. Without WP4, the strict combined build intentionally
fails on its 14 real `mhi_*` imports, rather than emitting success stubs.
