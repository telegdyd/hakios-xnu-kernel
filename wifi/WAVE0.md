# Wi-Fi wave 0: frozen interfaces

Completed 2026-11-12 on `negroni-dev`. Scope: the C1–C8 seams of the workspace's
`WIFI-PLAN.md`, before wave 1. No phone access, firmware extraction, Wi-Fi enablement,
boot, flashing, or work-package implementation.

## Files and owners

| Contract | Source | Implementation owner |
|---|---|---|
| C1 stages | `xnu/files/bsd/sys/hakios.wifi.h`; `drivers/wifi/hakios.wifi.stage.c` | Main session, implemented |
| C2 PCIe | `xnu/files/bsd/sys/hakios.pcie.h` | WP1 |
| C3 SMMU | `xnu/files/bsd/sys/hakios.smmu.h` | WP2 |
| C4 kernel boundary | `xnu/files/bsd/sys/hakios.lx.h`; `wifi/lx/include/lx_os.h` | WP6; `lx_os_stage` already implemented |
| C5 Linux MHI | `wifi/lx/include/linux/mhi.h` | WP4 bodies in `lx_mhi.c`; public declarations frozen |
| C6 QRTR link | `xnu/files/bsd/sys/hakios.qrtr.h` | WP5; WP4 uses it through C4 |
| C7 words / C8 firmware | Workspace `WIFI-PLAN.md`, section 6 | WP6 and WP7 use the listed values/names |

`xnu/apply.sh` copies the `xnu/files` headers into the kernel's `bsd/sys`. The stage
source is registered in both `drivers/map` and `drivers/wifi/conf`, so it builds only
with the Wi-Fi driver. The existing redwood path has no new stage calls or hardware
behavior. New hardware functions are declarations only; no stub reports success.

## Boundary rules

- C2/C3/C4/C6 use positive errno; Linux C5 uses negative errno. WP3/WP4 convert at
  the boundary. `hakios_smmu_claim` instead returns a context or NULL.
- C2 owns RC0 hardware and borrowed BAR mappings. C3 owns a separate stage-1
  context, with 4-KiB pages, bounded TLB sync, and no CBAR/CBA2R restoration.
  `mask` is SMR ignored SID bits, not a DMA mask.
- C4 returns one pool, not an allocator: lx suballocates coherent buffers from it.
  Use the shared `HAKIOS_WIFI_IOVA_BASE`/`HAKIOS_WIFI_POOL_SIZE` constants from C3.
  ath11k's MHI `iova_stop` is inclusive and initially covers the whole DMA mask;
  WP4 must constrain allocations to this actual pool.
- C6 borrows link storage through synchronous link-down. TX copies asynchronous
  packets before returning; RX borrows bytes only during the call. Packets include
  the QRTR v1 header and padding. The QMI-facing `hakios_qrtr_ops` stays unchanged.
  C4 exposes `lx_os_qrtr_link(NULL)` for detach and `lx_os_qrtr_receive` for RX:
  WP4 does not import arbitrary kernel `hakios_qrtr_*` symbols, which the standalone
  object build does not permit.
- Normal teardown stops DMA before pool/context release. Panic teardown uses
  lock-free C2 hardware shutdown; it never acquires the shared SMMU mutex.
- Stage 12 is host-observed DHCP/ping/HTTPS; stage 13 is owner-observed Settings.
  Audio checks keep their existing pass lines outside the 13 Wi-Fi numbers.

## C5 inventory from the pinned Linux sources

Source: Linux **v7.0.14**, matching `tools/mk-ath11k.sh`.
Base: <https://github.com/gregkh/linux/tree/v7.0.14>.

`drivers/net/wireless/ath/ath11k/mhi.c` calls:

```
mhi_alloc_controller             mhi_free_controller
mhi_register_controller          mhi_unregister_controller
mhi_prepare_for_power_up         mhi_sync_power_up
mhi_power_down                   mhi_power_down_keep_dev
mhi_unprepare_after_power_down   mhi_pm_suspend
mhi_pm_resume_force              mhi_download_rddm_image
```

`ath11k/pci.c` also calls `mhi_device_get_sync` and `mhi_device_put`, and its
crash-dump paths read `fbc_image`, `rddm_image`, `image_info` and `mhi_buf`. These
types/fields are included even though RDDM returns `-EOPNOTSUPP` initially.

The IPCR client modeled on `net/qrtr/mhi.c` needs:

```
mhi_prepare_for_transfer         mhi_unprepare_from_transfer
mhi_queue_buf                    mhi_queue_skb
mhi_get_free_desc_count          mhi_get_mhi_state
__mhi_driver_register            mhi_driver_unregister
```

`mhi_async_power_up` is also declared for WP4's asynchronous power-up entry point.
There are **23 function declarations** in C5; registration macros map onto those
functions. Channel/event/controller configuration fields and client structures
retain upstream types; all enum values retain upstream encodings. The controller
exposes only caller-used public fields plus `private_data` for WP4's host state.
No Linux host lock/workqueue internals or host implementation is imported.

The declaration subset derived from `include/linux/mhi.h` retains its GPL-2.0
notice and upstream attribution. This is already a license category listed for
`wifi/lx/include/linux/` in `THIRD_PARTY_LICENSES.md`.

## Validation

Run from `hakios` with Python 3 and clang:

```
python3 tools/wifi-contract-test/check.py
```

Optional pinned-source validation takes a directory containing `mhi.h` from
`include/linux/`, `ath11k-mhi.c` from ath11k's `mhi.c`, and ath11k's `pci.c`:

```
python3 tools/wifi-contract-test/check.py --upstream SOURCE_DIRECTORY
```

Both modes passed in Debian WSL. Checks cover C17/C++17 header inclusion, the
single-source `lx_os.h` import, ARM64 iOS header syntax and a compiled ARM64 stage
logger object. The real logger is exercised for all 13 stage lines, failure,
embedded CR/LF, bounded detail truncation, and identical printf/loader sinks.
The upstream mode also compares public types/function declarations and compiles
ath11k's actual channel/event configurations and every controller field accessed
by its PCI/MHI callers. Linux foundational types and libkern functions are test
doubles; this is not a full LinuxKPI or kernel build.

`git diff --check` and driver registration checks passed. Full negroni/redwood
kernel builds are not claimed for wave 0; the new hardware code is not present
and Wi-Fi remains disabled on negroni. Each wave-1 package still owes the builds
specified in its brief.

## Pinned source SHA-256

Downloaded reference copies are local build scratch, not repository content.

| Upstream path | SHA-256 |
|---|---|
| `include/linux/mhi.h` | `b29d7537950bb8ed06860ecc4ccf10ce16a5a795ce7a3169e3b38094e6ac2b31` |
| `drivers/net/wireless/ath/ath11k/mhi.c` | `f717e0ab26a299621777c2b1caec8279b60a9d507d10e1bd315787593e3ed119` |
| `drivers/net/wireless/ath/ath11k/pci.c` | `4c6d9f6551f1f542bf241d590dd9d3dedf2731f89024cc082477da464d74aba2` |
| `net/qrtr/mhi.c` | `f4a2a42f954058f21075b977003dc1c42fb9ff21aae0251cf268501940aaac66` |
| `drivers/net/wireless/ath/ath11k/hw.h` | `30e228fe6268d0ee1d921b62f6051874e43313deece4855f40d35ea3db7e10d4` |
| `drivers/net/wireless/ath/ath11k/fw.h` | `09a79f490cf83be9c2f31414b9e9e306752d797e3200ef451493b45423202c64` |
