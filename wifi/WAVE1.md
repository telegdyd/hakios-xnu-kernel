# Wi-Fi Wave 1 dispatch: WP0–WP3

2026-11-12. Requested packages only: WP0 in the main agent, WP1/WP2/WP3
in three simultaneous agents (four active-agent limit). No phone connection,
RAM boot, install or flash. WP4/WP5/WP7 were not dispatched. Package branches
remain isolated; this is not the WP6 integration.

## Baseline and scope decisions

The existing Wave 0 changes in the original `negroni-dev` checkout were
uncommitted. They were preserved and copied as validation inputs, without
representing them as package-authored commits. In particular WP1's commit must
contain its PCIe conf line without the copied Wave 0 stage-logger line.

Main approved minimal extra WP3 ownership of `wifi/lx/src/lx_bus.c` and
`lx_base.c` for PCI lifecycle dispatch, coherent pool selection and guarded BAR
access; AHB behavior stays conditional on the original path. Main also approved
`tools/pci-test/` for actual-source fake-endpoint/MSI/poll validation.

`drivers/map` additions for `wifi bsd/dev/hakios.pcie.c` and
`core bsd/dev/hakios.smmu.c` remain integration work in the original checkout;
the isolated validation tree includes them. Do not register absent files in
the original checkout before importing their package implementations.

## Main-owned guarded MMIO prerequisite

WP1/WP2 found that the current checkout had no SEA-catching MMIO primitive.
Main added `xnu/files/bsd/sys/hakios.mmio.h` and a final `osfmk/arm64/sleh.c`
diff to `xnu/hakios.patch`. These remain separate from package commits.

The public functions are `hakios_mmio_try_read32(const volatile void *, unsigned *)`
and `hakios_mmio_try_write32(volatile void *, unsigned)`, returning positive
errno. Failed reads leave the output unchanged. NULL/misaligned accesses return
EINVAL; caught accesses return EIO. The raw ARM64 instruction/register shape and
Mach-O `alt_entry` annotations follow iOSPlus10 reference patch 0297.

The exception integration differs deliberately from that reference's test-only
expected-fault hook: a synchronous external abort is recovered only at the two
exact load/store instruction PCs, before interrupt-stack rejection. It does
not depend on FAR validity, CONFIG_XNUPOST, a mutable armed state, a lock or an
allocation. Asynchronous SError, translation, alignment, parity and unrelated
instruction faults retain the kernel's existing behavior. Already-mapped valid
device pointers are the caller's responsibility. Interrupt/panic behavior is a
code-review inference pending an actual hardware exception test.

Validation in ignored local `device-info/2026-11-08-initial/wifi/`:

- `check-mmio.py`: actual ARM64 Mach-O assembly compiles; host tests cover
  errno, output preservation, `0xffffffff` successful reads, exact-PC recovery,
  SEA versus translation/parity classification, and patch-output equivalence.
- WP2 independently reviewed the helper and exception integration; no actionable
  defect found. No hardware exception test was performed.
- The generated `sleh-mmio.c` used by both full validation builds has SHA256
  `39d99f5d7afc0fd1817b14566b74496f1dac21941c55ace1d13400f84e3debf3`.
- The existing `tools/wifi-contract-test/check.py` still passes.

## WP0 facts and firmware

`device-info/2026-11-08-initial/wifi/FACTS.md` is the local report. Extracted
198 modem firmware files plus the exact Android-used `bdwlang.b0e` from the
already-downloaded matching stock OTA's ODM: **199 files**, each in
`fw/SHA256SUMS`. The separate inventory preserves each file's source.
79 exact DT PHY triples, six resolved PMIC/cmd-db supply names, PCI windows,
clock/reset IDs, IOMMU and MSI maps are recorded with source hashes.

The actual Android log reports `soc_id=0x400c1211`, `board_id=0xff`, then uses
`/odm/etc/wifi/bdwlang.b0e` (59908 bytes), contrary to assuming the modem `.eNN`
or `.elf` rule is sufficient. The existing `wifi_board=bdwlang.b0e` override is
the explicit first-round choice; automatic selection still needs integration
work based on this evidence.

**MAC (resolved 2026-11-12):** no NV MAC in 587 saved persist entries, and none is
needed: Android uses the chip's default address, and ath11k takes the firmware's
(`mac.c` :10730-10742). `wifi_mac` is optional; with every source zero the driver
derives a stable locally administered address. No MAC was fabricated. Firmware and private evidence remain
ignored local data; WP0 has no tracked code changes to commit.

## Shared kernel validation for WP1 and WP2

WP1 created an isolated validation tree `/home/teleg/wifi-wp1-build`, without
mutating `/home/teleg/negroni`. It combines WP1/WP2 and the main MMIO baseline,
using the existing AHB ath11k object. Negroni temporarily enables
CONFIG_HAKIOS_WIFI and CONFIG_HAKIOS_QMI so C2 is actually compiled; installed
defaults are unchanged. The log includes PCIe, SMMU and exception objects;
linked symbols include all six `hakios_pcie_*`, both `hakios_mmio_try_*` and
`hakios_smmu_claim`. This does not validate WP3's PCI object or WP4/WP6.

| Kernel artifact | SHA256 |
|---|---|
| `logs/wp1-wp2-negroni-kernel` | `a56a133963e6a952b6cef64bfa59480b9aff5f40c7044c05a6e70a44dd23b175` |
| Redwood validation kernel | `3d6f697e16d84ee120a709f2120114457afb2e79b60ff8f4e5162e03cf583b5a` |

Negroni log: `/home/teleg/wifi-wp1-build/logs/wp1-wp2-negroni.log`.
Both builds were accepted by `make-kernel.sh` after the existing kext symbol-set
check exceptions; these are linked kernel results, not a claim that raw XNU make
returns zero. Logs/configs/input hashes were also copied to the ignored local
`device-info/2026-11-08-initial/wifi/validation/kernel-build/` directory.
Full package commit IDs and any remaining gates are maintained in the workspace's
`WIFI-PLAN.md` status table. A successful compile/link is not a phone pass.

## WP3 dependency boundary

The pinned upstream PCI/PCIC/MHI files compile and the combined relocatable
`all.o` links, but the strict final build refuses 14 actual MHI API functions
owned by undispatched WP4. They are not success-returning stubs. The AHB-only
18-source build passes with zero stubs and no PCI C4 imports.

The missing WP4 names are `mhi_alloc_controller`, `mhi_device_get_sync`,
`mhi_device_put`, `mhi_download_rddm_image`, `mhi_free_controller`,
`mhi_pm_resume_force`, `mhi_pm_suspend`, `mhi_power_down`,
`mhi_power_down_keep_dev`, `mhi_prepare_for_power_up`, `mhi_register_controller`,
`mhi_sync_power_up`, `mhi_unprepare_after_power_down`, and
`mhi_unregister_controller`.

Final reviewed WP3 commit is `bfb17f4` on `wifi/wp3-pci-build`. It includes
MSI descriptor/config address consistency, disabling MSI on every partial setup
failure, optional per-vector mask unmask/readback/restore, and fresh endpoint
metadata after failed probe/unregister. Expanded actual-source tests pass under
ASan/UBSan, including a write that takes effect before reporting EIO, stuck
mask refusal and failed-probe retry. Pinned upstream callers use descriptor data
and 64-bit capability, reading addresses from config; descriptor address fields
were nevertheless added to the scoped LinuxKPI compatibility patch for correct
Linux semantics. After review fixes all PCI sources still compile with only the
same 14 WP4 imports. AHB-only object: 510579 bytes, 51 imports, zero stubs.

WP6 must implement the kernel C4 bodies. Its `lx_os_pcie_up` wrapper must return
the already-active borrowed link idempotently after early C2/DMA setup, rather
than invoke C2 `up` twice. C2 itself intentionally rejects a second active `up`.
No full combined PCI kernel or probe-to-WMI execution is claimed before WP4 and
WP6 exist.
