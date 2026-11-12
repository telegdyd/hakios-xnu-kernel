# Wi-Fi WP8: review of the merged PCIe path

2026-11-12, read only, against `negroni-dev` at `75c77c0` (WP1–WP6 merged). No phone and no build. Checked: every
register write against its source, the teardown order, the panic path, the SMMU bank choice, the DMA window, and
whether redwood changed. The findings go to the packages that own the code.

## Findings

### F1 (high, WP1): teardown restores whole shared GCC vote words

`hakios.pcie.c` `clocks_phy()` sets bits in `GCC 0x62000` (VOTE0, bits 11/12/19/22) and `0x62008` (VOTE2, 0x2f,
later bit 4) through `bits()` → `save_write()`. That saves the **whole old word**, and `hakios_pcie_down()` writes the
whole word back. Other drivers use these words too:
`hakios.s3908.c` :1041 sets QUP0 bits 6/7/8/9/14 in `0x62008` (touch), and `hakios.adreno.c` :5065 uses
`0x62000` bits 15/16 (GPLL0 for the GPU).

Wi-Fi comes up on its own thread while the system starts. If touch or the GPU votes after Wi-Fi saved the word, any
teardown (a FAILED stage 4–11, `wifi_keep=0`, the end of the run, or a panic) clears their bits. The QUP0 or GPU clock
then stops, and the next access to that block hangs or takes an SEA. Round 1 is likely to stop at a FAILED stage, so
this can break touch and GPU in the same boot and spoil their regression lines. The read-modify-writes also race
each other, because there is no shared lock.

Fix: for the vote words, record only the bits this driver turned on (bits that were 0 before and are in the set
mask). Clear only those bits on down, with a fresh read-modify-write. Better still, use a small shared GCC-vote helper
with one lock for touch, GPU and PCIe; the panic path then does the read-modify-write without the lock.

### F2 (medium, WP6 + WP4): the `wifi_smmu=bypass` fallback cannot get past stage 6

- `hakios.lx.c` :691 asks `kmem_alloc_contig` for `max_pnum = 0xfffff` and calls that "below 4 GiB". On arm64 XNU a
  `ppnum_t` counts 16 KiB pages, so the limit is 16 GiB. The check after it (:697) catches a pool above 4 GiB and
  returns ENOMEM, so nothing unsafe happens, but bypass usually fails at stage 4. It should use
  `atop(0x100000000ull) - 1`. [I] units from XNU's pmap, not tested on the phone.
- Even with a pool below 4 GiB, `lx_mhi.c` :2940-2942 limits the MHI window to the fixed range
  `HAKIOS_WIFI_IOVA_BASE .. +64 MiB-1` (0xa0000000–0xa3ffffff). In bypass mode the pool's IOVA is its physical
  address (`lx_pool.iova = physical`), so `blk_get()` refuses every block ("outside the pool window"). Stage 6 then
  fails.

So round 2's planned answer for a stage-4 failure does not work. Fix: get the window from the pool's real IOVA (C4
reports it) rather than from the constant. Or, if the knob is to stay translate-only for now, remove it from the
stage table.

### F3 (low, WP6): the PSK is wiped but the SSID is kept

`pci_thread()` (`hakios.wifi.pci.c` :933) wipes `psk` and sets `psk_length = 0` but leaves `ssid_length`. After that,
`hakios_wifi_pci_network()` returns **true** with a 0-byte passphrase, so `hakios_wifi_network()` never falls back to
`fw/network`. No caller reaches it today after the wipe: rejoin uses the station's stored standing join, and
`lx_station_join` runs before the wipe. Any later caller would get a bad network, though. Fix: also zero `ssid` and
`ssid_length`.

### F4 (low, plan/C7): `wifi_psk` as a boot word is in the boot-args string

The C7 rule "never log the PSK" holds for our own lines. But a boot word is part of the kernel's boot-args string,
which the loader echoes, panics print, and userland can read through `kern.bootargs`. `hakios.wifi.c` :3787 already
gives this as the reason `fw/network` is not a boot argument. That is fine for a bring-up network. For round 3 the
credentials should come only from `fw/network` or from wifid, not from the words.

### Open item for round 3 (not a defect)

Automatic BDF choice (`lx_policy.h` :104) gives `bdwlan.elf` / `bdwlang.elf` for board id 0xff. Android uses
`bdwlang.b0e`. Round 1 covers this with `wifi_board=bdwlang.b0e` (WP7 build default). For default-on (round 3), that
word must become a kernel default, or the rule must produce `.b0e`.

## Fixes (2026-11-12, main session)

- **F1.** `hakios.pcie.c`: VOTE0/VOTE2 go through `vote_bits()`. It journals only the bits that were 0 and that the
  driver sets (a mask, not the old word). On down, `vote_clear()` reads the word again and clears only those bits.
  No lock was added: the read-modify-write can still race touch's or the GPU's one-time vote [I], but a teardown no
  longer clears their bits. The other journal entries still restore the whole word.
- **F2.** `hakios.lx.c`: the bypass pool limit is now `atop(4 GiB) - 1` (16 KiB pages). `lx_mhi.c` takes the window
  from the pool's real radio address (new `lx_pool_window()` in `lx_base.c`, which C4 reports), so bypass gets
  `pa .. pa+64 MiB-1`. With no pool, `register` returns ENODEV. New mhi-test cases: a full power-up with the pool at
  0x88000000, and the no-pool case.
- **F3.** `pci_thread()` wipes `ssid` and `ssid_length` together with the PSK.
- F4 and the BDF item stay open for round 3.

## Checked, no finding

- **PCIe writes vs source.** GCC GDSC, pipe mux (2 = XO, then 0), RCHNG (GPLL0/6), AUX (XO/1), VOTE0/VOTE2 values,
  CLKREF 0x9c004, BCRs 0x7b000/0x7c01c/0x7c020: match slop 0106/0120/0122. PARF device type, PHY_CTRL, SYS_CTRL
  bit 29, MHI bypass, PM_CTRL, AXI halt, the vendor DBI/slave-space recipe (0x350 = 0, 0x358 = 0x4000000): match 0262.
  CLKREQ pin 95 function and pull-up: match 0262. The PHY table follows the DT, with the NOCSR release before
  SW_RESET/START as in 0120. DBI bridge, bus, LNKCTL2 gen, iATU CFG0/MEM at `dbi+0x1000` (DT "atu"), PERST 100 ms →
  1 ms → LTSSM, 3 tries, DLLLA + ELBI XMLH_LINK_UP: match 0262. Timing: BT_EN low then ≥100 ms, XO low 10 ms, high
  200 µs, WLAN_EN, 1 + 5 ms, XO low, 50 ms: matches 0262 (cnss2). RPMh TCS register offsets and the AMC trigger match
  rpmh-rsc. W1C status bits are never echoed back in narrow config writes.
- **Teardown order.** The chain undoes the driver, the pool's mapping, the SMMU context, then PCIe. The driver's stop
  (`lx_pci_unregister_driver`) runs ath11k's remove, disables MSI, stops the poll thread, then
  `lx_os_pcie_down` (PERST and WLAN_EN low). A failed probe or start takes the same path. So the radio is held in
  reset before the context's SMR is cleared. The order matters: an unmatched stream may bypass to raw PA 0xa0000000,
  which is RAM. `hakios_pcie_down()` does PERST/WLAN_EN/XO low, then undoes its journal in reverse (LTSSM, iATU and
  DBI before PHY, PARF, clocks and GDSC), then removes the rail votes in reverse. A second call does nothing.
  The pool memory is kept, never freed, so it cannot be reused while the uncached alias exists.
- **Panic path.** `hakios_wifi_quiesce()` → `hakios_wifi_pci_quiesce()` → `hakios_pcie_down()` only: no mutex, no
  allocation, and the waits are spin loops. The RPMh slot belongs only to us, so `hakios_rsc_lock` is not needed. The
  SMMU is not touched (C3 release stays on normal threads, under the lock). A TCS slot with an unfinished transaction
  is quarantined: its rails stay voted on, but PERST and WLAN_EN still go low.
- **SMMU bank choice.** Banks start at `max(IDR1.NUMS2CB, 1)`. The claim skips banks in use by a valid SMR→S2CR,
  banks with M set or a pending TLB operation, and banks it has tried before. It takes a bank only if CBAR type is 0,
  or type 1 with VMID 0xff. Each claim write is read back. CBAR/CBA2R are never restored. TLB sync is bounded at
  500 ms. Release: SMR invalid, then S2CR, then SCTLR 0 and sync; on error the context and its tables are quarantined.
  Sound uses the same 17-write sequence (WP2). The tables (level 1 → 2 → 3, T0SZ 25, 4 KiB granule) and the PTEs are
  right for a 64 MiB pool (1 + 1 + 32 pages).
- **DMA window (translate).** Every DMA address comes from the one pool at IOVA 0xa0000000 + 64 MiB: coherent rings,
  noncoherent, MHI blocks (window-checked), the MSI mailbox, and skb data (pool slots). `pmap_kextract` and
  `lkpi_dma_map_phys` refuse anything outside the pool. The top address is 0xa3ffffff, under 32 bits. PAs are checked
  against the 36-bit PASize.
- **Redwood unchanged.** Without `wifi_soc`, `hakios_wifi_pci_init()` returns false and every new hook in
  `hakios.wifi.c` is a no-op. `wifi_keep` is forced to 1 only on the PCI path. The shared `hakios_wifi_mac_parse()`
  accepts exactly what the inline parser accepted. The ath11k object is AHB-only for redwood. The `wifi_smmu` numeric
  path (:4072) is reached only after the early PCI return.
- **Words.** Each word is read by its exact text and refused before any hardware access. `pcie=1` and `wifi=1` are
  refused together on both sides (C2 and the chain). `wifi_msi` absent means 32.

## Not covered

Nothing was built and no host test was rerun. Nothing was checked against the phone: GCC/PHY values come from the
references and the DT. lx_mhi.c's state machine was not re-reviewed (WP4's 2553 host tests stand). ath11k's upstream
files were not re-read.
