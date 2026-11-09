# Negroni native UFS: all eight cores

XNU performs reads, writes and synchronous cache flushes directly, leaving no
loader service running on core 0. Qualcomm's UEFI still initializes the
controller/PHY/clocks before entry and remains in reserved memory for return.
This is firmware-initialized native I/O, not a cold-start PHY/power driver.

## Boot configuration

Use `ufs=2 ufs_native=1 ufs_native_write=1 ufs_write=1 cpus=8`, with
`root_apfs=ufs_darwin var_hfs=ufs_darwin_data`. **Omit bootcpu entirely**:
`bootcpu=0` selects the other-core return convention and is not equivalent.
Negroni's configuration enables native storage by default. Other devices retain
their existing behavior unless they explicitly enable `ufs_native_write=1`.

The loader prepares version-1 snapshots of up to eight ordinary LUN GPT headers
in reserved memory before XNU starts. Address/version/mask fields extend the
native offer inside existing alignment padding: mailbox offsets and total size
stay compatible. XNU validates snapshot address, mapping and header CRC, then
compares native LUN reads against the snapshots and checks partition bounds.
Older loaders still work in legacy read mode; full native mode refuses startup
without valid snapshots instead of waiting for a nonexistent mailbox server.

The smaller cores join after the existing `small_cores_after` delay. Before
that, `hw.ncpu` reports eight but `hw.activecpu` can temporarily report five.
After they join, all eight report active.

## Writes and ownership

- WRITE(10), whole 4096-byte blocks, split into slot-sized transfers.
- One immutable window in absolute LUN-0 LBAs: only the loader-verified darwin
  data region, after root/cryptex block 2042367 and before the partition end.
- Exact command encoding checked before and after slot copy: LUN, direction,
  flags, lengths, operation and reserved bytes. No writes to other LUNs, boot
  firmware, root, cryptexes, GPT or past darwin.
- A mutex orders WRITE against synchronous SYNCHRONIZE CACHE(10), with IMMED
  clear. HFS journal flushes use this path. Raw writes additionally flush and
  compare read-back before returning; the block filesystem owns journaling.
- Native reads retain ownership while the data volume writes. No runtime
  mailbox requests, sysctl ownership switches, or fallback in this mode.
- Completion errors/underruns fail the operation. An uncertain submitted write
  disables the native controller path; timed-out DMA slots are quarantined.
  No replay through firmware after an uncertain write.
- Orderly return flushes acknowledged native writes while threads can still
  sleep. After other CPUs stop, quiesce restores UEFI's list without sleeping
  locks. Panic recovery does not promise a filesystem sync; HFS journal
  recovery remains necessary after abnormal shutdown.

The legacy loader-backed write path is retained. Its first write switches off
native reads until the raw darwin device closes; full native mode does not use
that ownership scheme.

## Validation (2026-11-09)

Host checks execute actual production functions with address/undefined-behavior
sanitizers, stopping on the first sanitizer error:

```
python3 tools/ufs-native-write-test.py
python3 tools/ufs-native-storage-test.py
python3 tools/ufs-native-snapshot-test.py
python3 devices/negroni/tools/standalone-payload-test.py
```

Coverage includes command/range guards, chunking, DMA preparation, timeout
quarantine, completion/flush failures, native data/raw routing and read-back,
no fallback, legacy routing, loader/kernel ABI, GPT snapshot alignment/mapping/
version/mask/CRC, and both standalone storage paths.

Phone evidence is outside git in `device-info/2026-11-08-initial/uefi/`:

- `native-ufs-readcheck.log`: 369 ranges, 864,084,366 compared bytes, zero
  differences/failed reads; 3,581 native reads, zero errors. Covers all six LUN
  GPTs, firmware partitions, super metadata and varied darwin ranges.
- `native-ufs-rw7-verified.json`: native HFS read/write/flush with a watching
  loader, exact 8 MiB file read-back, zero errors/mailbox requests. Owner
  confirmed screen, touch, volume buttons and apps.
- `native-ufs-rw0-persistence-verified.json` and
  `native-ufs-rw0-written-verified.json`: direct core-0 boot, all eight CPUs
  active, exact file persistence, zero loader requests/fallbacks/write errors.
  An eight-thread test consumed 1,685 ms CPU time in 254 ms elapsed.
- `native-ufs-rw0-return.log`: UEFI list restored, no CPU left running,
  35,940 successful native reads, zero errors/refused commands.
- `native-ufs-rw0-second-persistence-verified.json`: a second eight-core boot
  preserved the file written on core 0 exactly, with zero storage errors or
  mailbox requests. Owner confirmed the eight-core UI works normally.
- `native-menu-final-ram-verified.json`: the packaged menu's native write/flush
  and file read-back checks passed. The smaller cores subsequently joined and
  reported eight active processors before installation.
- `native-menu-image-readback-verified.json`: only boot_a installed, all
  29,261,824 bytes read back exactly. Image SHA256
  `396ee39d48f8800fa3591e2e0063db13611fe343834d21e62810fe0251c060fd`;
  kernel SHA256 `ae915602fac77f933f44dd3a4c453c0c05ff99619bb730da2353caabce38328c`.
- `native-menu-installed-verified.json`: an ordinary installed restart reached
  eight active CPUs, preserved the generated file, and reported zero native
  write errors, loader requests and fallbacks. The iOS default and ten-second
  countdown were retained. The previous boot menu is retained in
  `native-menu-preflight/previous-boot-a.img` for rollback; no data-volume backup
  was created.

The kernel compiles/links in the existing WSL tree, retaining the known kext
export-list failures accepted by the project's build wrapper. Hardware checks
cover normal read/write/persistence and return, not arbitrary power loss,
injected controller faults or long-term endurance. The owner declined a full
data-volume backup due to disk space and authorized continuing without it.
