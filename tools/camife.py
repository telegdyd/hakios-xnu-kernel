#!/usr/bin/env python3
"""Make the IFE init header the kernel's cam_probe level 5 replays (drivers/camera/hakios.cam_probe.c),
out of a decoded CamX IFE init packet (device-info/.../camera/isp/decode-mem.py output).

  camife.py <snap dir> <out.h> <packet tag>...
  camife.py device-info/2026-11-08-initial/camera/isp/android/snap-20261119-013855 hakios.cam_ife_init.h       req0-op0 req270-op1

Reads <snap dir>/mem0/packets.txt for each IFE packet's (opcode 0x0f00000N: 0 init, 1 update) first
command buffer (meta COMMON: the IFE's own CLC writes) and its DMI tables from <snap dir>/mem0/dmi/,
and joins them in the order given, as CamX applies the init packet and then a request's update. The header is one uint32_t stream in write
order: per op a word (kind << 24 | count), the register offset (IFE-relative; for DMI also the
DMISel << 24), then count values. Kind 1 is a run of consecutive registers (CDM REG_CONT), kind 2 a
DMI table of count 32-bit words for the module DMI port whose config register is the offset.
Also emits a CDM FIFO block and its LUT payload, with offsets for the DMA address fixups.
"""
import re
import struct
import sys
from pathlib import Path

args = sys.argv[1:]
live_path = None
if '--live' in args:
    i = args.index('--live')
    live_path = Path(args[i + 1])
    del args[i:i + 2]
snap, out, tags = Path(args[0]), Path(args[1]), args[2:]
lines = (snap / 'mem0' / 'packets.txt').read_text().splitlines()
ops, cont_words, tables = [], 0, 0
dmi_addresses = []


def packet(tag):
    # Tags repeat across devices (sensor, actuator, IFE): take the IFE's opcodes 0x0f00000N.
    start = next(i for i, l in enumerate(lines) if l.startswith(f'== packet {tag} ') and 'opcode 0f00000' in l)
    return lines[start + 1:]


for tag in tags:
  in_common = False
  for l in packet(tag):
      if l.startswith('== packet'):
          break
      s = l.strip()
      if s.startswith('cmd hdl'):
          if in_common:
              break  # only the first (IFE common) command buffer
          in_common = s.endswith('meta COMMON')
          continue
      if not in_common:
          continue
      m = re.match(r'CONT ([0-9a-f]+) x(\d+): (.*)', s)
      if m:
          reg, n = int(m[1], 16), int(m[2])
          vals = [int(v, 16) for v in m[3].split()]
          assert len(vals) == n, s
          ops.append((1, reg, vals))
          cont_words += n
          continue
      m = re.match(r'DMI reg ([0-9a-f]+) sel ([0-9a-f]+) bytes (\d+) lut@([0-9a-f]+) -> (dmi/\S+)', s)
      if m:
          reg, sel, n = int(m[1], 16), int(m[2], 16), int(m[3])
          data = (snap / 'mem0' / m[5]).read_bytes()
          assert len(data) == n and n % 4 == 0, s
          vals = [int.from_bytes(data[i:i + 4], 'little') for i in range(0, n, 4)]
          ops.append((2, reg | sel << 24, vals))
          dmi_addresses.append(int(m[4], 16))
          tables += 1
          continue
      raise SystemExit(f'unexpected line in {tag}: {s}')

# --live <ifesnap words.txt>: CLC words (0x3900-0xc0ff) live in stock preview that no packet writes
# (CamX's register-resident curves at 0x4100-0x5170 and 0x6d00-0x7570 among them), as kind 1 runs
# after the packets. Block version words (offset 0 of each 0x200 block) are left out.
live_words = 0
if live_path:
    covered = set()
    for kind, reg, vals in ops:
        if kind == 1:
            covered.update(reg + 4 * i for i in range(len(vals)))
    live = {}
    for l in live_path.read_text().splitlines():
        if l.strip():
            a, v = l.split()
            live[int(a, 16)] = int(v, 16)
    miss = sorted(a for a, v in live.items() if 0x3900 <= a < 0xc100 and a % 0x200 and a not in covered)
    run = []
    for a in miss + [None]:
        if run and (a is None or a != run[-1] + 4):
            ops.append((1, run[0], [live[x] for x in run]))
            live_words += len(run)
            run = []
        if a is not None:
            run.append(a)
    cont_words += live_words

words = []
# RT CDM CHANGE_BASE is relative to rt-wrapper-base. IFE0 is at the wrapper start.
commands = [0x08000000]
lookups = []
addresses = iter(dmi_addresses)
for kind, reg, vals in ops:
    words += [kind << 24 | len(vals), reg] + vals
    if kind == 1:
        assert 0 < len(vals) <= 0xffff
        commands.extend([0x03000000 | len(vals), reg, *vals])
    else:
        assert 0 < 4 * len(vals) <= 0x10000
        lookups.append((len(commands) + 1, next(addresses), vals))
        commands.extend([0x01000000 | (4 * len(vals) - 1), 0, reg])
# GEN_IRQ marks execution of the entire block. FIFO 0 uses byte 0 of the user data.
commands.extend([0x06000001, 0x35])
command_bytes = 4 * len(commands)
assert command_bytes <= 0x100000
payload = bytearray()
for index, stock_addr, vals in lookups:
    # Keep the stock alignment, including tables with an unaligned start.
    padding = (stock_addr - command_bytes - len(payload)) % 64
    payload.extend(bytes(padding))
    commands[index] = command_bytes + len(payload)
    payload.extend(struct.pack(f'<{len(vals)}I', *vals))


def word_rows(words):
    return ',\n'.join('\t' + ', '.join(f'0x{w:08x}' for w in words[i:i + 8]) for i in range(0, len(words), 8))


payload.extend(bytes(-len(payload) % 4))
cdm_words = commands + list(struct.unpack(f'<{len(payload) // 4}I', payload))
fixups = [index for index, _, _ in lookups]
out.write_text(f'''/* Generated by tools/camife.py from {snap.name} {' + '.join(tags)}: the stock CamX IFE init packet
 * (IMX789 mode 1 preview on OxygenOS), {cont_words} register writes and {tables} DMI tables in write order. */
#define CAM_IFE_INIT_SOURCE "{snap.name} {' + '.join(tags)}{' + live ' + live_path.parent.name if live_path else ''}"
#define CAM_IFE_INIT_WRITES {cont_words}u
#define CAM_IFE_INIT_TABLES {tables}u
static const uint32_t cam_ife_init[] = {{
{word_rows(words)}
}};
#define CAM_IFE_CDM_BYTES {command_bytes}u
static const uint32_t cam_ife_cdm[] = {{
{word_rows(cdm_words)}
}};
static const uint32_t cam_ife_cdm_fixups[] = {{
{word_rows(fixups)}
}};
''')
print(f'{out}: {len(ops)} ops, {cont_words} register writes ({live_words} live-only), {tables} DMI tables, {len(words)} words')
