#!/usr/bin/env python3
# Print IO80211.framework's event table (convertApple80211EventToString's eventArray):
# 0xab0 bytes of { int32 code; pad; char *name } at the vmaddr given, from the extracted dylib.
# usage: events.py IO80211 [0x1dabacb00]
import struct, sys

data = open(sys.argv[1], 'rb').read()
table = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x1dabacb00
magic, cpu, sub, ftype, ncmds, size, flags, _ = struct.unpack_from('<IiiIIIII', data, 0)
segments = []
at = 32
for _ in range(ncmds):
    cmd, cmdsize = struct.unpack_from('<II', data, at)
    if cmd == 0x19:
        name = data[at + 8:at + 24].rstrip(b'\0')
        vmaddr, vmsize, fileoff, filesize = struct.unpack_from('<QQQQ', data, at + 24)
        segments.append((vmaddr, vmsize, fileoff, filesize))
    at += cmdsize


def offset(vm):
    for vmaddr, vmsize, fileoff, filesize in segments:
        if vmaddr <= vm < vmaddr + filesize:
            return fileoff + vm - vmaddr
    return None


def cstring(vm):
    o = offset(vm)
    if o is None:
        return '<%#x>' % vm
    return data[o:data.index(b'\0', o)].decode('latin-1')


base = offset(table)
for i in range(0, 0xab0, 16):
    code, pointer = struct.unpack_from('<iQ', data, base + i)[0], struct.unpack_from('<Q', data, base + i + 8)[0]
    # (a dyld-cache pointer may carry high bits; the low 36 are the address within the cache)
    candidates = [pointer, pointer & 0xfffffffff, (pointer & 0xfffffffff) | 0x100000000]
    name = next((cstring(p) for p in candidates if offset(p) is not None), '<%#x>' % pointer)
    print(code, name)
