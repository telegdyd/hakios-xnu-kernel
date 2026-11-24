#!/usr/bin/env python3
"""Read a kernel-task stackshot from userland/src/kstacks.c: every kernel thread, its name, what it waits on (event and,
with STACKSHOT_THREAD_WAITINFO, the owner of a lock it waits for) and its stack, symbolized from the kernel's `nm -n`.

    kstacks.py STACKSHOT.bin NM.txt [--all]

NM.txt: `nm -n` of the kernel that ran (BUILD/obj/.../kernel.development). The load address comes from the stackshot's
kernelcache load info, the link address from the lowest text symbol; frames are matched after subtracting the slide.
Without --all, idle threads waiting in their usual places (continuations, no stack) are left out.
"""
import bisect, struct, sys

ARRAY_PAD = range(0x20, 0x30)
BEGIN, END, BUF_END = 0x13, 0x14, 0xF19158ED
THREAD, TASK = 0x904, 0x903
SNAP, NAME, LR64, WAITINFO, KC_LOAD, TEXT_EXEC = 0x906, 0x909, 0x914, 0x91D, 0x91C, 0x92E


def items(buf):
    at = 0
    while at + 16 <= len(buf):
        t, size, flags = struct.unpack_from('<IIQ', buf, at)
        data = buf[at + 16: at + 16 + size]
        yield t, flags, data
        if t == BUF_END:
            return
        at += 16 + size


def parse(buf):
    threads, waits, load = [], [], None
    stack = []
    cur = None
    task = {'name': 'kernel_task'}
    for t, flags, data in items(buf):
        if t == BEGIN:
            kind = struct.unpack_from('<I', data)[0]
            stack.append(kind)
            if kind == TASK:
                task = {'name': ''}
            if kind == THREAD:
                cur = {'name': '', 'lr': [], 'snap': None, 'task': task}
                threads.append(cur)
            continue
        if t == 0x905 and len(data) > 88:     # task_snapshot_v2: ... ts_p_comm[32] at the end
            task['name'] = data[-32:].split(b'\0')[0].decode(errors='replace')
            continue
        if t == END:
            kind = stack.pop() if stack else None
            if kind == THREAD:
                cur = None
            continue
        if t in ARRAY_PAD:
            etype, count = flags >> 32, flags & 0xffffffff
            pad = t - 0x20
            esize = (len(data) - pad) // count if count else 0
            elems = [data[i * esize:(i + 1) * esize] for i in range(count)]
            if etype == LR64 and cur is not None:
                cur['lr'] = [struct.unpack('<Q', e[:8])[0] for e in elems]
            elif etype == WAITINFO:
                for e in elems:
                    owner, waiter, context = struct.unpack_from('<QQQ', e)
                    waits.append((waiter, owner, context, e[24]))
            elif etype in (KC_LOAD, TEXT_EXEC) and load is None:
                load = struct.unpack_from('<Q', elems[0])[0]
            continue
        if t in (KC_LOAD, TEXT_EXEC) and load is None and len(data) >= 8:
            load = struct.unpack_from('<Q', data)[0]
        elif t == SNAP and cur is not None:
            tid, event, cont = struct.unpack_from('<QQQ', data)
            state = struct.unpack_from('<I', data, 88)[0] if len(data) >= 92 else 0
            cur['snap'] = (tid, event, cont, state)
        elif t == NAME and cur is not None:
            cur['name'] = data.split(b'\0')[0].decode(errors='replace')
    return threads, waits, load


def symbols(path):
    addrs, names = [], []
    for line in open(path, errors='replace'):
        p = line.split()
        if len(p) >= 3 and p[1] in 'tTsS':
            addrs.append(int(p[0], 16)); names.append(p[2])
    return addrs, names


def main():
    buf = open(sys.argv[1], 'rb').read()
    addrs, names = symbols(sys.argv[2])
    show_all = '--all' in sys.argv
    threads, waits, load = parse(buf)
    text = min(a for a, n in zip(addrs, names) if a > 0xfffffe0000000000) if addrs else 0
    slide = 0
    if load:
        # (load info: the kernel's __TEXT address as it runs; the lowest symbol is near the link address of __TEXT_EXEC)
        slide = load - (text & ~0x3fff) if load > text else 0

    def sym(a):
        a2 = a - slide
        i = bisect.bisect_right(addrs, a2) - 1
        return f'{names[i]}+{a2 - addrs[i]:#x}' if i >= 0 and a2 - addrs[i] < 0x40000 else f'{a:#x}'

    by_tid = {}
    for th in threads:
        if th['snap']:
            by_tid[th['snap'][0]] = th
    print(f'{len(threads)} kernel threads; load {load:#x} slide {slide:#x}' if load else f'{len(threads)} threads; no load info')
    for th in threads:
        if not th['snap']:
            continue
        tid, event, cont, state = th['snap']
        if not show_all and not th['lr'] and cont:
            continue
        print(f"\n== {th['task']['name']} thread {tid} {th['name']!r} state {state:#x} wait event {event:#x}"
              f"{' continuation ' + sym(cont) if cont else ''}")
        for w in waits:
            if w[0] == tid:
                o = by_tid.get(w[1])
                print(f"   waits (type {w[3]}) on {w[2]:#x}, owner thread {w[1]} {o['name'] if o else ''!r}")
        for a in th['lr']:
            print('   ', sym(a))


if __name__ == '__main__':
    main()
