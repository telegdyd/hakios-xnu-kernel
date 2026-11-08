#!/usr/bin/env python3
"""Shared, non-executable per-device boot settings."""
import os, re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def load(device=None):
    device = device or os.environ.get('DEVICE','redwood')
    if not re.fullmatch(r'[a-z0-9][a-z0-9-]*',device):
        raise ValueError('invalid device name')
    conf = dict(UEFI_TARGET='redwood', UEFI_MODE='RELEASE', PLATFORM='redwood', FD_BASE=0x9fd00000,
                FD_SIZE=0x200000, PAYLOAD_BASE=0xf0000000, PAYLOAD_MAX=0x4000000,
                MAX_MEM_SIZE=0x180000000, READ_ONLY=0, SESSION_SECONDS=0,
                HEADER_VERSION=1, LOADER_IMAGE_BASE=0xa1000000)
    path = ROOT/'devices'/device/'boot.conf'
    if path.exists():
        for line in path.read_text().splitlines():
            line = line.split('#',1)[0].strip()
            if not line: continue
            key, value = map(str.strip,line.split('=',1))
            if key not in conf: raise ValueError(f'unknown boot setting: {key}')
            conf[key] = value if key in ('UEFI_TARGET','UEFI_MODE','PLATFORM') else int(value,0)
    elif device not in ('redwood','qemu'):
        raise ValueError(f'{path} missing')
    if conf['PLATFORM'] not in ('redwood','negroni','qemu-virt'):
        raise ValueError('unsupported loader platform')
    if not re.fullmatch(r'[a-z0-9-]+',conf['UEFI_TARGET']):
        raise ValueError('invalid UEFI target')
    assert conf['PAYLOAD_BASE'] % 0x10000 == 0
    assert conf['PAYLOAD_MAX'] > 0x100000
    assert conf['UEFI_MODE'] in ('DEBUG','RELEASE')
    conf['DEVICE'] = device
    return conf

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--value')
    args = parser.parse_args()
    c = load()
    if args.value:
        print(c[args.value])
        raise SystemExit(0)
    print(f"#define PAYLOAD_BASE {c['PAYLOAD_BASE']:#x}ULL")
    print(f"#define PAYLOAD_MAX_SIZE {c['PAYLOAD_MAX']:#x}ULL")
    print(f"#define MAX_MEM_SIZE {c['MAX_MEM_SIZE']:#x}ULL")
    print(f"#define LOADER_READ_ONLY {c['READ_ONLY']}")
    print(f"#define LOADER_SESSION_SECONDS {c['SESSION_SECONDS']}")
    # QEMU still selects its platform from the virtual display, as before.
    if c['DEVICE'] == 'negroni':
        print('#define LOADER_DEFAULT_NEGRONI 1')
