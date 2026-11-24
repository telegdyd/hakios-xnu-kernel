#!/usr/bin/env python3
"""Execute the production timer section with a deterministic clock and sleep hooks."""
from pathlib import Path
import subprocess
import tempfile

here = Path(__file__).resolve().parent
source = (here.parents[1] / 'wifi/lx/src/lx_base.c').read_text()
section = source[source.index('/* A thousand jiffies'):source.index('/* ---- locks')]
fixture = (here / 'test.c').read_text()
with tempfile.TemporaryDirectory(prefix='lx-timer-') as directory:
    c = Path(directory) / 'test.c'
    binary = Path(directory) / 'test'
    c.write_text(fixture.replace('/* PRODUCTION */', section))
    subprocess.run(['clang', '-std=gnu17', '-Wall', '-Wextra', '-Wno-unused-parameter',
                    '-fsanitize=address,undefined', '-g', str(c), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
