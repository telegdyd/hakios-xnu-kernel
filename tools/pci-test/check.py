#!/usr/bin/env python3
"""Exercise the production PCI shim against a fake endpoint; no hardware access."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
cc = os.environ.get("CC") or shutil.which("clang") or shutil.which("cc")
if not cc:
    raise SystemExit("pci-test: a C compiler is required")
with tempfile.TemporaryDirectory(prefix="hakios-pci-test-") as tmp:
    exe = Path(tmp) / "pci-test"
    cmd = [cc, "-std=gnu17", "-g", "-O1", "-Wall", "-Wextra",
           "-Wno-unused-parameter", "-DCONFIG_ATH11K_PCI",
           "-I" + str(root / "tools/pci-test/include"),
           "-I" + str(root / "xnu/files/bsd/sys"),
           str(root / "tools/pci-test/test.c"), "-o", str(exe)]
    if os.name != "nt":
        cmd[1:1] = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    subprocess.run(cmd, check=True)
    subprocess.run([str(exe)], check=True)
