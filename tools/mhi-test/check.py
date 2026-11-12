#!/usr/bin/env python3
"""Host test of wifi/lx/src/lx_mhi.c against a scripted fake MHI chip; no hardware, no kernel."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
cc = os.environ.get("CC") or shutil.which("clang") or shutil.which("cc")
if not cc:
    raise SystemExit("mhi-test: a C compiler is required")
sys_dir = root / "xnu/files/bsd/sys"
with tempfile.TemporaryDirectory(prefix="hakios-mhi-test-") as tmp:
    exe = Path(tmp) / "mhi-test"
    cmd = [cc, "-std=gnu17", "-g", "-O1", "-Wall", "-Wextra",
           "-Wno-unused-parameter", "-Wno-unused-function", "-Wno-sign-compare", "-Wno-missing-field-initializers", "-DCONFIG_ATH11K_PCI",
           "-I" + str(root / "tools/mhi-test/include"),
           "-idirafter", str(root / "wifi/lx/include"),
           "-I" + str(sys_dir),
           '-DLX_HAKIOS_LX_H="' + str(sys_dir / "hakios.lx.h") + '"',
           str(root / "tools/mhi-test/test.c"), "-o", str(exe)]
    if os.name != "nt":
        cmd[1:1] = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-sanitize-recover=undefined"]
    subprocess.run(cmd, check=True)
    subprocess.run([str(exe)], check=True)
