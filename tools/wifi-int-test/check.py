#!/usr/bin/env python3
"""Host test of WP6's integration decisions (WIFI-PLAN.md): the words, the firmware table, the MAC rules, the chain's stage
order and teardown, and what userland/build.sh puts into fw/ for the OnePlus 10 Pro. The production sources are compiled in:
drivers/wifi/hakios.wifi.pci.c (its KERNEL half left out) and wifi/lx/src/lx_policy.h. No hardware, no kernel."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
cc = os.environ.get("CC") or shutil.which("clang") or shutil.which("cc")
if not cc:
    raise SystemExit("wifi-int-test: a C compiler is required")


def build_and_run():
    with tempfile.TemporaryDirectory(prefix="hakios-wifi-int-") as tmp:
        tmp = Path(tmp)
        (tmp / "inc/sys").mkdir(parents=True)
        shutil.copy(root / "xnu/files/bsd/sys/hakios.wifi.h", tmp / "inc/sys/hakios.wifi.h")   # <sys/hakios.wifi.h> as the kernel has it
        exe = tmp / "test"
        cmd = [cc, "-std=gnu17", "-g", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function", "-Wno-unused-parameter",
               "-I" + str(tmp / "inc"), str(root / "tools/wifi-int-test/test_int.c"), "-o", str(exe)]
        if os.name != "nt":
            cmd[1:1] = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-sanitize-recover=undefined"]
        subprocess.run(cmd, check=True)
        subprocess.run([str(exe)], check=True)


def fw_install_block():
    """The negroni block of userland/build.sh, run on a fake ../fw-wifi: which names reach root/fw."""
    text = (root / "userland/build.sh").read_text(encoding="utf-8")
    start = text.index("# ... and the OnePlus 10 Pro's (negroni)")
    end = text.index("# ... and the vibration motor's waveforms")
    block = text[start:end]
    assert "wpss.mdt" in block and block.rstrip().endswith("fi")
    bash = shutil.which("bash")
    if not bash:
        print("fw block: skipped (no bash)")
        return
    with tempfile.TemporaryDirectory(prefix="hakios-fw-block-") as tmp:
        tmp = Path(tmp)
        (tmp / "userland").mkdir()
        fw = tmp / "fw-wifi"
        fw.mkdir()

        def run(files, **env):
            shutil.rmtree(tmp / "userland/root", ignore_errors=True)
            shutil.rmtree(fw)
            fw.mkdir()
            for name in files:
                (fw / name).write_bytes(b"x")
            (tmp / "userland/block.sh").write_text("set -e\n" + block, encoding="utf-8")
            subprocess.run([bash, "block.sh"], cwd=tmp / "userland", check=True, env={**os.environ, **env})
            out = tmp / "userland/root/fw"
            return sorted(p.name for p in out.iterdir()) if out.exists() else []

        local = ["amss.bin", "amss20.bin", "m3.bin", "regdb.bin", "bdwlang.b0e", "bdwlan.elf", "bdwlang.elf", "bdwlan.e0e",
                 "mac.txt", "SHA256SUMS", "firmware-inventory.json", "extract.log", "credentials.local", "bdwlan.json",
                 "bdwlang.b0e.sha256", "notes.txt", "wpss.b02"]
        got = run(local)
        want = sorted(["amss.bin", "amss20.bin", "m3.bin", "regdb.bin", "bdwlang.b0e", "bdwlan.elf", "bdwlang.elf", "bdwlan.e0e"])
        assert got == want, (got, want)
        # nothing of the sort without any of the QCA6490's files; and redwood's files alone leave this block out
        assert run(["mac.txt", "SHA256SUMS", "bdwlang.b0e"]) == []
        assert run(["wpss.mdt", "amss.bin", "m3.bin"]) == []          # (redwood's branch is the one that applies; it is not in this block)
        # a network file follows redwood's rule: installed unless SHARE=1
        assert "network" in run(local + ["network"])
        assert "network" not in run(local + ["network"], SHARE="1")
        assert "network" not in run(local)
        # only m3.bin is enough to be this phone's
        assert run(["m3.bin", "mac.txt"]) == ["m3.bin"]
        print("userland/build.sh negroni fw block: ok")


build_and_run()
fw_install_block()
