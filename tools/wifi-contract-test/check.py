#!/usr/bin/env python3
"""Wave-0 boundary smoke check; no hardware, firmware or full kernel required.

Use --upstream DIR with Linux 7.0.14's mhi.h, ath11k-mhi.c and pci.c to also
check the public declarations against upstream and compile its channel configs.
Linux foundational types are test doubles: this is NOT a LinuxKPI/kernel build.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
SYS = ROOT / "xnu/files/bsd/sys"
LX = ROOT / "wifi/lx/include"


def run(*args):
    result = subprocess.run(args, text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError(result.stderr or result.stdout)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cc", default="clang")
    ap.add_argument("--upstream", type=Path)
    opts = ap.parse_args()
    if not shutil.which(opts.cc):
        ap.error("C compiler not found: " + opts.cc)
    with tempfile.TemporaryDirectory(prefix="wifi-contract-") as directory:
        tmp = Path(directory)
        def put(name, body):
            path = tmp / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(body)
            return str(path)

        # Include the actual headers twice to catch missing guards and type clashes.
        headers = ["hakios.wifi.h", "hakios.pcie.h", "hakios.smmu.h", "hakios.qrtr.h", "hakios.lx.h"]
        source = put("headers.c", "\n".join('#include "' + h + '"' for h in headers * 2))
        flags = [opts.cc, "-Wall", "-Wextra", "-Werror", "-I" + str(SYS), "-I" + str(SYS.parent)]
        run(*flags, "-std=c17", "-fsyntax-only", source)
        run(*flags, "-x", "c++", "-std=c++17", "-fsyntax-only", source)
        # Also compile lx_os.h via the same single-source boundary as mk-ath11k.sh.
        source = put("lx.c", '#include "lx_os.h"\n')
        run(*flags, "-idirafter", str(LX), '-DLX_HAKIOS_LX_H="' + str(SYS / "hakios.lx.h") + '"',
            "-fsyntax-only", source)

        put("libkern/libkern.h", "#include <stdio.h>\n#include <stdarg.h>\n")
        harness = put("stage.c", '''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "hakios.wifi.h"
static unsigned count;
void loader_log_line(const char *line) {
    assert(strncmp(line, "wifi: STAGE ", 12) == 0);
    assert(strchr(line, '\\n') == line + strlen(line) - 1);
    assert(strchr(line, '\\r') == NULL);
    assert(strlen(line) < 352);
    count++;
    fputs(line, stderr);
}
int main(void) {
#define STAGE(n, name) hakios_wifi_stage(n, #name, 1, "detail %u", n);
    HAKIOS_WIFI_STAGES(STAGE)
#undef STAGE
    hakios_wifi_stage(6, "mhi", 0, "%s", "stall\\nreason\\rmore");
    char long_detail[700];
    memset(long_detail, 'x', sizeof(long_detail) - 1);
    long_detail[sizeof(long_detail) - 1] = 0;
    hakios_wifi_stage(8, "qmi", 1, "%s", long_detail);
    assert(count == 15);
    return 0;
}
''')
        executable = str(tmp / "stage")
        stage_source = str(ROOT / "drivers/wifi/hakios.wifi.stage.c")
        run(*flags, "-I" + str(tmp), stage_source, harness, "-o", executable)
        result = run(executable)
        assert result.stdout == result.stderr, "printf and loader log differ"
        rows = re.findall(r"X\((\d+), (\w+)\)", (SYS / "hakios.wifi.h").read_text())
        assert [int(n) for n, _ in rows] == list(range(1, 14))
        expected = [f"wifi: STAGE {n} {name}: ok detail {n}" for n, name in rows]
        expected += ["wifi: STAGE 6 mhi: FAILED stall reason more", "wifi: STAGE 8 qmi: ok " + "x" * 255]
        assert result.stdout.splitlines() == expected, "stage wire format changed"

        # Compile the production logger to the kernel's real target, with only
        # libkern's C-library declarations substituted (no host stdio ABI).
        put("libkern/libkern.h", "#include <stdarg.h>\n"
            "int printf(const char *, ...);\n"
            "int snprintf(char *, unsigned long, const char *, ...);\n"
            "int vsnprintf(char *, unsigned long, const char *, va_list);\n")
        run(*flags, "-target", "arm64-apple-ios16.5", "-ffreestanding", "-I" + str(tmp),
            "-c", stage_source, "-o", str(tmp / "stage-arm64.o"))
        run(*flags, "-target", "arm64-apple-ios16.5", "-ffreestanding", "-fsyntax-only",
            put("arm64-headers.c", "\n".join('#include "' + h + '"' for h in headers)))

        # LinuxKPI scaffolding only. The MHI declarations themselves are real.
        put("linux/types.h", "#pragma once\n#include <stddef.h>\n#include <stdbool.h>\n"
            "typedef unsigned char u8; typedef unsigned u32;\n#define __iomem\n")
        put("linux/device.h", "#pragma once\nstruct device { void *p; };\nstruct device_driver { void *p; };\n")
        put("linux/dma-direction.h", "#pragma once\nenum dma_data_direction { DMA_BIDIRECTIONAL, DMA_TO_DEVICE, DMA_FROM_DEVICE, DMA_NONE };\n")
        put("linux/dma-mapping.h", "#pragma once\ntypedef unsigned long long dma_addr_t;\n")
        put("linux/bitops.h", "#pragma once\n#define BIT(n) (1UL << (n))\n")
        mhi = (LX / "linux/mhi.h").read_text()
        probe = ('#include "' + str(LX / "linux/mhi.h") + '"\n') * 2
        if opts.upstream:
            upstream = (opts.upstream / "mhi.h").read_text()
            norm = lambda s: re.sub(r"\s+", " ", re.sub(r"/\*.*?\*/", "", s, flags=re.S)).strip()
            for kind, name, body in re.findall(r"(struct|enum) (\w+) (\{.*?\n\};)", mhi, re.S):
                if name == "mhi_controller":
                    continue  # intentionally opaque host internals
                found = re.search(r"\b" + kind + " " + name + r" (\{.*?\n\};)", upstream, re.S)
                assert found and norm(body) == norm(found[1]), "upstream type mismatch: " + name
            bare = re.sub(r"/\*.*?\*/", "", mhi, flags=re.S)
            upstream_bare = re.sub(r"/\*.*?\*/", "", upstream, flags=re.S)
            for decl in re.findall(r"^(?:struct mhi_controller \*|enum mhi_state |void |int )(?:__)?mhi_\w+\([^;]+;", bare, re.M):
                assert norm(decl) in norm(upstream_bare), "upstream signature mismatch: " + decl
            ath = (opts.upstream / "ath11k-mhi.c").read_text()
            configs = ath[ath.index("static const struct mhi_channel_config"):ath.index("void ath11k_mhi_set_mhictrl_reset")]
            probe += "#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))\n" + configs
            probe += "\nvoid use_configs(void) { (void)ath11k_mhi_config_qca6390; (void)ath11k_mhi_config_qcn9074; }\n"
            # All controller members dereferenced by the actual PCI/MHI callers.
            fields = set(re.findall(r"\b(?:mhi_ctrl|mhi_cntrl)->(\w+)", ath + (opts.upstream / "pci.c").read_text()))
            probe += "\nvoid use_fields(struct mhi_controller *c) {\n"
            probe += "\n".join("(void)c->" + field + ";" for field in sorted(fields)) + "\n}\n"
        source = put("mhi.c", probe)
        run(*flags, "-I" + str(tmp), "-fsyntax-only", source)
        print("PASS: C/C++ and arm64 headers; arm64 logger object; lx_os boundary; all 13 stage lines, failure, sanitization, truncation, both log sinks; MHI header" +
              ("; pinned upstream types/signatures, actual ath11k configs and controller fields" if opts.upstream else ""))


if __name__ == "__main__":
    main()
