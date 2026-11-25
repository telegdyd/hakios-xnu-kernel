#!/bin/bash
# Host test of the MHI host under ath11k's PCI path (kernel/lx/src/lx_mhi.c) against a scripted fake MHI chip: BHI, BHIe,
# the execution environments, the channels, M0/M2/M3 and the IPCR link; no hardware, no kernel. Under the address and
# undefined-behaviour sanitizers. workbench/run.sh runs this.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
B=$(mktemp -d)
trap 'rm -rf "$B"' EXIT
CC=${CC:-$(command -v clang || echo cc)}
# shellcheck disable=SC2054  # (address,undefined is one argument)
FLAGS=(-std=gnu17 -g -O1 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Wno-sign-compare
	-Wno-missing-field-initializers -DCONFIG_ATH11K_PCI -fsanitize=address,undefined -fno-omit-frame-pointer
	-fno-sanitize-recover=undefined)
"$CC" "${FLAGS[@]}" -I"$HERE/include" -idirafter "$ROOT/kernel/lx/include" -I"$ROOT/include/board" \
	-DLX_HAKIOS_LX_H="\"$ROOT/include/board/lx.h\"" -o "$B/test" "$HERE/test.c"
"$B/test"
