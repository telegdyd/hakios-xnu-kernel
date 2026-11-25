#!/bin/bash
# Host test of the PCI shim under ath11k's PCI path (kernel/lx/src/lx_pci.c) against a fake endpoint: config space, BAR0,
# the MSI vectors, the register window; no hardware. Under the address and undefined-behaviour sanitizers. workbench/run.sh
# runs this.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
B=$(mktemp -d)
trap 'rm -rf "$B"' EXIT
CC=${CC:-$(command -v clang || echo cc)}
# shellcheck disable=SC2054  # (address,undefined is one argument)
FLAGS=(-std=gnu17 -g -O1 -Wall -Wextra -Wno-unused-parameter -DCONFIG_ATH11K_PCI -fsanitize=address,undefined
	-fno-omit-frame-pointer)
"$CC" "${FLAGS[@]}" -I"$HERE/include" -I"$ROOT/include/board" -o "$B/test" "$HERE/test.c"
"$B/test"
