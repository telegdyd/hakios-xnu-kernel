#!/bin/bash
# Host test of the PCIe radio's chain (drivers/board/wifi_pci.c with its KERNEL half left out, and kernel/lx/src/lx_policy.h):
# the boot words, the firmware names, the address rules, the stages' order and their teardown, over a table of fake
# hardware. Under the address and undefined-behaviour sanitizers. workbench/run.sh runs this.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
B=$(mktemp -d)
trap 'rm -rf "$B"' EXIT
mkdir -p "$B/inc/sys/board"
cp "$HERE/../../include/board/wifi.h" "$B/inc/sys/board/wifi.h"
# shellcheck disable=SC2054  # (address,undefined is one argument)
FLAGS=(-std=gnu17 -g -O1 -Wall -Wextra -Werror -Wno-unused-function -Wno-unused-parameter -fsanitize=address,undefined
	-fno-omit-frame-pointer -fno-sanitize-recover=undefined)
# (clang, as it was written for: gcc warns of the long values the tests cut on purpose)
CC=${CC:-$(command -v clang || echo cc)}
"$CC" "${FLAGS[@]}" -I"$B/inc" -o "$B/test" "$HERE/test_wifi_pci.c"
"$B/test"
