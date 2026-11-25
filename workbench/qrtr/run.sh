#!/bin/bash
# Build and run the tests of the QRTR router (drivers/board/qrtr.c: the link of either the Wi-Fi DSP's GLINK or the PCIe
# radio's MHI IPCR channels under the QMI helpers) on this machine, under the address and undefined-behaviour
# sanitizers, behind a fake link. workbench/run.sh runs this.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
B=$(mktemp -d)
trap 'rm -rf "$B"' EXIT
# (the kernel's files at their paths in its tree: tools/kernel-view.sh)
"$HERE/../../tools/kernel-view.sh" "$B/kernel"
K=$B/kernel/bsd
# shellcheck disable=SC2054  # (address,undefined is one argument)
FLAGS=(-std=gnu11 -O1 -g -Wall -Wextra -Werror -D_GNU_SOURCE -fsanitize=address,undefined -fno-sanitize-recover=all -pthread)
"${CC:-cc}" "${FLAGS[@]}" -I"$HERE/shim" -I"$K" -o "$B/test" "$HERE/test_qrtr.c" "$K/dev/board/qrtr.c"
"$B/test"
