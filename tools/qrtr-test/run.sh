#!/bin/bash
# Build and run the tests of the QRTR router (drivers/wifi/hakios.qrtr.c, the source the kernel is built from) on
# this machine, under the address and undefined-behaviour sanitizers, behind a fake link.
set -e
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
B=${QRTR_TEST_BUILD:-${TMPDIR:-/tmp}/qrtr-test-build}
rm -rf "$B"
mkdir -p "$B"
FLAGS="-std=gnu11 -O1 -g -Wall -Wextra -Werror -D_GNU_SOURCE -fsanitize=address,undefined -fno-sanitize-recover=all -pthread"
# the shim's <sys/systm.h> and the kernel's own <sys/hakios.qrtr.h> (xnu/files, where apply.sh takes it from)
mkdir -p "$B/inc/sys"
ln -s "$ROOT/xnu/files/bsd/sys/hakios.qrtr.h" "$B/inc/sys/"
${CC:-cc} $FLAGS -I"$HERE/shim" -I"$B/inc" -o "$B/test" "$HERE/test_qrtr.c" "$ROOT/drivers/wifi/hakios.qrtr.c"
"$B/test"
