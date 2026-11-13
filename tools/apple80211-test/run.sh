#!/bin/bash
# Build and run the tests of the Apple80211 request map (drivers/wifi/hakios.apple80211.c, the source the kernel is
# built from) on this machine, under the address and undefined-behaviour sanitizers, behind a fake station.
set -e
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
B=${APPLE80211_TEST_BUILD:-${TMPDIR:-/tmp}/apple80211-test-build}
rm -rf "$B"
mkdir -p "$B/inc/sys"
FLAGS="-std=gnu11 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wno-format -D_GNU_SOURCE -DKERNEL -fsanitize=address,undefined -fno-sanitize-recover=all -pthread"
# (-Wno-format: uint64_t is unsigned long here, unsigned long long in the kernel)
# the shim's kernel headers and the kernel's own <sys/hakios.sta.h> (xnu/files, where apply.sh takes it from)
ln -s "$ROOT/xnu/files/bsd/sys/hakios.sta.h" "$B/inc/sys/"
${CC:-clang} $FLAGS -I"$HERE/shim" -I"$B/inc" -o "$B/test" "$HERE/test.c" "$ROOT/drivers/wifi/hakios.apple80211.c"
"$B/test" > "$B/out.txt" || { grep -v "^apple80211: " "$B/out.txt"; exit 1; }
grep -v "^apple80211: " "$B/out.txt"
