#!/bin/bash
# Build and run the tests of the WPA2 handshake's arithmetic (kernel/lx/src/lx_wpa.c) on this
# machine, under the address and undefined-behaviour sanitizers. Needs Python 3 with the
# "cryptography" package: the access point the handshakes are held with is reference.py.
# usage: workbench/wpa/run.sh [SEED]      another seed, other made-up networks
set -e
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
B=$(mktemp -d)
trap 'rm -rf "$B"' EXIT
"$HERE/reference.py" "${1:-1}" > "$B/cases.inc"
${CC:-cc} -std=gnu11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all \
	-I"$B" -I"$ROOT/kernel/lx/src" -o "$B/test" "$HERE/test_wpa.c" "$ROOT/kernel/lx/src/lx_sae_crypto.c"
"$B/test"
python3 "$HERE/station.py"
python3 "$HERE/pmf_station.py"
