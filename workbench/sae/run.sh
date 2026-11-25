#!/bin/bash
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
B=$(mktemp -d)
trap 'rm -rf "$B"' EXIT
python3 "$HERE/reference.py" "${1:-1}" > "$B/cases.inc"
${CC:-clang} -std=gnu11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all \
    -I"$ROOT/kernel/lx/src" -I"$HERE" -I"$B" "$HERE/test_sae.c" "$ROOT/kernel/lx/src/lx_sae_crypto.c" "$ROOT/kernel/lx/src/lx_sae_math.c" -o "$B/test"
"$B/test"
python3 "$HERE/station.py"
