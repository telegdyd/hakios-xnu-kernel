#!/bin/bash
# P0c: build the Wi-Fi boot volume from this tree's userland sources.
# Run in the Linux build checkout, after build-wifi.sh has staged fw-wifi/.
# Needs the configured Darwin extraction/SDK, Mesa archives and host Mesa build.
# Does not contact the phone. No dependency on a previously assembled gate.apfs.
set -euo pipefail
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
OUT="${WIFI_GATE_BUILD:-$ROOT/xnu-deps/build/wifi-gate}"
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
cd "$ROOT"
source ./xnu-env.sh >/dev/null 2>&1

# userland/build.sh otherwise falls back to prebuilt programs when ld64 is absent.
# P0 specifically requires a source build, so fail before rebuilding anything.
for tool in xnu-clang ld64 make python3; do
  command -v "$tool" >/dev/null || { echo "build-wifi-gate: missing $tool" >&2; exit 1; }
done
SIGN="${LDID:-$HOME/theos/toolchain/linux/iphone/bin/ldid}"
test -x "$SIGN" || { echo "build-wifi-gate: missing signing tool $SIGN" >&2; exit 1; }
test -s sysimage/rootfs.slt || {
  echo "build-wifi-gate: missing sysimage/rootfs.slt; FLASHBOOT needs the full Darwin volume listing" >&2
  exit 1
}
for f in amss.bin amss20.bin m3.bin regdb.bin bdwlang.b0e; do
  test -s "fw-wifi/$f" || { echo "build-wifi-gate: missing fw-wifi/$f" >&2; exit 1; }
done
test ! -e fw-wifi/network || { echo "build-wifi-gate: remove fw-wifi/network; wifid owns saved networks" >&2; exit 1; }

# Build the CoreAnimation Metal implementation used by the installed image.
# MESA_HOST may select an existing host Mesa build; no hard-coded ~/vayu path.
metal_args=(MESA=1 "HAKIOS=$ROOT")
if [ -n "${MESA_HOST:-}" ]; then metal_args+=("MESA_HOST=$MESA_HOST"); fi
make -C hakios-metal out/ios/libhakios-metal.a "${metal_args[@]}" > "$OUT/metal-build.log" 2>&1
# build-wifi.sh's EXTRA is boot words; userland's EXTRA means files to copy.
# Pass the checked signer explicitly so keychain helpers do not silently stay unsigned.
EXTRA= LDID="$SIGN" PREBUILT= METAL_PLUGIN=ca HAKIOS_METAL="$ROOT/hakios-metal" FLASHBOOT=1 ROOT_ONLY=1 REST=1 GL=1 \
  WIFID=1 bash userland/build.sh > "$OUT/userland-build.log" 2>&1

# The GPU power daemon is part of the installed configuration and must remain
# in the freshly generated volume alongside the new Wi-Fi firmware.
xnu-clang -arch arm64 -isysroot "$THEOS/sdks/iPhoneOS16.5.sdk" -O2 -Wall -Wextra -Werror \
  -o "$OUT/gpu-power-policy" userland/src/gpu_power_policy.c
"$SIGN" -S "$OUT/gpu-power-policy"
xnu-clang -arch arm64 -isysroot "$THEOS/sdks/iPhoneOS16.5.sdk" -O2 -Wall -Wextra -Werror \
  -framework CoreFoundation -o "$OUT/wifi_start" userland/src/wifi_start.c
"$SIGN" -Suserland/src/wifi_start.entitlements "$OUT/wifi_start"
# The date from the PMIC clock's counter and a kept offset (WIFI-PLAN.md I8).
xnu-clang -arch arm64 -isysroot "$THEOS/sdks/iPhoneOS16.5.sdk" -O2 -Wall -Wextra -Werror   -o "$OUT/clock_keep" userland/src/clock_keep.c
"$SIGN" -S "$OUT/clock_keep"
STAGE=$(mktemp -d "$OUT/root.XXXXXX")
cp -a userland/root/. "$STAGE/"
# The flash-boot launchd creates a user/501 namespace as well as System.
# Keep global network/keychain services in System so both system clients and
# the user namespace can reach them (children can look up parent services).
python3 - "$STAGE/LaunchDaemons" <<'PY'
import plistlib
import sys
from pathlib import Path
for label in ('com.apple.securityd', 'com.apple.configd', 'com.apple.mDNSResponder', 'com.apple.wifid'):
    p = Path(sys.argv[1]) / (label + '.plist')
    job = plistlib.loads(p.read_bytes())
    job['LimitLoadToSessionType'] = 'System'
    p.write_bytes(plistlib.dumps(job, fmt=plistlib.FMT_BINARY))
p = Path(sys.argv[1]) / 'com.hakios.wifi.plist'
job = plistlib.loads(p.read_bytes())
job['ProgramArguments'] = ['/redwood/bin/wifi_start']
job['LimitLoadToSessionType'] = 'System'
job['KeepAlive'] = True   # wifi_start watches for the whole session; launchd restarts it if it ever exits
p.write_bytes(plistlib.dumps(job, fmt=plistlib.FMT_BINARY))
PY
"$SIGN" -S "$STAGE/lib/HakiosMetal.bundle/HakiosMetal"
install -m 0755 "$OUT/gpu-power-policy" "$STAGE/bin/gpu-power-policy"
install -m 0755 "$OUT/wifi_start" "$STAGE/bin/wifi_start"
install -m 0644 userland/com.hakios.gpu-power.plist "$STAGE/LaunchDaemons/"
install -m 0755 "$OUT/clock_keep" "$STAGE/bin/clock_keep"
install -m 0644 userland/com.hakios.clock.plist "$STAGE/LaunchDaemons/"
for f in amss.bin amss20.bin m3.bin regdb.bin bdwlang.b0e; do
  cmp "fw-wifi/$f" "$STAGE/fw/$f"
done
test ! -e "$STAGE/fw/network"
for f in sbin/init lib/HakiosMetal.bundle/HakiosMetal lib/libOSMesa.8.dylib lib/libhakios.gl.dylib \
         bin/net_test bin/sweetd bin/wifi_start bin/clock_keep LaunchDaemons/com.hakios.wifi.plist LaunchDaemons/com.hakios.clock.plist \
         LaunchDaemons/com.apple.wifid.plist LaunchDaemons/com.apple.securityd.plist \
         LaunchDaemons/com.apple.configd.plist; do
  test -s "$STAGE/$f" || { echo "build-wifi-gate: missing staged $f" >&2; exit 1; }
done
python3 tools/apfs/mkapfs.py -z --plain fw -o "$OUT/gate.apfs.new" "redwood=$STAGE" > "$OUT/apfs.log" 2>&1
bash tools/apfs/check-image.sh "$OUT/gate.apfs.new" "redwood=$STAGE" > "$OUT/apfs-check.log" 2>&1
mv "$OUT/gate.apfs.new" "$OUT/gate.apfs"
# Keep the exact staged directory and firmware/input hashes for review.
printf '%s\n' "$STAGE" > "$OUT/staged-root.txt"
sha256sum tools/build-wifi-gate.sh userland/build.sh userland/src/init.c \
  userland/src/HakiosMetalCA.m userland/src/gpu_power_policy.c \
  userland/src/wifi_start.c userland/src/wifi_start.entitlements \
  userland/src/clock_keep.c userland/com.hakios.clock.plist   userland/com.hakios.gpu-power.plist sysimage/rootfs.slt fw-wifi/* "$OUT/gate.apfs" > "$OUT/SHA256SUMS"
echo "build-wifi-gate: $OUT/gate.apfs (sources rebuilt; APFS check passed)"
