#!/bin/sh
# Builds xrizer (OpenVR on OpenXR) with our patches, as the bot user.
#   build_xrizer.sh [VR_ROOT]
# Needs: rustup (stable), clang (bindgen), git. Pins the commit in third_party/xrizer/README.md.
set -e
VR_ROOT=${1:-${VR_ROOT:-$HOME/vr}}
HERE=$(cd "$(dirname "$0")/../.." && pwd)
COMMIT=$(sed -n 's/^Base commit: `\([0-9a-f]*\)`.*/\1/p' "$HERE/third_party/xrizer/README.md")
mkdir -p "$VR_ROOT"
[ -d "$VR_ROOT/xrizer/.git" ] || git clone https://github.com/Supreeeme/xrizer.git "$VR_ROOT/xrizer"
cd "$VR_ROOT/xrizer"
git fetch -q origin && git checkout -q "$COMMIT"
git checkout -q -- .
for p in "$HERE"/third_party/xrizer/patches/*.patch; do git apply "$p"; done
cargo xbuild --release
ls -la target/release/bin/linux64/vrclient.so
