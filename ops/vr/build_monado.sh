#!/bin/sh
# Builds Monado with our null-compositor eye tap, as the bot user.
#   build_monado.sh <monado source dir or .zip> [VR_ROOT]
# Needs: cmake ninja eigen vulkan-headers glslang (Arch: pacman -S --needed cmake ninja eigen vulkan-headers glslang).
# Installs to $VR_ROOT/prefix (bin/monado-service, share/openxr/1/openxr_monado.json).
set -e
SRC=$1
VR_ROOT=${2:-${VR_ROOT:-$HOME/vr}}
HERE=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$VR_ROOT"
if [ "${SRC%.zip}" != "$SRC" ]; then
  rm -rf "$VR_ROOT/monado" "$VR_ROOT/monado-main"
  (cd "$VR_ROOT" && unzip -q "$SRC" && mv monado-main monado)
  SRC=$VR_ROOT/monado
fi
cd "$SRC"
for p in "$HERE"/third_party/monado/patches/*.patch; do
  patch -p1 -N --dry-run < "$p" >/dev/null 2>&1 && patch -p1 -N < "$p"
done
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX="$VR_ROOT/prefix"
ninja -C build install
