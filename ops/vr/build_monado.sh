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
# Each patch applied once: one already in is skipped, one that applies
# neither way (an older version of it in the tree) stops the build - start
# from a fresh source (the .zip) then.
for p in "$HERE"/third_party/monado/patches/*.patch; do
  if patch -p1 -R -s -f --dry-run < "$p" >/dev/null 2>&1; then
    echo "already applied: $(basename "$p")"
  else
    patch -p1 -N -s -t --dry-run < "$p" >/dev/null || { echo "patch $(basename "$p") does not apply" >&2; exit 1; }
    patch -p1 -N < "$p"
  fi
done
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX="$VR_ROOT/prefix"
ninja -C build install
