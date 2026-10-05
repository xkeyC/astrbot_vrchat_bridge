#!/bin/sh
# Steam launch options of VRChat for the bot user:  /usr/local/lib/vrc/vrc-launch.sh %command%
# ~/.config/vrc-mode picks the mode: "vr" (Monado + xrizer under $VR_ROOT), else desktop.
VR_ROOT=${VR_ROOT:-$HOME/vr}
SCREEN="-screen-width 1280 -screen-height 720 -screen-fullscreen 1"
if [ "$(cat "$HOME/.config/vrc-mode" 2>/dev/null)" = vr ]; then
  export XR_RUNTIME_JSON=$VR_ROOT/prefix/share/openxr/1/openxr_monado.json
  export VR_OVERRIDE=$VR_ROOT/xrizer/target/release
  # Steam's pressure-vessel container sees $HOME only: let it reach Monado's socket and the runtime.
  export PRESSURE_VESSEL_FILESYSTEMS_RW="$XDG_RUNTIME_DIR/monado_comp_ipc:$VR_ROOT"
  exec "$@" $SCREEN
fi
export DXVK_FRAME_RATE=20
exec "$@" --no-vr --fps=20 $SCREEN
