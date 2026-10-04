#!/bin/sh
# Installs or updates the bridge for the bot user (run as root from the directory holding
# vrc_bridge.py and vrc-bridge.service). Creates the token on first install.
set -e
U=${VRC_USER:-vrcbot}
H=$(getent passwd "$U" | cut -d: -f6)
if ! python3 -c 'import aiohttp, numpy' 2>/dev/null; then
  if command -v pacman >/dev/null; then
    pacman -S --needed --noconfirm python-aiohttp python-numpy
  else
    echo "install aiohttp and numpy for the system python3 first" >&2
    exit 1
  fi
fi
install -d -o "$U" -g "$U" -m 755 "$H/vrc-bridge"
for f in vrc_bridge.py vrc_api.py social.py follow.py tracking.py sightings.py drive.py nav.py scene.py odometry.py selfview.py; do install -o "$U" -g "$U" -m 644 "$f" "$H/vrc-bridge/$f"; done
# Made as the user: a missing ~/.config must not end up root's.
runuser -u "$U" -- mkdir -p "$H/.config/systemd/user"
install -d -o "$U" -g "$U" -m 700 "$H/.config/vrc-bridge"
if [ ! -s "$H/.config/vrc-bridge/token" ]; then
  umask 077
  head -c 48 /dev/urandom | base64 | tr -dc 'A-Za-z0-9' | head -c 40 > "$H/.config/vrc-bridge/token"
  chown "$U:$U" "$H/.config/vrc-bridge/token"
fi
if [ ! -e "$H/.config/vrc-bridge/env" ]; then
  # This machine's bridge arguments, kept across updates.
  cat > "$H/.config/vrc-bridge/env" <<'EOF'
# Arguments of vrc_bridge.py on this machine (vrc_bridge.py --help), e.g.
# VRC_BRIDGE_ARGS=--listen 0.0.0.0 --ocr-url http://127.0.0.1:17890/v1/ocr/lines --depth-url http://127.0.0.1:17890/v1/depth
VRC_BRIDGE_ARGS=
EOF
  chown "$U:$U" "$H/.config/vrc-bridge/env"
fi
install -o "$U" -g "$U" -m 644 vrc-bridge.service "$H/.config/systemd/user/vrc-bridge.service"
systemctl --user -M "$U@" daemon-reload
systemctl --user -M "$U@" enable vrc-bridge.service >/dev/null 2>&1
systemctl --user -M "$U@" restart vrc-bridge.service
sleep 2
systemctl --user -M "$U@" is-active vrc-bridge.service
