#!/bin/sh
# Wires the bot user to the headless VR runtime (run as root).
#   install_vr.sh <bot user> <VR_ROOT>
# Then set VRChat's Steam launch options to: /usr/local/lib/vrc/vrc-launch.sh %command%
# (Steam UI, or localconfig.vdf with Steam stopped) and `echo vr > ~/.config/vrc-mode`.
set -e
U=$1; VR_ROOT=$2
H=$(getent passwd "$U" | cut -d: -f6)
HERE=$(cd "$(dirname "$0")" && pwd)
install -D -m 755 "$HERE/vrc-launch.sh" /usr/local/lib/vrc/vrc-launch.sh
install -D -o "$U" -g "$U" -m 644 "$HERE/config_v0.json" "$H/.config/monado/config_v0.json"
install -d -o "$U" -g "$U" "$H/.config/openvr" "$H/.config/systemd/user"
sed "s#@VR_ROOT@#$VR_ROOT#; s#@HOME@#$H#g" "$HERE/openvrpaths.vrpath.in" > "$H/.config/openvr/openvrpaths.vrpath"
sed "s#^Environment=VR_ROOT=.*#Environment=VR_ROOT=$VR_ROOT#" "$HERE/vrc-monado.service" > "$H/.config/systemd/user/vrc-monado.service"
chown "$U:$U" "$H/.config/openvr/openvrpaths.vrpath" "$H/.config/systemd/user/vrc-monado.service"
echo "done: systemctl --user -M $U@ daemon-reload && systemctl --user -M $U@ start vrc-monado"
