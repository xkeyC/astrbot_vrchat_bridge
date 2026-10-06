#!/bin/sh
# One cold restart of VRChat and the bridge, then watches the automatic
# full body calibration. Run with sudo next to bapi.sh:  sh fbt_cold.sh N
N=${1:-1}
U="systemctl --user -M vrcbot@"
b() { sh "$(dirname "$0")/bapi.sh" "$@"; }
echo "== cycle $N $(date +%T)"
$U stop vrc-bridge
sh /usr/local/lib/vrc/stop_game.sh | tr '\n' ' '; echo
sh /usr/local/lib/vrc/start_game.sh
[ -n "${WAIT_WORLD:-}" ] && sh "$WAIT_WORLD"  # optional: waits until the eyes show a world
T0=$(date +%s); S=$(date '+%Y-%m-%d %H:%M:%S')
$U start vrc-bridge
echo "bridge started $(date +%T)"
for i in $(seq 1 36); do
  sleep 5
  TT=$(b GET /v1/vr/calibrate 2>/dev/null | grep -o '"tracking_type":[0-9a-z]*')
  echo "+$(( $(date +%s) - T0 ))s $TT"
  journalctl _UID=1002 --since "$S" --no-pager | grep -q '"ok":true\|"ok":false\|calibrate: .*: ' && \
    journalctl _UID=1002 --since "$S" --no-pager | grep -q 'calibrate: {' && break
done
journalctl _UID=1002 --since "$S" --no-pager | grep 'calibrate' | sed 's/.*calibrate: /calibrate: /' | cut -c1-400
b GET /v1/vr/calibrate
b GET /v1/status | grep -o '"instance":"[^"]*"\|"players":\[[^]]*\]' | cut -c1-200
# Seen, not only reported: bent over, the feet standing, the left lifted,
# the right stepped out.
q() { b "$@" >/dev/null; }
q POST /v1/vr/head '{"bend":35}'; sleep 1.5
q GET "/v1/screenshot?width=640&pitch=-80" "" /tmp/fbtcold${N}a.jpg
q POST /v1/vr/trackers '{"shift":{"left_foot":[0,0.3,0.35]}}'; sleep 1.5
q GET "/v1/screenshot?width=640&pitch=-80" "" /tmp/fbtcold${N}b.jpg
q POST /v1/vr/trackers '{"shift":{"right_foot":[0.35,0.05,0]}}'; sleep 1.5
q GET "/v1/screenshot?width=640&pitch=-80" "" /tmp/fbtcold${N}c.jpg
q POST /v1/vr/trackers '{"shift":{}}'; q POST /v1/vr/head '{"bend":0}'
echo "shots /tmp/fbtcold${N}[abc].jpg"
