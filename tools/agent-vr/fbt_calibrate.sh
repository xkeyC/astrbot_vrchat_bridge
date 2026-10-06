#!/bin/sh
# Calibrates full body: Quick Menu on the raised left hand, the right
# hand's ray on 校准, then standing straight, both triggers. Run with sudo.
b() { sh "$(dirname "$0")/bapi.sh" "$@"; }
q() { b "$@" >/dev/null; }
q POST /v1/vr/head '{"bend":0}'
q POST /v1/vr/trackers '{"on":true,"shift":{}}'
q GET "/v1/screenshot?width=320&pitch=-20" "" /tmp/x.jpg
q POST /v1/vr/hand '{"hand":"left","offset":[-0.1,-0.3,0.3],"turn":[0,70,0]}'
q POST /v1/vr/hand '{"hand":"right","offset":[0.095,-0.121,0.244],"turn":[24.3,4.3,0]}'
q POST /v1/vr/input '{"name":"QuickMenuToggleLeft"}'
sleep 1.5
b GET "/v1/screenshot?width=0" "" /tmp/cal_menu.jpg
[ "${1:-}" = look ] && exit 0
q POST /v1/vr/hand '{"hand":"right","offset":[0.095,-0.121,0.244],"turn":[24.3,4.3,0],"trigger":1,"press_ms":150}'
sleep 1.5
b GET "/v1/screenshot?width=640&pitch=-20" "" /tmp/cal_mode.jpg
[ "${1:-}" = mode ] && exit 0
q GET "/v1/screenshot?width=320&pitch=0" "" /tmp/x.jpg
q POST /v1/vr/hand '{"hand":"left","offset":[-0.25,-0.85,0.0],"turn":[0,-80,0]}'
q POST /v1/vr/hand '{"hand":"right","offset":[0.25,-0.85,0.0],"turn":[0,-80,0]}'
sleep 1
q POST /v1/vr/hand '{"hand":"left","offset":[-0.25,-0.85,0.0],"turn":[0,-80,0],"trigger":1}'
q POST /v1/vr/hand '{"hand":"right","offset":[0.25,-0.85,0.0],"turn":[0,-80,0],"trigger":1,"press_ms":400}'
sleep 1.5
q POST /v1/vr/hand '{"release":true}'
echo CALIBRATED
