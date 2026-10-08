#!/bin/sh
# The direction finder's calibration sweep (docs/full-vr/speaker.md, 4.2):
# a source stands still (a friend reading aloud, or a world's speaker) and
# the bot turns a full circle STEP degrees at a time, HOLD seconds a step,
# while the speaker tracker records. Run with sudo on the server, then copy
# the recording directory it prints and run doa_sweep_report.py on it.
#   sudo [STEP=1] [HOLD=1] sh doa_sweep.sh
b() { sh "$(dirname "$0")/bapi.sh" "$@"; }
STEP=${STEP:-1}
HOLD=${HOLD:-1}
N=$((360 / STEP))
# Recorded as long as a recording may run (600 s), and stopped once the
# last step has been held (or when the sweep is broken off).
LONGEST=$(awk "BEGIN { print 2 + $N * ($HOLD + 0.3) }")
if awk "BEGIN { exit !($LONGEST > 590) }"; then
  echo "warning: $N steps of ${HOLD} s may outlast the longest recording (600 s); the end may be missing" >&2
fi
# A follow would turn the head too: stopped for the sweep (start it again after).
b POST /v1/follow '{"stop":true}' >/dev/null
b POST /v1/speakers/record '{"seconds": 600}'
trap 'b POST /v1/speakers/record "{\"stop\": true}" >/dev/null; exit 130' INT TERM
sleep 2
i=0
while [ $i -lt $N ]; do
  b POST /v1/step "{\"turn\": $STEP}" >/dev/null
  sleep "$HOLD"
  i=$((i + 1))
done
trap - INT TERM
b POST /v1/speakers/record '{"stop": true}'
echo "swept $N steps of $STEP degrees; the recording stopped after the last"
