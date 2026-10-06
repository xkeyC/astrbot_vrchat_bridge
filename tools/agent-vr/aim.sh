#!/bin/sh
# Points the right hand's ray at pixel U V of the left eye (1920 px, 100 deg,
# head pitched -20): the hand on the line of sight D m out (0.25: the Quick
# Menu; 0.65: the big menu), turned along it plus VRChat's pointer offset
# (measured: pitch +28, yaw -3.2), then grabs a frame; "click" pulls the
# trigger. Runs on the PC (Git Bash, Windows OpenSSH); see
# docs/full-vr/agent-vr-use.md.
#   HOST=user@host [BAPI=path] [D=0.65] [OUT=dir] sh aim.sh U V [click] [name]
# Leaves name.jpg (the eye) and namec.png (a crop of the middle) in OUT;
# DRY=1 only prints the hand's JSON.
HOST=${HOST:-}
# Where bapi.sh is on the server.
BAPI=${BAPI:-agent-vr/bapi.sh}
SSH=${SSH:-/c/Windows/System32/OpenSSH/ssh.exe}
SCP=${SCP:-/c/Windows/System32/OpenSSH/scp.exe}
U=$1; V=$2; CLICK=${3:-}; N=${4:-aim}
J=$(python -c "
import math, os
f=960/math.tan(math.radians(50)); u,v=$U,$V
d=[(u-960)/f, -(v-960)/f, 1.0]; n=math.sqrt(sum(x*x for x in d)); r,up,ah=[x/n for x in d]
p=math.radians(-20); up2=up*math.cos(p)+ah*math.sin(p); ah2=ah*math.cos(p)-up*math.sin(p)
yaw=math.degrees(math.atan2(r,ah2)); pit=math.degrees(math.asin(up2))
D=float(os.environ.get('D','0.25')); off=[-0.0315+D*r, D*up2, D*ah2]
print('{\"hand\":\"right\",\"offset\":[%.3f,%.3f,%.3f],\"turn\":[%.2f,%.2f,0]' % (off[0],off[1],off[2],yaw-3.2,pit+28))
")
[ "$CLICK" = click ] && J="$J,\"trigger\":1,\"press_ms\":150"
J="$J}"
echo "$J"
[ -n "${DRY:-}" ] && exit 0
[ -n "$HOST" ] || { echo "set HOST=user@server" >&2; exit 1; }
$SSH -o BatchMode=yes $HOST "b() { sudo -n sh $BAPI \"\$@\" >/dev/null; }; b POST /v1/vr/hand '$J'; sleep 1; b GET '/v1/screenshot?width=0' '' /tmp/$N.jpg"
cd ${OUT:-.} && $SCP -q $HOST:/tmp/$N.jpg $N.jpg && python -c "
from PIL import Image
Image.open('$N.jpg').crop((400,200,1600,1000)).save('${N}c.png')"
