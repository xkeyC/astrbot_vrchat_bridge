#!/bin/sh
# Calls the bridge (on the server, as root: the token is vrcbot's):
#   sudo [BRIDGE=http://host:port] sh bapi.sh METHOD PATH [JSON] [OUTFILE]
T=$(cat /home/vrcbot/.config/vrc-bridge/token)
M=$1; P=$2; J=${3:-}; O=${4:-}
if [ -n "$O" ]; then
  curl -s -m 60 -X "$M" -H "Authorization: Bearer $T" "${BRIDGE:-http://10.88.0.2:6120}$P" -o "$O"; chmod 644 "$O"; ls -la "$O" | cut -c1-60
elif [ -n "$J" ]; then
  curl -s -m 60 -X "$M" -H "Authorization: Bearer $T" -H "Content-Type: application/json" -d "$J" "${BRIDGE:-http://10.88.0.2:6120}$P"; echo
else
  curl -s -m 60 -X "$M" -H "Authorization: Bearer $T" "${BRIDGE:-http://10.88.0.2:6120}$P"; echo
fi
