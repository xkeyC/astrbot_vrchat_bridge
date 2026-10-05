#!/usr/bin/env python3
"""Measures how long audio written into vrc_mic_in takes to reach VRChat's
voice (its Voice parameter, read over OSCQuery), push-to-talk held.

Run as the bot user (where the bridge runs) while the game is in a world:
  mic_latency.py [rounds]
"""
import json
import os
import math
import socket
import struct
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # beside vrc_bridge.py
from vrc_bridge import LOG_DIR, RE_OSCQUERY, osc_message  # noqa: E402


def oscquery_port() -> int:
    import glob

    log = max(glob.glob(os.path.expanduser(LOG_DIR) + "/output_log_*.txt"), key=os.path.getmtime)
    return int(RE_OSCQUERY.findall(open(log, encoding="utf-8", errors="replace").read())[-1])


def main() -> None:
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    port = oscquery_port()
    url = f"http://127.0.0.1:{port}/avatar/parameters/Voice"
    osc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 1 s of a loud 300 Hz tone, 48 kHz mono s16
    tone = b"".join(struct.pack("<h", int(12000 * math.sin(2 * math.pi * 300 * i / 48000)))
                    for i in range(48000))
    player = subprocess.Popen(["pacat", "--playback", "--device=vrc_mic_in", "--format=s16le",
                               "--rate=48000", "--channels=1", "--latency-msec=60", "--raw"],
                              stdin=subprocess.PIPE)
    osc.sendto(osc_message("/input/Voice", 1), ("127.0.0.1", 9000))
    time.sleep(1.0)
    results = []
    for _ in range(rounds):
        while json.load(urllib.request.urlopen(url, timeout=1))["VALUE"][0] > 0:
            time.sleep(0.05)
        time.sleep(0.5)
        t0 = time.monotonic()
        player.stdin.write(tone)
        player.stdin.flush()
        while True:
            if json.load(urllib.request.urlopen(url, timeout=1))["VALUE"][0] > 0:
                results.append(time.monotonic() - t0)
                break
            if time.monotonic() - t0 > 3:
                results.append(float("nan"))
                break
            time.sleep(0.01)
        time.sleep(1.5)
    osc.sendto(osc_message("/input/Voice", 0), ("127.0.0.1", 9000))
    player.stdin.close()
    player.wait()
    print("mic -> VRChat Voice latency (s):", [round(r, 3) for r in results])


if __name__ == "__main__":
    main()
