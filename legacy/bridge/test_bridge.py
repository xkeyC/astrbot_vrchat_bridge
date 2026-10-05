#!/usr/bin/env python3
"""Smoke test of a running bridge: status, chatbox, both audio directions, push-to-talk.

  test_bridge.py URL TOKEN SPEECH.wav
Needs aiohttp and ffmpeg. Push-to-talk is checked through the status' VRAM-free
route: the bot's voice must reach VRChat (Voice parameter, read by the caller).
"""
import asyncio
import json
import subprocess
import sys
import time

import aiohttp


async def main(url: str, token: str, wav: str) -> None:
    headers = {"Authorization": f"Bearer {token}"}
    pcm = subprocess.run(["ffmpeg", "-loglevel", "error", "-i", wav, "-f", "s16le", "-ac", "1",
                          "-ar", "48000", "-"], capture_output=True, check=True).stdout
    async with aiohttp.ClientSession(headers=headers) as http:
        async with http.get(f"{url}/v1/status") as r:
            print("status", r.status, json.dumps(await r.json(), ensure_ascii=False))
        async with http.post(f"{url}/v1/chatbox", json={"text": "bridge test 桥接测试"}) as r:
            print("chatbox", r.status, await r.json())
        async with http.get(f"{url}/v1/status", headers={"Authorization": "Bearer wrong"}) as r:
            print("bad token ->", r.status)
        async with http.ws_connect(f"{url.replace('http', 'ws', 1)}/v1/stream") as ws:
            first = await ws.receive(timeout=5)
            print("first text frame:", first.data[:200] if first.type == aiohttp.WSMsgType.TEXT else first.type)
            inbound = 0
            # send the speech at real-time pace in 20 ms chunks while counting inbound audio
            t0 = time.monotonic()
            for i in range(0, len(pcm), 1920):
                await ws.send_bytes(pcm[i:i + 1920])
                wait = t0 + (i + 1920) / 96000 - time.monotonic()
                while True:
                    try:
                        msg = await ws.receive(timeout=max(wait, 0.001))
                    except asyncio.TimeoutError:
                        break
                    if msg.type == aiohttp.WSMsgType.BINARY:
                        inbound += len(msg.data)
                    wait = t0 + (i + 1920) / 96000 - time.monotonic()
                    if wait <= 0:
                        break
            elapsed = time.monotonic() - t0
            print(f"sent {len(pcm) / 96000:.1f}s of speech in {elapsed:.1f}s; "
                  f"received {inbound / 96000:.1f}s of game audio")


if __name__ == "__main__":
    asyncio.run(main(*sys.argv[1:4]))
