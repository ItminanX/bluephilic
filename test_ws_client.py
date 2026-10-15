"""
test_ws_client.py
Tests the backend's /ws WebSocket endpoint from your PC — no ESP32
involved. If this works but the ESP32 can't connect, the problem is on
the ESP32 side. If this fails too, the problem is the server/deploy.

Setup (once):
    pip install websockets

Usage:
    python test_ws_client.py wss://bluephilic.onrender.com/ws C:\\path\\to\\test_object.jpg

For a server running on your own PC:
    python test_ws_client.py ws://localhost:8000/ws C:\\path\\to\\test_object.jpg
"""
import asyncio
import sys
import time

import websockets


async def main(url: str, image_path: str, n: int = 20):
    with open(image_path, "rb") as f:
        jpg = f.read()
    print(f"Image: {len(jpg)} bytes")
    print(f"Connecting to {url} ...")
    print("(a sleeping Render free-tier service can take 30-60s to wake up)")

    t0 = time.time()
    async with websockets.connect(url, open_timeout=90, max_size=None) as ws:
        print(f"Connected in {time.time() - t0:.1f}s")

        # Test 1: one frame, wait for its reply, repeat. Measures the
        # round-trip time to the server.
        t0 = time.time()
        for i in range(n):
            await ws.send(jpg)
            reply = await ws.recv()
            if i == 0:
                print("First reply from server:", reply)
        dt = time.time() - t0
        print(f"\nOne-at-a-time: {n} frames in {dt:.2f}s = {n / dt:.1f} fps "
              f"(~{dt / n * 1000:.0f} ms round trip per frame)")

        # Test 2: send everything without waiting for replies, like the
        # ESP32 does. Shows the throughput you can expect from streaming.
        t0 = time.time()
        for _ in range(n):
            await ws.send(jpg)
        for _ in range(n):
            await ws.recv()
        dt = time.time() - t0
        print(f"Streaming:     {n} frames in {dt:.2f}s = {n / dt:.1f} fps")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    asyncio.run(main(sys.argv[1], sys.argv[2]))
