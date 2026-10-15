#!/usr/bin/env python3
"""
Minimal WebSocket test from your PC.
Connects to the server and sends a test frame.

Setup:
    pip install websockets

Usage:
    python test_ws_simple.py ws://localhost:8000/ws
    python test_ws_simple.py ws://192.168.0.105:8000/ws
"""

import asyncio
import sys

try:
    import websockets
except ImportError:
    print("ERROR: websockets not installed")
    print("  pip install websockets")
    sys.exit(1)


async def test(url: str):
    print(f"\nConnecting to {url} ...")
    try:
        async with websockets.connect(url, open_timeout=5) as ws:
            print("✓ Connected!\n")
            print("Sending test frame (100 bytes)...")
            await ws.send(b'\xff\xd8' + b'\x00' * 96 + b'\xff\xd9')
            
            print("Waiting for server reply...")
            reply = await ws.recv()
            print(f"✓ Got reply: {reply}\n")
            
            print("=" * 50)
            print("SUCCESS: Server WebSocket endpoint is working")
            print("=" * 50)
            
    except Exception as e:
        print(f"\n✗ Connection failed: {type(e).__name__}")
        print(f"  {e}\n")
        print("=" * 50)
        print("FAILED: Check the server console for details")
        print("=" * 50)
        sys.exit(1)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    
    asyncio.run(test(sys.argv[1]))
