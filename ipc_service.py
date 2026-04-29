import asyncio
import json
from typing import Dict, Any, Optional, Set

import websockets

IPC_WS_URL = "ws://192.168.144.50:8765"

ipc_ws: Optional[websockets.WebSocketClientProtocol] = None
ipc_lock = asyncio.Lock()

amr_clients: Set[Any] = set()
stations: Dict[str, Any] = {}
current_amr_pose: Dict[str, Any] = {}


# ---------------- SEND ----------------
async def ipc_send(msg: Dict[str, Any]):
    global ipc_ws

    async with ipc_lock:
        if not ipc_ws:
            return
        await ipc_ws.send(json.dumps(msg))


# ---------------- BROADCAST ----------------
async def broadcast_amr(msg):
    dead = []
    text = json.dumps(msg)

    for ws in amr_clients:
        try:
            await ws.send_text(text)
        except:
            dead.append(ws)

    for ws in dead:
        amr_clients.discard(ws)


# ---------------- CONNECTOR ----------------
async def ipc_connector():
    global ipc_ws

    while True:
        try:
            async with websockets.connect(IPC_WS_URL) as ws:
                ipc_ws = ws
                print("[IPC] connected")

                async for raw in ws:
                    msg = json.loads(raw)
                    await broadcast_amr(msg)

        except Exception as e:
            print("[IPC ERROR]", e)

        ipc_ws = None
        await asyncio.sleep(3)