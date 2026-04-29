import asyncio
import time
import json
from typing import Dict, Any, Optional, Set, List, Tuple

from pymavlink import mavutil

# ---------------- CONFIG ----------------
CUBE_IP = "192.168.144.12"
CUBE_PORT = 19856
MAV_CONN_STR = f"udpout:{CUBE_IP}:{CUBE_PORT}"

GCS_SYS_ID = 255
MAV_CMD_COMPONENT_ARM_DISARM = 400
GLOBAL_POSITION_INT_ID = 33
MAV_CMD_REQUEST_MESSAGE = 512

# ---------------- STATE ----------------
mav_master: Optional[mavutil.mavlink_connection] = None
mavlink_lock = asyncio.Lock()

last_telemetry_by_sysid: Dict[int, Dict[str, Any]] = {}
valid_sysids_with_heartbeat: Set[int] = set()
uav_clients: Set[Any] = set()

missions: List[Dict[str, Any]] = []
notifications: List[Dict[str, Any]] = []
last_mavlink_addr_by_sysid: Dict[int, Tuple[str, int]] = {}  # kept for compatibility, unused now

last_mavlink_rx_ms = 0
mavlink_msg_count = 0


# ---------------- INIT ----------------
def init_mavlink():
    global mav_master
    if mav_master:
        return

    master = mavutil.mavlink_connection(
        MAV_CONN_STR,
        source_system=GCS_SYS_ID
    )

    master.mav.heartbeat_send(0, 0, 0, 0, 0)
    mav_master = master
    print("[MAV] Initialized")


def get_status():
    import time

    MAVLINK_TIMEOUT_MS = 5000

    now_ms = int(time.time() * 1000)

    # these must already exist in your file
    global last_mavlink_rx_ms, mavlink_msg_count, valid_sysids_with_heartbeat

    age_ms = (
        now_ms - last_mavlink_rx_ms
        if last_mavlink_rx_ms > 0 else None
    )

    connected = (
        age_ms is not None and age_ms < MAVLINK_TIMEOUT_MS
    )

    return {
        "connected": connected,
        "lastMessageAgeMs": age_ms,
        "totalMessages": mavlink_msg_count,
        "sysids": sorted(valid_sysids_with_heartbeat),
    }


# ---------------- SEND ----------------
async def send_command_long(target_sysid, command, params=None):
    global mav_master

    if not mav_master:
        return

    if params is None:
        params = [0.0] * 7

    async with mavlink_lock:
        mav_master.mav.command_long_send(
            target_sysid,
            1,
            command,
            0,
            *params
        )


# ---------------- TELEMETRY ----------------
async def mavlink_reader_task():
    global mav_master, last_mavlink_rx_ms, mavlink_msg_count

    loop = asyncio.get_running_loop()

    while True:
        if not mav_master:
            await asyncio.sleep(1)
            continue

        msg = await loop.run_in_executor(
            None,
            lambda: mav_master.recv_match(blocking=True, timeout=1)
        )

        if not msg:
            continue

        sysid = msg.get_srcSystem()
        mtype = msg.get_type()

        t = last_telemetry_by_sysid.get(sysid, {"sysid": sysid})

        if mtype == "HEARTBEAT":
            valid_sysids_with_heartbeat.add(sysid)

        elif mtype == "GLOBAL_POSITION_INT":
            t["position"] = {
                "lat": msg.lat / 1e7,
                "lng": msg.lon / 1e7
            }

        last_telemetry_by_sysid[sysid] = t
        now_ms = int(time.time() * 1000)
        last_mavlink_rx_ms = now_ms
        mavlink_msg_count += 1

        await broadcast_uav({
            "type": "telemetry",
            **t
        })


# ---------------- KEEPALIVE ----------------
async def keepalive_task():
    while True:
        if mav_master:
            await send_command_long(
                1,
                MAV_CMD_REQUEST_MESSAGE,
                [GLOBAL_POSITION_INT_ID, 1, 0, 0, 0, 0, 0]
            )
        await asyncio.sleep(2)


# ---------------- WS ----------------
async def broadcast_uav(msg: Dict[str, Any]):
    dead = []
    text = json.dumps(msg)

    for ws in uav_clients:
        try:
            await ws.send_text(text)
        except Exception:
            dead.append(ws)

    for ws in dead:
        uav_clients.discard(ws)


async def handle_uav_command(msg):
    cmd = msg.get("cmd")

    if cmd == "ARM":
        await send_command_long(1, MAV_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0])

    elif cmd == "DISARM":
        await send_command_long(1, MAV_CMD_COMPONENT_ARM_DISARM, [0, 21196, 0, 0, 0, 0, 0])

