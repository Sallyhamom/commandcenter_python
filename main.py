import asyncio
import subprocess
import base64
import json
import os
import socket
import struct
import time
from datetime import datetime
from typing import Dict, Any, List, Set, Tuple, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, Depends
from fastapi.responses import JSONResponse, FileResponse, HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
import websockets

RTSP_URL = "rtsp://192.168.144.26:8554/main.264"
HLS_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "hls")
HLS_PLAYLIST = os.path.join(HLS_OUTPUT_DIR, "stream.m3u8")


# ------------------------
# CONFIG
# ------------------------
HTTP_PORT = 4000               # HTTP + WebSocket
UDP_LISTEN_PORT = 14550        # Where drone sends MAVLink2 (same as Node)
UDP_LISTEN_HOST = "0.0.0.0"

UDP_TARGET_HOST = "192.168.144.12"  # Drone/MK32E MAVLink IP
UDP_TARGET_PORT = 19856             # Drone/MK32E MAVLink Port

# GCS identity in MAVLink
GCS_SYS_ID = 255
GCS_COMP_ID = 190
MAV_CMD_REQUEST_MESSAGE = 512   # for testing
GLOBAL_POSITION_INT_ID = 33     # for testing

# MAVLink sequence (0–255)
mav_seq = 0
current_amr_pose = {}

# ------------------------
# GIMBAL STATE (deg)
# ------------------------
gimbal_pitch_deg = 0.0
gimbal_yaw_deg = 0.0

GIMBAL_PITCH_MIN = -90.0   # look straight down
GIMBAL_PITCH_MAX = 30.0    # slight up
GIMBAL_YAW_MIN = -180.0
GIMBAL_YAW_MAX = 180.0

GIMBAL_COMP_ID = 1       # MAV_COMP_ID_GIMBAL; change to 1 to test via FC or 154

# ------------------------
# GLOBAL STATE
# ------------------------
app = FastAPI()

os.makedirs(HLS_OUTPUT_DIR, exist_ok=True)
FRONTEND_DIR = os.path.join(os.path.dirname(__file__), "mission-control-frontend")
if os.path.isdir(FRONTEND_DIR):
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")
app.mount("/fpv", StaticFiles(directory=HLS_OUTPUT_DIR), name="fpv")


if os.path.isdir(FRONTEND_DIR):
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")

# IP / port of the IPC on the AMR:
IPC_WS_URL = "ws://10.72.65.193:8765"   # change to your AMR IPC IP

# ---------- STATE ----------
stations: Dict[str, Any] = {}
current_odom_text: str = "Waiting for odometry..."
map_image_url: str = "/static/my_map.png"
STATIONS_FILE = os.path.join(os.path.dirname(__file__), "stations.json")


# connection from FastAPI → IPC (AMR)
ipc_ws: Optional[websockets.WebSocketClientProtocol] = None
ipc_lock = asyncio.Lock()


missions: List[Dict[str, Any]] = []
notifications: List[Dict[str, Any]] = []
last_telemetry_by_sysid: Dict[int, Dict[str, Any]] = {}
last_mavlink_addr_by_sysid: Dict[int, Tuple[str, int]] = {}

uav_clients: Set[WebSocket] = set()
amr_clients: Set[WebSocket] = set()
udp_transport = None  # type: ignore

# MAVLink link status
last_mavlink_rx_ms: int = 0    # timestamp (ms) of last MAVLink packet
mavlink_msg_count: int = 0     # total number of MAVLink packets seen
MAVLINK_TIMEOUT_MS = 5000      # consider "disconnected" if no packet in 5s


# ------------------------
# TELEMETRY DECODER
# ------------------------

valid_sysids_with_heartbeat: Set[int] = set()
# MAVLink constants we care about
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_MOUNT_ORIENTATION = 265
MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS = 284

MAV_MODE_FLAG_SAFETY_ARMED = 0x80  # from MAVLink spec


def get_current_user(request: Request):
    return "admin"


def _decode_and_update_telemetry(sysid: int,
                                 compid: int,
                                 seq: int,
                                 msgid: int,
                                 payload: bytes,
                                 addr: Tuple[str, int],
                                 received_at_ms: int) -> Dict[str, Any]:
    """Decode some common fields and merge into last_telemetry_by_sysid[sysid]."""
    t = last_telemetry_by_sysid.get(sysid, {
        "sysid": sysid,
        "compid": compid,
    })
    last_mavlink_addr_by_sysid[sysid] = addr
    # always basic fields
    t["compid"] = compid
    t["seq"] = seq
    t["msgid"] = msgid
    t["from"] = {"address": addr[0], "port": addr[1]}
    t["receivedAt"] = received_at_ms

    # HEARTBEAT
    if msgid == MAVLINK_MSG_ID_HEARTBEAT and len(payload) >= 9:
        custom_mode = struct.unpack_from("<I", payload, 0)[0]
        base_mode = payload[4]  # NOTE: for many firmwares: type(0), autopilot(1), base_mode(2)? -> layout can differ
        system_status = payload[5]

        t["customMode"] = custom_mode
        t["baseMode"] = base_mode
        t["systemStatus"] = system_status
        t["armed"] = bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)

        # this sysid is now trusted
        valid_sysids_with_heartbeat.add(sysid)

    # SYS_STATUS – battery
    elif msgid == MAVLINK_MSG_ID_SYS_STATUS and len(payload) >= 31:
        (
            onboard_present,
            onboard_enabled,
            onboard_health,
            load,
            voltage_battery,
            current_battery,
            battery_remaining,
            drop_rate_comm,
            errors_comm,
            errors_count1,
            errors_count2,
            errors_count3,
            errors_count4,
        ) = struct.unpack_from("<IIIHHhBHHHHHH", payload, 0)

        t["batteryVoltage"] = (
            voltage_battery / 1000.0 if voltage_battery != 0xFFFF else None
        )
        t["batteryRemaining"] = (
            battery_remaining if battery_remaining != 255 else None
        )
        t["load"] = load / 10.0

    # ATTITUDE
    elif msgid == MAVLINK_MSG_ID_ATTITUDE and len(payload) >= 28:
        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = \
            struct.unpack_from("<Iffffff", payload, 0)
        t["attitude"] = {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "rollspeed": rollspeed,
            "pitchspeed": pitchspeed,
            "yawspeed": yawspeed,
        }

    # GLOBAL_POSITION_INT
    elif msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT and len(payload) >= 28:
        time_boot_ms, lat, lon, alt, rel_alt, vx, vy, vz, hdg = \
            struct.unpack_from("<IiiiihhhH", payload, 0)
        t["position"] = {
            "lat": lat / 1e7,
            "lng": lon / 1e7,
            "alt": alt / 1000.0,
            "relAlt": rel_alt / 1000.0,
            "vx": vx / 100.0,
            "vy": vy / 100.0,
            "vz": vz / 100.0,
            "heading": None if hdg == 0xFFFF else hdg / 100.0,
        }

    elif msgid == MAVLINK_MSG_ID_MOUNT_ORIENTATION and len(payload) >= 12:
        pitch, roll, yaw = struct.unpack_from("<fff", payload, 0)
        t["gimbal"] = {
            "pitchDeg": pitch,
            "rollDeg": roll,
            "yawDeg": yaw,
        }

    elif msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS and len(payload) >= 48:
        # decode quaternion q = [w, x, y, z]
        q0, q1, q2, q3 = struct.unpack_from("<ffff", payload, 0)

        # convert quaternion → Euler
        import math
        # Yaw-Pitch-Roll convention — adjust if needed
        ysqr = q2 * q2

        t0 = +2.0 * (q0 * q1 + q2 * q3)
        t1 = +1.0 - 2.0 * (q1 * q1 + ysqr)
        roll = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (q0 * q2 - q3 * q1)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.degrees(math.asin(t2))

        t3 = +2.0 * (q0 * q3 + q1 * q2)
        t4 = +1.0 - 2.0 * (ysqr + q3 * q3)
        yaw = math.degrees(math.atan2(t3, t4))

        t["gimbal"] = {
            "rollDeg": roll,
            "pitchDeg": pitch,
            "yawDeg": yaw,
        }

    if msgid in (265, 284):
        print("🎯 Gimbal telemetry msgid", msgid, "sysid", sysid)

    # only keep telemetry for sysids that have sent heartbeat (or are sending one now)
    if sysid in valid_sysids_with_heartbeat or msgid == MAVLINK_MSG_ID_HEARTBEAT:
        last_telemetry_by_sysid[sysid] = t

    return t


# ------------------------
# UTIL: SERVER IP DISCOVERY
# ------------------------


def get_server_ip() -> str:
    candidates = []

    for res in socket.getaddrinfo(
        None, 0, family=socket.AF_INET, type=socket.SOCK_DGRAM
    ):
        _, _, _, _, sockaddr = res
        ip = sockaddr[0]
        if ip.startswith("127.") or ip.startswith("0."):
            continue
        candidates.append(ip)

    for ip in candidates:
        if ip.startswith("192."):
            return ip

    if candidates:
        return candidates[0]

    return "127.0.0.1"


# ------------------------
# CRC-X25 (MAVLink)
# ------------------------
def crc_accumulate(byte: int, crc: int) -> int:
    tmp = byte ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    new_crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return new_crc


def mav_crc_x25(buf: bytes, crc_extra: int) -> int:
    crc = 0xFFFF
    for b in buf:
        crc = crc_accumulate(b, crc)
    crc = crc_accumulate(crc_extra, crc)
    return crc


def build_command_long_packet(target_sysid: int, target_compid: int, command: int, params):

    global mav_seq

    msg_id = 76         # COMMAND_LONG
    crc_extra = 152     # CRC extra for COMMAND_LONG
    payload_len = 33    # 7*4 + 2 + 1 + 1 + 1

    if params is None:
        params = [0.0] * 7
    if len(params) < 7:
        params = list(params) + [0.0] * (7 - len(params))

    payload = bytearray(payload_len)

    # 7 floats param1..param7
    import struct
    for i in range(7):
        struct.pack_into("<f", payload, i * 4, float(params[i]))

    # command (uint16)
    struct.pack_into("<H", payload, 28, command)

    # target_system, target_component, confirmation
    payload[30] = target_sysid & 0xFF
    payload[31] = target_compid & 0xFF
    payload[32] = 0  # confirmation

    # Header
    seq = mav_seq & 0xFF
    mav_seq = (mav_seq + 1) & 0xFF

    frame = bytearray(10 + payload_len + 2)
    o = 0
    frame[o] = 0xFD; o += 1         # magic
    frame[o] = payload_len; o += 1  # length
    frame[o] = 0x00; o += 1         # incompat_flags
    frame[o] = 0x00; o += 1         # compat_flags
    frame[o] = seq; o += 1          # seq
    frame[o] = GCS_SYS_ID; o += 1   # sysid (GCS)
    frame[o] = GCS_COMP_ID; o += 1  # compid (GCS)
    # msgid (3 bytes, little endian)
    frame[o] = msg_id & 0xFF; o += 1
    frame[o] = (msg_id >> 8) & 0xFF; o += 1
    frame[o] = (msg_id >> 16) & 0xFF; o += 1

    # payload
    frame[o:o+payload_len] = payload
    o += payload_len

    # CRC over payload + msgid bytes
    crc_input = bytes(payload) + bytes([msg_id & 0xFF, (msg_id >> 8) & 0xFF, (msg_id >> 16) & 0xFF])
    crc = mav_crc_x25(crc_input, crc_extra)
    frame[o] = crc & 0xFF
    frame[o+1] = (crc >> 8) & 0xFF

    return bytes(frame)


def request_gimbal_msgs(sysid):
    # Request MOUNT_ORIENTATION
    send_command_long(sysid, GIMBAL_COMP_ID, MAV_CMD_REQUEST_MESSAGE, [265, 1, 0, 0, 0, 0, 0])
    # Request GIMBAL_DEVICE_ATTITUDE_STATUS
    send_command_long(sysid, GIMBAL_COMP_ID, MAV_CMD_REQUEST_MESSAGE, [284, 1, 0, 0, 0, 0, 0])


def send_command_long(target_sysid: int, target_compid: int, command: int, params):
    if udp_transport is None:
        print("❌ UDP transport not ready, cannot send command")
        return

    pkt = build_command_long_packet(target_sysid, target_compid, command, params)
#    udp_transport.sendto(pkt, (UDP_TARGET_HOST, UDP_TARGET_PORT))
    addr = last_mavlink_addr_by_sysid.get(target_sysid)
    if addr is None:
        addr = (UDP_TARGET_HOST, UDP_TARGET_PORT)  # fallback

    udp_transport.sendto(pkt, addr)
    print(f"➡ Sent COMMAND_LONG id={command} to {addr[0]}:{addr[1]} sys={target_sysid} comp={target_compid}")
#    print(f"➡ Sent COMMAND_LONG id={command} to {UDP_TARGET_HOST}:{UDP_TARGET_PORT} "
#          f"sys={target_sysid} comp={target_compid}")


# ------------------------
# BROADCAST TO WS CLIENTS
# ------------------------
async def broadcast_uav(obj: Dict[str, Any]) -> None:
    if not uav_clients:
        return
    text = json.dumps(obj)
    dead = []
    for ws in uav_clients:
        try:
            await ws.send_text(text)
        except Exception:
            dead.append(ws)
    for ws in dead:
        uav_clients.discard(ws)


async def broadcast_amr(obj: Dict[str, Any]) -> None:
    if not amr_clients:
        return
    text = json.dumps(obj)
    dead = []
    for ws in amr_clients:
        try:
            await ws.send_text(text)
        except Exception:
            dead.append(ws)
    for ws in dead:
        amr_clients.discard(ws)


# ------------------------
# UDP MAVLINK2 LISTENER
# ------------------------
class MavlinkUDPProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        global last_mavlink_rx_ms, mavlink_msg_count

        last_mavlink_rx_ms = int(time.time() * 1000)
        mavlink_msg_count += 1

        if not data or len(data) < 8:
            return

        stx = data[0]
        if stx not in (0xFD, 0xFE):
            # not a MAVLink frame
            # print(f"⚠️ Not MAVLink: stx=0x{stx:02X}")
            return

        try:
            payload_len = data[1]

            if stx == 0xFD:
                # MAVLink2
                if len(data) < 10 + payload_len:
                    return
                seq = data[4]
                sysid = data[5]
                compid = data[6]
                msgid = data[7] | (data[8] << 8) | (data[9] << 16)
                payload_start = 10

            else:
                # MAVLink1 (0xFE)
                if len(data) < 6 + payload_len:
                    return
                seq = data[2]
                sysid = data[3]
                compid = data[4]
                msgid = data[5]
                payload_start = 6

            payload_end = payload_start + payload_len
            payload = data[payload_start:payload_end]

            tel = _decode_and_update_telemetry(
                sysid=sysid,
                compid=compid,
                seq=seq,
                msgid=msgid,
                payload=payload,
                addr=addr,
                received_at_ms=last_mavlink_rx_ms,
            )

            # only broadcast for sysids we trust (with heartbeat)
            if sysid in valid_sysids_with_heartbeat and uav_clients:
                loop = asyncio.get_event_loop()
                loop.create_task(
                    broadcast_uav({
                        "type": "telemetry",
                        **tel,
                    })
                )

        except Exception as e:
            print("Error parsing MAVLink packet:", e)


# ------------------------
# COMMAND HANDLER (from UI)
# ------------------------


def handle_command_from_ui(message: Dict[str, Any]) -> None:
    global gimbal_pitch_deg, gimbal_yaw_deg

    cmd = message.get("cmd")
    mission = message.get("mission") or {}
    sysid = message.get("sysid")

    target_sys_id = sysid or mission.get("droneSysId") or 1
    target_comp_id = 1  # autopilot

    print(f"📨 UI Command: {cmd} → sysid={target_sys_id} extra={message}")

    MAV_CMD_DO_DIGICAM_CONTROL = 203
    MAV_CMD_VIDEO_START_CAPTURE = 250
    MAV_CMD_VIDEO_STOP_CAPTURE = 251
    MAV_CMD_SET_CAMERA_ZOOM = 531
    MAV_CMD_DO_MOUNT_CONTROL = 205

    if cmd == "TAKE_PHOTO":
        params = [0, 0, 0, 0, 1, 0, 0]
        send_command_long(target_sys_id, target_comp_id, MAV_CMD_DO_DIGICAM_CONTROL, params)

    elif cmd == "REC_START":
        params = [0, 1, 0, 0, 0, 0, 0]
        send_command_long(target_sys_id, target_comp_id, MAV_CMD_VIDEO_START_CAPTURE, params)

    elif cmd == "REC_STOP":
        params = [0, 1, 0, 0, 0, 0, 0]
        send_command_long(target_sys_id, target_comp_id, MAV_CMD_VIDEO_STOP_CAPTURE, params)

    elif cmd == "ZOOM_IN":
        params = [0, 1, 0, 0, 0, 0, 0]
        send_command_long(target_sys_id, target_comp_id, MAV_CMD_SET_CAMERA_ZOOM, params)

    elif cmd == "ZOOM_OUT":
        params = [0, -1, 0, 0, 0, 0, 0]
        send_command_long(target_sys_id, target_comp_id, MAV_CMD_SET_CAMERA_ZOOM, params)

    elif cmd in ("GIMBAL_UP", "GIMBAL_DOWN", "GIMBAL_LEFT", "GIMBAL_RIGHT", "GIMBAL_CENTER"):
        step = 5.0

        if cmd == "GIMBAL_UP":
            gimbal_pitch_deg += step
        elif cmd == "GIMBAL_DOWN":
            gimbal_pitch_deg -= step
        elif cmd == "GIMBAL_LEFT":
            gimbal_yaw_deg -= step
        elif cmd == "GIMBAL_RIGHT":
            gimbal_yaw_deg += step
        elif cmd == "GIMBAL_CENTER":
            gimbal_pitch_deg = 0.0
            gimbal_yaw_deg = 0.0

        gimbal_pitch_deg = max(GIMBAL_PITCH_MIN, min(GIMBAL_PITCH_MAX, gimbal_pitch_deg))
        gimbal_yaw_deg = max(GIMBAL_YAW_MIN, min(GIMBAL_YAW_MAX, gimbal_yaw_deg))

        params = [
            gimbal_pitch_deg,  # pitch
            0.0,               # roll
            gimbal_yaw_deg,    # yaw
            0.0,
            0.0,
            0.0,
            2.0,               # MAV_MOUNT_MODE_MAVLINK_TARGETING
        ]
        send_command_long(target_sys_id, GIMBAL_COMP_ID, MAV_CMD_DO_MOUNT_CONTROL, params)

    elif cmd in ("FPV_START", "FPV_STOP"):
        print(f"(No MAVLink sent for {cmd} — handled by video pipeline)")

    elif cmd == "MISSION_UPLOAD":
        print("🛰 Mission upload received:", mission)
        # Extract waypoints from mission sent by the UI
        waypoints = mission.get("waypoints") or []
        if not waypoints:

            print("⚠️ No waypoints in mission, nothing to send")

        else:

            target = sysid or mission.get("droneSysId") or 1
            loc = mission.get("location") or {}
            default_alt = float(loc.get("alt", 50.0))

            # Optional: hold 0 seconds at each waypoint

            send_waypoint_sequence_to_drone(
                waypoints=waypoints,
                default_alt=default_alt,
                target_sysid=int(target),
                hold_time=0.0,
            )

        # still notify UI that we "sent" the mission

        try:
            loop = asyncio.get_event_loop()
            loop.create_task(
                broadcast_uav({
                    "type": "mission_ack",
                    "status": "sent",
                    "missionName": mission.get("missionName", "Untitled"),
                })
            )

        except RuntimeError:

            # fallback if no running loop (shouldn't happen inside FastAPI)
            asyncio.create_task(
                broadcast_uav({
                    "type": "mission_ack",
                    "status": "sent",
                    "missionName": mission.get("missionName", "Untitled"),
                })

            )

    else:
        print("Unknown command from UI:", cmd)


def send_waypoint_sequence_to_drone(
    waypoints: List[Dict[str, Any]],
    default_alt: float,
    target_sysid: int,
    hold_time: float = 0.0,
) -> None:

    MAV_CMD_NAV_WAYPOINT = 16
    target_comp_id = 1  # autopilot

    if udp_transport is None:
        print("❌ UDP transport not ready, cannot send waypoints")
        return

    for idx, wp in enumerate(waypoints):
        # support both {lat,lng} and {lat,lon}
        lat = wp.get("lat") or wp.get("latitude")
        lon = wp.get("lng") or wp.get("lon") or wp.get("longitude")
        alt = wp.get("alt", default_alt)

        if lat is None or lon is None:
            print(f"⚠️ Waypoint {idx} missing lat/lon, skipping:", wp)
            continue

        params = [
            float(hold_time),  # param1: hold time (seconds)
            0.0,               # param2: unused
            0.0,               # param3: unused
            0.0,               # param4: yaw (0 = unchanged)
            float(lat),        # param5: target lat (deg)
            float(lon),        # param6: target lon (deg)
            float(alt),        # param7: target alt (m)
        ]

        print(
            f"➡ Sending WP#{idx} "
            f"lat={lat:.7f} lon={lon:.7f} alt={alt:.1f} "
            f"to sysid={target_sysid}"
        )
        send_command_long(
            target_sysid=target_sysid,
            target_compid=target_comp_id,
            command=MAV_CMD_NAV_WAYPOINT,
            params=params,
        )

        # Small delay so we don't spam FC too fast
        time.sleep(0.2)


# ----------ROS HELPERS ----------
def load_stations_from_file() -> None:
    global stations
    if not os.path.exists(STATIONS_FILE):
        stations = {}
        return
    try:
        with open(STATIONS_FILE, "r") as f:
            stations = json.load(f)
    except Exception as e:
        print(f"[WARN] Failed to load stations file: {e}")
        stations = {}


def save_stations_to_file() -> None:
    try:
        with open(STATIONS_FILE, "w") as f:
            json.dump(stations, f, indent=2)
        print(f"[INFO] Saved {len(stations)} stations to {STATIONS_FILE}")
    except Exception as e:
        print(f"[ERROR] Failed to save stations: {e}")


async def ipc_send(msg: Dict[str, Any]) -> None:
    """Send a JSON message to the IPC on the AMR."""
    global ipc_ws
    data = json.dumps(msg)
    async with ipc_lock:
        if ipc_ws is None:
            print("[IPC] not connected, dropping:", msg)
            return
        try:
            await ipc_ws.send(data)
        except Exception as e:
            print(f"[IPC] send error: {e}")
            # drop connection; background task will reconnect
            try:
                await ipc_ws.close()
            except Exception:
                pass
            ipc_ws = None


# ------------------------
# HTTP ROUTES
# ------------------------
@app.get("/server-ip")
async def server_ip():
    return {"ip": get_server_ip()}


@app.get("/api/health")
async def health():
    return {
        "ok": True,
        "mode": "udp+ws",
        "udpListen": f"{UDP_LISTEN_HOST}:{UDP_LISTEN_PORT}",
    }


@app.post("/api/missions")
async def save_mission(mission: Dict[str, Any]):
    mission = mission or {}
    mission_id = len(missions) + 1
    mission["id"] = mission_id
    mission["createdAt"] = datetime.utcnow().isoformat() + "Z"
    missions.append(mission)
    print("💾 Mission saved:", mission.get("missionName", "Untitled"), "status:", mission.get("status"))
    return {"ok": True, "mission": mission}


@app.post("/api/addnotif")
async def add_notification(n: Dict[str, Any]):
    n = n or {}
    n_id = len(notifications) + 1
    n["id"] = n_id
    n["createdAt"] = datetime.utcnow().isoformat() + "Z"
    notifications.insert(0, n)
    print("🔔 Notification:", n.get("message"))
    return {"ok": True}


@app.post("/api/login")
async def login(body: Dict[str, Any]):
    username = body.get("username")
    password = body.get("password")
    if username == "admin" and password == "12345":
        return RedirectResponse(url="/menu", status_code=303)
    return JSONResponse({"error": "Invalid credentials"}, status_code=401)


@app.post("/api/test-mavlink")      # for testing
async def test_mavlink(sysid: Optional[int] = None):
    """
    Fire a harmless MAVLink COMMAND_LONG (REQUEST_MESSAGE for GLOBAL_POSITION_INT)
    so we can check that:
      - UDP transport works
      - Drone responds with telemetry
    """
    # 1) Choose target sysid
    if sysid is None:
        # Prefer a sysid that has sent HEARTBEAT
        if valid_sysids_with_heartbeat:
            target_sysid = sorted(valid_sysids_with_heartbeat)[0]
        elif last_telemetry_by_sysid:
            target_sysid = sorted(last_telemetry_by_sysid.keys())[0]
        else:
            target_sysid = 1  # fallback
    else:
        target_sysid = sysid

    target_compid = 1  # autopilot

    # 2) Check UDP is ready
    if udp_transport is None:
        return JSONResponse(
            {
                "ok": False,
                "error": "UDP transport not ready (no MAVLink socket). "
                         "Check startup logs and UDP_LISTEN_HOST/PORT.",
            },
            status_code=500,
        )

    # 3) Build params for MAV_CMD_REQUEST_MESSAGE
    # param1 = message ID to request (GLOBAL_POSITION_INT = 33)
    params = [
        float(GLOBAL_POSITION_INT_ID),  # param1: message id
        1.0,                            # param2: request ON (1) or OFF (0)
        0.0, 0.0, 0.0, 0.0, 0.0         # remaining unused
    ]

    # 4) Send the command
    send_command_long(
        target_sysid,
        target_compid,
        MAV_CMD_REQUEST_MESSAGE,
        params,
    )
    print(
        f"➡ TEST: sent MAV_CMD_REQUEST_MESSAGE ({GLOBAL_POSITION_INT_ID}) "
        f"to sys={target_sysid} comp={target_compid}"
    )

    return {
        "ok": True,
        "info": "Sent REQUEST_MESSAGE for GLOBAL_POSITION_INT",
        "target_sysid": target_sysid,
        "target_compid": target_compid,
        "requestedMessageId": GLOBAL_POSITION_INT_ID,
    }


@app.post("/api/test-gimbal-nudge")
async def test_gimbal_nudge(sysid: Optional[int] = None):
    """
    Simple HTTP test to nudge the gimbal DOWN once.
    Usage:
      POST /api/test-gimbal-nudge          -> uses first heartbeat sysid or 1
      POST /api/test-gimbal-nudge?sysid=5  -> forces sysid 5
    """
    # --- pick a target sysid ---
    if sysid is not None:
        try:
            target_sys_id = int(sysid)
        except (TypeError, ValueError):
            target_sys_id = 1
    else:
        # If we already have heartbeats, pick the first one,
        # otherwise fall back to 1
        if valid_sysids_with_heartbeat:
            target_sys_id = sorted(valid_sysids_with_heartbeat)[0]
        else:
            target_sys_id = 1

    # --- call the same logic as the UI buttons ---
    handle_command_from_ui({
        "cmd": "GIMBAL_DOWN",
        "sysid": target_sys_id,
    })

    return {
        "ok": True,
        "info": "Sent GIMBAL_DOWN one step",
        "target_sys_id": target_sys_id,
    }


@app.get("/menu", response_class=HTMLResponse)
async def menu_page():
    return FileResponse(os.path.join(FRONTEND_DIR, "menu.html"))


@app.get("/ugv-cc", response_class=HTMLResponse)
async def amr_stations_page(request: Request, user: str = Depends(get_current_user)):
    return FileResponse(os.path.join(FRONTEND_DIR, "direction.html"))


@app.get("/uav-cc", response_class=HTMLResponse)
async def amr_stations_page(request: Request, user: str = Depends(get_current_user)):
    return FileResponse(os.path.join(FRONTEND_DIR, "commandcenter.html"))


@app.get("/api/drones")
async def list_drones():
    drones = []
    for sysid, t in last_telemetry_by_sysid.items():
        if sysid not in valid_sysids_with_heartbeat:
            continue
        drones.append({
            "sysid": t["sysid"],
            "compid": t["compid"],
            "lastSeen": t["receivedAt"],
        })
    return {"ok": True, "drones": drones}


# Serve main HTML (same as Node's login.html)
@app.get("/login")
async def root():
    if os.path.isdir(FRONTEND_DIR):
        login_path = os.path.join(FRONTEND_DIR, "login.html")
        if os.path.isfile(login_path):
            return FileResponse(login_path)
    return JSONResponse({"message": "Frontend not found"}, status_code=404)


@app.get("/api/mavlink-status")
async def mavlink_status():
    now_ms = int(time.time() * 1000)
    age_ms = now_ms - last_mavlink_rx_ms if last_mavlink_rx_ms > 0 else None
    connected = age_ms is not None and age_ms < MAVLINK_TIMEOUT_MS

    return {
        "connected": connected,
        "lastMessageAgeMs": age_ms,
        "totalMessages": mavlink_msg_count,
        # only sysids that have actually sent HEARTBEAT
        "sysids": sorted(valid_sysids_with_heartbeat),
    }


@app.get("/api/amr_pose")
async def get_amr_pose():
    if not current_amr_pose:
        return {"ok": False, "pose": None}
    return {"ok": True, "pose": current_amr_pose}


# ------------------------
# WEBSOCKET ROUTE
# ------------------------
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    uav_clients.add(ws)
    print("🌐 WebSocket client connected")

    await ws.send_text(json.dumps({
        "type": "info",
        "message": "Connected to command & telemetry server (Python)"
    }))

    # send cached telemetry
    for tel in last_telemetry_by_sysid.values():
        await ws.send_text(json.dumps({
            "type": "telemetry",
            **tel
        }))

    try:
        while True:
            text = await ws.receive_text()
            try:
                msg = json.loads(text)
            except json.JSONDecodeError:
                print("Bad WS message (not JSON):", text)
                continue

            if msg.get("type") == "command":
                handle_command_from_ui(msg)
            else:
                print("WS message (unknown type):", msg)
    except WebSocketDisconnect:
        print("WebSocket client disconnected")
    finally:
        uav_clients.discard(ws)


@app.websocket("/ws/amr_stations")
async def amr_stations_ws(websocket: WebSocket):
    await websocket.accept()
    amr_clients.add(websocket)
    print("[WS] browser client connected (stations)")

    # send current snapshot
    await websocket.send_text(json.dumps({"type": "stations_data", "stations": stations}))
    await websocket.send_text(json.dumps({"type": "odom_station", "text": current_odom_text}))
    await websocket.send_text(json.dumps({"type": "map_update", "url": map_image_url}))

    try:
        while True:
            raw = await websocket.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                print("[WS] bad JSON from browser:", raw)
                continue

            msg_type = msg.get("type")

            # ---- STATION SAVE/DELETE (handled locally) ----
            if msg_type == "save_station":
                name = (msg.get("name") or "").strip()
                if not name:
                    continue

                # for now just create a placeholder; ROS/IPC can later fill pose
                existing = stations.get(name)
                if existing is None:
                    stations[name] = {"x": 0, "y": 0}  # or leave {} if you prefer
                # if you want to allow waypoints list, you can adapt this logic

                save_stations_to_file()

                # Optionally forward to IPC if connected
                await ipc_send({"type": "save_station", "name": name})
                continue

            if msg_type == "delete_station":
                name = (msg.get("name") or "").strip()
                if name in stations:
                    del stations[name]
                    save_stations_to_file()
                await ipc_send({"type": "delete_station", "name": name})
                continue

            # ---- OTHER COMMANDS: just forward to IPC / ROS2 ----
            if msg_type in {
                "goto_station",
                "cancel_navigation",
                "emergency_stop",
                "move",
                "set_initial_pose",
            }:
                await ipc_send(msg)
                continue

            print("[WS] unknown msg type from browser:", msg_type, msg)

    except WebSocketDisconnect:
        print("[WS] browser client disconnected (stations)")
    finally:
        amr_clients.discard(websocket)


# ------------------------
# APP STARTUP: create UDP listener
# ------------------------
@app.on_event("startup")
async def startup_event():
    global udp_transport
    load_stations_from_file()
    loop = asyncio.get_running_loop()
#   loop.create_task(broadcast_uav(...)) if it breaks in later versions
    udp_transport, _ = await loop.create_datagram_endpoint(
        lambda: MavlinkUDPProtocol(),
        local_addr=(UDP_LISTEN_HOST, UDP_LISTEN_PORT),
    )
    print(f"✅ UDP listening on {UDP_LISTEN_HOST}:{UDP_LISTEN_PORT}")
    os.makedirs(HLS_OUTPUT_DIR, exist_ok=True)

    ff_cmd = [
        "ffmpeg",
        "-rtsp_transport", "tcp",
        "-i", RTSP_URL,
        "-codec:v", "copy",
        "-codec:a", "aac", "-an",
        "-f", "hls",
        "-hls_time", "1",
        "-hls_list_size", "5",
        "-hls_flags", "delete_segments",
        os.path.join(HLS_OUTPUT_DIR, "stream.m3u8"),
    ]

    try:
        subprocess.Popen(ff_cmd)
        print("🎥 ffmpeg started for RTSP → HLS")
    except Exception as e:
        print("⚠️ Failed to start ffmpeg:", e)
    asyncio.create_task(ipc_connector())
    asyncio.create_task(gcs_keepalive_task())


async def gcs_keepalive_task():
    """Periodically poke the drone so it keeps sending us MAVLink."""
    await asyncio.sleep(5.0)  # wait for startup
    while True:
        try:
            # pick a sysid, same logic as /api/test-mavlink
            if valid_sysids_with_heartbeat:
                target_sysid = sorted(valid_sysids_with_heartbeat)[0]
            elif last_telemetry_by_sysid:
                target_sysid = sorted(last_telemetry_by_sysid.keys())[0]
            else:
                target_sysid = 1

            target_compid = 1  # autopilot
            params = [float(GLOBAL_POSITION_INT_ID), 1.0, 0, 0, 0, 0, 0]

            send_command_long(target_sysid, target_compid, MAV_CMD_REQUEST_MESSAGE, params)
        except Exception as e:
            print("Keepalive error:", e)

        await asyncio.sleep(2.0)  # every 2 seconds


async def ipc_connector():
    """Background task: connect to IPC and listen for messages."""
    global ipc_ws, stations, current_odom_text, map_image_url

    while True:
        try:
            print(f"[IPC] Connecting to {IPC_WS_URL} ...")
            async with websockets.connect(IPC_WS_URL) as ws:
                print("[IPC] Connected.")
                async with ipc_lock:
                    ipc_ws = ws

                # send initial stations we have on disk
                await ipc_send({"type": "stations_data", "stations": stations})

                async for raw in ws:
                    try:
                        msg = json.loads(raw)
                    except Exception:
                        print("[IPC] bad JSON from IPC:", raw)
                        continue

                    msg_type = msg.get("type")

                    # Messages coming from IPC → forward to browsers
                    if msg_type == "stations_data":
                        stations = msg.get("stations", {}) or {}
                        save_stations_to_file()
                        await broadcast_amr({"type": "stations_data", "stations": stations})

                    elif msg_type == "odom_station":
                        current_odom_text = msg.get("text", current_odom_text)
                        await broadcast_amr({"type": "odom_station", "text": current_odom_text})

                    elif msg_type == "map_update":
                        map_image_url = msg.get("url", map_image_url)
                        await broadcast_amr({"type": "map_update", "url": map_image_url})

                    elif msg_type == "amr_pose":
                        global current_amr_pose
                        current_amr_pose = {
                            "x": msg.get("x"),
                            "y": msg.get("y"),
                            "theta": msg.get("theta"),
                        }
                        await broadcast_amr({
                            "type": "amr_pose", **current_amr_pose,
                        })

                    else:
                        print("[IPC] unhandled message from IPC:", msg)

        except Exception as e:
            print(f"[IPC] connection error: {e}")
        finally:
            async with ipc_lock:
                ipc_ws = None
        # retry every 3 seconds
        await asyncio.sleep(3.0)


# ------------------------
# RUN
# ------------------------
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=HTTP_PORT, reload=True)
