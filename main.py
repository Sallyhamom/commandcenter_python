import asyncio
import base64
import json
import os
import socket
import struct
import time
from datetime import datetime
from typing import Dict, Any, List, Set, Tuple

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles

import subprocess
import os

RTSP_URL = "rtsp://192.168.144.26:8554/main.264"
# RTSP_URL = "rtsp://rtspstream:POB-48SOLYWIPDJE5uX4v@zephyr.rtsp.stream/people"
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

# MAVLink sequence (0–255)
mav_seq = 0

# ------------------------
# GIMBAL STATE (deg)
# ------------------------
gimbal_pitch_deg = 0.0
gimbal_yaw_deg = 0.0

GIMBAL_PITCH_MIN = -90.0   # look straight down
GIMBAL_PITCH_MAX = 30.0    # slight up
GIMBAL_YAW_MIN = -180.0
GIMBAL_YAW_MAX = 180.0

GIMBAL_COMP_ID = 154       # MAV_COMP_ID_GIMBAL; change to 1 to test via FC

# ------------------------
# GLOBAL STATE
# ------------------------
app = FastAPI()

os.makedirs(HLS_OUTPUT_DIR, exist_ok=True)
FRONTEND_DIR = os.path.join(os.path.dirname(__file__), "mission-control-frontend")
if os.path.isdir(FRONTEND_DIR):
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")
app.mount("/fpv", StaticFiles(directory=HLS_OUTPUT_DIR), name="fpv")

FRONTEND_DIR = os.path.join(os.path.dirname(__file__), "mission-control-frontend")

if os.path.isdir(FRONTEND_DIR):
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")

missions: List[Dict[str, Any]] = []
notifications: List[Dict[str, Any]] = []
last_telemetry_by_sysid: Dict[int, Dict[str, Any]] = {}

connected_websockets: Set[WebSocket] = set()
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

MAV_MODE_FLAG_SAFETY_ARMED = 0x80  # from MAVLink spec


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
            "lon": lon / 1e7,
            "alt": alt / 1000.0,
            "relAlt": rel_alt / 1000.0,
            "vx": vx / 100.0,
            "vy": vy / 100.0,
            "vz": vz / 100.0,
            "heading": None if hdg == 0xFFFF else hdg / 100.0,
        }

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


def send_command_long(target_sysid: int, target_compid: int, command: int, params):
    if udp_transport is None:
        print("❌ UDP transport not ready, cannot send command")
        return

    pkt = build_command_long_packet(target_sysid, target_compid, command, params)
    udp_transport.sendto(pkt, (UDP_TARGET_HOST, UDP_TARGET_PORT))
    print(f"➡ Sent COMMAND_LONG id={command} to {UDP_TARGET_HOST}:{UDP_TARGET_PORT} sys={target_sysid} comp={target_compid}")


# ------------------------
# BROADCAST TO WS CLIENTS
# ------------------------
async def broadcast(obj: Dict[str, Any]) -> None:
    if not connected_websockets:
        return
    text = json.dumps(obj)
    disconnected = []
    for ws in connected_websockets:
        try:
            await ws.send_text(text)
        except Exception:
            disconnected.append(ws)
    for ws in disconnected:
        connected_websockets.discard(ws)


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
            if sysid in valid_sysids_with_heartbeat and connected_websockets:
                loop = asyncio.get_event_loop()
                loop.create_task(
                    broadcast({
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
        asyncio.create_task(
            broadcast({
                "type": "mission_ack",
                "status": "sent",
                "missionName": mission.get("missionName", "Untitled"),
            })
        )

    else:
        print("Unknown command from UI:", cmd)


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
        return {"success": True}
    return JSONResponse({"error": "Invalid credentials"}, status_code=401)


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
@app.get("/")
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


# ------------------------
# WEBSOCKET ROUTE
# ------------------------
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    connected_websockets.add(ws)
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
        connected_websockets.discard(ws)


# ------------------------
# APP STARTUP: create UDP listener
# ------------------------
@app.on_event("startup")
async def startup_event():
    global udp_transport
    loop = asyncio.get_running_loop()
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


# ------------------------
# RUN
# ------------------------
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=HTTP_PORT, reload=True)
