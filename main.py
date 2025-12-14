import asyncio
import subprocess
import base64
import json
import signal
import sys
import threading
import os
import socket
import time
from datetime import datetime
from typing import Dict, Any, List, Set, Tuple, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, Depends
from fastapi.responses import JSONResponse, FileResponse, HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
import websockets

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2


# ------------------------
# VIDEO / RTSP CONFIG
# ------------------------
RTSP_URL = "rtsp://192.168.144.26:8554/main.264"
HLS_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "hls")
HLS_PLAYLIST = os.path.join(HLS_OUTPUT_DIR, "stream.m3u8")

# ------------------------
# C2 / HTTP CONFIG
# ------------------------
HTTP_PORT = 4000  # HTTP + WebSocket

# ------------------------
# MAVLINK LINK (pymavlink, same as udp_test.py)
# ------------------------
CUBE_IP = "192.168.144.12"
CUBE_PORT = 19856
MAV_CONN_STR = f"udpout:{CUBE_IP}:{CUBE_PORT}"

# GCS identity in MAVLink
GCS_SYS_ID = 255
GCS_COMP_ID = 190

MAV_CMD_REQUEST_MESSAGE = 512
GLOBAL_POSITION_INT_ID = 33
MAV_CMD_COMPONENT_ARM_DISARM = 400

# pymavlink master connection
mav_master: Optional[mavutil.mavlink_connection] = None
mavlink_lock = asyncio.Lock()

# MAVLink link status
last_mavlink_rx_ms: int = 0    # timestamp (ms) of last MAVLink packet
mavlink_msg_count: int = 0     # total number of MAVLink packets seen
MAVLINK_TIMEOUT_MS = 5000      # consider "disconnected" if no packet in 5s

# ------------------------
# GIMBAL STATE (deg)
# ------------------------
gimbal_pitch_deg = 0.0
gimbal_yaw_deg = 0.0

GIMBAL_PITCH_MIN = -90.0   # look straight down
GIMBAL_PITCH_MAX = 30.0    # slight up
GIMBAL_YAW_MIN = -180.0
GIMBAL_YAW_MAX = 180.0

GIMBAL_COMP_ID = 1       # MAV_COMP_ID_AUTOPILOT (FC); gimbal manager handled there
MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000

ffmpeg_proc = None
ffmpeg_running = False
ffmpeg_thread = None
stop_event = threading.Event()
current_amr_pose = {}

# Latest ZED frame (optional cache)
latest_zed_frame: Dict[str, Any] = {}

# ------------------------
# APP & STATIC
# ------------------------
app = FastAPI()

os.makedirs(HLS_OUTPUT_DIR, exist_ok=True)
FRONTEND_DIR = os.path.join(os.path.dirname(__file__), "mission-control-frontend")
if os.path.isdir(FRONTEND_DIR):
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")

app.mount("/hls", StaticFiles(directory=HLS_OUTPUT_DIR), name="hls")

# IP / port of the IPC on the AMR:
IPC_WS_URL = "ws://192.168.12.165:8765"   # change to your AMR IPC IP

# ---------- STATE ----------
stations: Dict[str, Any] = {}
current_odom_text: str = "Waiting for odometry..."
map_image_url: str = "/static/my_map.png"
STATIONS_FILE = os.path.join(os.path.dirname(__file__), "stations.json")

ipc_ws: Optional[websockets.WebSocketClientProtocol] = None
ipc_lock = asyncio.Lock()

missions: List[Dict[str, Any]] = []
notifications: List[Dict[str, Any]] = []
last_telemetry_by_sysid: Dict[int, Dict[str, Any]] = {}
last_mavlink_addr_by_sysid: Dict[int, Tuple[str, int]] = {}  # kept for compatibility, unused now

uav_clients: Set[WebSocket] = set()
amr_clients: Set[WebSocket] = set()

# TELEMETRY FILTERS
valid_sysids_with_heartbeat: Set[int] = set()

# MAVLink constants we care about
MAV_MODE_FLAG_SAFETY_ARMED = 0x80
mav_reader_task_handle: Optional[asyncio.Task] = None
gcs_keepalive_task_handle: Optional[asyncio.Task] = None
ipc_connector_task_handle: Optional[asyncio.Task] = None


def get_current_user(request: Request):
    return "admin"


def ffmpeg_loop(ff_cmd):
    global ffmpeg_proc

    while not stop_event.is_set():
        print("▶ Starting FFmpeg...")
        ffmpeg_proc = subprocess.Popen(
            ff_cmd,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
        )

        while ffmpeg_proc.poll() is None and not stop_event.is_set():
            time.sleep(0.5)

        if stop_event.is_set():
            break

        print("⚠ FFmpeg exited, restarting in 2s...")
        time.sleep(2)

    print("🛑 FFmpeg loop stopped")


def ffmpeg_watchdog(ff_cmd):
    global ffmpeg_proc, ffmpeg_running

    ffmpeg_running = True

    while ffmpeg_running:
        print("▶ starting ffmpeg")
        ffmpeg_proc = subprocess.Popen(ff_cmd)

        ffmpeg_proc.wait()

        if not ffmpeg_running:
            break

        print("⚠ ffmpeg stopped, restarting in 2s")
        time.sleep(2)

    print("🛑 ffmpeg watchdog exited")


# ------------------------
# SERVER IP DISCOVERY
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
# MAVLINK CONNECT (BLOCKING) - SAME PATTERN AS udp_test.py
# ------------------------
def init_mavlink():
    """
    Non-blocking MAVLink init.
    We just create the connection and send one heartbeat.
    The reader task will wait for and print the first HEARTBEAT.
    """
    global mav_master

    if mav_master is not None:
        return

    print(f"[MAV] Connecting to {MAV_CONN_STR} ...")
    master = mavutil.mavlink_connection(MAV_CONN_STR, source_system=GCS_SYS_ID)

    # Kick UDP socket (Windows + udpout quirk)
    print("[MAV] Sending initial heartbeat to open UDP socket...")
    master.mav.heartbeat_send(0, 0, 0, 0, 0)

    mav_master = master
    print("[MAV] MAVLink connection handle created; waiting for HEARTBEAT in reader task...")


# ------------------------
# COMMAND_LONG (PYMAVLINK)
# ------------------------
def send_command_long_pymav(target_sysid: int, target_compid: int, command: int, params):
    global mav_master
    if mav_master is None:
        print("❌ MAVLink master not ready, cannot send COMMAND_LONG")
        return

    if params is None:
        params = [0.0] * 7
    if len(params) < 7:
        params = list(params) + [0.0] * (7 - len(params))

    try:
        mav_master.mav.command_long_send(
            target_sysid,
            target_compid,
            command,
            0,  # confirmation
            float(params[0]),
            float(params[1]),
            float(params[2]),
            float(params[3]),
            float(params[4]),
            float(params[5]),
            float(params[6]),
        )
    except Exception as e:
        print("Error sending COMMAND_LONG via pymavlink:", e)


# ------------------------
# MAVLINK READER TASK (TELEMETRY)
# ------------------------
async def mavlink_reader_task():
    """
    Background task: read MAVLink using pymavlink and update telemetry + broadcast.
    Uses run_in_executor so we don't block the asyncio loop.
    """
    global mav_master, last_mavlink_rx_ms, mavlink_msg_count

    print("[MAV] Reader task started")
    loop = asyncio.get_running_loop()

    while True:
        if mav_master is None:
            await asyncio.sleep(1.0)
            continue

        try:
            # Offload the blocking recv_match to a thread
            msg = await loop.run_in_executor(
                None,
                lambda: mav_master.recv_match(blocking=True, timeout=1)
            )
        except asyncio.CancelledError:
            print("[MAV] Reader task cancelled")
            break
        except Exception as e:
            print("[MAV] recv error:", e)
            await asyncio.sleep(1.0)
            continue

        if msg is None:
            continue

        now_ms = int(time.time() * 1000)
        last_mavlink_rx_ms = now_ms
        mavlink_msg_count += 1

        mtype = msg.get_type()
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()

        # --- DEBUG PRINTS, like your udp_test.py ---
        if mtype == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            # print(f"[GPS] lat={lat:.7f}, lon={lon:.7f}, alt={alt:.1f}m")
        elif mtype == "SYS_STATUS":
            v = msg.voltage_battery / 1000.0
            # print(f"[BAT] {v:.2f} V (remaining={msg.battery_remaining}%)")

        # --- Build telemetry dict for UI ---
        t = last_telemetry_by_sysid.get(sysid, {
            "sysid": sysid,
            "compid": compid,
        })
        t["compid"] = compid
        t["msgType"] = mtype
        t["receivedAt"] = now_ms

        if mtype == "HEARTBEAT":
            t["mode"] = msg.custom_mode
            t["baseMode"] = msg.base_mode
            t["systemStatus"] = msg.system_status
            t["armed"] = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
            valid_sysids_with_heartbeat.add(sysid)

            if t["armed"]:
                t["uiStatus"] = "ARMED"
            elif msg.system_status in (
                    mavutil.mavlink.MAV_STATE_UNINIT,
                    mavutil.mavlink.MAV_STATE_CALIBRATING,
            ):
                t["uiStatus"] = "INITIALIZING"
            else:
                t["uiStatus"] = "READY"

        elif mtype == "SYS_STATUS":
            v_batt = msg.voltage_battery
            batt_rem = msg.battery_remaining
            t["batteryVoltage"] = (v_batt / 1000.0 if v_batt != 65535 else None)
            t["batteryRemaining"] = (batt_rem if batt_rem != 255 else None)
            t["load"] = msg.load / 10.0

        elif mtype == "ATTITUDE":
            t["attitude"] = {
                "roll": msg.roll,
                "pitch": msg.pitch,
                "yaw": msg.yaw,
                "rollspeed": msg.rollspeed,
                "pitchspeed": msg.pitchspeed,
                "yawspeed": msg.yawspeed,
            }

        elif mtype == "GLOBAL_POSITION_INT":
            t["position"] = {
                "lat": msg.lat / 1e7,
                "lng": msg.lon / 1e7,
                "alt": msg.alt / 1000.0,
                "relAlt": msg.relative_alt / 1000.0,
                "vx": msg.vx / 100.0,
                "vy": msg.vy / 100.0,
                "vz": msg.vz / 100.0,
                "heading": None if msg.hdg == 65535 else msg.hdg / 100.0,
            }

        elif mtype == "MOUNT_ORIENTATION":
            t["gimbal"] = {
                "pitchDeg": msg.pitch,
                "rollDeg": msg.roll,
                "yawDeg": msg.yaw,
            }
            print("🎯 Gimbal telemetry MOUNT_ORIENTATION sysid", sysid)

        # Only keep telemetry for sysids that have sent heartbeat (or are sending one now)
        if sysid in valid_sysids_with_heartbeat or mtype == "HEARTBEAT":
            last_telemetry_by_sysid[sysid] = t

        # Broadcast to WS clients
        if sysid in valid_sysids_with_heartbeat and uav_clients:
            await broadcast_uav({
                "type": "telemetry",
                **t,
            })


# ------------------------
# KEEPALIVE TASK
# ------------------------
async def gcs_keepalive_task():
    """Periodically poke the drone so it keeps sending us MAVLink."""
    await asyncio.sleep(5.0)  # wait for startup
    while True:
        try:
            if mav_master is None:
                await asyncio.sleep(1.0)
                continue

            # pick a sysid, same logic as /api/test-mavlink
            if valid_sysids_with_heartbeat:
                target_sysid = sorted(valid_sysids_with_heartbeat)[0]
            else:
                target_sysid = mav_master.target_system or 1

            target_compid = mav_master.target_component or 1
            params = [float(GLOBAL_POSITION_INT_ID), 1.0, 0, 0, 0, 0, 0]

            mav_master.mav.command_long_send(
                target_sysid,
                target_compid,
                MAV_CMD_REQUEST_MESSAGE,
                0,
                *params
            )
        except Exception as e:
            print("Keepalive error:", e)

        await asyncio.sleep(2.0)  # every 2 seconds


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
# COMMAND HANDLER (from UI)
# ------------------------
def handle_command_from_ui(message: Dict[str, Any]) -> None:
    global gimbal_pitch_deg, gimbal_yaw_deg

    cmd = message.get("cmd")
    mission = message.get("mission") or {}
    sysid = message.get("sysid")

    target_sys_id = sysid or mission.get("droneSysId") or 1
    target_comp_id = 1  # autopilot

    print(f"UI Command: {cmd} → sysid={target_sys_id} extra={message}")

    MAV_CMD_DO_DIGICAM_CONTROL = 203
    MAV_CMD_VIDEO_START_CAPTURE = 250
    MAV_CMD_VIDEO_STOP_CAPTURE = 251
    MAV_CMD_SET_CAMERA_ZOOM = 531
    MAV_CMD_DO_MOUNT_CONTROL = 205

    if cmd == "TAKE_PHOTO":
        params = [0, 0, 0, 0, 1, 0, 0]
        send_command_long_pymav(target_sys_id, target_comp_id, MAV_CMD_DO_DIGICAM_CONTROL, params)

    elif cmd == "REC_START":
        params = [0, 1, 0, 0, 0, 0, 0]
        send_command_long_pymav(target_sys_id, target_comp_id, MAV_CMD_VIDEO_START_CAPTURE, params)

    elif cmd == "REC_STOP":
        params = [0, 1, 0, 0, 0, 0, 0]
        send_command_long_pymav(target_sys_id, target_comp_id, MAV_CMD_VIDEO_STOP_CAPTURE, params)

    elif cmd == "ZOOM_IN":
        params = [1, 0.05, 0, 0, 0, 0, 0]
        send_command_long_pymav(target_sys_id, 1, MAV_CMD_SET_CAMERA_ZOOM, params)

    elif cmd == "ZOOM_OUT":
        params = [1, -0.05, 0, 0, 0, 0, 0]
        send_command_long_pymav(target_sys_id, 1, MAV_CMD_SET_CAMERA_ZOOM, params)

    elif cmd in ("GIMBAL_UP", "GIMBAL_DOWN", "GIMBAL_LEFT", "GIMBAL_RIGHT", "GIMBAL_CENTER"):
        step = 5.0  # degrees per click
        if cmd == "GIMBAL_UP":
            gimbal_pitch_deg += step + 5
        elif cmd == "GIMBAL_DOWN":
            gimbal_pitch_deg -= step - 5
        elif cmd == "GIMBAL_LEFT":
            gimbal_yaw_deg -= step
        elif cmd == "GIMBAL_RIGHT":
            gimbal_yaw_deg += step
        elif cmd == "GIMBAL_CENTER":
            gimbal_pitch_deg = 0.0
            gimbal_yaw_deg = 0.0

        gimbal_pitch_deg = max(GIMBAL_PITCH_MIN, min(GIMBAL_PITCH_MAX, gimbal_pitch_deg))
        gimbal_yaw_deg = max(GIMBAL_YAW_MIN, min(GIMBAL_YAW_MAX, gimbal_yaw_deg))

        # --- 1) Classic DO_MOUNT_CONTROL style (many ArduPilot setups) ---
        mount_params = [
            gimbal_pitch_deg,  # param1: pitch (deg)
            0.0,               # param2: roll
            gimbal_yaw_deg,    # param3: yaw (deg)
            0.0, 0.0, 0.0,
            2.0,               # param7: MAV_MOUNT_MODE_MAVLINK_TARGETING
        ]
        send_command_long_pymav(target_sys_id, 1, MAV_CMD_DO_MOUNT_CONTROL, mount_params)

        # --- 2) New Gimbal Manager style (MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW) ---
        gm_params = [
            gimbal_pitch_deg,  # param1: pitch (deg)
            gimbal_yaw_deg,    # param2: yaw (deg)
            0.0,               # param3: pitch_rate (deg/s)
            0.0,               # param4: yaw_rate (deg/s)
            0.0,               # param5: flags (0 = body frame)
            0.0,               # param6: reserved
            0.0,               # param7: gimbal instance id (0 for only gimbal)
        ]
        send_command_long_pymav(target_sys_id, 1, MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, gm_params)

        print(
            f"📨 Sent gimbal cmd={cmd} pitch={gimbal_pitch_deg:.1f} "
            f"yaw={gimbal_yaw_deg:.1f} to sysid={target_sys_id}"
        )

    elif cmd in ("FPV_START", "FPV_STOP"):
        print(f"(No MAVLink sent for {cmd} — handled by video pipeline)")

    elif cmd == "MISSION_UPLOAD":
        print("🛰 Mission upload requested (still using C2-side simulation):", mission)
        # For now we keep this as C2-only "logical mission"; no real mission upload over RC
        # You can keep or extend your existing send_waypoint_sequence_to_drone if you want to simulate
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
            asyncio.create_task(
                broadcast_uav({
                    "type": "mission_ack",
                    "status": "sent",
                    "missionName": mission.get("missionName", "Untitled"),
                })
            )
    elif cmd == "ARM":
        params = [1, 0, 0, 0, 0, 0, 0]  # arm
        send_command_long_pymav(
            target_sys_id,
            1,
            MAV_CMD_COMPONENT_ARM_DISARM,  # MAV_CMD_COMPONENT_ARM_DISARM
            params
        )
        # send_command_long_pymav(target_sys_id, 1, MAV_CMD_COMPONENT_ARM_DISARM, params)

    elif cmd == "DISARM":
        params = [0, 0, 0, 0, 0, 0, 0]
        send_command_long_pymav(
            target_sys_id,
            1,
            MAV_CMD_COMPONENT_ARM_DISARM,  # MAV_CMD_COMPONENT_ARM_DISARM
            params
        )

    else:
        print("Unknown command from UI:", cmd)


def arm_disarm(master, target_sysid: Optional[int] = None, arm: bool = True, timeout_s: float = 5.0):
    """
    Send MAV_CMD_COMPONENT_ARM_DISARM to target_sysid (or choose a default).
    Blocks waiting for COMMAND_ACK (up to timeout_s).
    Returns a dict {ok: bool, result: int, text: str}
    """
    global mav_master, valid_sysids_with_heartbeat, last_telemetry_by_sysid

    if master is None:
        return {"ok": False, "error": "MAVLink master not ready"}

    # pick a target sysid if not provided
    if target_sysid is None:
        if valid_sysids_with_heartbeat:
            target_sysid = sorted(valid_sysids_with_heartbeat)[0]
        elif last_telemetry_by_sysid:
            target_sysid = sorted(last_telemetry_by_sysid.keys())[0]
        else:
            target_sysid = master.target_system or 1

    target_compid = master.target_component or 1

    # command id for arm/disarm
    CMD = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    param1 = 1.0 if arm else 0.0

    try:
        # send COMMAND_LONG
        master.mav.command_long_send(
            target_sysid,
            target_compid,
            CMD,
            0,              # confirmation
            float(param1), # param1: 1=arm, 0=disarm
            0,0,0,0,0,0
        )
    except Exception as e:
        return {"ok": False, "error": f"send failed: {e}"}

    # wait for COMMAND_ACK for this command
    start = time.time()
    while time.time() - start < timeout_s:
        try:
            ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        except Exception as e:
            # recv error — continue trying until timeout
            ack = None

        if ack is None:
            continue

        # Some implementations return .command (uint16) and .result (uint8)
        # ack.command may be the numeric command id
        try:
            if int(getattr(ack, "command", -1)) == int(CMD):
                result = int(getattr(ack, "result", -1))
                # map result codes to human text (common ones)
                result_map = {
                    0: "ACCEPTED",
                    1: "TEMPORARILY_REJECTED",
                    2: "DENIED",
                    3: "UNSUPPORTED",
                    4: "FAILED",
                    5: "IN_PROGRESS",
                }
                text = result_map.get(result, f"RESULT_{result}")
                return {"ok": result == 0, "result": result, "text": text}
        except Exception:
            # not the ack we want; continue waiting
            continue

    return {"ok": False, "error": "timeout waiting for COMMAND_ACK"}


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
        "mode": "udp+pymavlink+ws",
        "mavConn": MAV_CONN_STR,
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
      - MAVLink transport works
      - Drone responds with telemetry
    """
    global mav_master

    if mav_master is None:
        return JSONResponse(
            {"ok": False, "error": "MAVLink master not ready"},
            status_code=500,
        )

    # 1) Choose target sysid
    if sysid is None:
        # Prefer a sysid that has sent HEARTBEAT
        if valid_sysids_with_heartbeat:
            target_sysid = sorted(valid_sysids_with_heartbeat)[0]
        elif last_telemetry_by_sysid:
            target_sysid = sorted(last_telemetry_by_sysid.keys())[0]
        else:
            target_sysid = mav_master.target_system or 1
    else:
        target_sysid = sysid

    target_compid = mav_master.target_component or 1

    # 3) Build params for MAV_CMD_REQUEST_MESSAGE
    params = [
        float(GLOBAL_POSITION_INT_ID),  # param1: message id
        1.0,                            # param2: request ON (1) or OFF (0)
        0.0, 0.0, 0.0, 0.0, 0.0
    ]

    # 4) Send the command
    mav_master.mav.command_long_send(
        target_sysid,
        target_compid,
        MAV_CMD_REQUEST_MESSAGE,
        0,
        *params
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
    """
    if sysid is not None:
        try:
            target_sys_id = int(sysid)
        except (TypeError, ValueError):
            target_sys_id = 1
    else:
        if valid_sysids_with_heartbeat:
            target_sys_id = sorted(valid_sysids_with_heartbeat)[0]
        else:
            target_sys_id = 1

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
async def uav_cc_page(request: Request, user: str = Depends(get_current_user)):
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
        "sysids": sorted(valid_sysids_with_heartbeat),
    }


@app.get("/api/amr_pose")
async def get_amr_pose():
    if not current_amr_pose:
        return {"ok": False, "pose": None}
    return {"ok": True, "pose": current_amr_pose}


@app.post("/api/fpv/start")
async def fpv_start():
    global ffmpeg_thread

    if ffmpeg_thread and ffmpeg_thread.is_alive():
        return {"ok": True, "status": "already_running"}

    stop_event.clear()

    ff_cmd = [
        "ffmpeg",

        "-rtsp_transport", "tcp",

        # tolerate broken input
        "-fflags", "+discardcorrupt",
        "-err_detect", "ignore_err",

        "-i", RTSP_URL,

        # video only
        "-map", "0:v:0",
        "-an",

        # transcode to browser-safe H.264
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-pix_fmt", "yuv420p",

        # 🔽 low FPS + fast recovery
        "-r", "10",  # 10 FPS
        "-g", "10",  # keyframe every second
        "-keyint_min", "10",
        "-sc_threshold", "0",

        # HLS output
        "-f", "hls",
        "-hls_time", "1",
        "-hls_list_size", "3",
        "-hls_flags", "delete_segments+append_list",

        os.path.join(HLS_OUTPUT_DIR, "stream.m3u8"),
    ]

    ffmpeg_thread = threading.Thread(
        target=ffmpeg_watchdog,
        args=(ff_cmd,),
        daemon=True
    )
    ffmpeg_thread.start()

    return {"ok": True, "status": "started"}


@app.post("/api/fpv/stop")
async def fpv_stop():
    global ffmpeg_proc

    stop_event.set()

    if ffmpeg_proc and ffmpeg_proc.poll() is None:
        try:
            ffmpeg_proc.send_signal(signal.CTRL_BREAK_EVENT)
            time.sleep(1)
            ffmpeg_proc.kill()
        except Exception:
            pass

    ffmpeg_proc = None
    return {"ok": True, "status": "stopped"}


# ------------------------
# WEBSOCKET ROUTES
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


current_amr_pose: Dict[str, Any] = {}


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

                existing = stations.get(name)
                if existing is None:
                    stations[name] = {"x": 0, "y": 0}

                save_stations_to_file()
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
# IPC CONNECTOR (AMR)
# ------------------------
async def ipc_connector():
    """Background task: connect to IPC and listen for messages."""
    global ipc_ws, stations, current_odom_text, map_image_url, current_amr_pose

    while True:
        try:
            print(f"[IPC] Connecting to {IPC_WS_URL} ...")
            async with websockets.connect(IPC_WS_URL) as ws:
                print("[IPC] Connected.")
                async with ipc_lock:
                    ipc_ws = ws

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
                        current_amr_pose = {
                            "x": msg.get("x"),
                            "y": msg.get("y"),
                            "theta": msg.get("theta"),
                        }
                        await broadcast_amr({
                            "type": "amr_pose", **current_amr_pose,
                        })

                    elif msg_type == "zed_frame":
                        # cache latest (optional)
                        latest_zed_frame.clear()
                        latest_zed_frame.update({
                            "jpeg": msg.get("jpeg"),
                            "stamp": msg.get("stamp"),
                            "frame_id": msg.get("frame_id"),
                        })
                        # LIVE forward to all interested clients
                        await broadcast_zed_frame({
                            "type": "zed_frame",
                            **latest_zed_frame,
                        })

                    else:
                        print("[IPC] unhandled message from IPC:", msg)


        except Exception as e:
            print(f"[IPC] connection error: {e}")
        finally:
            async with ipc_lock:
                ipc_ws = None
        await asyncio.sleep(3.0)


async def broadcast_zed_frame(obj: Dict[str, Any]) -> None:
    text = json.dumps(obj)

    dead_uav = []
    dead_amr = []

    for ws in uav_clients:
        try:
            await ws.send_text(text)
        except Exception:
            dead_uav.append(ws)

    for ws in amr_clients:
        try:
            await ws.send_text(text)
        except Exception:
            dead_amr.append(ws)

    for ws in dead_uav:
        uav_clients.discard(ws)
    for ws in dead_amr:
        amr_clients.discard(ws)


# ------------------------
# APP STARTUP
# ------------------------
@app.on_event("startup")
async def startup_event():
    global mav_reader_task_handle, gcs_keepalive_task_handle, ipc_connector_task_handle, ffmpeg_thread

    load_stations_from_file()

    # Non-blocking MAVLink init
    init_mavlink()

    # Start MAVLink reader + keepalive + IPC
    mav_reader_task_handle = asyncio.create_task(mavlink_reader_task(), name="mavlink_reader")
    gcs_keepalive_task_handle = asyncio.create_task(gcs_keepalive_task(), name="gcs_keepalive")
    ipc_connector_task_handle = asyncio.create_task(ipc_connector(), name="ipc_connector")

    # Video directory + ffmpeg stuff
    os.makedirs(HLS_OUTPUT_DIR, exist_ok=True)
    ff_cmd = [
        "ffmpeg",
        "-rtsp_transport", "tcp",
        "-i", RTSP_URL,

        # video: transcode so FPS & latency can change
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-pix_fmt", "yuv420p",

        # ↓ LOWER FPS + FAST RECOVERY
        "-r", "10",  # 10 FPS
        "-g", "10",  # keyframe every second
        "-sc_threshold", "0",

        # no audio
        "-an",

        # HLS
        "-f", "hls",
        "-hls_time", "1",
        "-hls_list_size", "3",
        "-hls_flags", "delete_segments+append_list",

        os.path.join(HLS_OUTPUT_DIR, "stream.m3u8"),
    ]

    try:
        ffmpeg_thread = threading.Thread(
            target=ffmpeg_watchdog,
            args=(ff_cmd,),
            daemon=True
        )
        ffmpeg_thread.start()
    except Exception as e:
        print("⚠️ Failed to start ffmpeg:", e)

    asyncio.create_task(ipc_connector())
    asyncio.create_task(gcs_keepalive_task())


@app.on_event("shutdown")
async def shutdown_event():
    global ffmpeg_proc, mav_reader_task_handle, gcs_keepalive_task_handle, ipc_connector_task_handle, ipc_ws, mav_master
    stop_event.set()

    global ffmpeg_running, ffmpeg_proc

    print("🛑 shutting down ffmpeg")

    ffmpeg_running = False

    if ffmpeg_proc and ffmpeg_proc.poll() is None:
        try:
            ffmpeg_proc.terminate()
            ffmpeg_proc.wait(timeout=3)
        except Exception:
            ffmpeg_proc.kill()

    for task in (mav_reader_task_handle, gcs_keepalive_task_handle, ipc_connector_task_handle):
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    # Close IPC websocket if open
    if ipc_ws is not None:
        try:
            await ipc_ws.close()
        except Exception:
            pass
        ipc_ws = None

    # Close MAVLink connection
    if mav_master is not None:
        try:
            mav_master.close()
        except Exception:
            pass
        mav_master = None

        if ffmpeg_proc is not None:
            try:
                ffmpeg_proc.terminate()
            except Exception:
                pass
            ffmpeg_proc = None

    print("[APP] Shutdown complete.")
