from socket import socket

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, Depends
from fastapi.responses import JSONResponse, FileResponse, HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
import asyncio
from typing import Dict, Any, List, Set, Tuple, Optional
import os
from datetime import datetime
import json

import mavlink_service as mav
import ipc_service as ipc
import video_service as video
from mqtt_ws_bridge import MQTTWebSocketBridge


app = FastAPI()

FRONTEND_DIR = os.path.join(os.path.dirname(__file__), "mission-control-frontend")
if os.path.isdir(FRONTEND_DIR):
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")

app.mount("/hls", StaticFiles(directory=video.HLS_OUTPUT_DIR), name="hls")


bridge = MQTTWebSocketBridge(
    broker="192.168.1.3",
    topics=[
        "/api/map",
        "/api/odom",
        "/api/job/feedback",
        "/api/job/result",
        "/api/map_response",
        "/api/station/list"
    ]
)

TOPIC_MAP = {
    "cmd_vel": "/api/cmd_vel",
    "estop": "/api/estop",
    "job_goal": "/api/job/goal",
    "job_cancel": "/api/job/cancel",
    "map_request": "/api/map_request",
    "station_list_request": "/api/station/list/request",
    "station_delete": "/api/station/delete"
}


STATION_FILE = "stations.json"


# ---------- helpers ----------
def load_stations():
    if os.path.exists(STATION_FILE):
        with open(STATION_FILE, "r") as f:
            data = json.load(f)

            # ensure list
            if isinstance(data, list):
                return data
            elif isinstance(data, dict):
                return list(data.values())

    return []


def save_stations(stations):
    with open(STATION_FILE, "w") as f:
        json.dump(stations, f, indent=2)


def get_current_user(request: Request):
    return "admin"


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

# ---------- STARTUP ----------
@app.on_event("startup")
async def startup():
    """mav.init_mavlink()

    asyncio.create_task(mav.mavlink_reader_task())
    asyncio.create_task(mav.keepalive_task())
    asyncio.create_task(ipc.ipc_connector())"""
    loop = asyncio.get_running_loop()
    bridge.start(loop)


# ---------- UAV WS ----------
@app.websocket("/ws")
async def uav_ws(ws: WebSocket):
    await ws.accept()
    mav.uav_clients.add(ws)

    try:
        while True:
            msg = await ws.receive_json()
            await mav.handle_uav_command(msg)

    except WebSocketDisconnect:
        print("UAV client disconnected")

    except Exception as e:
        print("WS error:", e)

    finally:
        mav.uav_clients.discard(ws)


# ---------- AMR WS ----------
@app.websocket("/ws/amr_stations")
async def amr_ws(ws: WebSocket):
    await ws.accept()
    ipc.amr_clients.add(ws)

    try:
        while True:
            msg = await ws.receive_json()
            await ipc.ipc_send(msg)

    except WebSocketDisconnect:
        print("AMR client disconnected")

    except Exception as e:
        print("WS error:", e)

    finally:
        ipc.amr_clients.discard(ws)


@app.websocket("/ws/mqtt_ugv")
async def mqtt_ws(ws: WebSocket):
    await ws.accept()
    await bridge.register(ws)

    print("MQTT WS client connected")
    try:
        while True:
            data = await ws.receive_text()
            msg = json.loads(data)

            msg_type = msg.get("type")

            # -------- ROUTE TO MQTT --------
            if msg_type in TOPIC_MAP:
                topic = TOPIC_MAP[msg_type]

                # remove "type" before sending
                payload = {k: v for k, v in msg.items() if k != "type"}

                bridge.mqtt_client.publish(
                    topic,
                    json.dumps(payload)
                )

    except WebSocketDisconnect:
        print("️MQTT client disconnected")

    except Exception as e:
        print("WS error:", e)

    finally:
        print("MQTT WS client disconnected")
        await bridge.unregister(ws)


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
        "mavConn": mav.MAV_CONN_STR,
    }


@app.post("/api/missions")
async def save_mission(mission: Dict[str, Any]):
    mission = mission or {}
    mission_id = len(mav.missions) + 1
    mission["id"] = mission_id
    mission["createdAt"] = datetime.utcnow().isoformat() + "Z"
    mav.missions.append(mission)
    print("Mission saved:", mission.get("missionName", "Untitled"), "status:", mission.get("status"))
    return {"ok": True, "mission": mission}


@app.post("/api/addnotif")
async def add_notification(n: Dict[str, Any]):
    n = n or {}
    n_id = len(mav.notifications) + 1
    n["id"] = n_id
    n["createdAt"] = datetime.utcnow().isoformat() + "Z"
    mav.notifications.insert(0, n)
    print("Notification:", n.get("message"))
    return {"ok": True}


@app.post("/api/login")
async def login(body: Dict[str, Any]):
    username = body.get("username")
    password = body.get("password")
    print(password)
    if username == "admin" and password == "V1n1maya":
        return RedirectResponse(url="/amr-cc", status_code=200)
    return JSONResponse({"error": "Invalid credentials"}, status_code=401)


@app.get("/menu", response_class=HTMLResponse)
async def menu_page():
    return FileResponse(os.path.join(FRONTEND_DIR, "menu.html"))


@app.get("/amr-cc", response_class=HTMLResponse)
async def amr_stations_page(request: Request, user: str = Depends(get_current_user)):
    return FileResponse(os.path.join(FRONTEND_DIR, "direction.html"))


@app.get("/uav-cc", response_class=HTMLResponse)
async def uav_cc_page(request: Request, user: str = Depends(get_current_user)):
    return FileResponse(os.path.join(FRONTEND_DIR, "commandcenter.html"))


@app.get("/ugv-cc", response_class=HTMLResponse)
async def uav_cc_page(request: Request, user: str = Depends(get_current_user)):
    return FileResponse(os.path.join(FRONTEND_DIR, "ugv.html"))


@app.get("/api/drones")
async def list_drones():
    drones = []
    for sysid, t in mav.last_telemetry_by_sysid.items():
        if sysid not in mav.valid_sysids_with_heartbeat:
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
    return mav.get_status()


@app.get("/api/amr_pose")
async def get_amr_pose():
    if not ipc.current_amr_pose:
        return {"ok": False, "pose": None}
    return {"ok": True, "pose": ipc.current_amr_pose}


# ---------- GET ALL ----------
@app.get("/stations")
def get_stations():
    stations = load_stations()

    # ensure it's always a list
    if isinstance(stations, dict):
        stations = list(stations.values())

    return {"stations": stations}


# ---------- ADD ----------
@app.get("/stations/add")
def add_station(name: str, x: float, y: float, theta: float):

    stations = load_stations()

    # prevent duplicates
    if any(s["name"] == name for s in stations):
        return {"status": "exists"}

    stations.append({
        "name": name,
        "x": x,
        "y": y,
        "theta": theta
    })

    save_stations(stations)

    return {"status": "added"}


# ---------- DELETE ----------
@app.get("/stations/delete")
def delete_station(name: str):

    stations = load_stations()

    stations = [s for s in stations if s["name"] != name]

    save_stations(stations)

    return {"status": "deleted"}