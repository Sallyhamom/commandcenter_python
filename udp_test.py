from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import time

CUBE_IP = "192.168.144.12"
CUBE_PORT = 19856
CONN_STR = f"udpout:{CUBE_IP}:{CUBE_PORT}"


def connect():
    print(f"[GCS] Connecting to {CONN_STR} ...")
    master = mavutil.mavlink_connection(CONN_STR, source_system=255)

    # 🔑 IMPORTANT: kick the UDP socket once (this is what made it work for you)
    print("[GCS] Sending initial heartbeat to open UDP socket...")
    master.mav.heartbeat_send(
        0,  # type
        0,  # autopilot
        0,  # base_mode
        0,  # custom_mode
        0,  # system_status
    )
    time.sleep(0.1)

    print("[GCS] Waiting for HEARTBEAT from Cube...")
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=15)

    if hb is None:
        raise TimeoutError("❌ No HEARTBEAT received (timeout)")

    print(f"✅ HEARTBEAT from sysid={master.target_system}, compid={master.target_component}")
    print(hb)
    return master


# -------- Telemetry --------

def telemetry_loop(master):
    """
    Simple telemetry printer. You can instead push these values into your UI.
    """
    while True:
        msg = master.recv_match(blocking=True, timeout=5)
        if msg is None:
            print("[TEL] No MAVLink for 5s...")
            continue

        mtype = msg.get_type()

        if mtype == "ATTITUDE":
            print(f"[ATT] roll={msg.roll:.2f}, pitch={msg.pitch:.2f}, yaw={msg.yaw:.2f}")

        elif mtype == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            print(f"[GPS] lat={lat:.7f}, lon={lon:.7f}, alt={alt:.1f}m")

        elif mtype == "SYS_STATUS":
            v = msg.voltage_battery / 1000.0
            print(f"[BAT] {v:.2f} V (remaining={msg.battery_remaining}%)")

        # Add more message types if you want (VFR_HUD, RC_CHANNELS, etc.)


# -------- Gimbal control --------

def gimbal_move(master, pitch_deg=0.0, roll_deg=0.0, yaw_deg=0.0):
    """
    Move gimbal using both DO_MOUNT_CONTROL and DO_GIMBAL_MANAGER_PITCHYAW.
    Angles in degrees, as used by ArduPilot.
    """
    sysid = master.target_system or 1
    # ArduPilot autopilot component is usually 1 or 0; both are accepted.
    compid = master.target_component or 1

    print(f"[GIMBAL] cmd: pitch={pitch_deg}, roll={roll_deg}, yaw={yaw_deg}")

    # 1) Legacy DO_MOUNT_CONTROL (205)
    master.mav.command_long_send(
        sysid,
        compid,
        mavlink2.MAV_CMD_DO_MOUNT_CONTROL,
        0,                  # confirmation
        float(pitch_deg),   # param1: pitch (deg)
        float(roll_deg),    # param2: roll (deg)
        float(yaw_deg),     # param3: yaw (deg)
        0.0,                # param4
        0.0,                # param5
        0.0,                # param6
        2.0,                # param7: MAV_MOUNT_MODE_MAVLINK_TARGETING
    )

    # 2) New Gimbal Manager DO_GIMBAL_MANAGER_PITCHYAW (1000)
    master.mav.command_long_send(
        sysid,
        compid,
        mavlink2.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        0,                  # confirmation
        float(pitch_deg),   # param1: pitch (deg, +up / -down per docs)
        float(yaw_deg),     # param2: yaw (deg)
        0.0,                # param3: pitch rate (deg/s)
        0.0,                # param4: yaw rate (deg/s)
        0.0,                # param5: flags (0=body frame)
        0.0,                # param6: reserved
        0.0,                # param7: gimbal instance id (0 = primary)
    )


def gimbal_center(master):
    """Convenience: center gimbal."""
    gimbal_move(master, pitch_deg=0.0, roll_deg=0.0, yaw_deg=0.0)


import time
from typing import Optional

# assume mav_master is your global pymavlink connection (mavutil.mavlink_connection)
# e.g. mav_master = mavutil.mavlink_connection(...)


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


# -------- MAIN --------

if __name__ == "__main__":
    master = connect()

    # Example test moves (comment out when wiring to UI)
    # gimbal_move(master, pitch_deg=10)   # tilt down 10°
    # time.sleep(1)
    # gimbal_move(master, pitch_deg=-10)  # tilt up 10°
    # time.sleep(1)
    # gimbal_center(master)
    res = arm_disarm(master, target_sysid=1, arm=True)
    print(res)
    time.sleep(10)
    arm_disarm(master, target_sysid=1, arm=False)
    print("[GCS] Listening telemetry... (Ctrl+C to stop)")
    try:
        # telemetry_loop(master)
        gimbal_move(master, pitch_deg=10)  # tilt down 10°
        time.sleep(5)
        gimbal_move(master, pitch_deg=-10)  # tilt up 10°
        # time.sleep(1)
        # gimbal_center(master)
    except KeyboardInterrupt:
        print("\n[GCS] Exiting...")
