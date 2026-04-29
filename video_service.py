import subprocess
import threading
import time
import os
import signal

RTSP_URL = "rtsp://192.168.144.26:8554/main.264"
HLS_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "hls")

ffmpeg_proc = None
ffmpeg_thread = None
ffmpeg_running = False
stop_event = threading.Event()


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


def start_fpv():
    global ffmpeg_thread

    if ffmpeg_thread and ffmpeg_thread.is_alive():
        return {"ok": True, "status": "already_running"}

    os.makedirs(HLS_OUTPUT_DIR, exist_ok=True)
    stop_event.clear()

    ff_cmd = [
        "ffmpeg",
        "-rtsp_transport", "tcp",
        "-i", RTSP_URL,

        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-pix_fmt", "yuv420p",

        "-r", "10",
        "-g", "10",
        "-sc_threshold", "0",

        "-an",

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


def stop_fpv():
    global ffmpeg_proc, ffmpeg_running

    ffmpeg_running = False
    stop_event.set()

    if ffmpeg_proc and ffmpeg_proc.poll() is None:
        try:
            ffmpeg_proc.terminate()
            ffmpeg_proc.wait(timeout=3)
        except Exception:
            ffmpeg_proc.kill()

    ffmpeg_proc = None
    return {"ok": True, "status": "stopped"}