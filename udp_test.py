import socket
import time

UDP_PORT = 14550  # try this first

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.settimeout(1.0)

print(f"Listening on UDP 0.0.0.0:{UDP_PORT} ... (Ctrl+C to stop)")

try:
    while True:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue

        print(f"[{time.strftime('%H:%M:%S')}] Got {len(data)} bytes from {addr}, first byte=0x{data[0]:02X}")
except KeyboardInterrupt:
    print("\nStopping listener...")
finally:
    sock.close()
