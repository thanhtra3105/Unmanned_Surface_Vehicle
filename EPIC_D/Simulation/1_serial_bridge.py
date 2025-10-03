# 1_serial_bridge.py
# Mô tả ngắn: TCP server nhận chuỗi "pwm_l,pwm_r\n" từ backend,
# chuyển thành rc_channels_override gửi tới SITL (MAVLink).
#
# Mặc định: SITL (ArduSub) chạy trên máy này, lắng nghe UDP 127.0.0.1:14550
# Nếu SITL bạn dùng port/địa chỉ khác, sửa MAV_CONN phía dưới.

# mkdir -p ~/code_PBL3
# cp /media/sf_code_PBL3/{1_serial_bridge.py,2_backend.py,3_manual.html} ~/code_PBL3/
# ls -l ~/code_PBL3/


import socket
import sys
from pymavlink import mavutil

# ---------- CẤU HÌNH (mặc định cho laptop/VM) ----------
TCP_IP = "127.0.0.1"        # serial bridge chỉ lắng nghe localhost
TCP_PORT = 62000
# ArduSub SITL mở cổng TCP 5760
#MAV_CONN = "udp:127.0.0.1:14550"   # SITL mặc định
# MAV_CONN = "tcp:127.0.0.1:5760"
MAV_CONN = "udp:127.0.0.1:14550"
# ---------- KẾT NỐI TỚI SITL ----------

print("Connecting to SITL at", MAV_CONN)
master = mavutil.mavlink_connection(MAV_CONN,source_system=255,autoreconnect=True)
print("Waiting for heartbeat from SITL...")
master.wait_heartbeat()
print("Heartbeat received: system %u component %u" % (master.target_system, master.target_component))

# ---------- START TCP SERVER ----------
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
print("Serial bridge listening on", TCP_IP, "port", TCP_PORT)
conn, addr = s.accept()
print("Connection from", addr)

buffer = b""
try:
    while True:
        data = conn.recv(1024)
        if not data:
            break
        buffer += data
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            text = line.decode().strip()
            if not text:
                continue
            parts = text.split(",")
            if len(parts) < 2:
                print("Bad payload:", text)
                continue
            try:
                pwm_l = int(parts[0])
                pwm_r = int(parts[1])
            except Exception as e:
                print("Parse int error:", parts, e)
                continue

            # NOTE: rc_channels_override_send signature:
            # master.mav.rc_channels_override_send(target_system, target_component,
            #    chan1, chan2, chan3, chan4, chan5, chan6, chan7, chan8)
            # Ở đây mình đặt pwm_l -> chan3, pwm_r -> chan4 (thử nếu SITL không phản hồi thì đổi vị trí)
            try:
                master.mav.rc_channels_override_send(
                    master.target_system,
                    master.target_component,
                    0, 0, pwm_l, pwm_r, 0, 0, 0, 0
                )
                print(f"Bridge -> PWM left={pwm_l} right={pwm_r}")
            except Exception as e:
                print("rc override send error:", e)

except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    try:
        conn.close()
    except:
        pass
    s.close()
    sys.exit(0)

# ---------- CHÚ THÍCH NGẮN ----------
# - Nếu bạn dùng phần cứng ngoài SITL: MAV_CONN có thể là serial, ví dụ "serial:/dev/ttyUSB0:57600"
# - Nếu rc_channels_override không làm động cơ hoạt động: thử thay vị trí pwm_l/pwm_r vào chan1/chan2...
