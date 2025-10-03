# 2_backend.py
# Serve 3_manual.html + xử lý socket.io
# Gửi PWM -> serial_bridge (TCP) + MAVLink RC_OVERRIDE -> SITL

import os
import time
import socket
import threading
from flask import Flask, send_file
from flask_socketio import SocketIO
from pymavlink import mavutil

# ---------- Flask + SocketIO ----------
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ---------- CẤU HÌNH ----------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# serial_bridge (có thể nối ra Arduino / in console)
SERIAL_BRIDGE_IP = "127.0.0.1"
SERIAL_BRIDGE_PORT = 62000

# PWM mapping
PWM_MIN = 1100
PWM_MAX = 1900
PWM_MID = 1500

# MAVLink (gửi thẳng SITL)
MAVLINK_UDP = "udpout:127.0.0.1:14550"
master = mavutil.mavlink_connection(MAVLINK_UDP)

# ---------- TCP client tới serial_bridge ----------
tcp = None
tcp_lock = threading.Lock()

def tcp_connect_loop():
    global tcp
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.connect((SERIAL_BRIDGE_IP, SERIAL_BRIDGE_PORT))
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with tcp_lock:
                tcp = s
            print("Connected to serial_bridge at", SERIAL_BRIDGE_IP, SERIAL_BRIDGE_PORT)
            return
        except Exception as e:
            print("tcp connect failed:", e, "-> retry in 2s")
            time.sleep(2)

threading.Thread(target=tcp_connect_loop, daemon=True).start()

# ---------- trạng thái ----------
control_enabled = False
kill_active = False

# ---------- helper ----------
def norm_to_pwm(v):
    v = max(-1.0, min(1.0, float(v)))
    span = (PWM_MAX - PWM_MIN) / 2.0
    return int(PWM_MID + v * span)

def send_to_bridge(pwm_l, pwm_r):
    global tcp
    msg = f"{int(pwm_l)},{int(pwm_r)}\n".encode()
    with tcp_lock:
        if tcp is None:
            print("No TCP connection to bridge; dropping:", msg)
            return False
        try:
            tcp.sendall(msg)
            return True
        except Exception as e:
            print("Send failed:", e)
            try:
                tcp.close()
            except:
                pass
            tcp = None
            threading.Thread(target=tcp_connect_loop, daemon=True).start()
            return False

def send_to_mavlink(pwm_l, pwm_r):
    try:
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            pwm_l,  # chan1
            pwm_r,  # chan2
            1500,   # chan3 neutral
            1500,   # chan4 neutral
            0,0,0,0
        )
        return True
    except Exception as e:
        print("MAVLink send failed:", e)
        return False

# ---------- serve 3_manual.html ----------
@app.route("/")
def index():
    return send_file(os.path.join(BASE_DIR, "3_manual.html"))

# ---------- socket.io events ----------
@socketio.on("toggle_control")
def on_toggle(data):
    global control_enabled, kill_active
    enabled = bool(data.get("enabled", False))
    if kill_active and enabled:
        control_enabled = False
        print("Ignored enable request while kill_active")
        return
    control_enabled = enabled
    print("Control enabled =", control_enabled)

@socketio.on("kill")
def on_kill(data):
    global kill_active, control_enabled
    kill_active = True
    control_enabled = False
    print("KILL activated -> neutral PWM")
    send_to_bridge(PWM_MID, PWM_MID)
    send_to_mavlink(PWM_MID, PWM_MID)

@socketio.on("unkill")
def on_unkill(data):
    global kill_active, control_enabled
    kill_active = False
    control_enabled = True
    print("KILL deactivated -> control enabled")

@socketio.on("dual_cmd")
def on_dual_cmd(data):
    global control_enabled, kill_active
    if not control_enabled or kill_active:
        return
    try:
        left = float(data.get("left", 0.0))
        right = float(data.get("right", 0.0))
    except Exception:
        return
    pwm_l = norm_to_pwm(left)
    pwm_r = norm_to_pwm(right)
    sent_bridge = send_to_bridge(pwm_l, pwm_r)
    sent_mav = send_to_mavlink(pwm_l, pwm_r)
    print(f"DUAL_CMD: left={left:.2f} right={right:.2f} -> pwm {pwm_l},{pwm_r} "
          f"bridge={sent_bridge} mav={sent_mav}")

# health check
@app.route("/health")
def health():
    return "OK"

if __name__ == "__main__":
    print("Starting backend on 0.0.0.0:5000 (serving 3_manual.html)")
    socketio.run(app, host="0.0.0.0", port=5000)






# # 2_backend.py
# # Mô tả: backend serve file 3_manual.html và xử lý socket.io
# # Nhận event 'dual_cmd' (left,right normalized -1..1), 'toggle_control', 'kill', 'unkill'
# # Ánh xạ normalized -> PWM và gửi "pwm_l,pwm_r\n" tới serial_bridge (TCP).

# import os
# import time
# import socket
# import threading
# from flask import Flask, send_from_directory
# from flask_socketio import SocketIO

# # ---------- Flask + SocketIO ----------
# app = Flask(__name__)
# socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# # ---------- CẤU HÌNH (mặc định cho LAPTOP/VM) ----------
# BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# SERIAL_BRIDGE_IP = "127.0.0.1"   # serial_bridge trên cùng máy
# SERIAL_BRIDGE_PORT = 62000
# PWM_MIN = 1100
# PWM_MAX = 1900
# PWM_MID = 1500

# # Serve frontend (file 3_manual.html đặt cùng thư mục)
# @app.route("/")
# def index():
#     return send_from_directory(BASE_DIR, "3_manual.html")

# # ---------- TCP client tới serial_bridge ----------
# tcp = None
# tcp_lock = threading.Lock()

# def tcp_connect_loop():
#     """Cố gắng kết nối tới serial_bridge (retry nếu chưa có)."""
#     global tcp
#     while True:
#         try:
#             s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#             s.connect((SERIAL_BRIDGE_IP, SERIAL_BRIDGE_PORT))
#             s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
#             with tcp_lock:
#                 tcp = s
#             print("Connected to serial_bridge at", SERIAL_BRIDGE_IP, SERIAL_BRIDGE_PORT)
#             return
#         except Exception as e:
#             print("tcp connect failed:", e, "-> retry in 2s")
#             time.sleep(2)

# # start connect thread
# threading.Thread(target=tcp_connect_loop, daemon=True).start()

# # ---------- trạng thái ----------
# control_enabled = False
# kill_active = False

# # ---------- helper ----------
# def norm_to_pwm(v):
#     """Map -1..1 -> PWM (int)."""
#     v = max(-1.0, min(1.0, float(v)))
#     span = (PWM_MAX - PWM_MIN) / 2.0
#     return int(PWM_MID + v * span)

# def send_to_bridge(pwm_l, pwm_r):
#     """Gửi chuỗi 'pwm_l,pwm_r\\n' tới serial_bridge qua TCP."""
#     global tcp
#     msg = f"{int(pwm_l)},{int(pwm_r)}\n".encode()
#     with tcp_lock:
#         if tcp is None:
#             print("No TCP connection to bridge; dropping:", msg)
#             return False
#         try:
#             tcp.sendall(msg)
#             return True
#         except Exception as e:
#             print("Send failed:", e)
#             try:
#                 tcp.close()
#             except:
#                 pass
#             tcp = None
#             threading.Thread(target=tcp_connect_loop, daemon=True).start()
#             return False

# # ---------- socket.io events ----------
# @socketio.on("toggle_control")
# def on_toggle(data):
#     global control_enabled, kill_active
#     enabled = bool(data.get("enabled", False))
#     if kill_active and enabled:
#         control_enabled = False
#         print("Ignored enable request while kill_active")
#         return
#     control_enabled = enabled
#     print("Control enabled =", control_enabled)

# @socketio.on("kill")
# def on_kill(data):
#     global kill_active, control_enabled
#     kill_active = True
#     control_enabled = False
#     print("KILL activated -> sending neutral PWM")
#     send_to_bridge(PWM_MID, PWM_MID)

# @socketio.on("unkill")
# def on_unkill(data):
#     global kill_active, control_enabled
#     kill_active = False
#     control_enabled = True   # theo yêu cầu: unkill sẽ bật control
#     print("KILL deactivated -> control enabled")

# @socketio.on("dual_cmd")
# def on_dual_cmd(data):
#     """Nhận left/right (-1..1). Nếu control enabled và không kill, gửi PWM."""
#     global control_enabled, kill_active
#     if not control_enabled or kill_active:
#         return
#     try:
#         left = float(data.get("left", 0.0))
#         right = float(data.get("right", 0.0))
#     except Exception:
#         return
#     pwm_l = norm_to_pwm(left)
#     pwm_r = norm_to_pwm(right)
#     sent = send_to_bridge(pwm_l, pwm_r)
#     print(f"DUAL_CMD: left={left:.3f} right={right:.3f} -> pwm {pwm_l},{pwm_r} sent={sent}")

# # health check
# @app.route("/health")
# def health():
#     return "OK"

# if __name__ == "__main__":
#     print("Starting backend on 0.0.0.0:5000 (serving 3_manual.html)")
#     socketio.run(app, host="0.0.0.0", port=5000)

# # ---------- CHÚ THÍCH NGẮN (dùng điện thoại) ----------
# # Nếu bạn muốn mở frontend từ điện thoại:
# # 1) Trên điện thoại mở: http://<UBUNTU_IP>:5000  (thay <UBUNTU_IP> bằng IP của máy Ubuntu)
# # 2) Nếu serial_bridge không ở localhost, chỉnh SERIAL_BRIDGE_IP cho phù hợp.
# # 3) Mở port 5000 trên firewall nếu cần.
