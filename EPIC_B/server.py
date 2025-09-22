'''
from flask import Flask, request, jsonify
from pymavlink import mavutil
import os, time

app = Flask(__name__)

# Config SITL / Vehicle
VEHICLE_ADDR = os.environ.get("VEHICLE_ADDR", "127.0.0.1")
VEHICLE_PORT = int(os.environ.get("VEHICLE_PORT", "14550"))
VEHICLE_PROTO = os.environ.get("VEHICLE_PROTO", "udp")  # udp hoặc tcp
ACK_TIMEOUT = 10

def connect_to_vehicle(addr, port, proto="udp", timeout=10):
    target = f"{proto}:{addr}:{port}"
    print(f"🔌 Connecting to {target}")
    master = mavutil.mavlink_connection(target, timeout=timeout)
    hb = master.wait_heartbeat(timeout=timeout)
    if not hb:
        raise Exception("❌ Không nhận được HEARTBEAT")
    print(f"✅ Heartbeat from system: {master.target_system}, component: {master.target_component}")
    return master

def send_mission_via_mavlink(master, mission):
    wp_count = len(mission)
    # Clear mission cũ
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    time.sleep(1)

    # Gửi mission_count
    master.mav.mission_count_send(master.target_system, master.target_component, wp_count)
    print(f"📤 Sending MISSION_COUNT={wp_count}")


    # Gửi từng waypoint
    for seq, wp in enumerate(mission):
        lat = int(wp["lat"] * 1e7)
        lon = int(wp["lng"] * 1e7)
        alt = 0
        hold_time = wp.get("hold_time", 0)

        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,          # current, autocontinue
            hold_time, 0, 0, 0,  # params
            lat, lon, alt
        )
        print(f"   ✅ Sent WP {seq+1}: lat={wp['lat']}, lon={wp['lng']}")
        time.sleep(0.5)
    
    # Chờ ACK
    msg = master.recv_match(type='MISSION_ACK', timeout=ACK_TIMEOUT)
    if msg:
        print(f"🎉 Got final MISSION_ACK: {msg}")
        return {"success": True, "message": "Mission upload complete", "ack": msg.to_dict()}
    else:
        return {"success": False, "message": "No MISSION_ACK received"}

@app.route("/", methods=["GET"])
def home():
    return "✅ USV Mission Uploader API is running"

@app.route("/upload-mission", methods=["POST"])
def upload_mission():
    data = request.get_json()
    if not data or "mission" not in data:
        return jsonify({"success": False, "message": "Missing mission in payload"}), 400

    mission = data["mission"]

    try:
        master = connect_to_vehicle(VEHICLE_ADDR, VEHICLE_PORT, VEHICLE_PROTO)
        res = send_mission_via_mavlink(master, mission)
        master.close()
        return jsonify(res), (200 if res["success"] else 500)
    except Exception as e:
        return jsonify({"success": False, "message": f"Exception: {e}"}), 503

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
'''


from flask import Flask, request, jsonify, render_template
from pymavlink import mavutil
import os, time

app = Flask(__name__)

# =============================
# Config SITL / Vehicle
# =============================
VEHICLE_ADDR = os.environ.get("VEHICLE_ADDR", "127.0.0.1")
VEHICLE_PORT = int(os.environ.get("VEHICLE_PORT", "14550"))
VEHICLE_PROTO = os.environ.get("VEHICLE_PROTO", "udp")  # udp hoặc tcp
ACK_TIMEOUT = 10


# =============================
# Hàm kết nối MAVLink
# =============================
def connect_to_vehicle(addr, port, proto="udp", timeout=10):
    target = f"{proto}:{addr}:{port}"
    print(f"🔌 Connecting to {target}")
    master = mavutil.mavlink_connection(target, timeout=timeout)
    hb = master.wait_heartbeat(timeout=timeout)
    if not hb:
        raise Exception("❌ Không nhận được HEARTBEAT")
    print(f"✅ Heartbeat from system: {master.target_system}, component: {master.target_component}")
    return master


# =============================
# Upload mission
# =============================
def send_mission_via_mavlink(master, mission):
    wp_count = len(mission)
    if wp_count == 0:
        return {"success": False, "message": "Mission trống"}

    # Clear mission cũ
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    time.sleep(1)

    # Gửi mission_count
    master.mav.mission_count_send(master.target_system, master.target_component, wp_count)
    print(f"📤 Sending MISSION_COUNT={wp_count}")

    # Gửi từng waypoint
    for seq, wp in enumerate(mission):
        lat = int(wp["lat"] * 1e7)
        lon = int(wp["lng"] * 1e7)
        alt = 0
        hold_time = wp.get("hold_time", 0)

        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,          # current, autocontinue
            hold_time, 0, 0, 0,  # params
            lat, lon, alt
        )
        print(f"   ✅ Sent WP {seq+1}: lat={wp['lat']}, lon={wp['lng']}")
        time.sleep(0.5)

    # Chờ ACK
    msg = master.recv_match(type='MISSION_ACK', timeout=ACK_TIMEOUT)
    if msg:
        print(f"🎉 Got final MISSION_ACK: {msg}")
        return {"success": True, "message": "Mission upload complete", "ack": msg.to_dict()}
    else:
        return {"success": False, "message": "No MISSION_ACK received"}


# =============================
# API endpoints
# =============================
@app.route("/", methods=["GET"])
def home():
    return render_template("index.html")

# @app.route("/upload-mission", methods=["POST"])
# def upload_mission():
#     data = request.get_json()
#     if not data or "mission" not in data:
#         return jsonify({"success": False, "message": "Missing mission in payload"}), 400

#     mission = data["mission"]
#     try:
#         master = connect_to_vehicle(VEHICLE_ADDR, VEHICLE_PORT, VEHICLE_PROTO)
#         res = send_mission_via_mavlink(master, mission)
#         master.close()
#         return jsonify(res), (200 if res["success"] else 500)
#     except Exception as e:
#         return jsonify({"success": False, "message": f"Exception: {e}"}), 503

@app.route("/upload-mission", methods=["POST"])
def upload_mission():
    data = request.get_json()
    if not data or "mission" not in data:
        return jsonify({"success": False, "message": "Missing mission in payload"}), 400

    mission = data["mission"]

    # 👉 Nhân đôi waypoint đầu tiên
    if len(mission) > 0:
        first_wp = mission[0].copy()
        mission.insert(0, first_wp)
        print(f"🔁 Added duplicate first WP: {first_wp}")

    try:
        master = connect_to_vehicle(VEHICLE_ADDR, VEHICLE_PORT, VEHICLE_PROTO)
        res = send_mission_via_mavlink(master, mission)
        master.close()
        return jsonify(res), (200 if res["success"] else 500)
    except Exception as e:
        return jsonify({"success": False, "message": f"Exception: {e}"}), 503


@app.route("/arm", methods=["POST"])
def arm_vehicle():
    try:
        master = connect_to_vehicle(VEHICLE_ADDR, VEHICLE_PORT, VEHICLE_PROTO)
        master.arducopter_arm()
        master.motors_armed_wait()
        master.close()
        return jsonify({"success": True, "message": "Vehicle armed"})
    except Exception as e:
        return jsonify({"success": False, "message": f"Arm failed: {e}"}), 500


@app.route("/disarm", methods=["POST"])
def disarm_vehicle():
    try:
        master = connect_to_vehicle(VEHICLE_ADDR, VEHICLE_PORT, VEHICLE_PROTO)
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        master.close()
        return jsonify({"success": True, "message": "Vehicle disarmed"})
    except Exception as e:
        return jsonify({"success": False, "message": f"Disarm failed: {e}"}), 500


@app.route("/set-mode", methods=["POST"])
def set_mode():
    data = request.get_json()
    if not data or "mode" not in data:
        return jsonify({"success": False, "message": "Missing mode"}), 400

    mode = data["mode"].upper()
    try:
        master = connect_to_vehicle(VEHICLE_ADDR, VEHICLE_PORT, VEHICLE_PROTO)
        master.set_mode(mode)
        master.close()
        return jsonify({"success": True, "message": f"Mode set to {mode}"})
    except Exception as e:
        return jsonify({"success": False, "message": f"Set mode failed: {e}"}), 500


@app.route("/status", methods=["GET"])
def get_status():
    try:
        master = connect_to_vehicle(VEHICLE_ADDR, VEHICLE_PORT, VEHICLE_PROTO)
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        batt = master.recv_match(type="BATTERY_STATUS", blocking=True, timeout=5)
        master.close()
        return jsonify({
            "success": True,
            "system": hb.to_dict() if hb else {},
            "battery": batt.to_dict() if batt else {}
        })
    except Exception as e:
        return jsonify({"success": False, "message": f"Status failed: {e}"}), 500

@app.route("/start-mission", methods=["POST"])
def start_mission():
    try:
        master = connect_to_vehicle(VEHICLE_ADDR, VEHICLE_PORT, VEHICLE_PROTO)

        # ARM
        master.arducopter_arm()
        master.motors_armed_wait()
        print("✅ Vehicle armed")

        # Reset mission về WP1
        master.mav.mission_set_current_send(
            master.target_system,
            master.target_component,
            1
        )
        print("📌 Mission set to start from waypoint 1")

        # Trick: chuyển mode tạm sang GUIDED trước
        master.set_mode("GUIDED")
        time.sleep(1)
        print("🔄 Mode set to GUIDED (reset)")

        # Sau đó chuyển lại sang AUTO
        master.set_mode("AUTO")
        print("✅ Mode set to AUTO")

        # Gửi lệnh start mission
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("🚀 Mission started")

        master.close()
        return jsonify({"success": True, "message": "Mission started (reset to GUIDED → AUTO)"})
    except Exception as e:
        return jsonify({"success": False, "message": f"Start mission failed: {e}"}), 500


@app.route("/vehicle-position", methods=["GET"])
def vehicle_position():
    try:
        master = connect_to_vehicle(VEHICLE_ADDR, VEHICLE_PORT, VEHICLE_PROTO)
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        master.close()

        if not msg:
            return jsonify({"success": False, "message": "No GPS data"}), 500

        pos = msg.to_dict()
        lat = pos["lat"] / 1e7
        lon = pos["lon"] / 1e7
        alt = pos["alt"] / 1000.0

        return jsonify({
            "success": True,
            "lat": lat,
            "lon": lon,
            "alt": alt
        })
    except Exception as e:
        return jsonify({"success": False, "message": f"Position failed: {e}"}), 500


# =============================
# Main
# =============================
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)

