import json, time
import paho.mqtt.client as mqtt
from pymavlink import mavutil

# =============================
# Config
# =============================
MQTT_BROKER = "broker.emqx.io"
MQTT_PORT   = 1883
MQTT_TOPIC_TELE = "usv/telemetry"
MQTT_TOPIC_CMD  = "usv/command/#"

VEHICLE_PORT = '/dev/ttyAMA0'
VEHICLE_BAUD = 57600
ACK_TIMEOUT  = 10

master = mavutil.mavlink_connection(VEHICLE_PORT, baud=VEHICLE_BAUD)
master.wait_heartbeat()
print("✅ Connected to MAVLink")

# =============================
# Functions
# =============================
def send_mission(mission):
    wp_count = len(mission)
    if wp_count == 0:
        return

    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    time.sleep(1)

    master.mav.mission_count_send(master.target_system, master.target_component, wp_count)
    print(f"📤 Sending mission count={wp_count}")

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
            0, 1,
            hold_time, 0, 0, 0,
            lat, lon, alt
        )
        print(f"✅ Sent WP {seq+1}: {wp}")
        time.sleep(0.2)

    msg = master.recv_match(type='MISSION_ACK', timeout=ACK_TIMEOUT)
    if msg:
        print("🎉 Mission upload complete")

def publish_telemetry():
    # đọc data từ MAVLink
    msg_gps = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    msg_ph  = master.recv_match(type="NAMED_VALUE_FLOAT", blocking=False)

    telemetry = {
        "battery": 75,
        "speed": 2.5,
        "heading": 90,
        "ph": None,
        "do": None,
        "cod": None,
        "tss": None,
        "lat": None,
        "lon": None,
        "alt": None
    }

    if msg_gps:
        telemetry["lat"] = msg_gps.lat / 1e7
        telemetry["lon"] = msg_gps.lon / 1e7
        telemetry["alt"] = msg_gps.alt / 1000.0

    if msg_ph:
        name = msg_ph.name.lower()
        if name == "ph":
            telemetry["ph"] = msg_ph.value
        elif name == "do":
            telemetry["do"] = msg_ph.value
        elif name == "cod":
            telemetry["cod"] = msg_ph.value
        elif name == "tss":
            telemetry["tss"] = msg_ph.value


    mqtt_client.publish(MQTT_TOPIC_TELE, json.dumps(telemetry))

def handle_command(topic, payload):
    try:
        if topic.endswith("arm"):
            if payload.decode() == "1":
                master.arducopter_arm()
                master.motors_armed_wait()
                print("✅ Vehicle armed")
            else:
                master.arducopter_disarm()
                master.motors_disarmed_wait()
                print("✅ Vehicle disarmed")

        elif topic.endswith("mode"):
            mode = payload.decode().upper()
            master.set_mode(mode)
            print(f"⚙️ Set mode {mode}")

        elif topic.endswith("start"):
            master.set_mode("AUTO")
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_MISSION_START,
                0, 0,0,0,0,0,0,0
            )
            print("🚀 Mission started")

        elif topic.endswith("stop"):
            master.set_mode("MANUAL")
            print("🛑 Mission stopped")

        elif topic.endswith("mission"):
            data = json.loads(payload.decode())
            mission = data.get("mission", [])
            send_mission(mission)

    except Exception as e:
        print(f"❌ Command error: {e}")

# =============================
# MQTT setup
# =============================
def on_connect(client, userdata, flags, rc):
    print("✅ Connected MQTT")
    client.subscribe(MQTT_TOPIC_CMD)

def on_message(client, userdata, msg):
    print(f"📥 {msg.topic} -> {msg.payload}")
    handle_command(msg.topic, msg.payload)

mqtt_client = mqtt.Client(client_id="usv_pi")


mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

# =============================
# Main loop
# =============================
while True:
    publish_telemetry()
    time.sleep(2)
