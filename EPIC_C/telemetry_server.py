from flask import Flask, render_template
from flask_socketio import SocketIO
import paho.mqtt.client as mqtt
import json, threading

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

@app.route("/")
def index():
    return render_template("dashboard.html")

# MQTT callback
def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    socketio.emit("telemetry", data)

def mqtt_thread():
    client = mqtt.Client()
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.subscribe("usv/telemetry")
    client.loop_forever()

# Chạy MQTT listener song song
threading.Thread(target=mqtt_thread, daemon=True).start()

if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)
