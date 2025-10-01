from flask import Flask, request, jsonify, render_template

app = Flask(__name__)

@app.route("/")
def index():
    return render_template("test_dashboard.html")
@app.route("/telemetry")
def telemetry():
    import random
    data = {
        "speed": round(random.uniform(0, 2), 2),
        "heading": random.randint(0, 360),
        "battery": random.randint(50, 100),
        "ph": round(random.uniform(6.5, 8.5), 2),
        "do": round(random.uniform(4, 10), 2),
        "cod": random.randint(50, 300),
        "tss": random.randint(100, 800)
    }
    return jsonify({"success": True, "telemetry": data})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)