from picamera2 import Picamera2
from flask import Flask, Response
import cv2
from pyngrok import ngrok
app = Flask(__name__)

# Khởi tạo camera
camera = Picamera2()
camera.configure(camera.create_preview_configuration(main={"size": (640, 480)}))
camera.start()

def gen_frames():
    while True:
        frame = camera.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        _, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h1>📷 Raspberry Pi Camera Stream</h1><img src='/video_feed'>"

if __name__ == '__main__':
    url = ngrok.connect(5000)
    print(f"url:{url}")
    app.run(host='0.0.0.0', port=5000, debug=False)
