from flask import Flask, Response
from picamera2 import Picamera2
import serial
import time
import cv2

app = Flask(__name__)

# Initialize camera
camera = Picamera2()
config = camera.create_preview_configuration(main={"size": (768,576), "format": "XRGB8888"},
                                             sensor={"output_size": (3280, 2464)})
camera.configure(config)
camera.start()
camera.set_controls({
    "ScalerCrop": (0, 0, 3280, 2464),
    "AwbMode": 1,
    "AeEnable": True
})

# Connect to Arduino (adjust port as needed)
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

# Intialize signs (https://github.com/taquy/traffic-sign-detection-using-opencv-haar-cascade/tree/master)
stop_sign_cascade = cv2.CascadeClassifier('stop_sign.xml')
yield_sign_cascade = cv2.CascadeClassifier('yield_sign.xml')

def generate_frames():
    last_stop_time = 0
    cooldown_duration = 5  # seconds
    trigger_area_ratio = 0.07  # 7% of the screen area

    while True:
        frame = camera.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        frame_height, frame_width = frame.shape[:2]
        frame_area = frame_width * frame_height
        center_third_left = frame_width / 3
        center_third_right = 2 * frame_width / 3

        stop_signs = stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        current_time = time.time()
        in_cooldown = (current_time - last_stop_time) < cooldown_duration

        for (x, y, w, h) in stop_signs:
            area = w * h
            center_x = x + w / 2

            if in_cooldown:
                box_color = (0, 255, 0)
                label = "STOP (Cooldown)"
            elif area >= trigger_area_ratio * frame_area and center_third_left <= center_x <= center_third_right:
                arduino.write(b'STOP\n')
                print("Sent STOP to Arduino")
                time.sleep(3)
                arduino.write(b'GO\n')
                print("Sent GO to Arduino")
                last_stop_time = current_time
                box_color = (0, 0, 255)
                label = "STOP"
            else:
                box_color = (0, 225, 225)
                label = "STOP (Too Far)"

            cv2.rectangle(frame, (x, y), (x+w, y+h), box_color, 2)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, box_color, 2)

        # Detect YIELD signs
        yield_signs = yield_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        for (x, y, w, h) in yield_signs:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "YIELD", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Encode frame for streaming
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type:image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)



