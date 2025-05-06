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
speedlimit_sign_cascade = cv2.CascadeClassifier('speedlimit_sign.xml')

trigger_area_ratio = 0.07  # Sign must occupy at least 7% of the screen to trigger

def generate_frames():
    last_stop_time = 0
    cooldown = False
    currently_yielding = False  # Track if we are currently in "YIELD" mode

    while True:
        frame = camera.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        screen_area = frame.shape[0] * frame.shape[1]

        # === STOP Sign Detection ===
        stop_signs = stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        for (x, y, w, h) in stop_signs:
            area = w * h
            is_close_enough = (area / screen_area) > trigger_area_ratio
            current_time = time.time()
            cooldown = (current_time - last_stop_time) < 3

            if not cooldown and is_close_enough:
                arduino.write(b'STOP\n')
                print("Sent STOP to Arduino")
                last_stop_time = current_time
                cooldown = True
                label = "STOP"
                box_color = (0, 0, 255)
            elif cooldown:
                label = "STOP (Cooldown)"
                box_color = (0, 255, 0)
            else:
                label = "STOP (Too Far)"
                box_color = (255, 255, 0)

            cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, box_color, 2)

        # === YIELD Sign Detection ===
        yield_signs = yield_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        found_yield = False

        for (x, y, w, h) in yield_signs:
            area = w * h
            if (area / screen_area) > trigger_area_ratio:
                found_yield = True
                if not currently_yielding:
                    arduino.write(b'YIELD\n')
                    print("Sent YIELD to Arduino")
                    currently_yielding = True

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, "YIELD", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                break  # Only handle one yield sign at a time

        # If no YIELD sign is visible but we were previously yielding
        if not found_yield and currently_yielding:
            arduino.write(b'SPEED UP\n')
            print("Sent SPEED UP to Arduino")
            currently_yielding = False

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
