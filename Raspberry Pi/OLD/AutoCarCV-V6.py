from flask import Flask, Response
from picamera2 import Picamera2
import serial
import time
import cv2
import pytesseract
import sys

app = Flask(__name__)
sys.path.append('/home/pi/.local/pipx/venvs/pytesseract/lib/python3.11.2/site-packages')

# Camera setup
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

# Serial setup
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)

# Load Haar cascades
stop_sign_cascade = cv2.CascadeClassifier('stop_sign.xml')
yield_sign_cascade = cv2.CascadeClassifier('yield_sign.xml')
speedlimit_sign_cascade = cv2.CascadeClassifier('speedlimit_sign.xml')

trigger_area_ratio = 0.07  # Minimum size to be considered close

def generate_frames():
    last_stop_time = 0
    yield_active = False

    while True:
        frame = camera.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        screen_area = frame.shape[0] * frame.shape[1]

        # === STOP Sign ===
        stop_signs = stop_sign_cascade.detectMultiScale(gray, 1.1, 5)
        for (x, y, w, h) in stop_signs:
            area = w * h
            is_close_enough = (area / screen_area) > trigger_area_ratio
            current_time = time.time()
            cooldown = (current_time - last_stop_time) < 3

            if is_close_enough and not cooldown:
                arduino.write(b'STOP\n')
                last_stop_time = current_time
                label = "STOP"
                box_color = (0, 0, 255)
            elif cooldown:
                label = "STOP (Cooldown)"
                box_color = (0, 255, 0)
            else:
                label = "STOP (Too Far)"
                box_color = (255, 255, 0)

            cv2.rectangle(frame, (x, y), (x+w, y+h), box_color, 2)
            cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, box_color, 2)

        # After STOP cooldown, send GO
        if time.time() - last_stop_time >= 3 and last_stop_time != 0:
            arduino.write(b'GO\n')
            last_stop_time = 0  # reset

        # === YIELD Sign ===
        yield_signs = yield_sign_cascade.detectMultiScale(gray, 1.1, 5)
        if yield_signs != ():
            yield_active = True
            for (x, y, w, h) in yield_signs:
                area = w * h
                is_close_enough = (area / screen_area) > trigger_area_ratio
                if is_close_enough:
                    arduino.write(b'YIELD\n')
                    label = "YIELD"
                    box_color = (0, 0, 255)
                else:
                    label = "YIELD (Too Far)"
                    box_color = (255, 255, 0)

                cv2.rectangle(frame, (x, y), (x+w, y+h), box_color, 2)
                cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, box_color, 2)
        elif yield_active:
            arduino.write(b'SPEED UP\n')
            yield_active = False

        # === SPEED LIMIT Sign + OCR ===
        speed_signs = speedlimit_sign_cascade.detectMultiScale(gray, 1.1, 5)
        for (x, y, w, h) in speed_signs:
            area = w * h
            is_close_enough = (area / screen_area) > trigger_area_ratio

            if is_close_enough:
                roi = gray[y:y+h, x:x+w]
                roi_thresh = cv2.threshold(roi, 100, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
                roi_resized = cv2.resize(roi_thresh, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)

                # OCR for digits only
                text = pytesseract.image_to_string(roi_resized, config='--psm 6 digits')
                recognized_digits = ''.join(filter(str.isdigit, text))

                if recognized_digits:
                    arduino.write(f"SPEED {recognized_digits}\n".encode())
                    label = f"SPEED LIMIT: {recognized_digits}"
                else:
                    arduino.write(b'CHANGE SPEED\n')
                    label = "SPEED LIMIT"
                box_color = (255, 0, 255)
            else:
                label = "SPEED LIMIT (Too Far)"
                box_color = (255, 255, 0)

            cv2.rectangle(frame, (x, y), (x+w, y+h), box_color, 2)
            cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, box_color, 2)

        # Stream result
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type:image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
