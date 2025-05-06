from flask import Flask, Response
from picamera2 import Picamera2
import serial
import time
import cv2
import pytesseract

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
    stop_triggered_time = 0
    waiting_to_go = False
    cooldown = False
    currently_yielding = False
    yield_disappear_time = None
    last_speed_limit = None
    valid_speed_limits = {15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70}

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

            if is_close_enough and not cooldown and not waiting_to_go:
                # Detected STOP, begin 3-second wait for GO
                arduino.write(b'STOP\n')
                print("Sent STOP to Arduino")
                stop_triggered_time = current_time
                waiting_to_go = True
                label = "STOP"
                box_color = (0, 0, 255)

            elif waiting_to_go:
                # Still waiting 3 seconds before sending GO
                if current_time - stop_triggered_time >= 3:
                    arduino.write(b'GO\n')
                    print("Sent GO to Arduino")
                    last_stop_time = current_time  # Start cooldown now
                    waiting_to_go = False
                    cooldown = True
                label = "STOP"
                box_color = (0, 0, 255)

            elif cooldown:
                # Cooldown after sending GO
                if current_time - last_stop_time >= 5:
                    cooldown = False  # Cooldown finished
                label = "STOP (Cooldown)"
                box_color = (0, 155, 255)

            else:
                label = "STOP (Too Far)"
                box_color = (0, 255, 255)

            cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, box_color, 2)


        # === YIELD Sign Detection ===
        yield_signs = yield_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        found_yield = False

        for (x, y, w, h) in yield_signs:
            area = w * h
            is_close_enough = (area / screen_area) > trigger_area_ratio
            if is_close_enough:
                found_yield = True
                if not currently_yielding:
                    arduino.write(b'YIELD\n')
                    print("Sent YIELD to Arduino")
                    currently_yielding = True
                    yield_disappear_time = None  # Reset disappear timer

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, "YIELD", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                break  # Only handle one yield sign at a time
            else:
                label = "YIELD (Too Far)"
                box_color = (0, 255, 255)

        # If no YIELD sign is visible
        if not found_yield and currently_yielding:
            if yield_disappear_time is None:
                yield_disappear_time = time.time()
            elif time.time() - yield_disappear_time >= 2:
                arduino.write(b'SPEED UP\n')
                print("Sent SPEED UP to Arduino")
                currently_yielding = False
                yield_disappear_time = None

        # === SPEED LIMIT Sign Detection ===
        speedlimit_signs = speedlimit_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        for (x, y, w, h) in speedlimit_signs:
            area = w * h
            is_close_enough = (area / screen_area) > trigger_area_ratio
            if is_close_enough:
                # Preprocess ROI for OCR
                roi = gray[y:y + h, x:x + w]
                roi = cv2.resize(roi, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
                roi = cv2.GaussianBlur(roi, (5, 5), 0)
                roi_thresh = cv2.adaptiveThreshold(roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                   cv2.THRESH_BINARY_INV, 11, 2)
        
                # OCR
                config = '--psm 6 -c tessedit_char_whitelist=0123456789'
                text = pytesseract.image_to_string(roi_thresh, config=config)
                digits = ''.join(filter(str.isdigit, text))
        
                if digits.isdigit():
                    new_speed_limit = int(digits)
                    if new_speed_limit in valid_speed_limits and new_speed_limit != last_speed_limit:
                        command = f'CHANGE SPEED {new_speed_limit}\n'
                        arduino.write(command.encode())
                        print(f"Sent {command.strip()} to Arduino")
                        last_speed_limit = new_speed_limit
                    else:
                        print(f"Ignored unrecognized or duplicate speed: {new_speed_limit}")
        
                # Draw rectangle and label
                label = f"SPEED LIMIT {digits if digits else 'Unclear'}"
                box_color = (255, 255, 0) if digits else (0, 155, 255)
                cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, box_color, 2)
        
                break  # Only handle one speed limit sign at a time
            else:
                label = "SPEED LIMIT (Too Far)"
                box_color = (0, 255, 255)

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