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

# Intialize signs (https://github.com/cfizette/road-sign-cascades/tree/master)
stop_sign_cascade = cv2.CascadeClassifier('cascade_stop_sign.xml')


# arduino = serial.Serial

def generate_frames():
    while True:
        frame = camera.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # Detect stop signs
        stop_signs = stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

        # Draw rectangles around detected stop signs
        for (x, y, w, h) in stop_signs:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(frame, "STOP", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
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

