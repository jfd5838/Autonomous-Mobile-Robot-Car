# Autonomous Mobile Robot Car
This project is about building an autonomous wheeled robot that can: Follow a line, Avoid obstacles, Recognize traffic signs like STOP, YIELD. The robot uses a Raspberry Pi 4 as its brain and two Arduino boards for controlling motors and reading sensors. It uses a Pi camera to recognize signs and ultrasonic sensors to detect obstacles. When it sees a stop sign, it stops for a moment before moving again. When it sees a yield sign, it slows down. When it sees a speed limit sign, it speeds up. 

# List of Materials
    Raspberry Pi 4 (x1) 
    Freenove Wheeled Robot Kit (x1) 
    Arduino Boards (x2) 
    Ultrasonic Sensor (x1) 
    Infrared Sensors (x1) 
    Raspberry Pi Camera (x1) 
    LEDs 
    Wi-Fi Adapter or Ethernet (x1) 
    Sign Images (e.g., stop, yield, speed limit) 
    2nd robot car as Obstacles 
    USB-A to USB-B Cables (x2)

# Setup
Step 1: Component TEST (Ensuring they work) 
    Make sure that all component of robot car is well attached such as the wheels, cables etc. (if not well attached, this will impact on how the robot function/moves) 
    Make sure ultrasonic sensor works (Connect to Arduino and computer make sure that it reads/send accurate data) 
    Make sure infrared sensor underneath for line-following works properly. 
    Connect your Raspberry Pi and Arduino boards using the USB-A to USB-B cables. 
    Do not install Raspberry pi camera until recognition test is completed 

Step 2: Install & Setup Raspberry Pi OS 
    Flash Raspberry Pi OS onto a microSD card using Raspberry Pi Imager. 
    Insert the SD card into the Raspberry Pi and boot it up. 
    Connect to Wi-Fi or use Ethernet for internet access. 
    More Detail Explanation: https://www.raspberrypi.com/documentation/computers/getting-started.html

Step 3: Install & Setup Arduino 
    Download and install Arduino IDE on your computer. 
    Open and upload provided motor control and sensor code to Arduino. 
    One Arduino handles motor control and line-following with the IR sensor. 
    The second Arduino handles ultrasonic obstacle detection. 
    Basic Arduino code for Sensors are found: Tutorial.pdf 

Step 4: Connect Raspberry Pi to Arduino 
    Use USB cables to connect both Arduinos to the Raspberry Pi. 
    Use serial communication (via USB) to send and receive data between the Pi and Arduinos. 
    
Step 5: Load and Test the Line-Following Code 
    Run the IR sensor code to test the line-following. 
    Adjust thresholds if the robot misses or goes off track. 

Step 6: Test Ultrasonic Obstacle Avoidance 
    Run the ultrasonic sensor code to make the robot stop or reroute when it sees obstacles. 

Step 7: Train Image Processing Model 
    Collect or download images of STOP and YIELD signs. 
    Use OpenCV and Python to detect these signs. 
    Save the model or code logic to run in real-time using the Pi Camera. 
    Mount the Raspberry Pi Camera on the front-facing direction of robot once image processing is completed 

Step 8: Integrate Everything 
    Merge line-following, obstacle detection, and image recognition into one master Python script on the Raspberry Pi. 
    Use decision logic like: 
    If line is visible: follow it 
    If object detected: stop 
    If STOP sign: stop, wait, continue 
    If YIELD sign: slow down
    If SPEED LIMIT sign: speed up

Step 10: Build Your Track 
    Have black paper, use white tape to outline track, making sure that the car will stay within the given line. 
    Place STOP, SPEED LIMIT and YIELD sign images along the track. 
    Use second robot or other object as obstacle making sure that the car stops when it recognize that there is something in front of it.  

Step 11: Final Testing 
    Run the full robot script. 
    Observe how it responds to lines, signs, and obstacles. 
    Make Appropriate changes to final script.

# Instrucions for Use
    Power on the Raspberry Pi and Arduinos. 
    Place the robot at the start of the track. 
    Watch the robot follow the line, stop for STOP signs, slow for YIELD signs, and stop when there is an obstacles. 
    Power off when done.

# Tips and Tricks
    Make sure wires are neatly connected to avoid disconnection during movement. 
    If something doesn’t work, test each module separately (IR, Ultrasonic, Camera, motor, LED, etc). 
    Keep the Pi cool using a fan or heat sinks if running long. 
    If everything seem to work fine but car wont follow line (Check if wheels are neatly connected to the car, can cause car to malfunction) 
    Also make sure that when coding PIN usage do not overlap, can cause malfunction. 

    HAVE FUNN!!! 
