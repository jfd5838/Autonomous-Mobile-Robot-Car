#include "Freenove_WS2812B_RGBLED_Controller.h"
#include <Servo.h>
#include <stdint.h>

// === Pin Definitions ===
#define PIN_SERVO      2
#define MOTOR_DIRECTION     0 //If the direction is reversed, change 0 to 1
#define PIN_DIRECTION_LEFT  4
#define PIN_DIRECTION_RIGHT 3
#define PIN_MOTOR_PWM_LEFT  6
#define PIN_MOTOR_PWM_RIGHT 5
#define PIN_SONIC_TRIG    7
#define PIN_SONIC_ECHO    8
#define PIN_IRREMOTE_RECV 9
#define PIN_SPI_CE      9
#define PIN_SPI_CSN     10
#define PIN_SPI_MOSI    11
#define PIN_SPI_MISO    12
#define PIN_SPI_SCK     13
#define PIN_BATTERY     A0
#define PIN_BUZZER      A4
#define PIN_TRACKING_LEFT A1
#define PIN_TRACKING_CENTER A2
#define PIN_TRACKING_RIGHT  A3
#define MOTOR_PWM_DEAD    10

#define I2C_ADDRESS  0x20
#define LEDS_COUNT   10  //it defines number of lEDs. 

#define MAX_DISTANCE         1000
#define SONIC_TIMEOUT        (MAX_DISTANCE * 60)
#define SOUND_VELOCITY       340
#define OBSTACLE_THRESHOLD   10  // in cm
#define OBSTACLE_THRESHOLD 10

#define TK_STOP_SPEED          0
// Base speeds (non-adjustable)
int BASE_FORWARD_SPEED;
int BASE_TURN_SPEED_LV4;
int BASE_TURN_SPEED_LV3;
int BASE_TURN_SPEED_LV2;
int BASE_TURN_SPEED_LV1;

// Adjustable speeds
int TK_FORWARD_SPEED;
int TK_TURN_SPEED_LV4;
int TK_TURN_SPEED_LV3;
int TK_TURN_SPEED_LV2;
int TK_TURN_SPEED_LV1;

int tk_VoltageCompensationToSpeed;  //define Voltage Speed Compensation

float batteryVoltage = 0;
bool isBuzzered = false;
int tk_SpeedAdjustment = 0;
bool isStopped = false;
Freenove_WS2812B_Controller strip(I2C_ADDRESS, LEDS_COUNT, TYPE_GRB); //initialization 

void setup() {
  Serial.begin(9600);
  pinsSetup(); //set up pins
  getTrackingSensorVal();//Calculate Voltage speed Compensation
  tk_CalculateVoltageCompensation();
  setBaseSpeeds(); // Base Speed = 40mph
  strip.begin();  // Initialize the LED controller
  strip.setAllLedsColor(0, 0, 0);  // Ensure LEDs start off

  Serial.print("Battery Voltage: "); Serial.println(batteryVoltage);
  Serial.print("Voltage Compensation: "); Serial.println(tk_VoltageCompensationToSpeed);
  Serial.print("Forward Speed: "); Serial.println(BASE_FORWARD_SPEED);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any extra newline characters

    if (command == "START"){
      strip.setAllLedsColor(0, 0, 255); //set all LED color to green
      delay(1000);
      strip.setAllLedsColor(0, 0, 255); //set all LED color to green
      delay(1000);
      strip.setAllLedsColor(0, 0, 0); 
    }
  }
  
  u8 trackingSensorVal = 0;
  trackingSensorVal = getTrackingSensorVal(); //get sensor value

  switch (trackingSensorVal)
  {
    case 0:   // 000 – All white (off the line)
      // Try rotating in place to search for line
      motorRun(-TK_TURN_SPEED_LV1, TK_TURN_SPEED_LV1);  // spin right slowly
      break;

    case 7:   // 111 – All black (intersection or T-junction)
      motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED);
      break;

    case 1:   // 001 – Right sensor on line
      motorRun(TK_TURN_SPEED_LV4, TK_TURN_SPEED_LV1); // sharp right
      break;

    case 3:   // 011 – Right + center
      motorRun(TK_TURN_SPEED_LV3, TK_TURN_SPEED_LV2); // medium right
      break;

    case 2:   // 010 – Center
    case 5:   // 101 – Left + right, possibly crossing
      motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED);
      break;

    case 6:   // 110 – Left + center
      motorRun(TK_TURN_SPEED_LV2, TK_TURN_SPEED_LV3); // medium left
      break;

    case 4:   // 100 – Left only
      motorRun(TK_TURN_SPEED_LV1, TK_TURN_SPEED_LV4); // sharp left
      break;

    default:
      // Unknown or noisy reading — stop briefly
      motorRun(TK_STOP_SPEED, TK_STOP_SPEED);
      delay(100);
      break;
  }

  // --- Check for Pi Sign Commands ---
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any extra newline characters

    if (command == "STOP"){
      isStopped = true;
      strip.setAllLedsColor(255, 0, 0); //set all LED color to red
    }
    else if (command == "GO"){
      isStopped = false;
      strip.setAllLedsColor(0, 0, 0);    //set all LED off 
    }
    else if (command == "YIELD"){
      adjustSpeed(-20); // slow down
      strip.setAllLedsColor(255, 200, 0); //set all LED color to yellow
    }
    else if (command == "SPEED UP"){
      adjustSpeed(20); // speed up
      strip.setAllLedsColor(0, 0, 0); 
    }
    else if (command.startsWith("CHANGE SPEED ")) {
      int delta = command.substring(13).toInt(); // extract number after "CHANGE SPEED "
      if (delta >= 40){
        adjustSpeed(delta);
      } else {
        adjustSpeed(-1 * delta);
      }
      strip.setAllLedsColor(0, 175, 255); //set all LED color to cyan
      delay(2000);
      strip.setAllLedsColor(0, 0, 0);  
    }
  }

  // --- STOP mode: override everything ---
  if (isStopped) {
    motorRun(TK_STOP_SPEED, TK_STOP_SPEED);
    return;
  }

    // float distance = getSonar();
    // Serial.print("Distance: ");
    // Serial.println(distance);

    // while (distance < OBSTACLE_THRESHOLD) {
    //   motorRun(TK_STOP_SPEED, TK_STOP_SPEED);
    //   Serial.println("Obstacle too close! Waiting 3 seconds...");
    //   strip.setAllLedsColor(255, 0, 0); 
    //   delay(3000);
    //   strip.setAllLedsColor(0, 0, 0); 
    //   distance = getSonar();
    //   Serial.print("Rechecking distance: ");
    //   Serial.println(distance);
    // }
}
  

void tk_CalculateVoltageCompensation() {
  getBatteryVoltage();
  float voltageOffset = 7 - batteryVoltage;
  tk_VoltageCompensationToSpeed = constrain(20 * voltageOffset, 0, 10);  // more conservative
}


// === Sensor Logic: Treat BLACK as 1, WHITE as 0 ===
u8 getTrackingSensorVal() {
  u8 trackingSensorVal = 0;
  trackingSensorVal = (digitalRead(PIN_TRACKING_LEFT) == 1 ? 1 : 0) << 2 | 
                      (digitalRead(PIN_TRACKING_CENTER) == 1 ? 1 : 0) << 1 | 
                      (digitalRead(PIN_TRACKING_RIGHT) == 1 ? 1 : 0) << 0;
  return trackingSensorVal;
}

// === Pin Setup ===
void pinsSetup() {
  //define motor pin
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
  //define ultrasonic moduel pin
  pinMode(PIN_SONIC_TRIG, OUTPUT);
  pinMode(PIN_SONIC_ECHO, INPUT);
  //define tracking sensor pin
  pinMode(PIN_TRACKING_LEFT, INPUT);
  pinMode(PIN_TRACKING_RIGHT, INPUT);
  pinMode(PIN_TRACKING_CENTER, INPUT);
  setBuzzer(false);
}

// === Motor Control ===
void motorRun(int speedl, int speedr) {
  int dirL = 0, dirR = 0;
  if (speedl > 0) {
    dirL = 0 ^ MOTOR_DIRECTION;
  } else {
    dirL = 1 ^ MOTOR_DIRECTION;
    speedl = -speedl;
  }

  if (speedr > 0) {
    dirR = 1 ^ MOTOR_DIRECTION;
  } else {
    dirR = 0 ^ MOTOR_DIRECTION;
    speedr = -speedr;
  }
  speedl = constrain(speedl, 0, 255); // speedl absolute value should be within 0~255
  speedr = constrain(speedr, 0, 255); // speedr absolute value should be within 0~255
  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}

// === Battery Voltage Check ===
bool getBatteryVoltage() {
  if (!isBuzzered) {
    pinMode(PIN_BATTERY, INPUT);
    int batteryADC = analogRead(PIN_BATTERY);
    if (batteryADC < 614)    // 3V/12V ,Voltage read: <2.1V/8.4V
    {
      batteryVoltage = batteryADC / 1023.0 * 5.0 * 4;
      return true;
    }
  }
  return false;
}

// === Buzzer Functions ===
void setBuzzer(bool flag) {
  isBuzzered = flag;
  pinMode(PIN_BUZZER, flag);
  digitalWrite(PIN_BUZZER, flag);
}

void alarm(u8 beat, u8 repeat) {
  beat = constrain(beat, 1, 9);
  repeat = constrain(repeat, 1, 255);
  for (int j = 0; j < repeat; j++) {
    for (int i = 0; i < beat; i++) {
      setBuzzer(true);
      delay(100);
      setBuzzer(false);
      delay(100);
    }
    delay(500);
  }
}

// === Safety Reset ===
void resetCarAction() {
  motorRun(0, 0);
  setBuzzer(false);
}

float getSonar() {
  digitalWrite(PIN_SONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);

  long duration = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT);
  if (duration == 0) return MAX_DISTANCE;
  return (float)duration * SOUND_VELOCITY / 2 / 10000;
}

void setBaseSpeeds() {
  BASE_FORWARD_SPEED   =  75 + tk_VoltageCompensationToSpeed;
  BASE_TURN_SPEED_LV4  = 115 + tk_VoltageCompensationToSpeed;  // outer wheel
  BASE_TURN_SPEED_LV3  = 100 + tk_VoltageCompensationToSpeed;
  BASE_TURN_SPEED_LV2  =  70 + tk_VoltageCompensationToSpeed;
  BASE_TURN_SPEED_LV1  = -60 + tk_VoltageCompensationToSpeed;  // reverse inner wheel
  resetSpeed();
}

void adjustSpeed(int delta) {
  TK_FORWARD_SPEED   = BASE_FORWARD_SPEED + delta;
  TK_TURN_SPEED_LV4  = BASE_TURN_SPEED_LV4 + delta;
  TK_TURN_SPEED_LV3  = BASE_TURN_SPEED_LV3 + delta;
  TK_TURN_SPEED_LV2  = BASE_TURN_SPEED_LV2 + delta;
  TK_TURN_SPEED_LV1  = BASE_TURN_SPEED_LV1 + delta;
}

void resetSpeed() {
  TK_FORWARD_SPEED   = BASE_FORWARD_SPEED;
  TK_TURN_SPEED_LV4  = BASE_TURN_SPEED_LV4;
  TK_TURN_SPEED_LV3  = BASE_TURN_SPEED_LV3;
  TK_TURN_SPEED_LV2  = BASE_TURN_SPEED_LV2;
  TK_TURN_SPEED_LV1  = BASE_TURN_SPEED_LV1;
}
