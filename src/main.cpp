// ------BLDC--------
#include <Servo.h>
#define MAX_PWM 2000
#define MIN_PWM 1000
#define STATIONARY_PWM 1500
Servo ESC;
double motorSpeed = 0;
// ------------------

// ------BNO055-----
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define MPU_ADDRESS 0x28 // I2C address of the BNO055
#define BNO055_SAMPLERATE_DELAY_MS 10
Adafruit_BNO055 bno = Adafruit_BNO055(-1, MPU_ADDRESS, &Wire);
double yawAngle = 0;
double yawAngularSpeed = 0;
// ------------------

// -------PID--------
#include "PID.h"
const double xkP = 2.5; // displacement kP 
const double xkI = 0.0;
const double xkD = 400; // 400 kD lmao
PIDAngleController pidAngle(xkP, xkI, xkD); 
const double vkP = 0.050; // velocity kP // TODO: ESC already has a velocity speed, so sus.
const double vkI = 0.000;
const double vkD = 0.017; 
PIDController pidSpeed(vkP, vkI, vkD);
// ------------------

// ------GENERAL-----
long timeCur, timePrev, timeStart; 
const int numReadings = 5;
double readings[numReadings];
int readIndex = 0;
double total = 0;
double rollingAvg = 0;
double targetPos = 0;
// FSM variables
byte controllerState = 0;
// ------------------

// Calibrate ESC by plugging in battery when MAX_PWM is being outputted.
void calibrateESC() {
  ESC.writeMicroseconds(STATIONARY_PWM);
  delay(8000);
}

// Set the current speed and direction of the motor
void setSpeed(double targetSpeed) {
  // the question is what the difference between stationary and min is.
  motorSpeed = constrain(STATIONARY_PWM + targetSpeed, MIN_PWM, MAX_PWM); 
  ESC.writeMicroseconds(motorSpeed);
}

void updateYawAngle() {
  sensors_event_t event;
  bno.getEvent(&event);
  yawAngle = event.orientation.roll;
}

void updateYawSpeed() {
  sensors_event_t event;
  bno.getEvent(&event);
  yawAngularSpeed = event.gyro.x;
}

// Smooth the angular speed --> rolling average
void updateRollingAvg() {
    total = total - readings[readIndex]; 
    readings[readIndex] = yawAngularSpeed; 
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings) { readIndex = 0; }
    rollingAvg = total / numReadings;
}

void setup() {
  Serial.begin(9600);
  
  // IMU setup
  if(!bno.begin()) {
    Serial.print(F("No BNO055 detected"));
    exit(1);
  }
  bno.setExtCrystalUse(true);
  
  // Start ESC on pin 9
  calibrateESC();

  updateYawAngle();
  targetPos = yawAngle;

  Serial.println("Setup finished!");

  timeCur = millis();
  timeStart = timeCur;
}

void loop() {
  // Every 10ms, read IMU and call controllers
  if (millis() - timeCur > 10) {
    timePrev = timeCur;
    timeCur = millis();

    // update values
    updateYawAngle();
    updateYawSpeed();
    updateRollingAvg();

    // FSM transition
    if (controllerState == 1 && fabs(rollingAvg) > 360 /* °/s */) {
      controllerState = 0;
    } else if (controllerState == 0 && fabs(rollingAvg) < 45 /* °/s */) {
      controllerState = 1;
    }
    // FSM action 
    if (controllerState == 0) { // state 0 = detumble + update time for angle 
      pidAngle.compute(targetPos, yawAngle, timeCur - timePrev); 
    } else { // state 1 = set setpoint to calculated motor output. Error = desired pwm -avg pwm 
      motorSpeed += pidSpeed.compute(pidAngle.compute(targetPos, yawAngle, timeCur - timePrev), rollingAvg, timeCur - timePrev);
    } 
    setSpeed(motorSpeed);

    delay(BNO055_SAMPLERATE_DELAY_MS);

    // Print info to console
    Serial.print(yawAngle);
    Serial.print(F(" "));
    Serial.println(rollingAvg);
  }
}