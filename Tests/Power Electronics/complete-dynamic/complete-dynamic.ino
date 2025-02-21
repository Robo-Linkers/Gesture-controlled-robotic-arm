// Complete test code for PE
// parallel operation
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define pins for stepper motor
const int DIR = 2;  // Direction pin
const int STEP = 4; // Step pin

#define motorInterfaceType 1
AccelStepper motor(motorInterfaceType, STEP, DIR);


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo parameters
#define SERVOMIN 150  // Minimum pulse length count
#define SERVOMAX 600  // Maximum pulse length count
#define SERVO_FREQ 50 // Frequency for analog servos (~50 Hz)

// Variables for servo movement control
unsigned long previousMillis = 0;
const long interval = 10;  // Update every 10ms for smooth movement
int currentServoPosition = SERVOMIN;
int targetServoPosition = SERVOMAX;
bool isAscending = true;

void moveServos();  

void setup() {
  Serial.begin(115200);


  motor.setMaxSpeed(32000);  // Set max speed for stepper motor (adjust as necessary)
  motor.setAcceleration(10000);  // Set reasonable acceleration for stepper motor
  motor.setSpeed(10000);  // Set initial speed for stepper motor

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  
}

void loop() {
  // Move stepper motor 360° clockwise (move forward)
  if (motor.distanceToGo() == 0) {
    motor.moveTo(6400);  // Move 6400 steps (360° for 1/32 microstepping)
    targetServoPosition = SERVOMAX;  // Move servos to 180°
    isAscending = true;  // Ascend servo from 0 to 180°
  }

  // Move stepper motor
  motor.run();
  
  // Move servos incrementally
  moveServos();

  // Once the stepper reaches 360° clockwise, reverse direction
  if (motor.distanceToGo() == 0) {
    motor.moveTo(0);  // Move stepper motor back to 0 (360° counterclockwise)
    targetServoPosition = SERVOMIN;  // Move servos back to 0°
    isAscending = false;  // Descend servo from 180° back to 0°
  }

  // Allow both motors to move simultaneously
  motor.run();
  moveServos();
}

void moveServos() {
  // Check if it's time to update the servo position
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Move servos incrementally
    if (isAscending) {
      currentServoPosition++;
      if (currentServoPosition >= targetServoPosition) {
        isAscending = false;  // Reverse servo direction when target is reached 
      }
    } else {
      currentServoPosition--;
      if (currentServoPosition <= targetServoPosition) {
        isAscending = true;  // Reverse servo direction when target is reached
      }
    }

    // Update the servo positions
    pwm.setPWM(0, 0, currentServoPosition);
    pwm.setPWM(1, 0, currentServoPosition);
    pwm.setPWM(2, 0, currentServoPosition);
    pwm.setPWM(3, 0, currentServoPosition);
  }
}
