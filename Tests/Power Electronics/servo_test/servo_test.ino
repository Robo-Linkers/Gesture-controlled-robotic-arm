#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create a PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define the servo parameters
#define SERVOMIN 150  // Minimum pulse length count
#define SERVOMAX 600  // Maximum pulse length count
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ); // Set PWM frequency to 50 Hz
}

void loop() {
  // Sweep the servo back and forth
  for (int pulselen = SERVOMIN; pulselen <= SERVOMAX; pulselen++) {
    pwm.setPWM(0, 0, pulselen); // Channel 0 controls the servo
    pwm.setPWM(1, 0, pulselen); // Channel 0 controls the servo
    pwm.setPWM(2, 0, pulselen); // Channel 0 controls the servo
    pwm.setPWM(3, 0, pulselen); // Channel 0 controls the servo
    delay(10);
  }
  for (int pulselen = SERVOMAX; pulselen >= SERVOMIN; pulselen--) {
    pwm.setPWM(0, 0, pulselen);
    pwm.setPWM(1, 0, pulselen);
    pwm.setPWM(2, 0, pulselen);
    pwm.setPWM(3, 0, pulselen);
    delay(10);
  }
}