#include <AccelStepper.h>

const int DIR = 4;  // Direction pin
const int STEP = 2; // Step pin

#define motorInterfaceType 1
AccelStepper motor(motorInterfaceType, STEP, DIR);

int currentSpeed = 200;
int currentAcceleration = 1000;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 2000;

void setup() {
  Serial.begin(115200);
  motor.setMaxSpeed(32000);
  motor.setAcceleration(currentAcceleration);
  motor.setSpeed(currentSpeed);
  motor.moveTo(6400); // Move 6400 steps (360 rotation for 1/32 microstepping)
}

void loop() {
  if (motor.distanceToGo() == 0) {
    motor.moveTo(-motor.currentPosition());
  }

  motor.run();

  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdate > updateInterval) {
    lastUpdate = currentMillis;

    currentSpeed += 100;
    currentAcceleration += 200;

    if (currentSpeed > 32000) currentSpeed = 200;
    if (currentAcceleration > 8000) currentAcceleration = 1000;

    motor.setSpeed(currentSpeed);
    motor.setAcceleration(currentAcceleration);

    Serial.print("Speed: ");
    Serial.print(currentSpeed);
    Serial.print(" | Acceleration: ");
    Serial.println(currentAcceleration);
  }

  Serial.print("Position: ");
  Serial.println(motor.currentPosition());
}