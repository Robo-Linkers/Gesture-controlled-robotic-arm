#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>

// PWM Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo Channels
// #define BASE_SERVO 0
#define SHOULDER_SERVO 0
#define ELBOW_SERVO 1
#define WRIST_SERVO 2

// Stepper Motor (Base Rotation)
#define STEP_PIN 26
#define DIR_PIN 27
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Servo Angles
int baseAngle = 90, shoulderAngle = 90, elbowAngle = 90, wristAngle = 90;
bool gripperClosed = false;

// Wi-Fi and MQTT Credentials
const char *ssid = "Your_SSID";
const char *password = "Your_PASSWORD";
const char *mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char *mqttUser = "Your_MQTT_Username";
const char *mqttPassword = "Your_MQTT_Password";

// MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Function Prototypes
void updateServos();
void mqttCallback(char *topic, byte *message, unsigned int length);
int angleToPulse(int angle);
void controlStepper(float angle);

/**
 * @brief Initializes the serial communication, I2C bus, PWM servo driver, and stepper motor.
 *        Connects to WiFi and MQTT broker.
 * @details
 * - Initializes serial communication with a baud rate of 115200.
 * - Initializes the I2C bus.
 * - Initializes the PWM servo driver and sets the PWM frequency to 50 Hz.
 * - Initializes the stepper motor and sets its maximum speed and acceleration.
 * - Connects to WiFi with the provided ssid and password.
 * - Connects to the MQTT broker at the provided server and port.
 * - Subscribes to the "robot/angles" topic.
 * - Updates the servo angles to the initial values.
 */
void setup()
{
    Serial.begin(115200);
    Wire.begin();

    pwm.begin();
    pwm.setPWMFreq(50);

    stepper.setMaxSpeed(2000);
    stepper.setAcceleration(100);

    WiFi.begin(ssid, password);
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("WiFi connected!");

    while (!mqttClient.connected())
    {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32_ACTUATOR", mqttUser, mqttPassword))
        {
            Serial.println("Connected to MQTT.");
            mqttClient.subscribe("robot/angles");
        }
        else
        {
            Serial.println("Failed. Retrying...");
            delay(2000);
        }
    }
    updateServos();
}

/**
 * @brief Main loop of the program.
 *        Checks for any MQTT messages and updates the stepper motor
 *        if a new angle is received.
 */
void loop()
{
    mqttClient.loop();
    stepper.run();
}

/**
 * @brief MQTT callback function.
 *        Updates the servo angles and the stepper motor when an MQTT message is received.
 *        The message should be in the format "SX:<angleServoX>,SX2:<angleServoX2>,ST:<angleStepperY>,GR:<grip>".
 * @param[in] topic MQTT topic of the message.
 * @param[in] message MQTT message payload.
 * @param[in] length Length of the MQTT message payload.
 */
void mqttCallback(char* topic, byte* message, unsigned int length) {
    String data;
    for (unsigned int i = 0; i < length; i++) {
        data += (char)message[i];
    }
    
    float angleServoX, angleServoX2, angleStepperY;
    int grip;
    sscanf(data.c_str(), "SX:%f,SX2:%f,ST:%f,GR:%d", &angleServoX, &angleServoX2, &angleStepperY, &grip);

    shoulderAngle = map(angleServoX, -90, 90, 0, 180);
    elbowAngle = map(angleServoX2, -90, 90, 0, 180);
    gripperClosed = grip;

    controlStepper(angleStepperY);
    updateServos();
}

/**
 * @brief Move the stepper motor to a specific angle.
 *        The angle should be between -90 and 90 degrees.
 *        The stepper motor will move to the specified angle and stop.
 * @param[in] angle Angle to move the stepper motor to, in degrees.
 */
void controlStepper(float angle)
{
    stepper.moveTo(map(angle, -90, 90, -1000, 1000));
}

/**
 * @brief Updates the servo angles of the robotic arm.
 *        The angles should be in the range of 0-180 degrees.
 *        The servos will move to the specified angle and stop.
 * @details
 * - Updates the servo angles using the Adafruit_PWMServoDriver library.
 * - The angles are mapped from 0-180 degrees to the specified range of the servos.
 * - The servos will move to the specified angle and stop.
 */
void updateServos()
{
    pwm.setPWM(SHOULDER_SERVO, 0, angleToPulse(shoulderAngle));
    pwm.setPWM(ELBOW_SERVO, 0, angleToPulse(elbowAngle));
    pwm.setPWM(WRIST_SERVO, 0, angleToPulse(gripperClosed ? 0 : 180));
}

/**
 * @brief Converts an angle in degrees to a pulse length suitable for controlling a servo motor.
 *        The angle should be in the range of 0-180 degrees.
 *        The pulse length is mapped from the range of 0-180 degrees to the range of 150-600,
 *        which is a good range for most servo motors.
 * @param[in] angle Angle to convert, in degrees.
 * @return Pulse length suitable for controlling a servo motor, in microseconds.
 */
int angleToPulse(int angle)
{
    int pulse = map(angle, 0, 180, 150, 600); // 150-600 is a good range for most servos
    return pulse;
}