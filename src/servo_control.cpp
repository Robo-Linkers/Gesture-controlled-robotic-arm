#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Create the PWM driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channels on PCA9685
#define BASE_SERVO 0
#define SHOULDER_SERVO 1
#define ELBOW_SERVO 2
#define WRIST_SERVO 3

// Servo angle variables
int baseAngle = 90, shoulderAngle = 90, elbowAngle = 90, wristAngle = 90;

// WiFi and MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Function Prototypes
void updateServos();
void mqttCallback(char* topic, byte* message, unsigned int length);
void parseAndProcessData(String data);
int angleToPulse(int angle);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize PCA9685
    pwm.begin();
    pwm.setPWMFreq(50);  // Standard servo frequency (50Hz)

    // Connect to WiFi and MQTT
    WiFi.begin("Wokwi-GUEST", "");
    mqttClient.setServer("broker.emqx.io", 1883);
    mqttClient.setCallback(mqttCallback);
    
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("WiFi connected!");

    connectToMQTT();
    updateServos();
}

void loop() {
    mqttClient.loop();
}

void connectToMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32", "Your_MQTT_Username", "Your_MQTT_Password")) {
            Serial.println("Connected to MQTT Broker.");
            mqttClient.subscribe("sensor/angles");
        } else {
            Serial.print("Failed. State=");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
}

void mqttCallback(char* topic, byte* message, unsigned int length) {
    String data;
    for (unsigned int i = 0; i < length; i++) {
        data += (char)message[i];
    }
    parseAndProcessData(data);
    updateServos();
}

void parseAndProcessData(String data) {
    float angleX = 0, angleY = 0;
    sscanf(data.c_str(), "X:%f,Y:%f", &angleX, &angleY);
    // Process angles
    if (angleX > 30) shoulderAngle = constrain(shoulderAngle + 2, 0, 180);
    else if (angleX < -30) shoulderAngle = constrain(shoulderAngle - 2, 0, 180);
    Serial.println(angleX);
    Serial.println(angleY);

    if (angleY > 30) elbowAngle = constrain(elbowAngle + 2, 0, 180);
    else if (angleY < -30) elbowAngle = constrain(elbowAngle - 2, 0, 180);

    if (angleX > 45 && angleY > 45) wristAngle = constrain(wristAngle + 5, 0, 180);
}

void updateServos() {
    pwm.setPWM(BASE_SERVO, 0, angleToPulse(baseAngle));
    pwm.setPWM(SHOULDER_SERVO, 0, angleToPulse(shoulderAngle));
    pwm.setPWM(ELBOW_SERVO, 0, angleToPulse(elbowAngle));
    pwm.setPWM(WRIST_SERVO, 0, angleToPulse(wristAngle));
}

// Convert angle (0-180) to PWM pulse (servo range: 500-2500µs)
int angleToPulse(int angle) {
    int pulse = map(angle, 0, 180, 150, 600);  // 150-600 is a good range for most servos
    return pulse;
}
