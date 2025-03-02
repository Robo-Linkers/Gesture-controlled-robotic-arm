#include <Adafruit_Servo.h>
#include <AccelStepper.h>

// Servo objects
Servo baseServo, shoulderServo, elbowServo, wristServo;

// Servo angles
int baseAngle = 90, shoulderAngle = 90, elbowAngle = 90, wristAngle = 90;

// MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Function Prototypes
void updateServos();
void mqttCallback(char* topic, byte* message, unsigned int length);
void parseAndProcessData(String data);

void setup() {
    Serial.begin(115200);

    // Attach servos to respective pins
    baseServo.attach(9);
    shoulderServo.attach(10);
    elbowServo.attach(11);
    wristServo.attach(12);

    // Initialize MQTT
    mqttClient.setServer("broker.emqx.io", 1883);
    mqttClient.setCallback(mqttCallback);
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

    if (angleY > 30) elbowAngle = constrain(elbowAngle + 2, 0, 180);
    else if (angleY < -30) elbowAngle = constrain(elbowAngle - 2, 0, 180);

    if (angleX > 45 && angleY > 45) wristAngle = constrain(wristAngle + 5, 0, 180);
}

void updateServos() {
    baseServo.write(baseAngle);
    shoulderServo.write(shoulderAngle);
    elbowServo.write(elbowAngle);
    wristServo.write(wristAngle);
}
