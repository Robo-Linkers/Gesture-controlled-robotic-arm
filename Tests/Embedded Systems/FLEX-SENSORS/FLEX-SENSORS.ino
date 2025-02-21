const int flexPin1 = 4;  // First flex sensor connected to GPIO4
const int flexPin2 = 2;  // Second flex sensor connected to GPIO2

void setup() {
  Serial.begin(115200); 
  analogSetAttenuation(ADC_11db);  // Configure ADC for a wider input range-> https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
}

void loop() {

  int flexValue1 = analogRead(flexPin1);
  Serial.print("Flex Sensor 1 (GPIO4): ");
  Serial.println(flexValue1);


  int flexValue2 = analogRead(flexPin2);
  Serial.print("Flex Sensor 2 (GPIO2): ");
  Serial.println(flexValue2);

  delay(100);  
}
