#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("========================================");
  Serial.println("WeAct ESP32 Hello World");
  Serial.println("Board: ESP32-DOWD-V3");
  Serial.println("Serial output is working.");
  Serial.println("========================================");
}

void loop() {
  Serial.println("Hello from ESP32!");
  delay(2000);
}
