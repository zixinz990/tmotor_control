#define FORCE_SENSOR_PIN A0

void setup() {
  Serial.begin(115200);
}

void loop() {
  int analogReading = analogRead(FORCE_SENSOR_PIN);
  Serial.println(analogReading);
  delay(5);
}
