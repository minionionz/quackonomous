#include <Arduino.h>

#define TRIG_PIN 14   // GPIO14
#define ECHO_PIN 12   // GPIO12

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Trigger-Puls senden
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);   // mind. 10µs HIGH
  digitalWrite(TRIG_PIN, LOW);

  // Echo messen
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout 30ms
  float distance_cm = duration * 0.034 / 2.0;

  Serial.printf("Distanz: %.1f cm\n", distance_cm);
  delay(100);
}