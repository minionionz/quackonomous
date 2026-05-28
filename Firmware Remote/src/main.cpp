#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Diese MAC-Adresse muss zur MAC deines Empfängers passen.
uint8_t empfaengerMac[] = {0xD4, 0xE9, 0xF4, 0xBD, 0xB5, 0x0C};

int STICK_X = 1;
int STICK_Y = 0;

struct __attribute__((packed)) StickData
{
  uint16_t x;
  uint16_t y;
};

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  // Eigene MAC anzeigen (praktisch zum Kopieren).
  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
    Serial.println("Fehler: ESP-NOW konnte nicht gestartet werden.");

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, empfaengerMac, 6);

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
    Serial.println("Fehler: Empfaenger konnte nicht hinzugefuegt werden.");

  Serial.println("ESP-NOW ready");

  pinMode(STICK_X, INPUT);
  pinMode(STICK_Y, INPUT);
}

void loop() {
  uint16_t xValue = analogRead(STICK_X);
  uint16_t yValue = analogRead(STICK_Y);

  StickData data = {xValue, yValue};

  esp_err_t result = esp_now_send(empfaengerMac, (uint8_t *)&data, sizeof(data));

  if (result == ESP_OK) {
    Serial.println("Nachricht gesendet");
  } else {
    Serial.println("Fehler beim Senden der Nachricht");
  }

  Serial.print("Stick X: ");
  Serial.print(xValue);
  Serial.print(" Stick Y: ");
  Serial.println(yValue);
  delay(100);
}