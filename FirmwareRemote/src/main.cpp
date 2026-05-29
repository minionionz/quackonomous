#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "common.h"

#ifndef ESP32_COMPAT_H
#define ESP32_COMPAT_H
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define pdMS_TO_TICKS(ms) (ms) // no FreeRTOS ticks on ESP8266
#endif

const int STICK_X = A0;
const int STICK_Y = D1;

// Diese MAC-Adresse muss zur MAC deines Empfängers passen.
// uint8_t empfaengerMac[] = {0xD4, 0xE9, 0xF4, 0xBD, 0xB5, 0x0C};
uint8_t empfaengerMac[] = {0x14, 0x63, 0x93, 0xC8, 0x92, 0xC0};


void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == ESP_OK) {
    Serial.println("[OK] Nachricht gesendet");
  } else {
    Serial.println("[ERR] Fehler beim Senden der Nachricht");
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  delay(2000); // Delay for monitor

  // Eigene MAC anzeigen (praktisch zum Kopieren).
  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
    Serial.println("Fehler: ESP-NOW konnte nicht gestartet werden.");

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // Register receiver as slave
  if (esp_now_add_peer(empfaengerMac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != ESP_OK)
    Serial.println("Fehler: Empfaenger konnte nicht hinzugefuegt werden.");

  esp_now_register_send_cb(OnDataSent);

  Serial.println("ESP-NOW ready");

  pinMode(STICK_X, INPUT);
  pinMode(STICK_Y, INPUT);
}

void loop() {
  uint16_t xValue = analogRead(STICK_X);
  uint16_t yValue = analogRead(STICK_Y);

  Serial.print("Stick X: ");
  Serial.print(xValue);
  Serial.print(" Stick Y: ");
  Serial.println(yValue);

  Message data;
  data.msg_type = STICK_DATA;
  data.data.stick_data = StickData { .x = xValue, .y = yValue };
  esp_now_send(empfaengerMac, (uint8_t *)&data, sizeof(data));

  delay(100);
}

