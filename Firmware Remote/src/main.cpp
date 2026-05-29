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

// Diese MAC-Adresse muss zur MAC deines Empfängers passen.
uint8_t empfaengerMac[] = {0xD4, 0xE9, 0xF4, 0xBD, 0xB5, 0x0C};

int STICK_X = 1;
int STICK_Y = 0;

void OnDataSend(uint8_t *mac_addr, uint8_t sendStatus);
void OnDataRecv(uint8_t *mac_addr, uint8_t *data, uint8_t len);

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  // Eigene MAC anzeigen (praktisch zum Kopieren).
  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
    Serial.println("Fehler: ESP-NOW konnte nicht gestartet werden.");

  u8 *peerInfo = {};
  memcpy(peerInfo, empfaengerMac, 6);

  // Register receiver as slave
  if (esp_now_add_peer(empfaengerMac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != ESP_OK)
    Serial.println("Fehler: Empfaenger konnte nicht hinzugefuegt werden.");
  esp_now_register_send_cb(OnDataSend);
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP-NOW ready");

  pinMode(STICK_X, INPUT);
  pinMode(STICK_Y, INPUT);
}

void loop() {
  uint16_t xValue = analogRead(STICK_X);
  uint16_t yValue = analogRead(STICK_Y);

  Message data = { .msg_type = STICK_DATA, .data = { .x = xValue, .y = yValue } };

  esp_err_t result = esp_now_send(empfaengerMac, (uint8_t *)&data, sizeof(data));

  Serial.print("Stick X: ");
  Serial.print(xValue);
  Serial.print(" Stick Y: ");
  Serial.println(yValue);
  delay(100);
}
void OnDataRecv(uint8_t *mac_addr, uint8_t *data, uint8_t len) {
  Serial.printf("[MSG] Message received from [%xHH]\n", *mac_addr);
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == ESP_OK) {
    Serial.println("[OK] Nachricht gesendet");
  } else {
    Serial.println("[ERR] Fehler beim Senden der Nachricht");
  }
}
