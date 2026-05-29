#include <Arduino.h>

#ifdef ESP8266
/* ESP8266 */

#include <ESP8266WiFi.h>
#include <espnow.h>
// Compat variables
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define pdMS_TO_TICKS(ms) (ms) // no FreeRTOS ticks on ESP8266

const int STICK_X = A0;
const int STICK_Y = D1;

#else
/* ESP32 */
#include <WiFi.h>
#include <esp_now.h>
const int STICK_X = 0;
const int STICK_Y = 1;

#endif
#include "common.h"


uint8_t broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
// Diese MAC-Adresse muss zur MAC deines Empfängers passen.
uint8_t empfaengerMac[] = {0xD4, 0xE9, 0xF4, 0xBD, 0xB5, 0x0C};
// uint8_t empfaengerMac[] = {0x14, 0x63, 0x93, 0xC8, 0x92, 0xC0};

uint8_t *active_mac = broadcast;

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  delay(2000); // Delay for monitor

  // Eigene MAC anzeigen (praktisch zum Kopieren).
  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
    Serial.println("Fehler: ESP-NOW konnte nicht gestartet werden.");

#ifdef ESP8266
  // Register receiver as slave
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  if (esp_now_add_peer(active_mac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != ESP_OK) {
#else
  esp_now_peer_info_t peer_info = {};
  memcpy(&peer_info.peer_addr, active_mac, 6);
  if (esp_now_add_peer(&peer_info) != ESP_OK) {
#endif // ESP8266
    Serial.println("Fehler: Empfaenger konnte nicht hinzugefuegt werden.");
  }

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
  auto res = esp_now_send(NULL, (uint8_t *)&data, sizeof(data));
  if (res == ESP_OK) {
    Serial.println("[OK] Nachricht gesendet");
  } else {
    Serial.println("[ERR] Fehler beim Senden der Nachricht");
  }

  delay(100);
}

