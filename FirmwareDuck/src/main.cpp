#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include "common.h"

Servo esc_L;
Servo esc_R;

int Motor_R = 0;
int Motor_L = 1;

// Callback-Funktion, die automatisch aufgerufen wird, wenn eine ESP-NOW-Nachricht empfangen wird
// Sie erhält die MAC-Adresse des Senders und die empfangenen Daten.
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  // MAC address des Senders ausgeben
  Serial.print("[MSG] From: ");
  for (int i = 0; i < 6; i++)
  {
    printf("%02X", mac_addr[i]);
    if (i < 5)
      Serial.print(":");
  }

  Message msg;
  memcpy(&msg, data, len);
  Serial.printf("[MSG] Message received. Type: %s\n", msg_type_name(msg.msg_type));
  // Idk how unions are handled
  /*
  if (len < static_cast<int>(sizeof(Message)))
  {
    Serial.print("ungültige Länge (");
    Serial.print(len);
    Serial.print(")");
  }
  */
  if (msg.msg_type == STICK_DATA) {
    Serial.print(" | StickData: ");
    StickData stickData = msg.data.stick_data;
    Serial.print("x=");
    Serial.print(stickData.x);
    Serial.print(", y=");
    Serial.print(stickData.y);
  }

  Serial.print(" ");
  Serial.println();
}

// Initialisierung des ESP32
void setup()
{
  Serial.begin(115200);

  // Setze den WiFi-Modus auf Station (Empfänger-Modus)
  WiFi.mode(WIFI_STA);
  delay(2000); // Delay for monitor

  // Zur Information: Die MAC-Adresse des ESP32 wird hier ausgegeben,
  // damit du sie für den Sender verwenden kannst
  Serial.print("Empfänger MAC: ");
  Serial.println(WiFi.macAddress());

  // Initialisiere ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW Fehler");
    while (true)
      delay(1000);
  }

  // Registriere die Callback-Funktion, die aufgerufen wird, wenn Daten empfangen werden
  // OnDataRecv wird automatisch aufgerufen, wenn eine ESP-NOW-Nachricht ankommt
  esp_now_register_recv_cb(OnDataRecv);

  // Bestätige, dass der Empfänger bereit ist
  Serial.println("Empfänger bereit");

  esc_L.attach(Motor_L, 1000, 2000);
  esc_R.attach(Motor_R, 1000, 2000);

  esc_L.writeMicroseconds(1000);
  esc_R.writeMicroseconds(1000);
  
  delay(3000);
}

void loop()
{
  esc_R.writeMicroseconds(1200); // Langsam drehen
  esc_L.writeMicroseconds(1200); // Langsam drehen
  delay(2000);

  esc_R.writeMicroseconds(1500); // Mehr Gas
  esc_L.writeMicroseconds(1500); // Mehr Gas
  delay(2000);

  esc_R.writeMicroseconds(1000); // Stop
  esc_L.writeMicroseconds(1000); // Stop
  delay(2000);
}
}
