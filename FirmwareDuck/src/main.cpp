#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

struct __attribute__((packed)) StickData
{
  uint16_t x;
  uint16_t y;
};

// Callback-Funktion, die automatisch aufgerufen wird, wenn eine ESP-NOW-Nachricht empfangen wird
// Sie erhält die MAC-Adresse des Senders und die empfangenen Daten.
void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  // Gib "Von: " aus, gefolgt von der MAC-Adresse des Senders
  Serial.print("Von: ");

  // MAC address des Senders ausgeben
  for (int i = 0; i < 6; i++)
  {
    Serial.printf("%02X", mac_addr[i]);
    if (i < 5)
      Serial.print(":");
  }

  Serial.print(" | StickData: ");
  if (len >= static_cast<int>(sizeof(StickData)))
  {
    StickData stickData;
    memcpy(&stickData, data, sizeof(StickData));
    Serial.print("x=");
    Serial.print(stickData.x);
    Serial.print(", y=");
    Serial.print(stickData.y);
  }
  else
  {
    Serial.print("ungültige Länge (");
    Serial.print(len);
    Serial.print(")");
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
  // onDataRecv wird automatisch aufgerufen, wenn eine ESP-NOW-Nachricht ankommt
  esp_now_register_recv_cb(onDataRecv);

  // Bestätige, dass der Empfänger bereit ist
  Serial.println("Empfänger bereit");
}

void loop()
{
  delay(1000);
}