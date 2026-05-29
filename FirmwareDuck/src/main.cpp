#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include "common.h"

Servo esc_L;
Servo esc_R;

const int X_MIN = 0;
const int X_MAX = 1024;
const int Y_MIN = 0;
const int Y_MAX = 1024;

const int MOTOR_MIN = 1000; // Minimaler PWM-Wert für die Motoren
const int MOTOR_MAX = 2000; // Maximaler PWM-Wert für die Motoren

const int X_DEADZONE = 60; // Toter Bereich für die X-Achse
const int Y_DEADZONE = 60; // Toter Bereich für die Y-Achse
const int X_CENTER = 312;  // Zentrum der X-Achse
const int Y_CENTER = 512;  // Zentrum der Y-Achse

int Motor_R = 0;
int Motor_L = 1;

static int mapToEsc(int value, int inMin, int inMax)
{
  return constrain(map(value, inMin, inMax, MOTOR_MIN, MOTOR_MAX), MOTOR_MIN, MOTOR_MAX);
}

static void driveEscFromStick(const StickData &stickData)
{
  const int x = stickData.x;
  const int y = stickData.y;

  int motorLeft = MOTOR_MIN;
  int motorRight = MOTOR_MIN;

  if (x < X_CENTER - X_DEADZONE)
  {
    motorLeft = constrain(map(x, X_MIN, X_CENTER - X_DEADZONE, MOTOR_MAX, MOTOR_MIN), MOTOR_MIN, MOTOR_MAX);
    motorRight = MOTOR_MIN;
  }
  else if (x > X_CENTER + X_DEADZONE)
  {
    motorLeft = MOTOR_MIN;
    motorRight = constrain(map(x, X_CENTER + X_DEADZONE, X_MAX, MOTOR_MIN, MOTOR_MAX), MOTOR_MIN, MOTOR_MAX);
  }
  else
  {
    if (y > Y_CENTER + Y_DEADZONE)
    {
      const int throttle = constrain(map(y, Y_CENTER + Y_DEADZONE, Y_MAX, MOTOR_MIN, MOTOR_MAX), MOTOR_MIN, MOTOR_MAX);
      motorLeft = throttle;
      motorRight = throttle;
    }
  }

  esc_L.writeMicroseconds(motorLeft);
  esc_R.writeMicroseconds(motorRight);
}

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

  Serial.print(" | Message: ");
  if (len >= static_cast<int>(sizeof(Message)))
  {
    Message message;
    memcpy(&message, data, sizeof(Message));

    Serial.print(msg_type_name(message.msg_type));

    if (message.msg_type == STICK_DATA)
    {
      Serial.print(" x=");
      Serial.print(message.data.stick_data.x);
      Serial.print(", y=");
      Serial.print(message.data.stick_data.y);

      driveEscFromStick(message.data.stick_data);
    }
    else if (message.msg_type == QUACK)
    {
      Serial.print(" i=");
      Serial.print(message.data.i);
    }
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
  // esc_R.writeMicroseconds(1200); // Langsam drehen
  // esc_L.writeMicroseconds(1200); // Langsam drehen
  // delay(2000);

  // esc_R.writeMicroseconds(1500); // Mehr Gas
  // esc_L.writeMicroseconds(1500); // Mehr Gas
  // delay(2000);

  // esc_R.writeMicroseconds(1000); // Stop
  // esc_L.writeMicroseconds(1000); // Stop
  delay(2000);
}
