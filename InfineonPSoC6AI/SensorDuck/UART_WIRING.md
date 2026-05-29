# UART WIRING
## Wiring PSoC6 → ESP32 Feather:

┌────────────────────────┬─────────────────┬────────────────────────────────┐
│   CY8CKIT-062S2-AI     │  ESP32 Feather  │              Note              │
│    (Arduino header)    │                 │                                │
├────────────────────────┼─────────────────┼────────────────────────────────┤
│ P9_1 (Arduino D1)      │ Any free RX pin │ PSoC TX → ESP RX               │
│                        │  (e.g. GPIO16)  │                                │
├────────────────────────┼─────────────────┼────────────────────────────────┤
│                        │ Any free TX pin │ PSoC RX ← ESP TX (optional,    │
│ P9_0 (Arduino D0)      │  (e.g. GPIO17)  │ only needed if ESP sends       │
│                        │                 │ commands back)                 │
├────────────────────────┼─────────────────┼────────────────────────────────┤
│ GND                    │ GND             │ Required — common ground       │
└────────────────────────┴─────────────────┴────────────────────────────────┘

Both sides are 3.3 V logic so no level shifting needed. On the ESP32 side use Serial2.begin(115200) and read lines with Serial2.readStringUntil('\n'), then parse with ArduinoJson. The "imu":true,"mag":true fields let you skip malformed packets.

## How do I parse this on the ESP32 side?

### platformio.ini
Use ArduinoJson. Add it to Firmware Duck/platformio.ini:
```YAML
lib_deps =
    bblanchon/ArduinoJson@^7
```

### Sketch
Then in your sketch:

```C
#include <ArduinoJson.h>

// PSoC6 P9_1 (TX) → GPIO16, P9_0 (RX) ← GPIO17
#define PSOC_RX_PIN 16
#define PSOC_TX_PIN 17

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, PSOC_RX_PIN, PSOC_TX_PIN);
}

void loop() {
    if (!Serial2.available()) return;

    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) return;

    JsonDocument doc;
    if (deserializeJson(doc, line) != DeserializationError::Ok) return;
    if (!doc["ok"]["imu"] || !doc["ok"]["mag"]) return;  // sensor not ready yet

    uint32_t t      = doc["t"];
    float heading   = doc["mag"]["hdg"];
    float accel_x   = doc["imu"]["accel"]["x"];   // mg
    float accel_y   = doc["imu"]["accel"]["y"];
    float accel_z   = doc["imu"]["accel"]["z"];
    float gyro_z    = doc["imu"]["gyro"]["z"];     // dps — yaw rate, most useful for steering
    float mag_x     = doc["mag"]["field"]["x"];    // µT — for tilt-compensated heading
    float mag_y     = doc["mag"]["field"]["y"];
    float mag_z     = doc["mag"]["field"]["z"];

    // use the values here, e.g.:
    Serial.printf("t=%lu  hdg=%.1f°  yaw=%.2f dps\n", t, heading, gyro_z);
}
’’’

A few things to note:

- readStringUntil('\n') blocks until it sees \n — the PSoC sends \r\n so trim() strips the trailing \r.
- Check doc["ok"] before using values — during the first ~500 ms after boot the sensors may not be ready yet.
- JsonDocument (no size parameter) is ArduinoJson 7. If you're on v6 use StaticJsonDocument<256> doc; instead.
- For steering you'll mostly care about mag.hdg (where are we pointing) and imu.gyro.z (how fast are we turning). The accel and mag field X/Y/Z are there when you want tilt compensation later.
