# UART WIRING
## Wiring PSoC6 → ESP32 Feather:
```
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
```

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
#include <Arduino.h>
#include <ArduinoJson.h>

// ── Feather ESP32 (classic HUZZAH32): GPIO16/17 are fine
// ── Feather ESP32 V2: GPIO16 = PSRAM — use something else, e.g. GPIO27/33
#define PSOC_RX_PIN 16
#define PSOC_TX_PIN 17

static char     _rxbuf[256];
static uint16_t _rxpos = 0;

void setup() {
    Serial.begin(115200);

    // Must be called BEFORE begin() — doubles the default RX buffer
    Serial2.setRxBufferSize(512);
    Serial2.begin(115200, SERIAL_8N1, PSOC_RX_PIN, PSOC_TX_PIN);
}

void onSensorLine(const char* line) {
    JsonDocument doc;
    if (deserializeJson(doc, line) != DeserializationError::Ok) {
        Serial.println("bad json");
        return;
    }
    if (!doc["ok"]["imu"] || !doc["ok"]["mag"]) return;

    float heading = doc["mag"]["hdg"];
    float gyro_z  = doc["imu"]["gyro"]["z"];
    // … use values
    Serial.printf("hdg=%.1f  gyro_z=%.2f\n", heading, gyro_z);
}

void loop() {
    // Drain everything available this tick — never blocks
    while (Serial2.available()) {
        char c = (char)Serial2.read();

        if (c == '\n') {
            // Strip trailing \r if present
            if (_rxpos > 0 && _rxbuf[_rxpos - 1] == '\r') _rxpos--;
            _rxbuf[_rxpos] = '\0';

            if (_rxpos > 0) onSensorLine(_rxbuf);
            _rxpos = 0;

        } else if (_rxpos < sizeof(_rxbuf) - 1) {
            _rxbuf[_rxpos++] = c;

        } else {
            // Line longer than buffer — discard and resync
            _rxpos = 0;
        }
    }

    // … rest of your loop (motors, ESP-NOW, etc.)
}
```

A few things to note:

- readStringUntil('\n') blocks until it sees \n — the PSoC sends \r\n so trim() strips the trailing \r.
- Check doc["ok"] before using values — during the first ~500 ms after boot the sensors may not be ready yet.
- JsonDocument (no size parameter) is ArduinoJson 7. If you're on v6 use StaticJsonDocument<256> doc; instead.
- For steering you'll mostly care about mag.hdg (where are we pointing) and imu.gyro.z (how fast are we turning). The accel and mag field X/Y/Z are there when you want tilt compensation later.

## Magnet data hints

### Why does mag.field have X/Y/Z?

The magnetometer measures the full 3D magnetic field vector, not a heading directly. Heading is computed as atan2(field.y, field.x) — but that formula only gives the correct compass bearing when the board is perfectly flat. The moment the duck tilts (waves, acceleration), the answer drifts.

field.z (the vertical dip component) lets the ESP32 do tilt-compensated heading by combining all three magnetic field components with the IMU accelerometer. The formula rotates the magnetic vector out of the tilted frame back into the horizontal plane before computing the angle. Without Z you can't do that.

So the short version: hdg is the fast and easy bearing, fine for calm water. If you want accurate heading while the duck is rocking, the ESP32 can compute a corrected heading from field.x/y/z + imu.accel.x/y/z.
## Thoughts about "Regelung"
❯ Can we use the degree data and its derivative to differentiate degree changes coming from floating on the water from degree changes coming from turning left or right? Is this a good way to differentiate these two kinds of motions or are there better ones (we only want to counteract drifts to the sides while driving and not the usual floating)?

Good question, and the honest answer is: not reliably using those two signals alone. Here's why, and what actually works.

Why heading + turn rate isn't enough

Both drift and intentional turning produce the same signature in those signals — a changing heading and a non-zero turn rate. You could try a threshold ("if turn rate < 3 °/s it's drift, if > 10 °/s it's intentional"), but this breaks whenever:
- You command a slow, gentle correction turn
- A strong current or wind causes fast drift
- Waves make the heading oscillate rapidly at small amplitude

What actually distinguishes them

The real discriminator is whether you commanded the motion. The signal you're missing is the motor/steering output. Once you know that:

┌──────────────────────────┬────────────────────┬────────────────────────────────────────────────┐
│        Situation         │ Commanded steering │            Gyro.z / heading change             │
├──────────────────────────┼────────────────────┼────────────────────────────────────────────────┤
│ Going straight, no drift │ ~zero              │ ~zero                                          │
├──────────────────────────┼────────────────────┼────────────────────────────────────────────────┤
│ Going straight, drifting │ ~zero              │ non-zero ← this is what you want to counteract │
├──────────────────────────┼────────────────────┼────────────────────────────────────────────────┤
│ Intentional turn         │ non-zero           │ non-zero (expected)                            │
├──────────────────────────┼────────────────────┼────────────────────────────────────────────────┤
│ Turn complete, coasting  │ ~zero              │ small residual                                 │
└──────────────────────────┴────────────────────┴────────────────────────────────────────────────┘

Drift is simply unexpected heading change — motion that occurs without a corresponding steering command.

The standard solution: heading hold

Instead of classifying motion, just hold a target heading. This sidesteps the problem entirely:

# Pseudocode on the ESP32 side

```C
target_hdg = None   # set once duck starts straight-line driving

def loop():
    current_hdg = read_psoc_heading()
    joystick_turn = read_joystick_x()

    if joystick_turn != 0:
        # Intentional turn: don't fight it, update target when done
        apply_steering(joystick_turn)
        target_hdg = None  # will re-lock when joystick returns to center

    else:
        if target_hdg is None:
            target_hdg = current_hdg  # lock current heading as target

        # Wrap-corrected error: positive = drifted clockwise, needs left correction
        error = ((target_hdg - current_hdg + 180) % 360) - 180

        if abs(error) > DEAD_ZONE_DEG:   # e.g. 5°
            correction = Kp * error - Kd * gyro_z   # PD controller
            apply_steering(correction)
```

gyro_z from the IMU is the D term here — it damps oscillation so the duck doesn't overshoot and wobble. This is actually the best use of that value; the numerical derivative of heading you're already plotting is the noisier version of the same thing.

Are there better sensors for this?

If you want to be more sophisticated:

- GPS would give you actual lateral displacement (best, but overkill for a pool)
- Optical flow / camera — can detect sideways drift directly from pixel motion
- Accelerometer lateral axis — imu.accel.x (sideways) shows if the duck is being pushed sideways by current; integrating it gives an estimate of lateral velocity. This won't drift-compensate the yaw but tells you why the duck is drifting.

Practical recommendation for the hackathon

The heading-hold PD loop above is the right call. It's ~20 lines on the ESP32, works with the data you're already sending, and doesn't require classifying anything. The wave wobble (high-frequency oscillation in heading) gets filtered out naturally by the dead zone and the D term. What remains and accumulates over time — slow drift — is exactly what the P term corrects.

The compass viewer derivative plot is still useful for tuning: if you see the amber line oscillating while the duck is supposed to go straight, that's wave noise you need to dead-zone out. If it slowly ramps in one direction without a joystick command, that's drift your heading hold will correct.

✻ Sautéed for 52s
