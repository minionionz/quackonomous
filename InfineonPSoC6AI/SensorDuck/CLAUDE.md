# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

**Quackonomous** — autonomous rubber duck for Hackaburg 2026 (72h hackathon). The duck races two other autonomous ducks on a pool that has waterfall/infinity-edge hazards. It must detect those hazards and steer away autonomously, while also supporting manual remote control.

## Repository Structure

The repo root is `quackonomous/`. This working directory (`InfineonPSoC6AI/SensorDuck/`) is the PSoC 6 sensor firmware. The full multi-component layout:

```
quackonomous/
├── InfineonPSoC6AI/
│   ├── SensorDuck/          ← THIS PROJECT: PSoC 6 AI board, ModusToolbox/make
│   ├── PSoC6AI/             ← Arduino-style autonomy sketches (reference/staging)
│   ├── InfineonSensors/     ← BMM350 magnetometer driver header
│   └── arduino-xensiv-radar-sensor-bgt60tr13/  ← BGT60TR13C radar lib (git submodule)
├── Firmware Duck/           ← ESP32 (Adafruit Feather) — motor/steering, PlatformIO
├── Firmware Remote/         ← ESP32 — joystick remote, sends via ESP-NOW, PlatformIO
└── FirmwareESPWroom/        ← ESP8266 (ESP-WROOM-02) — ultrasonic sensor test, PlatformIO
```

## Hardware: CY8CKIT-062S2-AI Board Sensors

| Sensor | Type | Interface | Distance Sweet Spot | What it detects |
|--------|------|-----------|---------------------|-----------------|
| **BGT60TR13C** | 60 GHz FMCW radar | SPI (MOSI=41, MISO=42, SCLK=43, CS=44, RST=40) | 0.2 m – 5 m | Waterfall (high Doppler velocity, elevation > 0°), walls, obstacles. Cannot penetrate water. |
| **BMI270** | 6-axis IMU (accel+gyro) | I2C **0x68** (on-board) | N/A (inertial) | Boat agitation/turbulence; range ±16g (0.488 mg/LSB) |
| **BMM350** | Magnetometer | I2C **0x15** (on-board; SDO=high) | N/A | Compass heading. **Needs 2 dummy bytes discarded on every I2C read** (BMM350_DUMMY_BYTES=2). |
| **DPS310** | Pressure/temperature | I2C **0x77** (on-board; ADDR=high) | N/A | Bonus sensor; chip ID 0x10 at reg 0x0D. Standard I2C read (no dummy byte). |
| **PDM Mic** | Microphone | PDM P10_4/P10_5 (on-board) | N/A (audio) | Waterfall noise signature |
| **RCWL-1655** | Ultrasonic (external) | UART P9_0/P9_1 (send `0xFF`, read 4 bytes+checksum) | 2 cm – 450 cm | Obstacle proximity; sudden dropout (→ ∞) = pool edge/waterfall |

**No camera** — the CY8CKIT-062S2-AI has no camera interface. For vision, a Raspberry Pi or separate SBC would be needed (overkill for this hackathon).

## System Architecture & Communication

```
[Joystick Remote ESP32] --(ESP-NOW)--> [Duck ESP32 (Feather)]
                                              ↑
                              [PSoC6 AI Board] --(UART/I2C, wired)--┘
                              (runs state machine, reads all sensors)
```

- **ESP-NOW** connects the remote joystick to the Duck ESP32 wirelessly. The `StickData` struct (`uint16_t x, y`) is the payload. MAC addresses are configured in `include/config.h` in each PlatformIO project.
- **PSoC6 → Duck ESP32**: planned wired connection inside the duck (UART is the simplest; I2C also viable). The PSoC6 state machine outputs navigation commands; the Duck ESP32 translates them to PWM/motor signals.
- In autonomous mode the PSoC6 overrides or supplements the ESP-NOW joystick input on the Duck ESP32.

## Autonomy State Machine (PSoC6AI/PSoC6AI.ino)

The reference logic (Arduino sketch, to be ported into `SensorDuck/main_hello_world.c`):

```
NORMALFAHRT → ERKENNUNG (300 ms debounce) → BESTAETIGEN (2-of-3 vote) → AUSWEICHEN
     ↑                                              │ (false alarm)           │
     └──────────────────────────────────────────────┘                         │
     └─── quiet for T_SEKTOR_FREI (1500 ms) ─────────────────────────────────┘
```

- **Trigger**: radar Doppler ≥ 1 m/s + elevation ≥ 10° **OR** ultrasonic dropout (> 400 cm or no echo)
- **Confirmation** (2-of-3): radar + microphone (RMS > 0.60) + IMU agitation (> 0.50)
- **Hysteresis**: return thresholds are lower (mic < 0.30, IMU < 0.25) to prevent oscillation
- All timing uses `millis()` — no `delay()` in the loop

The four `TODO` stubs to implement: `readRadar()`, `readUltrasonicCm()`, `readImuAgitation()`, `readMicLoudness()`, plus the actuator stubs `setSpeed()`, `steerAwayFrom()`, `steerStraight()`.

## RCWL-1655 Wiring to CY8CKIT-062S2-AI

The RCWL-1655 (waterproof ultrasonic) communicates over 3.3 V UART at 9600 baud.

| Sensor pin | Board pin | Notes |
|------------|-----------|-------|
| VCC | 3.3 V (Arduino header) | **Must be 3.3 V** — sensor logic is 3.3 V |
| GND | GND | |
| TX (sensor output) | **P9_0** (Arduino D0/RX) | Sensor sends distance frames |
| RX (sensor input) | **P9_1** (Arduino D1/TX) | Used to send trigger byte `0xFF` |

**UART protocol** (4-byte frame, trigger-based):
1. MCU sends `0xFF` → sensor takes one measurement (~40 ms)
2. Sensor replies with 4 bytes: `[0xFF, D_H, D_L, SUM]`
   - `distance_mm = (D_H << 8) | D_L`
   - `SUM = (D_H + D_L) & 0xFF` (checksum)
   - Valid range: 20 mm – 4500 mm
3. If your specific unit only sends 3 bytes (no checksum): define `RCWL1655_3BYTE_VARIANT` in `main.c`

## Build Commands

### PSoC 6 SensorDuck (ModusToolbox — run from `InfineonPSoC6AI/SensorDuck/`)

```bash
make getlibs          # fetch/update MTB libraries (first time or after libs/*.mtb changes)
make build            # compile
make program          # build + flash via KitProg3
make clean            # clean build artifacts
make VERBOSE=1 build  # show full compiler commands
```

UART debug: 115200 baud, 8N1, on KitProg3 USB COM port.

### ESP32/ESP8266 PlatformIO (run from respective firmware directory)

```bash
pio run                        # build
pio run --target upload        # build + flash
pio device monitor             # open serial monitor (115200 baud)
pio run --target upload && pio device monitor  # flash and monitor in one step
```

Projects:
- `Firmware Duck/` → `featheresp32` (Adafruit Feather ESP32)
- `Firmware Remote/` → `adafruit_qtpy_esp32c3`
- `FirmwareESPWroom/` → `esp_wroom_02` (ESP8266, ultrasonic test only)

## Key Configuration

- **Duck MAC address**: set in `Firmware Duck/include/config.h` (`REMOTE_MAC`) and `FirmwareESPWroom/include/config.h` (`DUCK_MAC`). Run `Serial.println(WiFi.macAddress())` on each board to find its MAC.
- **Radar SPI pins**: hardcoded as `RSPI_MOSI=41, MISO=42, SCLK=43, CS=44, RST=40` for the CY8CKIT-062S2-AI. Uses `SPIClassPSOC` (not the standard Arduino `SPI`) — guarded by `#ifdef TARGET_APP_CY8CKIT_062S2_AI`.
- **BGT60TR13C library**: lives in `InfineonPSoC6AI/arduino-xensiv-radar-sensor-bgt60tr13/src/`. Key API: `BGT60TR13C(words, isr, cs, rst, chip_freq, spi)`, then `configure_chirp(FSU, RTU, RSU)` → `init_sensor()` → `start_frame()` → loop: `read_distance()` / `get_fft_data()`.
