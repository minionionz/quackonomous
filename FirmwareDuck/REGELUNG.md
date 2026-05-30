# First draft
## What was added

heading_ctrl.h — the PD controller:
- tilt_heading() — rotates the 3D magnetic field vector into the horizontal plane using the accelerometer's pitch/roll, giving a better heading than mag.hdg alone when the duck rocks on waves
- update() — returns a correction in [-0.8, +0.8] (as a fraction of STEERING_STRENGTH)
- set_gains() — clamps to safe limits before accepting any value
- HEADING_CORR_SIGN = 1.0f — flip to -1.0f if the correction steers the wrong direction on the water

main.cpp changes:
- Non-blocking PSoC UART reader on GPIO27 (RX) / GPIO26 (TX) — safe because GPIO16/17 were already taken
- Drift correction mixed into driveEscFromStick, active only when throttle > 5%
- Controller unlocks heading target during manual turns and when stopped
- Publishes duck/heading at 1 Hz with full state for monitoring

---
Calibration from your PC via MQTT

# Enable drift correction
mosquitto_pub -h 10.42.0.1 -t duck/cmd -m "drift_on"

# Send gains (any field can be omitted — missing keys keep current value)
mosquitto_pub -h 10.42.0.1 -t duck/gains -m '{"kp":0.025,"kd":0.015,"dead_zone":5.0}'

# Watch the clamped values that were actually applied
mosquitto_sub -h 10.42.0.1 -t "duck/gains/ack"

# Monitor heading state (1 Hz)
mosquitto_sub -h 10.42.0.1 -t duck/heading

# Emergency stop
mosquitto_pub -h 10.42.0.1 -t duck/cmd -m "stop"

Safety limits enforced silently: Kp ≤ 0.10, Kd ≤ 0.05, dead zone 1°–20°. The ACK topic echoes back the clamped values so you always know what's actually running.
