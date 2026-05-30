#pragma once
#include <Arduino.h>
#include <math.h>

// ── Safety clamps ─────────────────────────────────────────────────────────────
// Kp: at 0.10, a 10° error produces full-scale correction — anything higher
//     risks oscillation on a floating platform.
// Kd: at 0.05, a gyro rate of 20 °/s produces full-scale damping.
// Dead zone: below ~2° the magnetometer noise floor makes corrections pointless.
static constexpr float KP_MAX     = 0.10f;
static constexpr float KD_MAX     = 0.05f;
static constexpr float DZ_MIN     =  1.0f;  // degrees
static constexpr float DZ_MAX     = 20.0f;  // degrees

// Fraction of STEERING_STRENGTH the controller may apply at most.
// Keeps autopilot corrections weaker than full manual steering.
static constexpr float CORR_LIMIT =  0.80f;

// ── Sign convention ───────────────────────────────────────────────────────────
// +1 or -1. Flip this if the duck corrects in the wrong direction on the water.
static constexpr float HEADING_CORR_SIGN = 1.0f;

class HeadingCtrl {
public:
    float kp        = 0.025f;
    float kd        = 0.015f;
    float dead_zone =   5.0f;
    bool  enabled   =  false;

    float target() const { return _target; }

    // Set gains — clamped to safe limits. Call from MQTT callback.
    void set_gains(float new_kp, float new_kd, float new_dz) {
        kp        = constrain(new_kp, 0.0f, KP_MAX);
        kd        = constrain(new_kd, 0.0f, KD_MAX);
        dead_zone = constrain(new_dz, DZ_MIN, DZ_MAX);
    }

    // Call when manual steering is active or duck is stopped.
    void unlock() { _target = -1.0f; }

    // Tilt-compensated heading using the full 3D magnetic field + accelerometer.
    // Returns NAN when sensor data is implausible (shock, free-fall, etc.).
    // Falls back to simple mag.hdg in those cases.
    //
    // Inputs: accel in mg (milli-g), mag field in µT
    // Output: 0–360 °, 0 = magnetic north, 90 = east
    static float tilt_heading(
        float ax_mg, float ay_mg, float az_mg,
        float mx_uT, float my_uT, float mz_uT)
    {
        // Normalize accel to unit vector (should be ≈ 1 g = 1000 mg when still)
        float ax = ax_mg * 1e-3f, ay = ay_mg * 1e-3f, az = az_mg * 1e-3f;
        float an = sqrtf(ax*ax + ay*ay + az*az);
        if (an < 0.5f || an > 2.0f) return NAN;  // implausible — reject
        ax /= an;  ay /= an;  az /= an;

        // Pitch and roll from gravity vector
        float roll  = atan2f(ay, az);
        float pitch = asinf(constrain(-ax, -1.0f, 1.0f));

        // Rotate magnetic field into the horizontal plane (Freescale AN4248)
        float cp = cosf(pitch), sp = sinf(pitch);
        float cr = cosf(roll),  sr = sinf(roll);
        float mx_c =  mx_uT * cp                      + mz_uT * sp;
        float my_c =  mx_uT * sp * sr + my_uT * cr   - mz_uT * cp * sr;

        float h = atan2f(-my_c, mx_c) * (180.0f / (float)M_PI);
        if (h < 0.0f) h += 360.0f;
        return h;
    }

    // Main update — call every time a new sensor packet arrives (10 Hz).
    //
    // current_hdg    : heading in degrees (tilt-compensated preferred)
    // gyro_z_dps     : yaw rate in °/s from IMU (positive = clockwise)
    // manual_steering: true when joystick x-axis is active
    //
    // Returns a correction in [-CORR_LIMIT, +CORR_LIMIT] to be multiplied by
    // STEERING_STRENGTH and added to the existing steering delta.
    // Positive = turn left, negative = turn right (same sign as steeringDelta).
    float update(float current_hdg, float gyro_z_dps, bool manual_steering) {
        if (!enabled || isnan(current_hdg)) return 0.0f;

        if (manual_steering) {
            unlock();
            return 0.0f;
        }

        // Re-lock on current heading after a manual turn or at startup
        if (_target < 0.0f) {
            _target = current_hdg;
            return 0.0f;
        }

        // Shortest angular error in [-180, +180]
        float error = _target - current_hdg;
        while (error >  180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        if (fabsf(error) < dead_zone) return 0.0f;

        // PD: P term corrects steady-state drift, D term damps oscillation
        float corr = kp * error - kd * gyro_z_dps;
        return constrain(corr * HEADING_CORR_SIGN, -CORR_LIMIT, CORR_LIMIT);
    }

private:
    float _target = -1.0f;  // negative = not locked yet
};
