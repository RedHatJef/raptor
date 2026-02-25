#pragma once
#include <Arduino.h>

// ── Tuning ────────────────────────────────────────────────────────────────────

// Inertial interrupt threshold.  The LSM6DS3TR-C wake-on-motion threshold
// register is 6-bit, in units of 31.25 mg (at ±16 g full-scale).
// 8 × 31.25 mg = 250 mg ≈ 2.45 m/s² — well above handling noise but well
// below any real recoil impulse.  Increase if you get false triggers.
static constexpr uint8_t WAKE_THRESHOLD_LSB = 8;

// Debounce: ignore re-triggers for this many ms after a capture finishes.
static constexpr uint32_t REARM_DELAY_MS = 2000;

// INT1 is connected to nRF52840 P1.11 = Arduino digital pin 47 on the
// Adafruit Feather nRF52840 Sense.
static constexpr uint8_t IMU_INT1_PIN = 3;  // P1.11 = D3 on Feather nRF52840 Sense

// LSM6DS3TR-C I2C address (SDO/SA0 tied to GND on the Sense board)
static constexpr uint8_t LSM_ADDR = 0x6A;

// ── Sample ────────────────────────────────────────────────────────────────────
struct IMUSample
{
    uint32_t timestamp_us;  // raw micros() at drain time (zero-ref'd on print)
    float    ax, ay, az;    // accelerometer (m/s²)
    float    gx, gy, gz;    // gyroscope     (rad/s)
};

// ── Class ─────────────────────────────────────────────────────────────────────
class IMU
{
public:
    bool setup();
    void loop();
    const char *getName() const;

private:
    // ── register helpers ──────────────────────────────────────────────────────
    void     writeReg(uint8_t reg, uint8_t val);
    uint8_t  readReg(uint8_t reg);

    // ── configuration helpers ─────────────────────────────────────────────────
    void     configureFIFO_Continuous();
    void     configureFIFO_StopOnFull();
    void     configureINT1_Inertial();
    void     configureINT1_FIFOFull();
    void     disableINT1();
    void     clearFIFO();
    uint16_t getFIFOCount();

    // ── drain helpers ─────────────────────────────────────────────────────────
    void     drainFIFO();
    void     printCapture();

    // ── state machine ─────────────────────────────────────────────────────────
    enum class State { ARMING, ARMED, CAPTURING, DRAINING };
    State    _state = State::ARMING;
    bool     isUP   = false;

    // ── sample storage ────────────────────────────────────────────────────────
    // 511 samples × (3 accel + 3 gyro) × 2 bytes raw + overhead — fits easily
    // in the nRF52840's 256 KB RAM.
    static constexpr uint16_t BUF_CAPACITY = 511;
    IMUSample _buf[BUF_CAPACITY];
    uint16_t  _bufCount = 0;

    // ── trigger timing ────────────────────────────────────────────────────────
    uint32_t  _triggerTime_us = 0;  // micros() when inertial interrupt fired
};