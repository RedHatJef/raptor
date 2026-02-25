#pragma once
#include <Arduino.h>
#include "clap_event.h"

// ── Tuning ────────────────────────────────────────────────────────────────────

// Wake-on-motion threshold: 6-bit register, units of 31.25 mg at ±16 g.
// 8 × 31.25 mg = 250 mg — above handling noise, well below any recoil impulse.
// Increase if you get false triggers.
static constexpr uint8_t WAKE_THRESHOLD_LSB = 8;

// INT1 connected to nRF52840 P1.11 = Arduino D3 on the Feather nRF52840 Sense.
static constexpr uint8_t IMU_INT1_PIN = 3;

// LSM6DS3TR-C I2C address (SDO/SA0 tied to GND on the Sense board).
static constexpr uint8_t LSM_ADDR = 0x6A;

// Delay (ms) between the end of a drain and the next rearm.
// This also controls the pre/post event balance in Continuous-to-FIFO mode:
// the FIFO window is ~205 ms (341 samples at 1.66 kHz).
//   REARM_DELAY_MS >= 205 → FIFO is full at trigger → ~205 ms pre-event, 0 post.
//   REARM_DELAY_MS <  205 → FIFO partially empty → some post-event data captured.
static constexpr uint16_t REARM_DELAY_MS = 500;

// Maximum samples to store in ClapEvent (must not exceed ClapData samples[]).
static constexpr uint16_t BUF_CAPACITY = 511;

// Nominal sample interval at 1.66 kHz ODR.
static constexpr uint32_t SAMPLE_INTERVAL_US = 603; // 1 / 1660 Hz ≈ 603 µs

// ── Class ─────────────────────────────────────────────────────────────────────
class IMU
{
public:
    bool        setup();
    void        loop();
    const char *getName() const;
    void        printCapture() const;

private:
    enum State { ARMING, ARMED, CAPTURING, DRAINING };

    void arm();
    void drainFIFO();

    State    _state           = ARMING;
    bool     _isUp            = false;
    uint32_t _triggerTime_us  = 0;
    uint32_t _drainTime_us    = 0;
    uint32_t _captureStart_ms = 0;

    ClapEvent _clapEvent;
};
