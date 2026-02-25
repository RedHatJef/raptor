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
// Must be long enough for the ring buffer to fill (~51 ms = 341 samples at 6.66 kHz).
static constexpr uint16_t REARM_DELAY_MS = 500;

// How long to keep the ring buffer running AFTER the trigger fires before freezing it.
// The FIFO holds 341 samples; POST_EVENT_MS controls the pre/post split:
//   post-event samples ≈ POST_EVENT_MS × 6.66
//   pre-event samples  ≈ 341 − post-event samples
// Example: 35 ms → ~233 post-event, ~108 pre-event (~68 % post).
// Set to 0 for all pre-event (snapshot just before the event).
// NOTE: keep POST_EVENT_MS < 51 ms (the full FIFO window at 6.66 kHz = 341 / 6660).
static constexpr uint16_t POST_EVENT_MS = 40;

// Maximum samples to store in ClapEvent (must not exceed ClapData samples[]).
static constexpr uint16_t BUF_CAPACITY = 511;

// Nominal sample interval at 6.66 kHz ODR.
static constexpr uint32_t SAMPLE_INTERVAL_US = 150; // 1 / 6660 Hz ≈ 150 µs

// ── Class ─────────────────────────────────────────────────────────────────────
class IMU
{
public:
    bool        setup();
    void        loop();
    const char *getName() const;
    void        printCapture() const;

    // Returns true (once) the first time it is called after an impact is detected.
    // Clears the flag on read, so subsequent calls return false until the next event.
    bool        takeImpactFlag();

    // Returns true (once) after the FIFO drain and serial print are complete.
    // Clears the flag on read, so subsequent calls return false until the next event.
    bool        takeDrainDoneFlag();

    // Software trigger: equivalent to the hardware INT1 firing.
    // Call from main loop when an external source (e.g. MicPDM) detects an event.
    // Only has effect while the state machine is in ARMED state.
    void        softTrigger();

private:
    enum State { ARMING, ARMED, CAPTURING, DRAINING };

    void arm();
    void drainFIFO();

    State    _state           = ARMING;
    bool     _isUp            = false;
    bool     _fifoFrozen      = false;
    bool     _impactFlag      = false;
    bool     _drainDoneFlag   = false;
    uint32_t _triggerTime_us  = 0;
    uint32_t _drainTime_us    = 0;
    uint32_t _captureStart_ms = 0;

    ClapEvent _clapEvent;
};
