#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "neopixelcolor.h"

// ── Tuning ────────────────────────────────────────────────────────────────────

// Onboard NeoPixel pin.  PIN_NEOPIXEL is defined by the board package for the
// Feather nRF52840 Sense; fall back to pin 8 if somehow not defined.
#ifdef PIN_NEOPIXEL
static constexpr uint8_t NEO_PIN = PIN_NEOPIXEL;
#else
static constexpr uint8_t NEO_PIN = 8;
#endif

// Resting state: dim green.
static constexpr NeoPixelColor REST_COLOR      { 0, 255, 0, 1 };

// Capturing state: amber — impact detected, FIFO drain in progress.
static constexpr NeoPixelColor CAPTURING_COLOR { 255, 0, 0, 15 };

// Impact done: bright red — shown once drain is complete, then fades to REST.
static constexpr NeoPixelColor IMPACT_COLOR    { 0, 0, 255, 15 };

// Duration of the fade from impact colour back to resting colour (milliseconds).
static constexpr uint16_t NEO_FADE_DURATION_MS = 1500;

// ── Class ─────────────────────────────────────────────────────────────────────
class NeoPixel
{
public:
    void setup();
    void loop();

    // Call when an impact is first detected (ISR fires).
    // Shows CAPTURING_COLOR and holds until captureComplete() is called.
    void trigger();

    // Call once the FIFO drain is finished and we are no longer busy.
    // Switches to IMPACT_COLOR (red) and begins the fade back to REST.
    void captureComplete();

private:
    enum State { INIT, REST, IMPACT_STARTED, IMPACT_DONE, FADE };

    void applyColor(const NeoPixelColor &color);

    Adafruit_NeoPixel _pixels{1, NEO_PIN, NEO_GRB + NEO_KHZ800};
    State    _state     = INIT;
    uint32_t _fadeStart = 0;
};
