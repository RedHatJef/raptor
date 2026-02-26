#pragma once
#include <Arduino.h>

// ── Tuning ────────────────────────────────────────────────────────────────────

// I2C address: ADDR pin tied to GND on the Feather nRF52840 Sense → 0x44.
static constexpr uint8_t  SHT30_ADDR              = 0x44;

// How often the sensor is polled and cached values are updated (milliseconds).
static constexpr uint16_t SHT30_UPDATE_INTERVAL_MS = 1000;

// ── Class ─────────────────────────────────────────────────────────────────────
class SHT30
{
public:
    bool setup();
    void loop();

    float tempC()    const;  ///< Cached temperature in °C
    float tempF()    const;  ///< Cached temperature in °F
    float humidity() const;  ///< Cached relative humidity in %RH

    void printCached() const;  ///< Print all cached readings to Serial

private:
    bool     _isUp     = false;
    float    _tempC    = NAN;
    float    _humidity = NAN;
    uint32_t _lastMs   = 0;
};
