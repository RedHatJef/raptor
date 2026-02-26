#pragma once
#include <Arduino.h>

// ── Tuning ────────────────────────────────────────────────────────────────────

// I2C address: SDO pin tied to VCC on the Feather nRF52840 Sense → 0x77.
static constexpr uint8_t  BMP280_ADDR               = 0x77;

// How often the cached readings are refreshed (milliseconds).
// The sensor runs free-running in Normal mode; this just controls how often
// we re-read its output registers.
static constexpr uint16_t BMP280_UPDATE_INTERVAL_MS  = 1000;

// ── Class ─────────────────────────────────────────────────────────────────────
class BMP280
{
public:
    bool setup();
    void loop();

    float tempC()        const;  ///< Cached temperature in °C
    float tempF()        const;  ///< Cached temperature in °F
    float pressureHPa()  const;  ///< Cached pressure in hPa
    float pressureInHg() const;  ///< Cached pressure in inHg

    void printCached() const;    ///< Print all cached readings to Serial

private:
    bool     _isUp        = false;
    float    _tempC       = NAN;
    float    _pressureHPa = NAN;
    uint32_t _lastMs      = 0;
};
