#include "sht30.h"
#include <Adafruit_SHT31.h>

static Adafruit_SHT31 _sht;

// ── Public: setup ──────────────────────────────────────────────────────────────
bool SHT30::setup()
{
    _isUp = false;

    if (!_sht.begin(SHT30_ADDR)) {
        Serial.println(F("SHT30: startup failed."));
        return false;
    }

    // Disable internal heater — it warms the die and biases temperature readings.
    _sht.heater(false);

    _isUp = true;
    Serial.println(F("SHT30: started (high repeatability, heater off)."));
    return true;
}

// ── Public: loop ──────────────────────────────────────────────────────────────
// Triggers a high-repeatability single-shot measurement every UPDATE_INTERVAL_MS.
// The SHT30 takes ~15.5 ms per high-rep measurement; at 1 Hz this is negligible.
void SHT30::loop()
{
    if (!_isUp) return;

    uint32_t now = millis();
    if (now - _lastMs < SHT30_UPDATE_INTERVAL_MS) return;
    _lastMs = now;

    // readTemperature() and readHumidity() each trigger a full high-repeatability
    // single-shot (command 0x2C06, clock-stretch).  This is the most accurate
    // measurement mode the SHT30 supports.
    float t = _sht.readTemperature();
    float h = _sht.readHumidity();

    if (!isnan(t)) _tempC    = t;
    if (!isnan(h)) _humidity = h;
}

// ── Getters ───────────────────────────────────────────────────────────────────
float SHT30::tempC()    const { return _tempC; }
float SHT30::tempF()    const { return isnan(_tempC)    ? NAN : _tempC * 9.0f / 5.0f + 32.0f; }
float SHT30::humidity() const { return _humidity; }

// ── Print ─────────────────────────────────────────────────────────────────────
void SHT30::printCached() const
{
    if (isnan(_tempC) || isnan(_humidity)) {
        Serial.println(F("SHT30 : no data yet."));
        return;
    }
    Serial.print(F("SHT30 : Temp="));
    Serial.print(_tempC,    1);  Serial.print(F("C ("));
    Serial.print(tempF(),   1);  Serial.print(F("F)"));
    Serial.print(F("  Humidity="));
    Serial.print(_humidity, 1);  Serial.println(F("%RH"));
}
