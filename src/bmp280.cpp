#include "bmp280.h"
#include <Adafruit_BMP280.h>

static Adafruit_BMP280 _bmp;

// ── Public: setup ──────────────────────────────────────────────────────────────
bool BMP280::setup()
{
    _isUp = false;

    if (!_bmp.begin(BMP280_ADDR)) {
        Serial.println(F("BMP280: startup failed."));
        return false;
    }

    // Maximum accuracy / minimum noise configuration (BMP280 datasheet §3.3–3.4):
    //
    //   Normal mode      — sensor runs free-running; no need to trigger reads.
    //
    //   Temperature ×2   — required input to the pressure compensation formula;
    //                       ×2 gives a 17-bit result with negligible added noise.
    //
    //   Pressure ×16     — highest oversampling: 20-bit result, RMS noise ≈ 0.0016 hPa
    //                       (≈ 0.013 m altitude equivalent).
    //
    //   IIR filter ×16   — exponential moving average that heavily damps transient
    //                       spikes (door slams, vibration).  The filter reaches 75 %
    //                       of a step change in ~21 samples; at the ~83 Hz output
    //                       rate this corresponds to ~250 ms — well within 1 s.
    //
    //   Standby 1 ms     — shortest inter-measurement gap; keeps the IIR filter
    //                       running at ~83 Hz so it has plenty of samples to work with.
    _bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,    // temperature oversampling
        Adafruit_BMP280::SAMPLING_X16,   // pressure oversampling
        Adafruit_BMP280::FILTER_X16,     // IIR filter coefficient
        Adafruit_BMP280::STANDBY_MS_1    // 0.5 ms standby (shortest option)
    );

    _isUp = true;
    Serial.println(F("BMP280: started (Normal mode, P x16, IIR x16)."));
    return true;
}

// ── Public: loop ──────────────────────────────────────────────────────────────
// The sensor runs free-running in Normal mode.  We simply re-read its output
// registers every UPDATE_INTERVAL_MS; by then the IIR filter has long settled.
void BMP280::loop()
{
    if (!_isUp) return;

    uint32_t now = millis();
    if (now - _lastMs < BMP280_UPDATE_INTERVAL_MS) return;
    _lastMs = now;

    float t = _bmp.readTemperature();  // °C
    float p = _bmp.readPressure();     // Pascals

    if (!isnan(t)) _tempC       = t;
    if (!isnan(p)) _pressureHPa = p / 100.0f;
}

// ── Getters ───────────────────────────────────────────────────────────────────
float BMP280::tempC()        const { return _tempC; }
float BMP280::tempF()        const { return isnan(_tempC)       ? NAN : _tempC * 9.0f / 5.0f + 32.0f; }
float BMP280::pressureHPa()  const { return _pressureHPa; }
float BMP280::pressureInHg() const { return isnan(_pressureHPa) ? NAN : _pressureHPa / 33.8639f; }

// ── Print ─────────────────────────────────────────────────────────────────────
void BMP280::printCached() const
{
    if (isnan(_tempC) || isnan(_pressureHPa)) {
        Serial.println(F("BMP280: no data yet."));
        return;
    }
    Serial.print(F("BMP280: Temp="));
    Serial.print(_tempC,        1);  Serial.print(F("C ("));
    Serial.print(tempF(),       1);  Serial.print(F("F)"));
    Serial.print(F("  Pressure="));
    Serial.print(_pressureHPa,  2);  Serial.print(F(" hPa ("));
    Serial.print(pressureInHg(), 4); Serial.println(F(" inHg)"));
}
