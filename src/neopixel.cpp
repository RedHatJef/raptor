#include "neopixel.h"

static NeoPixelColor lastColor;

// ── Public: setup ─────────────────────────────────────────────────────────────
void NeoPixel::setup()
{
    // Some Feather nRF52840 boards require a dedicated power-enable pin to be
    // driven high before the NeoPixel will respond.
#ifdef NEOPIXEL_POWER
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    delay(5); // allow the power rail to stabilise
#endif

    _pixels.begin();
    delay(10); // allow the NeoPixel to initialise
    applyColor(REST_COLOR);
    _state = REST;
}

// ── Public: loop ──────────────────────────────────────────────────────────────
void NeoPixel::loop()
{
    switch (_state)
    {
    case INIT:
        // Hardware not initialised yet; nothing to do until setup() is called.
        break;

    case REST:
        // Steady resting colour — nothing to update each tick.
        break;

    case IMPACT_STARTED:
        // Holding amber while the IMU FIFO drains.  captureComplete() will
        // advance us to IMPACT_DONE once the busy work is finished.
        break;

    case IMPACT_DONE:
        // Drain is finished.  Show the full impact colour, stamp the fade
        // timer, and drop straight into FADE on this tick.
        applyColor(IMPACT_COLOR);
        _fadeStart = millis();
        _state     = FADE;
        break;

    case FADE:
    {
        uint32_t elapsed = millis() - _fadeStart;
        if (elapsed >= NEO_FADE_DURATION_MS)
        {
            applyColor(REST_COLOR);
            _state = REST;
        }
        else
        {
            // t: 0.0 = start of fade (impact colour), 1.0 = end of fade (rest colour).
            float         t     = (float)elapsed / (float)NEO_FADE_DURATION_MS;
            NeoPixelColor color = NeoPixelColor::lerp(IMPACT_COLOR, REST_COLOR, t);
            if (color != lastColor)
            {
                // Serial.print(F("NeoPixel: fading t="));
                // Serial.print(t, 2);
                // Serial.print(F("  "));
                // color.print();
                // Serial.println();
                applyColor(color);
                lastColor.set(color);
            }
        }
        break;
    }
    }
}

// ── Public: trigger ───────────────────────────────────────────────────────────
void NeoPixel::trigger()
{
    applyColor(CAPTURING_COLOR);
    _state = IMPACT_STARTED;
}

// ── Public: captureComplete ───────────────────────────────────────────────────
void NeoPixel::captureComplete()
{
    _state = IMPACT_DONE;
}

// ── Private: applyColor ───────────────────────────────────────────────────────
void NeoPixel::applyColor(const NeoPixelColor &color)
{
    _pixels.setBrightness(color.brightness);
    _pixels.setPixelColor(0, color.r, color.g, color.b);
    _pixels.show();
}
