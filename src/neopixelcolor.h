#pragma once
#include <Arduino.h>

// ── NeoPixelColor ─────────────────────────────────────────────────────────────
// Encapsulates an RGB colour and a brightness level for use with the
// Adafruit NeoPixel API.  Brightness range matches setBrightness(): 0–255.
struct NeoPixelColor
{
    // Brightness range supported by the Adafruit NeoPixel API.
    static constexpr uint8_t MIN_BRIGHTNESS = 1;
    static constexpr uint8_t MAX_BRIGHTNESS = 255;

    uint8_t r          = 0;
    uint8_t g          = 0;
    uint8_t b          = 0;
    uint8_t brightness = 0;

    constexpr NeoPixelColor() = default;
    constexpr NeoPixelColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
        : r(r), g(g), b(b), brightness(brightness) {}

    // Returns a new colour linearly interpolated between 'from' and 'to'.
    // t = 0.0 → from,  t = 1.0 → to.
    static NeoPixelColor lerp(const NeoPixelColor &from, const NeoPixelColor &to, float t);

    // Prints the colour to Serial in human-readable form (no trailing newline).
    void print() const;

    // Copies all fields from 'other' into this colour.
    void set(const NeoPixelColor &other);

    bool operator==(const NeoPixelColor &other) const;
    bool operator!=(const NeoPixelColor &other) const;
};
