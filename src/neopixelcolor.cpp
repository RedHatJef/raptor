#include "neopixelcolor.h"

static uint8_t lerpCh(uint8_t from, uint8_t to, float t)
{
    return (uint8_t)((float)from + ((float)to - (float)from) * t);
}

NeoPixelColor NeoPixelColor::lerp(const NeoPixelColor &from, const NeoPixelColor &to, float t)
{
    return NeoPixelColor(
        lerpCh(from.r,          to.r,          t),
        lerpCh(from.g,          to.g,          t),
        lerpCh(from.b,          to.b,          t),
        lerpCh(from.brightness, to.brightness, t)
    );
}

void NeoPixelColor::set(const NeoPixelColor &other)
{
    r          = other.r;
    g          = other.g;
    b          = other.b;
    brightness = other.brightness;
}

bool NeoPixelColor::operator==(const NeoPixelColor &other) const
{
    return r == other.r && g == other.g && b == other.b && brightness == other.brightness;
}

bool NeoPixelColor::operator!=(const NeoPixelColor &other) const
{
    return !(*this == other);
}

void NeoPixelColor::print() const
{
    Serial.print(F("NeoPixelColor(r="));
    Serial.print(r);
    Serial.print(F(", g="));
    Serial.print(g);
    Serial.print(F(", b="));
    Serial.print(b);
    Serial.print(F(", brightness="));
    Serial.print(brightness);
    Serial.print(')');
}
