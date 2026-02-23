#pragma once
#include <Arduino.h>

class IMU
{
public:
    bool setup();
    void loop();

    const char* getName() const;

private:
    bool isUP;
};