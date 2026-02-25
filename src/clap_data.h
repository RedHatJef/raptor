#pragma once
#include <Arduino.h>

struct ClapData
{
    int32_t timestamp = 0;        ///< timestamp of reading, relative to the event, in micros()
    float accX = 0;               ///< Last reading's accelerometer X axis m/s^2
    float accY = 0;               ///< Last reading's accelerometer Y axis m/s^2
    float accZ = 0;               ///< Last reading's accelerometer Z axis m/s^2
    float gyroX = 0;              ///< Last reading's gyro X axis in rad/s
    float gyroY = 0;              ///< Last reading's gyro Y axis in rad/s
    float gyroZ = 0;              ///< Last reading's gyro Z axis in rad/s

    void clear() {
        timestamp = 0;
        accX = accY = accZ = 0;
        gyroX = gyroY = gyroZ = 0;
    }
};