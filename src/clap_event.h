#pragma once

#include <Arduino.h>
#include <vector>
#include "clap_data.h"

class ClapEvent
{
public:
    /// Constructor
    explicit ClapEvent();

    void clear();

    /// Event timestamp (micros relative to trigger)
    int32_t timestamp = 0;

    /// Environmental data captured at event time
    float temperature = 0;   ///< Degrees C
    float pressure = 0;      ///< Pascals or hPa
    float humidity = 0;      ///< Percent RH

    /// Number of valid entries in samples[]
    uint16_t sampleCount = 0;

    /// All IMU samples associated with this event
    const static uint16_t MAX_SAMPLES = 512;
    ClapData samples[MAX_SAMPLES];
};