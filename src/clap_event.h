#pragma once

#include <Arduino.h>
#include <vector>
#include "clap_data.h"

class ClapEvent
{
public:
    /// Constructor
    explicit ClapEvent(int32_t _timestamp, size_t reserveCount = 0);

    /// Adds a ClapData object to the event
    void addClapData(const ClapData& data);

    /// Optional: reserve memory ahead of time for deterministic performance
    void reserve(size_t count);

    /// Event timestamp (micros relative to trigger)
    int32_t timestamp = 0;

    /// Environmental data captured at event time
    float temperature = 0;   ///< Degrees C
    float pressure = 0;      ///< Pascals or hPa
    float humidity = 0;      ///< Percent RH

    /// All IMU samples associated with this event
    std::vector<ClapData> samples;
};