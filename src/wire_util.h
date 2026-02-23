#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace WireUtil
{
    /**
     * Attempt to recover a stuck I2C bus by:
     *  - Checking SDA/SCL levels
     *  - Clocking SCL if SDA is stuck low
     *  - Forcing a STOP condition
     *
     * @param sdaPin SDA pin number
     * @param sclPin SCL pin number
     * @return true if bus lines are released (both high), false otherwise
     */
    bool recoverBus(uint8_t sdaPin = PIN_WIRE_SDA, uint8_t sclPin = PIN_WIRE_SCL);

    /**
     * Scan an I2C bus and collect found device addresses.
     *
     * @param wire       TwoWire instance (Wire or Wire1)
     * @param outList    Buffer to receive found addresses
     * @param maxEntries Size of outList buffer
     * @return number of devices found (up to maxEntries)
     */
    uint8_t scan(TwoWire &wire, uint8_t *outList, uint8_t maxEntries);
}