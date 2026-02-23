#include "wire_util.h"

namespace WireUtil
{

    bool recoverBus(uint8_t sda, uint8_t scl)
    {
        pinMode(sda, INPUT_PULLUP);
        pinMode(scl, INPUT_PULLUP);
        delay(2);

        bool sdaHigh = digitalRead(sda);
        bool sclHigh = digitalRead(scl);

        // If SCL is stuck low, recovery usually can't fix it
        if (!sclHigh)
            return false;

        // If SDA is stuck low, attempt clock recovery
        if (!sdaHigh)
        {
            pinMode(scl, OUTPUT);
            digitalWrite(scl, HIGH);
            delayMicroseconds(5);

            for (int i = 0; i < 18; i++) // 9 clock pulses (18 edges)
            {
                digitalWrite(scl, LOW);
                delayMicroseconds(5);
                digitalWrite(scl, HIGH);
                delayMicroseconds(5);

                pinMode(sda, INPUT_PULLUP);
                if (digitalRead(sda) == HIGH)
                    break;
            }
        }

        // Generate STOP condition
        pinMode(sda, OUTPUT);
        digitalWrite(sda, LOW);
        delayMicroseconds(5);

        pinMode(scl, OUTPUT);
        digitalWrite(scl, HIGH);
        delayMicroseconds(5);

        pinMode(sda, INPUT_PULLUP);
        delayMicroseconds(5);

        // Final state check
        pinMode(sda, INPUT_PULLUP);
        pinMode(scl, INPUT_PULLUP);
        delay(2);

        return (digitalRead(sda) == HIGH) &&
               (digitalRead(scl) == HIGH);
    }

    uint8_t scan(TwoWire &wire, uint8_t *outList, uint8_t maxEntries)
    {
        uint8_t count = 0;

        for (uint8_t addr = 8; addr < 120; addr++) // skip reserved ranges
        {
            wire.beginTransmission(addr);
            uint8_t err = wire.endTransmission();

            if (err == 0)
            {
                if (count < maxEntries)
                    outList[count++] = addr;
            }

            delayMicroseconds(300); // pacing for stability
            yield();                // allow background tasks (nRF52 USB/BLE)
        }

        return count;
    }

} // namespace WireUtil