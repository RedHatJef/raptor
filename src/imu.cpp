#include "imu.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LSM6DS.h>

static constexpr bool verbose = true;

// ── Sensitivity (datasheet values at the chosen full-scale ranges) ─────────────
static constexpr float ACCEL_SENS    = 0.488f;                        // mg / LSB  at ±16 g
static constexpr float GYRO_SENS     = 70.0f;                         // mdps / LSB at ±2000 dps
static constexpr float MG_TO_MS2     = 9.80665e-3f;                   // mg  → m/s²
static constexpr float MDPS_TO_RADS  = (float)(M_PI / 180.0) * 1e-3f; // mdps → rad/s

// ── FIFO register addresses (LSM6DS3TR-C datasheet, Table 15) ────────────────
static constexpr uint8_t REG_FIFO_CTRL3      = 0x08; // dataset decimation factors
static constexpr uint8_t REG_FIFO_CTRL5      = 0x0A; // FIFO ODR + mode
static constexpr uint8_t REG_FIFO_STATUS1    = 0x3A; // DIFF_FIFO[7:0]
static constexpr uint8_t REG_FIFO_STATUS2    = 0x3B; // FTH|OVER_RUN|FIFO_FULL|DIFF_FIFO[11:8]
static constexpr uint8_t REG_FIFO_STATUS3    = 0x3C; // FIFO_PATTERN[7:0] — next word type
static constexpr uint8_t REG_FIFO_DATA_OUT_L = 0x3E; // FIFO output, LSB
static constexpr uint8_t REG_FIFO_DATA_OUT_H = 0x3F; // FIFO output, MSB

// FIFO_CTRL5 encoding:
//   bits [6:3] = ODR_FIFO  — 1010 → 6.66 kHz
//   bits [2:0] = FIFO_MODE — 000 bypass / 001 stop-on-full / 011 continuous (ring) / 100 continuous-to-FIFO
static constexpr uint8_t FIFO_CFG_BYPASS      = 0x00; // resets FIFO, clears pattern counter
static constexpr uint8_t FIFO_CFG_FIFO_MODE   = 0x51; // (10<<3)|1 — stop-on-full at 6.66 kHz
static constexpr uint8_t FIFO_CFG_CONTINUOUS  = 0x53; // (10<<3)|3 — ring buffer at 6.66 kHz

// FIFO_CTRL3: DEC_FIFO_G[5:3]=001 (gyro, no decimation) | DEC_FIFO_XL[2:0]=001 (accel, no decimation)
static constexpr uint8_t FIFO_CTRL3_NO_DEC = 0x09;

// FIFO_STATUS2 flag masks
static constexpr uint8_t FIFO_STATUS2_OVER_RUN  = 0x40; // bit 6: FIFO full, new data discarded
static constexpr uint8_t FIFO_STATUS2_FIFO_FULL = 0x20; // bit 5: FIFO at capacity

// ── Adafruit driver instance ──────────────────────────────────────────────────
static Adafruit_LSM6DS3TRC _IMU3TRC;
static const char         *_IMUname = "none";

// ── ISR ───────────────────────────────────────────────────────────────────────
static volatile bool _isrFired = false;
static void imuISR() { _isrFired = true; }

// ── Low-level register helpers (Wire, direct to LSM_ADDR) ────────────────────
static void writeReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t readReg(uint8_t reg)
{
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(LSM_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

// Reads one 16-bit word from the FIFO output registers and advances the
// FIFO read pointer by one word.
static int16_t readFIFOWord()
{
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(REG_FIFO_DATA_OUT_L);
    Wire.endTransmission(false);
    Wire.requestFrom(LSM_ADDR, (uint8_t)2);
    uint8_t lo = Wire.available() ? Wire.read() : 0;
    uint8_t hi = Wire.available() ? Wire.read() : 0;
    return (int16_t)((uint16_t)hi << 8 | lo);
}

// Returns the number of unread 16-bit words currently in the FIFO.
static uint16_t fifoWordCount()
{
    uint8_t s1 = readReg(REG_FIFO_STATUS1);
    uint8_t s2 = readReg(REG_FIFO_STATUS2);
    return ((uint16_t)(s2 & 0x0F) << 8) | s1;
}

// ── printCapture ──────────────────────────────────────────────────────────────
void IMU::printCapture() const
{
    Serial.println(F("\n--- Recoil Event Captured ---"));
    Serial.println(F("dt_us,ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads"));

    for (uint16_t i = 0; i < _clapEvent.sampleCount; i++)
    {
        const ClapData &s = _clapEvent.samples[i];
        Serial.print(s.timestamp);  Serial.print(',');
        Serial.print(s.accX,  4);   Serial.print(',');
        Serial.print(s.accY,  4);   Serial.print(',');
        Serial.print(s.accZ,  4);   Serial.print(',');
        Serial.print(s.gyroX, 4);   Serial.print(',');
        Serial.print(s.gyroY, 4);   Serial.print(',');
        Serial.println(s.gyroZ, 4);
    }

    Serial.print(F("# samples: "));
    Serial.println(_clapEvent.sampleCount);
    Serial.println(F("--- End of capture ---\n"));
}

// ── Public: setup ─────────────────────────────────────────────────────────────
bool IMU::setup()
{
    _isUp = false;

    if (!_IMU3TRC.begin_I2C(LSM_ADDR))
    {
        Serial.println(F("IMU Startup Failed."));
        return false;
    }

    _IMUname = "LSM6DS3TR-C";
    _isUp    = true;

    // Use the Adafruit library to configure sensor ODR and full-scale ranges.
    // These write CTRL1_XL and CTRL2_G; the FIFO ODR is set separately below.
    _IMU3TRC.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    _IMU3TRC.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    _IMU3TRC.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
    _IMU3TRC.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

    if (verbose)
    {
        Serial.print(F("Started IMU: "));
        Serial.println(_IMUname);
    }

    _state = ARMING;
    return true;
}

// ── Public: loop ──────────────────────────────────────────────────────────────
void IMU::loop()
{
    if (!_isUp) return;

    switch (_state)
    {
    case ARMING:
        arm();
        break;

    case ARMED:
        if (_isrFired)
        {
            _isrFired        = false;
            _triggerTime_us  = micros();
            _captureStart_ms = millis();
            _impactFlag      = true;
            detachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN));
            if (verbose) Serial.println(F("IMU: trigger detected"));
            _state = CAPTURING;
        }
        break;

    case CAPTURING:
        // Phase 1 (0 … POST_EVENT_MS ms): ring buffer keeps rolling in Continuous mode.
        //   New post-event samples push old pre-event samples out of the window,
        //   shifting the capture toward the event and beyond.
        //
        // Phase 2 (POST_EVENT_MS elapsed): freeze by switching to stop-on-full WITHOUT
        //   a Bypass reset — Bypass would erase the data.  Since the ring buffer was
        //   full, OVER_RUN fires within one ODR period (603 µs).
        //   A 300 ms safety timeout guards against edge cases.
        //
        // Pre/post split: post ≈ POST_EVENT_MS × 1.66 samples, pre ≈ 341 − post.
        {
            uint32_t elapsed = millis() - _captureStart_ms;

            if (!_fifoFrozen && elapsed >= POST_EVENT_MS)
            {
                writeReg(REG_FIFO_CTRL5, FIFO_CFG_FIFO_MODE);
                _fifoFrozen = true;
            }

            if (_fifoFrozen)
            {
                uint8_t s2      = readReg(REG_FIFO_STATUS2);
                bool    overrun = (s2 & FIFO_STATUS2_OVER_RUN) != 0;
                bool    timeout = elapsed > POST_EVENT_MS + 300;
                if (overrun || timeout)
                {
                    _drainTime_us = micros();
                    if (verbose)
                    {
                        uint8_t s1  = readReg(REG_FIFO_STATUS1);
                        uint8_t fc5 = readReg(REG_FIFO_CTRL5);
                        Serial.print(F("IMU: cap->drain  OR="));
                        Serial.print(overrun ? 'Y' : 'N');
                        Serial.print(F(" TO="));
                        Serial.print(timeout ? 'Y' : 'N');
                        Serial.print(F("  S2=0x")); Serial.print(s2,  HEX);
                        Serial.print(F(" S1=0x"));  Serial.print(s1,  HEX);
                        Serial.print(F(" FC5=0x")); Serial.println(fc5, HEX);
                    }
                    _state = DRAINING;
                }
            }
        }
        break;

    case DRAINING:
        drainFIFO();
        printCapture();
        _drainDoneFlag = true;
        delay(REARM_DELAY_MS);
        _state = ARMING;
        break;
    }
}

// ── Private: arm ──────────────────────────────────────────────────────────────
void IMU::arm()
{
    _fifoFrozen = false;

    // 1. Bypass mode resets the FIFO and clears the pattern counter.
    writeReg(REG_FIFO_CTRL5, FIFO_CFG_BYPASS);
    delayMicroseconds(200);

    // 2. Include both gyro (dataset 2) and accel (dataset 1) in the FIFO,
    //    with no decimation so every ODR sample is stored.
    writeReg(REG_FIFO_CTRL3, FIFO_CTRL3_NO_DEC);

    // 3. Start FIFO in Continuous (ring-buffer) mode at 1.66 kHz.
    //    The FIFO continuously overwrites the oldest samples so there is always
    //    ~205 ms of pre-event data available.  At trigger time we switch to
    //    stop-on-full (FIFO mode) in software, preserving the ring-buffer content.
    //    (Hardware Continuous-to-FIFO mode 0x44 requires INT2 as its trigger and
    //    does not fill the ring buffer via INT1, so we replicate the behaviour here.)
    writeReg(REG_FIFO_CTRL5, FIFO_CFG_CONTINUOUS);

    // 4. Configure wake-up threshold and duration via the Adafruit library
    //    (writes TAP_CFG, WAKE_UP_THS, WAKE_UP_DUR).
    _IMU3TRC.enableWakeup(true, /*duration=*/0, /*thresh=*/WAKE_THRESHOLD_LSB);

    // 5. Route the wake-up interrupt to INT1 (writes MD1_CFG via Adafruit library).
    _IMU3TRC.configInt1(false, false, false, /*step=*/false, /*wakeup=*/true);

    // 6. Attach edge-triggered ISR and wait for the event.
    _isrFired = false;
    attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imuISR, RISING);

    if (verbose) Serial.println(F("IMU: armed (Continuous ring-buffer mode, 6.66 kHz)"));
    _state = ARMED;
}

// ── Private: drainFIFO ────────────────────────────────────────────────────────
void IMU::drainFIFO()
{
    _clapEvent.clear();
    uint16_t sampleCount = 0;

    // Determine how many words are in the FIFO.  After switching from Continuous
    // to FIFO mode, the hardware resets DIFF_FIFO to 0 even though the physical
    // buffer holds 2048 words.  FIFO_FULL and OVER_RUN are sticky flags that
    // reliably indicate the buffer is full, so we use them as a fallback.
    uint8_t  s2    = readReg(REG_FIFO_STATUS2);
    uint16_t words = fifoWordCount();
    if (words == 0 && (s2 & (FIFO_STATUS2_FIFO_FULL | FIFO_STATUS2_OVER_RUN)))
    {
        words = 2048;
    }

    if (verbose)
    {
        Serial.print(F("IMU: draining ~"));
        Serial.print(words / 6);
        Serial.println(F(" samples"));
    }

    // Read samples until words is exhausted or the output buffer is full.
    //
    // CRITICAL: Check FIFO_STATUS3 (the pattern register) before EVERY sample.
    // FIFO_STATUS3 reports the type of the next word to be read:
    //   0=Gx, 1=Gy, 2=Gz, 3=Ax, 4=Ay, 5=Az
    // If it is non-zero we are mid-sample — an extra word slipped in somewhere
    // (step counter, timestamp, or late write in continuous mode).  Discard the
    // remainder of the partial sample so the next read starts on a clean Gx boundary.
    // Without this per-sample check, one slip corrupts all subsequent samples until
    // the next drain — which is what causes the rotating +1g anomaly in ax/ay/az.
    //
    // words is decremented manually rather than re-reading DIFF_FIFO each iteration,
    // because DIFF_FIFO may stay at 0 (per above) even while the FIFO has data.
    while (sampleCount < BUF_CAPACITY)
    {
        if (words < 6) break;

        // ── Per-sample alignment check ────────────────────────────────────────
        uint8_t pattern = readReg(REG_FIFO_STATUS3);
        if (pattern != 0)
        {
            uint8_t skip = 6 - pattern; // words to reach the next Gx boundary
            if (skip >= words) break;
            for (uint8_t j = 0; j < skip; j++) readFIFOWord();
            words -= skip;
            if (words < 6) break;
        }

        // ── Read one complete sample: Gx, Gy, Gz, Ax, Ay, Az ─────────────────
        int16_t rawGx = readFIFOWord();
        int16_t rawGy = readFIFOWord();
        int16_t rawGz = readFIFOWord();
        int16_t rawAx = readFIFOWord();
        int16_t rawAy = readFIFOWord();
        int16_t rawAz = readFIFOWord();
        words -= 6;

        ClapData &d = _clapEvent.samples[sampleCount++];
        d.accX  = rawAx * ACCEL_SENS * MG_TO_MS2;
        d.accY  = rawAy * ACCEL_SENS * MG_TO_MS2;
        d.accZ  = rawAz * ACCEL_SENS * MG_TO_MS2;
        d.gyroX = rawGx * GYRO_SENS  * MDPS_TO_RADS;
        d.gyroY = rawGy * GYRO_SENS  * MDPS_TO_RADS;
        d.gyroZ = rawGz * GYRO_SENS  * MDPS_TO_RADS;
    }

    _clapEvent.sampleCount = sampleCount;

    // Assign timestamps now that we know the final sampleCount.
    // samples[0] = oldest (most negative dt), samples[N-1] = newest (~0).
    // The last sample in the FIFO corresponds to _drainTime_us; work backwards.
    for (uint16_t i = 0; i < sampleCount; i++)
    {
        _clapEvent.samples[i].timestamp =
            (int32_t)(_drainTime_us - _triggerTime_us)
            - (int32_t)(sampleCount - 1 - i) * (int32_t)SAMPLE_INTERVAL_US;
    }

    // ── Gravity compensation ──────────────────────────────────────────────────
    // Average the pre-event samples (timestamp < 0) to estimate the static
    // gravity vector, then subtract it from every sample so the accel channels
    // represent only dynamic acceleration (gravity ≈ 0 at rest).
    {
        float    sumX = 0, sumY = 0, sumZ = 0;
        uint16_t n    = 0;
        for (uint16_t i = 0; i < sampleCount; i++)
        {
            if (_clapEvent.samples[i].timestamp < 0)
            {
                sumX += _clapEvent.samples[i].accX;
                sumY += _clapEvent.samples[i].accY;
                sumZ += _clapEvent.samples[i].accZ;
                n++;
            }
        }
        if (n > 0)
        {
            float bX = sumX / n, bY = sumY / n, bZ = sumZ / n;
            for (uint16_t i = 0; i < sampleCount; i++)
            {
                _clapEvent.samples[i].accX -= bX;
                _clapEvent.samples[i].accY -= bY;
                _clapEvent.samples[i].accZ -= bZ;
            }
            if (verbose)
            {
                Serial.print(F("IMU: gravity bias  bX="));  Serial.print(bX, 3);
                Serial.print(F(" bY="));                     Serial.print(bY, 3);
                Serial.print(F(" bZ="));                     Serial.print(bZ, 3);
                Serial.print(F("  ("));                      Serial.print(n);
                Serial.println(F(" pre-event samples)"));
            }
        }
    }
}

const char *IMU::getName() const
{
    return _IMUname;
}

bool IMU::takeImpactFlag()
{
    if (!_impactFlag) return false;
    _impactFlag = false;
    return true;
}

bool IMU::takeDrainDoneFlag()
{
    if (!_drainDoneFlag) return false;
    _drainDoneFlag = false;
    return true;
}

void IMU::softTrigger()
{
    _isrFired = true;
}
