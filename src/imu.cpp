#include "imu.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LSM6DS.h>

// ── LSM6DS3TR-C register addresses ───────────────────────────────────────────
// (from ST RM0409 / LSM6DS3TR-C datasheet)
static constexpr uint8_t REG_FUNC_CFG_ACCESS  = 0x01;
static constexpr uint8_t REG_INT1_CTRL        = 0x0D; // FIFO full/watermark on INT1
static constexpr uint8_t REG_WHO_AM_I         = 0x0F;
static constexpr uint8_t REG_CTRL1_XL         = 0x10; // Accel ODR/range
static constexpr uint8_t REG_CTRL2_G          = 0x11; // Gyro ODR/range
static constexpr uint8_t REG_CTRL3_C          = 0x12; // BDU, auto-increment
static constexpr uint8_t REG_CTRL10_C         = 0x19; // embedded func enable
static constexpr uint8_t REG_WAKE_UP_SRC      = 0x1B; // wake interrupt source
static constexpr uint8_t REG_MD1_CFG          = 0x5E; // inertial events → INT1
static constexpr uint8_t REG_WAKE_UP_THS      = 0x5B; // wake-up threshold
static constexpr uint8_t REG_WAKE_UP_DUR      = 0x5C; // wake-up duration
static constexpr uint8_t REG_FIFO_CTRL1       = 0x06; // FIFO watermark LSB
static constexpr uint8_t REG_FIFO_CTRL2       = 0x07; // FIFO watermark MSB
static constexpr uint8_t REG_FIFO_CTRL3       = 0x08; // accel/gyro decimation
static constexpr uint8_t REG_FIFO_CTRL5       = 0x0A; // FIFO mode + ODR
static constexpr uint8_t REG_FIFO_STATUS1     = 0x3A; // FIFO sample count LSB
static constexpr uint8_t REG_FIFO_STATUS2     = 0x3B; // FIFO sample count MSB + flags
static constexpr uint8_t REG_FIFO_DATA_OUT_L  = 0x3E; // FIFO output register
static constexpr uint8_t REG_TAP_CFG          = 0x58; // interrupt enable/latch control

// FIFO mode values (FIFO_CTRL5 bits [2:0])
static constexpr uint8_t FIFO_MODE_BYPASS      = 0x00; // disabled / reset
static constexpr uint8_t FIFO_MODE_FIFO        = 0x01; // stop-on-full
static constexpr uint8_t FIFO_MODE_CONTINUOUS  = 0x06; // overwrite oldest

// FIFO ODR = 1.66 kHz → bits [6:3] = 0b1000
static constexpr uint8_t FIFO_ODR_1660HZ      = (0x08 << 3);

// Accel: 1.66 kHz ODR, ±16 g  → CTRL1_XL = 0x84
static constexpr uint8_t CTRL1_XL_VAL         = 0x84;
// Gyro:  1.66 kHz ODR, ±2000 dps → CTRL2_G  = 0x8C
static constexpr uint8_t CTRL2_G_VAL          = 0x8C;

// Sensitivity constants (from datasheet, at chosen full-scale ranges)
static constexpr float ACCEL_SENS = 0.488f;   // mg/LSB at ±16 g
static constexpr float GYRO_SENS  = 70.0f;    // mdps/LSB at ±2000 dps

static constexpr bool verbose = true;

// ── ISR shared state ──────────────────────────────────────────────────────────
static volatile bool     _isrFired   = false;
static volatile uint32_t _isrTime_us = 0;

static void imuISR()
{
    _isrFired   = true;
    _isrTime_us = micros();
}

// ── Adafruit objects (used only for begin_I2C detection) ──────────────────────
static Adafruit_LSM6DS33   _IMU33;
static Adafruit_LSM6DS3TRC _IMU3TRC;
static const char         *_IMUname = "none";

static bool startIMU()
{
    if (_IMU3TRC.begin_I2C())
    {
        _IMUname = "LSM6DS3TR-C";
        return true;
    }
    if (_IMU33.begin_I2C())
    {
        _IMUname = "LSM6DS33";
        return true;
    }
    return false;
}

// ── Register helpers ──────────────────────────────────────────────────────────
void IMU::writeReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t IMU::readReg(uint8_t reg)
{
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)LSM_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// ── FIFO configuration ────────────────────────────────────────────────────────

// Reset FIFO by briefly switching to bypass mode.
void IMU::clearFIFO()
{
    writeReg(REG_FIFO_CTRL5, FIFO_MODE_BYPASS);
    delayMicroseconds(100);
}

// Store both accel and gyro into FIFO at no decimation (every sample).
// ODR matches the sensor ODR so we get every reading.
void IMU::configureFIFO_Continuous()
{
    clearFIFO();
    writeReg(REG_FIFO_CTRL3, 0x09);  // gyro dec=1, accel dec=1 (no skip)
    writeReg(REG_FIFO_CTRL5, FIFO_ODR_1660HZ | FIFO_MODE_CONTINUOUS);
}

// Switch to stop-on-full — keep filling until 511 words, then halt.
void IMU::configureFIFO_StopOnFull()
{
    // Do NOT reset here — we want to keep the samples already in the FIFO.
    // Just change the mode bits in FIFO_CTRL5, leaving ODR unchanged.
    writeReg(REG_FIFO_CTRL5, FIFO_ODR_1660HZ | FIFO_MODE_FIFO);
}

uint16_t IMU::getFIFOCount()
{
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(REG_FIFO_STATUS1);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)LSM_ADDR, (uint8_t)2);
    uint8_t lsb = Wire.available() ? Wire.read() : 0;
    uint8_t msb = Wire.available() ? Wire.read() : 0;
    return ((uint16_t)(msb & 0x07) << 8) | lsb;  // 11-bit count, MSB is bits [2:0] of STATUS2
}

// ── INT1 configuration ────────────────────────────────────────────────────────

// Route wake-up (inertial) interrupt to INT1.
// WAKE_UP_THS: 6-bit threshold in units of 31.25 mg (at ±16 g FS).
// WAKE_UP_DUR: require the threshold to be exceeded for at least 1 ODR cycle.
void IMU::configureINT1_Inertial()
{
    disableINT1();

    // Enable embedded functions (required for wake-up interrupt)
    writeReg(REG_CTRL10_C, readReg(REG_CTRL10_C) | 0x04);

    writeReg(REG_WAKE_UP_THS, WAKE_THRESHOLD_LSB & 0x3F);
    writeReg(REG_WAKE_UP_DUR, 0x01);   // 1 ODR cycle duration filter
    writeReg(REG_MD1_CFG, 0x20);        // INT1_WU bit → wake-up → INT1

    // TAP_CFG (0x58): INTERRUPTS_ENABLE (bit 7) must be set to actually
    // latch and route embedded function interrupts to the INT pins.
    writeReg(REG_TAP_CFG, readReg(REG_TAP_CFG) | 0x80);
}

// Route FIFO-full flag to INT1.
void IMU::configureINT1_FIFOFull()
{
    disableINT1();
    writeReg(REG_INT1_CTRL, 0x20);  // INT1_FULL_FLAG bit
}

void IMU::disableINT1()
{
    writeReg(REG_INT1_CTRL, 0x00);
    writeReg(REG_MD1_CFG,   0x00);
}

// ── Drain FIFO into _buf ──────────────────────────────────────────────────────
// The FIFO stores 16-bit words.  With gyro + accel both at decimation=1, the
// pattern is: [Gx][Gy][Gz][Ax][Ay][Az] — 6 words (12 bytes) per sample.
// We read all words and reconstruct samples, timestamping each one by
// interpolating backwards from _triggerTime_us at 1.66 kHz spacing.
void IMU::drainFIFO()
{
    _bufCount = 0;
    const float accelScale = ACCEL_SENS * 0.001f * 9.80665f;
    const float gyroScale  = GYRO_SENS  * 0.001f * (3.14159265f / 180.0f);

    while (true)
    {
        // Check FIFO empty flag (STATUS2 bit 4)
        uint8_t status = readReg(REG_FIFO_STATUS2);
        if (status & 0x10)
            break;  // FIFO empty

        if (_bufCount >= BUF_CAPACITY)
            break;  // safety limit

        // Read one sample (6 words = 12 bytes)
        int16_t raw[6];
        for (uint8_t w = 0; w < 6; w++)
        {
            Wire.beginTransmission(LSM_ADDR);
            Wire.write(REG_FIFO_DATA_OUT_L);
            Wire.endTransmission(false);
            Wire.requestFrom((uint8_t)LSM_ADDR, (uint8_t)2);
            uint8_t lo = Wire.available() ? Wire.read() : 0;
            uint8_t hi = Wire.available() ? Wire.read() : 0;
            raw[w] = (int16_t)((uint16_t)hi << 8 | lo);
        }

        IMUSample &s = _buf[_bufCount++];
        s.gx = raw[0] * gyroScale;
        s.gy = raw[1] * gyroScale;
        s.gz = raw[2] * gyroScale;
        s.ax = raw[3] * accelScale;
        s.ay = raw[4] * accelScale;
        s.az = raw[5] * accelScale;
    }

    // Assign timestamps now we know total sample count
    static constexpr uint32_t SAMPLE_INTERVAL_US = 603;
    for (uint16_t i = 0; i < _bufCount; i++)
    {
        int32_t offsetFromTrigger =
            (int32_t)(i - (_bufCount - 1)) * (int32_t)SAMPLE_INTERVAL_US;
        _buf[i].timestamp_us = (uint32_t)((int64_t)_triggerTime_us + offsetFromTrigger);
    }
}

// ── Print captured data as CSV ────────────────────────────────────────────────
void IMU::printCapture()
{
    Serial.println(F("\n--- Recoil Event Captured ---"));
    Serial.print(F("Samples: "));
    Serial.println(_bufCount);
    Serial.println(F("n,t_us,ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads"));

    for (uint16_t i = 0; i < _bufCount; i++)
    {
        const IMUSample &s = _buf[i];

        // Zero-reference: positive = after event, negative = before event
        int32_t t_rel = (int32_t)(s.timestamp_us - _triggerTime_us);

        Serial.print(i);          Serial.print(',');
        Serial.print(t_rel);      Serial.print(',');
        Serial.print(s.ax, 4);    Serial.print(',');
        Serial.print(s.ay, 4);    Serial.print(',');
        Serial.print(s.az, 4);    Serial.print(',');
        Serial.print(s.gx, 4);    Serial.print(',');
        Serial.print(s.gy, 4);    Serial.print(',');
        Serial.println(s.gz, 4);
    }

    Serial.println(F("--- End of capture ---\n"));
}

// ── Public: setup ─────────────────────────────────────────────────────────────
bool IMU::setup()
{
    Wire.begin();
    isUP = startIMU();
    if (!isUP)
    {
        Serial.println(F("IMU Startup Failed."));
        return false;
    }

    if (verbose)
    {
        Serial.print(F("Started IMU: "));
        Serial.println(_IMUname);
    }

    // CTRL3_C: enable auto-increment (bit 2), BDU (bit 6)
    writeReg(REG_CTRL3_C, readReg(REG_CTRL3_C) | 0x44);

    // Set ODR and ranges directly (Adafruit init may have set these already,
    // but we re-assert to be sure they match our sensitivity constants above)
    writeReg(REG_CTRL1_XL, CTRL1_XL_VAL);  // accel: 1.66 kHz, ±16 g
    writeReg(REG_CTRL2_G,  CTRL2_G_VAL);   // gyro:  1.66 kHz, ±2000 dps

    // Set up INT1 pin on the nRF52840 side
    pinMode(IMU_INT1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imuISR, RISING);

    _state = State::ARMING;

    Serial.println(F("Recoil recorder ready."));
    Serial.print(F("  Wake threshold : "));
    Serial.print(WAKE_THRESHOLD_LSB * 31.25f, 0);
    Serial.println(F(" mg"));
    Serial.print(F("  FIFO depth     : 511 samples @ 1.66 kHz = ~"));
    Serial.print(511 * 1000 / 1660);
    Serial.println(F(" ms total window"));

    return true;
}

// ── Public: loop ──────────────────────────────────────────────────────────────
void IMU::loop()
{
    if (!isUP)
        return;

    switch (_state)
    {
case State::ARMING:
{
    _bufCount = 0;
    _isrFired = false;

    disableINT1();
    configureFIFO_Continuous();
    configureINT1_Inertial();
    readReg(REG_WAKE_UP_SRC);

    // ── Temporary diagnostics — remove once working ───────────────────
    delay(500); // wait half a second for FIFO to accumulate some samples
    static volatile uint8_t  dbg_fifoCtrl3  = readReg(REG_FIFO_CTRL3);
    static volatile uint8_t  dbg_fifoCtrl5  = readReg(REG_FIFO_CTRL5);
    static volatile uint8_t  dbg_fifoStatus1 = readReg(REG_FIFO_STATUS1);
    static volatile uint8_t  dbg_fifoStatus2 = readReg(REG_FIFO_STATUS2);
    static volatile uint16_t dbg_fifoCount  = getFIFOCount();
    (void)dbg_fifoCtrl3;
    (void)dbg_fifoCtrl5;
    (void)dbg_fifoStatus2;
    (void)dbg_fifoCount; // <-- breakpoint here
    // ─────────────────────────────────────────────────────────────────

    Serial.println(F("--- Armed. Waiting for recoil event... ---"));
    _state = State::ARMED;
    break;
}

// ── ARMED: wait for inertial interrupt ────────────────────────────────
case State::ARMED:
{
    if (!_isrFired)
        return;

    // Capture trigger time before clearing the flag
    _triggerTime_us = _isrTime_us;
    _isrFired       = false;

    if (verbose)
    {
        Serial.print(F("Trigger! t="));
        Serial.println(_triggerTime_us);
    }

    // Freeze the FIFO — stop overwriting pre-event data.
    // Switch INT1 to fire when FIFO is full (post-event fill complete).
    disableINT1();
    configureFIFO_StopOnFull();
    configureINT1_FIFOFull();

    // ── Temporary diagnostics — remove once working ───────────────────
    static volatile uint8_t  dbg_fifoCtrl3 = readReg(REG_FIFO_CTRL3);
    static volatile uint8_t  dbg_fifoCtrl5 = readReg(REG_FIFO_CTRL5);
    static volatile uint8_t  dbg_int1Ctrl  = readReg(REG_INT1_CTRL);
    static volatile uint16_t dbg_fifoCount = getFIFOCount();
    (void)dbg_fifoCtrl3;
    (void)dbg_fifoCtrl5;
    (void)dbg_int1Ctrl;
    (void)dbg_fifoCount; // <-- breakpoint here
    // ─────────────────────────────────────────────────────────────────

    // Clear the latch so the next ISR fire means FIFO-full, not wake-up
    _isrFired = false;

    _state = State::CAPTURING;
    break;
}

    // ── CAPTURING: wait for FIFO-full interrupt ───────────────────────────
case State::CAPTURING:
{
    static volatile uint8_t dbg_captureStatus = readReg(REG_FIFO_STATUS2);
    (void)dbg_captureStatus; // <-- breakpoint here, watch value climb toward 96

    if (!(dbg_captureStatus & 0x40))  // wait for FIFO full/overrun flag
        return;

    static volatile bool dbg_fifoFullSeen = true; // <-- second breakpoint here
    (void)dbg_fifoFullSeen;

    if (verbose)
        Serial.println(F("FIFO full. Draining..."));

    disableINT1();
    _state = State::DRAINING;
    break;
}

    // ── DRAINING: read FIFO, print, then re-arm ───────────────────────────
    case State::DRAINING:
    {
        drainFIFO();
        printCapture();

        // Brief pause before re-arming so Serial output can flush and the
        // shooter isn't immediately re-triggered by vibration settling.
        delay(REARM_DELAY_MS);

        _state = State::ARMING;
        break;
    }
    }
}

const char *IMU::getName() const
{
    return _IMUname;
}