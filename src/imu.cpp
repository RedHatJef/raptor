#include "imu.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LSM6DS.h>

static Adafruit_LSM6DS33 _IMU33;
static Adafruit_LSM6DS3TRC _IMU3TRC;
static Adafruit_LSM6DS *_IMU = nullptr;
static const char *_IMUname = "none";
static constexpr bool verbose = true;

static bool startIMU()
{
    if (_IMU3TRC.begin_I2C())
    {
        _IMU = &_IMU3TRC;
        _IMUname = "LSM6DS3TR-C";
        return true;
    }
    if (_IMU33.begin_I2C())
    {
        _IMU = &_IMU33;
        _IMUname = "LSM6DS33";
        return true;
    }
    return false;
}

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
        Serial.print(_IMUname);
    }

    _IMU->setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    _IMU->setAccelRange(LSM6DS_ACCEL_RANGE_16_G);

    _IMU->setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
    _IMU->setAccelDataRate(LSM6DS_RATE_1_66K_HZ);

    return true;
}

void IMU::loop()
{
    if (!isUP)
        return;
}

const char *IMU::getName() const
{
    return _IMUname;
}
