#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include "imu.h"
#include "neopixel.h"
#include "wire_util.h"

static IMU      imu;
static NeoPixel neo;

void setup() {
  Serial.begin(115200);
  delay(50);

  WireUtil::recoverBus();

  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(50);
  delay(10);

  // uint8_t devices[16];
  // WireUtil::scan(Wire, devices, sizeof(devices)/sizeof(devices[0]));

  neo.setup();
  imu.setup();
}

void loop() {
  imu.loop();

  if (imu.takeImpactFlag())
    neo.trigger();

  if (imu.takeDrainDoneFlag())
    neo.captureComplete();

  neo.loop();
}