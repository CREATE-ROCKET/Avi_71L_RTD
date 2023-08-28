#include <Arduino.h>
#include "LPS25HB.h"
#include "SPICREATE.h"
#include "S25FL512S.h"
#include "ICM20948.h"
#include "NEC920.hpp"

#include "esp_log.h"

static const char *TAG = "INIT";

#define SCK 27
#define MISO 33
#define MOSI 26
#define CS_PRESSURE 13
#define CS_FLASH 32
#define CS_IMU 25
#define SPIFREQ 5000000

SPICREATE::SPICreate spibus;
LPS pressureSensor;
Flash flash;
ICM imu;

NEC920 nec920;

void setup()
{
  Serial.begin(115200);

  spibus.begin(VSPI, SCK, MISO, MOSI);
  ESP_LOGV(TAG, "SPIBUS");
  pressureSensor.begin(&spibus, CS_PRESSURE, SPIFREQ);
  ESP_LOGV(TAG, "pressure");
  flash.begin(&spibus, CS_FLASH, SPIFREQ);
  ESP_LOGV(TAG, "flash");
  imu.begin(&spibus, CS_IMU, SPIFREQ);
  ESP_LOGV(TAG, "IMU");

  ESP_LOGV(TAG, "whoAmI p:%d, IMU:%d, Flash:%d", pressureSensor.WhoAmI(), imu.WhoAmI(), 128);

  uint8_t id[4] = {0xC2, 0xA0, 0x03, 0xB1};
  nec920.setDstID(id);

  Serial1.begin(115200, SERIAL_8N1, 18, 19);
  nec920.setSerial(&Serial1);

  nec920.setRfConf(3, 37, 10, 5);
}

uint8_t i = 0;

void loop()
{
  uint8_t rxbff[254];
  uint8_t rxlength = 0;
  nec920.recieve();
  rxlength = nec920.getRecievedData(rxbff);
  if (rxlength > 0)
  {
    ESP_LOGV(TAG, "data recieved");
  }

  uint8_t parameter[240];
  uint8_t parameterlength = 240;
  parameter[0] = 255 - i;
  parameter[1] = i++;

  nec920.sendData(0x13, i, parameter, parameterlength);
  ESP_LOGV(TAG, "send:%02X", i);

  delay(1000);
}