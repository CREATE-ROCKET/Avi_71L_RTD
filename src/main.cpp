#include <Arduino.h>
#include "LPS25HB.h"
#include "SPICREATE.h"
#include "S25FL512S.h"
#include "SPI_devices/ICM20602.h"
#include "NEC920.hpp"
#include "../RFparam/RTD_PARAM.h"

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

  uint8_t id[4] = {rtdRFparam::DST_1, rtdRFparam::DST_2, rtdRFparam::DST_3, rtdRFparam::DST_4};
  nec920.setDstID(id);

  pinMode(23, INPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);

  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);

  pinMode(19, OUTPUT);
  pinMode(18, OUTPUT);

  Serial1.begin(115200, SERIAL_8N1, 18, 19);
  nec920.setSerial(&Serial1);

  nec920.setRfConf(rtdRFparam::POWER, rtdRFparam::CHANNEL, rtdRFparam::RF_BAND, rtdRFparam::CS_MODE);
}

uint8_t i = 0;

void loop()
{
  int16_t imuData[6];
  uint8_t imuRaw[14];
  imu.Get(imuData, imuRaw);
  ESP_LOGV(TAG, "IMU:%d, %d, %d, %d, %d, %d", imuData[0], imuData[1], imuData[2], imuData[3], imuData[4], imuData[5]);

  uint8_t rxbff[254];
  uint8_t rxlength = 0;
  nec920.recieve();
  rxlength = nec920.getRecievedData(rxbff);
  if (rxlength > 0)
  {
    ESP_LOGV(TAG, "data recieved");
  }

  if (nec920.isWirelessModuleDead())
  {
    ESP_LOGV(TAG, "nec920 stopped");
  }
  else
  {
    ESP_LOGV(TAG, "nec920 ok");
  }

  uint8_t parameter[2];
  uint8_t parameterlength = 2;
  parameter[0] = 255 - i;
  parameter[1] = i++;

  nec920.sendData(0x13, i, parameter, parameterlength);
  ESP_LOGV(TAG, "send:%02X", i);

  delay(500);
}