#include <Arduino.h>
#include "LPS25HB.h"
#include "SPICREATE.h"
#include "S25FL512S.h"
#include "SPI_devices/ICM20602.h"
#include "NEC920.hpp"
#include "../RFparam/RTD_PARAM.h"

#include "esp_log.h"

constexpr char *TAG = "INIT";

SPICREATE::SPICreate spibus;
LPS pressureSensor;
Flash flash;
ICM imu;

NEC920 nec920;

namespace RTD_SPI_CONF
{
  constexpr uint8_t SCK = 27;
  constexpr uint8_t MISO = 33;
  constexpr uint8_t MOSI = 26;
  constexpr uint8_t CS_PRESSURE = 13;
  constexpr uint8_t CS_FLASH = 32;
  constexpr uint8_t CS_IMU = 25;
  constexpr uint32_t SPIFREQ = 5000000;
}

namespace RTD_W_PINOUT
{
  constexpr uint8_t LED = 14;
  constexpr uint8_t pin920Rx = 18;
  constexpr uint8_t pin920Tx = 19;
  constexpr uint8_t pin920Reset = 21;
  constexpr uint8_t pin920Wakeup = 22;
  constexpr uint8_t pin920Mode = 23;
}

void setup()
{
  Serial.begin(115200);

  spibus.begin(VSPI, RTD_SPI_CONF::SCK, RTD_SPI_CONF::MISO, RTD_SPI_CONF::MOSI);
  ESP_LOGV(TAG, "SPIBUS");
  pressureSensor.begin(&spibus, RTD_SPI_CONF::CS_PRESSURE, RTD_SPI_CONF::SPIFREQ);
  ESP_LOGV(TAG, "pressure");
  flash.begin(&spibus, RTD_SPI_CONF::CS_FLASH, RTD_SPI_CONF::SPIFREQ);
  ESP_LOGV(TAG, "flash");
  imu.begin(&spibus, RTD_SPI_CONF::CS_IMU, RTD_SPI_CONF::SPIFREQ);
  ESP_LOGV(TAG, "IMU");

  ESP_LOGV(TAG, "whoAmI p:%d, IMU:%d, Flash:%d", pressureSensor.WhoAmI(), imu.WhoAmI(), 128);

  uint8_t GNDid[4] = {rtdRFparam::DST_1, rtdRFparam::DST_2, rtdRFparam::DST_3, rtdRFparam::DST_4};

  ESP_LOGV(TAG, "NEC920 init start");
  nec920.beginSerial(&Serial1, 115200, RTD_W_PINOUT::pin920Rx, RTD_W_PINOUT::pin920Tx);
  ESP_LOGV(TAG, "NEC920 serial init");
  nec920.setPin(RTD_W_PINOUT::pin920Reset, RTD_W_PINOUT::pin920Wakeup, RTD_W_PINOUT::pin920Mode);
  ESP_LOGV(TAG, "NEC920 pin init");
}

bool booted = 0;
uint8_t i = 0;
uint8_t j = 0x71;

void loop()
{
  if (i == 0)
  {
    if (nec920.isBootFinished(400000) == 0)
    {
      ESP_LOGV(TAG, "booting...");
    }
    else if (booted == 0)
    {
      booted = 1;
      i++;
      ESP_LOGV(TAG, "boot finished");
      ESP_LOGV(TAG, "NEC920 can send msg");
      nec920.setRfConf(j, rtdRFparam::POWER, rtdRFparam::CHANNEL, rtdRFparam::RF_BAND, rtdRFparam::CS_MODE);
      ESP_LOGV(TAG, "NEC920 set RF conf start");
    }
  }

  if (i == 1)
  {
    if (nec920.recieve())
    {
      ESP_LOGV(TAG, "NEC920 recieve success");
      if (nec920.isRecieveCmdResult())
      {
        if (nec920.checkCmdResult(j++))
        {
          ESP_LOGV(TAG, "NEC920 set RF conf failed");
          i++;
          delay(1000);
        }
        else
        {
          ESP_LOGV(TAG, "NEC920 set RF conf success");
          i++;
          delay(1000);
        }
      }
      nec920.dataUseEnd();
    }
    else
    {

      ESP_LOGV(TAG, "NEC920 recieve failed");
      if (nec920.isModuleDeadByTimeout(1000000))
      {
        ESP_LOGV(TAG, "NEC920 module dead");
        i = 11;
      }
    }
  }

  if (i == 2)
  {
    if (nec920.canSendMsgCheck())
    {
      ESP_LOGV(TAG, "NEC920 can send msg");
      uint8_t dstID[4] = {rtdRFparam::DST_1, rtdRFparam::DST_2, rtdRFparam::DST_3, rtdRFparam::DST_4};
      uint8_t parameter[1] = {0x71};
      nec920.sendTxCmd(0x13, j, dstID, parameter, 1);
      i++;
    }
    else
    {
      ESP_LOGV(TAG, "NEC920 can't send msg");
      if (nec920.isModuleDeadByTimeout(1000000))
      {
        ESP_LOGV(TAG, "NEC920 module dead");
        i = 11;
      }
    }
  }

  if (i == 3)
  {
    if (nec920.recieve())
    {
      ESP_LOGV(TAG, "NEC920 recieve success");

      if (nec920.isRecieveCmdData())
      {
        ESP_LOGV(TAG, "NEC920 recieve data");
        uint8_t tmpArr[256];
        uint8_t datalength = nec920.getRecieveData(tmpArr);
        for (int i = 0; i < datalength; i++)
        {
          Serial.printf("%02X,", tmpArr[i]);
        }
        Serial.println();
      }

      if (nec920.isRecieveCmdResult())
      {
        if (nec920.checkCmdResult(j++))
        {
          ESP_LOGV(TAG, "NEC920 send recieve failed");
          i = 2;
          delay(1000);
        }
        else
        {
          ESP_LOGV(TAG, "NEC920 send recieve success");
          i = 2;
          delay(1000);
        }
      }

      nec920.dataUseEnd();
    }
    else
    {
      ESP_LOGV(TAG, "NEC920 recieve failed");
      if (nec920.isModuleDeadByTimeout(1000000))
      {
        ESP_LOGV(TAG, "NEC920 module dead");
        i = 11;
      }
    }
  }

  if (i == 11)
  {
    ESP_LOGV(TAG, "reboot start");
    nec920.startReboot();
    i++;
    booted = 0;
  }

  if (i == 12)
  {

    if (nec920.doReboot(100000))
    {
      ESP_LOGV(TAG, "rebooting...");
    }
    else
    {
      ESP_LOGV(TAG, "reboot finished");
      i = 0;
    }
  }

  delay(10);
}