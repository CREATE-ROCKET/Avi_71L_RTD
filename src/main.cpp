#include <Arduino.h>
#include "LPS25HB.h"
#include "SPICREATE.h"
#include "S25FL512S.h"
#include "SPI_devices/ICM20602.h"
#include "NEC920.hpp"
#include "../RFparam/RTD_PARAM.h"
#include "../../../Avi_71L_valveSystem/communication/gseCom.hpp"

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

namespace PWR_PINOUT
{
  HardwareSerial SER_PWR = Serial;
  constexpr uint8_t SER_PWR_TX = 4;
  constexpr uint8_t SER_PWR_RX = 5;
}

namespace VALVE_PINOUT
{
  HardwareSerial SER_VALVE = Serial2;
  constexpr uint8_t SER_VALVE_TX = 16;
  constexpr uint8_t SER_VALVE_RX = 17;
}

/**受信バッファ*/
class rxBff
{
public:
  uint8_t index = 0;
  uint8_t data[128];
};
rxBff ValveRxBff;

/** 受信用関数，パケット受信完了したらtrueを返す*/
IRAM_ATTR bool
recieve(HardwareSerial &SER, rxBff &rx)
{
  while (SER.available())
  {
    uint8_t tmp = SER.read();

    if (rx.index == 0) /**ヘッダ受信*/
    {
      if (tmp == 0x43)
      {
        rx.data[rx.index++] = tmp;
      }
    }
    else if ((rx.index == 1) || (rx.index == 2)) /**cmdidおよびlengthの受信*/
    {
      rx.data[rx.index++] = tmp;
    }
    else if (rx.index < (rx.data[2] - 1)) /**受信完了1個前までの処理*/
    {
      rx.data[rx.index++] = tmp;
    }
    else if (rx.index == rx.data[2] - 1) /**受信完了*/
    {
      rx.data[rx.index] = tmp;
      rx.index = 0;
      if (GseCom::checkPacket(rx.data) == 0)
      {
        return true;
      }
    }
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  VALVE_PINOUT::SER_VALVE.begin(115200, SERIAL_8N1, VALVE_PINOUT::SER_VALVE_RX, VALVE_PINOUT::SER_VALVE_TX);

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

void loop()
{
  // バルブ基板からの受信に対するコマンド処理系
  if (recieve(VALVE_PINOUT::SER_VALVE, ValveRxBff))
  {
    uint8_t tmpCmdId = GseCom::getCmdId(ValveRxBff.data);

    if (tmpCmdId == 0x61)
    {
      uint8_t rxPayload[1];
      uint8_t rxPayloadLength;
      GseCom::getPayload(ValveRxBff.data, rxPayload, &rxPayloadLength);
      uint8_t txPacket[5];
      uint8_t txPayload = 0x00;
      if (rxPayload[0] == 0x00)
      {
        // flight mode
        txPayload = 0x40;
      }
      else if (rxPayload[0] == 0x01)
      {
        // sleep mode
        txPayload = 0x41;
      }
      else if (rxPayload[0] == 0x05)
      {
        // delete mode
        txPayload = 0x45;
      }
      GseCom::makePacket(txPacket, 0x61, &txPayload, 1);
      VALVE_PINOUT::SER_VALVE.write(txPacket, 5);

      if (rxPayload[0] == 0x05)
      {
        // delete mode delete start
        flash.erase();
        txPayload = 0x46;
        GseCom::makePacket(txPacket, 0x61, &txPayload, 1);
        VALVE_PINOUT::SER_VALVE.write(txPacket, 5);
      }
    }
  }

  // if (i == 0)
  // {
  //   if (nec920.isBootFinished(400000) == 0)
  //   {
  //     ESP_LOGV(TAG, "booting...");
  //   }
  //   else if (booted == 0)
  //   {
  //     booted = 1;
  //     i++;
  //     ESP_LOGV(TAG, "boot finished");
  //     ESP_LOGV(TAG, "NEC920 can send msg");
  //     nec920.setRfConf(j, rtdRFparam::POWER, rtdRFparam::CHANNEL, rtdRFparam::RF_BAND, rtdRFparam::CS_MODE);
  //     ESP_LOGV(TAG, "NEC920 set RF conf start");
  //   }
  // }

  // if (i == 1)
  // {
  //   if (nec920.recieve())
  //   {
  //     ESP_LOGV(TAG, "NEC920 recieve success");
  //     if (nec920.isRecieveCmdResult())
  //     {
  //       if (nec920.checkCmdResult(j++))
  //       {
  //         ESP_LOGV(TAG, "NEC920 set RF conf failed");
  //         i++;
  //         delay(1000);
  //       }
  //       else
  //       {
  //         ESP_LOGV(TAG, "NEC920 set RF conf success");
  //         i++;
  //         delay(1000);
  //       }
  //     }
  //     nec920.dataUseEnd();
  //   }
  //   else
  //   {

  //     ESP_LOGV(TAG, "NEC920 recieve failed");
  //     if (nec920.isModuleDeadByTimeout(1000000))
  //     {
  //       ESP_LOGV(TAG, "NEC920 module dead");
  //       i = 11;
  //     }
  //   }
  // }

  // if (i == 2)
  // {
  //   if (nec920.canSendMsgCheck())
  //   {
  //     ESP_LOGV(TAG, "NEC920 can send msg");
  //     uint8_t dstID[4] = {rtdRFparam::DST_1, rtdRFparam::DST_2, rtdRFparam::DST_3, rtdRFparam::DST_4};
  //     uint8_t parameter[1] = {0x71};
  //     nec920.sendTxCmd(0x13, j, dstID, parameter, 1);
  //     i++;
  //   }
  //   else
  //   {
  //     ESP_LOGV(TAG, "NEC920 can't send msg");
  //     if (nec920.isModuleDeadByTimeout(1000000))
  //     {
  //       ESP_LOGV(TAG, "NEC920 module dead");
  //       i = 11;
  //     }
  //   }
  // }

  // if (i == 3)
  // {
  //   if (nec920.recieve())
  //   {
  //     ESP_LOGV(TAG, "NEC920 recieve success");

  //     if (nec920.isRecieveCmdData())
  //     {
  //       ESP_LOGV(TAG, "NEC920 recieve data");
  //       uint8_t tmpArr[256];
  //       uint8_t datalength = nec920.getRecieveData(tmpArr);
  //       for (int i = 0; i < datalength; i++)
  //       {
  //         Serial.printf("%02X,", tmpArr[i]);
  //       }
  //       Serial.println();
  //     }

  //     if (nec920.isRecieveCmdResult())
  //     {
  //       if (nec920.checkCmdResult(j++))
  //       {
  //         ESP_LOGV(TAG, "NEC920 send recieve failed");
  //         i = 2;
  //         delay(1000);
  //       }
  //       else
  //       {
  //         ESP_LOGV(TAG, "NEC920 send recieve success");
  //         i = 2;
  //         delay(1000);
  //       }
  //     }

  //     nec920.dataUseEnd();
  //   }
  //   else
  //   {
  //     ESP_LOGV(TAG, "NEC920 recieve failed");
  //     if (nec920.isModuleDeadByTimeout(1000000))
  //     {
  //       ESP_LOGV(TAG, "NEC920 module dead");
  //       i = 11;
  //     }
  //   }
  // }

  // if (i == 11)
  // {
  //   ESP_LOGV(TAG, "reboot start");
  //   nec920.startReboot();
  //   i++;
  //   booted = 0;
  // }

  // if (i == 12)
  // {

  //   if (nec920.doReboot(100000))
  //   {
  //     ESP_LOGV(TAG, "rebooting...");
  //   }
  //   else
  //   {
  //     ESP_LOGV(TAG, "reboot finished");
  //     i = 0;
  //   }
  // }

  // delay(10);
}