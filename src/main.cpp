#include <Arduino.h>
#include "LPS25HB.h"
#include "SPICREATE.h"
#include "S25FL512S.h"
#include "SPI_devices/ICM20602.h"
#include "NEC920.hpp"
#include "../RFparam/RTD_PARAM.h"
#include "../../../Avi_71L_valveSystem/communication/gseCom.hpp"
#include "PMB/PMBCom.h"
#include "PMB/SLIP.h"
#include "Quarternion.h"

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
  constexpr uint32_t SPIFREQ = 4000000;
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

PMBCOM pmb;
SLIP slip;

Quarternion q4;

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

namespace LOGGING
{
  uint32_t latestFlashPage = 0;
  uint8_t isFlashErased = 0;  // 1:Erased 0:not Erased
  uint8_t isLoggingGoing = 0; // 1:Going 0:not Going

  uint8_t wirelessDatasetWaitingSend[170];
  uint8_t isDatainWirelessDatasetWaitingSend = 0;
  xTaskHandle sendWirelessmoduleHandle;

  IRAM_ATTR void sendWirelessmodule(void *parameters)
  {
    for (;;)
    {
      portTickType xLastWakeTime = xTaskGetTickCount();
      if (isDatainWirelessDatasetWaitingSend)
      {
        uint8_t dstID[4] = {rtdRFparam::DST_1, rtdRFparam::DST_2, rtdRFparam::DST_3, rtdRFparam::DST_4};
        nec920.sendTxCmd(0x13, 0x71, dstID, wirelessDatasetWaitingSend, 174);
        isDatainWirelessDatasetWaitingSend = 0;
      }

      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_PERIOD_MS);
    }
  }

  uint32_t wirelessDatasetIndex = 0;
  uint8_t wirelessDataset[174];
  uint8_t flashDatasetIndex = 0;
  uint8_t flashDataset[256];
  TaskHandle_t LoggingHandle;

  int16_t openRate = -300;

  IRAM_ATTR void logging(void *parameters)
  {
    uint8_t loggingIndex = 0;
    uint32_t lpsDataGetTime = 0;
    uint8_t lpsData[3];
    portTickType xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
      uint32_t startTime = micros();
      loggingIndex++;
      // -------------------------------SPIセンサ通信開始--------------------------------
      int16_t imuData[6];
      uint32_t imuDataGetTime = micros();
      imu.Get(imuData);
      q4.Calc(imuData);

      // loggingIndex == 38の時，lpsの値を取得
      if (loggingIndex == 38)
      {
        lpsDataGetTime = micros();
        pressureSensor.Get(lpsData);
      }
      // -------------------------------SPIセンサ通信終了--------------------------------

      // -------------------------------UART送信開始--------------------------------
      if (loggingIndex == 1)
      {
        // loggingIndex == 0の時，バルブ基板に開栓率調査要求
        uint8_t payload = 0xFF;
        uint8_t txPacket[5];
        GseCom::makePacket(txPacket, 0xF0, &payload, 1);
        VALVE_PINOUT::SER_VALVE.write(txPacket, 5);
        // loggingIndex == 0の時，PMBに電圧調査要求はやめた
      }
      // -------------------------------UART送信終了--------------------------------

      // -------------------------------flashのデータ作成開始--------------------------------
      // flashのデータセットのヘッダ初期化 // loggingIndex == 1,11,21,31 で実行
      if (loggingIndex % 10 == 1)
      {
        flashDatasetIndex = 0;
        flashDataset[flashDatasetIndex++] = 0x40; // header
      }
      // flashのデータセットにimuの生データとタイムスタンプを追加
      for (int i = 0; i < 4; i++)
      {
        flashDataset[flashDatasetIndex++] = imuDataGetTime >> (8 * i);
      }
      for (int i = 0; i < 6; i++)
      {
        flashDataset[flashDatasetIndex++] = imuData[i] & 0xFF;
        flashDataset[flashDatasetIndex++] = imuData[i] >> 8;
      }
      // loggingIndex == 40の時，lpsの値を追加，開栓率，電圧を追加
      if (loggingIndex == 40)
      {
        for (int i = 0; i < 4; i++)
        {
          flashDataset[flashDatasetIndex++] = lpsDataGetTime >> (8 * i);
        }
        for (int i = 0; i < 3; i++)
        {
          flashDataset[flashDatasetIndex++] = lpsData[i];
        }
        flashDataset[flashDatasetIndex++] = openRate & 0xFF;
        flashDataset[flashDatasetIndex++] = openRate >> 8;
        // for (int i = 0; i < 3; i++)
        // {
        //   for (int j = 0; j < 4; j++)
        //   {
        //     flashDataset[flashDatasetIndex++] = voltage[i] >> (8 * j);
        //   }
        // }
      }
      // loggingIndex == 10,20,30,40 で空きを0xEE埋めし，flashのデータセットをflashに書き込み
      if (loggingIndex % 10 == 0)
      {
        for (int i = flashDatasetIndex; i < 256; i++)
        {
          flashDataset[i] = 0xEE;
        }
        flash.write(latestFlashPage << 8, flashDataset);
        latestFlashPage++;
      }
      // -------------------------------flashのデータ作成終了--------------------------------

      // -------------------------------無線機用データ作成開始--------------------------------
      // 無線機用データセットのヘッダ初期化
      if (loggingIndex == 1)
      {
        wirelessDatasetIndex = 0;
        wirelessDataset[wirelessDatasetIndex++] = 0x40; // header
      }
      if ((loggingIndex % 5) == 1) // loggingIndex == 1,6,11,16,21,26,31,36 で実行
      {
        // 無線機用データセットにタイムスタンプの下位とクオータニオンを追加
        for (int i = 0; i < 4; i++)
        {
          wirelessDataset[wirelessDatasetIndex++] = imuDataGetTime >> (8 * i);
        }
        for (int i = 0; i < 4; i++)
        {
          memcpy(&wirelessDataset[wirelessDatasetIndex], &q4.q[i], sizeof(float));
          wirelessDatasetIndex += 4;
        }
      }

      // 無線機用データセットにlps，開栓率，電圧を追加，送信，ヘッダを変更
      if (loggingIndex == 39)
      {
        wirelessDataset[0] = 0x41; // header
        for (int i = 0; i < 4; i++)
        {
          wirelessDataset[wirelessDatasetIndex++] = lpsDataGetTime >> (8 * i);
        }
        for (int i = 0; i < 3; i++)
        {
          wirelessDataset[wirelessDatasetIndex++] = lpsData[i];
        }
        for (int i = 0; i < 2; i++)
        {
          wirelessDataset[wirelessDatasetIndex++] = openRate >> (8 * i);
        }
        // for (int i = 1; i < 3; i++)
        // {
        //   for (int j = 0; j < 2; j++)
        //   {
        //     wirelessDataset[wirelessDatasetIndex++] = voltage[i] >> (8 * j);
        //   }
        // }
        // -------------------------------------------------無線モジュールへ送信を行え-------------------------------------------------
        memcpy(wirelessDatasetWaitingSend, wirelessDataset, 170);
        isDatainWirelessDatasetWaitingSend = 1;
      }
      // -------------------------------無線機用データ作成終了--------------------------------
      // ESP_LOGV(TAG, "logging time:%d us", micros() - startTime);

      // if (loggingIndex % 10 == 0)
      // {
      //   Serial.printf("flashDataset\r\n");
      //   for (int i = 0; i < 256; i++)
      //   {
      //     Serial.printf("%02X,", flashDataset[i]);
      //   }
      //   Serial.println();
      // }

      // if (loggingIndex == 40)
      // {
      //   Serial.printf("wirelessDataset\r\n");
      //   for (int i = 0; i < 174; i++)
      //   {
      //     Serial.printf("%02X,", wirelessDataset[i]);
      //   }
      //   Serial.println();
      // }
      // Serial.flush();

      if (latestFlashPage >= 0x10000)
      {
        delay(100);
        LOGGING::isLoggingGoing = 0;
        vTaskDelete(LOGGING::sendWirelessmoduleHandle);
        vTaskDelete(LOGGING::LoggingHandle);
      }

      if (loggingIndex == 40)
      {
        loggingIndex = 0;
        // vTaskDelete(sendWirelessmoduleHandle);
        // vTaskDelete(LoggingHandle);
      }

      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_PERIOD_MS);
    }
  }
}

void setup()
{
  PWR_PINOUT::SER_PWR.begin(115200, SERIAL_8N1, PWR_PINOUT::SER_PWR_RX, PWR_PINOUT::SER_PWR_TX);
  VALVE_PINOUT::SER_VALVE.begin(115200, SERIAL_8N1, VALVE_PINOUT::SER_VALVE_RX, VALVE_PINOUT::SER_VALVE_TX);

  spibus.begin(VSPI, RTD_SPI_CONF::SCK, RTD_SPI_CONF::MISO, RTD_SPI_CONF::MOSI);
  // ESP_LOGV(TAG, "SPIBUS");
  pressureSensor.begin(&spibus, RTD_SPI_CONF::CS_PRESSURE, RTD_SPI_CONF::SPIFREQ);
  // ESP_LOGV(TAG, "pressure");
  flash.begin(&spibus, RTD_SPI_CONF::CS_FLASH, RTD_SPI_CONF::SPIFREQ);
  // ESP_LOGV(TAG, "flash");
  imu.begin(&spibus, RTD_SPI_CONF::CS_IMU, RTD_SPI_CONF::SPIFREQ);
  // ESP_LOGV(TAG, "IMU");

  // ESP_LOGV(TAG, "whoAmI p:%d, IMU:%d, Flash:%d", pressureSensor.WhoAmI(), imu.WhoAmI(), 128);

  uint8_t GNDid[4] = {rtdRFparam::DST_1, rtdRFparam::DST_2, rtdRFparam::DST_3, rtdRFparam::DST_4};

  // ESP_LOGV(TAG, "NEC920 init start");
  nec920.beginSerial(&Serial1, 115200, RTD_W_PINOUT::pin920Rx, RTD_W_PINOUT::pin920Tx);
  // ESP_LOGV(TAG, "NEC920 serial init");
  nec920.setPin(RTD_W_PINOUT::pin920Reset, RTD_W_PINOUT::pin920Wakeup, RTD_W_PINOUT::pin920Mode);
  // ESP_LOGV(TAG, "NEC920 pin init");

  delay(400);
  nec920.setRfConf(0x44, rtdRFparam::POWER, rtdRFparam::CHANNEL, rtdRFparam::RF_BAND, rtdRFparam::CS_MODE);

  slip.begin(&PWR_PINOUT::SER_PWR);
  pmb.begin(&slip);
}

uint32_t pmbRequestTime = 0;
uint8_t isRequesting = 0;

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
        if (LOGGING::isLoggingGoing == 0)
        {
          // quaternion初期化
          q4.Init(1, 0, 0, 0, 0.001);
          int32_t gyro_drift_raw[3] = {0, 0, 0};
          float gyro_drift[3] = {0, 0, 0};
          for (int i = 0; i < 1000; i++)
          {
            int16_t arr[6];
            imu.Get(arr);
            gyro_drift_raw[0] += arr[3];
            gyro_drift_raw[1] += arr[4];
            gyro_drift_raw[2] += arr[5];
            delay(1);
          }
          for (int i = 0; i < 3; i++)
          {
            gyro_drift[i] = (float)gyro_drift_raw[i] / 1000;
          }
          q4.Drift(gyro_drift);

          LOGGING::isLoggingGoing = 1;
          xTaskCreatePinnedToCore(LOGGING::sendWirelessmodule, "send", 4096, NULL, 1, &LOGGING::sendWirelessmoduleHandle, 0);
          xTaskCreatePinnedToCore(LOGGING::logging, "logging", 4096, NULL, 1, &LOGGING::LoggingHandle, 1);
        }
        txPayload = 0x40;
      }
      else if (rxPayload[0] == 0x01)
      {
        // sleep mode
        if (LOGGING::isLoggingGoing == 1)
        {
          delay(100);
          vTaskDelete(LOGGING::sendWirelessmoduleHandle);
          vTaskDelete(LOGGING::LoggingHandle);
          LOGGING::isLoggingGoing = 0;
        }
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
        LOGGING::latestFlashPage = 0;
        txPayload = 0x46;
        GseCom::makePacket(txPacket, 0x61, &txPayload, 1);
        VALVE_PINOUT::SER_VALVE.write(txPacket, 5);
      }
    }

    if (tmpCmdId == 0xF0)
    {
      LOGGING::openRate = ValveRxBff.data[4] + ValveRxBff.data[5] << 8;
    }

    if (tmpCmdId == 0xAA)
    {
      // read mode
      for (uint32_t i = 0; i < 0x10010; i++)
      {
        uint8_t data[256];
        flash.read(i << 8, data);
        VALVE_PINOUT::SER_VALVE.write(data, 256);
      }
    }
  }

  // if (Serial.available())
  // {
  //   uint8_t tmp = Serial.read();
  //   if (tmp == 'r')
  //   {
  //     for (int i = 0; i < 100; i++)
  //     {
  //       uint8_t tmpArr[256];
  //       flash.read(i << 8, tmpArr);
  //       for (int j = 0; j < 256; j++)
  //       {
  //         Serial.printf("%02X,", tmpArr[j]);
  //       }
  //       Serial.println();
  //     }
  //   }
  // }

  // PMBにリクエストを発行，1秒後にデータを取得，4秒間は何もしない

  if (!LOGGING::isLoggingGoing)
  {
    if (isRequesting == 0)
    {
      pmb.request(PMBCMD_ALL);
      pmbRequestTime = millis();
      isRequesting = 1;
    }
    if (isRequesting == 1)
    {
      if (millis() - pmbRequestTime > 1000)
      {
        isRequesting = 2;
        if (pmb.Get())
        {
          uint8_t payLoad[9];
          for (int i = 0; i < 3; i++)
          {
            for (int j = 0; j < 3; j++)
            {
              payLoad[i * 3 + j] = pmb.voltage[i] >> (8 * j);
            }
          }
          uint8_t Packet[13];
          GseCom::makePacket(Packet, 0x51, payLoad, 9);
          VALVE_PINOUT::SER_VALVE.write(Packet, 13);

          uint8_t wirelessPayload[10];
          wirelessPayload[0] = 0x51;
          for (int i = 0; i < 9; i++)
          {
            wirelessPayload[i + 1] = payLoad[i];
          }
          uint8_t dstID[4] = {rtdRFparam::DST_1, rtdRFparam::DST_2, rtdRFparam::DST_3, rtdRFparam::DST_4};
          nec920.sendTxCmd(0x13, 0x71, dstID, wirelessPayload, 10);
        }
        else
        {
          uint8_t dummy[1] = {0x51};
          uint8_t Packet[5];
          GseCom::makePacket(Packet, 0x51, dummy, 1);
          VALVE_PINOUT::SER_VALVE.write(Packet, 5);

          uint8_t dstID[4] = {rtdRFparam::DST_1, rtdRFparam::DST_2, rtdRFparam::DST_3, rtdRFparam::DST_4};
          nec920.sendTxCmd(0x13, 0x71, dstID, dummy, 1);
        }
      }
    }
    if (isRequesting == 2)
    {
      if (millis() - pmbRequestTime > 5000)
      {
        isRequesting = 0;
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