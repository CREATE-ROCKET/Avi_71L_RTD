#include <Arduino.h>
#include "SPICREATE.h"
#include "SPIFlash.h"

constexpr char *TAG = "INIT";

SPICREATE::SPICreate spibus;
Flash flash;

namespace RTD_SPI_CONF
{
  constexpr uint8_t SCK = 27;
  constexpr uint8_t MISO = 33;
  constexpr uint8_t MOSI = 26;
  constexpr uint8_t CS_PRESSURE = 13;
  constexpr uint8_t CS_FLASH = 32;
  constexpr uint8_t CS_IMU = 25;
  constexpr uint32_t SPIFREQ = 4000000;
};

// データ解析関数（ビッグエンディアン）
void parseAndOutputData(const uint8_t *data, size_t size)
{
  // ヘッダの確認
  if (data[0] != 0x40 && data[0] != 0x41)
  {
    Serial.println("Invalid header");
    return;
  }

  size_t index = 1; // 配列のインデックス

  // 6軸データの解析
  for (int i = 0; i < 10; ++i)
  {
    if (index + 14 > size)
      break; // データの終わりを超えないようにチェック

    // 時刻データ
    uint32_t time = (uint32_t)data[index + 3] << 24 | (uint32_t)data[index + 2] << 16 |
                    (uint32_t)data[index + 1] << 8 | (uint32_t)data[index];
    index += 4;

    // 6軸データ
    int16_t ax = (int16_t)(data[index + 1] << 8 | data[index]);
    int16_t ay = (int16_t)(data[index + 3] << 8 | data[index + 2]);
    int16_t az = (int16_t)(data[index + 5] << 8 | data[index + 4]);
    int16_t gx = (int16_t)(data[index + 7] << 8 | data[index + 6]);
    int16_t gy = (int16_t)(data[index + 9] << 8 | data[index + 8]);
    int16_t gz = (int16_t)(data[index + 11] << 8 | data[index + 10]);
    index += 12;

    // CSV出力
    Serial.printf("imu,%lu, %d, %d, %d, %d, %d, %d, , \r\n", time, ax, ay, az, gx, gy, gz);
  }

  // 気圧データの解析
  if (data[0] == 0x41)
  {
    if (index + 7 <= size)
    {
      uint32_t pressureTime = (uint32_t)data[index + 3] << 24 | (uint32_t)data[index + 2] << 16 |
                              (uint32_t)data[index + 1] << 8 | (uint32_t)data[index];
      index += 4;

      uint32_t pressure = (uint32_t)data[index + 2] << 16 | (uint32_t)data[index + 1] << 8 | (uint32_t)data[index];
      index += 3;

      // 解放率
      int16_t releaseRate = data[index + 1] << 8 | data[index];
      index += 2;

      // CSV出力
      Serial.printf("lps,%lu, , , , , , , %lu, %d\r\n", pressureTime, pressure, releaseRate);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  spibus.begin(VSPI, RTD_SPI_CONF::SCK, RTD_SPI_CONF::MISO, RTD_SPI_CONF::MOSI, RTD_SPI_CONF::CS_IMU);
  flash.begin(&spibus, RTD_SPI_CONF::CS_FLASH, RTD_SPI_CONF::SPIFREQ);
}

void loop()
{
  if (Serial.available())
  {
    uint8_t data = Serial.read();
    if (data = 'a')
    {
      // read all data
      for (uint32_t i = 0; i < 0x10000; i++)
      {
        uint8_t rxbff[256];
        flash.read(i << 8, rxbff);
        parseAndOutputData(rxbff, 256);
      }
    }
    if (data = 'r')
    {
      for (uint32_t i = 0; i < 0x10000; i++)
      {
        uint8_t rxbff[256];
        flash.read(i << 8, rxbff);
        for (int j = 0; j < 256; j++)
        {
          Serial.printf("%02x", rxbff[j]);
        }
        Serial.println();
      }
    }
  }
}