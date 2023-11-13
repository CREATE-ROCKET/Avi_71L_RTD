#include <Arduino.h>
#include "SPICREATE.h"
#include "SPIflash.h"

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
  constexpr uint32_t SPIFREQ = 8000000;
};

void setup()
{
  Serial.begin(115200);
  // spibus.begin(VSPI, RTD_SPI_CONF::SCK, RTD_SPI_CONF::MISO, RTD_SPI_CONF::MOSI);
  spibus.begin(VSPI, 27, 33, 26);
  // flash.begin(&spibus, RTD_SPI_CONF::CS_FLASH, RTD_SPI_CONF::SPIFREQ);
  flash.begin(&spibus, 32, 500000);
}

void loop()
{
  if (Serial.available())
  {
    uint8_t sake = Serial.read();
    if (sake == 'w')
    {
      // flash write start
      Serial.println("write start");
      for (int i = 0; i < 0x100; i++)
      {
        uint8_t data[256] = {};
        data[0] = i;
        for (int j = 1; j < 256; j++)
        {
          data[j] = 255 - j;
        }
        flash.write(i * 256, data);
        delay(1);
      }
      Serial.println("write done");
    }
    if (sake == 'r')
    {
      // flash read start
      Serial.println("read start");
      for (int i = 0; i < 0x100; i++)
      {
        Serial.printf("addr:0x%04x\r\n", i * 256);
        uint8_t data[256];
        flash.read(i * 256, data);
        for (int j = 0; j < 256; j++)
        {
          Serial.printf("%02x ", data[j]);
        }
        Serial.println();
        delay(1);
      }
      Serial.println("read done");
    }
    if (sake == 'e')
    {
      // flash erase start
      Serial.println("erase start");
      flash.erase();
      Serial.println("erase done");
    }
    if (sake == 'c')
    {
      // flashの全領域に書き込み，読みだせるか確認
      Serial.println("write start");
      for (int i = 0; i < 0x10000; i++)
      {
        uint8_t data[256] = {};
        data[0] = i;
        data[1] = i >> 8;
        data[2] = i >> 16;
        data[3] = i >> 24;
        for (int j = 4; j < 256; j++)
        {
          data[j] = 255 - j;
        }
        flash.write(i * 256, data);
        Serial.printf("%d,", i);
      }
      Serial.println("write done");

      Serial.println("read start");
      for (int i = 0; i < 0x10000; i++)
      {
        uint8_t data[256];
        flash.read(i * 256, data);
        uint8_t is_error = 0;
        if (data[0] != (i % 0x100))
        {
          is_error = 1;
        }
        if (data[1] != (i >> 8))
        {
          is_error = 1;
        }
        if (data[2] != (i >> 16))
        {
          is_error = 1;
        }
        if (data[3] != (i >> 24))
        {
          is_error = 1;
        }
        if (is_error)
        {
          Serial.printf("error: %d\n", i);
          break;
        }
        else
        {
          Serial.printf("%d,", i);
        }
      }
      Serial.println("read done");
      Serial.println("check done");
    }
  }
}