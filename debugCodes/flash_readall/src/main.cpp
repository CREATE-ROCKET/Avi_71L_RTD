#include <Arduino.h>
#include "SPICREATE.h"
#include "S25FL512S.h"

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
    if (data = 'r')
    {
      // read all data
      for (uint32_t i = 0; i < 0x10010; i++)
      {
        uint8_t data[256];
        flash.read(i << 8, data);

        for (uint8_t j = 0; j < 256; j++)
        {
          Serial.printf("%02X,", data[j]);
        }
        Serial.println();
      }
    }
  }
}