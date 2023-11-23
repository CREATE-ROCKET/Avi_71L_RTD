#include <Arduino.h>
#include "SPICREATE.h"
#include "ICM20602.h"

#include "esp_log.h"

constexpr char *TAG = "INIT";

SPICREATE::SPICreate spibus;
ICM imu;

namespace RTD_SPI_CONF
{
  constexpr uint8_t SCK = 27;
  constexpr uint8_t MISO = 33;
  constexpr uint8_t MOSI = 26;
  constexpr uint8_t CS_PRESSURE = 13;
  constexpr uint8_t CS_FLASH = 32;
  constexpr uint8_t CS_IMU = 25;
  constexpr uint32_t SPIFREQ = 5000000;
};

IRAM_ATTR void imu_task(void *arg)
{
  int16_t arr[6];
  uint32_t index = 0;
  int64_t ave_ax = 0, ave_ay = 0, ave_az = 0;
  while (1)
  {
    portTickType xLastWakeTime = xTaskGetTickCount();
    imu.Get(arr);
    ave_ax += arr[0];
    ave_ay += arr[1];
    ave_az += arr[2];

    if (++index >= 10000)
    {
      index = 0;
      ESP_LOGI(TAG, "ave: x:%f, y:%f, z:%f", (float)ave_ax / 10000, (float)ave_ay / 10000, (float)ave_az / 10000);
      ave_ax = 0;
      ave_ay = 0;
      ave_az = 0;
    }
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  spibus.begin(VSPI, RTD_SPI_CONF::SCK, RTD_SPI_CONF::MISO, RTD_SPI_CONF::MOSI, RTD_SPI_CONF::CS_IMU);
  imu.begin(&spibus, RTD_SPI_CONF::CS_IMU, RTD_SPI_CONF::SPIFREQ);

  xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}

void loop()
{
}