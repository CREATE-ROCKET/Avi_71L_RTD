#include <Arduino.h>
#include "SPICREATE.h"
#include "ICM20602.h"
#include "Quarternion.h"

#include "esp_log.h"

constexpr char *TAG = "INIT";

SPICREATE::SPICreate spibus;
ICM imu;

Quarternion q4;
float accOffset[3] = {-82.313202, 7.249300, 7.942400};

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

float velocity[3] = {0, 0, 0};
float position[3] = {0, 0, 0};

IRAM_ATTR void imu_task(void *arg)
{
  int16_t arr[6];
  uint32_t index = 0;
  while (1)
  {
    portTickType xLastWakeTime = xTaskGetTickCount();
    imu.Get(arr);
    q4.Calc(arr, 0.001);
    float acc[3];
    for (int i = 0; i < 3; i++)
    {
      acc[i] = ((float)arr[i] - accOffset[i]) / 2048 * 9.80665;
    }
    q4.transform_acceleration(acc, q4.q);
    for (int i = 0; i < 3; i++)
    {
      velocity[i] += acc[i] * 0.001;
      position[i] += velocity[i] * 0.001;
    }

    if (++index >= 1000)
    {
      index = 0;
      ESP_LOGI(TAG, "q: %f, %f, %f, %f", q4.q[0], q4.q[1], q4.q[2], q4.q[3]);
      ESP_LOGI(TAG, "imu: %d, %d, %d, %d, %d, %d", arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]);
      ESP_LOGI(TAG, "acc: %f, %f, %f", acc[0], acc[1], acc[2]);
      ESP_LOGI(TAG, "pos: x:%f, y:%f, z:%f", position[0], position[1], position[2]);
    }
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  spibus.begin(VSPI, RTD_SPI_CONF::SCK, RTD_SPI_CONF::MISO, RTD_SPI_CONF::MOSI, RTD_SPI_CONF::CS_IMU);
  imu.begin(&spibus, RTD_SPI_CONF::CS_IMU, RTD_SPI_CONF::SPIFREQ);

  q4.Init_by_launcher_inclination(0 / 180. * PI);

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
  ESP_LOGI(TAG, "Gyro Drift: %f, %f, %f", gyro_drift[0], gyro_drift[1], gyro_drift[2]);
  q4.Drift(gyro_drift);

  delay(1000);

  xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}

void loop()
{
}