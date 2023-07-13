#include <Arduino.h>
#include "LPS25HB.h"
#include "SPICREATE.h"
#include "S25FL512S.h"
#include "ICM20948.h"

#include "esp_log.h"

static const char* TAG = "INIT";

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

void setup()
{
  Serial.begin(115200);

  spibus.begin(VSPI, SCK, MISO, MOSI);
  ESP_LOGV(TAG,"SPIBUS");
  pressureSensor.begin(&spibus, CS_PRESSURE, SPIFREQ);
  ESP_LOGV(TAG,"pressure");
  flash.begin(&spibus, CS_FLASH,SPIFREQ);
  ESP_LOGV(TAG,"flash");
  imu.begin(&spibus, CS_IMU,SPIFREQ);
  ESP_LOGV(TAG,"IMU");

  ESP_LOGV(TAG,"whoAmI p:%d, IMU:%d, Flash:%d",pressureSensor.WhoAmI(),imu.WhoAmI(),128);

  uint8_t bf[256];
  flash.read(0,bf);
  for (int i = 0; i < 256; i++)
  {
    Serial.printf("%x,",bf[i]);
  }
  Serial.println();

  for (int i = 0; i < 256; i++)
  {
    bf[i] = 255-i;
  }

  flash.write(0,bf);

  delay(1000);

  flash.read(0,bf);
  for (int i = 0; i < 256; i++)
  {
    Serial.printf("%x,",bf[i]);
  }
  Serial.println();
}

void loop()
{
  uint8_t bf[3];
  pressureSensor.Get(bf);
  uint32_t marged = 0;
  for (int i = 0; i < 3; i++)
  {
    marged += bf[i]<<(i*8);
    printf("%d,\t",bf[i]);
  }
  printf("%d\r\n",marged);

  delay(1000);

  if(Serial.available()){
    uint8_t tmp = Serial.read();
    if(tmp == 'r'){
      ESP.restart();
    }
  }
}