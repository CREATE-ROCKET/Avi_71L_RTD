/*
Author : Ryota KAMEWAKA, Yuto Noguchi
Date : 2023/11/08
*/

#ifndef Quarternion_
#define Quarternion_
#include <Arduino.h>

class Quarternion
{
private:
    float dt;
    float gyro_drift[3];

public:
    float q[4];
    void Init(float q_w, float q_x, float q_y, float q_z, float q_dt);
    void Drift(float *gyro_drift_data);
    void Calc(int16_t *Sensor_Data, float q_dt = -1);
};

IRAM_ATTR void Quarternion::Init(float q_w, float q_x, float q_y, float q_z, float q_dt)
{
    q[0] = q_w;
    q[1] = q_x;
    q[2] = q_y;
    q[3] = q_z;
    dt = q_dt;
}

IRAM_ATTR void Quarternion::Drift(float *gyro_drift_data)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro_drift[i] = gyro_drift_data[i];
    }
}

IRAM_ATTR void Quarternion::Calc(int16_t *Sensor_Data, float q_dt)
{
    float omega[3];
    float q_buf[4];
    float q_norm;
    if (q_dt < 0)
    {
        q_dt = dt;
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        omega[i] = ((float)Sensor_Data[i + 3] - gyro_drift[i]) * 0.001065264;
    }
    q_buf[0] = q[0] + 0.5 * (-omega[0] * q[1] - omega[1] * q[2] - omega[2] * q[3]) * q_dt;
    q_buf[1] = q[1] + 0.5 * (omega[0] * q[0] + omega[2] * q[2] - omega[1] * q[3]) * q_dt;
    q_buf[2] = q[2] + 0.5 * (omega[1] * q[0] - omega[2] * q[1] + omega[0] * q[3]) * q_dt;
    q_buf[3] = q[3] + 0.5 * (omega[2] * q[0] + omega[1] * q[1] - omega[0] * q[2]) * q_dt;
    q_norm = sqrt(q_buf[0] * q_buf[0] + q_buf[1] * q_buf[1] + q_buf[2] * q_buf[2] + q_buf[3] * q_buf[3]);
    for (uint8_t i = 0; i < 4; i++)
    {
        q[i] = q_buf[i] / q_norm;
    }
}

#endif