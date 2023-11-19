/*
Author : Yuto Noguchi
Date : 2023/11/08
*/

#ifndef Quarternion_
#define Quarternion_
#include <Arduino.h>

class Quarternion
{
private:
    float gyro_drift[3];
    void q_norm(float *q);
    void q_rot_by_omega(float *q, float *omega, float q_dt);
    float transform_matrix[3][3] = {
        {cosf(1. / 8. * PI), sinf(1. / 8. * PI), 0},
        {sinf(1. / 8. * PI), cosf(1. / 8. * PI), 0},
        {0, 0, 1}};

public:
    float q[4];
    void Init_by_launcher_inclination(float inclination);
    void Init(float q_w, float q_x, float q_y, float q_z, float q_dt);
    void Drift(float *gyro_drift_data);
    void Calc(int16_t *Sensor_Data, float q_dt);
};

IRAM_ATTR void Quarternion::q_norm(float *_q)
{
    float q_n;
    q_n = sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    for (uint8_t i = 0; i < 4; i++)
    {
        q[i] = _q[i] / q_n;
    }
}

IRAM_ATTR void Quarternion::q_rot_by_omega(float *_q, float *omega, float q_dt)
{
    float q_buf[4];
    q_buf[0] = _q[0] + 0.5 * (-omega[0] * _q[1] - omega[1] * _q[2] - omega[2] * _q[3]) * q_dt;
    q_buf[1] = _q[1] + 0.5 * (omega[0] * _q[0] + omega[2] * _q[2] - omega[1] * _q[3]) * q_dt;
    q_buf[2] = _q[2] + 0.5 * (omega[1] * _q[0] - omega[2] * _q[1] + omega[0] * _q[3]) * q_dt;
    q_buf[3] = _q[3] + 0.5 * (omega[2] * _q[0] + omega[1] * _q[1] - omega[0] * _q[2]) * q_dt;
    q_norm(q_buf);
}

/**
 * @brief
 *
 * @param inclination ランチャの傾き，単位はrad，5degのとき，-5/180*PI
 * @return IRAM_ATTR
 */
IRAM_ATTR void Quarternion::Init_by_launcher_inclination(float inclination)
{
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;

    float omega[3];
    omega[0] = 0;
    omega[1] = inclination;
    omega[2] = 0;

    q_rot_by_omega(q, omega, 1);
}

IRAM_ATTR void Quarternion::Init(float q_w, float q_x, float q_y, float q_z, float q_dt)
{
    q[0] = q_w;
    q[1] = q_x;
    q[2] = q_y;
    q[3] = q_z;
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
    float _omega[3];
    float q_buf[4];
    float q_norm;
    for (uint8_t i = 0; i < 3; i++)
    {
        omega[i] = ((float)Sensor_Data[i + 3] - gyro_drift[i]) * 0.001065264;
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        _omega[i] = transform_matrix[i][0] * omega[0] + transform_matrix[i][1] * omega[1] + transform_matrix[i][2] * omega[2];
    }
    q_rot_by_omega(q, _omega, q_dt);
}

#endif