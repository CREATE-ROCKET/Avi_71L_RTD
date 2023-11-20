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
        {-sinf(1. / 8. * PI), cosf(1. / 8. * PI), 0},
        {0, 0, 1}};
    // float transform_matrix[3][3] = {
    //     {1, 0, 0},
    //     {0, 1, 0},
    //     {0, 0, 1}};
    void quaternion_multiply(float *q1, float *q2, float *result);

public:
    float q[4];
    void Init_by_launcher_inclination(float inclination);
    void Init(float q_w, float q_x, float q_y, float q_z, float q_dt);
    void Drift(float *gyro_drift_data);
    void Calc(int16_t *Sensor_Data, float q_dt);
    void transform_acceleration(float *accel, float *q);
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

IRAM_ATTR void Quarternion::transform_acceleration(float *accel, float *_q)
{
    // 加速度ベクトルをクオータニオンに変換（実部は0）
    float accel_quat[4] = {0, accel[0], accel[1], accel[2]};

    // クオータニオンの共役を計算
    float q_conj[4] = {_q[0], -_q[1], -_q[2], -_q[3]};

    // 回転を適用（q * accel_quat * q_conj）
    float temp[4], result[4];
    quaternion_multiply(_q, accel_quat, temp); // temp = q * accel_quat
    quaternion_multiply(temp, q_conj, result); // result = temp * q_conj

    // 結果のベクトル部分を加速度として返す
    accel[0] = result[1];
    accel[1] = result[2];
    accel[2] = result[3];
}

// クオータニオンの積を計算する関数
IRAM_ATTR void Quarternion::quaternion_multiply(float *q1, float *q2, float *result)
{
    result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

#endif