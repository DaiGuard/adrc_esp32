#include <Arduino.h>
#include <Wire.h>
#include <RobotLocalization/EKF.h>

#include "Encoder_AS5600.h"
#include "Imu_BMX055.h"

// x, y, th, x', y', th'
float x[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// v, arx, ary, w
float y[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float y_offset[4] = {0.0f, 0.749f, -0.806f, 0.029f};
float P[6] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
float Q[6] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
float R[3] = {0.01f, 0.01f, 0.8f};
float x_new[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
RobotLocalization::EKF ekf(
  x, // [x, y, th, x', y', th']
  P, // P
  Q, // Q
  R // R
);

Encoder_AS5600 g_encoder(0.0638*M_PI/45056);
Imu_BMX055 g_imu;

TaskHandle_t calcHandle;


void calc(void* arg)
{
    // I2Cの初期化
    Wire.begin();
    Wire.setClock(400000);

    g_encoder.begin(&Wire, 0x36);
    g_imu.begin(&Wire, 0x19, 0x69, 0x13);

    int64_t pulse_interval;
    float vel;
    float dt;
    float imu_acc[3], imu_gyro[3];
    uint64_t start_time, stop_time;
    
    start_time = millis();
    stop_time = start_time;

    while(1)
    {        
        // エンコーダパルス取得
        pulse_interval = g_encoder.readInterval();
        // IMU値取得
        g_imu.readAcc(imu_acc);
        g_imu.readGyro(imu_gyro);

        // 計測時間間隔
        stop_time = millis();
        dt = (stop_time - start_time)/1000.0f;
        start_time = stop_time;

        // 速度計算
        vel = g_encoder.pulse2vel(pulse_interval, dt);

        // 観測情報の登録
        y[0] = vel - y_offset[0];
        y[1] = - imu_acc[0] - y_offset[1];
        y[2] = - imu_acc[1] - y_offset[2];
        y[3] = (imu_gyro[2] - y_offset[3])  / 180.0f * M_PI;

        // 現在状態の確保
        memcpy(x, x_new, sizeof(float)*6);
        ekf.update(x, y, dt, x_new);

        Serial.print(x_new[0]); Serial.print(", ");
        Serial.print(x_new[1]); Serial.print(", ");
        Serial.print(x_new[2]); Serial.print(", ");
        Serial.println();

        delay(5);
    }
}


void setup()
{
    Serial.begin(115200);

    xTaskCreateUniversal(calc, "calc", 8192, NULL, 3, &calcHandle, APP_CPU_NUM);
}


void loop()
{
    delay(100);
}