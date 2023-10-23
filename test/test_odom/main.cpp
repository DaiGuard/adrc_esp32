#include <Arduino.h>
#include <Wire.h>
#include <RobotLocalization/EKF.h>

#include "Encoder_AS5600.h"
#include "Imu_BMX055.h"
#include "Driver_DRV8835.h"


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

// エンコーダクラス
Encoder_AS5600 g_encoder(0.0638*M_PI/45056);
// IMUクラス
Imu_BMX055 g_imu;
// モータドライバクラス
Driver_DRV8835 g_drive(33, 25, 26, 27);

// 目標速度
float target_vel[2] = {0.0f, 0.0f};
float current_vel[2] = {0.0f, 0.0f};

TaskHandle_t calcHandle;


void calc(void* arg)
{
    // I2Cの初期化
    Wire.begin();
    Wire.setClock(400000);

    // エンコーダ初期化
    g_encoder.begin(&Wire, 0x36);
    // IMU初期化
    g_imu.begin(&Wire, 0x19, 0x69, 0x13);
    // モータドライバ初期化
    g_drive.setAchRange(500, 2400, 1450);
    g_drive.setBchRange(700, 2700, 1650);
    g_drive.begin();

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

        current_vel[0] = vel;
        current_vel[1] = x_new[5];

        // モータ制御        
        g_drive.setAvalue(target_vel[0]);
        g_drive.setBvalue(target_vel[1]);

        delay(1);
    }
}


void setup()
{
    // シリアルデバック用設定
    Serial.begin(115200);

    // センサ用ループのタスク登録    
    xTaskCreateUniversal(calc, "calc", 8192, NULL, 3, &calcHandle, APP_CPU_NUM);
}


void loop()
{   
    uint32_t keys = 0x00000000;
    while(Serial.available() > 0){
        uint8_t key = Serial.read();        
        keys = (keys << 8) | key;

        switch(keys){
            case 0x1b5b41:
                target_vel[0] += 0.01;
                break;
            case 0x1b5b42:
                target_vel[0] -= 0.01;
                break;
            case 0x1b5b43:
                target_vel[1] += 0.01;
                break;
            case 0x1b5b44:
                target_vel[1] -= 0.01;
                break;
        }        
    }

    // target_vel[0] = max: 0.07, min: -0.06
    // target_vel[1] = max: 0.5, min: -0.5

    Serial.print(target_vel[0]); Serial.print(" ");
    Serial.print(target_vel[1]); Serial.print(" ");
    Serial.print(current_vel[0]); Serial.print(" ");
    Serial.print(current_vel[1]); Serial.print(" ");
    Serial.println();

    delay(100);
}