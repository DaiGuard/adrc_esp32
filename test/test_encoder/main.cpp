#include <Arduino.h>
#include <Wire.h>

#include "Encoder_AS5600.h"


uint64_t start_time, stop_time;
float dt;

void setup()
{
    // シリアル通信初期化
    Serial.begin(115200);

    // I2Cの初期化
    Wire.begin();
    Wire.setClock(400000);

    // エンコーダ初期化
    Encoder.begin(&Wire, 0.0638*M_PI/45056, 0x36);

    // 初期時間計測
    start_time = millis();
}

void loop()
{    
    // 計測時間間隔
    stop_time = millis();
    dt = (stop_time - start_time)/1000.0f;
    start_time = stop_time;

    // エンコーダパルス取得
    int32_t pulse_interval = Encoder.readInterval();          

    // 速度計算
    float vel = Encoder.pulse2vel(pulse_interval, dt);

    Serial.println(vel);

    delay(1);
}