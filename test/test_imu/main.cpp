#include <Arduino.h>
#include <Wire.h>

#include "Imu_BMX055.h"

void setup()
{
    // シリアル通信初期化
    Serial.begin(115200);

    // I2Cの初期化
    Wire.begin();
    Wire.setClock(400000);

    // IMU初期化
    Imu.begin(&Wire, 0x19, 0x69, 0x13);

    delay(500);
}


void loop()
{
    float acc[3], gyro[3], mag[3];
    Imu.readAcc(acc);
    Imu.readGyro(gyro);
    Imu.readMag(mag);

    delay(100);

    Serial.print(acc[0]); Serial.print(", ");
    Serial.print(acc[1]); Serial.print(", ");
    Serial.print(acc[2]); Serial.print(", ");
    Serial.println();
}