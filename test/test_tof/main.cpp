#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_VL53L1X.h>

// Right ToF Sensor
const uint8_t rtof_address = 0x28;
const int rtof_shut_pin = 5;
Adafruit_VL53L1X right_tof = Adafruit_VL53L1X(rtof_shut_pin);
// Center ToF Sensor
const uint8_t ctof_address = 0x27;
const int ctof_shut_pin = 18;
Adafruit_VL53L1X center_tof = Adafruit_VL53L1X(ctof_shut_pin);
// Left ToF Sensor
const uint8_t ltof_address = 0x26;
const int ltof_shut_pin = 19;
Adafruit_VL53L1X left_tof = Adafruit_VL53L1X(ltof_shut_pin);


void setup()
{
    // シリアル通信初期化
    Serial.begin(115200);

    // I2Cの初期化
    Wire.begin();
    Wire.setClock(400000);

    // ToF初期化
    right_tof.VL53L1X_Off();
    center_tof.VL53L1X_Off();
    left_tof.VL53L1X_Off();

    right_tof.begin(rtof_address, &Wire);
    center_tof.begin(ctof_address, &Wire);
    left_tof.begin(ltof_address, &Wire);

    right_tof.startRanging();
    right_tof.setTimingBudget(100);

    center_tof.startRanging();
    center_tof.setTimingBudget(100);

    left_tof.startRanging();
    left_tof.setTimingBudget(100);

}

void loop()
{
    float range[3];

    // ToF距離測定
    if(right_tof.dataReady() 
        && center_tof.dataReady()
        && left_tof.dataReady()
        )
    {
        range[0] = right_tof.distance();
        range[1] = center_tof.distance();
        range[2] = left_tof.distance();

        Serial.print(range[0]); Serial.print(", ");
        Serial.print(range[1]); Serial.print(", ");
        Serial.print(range[2]); Serial.print(", ");
        Serial.println();
    }

    delay(100);
}