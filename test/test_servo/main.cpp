#include <Arduino.h>
#include <Wire.h>

#include "Driver_DRV8835.h"

bool dir_a = true;
bool dir_b = true;
float target_a = 0.0f;
float target_b = 0.0f;

void setup()
{
    // モータドライバ初期化
    Driver.setAchRange(500, 2400, 1400);
    Driver.setBchRange(1100, 2200, 1650);

    // モータドライバ開始
    Driver.begin(33, 25, 26, 27);

    // モータ制御        
    Driver.setAvalue(0.0);
    Driver.setBvalue(0.0);

    delay(500);
}

void loop()
{
    if(dir_a)
    {
        target_a += 0.01f;
        if(target_a >= 0.3)
        {
            dir_a = false;
        }
    }
    else
    {
        target_a -= 0.01f;
        if(target_a <= -0.3)
        {
            dir_a = true;
        }
    }

    if(dir_b)
    {
        target_b += 0.1f;
        if(target_b >= 1.0)
        {
            dir_b = false;
        }
    }
    else
    {
        target_b -= 0.1f;
        if(target_b <= -1.0)
        {
            dir_b = true;
        }
    }

    Driver.setAvalue(0.0f);
    Driver.setBvalue(0.0f);

    delay(50);
}