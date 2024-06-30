#ifndef __DRVIER_DRV8835_H__
#define __DRVIER_DRV8835_H__


#include <ESP32Servo.h>


class Driver_DRV8835
{
    public:
        Driver_DRV8835(){}
        ~Driver_DRV8835(){}

        void setAchRange(int min, int max, int center);
        void setBchRange(int min, int max, int center);

        bool begin(int ap, int ae, int bp, int be);

        void setAvalue(float value);
        void setBvalue(float value);

        void controlAvalue(float dif, float p, float i);
        void controlBvalue(float dif, float p, float i);

    private:
        void setRange(int min, int max, int center, int *range);

        int _a_phase_pin;
        int _a_enable_pin;
        int _b_phase_pin;
        int _b_enable_pin;

        Servo _a_servo;
        Servo _b_servo;
        
        float _a_value;
        float _b_value;

        float _a_i_value;
        float _b_i_value;

        int _a_range[3];
        int _b_range[3];
};


extern Driver_DRV8835 Driver;

#endif