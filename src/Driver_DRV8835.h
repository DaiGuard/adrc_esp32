#ifndef __DRVIER_DRV8835_H__
#define __DRVIER_DRV8835_H__


#include <ESP32Servo.h>


class Driver_DRV8835
{
    public:
        Driver_DRV8835(int ap, int ae, int bp, int be);
        ~Driver_DRV8835();

        void setAchRange(int min, int max, int center);
        void setBchRange(int min, int max, int center);

        bool begin();

        void setAvalue(float value);
        void setBvalue(float value);

    private:
        void setRange(int min, int max, int center, int *range);

        const int _a_phase_pin;
        const int _a_enable_pin;
        const int _b_phase_pin;
        const int _b_enable_pin;

        Servo _a_servo;
        Servo _b_servo;

        int _a_range[3];
        int _b_range[3];
};

#endif