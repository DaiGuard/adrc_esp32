#include "Driver_DRV8835.h"



Driver_DRV8835::Driver_DRV8835(int ap, int ae, int bp, int be):
    _a_phase_pin(ap), _a_enable_pin(ae), _b_phase_pin(bp), _b_enable_pin(be)
{
    _a_range[0] = 0;
    _a_range[1] = 0;
    _a_range[2] = 0;

    _b_range[0] = 0;
    _b_range[1] = 0;
    _b_range[2] = 0;
}


Driver_DRV8835::~Driver_DRV8835()
{

}


void Driver_DRV8835::setRange(int min, int max, int center, int *range)
{
    range[0] = min;
    range[1] = max;
    range[2] = center;
}

void Driver_DRV8835::setAchRange(int min, int max, int center)
{
    setRange(min, max, center, _a_range);
}


void Driver_DRV8835::setBchRange(int min, int max, int center)
{
    setRange(min, max, center, _b_range);
}


bool Driver_DRV8835::begin()
{
    // A channel initialize
    pinMode(_a_phase_pin, OUTPUT);
    digitalWrite(_a_phase_pin, LOW);
    _a_servo.setPeriodHertz(50);
    _a_servo.attach(_a_enable_pin, _a_range[0], _a_range[1]);

    // B channel initialize
    pinMode(_b_phase_pin, OUTPUT);
    digitalWrite(_b_phase_pin, LOW);
    _b_servo.setPeriodHertz(50);
    _b_servo.attach(_b_enable_pin, _b_range[0], _b_range[1]);

    // output initial duty
    _a_servo.write(_a_range[2]);
    _b_servo.write(_b_range[2]);

    return true;
}


void Driver_DRV8835::setAvalue(float value)
{
    if(value >= 0.0)
    {
        _a_servo.write((int)(_a_range[1] - _a_range[2]) * value + _a_range[2]);
    }
    else
    {
        _a_servo.write((int)(_a_range[2] - _a_range[0]) * value + _a_range[2]);
    }
}


void Driver_DRV8835::setBvalue(float value)
{
    if(value >= 0.0)
    {
        _b_servo.write((int)(_b_range[1] - _b_range[2]) * value + _b_range[2]);
    }
    else
    {
        _b_servo.write((int)(_b_range[2] - _b_range[0]) * value + _b_range[2]);
    }
}