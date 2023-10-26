#include "Driver_DRV8835.h"


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


bool Driver_DRV8835::begin(int ap, int ae, int bp, int be)
{
    _a_phase_pin = ap; 
    _a_enable_pin = ae;
    _b_phase_pin = bp;
    _b_enable_pin = be;

    _a_i_value = 0.0f;
    _b_i_value = 0.0f;

    _a_value = 0.0f;
    _b_value = 0.0f;

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

    delay(1000);

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


void Driver_DRV8835::controlAvalue(float dif, float p, float i)
{
    // _a_i_value += i * dif;
    // if(_a_i_value > 0.1f){ _a_i_value = 0.1f; }
    // else if(_a_i_value < -0.1f){ _a_i_value = -0.1f; }

    _a_value = p * dif  + _a_i_value;

    if(_a_value > 1.0f){ _a_value = 1.0f; }
    else if(_a_value < -1.0f){ _a_value = -1.0f; }

    setAvalue(_a_value);
}


void Driver_DRV8835::controlBvalue(float dif, float p, float i)
{
    _b_i_value += i * dif;
    if(_b_i_value > 0.1f){ _b_i_value = 0.1f; }
    else if(_b_i_value < -0.1f){ _b_i_value = -0.1f; }

    _b_value = p * dif  + _b_i_value;

    if(_b_value > 1.0f){ _b_value = 1.0f; }
    else if(_b_value < -1.0f){ _b_value = -1.0f; }

    setBvalue(_b_value);
}


Driver_DRV8835 Driver;