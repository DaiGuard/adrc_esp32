#ifndef __ENCOER_AS5600_H__
#define __ENCOER_AS5600_H__


#include <stdint.h>


class TwoWire;


class Encoder_AS5600
{
    public:
        Encoder_AS5600(){}
        ~Encoder_AS5600(){}

        bool begin(TwoWire* wire, float ratio, uint8_t address=0x36);

        float pulse2vel(int32_t pulse, float timeval) {
            return (float)pulse * _ratio / timeval;            
        }

        uint16_t readAngle();
        uint16_t readRawAngle();

        uint16_t readStatus();

        int32_t readInterval();
        int64_t readPulseCount();

    private:
        TwoWire*    _wire;
        uint8_t     _address;

        uint16_t    _last_angle;
        int64_t     _pulse_count;
        float       _ratio;
};

extern Encoder_AS5600 Encoder;

#endif