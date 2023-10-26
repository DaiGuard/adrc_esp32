#include "Encoder_AS5600.h"

#include <Wire.h>


const uint8_t   AS5600_ZMCO        = 0x00;
const uint8_t   AS5600_ZMCO_SIZE   = 1;
const uint8_t   AS5600_ZPOS        = 0x01;
const uint8_t   AS5600_ZPOS_SIZE   = 2;
const uint8_t   AS5600_MPOS        = 0x03;
const uint8_t   AS5600_MPOS_SIZE   = 2;
const uint8_t   AS5600_MANG        = 0x05;
const uint8_t   AS5600_MANG_SIZE   = 2;
const uint8_t   AS5600_CONF        = 0x07;
const uint8_t   AS5600_CONF_SIZE   = 2;
const uint8_t   AS5600_RAWANGLE    = 0x0c;
const uint8_t   AS5600_RAWANGLE_SIZE = 2;
const uint8_t   AS5600_ANGLE       = 0x0e;
const uint8_t   AS5600_ANGLE_SIZE  = 2;
const uint8_t   AS5600_STATUS      = 0x0b;
const uint8_t   AS5600_STATUS_SIZE = 2;
const uint8_t   AS5600_AGC         = 0x1a;
const uint8_t   AS5600_AGC_SIZE    = 1;
const uint8_t   AS5600_MAGNITUDE   = 0x1b;
const uint8_t   AS5600_MAGNITUDE_SIZE= 2;
const uint8_t   AS5600_BURN        = 0xff;
const uint8_t   AS5600_BURN_SIZE   = 1;


const int32_t   AS5600_RESOLUTION_PPR  = 4096;
const int32_t   AS5600_RESOLUTION_PPR_1_4 = AS5600_RESOLUTION_PPR / 4;
const int32_t   AS5600_RESOLUTION_PPR_3_4 = AS5600_RESOLUTION_PPR_1_4 * 3;


bool Encoder_AS5600::begin(TwoWire* wire, float ratio, uint8_t address)
{
    _wire       = wire;
    _address    = address;

    _pulse_count    = 0;
    _ratio          = ratio;

    return true;
}


uint16_t Encoder_AS5600::readAngle()
{
    if(_wire != NULL) 
    {
        _wire->beginTransmission(_address);
        _wire->write(AS5600_ANGLE);
        _wire->endTransmission(false);
        _wire->requestFrom(_address, AS5600_ANGLE_SIZE);        

        uint16_t angle = 0;
        angle  = ((uint16_t)_wire->read() << 8) & 0x0F00;
        angle |= (uint16_t)_wire->read();

        return angle;
    }

    return (uint16_t)0;
}


uint16_t Encoder_AS5600::readRawAngle()
{
    if(_wire != NULL) 
    {
        _wire->beginTransmission(_address);
        _wire->write(AS5600_RAWANGLE);
        _wire->endTransmission(false);
        _wire->requestFrom(_address, AS5600_RAWANGLE_SIZE);        

        uint16_t angle = 0;
        angle  = ((uint16_t)_wire->read() << 8) & 0x0F00;
        angle |= (uint16_t)_wire->read();

        return angle;
    }

    return (uint16_t)0;
}


uint16_t Encoder_AS5600::readStatus()
{
    if(_wire != NULL) 
    {


    }

    return (uint16_t)0;
}


int32_t Encoder_AS5600::readInterval()
{
    if(_wire != NULL)
    {
        int32_t interval = 0u;
        uint16_t current_angle = readRawAngle();

        if(_last_angle >= AS5600_RESOLUTION_PPR_3_4)
        {
            if(current_angle <= AS5600_RESOLUTION_PPR_1_4)
            {
                interval = AS5600_RESOLUTION_PPR - _last_angle + current_angle;
            }
            else
            {
                interval = current_angle - _last_angle;
            }
        }
        else if(_last_angle <= AS5600_RESOLUTION_PPR_1_4)
        {
            if(current_angle >= AS5600_RESOLUTION_PPR_3_4)
            {
                interval = current_angle - AS5600_RESOLUTION_PPR - _last_angle;
            }
            else
            {
                interval = current_angle - _last_angle;
            }
        }
        else{
            interval = current_angle - _last_angle;
        }        

        _last_angle = current_angle;
        _pulse_count += interval;

        return interval;
    }

    return 0;
}


int64_t Encoder_AS5600::readPulseCount()
{
    readInterval();

    return _pulse_count;
}


Encoder_AS5600 Encoder;