#include "Encoder_AS5600.h"

#include <Wire.h>


const uint8_t   ZMCO        = 0x00;
const int32_t   ZMCO_SIZE   = 1;
const uint8_t   ZPOS        = 0x01;
const int32_t   ZPOS_SIZE   = 2;
const uint8_t   MPOS        = 0x03;
const int32_t   MPOS_SIZE   = 2;
const uint8_t   MANG        = 0x05;
const int32_t   MANG_SIZE   = 2;
const uint8_t   CONF        = 0x07;
const int32_t   CONF_SIZE   = 2;
const uint8_t   RAWANGLE    = 0x0c;
const int32_t   RAWANGLE_SIZE = 2;
const uint8_t   ANGLE       = 0x0e;
const int32_t   ANGLE_SiZE  = 2;
const uint8_t   STATUS      = 0x0b;
const int32_t   STATUS_SIZE = 2;
const uint8_t   AGC         = 0x1a;
const int32_t   AGC_SIZE    = 1;
const uint8_t   MAGNITUDE   = 0x1b;
const int32_t   MAGNITUDE_SiZE= 2;
const uint8_t   BURN        = 0xff;
const int32_t   BURN_SIZE   = 1;


const int32_t   RESOLUTION_PPR  = 4096;
const int32_t   RESOLUTION_PPR_1_4 = RESOLUTION_PPR / 4;
const int32_t   RESOLUTION_PPR_3_4 = RESOLUTION_PPR_1_4 * 3;


Encoder_AS5600::Encoder_AS5600()
{
    _wire = NULL;
    _address = 0x00;
}

Encoder_AS5600::~Encoder_AS5600()
{

}


bool Encoder_AS5600::begin(TwoWire* wire, uint8_t address)
{
    _wire       = wire;
    _address    = address;
}


uint16_t Encoder_AS5600::readAngle()
{
    if(_wire != NULL) 
    {
        _wire->beginTransmission(_address);
        _wire->write(ANGLE);
        _wire->endTransmission(false);
        _wire->requestFrom(_address, ANGLE_SIZE);
        _wire->read() << 8 | _wire->read();
    }
}