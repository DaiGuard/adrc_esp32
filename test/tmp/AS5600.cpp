#include "AS5600.h"

#define DEV_ADDR    0x36
#define REG_RAW_ANGLE   0x0c


TwoWire* as5600_wire;


void AS5600_init(TwoWire* wire)
{
    as5600_wire = wire;
}


uint16_t AS5600_angle()
{
    as5600_wire->beginTransmission(DEV_ADDR);
    as5600_wire->write(REG_RAW_ANGLE);
    as5600_wire->endTransmission(false);
    as5600_wire->requestFrom(DEV_ADDR, 2);

    uint16_t angle = 0;
    angle  = ((uint16_t)as5600_wire->read() << 8) & 0x0F00;
    angle |= (uint16_t)as5600_wire->read();

    return angle;    
}