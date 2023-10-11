#ifndef __AS5600_H__
#define __AS5600_H__

#include <Wire.h>

void AS5600_init(TwoWire *wire);
uint16_t AS5600_angle();

#endif