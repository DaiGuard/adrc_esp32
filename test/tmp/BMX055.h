#ifndef __BMX055_H__
#define __BMX055_H__

#include <Wire.h>

void BMX055_init(TwoWire* wire);
void BMX055_acc(float* acc);
void BMX055_gyro(float* gyro);
void BMX055_mag(float* mag);

#endif