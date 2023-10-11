#include "BMX055.h"

// IMU
#define ACC_ADDR    0x19
#define GYRO_ADDR   0x69
#define MAG_ADDR    0x13


TwoWire* bmx5600_wire;


void BMX055_init(TwoWire* wire)
{
    bmx5600_wire = wire;

    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(ACC_ADDR);
    bmx5600_wire->write(0x0F); // Select PMU_Range register
    bmx5600_wire->write(0x03);   // Range = +/- 2g
    bmx5600_wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(ACC_ADDR);
    bmx5600_wire->write(0x10);  // Select PMU_BW register
    bmx5600_wire->write(0x08);  // Bandwidth = 7.81 Hz
    bmx5600_wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(ACC_ADDR);
    bmx5600_wire->write(0x11);  // Select PMU_LPW register
    bmx5600_wire->write(0x00);  // Normal mode, Sleep duration = 0.5ms
    bmx5600_wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(GYRO_ADDR);
    bmx5600_wire->write(0x0F);  // Select Range register
    bmx5600_wire->write(0x04);  // Full scale = +/- 125 degree/s
    bmx5600_wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(GYRO_ADDR);
    bmx5600_wire->write(0x10);  // Select Bandwidth register
    bmx5600_wire->write(0x07);  // ODR = 100 Hz
    bmx5600_wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(GYRO_ADDR);
    bmx5600_wire->write(0x11);  // Select LPM1 register
    bmx5600_wire->write(0x00);  // Normal mode, Sleep duration = 2ms
    bmx5600_wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(MAG_ADDR);
    bmx5600_wire->write(0x4B);  // Select Mag register
    bmx5600_wire->write(0x83);  // Soft reset
    bmx5600_wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(MAG_ADDR);
    bmx5600_wire->write(0x4B);  // Select Mag register
    bmx5600_wire->write(0x01);  // Soft reset
    bmx5600_wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(MAG_ADDR);
    bmx5600_wire->write(0x4C);  // Select Mag register
    bmx5600_wire->write(0x00);  // Normal Mode, ODR = 10 Hz
    bmx5600_wire->endTransmission();
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(MAG_ADDR);
    bmx5600_wire->write(0x4E);  // Select Mag register
    bmx5600_wire->write(0x84);  // X, Y, Z-Axis enabled
    bmx5600_wire->endTransmission();
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(MAG_ADDR);
    bmx5600_wire->write(0x51);  // Select Mag register
    bmx5600_wire->write(0x04);  // No. of Repetitions for X-Y Axis = 9
    bmx5600_wire->endTransmission();
    //------------------------------------------------------------//
    bmx5600_wire->beginTransmission(MAG_ADDR);
    bmx5600_wire->write(0x52);  // Select Mag register
    bmx5600_wire->write(0x16);  // No. of Repetitions for Z-Axis = 15
    bmx5600_wire->endTransmission();
}


void BMX055_acc(float* acc)
{
    unsigned int data[6];
    for(int i=0; i < 6; i++)
    {
        bmx5600_wire->beginTransmission(ACC_ADDR);
        bmx5600_wire->write((2+i));
        bmx5600_wire->endTransmission();
        bmx5600_wire->requestFrom(ACC_ADDR, 1);
        if(bmx5600_wire->available() == 1)
        {
            data[i] = bmx5600_wire->read();
        }
    }

    // Convert the data to 12-bits
    acc[0] = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (acc[0] > 2047)  acc[0] -= 4096;
    acc[1] = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (acc[1] > 2047)  acc[1] -= 4096;
    acc[2] = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (acc[2] > 2047)  acc[2] -= 4096;
    acc[0] = acc[0] * 0.0098; // range = +/-2g
    acc[1] = acc[1] * 0.0098; // range = +/-2g
    acc[2] = acc[2] * 0.0098; // range = +/-2g
}


void BMX055_gyro(float* gyro)
{
  unsigned int data[6];
  for (int i = 0; i < 6; i++)
  {
    bmx5600_wire->beginTransmission(GYRO_ADDR);
    bmx5600_wire->write((2 + i));    // Select data register
    bmx5600_wire->endTransmission();
    bmx5600_wire->requestFrom(GYRO_ADDR, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (bmx5600_wire->available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  gyro[0] = (data[1] * 256) + data[0];
  if (gyro[0] > 32767)  gyro[0] -= 65536;
  gyro[1] = (data[3] * 256) + data[2];
  if (gyro[1] > 32767)  gyro[1] -= 65536;
  gyro[2] = (data[5] * 256) + data[4];
  if (gyro[2] > 32767)  gyro[2] -= 65536;

  gyro[0] = gyro[0] * 0.0038; //  Full scale = +/- 125 degree/s
  gyro[1] = gyro[1] * 0.0038; //  Full scale = +/- 125 degree/s
  gyro[2] = gyro[2] * 0.0038; //  Full scale = +/- 125 degree/s
}
//=====================================================================================//
void BMX055_mag(float *mag)
{
    unsigned int data[8];
    for (int i = 0; i < 8; i++)
    {
        bmx5600_wire->beginTransmission(MAG_ADDR);
        bmx5600_wire->write((0x42 + i));    // Select data register
        bmx5600_wire->endTransmission();
        bmx5600_wire->requestFrom(MAG_ADDR, 1);    // Request 1 byte of data
        // Read 6 bytes of data
        // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
        if (bmx5600_wire->available() == 1)
        data[i] = bmx5600_wire->read();
    }
    // Convert the data
    mag[0] = ((data[1] <<5) | (data[0]>>3));
    if (mag[0] > 4095)  mag[0] -= 8192;
    mag[1] = ((data[3] <<5) | (data[2]>>3));
    if (mag[1] > 4095)  mag[1] -= 8192;
    mag[2] = ((data[5] <<7) | (data[4]>>1));
    if (mag[2] > 16383)  mag[2] -= 32768;
}