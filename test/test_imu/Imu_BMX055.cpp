#include "Imu_BMX055.h"

#include <Wire.h>



bool Imu_BMX055::begin(TwoWire* wire, 
            uint8_t acc_address, uint8_t gyro_address, uint8_t mag_address)
{
    _wire = wire;
    _acc_address = acc_address;
    _gyro_address = gyro_address;
    _mag_address = mag_address;

    //------------------------------------------------------------//
    _wire->beginTransmission(_acc_address);
    _wire->write(0x0F); // Select PMU_Range register
    _wire->write(0x03);   // Range = +/- 2g
    _wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    _wire->beginTransmission(_acc_address);
    _wire->write(0x10);  // Select PMU_BW register
    _wire->write(0x08);  // Bandwidth = 7.81 Hz
    _wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    _wire->beginTransmission(_acc_address);
    _wire->write(0x11);  // Select PMU_LPW register
    _wire->write(0x00);  // Normal mode, Sleep duration = 0.5ms
    _wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    _wire->beginTransmission(_gyro_address);
    _wire->write(0x0F);  // Select Range register
    _wire->write(0x04);  // Full scale = +/- 125 degree/s
    _wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    _wire->beginTransmission(_gyro_address);
    _wire->write(0x10);  // Select Bandwidth register
    _wire->write(0x07);  // ODR = 100 Hz
    _wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    _wire->beginTransmission(_gyro_address);
    _wire->write(0x11);  // Select LPM1 register
    _wire->write(0x00);  // Normal mode, Sleep duration = 2ms
    _wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    _wire->beginTransmission(_mag_address);
    _wire->write(0x4B);  // Select Mag register
    _wire->write(0x83);  // Soft reset
    _wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    _wire->beginTransmission(_mag_address);
    _wire->write(0x4B);  // Select Mag register
    _wire->write(0x01);  // Soft reset
    _wire->endTransmission();
    delay(100);
    //------------------------------------------------------------//
    _wire->beginTransmission(_mag_address);
    _wire->write(0x4C);  // Select Mag register
    _wire->write(0x00);  // Normal Mode, ODR = 10 Hz
    _wire->endTransmission();
    //------------------------------------------------------------//
    _wire->beginTransmission(_mag_address);
    _wire->write(0x4E);  // Select Mag register
    _wire->write(0x84);  // X, Y, Z-Axis enabled
    _wire->endTransmission();
    //------------------------------------------------------------//
    _wire->beginTransmission(_mag_address);
    _wire->write(0x51);  // Select Mag register
    _wire->write(0x04);  // No. of Repetitions for X-Y Axis = 9
    _wire->endTransmission();
    //------------------------------------------------------------//
    _wire->beginTransmission(_mag_address);
    _wire->write(0x52);  // Select Mag register
    _wire->write(0x16);  // No. of Repetitions for Z-Axis = 15
    _wire->endTransmission();

    return true;
}


bool Imu_BMX055::readAcc(float* acc)
{
    unsigned int data[6];

    if(_wire != NULL) 
    {
        for(int i=0; i < 6; i++)
        {
            _wire->beginTransmission(_acc_address);
            _wire->write((2+i));
            _wire->endTransmission();
            _wire->requestFrom(_acc_address, (uint8_t)1u);
            if(_wire->available() == 1)
            {
                data[i] = _wire->read();
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

        return true;
    }

    return false;
}


bool Imu_BMX055::readGyro(float* gyro)
{
    unsigned int data[6];

    if(_wire != NULL) 
    {
        for (int i = 0; i < 6; i++)
        {
            _wire->beginTransmission(_gyro_address);
            _wire->write((2 + i));    // Select data register
            _wire->endTransmission();
            _wire->requestFrom(_gyro_address, (uint8_t)1u);    // Request 1 byte of data
            // Read 6 bytes of data
            // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
            if (_wire->available() == 1)
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

        return true;
    }

    return false;
}


bool Imu_BMX055::readMag(float* mag)
{
    unsigned int data[8];

    if(_wire != NULL) 
    {
        for (int i = 0; i < 8; i++)
        {
            _wire->beginTransmission(_mag_address);
            _wire->write((0x42 + i));    // Select data register
            _wire->endTransmission();
            _wire->requestFrom(_mag_address, (uint8_t)1u);    // Request 1 byte of data
            // Read 6 bytes of data
            // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
            if (_wire->available() == 1)
            data[i] = _wire->read();
        }
        // Convert the data
        mag[0] = ((data[1] <<5) | (data[0]>>3));
        if (mag[0] > 4095)  mag[0] -= 8192;
        mag[1] = ((data[3] <<5) | (data[2]>>3));
        if (mag[1] > 4095)  mag[1] -= 8192;
        mag[2] = ((data[5] <<7) | (data[4]>>1));
        if (mag[2] > 16383)  mag[2] -= 32768;

        return true;
    }

    return false;
}


Imu_BMX055 Imu;