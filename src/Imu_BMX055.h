#ifndef __IMU_BMX055_H__
#define __IMU_BMX055_H__

#include <stdint.h>


class TwoWire;


class Imu_BMX055
{
    public:
        Imu_BMX055(){}
        ~Imu_BMX055(){}

        bool begin(TwoWire* wire, 
            uint8_t acc_address=0x19,
            uint8_t gyro_address=0x69,
            uint8_t mag_address=0x13);

        bool readAcc(float* acc);
        bool readGyro(float* gyro);
        bool readMag(float* mag);

    private:
        TwoWire*    _wire;
        uint8_t     _acc_address;
        uint8_t     _gyro_address;
        uint8_t     _mag_address;
};


extern Imu_BMX055 Imu;

#endif