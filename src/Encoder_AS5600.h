#ifndef __ENCOER_AS5600__
#define __ENCOER_AS5600__


class TwoWire;


class Encoder_AS5600
{
    public:
        Encoder_AS5600(){}
        ~Encoder_AS5600(){}

        bool begin(TwoWire* wire, uint8_t address=0x36);

        uint16_t readAngle();
        uint16_t readRawAngle();

        uint16_t readStatus();

    private:
        TwoWire*    _wire;
        uint8_t     _address;
};

#endif