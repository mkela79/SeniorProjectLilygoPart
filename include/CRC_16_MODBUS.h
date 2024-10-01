#ifndef CRC_16_MODBUS_H
#define CRC_16_MODBUS_H

#define TOF400F_BAUDRATE 115200

#include <Arduino.h>

class CRC_16_MODBUS {

    public:
        uint8_t reflectByte(uint8_t data);
        uint16_t reflectCRC16(uint16_t crc);
        uint16_t calculateCRC(uint8_t* data, uint8_t length);
        bool checkCRC(uint8_t*  data);
};

#endif