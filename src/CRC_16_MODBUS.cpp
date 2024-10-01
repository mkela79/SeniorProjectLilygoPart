#include "CRC_16_MODBUS.h"

uint8_t CRC_16_MODBUS::reflectByte(uint8_t data) {
    uint8_t reflection = 0;
    
    for (int i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            reflection |= (1 << (7 - i));
        }
    }
    
    return reflection;
}

uint16_t CRC_16_MODBUS::reflectCRC16(uint16_t crc) {
    uint16_t reflection = 0;
    
    for (int i = 0; i < 16; i++) {
        if (crc & (1 << i)) {
            reflection |= (1 << (15 - i));  
        }
    }
    
    return reflection;
}

uint16_t CRC_16_MODBUS::calculateCRC(uint8_t* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for(uint8_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t) data[pos];

        for(uint8_t i = 8; i != 0; i--) {
            if((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool CRC_16_MODBUS::checkCRC(uint8_t*  data) {
    uint16_t receivedCRC = (data[6] << 8) | data[5];
    uint16_t calculatedCRC = calculateCRC(data, 5);
    return receivedCRC == calculatedCRC;
}
