#ifndef CONTROLLERNODE_H
#define CONTROLLERNODE_H

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "CRC_16_MODBUS.h"

class ControllerNode {
    private:
        SoftwareSerial* uart;
        unsigned long baudRate;
        CRC_16_MODBUS crcCheck;
        
        uint16_t waitForResponse(uint8_t mode, int timeout);

    public:
        ControllerNode(unsigned long baudRate, SoftwareSerial* uart);
        void init();
        uint16_t readSensorData(int index, int timeOut);
        uint16_t moveStepperUp();
        uint16_t moveStepperDown();
        uint16_t measureHumidity();

};

#endif


