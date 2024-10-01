#include "ControllerNode.h"


ControllerNode::ControllerNode(unsigned long baudRateArg, SoftwareSerial* uartPort) {
    uart = uartPort;
    baudRate = baudRateArg;
}

void ControllerNode:: init() {
    uart->begin(baudRate);
}


uint16_t ControllerNode::waitForResponse(uint8_t mode, int timeout) { // TODO -intad aris gadasaketebeli
    unsigned long startTime = millis();
    char responseBuffer[16];  
    int index = 0;
    while (millis() - startTime < timeout) {
        if (uart->available()) {
            char c = uart->read();  
            if (c == '\n') {  
                responseBuffer[index] = '\0';  
                if (mode == 2) {
                    return atoi(responseBuffer);  
                } else {
                    // Serial.println(responseBuffer);
                }
                
            } else if (index < sizeof(responseBuffer) - 1) {
                responseBuffer[index++] = c;  
            }
        }
    }

    return 1;  
}


uint16_t ControllerNode::readSensorData(int index, int timeout) {
    uart->print("get_");
    uart->println(index);
    return waitForResponse(2, timeout);
}


uint16_t ControllerNode::moveStepperUp() {
    uart->println("up");
    return waitForResponse(3, 8000);
}

uint16_t ControllerNode::moveStepperDown() {
    uart->println("down");
    return waitForResponse(4, 8000);
}

uint16_t ControllerNode::measureHumidity() {
    Serial.println("Stepper Down");
    moveStepperDown();
    delay(1000);
    uint16_t humidity = readSensorData(5, 1000);
    Serial.print("Soil moisture: ");
    Serial.println(humidity/10.0);
    Serial.println("Stepper UP");
    moveStepperUp();
    return humidity;
}
