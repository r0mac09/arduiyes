//
// Created by Alexandru Mihaescu on 26.05.2019.
//

#ifndef ARDUINO_PIN_H
#define ARDUINO_PIN_H

#include <string>

#include "Wire.h"

class Wire;

class Pin {
private:
    std::string pinId; //Used for pin identification
    Wire* connectedWire;
public:
    Pin(std::string pinId);
    void connect(Wire* connectedWire); //Connect a wire to that specific pin
    uint16_t read();
    void write(uint16_t); // Function to write data
};


#endif //ARDUINO_PIN_H
