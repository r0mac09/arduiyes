//
// Created by Alexandru Mihaescu on 26.05.2019.
//

#ifndef ARDUINO_WIRE_H
#define ARDUINO_WIRE_H

#include "Pin.h"

class Pin;

class Wire {
private:
    Pin* firstEndpoint;
    Pin* secondEndpoint;
public:
    Wire();
    void connectFirst(Pin* firstEndpoint);
    void connectSecond(Pin* secondEndpoint);
};


#endif //ARDUINO_WIRE_H
