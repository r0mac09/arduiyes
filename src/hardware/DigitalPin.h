//
// Created by Alexandru Mihaescu on 26.05.2019.
//

#ifndef ARDUINO_DIGITALPIN_H
#define ARDUINO_DIGITALPIN_H

#include <string>

#include "Pin.h"
class DigitalPin : public Pin {
public:
    DigitalPin(std::string pinId);
    uint8_t read();
};


#endif //ARDUINO_DIGITALPIN_H
