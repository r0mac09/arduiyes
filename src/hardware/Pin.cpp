//
// Created by Alexandru Mihaescu on 26.05.2019.
//

#include <string>

#include "Pin.h"
#include "Wire.h"

Pin::Pin(std::string pinId) {

}

void Pin::connect(Wire* connectedWire) {
    if(connectedWire != nullptr) this->connectedWire = connectedWire;
    else {}
};