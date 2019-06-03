//
// Created by Alexandru Mihaescu on 26.05.2019.
//

#include "Wire.h"

Wire::Wire() {

}

void Wire::connectFirst(Pin *firstEndpoint) {
    if(firstEndpoint == nullptr) {
        // ERROR HANDLING
    } else {
        this->firstEndpoint = firstEndpoint;
        firstEndpoint->connect(this);
    }
}

void Wire::connectSecond(Pin *secondEndpoint) {
    if(secondEndpoint == nullptr) {
        // ERROR HANDLING
    } else {
        this->secondEndpoint = secondEndpoint;
    }
}
