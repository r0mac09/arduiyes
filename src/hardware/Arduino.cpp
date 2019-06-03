//
// Created by Alexandru Mihaescu on 26.05.2019.
//

#include <chrono>

#include "Arduino.h"

Arduino* Arduino::instance = nullptr;

Arduino::Arduino() {
    analogPinsResolution = 10;
    startTime = std::chrono::high_resolution_clock::now();
}

Arduino* Arduino::arduino() {
    if(instance == nullptr) {
        instance = new Arduino();
    }

    return instance;
}

std::chrono::time_point<std::chrono::high_resolution_clock> Arduino::getStartTime() {
    return startTime;
}