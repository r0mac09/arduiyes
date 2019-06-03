//
// Created by Alexandru Mihaescu on 26.05.2019.
//

#ifndef ARDUINO_ARDUINO_H
#define ARDUINO_ARDUINO_H

#include <cstdint>
#include <chrono>

class Arduino {
private:
    Arduino();
    static Arduino* instance;
    uint8_t analogPinsResolution;
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
public:
    static Arduino* arduino();
    std::chrono::time_point<std::chrono::high_resolution_clock> getStartTime();
};


#endif //ARDUINO_ARDUINO_H
