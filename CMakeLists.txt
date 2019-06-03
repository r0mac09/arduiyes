cmake_minimum_required(VERSION 3.13)
project(Arduino)

set(CMAKE_CXX_STANDARD 17)

add_executable(Arduino main.cpp src/hardware/Wire.cpp src/hardware/Wire.h src/hardware/Pin.cpp src/hardware/Pin.h src/hardware/Arduino.cpp src/hardware/Arduino.h src/hardware/definitions.h src/hardware/DigitalPin.cpp src/hardware/DigitalPin.h src/hardware/AnalogPin.cpp src/hardware/AnalogPin.h src/arduinoFunctions.cpp src/arduinoFunctions.h)