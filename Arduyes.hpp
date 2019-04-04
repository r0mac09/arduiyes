// https://www.arduino.cc/reference/en/

#ifndef ARDUYES_HPP
#define ARDUYES_HPP

#include <stdint.h>


// TEMPORARY DEFINITIONS
#define LOW           0
#define HIGH          1
#define INPUT         2
#define INPUT_PULLUP  3
#define OUTPUT        4
#define MSBFIRST      5
#define LSBFIRST      6

// DIGITAL I/O

uint8_t digitalRead(uint8_t pin) {
    // TO BE IMPLEMENTED
}

void digitalWrite(uint8_t pin, uint8_t value) {
    // TO BE IMPLEMENTED
}

void pinMode(uint8_t pin, uint8_t mode) {
    // TO BE IMPLEMENTED
}


// ANALOG I/O

uint16_t analogRead(uint8_t pin) {
    // NOTE: IF > 1023 RETURN 1023(10 BIT VALUE)
    // TO BE IMPLEMENTED
}

void analogReference(uint8_t type) {
    // SEE https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
    // TO BE IMPLEMENTED
}

void analogWrite(uint8_t pin, uint16_t value) {
    // TO BE IMPLEMENTED
}

// Zero, Due & MKR Family
// 12 bits analog pins

void analogReadResolution(uint8_t bits) {
    // SEE https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogreadresolution/
    // TO BE IMPLEMENTED
}

void analogWriteResolution(uint8_t bits) {
    // SEE https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogwriteresolution/
    // TO BE IMPLEMENTED
}


// ADVANCED I/0

void noTone(uint8_t pin) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/notone/
    // TO BE IMPLEMENTED
}

uint64_t pulseIn(uint8_t pin, uint8_t type, uint64_t timeout = 0) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/pulsein/
    // TO BE IMPLEMENTED
}

uint64_t pulseInLong(uint8_t pin, uint8_t value, uint64_t timeout = 0) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/pulseinlong/
    // TO BE IMPLEMENTED
}

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/shiftin/
    // TO BE IMPLEMENTED
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/shiftout/
    // TO BE IMPLEMENTED
}

void tone(uint8_t pin, uint32_t frequency, uint64_t duration = 0) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/tone/
    // TO BE IMPLEMENTED
}


// TIME

void delay(uint64_t ms) {
    // SEE https://www.arduino.cc/reference/en/language/functions/time/delay/
    // TO BE IMPLEMENTED
}


void delayMicroseconds(uint32_t us) {
    // SEE https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/
    // TO BE IMPLEMENTED
}

uint64_t micros(void) {
    // SEE https://www.arduino.cc/reference/en/language/functions/time/micros/
    // TO BE IMPLEMENTED
}

uint64_t milis(void) {
    // SEE https://www.arduino.cc/reference/en/language/functions/time/millis/
    // TO BE IMPLEMENTED
}

#endif