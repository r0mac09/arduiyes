//
// Created by Alexandru Mihaescu on 29.05.2019.
//

#include "arduinoFunctions.h"

//
// Created by Alexandru Mihaescu on 29.05.2019.
//

// https://www.arduino.cc/reference/en/


#include <cstdint>
#include <chrono>
#include <thread>
#include <random>

#include "hardware/Arduino.h"

// TEMPORARY DEFINITIONS
#define LOW           0
#define HIGH          1
#define INPUT         2
#define INPUT_PULLUP  3
#define OUTPUT        4
#define MSBFIRST      5
#define LSBFIRST      6


// DIGITAL I/O


// Description
// Reads the value from a specified digital pin, either HIGH or LOW.

// Syntax
// digitalRead(pin)

// Parameters
// pin: the number of the digital pin you want to read

// Returns
// HIGH or LOW

// Notes and Warnings
// If the pin isn’t connected to anything, digitalRead() can return either HIGH or LOW (and this can change randomly).

// The analog input pins can be used as digital pins, referred to as A0, A1, etc.
// The exception is the Arduino Nano, Pro Mini, and Mini’s A6 and A7 pins, which can only be used as analog inputs.

uint8_t digitalRead(uint8_t pin) {
    // TO BE IMPLEMENTED

    return 0;
}


// Description
// Write a HIGH or a LOW value to a digital pin.

// If the pin has been configured as an OUTPUT with pinMode(), its voltage will be set to the
// corresponding value: 5V (or 3.3V on 3.3V boards) for HIGH, 0V (ground) for LOW.

// If the pin is configured as an INPUT, digitalWrite() will enable (HIGH) or disable (LOW)
// the internal pullup on the input pin. It is recommended to set the pinMode() to INPUT_PULLUP to enable the
// internal pull-up resistor. See the digital pins tutorial for more information.

// If you do not set the pinMode() to OUTPUT, and connect an LED to a pin, when calling digitalWrite(HIGH),
// the LED may appear dim. Without explicitly setting pinMode(), digitalWrite() will have enabled the internal
// pull-up resistor, which acts like a large current-limiting resistor.

// Syntax
// digitalWrite(pin, value)

// Parameters
// pin: the pin number

// value: HIGH or LOW

// Returns
// Nothing

// Notes and Warnings
// The analog input pins can be used as digital pins, referred to as A0, A1, etc.
// The exception is the Arduino Nano, Pro Mini, and Mini’s A6 and A7 pins, which can only be used as analog inputs.

void digitalWrite(uint8_t pin, uint8_t value) {
    // TO BE IMPLEMENTED
}


// Description
// Configures the specified pin to behave either as an input or an output.
// See the description of (digital pins) for details on the functionality of the pins.

// As of Arduino 1.0.1, it is possible to enable the internal pullup resistors with the
// mode INPUT_PULLUP. Additionally, the INPUT mode explicitly disables the internal pullups.

// Syntax
// pinMode(pin, mode)

// Parameters
// pin: the number of the pin whose mode you wish to set

// mode: INPUT, OUTPUT, or INPUT_PULLUP. (see the (digital pins) page for a more complete description of the functionality.)

// Returns
// Nothing

// Notes and Warnings
// The analog input pins can be used as digital pins, referred to as A0, A1, etc.

void pinMode(uint8_t pin, uint8_t mode) {
    // TO BE IMPLEMENTED
}


// ANALOG I/O


// Description
// Reads the value from the specified analog pin. Arduino boards contain a multichannel,
// 10-bit analog to digital converter. This means that it will map input voltages between 0 and
// the operating voltage(5V or 3.3V) into integer values between 0 and 1023. On an Arduino UNO,
// for example, this yields a resolution between readings of: 5 volts / 1024 units or, 0.0049 volts (4.9 mV)
// per unit. See the table below for the usable pins, operating voltage and maximum resolution for some Arduino boards.

// The input range can be changed using analogReference(), while the resolution can be changed
// (only for Zero, Due and MKR boards) using analogReadResolution().

// On ATmega based boards (UNO, Nano, Mini, Mega), it takes about 100 microseconds (0.0001 s) to
// read an analog input, so the maximum reading rate is about 10,000 times a second.

// BOARD	                   OPERATING VOLTAGE  USABLE PINS  MAX RESOLUTION
// Uno                        5 Volts            A0 to A5     10 bits
// Mini, Nano                 5 Volts            A0 to A7     10 bits
// Mega, Mega2560, MegaADK    5 Volts            A0 to A14    10 bits
// Micro                      5 Volts            A0 to A11*   10 bits
// Leonardo                   5 Volts            A0 to A11*   10 bits
// Zero                       3.3 Volts          A0 to A5     12 bits**
// Due                        3.3 Volts          A0 to A11    12 bits**
// MKR Family boards          3.3 Volts          A0 to A6     12 bits**

// *A0 through A5 are labelled on the board, A6 through A11 are respectively available on pins 4, 6, 8, 9, 10, and 12
// **The default analogRead() resolution for these boards is 10 bits, for compatibility.
// You need to use analogReadResolution() to change it to 12 bits.

// Syntax
// analogRead(pin)

// Parameters
// pin: the name of the analog input pin to read from (A0 to A5 on most boards, A0 to A6 on MKR boards,
// A0 to A7 on the Mini and Nano, A0 to A15 on the Mega).

// Returns
// The analog reading on the pin (int). Although it is limited to the resolution of the analog to
// digital converter (0-1023 for 10 bits or 0-4095 for 12 bits).

// Notes and Warnings
// If the analog input pin is not connected to anything, the value returned by analogRead() will
// fluctuate based on a number of factors (e.g. the values of the other analog inputs, how close your hand is to the board, etc.).

uint16_t analogRead(uint8_t pin) {
    // NOTE: IF > 1023 RETURN 1023(10 BIT VALUE)
    // TO BE IMPLEMENTED

    return 0;
}


// Description
// Configures the reference voltage used for analog input (i.e. the value used as the top of the input range). The options are:

// Arduino AVR Boards (Uno, Mega, etc.)

// DEFAULT: the default analog reference of 5 volts (on 5V Arduino boards) or 3.3 volts (on 3.3V Arduino boards)
// INTERNAL: an built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328P and 2.56 volts on the ATmega8 (not available on the Arduino Mega)
// INTERNAL1V1: a built-in 1.1V reference (Arduino Mega only)
// INTERNAL2V56: a built-in 2.56V reference (Arduino Mega only)
// EXTERNAL: the voltage applied to the AREF pin (0 to 5V only) is used as the reference.
// Arduino SAMD Boards (Zero, etc.)
// AR_DEFAULT: the default analog reference of 3.3V
// AR_INTERNAL: a built-in 2.23V reference
// AR_INTERNAL1V0: a built-in 1.0V reference
// AR_INTERNAL1V65: a built-in 1.65V reference
// AR_INTERNAL2V23: a built-in 2.23V reference
// AR_EXTERNAL: the voltage applied to the AREF pin is used as the reference
// Arduino SAM Boards (Due)
// AR_DEFAULT: the default analog reference of 3.3V. This is the only supported option for the Due.

// Syntax
// analogReference(type)

// Parameters
// type: which type of reference to use (see list of options in the description).

// Returns
// Nothing

// Notes and Warnings
// After changing the analog reference, the first few readings from analogRead() may not be accurate.

// Don’t use anything less than 0V or more than 5V for external reference voltage on the AREF pin!
// If you’re using an external reference on the AREF pin, you must set the analog reference to
// EXTERNAL before calling analogRead(). Otherwise, you will short together the active reference voltage (internally generated) and
// the AREF pin, possibly damaging the microcontroller on your Arduino board.

// Alternatively, you can connect the external reference voltage to the AREF pin through a 5K resistor,
// allowing you to switch between external and internal reference voltages. Note that the resistor will
// alter the voltage that gets used as the reference because there is an internal 32K resistor on the AREF pin.
// The two act as a voltage divider, so, for example, 2.5V applied through the resistor will yield 2.5 * 32 / (32 + 5) = ~2.2V at the AREF pin.

void analogReference(uint8_t type) {
    // SEE https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
    // TO BE IMPLEMENTED
}


// Description
// Writes an analog value (PWM wave) to a pin. Can be used to light a LED at varying brightnesses or drive a
// motor at various speeds. After a call to analogWrite(), the pin will generate a steady square wave of the
// specified duty cycle until the next call to analogWrite() (or a call to digitalRead() or digitalWrite()) on
// the same pin. The frequency of the PWM signal on most pins is approximately 490 Hz. On the Uno and similar boards,
// pins 5 and 6 have a frequency of approximately 980 Hz.

// On most Arduino boards (those with the ATmega168 or ATmega328P), this function works on pins 3, 5, 6, 9, 10, and 11.
// On the Arduino Mega, it works on pins 2 - 13 and 44 - 46. Older Arduino boards with an ATmega8 only
// support analogWrite() on pins 9, 10, and 11.
// The Arduino DUE supports analogWrite() on pins 2 through 13, plus pins DAC0 and DAC1. Unlike the PWM pins,
// DAC0 and DAC1 are Digital to Analog converters, and act as true analog outputs.
// You do not need to call pinMode() to set the pin as an output before calling analogWrite().
// The analogWrite function has nothing to do with the analog pins or the analogRead function.

// Syntax
// analogWrite(pin, value)

// Parameters
// pin: the pin to write to. Allowed data types: int.
// value: the duty cycle: between 0 (always off) and 255 (always on). Allowed data types: int

// Returns
// Nothing

// Notes and Warnings
// The PWM outputs generated on pins 5 and 6 will have higher-than-expected duty cycles.
// This is because of interactions with the millis() and delay() functions, which share the
// same internal timer used to generate those PWM outputs. This will be noticed mostly on
// low duty-cycle settings (e.g. 0 - 10) and may result in a value of 0 not fully turning off the output on pins 5 and 6.

void analogWrite(uint8_t pin, uint16_t value) {
    // TO BE IMPLEMENTED
}


// Zero, Due & MKR Family
// 12 bits analog pins

// Description
// analogReadResolution() is an extension of the Analog API for the Arduino Due, Zero and MKR Family.

// Sets the size (in bits) of the value returned by analogRead(). It defaults to 10 bits (returns values between 0-1023) for
// backward compatibility with AVR based boards.

// The Due, Zero and MKR Family boards have 12-bit ADC capabilities that can be accessed by changing the resolution to 12.
// This will return values from analogRead() between 0 and 4095.

// Syntax
// analogReadResolution(bits)

// Parameters
// bits: determines the resolution (in bits) of the value returned by the analogRead() function. You can set this between 1 and 32.
// You can set resolutions higher than 12 but values returned by analogRead() will suffer approximation. See the note below for details.

// Returns
// Nothing

// Notes and Warnings
// If you set the analogReadResolution() value to a value higher than your board’s capabilities,
// the Arduino will only report back at its highest resolution, padding the extra bits with zeros.

// For example: using the Due with analogReadResolution(16) will give you an approximated 16-bit number with
// the first 12 bits containing the real ADC reading and the last 4 bits padded with zeros.

// If you set the analogReadResolution() value to a value lower than your board’s capabilities,
// the extra least significant bits read from the ADC will be discarded.

// Using a 16 bit resolution (or any resolution higher than actual hardware capabilities)
// allows you to write sketches that automatically handle devices with a higher resolution ADC when
// these become available on future boards without changing a line of code.

void analogReadResolution(uint8_t bits) {
    // SEE https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogreadresolution/
    // TO BE IMPLEMENTED
}


// Description
// analogWriteResolution() is an extension of the Analog API for the Arduino Due.
// analogWriteResolution() sets the resolution of the analogWrite() function.
// It defaults to 8 bits (values between 0-255) for backward compatibility with AVR based boards.
// The Due has the following hardware capabilities:
// 12 pins which default to 8-bit PWM, like the AVR-based boards. These can be changed to 12-bit resolution.
// 2 pins with 12-bit DAC (Digital-to-Analog Converter)
// By setting the write resolution to 12, you can use analogWrite() with values between 0 and 4095 to exploit the
// full DAC resolution or to set the PWM signal without rolling over.
// The Zero has the following hardware capabilities:
// 10 pins which default to 8-bit PWM, like the AVR-based boards. These can be changed to 12-bit resolution.
// 1 pin with 10-bit DAC (Digital-to-Analog Converter).
// By setting the write resolution to 10, you can use analogWrite() with values between 0 and 1023 to exploit the full DAC resolution
// The MKR Family of boards has the following hardware capabilities:
// 4 pins which default to 8-bit PWM, like the AVR-based boards. These can be changed from 8 (default) to 12-bit resolution.
// 1 pin with 10-bit DAC (Digital-to-Analog Converter)
// By setting the write resolution to 12 bits, you can use analogWrite() with values between 0 and 4095 for PWM signals;
// set 10 bit on the DAC pin to exploit the full DAC resolution of 1024 values.

// Syntax
// analogWriteResolution(bits)

// Parameters
// bits: determines the resolution (in bits) of the values used in the analogWrite() function.
// The value can range from 1 to 32. If you choose a resolution higher or lower than your board’s hardware capabilities,
// the value used in analogWrite() will be either truncated if it’s too high or padded with zeros if it’s too low. See the note below for details.

// Returns
// Nothing

// Notes and Warnings
// If you set the analogWriteResolution() value to a value higher than your board’s capabilities,
// the Arduino will discard the extra bits. For example: using the Due with analogWriteResolution(16) on
// a 12-bit DAC pin, only the first 12 bits of the values passed to analogWrite() will be used and the last 4 bits will be discarded.
// If you set the analogWriteResolution() value to a value lower than your board’s capabilities,
// the missing bits will be padded with zeros to fill the hardware required size.
// For example: using the Due with analogWriteResolution(8) on a 12-bit DAC pin,
// the Arduino will add 4 zero bits to the 8-bit value used in analogWrite() to obtain the 12 bits required.

void analogWriteResolution(uint8_t bits) {
    // SEE https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogwriteresolution/
    // TO BE IMPLEMENTED
}


// ADVANCED I/0


// Description
// Stops the generation of a square wave triggered by tone(). Has no effect if no tone is being generated.

// Syntax
// noTone(pin)

// Parameters
// pin: the pin on which to stop generating the tone

// Returns
// Nothing

// Notes and Warnings
// If you want to play different pitches on multiple pins, you need to call noTone() on one pin before calling tone() on the next pin.

void noTone(uint8_t pin) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/notone/
    // TO BE IMPLEMENTED
}


// Description
// Reads a pulse (either HIGH or LOW) on a pin. For example, if value is HIGH,
// pulseIn() waits for the pin to go from LOW to HIGH, starts timing, then waits for the pin to go LOW and stops timing.
// Returns the length of the pulse in microseconds or gives up and returns 0 if no complete pulse was received within the timeout.

// The timing of this function has been determined empirically and will probably show errors in longer pulses.
// Works on pulses from 10 microseconds to 3 minutes in length.

// Syntax
// pulseIn(pin, value)

// pulseIn(pin, value, timeout)

// Parameters
// pin: the number of the pin on which you want to read the pulse. (int)
// value: type of pulse to read: either HIGH or LOW. (int)
// timeout (optional): the number of microseconds to wait for the pulse to start; default is one second (unsigned long)

// Returns
// the length of the pulse (in microseconds) or 0 if no pulse started before the timeout (unsigned long)

uint64_t pulseIn(uint8_t pin, uint8_t type, uint64_t timeout) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/pulsein/
    // TO BE IMPLEMENTED

    return 0;
}


// Description
// pulseInLong() is an alternative to pulseIn() which is better at handling long pulse and interrupt affected scenarios.
// Reads a pulse (either HIGH or LOW) on a pin. For example, if value is HIGH,
// pulseInLong() waits for the pin to go from LOW to HIGH, starts timing, then waits for the pin to go LOW and stops timing.
// Returns the length of the pulse in microseconds or gives up and returns 0 if no complete pulse was received within the timeout.
// The timing of this function has been determined empirically and will probably show errors in shorter pulses.
// Works on pulses from 10 microseconds to 3 minutes in length. This routine can be used only if interrupts are activated.
// Furthermore the highest resolution is obtained with large intervals.

// Syntax
// pulseInLong(pin, value)
// pulseInLong(pin, value, timeout)

// Parameters
// pin: the number of the pin on which you want to read the pulse. (int)
// value: type of pulse to read: either HIGH or LOW. (int)
// timeout (optional): the number of microseconds to wait for the pulse to start; default is one second (unsigned long)

// Returns
// the length of the pulse (in microseconds) or 0 if no pulse started before the timeout (unsigned long)

// Notes and Warnings
// This function relies on micros() so cannot be used in noInterrupts() context.

uint64_t pulseInLong(uint8_t pin, uint8_t value, uint64_t timeout) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/pulseinlong/
    // TO BE IMPLEMENTED

    return 0;
}


// Description
// Shifts in a byte of data one bit at a time. Starts from either the most (i.e. the leftmost) or least (rightmost) significant bit.
// For each bit, the clock pin is pulled high, the next bit is read from the data line, and then the clock pin is taken low.

// If you’re interfacing with a device that’s clocked by rising edges,
// you’ll need to make sure that the clock pin is low before the first call to shiftIn(), e.g. with a call to digitalWrite(clockPin, LOW).

// Note: this is a software implementation; Arduino also provides an SPI library that uses the hardware implementation,
// which is faster but only works on specific pins.

// Syntax
// byte incoming = shiftIn(dataPin, clockPin, bitOrder)

// Parameters
// dataPin: the pin on which to input each bit (int)
// clockPin: the pin to toggle to signal a read from dataPin
// bitOrder: which order to shift in the bits; either MSBFIRST or LSBFIRST. (Most Significant Bit First, or, Least Significant Bit First)

// Returns
// the value read (byte)

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/shiftin/
    // TO BE IMPLEMENTED

    return 0;
}


// Description
// Shifts out a byte of data one bit at a time. Starts from either the most (i.e. the leftmost) or least (rightmost) significant bit.
// Each bit is written in turn to a data pin, after which a clock pin is pulsed (taken high, then low) to indicate that the bit is available.
// Note- if you’re interfacing with a device that’s clocked by rising edges, you’ll need to make sure that the clock pin is low before the call to shiftOut(),
// e.g. with a call to digitalWrite(clockPin, LOW).
// This is a software implementation; see also the SPI library, which provides a hardware implementation that is faster but works only on specific pins.

// Syntax
// shiftOut(dataPin, clockPin, bitOrder, value)

// Parameters
// dataPin: the pin on which to output each bit (int)
// clockPin: the pin to toggle once the dataPin has been set to the correct value (int)
// bitOrder: which order to shift out the bits; either MSBFIRST or LSBFIRST. (Most Significant Bit First, or, Least Significant Bit First)
// value: the data to shift out. (byte)

// Returns
// Nothing

// Notes and Warnings
// The dataPin and clockPin must already be configured as outputs by a call to pinMode().
// shiftOut is currently written to output 1 byte (8 bits) so it requires a two step operation to output values larger than 255.

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/shiftout/
    // TO BE IMPLEMENTED
}


// Description
// Generates a square wave of the specified frequency (and 50% duty cycle) on a pin. A duration can be specified, otherwise the wave continues until a call to noTone(). The pin can be connected to a piezo buzzer or other speaker to play tones.
// Only one tone can be generated at a time. If a tone is already playing on a different pin, the call to tone() will have no effect. If the tone is playing on the same pin, the call will set its frequency.
// Use of the tone() function will interfere with PWM output on pins 3 and 11 (on boards other than the Mega).
// It is not possible to generate tones lower than 31Hz. For technical details, see Brett Hagman’s notes.

// Syntax
// tone(pin, frequency)
// tone(pin, frequency, duration)

// Parameters
// pin: the pin on which to generate the tone
// frequency: the frequency of the tone in hertz - unsigned int
// duration: the duration of the tone in milliseconds (optional) - unsigned long

// Returns
// Nothing

// Notes and Warnings
// If you want to play different pitches on multiple pins, you need to call noTone() on one pin before calling tone() on the next pin.

void tone(uint8_t pin, uint32_t frequency, uint64_t duration) {
    // SEE https://www.arduino.cc/reference/en/language/functions/advanced-io/tone/
    // TO BE IMPLEMENTED
}


// TIME


// Description
// Pauses the program for the amount of time (in milliseconds) specified as parameter. (There are 1000 milliseconds in a second.)

// Syntax
// delay(ms)

// Parameters
// ms: the number of milliseconds to pause (unsigned long)

// Returns
// Nothing

// Notes and Warnings
// While it is easy to create a blinking LED with the delay() function, and many sketches use short delays for such tasks as switch debouncing,
// the use of delay() in a sketch has significant drawbacks. No other reading of sensors, mathematical calculations,
// or pin manipulation can go on during the delay function, so in effect, it brings most other activity to a halt.
// For alternative approaches to controlling timing see the millis() function and the sketch sited below.
// More knowledgeable programmers usually avoid the use of delay() for timing of events longer than 10’s of milliseconds unless the Arduino sketch is very simple.

// Certain things do go on while the delay() function is controlling the Atmega chip however,
// because the delay function does not disable interrupts. Serial communication that appears at the RX pin is recorded,
// PWM (analogWrite) values and pin states are maintained, and interrupts will work as they should.

void delay(uint64_t ms) {
    // SEE https://www.arduino.cc/reference/en/language/functions/time/delay/

    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


// Description
// Pauses the program for the amount of time (in microseconds) specified as parameter.
// There are a thousand microseconds in a millisecond, and a million microseconds in a second.
// Currently, the largest value that will produce an accurate delay is 16383.
// This could change in future Arduino releases. For delays longer than a few thousand microseconds, you should use delay() instead.

// Syntax
// delayMicroseconds(us)

// Parameters
// us: the number of microseconds to pause (unsigned int)

// Returns
// Nothing

void delayMicroseconds(uint64_t us) {
    // SEE https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/

     std::this_thread::sleep_for(std::chrono::microseconds(us));
}


// Description
// Returns the number of microseconds since the Arduino board began running the current program. This number will overflow (go back to zero), after approximately 70 minutes. On 16 MHz Arduino boards (e.g. Duemilanove and Nano), this function has a resolution of four microseconds (i.e. the value returned is always a multiple of four). On 8 MHz Arduino boards (e.g. the LilyPad), this function has a resolution of eight microseconds.

// Syntax
// time = micros()

// Parameters
// Nothing

// Returns
// Returns the number of microseconds since the Arduino board began running the current program.(unsigned long)

// Notes and Warnings
// There are 1,000 microseconds in a millisecond and 1,000,000 microseconds in a second.

uint64_t micros(void) {
    // SEE https://www.arduino.cc/reference/en/language/functions/time/micros/

    auto now = std::chrono::high_resolution_clock::now();
    return uint64_t (std::chrono::duration_cast<std::chrono::microseconds>(now - Arduino::arduino()->getStartTime()).count());
}


// Description
// Returns the number of milliseconds passed since the Arduino board began running the current program. This number will overflow (go back to zero), after approximately 50 days.

// Syntax
// time = millis()

// Parameters
// None

// Returns
// Number of milliseconds passed since the program started (unsigned long)

// Notes and Warnings
// Please note that the return value for millis() is of type unsigned long, logic errors may occur if a programmer tries to do arithmetic with smaller data types such as int.
// Even signed long may encounter errors as its maximum value is half that of its unsigned counterpart.

uint64_t milis(void) {
    // SEE https://www.arduino.cc/reference/en/language/functions/time/millis/

    auto now = std::chrono::high_resolution_clock::now();
    return uint64_t (std::chrono::duration_cast<std::chrono::milliseconds>(now - Arduino::arduino()->getStartTime()).count());
}
