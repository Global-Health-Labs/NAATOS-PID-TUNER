
#ifndef TMP23X_H
#define TMP23X_H

#include "Arduino.h"


class TMP23X {
private:


public:
    TMP23X(); 
    uint8_t adc_input_pin;

    void set_analog_pin(uint8_t pin); 
    void set_adc_reference();
    uint16_t read_supply_voltage();
    float read_temperature_C(); 

};

#endif