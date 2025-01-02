


#include "TMP23x.h"

/*
ISSUE: This has ADC references embeded into external part libarary. Proper  class should decouple the two and have inheritance of ADC class
        I had previous firmware doing so but can't seem to find it.
*/

// default constructor
TMP23X::TMP23X(){}



void TMP23X::set_analog_pin(uint8_t pin){
    pinMode(pin, INPUT);
    TMP23X::adc_input_pin = pin;
}

// TODO: hardcoded to VDD, made configurable to internal references
void TMP23X::set_adc_reference(){
    analogReference(VDD);
    VREF.CTRLA = VREF_ADC0REFSEL_1V5_gc;
    // there is a settling time between when reference is turned on, and when it becomes valid.
    // since the reference is normally turned on only when it is requested, this virtually guarantees
    // that the first reading will be garbage; subsequent readings taken immediately after will be fine.
    // VREF.CTRLB|=VREF_ADC0REFEN_bm;
    // delay(10);
    uint16_t reading = analogRead(ADC_INTREF);
    reading = analogRead(ADC_INTREF);
}

uint16_t TMP23X::read_supply_voltage() { // returns value in millivolts to avoid floating point
 
  uint16_t reading = analogRead(ADC_INTREF);
  reading = analogRead(ADC_INTREF);
  uint32_t intermediate = 1023UL * 1500;
  reading = intermediate / reading;
  return reading;
}

float TMP23X::read_temperature_C() {           // member method
    // TODO: should be able to get analog resolution since it can change
    //int8_t adc_b = getAnalogReadResolution();
    // ATTINY 0/1 series is ONLY 10-bit
    if (int adc_value = analogRead(adc_input_pin)) {
        float millivolts = adc_value * (read_supply_voltage() / 1023.0);
        return (millivolts - 500) / 10;
    } else {
        // error
    }
}

