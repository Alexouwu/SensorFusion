#ifndef _ADC_READ_H
#define _ADC_READ_H

#include <Arduino.h>

class ADC
{
public:
    ADC(); //Constructor
    ~ADC(); //Destructor
    void begin(byte pin, byte bits_resolution, float reference_voltage);
    uint readRaw();  //Lee val digital
    float readVoltage(); //Leer valor analogo

private:
    byte _pin;
    float _conversion;
};

#endif