#include "ADC_Read.h"

ADC::ADC()
{
}

ADC::~ADC()
{
}

void ADC::begin(byte pin, byte bits_resolution, float reference_voltage)
{
    _pin = pin;
    pinMode(_pin,INPUT);
    _conversion = reference_voltage / (pow(2, bits_resolution)-1);
}

uint ADC::readRaw()
{
    return analogRead(_pin);
}

float ADC::readVoltage()
{
   return (readRaw() * _conversion);
}