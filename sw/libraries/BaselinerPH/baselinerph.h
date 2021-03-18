
#pragma once

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <EEPROM.h>

#define BASELINERPH_EEPROM_ADDR_NEUTRAL 1000
#define BASELINERPH_ADC_REFERENCE 2500.0
#define BASELINERPH_ADC_STEPS 1024.0
#define BASELINERPH_DIVIDER_RATIO 2.0

typedef struct 
{
    float voltage;
    float pH;
} phVal_t;

enum
{
    PH_NEUTRAL,
    PH_ACID,
    PH_BASE
};

class BaselinerPH
{

public:
    uint8_t _pinPH;
    phVal_t _phNeutral;
    phVal_t _phAcid;

public:
    BaselinerPH(uint8_t port=A5);
    void begin();
    float read();
    float readVoltage();
    void calibrate(float pH, float voltage);
    void loadCalib();
    phVal_t getCalib(uint8_t type);
};