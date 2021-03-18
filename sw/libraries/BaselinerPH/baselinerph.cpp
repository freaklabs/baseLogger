#include "baselinerph.h"

#define PH_NEUTRAL_DEFAULT 7.0
#define PH_NEUTRAL_VOLTAGE_DEFAULT 1500.0

#define PH_ACID_DEFAULT 4.0
#define PH_ACID_VOLTAGE_DEFAULT 2032.4


/************************************************************************/
// 
//
//
/************************************************************************/
BaselinerPH::BaselinerPH(uint8_t pinPH)
{
    if (pinPH < A0)
    {
        Serial.println("PORT ERROR: Port cannot be less than A0");
        _pinPH = A5;
    }
    else
    {
        _pinPH = pinPH;
    }
}

/************************************************************************/
// 
//
//
/************************************************************************/
void BaselinerPH::begin()
{
    memset((uint8_t *)&_phNeutral, 0, sizeof(_phNeutral));
    memset((uint8_t *)&_phAcid, 0, sizeof(_phAcid));

    analogReference(EXTERNAL);

    // check EEPROM calibration constants. If blank, add starting values
    loadCalib();
}

/************************************************************************/
// 
//
//
/************************************************************************/
float BaselinerPH::read()
{
    float pH;
    float voltage = readVoltage();
    float m = (_phNeutral.pH - _phAcid.pH) / (((_phNeutral.voltage - 1500.0)/3.0) - (((_phAcid.voltage - 1500.0)/3.0)));
    float b = _phNeutral.pH - (m * ((_phNeutral.voltage - 1500.0)/3.0));
    pH = m * ((voltage - PH_NEUTRAL_VOLTAGE_DEFAULT)/3.0) + b;
    //printf("Slope = %f, Intercept = %f, Voltage: %f, pH = %f.\n", m, b, voltage, pH);
    return pH;
}

/************************************************************************/
// 
//
//
/************************************************************************/
float BaselinerPH::readVoltage()
{
    uint16_t rawAdc = analogRead(_pinPH);
    float voltage = (rawAdc * (BASELINERPH_ADC_REFERENCE/BASELINERPH_ADC_STEPS)) * BASELINERPH_DIVIDER_RATIO;
    //printf("RawADC: %d, Voltage: %.4f.\n", rawAdc, voltage);
    return voltage;
}

/************************************************************************/
// 
//
//
/************************************************************************/
void BaselinerPH::calibrate(float ph, float voltage)
{
    uint16_t baseAddr;

    if (ph < 6.0)
    {
        _phAcid.voltage = voltage;
        _phAcid.pH = ph;

        // set acid eeprom address
        baseAddr = BASELINERPH_EEPROM_ADDR_NEUTRAL + sizeof(phVal_t);
        EEPROM.put(baseAddr, _phAcid);
        printf("Saving calibration for acid.\n");
    }
    else if (ph >= 6.0 && ph <7.5)
    {
        _phNeutral.voltage = voltage;
        _phNeutral.pH = ph; 

        // set neutral eeprom address
        baseAddr = BASELINERPH_EEPROM_ADDR_NEUTRAL;
        EEPROM.put(baseAddr, _phNeutral);
        printf("Saving calibration for neutral.\n");
    }
    else
    {
        Serial.println("ERROR: We can't handle base calibration yet.");
    }
}

/************************************************************************/
// 
//
//
/************************************************************************/
void BaselinerPH::loadCalib()
{
    uint16_t baseAddr;
    phVal_t ph;

    // get ph neutral from eeprom
    baseAddr = BASELINERPH_EEPROM_ADDR_NEUTRAL;
    EEPROM.get(baseAddr, ph);

    // if no calibration values are loaded, add default values
    if (isnan(ph.voltage))
    {
        //Serial.println("phNeutral is uninitialized. Using default values.\n");
        _phNeutral.pH = PH_NEUTRAL_DEFAULT;
        _phNeutral.voltage = PH_NEUTRAL_VOLTAGE_DEFAULT;
    } 
    else
    {
        //printf("phNeutral is initialized.\n");
        memcpy((uint8_t *)&_phNeutral, (uint8_t *)&ph, sizeof(phVal_t));
    }   

    baseAddr = BASELINERPH_EEPROM_ADDR_NEUTRAL + sizeof(phVal_t);
    EEPROM.get(baseAddr, ph);
    
    if  (isnan(ph.voltage))
    {
        //Serial.println("phAcid is uninitialized. Using default values.\n");
        _phAcid.pH = PH_ACID_DEFAULT;
        _phAcid.voltage = PH_ACID_VOLTAGE_DEFAULT;        
    }
    else
    {
        //printf("phAcid is initialized.\n");
        memcpy((uint8_t *)&_phAcid, (uint8_t *)&ph, sizeof(phVal_t));
    }

//    printf("_phNeutral.pH: %.2f, _phNeutral.voltage: %.2f.\n", _phNeutral.pH, _phNeutral.voltage);
//    printf("_phAcid.pH: %.2f, _phAcid.voltage: %.2f.\n", _phAcid.pH, _phAcid.voltage);
  
}

/************************************************************************/
// 
//
//
/************************************************************************/
phVal_t BaselinerPH::getCalib(uint8_t type)
{
    phVal_t ph;
    memset((uint8_t *)&ph, 0, sizeof(phVal_t));

    switch (type)
    {
        case PH_NEUTRAL:
            return _phNeutral;

        case PH_ACID:
            return _phAcid;

        default:
            printf("ERROR: Not supported");
            return ph;
    }
}