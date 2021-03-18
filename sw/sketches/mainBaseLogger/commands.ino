/**************************************************************************/
// Temperature Command
/**************************************************************************/
void cmdTemp(int argc, char **args)
{
    soilTempSensor.requestTemperatures();
    float tempC = soilTempSensor.getTempCByIndex(0); 

    if(tempC != DEVICE_DISCONNECTED_C) 
    {
        printf("Temperature for device 0 is: %0.2f.\n", tempC);
    } 
    else
    {
        printf("Error: Could not read temperature data.\n");
    }    
}

/**************************************************************************/
// Soil Moisture Enable Command
/**************************************************************************/
void cmdSoilEnb(int argc, char **args)
{
    int enb = cmd.conv(args[1]);
    
    if (enb)
    {
        digitalWrite(pinSoilEnb, POWER_ON);
    }
    else
    {
        digitalWrite(pinSoilEnb, POWER_OFF);
    }
}

/**************************************************************************/
// Soil Moisture Command
/**************************************************************************/
void cmdSoil(int argc, char **args)
{
    // check to see that correct number of args entered
    if (argc != 2)
    {
        printf("Need to specify argument between 0 and 4 for soil index.\n");
    }

    int index = cmd.conv(args[1]); 

    // check to see correct indices entered and not overflowed
    if (index > 4)
    {
       printf("Argument out of range.\n"); 
       return;
    }

    // get soil moisture val
    int soilVal = analogRead(pinSoilSensor[index]);
    float voltage = soilVal * (ANALOGREFERENCE/ANALOGLEVELS) * 2;
    printf("ADC value: %d, Voltage: %f V.\n", soilVal, voltage);
}

/**************************************************************************/
// CO2 Measurement Command
/**************************************************************************/
void cmdCo2Enb(int argCnt, char **args)
{
    int enb = cmd.conv(args[1]);
    if (enb)
    {
        digitalWrite(pinCo2Enb, HIGH);
    }
    else
    {
        digitalWrite(pinCo2Enb, LOW);
    }
}

/**************************************************************************/
// CO2 Measurement Command
/**************************************************************************/
void cmdCo2(int argCnt, char **args)
{
    uint16_t co2 = getCO2();
    
    Serial.print("CO2 Concentration is ");
    Serial.print(co2);
    Serial.println("ppm");
}


/**************************************************************************/
// CO2 infinite loop testing
/**************************************************************************/
void cmdCo2Test(int argCnt, char **args)
{
    while(1)
    {
        if (co2.measure())
        {
            Serial.print("CO2 Concentration is ");
            Serial.print(co2.ppm);
            Serial.println("ppm");
            delay(300);
        }
    }
}

/**************************************************************************/
// Pressure, Temperature, Humidity (BME280) Command
/**************************************************************************/
void cmdPTH(int argCnt, char **args)
{
    if (bme.takeForcedMeasurement())
    {    
        float temperature = bme.readTemperature();
        float pressure = bme.readPressure() / 100.0;
        float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
        float humidity = bme.readHumidity();

        printf("Temperature = %f *C.\n", temperature);
        printf("Pressure = %f hPa.\n", pressure);
        printf("Approximate altitude = %f m.\n", altitude);
        printf("Humidity = %f %RH.\n", humidity);         
    }
    else
    {
        printf("Error taking BME280 measurement.\n");
    }
}

/**************************************************************************/
// Get PH value
/**************************************************************************/
void cmdPH(int argCnt, char **args)
{
    float phVal = ph.read();   
    float phVoltage = ph.readVoltage();

    printf("pH value is: %.2f, pH voltage is: %.2f.\n", phVal, phVoltage);
}

/**************************************************************************/
// Set PH value
/**************************************************************************/
void cmdSetPHCalib(int argCnt, char **args)
{
    double phVal, phVoltage;
    
    if (argCnt != 3)
    {    
        printf("This command requires three arguments.\n");
        return;
    }

    phVal = atof(args[1]);
    phVoltage = atof(args[2]);

    printf("PH Calibration constant - pH: %.2f, Voltage: %.2f.\n", phVal, phVoltage);
    ph.calibrate(phVal, phVoltage);    
}

/**************************************************************************/
// Get current pH calibration constants
/**************************************************************************/
void cmdGetPHCalib(int argCnt, char **args)
{
    phVal_t phVal;

    phVal = ph.getCalib(PH_NEUTRAL);
    printf("phNeutral: pH: %.2f, voltage: %.2f.\n", phVal.pH, phVal.voltage);

    phVal = ph.getCalib(PH_ACID);
    printf("phAcid: pH: %.2f, voltage: %.2f.\n", phVal.pH, phVal.voltage);
}

/**************************************************************************/
// Set RTC Time
/**************************************************************************/
void cmdSetTime(int argCnt, char **args)
{
    struct ts time;
    memset(&time, 0, sizeof(time));
    
    int year = cmd.conv(args[1]);
    int month = cmd.conv(args[2]);
    int day = cmd.conv(args[3]);
    int hour = cmd.conv(args[4]);
    int min = cmd.conv(args[5]);
    int sec = cmd.conv(args[6]);

    time.year = year;
    time.mon = month;
    time.mday = day;
    time.hour = hour;
    time.min = min;
    time.sec = sec;
    DS3231_set(time);

    printf("%04d/%02d/%02d,%02d:%02d:%02d", time.year, time.mon, time.mday, time.hour, time.min, time.sec);
}


/**************************************************************************/
// 
/**************************************************************************/
void cmdRadioSleep(int argCnt, char **args)
{
/*    
    int enb = cmd.conv(args[1]);
    
    if (enb)
    {
        rf95.sleep();       
    }
    else
    {
        rf95.setModeIdle();
    }
*/    
}


/**************************************************************************/
/*!
    Read regs
*/
/**************************************************************************/
void cmdMcuReadReg(int arg_cnt, char **args)
{
  unsigned char addr, val;

  addr = cmd.conv(args[1], 16);

  val = *(volatile unsigned char *)addr;
  printf("Addr %02X = %02X\n", addr, val);
}

/**************************************************************************/
/*!
    Write regs
*/
/**************************************************************************/
void cmdMcuWriteReg(int arg_cnt, char **args)
{
  unsigned char addr, val;

  addr = cmd.conv(args[1], 16);
  val = cmd.conv(args[2], 16);

  *(volatile unsigned char *)addr = val;

  val = *(volatile unsigned char *)addr;
  printf("Addr %02X = %02X.\n", addr, val);
}

/**************************************************************************/
// 
/**************************************************************************/
void cmdSleep(int argCnt, char **args)
{
    uint8_t portBReg, portCReg, portDReg;
    uint8_t ddrBReg, ddrCReg, ddrDReg;

    wdt_disable();
    LoRa.sleep();

    digitalWrite(pinSoilEnb, POWER_OFF);
    digitalWrite(pinCo2Enb, LOW);

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
/*
    // disable UARTs
    UCSR0B = 0x00;
    UCSR1B = 0x00;

    // Disable SPI device
    SPCR &= ~_BV(SPE);
    pinMode(11, OUTPUT);
    digitalWrite(11, LOW);

    delay(100);

    ADCSRA &= ~(1 << ADEN);    // Disable ADC

    // write sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    wdt_disable();      // disable watchdog while sleeping
    sleep_enable();     // setting up for sleep ...
    sleep_mode();
    // sleeping here
*/    
}

/**************************************************************************/
// Get RTC Time
/**************************************************************************/
void cmdGetTime(int argCnt, char **args)
{
  struct ts time = rtcGetTime();
  printf("%04d/%02d/%02d,%02d:%02d:%02d", time.year, time.mon, time.mday, time.hour, time.min, time.sec);
}

/**************************************************************************/
/*!
    Transmit data to another node wirelessly using Chibi stack.
    Usage: send <addr> <string...>
*/
/**************************************************************************/
void cmdSend(int arg_cnt, char **args)
{   
    char data[100];
    int len;
    
    len = strCat(data, 1, arg_cnt, args);
    LoRa.beginPacket();
    LoRa.print(data);
    LoRa.endPacket();
    LoRa.receive();   
}

/**************************************************************************/
/*!
    Call the logging function
*/
/**************************************************************************/
void cmdLog(int arg_cnt, char **args)
{   
    periodicHandler(); 
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdFileRead(int argCnt, char **args)
{
    readFile(args[1]);
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdFileLogRead(int argCnt, char **args)
{
   readFile(LOGFILE); 
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdFileDataRead(int argCnt, char **args)
{
   readFile(DATAFILE); 
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void cmdFileList(int argCnt, char **args)
{
    if (sdIsPresent)
    {    
        sd.ls(LS_DATE | LS_SIZE);
    }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void cmdFileDelete(int argCnt, char **args)
{
    if (sdIsPresent)
    {      
        sd.remove(args[1]);
        Serial.print(args[1]);
        Serial.println(" deleted.");
    }
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetID(int arg_cnt, char **args)
{
    char tmpBuf[50];
    
    getMeta(); 
    memset(meta.stationID, 0, sizeof(meta.stationID));
    strCat(tmpBuf, 1, arg_cnt, args);
    strncpy(meta.stationID, tmpBuf, META_FIELDSIZE_MAX-1); 
    putMeta();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetSite(int arg_cnt, char **args)
{
    char tmpBuf[50];
    
    getMeta();     
    memset(meta.stationSite, 0, sizeof(meta.stationSite));
    strCat(tmpBuf, 1, arg_cnt, args);
    strncpy(meta.stationSite, args[1], META_FIELDSIZE_MAX-1); 
    putMeta();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetNum(int arg_cnt, char **args)
{
    getMeta();  
    uint32_t num = cmd.conv(args[1]);
    meta.stationNum = num;
    putMeta();
}


/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetGateway(int arg_cnt, char **args)
{
    getMeta();  
    uint32_t num = cmd.conv(args[1]);
    meta.gatewayNum = num;
    putMeta();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetTxInterval(int arg_cnt, char **args)
{
    getMeta();  
    uint32_t num = cmd.conv(args[1]);
    meta.txInterval = num;
    putMeta();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetLoc(int arg_cnt, char **args)
{
    char tmpBuf[50];
    
    getMeta();  
    memset(meta.stationLoc, 0, sizeof(meta.stationLoc));
    strCat(tmpBuf, 1, arg_cnt, args);
    strncpy(meta.stationLoc, args[1], META_FIELDSIZE_MAX-1);     
    putMeta();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetVwc(int arg_cnt, char **args)
{
    getMeta(); 
    uint8_t index = cmd.conv(args[1]);
    switch(index)
    {
        case 0: 
            memset(meta.vwcSensorID_shallow, 0, sizeof(meta.vwcSensorID_shallow));
            strncpy(meta.vwcSensorID_shallow, args[2], META_FIELDSIZE_MAX-1);     
            break;
        case 1: 
            memset(meta.vwcSensorID_mid, 0, sizeof(meta.vwcSensorID_mid));
            strncpy(meta.vwcSensorID_mid, args[2], META_FIELDSIZE_MAX-1);                 
            break;
        case 2: 
            memset(meta.vwcSensorID_deep, 0, sizeof(meta.vwcSensorID_deep));
            strncpy(meta.vwcSensorID_deep, args[2], META_FIELDSIZE_MAX-1);     
            break;
        default: 
            Serial.println("Invalid value");
            break;
    }
    putMeta();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetPH(int arg_cnt, char **args)
{
    getMeta();  
    memset(meta.phSensorID, 0, sizeof(meta.phSensorID));
    strncpy(meta.phSensorID, args[1], META_FIELDSIZE_MAX-1);     
    putMeta();   
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetCo2(int arg_cnt, char **args)
{
    getMeta();  
    memset(meta.co2SensorID, 0, sizeof(meta.co2SensorID));
    strncpy(meta.co2SensorID, args[1], META_FIELDSIZE_MAX-1);    
    putMeta();     
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetTemp(int arg_cnt, char **args)
{
    getMeta(); 
    memset(meta.tempSensorID, 0, sizeof(meta.tempSensorID));
    strncpy(meta.tempSensorID, args[1], META_FIELDSIZE_MAX-1);    
    putMeta();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdSetBme(int arg_cnt, char **args)
{
    getMeta();  
    memset(meta.bmeSensorID, 0, sizeof(meta.bmeSensorID));
    strncpy(meta.bmeSensorID, args[1], META_FIELDSIZE_MAX-1);  
    putMeta();  
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdPutMeta(int arg_cnt, char **args)
{
    putMeta();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdGetMeta(int arg_cnt, char **args)
{
    getMeta();
    dumpMeta();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmdDispMeta(int arg_cnt, char **args)
{
    dumpMeta();
}

/********************************************************************/
//
//
//
/********************************************************************/
void cmdNormalModeEnb(int arg_cnt, char **args)
{
  normalMode = true;
  DS3231_clear_a1f();
  DS3231_clear_a2f();
}
