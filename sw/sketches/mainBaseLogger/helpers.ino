/********************************************************************/
//  DevMode - allows system to go into developer mode rather than normal operating mode
/********************************************************************/
void selectMode()
{   
    uint32_t now = millis();;
     
    Serial.println("Press 'c' to go into command mode");
    while ((millis() - now) < START_WAIT_TIME)
    {
      if (Serial.available() > 0)
      {
        if (Serial.read() == 'c')
        {
            normalMode = false;
            Serial.println("Type 'normal' to go back to normal mode");
            Serial.println("Command line mode times out in 5 minutes");
            cmdModeTimeCnt = millis();
            return;
        }
      }
    }
    normalMode = true;
    Serial.println("Going into normal operation mode.");
    Serial.flush();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void receivePacket(uint8_t pktLen, pktHdr_t *hdr, char *payload)
{
    uint8_t payloadSize = pktLen - sizeof(pktHdr_t);
    char *phdr = (char *)hdr;
    int16_t rssi = LoRa.packetRssi();

    // read out header
    for (int i=0; i<sizeof(pktHdr_t); i++)
    {
        *phdr++ = LoRa.read();
    }
    
    for (int i=0; i<payloadSize; i++)
    {
        *payload++ = LoRa.read();
        //Serial.print(buf[i]);
    }
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void logMsg(char *msg)
{
    char temp[300];
    struct ts time = rtcGetTime();
    
    memset(temp, 0, sizeof(temp));
    sprintf(temp, "%04d/%02d/%02d,%02d:%02d:%02d: %s", time.year, time.mon, time.mday, time.hour, time.min, time.sec, msg);
    printf(temp);

#if (LOG_MESSAGES == 1)     
    if (sdIsPresent)
    {
        myFile = sd.open(LOGFILE, O_RDWR | O_CREAT | O_APPEND);
        if (myFile)
        {
            myFile.print(temp); 
            myFile.close();
        }
        else
        {
            Serial.println("Error opening file for writing.");
        } 
    }
#endif // LOG_MESSAGES   
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void logData(char *msg)
{
    char temp[300];
    struct ts time = rtcGetTime();

#if (LOG_DATA == 1)    
    memset(temp, 0, sizeof(temp));
    sprintf(temp, "%04d/%02d/%02d,%02d:%02d:%02d,%s", time.year, time.mon, time.mday, time.hour, time.min, time.sec, msg);
    if (sdIsPresent)
    {
        myFile = sd.open(DATAFILE, O_RDWR | O_CREAT | O_APPEND);
        if (myFile)
        {
            myFile.print(temp); 
            myFile.close();
        }
        else
        {
            Serial.println("Error opening file for writing.");
        } 
        
    }
#endif // LOG_DATA        
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void readFile(char *filename)
{
    if (sdIsPresent)
    {
        myFile = sd.open(filename, O_READ);
        if (myFile == true)
        {
            while(myFile.available())
            {
                wdt_reset();
                Serial.write(myFile.read());
            }       
            myFile.close();
        }
        else
        {
            Serial.println("Error opening file for reading.");
        }    
    }
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void getMeta()
{  
    EEPROM.get(METADATA_EEPROM_LOC, meta);
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void putMeta()
{  
    EEPROM.put(METADATA_EEPROM_LOC, meta);
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void dumpMeta()
{   
    phVal_t phVal;
    printf("Station ID:     \t%s\n", meta.stationID);
    printf("Station Site:   \t%s\n", meta.stationSite);
    printf("Station Number: \t%d\n", meta.stationNum);
    printf("Gateway Number: \t%d\n", meta.gatewayNum);
    printf("Transmit Interval: \t%d\n", meta.txInterval);
    printf("Station Location: \t%s\n", meta.stationLoc);
    printf("VWC Sensor ID Shallow: \t%s\n", meta.vwcSensorID_shallow);
    printf("VWC Sensor ID Mid: \t%s\n", meta.vwcSensorID_mid);
    printf("VWC Sensor ID Deep: \t%s\n", meta.vwcSensorID_deep);
    printf("pH Sensor ID: \t\t%s\n", meta.phSensorID);
    printf("CO2 Sensor ID: \t\t%s\n", meta.co2SensorID);
    printf("Temp Sensor ID: \t%s\n", meta.tempSensorID);
    printf("BME280 Sensor ID: \t%s\n", meta.bmeSensorID);

    phVal = ph.getCalib(PH_NEUTRAL);
    printf("pH Cal Neutral: pH: \t%.2f\n", phVal.pH);
    printf("pH Cal Neutral: Volt: \t%0.0f mV\n", phVal.voltage);

    phVal = ph.getCalib(PH_ACID);
    printf("pH Cal Acid: pH: \t%.2f\n", phVal.pH);
    printf("pH Cal Acid: Volt: \t%0.0f mV\n", phVal.voltage);    
}

/**************************************************************************/
/*!
    Concatenate multiple strings from the command line starting from the
    given index into one long string separated by spaces.
*/
/**************************************************************************/
int strCat(char *buf, unsigned char index, char arg_cnt, char **args)
{
  uint8_t i, len;
  char *data_ptr;

  data_ptr = buf;
  for (i = 0; i < arg_cnt - index; i++)
  {
    len = strlen(args[i + index]);
    strcpy((char *)data_ptr, (char *)args[i + index]);
    data_ptr += len;
    *data_ptr++ = ' ';
  }
  // remove the trailing space
  data_ptr--;
  *data_ptr++ = '\0';

  return data_ptr - buf;
}

/**************************************************************************/
// Helper function to get time
/**************************************************************************/
struct ts rtcGetTime()
{
    struct ts time;
    DS3231_get(&time);
    return time;
}

/**************************************************************************/
// 
/**************************************************************************/
float getSoilMoisture(uint8_t index)
{
    // get soil moisture val
    int soilVal = analogRead(pinSoilSensor[index]);
    float voltage = soilVal * (ANALOGREFERENCE/ANALOGLEVELS) * 2;
    return voltage;
}

/**************************************************************************/
// 
/**************************************************************************/
float getBattery()
{
    int rawAdc = analogRead(pinVbat);
    float voltage = rawAdc * (ANALOGREFERENCE/ANALOGLEVELS) * 2;
    return voltage;
}

/**************************************************************************/
// 
/**************************************************************************/
float getSolar()
{
    int rawAdc = analogRead(pinVSol);
    float voltage = rawAdc * (ANALOGREFERENCE/ANALOGLEVELS) * 2;
    return voltage;
}

/**************************************************************************/
// 
/**************************************************************************/
float getSoilTemperature()
{
    soilTempSensor.requestTemperatures();
    float tempC = soilTempSensor.getTempCByIndex(0); 
    
    if(tempC != DEVICE_DISCONNECTED_C) 
    {
        return tempC;
    } 
    else
    {
        return SOIL_TEMP_ERROR_VAL;
    } 
}

/**************************************************************************/
// 
/**************************************************************************/
uint16_t getCO2()
{   
    bool co2good = false;
    
    // initialize co2 sensor
    for (int i=0; i<CO2_NUM_RETRIES; i++)
    {
        wdt_reset();
        co2good = co2.begin();
        if (co2good)
        {
            //printf("CO2 initialized.\n");
            break;
        }
        delay(1000);
    }

    // something wrong with sensor. return error value
    if (!co2good) 
    {
        printf("Could not initialize CO2 sensor.\n");
        return 0;
    }

    // delay to warm up co2 sensor
    for (int i=0; i<CO2_POWER_ON_DELAY; i++)
    {
        wdt_reset();
        delay(1000);
    }

    // perform measurement
    if (co2.measure())
    {
        return co2.ppm;
    }

    // should not reach here. If we do, return error value
    return 0;
}

/**************************************************************************/
// 
/**************************************************************************/
void sdDateTime(uint16_t *date, uint16_t *time)
{    
    struct ts timeStruct = rtcGetTime();

    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE(timeStruct.year, timeStruct.mon, timeStruct.mday);

    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME(timeStruct.hour, timeStruct.min, timeStruct.sec);
}

/************************************************************************/
// elapsedTime - calculates time elapsed from startTime
// startTime : time to start calculating
/************************************************************************/
uint32_t elapsedTime(uint32_t startTime)
{
  uint32_t stopTime = millis();
  
  if (stopTime >= startTime)
  {
    return stopTime - startTime;
  }
  else
  {
    return (ULONG_MAX - (startTime - stopTime));
  }
}

/**************************************************************************/
// This is to implement the printf function from within arduino
/**************************************************************************/
static int uart_putchar (char c, FILE *stream)
{
    Serial.write(c);
    return 0;
}
