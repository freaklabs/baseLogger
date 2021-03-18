#include <cmdArduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ds3231.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SdFat.h>
#include <LowPower.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <NDIR_SoftwareSerial.h>
#include <baselinerph.h>
#include <avr/wdt.h>
#include <limits.h>

// define what we log to SD card
#define LOG_DATA 1
#define LOG_MESSAGES 0

// set this for each release
#define VERSION "0.50"

#define DATAFILE "DATAFILE.TXT"
#define LOGFILE "LOGFILE.TXT"
#define ANALOGREFERENCE 2.5
#define ANALOGLEVELS 1024
#define NUM_ANALOG_PORTS 6
#define NUM_SOIL_SENSORS 3
#define NUM_AUX_LEDS 2
#define BME280_ADDR 0x76
#define SEALEVELPRESSURE_HPA (1013.25)
#define POWER_OFF HIGH
#define POWER_ON LOW
#define SOIL_TEMP_ERROR_VAL -99.9
//#define CO2_POWER_ON_DELAY 10
#define CO2_POWER_ON_DELAY 1
#define CO2_NUM_RETRIES 5
#define MAX_PAYLOAD_SIZE 140
#define META_FIELDSIZE_MAX 20
#define METADATA_EEPROM_LOC 0
#define START_WAIT_TIME 2000
#define RH95_SERVER_ADDRESS 0x0002
#define RF95_FREQ 915E6
#define RF95_POWER 20
#define RF95_SPREADING_FACTOR 10
#define RF95_BANDWIDTH 125E3
#define MAX_TRANSMIT_RETRIES 5
#define ACK_WAIT_TIMEOUT 2000
#define ONE_MINUTE 60000
#define CMD_MODE_TIME_LIMIT 5

enum
{
    PKT_DATA = 0,
    PKT_ACK = 1,
    PKT_CTRL = 2
};

typedef struct
{
    uint8_t seq;
    uint16_t destAddr;
    uint16_t srcAddr;
    uint8_t type;
} pktHdr_t;

typedef struct
{
    uint8_t retries;
    uint32_t failedTx;
} stats_t;

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;    
    float soilMoisture[NUM_SOIL_SENSORS];
    float soilTemp;
    float ph;
    float phVoltage;
    float bmeTemp;
    float bmePressure;
    float bmeHumidity;
    float bmeAltitude;
    uint16_t co2PPM;
    float battVal;
    float solarVal;
    float internalTemp;
} data_t;

typedef struct
{
    char stationID[META_FIELDSIZE_MAX];
    char stationSite[META_FIELDSIZE_MAX];
    uint16_t stationNum;
    uint16_t gatewayNum;
    uint16_t txInterval;
    char stationLoc[META_FIELDSIZE_MAX];
    char vwcSensorID_shallow[META_FIELDSIZE_MAX];
    char vwcSensorID_mid[META_FIELDSIZE_MAX];
    char vwcSensorID_deep[META_FIELDSIZE_MAX];
    char phSensorID[META_FIELDSIZE_MAX];
    char co2SensorID[META_FIELDSIZE_MAX];
    char tempSensorID[META_FIELDSIZE_MAX];
    char bmeSensorID[META_FIELDSIZE_MAX];
} baselinerMeta_t;

baselinerMeta_t meta;
stats_t stats;
pktHdr_t hdr;

// pin definitions
int pinSoilTemp = 4;
int pinSoilEnb = 7;
int pinCo2Enb = 20;
int pinRadioSelN = 21;
int pinRadioIntp = 6;
int pinSdSelN = 10;
int pinSoilSensor[NUM_SOIL_SENSORS] = {A0, A3, A4};
int pinPH = A5;
int pinVbat = A6;
int pinVSol = A7;
int pinCo2Tx = 9;
int pinCo2Rx = 8;   
int pinAuxLed[2] = {18, 19};
int pinRadioReset = 8;

// interrupt number definitions
int IntpRtc = 0;
int IntpRadio = 2;  

volatile bool flagRtc = false;
volatile bool flagRadio = false;

// object definitions
OneWire oneWire(pinSoilTemp);
DallasTemperature soilTempSensor(&oneWire);
Adafruit_BME280 bme;
SdFat sd;
File myFile;
NDIR_SoftwareSerial co2(pinCo2Rx, pinCo2Tx);
BaselinerPH ph;

static char tmpBuf[500];
static FILE uartout = {0};

// for rtc
uint8_t minuteFlags[5] = {0, 1, 1, 1, 1};
uint8_t hourFlags[4] = {0, 1, 1, 1};

bool bmeIsPresent = false;
bool sdIsPresent = false;

// lora params
uint8_t pktLen;
data_t data;
bool normalMode = true;
uint8_t timeCount = 0;
uint32_t cmdModeTimeCnt;
uint32_t cmdModeTimeLimit = CMD_MODE_TIME_LIMIT * ONE_MINUTE;

/**************************************************************************/
// 
/**************************************************************************/
void setup() 
{
    struct ts time;
    
    // disable watchdog here
    wdt_disable();
    
    // to set up the printf command. it will make life a lot easier
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &uartout ;

    // init  stats
    memset(&stats, 0, sizeof(stats_t));

    // init pins  
    pinMode(pinSoilEnb, OUTPUT);
    digitalWrite(pinSoilEnb, POWER_ON);

    pinMode(pinCo2Enb, OUTPUT);
    digitalWrite(pinCo2Enb, HIGH);    

    pinMode(pinRadioSelN, OUTPUT);
    digitalWrite(pinRadioSelN, HIGH); 

    pinMode(pinSdSelN, OUTPUT);
    digitalWrite(pinSdSelN, HIGH);    
    
    // init auxiliar LEDs
    for (int i=0; i<NUM_AUX_LEDS; i++)
    {
        pinMode(pinAuxLed[i], OUTPUT);
        digitalWrite(pinAuxLed[i], LOW);
    }

    // use external voltage reference for ADC
    analogReference(EXTERNAL);

    // init soil temperature sensor DS18B20
    soilTempSensor.begin();

    getMeta();
    cmd.begin(57600);
    printf("BaseLogger Soil Health Data Logger Application\n");
    Serial.println(VERSION); 

    // init packet header
    hdr.seq = 0;
    hdr.srcAddr = meta.stationNum;
    hdr.destAddr = meta.gatewayNum;
    hdr.type = PKT_DATA;    

    // init DS3231 RTC
    Wire.begin();    
    DS3231_init(DS3231_INTCN);

    // set minute alarm timer
    DS3231_clear_a1f();
    DS3231_set_a1(0, 0, 0, 0, minuteFlags);    
    DS3231_set_creg(DS3231_INTCN | DS3231_A1IE);  

    // init interrupt service routines
    attachInterrupt(IntpRtc, isrRtc, FALLING);

    // init BME280 pressure, temp, humidity sensor
    bmeIsPresent = bme.begin(BME280_ADDR);
    if (bmeIsPresent)
    {
        bme.setSampling(bme.MODE_FORCED);
    }

    // initialize the pH sensor
    ph.begin();

    sdIsPresent = sd.begin(pinSdSelN);

    // init LoRa module
    LoRa.setPins(pinRadioSelN, pinRadioReset, pinRadioIntp);
    if (!LoRa.begin(RF95_FREQ)) 
    {
        LoRa.end();
        while (1) 
        { 
            digitalWrite(pinAuxLed[1], HIGH); 
            delay(500);
            digitalWrite(pinAuxLed[1], LOW); 
            delay(500);
            //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
        }
    }
    LoRa.setTxPower(RF95_POWER);  
    LoRa.setSpreadingFactor(RF95_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(RF95_BANDWIDTH);  
    LoRa.onReceive(onReceive);
    LoRa.enableCrc();
    LoRa.receive();

    // set up the file system callback so that the file can be date and timestamped
    if (sdIsPresent)
    {
       myFile.dateTimeCallback(sdDateTime);  
    }

    // enable watchdog timer
    wdt_enable(WDTO_8S);   
             
    // command table
    cmd.add("temp", cmdTemp);
    cmd.add("soil", cmdSoil);
    cmd.add("pth", cmdPTH);
    cmd.add("settime", cmdSetTime);
    cmd.add("gettime", cmdGetTime);
    cmd.add("readph", cmdPH);
    cmd.add("getcalph",cmdGetPHCalib);
    cmd.add("setcalph", cmdSetPHCalib);
    cmd.add("sleep", cmdSleep);
    cmd.add("radiosleep", cmdRadioSleep);
    cmd.add("rd", cmdMcuReadReg);
    cmd.add("wr", cmdMcuWriteReg);
    cmd.add("soilenb", cmdSoilEnb);
    cmd.add("send", cmdSend);
    cmd.add("co2", cmdCo2);
    cmd.add("co2enb", cmdCo2Enb);
    cmd.add("co2test", cmdCo2Test);
    cmd.add("normal", cmdNormalModeEnb);
    cmd.add("log", cmdLog);
    cmd.add("fread", cmdFileRead);
    cmd.add("logread", cmdFileLogRead);
    cmd.add("dataread", cmdFileDataRead);
    cmd.add("ls", cmdFileList);
    cmd.add("rm", cmdFileDelete);

    // for setting metadata
    cmd.add("setid", cmdSetID);
    cmd.add("setsite", cmdSetSite);
    cmd.add("setnum", cmdSetNum);
    cmd.add("setgw", cmdSetGateway);
    cmd.add("setinterval", cmdSetTxInterval);
    cmd.add("setloc", cmdSetLoc);
    cmd.add("setvwc", cmdSetVwc);
    cmd.add("setph", cmdSetPH);
    cmd.add("setco2", cmdSetCo2);
    cmd.add("settemp", cmdSetTemp);
    cmd.add("setbme", cmdSetBme);
    cmd.add("putmeta", cmdPutMeta);
    cmd.add("getmeta", cmdGetMeta);
    cmd.add("dispmeta", cmdDispMeta);

    // seed random number generator with current time seconds
    time = rtcGetTime();
    randomSeed(time.sec);

    // set sequence ID to random number on start
    hdr.seq = random(255);

    // indicate that all systems are go
    for (int i=0; i<3; i++)
    {
        digitalWrite(pinAuxLed[0], HIGH);
        delay(100);
        digitalWrite(pinAuxLed[0], LOW);       
        delay(100); 
    }

    selectMode();
    //periodicHandler();

    logMsg("Reset occurred\n");
    Serial.flush();
}

/**************************************************************************/
// 
/**************************************************************************/
void loop() 
{
    // reset timer
    wdt_reset();
    
    cmd.poll();

    if (normalMode)
    {
        if (flagRtc)
        {
            flagRtc = false;

            
            // handle rtc interrupt
            // minute alarm has occurred. handle it
            if (DS3231_triggered_a1())
            {                
                digitalWrite(pinAuxLed[0], HIGH);
                delay(200);
                digitalWrite(pinAuxLed[0], LOW);
                
                // minute alarm
                DS3231_clear_a1f();

                sprintf(tmpBuf, "RTC Interrupt - Timecount: %d.\n", timeCount);
                logMsg(tmpBuf);
                
                if (timeCount >= (meta.txInterval-1))
                {
                    timeCount = 0;
                    periodicHandler();     
                }
                else
                {
                    timeCount++;
                }        
            }             
        }
/*
        // if we receive a packet, handle it
        if (pktLen)
        {
            pktHdr_t hdr;
            char data[MAX_PAYLOAD_SIZE];
            int16_t rssi = LoRa.packetRssi();

            memset(data, 0, MAX_PAYLOAD_SIZE);
            receivePacket(pktLen, &hdr, data); 
            if (hdr.destAddr == meta.stationNum)
            {
                // make sure packet is for us
                sprintf(tmpBuf, "Packet received with RSSI %d: %s.\n", rssi, data);
                logMsg(tmpBuf);
            }
            pktLen = 0;        
        }
*/

        // go to sleep
        sysSleep();           
    }
    else
    {
        // implement timeout in command line mode to confirm we want to stay in that mode. 
        if (elapsedTime(cmdModeTimeCnt) > cmdModeTimeLimit)
        {
            selectMode();
        }
    }
}

/**************************************************************************/
// 
/**************************************************************************/
void isrRtc()
{
    flagRtc = true;
}

/**************************************************************************/
// LoRa ISR
/**************************************************************************/
void onReceive(int len)
{
    pktLen = len;
}

/**************************************************************************
// 
**************************************************************************/
void periodicHandler()
{
    bool ackd = false;
    char payload[MAX_PAYLOAD_SIZE];
    char buf[MAX_PAYLOAD_SIZE];
    wdt_reset();

    memset(&data, 0, sizeof(data_t));
    
    // turn on soil moisture sensors
    digitalWrite(pinSoilEnb, POWER_ON);

    // turn on CO2 power
//    digitalWrite(pinCo2Enb, HIGH); 

    delay(1000);

    // read soil moisture sensors
    for (int i=0; i<NUM_SOIL_SENSORS; i++)
    {
        data.soilMoisture[i] = getSoilMoisture(i);
        delay(100);
    }

    // read pH
    data.phVoltage = ph.readVoltage();
    data.ph = ph.read();

    // read BME
    if (bmeIsPresent)
    {
        if (bme.takeForcedMeasurement())
        {
            data.bmeTemp = bme.readTemperature();
            data.bmePressure = bme.readPressure() / 100.0;
            data.bmeAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
            data.bmeHumidity = bme.readHumidity();
        }
        else
        {
            data.bmeTemp = 0.0;
            data.bmePressure = 0.0;
            data.bmeAltitude = 0.0;
            data.bmeHumidity = 0.0;   
        }
    }
    else
    {
        data.bmeTemp = 0.0;
        data.bmePressure = 0.0;
        data.bmeAltitude = 0.0;
        data.bmeHumidity = 0.0;    
    }
    
    // read soil temperature
    data.soilTemp = getSoilTemperature();

    // get co2
//    data.co2PPM = getCO2();

    // get battery voltage
    data.battVal = getBattery();

    // get solar voltage
    data.solarVal = getSolar();
    
    // get internal temperature
    data.internalTemp =  DS3231_get_treg();

    for (int i=0; i<MAX_TRANSMIT_RETRIES; i++)
    {
        uint32_t now;
        wdt_reset();
        memset(payload, 0, sizeof(payload));
        sprintf(payload, "%s,%d,%d,%lu,%3.2f,%2.2f,%2.2f,%3.2f,%5.2f,%3.2f,%5.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%d\n", \
                    meta.stationID, meta.stationNum, stats.retries, stats.failedTx, \
                    data.internalTemp, data.battVal, data.solarVal, \
                    data.bmeTemp, data.bmePressure, data.bmeHumidity, data.bmeAltitude, \
                    data.soilMoisture[0], data.soilMoisture[1], data.soilMoisture[2], \
                    data.phVoltage, data.soilTemp, data.co2PPM);
                    
        // write to data file
        logData(payload);

        // write to logfile with sequence number
        sprintf(tmpBuf, "Seq: %d: %s", hdr.seq, payload);
        logMsg(tmpBuf);

        Serial.flush();
        break;

/*    
        // send data out
        uint8_t *phdr = (uint8_t *)&hdr;
        LoRa.beginPacket();
        for (int i=0; i<sizeof(pktHdr_t); i++)
        {
            LoRa.write(*phdr);  
            phdr++; 
        }
        LoRa.print(payload);
        LoRa.endPacket();
        LoRa.flush(); 
        LoRa.receive();     

        // wait for ACK
        now = millis();  
        while (elapsedTime(now) < ACK_WAIT_TIMEOUT)
        {
            if (pktLen)
            {
                pktHdr_t rxHdr;
                char buf[MAX_PAYLOAD_SIZE];
                
                memset(buf, 0, MAX_PAYLOAD_SIZE);
                receivePacket(pktLen, &rxHdr, buf); 
                

                // is it an ACK packet?
                if (rxHdr.type == PKT_ACK)
                {
                    // is it for us?
                    if (rxHdr.destAddr == meta.stationNum)
                    {
                        sprintf(tmpBuf, "ACK Received - src: %d, dest: %d, seq: %d, type: %d.\n", rxHdr.srcAddr, rxHdr.destAddr, rxHdr.seq, rxHdr.type);
                        logMsg(tmpBuf);                      
                        stats.retries = 0;
                        ackd = true;            
                    }                    
                }
                pktLen = 0;
                break;        
            }
        }   
        
        if (ackd)
        {
            break;     
        }
        else
        {
            stats.retries++;
        }
*/        
    }  

/*
    if (!ackd)
    {
        // the transmission attempt failed
        stats.retries = 0;
        stats.failedTx++;    
        logMsg("Data transmission attempt failed\n");
    
    }
    hdr.seq++;    
*/        
    Serial.flush();
}

/**************************************************************************/
// 
/**************************************************************************/
void sysSleep()
{
    wdt_disable();
    
    // sleep LoRa device
    LoRa.sleep();

    // shut down power to soil moisture and co2 sensors
    digitalWrite(pinSoilEnb, POWER_OFF);
    digitalWrite(pinCo2Enb, LOW);

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
    
    // enable watchdog timer
    wdt_enable(WDTO_8S);   
}
