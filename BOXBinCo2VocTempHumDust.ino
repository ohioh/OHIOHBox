/*
   ###################################################################################################################
   ###################################################################################################################
   OHIOH LoRaWAN-1.0.2 use OTAA, CLASS A
   Band: 868300000 Hz at DR 5
   Author: Tjark Ziehm
   Date: August 2021
   Version: 1.0.0
   Hardware: Heltec Wireless Stick && Heltec Wireless Stick Lite (ESP32 BASE)
   Sensors: DHT22, SGP30 and
   Peripherals: Display and multicolor LED
   MultiProcessor Support: YES

   Parser-Code:
   Github-Repo:

   ESP32-Function summary:
   LoRaWan Settings:
   You can change some definition in "Commissioning.h" and "LoRaMac-definitions.h"

   General:
   - You can use port 4 to control the LED light.
   - If the issued value is 1(ASCII), the lamp will be lit.
   - The release value is 2(ASCII) and the light will be turned off.
   - use internal RTC(150KHz);
   - Include stop mode and deep sleep mode;
   - 15S data send cycle;
   - Informations output via serial(115200);
   - Only ESP32 + LoRa series boards can use this library, need a license
     to make the code run(check you license here: http://www.heltec.cn/search/);

  LoRaWan-Specification: 1.0.2
  -> https://lora-alliance.org/resource_hub/lorawan-specification-v1-0-2/

  LoRaWan Package Size:
  defined here: ->PAYLOAD DATAFRAME BUILDER
  uintXX_t appData[LORAWAN_APP_DATA_MAX_SIZE];
  XX should be 16 or 32
  Files -> ESP32_LoRaWAN.h & ESP32_LoRaWAN.cpp
  default-setting is uint16_t and is sendind 16 digits per value in payload
  (The Zenner converter is prepaired for 16 digits [no 32 bits support yet])
  example 0b0000000000000000; //for integer values till to 65535

  Sensor-Function summary:
  COMING SOON

  Code-Function summary:
  1.  getting Data from Sensor in float or int
  2.  convert with decToBinary() in binary to binSensorData ( = 0bxxxx xxxx xxxx xxxx)
  3.  prepair for Zenner Platform Logic ( everytime readable now ) zennerParserPrepair()
  4.  store in the right messured value
  5.  repeat till all messurments are done
  6.  prepair data in the payload with prepareTxFrame() in [loop]:DEVICE_STATE_SEND
  7.  with LoRaWAN.send(loraWanClass) data will be send and be avaible in Zenner Platform

  Index:
  0.
  1.  Tasks
  2.  Prepair LoRaWan Hardware
  3.  Sensor-Variables
  4.  Convert ( Concat and convert in HEX and BIN in PROGRESS )
  5.  Convert Integer to Binary
  6.  Zenner Pakage Logic
  X.  Connect Temperature and Humidity Sensor
  X.  Connect CO² and VOC Sensor
  XX. Connect Battery-Status
  XX. Connect Dust-Sensor
  XX. Connect Status-Communication
  xx. Payload DataFrame Builder
  xx. Connect LED
  1x. Connect Multiprocess Feature
  1x. Setup the ESP32
  1x. Loop the ESP32
   ###################################################################################################################
   ###################################################################################################################
*/

/////////////////////////////////////////////////---INCLUDE---//////////////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <ESP32_LoRaWAN.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "SparkFun_SGP30_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP30
#include <Wire.h>
#include <bitset>
#include <cassert>
#include <string>
#include <sps30.h>


////////////////////////////////////////////////---TASKS---///////////////////////////////////////////////////////////////////////////
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TaskHandle_t Task1; //Core specific Task 1
TaskHandle_t Task2; //Core specific Task 2
TaskHandle_t Task3; //Core specific Task 3
SGP30 mySensor; //create an object of the SGP30 class

// uncomment the next line to use the serial plotter
//#define PLOTTER_FORMAT


//////////////////////////////////////////////---PREPAIR LORAWAN HARDWARE---//////////////////////////////////////////////////////////
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t license[4] = {0x3BF994AB, 0x6E5C029E, 0xDC3BE428, 0x28205375};

/* OTAA para*/
uint8_t DevEui[] = {0x12, 0x34, 0x56, 0x77, 0x77, 0x77, 0x77, 0x01};
uint8_t AppEui[] = {0x12, 0x34, 0x56, 0x77, 0x77, 0x77, 0x77, 0x01};
uint8_t AppKey[] = {0x12, 0x34, 0x56, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x01};

/* ABP para*/
uint8_t NwkSKey[] = {0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85};
uint8_t AppSKey[] = {0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67};
uint32_t DevAddr = (uint32_t)0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 1500;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

/*CHIP-ID*/
uint64_t chipid;

/*WakeUpButton*/
#define INT_PIN GPIO_NUM_0

/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/*LoraWan debug level, select in arduino IDE tools.
  None : print basic info.
  Freq : print Tx and Rx freq, DR info.
  Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
  Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt and MCU deepsleep info.
*/
uint8_t debugLevel = LoRaWAN_DEBUG_LEVEL;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

#define LEDPin 25 //LED light


//////////////////////////////////////////////---SENSOR-VARIABLES---///////////////////////////////////////////////////////////////////
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Hardware status and device communication
unsigned int hardwareState = 0;
uint16_t binaryHardwareStatus = 0b1100110011001100; //52428 and CCCC

// Battery-Power
unsigned int batteryStatus = 65;
uint16_t binaryBatteryStatus = 0b1100110011001100; //52428 and CCCC

unsigned int sensorDataTemperature = 0;
uint16_t binaryTemperature = 0b0000000000000000; //0 and CCCC

int sensorDataHumidity = 0;
uint16_t binaryHumidity = 0b0000000000000000; //0 and CCCC

unsigned int sensorDataVOC = 0;
uint16_t binaryVOC = 0b1100110011001100; //52428 and CCCC

unsigned int sensorDataCO2 = 0;
uint16_t binaryCO2 = 0b1100110011001100; //52428 and CCCC

unsigned int sensorDataPM1 = 0;
uint16_t binaryPM1 = 0b1100110011001100; //52428 and CCCC

unsigned int sensorDataPM25 = 0;
uint16_t binaryPM25 = 0b1100110011001100; //52428 and CCCC

unsigned int sensorDataPM4 = 0;
uint16_t binaryPM4 = 0b1100110011001100; //52428 and CCCC

unsigned int sensorDataPM5 = 0;
uint16_t binaryPM5 = 0b1100110011001100; //52428 and CCCC

unsigned int sensorDataPM10 = 0;
uint16_t binaryPM10 = 0b1100110011001100; //52428 and CCCC


///////////////////////////////////////////////--CONVERTER [in progress]--////////////////////////////////////////////////////////////////
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*WORK IN PROGRESS*/
/*
  #define B_0000    0
  #define B_0001    1
  #define B_0010    2
  #define B_0011    3
  #define B_0100    4
  #define B_0101    5
  #define B_0110    6
  #define B_0111    7
  #define B_1000    8
  #define B_1001    9
  #define B_1010    a
  #define B_1011    b
  #define B_1100    c
  #define B_1101    d
  #define B_1110    e
  #define B_1111    f

  #define _B2H(bits)    B_##bits
  #define B2H(bits)    _B2H(bits)
  #define _HEX(n)        0x##n
  #define HEX(n)        _HEX(n)
  //#define _BIN(n)        0b##n
  //#define BIN(n)        _BIN(n)
  #define _CCAT(a,b)    a##b
  #define CCAT(a,b)   _CCAT(a,b)

  #define BYTE(a,b)        HEX( CCAT(B2H(a),B2H(b)) )
  #define WORD(a,b,c,d)    HEX( CCAT(CCAT(B2H(a),B2H(b)),CCAT(B2H(c),B2H(d))) )
  #define DWORD(a,b,c,d,e,f,g,h)    HEX( CCAT(CCAT(CCAT(B2H(a),B2H(b)),CCAT(B2H(c),B2H(d))),CCAT(CCAT(B2H(e),B2H(f)),CCAT(B2H(g),B2H(h)))) )

  #define binBYTE(a,b)        BIN( CCAT(B2H(a),B2H(b)) )
  #define binWORD(a,b,c,d)    BIN( CCAT(CCAT(B2H(a),B2H(b)),CCAT(B2H(c),B2H(d))) )
  #define binDWORD(a,b,c,d,e,f,g,h)    BIN( CCAT(CCAT(CCAT(B2H(a),B2H(b)),CCAT(B2H(c),B2H(d))),CCAT(CCAT(B2H(e),B2H(f)),CCAT(B2H(g),B2H(h)))) )


  // Using example
  char b = BYTE(0100,0001); // Equivalent to b = 65; or b = 'A'; or b = 0x41;
  unsigned int w = word(1101, 1111, 0100, 0011); // Equivalent to w = 57155; or w = 0xdf43;
  unsigned long int dw = DWORD(1101, 1111, 0100, 0011, 1111, 1101, 0010, 1000); //Equivalent to dw = 3745774888; or dw = 0xdf43fd28;
*/


///////////////////////////////////////////////---CONVERT INTEGER TO BINARY---//////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

constexpr size_t arraySize = 16;
unsigned int invertedBinaryNum[arraySize] {0};
unsigned int binaryNum[arraySize] {0};
uint32_t binValue = 0;
uint16_t binSensorData = 0b0000000000000000; //for integer values till to 65535

//TODO: change the int to size_t
void decToBinary(int input)
{
  // array to store binary number
  //Serial.printf("\nGet Binary for %d \n", input);

  //Reset of the used Arrays and Values
  //TODO: overwrite tempBinaryArray with value:0
  //TODO: Controll the Size of writing width
  for (size_t counter = 0; counter < (arraySize); counter++)
  {
    invertedBinaryNum[counter] = 0;
  }

  for (size_t counter = 0; counter < (arraySize); counter++) {
    int printValue = 0;
    printValue = invertedBinaryNum[counter];
    //Serial.println("Value in invertedBinaryNum:");
    //Serial.print(printValue);
    //Serial.println("\n");
  }

  //TODO: overwrite BinaryArray with value:0
  //TODO: Controll the Size of writing width
  for (size_t counter = 0; counter < (arraySize); counter++)
  {
    binaryNum[counter] = 0;
  }

  for (size_t counter = 0; counter < (arraySize); counter++) {
    int printValue = 0;
    printValue = binaryNum[counter];
    //Serial.println("Value in BinaryNum:");
    //Serial.print(printValue);
    //Serial.println("\n");
  }

  //TODO: overwrite binValue with 0
  //Check delivered value from uint32_t or byte for binValue
  //uint32_t binValue = 0;


  /*
    char lengthInput = sizeof(input);
    Serial.println("########################################\n");
    Serial.print("\nValue of length: ");
    Serial.print(lengthInput);
    Serial.println("\n########################################\n");
  */
  // counter for binary array
  int arraySizeCounter = 0;

  //Write the inverted binary to an array
  //highest place number in the array is the beginning of the binary
  while (input > 0)
  {
    // storing remainder in binary array
    invertedBinaryNum[arraySizeCounter] = input % 2;
    input = input / 2;
    arraySizeCounter++;  //length of value
  }

  //Serial.print("\n#################\n");
  //Serial.print("Values in Array:\n");
  //Serial.print(arraySizeCounter);
  //Serial.print("\n#################\n");

  // safing & printing binary array in reverse ("right" -> EU) order
  for (size_t counter = arraySizeCounter; counter >= 1; counter--)
  {
    int arrayPlace = arraySizeCounter - counter;
    binaryNum[arrayPlace] = invertedBinaryNum[counter - 1];
    /*
      Serial.print("\n####Step:####\n");
      Serial.println(counter);
      Serial.print("\n");
      Serial.println(arrayPlace);
      Serial.print("\n");
      Serial.println(invertedBinaryNum[counter - 1]);
      Serial.print("\n");
      Serial.println(binaryNum[arrayPlace]);
      Serial.print("\n########\n");
    */
  }

  //Write bin Array Values with bitWrite to binSensorData

  size_t writeSpace = arraySizeCounter - 1;

  //Serial.print("\n########\n");
  //Serial.print(writeSpace);
  //Serial.print("\n########\n");

  for (size_t binDataPlace = 0; binDataPlace < arraySizeCounter; binDataPlace ++) {
    byte transmitValue = invertedBinaryNum[binDataPlace];
    /*
      Serial.print("\n########\n");
      Serial.print("Transmitted Value:\n");
      Serial.print(transmitValue);
      Serial.print("\n");
      Serial.print(writeSpace);
      Serial.print("\n########\n");
    */
    bitWrite(binSensorData, binDataPlace, transmitValue);  //Schreibe 1 auf das niedrigstwertige Bit von x
    writeSpace = writeSpace - 1;
  }
  /*
    Serial.print("\n########\n");
    Serial.print("BinValue\n");
    Serial.println(binSensorData);
    Serial.print("\n########\n");
  */
  //TODO: writing the binValue with an "0b"
  // test
  /*
    Serial.println("########################################\n");
    byte binSensorData = 0b10000000;  // Das 0b-Präfix gibt eine binäre Konstante an
    Serial.println(x, BIN); // 10000000
    bitWrite(binSensorData, 0, 1);  // Schreibe 1 auf das niedrigstwertige Bit von x
    Serial.println(x, BIN); // 10000001
    Serial.println("########################################\n");
  */
}


///////////////////////////////////////////---ZENNER PLATFORM LOGIC---/////////////////////////////////////////////////////////////////////
//
//  the number 6500 ( FD E8 ) gets E8DF in the Zenner Package View.
//  This is changed in this logic, because the transmitted value is switched to read logic of zenner-IoT
//  The Read out for the parser will be again the right number
//  binPlatformData is overwritten in each call -> use it convert the messurment
//  The Zenner converter is prepaired for 16 digits
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t binPlatformData = 0b0000000000000000; //for integer values till to 65535

void zennerParserPrepair()
{
  /*
    Serial.println("#########---Zenner-Logic---#");
    Serial.println("Get bin data:");
    Serial.println(binSensorData, BIN);
    Serial.println("#########");
  */
  binPlatformData = 0b0000000000000000;
  int writeDataPlace = 15;
  /*
    Serial.println(binSensorData);
    Serial.println(binPlatformData);
  */
  //Serial.println("#########---Zenner-Switch-1---##");
  // First (left Block switch to right block)
  for (size_t binDataPlace = 15; binDataPlace >= 8; binDataPlace --) {
    byte transmitValue = bitRead(binSensorData, binDataPlace);
    //Serial.println(transmitValue);
    bitWrite(binPlatformData, (binDataPlace - 8), transmitValue); //Schreibe 1 auf das niedrigstwertige Bit von x
  }
  /*
    Serial.println("#########");
    Serial.println(binPlatformData, BIN);
    Serial.println("#########");
  */
  //Serial.println("#########---Zenner-Switch-2---###");
  //Second (right Block switch to left)
  //Thi Output is inverted
  for (size_t binDataPlace = 0; binDataPlace <= 7; binDataPlace ++) {
    byte transmitValue = bitRead(binSensorData, binDataPlace);
    //Serial.println(transmitValue);
    bitWrite(binPlatformData, (binDataPlace + 8), transmitValue); //Schreibe 1 auf das niedrigstwertige Bit von x

  }
  /*
    Serial.println("#########");
    Serial.println(binPlatformData, BIN);
    Serial.println("#########");
  */
  Serial.println("[Zenner-Convert]: Done");
}


/////////////////////////////////////////---CONNECT TEMPERATURE AND HUMIDITY SENSOR---///////////////////////////////////////////////////
//  Variables:
//    unsigned int sensorDataTemperature = 0;
//    uint16_t binaryTemperature = 0b1100110011001100;
//
//    int sensorDataHumidity = 0;
//    uint16_t binaryHumidity = 0b1100110011001100;
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define DHTPIN 33 // Digital pin connected to the DHT sensor

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE DHT22 // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS = 50;
bool DHTactivated = true;
bool messureTemperature = true;  //true means only Temperature, false means only Humidity

int connectDHT()
{
  binSensorData = 0b0000000000000000;
  binPlatformData = 0b0000000000000000;
  Serial.println("[Messurment]:Getting Data from DHT22");
  dht.begin();
  delay(50);
  sensors_event_t event;
  // Delay between measurements.
  delay(2000);
  if (DHTactivated == false)
  {

    DHTactivated = true;
    Serial.print("[Messurment]:DHT22 activated\n");
  }
  /////////////////***************--Get temperature event and print its value--****************/////////////////
  if (DHTactivated == true)
  {
    if ( messureTemperature == true) {
      if (isnan(event.temperature))
      {
        Serial.println(F("[Messurment]:Error reading temperature!"));

      }
      else
      {
        // Delay between measurements.
        delay(2000);
        dht.temperature().getEvent(&event);
        // Get temperature event and print its value.
        Serial.print(F("[Messurment]:Temperature: "));
        delay(50);
        sensorDataTemperature = event.temperature;
        delay(500);
        Serial.print(sensorDataTemperature);
        Serial.print(F(" °C\n"));
        //Convert Deicimal Value in Binary in binSensorData
        decToBinary(sensorDataTemperature);
        //Convert Binary for Zenner-Logic in binPlatformData
        zennerParserPrepair();
        binaryTemperature = binPlatformData;

      }
    }
    /////////////////***************--Get humidity event and print its value--****************/////////////////
    if (messureTemperature == false) {
      // Delay between measurements.
      delay(delayMS);
      dht.humidity().getEvent(&event);
      delay(delayMS);
      if (isnan(event.relative_humidity))
      {
        Serial.println(F("[Messurment]:Error reading humidity!"));
      }
      else
      {
        Serial.print(F("[Messurment]:Humidity: "));
        sensorDataHumidity = event.relative_humidity;
        Serial.print(event.relative_humidity);
        Serial.print(F("%\n"));
        //Convert Deicimal Value in Binary in binSensorData
        decToBinary(sensorDataHumidity);
        //Convert Binary for Zenner-Logic in binPlatformData
        zennerParserPrepair();
        binaryHumidity = binPlatformData;

      }
    }
  }
}


/////////////////////////////////////////////---CONNECT CO² AND VOC SENSOR---//////////////////////////////////////////////////////////
//  Variables:
//    unsigned int sensorDataVOC = 0;
//    uint16_t binaryVOC = 0b1100110011001100;
//
//    unsigned int sensorDataCO2 = 0;
//    uint16_t binaryCO2 = 0b1100110011001100;
//
//    CO2: 375-450   ppm is normal outdoor air ( ppm = parts per million )
//         450-800   ppm good air quality
//         800-1000  ppm acceptable
//         1000-1500 ppm infection risc high
//         1000-2000 ppm infection risc very high
//         2000-5000 ppm without infection risk not longer then 8 hours
//         5000-6000 ppm Questionable for health
//         6000+     ppm high risk for health (100k dangerous for life, 200k deadly)
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool messureCO2 = true; // true means only CO2, false means only VOC

int connectSGP30()
{
  binSensorData = 0b0000000000000000;
  binPlatformData = 0b0000000000000000;
  //First fifteen readings will be
  //in normal situation [output]: CO2: 400 ppm  TVOC: 0 ppb
  Serial.println("[Messurment]:Getting Data from SGP30");
  delay(500); //Wait 1 second
  //measure CO2 and TVOC levels
  if (messureCO2 == true) {
    binSensorData = 0b0000000000000000;
    binPlatformData = 0b0000000000000000;
    mySensor.measureAirQuality();
    sensorDataCO2 = mySensor.CO2;
    //Convert Deicimal Value in Binary in binSensorData
    decToBinary(sensorDataCO2);
    //Convert Binary for Zenner-Logic in binPlatformData
    zennerParserPrepair();
    binaryCO2 = binPlatformData;
    Serial.print("[Messurment]:CO2: ");
    Serial.print(sensorDataCO2);
    Serial.print(" ppm\n");
    messureCO2 = false;
  }
  if (messureCO2 == false) {
    binSensorData = 0b0000000000000000;
    binPlatformData = 0b0000000000000000;
    mySensor.measureAirQuality();
    sensorDataVOC = mySensor.TVOC;
    //Convert Deicimal Value in Binary in binSensorData
    decToBinary(sensorDataVOC);
    //Convert Binary for Zenner-Logic in binPlatformData
    zennerParserPrepair();
    binaryVOC = binPlatformData;
    Serial.print("[Messurment]:tTVOC:");
    Serial.print(sensorDataVOC);
    Serial.print(" ppb\n");
    messureCO2 = true;
  }
}


////////////////////////////////////////////////////---CONNECT BATTERY STATUS---/////////////////////////////////////////////////////////////
//  Variables:
//    unsigned int batteryStatus; -> uint16_t binaryBatteryStatus;
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//TODO: get battery Status

int connectBatteryStatus() {
  binSensorData = 0b0000000000000000;
  binPlatformData = 0b0000000000000000;
  batteryStatus = 65;
  //Convert Deicimal Value in Binary in binSensorData
  decToBinary(batteryStatus);
  //Convert Binary for Zenner-Logic in binPlatformData
  zennerParserPrepair();
  binaryBatteryStatus = binPlatformData;
  /*
    uint16_t ADC_voltage = analogRead(37);
    digitalWrite(Vext, HIGH);
    ADC_Process( ADC_voltage );
  */
}


////////////////////////////////////////////////////---CONNECT DUST SENSOR---//////////////////////////////////////////////////////////////
//  Variables:
//    unsigned int sensorDataPM1; -> uint16_t binaryPM1;
//    unsigned int sensorDataPM25; -> uint16_t binaryPM25;
//    unsigned int sensorDataPM4; -> uint16_t binaryPM4;
//    unsigned int sensorDataPM5; -> uint16_t binaryPM5;
//    unsigned int sensorDataPM10; -> uint16_t binaryPM10;
//    Luftqualitätsnorm DIN EN 15267
//    Mass concentration precision: ±10 μg/m3 @ 0 to 100 μg/m3
//    Lower limit: 0.3 μm
//    Mass concentration Messurment:   PM1.0, PM2.5, PM4 and PM10
//    Number concentration Messurment:  PM0.5, PM1.0, PM2.5, PM4 and PM10
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//TODO: get Dust Sensor Data

int sensorDataDust = 42999;
//unsigned int sensorDataPM1 = 0;
//uint16_t binaryPM1 = 0b1100110011001100; //52428 and CCCC
int pmState = 1;

int connectDustSensor() {
  Serial.println("Connect Dust Sensor");

  struct sps30_measurement m;
  char serial[SPS30_MAX_SERIAL_LEN];
  uint16_t data_ready;
  int16_t ret;

  binSensorData = 0b0000000000000000;
  binPlatformData = 0b0000000000000000;

  //  do {
  //    ret = sps30_read_data_ready(&data_ready);
  //    if (ret < 0) {
  //      Serial.print("error reading data-ready flag: ");
  //      Serial.println(ret);
  //    } else if (!data_ready)
  //      Serial.print("data not ready, no new measurement available\n");
  //    else
  //      break;
  //    delay(100); /* retry in 100ms */
  //  } while (1);
  //
  ret = sps30_read_measurement(&m);
  if (ret < 0) {
    Serial.print("error reading measurement\n");
  } else {

#ifndef PLOTTER_FORMAT
    //Convert Deicimal Value in Binary in binSensorData
    switch (pmState)
    {
      ///////////////--PM1--////////////////////
      case 1:
        {

          sensorDataPM1 = m.mc_1p0;
          decToBinary(sensorDataPM1);
          //Convert Binary for Zenner-Logic in binPlatformData
          zennerParserPrepair();
          binaryPM1 = binPlatformData;

          Serial.print("PM  1.0: ");
          Serial.println(m.mc_1p0);
          break;
        }
      ///////////////--PM2.5--////////////////////
      case 2:
        {
          binSensorData = 0b0000000000000000;
          binPlatformData = 0b0000000000000000;
          sensorDataPM25 = m.mc_1p0;
          decToBinary(sensorDataPM25);
          //Convert Binary for Zenner-Logic in binPlatformData
          zennerParserPrepair();
          binaryPM25 = binPlatformData;

          Serial.print("PM  2.5: ");
          Serial.println(m.mc_2p5);
          break;
        }
      ///////////////--PM4--////////////////////
      case 3:
        {
          binSensorData = 0b0000000000000000;
          binPlatformData = 0b0000000000000000;
          sensorDataPM4 = m.mc_4p0;
          decToBinary(sensorDataPM4);
          //Convert Binary for Zenner-Logic in binPlatformData
          zennerParserPrepair();
          binaryPM4 = binPlatformData;

          Serial.print("PM  4: ");
          Serial.println(m.mc_4p0);
          break;
        }
      ///////////////--PM10--////////////////////
      case 4:
        {
          binSensorData = 0b0000000000000000;
          binPlatformData = 0b0000000000000000;
          sensorDataPM10 = m.mc_10p0;
          decToBinary(sensorDataPM10);
          //Convert Binary for Zenner-Logic in binPlatformData
          zennerParserPrepair();
          binaryPM10 = binPlatformData;

          Serial.print("PM  10: ");
          Serial.println(m.mc_10p0);
          break;
        }
        /*
          Serial.print("PM  2.5: ");
          Serial.println(m.mc_2p5);
          Serial.print("PM  4.0: ");
          Serial.println(m.mc_4p0);
          Serial.print("PM 10.0: ");
          Serial.println(m.mc_10p0);
        */
    }
    /*
      #ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
      Serial.print("NC  0.5: ");
      Serial.println(m.nc_0p5);
      Serial.print("NC  1.0: ");
      Serial.println(m.nc_1p0);
      Serial.print("NC  2.5: ");
      Serial.println(m.nc_2p5);
      Serial.print("NC  4.0: ");
      Serial.println(m.nc_4p0);
      Serial.print("NC 10.0: ");
      Serial.println(m.nc_10p0);

      Serial.print("Typical partical size: ");
      Serial.println(m.typical_particle_size);
      #endif
    */

    Serial.println();

#else
    // since all values include particles smaller than X, if we want to create buckets we
    // need to subtract the smaller particle count.
    // This will create buckets (all values in micro meters):
    // - particles        <= 0,5
    // - particles > 0.5, <= 1
    // - particles > 1,   <= 2.5
    // - particles > 2.5, <= 4
    // - particles > 4,   <= 10

    Serial.print(m.nc_0p5);
    Serial.print(" ");
    Serial.print(m.nc_1p0  - m.nc_0p5);
    Serial.print(" ");
    Serial.print(m.nc_2p5  - m.nc_1p0);
    Serial.print(" ");
    Serial.print(m.nc_4p0  - m.nc_2p5);
    Serial.print(" ");
    Serial.print(m.nc_10p0 - m.nc_4p0);
    Serial.println();


#endif /* PLOTTER_FORMAT */

  }

  delay(1000);
}


////////////////////////////////////////////////////---CONNECT STATUS COMMUNICATION---///////////////////////////////////////////////////
//  Variables:
//    hardwareState;
//    binaryHardwareStatus;
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//TODO: get Status Communication

int connectStatusCommunication() {
  binSensorData = 0b0000000000000000;
  binPlatformData = 0b0000000000000000;
  Serial.println("Connect Status Communication");
  hardwareState = 1;
  //Convert Deicimal Value in Binary in binSensorData
  decToBinary(hardwareState);
  //Convert Binary for Zenner-Logic in binPlatformData
  zennerParserPrepair();
  binaryHardwareStatus = binPlatformData;
}


///////////////////////////////////////---PAYLOAD DATAFRAME BUILDER---/////////////////////////////////////////////////////////////////
//  Collecting Data from Sensors and ESP32 and prepair this values for payload transfer
//
//  Called Variables:
//    1.binaryHardwareStatus    7.PM1           13.qmPM2.5[soon]
//    2.binaryBatteryStatus     8.PM2.5         14.qmPM4[soon]
//    3.binaryTemperature       9.PM4           15.qmPM5[soon]
//    4.binaryHumidity         10.PM5           16.qmPM10[soon]
//    5.binaryCO2              11.PM10
//    6.binaryVOC              12.qmPM1[soon]
//
//  Mass concentration:   PM1.0, PM2.5, PM4 and PM10
//  Number concentration:  PM0.5, PM1.0, PM2.5, PM4 and PM10
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern uint16_t appData[];

void prepareTxFrame(uint8_t port)
{
  Serial.println("[Payload]: prepair TX Frame");

  //Size of values in Payload as binary
  //depends to uintXX_t appData[]
  appDataSize = 20; //AppDataSize max value is 64 -> each number is for 2 digits
  //Important: AppDataSize representate the number of transmitted bytes

  //Parser: status: state
  //TODO: get actual state here
  //TODO: convert to binary
  connectStatusCommunication();
  appData[0] = binaryHardwareStatus;
  delay(50);

  //Parser: battery: bat
  //TODO: get actual battery state here
  //TODO: convert to binary
  connectBatteryStatus();
  appData[1] = binaryBatteryStatus;
  delay(50);

  //Parser: temperature: temp,
  //TODO: get temperature
  //TODO: convert to binary
  connectDHT();
  appData[2] = binaryTemperature;
  messureTemperature = false;
  delay(1000);

  //Parser: humidity: hum
  //TODO: get humidity
  //TODO: convert to binary
  connectDHT();
  appData[3] = binaryHumidity;
  messureTemperature = true;
  delay(50);

  //Parser: codioxid: co2
  //TODO: get CO2
  //TODO: convert to binary
  connectSGP30();
  appData[4] = binaryCO2;
  messureCO2 = false;
  delay(500);

  //Parser: loesemittel: voc
  //TODO: get VOC
  //TODO: convert to binary
  connectSGP30();
  appData[5] = binaryVOC;
  messureCO2 = true;
  delay(500);

  //Parser: feinstaub1: pm1
  connectDustSensor();
  appData[6] = binaryPM1;
  pmState = 2;
  delay(500);

  //Parser: feinstaub1: pm2.5
  connectDustSensor();
  appData[7] = binaryPM25;
  pmState = 3;
  delay(2000);

  //Parser: feinstaub1: pm4
  connectDustSensor();
  appData[8] = binaryPM4;
  pmState = 4;
  delay(2000);

  //Parser: feinstaub1: pm10
  connectDustSensor();
  appData[9] = binaryPM10;
  pmState = 1;
  delay(2000);

}



////////////////////////////////////////////////////---CONNECT LED---////////////////////////////////////////////////////////////////
//  Variables:
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int connectLED() {

}

////////////////////////////////////////////////////---CONNECT MULTIPROCESSOR FEATURE---/////////////////////////////////////////////
//  Variables:
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int connectProcessors() {

}


////////////////////////////////////////////////////////---SETUP---///////////////////////////////////////////////////////////////////
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  int16_t ret;
  uint8_t auto_clean_days = 4;
  uint32_t auto_clean;

  Serial.begin(115200);
  pinMode(LEDPin, OUTPUT);
  while (!Serial)
    ;
  SPI.begin(SCK, MISO, MOSI, SS);
  Mcu.init(SS, RST_LoRa, DIO0, DIO1, license);
  adcAttachPin(37);
  analogSetClockDiv(255); // 1338mS
  pinMode(Vext, OUTPUT);

  //////////////////////////////////
  deviceState = DEVICE_STATE_INIT;

  ///////////////--Initialize sensor--////////////////////
  Wire.begin();
  dht.begin();
  if (mySensor.begin() == false)
  {
    Serial.println("No SGP30 Detected. Check connections.");
    while (1)
      ;
  }
  //Initializes sensor for air quality readings
  //measureAirQuality should be called in one second increments after a call to initAirQuality
  mySensor.initAirQuality();

  //Test-Area
  /*
    Serial.println("########################################\n");
    Serial.println("########################################\n");
    int a = 1011;
    int b = 1101;
    int c = 0101;
    int d = 1111;
    //Serial.println(binWORD(a, b, c, d));
    Serial.println("########################################\n");
    Serial.println("########################################\n");
  */
  ///////////////--Dust Sensor INIT--////////////////////
  delay(2000);
  sensirion_i2c_init();

  while (sps30_probe() != 0) {
    Serial.print("SPS sensor probing failed\n");
    delay(500);
  }

#ifndef PLOTTER_FORMAT
  Serial.print("SPS sensor probing successful\n");
#endif /* PLOTTER_FORMAT */

  ret = sps30_set_fan_auto_cleaning_interval_days(60);
  if (ret) {
    Serial.print("error setting the auto-clean interval: ");
    Serial.println(ret);
  }

  ret = sps30_start_measurement();
  if (ret < 0) {
    Serial.print("error starting measurement\n");
  }


#ifndef PLOTTER_FORMAT
  Serial.print("measurements started\n");
#endif /* PLOTTER_FORMAT */

#ifdef SPS30_LIMITED_I2C_BUFFER_SIZE
  Serial.print("Your Arduino hardware has a limitation that only\n");
  Serial.print("  allows reading the mass concentrations. For more\n");
  Serial.print("  information, please check\n");
  Serial.print("  https://github.com/Sensirion/arduino-sps#esp8266-partial-legacy-support\n");
  Serial.print("\n");
  delay(2000);
#endif

  delay(1000);
}

//////////////////////////////////////////////////---LOOP---/////////////////////////////////////////////////////////////////////////
// The loop function is called in an endless loop
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  //Test-Area
  /*
    Serial.println("########################################\n");
    Serial.println("########################################\n");
    int a = 1011;
    int b = 1101;
    int c = 0101;
    int d = 1111;
    //Serial.println(binWORD(a, b, c, d));
    Serial.println("########################################\n");
    Serial.println("########################################\n");
  */
  //connectDHT();


  switch (deviceState)
  {
    ///////////////--Initialize--////////////////////
    case DEVICE_STATE_INIT:
      {
#if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
        LoRaWAN.init(loraWanClass, loraWanRegion);
        break;
      }

    ///////////////--Join--////////////////////
    case DEVICE_STATE_JOIN:
      {
        Serial.println("[Loop]:Join");
        LoRaWAN.join();
        break;
      }

    ///////////////--State send--////////////////////
    case DEVICE_STATE_SEND:
      {
        /*
          Serial.println("########################################\n");
          Serial.println("########################################\n");
          Serial.println("Send payload:");
          digitalWrite(Vext, LOW);
          delay(50);
          Serial.println("########################################\n");
          Serial.println("Check Values from Sensor\n");
          Serial.println("Hardware State:\n");
          Serial.println(hardwareState);
          Serial.println("\n");
          Serial.println(binaryHardwareStatus);

          Serial.println("\nBattery Status:\n");
          Serial.println(batteryStatus);
          Serial.println("\n");
          Serial.println(binaryBatteryStatus);

          Serial.println("\nTemperature:\n");
          Serial.println(sensorDataTemperature);
          Serial.println("\n");
          Serial.println(binaryTemperature);

          Serial.println("\nHumidity:\n");
          Serial.println(sensorDataHumidity);
          Serial.println("\n");
          Serial.println(binaryHumidity);

          Serial.println("\nCO2:\n");
          Serial.println(sensorDataCO2);
          Serial.println("\n");
          Serial.println(binaryCO2);

          Serial.println("\nVOC:\n");
          Serial.println(sensorDataVOC);
          Serial.println("\n");
          Serial.println(binaryVOC);
          Serial.println("\n########################################\n");
          Serial.println("########################################\n");
          delay(500);
          connectDHT();
          delay(500);
        */
        prepareTxFrame(appPort);
        LoRaWAN.send(loraWanClass);
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }

    ///////////////--Cycle--////////////////////
    case DEVICE_STATE_CYCLE:
      {
        Serial.println("[Loop]:Cycle");
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle;
        //+ randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }

    ///////////////--Sleep--////////////////////
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep(loraWanClass, debugLevel);
        break;
      }

    ///////////////--default--////////////////////
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }


}
