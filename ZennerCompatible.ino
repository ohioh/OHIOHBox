/*
   HelTec Automation(TM) LoRaWAN 1.0.2 OTAA example use OTAA, CLASS A

   Function summary:

   - You can use port 4 to control the LED light.

   - If the issued value is 1(ASCII), the lamp will be lit.

   - The release value is 2(ASCII) and the light will be turned off.

   - use internal RTC(150KHz);

   - Include stop mode and deep sleep mode;

   - 15S data send cycle;

   - Informations output via serial(115200);

   - Only ESP32 + LoRa series boards can use this library, need a license
     to make the code run(check you license here: http://www.heltec.cn/search/);

   You can change some definition in "Commissioning.h" and "LoRaMac-definitions.h"

   HelTec AutoMation, Chengdu, China.
   成都惠利特自动化科技有限公司
   https://heltec.org
   support@heltec.cn

  this project also release in GitHub:
  https://github.com/HelTecAutomation/ESP32_LoRaWAN
*/

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


///////////////TASKS///////////////////////////////////////////////////////////////
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

SGP30 mySensor; //create an object of the SGP30 class
//////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////--Sensor-Variables--//////////////////////////////////
// packet_version and Selftest:
//First Digit is the Type, second the Selftest-Status 0-F
unsigned int packageState = 0;
uint32_t binaryPackageState = 0b110;

// Battery-Power
char value = 'ABBA';
unsigned int batteryStatus = 65;
//unsigned long batteryStatus = std::bitset<8> (value){value, /*length*/4, /*0:*/'A', /*1:*/'B'};
uint8_t binaryBatteryStatus = batteryStatus;

unsigned int sensorDataTemperature = 0;
uint32_t binaryTemperature = 0b0;
int sensorDataHumidity = 0;
uint32_t binaryHumidity = 0b0;

unsigned int sensorDataVOC = 0;
uint32_t binaryVOC = 0b111110100;
unsigned int sensorDataCO2 = 0;
uint32_t binaryCO2 = 0b111110;

unsigned int sensorDataPM1 = 0;
uint32_t binaryPM1 = 0b1;
unsigned int sensorDataPM25 = 0;
uint32_t binaryPM25 = 0b11;
unsigned int sensorDataPM4 = 0;
uint32_t binaryPM4 = 0b100;
unsigned int sensorDataPM5 = 0;
uint32_t binaryPM5 = 0b101;
unsigned int sensorDataPM10 = 0;
uint32_t binaryPM10 = 0b1010;

///////////////////////////////////////////////--Convert--///////////////////////////////////////////////
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

///////////////////////////////////////////////--Convert Integer to Binary///////////////////////////////////////////////
constexpr size_t arraySize = 16;
unsigned int invertedBinaryNum[arraySize] {0};
unsigned int binaryNum[arraySize] {0};
uint32_t binValue = 0;
uint16_t binSensorData = 0b0000000000000000; //for integer values till to 65535

//TODO: change the int to size_t
void decToBinary(int input)
{
  // array to store binary number
  Serial.printf("\nGet Binary for %d \n", input);

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

  // safing & printing binary array in reverse ("right"->EU) order
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

  Serial.print("\n########\n");
  Serial.print("BinValue\n");
  Serial.println(binSensorData);
  Serial.print("\n########\n");

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


//////////////////////////////--Zenner Pakage Logic--//////////////////////////////////////////////////
// the number 6500 ( FD E8 ) gets E8DF in the Zenner Package View.
// This is changed in this logic, because the transmitted value is switched to read logic of zenner-IoT
// The Read out for the parser will be again the right number
///////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t binPlatformData = 0b0000000000000000; //for integer values till to 65535

void zennerParserPrepair()
{
  Serial.println("#########---Zenner-Logic---#");
  Serial.println("Get bin data:");
  Serial.println(binSensorData, BIN);
  Serial.println("#########");
  //old intput        0b11111101 11101000
  
  binPlatformData = 0b0000000000000000;
  int writeDataPlace = 15;
  Serial.println(binSensorData);
  Serial.println(binPlatformData);

  Serial.println("#########---Zenner-Switch-1---##");
  // First (left Block switch to right block)
  for (size_t binDataPlace = 15; binDataPlace >= 8; binDataPlace --) {
    byte transmitValue = bitRead(binSensorData, binDataPlace);
    Serial.println(transmitValue);
    bitWrite(binPlatformData, (binDataPlace-8), transmitValue);  //Schreibe 1 auf das niedrigstwertige Bit von x
  }
  Serial.println("#########");
  Serial.println(binPlatformData, BIN);
  Serial.println("#########");

  Serial.println("#########---Zenner-Switch-2---###");
  //Second (right Block switch to left)
  //Thi Output is inverted
  for (size_t binDataPlace = 0; binDataPlace <= 7; binDataPlace ++) {
    byte transmitValue = bitRead(binSensorData, binDataPlace);
    Serial.println(transmitValue);
    bitWrite(binPlatformData, (binDataPlace+8), transmitValue);  //Schreibe 1 auf das niedrigstwertige Bit von x
    
  }
  Serial.println("#########");
  Serial.println(binPlatformData, BIN);
  Serial.println("#########");
  Serial.println("#########---Zenner-Switch-Done---####");
}

//////////////////////////////--DataFrame--///////////////////////////////////////////////
extern uint32_t appData[];
/*
        status: state,
        battery: bat,
        temperature: temp,
        humidity: hum,
        codioxid: co2
        loesemittel: voc,
        feinstaub1: pm1,
        feinstaub25: pm25,
        feinstaub4: pm4,
        feinstaub5: pm5,
        feinstaub10: pm10
*/


void prepareTxFrame(uint8_t port)
{
  Serial.println("prepair TX Frame");

  //Size of values in Payload as binary
  appDataSize = 2; //AppDataSize max value is 64 -> each number is for 2 digits
  //Important: AppDataSize representate the number of transmitted bytes

  //Parser: status: state
  //TODO: get actual state here
  //TODO: convert to binary
  //appData[0] = binaryPackageState;
  appData[0] = binPlatformData;
  delay(50);

  //Parser: battery: bat
  //TODO: get actual battery state here
  //TODO: convert to binary
  appData[1] = binaryBatteryStatus;
  delay(50);

  //Parser: temperature: temp,
  //TODO: get temperature
  //TODO: convert to binary
  connectDHT();
  appData[2] = binaryTemperature;
  ;
  delay(50);

  //Parser: humidity: hum
  //TODO: get humidity
  //TODO: convert to binary
  appData[3] = binaryHumidity;
  delay(50);

  //Parser: codioxid: co2
  //TODO: get CO2
  //TODO: convert to binary
  connectSGP30();
  appData[4] = binaryCO2;
  delay(50);

  //Parser: loesemittel: voc
  //TODO: get VOC
  //TODO: convert to binary
  appData[5] = binaryVOC;
  delay(50);

  /*
    //Parser: feinstaub1: pm1
    appData[6] = PM1;
    //Parser: feinstaub1: pm2.5
    appData[7] = PM25;
    //Parser: feinstaub1: pm4
    appData[8] = PM4;
    //Parser: feinstaub1: pm5
    appData[9] = PM5;
    //Parser: feinstaub1: pm10
    appData[10] = PM10;



    Serial.println("Seperated lines: \n");
    for (int i = 0; i < appDataSize; i++)
    {
    Serial.println("\n");
    Serial.print(appData[i]);
    }

    Serial.println("\n Single lines: \n");
    for (int i = 0; i < appDataSize; i++)
    {
    Serial.print(appData[i]);
    }
  */
}

////////////////////////////////--Connect Temperature and Humidity Sensor--//////////////////////////////////////////////
#define DHTPIN 33 // Digital pin connected to the DHT sensor

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE DHT22 // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS = 50;
bool DHTactivated = false;

int connectDHT()
{
  Serial.println("[Messurment]:Getting Data from DHT22\n");
  dht.begin();
  if (DHTactivated == false)
  {

    DHTactivated = true;
    Serial.println("[Messurment]:DHT22 activated\n");
  }
  //////////////--Get temperature event and print its value--/////////////////////////
  else if (DHTactivated == true)
  {
    delay(delayMS );
    sensor_t sensor;
    // Delay between measurements.
    delay(delayMS);
    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    delay(delayMS );

    if (isnan(event.temperature))
    {
      Serial.println(F("[Messurment]:Error reading temperature!"));
    }
    else
    {
      Serial.print(F("[Messurment]:Temperature: "));
      sensorDataTemperature = event.temperature;
      Serial.print(event.temperature);
      Serial.println(F("°C"));
    }

    //////////////--Get humidity event and print its value--/////////////////////////
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
      Serial.println(F("%\n"));
    }
  }
  else
  {
    Serial.println("DHT not working");
  }
}

/////////////////////////////////////--Connect CO² and VOC Sensor--////////////////////////////////////////////////////
int connectSGP30()
{
  //First fifteen readings will be
  //in normal situation [output]: CO2: 400 ppm  TVOC: 0 ppb
  Serial.println("[Messurment]:Getting Data from SGP30\n");
  delay(1000); //Wait 1 second
  //measure CO2 and TVOC levels
  mySensor.measureAirQuality();
  sensorDataVOC = mySensor.TVOC;
  sensorDataCO2 = mySensor.CO2;
  Serial.print("[Messurment]:CO2: ");
  Serial.print(sensorDataCO2);
  Serial.println("\n");
  Serial.print("[Messurment]:tTVOC: ");
  Serial.print(sensorDataVOC);
  Serial.println(" ppb\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//Battery-Power
/////////////////////////////////////////////////////////////////////////////////////////
//TODO: get battery Status

/////////////////////////////////--Setup--//////////////////////////////////////////////////

void setup()
{
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
}

/////////////////////////////////////////////////////////////////////////////////////////
// The loop function is called in an endless loop
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
        Serial.println("Join");
        LoRaWAN.join();
        break;
      }

    ///////////////--State send--////////////////////
    case DEVICE_STATE_SEND:
      {
        Serial.println("########################################\n");
        Serial.println("########################################\n");
        Serial.println("Send payload:");
        digitalWrite(Vext, LOW);
        delay(50);
        Serial.println("########################################\n");
        Serial.println("Check Values from Sensor\n");
        Serial.println("Package State:\n");
        Serial.println(packageState);
        Serial.println("\n");
        Serial.println(binaryPackageState);

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
        int number = 65000;
        decToBinary(number);
        zennerParserPrepair();
        delay(500);
        prepareTxFrame(appPort);

        /*
            uint16_t ADC_voltage = analogRead(37);
            digitalWrite(Vext, HIGH);
            ADC_Process( ADC_voltage );
        */
        LoRaWAN.send(loraWanClass);
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }

    ///////////////--Cycle--////////////////////
    case DEVICE_STATE_CYCLE:
      {
        Serial.println("Cycle");
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
