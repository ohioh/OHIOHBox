/*
 * HelTec Automation(TM) LoRaWAN 1.0.2 OTAA example use OTAA, CLASS A
 *
 * Function summary:
 *
 * - You can use port 4 to control the LED light.
 *
 * - If the issued value is 1(ASCII), the lamp will be lit.
 *
 * - The release value is 2(ASCII) and the light will be turned off.
 *
 * - use internal RTC(150KHz);
 *
 * - Include stop mode and deep sleep mode;
 *
 * - 15S data send cycle;
 *
 * - Informations output via serial(115200);
 *
 * - Only ESP32 + LoRa series boards can use this library, need a license
 *   to make the code run(check you license here: http://www.heltec.cn/search/);
 *
 * You can change some definition in "Commissioning.h" and "LoRaMac-definitions.h"
 *
 * HelTec AutoMation, Chengdu, China.
 * 成都惠利特自动化科技有限公司
 * https://heltec.org
 * support@heltec.cn
 *
 *this project also release in GitHub:
 *https://github.com/HelTecAutomation/ESP32_LoRaWAN
*/

#include <ESP32_LoRaWAN.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

///////////////TASKS///////////////////////////////////////////////////////////////
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

//////////////////////////////////////////////////////////////////////////////////
/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t  license[4] = {0x3BF994AB, 0x6E5C029E, 0xDC3BE428, 0x28205375};

/* OTAA para*/
uint8_t DevEui[] = { 0x12, 0x34, 0x56, 0x77, 0x77, 0x77, 0x77, 0x01 };
uint8_t AppEui[] = { 0x12, 0x34, 0x56, 0x77, 0x77, 0x77, 0x77, 0x01 };
uint8_t AppKey[] = { 0x12, 0x34, 0x56, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x01 };

/* ABP para*/
uint8_t NwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t AppSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t DevAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

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
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 1;

/*LoraWan debug level, select in arduino IDE tools.
* None : print basic info.
* Freq : print Tx and Rx freq, DR info.
* Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
* Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt and MCU deepsleep info.
*/
uint8_t debugLevel = LoRaWAN_DEBUG_LEVEL;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;


#define LEDPin 25  //LED light

void app(uint8_t data)
 {
    // lora_printf("data:%d\r\n",data);
   switch(data)
     {
    case 49:
    {
      pinMode(LEDPin,OUTPUT);
      digitalWrite(LEDPin, HIGH);
      break;
    }
    case 50:
    {
      pinMode(LEDPin,OUTPUT);
      digitalWrite(LEDPin, LOW);
      break;
    }
    case 51:
    {
      break;
    }
    default:
    {
      break;
    }
     }
 }


void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  lora_printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  lora_printf("+REV DATA:");
    app(mcpsIndication->Buffer[0]);

  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
  {
    lora_printf("%02X",mcpsIndication->Buffer[i]);
  }
  lora_printf("\r\n");
}


//////////////////////////////////--DHT--////////////////////////////////////////////////////
#define DHTPIN 33     // Digital pin connected to the DHT sensor

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
bool DHTactivated = false;


////////////////////////////////--Connect Temperature and Humidity Sensor--//////////////////////////////////////////////
int Temperature;
int Humidity;

int connectDHT() {
  Serial.println("Getting Data from DHT22");
  dht.begin();
  if ( DHTactivated == false ){
    
    DHTactivated = true;
    Serial.println("DHT22 activated");
  }
  else if (DHTactivated == true){
    delay(1000);
    sensor_t sensor;
    // Delay between measurements.
    delay(delayMS);
    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    
    appData[2] = 0x3D;
    Temperature = event.temperature;
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
   }else{
      Serial.print(F("Temperature: "));
      //Serial.print(event.temperature);
      Serial.println(Temperature);
      Serial.println(F("°C"));
   }
   
//////////////Get humidity event and print its value./////////////////////////
  dht.humidity().getEvent(&event);
  
  
  Humidity = event.relative_humidity;
  appData[3] = 0xB7;
  //appData[3] = Humidity;
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    //Serial.print(event.relative_humidity);
    Serial.println(Humidity);
    Serial.println(F("%"));
  }
  }else {
    Serial.println("DHT not working");
  }

  return (Temperature);
}

/////////////////////////////////////////////////////////////////////////////////////////
void getSensorData() {
  connectDHT();
  delay(10000);

}

/////////////////////////////////////////////////////////////////////////////////////////
void buildTXFrame(){

}


/////////////////////////////////////////////////////////////////////////////////////////
//Values from the Sensors converted from Decimal to Hexadecimal
/////////////////////////////////////////////////////////////////////////////////////////

// packet_version and Selftest:
//First Digit is the Type, second the Selftest-Status 0-F
int PACKET_VERSION = 0xAA;
// Battery-Power in 6,25% Steps ( 16 Steps
int BATTERYSTATUS = 0x41;

int TEMPERATURE = 0x17;
long HUMIDITY = 0x2B;
int PM1 = 0xFF;
int PM25 = 0x10;
int PM4 = 0x00;
int PM5 = 0xC0;
int PM10 = 0xB0;
int VOC = 0x25;
int CO2 = 0xF5;



//////////////////////////////--DataFrame--///////////////////////////////////////////////
extern uint8_t appData[];

uint8_t prepareTxFrame( uint8_t port )
{
    connectDHT();
    appDataSize = 11;  //AppDataSize max value is 64
    
    appData[0] = PACKET_VERSION;    
    appData[1] = BATTERYSTATUS;    
    appData[2] = Temperature;    
    appData[3] = HUMIDITY;    
    
    appData[4] = PM1;
    appData[5] = PM25;
    appData[6] = PM4;
    appData[7] = PM5;
    appData[8] = PM10;

    appData[9] = VOC;
    appData[10] = CO2;

    
}

////////////////////////////--TASKS--/////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
void TaskCore1( void * parameter )
{
  for(;;) {
    Serial.print("------------------TaskCore1--------------------------\n");
    Serial.print("This Task run on Core: ");
    Serial.println(xPortGetCoreID()); 
    //connectDHT();
    digitalWrite(LEDPin,HIGH);
    delay(1000);
    digitalWrite(LEDPin,LOW);
    Serial.print("\n**************************1****************************\n");
    vTaskDelay(1000/ portTICK_PERIOD_MS);
    //vTaskDelete(NULL);
    
  }  
}

/////////////////////////////////////////////////////////////////////////////////////////
void TaskCore2( void * parameter )
{  
  for(;;) {
          Serial.print("------------------TaskCore2--------------------------\n");
          Serial.print("This Task run on Core: ");
          Serial.println(xPortGetCoreID());          
          digitalWrite(LEDPin,HIGH);
          delay(1000);
          digitalWrite(LEDPin,LOW);
          
          Serial.print("\n***********************2*******************************\n");
          vTaskDelay(1000/ portTICK_PERIOD_MS);
          //vTaskDelete(NULL);
          //connectDHT();          
      }   
}

/////////////////////////////////////////////////////////////////////////////////////////
void TaskCore3( void * parameter )
{  
  for(;;) {
    Serial.print("------------------TaskCore3--------------------------\n");
    connectDHT();
    vTaskDelay(100000/ portTICK_PERIOD_MS);
    //vTaskDelete(NULL);     
    //connectDHT();          
    }     
}

/////////////////////////////////--Setup--//////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);
  pinMode(LEDPin,OUTPUT);  
  while (!Serial);
  SPI.begin(SCK,MISO,MOSI,SS);
  Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);
  //////////////////////////////////
  deviceState = DEVICE_STATE_INIT;
  /////////////////////////////////
  //getSensorData();    
}

/////////////////////////////////////////////////////////////////////////////////////////
// The loop function is called in an endless loop
void loop()
{   
    /*
    xTaskCreatePinnedToCore(
      TaskCore1,           
      "Task_1",       
      1000,           
      NULL,           
      1,              
      &Task1,         
      0
     );             
    */
    

    /*
    xTaskCreatePinnedToCore(
    TaskCore2,           //Task Function. 
    "Task_2",       //name of task. 
    10000,           //Stack size of task. 
    NULL,           // parameter of the task. 
    2,              // proiority of the task. 
    &Task2,         // Task handel to keep track of created task. 
    0);             // choose Core 
    */
    

    
    xTaskCreatePinnedToCore(
    TaskCore3,           //Task Function. 
    "Task_3",       //name of task. 
    10000,           //Stack size of task. 
    NULL,           // parameter of the task. 
    1,              //proiority of the task. 
    &Task3,         // Task handel to keep track of created task. 
    1);             // choose Core 
    
    //connectDHT();
    //delay(1000);
    Serial.println("[Loop]:Temperature\n");
    Serial.println(appData[3]);
    
    switch( deviceState )
      { 
        case DEVICE_STATE_INIT:
        {
          #if(LORAWAN_DEVEUI_AUTO)
          LoRaWAN.generateDeveuiByChipID();
          #endif
          LoRaWAN.init(loraWanClass,loraWanRegion);
          break;
        }
        case DEVICE_STATE_JOIN:
        {
          LoRaWAN.join();
          break;
        }
        case DEVICE_STATE_SEND:
        {
          Serial.println("Send LoRaWan-Payload");
          Serial.println("[Loop]:Temperature\n");
          Serial.println(appData[3]);
          Serial.println("\n[Loop]:Humidity\n");
          Serial.println(appData[4]);
          prepareTxFrame(appPort);
          LoRaWAN.send(loraWanClass);
          deviceState = DEVICE_STATE_CYCLE;
          delay(5000);
          break;
        }
        case DEVICE_STATE_CYCLE:
        {
          // Schedule next packet transmission
          txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
          //Serial.println(txDutyCycleTime);
          LoRaWAN.cycle(txDutyCycleTime);
          deviceState = DEVICE_STATE_SLEEP;
          break;
        }
        case DEVICE_STATE_SLEEP:
        {
          esp_sleep_enable_ext0_wakeup(INT_PIN,0);
          LoRaWAN.sleep(loraWanClass,debugLevel);
          //deviceState = DEVICE_STATE_JOIN;
          //deviceState = DEVICE_STATE_SEND;
          //Serial.println("next cycle.");
          break;
        }
        default:
        {
          deviceState = DEVICE_STATE_SEND;
          break;
        }
      }
      
    //delay(1000);
    Serial.print("\n***********************Loop-End*******************************\n");

}
