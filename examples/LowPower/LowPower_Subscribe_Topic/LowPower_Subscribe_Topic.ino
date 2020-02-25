/*!
 * @file Subscribe_Topic.ino
 * 
 * @brief After the program download is complete.
 * @brief You can use the BC20 module to connect to DFRobot Easy IOT cloud platform,
 * @brief and Subscribe your Topic
 * @If you change the demo, you need to press the RST button
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-10-29
 * @get from https://www.dfrobot.com
 */
#include "DFRobot_BC20.h"
#include "DFRobot_Iot.h"
#define  RED 0
#define  BLUE 1
#define  GREEN 2
#define  YELLOW 3
#define  PURPLE 4
#define  CYAN 5
#define  WHITE 6
/*Configure device certificate information*/
char* Iot_id = "Cv3YouPZR";
char* Client_ID  = "2";
char* Iot_pwd    = "CD3YTXEZRz";

/*Configure the domain name and port number*/
char* EasyIot_SERVER = "182.254.130.180";
char* PORT = "1883";

/*Set the Topic you need to publish to*/
char* subTopic = "QjREoXEZg";

/*
 *Use IIC for communication
 */
#define USE_IIC

/*
 *Use SoftwareSerial port for communication
 */
//#define USE_HSERIAL

/*
 *Use HardwareSerial  port for communication
 */
//#define USE_SSERIAL
/******************IIC******************/
#ifdef USE_IIC
/*
 * For general controllers. Communicate by IIC
 * Connect Instructions
 *    Controller     |    Module(BC20)
 *        SDA        |       D/T
 *        SCL        |       C/R
 *        GND        |       GND
 *     5V or 3V3     |       VCC
 *
 *
 * IIC_addr(A0,A1)
 *   0x30:(A0=0,A1=0)
 *   0x31:(A0=0,A1=1)
 *   0x32:(A0=1,A1=0)
 *   0x33:(A0=1,A1=1)
 */
DFRobot_BC20_IIC myBC20(0x33);
/******************HardwareSerial******************/
#elif defined(USE_HSERIAL)
/*
 * Connect Instructions
 * esp32      |               MEGA Series    |    Module(BC20)
 * IO17       |               D16(RX)        |       D/T
 * IO16       |               D17(TX)        |       C/R
 * GND        |               GND            |       GND
 * 5V(USB) or 3V3(battery)  | 5V or 3V3      |       VCC
 */
//For MEGA2560/ESP32 HardwareSerial
#if defined(ARDUINO_ESP32_DEV)
HardwareSerial Serial2(2);
DFRobot_BC20_Serial myBC20(&Serial2);//ESP32HardwareSerial
#else
DFRobot_BC20_Serial myBC20(&Serial1);//others
#endif
/******************SoftwareSerial******************/
#elif defined(USE_SSERIAL)
/*
 *  Connect Instructions
 *      UNO     |    Module(BC20)
 *    PIN_RXD   |       D/T
 *    PIN_TXD   |       C/R
 *      GND     |       GND
 *   5V or 3V3  |       VCC
*/
#define PIN_TXD   3
#define PIN_RXD   4
SoftwareSerial ss(PIN_TXD,PIN_RXD);
DFRobot_BC20_SW_Serial myBC20(&ss);
#endif

void callback(char * topic, uint8_t * payload, unsigned int len){
  Serial.print("Recevice [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < len; i++){
    Serial.print((char)payload[i]);
  }
  Serial.println();

  /**
   * Enable entering PSM.
   * When PSM is entered, BC20 will not receive any commands or signal from the moblie station (i.e. not controllable)
   * However, when during DRX/eDRX, BC20 will still response to AT commands or NB signal.
   */
  if (myBC20.configSleepMode(eSleepMode_DeepSleep)) {
    Serial.println("BC20 enturn PSM!");
  }  
  if(!myBC20.stmLowpower()){
    Serial.println("stm32 enturn PSM!");
  } 
}

void ConnectCloud(){
  while(!myBC20.connected()){
    Serial.println("Attempting MQTT connection...");
    if(myBC20.connect(Client_ID, Iot_id, Iot_pwd)){          
      Serial.println("Connect Server OK");
    }else{
      /**
       * Used to detect the connection between the device and the server
       */
      if(myBC20.getQMTCONN())
        break;
    }
  }
  myBC20.configSleepMode(eSleepMode_Disable);
  while(!myBC20.subTopic('0','1',subTopic,'0')){
    Serial.println("subTopicing...");
    delay(500);
  }
}

void setup(){
  Serial.begin(115200);
  Serial.println("Starting the BC20.Please wait. . . ");
  myBC20.changeColor(RED);
  while(!myBC20.powerOn()){
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);    
    Serial.print(".");
  }
  Serial.println("BC20 started successfully !");
  
  myBC20.changeColor(GREEN);
  while(!myBC20.checkNBCard()){
    Serial.println("Please insert the NB card !");
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);
  }
  Serial.println("Waitting for access ...");
  
  /**
   * For network connection, return 1 on success, 0 on failure
   */  
  myBC20.changeColor(BLUE);
  while(myBC20.getGATT()==0){
    Serial.print(".");
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);    
  }
  Serial.println("");
  Serial.println("access success!");
  Serial.println("Try to connect ...");

  /**
   * Use to connect to Internet of things sites
   */
  myBC20.setServer(EasyIot_SERVER,PORT);
  Serial.println("Server is available!");   
  ConnectCloud();
  Serial.println("Connect Cloud success!");
  myBC20.setCallback(callback);
  if (myBC20.configSleepMode(eSleepMode_DeepSleep)) {
    Serial.println("BC20 enturn PSM!");
  }  
  if(!myBC20.stmLowpower()){
    Serial.println("stm32 enturn PSM!");
  }  
}

void loop(){
    myBC20.loop(); 
}