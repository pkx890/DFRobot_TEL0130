/*!
 *@file BC20_DFOTA.ino
 *@  
 * @ This code demonstrates how to implement a firmware upgrade via OTA.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-11-18
 * @get from https://www.dfrobot.com
 */

#include "DFRobot_BC20.h"
#define USE_IIC
//#define USE_HSERIAL
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

void setup(){
  Serial.begin(115200);
  Serial.println("Starting the BC20.Please wait. . . ");
  while(!myBC20.powerOn()){
    delay(1000);
    myBC20.control_LED("LED_R_ON");
    delay(10);   
    myBC20.control_LED("LED_R_OFF"); 
    delay(10);    
    Serial.print(".");
  }
  Serial.println("BC20 started successfully !");
  while(!myBC20.checkNBCard()){
    Serial.println("Please insert the NB card !");
    delay(1000);
    myBC20.control_LED("LED_G_ON");
    delay(10);   
    myBC20.control_LED("LED_G_OFF"); 
    delay(10);    
  }
  Serial.println("Waitting for access ...");
  while(myBC20.getGATT()==0){
    Serial.print(".");
    delay(1000);
    myBC20.control_LED("LED_B_ON");
    delay(10);   
    myBC20.control_LED("LED_B_OFF"); 
    delay(10);    
  }
  Serial.println("");
  Serial.println("access success!");
  myBC20.configSleepMode(eSleepMode_Disable);
  myBC20.getESQ();
  myBC20.getEREG();
  myBC20.getGPADDR();
  myBC20.setQFOTADL("http://download3.dfrobot.com.cn/nbtest/Update0406.bin");
}
void loop(){
  if(Serial.available()){
    myBC20.sendATCMDBychar((char)Serial.read());
  }
  if(myBC20.available()){
    Serial.println(myBC20.readData());
  }
}
