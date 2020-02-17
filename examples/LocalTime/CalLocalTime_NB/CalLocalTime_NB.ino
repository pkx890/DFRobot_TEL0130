/*!
 * @file CalLocalTime_NB.ino
 *
 * @n After the program download is complete,
 * @n Update local time with NB-IoT
 * @n Print out the current time
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @date  2019-07-18
 * @get from https://www.dfrobot.com
 */
#include "DFRobot_BC20.h"

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

void printLocalTime()
{
  //Date
  Serial.print((String)sCLK.Year + "/" + (String)sCLK.Month + "/" + (String)sCLK.Day + " ");
  //Time
  Serial.println((String)sCLK.Hour + ":" + (String)sCLK.Minute + ":" + (String)sCLK.Second);  
  
  while( myBC20.getCLK()){
    if(sCLK.Year > 2000){
      break;
    }
    Serial.print(".");
    myBC20.controlLED("LED_P_ON");
    delay(500);
    myBC20.controlLED("LED_P_OFF");
    delay(500);    
    delay(1000);
  }  
}

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting the BC20.Please wait. . . ");
  while(!myBC20.powerOn()){
    delay(1000);
    Serial.print(".");
    myBC20.controlLED("LED_G_ON");
    delay(500);
    myBC20.controlLED("LED_G_OFF");
    delay(500);    
  }
  Serial.println("BC20 started successfully !");
  
  while(!myBC20.checkNBCard()){
    Serial.println("Please insert the NB SIM card !");
    myBC20.controlLED("LED_Y_ON");
    delay(500);
    myBC20.controlLED("LED_Y_OFF");
    delay(500);    
    delay(1000);
  }

  /**
   * Print IMEI, ICCID and IMSI
   */    
  myBC20.getGSN(IMEI);
  Serial.print("BC20 IMEI: ");
  Serial.println(sGSN.imei);
  Serial.print("SIM card ICCID:");
  Serial.println(myBC20.getQCCID());
  Serial.print("SIM card IMSI: ");
  Serial.println((char *)myBC20.getIMI());

  /** 
   * The module will automatically attempt to connect to the network (mobile station).
   * Check whether it is connected to the network. 
   */  
  Serial.println("Waitting for access ...");
  while(myBC20.getGATT() == 0){
    Serial.print(".");
    myBC20.controlLED("LED_G_ON");
    delay(500);
    myBC20.controlLED("LED_G_OFF");
    delay(500);    
    delay(1000);
  }
  
  Serial.println("Waiting for NB time...");
  /**
   * Get system time
   */  
  while( myBC20.getCLK()){
    if(sCLK.Year > 2000){
      break;
    }
    Serial.print(".");
    myBC20.controlLED("LED_P_ON");
    delay(500);
    myBC20.controlLED("LED_P_OFF");
    delay(500);    
    delay(1000);
  }
  Serial.println();
  Serial.println("Configure local time");
}

void loop()
{
  printLocalTime();
  delay(1000);
}
