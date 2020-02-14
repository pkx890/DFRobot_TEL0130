/*!
 * @file LED.ino
 * @  
 * @ this demo Used to control the entry and exit of low-power mode STM32 low-power mode, and to control the led lights on and off and color
 * @
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-11-18
 * @get from https://www.dfrobot.com
 */
#include "DFRobot_BC20.h"
#define  RED 0
#define  BLUE 1
#define  GREEN 2
#define  YELLOW 3
#define  PURPLE 4
#define  CYAN 5
#define  WHITE 6

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

void setup() {
  Serial.begin(115200);
  Serial.print("Starting the BC20.Please wait. . . ");
  while(!myBC20.powerOn()){
    delay(1000);
    Serial.print(".");
  }
  Serial.println("BC20 started successfully !");
}
void loop() {
/**
 * Available colors:
 * RED;BLUE; 
 * GREEN;YELLOW 
 * PURPLE;CYAN 
 * WHITE
 */
  for(int j=0;j<7;j++){
    for(int i=0;i<5;i++){
      myBC20.LED_ON();
      delay(100);
      myBC20.LED_OFF();
      delay(100);
    }
    delay(5000);
    myBC20.changeColor(j);//Change the color to display 
  }
} 
