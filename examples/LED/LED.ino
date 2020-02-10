/*!
 * @file BC20_LED.ino
 * @  
 * @ this demo Used to control the entry and exit of low-power mode STM32d low-power mode, and to control the led lights on and off and color
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

/*IIC_addr(A0,A1):0x30(A0=0,A1=0),0x31(A0=0,A1=1),0x32(A0=1,A1=0),0x33(A0=1,A1=1)*/
DFRobot_BC20_IIC myBC20(0x33);
//For general controllers. Communicate by IIC
/*
   Connect Instructions
      Controller     |    Module(BC20)
          SDA        |       D/T
          SCL        |       C/R
          GND        |       GND
       5V or 3V3     |       VCC

*/

/*HardwareSerial*/
//For MEGA2560/ESP32 HardwareSerial
//HardwareSerial Serial2(2);DFRobot_BC20_Serial myBC20(&Serial2);//ESP32HardwareSerial
//DFRobot_BC20_Serial myBC20(&Serial1);//others
/*
   Connect Instructions
   esp32      |               MEGA Series    |    Module(BC20)
   IO17       |               D16(RX)        |       D/T
   IO16       |               D17(TX)        |       C/R
   GND        |               GND            |       GND
   5V(USB) or 3V3(battery)  | 5V or 3V3      |       VCC
*/

/*SoftwareSerial*/
//#define PIN_TXD   3
//#define PIN_RXD   4
//SoftwareSerial ss(PIN_TXD,PIN_RXD);
//DFRobot_BC20_SW_Serial myBC20(&ss);
/*
   Connect Instructions
        UNO     |    Module(BC20)
      PIN_RXD   |       D/T
      PIN_TXD   |       C/R
        GND     |       GND
     5V or 3V3  |       VCC
*/

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
  for(int i=0;i<5;i++)
  {
	myBC20.LED_ON();//turn on the light,Default is white
	delay(100);
	myBC20.LED_OFF();//turn off the light
	delay(100);
  }
  delay(5000);
  myBC20.changeColor(RED);//Change the color to display 
  /*Available colors:
  RED 
  BLUE 
  GREEN 
  YELLOW 
  PURPLE 
  CYAN 
  WHITE*/ 
   for(int i=0;i<5;i++)
  {
	myBC20.LED_ON();
	delay(100);
	myBC20.LED_OFF();
	delay(100);
  }
  delay(5000);
  myBC20.changeColor(BLUE);
  for(int i=0;i<5;i++)
  {
	myBC20.LED_ON();
	delay(100);
	myBC20.LED_OFF();
	delay(100);
  }
  delay(5000);
  myBC20.changeColor(GREEN);
  for(int i=0;i<5;i++)
  {
	myBC20.LED_ON();
	delay(100);
	myBC20.LED_OFF();
	delay(100);
  }
  delay(5000);
  myBC20.changeColor(YELLOW);
  for(int i=0;i<5;i++)
  {
	myBC20.LED_ON();
	delay(100);
	myBC20.LED_OFF();
	delay(100);
  }
  delay(5000);
  myBC20.changeColor(PURPLE);
  for(int i=0;i<5;i++)
  {
	myBC20.LED_ON();
	delay(100);
	myBC20.LED_OFF();
	delay(100);
  }
  delay(5000);
  myBC20.changeColor(CYAN);
  for(int i=0;i<5;i++)
  {
	myBC20.LED_ON();
	delay(100);
	myBC20.LED_OFF();
	delay(100);
  }
  delay(5000);
  myBC20.changeColor(WHITE);
  for(int i=0;i<5;i++)
  {
	myBC20.LED_ON();
	delay(100);
	myBC20.LED_OFF();
	delay(100);
  } 
  delay(5000);
} 
