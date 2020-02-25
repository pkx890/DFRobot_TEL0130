/*!
   @file getGNSS.ino
   @brief Print all the GNSS info available in BC20.
   @Compiling this DEMO DOSE NOT work on Arduino UNO, requiring a Dev. board with more RAM.
   @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
   @licence     The MIT License (MIT)
   @author      [Peng Kaixing](kaixing.peng@dfrobot.com)
   @version  V1.0
   @date  2019-10-29
   @get from https://www.dfrobot.com
*/
#include "DFRobot_BC20.h"

/* 7 colors are available */
#define  RED 0
#define  BLUE 1
#define  GREEN 2
#define  YELLOW 3
#define  PURPLE 4
#define  CYAN 5
#define  WHITE 6

/*Communication by IIC*/
#define USE_IIC

/*Communication by SoftwareSerial*/
//#define USE_SSERIAL


/******************IIC******************/
#ifdef USE_IIC
/*
   For general controllers. Communicate by IIC
   Connect Instructions
      Controller     |    Module(BC20)
          SDA        |       D/T
          SCL        |       C/R
          GND        |       GND
       5V or 3V3     |       VCC

   IIC address(A0,A1)
     0x30:(A0=0,A1=0)
     0x31:(A0=0,A1=1)
     0x32:(A0=1,A1=0)
     0x33:(A0=1,A1=1) default
*/
DFRobot_BC20_IIC myBC20(0x33);

/******************SoftwareSerial******************/
#elif defined(USE_SSERIAL)
/*
    For Arduino Series SoftwareSerial
    Connect Instructions
        UNO     |    Module(BC20)
      PIN_RXD   |       D/T
      PIN_TXD   |       C/R
        GND     |       GND
     5V or 3V3  |       VCC
*/
#define PIN_TXD   3
#define PIN_RXD   4
SoftwareSerial ss(PIN_TXD, PIN_RXD);
DFRobot_BC20_SW_Serial myBC20(&ss);
#endif

void Display_Location_Information() {

  /*
     UTC time of the anchor point
  */
  Serial.print("Time:\t\t");
  Serial.print(sCLK.Year);
  Serial.print("/");
  Serial.print(sCLK.Month);
  Serial.print("/");
  Serial.print(sCLK.Day);
  Serial.print("  ");
  Serial.print(sCLK.Hour);
  Serial.print("：");
  Serial.print(sCLK.Minute);
  Serial.print("：");
  Serial.println(sCLK.Second);

  Serial.print("Latitude:\t");
  Serial.print(sGGNS.LatitudeVal());
  Serial.println(sGGNS.LatitudeDir());
  Serial.print("Longitude:\t");
  Serial.print(sGGNS.LongitudeVal());
  Serial.println(sGGNS.LongitudeDir());
  Serial.print("Altitude:\t");
  Serial.print(sGGNS.Speed());
  Serial.println(" km/h");
}

void setup() {
  Serial.begin(115200);
  myBC20.LED_OFF();

  /*Initialize BC20*/
  Serial.print("Starting the BC20.Please wait. . . ");
  myBC20.changeColor(RED);
  while (!myBC20.powerOn()) {
    Serial.print(".");
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);
  }
  Serial.println("BC20 started successfully!");

  /* Disable sleep mode */
  myBC20.configSleepMode(eSleepMode_Disable);

  /*Power up GNSS*/
  Serial.print("Turning on GNSS ... ");
  myBC20.setQGNSSC(ON);
  myBC20.changeColor(YELLOW);
  if (myBC20.getQGNSSC() == OFF) {
    Serial.print(".");
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);
  }
  Serial.println("GNSS is ON.");
  myBC20.changeColor(CYAN);
}

void loop() {
  myBC20.getQGNSSRD();
  Display_Location_Information();
  myBC20.clearGPS();

  myBC20.LED_ON();
  delay(500);
  myBC20.LED_OFF();
  delay(5000);
}