/*!
 * @file getVTG.ino
 * @brief Print all the VTG info available in BC20.
 * @n VTG: Course Over Ground and Ground Speed.
 * @n The actual course and speed relative to the ground
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-10-29
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
  Serial.print("Starting the BC20.Please wait. . . ");
  while(!myBC20.powerOn()){
    delay(1000);
    myBC20.controlLED("LED_R_ON");
    delay(10);   
    myBC20.controlLED("LED_R_OFF"); 
    delay(10);       
    Serial.print(".");
  }
  Serial.println("BC20 started successfully !");
  Serial.println("check OK");
  
/**
 * Used for module power control. If the return value is 1, the module is in the state of power supply; 
 * if the return value is 0, the module is in the state of power loss    
 */
  if(myBC20.getQGNSSC() == OFF){
    myBC20.LEDFlash("Y");
    Serial.println("open QGNSSC");
    myBC20.setQGNSSC(ON);
  }
}

void loop(){
  delay(1000);
  myBC20.controlLED("LED_B_ON");
  delay(100);
  myBC20.controlLED("LED_B_OFF");
  delay(100);
  myBC20.getQGNSSRD(NMEA_VTG);
  
/*
 * Course over ground (true), unit in degrees 
 */
  Serial.print("Course over ground (true): ");
  Serial.print(sVTG.GroundCourse_True());
  Serial.println(" deg");
  
/* 
 * Course over ground (magnetic), unit in degrees 
 */
  Serial.print("Course over ground (magnetic): ");
  Serial.print(sVTG.GroundCourse_Mag());
  Serial.println(" deg");
  
/*
 * Speed over ground, unit in knots 
 */
  Serial.print("Ground Speed (knots): ");
  Serial.print(sVTG.GroundCourse_Knots());
  Serial.println(" knots");
  
/*  
 * Speed over ground, unit in km/h 
 */
  Serial.print("Ground Speed (km/h): ");
  Serial.print(sVTG.GroundCourse_Kmh());
  Serial.println(" km/h");
  
/*
 * Positioning Mode
 * N - No fix
 * A - Autonomous GPS fix
 * D - Differential GPS fix 
 */
  Serial.print("Positioning Mode: ");
  Serial.println(sVTG.PositioningMode());
  Serial.println();
  Serial.println();
  myBC20.clearGPS();
}
