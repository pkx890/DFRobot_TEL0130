/*!
 * @file getRMC.ino
 * @brief Print all the RMC info available in BC20.
 * @n RMC: Recommended Minimum Position Data (including position, velocity and time)
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-10-29
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
  
  myBC20.configSleepMode(eSleepMode_Disable);
  
  Serial.println("check OK");
  myBC20.changeColor(YELLOW);
  /**
   * Used for module power control. If the return value is 1, the module is in the state of power supply; 
   * if the return value is 0, the module is in the state of power loss    
   */  
  if(myBC20.getQGNSSC() == OFF){
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);
    myBC20.setQGNSSC(ON);
  }
  Serial.println("open QGNSSC");  
  myBC20.changeColor(CYAN);
}

void loop(){
  myBC20.LED_ON();
  delay(500);
  myBC20.LED_OFF();
  delay(5000);	
  myBC20.getQGNSSRD(NMEA_RMC);
  
  /*
   * UTC Date, format: ddmmyy, ex. 290519 = 2019/05/29
   */
  Serial.print("UTC Date: ");
  Serial.println(sRMC.UTC_Date());
  
  /*
   * UTC time, format: hhmmss.ss, ex. 162436.54 = 16:24:36.54
   * h - hours
   * m - minutes
   * s - seconds
   */
  Serial.print("UTC Time: ");
  Serial.println(sRMC.UTC_Time());
  
  /*
   * DataStatus
   * V - Invalid
   * A - Valid
   */
  Serial.print("Data Status: ");
  Serial.println(sRMC.DataStatus());
  
  /*
   * Latitude, format: ddmm.mmmmm, ex. 3150.7820 => 31deg 50.7820min
   * d - degrees
   * m - minutes
   */
  Serial.print("Latitude: ");
  Serial.print(sRMC.LatitudeVal());
  Serial.print(" ");
  /*
   * Latitude north or south
   * N - North
   * S - South
   */
  Serial.println(sRMC.LatitudeDir());
  
  /*
   * Longitude, format: dddmm.mmmmm, ex. 12135.6794 => 121deg 35.6794min
   * d - degrees
   * m - minutes
   */
  Serial.print("Longitude: ");
  Serial.print(sRMC.LongitudeVal());
  Serial.print(" ");
  
  /*
   * Longitude east or west
   * E - East
   * W - West
   */
  Serial.println(sRMC.LongitudeDir());
  
  /*
   * Ground Speed, speed over ground, unit in knots
   */
  Serial.print("Ground Speed: ");
  Serial.print(sRMC.GroundSpeed());
  Serial.println(" knots");
  
  /* 
   * Ground Heading, heading over ground, unit in degrees 
   */
  Serial.print("Ground Heading: ");
  Serial.println(sRMC.GroundHeading());
  
  /*
   * Magnetic Declination, unit in degrees 
   */
  Serial.print("Magnetic Declination: ");
  Serial.println(sRMC.MagDeclination());
  
  /* 
   * Magnetic Declination Direction, Magnetic declination E/W indicator 
   */
  Serial.print("Magnetic Declination Direction: ");
  Serial.println(sRMC.MagDeclinationDir());
  
  /*   
   * Positioning Mode
   * N - No fix
   * A - Autonomous GPS fix
   * D - Differential GPS fix 
   */
  Serial.print("Positioning Mode: ");
  Serial.println(sRMC.PositioningMode());
  
  /*
   * Navigation Status
   * V - Invalid 
   */
  Serial.print("Navigation Status: ");
  Serial.println(sRMC.NaviStatus());
  Serial.println();
  Serial.println();
  myBC20.clearGPS();
}
