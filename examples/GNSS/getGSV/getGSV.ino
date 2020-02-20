/*!
 * @file getGSV.ino
 *
 * @brief Print all the GSV info available in BC20.
 *
 * @n GSV: GPS Satellites in View.
 * @n Since GSV includes satellites that are not used as
 * @n part of the solution, GSV contains more satellites than GGA does.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-10-29
 * @get from https://www.dfrobot.com
 */

#include "DFRobot_BC20.h"
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

void Display_Satellite_Information(){	
  Serial.print(sSAT.NUM);
  Serial.println(" in view.");
  /**
   * Satellite PRN number
   */
  Serial.print("PRN\t");
  /**
   * Elevation angle, unit in degrees
   */
  Serial.print("Elev(deg)\t");
  /**
   * Azimuth angle, unit in degrees
   */
  Serial.print("Azim(deg)\t");
  /**
   * Signal Noise Ratio, unit in dBHz
   */
  Serial.print("SNR(dBHz)\t");
  Serial.println("SYS");
  for(uint8_t i = 0; i <sSAT.NUM; i++){
    Serial.print(sSAT.data[i].PRN());
    Serial.print("\t");
    Serial.print(sSAT.data[i].Elev());
    Serial.print("\t\t");
    Serial.print(sSAT.data[i].Azim());
    Serial.print("\t\t");
    Serial.print(sSAT.data[i].SNR());
    Serial.print("\t\t");
    Serial.println(sSAT.data[i].SYS());
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
  myBC20.getQGNSSRD(NMEA_GSV);
  Display_Satellite_Information();
  myBC20.clearGPS();
}
