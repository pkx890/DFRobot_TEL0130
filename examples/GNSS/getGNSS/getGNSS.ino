/*!   
 * @file getGNSS.ino
 * @brief Print all the GNSS info available in BC20.
 * @Compiling this DEMO with UNO will fail, requiring a more advanced master board
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [Peng Kaixing](kaixing.peng@dfrobot.com)
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
 * esp32      |    Module(BC20)
 * IO17       |       D/T
 * IO16       |       C/R
 * GND        |       GND
 * 5V(USB) or 3V3(battery)  |       VCC
 */

HardwareSerial Serial2(2);
DFRobot_BC20_Serial myBC20(&Serial2);//ESP32HardwareSerial

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

void Display_Location_Information(){
  
  /*
   * UTC time of the anchor point
   */
  Serial.print("Time:\t\t");
  Serial.print(sCLK.Year);
  Serial.print("/");
  Serial.printf("%02d",sCLK.Month);
  Serial.print("/");
  Serial.printf("%02d ",sCLK.Day);
  Serial.printf(" %02d",sCLK.Hour);
  Serial.printf(":%02d",sCLK.Minute);
  Serial.printf(":%02d\r\n",sCLK.Second);
    
  Serial.print("Latitude:\t");
  Serial.print(sGGNS.LatitudeVal,6);
  Serial.print(" deg ");
  Serial.println(sGGNS.LatitudeDir);
  Serial.print("Longitude:\t");
  Serial.print(sGGNS.LongitudeVal,6);
  Serial.print(" deg ");
  Serial.println(sGGNS.LongitudeDir);
  Serial.print("Altitude:\t");
  Serial.print(sGGNS.Altitude,1);
  Serial.println(" m");
  Serial.print("Speed:\t\t");
  Serial.print(sGGNS.Speed);
  Serial.println(" km/h");
  Serial.print("Heading:\t");
  Serial.print(sGGNS.Heading);
  Serial.println(" deg");
  Serial.print("Status:\t\t");
  Serial.println(sGGNS.FixStatus);
  Serial.print("PDOP:\t\t");
  Serial.println(sGGNS.PDOP);
  Serial.print("HDOP:\t\t");
  Serial.println(sGGNS.HDOP);
  Serial.print("VDOP:\t\t");
  Serial.println(sGGNS.VDOP);
  Serial.println();
}

void Display_Satellite_Information(){
  Serial.print(sSAT.NUM);
  Serial.println(" in view.");
  Serial.print(sSAT.USE);
  Serial.println(" in used.");
  Serial.print("PRN\t");
  Serial.print("Elev(deg)\t");
  Serial.print("Azim(deg)\t");
  Serial.print("SNR(dBHz)\t");
  Serial.print("SYS\t");
  Serial.println("Used");
  for(uint8_t i = 0; i <sSAT.NUM; i++){
    Serial.print(sSAT2.data[i].PRN);
    Serial.print("\t");
    Serial.print(sSAT2.data[i].Elev);
    Serial.print("\t\t");
    Serial.print(sSAT2.data[i].Azim);
    Serial.print("\t\t");
    Serial.print(sSAT2.data[i].SNR);
    Serial.print("\t\t");
    Serial.print(sSAT2.data[i].SYS);
    Serial.print("\t");
    Serial.println(sSAT2.data[i].Status);
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
  /**
   * Is used to obtain the specified satellite information, and the parameter is used to specify the type of information to be obtained. 
   * The parameter is selected as follows:
   */
  myBC20.getQGNSSRD();
  Display_Location_Information();
  Display_Satellite_Information();
  myBC20.clearGPS();
}