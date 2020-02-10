/*!   
 * @file getGLL.ino
 * @brief Print all the GLL info available in BC20.
 * @n GLL: Geographic Latitude and Longitude.   
 * @n It contains position information, time of position fix and status.
 * 
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-10-29
 * @get from https://www.dfrobot.com
 */
#include "DFRobot_BC20.h"

/*IIC_addr(A0,A1):0x30(A0=0,A1=0),0x31(A0=0,A1=1),0x32(A0=1,A1=0),0x33(A0=1,A1=1)*/
//DFRobot_BC20_IIC myBC20(0x33);
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

void setup(){
  Serial.begin(115200);
  Serial.print("Starting the BC20.Please wait. . . ");
  while(!myBC20.powerOn()){
    delay(1000);
    myBC20.control_LED("LED_R_ON");
    delay(10);   
    myBC20.control_LED("LED_R_OFF"); 
    delay(10);    
    Serial.print(".");
  }
  Serial.println("BC20 started successfully !");
  if(myBC20.getQGNSSC() == 0){
	myBC20.LED_flash("Y");
    Serial.println("open QGNSSC");
    myBC20.setQGNSSC(1);
  }
}

void loop(){
  delay(3000);
  myBC20.control_LED("LED_B_ON");
  delay(100);
  myBC20.control_LED("LED_B_OFF");
  delay(100);
  myBC20.getQGNSSRD(NMEA_GLL);
  // UTC time, format: hhmmss.ss, ex. 162436.54 => 16:24:36.54
  // h - hours
  // m - minutes
  // s - seconds
  Serial.print("UTC Time: ");
  Serial.println(sGLL.UTC_Time());
  
  // Latitude, format: ddmm.mmmmm, ex. 3150.7820 => 31deg 50.7820min
  // d - degrees 
  // m - minutes
  Serial.print("Latitude: ");  
  Serial.print(sGLL.LatitudeVal());
  // Latitude north or south
  // N - North
  // S - South
  Serial.print(" ");
  Serial.println(sGLL.LatitudeDir());
   
  // Longitude, format: dddmm.mmmmm, ex. 12135.6794 = 121deg 35.6794min
  // d - degrees
  // m - minutes
  Serial.print("Longitude: ");
  Serial.print(sGLL.LongitudeVal());
  Serial.print(" ");
  // Longitude wast or west
  // E - East
  // W - West
  Serial.println(sGLL.LongitudeDir());
  
  // DataStatus
  // V - Invalid
  // A - Valid  
  Serial.print("Data Status: ");
  Serial.println(sGLL.DataStatus());

  // Positioning Mode
  // N - No fix
  // A - Autonomous GPS fix
  // D - Differential GPS fix
  Serial.print("Positioning Mode: ");
  Serial.println(sGLL.PositionMode());
  Serial.println();
  Serial.println();
  myBC20.clearGPS();
}
