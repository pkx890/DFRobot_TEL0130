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
    Serial.println("check OK");
    if(myBC20.getQGNSSC() == OFF){
		myBC20.LED_flash("Y");
        Serial.println("open QGNSSC");
        myBC20.setQGNSSC(ON);
    }
}

void loop(){
    delay(3000);
    myBC20.control_LED("LED_B_ON");
    delay(100);
    myBC20.control_LED("LED_B_OFF");
    delay(100);	
    myBC20.getQGNSSRD(NMEA_RMC);
    
    // UTC Date, format: ddmmyy, ex. 290519 = 2019/05/29
    Serial.print("UTC Date: ");
    Serial.println(sRMC.UTC_Date());
    // UTC time, format: hhmmss.ss, ex. 162436.54 = 16:24:36.54
    // h - hours
    // m - minutes
    // s - seconds
    Serial.print("UTC Time: ");
    Serial.println(sRMC.UTC_Time());
    
    // DataStatus
    // V - Invalid
    // A - Valid
    Serial.print("Data Status: ");
    Serial.println(sRMC.DataStatus());
    
    // Latitude, format: ddmm.mmmmm, ex. 3150.7820 => 31deg 50.7820min
    // d - degrees
    // m - minutes
    Serial.print("Latitude: ");
    Serial.print(sRMC.LatitudeVal());
    Serial.print(" ");
    // Latitude north or south
    // N - North
    // S - South
    Serial.println(sRMC.LatitudeDir());

    // Longitude, format: dddmm.mmmmm, ex. 12135.6794 => 121deg 35.6794min
    // d - degrees
    // m - minutes
    Serial.print("Longitude: ");
    Serial.print(sRMC.LongitudeVal());
    Serial.print(" ");
    // Longitude east or west
    // E - East
    // W - West
    Serial.println(sRMC.LongitudeDir());
    // Ground Speed, speed over ground, unit in knots
    Serial.print("Ground Speed: ");
    Serial.print(sRMC.GroundSpeed());
    Serial.println(" knots");

    // Ground Heading, heading over ground, unit in degrees
    Serial.print("Ground Heading: ");
    Serial.println(sRMC.GroundHeading());

    // Magnetic Declination, unit in degrees
    Serial.print("Magnetic Declination: ");
    Serial.println(sRMC.MagDeclination());
    
    // Magnetic Declination Direction, Magnetic declination E/W indicator
    Serial.print("Magnetic Declination Direction: ");
    Serial.println(sRMC.MagDeclinationDir());

    // Positioning Mode
    // N - No fix
    // A - Autonomous GPS fix
    // D - Differential GPS fix
    Serial.print("Positioning Mode: ");
    Serial.println(sRMC.PositioningMode());
    
    // Navigation Status
    // V - Invalid
    Serial.print("Navigation Status: ");
    Serial.println(sRMC.NaviStatus());
    Serial.println();
    Serial.println();
	myBC20.clearGPS();
}
