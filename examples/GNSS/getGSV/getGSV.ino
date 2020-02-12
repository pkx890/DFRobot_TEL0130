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

void Display_Satellite_Information(){	
    Serial.print(sSAT.NUM);
    Serial.println(" in view.");
    // Satellite PRN number
    Serial.print("PRN\t");
    // Elevation angle, unit in degrees
    Serial.print("Elev(deg)\t");
    // Azimuth angle, unit in degrees
    Serial.print("Azim(deg)\t");
    // Signal Noise Ratio, unit in dBHz
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
    if(myBC20.getQGNSSC() == OFF){
        myBC20.LED_flash("Y");
        Serial.println("open QGNSSC");
        myBC20.setQGNSSC(ON);
    }
}

void loop(){
    delay(5000);
    myBC20.control_LED("LED_B_ON");
    delay(100);
    myBC20.control_LED("LED_B_OFF");
    delay(100);		
    myBC20.getQGNSSRD(NMEA_GSV);
    Display_Satellite_Information();
    myBC20.clearGPS();
}
