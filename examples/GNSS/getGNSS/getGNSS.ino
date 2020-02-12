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

void Display_Location_Information(){
  
    //UTC time of the anchor point
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
    Serial.print("Starting the BC20.Please wait. . . ");
    while(!myBC20.powerOn()){
        delay(1000);
        Serial.print(".");
    }
    Serial.println("BC20 started successfully !");
    Serial.println("check OK");
    if(myBC20.getQGNSSC() == OFF){
        Serial.println("open QGNSSC");
        myBC20.setQGNSSC(ON);
    }
}

void loop(){
    delay(5000);
    myBC20.getQGNSSRD();
    Display_Location_Information();
    Display_Satellite_Information();
    myBC20.clearGPS();
}