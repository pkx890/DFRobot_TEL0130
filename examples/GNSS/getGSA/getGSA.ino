  /*!
 * @file getGSA.ino
 * @brief Print all the GSA info available in BC20.
 * @n GSA: GPS DOP and Active Satellites
 * @n It provides details on the fix, including the numbers of the satellites 
 * @n being used and the DOP. At most the first 12 satellite IDs are output.
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
  delay(3000);
  myBC20.controlLED("LED_B_ON");
  delay(100);
  myBC20.controlLED("LED_B_OFF");
  delay(100);
  myBC20.getQGNSSRD(NMEA_GSA);
  Serial.print("Mode\t");
  Serial.print("Fix Status\t");
  for(uint8_t i = 0; i < 12; i++){
    Serial.print("CH");
    Serial.print(i+1);
    Serial.print("\t");
  }
  Serial.print("PDOP\t");
  Serial.print("HDOP\t");
  Serial.print("VDOP\t");
  Serial.print("System ID\t");
  Serial.println();
  for(uint8_t i =0; i < sGSA.NUM; i++){        
/*  
 * Selection of 2D or 3D fix
 * 'M'=Manual, forced to switch 2D/3D mode
 * 'A'=Allowed to automatically switch 2D/3D mode
 */
  Serial.print(sGSA.data[i].Mode());
  Serial.print("\t");
/*
 * Fix Status
 * 1 - No fix
 * 2 - 2D fix
 * 3 - 3D fix
 */
  String a=sGSA.data[i].FixStatus();
  Serial.print(sGSA.data[i].FixStatus());
  Serial.print("\t\t");
  
/*
 * A list of satellite being used on each channel
 */
  for(uint8_t j =0; j < 12; j++){
      Serial.print(sGSA.data[i].Statellite_CH(j));
      Serial.print("\t");
  }
  
/*
 * PDOP: Position Dilution Of Precision
 */
  Serial.print(sGSA.data[i].PDOP());
  Serial.print("\t");
  
/*
 * HDOP: Horizontal Dilution Of Precision
 */
  Serial.print(sGSA.data[i].HDOP());
  Serial.print("\t");
  
/*
 * VDOP: Vertical Dilution Of Precision
 */
  Serial.print(sGSA.data[i].VDOP());
  Serial.print("\t");
  
/*
 * GNSS system ID
 * 1 - GPS
 * 2 - GLONASS (unavailable)
 * 3 - Galileo (unavailable)
 * 4 - BeiDou
 */
  Serial.println(sGSA.data[i].GNSS_SystemID());
  }
  myBC20.clearGPS();
}
