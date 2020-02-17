/*!
 * @file BC20_Serial.ino
 * @brief After the program download is complete,
 * @brief AT commands can be sent to the BC20 module via USB Serial.
 * @n Commonly used AT commands:
 * @n  AT - AT command test
 * @n  AT+QRST=1 - Reset BC20
 * @n  ATI - Revision of the firmware release
 * @n  AT+CSQ - Signal quality report
 * @n      0 - <=-113 dBm
 * @n      1 - -111 dBm
 * @n      2 - -109 dBm
 * @n      3 - -107 dBm
 * @n      ...
 * @n      30 - -53 dBm
 * @n      31 - >-51 dBm
 * @n     99 - Not known or not detectable
 * @n AT+CGATT? - PS attach or detach. Query network connection state.
 * @n     0 - Disconnected from the network
 * @n     1 - Connected to the network
 * @n AT+CGATT=1 - Connect to the network
 * @n AT+CGATT=0 - Disconnected from the network
 * @n AT+CIMI - Query the IMSI number of BC20
 * @n AT+CGSN=1 - Query the IMEI of the BC20.
 * @n AT+CGSN=0 - Query the SN(Serial Number) of the BC20.
 * @n AT+QCCID - USIM Card Identification(ICCID). This is usually used to check SIM card state.
 * @n AT+CCLK? - Return current date and time
 * @n AT+QPOWD=0 - Power off the module.(use "myBC20.powerOn()" to power on the module)
 *
 * @n The following AT commands are for GNSS:
 * @n AT+QGNSSC? - Query GNSS power state
 * @n     0 - GNSS power off
 * @n     1 - GNSS power on
 * @n AT+QGNSSC=1 - power on GNSS
 * @n AT+QGNSSC=0 - power off GNSS
 * @n AT+QGNSSRD? - Obtain all GNSS info
 *
 * @n For more details please refer to the "Quectel BC20 AT Commands Manual"
 * @n or the "BC20 GNSS AT Commands Manual"
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date   2019-10-29
 * @get from https://www.dfrobot.com
 */
#include "DFRobot_BC20.h"

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
  Serial.print("Starting the BC20.Please wait. . . ");
  while(!myBC20.powerOn()){
    delay(1000);
    Serial.print(".");
    myBC20.controlLED("LED_R_ON");
    delay(500);   
    myBC20.controlLED("LED_R_OFF"); 
    delay(500); 
  }
  Serial.println("BC20 started successfully !");
  myBC20.configSleepMode(eSleepMode_Disable);
  /**  
    * Deep Sleep Mode is automatically enable every time upon power up.
    * When this mode is entered, BC20 will not respond any AT commands from ESP32
    * myBC20.ConfigSleepMode(eSleepMode_Disable);
    * Each AT command should begin with "AT" or "at" and end with "Carriage return".
    * The commands can be upper-case or lower-case. ex. "AT+CSQ" or "at+csq".
    * Serial.println("Enter AT commands:");
   */
}
void loop(){
  /**
   * Receive data when it comes in and send it in characters when it needs to be sent
   */
  if(Serial.available()){
    myBC20.sendATCMDBychar((char)Serial.read());
  }
  if(myBC20.available()){
    Serial.println(myBC20.readData());
  }
}
