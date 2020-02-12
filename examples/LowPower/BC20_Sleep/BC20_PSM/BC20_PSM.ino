/*!
 * @file BC20_PSM.ino
 *
 * @brief This codes demo how to configure PSM and eDRX for low power application.
 * @brief After waking STM32 with an external interrupt IRQ, wake BC20
 *
 * @n AT commands can be sent to the BC20 module via USB Serial.
 * @n Commonly used AT commands:
 * @n    AT - AT command test
 * @n    AT+QRST=1 - Reset BC20
 * @n    ATI - Revision of the firmware release
 * @n    AT+CSQ - Signal quality report
 * @n        0 - <=-113 dBm
 * @n        1 - -111 dBm
 * @n        2 - -109 dBm
 * @n        3 - -107 dBm
 * @n        ...
 * @n        30 - -53 dBm
 * @n        31 - >-51 dBm
 * @n        99 - Not known or not detectable
 * @n    AT+CGATT? - PS attach or detach. Query network connection state.
 * @n        0 - Disconnected from the network
 * @n        1 - Connected to the network
 * @n    AT+CGATT=1 - Connect to the network
 * @n    AT+CGATT=0 - Disconnected from the network
 * @n    AT+CIMI - Query the IMSI number of BC20
 * @n    AT+CGSN=1 - Query the IMEI of the BC20.
 * @n    AT+CGSN=0 - Query the SN(Serial Number) of the BC20.
 * @n    AT+QCCID - USIM Card Identification(ICCID). This is usually used to check SIM card state.
 * @n    AT+CCLK? - Return current date and time
 * @n    AT+QPOWD=0 - Power off the module.(use "myBC20.powerOn()" to power on the module)
 *
 * @n    The following AT commands are for GNSS:
 * @n    AT+QGNSSC? - Query GNSS power state
 * @n        0 - GNSS power off
 * @n        1 - GNSS power on
 * @n    AT+QGNSSC=1 - power on GNSS
 * @n    AT+QGNSSC=0 - power off GNSS
 * @n    AT+QGNSSRD? - Obtain all GNSS info
 *
 * @n For more details please refer to the "Quectel BC20 AT Commands Manual"
 * @n or the "BC20 GNSS AT Commands Manual"
 *
 * @n One of the key feature of NB-IoT is power saving.
 * @n This is achieved mainly by putting the CPU and modem of BC20 into different operating mode.
 *
 * @n CPU modes (Power from high to low): Active > Light Sleep > Deep Sleep
 * @n Modem modes(Power from high to low): Connected > Idle > PSM
 *
 * @n It is clear that BC20 achieves lowest power when CPU enters Deep Sleep mode and Modem enters PSM.
 *
 * @n BC20 enters PSM when ALL the following requirements are met:
 * @n 1. Network is connected. (SIM is inserted)
 * @n 2. PSM is enabled (by calling "myBC20.setPSMMode(ePSM_ON)") and properly configured.
 *
 * @n BC20 enters Deep Sleep Mode when ONE OF the following requirements are met:
 * @n 1. AT+CFUN=0.
 * @n 2. PSM is entered.
 * @n 3. eDRX is set to 81.92s or above.
 * @n 
 * @n BC20 enters Light Sleep Mode when the following requirements are met:
 * @n 1. Light Sleep Mode is enabled (by calling "myBC20.ConfigSleepMode(eSleepMode_Light)")
 *
 * @n Important terms and abbreviations
 * @n RSSI: Received Signal Strength Indication
 * @n ICCID: Integrate Circuit Card Identity
 * @n IMSI:International Mobile Subscriber Identity
 * @n IMEI:International Mobile Equipment Identity
 *
 * @n PSM: Power Saving Mode
 * @n DRX: Discontinuous Reception
 * @n eDRX: extended DRX
 * @n TAU: Tracking Area Update
 * @n PTW: Paging Time Window
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

//To wake STM32, the IRQ pin in the NB module is connected to the wakeup_pin
#define wakeup_pin 7

String tempdata="";
int nowTime=0;
void setup() {
  Serial.begin(115200);
  Serial.println("Starting the BC20.Please wait. . . ");
  //Power on BC20.
  while (!myBC20.powerOn()) {
    delay(1000);
    myBC20.control_LED("LED_R_ON");
    delay(10);   
    myBC20.control_LED("LED_R_OFF"); 
    delay(10);    
    Serial.print(".");
  }
  Serial.println("BC20 started successfully !");

  //Check whether a NB-IoT SIM card is available.
  while (!myBC20.checkNBCard()) {
    Serial.println("Please insert the NB SIM card !");
    delay(1000);
    myBC20.control_LED("LED_G_ON");
    delay(10);   
    myBC20.control_LED("LED_G_OFF"); 
    delay(10);    
  }
  myBC20.getGSN(IMEI);
  Serial.print("BC20 IMEI: ");
  Serial.println(sGSN.imei);
  Serial.print("SIM card ICCID:");
  Serial.println(myBC20.getQCCID());
  Serial.print("SIM card IMSI: ");
  Serial.println((char *)myBC20.getIMI());
  Serial.println("Connecting network ");

  //Check whether it is attached to the network
  //BC20 will automatically connect and register on network after power on
  while (myBC20.getGATT() == 0) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("Network connected!");

  //Turn on PSM mode
  if (myBC20.setPSMMode(ePSM_ON)) {
    Serial.println("set psm OK");
  }
  //Turn off PSM by using the following line (PSM is ON by default)
  //myBC20.setPSMMode(ePSM_OFF);

  //BC20 serial print "QATWAKEUP" when it is woken up from PSM
  if (myBC20.setQATWAKEUP(ON)) {
    Serial.println("set QATWAKEUP\r\n");
  }
  /*
   *  Deep/Light Sleep Mode.
   *
   *  When in deep sleep mode, BC20 will not respond to any AT commands from the master panel
   *  Until it is awakened by STM32 (again, BC20 exits the PSM)
   *
   *  When Light Sleep Mode is entered, BC20 can still receive AT commands from ESP32.
   *  However, it's power consumption is greater than the the former.
   *
   */
   
  if (myBC20.configSleepMode(eSleepMode_DeepSleep)) {
    Serial.println("enable sleep");
  }
  //STM32 enters low power mode
  myBC20.stmLowpower();
  nowTime=millis();
  //Each AT command should begin with "AT" or "at" and end with "Carriage return".
  //The commands can be upper-case or lower-case. ex. "AT+CSQ" or "at+csq".
  Serial.println("Enter AT commands:");
}

void loop() {
  //STM32 and BC20 will go into low power mode every 5 seconds
  if(millis()-nowTime>5000){
    Serial.println("Entering PSM!");
    myBC20.configSleepMode(eSleepMode_DeepSleep);
    myBC20.stmLowpower();
    nowTime=millis();
    }
    if(Serial.available()){
      tempdata+=(char)Serial.read();
    }
    if(tempdata.length()>0){
      myBC20.sendATCMD(tempdata);
      tempdata="";
    }
    if(myBC20.available()){
      Serial.println(myBC20.readData());
    } 
//Provide the IRQ with a rising edge pulse
    myBC20.stmAwake(wakeup_pin);
    Serial.println("Wake up from PSM!");
    if(myBC20.BC20WakeUp()){
      Serial.println("quit PSM success!");
    }else{
      Serial.println("quit PSM fail!");
    }    
    delay(1000);
}