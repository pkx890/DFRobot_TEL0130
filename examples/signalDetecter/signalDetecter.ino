/*!
 * @file signalDetecter.ino
 *
 * @n Commonly used AT commands:
 * @n AT - AT command test
 * @n AT+QRST=1 - Reset BC20
 * @n ATI - Revision of the firmware release
 * @n AT+CSQ - Signal quality report
 * @n      0 - <=-113 dBm
 * @n      1 - -111 dBm
 * @n      2 - -109 dBm
 * @n      3 - -107 dBm
 * @n      ...
 * @n      30 - -53 dBm
 * @n      31 - >-51 dBm
 * @n      99 - Not known or not detectable
 * @n AT+CGATT? - PS attach or detach. Query network connection state.
 * @n      0 - Disconnected from the network
 * @n      1 - Connected to the network
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
 * @n      0 - GNSS power off
 * @n      1 - GNSS power on
 * @n AT+QGNSSC=1 - power on GNSS
 * @n AT+QGNSSC=0 - power off GNSS
 * @n AT+QGNSSRD? - Obtain all GNSS info
 *
 * @n For more details please refer to the "Quectel BC20 AT Commands Manual"
 * @n or the "BC20 GNSS AT Commands Manual"
 *
 * @Copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [Peng kaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date   2019-12-18
 * @get from https://www.dfrobot.com
 */

#include <DFRobot_BC20.h>

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

static void NB_Signal_Fun() {
/*
 * Introduction:
 * Turn on the BC20 and automatically connect to the network,
 * print the signal strength through the serial port,
 * and indicate the signal strength (high, medium, low) by the blinking frequency of the RGB or L indicator.
 * RGB:
 * When NB is connected, the signal strength is indicated by the frequency of the flashing light,
 * which divided into three steps:
 * Strong signal - fast flash
 * Medium signal - slow flash
 * Weak signal - burst flash.
 */
  Serial.println("NB-IoT signal detection mode.");
  Serial.println("Long press SET for over 1 sec to start...");
  Serial.print("Starting the BC20. Please wait... ");
  while (!myBC20.powerOn()) {
    myBC20.LEDFlash("R");
    Serial.print(".");
  }
  Serial.println("BC20 started successfully !");
  
  //Check whether a NB-IoT SIM card is available.
  while (!myBC20.checkNBCard()) {
    myBC20.LEDFlash("G");
    Serial.println("Please insert the NB SIM card !");
    delay(1000);
  }

/**
 * Get the serial number of the NB card
 */
  myBC20.getGSN(IMEI);
  Serial.print("BC20 IMEI: ");
  Serial.println(sGSN.imei);
  Serial.print("SIM card ICCID:");
  Serial.println(myBC20.getQCCID());
  Serial.print("SIM card IMSI: ");
  Serial.println((char *)myBC20.getIMI());
  Serial.println("Connecting network ");

/*
 * Check whether it is attached to the network
 * BC20 will automatically connect and register on network after power on
 */
  while (myBC20.getGATT() == 0) {
    myBC20.LEDFlash("B");
    Serial.print(".");
    delay(1000);
  }
  Serial.println("Network connected!");

  while (1) {
/**
 * Used to obtain the strength of the current network signal
 */
    myBC20.getSQ();

/**
 * Signal quality RSSI<10, weak signal strength
 */
    if(sSQ.rssi < 10 || sSQ.rssi == 99){
      myBC20.controlLED("LED_W_ON");  
      myBC20.controlLED("LED_W_OFF"); 
      if(sSQ.rssi == 99){
        Serial.println("Signal not detectable");
      }else if(sSQ.rssi ==0) {
        Serial.println("Signal Strength: -113 dBm or less");
      }else{
        Serial.print("Signal Strength: ");
        Serial.print((sSQ.rssi - 2) * 2 - 109);
        Serial.println(" dBm Weak");
      }
    }
/*
 * Signal quality 10<=RSSI<25, medium signal strength
 */
    else if(sSQ.rssi >= 10  && sSQ.rssi < 25){
      myBC20.controlLED("LED_W_ON");
      delay(500);
      myBC20.controlLED("LED_W_OFF");
      delay(500);
      Serial.print("Signal Strength: ");
      Serial.print((sSQ.rssi - 2) * 2 - 109);
      Serial.println(" dBm Medium");
    }

/*
 * Signal quality RSSI>=25, strong signal strength
 */
    else if (sSQ.rssi >= 25){
      if(sSQ.rssi < 31){
        for (int i = 0; i < 5 ; i++) {
          myBC20.controlLED("LED_W_ON");
          delay(100);		
          myBC20.controlLED("LED_W_OFF");
          delay(100);
        }
        Serial.print("Signal Strength: ");
        Serial.print((sSQ.rssi - 2) * 2 - 109);
        Serial.println(" dBm Strong");
      }else if (sSQ.rssi == 31) {
         Serial.print("Signal Strength: -51 dBm or greater");
       }
    }else{
      break;
    }
  }
}
void setup() {
  Serial.begin(115200);
  NB_Signal_Fun();
}

void loop() {
}
