/*!
 * @file Publish_Topic.ino
 * 
 * @brief After the program download is complete.
 * @brief You can use the BC20 module to connect to Aliyun's cloud platform,
 * @brief and publish data to your Topic
 * @Because it takes a lot of RAM to calculate the password, it's only available for ESP32 for now
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-10-29
 * @get from https://www.dfrobot.com
 */

#include "DFRobot_BC20.h"
#include "DFRobot_Iot.h"

/*7 colors are available*/
#define  RED 0
#define  BLUE 1
#define  GREEN 2
#define  YELLOW 3
#define  PURPLE 4
#define  CYAN 5
#define  WHITE 6

/*
 *Configure device certificate information
 */
char* ProductKey = "a1QAq4WoEou";
char* ClientId = "Tinker_A";/*Custom id*/
char* DeviceName = "YEYMxzpGI3TlwvCRHWtQ";
char* DeviceSecret = "9FYJoMXlsxXiyO6wNH3z7gJ0QKPrmWAn";

/*
 *Configure the domain name and port number
 */
char* ALIYUN_SERVER ="iot-as-mqtt.cn-shanghai.aliyuncs.com";
char* PORT = "1883";

/*
 *Product identifier that needs to be operated
 */
char* Identifier = "your_Identifier";

/*
 *TOPIC that need to be published and subscribed
 */
char* subTopic = "/a1QAq4WoEou/YEYMxzpGI3TlwvCRHWtQ/user/get";//****set
char* pubTopic = "/a1QAq4WoEou/YEYMxzpGI3TlwvCRHWtQ/user/update/error";//******post

/*Communication by IIC*/
#define USE_IIC

/*Communication by HardwareSerial*/
//#define USE_HSERIAL

/*Communication by SoftwareSerial*/
//#define USE_SSERIAL


/******************IIC******************/
#ifdef USE_IIC
/*
   For general controllers. Communicate by IIC
   Connect Instructions
      Controller     |    Module(BC20)
          SDA        |       D/T
          SCL        |       C/R
          GND        |       GND
       5V or 3V3     |       VCC

   IIC address(A0,A1)
     0x30:(A0=0,A1=0)
     0x31:(A0=0,A1=1)
     0x32:(A0=1,A1=0)
     0x33:(A0=1,A1=1) default
*/
DFRobot_BC20_IIC myBC20(0x33);

/******************HardwareSerial******************/
#elif defined(USE_HSERIAL)
/*
   For MEGA2560/ESP32 HardwareSerial
   Connect Instructions
   esp32      |               MEGA Series    |    Module(BC20)
   IO16       |               D17(RX)        |       D/T
   IO17       |               D16(TX)        |       C/R
   GND        |               GND            |       GND
   5V(USB) or 3V3(battery)  | 5V or 3V3      |       VCC
*/
#if defined(ARDUINO_ESP32_DEV)
HardwareSerial Serial2(2);
DFRobot_BC20_Serial myBC20(&Serial2);//ESP32HardwareSerial
#else
DFRobot_BC20_Serial myBC20(&Serial2);//others
#endif

/******************SoftwareSerial******************/
#elif defined(USE_SSERIAL)
/*
    For Arduino Series SoftwareSerial
    Connect Instructions
        UNO     |    Module(BC20)
      PIN_RXD   |       D/T
      PIN_TXD   |       C/R
        GND     |       GND
     5V or 3V3  |       VCC
*/
#define PIN_TXD   3
#define PIN_RXD   4
SoftwareSerial ss(PIN_RXD,PIN_TXD);
DFRobot_BC20_SW_Serial myBC20(&ss);
#endif
DFRobot_Iot myDevice;

void ConnectCloud() {
  Serial.print("Attempting MQTT connection...");
  myBC20.changeColor(YELLOW);
  while (!myBC20.connected()) {
    Serial.print(".");
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);

    if (myBC20.connect(myDevice._clientId, myDevice._username, myDevice._password)) {
      Serial.println("\nConnect Server OK");
    } else {
      /**
         Used to detect the connection between the device and the server
      */
      if (myBC20.getQMTCONN())
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  myBC20.LED_OFF();

  /*Initialize BC20*/
  Serial.print("Starting the BC20.Please wait. . . ");
  myBC20.changeColor(RED);
  while (!myBC20.powerOn()) {
    Serial.print(".");
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);
  }
  Serial.println("BC20 started successfully!");

  /*Check whether SIM card is inserted*/
  Serial.println("Checking SIM card ...");
  myBC20.changeColor(GREEN);
  while (!myBC20.checkNBCard()) {
    Serial.println("Please insert the NB SIM card !");
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);
  }
  Serial.println("SIM card check OK!");

  /*Print IMEI, ICCID and IMSI*/
  myBC20.getGSN(IMEI);
  Serial.print("BC20 IMEI: ");
  Serial.println(sGSN.imei);
  Serial.print("SIM card ICCID:");
  Serial.print(myBC20.getQCCID());
  Serial.print("SIM card IMSI: ");
  Serial.println((char *)myBC20.getIMI());

  /**
     The module will automatically attempt to connect to the network (mobile station).
     Check whether it is connected to the network.
  */
  Serial.println("Connecting network ...");
  myBC20.changeColor(BLUE);
  while (myBC20.getGATT() == 0) {
    Serial.print(".");
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);
  }
  Serial.println("Network is connected!");
  myDevice.init(ALIYUN_SERVER,ProductKey,ClientId,DeviceName,DeviceSecret);
  Serial.println("Connecting to DFRobot Easy-IoT");

  //Configure IoT Server
  myBC20.setServer(myDevice._mqttServer, PORT);
  Serial.println("Server is available!");
  ConnectCloud();
}

void loop() {
  delay(5000);
  Serial.println("send message to cloud...");
  myBC20.publish(pubTopic, "hello");
  Serial.println("Message is sent.");
}
