/*!
   @file Subscribe_Topic.ino

   @brief Connect to DFRobot Easy IoT cloud platform,
   @brief and subscribe your Topic

   @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
   @licence     The MIT License (MIT)
   @author      [PengKaixing](kaixing.peng@dfrobot.com)
   @version  V1.0
   @date  2019-10-29
   @get from https://www.dfrobot.com
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

/*Configure device certificate information*/
char* Iot_id = "HJZv1ZFRSQ";
char* Client_ID  = "1234";
char* Iot_pwd    = "ByfP1-YABX";

/*Configure the domain name and port number*/
char* EasyIot_SERVER = "182.254.130.180";
char* PORT = "1883";

/*Set the Topic you need to publish to*/
char* subTopic = "JbG9-uBZg";

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
   IO17       |               D16(RX)        |       D/T
   IO16       |               D17(TX)        |       C/R
   GND        |               GND            |       GND
   5V(USB) or 3V3(battery)  | 5V or 3V3      |       VCC
*/
#if defined(ARDUINO_ESP32_DEV)
HardwareSerial Serial2(2);
DFRobot_BC20_Serial myBC20(&Serial2);//ESP32HardwareSerial
#else
DFRobot_BC20_Serial myBC20(&Serial1);//others
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
SoftwareSerial ss(PIN_TXD, PIN_RXD);
DFRobot_BC20_SW_Serial myBC20(&ss);
#endif

/*
  Callback function.
  This will be called when message is received from the subscribed topic.
*/
void callback(char * topic, uint8_t * payload, unsigned int len) {
  Serial.print("Recevice [Topic:");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < len; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  //Control onboard RGB LED according to received message.
  String receivedData = payload;
  if (receivedData.equals("OFF") == true) {
    myBC20.LED_OFF();
    Serial.println("LED is OFF.");
  }
  else if (receivedData.equals("RED") == true) {
    myBC20.changeColor(RED);
    myBC20.LED_ON();
    Serial.println("LED is red.");
  }
  else if (receivedData.equals("GREEN") == true) {
    myBC20.changeColor(GREEN);
    myBC20.LED_ON();
    Serial.println("LED is green.");
  }
  //  else if (receivedData.equals("BLUE") == true) {
  //    myBC20.changeColor(BLUE);
  //    myBC20.LED_ON();
  //    Serial.println("LED is blue.");
  //  }
  //  else if (receivedData.equals("YELLOW") == true) {
  //    myBC20.changeColor(YELLOW);
  //    myBC20.LED_ON();
  //    Serial.println("LED is yellow.");
  //  }
  //  else if (receivedData.equals("PURPLE") == true) {
  //    myBC20.changeColor(PURPLE);
  //    myBC20.LED_ON();
  //    Serial.println("LED is purple.");
  //  }
  //  else if (receivedData.equals("CYAN") == true) {
  //    myBC20.changeColor(CYAN);
  //    myBC20.LED_ON();
  //    Serial.println("LED is cyan.");
  //  }
  //  else if (receivedData.equals("WHITE") == true) {
  //    myBC20.changeColor(WHITE);
  //    myBC20.LED_ON();
  //    Serial.println("LED is white.");
  //  }
  //  else {
  //    ;
  //  }
}

void ConnectCloud() {
  Serial.print("Attempting MQTT connection...");
  while (!myBC20.connected()) {
    Serial.print(".");
    if (myBC20.connect(Client_ID, Iot_id, Iot_pwd)) {
      Serial.println("\nConnect Server OK");
    } else {
      /**
         Used to detect the connection between the device and the server
      */
      if (myBC20.getQMTCONN())
        break;
    }
  }

  //Disable sleep mode
//  myBC20.configSleepMode(eSleepMode_Disable);
//  //Disable PSM
//  myBC20.setPSMMode(ePSM_OFF);

  while (!myBC20.subTopic('0', '1', subTopic, '0')) {
    Serial.print(".");
  }
  Serial.print("\nTopic:");
  Serial.print(subTopic);
  Serial.println(" subscribed!");
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

  //Set callback function
  myBC20.setCallback(callback);

  Serial.println("Connecting to DFRobot Easy-IoT");

  //Configure IoT Server
  myBC20.setServer(EasyIot_SERVER, PORT);
  Serial.println("Server is available!");

  //Conect to DFRobot Easy-IoT
  ConnectCloud();
}

void loop() {
  myBC20.loop();
}
