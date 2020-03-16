/*!
   @file LowPower_Subscribe_Topic.ino

   @brief Connect to DFRobot Easy IoT cloud platform,
   @brief and subscribe your Topic in a low power way.
   
   @note If you change the demo or upload this demo again,
   @note you need to press the RST button on the module

   @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
   @licence     The MIT License (MIT)
   @author      [PengKaixing](kaixing.peng@dfrobot.com)
   @version  V1.0
   @date  2019-10-29
   @get from https://www.dfrobot.com
*/
#include "DFRobot_BC20.h"

/*7 colors are available*/
#define  RED 0
#define  BLUE 1
#define  GREEN 2
#define  YELLOW 3
#define  PURPLE 4
#define  CYAN 5
#define  WHITE 6

/*Configure device certificate information*/
char* Iot_id = "your_Iot_id";
char* Client_ID  = "your_Client_ID";
char* Iot_pwd    = "your_Iot_pwd";

/*Configure the domain name and port number*/
char* EasyIot_SERVER = "182.254.130.180";
char* PORT = "1883";

/*Set the Topic you need to publish to*/
char* subTopic = "your_subTopic";

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
SoftwareSerial ss(PIN_TXD, PIN_RXD);
DFRobot_BC20_SW_Serial myBC20(&ss);
#endif

void callback(char * topic, uint8_t * payload, unsigned int len) {
  Serial.print("Recevice [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < len; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (!myBC20.stmLowpower()) {
    Serial.println("STM32 enters Sleep Mode!");
  }
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
  Serial.println("started successfully!");

  /*Check whether SIM card is inserted*/
  Serial.print("Checking SIM card ...");
  myBC20.changeColor(GREEN);
  while (!myBC20.checkNBCard()) {
    Serial.print(".");
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);
  }
  Serial.println("OK!");

  //Enable Deep Sleep Mode
    myBC20.configSleepMode(eSleepMode_DeepSleep);
  //myBC20.configSleepMode(eSleepMode_Disable);

  /*
     When entered PSM, BC20 will not recive any message issued from the cloud.
  */
  //Disable PSM
  myBC20.setPSMMode(ePSM_ON);

  /**
     The module will automatically attempt to connect to the network (mobile station).
     Check whether it is connected to the network.
  */
  Serial.print("Connecting network ...");
  myBC20.changeColor(BLUE);
  while (myBC20.getGATT() == 0) {
    Serial.print(".");
    myBC20.LED_ON();
    delay(500);
    myBC20.LED_OFF();
    delay(500);
  }
  Serial.println("connected!");

  //Set callback function
  myBC20.setCallback(callback);

  Serial.println("Connecting to DFRobot Easy-IoT");

  //Configure IoT Server
  myBC20.setServer(EasyIot_SERVER, PORT);
  Serial.println("Server is available!");

  //Conect to DFRobot Easy-IoT
  ConnectCloud();

  if (!myBC20.stmLowpower()) {
    Serial.println("STM32 enters PSM!");
  }
}

void loop() {
  myBC20.loop();
}
