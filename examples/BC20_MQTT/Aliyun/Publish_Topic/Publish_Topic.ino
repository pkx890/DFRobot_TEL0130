/*!
 * @file Publish_Topic.ino
 * 
 * @brief After the program download is complete.
 * @brief You can use the BC20 module to connect to Aliyun's cloud platform,
 * @brief and publish data to your Topic
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [PengKaixing](kaixing.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-10-29
 * @get from https://www.dfrobot.com
 */

#include "DFRobot_BC20.h"
#include "DFRobot_Iot.h"

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

DFRobot_Iot myDevice;
/*Configure device certificate information*/
const char* ProductKey = "a1QAq4WoEou";
const char* ClientId = "Tinker_A";/*Custom id*/
const char* DeviceName = "YEYMxzpGI3TlwvCRHWtQ";
const char* DeviceSecret = "9FYJoMXlsxXiyO6wNH3z7gJ0QKPrmWAn";

/*Configure the domain name and port number*/
const char* ALIYUN_SERVER = "iot-as-mqtt.cn-shanghai.aliyuncs.com";
const char* PORT = "1883";

/*Product identifier that needs to be operated*/
const char* Identifier = "your_Identifier";

/*TOPIC that need to be published and subscribed*/
const char* subTopic = "your_sub_Topic";//****set
const char* pubTopic = "your_pub_Topic";//******post


void callback(char * topic, byte * payload, unsigned int len){
  Serial.print("Recevice [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < len; i++){
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void ConnectCloud(){
  while(!myBC20.connected()){
    Serial.print("Attempting MQTT connection...");
    if(myBC20.connect(myDevice._clientId, myDevice._username, myDevice._password)){
      Serial.println("Connect Server OK");
    }else{
            if(myBC20.getQMTCONN())
            break;
    }
  }
}
void setup(){
  Serial.begin(115200);
    Serial.println("Starting the BC20.Please wait. . . ");
    while(!myBC20.powerOn()){
    delay(1000);
    myBC20.control_LED("LED_R_ON");
    delay(10);   
    myBC20.control_LED("LED_R_OFF"); 
    delay(10);    
    Serial.print(".");
  }
  Serial.println("BC20 started successfully !");
  while(!myBC20.checkNBCard()){
    Serial.println("Please insert the NB card !");
    delay(1000);
    myBC20.control_LED("LED_G_ON");
    delay(10);   
    myBC20.control_LED("LED_G_OFF"); 
    delay(10);    
  }
  Serial.println("Waitting for access ...");
  while(myBC20.getGATT() == 0){
    Serial.print(".");
    delay(1000);
    myBC20.control_LED("LED_B_ON");
    delay(10);   
    myBC20.control_LED("LED_B_OFF"); 
    delay(10);    
  }
  myDevice.init(ALIYUN_SERVER,ProductKey,ClientId,DeviceName,DeviceSecret);
  myBC20.setServer(myDevice._mqttServer,PORT);
  myBC20.setCallback(callback);
  ConnectCloud();
}

void loop(){
  delay(10000);
  myBC20.publish(pubTopic,"Hello test");
}
