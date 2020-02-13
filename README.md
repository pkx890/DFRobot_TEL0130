# TEL0130
The library is used to capture and parse satellite data and communicate with Internet of things platforms.
The DFRobot_TEL0130 library is an arduino-based library

## DFRobot_TEL0130 Library for Arduino
---------------------------------------------------------
Arduino libraries are provided for satellite messaging and iot communication through MQTT

## Table of Contents

* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [Credits](#credits)

<snippet>
<content>

## Installation

To use this library, download the zip file and extract it into a folder called DFRobot_TEL0130.

## Methods

```C++

  /**
    * @Used to open BC20 with a bool return value.  
    */  
  bool powerOn(void);  

/*
    * @Used to check for the existence of a NB card with a bool return value
    */  
  bool  checkNBCard(void);
  
/*
    * @For network connection, return 1 on success, 0 on failure
*/  
  uint8_t  getGATT(void);

/*
    * @Used to configure the sleep mode of bc20, the parameter is an enumeration type  
    */
  typedef enum {
    eSleepMode_Disable,   
    eSleepMode_DeepSleep  
    eSleepMode_Light   
  }eSleepMode_t;
  bool  configSleepMode(eSleepMode_t mode);
  
/*
    * @Obtain extended signal quality   
*/  
  void  getESQ(void);
  
/*
    * @Gets the network registration status  
*/  
  void  getEREG(void);
  
/*
    * @Get the PDP address
*/
  void  getGPADDR(void);
  
/*
    * @For the remote firmware upgrade to bc20, the parameter is the url for the upgrade
*/
  bool  setQFOTADL(String url);
  
/*
    * @It is used to detect whether the current device is connected
*/
  bool  connected();
  
/*
    * @Used to connect the device to the server
*/
  bool  connect(char * clientID, char * username,char * password,char connectID ='0');
  
/*
    * @Used to detect the connection between the device and the server
*/
  bool  getQMTCONN(void);
  
/*
    * @Use to connect to Internet of things sites
*/
  bool  setServer(char* IPAddress,char* port,char connectID = '0');
  
/*
    * @Used to send a message to the server, parameter description: the first parameter is used to specify the topic,   
    * @the second parameter is the specific data to be sent
*/
  bool  publish(char* topic,char* msg);
  
/*
    * @This function is used to receive data when the server sends it to subscribers
*/
  void  loop();
  
/*
    * @The third parameter used to subscribe is the specified topic
*/
  bool  subTopic(char connectID, char msgID, char* topic, char qos);
  
/*
    * @Used for module power control. If the return value is 1, the module is in the state of power supply; 
    * @if the return value is 0, the module is in the state of power loss    
*/
  uint8_t  getQGNSSC(void);
  
/*
    * @Is used to obtain the specified satellite information, and the parameter is used to specify the type of information to be obtained. 
    * @The parameter is selected as follows:
*/
  bool  getQGNSSRD(char* sth);
  
/*
    * @These two functions are used to control the lighting of led lights
*/
  void  LED_ON();void LED_OFF();
  
/*
    * @Get the serial number of the NB card
*/
  void  getGSN(uint8_t cmd=0);
  
/*
    * @Get the SIM card number
*/
  String  getQCCID(void);
  
/*
    * @Gets the current network time
*/
  String  getCLK(void);
  
/*
    * @Used to wake up stm32 by external interrupt with external interrupt pin for output signal
*/
  bool  stmAwake(uint8_t Awake_Pin);
  
/*
    * @Used to obtain the strength of the current network signal
*/
  void  getSQ(void);  
```  

## Compatibility  

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32  |      √       |             |            | 
ATMEGA2560  |      √       |             |            | 
Arduino uno |       √      |             |            | 
  
## Credits
Written by Peng Kaixing(kaixing.peng@dfrobot.com), 2020. (Welcome to our website)
