/*!
 * @file BC20_DFOTA.ino
 *
 * @n This code demonstrates how to implement a firmware upgrade via OTA.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [Wuxiao](xiao.wu@dfrobot.com)
 * @version  V1.0
 * @date  2019-04-12
 * @get from https://www.dfrobot.com
 */
 
#include "DFRobot_BC20.h"

DFRobot_BC20 myBC20;
void setup(){
  Serial.begin(115200);
  Serial.print("Starting the BC20.Please wait. . . ");
  while(!myBC20.powerOn()){
    delay(1000);
    Serial.print(".");
  }
  Serial.println("BC20 started successfully !");
  while(!myBC20.checkNBCard()){
    Serial.println("Please insert the NB card !");
    delay(1000);
  }
  Serial.println("Waitting for access ...");
  while(myBC20.getGATT() == 0){
    Serial.print(".");
    delay(1000);
  }
  myBC20.ConfigSleepMode(eSleepMode_Disable);
  myBC20.getESQ();
  myBC20.getEREG();
  myBC20.getGPADDR();
  myBC20.setQFOTADL("http://download3.dfrobot.com.cn/nbtest/Update0406.bin");
}

void loop(){
  if(Serial.available()){
    BC20Serial.write((char)Serial.read());
  }
  if(BC20Serial.available()){
    Serial.write((char)BC20Serial.read());
  }
}