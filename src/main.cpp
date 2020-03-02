/*
  main.cpp
  Main code where the control takes place
  @author  Leo Korbee (c), Leo.Korbee@xs4all.nl
  @website iot-lab.org
  @license Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
  Thanks to all the folks who contributed beforme me on this code.

  Rewritten by befinitiv, 2020

*/

#include <Arduino.h>

#include "STM32LowPower.h"

#include "LoRaWAN.h"
#include "secconfig.h" // remember to rename secconfig_example.h to secconfig.h and to modify this file


// RFM95W
#define DIO0 PA1
#define NSS  PA4
#define RESET PC14
RFM95 rfm(DIO0, NSS);

// define LoRaWAN layer
LoRaWAN lora = LoRaWAN(rfm);


void setPinModes() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RESET, OUTPUT);
}

void setup()
{
  SerialUSB.begin(115200);
  SerialUSB.println("Starting ...");

  setPinModes();
  
  //reset RFM
  digitalWrite(RESET, LOW);
  delay(10);
  digitalWrite(RESET, HIGH);
  delay(2000);



  //Initialize RFM module
  rfm.init();

  lora.setKeys(NwkSkey, AppSkey, DevAddr);

  LowPower.begin();

}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  
  
  SerialUSB.println("Sending PKG");
  // define bytebuffer
  uint8_t Data_Length = 0x06;
  uint8_t Data[Data_Length];

  Data[0] = 0;
  Data[1] = 1;

  // move into bytebuffer
  Data[2] = 12;
  Data[3] = 13;

  Data[4] = 14;
  Data[5] = 15;

  lora.Send_Data(Data, Data_Length, 0);

  LowPower.deepSleep(20000);

}

