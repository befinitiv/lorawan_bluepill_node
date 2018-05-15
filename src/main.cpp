/*
  main.cpp
  Main code where the control takes place
  @author  Leo Korbee (c), Leo.Korbee@xs4all.nl
  @website iot-lab.org
  @license Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
  Thanks to all the folks who contributed beforme me on this code.

  Hardware information at the end of this file.
  @version 2018-03-31

*/
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "tinySPI.h"
#include "BME280Spi.h"
#include "LoRaWAN.h"
#include "secconfig.h" // remember to rename secconfig_example.h to secconfig.h and to modify this file


// RFM95W
#define DIO0 10
#define NSS  9
RFM95 rfm(DIO0, NSS);

// define LoRaWAN layer
LoRaWAN lora = LoRaWAN(rfm);
// frame counter for lora
unsigned int Frame_Counter_Tx = 0x0000;

// BME280 SPI Chipselect pin
#define BME_CS 7
// Default : forced mode, standby time = 1000 ms
//Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off
BME280Spi::Settings settings(BME_CS);
BME280Spi bme(settings);

// the things network stuff
// get them from the device overview page! uncomment and put this information here, remove the #include secconfig.h
// due to security reasons this information from the author is put in secconfig.h

// TTN, msb left
// unsigned char NwkSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// unsigned char AppSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// unsigned char DevAddr[4] = { 0x00, 0x00, 0x00, 0x00 };

// sleep cycles that will be counted, start with more than sleep_total to start after 8 seconds with first broadcast.
volatile int sleep_count = 38;

// set sleep time between broadcasts. The processor awakes after 8 seconds deep-sleep_mode,
// increase and checks the counter and sleep again until sleep_total is reached.
// 5min * 60s = 300/8 = 37,5 = 38.
// 17 seconds longer than 5 minutes with 37, so 35 is more apropriate
const int sleep_total = 35; // was 35

// all functions declared
ISR(WDT_vect);
void readData(float &temp, float &hum, float &press);
uint16_t vccVoltage();
void setUnusedPins();
void goToSleep();
void watchdogSetup();


void setup()
{
  // define unused pins
  setUnusedPins();

  //Initialize RFM module
  rfm.init();

  lora.setKeys(NwkSkey, AppSkey, DevAddr);

  // for BME sensor
  while(!bme.begin())
  {
     // Serial.println("Could not find BME280 sensor!");
     delay(200);
  }

}

void loop()
{

  // goToSleep for all devices...
  // The watchdog timer will reset.
  goToSleep();

  // use this for non-sleep testing:
  //delay(8000);
  //sleep_count++;

  // do action if sleep_total is reached
  if (sleep_count >= sleep_total)
  {
    uint16_t tempInt;
    uint16_t humInt;

    // define bytebuffer
    uint8_t Data_Length = 0x06;
	  uint8_t Data[Data_Length];

    float temp, hum, press;
    // read data from sensor
    readData(temp, hum, press);
    // from float to uint16_t
    tempInt = temp * 100;
    humInt = hum * 100;

    // read vcc voltage (mv)
    uint16_t vcc = vccVoltage();
    Data[0] = (vcc >> 8) & 0xff;
    Data[1] = (vcc & 0xff);

    // move into bytebuffer
    Data[2] = (tempInt >> 8) & 0xff;
    Data[3] = tempInt & 0xff;

    Data[4] = (humInt >> 8) & 0xff;
    Data[5] = humInt & 0xff;

    lora.Send_Data(Data, Data_Length, Frame_Counter_Tx);

    Frame_Counter_Tx++;

    // reset sleep count
    sleep_count = 0;

  }

}


/*
  set unused pins so no undefined situation takes place
*/
void setUnusedPins()
{
  // 0, 1, 2, 3, 8
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
}

/**
 read temperature from BME sensor
*/
void readData(float &temp, float &hum, float &press)
{
  // set units sensor
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  // read sensor (SPI interface)
  bme.read(press, temp, hum, tempUnit, presUnit);

}

/**
  read voltage of the rail (Vcc)
  output mV (2 bytes)
*/
uint16_t vccVoltage()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  // default ADMUX REFS1 and REFS0 = 0

  // #define _BV(bit) (1 << (bit))

  // 1.1V (I Ref)(2) 100001
  ADMUX = _BV(MUX5) | _BV(MUX0);

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  uint16_t result = (high<<8) | low;

  // result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  // number of steps = 1023??
  result = (1125300L / result) ; // Calculate Vcc (in mV);

  return result;
}


void watchdogSetup()
{
  // Table for clearing/setting bits
  //WDP3 - WDP2 - WPD1 - WDP0 - time
  // 0      0      0      0      16 ms
  // 0      0      0      1      32 ms
  // 0      0      1      0      64 ms
  // 0      0      1      1      0.125 s
  // 0      1      0      0      0.25 s
  // 0      1      0      1      0.5 s
  // 0      1      1      0      1.0 s
  // 0      1      1      1      2.0 s
  // 1      0      0      0      4.0 s
  // 1      0      0      1      8.0 s


  // Reset the watchdog reset flag
  bitClear(MCUSR, WDRF);
  // Start timed sequence
  bitSet(WDTCSR, WDCE); //Watchdog Change Enable to clear WD
  bitSet(WDTCSR, WDE); //Enable WD

  // Set new watchdog timeout value to 8 second
  bitSet(WDTCSR, WDP3);
  bitClear(WDTCSR, WDP2);
  bitClear(WDTCSR, WDP1);
  bitSet(WDTCSR, WDP0);
  // Enable interrupts instead of reset
  bitSet(WDTCSR, WDIE);
}

void goToSleep()
{
  //Disable ADC, saves ~230uA
  ADCSRA &= ~(1<<ADEN);
  watchdogSetup(); //enable watchDog
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // sleep_mode does set and reset the SE bit. sleep_enable and sleep_disable are not usefull
  sleep_mode();
  //disable watchdog after sleep
  wdt_disable();
  // enable ADC
  ADCSRA |=  (1<<ADEN);
}


ISR(WDT_vect)
{
  sleep_count++; // keep track of how many sleep cycles have been completed.
}


/*
  Hardware setup
  Attiny84 using the Arduino pin numbers! PB0 = 0 etc.


  Atmel ATtiny84A PU
  RFM95W
  BME280

  Power: 3V3 - 470uF elco over power connectors, 100nF over power connector for interference suppression
  Connections:
  RFM95W   ATtiny84

  DIO0 -- PB0
  MISO -- MOSI
  MOSI -- MISO
  SCK  -- SCK
  NSS  -- PB1 (this is Chipselect)

  Bosch BME280 sensor used with tinySPI....
  BME280  ATtiny84
  SCL -- SCK
  SDA -- MISO
  SDO -- MOSI
  CSB -- PA7 (this is Chipselect)(Arduino pin 7)
*/
