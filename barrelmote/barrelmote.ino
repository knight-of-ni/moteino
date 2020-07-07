// Ultrasonic distance sensor sketch
// By Andrew Bauer <zonexpertconsulting@outlook.com>
//
// IMPORTANT: adjust the settings in the configuration section below !!!
//
// This sketch is similar in function to the SonarMote sketch written by Felix Rusu of LowerPowerLab.com
// https://lowpowerlab.com/sonarmote/
//
// **********************************************************************************
// Copyright Felix Rusu of LowPowerLab.com, 2016
// RFM69 library and sample code by Felix Rusu - lowpowerlab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
//
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_OTA.h> //get it here: https://github.com/lowpowerlab/rfm69
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID        12    //unique for each node on same network
#define NETWORKID     34  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ENCRYPTKEY    "xxxxxxxxxxxxxxxx" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -70

#define ACK_TIME      75  // max # of ms to wait for an ack

//*********************************************************************************************
#define SENSOR_EN1     4  // digital pin that enables power to ultrasonic sensor
#define SENSOR_EN2     5  // digital pin that enables power to ultrasonic sensor
#define TRIG           6  // digital pin wired to TRIG pin of ultrasonic sensor
#define ECHO           7  // digital pin wired to ECHO pin of ultrasonic sensor
#define MAX_DISTANCE 32.3 // maximum distance in inches representing an empty barrel
#define MIN_DISTANCE  2.6 // minimum distance in inches representing a full barrel
//*********************************************************************************************

#define BATT_MONITOR  A2  // Directly wired to battery +V
#define BATT_FORMULA(reading) reading / 153.83 // value was experimentally determined to be the scale factor that produced the most accurate results

//*********************************************************************************************
//#define SERIAL_EN             //comment this out when deploying to an installed Mote to save a few KB of sketch size
#define SERIAL_BAUD    115200
#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
  #define DEBUGFlush() { Serial.flush(); }
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define DEBUGFlush();
#endif
//*********************************************************************************************

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

// Define variables
int sleepCycles = 0;
float batteryVolts = 3;
char BATstr[10]; // longest battery voltage reading message = 9chars
char DISTstr[6]; // longest battery voltage reading message = 6chars
char sendBuff[50];
byte sendLen;
boolean startup;

SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

void setup() { // Setup routine called once during startup
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif
  
  sprintf(sendBuff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(sendBuff);
  
#ifdef ENABLE_ATC
  DEBUGln("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

  for (uint8_t i=0; i<=A5; i++) {
    if (i == RF69_SPI_CS) continue;
    if (i == BATT_MONITOR) continue;
    if (i == SENSOR_EN1) continue;
    if (i == SENSOR_EN2) continue;
    if (i == TRIG) continue;
    if (i == ECHO) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  pinMode(SENSOR_EN1, OUTPUT);
  pinMode(SENSOR_EN2, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(BATT_MONITOR, INPUT);
  
  if (flash.initialize()) flash.sleep();
  readDistance(1); //first reading seems to always be low
  reportStatus(startup=true);
}

void loop() { // Our Main loop. Nice and short.

  sleepCycles++; // increment our sleep cycle counter
  if ( sleepCycles > 30 ) { // This will report status roughly every 4 minutes
    reportStatus(startup=false);
    sleepCycles = 0;
    DEBUGln("Sent checkin data");
  }

  DEBUGln("SLEEP8s");
  gotoSleep(SLEEP_8S);
  DEBUGln("WAKEUP8s");
}

void checkBattery() { // Read the battery voltage then update BATstr
  unsigned int readings=0;
  for (byte i=0; i<10; i++) //take 10 samples, and average
    readings+=analogRead(BATT_MONITOR);
  batteryVolts = BATT_FORMULA(readings / 10.0);
  dtostrf(batteryVolts, 3,2, BATstr); //update the BATStr, which gets sent every reportstatus cycle
}

void reportStatus(boolean startup) {
  if (startup) { // report to the gateway we just started up
    sprintf(sendBuff, "START");
  } else {
   checkBattery();
   byte percentFull = inchesToPercent(readDistance(3));
   sprintf(sendBuff, "%sin BAT:%sv LVL:%u" , DISTstr, BATstr, percentFull);
  }

  sendLen = strlen(sendBuff);
  DEBUG("Ultrasonic Sensor Status: "); DEBUGln(sendBuff);
  if (radio.sendWithRetry(GATEWAYID, sendBuff, sendLen, 3, ACK_TIME) ) {
    DEBUGln("ACK: OK");
  } else {
    DEBUGln("ACK: NOT OK");
  }
}

void gotoSleep(period_t period) {
  DEBUGFlush();
  radio.sleep();
  LowPower.powerDown(period, ADC_OFF, BOD_OFF); //sleep the microcontroller 
}

float readDistance(byte samples) {
  if (samples == 0) samples = 1;
  if (samples > 10) samples = 10;
  
  digitalWrite(SENSOR_EN1, HIGH);
  digitalWrite(SENSOR_EN2, HIGH);  
  
  //need about 60-75ms after power up before HC-SR04 will be usable, so just sleep in the meantime
  gotoSleep(SLEEP_60MS);
  gotoSleep(SLEEP_15MS);
  PING();
  gotoSleep(SLEEP_15MS);
  
  unsigned long duration = 0;
  for (byte i=0; i<samples; i++)
  {
    duration += PING();
    if (samples > 1) gotoSleep(SLEEP_15MS);
  }
  digitalWrite(SENSOR_EN1, LOW);
  digitalWrite(SENSOR_EN2, LOW);
  float distance = microsecondsToInches(duration / samples);
  dtostrf(distance,3,2, DISTstr);

  return distance;
}

long PING() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH);
}

float microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74.0 / 2.0f;
}

float microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return (float)microseconds / 29.0f / 2.0f;
}

byte inchesToPercent(float distance) {
  byte percentFull;
  if ( distance > MAX_DISTANCE ) { // anything greater than MAX_DISTANCE we call empty
    percentFull = 0;
  } else if ( distance < MIN_DISTANCE ) { // anything less than MIN_DISTANCE we call 100% full
    percentFull = 100;
  } else {
    percentFull=round(100.0-(((distance-MIN_DISTANCE)*100.0)/MAX_DISTANCE));
  }
  return percentFull;
}
