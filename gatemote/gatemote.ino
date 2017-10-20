// GateMote sketch to monitor an SPDT door or gate status contact, such as the Sentrol 2507-A
// By Andrew Bauer <zonexpertconsulting@outlook.com>
// Based on conversation in this thread: 
// https://lowpowerlab.com/forum/low-power-techniques/interrupt-driven-reed-switch-circuit/msg14267/
//
// IMPORTANT: adjust the settings in the configuration section below !!!
//
// This sketch was primarly based on the MotionMote sketch written by Felix Rusu of LowerPowerLab.com
// http://lowpowerlab.com/motionmote
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
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_OTA.h> //get it here: https://github.com/lowpowerlab/rfm69
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SparkFunBME280.h> //get it here: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/tree/master/Libraries/Arduino/src
#include <Wire.h>     //comes with Arduino

//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID        10    //unique for each node on same network
#define NETWORKID     1  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ENCRYPTKEY    "xxxxxxxxxxxxxxxx" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -70

#define ACK_TIME      50  // max # of ms to wait for an ack
#define GATE_STATUS_PIN     3  // hardware interrupt 1 (D3) - where the gate status common wire is attached, this will generate an interrupt every time the gate changes state
#define BATT_MONITOR  A0  // Directly wired to battery +V
#define BATT_FORMULA(reading) reading / 183.0 // 233 was experimentally determined to be the scale factor that produced the most accurate results
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

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

#define FLASH_SS      8 // and FLASH SS on D8 on regular Moteinos (D23 on MoteinoMEGA)
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

BME280 bme280;

// Define variables
volatile boolean gateStatusChange = false;
boolean reattach = false;
boolean startup = true;
int sleepCycles = 0;
int buttonState; // the current reading from the input pin
float batteryVolts = 3;
char BATstr[10]; // longest battery voltage reading message = 9chars
char sendBuff[50];
char statusText[20];
byte sendLen;

char Pstr[10];
char Fstr[10];
char Hstr[10];
double F,P,H;

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

  if (flash.initialize()) flash.sleep(); //if Moteino has FLASH-MEM, make sure it sleeps

  bme280.settings.commInterface = I2C_MODE;
  bme280.settings.I2CAddress = 0x76;
  bme280.settings.runMode = 3; //Normal mode
  bme280.settings.tStandby = 0;
  bme280.settings.filter = 0;
  bme280.settings.tempOverSample = 1;
  bme280.settings.pressOverSample = 1;
  bme280.settings.humidOverSample = 1;

  for (uint8_t i=0; i<=A5; i++) {
    if (i == RF69_SPI_CS) continue;
    if (i == FLASH_SS) continue;
    if (i == BATT_MONITOR) continue;
    if (i == GATE_STATUS_PIN) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  pinMode(BATT_MONITOR, INPUT);  
  pinMode(GATE_STATUS_PIN, INPUT);
  buttonState = digitalRead(GATE_STATUS_PIN); // set the initial state of the gate
  reportStatus(buttonState,false);
  EIFR = bit (INTF1);  // clear flag for interrupt 1
  attachInterrupt(digitalPinToInterrupt(GATE_STATUS_PIN), gateIRQ, CHANGE);
}

void loop() { //Main loop

  sleepCycles++; // increment our sleep cycle counter
  if (gateStatusChange) { // We have been waken up via interrupt

    DEBUGln("SLEEP60ms");
    gotoSleep(SLEEP_60MS, false); // sleep w/o interrupts to allow the gate status switch to debounce
    DEBUGln("WAKEUP60ms");

    // At this point we assume the gate has debounced
    int reading = digitalRead(GATE_STATUS_PIN); // read the state of the switch into a local variable   
    
    if (reading != buttonState) { // if the actual button state has changed
      buttonState = reading;
      reportStatus(buttonState,false);
      DEBUGln("Gate Status Changed State");
    }
    
    gateStatusChange = false; // reset the flag previsouly set by our interrupt
    reattach = true; // signal we want to renable interrupts before sleeping
    DEBUGln("Status Change Trigger Processed");
    
  } else { // else we have been waken up via time clock
    if ( sleepCycles > 14 ) { // If no interrupts occur, this will report status roughly every minute
      reportStatus(buttonState,true);
      sleepCycles = 0;
      DEBUGln("Sent checkin data");
    }
  }

  //When this sketch is on a node where you can afford the power to keep the radio awake all the time
  //   you can make it receive messages and also make it wirelessly programmable
  //   otherwise this section can be removed

/*
  if (radio.receiveDone()) {
    DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      DEBUG((char)radio.DATA[i]);

    flash.wakeup();
    // wireless programming token check - this only works when radio is kept awake to listen for WP tokens
    CheckForWirelessHEX(radio, flash, true);

    //first send any ACK to request
    DEBUG("   [RX_RSSI:");DEBUG(radio.RSSI);DEBUG("]");
    if (radio.ACKRequested()) {
      radio.sendACK();
      DEBUG(" - ACK sent.");
    }
    DEBUGln();
  } // End check for wireless token
*/

  DEBUGln("SLEEP8s");
  gotoSleep(SLEEP_8S, reattach); // Re-enable interrupts just prior to sleeping if previously disabled
  DEBUGln("WAKEUP8s");
}

void gateIRQ() { // Called when an interrupt occurs (i.e. when the gate contact changes state)
  gateStatusChange = true;
  detachInterrupt(digitalPinToInterrupt(GATE_STATUS_PIN));
  DEBUGln("IRQ Triggered");
}

void checkBattery() { // Read the battery voltage then update BATstr
  unsigned int readings=0;
  for (byte i=0; i<10; i++) //take 10 samples, and average
    readings+=analogRead(BATT_MONITOR);
  batteryVolts = BATT_FORMULA(readings / 10.0);
  dtostrf(batteryVolts, 3,2, BATstr); //update the BATStr, which gets sent every reportstatus cycle
}

void reportStatus(int gateStatus, boolean readbme) { // xmit the status of the gate

  if (startup) { // report to the gateway we just started up
    sprintf(statusText, "START %s", gateStatus==LOW ? "CLOSED" : gateStatus==HIGH ? "OPEN" : "UNKNOWN");
    startup = false;
  } else {
    sprintf(statusText, "%s", gateStatus==LOW ? "CLOSED" : gateStatus==HIGH ? "OPEN" : "UNKNOWN");
  }
    
  if (readbme) {
    checkBattery();
    readBME280();
    sprintf(sendBuff, "%s BAT:%sv F:%s H:%s P:%s" , statusText, BATstr, Fstr, Hstr, Pstr);
  } else {
    sprintf(sendBuff, "%s", statusText);    
  }

  sendLen = strlen(sendBuff);
  DEBUG("Gate Status: "); DEBUGln(sendBuff);
  if (radio.sendWithRetry(GATEWAYID, sendBuff, sendLen, 3, ACK_TIME) ) {
    DEBUGln("ACK: OK");
  } else {
    DEBUGln("ACK: NOT OK");
  }
}

void gotoSleep(period_t period, boolean reattach) {
  DEBUGFlush();
  //flash.sleep();
  radio.sleep();
  //interrupts(); // Make certain we don't ever go to sleep w/o interrupts enabled
  if ( reattach ) {
    reattach = false;
    EIFR = bit (INTF1);  // clear flag for interrupt 1
    attachInterrupt(digitalPinToInterrupt(GATE_STATUS_PIN), gateIRQ, CHANGE); // reenable our interrupt just prior to sleeping
  }  
  LowPower.powerDown(period, ADC_OFF, BOD_OFF); 
}

void readBME280() { // Read the BME280 sensor and xmit the results   
    
    bme280.begin();
    P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
    F = bme280.readTempF();
    H = bme280.readFloatHumidity();
    bme280.writeRegister(BME280_CTRL_MEAS_REG, 0x00); //sleep the BME280

    dtostrf(F, 3,2, Fstr);
    dtostrf(H, 3,2, Hstr);
    dtostrf(P, 3,2, Pstr);
}
