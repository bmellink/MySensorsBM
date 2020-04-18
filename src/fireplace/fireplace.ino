/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * Note: using timer0 to generate a second interrupt for our 1 ms counters. timer0 is still also  used for delay()
 *
 * Fireplace status and command module
 * 
 *******************************  */


// BOARD: PRO MINI 3.3V/ 8Mhz ATMEGA328 8Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 1

// Enable and select radio type attached
#define MY_RADIO_NRF24


// node ID's
#define MY_NODE_ID 28             // start naming my own nodes at number 10
#define STATUS_ID 1               // used to show status and when clicked ignite or turn off
#define UPDOWN_ID 2               // used to turn heating somewhat up/ down or stop

#include <SPI.h>
#include <MySensors.h>  

// PIN connections

// analog pins: when AREF and SWPIN are equal, the motor switch is closed (either fully open of fully closed)
// consider the switch open if the analog difference > ADIFF
#define AREF  A0          // reference voltage
#define SWPIN A1          // switch for valve at end of row

// digital input/output
#define CMDPIN1 5         // outputs: connected to the remoted control switch
#define CMDPIN2 6
#define CMDPIN3 7
#define MOTOROPEN 4       // input gets 1 when motor is running towards open
#define MOTORCLOSE 3      // input gets 1 when motor is running towards close

// delay times
#define CHECK_FREQUENCY 200     // time in milliseconds between loop (where we check the sensor) - 200ms   
#define MIN_SEND_FREQ 4         // Minimum time between send (in seconds). We don't want to spam the gateway (4 seconds)
#define MAX_SEND_FREQ 300       // Maximum time between send (in seconds). We need to show we are alive (300 sec/5 min)
#define CMD_LONG 10000          // number of millisecond for key commnand - long 10 secs
#define CMD_SHORT 400           // number of millisecond for key commnand - short 0.4 secs
#define CMD_INITIAL 3000        // number of millisecond for initial turning on (when off) - 3 secs

// constants
#define ADIFF 40                // difference befor we consider this motor in middle

// Message types for sending
MyMessage fireplaceMsg(STATUS_ID,V_STATUS);

// vars
bool lastclose = true;    // last motor action we say was a closing
bool statsave = true;     // save value for status (set to open)
unsigned int cnt = 0;     // counter for layout in main loop

// vars updated in ISR
volatile unsigned int sendcnt = 0;      // counter when change happens
volatile unsigned int sendcntmax = 0;   // counter when sending status anyway
volatile unsigned int wait1000 = 1000;  // towards seconds
volatile unsigned int cmdstop = 0;      // counter that will cancel CMD_SHORT, CMD_INITIAL, CMD_LONG

void before() { 
   pinMode(CMDPIN1, OUTPUT); 
   pinMode(CMDPIN2, OUTPUT); 
   pinMode(CMDPIN3, OUTPUT); 
   docmdstop();
   pinMode(MOTOROPEN, INPUT);
   pinMode(MOTORCLOSE, INPUT);
   pinMode(AREF, INPUT);
   pinMode(SWPIN, INPUT);
   analogRead(AREF);
}

// stop command (after timeout)
void docmdstop() {
   digitalWrite(CMDPIN1, 0); 
   digitalWrite(CMDPIN2, 0); 
   digitalWrite(CMDPIN3, 0); 
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Fireplace", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  present(STATUS_ID, S_BINARY);
  present(UPDOWN_ID, S_COVER);    // window blinds allow us to have up/down etc
}

void setup() {
   Serial.println("setup()");
   // Timer0 is already used for millis() - we'll just interrupt somewhere
   // in the middle and call the TIMER0_COMPA_vect interrupt
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
}


void loop() {
  // we come here every 200 ms (defined in CHECK_FREQUENCY)
  bool stat = false;
  unsigned int atmp1, atmp2;
  if (cnt>0) cnt--; else { cnt=50; Serial.println();}
  bool motoropen = (digitalRead(MOTOROPEN) == HIGH);
  if (motoropen) lastclose = false;
  bool motorclose = (digitalRead(MOTORCLOSE) == HIGH);
  if (motorclose) lastclose = true;
  atmp1 = analogRead(AREF);
  atmp2 = analogRead(SWPIN);
  if (!motoropen && !motorclose) {
     // do not report while motor runs
     if (atmp1 > atmp2 + ADIFF) {
        // motor not in end position, so we are open
        stat = true;
        Serial.print(2);
     } else if (lastclose) {
        stat = false; // was closing and we are now at end position
        Serial.print(1);
     } else {
        stat = true; // was opening and we are now at end pos
        Serial.print(3);
     }
     
     if ((statsave!=stat && sendcnt==0) || sendcntmax==0) {
        Serial.println();
        statsave = stat;
        send(fireplaceMsg.set(stat ? "1" : "0"));
        sendcnt = MIN_SEND_FREQ;
        sendcntmax = MAX_SEND_FREQ;
        wait1000 = 1000;  // ensure whole seconds
     }
  }
  // wait again
  wait(CHECK_FREQUENCY);
}

void receive(const MyMessage &message) {
   if (message.type==V_STATUS && message.sensor==STATUS_ID) {
      bool st=message.getBool();
      docmdstop(); 
      if (st) {
         // ignite
         Serial.println(" Ignite"); 
         digitalWrite(CMDPIN1, 1); 
         digitalWrite(CMDPIN3, 1);  
         cmdstop = CMD_INITIAL;
      } else {
         // completely turn off. According to specs CMDPIN1+2+3 should be on, however this is unreliable
         // only CMDPIN2 works much better
         Serial.println(" Turn off"); 
         // digitalWrite(CMDPIN1, 1);  
         digitalWrite(CMDPIN2, 1);  
         // digitalWrite(CMDPIN3, 1);  
         cmdstop = CMD_INITIAL;
      }
      // statsave = st;
      sendcnt = MIN_SEND_FREQ; // look soon to status
      wait1000 = 1000;
   }
   if (message.sensor==UPDOWN_ID) {
      docmdstop(); 
      switch (message.type) {
      case V_UP:
           Serial.println(" Up"); 
           digitalWrite(CMDPIN1, 1);
           cmdstop = (statsave ? CMD_SHORT : CMD_INITIAL);
           break;
      case V_DOWN:
          Serial.println(" Down"); 
          digitalWrite(CMDPIN3, 3);
          cmdstop = CMD_SHORT;
          break;
      case V_STOP:
          // stop is down command for 10 secs
          Serial.println(" Stop"); 
          digitalWrite(CMDPIN3, 3);
          cmdstop = CMD_LONG;
          // statsave = false;
          break;
      }
      sendcnt = MIN_SEND_FREQ; // look soon to status
      wait1000 = 1000;
   }
}


// Interrupt on timer0 - called as part of timer0 - already running at 1ms intervals
//  MsTimer2::set(1000, onesec); // 1000ms period
//  MsTimer2::start();
SIGNAL(TIMER0_COMPA_vect) {
   if (wait1000>0) wait1000--;
   if (wait1000==0) {
      if (sendcnt>0) sendcnt--;
      if (sendcntmax>0) sendcntmax--;
      wait1000 = 1000;
   }
   if (cmdstop>0) {
      cmdstop--;
      if (cmdstop==0) {
         // now stop command code
         docmdstop();
      }
   }
}

