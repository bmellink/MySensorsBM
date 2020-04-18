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
 * Functions
 * - Receive pulse on doorbell (low is ring)
 * - Send IR code to turn on/off TV (pin 8)
 * - Sense light in the room A0
 * - moving PIR
 * - is repeater in network
 *******************************  */

// NOTE: in MyConfig.h: due to use of NRF24 with amplifier
// #define MY_RF24_PA_LEVEL (RF24_PA_MAX) 

// BOARD: PRO MINI 5V V/ 16Mhz ATMEGA328 8Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_RF24
#define MY_REPEATER_FEATURE       // repeater

// node ID's
#define MY_NODE_ID 11             // start naming my own nodes at number 10
#define PIR_ID 1                  // sensor ID PIR
#define RING_ID 2                 // ring doorbell
#define LIGHT_ID 3                // light in room
#define IR_ID 4                   // IR send ID

#include <SPI.h>
#include <MySensors.h>  

#include <IRremote.h>
IRsend irsend;


// PIN connections
#define IR_POWER_PIN 3            // pin that will provide power to IR LED (hardcoded in IRremote lib)
#define LIGHT_PIN A0              // analog input for light measurement
#define PIR_SENSOR_PIN 6          // pin for PIR
#define DOORBELL_PIN 7            // pin doorbell (low is ring)

// delay times
#define TRIP_TIME 20 // Number of seconds we wait until we send the no movement code after movement has stopped
#define RING_TIME 5  // ring time stable signal
#define LIGHT_TIME 10 // minimum time between light updates

#define ALIVE_TIME 120 // Number of seconds we wait until we send our data anyway

// Motion message types
MyMessage pirmsg(PIR_ID, V_TRIPPED);
MyMessage ringmsg(RING_ID, V_TRIPPED);
MyMessage lightmsg(LIGHT_ID, V_LIGHT_LEVEL); // value in % 0-100

// sensor value intermediate storage
bool tripped1 = false;
bool tripped2 = false;
unsigned int LightValue = 0;

// counters updated in ISR routines
volatile unsigned int countdown1 = 0; 
volatile unsigned int countdown2 = 0; 
volatile unsigned int countdown3 = 0; 
volatile unsigned int alive1 = 2; // all data need to be sent once
volatile unsigned int alive2 = 2;
volatile unsigned int alive3 = 2;
volatile unsigned int thousant = 1000;

void before() { 
  pinMode(IR_POWER_PIN, OUTPUT); 
  digitalWrite(IR_POWER_PIN, LOW);
  pinMode(PIR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(DOORBELL_PIN, INPUT_PULLUP);
  pinMode(LIGHT_PIN, INPUT);   // no pull up on analog input
  analogReference(DEFAULT);
  analogRead(LIGHT_PIN); 
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Doorbell Sensor", "1.1");

  // Register all sensors to gw (they will be created as child devices)
  present(PIR_ID, S_MOTION);
  present(RING_ID, S_DOOR);
  present(LIGHT_ID, S_LIGHT_LEVEL);
  present(IR_ID, S_BINARY);
  
}

void setup() {
   Serial.println("setup()");
   // Timer0 is already used for millis() - we'll just interrupt somewhere
   // in the middle and call the TIMER0_COMPA_vect interrupt
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
}

void loop() {
  // Read digital motion value  
  bool tripval = (digitalRead(PIR_SENSOR_PIN) == HIGH);
  if (tripval) {
    // we tripped now, set time and send message if this is the first time
    countdown1 = TRIP_TIME;
    if (!tripped1) {
      Serial.println(1);
      send(pirmsg.set("1"));  // send trip msg
      tripped1 = true;
    }
  } else if (countdown1==0) {
    // send message clear only if timer has expired
    if (tripped1 || alive1==0) {
      Serial.println(0);
      send(pirmsg.set("0"));
      tripped1 = false;
      alive1 = ALIVE_TIME;
    }
  }
  
  // now handle doorbell
  tripval = (digitalRead(DOORBELL_PIN) == LOW);
  if (tripval) {
    countdown2 = RING_TIME;
    if (!tripped2) {
      Serial.println(2);
      send(ringmsg.set("1"));  // send trip msg
      tripped2 = true;
    }
  } else if (countdown2==0) {
    if (tripped2 || alive2==0) {
      Serial.println(0);
      send(ringmsg.set("0"));
      tripped2 = false;
      alive2 = ALIVE_TIME; 
    }
  }
  
  // now handle light sensor
  if (alive3==0 || countdown3==0) {
     unsigned int Light = (1023 - analogRead(LIGHT_PIN)) / 10;
     if (LightValue != Light || alive3==0) {
        Serial.print("Light: ");
        Serial.print(Light);
        send(lightmsg.set(Light));
        LightValue = Light;
     }
     alive3 = ALIVE_TIME;
     countdown3 = LIGHT_TIME;
  }
}

// send IR code to turn off television
unsigned int power[] = {4550,4400,600,1650,550,1650,600,1650,
550,550,600,500,600,550,550,550,600,500,600,1650,600,1600,600,
1650,550,550,600,500,600,550,600,500,600,500,650,450,650,1600,
600,500,650,450,650,500,600,500,600,500,600,550,600,1600,600,
500,650,1600,650,1550,650,1600,650,1550,650,1600,650,1600,600};

void send_ir() {
    Serial.println("send_ir()");
    for (int i = 0; i < 3; i++) {
       irsend.sendRaw(power, sizeof(power)/sizeof(int), 38);
       wait(100);
    }
}

// receive command from Domoticz gateway (only IR signal)
void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type==V_STATUS && message.sensor==IR_ID) {
     Serial.print("Incoming message IR: ");
     Serial.println(message.getBool());
     send_ir(); // both on and off do the same
   } 
}


void onesec(void) {
  if (countdown1>0) countdown1--;
  if (countdown2>0) countdown2--;
  if (countdown3>0) countdown3--;
  if (alive1>0) alive1--;
  if (alive2>0) alive2--;
  if (alive3>0) alive3--;
}

// Interrupt on timer0 - called as part of timer0 - already running at 1ms intervals
// alternative was using timer2, but we need the timer for tone()  
//  MsTimer2::set(1000, onesec); // 1000ms period
//  MsTimer2::start();
SIGNAL(TIMER0_COMPA_vect) {
  if (thousant>0)
    thousant--;
  else {
    thousant = 1000;
    onesec();
  }
}
