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
 * Functions DINING ROOM
 * - PIR movement in dining room
 * - Temperature (with Dallas temp)
 * - 2 relais for lamp 1 and 2 (on could be HIGH or LOW depending on other switch)
 * - 2 opto couplers to measure if lamp is on or off (low is on)
 * - light sensor with LDR for ambiant light. Low value is more light
 * 
 * The relais are part of a "hotel" switch circuit, so if the relais is on, the lamp can be off or the reverse.
 * We use the opto coupler to actually measure if the lamp is on or off and force the relais in the other direction if we want to change the status
 *******************************  */


// BOARD: PRO MINI 5V V/ 16Mhz ATMEGA328 8Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 1

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_REPEATER_FEATURE       // repeater

// node ID's
#define MY_NODE_ID 15             // start naming my own nodes at number 10
#define LAMP1_ID 1                // lamp 1
#define LAMP2_ID 2                // lamp 2
#define PIR_ID 3                  // sensor ID PIR movement
#define TEMP_ID 4                 // Temperature
#define LIGHT_ID 5                // Light sensor
#define CHILD_PINGID 8            // ID of ping counter

#include <SPI.h>
#include <MySensors.h>  
#include <DallasTemperature.h>
#include <OneWire.h>

// PIN connections
#define PIR_PIN 2                 // PIR movement sensor
#define TEMP_PIN 3                // temp sensor (must be 2 or 3)
#define OPTO1_PIN 5               // output opto coupler lamp1 (low is on)
#define OPTO2_PIN 6               // output opto coupler lamp2 (low is on)
#define RELAIS1_PIN 7             // switch lamp1
#define RELAIS2_PIN 8             // switch lamp2
#define LIGHT_PIN A0              // analog input light (lower voltage is higher intensity)

// delay times
#define CHECK_FREQUENCY 500       // time in milliseconds between loop (where we check the sensor) - 500ms - 0.5 sec
#define MIN_SEND_FREQ 30          // Minimum time between send (in multiplies of CHECK_FREQUENCY). We don't want to spam the gateway (15 seconds)
#define MIN_SEND_FREQ_ERR 20      // Minimum time between sending our last data again in case of comm error (multiplied by CHECK_FREQUENCY) - 10 sec
#define MIN_SEND_FREQ_SW 2        // Minimum time between setting switch and testing if switch was effective (1 sec)
#define MAX_SEND_FREQ 600         // Maximum time between send (in multiplies of CHECK_FREQUENCY). We need to show we are alive (300 sec/5 min)

// one wire config
#define ONE_WIRE_BUS TEMP_PIN
#define MAX_ATTACHED_DS18B20 16

// configs
#define MAXERRCNT 20               // max error count before we do reboot

// Motion message types
MyMessage lamp1msg(LAMP1_ID, V_STATUS);     // status of lamp1
MyMessage lamp2msg(LAMP2_ID, V_STATUS);     // status of lamp2
MyMessage pirmsg(PIR_ID, V_TRIPPED);        // movement
MyMessage tempmsg(TEMP_ID, V_TEMP);         // temperature
MyMessage lightmsg(LIGHT_ID, V_LIGHT_LEVEL); // value in % 0-100
MyMessage pingMsg(CHILD_PINGID,V_DISTANCE); // use distance to keep track of changing value (increment until reboot)

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

bool metric = true;
unsigned int oldBatteryPcnt = 1;          // check if changed
unsigned int numSensors = 0;

unsigned int minsendcnt = MIN_SEND_FREQ;  // counter for keeping minimum intervals between sending 
unsigned int maxsendcnt = MAX_SEND_FREQ;  // counter for keeping maximum intervals between sending 
unsigned int pirsendcnt = 0;              // check PIR statum immediately. Then set to MIN_SEND_FREQ;  
unsigned int tempsendcnt = MIN_SEND_FREQ;  
unsigned int tempsendcntmax = MAX_SEND_FREQ;
unsigned int lightcnt = MIN_SEND_FREQ;
unsigned int switchcnt = 0;               // check switches immediately (MIN_SEND_FREQ_SW set when receiving data)
unsigned long pingcnt = 0;                // ping counter (increments at each temperature emasurement, reset at reboot)
unsigned int errcnt = 0;                  // send message error counter
bool initdone = false;                    // not yet init fully completed

void(* resetFunc) (void) = 0;//declare reset function at address 0

// sensor value intermediate storage
bool lamp1relais = false;     // value of the lamp1 relais (on/off). Note that if the relais is on, the lamp can still be on or off
bool lamp2relais = false;
bool lamp1status = false;     // status of lamp1 (on/off)
bool lamp2status = false;     // status of lamp2 (on/off)
bool tripped1 = false;        // save value for PIR
unsigned int lastlight = 0;   // save value for light
float lastTemperature = 0;    // save value for temperature

void before() { 
   // before comm library is started, restore relais setting right at the start
   errcnt = 0;
   pingcnt = 0;
   lamp1relais =  loadState(1);
   lamp2relais =  loadState(2);
   pinMode(OPTO1_PIN, INPUT_PULLUP);
   pinMode(OPTO2_PIN, INPUT_PULLUP);
  
   pinMode(RELAIS1_PIN, OUTPUT); 
   digitalWrite(RELAIS1_PIN, lamp1relais);
   pinMode(RELAIS2_PIN, OUTPUT); 
   digitalWrite(RELAIS2_PIN, lamp2relais);
  
   pinMode(PIR_PIN, INPUT);
   pinMode(LIGHT_PIN, INPUT);   // no pull up on analog input
   analogReference(DEFAULT);
   analogRead(LIGHT_PIN); 

   // Startup up the OneWire library
   sensors.begin();
}

void setup() {
   // please note that setup happens before presentation(), so we cannot call send()
   sensors.setWaitForConversion(true);
}

void presentation()  {
   // Send the sketch version information to the gateway and Controller
   sendSketchInfo("Dining Room Sensors", "1.2");

   // Register all sensors to gw (they will be created as child devices)
   present(LAMP1_ID, S_BINARY);
   present(LAMP2_ID, S_BINARY);
   present(PIR_ID, S_MOTION);
   present(LIGHT_ID, S_LIGHT_LEVEL);
   present(CHILD_PINGID, S_DISTANCE); 

   numSensors = sensors.getDeviceCount();
   Serial.print("# temp sensors: ");
   Serial.println(numSensors);
   if (numSensors) present(TEMP_ID, S_TEMP);
}


void loop() {
   // we come here every 500 ms (defined in CHECK_FREQUENCY)
   unsigned long start_loop = millis();    // to allow adjusting wait time
   bool stat;

   if (switchcnt>0) switchcnt--; else {
      // Handle lamp1 status changes
      stat = !digitalRead(OPTO1_PIN);
      if (stat != lamp1status || !initdone) {
        if (send(lamp1msg.set((stat ? "1" : "0")))) {
          lamp1status = stat;
        } else errcnt++;
      }
      
      // Handle lamp2 status changes
      stat = !digitalRead(OPTO2_PIN);
      if (stat != lamp2status || !initdone) {
        if (send(lamp2msg.set((stat ? "1" : "0")))) {
          lamp2status = stat;
        } else errcnt++;
      }
   }
   // Handle PIR motion 
   bool tripval = (digitalRead(PIR_PIN) == HIGH);
   if (pirsendcnt>0) pirsendcnt--;
   if ((tripval != tripped1 && pirsendcnt == 0) || !initdone) {
      Serial.print("PIR: "); Serial.println(tripval);
      if (send(pirmsg.set((tripval ? "1" : "0")))) {
         tripped1 = tripval;
         pirsendcnt = MIN_SEND_FREQ;  
      } else {
         pirsendcnt = MIN_SEND_FREQ_ERR;  
         errcnt++;
      }
   }
  
   // now handle temperature
   if (tempsendcnt>0) tempsendcnt--;
   if (tempsendcntmax>0) tempsendcntmax--;
   if (numSensors>0 && tempsendcnt == 0) {
      sensors.requestTemperatures();
      // Fetch and round temperature to one decimal
      float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric?sensors.getTempCByIndex(0):sensors.getTempFByIndex(0)) * 10.)) / 10.;
    
      // Only send data if temperature has changed and no error or we need to do it anyway
      if ((lastTemperature != temperature || tempsendcntmax==0) && temperature != -127.00 && temperature != 85.00) {
         // Send in the new temperature
         if (send(tempmsg.setSensor(TEMP_ID).set(temperature,1))) {
            // Save new temperature for next compare if we were able to send data
            lastTemperature=temperature;
            tempsendcnt = MIN_SEND_FREQ;  
            tempsendcntmax = MAX_SEND_FREQ;  
         } else {
            tempsendcnt = MIN_SEND_FREQ_ERR;
            errcnt++;
         }
         pingcnt++;
         send(pingMsg.set(pingcnt));
      }
   }

   // now handle light sensor
   if (lightcnt>0) lightcnt--;
   unsigned int light = (1023 - analogRead(LIGHT_PIN)) / 10;
   if (lightcnt==0 && light!=lastlight) {
      Serial.print("Light: "); Serial.print(light);
      if (send(lightmsg.set(light))) {
         lastlight = light;
         lightcnt = MIN_SEND_FREQ;
      } else errcnt++;        
   }

   if (errcnt>=MAXERRCNT) resetFunc(); //call reboot
   initdone = true;
   
   // ready with all. Now wait for a while
   unsigned long end_loop = millis();
   // Serial.print(end_loop-start_loop);
   if (end_loop-start_loop<=CHECK_FREQUENCY)
      wait(CHECK_FREQUENCY - (end_loop-start_loop));
}

// receive command from Domoticz gateway (only lamp1/lamp2 on off signal)
void receive(const MyMessage &message) {
   // We only expect one type of message from controller. Only switch the relais if the lamp actually needs to change
   bool val;
   if (message.type==V_STATUS && message.sensor==LAMP1_ID) {
      Serial.print("Relais 1-");
      val = message.getBool();
      if (val != lamp1status) {
         switchcnt = MIN_SEND_FREQ_SW;
         lamp1status = val;
         lamp1relais = !lamp1relais;
         digitalWrite(RELAIS1_PIN, lamp1relais);
         saveState(1, lamp1relais);
         Serial.println(lamp1relais);
      } else Serial.println();
   } 
   if (message.type==V_STATUS && message.sensor==LAMP2_ID) {
      Serial.print("Relais 2-");
      val = message.getBool();
      if (val != lamp2status) {
         switchcnt = MIN_SEND_FREQ_SW;
         lamp2status = val;
         lamp2relais = !lamp2relais;
         digitalWrite(RELAIS2_PIN, lamp2relais);
         saveState(2, lamp2relais);
         Serial.println(lamp2relais);
      } else Serial.println();
   } 
}

