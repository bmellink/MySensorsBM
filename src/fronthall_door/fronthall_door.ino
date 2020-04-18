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
 * Functions FRONT HALL DOOR
 * - Battery operated
 * - PIR movement in hallway
 * - Door open/close contact (closed door is closed contact =0) - reed contact
 * - Door locked using IR sender receiver (locked signal is higher voltage on A1)
 * - Temperature (with Dallas temp)
 * - Battery power %
 * Difference with sidehall is the DIFFLOCKED code and ID
 *******************************  */


// BOARD: PRO MINI 3.3V V/ 8Mhz ATMEGA328 8Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 1

// Enable and select radio type attached
#define MY_RADIO_NRF24
// No repeater (battery)

// node ID's
#define MY_NODE_ID 14             // start naming my own nodes at number 10
#define PIR_ID 1                  // sensor ID PIR movement
#define DOOROPEN_ID 2             // Door open contact
#define DOORLOCK_ID 3             // Door locked contact
#define TEMP_ID 4                 // Temperature
#define DIST_ID 5                 // Distance between LED on and OFF
#define CHILD_PINGID 8            // ID of ping counter
#define BATT_ID 9                 // Batt percentage 0..100%

#include <SPI.h>
#include <MySensors.h>  
#include <DallasTemperature.h>
#include <OneWire.h>


// PIN connections
#define PIR_PIN 2
#define DOOROPEN_PIN 5            // door open (0 = closed)
#define DOORLOCK_PIN A1           // door lock receive (low voltage is locked)
#define DOORLOCK_PWR1 8           // doorlock power to receiver (=1)
#define DOORLOCK_PWR2 6           // doorlock pwoer to LED (=1)
#define TEMP_PIN 3                // temp sensor (must be 2 or 3)
#define BATTERY_SENSE_PIN  A0         // select the input pin for the battery sense point


// delay times
#define CHECK_FREQUENCY 6000         // time in milliseconds between loop (where we check the sensor) - 6000ms - 6 sec
#define MIN_SEND_FREQ 5              // Minimum time between send (in multiplies of CHECK_FREQUENCY). We don't want to spam the gateway (30 seconds)
#define MIN_SEND_FREQ_PIR 50         // Minimum time between sending IR signal (5 minutes)
#define MIN_SEND_FREQ_ERR 2          // Minimum time between sending our last data again in case of comm error (multiplied by CHECK_FREQUENCY) - 12 sec
#define MAX_SEND_FREQ 150            // Maximum time between send (in multiplies of CHECK_FREQUENCY). We need to show we are alive (900 sec/15 min)
#define PWR_ON_SETTLE 2              // number of milliseconds after we turned on the IR LED for the lock and we asume the receive signal is stable (in ms)

// one wire config
#define ONE_WIRE_BUS TEMP_PIN
#define MAX_ATTACHED_DS18B20 16

// configs
#define MINIMALLOCK 400            // minimal lock value. ADS value should be below this when LED is off (no direct sunlight on sensor) .. range 0..1023
#define DIFFLOCK 190               // difference in lock value when we asume lock is unlocked (extra light comes form LED)
#define DIFFLOCKED 300             // difference in lock value when fully locked (mechanical touch/ shortcut)
#define MAXERRCNT 20               // max error count before we do reboot

// Motion message types
MyMessage pirmsg(PIR_ID, V_TRIPPED);
MyMessage openmsg(DOOROPEN_ID, V_TRIPPED);
MyMessage lockmsg(DOORLOCK_ID, V_TRIPPED);
MyMessage tempmsg(TEMP_ID, V_TEMP);
MyMessage pingMsg(CHILD_PINGID,V_DISTANCE); // use distance to keep track of changing value
MyMessage distmsg(DIST_ID,V_DISTANCE); // use distance to show the lock difference between on/off LED
MyMessage battmsg(BATT_ID, V_PERCENTAGE);

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
float lastTemperature = 0;
bool receivedConfig = false;
bool metric = true;
unsigned int oldBatteryPcnt = 1;          // check if changed
unsigned int numSensors = 0;
int olddist = 1;

unsigned int minsendcnt = MIN_SEND_FREQ;  // counter for keeping minimum intervals between sending 
unsigned int maxsendcnt = MAX_SEND_FREQ;  // counter for keeping maximum intervals between sending 
unsigned int pirsendcnt = MIN_SEND_FREQ;  
unsigned int tempsendcnt = MIN_SEND_FREQ;  
unsigned int tempsendcntmax = MAX_SEND_FREQ;  
unsigned int distsendcnt = MIN_SEND_FREQ;  
unsigned long pingcnt = 0;
unsigned int errcnt = 0;
bool initdone = false;                    // not yet init fully completed

void(* resetFunc) (void) = 0;//declare reset function at address 0

// sensor value intermediate storage
bool tripped1 = false;
bool tripped2 = false;
bool tripped3 = false;
bool lockset = false; // if we have read this once

void before() { 
  errcnt = 0;
  pingcnt = 0;
  pinMode(DOORLOCK_PWR1, OUTPUT); 
  digitalWrite(DOORLOCK_PWR1, LOW);
  pinMode(DOORLOCK_PWR2, OUTPUT); 
  digitalWrite(DOORLOCK_PWR2, LOW);
  
  pinMode(PIR_PIN, INPUT);
  pinMode(DOOROPEN_PIN, INPUT_PULLUP);
  pinMode(DOORLOCK_PIN, INPUT);   // no pull up on analog input
#if defined(__AVR_ATmega2560__)
   analogReference(INTERNAL1V1);
#else
   analogReference(INTERNAL);
#endif
  analogRead(BATTERY_SENSE_PIN); // settle analogreference value
   // Startup up the OneWire library
  sensors.begin();
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Front Hall Sensors", "1.2");

  // Register all sensors to gw (they will be created as child devices)
  present(PIR_ID, S_MOTION);
  present(DOOROPEN_ID, S_BINARY);
  present(DOORLOCK_ID, S_BINARY);
  present(DIST_ID, S_DISTANCE); 
  present(CHILD_PINGID, S_DISTANCE); 
  present(BATT_ID, V_PERCENTAGE);

  numSensors = sensors.getDeviceCount();
  Serial.print("# temp sensors: ");
  Serial.println(numSensors);
  if (numSensors) present(TEMP_ID, S_TEMP);
}

void setup() {
   // please note that setup happens before presentation(), so we cannot call send()
   Serial.println("setup()");
   sensors.setWaitForConversion(true);
}

void loop() {
  // we come here every 6000 ms (defined in CHECK_FREQUENCY)
  unsigned long start_loop = millis();    // to allow adjusting wait time
  pinMode(DOOROPEN_PIN, INPUT_PULLUP);  // provide power to pull up for door open/close
  
  // Read digital motion value  
  bool tripval = (digitalRead(PIR_PIN) == HIGH);
  if (pirsendcnt>0) pirsendcnt--;
  if ((tripval != tripped1 && pirsendcnt == 0) || !initdone) {
    Serial.print("PIR: "); Serial.println(tripval);
    if (send(pirmsg.set((tripval ? "1" : "0")))) {
      tripped1 = tripval;
      pirsendcnt = (tripval ? MIN_SEND_FREQ : MIN_SEND_FREQ_PIR) ;  // if we go from 1 to 0 then we have to wait at least 5 minutes
    } else {
      pirsendcnt = MIN_SEND_FREQ_ERR;  
      errcnt++;
    }
  }
  
  // now handle door open
  tripval = (digitalRead(DOOROPEN_PIN) == HIGH);
  if (tripval != tripped2 || !initdone) {
    Serial.print("Door open: "); Serial.println(tripval);
    if (send(openmsg.set((tripval ? "1" : "0")))) {
      tripped2 = tripval; 
    } else errcnt++;
  }
  pinMode(DOOROPEN_PIN, INPUT);  // remove pull for door open/close

  // now handle door locked
  // start with turning on power for doorlock LED + receive circuit
  // This also solves the issue of having sunlight shining directly getting into the IR receiver 
  // First we only turn on the power of the receiver circuit, we measure the output (should be low voltage)
  // if the output turns out to be high, we skip measurement
  digitalWrite(DOORLOCK_PWR1, HIGH);
  wait(PWR_ON_SETTLE);
  unsigned int analock1 = analogRead(DOORLOCK_PIN);
  int diff = -1; // error distance value
  Serial.print(analock1);
  if (analock1<MINIMALLOCK) {
    // so this looks right now turn on second part  
    digitalWrite(DOORLOCK_PWR2, HIGH);
    wait(PWR_ON_SETTLE);
    unsigned int analock2 = analogRead(DOORLOCK_PIN);
    Serial.print("->");
    Serial.print(analock2);
    // lock is locked either when difference is very small or when differenc is really big due to
    // shortcut by metal lock bar, which seems to happen due to mechanical construction
    diff = analock2 - analock1;
    // tripval = (analock2 < analock1+DIFFLOCK); // this is the code as it should be
    tripval = (diff < DIFFLOCK || diff > DIFFLOCKED);
    digitalWrite(DOORLOCK_PWR1, LOW);   // power off
    digitalWrite(DOORLOCK_PWR2, LOW);
  } else {
    digitalWrite(DOORLOCK_PWR1, LOW);
    Serial.print(" Doorlock err assume locked");
    tripval = true;
  }
  if (tripval != tripped3 || !lockset) {
    Serial.print(" Locked: "); Serial.println(tripval);
    if (send(lockmsg.set((tripval ? "1" : "0")))) {
      tripped3 = tripval;
      lockset = true;    
    } else errcnt++;
  } else Serial.println(" ");
  if (distsendcnt>0) distsendcnt--;
  if (distsendcnt==0 && diff!=olddist) {
    if (send(distmsg.set(diff))) {
      olddist = diff;
      distsendcnt = MIN_SEND_FREQ; 
    } else distsendcnt = MIN_SEND_FREQ_ERR;
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

  // finally handle battery level
  unsigned int sensorValue = analogRead(BATTERY_SENSE_PIN);
  int batteryPcnt = sensorValue / 10;
  if (batteryPcnt>100) batteryPcnt = 100;
  // 1M, 470K divider across battery and using internal ADC ref of 1.1V
  // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
  // ((1e6+470e3)/1e6)*1.1 = Vmax = 1.67 Volts
  // 1.67/1023 = Volts per bit = 0.00158065
  if (minsendcnt>0) minsendcnt--;
  if (maxsendcnt>0) maxsendcnt--;
  if ((minsendcnt==0 && oldBatteryPcnt != batteryPcnt) || maxsendcnt==0) {
    minsendcnt = MIN_SEND_FREQ;
    maxsendcnt = MAX_SEND_FREQ;
    Serial.print("Battery %: ");
    Serial.println(batteryPcnt);
    // no check if we really send the data.... just wait until the next loop if we fail
    // send 2 ways: as as separate ID and as internal percentage
    sendBatteryLevel(batteryPcnt);
    send(battmsg.set(batteryPcnt));
    oldBatteryPcnt = batteryPcnt;
  }

  if (errcnt>=MAXERRCNT) resetFunc(); //call reboot
  initdone = true;

  // ready with all. Now sleep for a while
  unsigned long end_loop = millis();
  // Serial.print(end_loop-start_loop);
  if (end_loop-start_loop<=CHECK_FREQUENCY)   // added 7-1-2017
    sleep(CHECK_FREQUENCY - (end_loop>start_loop ? end_loop-start_loop : 0));
}



