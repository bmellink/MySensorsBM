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
 * (c) 2017 Bart Mellink - last edits: 15 May 2017
 * 
 * Note: using timer0 to generate a second interrupt for our 1 ms counters. timer0 is still also  used for delay()
 *
 * Functions WTW Flow and Temperature
 * 
 * WTW flow
 *  - Flow of shower cold water
 *  - Flow of boiler cold water entry
 *  
 * Temperature 
 * - Temperature of outgoing water (Dallas with cable)
 * - Temperature of shower head (Dallas)
 * 
 * Flow meter using Honeywell C7195A2001B which has a hall sensor that pulses at 7 Hz per liter/min
 * we count only upgoing pulses, so in our case we get: 
 *   7 pulses per second at 1 l/min
 *   420 pulses per liter
 *   420000 pulses per m3
 * Flow meter Caleffi 316 which has a hall sensor that pulses at 8.8 Hz per liter/min (according to specs)
 * In reality the frequency is about 30% lower in my case, so we have to correct this with a factor of 1.3.
 *   8.8 pulses per second at 1 l/min
 *   528 pulses per liter
 *   528000 pulses per m3
 *   528000/1.3 = 406154
 * 
 * If we use 250 m3 per year we come to 105 x 10^6 pulses/year
 *`this is stored in an unsigned long that can hold 4295 x 10^6 -> volume overflow after 41 years
 *
 *****************************************************************************************************  */

// BOARD: PRO MINI 5V V/ 16Mhz ATMEGA328 16Mhz

// type of flow meter
#define CALEFFI
// #define HONEYWELL

// Enable debug prints to serial monitor
#define MY_DEBUG 1

// Enable and select radio type attached
#define MY_RADIO_RF24
// No repeater 

// node ID's
#define MY_NODE_ID 27             // start naming my own nodes at number 10
#define FLOW_ID 1                 // flow meter 1 (start here if more)
#define NFLOWS 2                  // number of flow meters  
#define TEMP_ID 3                 // Temperature (start here if there are more linked)

#include <SPI.h>
#include <MySensors.h>  
#include <DallasTemperature.h>
#include <OneWire.h>

// PIN connections
uint8_t FlowPins[NFLOWS] = {2, 3};
#define TEMP_PIN 4                // temp sensor 

#ifdef CALEFFI
// #define PULSE_FACTOR 528000        // Nummber of blinks per m3 of your meter Caleffi
#define PULSE_FACTOR 406154        // Nummber of blinks per m3 of your meter Caleffi (measured)
#else
#define PULSE_FACTOR 420000        // Nummber of blinks per m3 of your meter Honeywell
#endif

// configs
#define MAXFLOWERROR 50             // maximum number of l/min that we accept

// delay times
#define CHECK_FREQUENCY 1000         // time in milliseconds between loop (where we check the sensor) - 1000ms
#define MIN_SEND_FREQ 10             // Minimum time between send (in multiplies of CHECK_FREQUENCY). We don't want to spam the gateway (10 seconds)
#define MIN_FLOW_SEND_FREQ 20        // Minimum time between send (in multiplies of CHECK_FREQUENCY). We don't want to spam the gateway (30 seconds)
#define MAX_SEND_FREQ 600            // Maximum time between send (in multiplies of CHECK_FREQUENCY). We need to show we are alive (600 sec/10 min)

// one wire config
#define ONE_WIRE_BUS TEMP_PIN
#define MAX_ATTACHED_DS18B20 8

// Motion message types
MyMessage volumeMsg(FLOW_ID,V_VOLUME);
MyMessage flowMsg(FLOW_ID,V_FLOW);
MyMessage lastCounterMsg(FLOW_ID,V_VAR1);
MyMessage tempmsg(TEMP_ID, V_TEMP);

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
float lastTemperature[MAX_ATTACHED_DS18B20];
volatile unsigned int numSensors = 0;

double ppl = ((double)PULSE_FACTOR / 1000.0);    // Pulses per liter
boolean pcReceived[NFLOWS];                      // received volume from prior reboot
double oldflow[NFLOWS];                          // keep prior flow (only send on change)
unsigned long oldflow_cnt[NFLOWS];               // only send when changed


// updated in ISR
volatile unsigned int mcnt = CHECK_FREQUENCY;   // decremented in ISR at 1000Hz. Cycles at one per second to update send counters
volatile unsigned int tempsendcnt[MAX_ATTACHED_DS18B20];  
volatile unsigned int maxtempsendcnt[MAX_ATTACHED_DS18B20];
volatile unsigned int flowsendcnt[NFLOWS];
volatile unsigned int maxflowsendcnt[NFLOWS];
volatile bool flow_status[NFLOWS];              // status of flow (on or off cycle)
volatile unsigned long flow_cnt[NFLOWS];        // counter volume for each flow sensor
volatile unsigned int intervalcnt[NFLOWS];      // keep track of number of milliseconds since we had previous flow info

void before() { 
  for (uint8_t i=0; i<NFLOWS; i++) {
    pinMode(FlowPins[i], INPUT);
  }
  // Startup up the OneWire library
  sensors.begin();
  for (uint8_t i=0; i<NFLOWS; i++) {
    oldflow[i] = 0;
    flow_status[i] = false;
    flow_cnt[i] = 0;
    oldflow_cnt[i] = 0;
    pcReceived[i] = false;
    flowsendcnt[i] = MIN_FLOW_SEND_FREQ;
    maxflowsendcnt[i] = MAX_SEND_FREQ;
    intervalcnt[i] = 0;
  }
  for (uint8_t i=0; i<MAX_ATTACHED_DS18B20; i++) { 
     lastTemperature[i]=0;
     tempsendcnt[i] = 0;
     maxtempsendcnt[i] = MAX_SEND_FREQ;
  }
}

void setup() {
   Serial.println("setup()");
   // Timer0 is already used for millis() - we'll just interrupt somewhere
   // in the middle and call the TIMER0_COMPA_vect interrupt
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
   sensors.setWaitForConversion(true);
   // Fetch last known pulse count value from gw
   for (uint8_t i=0; i<NFLOWS; i++) {
     request(FLOW_ID+i, V_VAR1);
   }
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("WTW flow sensor", "1.3");

  // Register all sensors to gw (they will be created as child devices)
  numSensors = sensors.getDeviceCount();
  Serial.print("# temp sensors: ");
  Serial.println(numSensors);
  DeviceAddress add;
  for (uint8_t i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) { 
     present(TEMP_ID+i, S_TEMP);
     Serial.print(i);
     Serial.print("=");
     sensors.getAddress(add, i);
     printAddress(add);
     Serial.println();
  }
  for (uint8_t i=0; i<NFLOWS; i++) {
    present(FLOW_ID+i, S_WATER);
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}


void loop() {
  // we come here every 1000 ms (defined in CHECK_FREQUENCY)
  
  // now handle temperature
  if (numSensors>0) {
    sensors.requestTemperatures();
    for (uint8_t i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
      // Fetch and round temperature to one decimal
      float temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(i) * 10.)) / 10.;
  
      // Only send data if temperature has changed and no error
      if (((lastTemperature[i] != temperature && tempsendcnt[i]==0) || maxtempsendcnt[i]==0) && temperature != -127.00 && temperature != 85.00) {
        // Send in the new temperature
        send(tempmsg.setSensor(TEMP_ID+i).set(temperature,1));
        // Save new temperature for next compare
        lastTemperature[i]=temperature;
        tempsendcnt[i] = MIN_SEND_FREQ;
        maxtempsendcnt[i] = MAX_SEND_FREQ;
      }
    }
    for (uint8_t i=0; i<NFLOWS; i++) {
       if ((flowsendcnt[i]==0 && (oldflow_cnt[i] != flow_cnt[i])) || (flowsendcnt[i]==0 && oldflow[i] != 0) || maxflowsendcnt[i]==0) {
          if (!pcReceived[i]) {   //Last Pulsecount not yet received from controller, request it again
             Serial.print("Re-request var1 for sensor ");
             Serial.println(FLOW_ID+i);
             request(FLOW_ID+i, V_VAR1);
             wait(2*CHECK_FREQUENCY); // wait at least 1000 ms for gateway (cannot be sleep or smartSleep)
             return;
          }
          flowsendcnt[i] = MIN_FLOW_SEND_FREQ;
          maxflowsendcnt[i] = MAX_SEND_FREQ;
          double volume = ((double)flow_cnt[i]/((double)PULSE_FACTOR));      
          double flow = (((double) (flow_cnt[i]-oldflow_cnt[i])) * ((double) 60000.0 / ((double) intervalcnt[i]))) / ppl;  // flow in liter/min

          Serial.print("Flow meter:");
          Serial.println(FLOW_ID+i);
          Serial.print("pulsecount:");
          Serial.println(flow_cnt[i]);
          Serial.print("volume:");
          Serial.println(volume, 3);
          Serial.print("l/min:");
          Serial.println(flow);
          intervalcnt[i] = 0;
          oldflow[i] = flow; 
          oldflow_cnt[i] = flow_cnt[i];
          send(lastCounterMsg.setSensor(FLOW_ID+i).set(flow_cnt[i]));     // Send  pulsecount value to gw in VAR1
          send(volumeMsg.setSensor(FLOW_ID+i).set(volume, 3));            // Send volume (set function 2nd argument is resolution)
          if (flow<MAXFLOWERROR) send(flowMsg.setSensor(FLOW_ID+i).set(flow, 2));                // Send flow value to gw
       }
    }
  }

  // Serial.print(end_loop-start_loop);
  wait(CHECK_FREQUENCY);
}

// Receive data from gateway
void receive(const MyMessage &message) {
  for (uint8_t i=0; i<NFLOWS; i++) {
     if (message.type==V_VAR1 && message.sensor==FLOW_ID+i) {
        unsigned long gwPulseCount=message.getULong();
        flow_cnt[i] += gwPulseCount;
        oldflow_cnt[i] += gwPulseCount;
        oldflow[i] = 0;
        Serial.print("Received last pulse count for ");
        Serial.print(FLOW_ID+i);
        Serial.print(" from gw:");
        Serial.println(gwPulseCount);
        pcReceived[i] = true;
     }
  }
}


// Interrupt on timer0 - called as part of timer0 - already running at 1ms intervals
// Alternative:
//  MsTimer2::set(1000, onesec); // 1000ms period
//  MsTimer2::start();
SIGNAL(TIMER0_COMPA_vect) {
  if (mcnt>0) mcnt--;
  for (uint8_t i=0; i<NFLOWS; i++) {
     if (mcnt==0) {
        if (flowsendcnt[i]>0) flowsendcnt[i]--;
        if (maxflowsendcnt[i]>0) maxflowsendcnt[i]--;
     }
     intervalcnt[i]++;
     bool val = digitalRead(FlowPins[i]);
     if (val != flow_status[i]) {
        flow_status[i] = val;
        if (!val) flow_cnt[i]++;      // we increment counter on down flank
     }
  }
  if (mcnt==0) {
     mcnt = CHECK_FREQUENCY;
     for (uint8_t i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) { 
        if (tempsendcnt[i]>0) tempsendcnt[i]--;
        if (maxtempsendcnt[i]>0) maxtempsendcnt[i]--;
     }
  }
}
