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
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 1.1 - GizMoCuz
 * Version 1.2 - changed BM: using low power separate circuit for infra red on pin 8 + analog A1
 * 
 * ISSUES WITH ORIGINAL CODE
 * Incorrect flow calc: micros() was used to calculate the flow, however micros() is wraps every 70 minutes which looks like a huge flow (which is discarded)
 * Volume calc: millis() wraps every 50 days which is not handled correctly
 * Too much current for battery use: The IR LED of the TCRT5000 is always on and the LM393 comparator is also taking a few mA's
 * Could not report flow in sleep mode because millis() does not increment on sleep - need to do this based on calculation of total sleep time
 * 
 * MODIFIED CIRCUIT IR SENSOR
 * Assumption that the wheel of the water meter turns slowly (takes at least a few seconds to turn around)
 * We will wake up every second to turn on the IR LED (connected to PIN 8). Pin 8 also powers the photo transistor that measures the reflection
 * The voltage from the photo transistor is then read using an analog read on A1. Based on a treshold value we will deduct if the mirror is in view
 * Pin 7 is connected to a learning switch which will turn the device in continous mode and the min/max values on A1 are used to recalc the treshold
 * during a 30 second period. After this period the new treshold is established and the LED on Pin 6 will show the actual on/off mirror signals
 * 
 * Number of pulses per liter = 1
 * 
 * DESCRIPTION
 * Use this sensor to measure volume and flow of your house watermeter.
 * You need to set the correct pulsefactor of your meter (pulses per m3).
 * The sensor starts by fetching current volume reading from gateway (VAR 1).
 * Reports both volume and flow back to gateway.
 *
 * http://www.mysensors.org/build/pulse_water
 */

// BOARD: PRO MINI 3.3V/ 8Mhz ATMEGA328 8Mhz
// NOTE SET POWER LEVEL TO MAX IN MyConfig.h:
// define MY_RF24_PA_LEVEL (RF24_PA_MAX) 

// Enable debug prints to serial monitor (disable when going into production)
// #define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_RF24
//#define MY_RADIO_RFM69

#define MY_NODE_ID 10                 // start naming my own nodes at number 10
#include <SPI.h>
#include <MySensors.h>  

#define SENSOR_POWER 8                // pin that will provide power to IR LED + sense circuit
#define IR_SENSE_PIN  A1              // input for IR voltage
#define BATTERY_SENSE_PIN  A0         // select the input pin for the battery sense point
#define LEARN_SWITCH_PIN 7            // switch (SW1 on battery module) to turn on learning mode (low==on)
#define LEARN_LED_PIN 6               // LED feedback during learning mode (LED on battery module)
#define LEARN_TIME 30                 // number of seconds we will keep learn loop

#define PULSE_FACTOR 1000             // Nummber of blinks per m3 of your meter (One rotation/1 liter)
#define MAX_FLOW 80                   // Max flow (l/min) value to report. This filters outliers.
#define CHILD_ID 1                    // Id of the sensor child (contains 3 subs: V_FLOW, V_VOLUME, VAR1)
#define CHILD_PINGID 2                // ID of ping counter
#define CHILD_ERRID 3                 // ID of error counter

#define CHECK_FREQUENCY 500           // time in milliseconds between loop (where we check the sensor) - 500ms   
#define MIN_SEND_FREQ 60              // Minimum time between send (in multiplies of CHECK_FREQUENCY). We don't want to spam the gateway (30 seconds)
#define MAX_SEND_FREQ 1200            // Maximum time between send (in multiplies of CHECK_FREQUENCY). We need to show we are alive (600 sec/10 min)
#define IR_ON_SETTLE 2                // number of milliseconds after we turned on the IR LED and we asume the receive signal is stable (in ms)
#define EE_TRESHOLD 10                // config addresses 0 + 1 used for treshold from learning (loadState() returns only uint8 value)
#define TRESHOLD_MARGIN 3             // additional margin before we actually see a one or zero
#define RESETMIN 5                    // number of cycle times (either 30 sec of 10 min) we consistenctly need to have transmission errors before we perform hard reset

MyMessage volumeMsg(CHILD_ID,V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID,V_VAR1);
MyMessage flowMsg(CHILD_ID,V_FLOW);
MyMessage pingMsg(CHILD_PINGID,V_DISTANCE); // use distance to keep track of changing value
MyMessage errMsg(CHILD_ERRID,V_DISTANCE); // use distance to keep track of changing value


double ppl = ((double)PULSE_FACTOR / 1000.0);    // Pulses per liter
unsigned int oldBatteryPcnt = 0;          // check if changed
unsigned int minsendcnt = MIN_SEND_FREQ;  // counter for keeping minimum intervals between sending
unsigned int maxsendcnt = MAX_SEND_FREQ;  // counter for keeping maximum intervals between sending 
unsigned int treshold = 512;              // threshold value when to swap on/off for pulse
unsigned long pulseCount = 0;             // total volume of this pulse meter (value stored/received on gateway on pcReceived)
unsigned long oldPulseCount = 0;          // to see if we have received something
boolean pcReceived = false;               // received volume from prior reboot
boolean onoff = false;                    // sensor value above/below treshold 
unsigned int intervalcnt = 0;             // number of cycles between last period (for flow calculation)
double flow = 0;                          // maintain flow
double oldflow = 0;                       // keep prior flow (only send on change)
unsigned int learntime=LEARN_TIME*2;      // timer for learning period
unsigned int learnlow = 1023;             // lowest value found during learning
unsigned int learnhigh = 0;               // highest value found during learning
boolean learnsaved = false;               // have saved learned value
unsigned long pingcnt = 0;
unsigned long errcnt = 0;                 // error count
unsigned int errcnt2 = 0;                 // error counter set to 0 when sending is ok

void(* resetFunc) (void) = 0;//declare reset function at address 0


void before() { 
  Serial.println("before()");
  // make sure a few vars have the right init value after softw reset
  pingcnt = 0;
  pcReceived = false;
  pulseCount = oldPulseCount = 0;

  // setup hardware
  pinMode(SENSOR_POWER, OUTPUT); 
  digitalWrite(SENSOR_POWER, LOW);
  pinMode(LEARN_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LEARN_LED_PIN, INPUT);      // default is input because this pin also has SW2 of battery block
}


void setup() {    
  Serial.println("setup()");

  // Fetch trshold value from EE prom
  treshold = readEeprom(EE_TRESHOLD);
  if (treshold<30 || treshold>1000) treshold = 512;   // wrong value in EEprom, take default
  Serial.print("Treshold: ");
  Serial.println(treshold);
        
  // use the 1.1 V internal reference for the battery and IR sensor
#if defined(__AVR_ATmega2560__)
   analogReference(INTERNAL1V1);
#else
   analogReference(INTERNAL);
#endif
  analogRead(IR_SENSE_PIN); // settle analogreference value
  wait(CHECK_FREQUENCY); // wait a bit

  // Fetch last known pulse count value from gw
  request(CHILD_ID, V_VAR1);
  wait(CHECK_FREQUENCY); // wait a bit
}

void presentation()  {
  Serial.println("Presentation()");
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Water Meter", "1.3");

  // Register this device as Waterflow sensor
  present(CHILD_ID, S_WATER);      
  present(CHILD_PINGID, S_DISTANCE); 
  present(CHILD_ERRID, S_DISTANCE);
}

void loop() {
  if (digitalRead(LEARN_SWITCH_PIN)==LOW) {
    pinMode(LEARN_LED_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER, HIGH);
    intervalcnt = 0;
    learn_loop();
  } else {
    learntime=LEARN_TIME*2;
    learnlow = 1023;
    learnhigh = 0;
    pinMode(LEARN_LED_PIN, INPUT);
    normal_loop();
  }
}

void learn_loop() {
  // will run into this loop as long as we are learning
  wait(500);
  unsigned int sensorValue = analogRead(IR_SENSE_PIN);
  Serial.print("IR: ");
  Serial.print(sensorValue);
  if (learntime>0) {
    // still learning
    learntime--;
    learnsaved = false;    
    digitalWrite(LEARN_LED_PIN, !digitalRead(LEARN_LED_PIN));  // blink led
    if (sensorValue < learnlow) {
      learnlow = sensorValue;
      Serial.println(" Lowest");
    } else if (sensorValue > learnhigh) {
      learnhigh = sensorValue;
      Serial.println(" Highest");
    } else Serial.println();
  } else {
    if (!learnsaved) {
      treshold = (learnhigh + learnlow)/2;
      Serial.print("Treshold: ");
      Serial.println(treshold);
      storeEeprom(EE_TRESHOLD, treshold);
    }
    learnsaved = true;
    // just display using LED
    digitalWrite(LEARN_LED_PIN, sensorValue>treshold);
    Serial.println((sensorValue>treshold ? " on" : " off"));
  }
}

void normal_loop() { 
  unsigned long start_loop = millis();    // to allow adjusting wait time
  intervalcnt++;
  // we start with doing a measurement
  digitalWrite(SENSOR_POWER, HIGH);
  wait(IR_ON_SETTLE); 
  unsigned int sensorValue = analogRead(IR_SENSE_PIN);
  digitalWrite(SENSOR_POWER, LOW); 
  #ifdef MY_DEBUG_DETAIL
  Serial.print("IR: ");
  Serial.println(sensorValue);
  #endif
  boolean nowvalue = onoff;
  if (onoff && (sensorValue<treshold-TRESHOLD_MARGIN)) nowvalue = false;
  if (!onoff && (sensorValue>treshold+TRESHOLD_MARGIN)) nowvalue = true;
  if (nowvalue != onoff) {
    // we have a pulse, only count on upwards pulse
    onoff = nowvalue;
    if (onoff) {
      pulseCount++;
      #ifdef MY_DEBUG
      Serial.print("p: ");
      Serial.println(pulseCount);
      #endif
    }
  }

    // Only send values at a maximum frequency or woken up from sleep
  if (minsendcnt>0) minsendcnt--;
  if (maxsendcnt>0) maxsendcnt--;
  // send minimum interval when we have pulse changes or if we had some flow the prior time or send on timeout
  if ((minsendcnt==0 && (pulseCount != oldPulseCount)) || (minsendcnt==0 && oldflow != 0) || maxsendcnt==0) {
    if (!pcReceived) {   //Last Pulsecount not yet received from controller, request it again
      Serial.print("Re-request var1 ..");
      request(CHILD_ID, V_VAR1);
      wait(2*CHECK_FREQUENCY); // wait at least 1000 ms for gateway (cannot be sleep or smartSleep)
      return;
    }
    minsendcnt = MIN_SEND_FREQ;
    maxsendcnt = MAX_SEND_FREQ;
    pingcnt++;

    sensorValue = analogRead(BATTERY_SENSE_PIN);
    int batteryPcnt = sensorValue / 10;
    // 1M, 470K divider across battery and using internal ADC ref of 1.1V
    // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
    // ((1e6+470e3)/1e6)*1.1 = Vmax = 1.67 Volts
    // 1.67/1023 = Volts per bit = 0.00158065

    Serial.print("Battery %: ");
    Serial.println(batteryPcnt);

    if (oldBatteryPcnt != batteryPcnt) {
      sendBatteryLevel(batteryPcnt);
      oldBatteryPcnt = batteryPcnt;
    }
    double volume = ((double)pulseCount/((double)PULSE_FACTOR));      
    flow = ((double) (pulseCount-oldPulseCount)) * (60000.0 / ((double) intervalcnt*(double) CHECK_FREQUENCY)) / ppl;  // flow in liter/min

    #ifdef MY_DEBUG
    Serial.print("pulsecount:");
    Serial.println(pulseCount);
    Serial.print("volume:");
    Serial.println(volume, 3);
    Serial.print("l/min:");
    Serial.println(flow);
    #endif
       
    bool b = send(lastCounterMsg.set(pulseCount));  // Send  pulsecount value to gw in VAR1
    if (b) errcnt2=0; else { errcnt++; errcnt2++; }
    b = send(volumeMsg.set(volume, 3));               // Send volume (set function 2nd argument is resolution)
    if (b) errcnt2=0; else { errcnt++; errcnt2++; }
    b = send(flowMsg.set(flow, 2));                   // Send flow value to gw
    if (b) errcnt2=0; else { errcnt++; errcnt2++; }
    b = send(pingMsg.set(pingcnt));                   // ensure at least this var has a different value
    if (b) errcnt2=0; else { errcnt++; errcnt2++; }
    b = send(errMsg.set(errcnt2+((float) errcnt2/100),2));    // ensure we always send error count
    if (b) errcnt2=0; else { errcnt++; errcnt2++; }
    oldPulseCount = pulseCount;
    intervalcnt = 0;
    oldflow = flow; 
    if (errcnt2>= (5*RESETMIN)) {
      Serial.println("Reset");
      wait(300);
      resetFunc(); //call reset     
    }
  }
  unsigned long end_loop = millis();
  // Serial.print(end_loop-start_loop);
  if (end_loop - start_loop < CHECK_FREQUENCY)
    sleep(CHECK_FREQUENCY - (end_loop > start_loop ? end_loop - start_loop : 0));
}

void receive(const MyMessage &message) {
  if (message.type==V_VAR1) {
    unsigned long gwPulseCount=message.getULong();
    pulseCount += gwPulseCount;
    oldPulseCount += gwPulseCount;
    flow=oldflow=0;
    Serial.print("Received last pulse count from gw:");
    Serial.println(pulseCount);
    pcReceived = true;
  }
}


void storeEeprom(int pos, int value) {
    // function for saving the values to the internal EEPROM
    // value = the value to be stored (as int)
    // pos = the first byte position to store the value in
    // only two bytes can be stored with this function (max 32.767)
    saveState(pos, ((unsigned int)value >> 8 ));
    pos++;
    saveState(pos, (value & 0xff));
}

int readEeprom(int pos) {
    // function for reading the values from the internal EEPROM
    // pos = the first byte position to read the value from 

    int hiByte;
    int loByte;

    hiByte = loadState(pos) << 8;
    pos++;
    loByte = loadState(pos);
    return (hiByte | loByte);
}
