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
 * Functions WTW Temperatures - location in hallway closet
 * - Temperature (with Dallas temp) - 4 times
 * - Temperature of incoming water (also Dallas with cable)
 * - analog input for power of hydrofoor pump
 * 
 * The power usage of the pump uses ACS712-5A hall sensor:
 * - At rest this circuit outputs VCC/2 -> 2.5 V (ADC=511)
 * - The extra diode in the circuit lowers the value a bit (2.21V). ADC output at rest is approx 452
 * - The characteristic is 185 mV/A (1A current is 185mV higher, ADC value 37.8 higher)
 * - 1A current (37.8 ADC ticks) is 220Watt -> factor is 5.82 for DC and 4.64 for AC (difference Vpeak and RMS)
 *   Based on measurements the factor should be 8.35
 *   
 * We send both the current power (in Watt) and the total usage (in kWh) to the gateway. 
 * Use VAR1 as storage of the current count (in case of reboot). In VAR1 we store the value in cWh (centi-Watt-hour = 0.01 Wh = 0.00001 kWh)
 * kept in a unsigned long (32 bits). Largest value there is 42949 kWH - which will roll over once ever 24 years on an everage usage of 200 W
 * In order to calculate the kWh number, we add the power usage once per second in a mWh variable once per second which is devided by 10 (to cWh)
 * when adding to the VAR1 value and the total deviced by 100000 before sending as float to the device 
 * 
 *******************************  */


// BOARD: PRO MINI 5V V/ 16Mhz ATMEGA328 16Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 1

// Enable and select radio type attached
#define MY_RADIO_NRF24
// No repeater (battery)

#define MAX_ATTACHED_DS18B20 8

// node ID's
#define MY_NODE_ID 26             // start naming my own nodes at number 10
#define TEMP_ID 1                 // Temperature (start here)
#define PUMP_MOTOR_ID (1+MAX_ATTACHED_DS18B20)  // Pump running power (watt) + kwh + VAR1
#define PUMP_RUNNING_ID (1+PUMP_MOTOR_ID)         // Pump on off status

#include <SPI.h>
#include <MySensors.h>  
#include <DallasTemperature.h>
#include <OneWire.h>

// PIN connections
#define TEMP_PIN 3                // temp sensor (must be 2 or 3)
#define PUMP_CURRENT_PIN A0       // Current measurement pump
#define ONE_WIRE_BUS TEMP_PIN

// Dallas addresses
// ID ADDRESS        sensor  location
// 0  28FF4A07B31604E5 (1)   L onder
// 1  28FFBAA4B2160597 (2)   L boven
// 2  28FF4905B316032F (3)   R onder
// 3  28FF3519B31604CA (4)   R boven
// 4  28FF37C161160402 (5)   water invoer

// delay times
#define CHECK_FREQUENCY 200         // time in milliseconds between loop (where we check the sensor) - 200ms   
#define MIN_SEND_FREQ 10            // Minimum time between send (in seconds). We don't want to spam the gateway (30 seconds)
#define MAX_SEND_FREQ 600           // Maximum time between send (in seconds). We need to show we are alive (600 sec/10 min)
#define MIN_SEND_MOTOR 5            // minimum send freq for motor send
#define MOTOROFFSETSECS 120         // number of seconds when we will start incrementing motor_offset due to instability

// configs
#define MOTOR_FACTOR 8.355          // based on ACS712-5A characteristic (see above)
#define MOTOR_OFFSET_START 452      // Initial start value of motor_offset variable (we calculate the zero value dynamic)
#define MOTOR_OFFSET_LOW 440        // if we come below this value we no longer trus the motor_offset calculation (loose contact) or switch off
#define MOTOR_ON_WATT 600           // Number of WATT of the motor when we consider the motor to be running
#define MAXERRCNT 50                // max error count before we do reboot

// Motion message types
MyMessage tempmsg(TEMP_ID, V_TEMP);
MyMessage motor_msg(PUMP_MOTOR_ID, V_WATT);
MyMessage motorkwh_msg(PUMP_MOTOR_ID, V_KWH);
MyMessage savekwh_msg(PUMP_MOTOR_ID, V_VAR1);
MyMessage motorrun_msg(PUMP_RUNNING_ID, V_STATUS);

// Variables
OneWire oneWire(ONE_WIRE_BUS);              // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);        // Pass the oneWire reference to Dallas Temperature. 
float lastTemperature[MAX_ATTACHED_DS18B20];
unsigned int numSensors = 0;                // number of discovered temperature sensors
unsigned int lastmotor = 0;                 // save value for motor current
volatile unsigned int motor_offset = MOTOR_OFFSET_START;   // startvalue for offset for motor (increamented in ISR)
byte motor_spike_cnt = 0;                   // need at least 3 measurements below current motor_offset value before we lower the value
unsigned long mWh = 0;                      // running total in milli Watt hour - added every second
unsigned long KWHCount = 0;                 // kWh value (will increment continuously) - value in cWh = 0.01 Wh = 0.00001 kWh
bool KWHReceived = false;                   // we have received the KWH value from the GW
unsigned int errcnt = 0;                    // send message error counter

// counters updated in ISR routines
volatile unsigned int thousant = 1000;      // counter from ms to seconds
volatile unsigned int offsetcnt = MOTOROFFSETSECS; // counter in seconds when to adjust motor offset
volatile unsigned int tempsendcnt = 0;      // trigger temperature meaasurement
volatile unsigned int tempsendmax[MAX_ATTACHED_DS18B20]; // trigger sending temperature at least once per this period
volatile unsigned int pumpsendcnt = 0;      // trigger pump current meaasurement
volatile unsigned int pumpsendcntmax = 0;   // trigger pump current meaasurement sending to gateway
volatile bool doKWH = false;                // must do KWH once per second

// reboot function
void(* resetFunc) (void) = 0;               //declare reset function at address 0

void setup() {
   Serial.println("setup()");
   pinMode(PUMP_CURRENT_PIN, INPUT); 
   analogReference(DEFAULT);
   analogRead(PUMP_CURRENT_PIN);
   for (int i=0; i<MAX_ATTACHED_DS18B20; i++) tempsendmax[i] = MAX_SEND_FREQ;
   sensors.setWaitForConversion(true);
   errcnt = 0;
   mWh = 0;
}

void before() { 
   // Startup up the OneWire library
   sensors.begin();
   // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the TIMER0_COMPA_vect interrupt
   // we need to process the RELAIS2 code early to ensure it always runs, also in case mysensor can not communicate with the server
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("WTW sensor", "1.1");

  // Register all sensors to gw (they will be created as child devices)
  numSensors = sensors.getDeviceCount();
  Serial.print("# temp sensors: ");
  Serial.println(numSensors);
  DeviceAddress add;
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) { 
     present(TEMP_ID+i, S_TEMP);  w();
     lastTemperature[i]=0;
     Serial.print(i);
     Serial.print("=");
     sensors.getAddress(add, i);
     printAddress(add);
     Serial.println();
  }
  present(PUMP_MOTOR_ID, S_POWER);  w();
  present(PUMP_RUNNING_ID, S_BINARY);  w();
  
  // Fetch last known KWH count value from gw
  Serial.println("Request cWh from GW");
  request(PUMP_MOTOR_ID, V_VAR1);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// we come here every 200 ms (defined in CHECK_FREQUENCY)
void loop() {
   // handle temperature (max onces per 10 seconds)
   if (numSensors>0 && tempsendcnt==0) {
      sensors.requestTemperatures();
      tempsendcnt = MIN_SEND_FREQ;    // try again in min period
     
      for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
         // Fetch and round temperature to one decimal
         float temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(i) * 10.)) / 10.;
    
         // Only send data if temperature has changed and no error
         if ((tempsendmax[i]==0 || lastTemperature[i] != temperature) && temperature != -127.00 && temperature != 85.00) {
            // Send in the new temperature
            if (send(tempmsg.setSensor(TEMP_ID+i).set(temperature,1))) {
               // Save new temperature for next compare
               lastTemperature[i]=temperature;
               tempsendmax[i] = MAX_SEND_FREQ;
               errcnt = 0;
            } else errcnt++;
         }
      }
   }

   // now handle pump current
    unsigned int motor = analogRead(PUMP_CURRENT_PIN);
   // See if our offset should be adjusted to a lower value.
   // please note the ISR (onces per 2 minutes - defined MOTOROFFSETSECS) we increment motor_offset to take care of sensor instability
   // in this code we decrement the value again
   if (motor<motor_offset) {
//    Serial.print("Moter lower: "); Serial.print(motor); Serial.print(" offset="); Serial.print(motor_offset);
      if (motor<MOTOR_OFFSET_LOW) {
         motor_offset=MOTOR_OFFSET_START; // no longer to be trusted
//       Serial.println(" - too low ignore");
         motor_spike_cnt = 0;
      } else {
         if (motor_spike_cnt>=3) {
            motor_offset = motor; // new offset accepted after at least 3 measurements below the current
            motor_spike_cnt = 0;
//          Serial.print(" - set offset "); Serial.println(motor_offset);
         } else {
            motor_spike_cnt++; // wait if next value is also lower
//          Serial.println(" - wait");
         }
      }
      motor = 0; // consider current to be zero
   } else {
      motor_spike_cnt = 0;      // value is higher anyway, so reset the spike counter
      motor -= motor_offset;    // calculate actual peak current value
      if (motor<=6) motor=0;    // ignore jitter close to zero (ignore values below 30 Watt)
   }
   // calculate total energy usage once per second
   if (doKWH && motor>0) {
       Serial.print("Motor="); Serial.print(motor); Serial.print(" offset="); Serial.println(motor_offset);
      // we come here once per second. If we want to add the power (in Watt) once per second and get to mWh 
      // we need to multiply by 1000 and devide by 3600 second, so the factor is devide by 3.6
      mWh += long(((float) motor) * MOTOR_FACTOR / 3.6);
      doKWH = false;
   }
   // now send values to gateway
   if ((pumpsendcnt==0 && (motor!=lastmotor || mWh!=0)) || pumpsendcntmax==0) {
      if (!KWHReceived) {
         Serial.print("Re-request cWh start value from GW ...");
         request(PUMP_MOTOR_ID, V_VAR1);  w();
      } else {
         Serial.print("Send power (W)= ");
         float watt = ((float) motor) * MOTOR_FACTOR;
         Serial.println(watt);
         bool t = send(motor_msg.set(watt,0)); w();
         t = send(motorrun_msg.set(motor>0 ? "1" : "0")) && t;  w(); 

         // send KWH value to GW
         KWHCount += long(mWh / 10);    // convert from mWh to cWh
         mWh = 0;
         Serial.print("Send total Wh = ");
         Serial.println(KWHCount/100);
         t = send(motorkwh_msg.set((float) KWHCount / 100000.0, 3)) && t;  w(); 
         t = send(savekwh_msg.set(KWHCount)) && t;
         if (t) {
            pumpsendcnt = MIN_SEND_MOTOR;
            pumpsendcntmax = MAX_SEND_FREQ;
            lastmotor = motor;
            errcnt = 0;
         } else errcnt++;
      }
   }

   if (errcnt>=MAXERRCNT) resetFunc(); //call reboot
   // wait
   wait(CHECK_FREQUENCY);
}

// receive data from gateway
void receive(const MyMessage &message) {
   if (message.type==V_VAR1 && message.sensor == PUMP_MOTOR_ID) {
      unsigned long gwKWHCount=message.getULong();
      KWHCount += gwKWHCount;
      Serial.print("Received last cWh from gw:");
      Serial.println(KWHCount);
      KWHReceived = true;
   }
}


void onesec() {
  doKWH = true;
  if (tempsendcnt>0) tempsendcnt--;
  for (int i=0; i<MAX_ATTACHED_DS18B20; i++) if (tempsendmax[i]>0) tempsendmax[i]--; 
  if (pumpsendcnt>0) pumpsendcnt--;
  if (pumpsendcntmax>0) pumpsendcntmax--;
  if (offsetcnt>0) offsetcnt--; else { 
     offsetcnt = MOTOROFFSETSECS; 
     motor_offset++; // increment to handle instability current sensor
  }
}

// Interrupt on timer0 - called as part of timer0 - already running at 1ms intervals
// we use the COMPA interrupt vector to also get a 1 ms trigger here and call onesec() once every 1000 times
SIGNAL(TIMER0_COMPA_vect) {
   if (thousant>0) thousant--; else {
      thousant = 1000;
      onesec();
   }
}

// wait (6 ms) routine called after each send() call to ensure the protocol also works when more than 2 hops
// are between this device and the domoticz server
void w(void) { 
   wait(6); 
}




