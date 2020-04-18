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
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * Functions SHOWER
 * - Temperature (with Dallas temp) of room
 * - Humidity using Hum part of DHT11 (ignore temp)
 * - Light sensor
 * 
 *  Show the symbol table after compiling:
 *  "C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avr-nm" -Crtd --size-sort \users\bmellink\AppData\Local\Temp\arduino_build_651097\BasementHub.ino.elf
 *******************************  */

// BOARD: PRO MINI 3.3V V/ 8Mhz ATMEGA328 8Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 1

// Enable and select radio type attached
#define MY_RADIO_NRF24
// #define MY_REPEATER_FEATURE       // no repeater
// #define MY_GATEWAY_SERIAL         // enable this line to allow test without signal coverage

// node IDs
#define MY_NODE_ID 25                // start naming my own nodes at number 10 / 25
#define TEMP_ID 1                    // Room temperature
#define HUM_ID 2                     // Room humidity
#define LIGHT_ID 4                   // Room light level
#define PING_ID 15                   // ID of ping counter

#include <SPI.h>
#include <MySensors.h>  
#include <DHT.h>
#include <DallasTemperature.h>
#include <OneWire.h>

// PIN connections
#define TEMP_PIN 3                // Temp sensors pin (Dallas) - temperature
#define DHT_PIN 4                 // Hum DHT 11 (hygro + temp), only use hygro
#define BATTERY_SENSE_PIN  A0     // select the input pin for the battery sense point
#define LDR_PIN A1                // Room Light LDR (lower voltage is more light)
#define LDR_POWER_PIN 5           // vcc output for LDR measurement

// delay times
#define CHECK_FREQUENCY 10000     // time in milliseconds between loop (where we check the sensor) - 10000ms - 10 sec
#define MIN_SEND_FREQ 3           // Minimum time between send (in multiplies of CHECK_FREQUENCY). We don't want to spam the gateway (30 seconds)
#define MIN_SEND_FREQ_ERR 2       // Minimum time between sending our last data again in case of comm error (multiplied by CHECK_FREQUENCY) - 20 sec
#define MAX_SEND_FREQ 60          // Maximum time between send (in multiplies of CHECK_FREQUENCY). We need to show we are alive (600 sec/10 min)
#define MIN_PINGSEND_FREQ 60      // Time between pings (10 min)

// one wire config
#define ONE_WIRE_BUS TEMP_PIN
#define MAX_ATTACHED_DS18B20 16

// configs
#define MAXERRCNT 20                // max error count before we do reboot
#define SENSOR_TEMP_OFFSET 0        // offset sensors temp
#define SENSOR_HUM_OFFSET 0         // offset sensors humidity

// Message types - Please note each message requires 51 byte RAM storage
MyMessage Temp_msg(TEMP_ID, V_TEMP); 
MyMessage Hum_msg(HUM_ID, V_HUM);
MyMessage Light_msg(LIGHT_ID, V_LIGHT_LEVEL);
MyMessage ping_msg(PING_ID, V_DISTANCE);

// DHT themperature and humidety sensors
#define DHTTYPE           DHT11     // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHT_PIN, DHTTYPE);

// sensor storage
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

// counters and vars
unsigned int checkcnt = CHECK_FREQUENCY;      // counter from ms to CHECK_FREQUENCY interval
unsigned int tempsendcnt = MIN_SEND_FREQ;  
unsigned int tempsendcntmax = MAX_SEND_FREQ;
unsigned int lightcnt = MIN_SEND_FREQ;
unsigned int lightcntmax = MAX_SEND_FREQ;
unsigned int humsendcnt = MIN_SEND_FREQ;
unsigned int humsendcntmax = MAX_SEND_FREQ;
unsigned int pingsendcnt = MIN_PINGSEND_FREQ;
unsigned int oldBatteryPcnt = 1;          // check if changed
unsigned int numSensors = 0;
unsigned int errcnt = 0;                  // send message error counter (reboot when too many errors)
bool initdone = false;                    // have sent all sensor values at least once
unsigned long start_loop = 0;             // for calculation timing

// sensor value intermediate storage
unsigned int lastLight = 0;           // save value for room light
float lastTemp = 0;                       // save value for temperature room
float lastHum = 0;                        // save value for humidity
unsigned long pingcnt = 0;                // ping counter (increments after each cycle, reset at reboot)

void(* resetFunc) (void) = 0;             //declare reboot function at address 0

void before() { 
   // before comm library is started
   errcnt = 0;
   pingcnt = 0;

   // analog inputs  
   pinMode(LDR_PIN, INPUT); 
   pinMode(BATTERY_SENSE_PIN, INPUT);
   pinMode(LDR_POWER_PIN, OUTPUT);
   digitalWrite(LDR_POWER_PIN, 0);
   analogReference(DEFAULT);
   analogRead(LDR_PIN); // settle analogreference value
 
   // Startup up the OneWire library
   sensors.begin();
   // start DHT
   dht.begin();
}

void setup() {
   // please note that setup happens before presentation(), so we cannot call send()
   sensors.setWaitForConversion(true);
   wait(500); // wait to settle DHT sensors
}

void presentation()  {
   // Send the sketch version information to the gateway and Controller
   sendSketchInfo("Shower", "1.1"); w();

   // Register all sensors to gw (they will be created as child devices)
  
   present(LIGHT_ID, S_LIGHT_LEVEL); w();
   present(PING_ID, S_DISTANCE); w();
   present(HUM_ID, S_HUM); w();
   
   numSensors = sensors.getDeviceCount();
   Serial.print("# temp sensors: ");
   Serial.println(numSensors);
   if (numSensors>0) { present(TEMP_ID, S_TEMP); w(); }
}

void loop() {
   unsigned int light=0;
   // we come here every 100 ms (defined in CHECK_FREQUENCY)
   start_loop = millis();    // to allow adjusting wait time

   if (lightcnt>0) lightcnt--;
   if (lightcntmax>0) lightcntmax--;
   // now handle light sensor in room
   if (lightcnt==0 || !initdone) {
      digitalWrite(LDR_POWER_PIN, 1);
      wait(5);
      light = (1023 - analogRead(LDR_PIN)) / 10;
      digitalWrite(LDR_POWER_PIN, 0);
      if (light!=lastLight || lightcntmax==0 || !initdone) {
         // Serial.print("Room Light: "); Serial.println(light);
         if (send(Light_msg.set(light))) {
            lastLight = light;
            lightcnt = MIN_SEND_FREQ;
            lightcntmax = MAX_SEND_FREQ;
         } else errcnt++;
         w();
      }
   }

   if (tempsendcnt>0) tempsendcnt--;
   if (tempsendcntmax>0) tempsendcntmax--;
   // now handle Dallas temperature sensor
   if (numSensors>0 && (tempsendcnt == 0 || !initdone)) {
      float temperature;
      sensors.requestTemperatures();

      // Fetch and round temperature to one decimal
      temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(0) * 10.)) / 10.;
    
      // Only send data if temperature has changed and no error or we need to do it anyway
      if ((lastTemp != temperature || tempsendcntmax==0) && temperature != -127.00 && temperature != 85.00) {
         // Send in the new temperature
         Serial.print("Temp: "); Serial.println(temperature);
         if (send(Temp_msg.set(temperature + SENSOR_TEMP_OFFSET,1))) {
            // Save new temperature for next compare if we were able to send data
            lastTemp=temperature;
            tempsendcnt = MIN_SEND_FREQ;  
         } else {
            tempsendcnt = MIN_SEND_FREQ_ERR;
            errcnt++;
         }
         w();
      }
   }
   if (humsendcnt>0) humsendcnt--;
   if (humsendcntmax>0) humsendcntmax--;
   if (humsendcnt == 0 || !initdone) {
      // Handle Humidity from DHT
      float humidity = dht.readHumidity();
      if (isnan(humidity)) {
         Serial.println("Failed reading hum");
         Serial.println(humidity);
      } else if (humidity != lastHum || humsendcntmax==0) {
          Serial.print("Hum: ");
          if (send(Hum_msg.set(humidity + SENSOR_HUM_OFFSET, 1))) {
             lastHum = humidity;
             humsendcnt = MIN_SEND_FREQ;
          } else {
             humsendcnt = MIN_SEND_FREQ_ERR;
             errcnt++;
          }
          w();
      }
   }

   if (pingsendcnt>0) pingsendcnt--;
   if (pingsendcnt==0 || !initdone) {
      pingcnt++;
      send(ping_msg.set(pingcnt));
      w();
      pingsendcnt = MIN_PINGSEND_FREQ;
      // We can not use the Vref 1.1 here as we use the ADC in 3.3 V mode also for the LDR and switching reference values is unreliable
      // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
      // Directly connected to 1.5 V of the battery. A 100% filled battery typically reads as 1.6 V 
      // we simply multiply by 2 and add 10 to the ADC reading
      unsigned int batteryPcnt = (analogRead(BATTERY_SENSE_PIN)*2 + 10) / 10;
      if (batteryPcnt>100) batteryPcnt=100;
      sendBatteryLevel(batteryPcnt);
      light = analogRead(LDR_PIN);
   }

   if (errcnt>=MAXERRCNT) resetFunc(); //call reboot
   initdone = true;
   
   // ready with all. Now wait for a while
   unsigned long end_loop = millis();
   // Serial.print(end_loop-start_loop);
   if (end_loop-start_loop<CHECK_FREQUENCY)
      sleep(CHECK_FREQUENCY - (end_loop-start_loop));
}

// wait (6 ms) routine called after each send() call to ensure the protocol also works when more than 2 hops
// are between this device and the domoticz server
void w(void) { 
   sleep(3); 
   start_loop -= 3;   // sleep does not update 1ms timer
}

   
