
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
 * Note: using timer0 to generate a second interrupt for our 1 ms counters. timer0 is still also  used for delay()
 *
 * Functions
 * - relais for boiler override (on port 8 + red LED)
 * - relais for living room valve (on port 7 + red LED) (take NEST input and add delay)
 * - input from NEST for living room valve (on port 4)
 * - temperature sensor on port 3
 * - is repeater in network
 * 
 ********************************************************************************************  */


// BOARD: PRO MINI 5V V/ 16Mhz ATMEGA328 8Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_REPEATER_FEATURE       // repeater

// node ID's
#define MY_NODE_ID 12             // start naming my own nodes 
#define RELAIS1_ID 1              // relais to force boiler on
#define RELAIS2_ID 0              // relais to open the valve of the living room
#define TEMP_ID 2                 // temperature

// Libraries
#include <SPI.h>
#include <MySensors.h>  
#include <DallasTemperature.h>
#include <OneWire.h>

// PIN connections
#define RELAIS1_PIN 8     // relais to override the boiler burning
#define RELAIS2_PIN 7     // relais to turn on the living room heating valve
#define TEMP_PIN 3        // can not change
#define NEST_PIN 4        // input from NEST relais to forward to RELAIS2 with delay

// defines and timing values
#define RELAY_ON 1        // GPIO value to write to turn on attached relay
#define RELAY_OFF 0       // GPIO value to write to turn off attached relay
#define RELAIS2_DELAY 3600  // number of seconds to keep relais2 engaged after trigger on NEST_PIN (3600 seconds = 1 hour)
#define RELAIS2_DELAYM 7200 // number of seconds to keep relais2 engaged after manual trigger (7200 = 2 hours)
#define ALIVE_TIME 120    // Number of seconds we wait until we send our data anyway
#define TEMP_TIME 20      // Minimum time between temp updates (when changing)
#define METRIC 1          // metric or fahrenheit temperature

// one wire config for temperature sensor
#define ONE_WIRE_BUS TEMP_PIN
#define MAX_ATTACHED_DS18B20 16

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

// Message types
MyMessage tempmsg(TEMP_ID, V_TEMP);
MyMessage relais1msg(RELAIS1_ID, V_STATUS);
MyMessage relais2msg(RELAIS2_ID, V_STATUS);

// counters updated in ISR routines
volatile unsigned int thousant = 1000;      // counter from ms to seconds
volatile unsigned int alive1 = ALIVE_TIME;  // count down counter for temp sensor - send at least once reaches 0
volatile unsigned int countdown1 = 0;       // count down counter for temp sensor - minimum wait time
volatile unsigned int relais2_cnt = 0;      // timeout value delays between NEST input and relais2
volatile bool relais2_status1 = false;      // relais2 status change detected in ISR

// various vars
bool relais2_status2 = false;               // relais2 status change also sent to gateway
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
bool initdone = false;      // we have sent sensor values to gateway at least once since reboot

void before() { 
   pinMode(RELAIS1_PIN, OUTPUT); 
   pinMode(RELAIS2_PIN, OUTPUT); 
   digitalWrite(RELAIS1_PIN, RELAY_OFF);
   digitalWrite(RELAIS2_PIN, RELAY_OFF);
   pinMode(NEST_PIN, INPUT_PULLUP); 
   // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the TIMER0_COMPA_vect interrupt
   // we need to process the NEST and RELAIS2 code early to ensure it always runs, also in case mysensor can not communicate with the server
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
   // Startup up the OneWire library
   sensors.begin();
}

void presentation()  {
   // Send the sketch version information to the gateway and Controller
   sendSketchInfo("Boiler relais", "1.1");

   // Register all sensors to gw (they will be created as child devices)
   present(RELAIS1_ID, S_BINARY);
   present(RELAIS2_ID, S_BINARY);

   // Fetch the number of attached temperature sensors  
   numSensors = sensors.getDeviceCount();
   Serial.print("# temp sensors: ");
   Serial.println(numSensors);
  
   for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
      present(TEMP_ID+i, S_TEMP);
   }
}

void setup() {
   Serial.println("setup()");
   // requestTemperatures() will not block current thread
   sensors.setWaitForConversion(true);
}

void loop() {
   // Fetch temperatures from Dallas sensors
   if (alive1==0 || countdown1==0 || !initdone) {
      sensors.requestTemperatures();
  
      // Read temperatures and send them to controller 
      for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
         // Fetch and round temperature to one decimal
         float temperature = static_cast<float>(static_cast<int>((METRIC ? sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
   
         // Only send data if temperature has changed and no error
         if ((lastTemperature[i] != temperature || alive1==0 || !initdone) && (temperature != -127.00 && temperature != 85.00)) {
            // Send in the new temperature
            send(tempmsg.setSensor(TEMP_ID+i).set(temperature,1));
            // Save new temperatures for next compare
            lastTemperature[i]=temperature;
            countdown1 = TEMP_TIME;
            alive1 = ALIVE_TIME;
         }
      }
   }
   // look if NEST input forcing relais2 has changed value (detection happens in ISR)
   if (relais2_status1 != relais2_status2 || !initdone) {
      // tell the gateway the relais has switched
      relais2_status2 = relais2_status1;
      Serial.print("Status change relais 2 (NEST): ");
      Serial.println(relais2_status1);
      send(relais2msg.set((relais2_status1 ? "1" : "0")));
   }

   // send relais status the first time
   if (!initdone) send(relais1msg.set("0"));

   initdone = true;
}


// receive command from Domoticz gateway (only boiler relais1)
void receive(const MyMessage &message) {
   // We only expect one type of message from controller for relais 1 or 2. But we better check anyway.
   if (message.type==V_STATUS && message.sensor==RELAIS1_ID) {
      Serial.print("Incoming heating command (relais 1): ");
      Serial.println(message.getBool());
      digitalWrite(RELAIS1_PIN, (message.getBool() ? RELAY_ON : RELAY_OFF));
   } 
   if (message.type==V_STATUS && message.sensor==RELAIS2_ID) {
      Serial.print("Incoming valve command (relais 2): ");
      relais2_status1 = relais2_status2 = message.getBool();
      relais2_cnt = RELAIS2_DELAYM;
      Serial.println(relais2_status2);
      digitalWrite(RELAIS2_PIN, (relais2_status2 ? RELAY_ON : RELAY_OFF));
   } 
}

void onesec(void) {
   if (alive1>0) alive1--;
   if (countdown1>0) countdown1--;
   if (relais2_cnt>0) relais2_cnt--;
   if (digitalRead(NEST_PIN)) {
      // pin high, NEST not engaged, see if we have timed out
      if (relais2_cnt==0 and relais2_status1) {
         digitalWrite(RELAIS2_PIN, RELAY_OFF);
         relais2_status1 = false;
      }
   } else {
      // pin low, NEST engaged, also engage RELAIS2
      digitalWrite(RELAIS2_PIN, RELAY_ON);
      relais2_cnt = RELAIS2_DELAY;
      relais2_status1 = true;
   }
}

// Interrupt on timer0 - called as part of timer0 - already running at 1ms intervals
// we use the COMPA interrupt vector to also get a 1 ms trigger here and call onesec() 
SIGNAL(TIMER0_COMPA_vect) {
   if (thousant>0)
      thousant--;
   else {
      thousant = 1000;
      onesec();
   }
}

