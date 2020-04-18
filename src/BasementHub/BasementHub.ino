// try https://www.mysensors.org/build/debug

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
 * Functions BASEMENT HUB
 * - Temperature + Humidity sensor pump area
 * - Temperature + Humidity sensor bath room
 * - Light sensor bath room
 * - Water leak sensor pump area
 * - Current (power) usage pump -> will send on/off switch value to BasementCheck sketch
 * - 1 relais to force on ventilator 
 * - 2 relais to override basement thermostat (auto/manual and manual on/off)
 * 
 * The power usage of the pump uses ACS712-20A hall sensor:
 * - At rest this circuit outputs VCC/2 -> 2.5 V (ADC=511)
 * - The extra diode in the circuit lowers the value a bit (2.1V). ADC output at rest is approx 420
 * - The characteristic is 100 mV/A (1A current is 100mV higher, ADC value 20.5 higher)
 * - 1A current (20.5 ADC ticks) is 220Watt -> factor is 10.7 for DC and 8.56 for AC (difference Vpeak and RMS)
 * 
 * Communicates also directly with BasementCheck module (ID=23, MessageID=20)
 * to let the BasementCheck know the motor is running (on/off status) using 
 * check_motor_msg -> this will set/clear the motorrun variable in BasementCheck
 * so we can handle themotor status and error bits as needed
 * 
 *  Show the symbol table after compiling:
 *  "C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avr-nm" -Crtd --size-sort \users\bmellink\AppData\Local\Temp\arduino_build_651097\BasementHub.ino.elf
 *******************************  */


// BOARD: PRO MINI 5V V/ 16Mhz ATMEGA328 8Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 1

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_REPEATER_FEATURE       // repeater

// node ID's
#define MY_NODE_ID 22             // My own node
#define CHECK_NODE_ID 23          // Node ID of BasementCheck sketch (will send pump on/off current there)
// Child nodes
#define SHOWER_TEMP_ID 20         // Shower DHT 22 (hygro + temp)
#define SHOWER_HUM_ID 21          // Shower DHT 22 (hygro + temp)
#define PUMP_TEMP_ID 30           // Pump room DHT 22 (hygro + temp)
#define PUMP_HUM_ID 31            // Pump room DHT 22 (hygro + temp)
#define SHOWER_LIGHT_ID 4         // Shower light level
#define PUMP_MOTOR_ID 5           // Pump running current
#define LEAK_ID 7                 // Leaking water
#define THERMOSTAT_ID 9           // Thermostat status
#define RELAIS1_ID 10             // Used to enable/disable heat signal from thermostat to boiler (on=enabled)
#define RELAIS2_ID 11             // Relais basement valve on (high=on - will stay on for period defined in RELAIS2_DELAYM)
#define RELAIS3_ID 12             // Relais ventillator overrule (high=manual)
#define PING_ID 15                // ID of ping counter
#define CHECK_PUMP_ID 20          // ID of cild node within CHECK_NODE_ID to receive value

// Libraries
#include <SPI.h>
#include <MySensors.h>  
#include <DHT.h>

// PIN connections
#define SHOWER_DHT_PIN 7          // Showed DHT 22 (hygro + temp)
#define PUMP_DHT_PIN 8            // Pump room DHT 22 (hygro + temp)
#define SHOWER_LDR_PIN A0         // Shower Light LDR (lower voltage is more light)
#define LEAK_PIN A6               // Leak sensor floor (<4 V then water on floor)
#define PUMP_CURRENT_PIN A5       // Current measurement pump
#define RELAIS1_PIN 4             // Relais heat signal to boiler (on=heat request)
#define RELAIS2_PIN 5             // Relais basement valve on (high=on)
#define RELAIS3_PIN 6             // Relais ventillator overrule (high=manual)
#define THERMOSTAT_PIN 3          // Input thermostat 

// delay times
#define CHECK_FREQUENCY 100       // time in milliseconds between loop (where we check the sensor) - 100ms - 0.1 sec
#define MIN_SEND_FREQ 300         // Minimum time between send (in multiplies of CHECK_FREQUENCY). We don't want to spam the gateway (30 seconds)
#define MIN_SEND_FREQ_ERR 100     // Minimum time between sending our last data again in case of comm error (multiplied by CHECK_FREQUENCY) - 10 sec
#define MAX_SEND_FREQ 6000        // Maximum time between send (in multiplies of CHECK_FREQUENCY). We need to show we are alive (600 sec/10 min)
#define MIN_PINGSEND_FREQ 600     // Time between pings (60 secs)
#define MIN_SEND_MOTOR 10         // send motor current max once per second (if fluctuating)

// configs
#define MAXERRCNT 20                // max error count before we do reboot
#define SHOWER_SENSOR_TEMP_OFFSET 1 // offset sensors shower temp
#define SHOWER_SENSOR_HUM_OFFSET -1 // offset sensors shower humidity
#define PUMP_SENSOR_TEMP_OFFSET 0   // offset sensors shower temp
#define PUMP_SENSOR_HUM_OFFSET 1    // offset sensors shower temp
#define RELAY_ON 1                  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0                 // GPIO value to write to turn off attached relay
#define RELAIS2_DELAY 3600          // number of seconds to keep relais2 engaged after trigger on NEST_PIN (3600 seconds = 1 hour)
#define RELAIS2_DELAYM 7200         // number of seconds to keep relais2 engaged after manual trigger (7200 = 2 hours)
#define LEAK_THRESHOLD 600          // analog value from LEAK_PIN. If below this value, consider a water leak
#define MOTOR_FACTOR 8.56           // ACS712-20A characteristic is 100mV/A -> 20.5 ADC ticks per 1 A current = 220 Watt (see above)
#define MOTOR_OFFSET_START 450      // Initial start value of motor_offset variable (we calculate the zero value dynamic)
#define MOTOR_OFFSET_LOW 300        // if we come below this value we no longer trus the motor_offset calculation (loose contact)
#define MOTOR_ON_WATT 600           // Number of WATT of the motor when we consider the motor to be running

// Sensor message types - Please note each message requires 51 byte RAM storage
MyMessage showerTemp_msg(SHOWER_TEMP_ID, V_TEMP); 
MyMessage showerHum_msg(SHOWER_HUM_ID, V_HUM);
MyMessage pumpTemp_msg(PUMP_TEMP_ID, V_TEMP); 
MyMessage pumpHum_msg(PUMP_HUM_ID, V_HUM);
MyMessage showerLight_msg(SHOWER_LIGHT_ID, V_LIGHT_LEVEL);
MyMessage leak_msg(LEAK_ID, V_TRIPPED);
MyMessage motor_msg(PUMP_MOTOR_ID, V_WATT);
MyMessage ping_msg(PING_ID, V_DISTANCE);
MyMessage thermostat_msg(THERMOSTAT_ID, V_STATUS);
MyMessage relais1_msg(RELAIS1_ID, V_STATUS);
MyMessage relais2_msg(RELAIS2_ID, V_STATUS);
MyMessage relais3_msg(RELAIS3_ID, V_STATUS);
MyMessage check_motor_msg(CHECK_PUMP_ID, V_STATUS);   // message not sent to gateway but to BasementCheck sketch to process motor on/off

// DHT themperature and humidety sensors
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT showerdht(SHOWER_DHT_PIN, DHTTYPE);
DHT pumpdht(PUMP_DHT_PIN, DHTTYPE);

// counters updated in ISR routines
volatile unsigned int thousant = 1000;      // counter from ms to seconds
volatile byte seconds  = 60;                // counter from seconds to minutes
volatile unsigned int relais2_cnt = 0;      // timeout value delays between thermostat input and relais2
volatile bool relais2_status1 = false;      // relais2 status change detected in ISR
volatile bool relais1_enabled = true;       // normal operation
volatile unsigned int checkcnt = CHECK_FREQUENCY;      // counter from ms to CHECK_FREQUENCY interval
volatile unsigned int lightcnt = MIN_SEND_FREQ;
volatile unsigned int lightcntmax = MAX_SEND_FREQ;
volatile unsigned int dhtcnt = MIN_SEND_FREQ;
volatile unsigned int pingsendcnt = MIN_PINGSEND_FREQ;
volatile unsigned int leakcntmax = 0;                // reset to MAX_SEND_FREQ
volatile unsigned int motorcnt = MIN_SEND_MOTOR;     // counter to keep track of motor current send status frequency
volatile unsigned int motorcntmax = MAX_SEND_FREQ;
volatile unsigned int thermoscntmax = MAX_SEND_FREQ;
volatile unsigned int relais2cntmax = MAX_SEND_FREQ;

// other counters and status vars
bool relais2_status2 = false;               // relais2 status change also sent to gateway
unsigned long pingcnt = 0;                  // ping counter (increments at each temperature emasurement, reset at reboot)
unsigned int errcnt = 0;                    // send message error counter
bool initdone = false;                      // not yet init fully completed
volatile unsigned int motor_offset = MOTOR_OFFSET_START;   // startvalue for offset for motor (increamented in ISR)
byte motor_spike_cnt = 0;                   // need at least 3 measurements below current motor_offset value before we lower the value

// sensor value intermediate storage (compare with previous measurement if value has changed)
unsigned int lastRoomlight = 0;     // save value for room light
unsigned int lastShowerlight = 0;   // save value for shower light
float lastShowerTemp = 0;           // save value for temperature shower
float lastPumpTemp = 0;             // save value for temperature pump area
float lastShowerHum = 0;            // save value for humidity shower
float lastPumpHum = 0;              // save value for humidity pump area
bool lastthermostat = false;        // save value thermostat input
bool lastleak = false;              // save value for water leak
unsigned int lastmotor = 0;         // save value for motor current
bool lastmotorstatus = false;       // save value for motor on/off status (based on MOTOR_ON_WATT)

// reboot function
void(* resetFunc) (void) = 0;               //declare reset function at address 0

void before() { 
   // before comm library is started
   errcnt = 0;
   pingcnt = 0;
   
   // analog inputs  
   pinMode(SHOWER_LDR_PIN, INPUT); 
   pinMode(LEAK_PIN, INPUT); 
   pinMode(PUMP_CURRENT_PIN, INPUT); 
   analogReference(DEFAULT);
   analogRead(SHOWER_LDR_PIN); // at least one measurement to stabilize

   // digital inputs
   pinMode(THERMOSTAT_PIN, INPUT_PULLUP);
   
   // relais outputs  
   pinMode(RELAIS1_PIN, OUTPUT); 
   digitalWrite(RELAIS1_PIN, RELAY_OFF);
   pinMode(RELAIS2_PIN, OUTPUT); 
   digitalWrite(RELAIS2_PIN, RELAY_OFF);
   pinMode(RELAIS3_PIN, OUTPUT); 
   digitalWrite(RELAIS3_PIN, RELAY_OFF);

   // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the TIMER0_COMPA_vect interrupt
   // we need to process the RELAIS2 code early to ensure it always runs, also in case mysensor can not communicate with the server
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
}

void setup() {
   // please note that setup happens before presentation(), so we cannot call send()
   wait(500); // wait to settle DHT sensors
}

void presentation()  {
   // Send the sketch version information to the gateway and Controller
   sendSketchInfo("Basement Hub", "1.2");

   // Register all sensors to gw (they will be created as child devices)
   present(SHOWER_TEMP_ID, S_TEMP);  w();
   present(SHOWER_HUM_ID, S_HUM); w();
   present(PUMP_TEMP_ID, S_TEMP); w();
   present(PUMP_HUM_ID, S_HUM); w();
   present(PUMP_MOTOR_ID, S_POWER); w();
   present(LEAK_ID, S_WATER_LEAK); w();
   present(SHOWER_LIGHT_ID, S_LIGHT_LEVEL); w();
   present(THERMOSTAT_ID, S_BINARY); w();
   present(RELAIS1_ID, S_BINARY); w();
   present(RELAIS2_ID, S_BINARY); w();
   present(RELAIS3_ID, S_BINARY); w();
   present(PING_ID, S_DISTANCE); w();
}


void loop() {
   // we come here approximately every 100 ms (defined in CHECK_FREQUENCY)
   unsigned long start_loop = millis();    // to allow adjusting wait time
   bool stat;
   float temperature;
   float humidity;
   unsigned int light;
   unsigned int motor;  

   // Handle Humidity and Temperature Shower
   if (dhtcnt==0 || !initdone) {
      // Shower DHT
      temperature = showerdht.readTemperature();
      humidity = showerdht.readHumidity();
      if (isnan(temperature) || isnan(humidity)) {
          Serial.println("Failed reading showerdht temp/hum");
      } else if (temperature != lastShowerTemp || humidity != lastShowerHum || !initdone) {
          temperature += SHOWER_SENSOR_TEMP_OFFSET;
          humidity += SHOWER_SENSOR_HUM_OFFSET;
          lastShowerTemp = temperature;
          lastShowerHum = humidity;
          Serial.print("ShowerTemp: ");
          send(showerTemp_msg.set(temperature, 1));
          w();
          Serial.print("ShowerHum: ");
          send(showerHum_msg.set(humidity, 1));
          w();
       }

       // Pump area DHT
       temperature = pumpdht.readTemperature();
       humidity = pumpdht.readHumidity();
       if (isnan(temperature) || isnan(humidity)) {
          Serial.println("Failed reading pumpdht temp/hum");
       } else if (temperature != lastPumpTemp || humidity != lastPumpHum || !initdone) {
          temperature += PUMP_SENSOR_TEMP_OFFSET;
          humidity += PUMP_SENSOR_HUM_OFFSET;
          lastPumpTemp = temperature;
          lastPumpHum = humidity;
          Serial.print("PumpTemp: ");
          send(pumpTemp_msg.set(temperature, 1));
          w();
          Serial.print("PumpHum: ");
          send(pumpHum_msg.set(humidity, 1));
          w();
       }
       dhtcnt = MIN_SEND_FREQ;
   }

   // publish thermostat value
   stat = !digitalRead(THERMOSTAT_PIN);  // pin low = stat true = thermostat engaged
   if (lastthermostat != stat || !initdone || thermoscntmax==0) {
      if (send(thermostat_msg.set(stat))) { // thermostat status
         lastthermostat = stat;
         thermoscntmax = MAX_SEND_FREQ;
      } else errcnt++;
      w();
   }

   // publish relais2 status
   if (relais2_status1 != relais2_status2 || relais2cntmax==0 || !initdone) {
      // tell the gateway the relais has switched
      Serial.print("Status change relais 2 (valve): ");
      Serial.println(relais2_status1);
      if (send(relais2_msg.set((relais2_status1 ? "1" : "0")))) {
         relais2_status2 = relais2_status1;
         relais2cntmax = MAX_SEND_FREQ;
      } else errcnt++;
      w();
   }

   if (!initdone) {
      // tell the gateway our initial state
      send(relais1_msg.set(1)); // normal mode heating
      w();
      send(relais3_msg.set(0)); // centillator override not engaged 
      w();
   }

   // now handle light sensors
   light = (1023 - analogRead(SHOWER_LDR_PIN)) / 10;
   if ((lightcnt==0 && light!=lastShowerlight) || !initdone || lightcntmax==0) {
      Serial.print("Shower Light: "); Serial.println(light);
      if (send(showerLight_msg.set(light))) {
         lastShowerlight = light;
         lightcnt = MIN_SEND_FREQ;
         lightcntmax = MAX_SEND_FREQ;
      } else errcnt++;        
      w();
   }

   // handle motor current
   motor = analogRead(PUMP_CURRENT_PIN);
   // See if our offset should be adjusted to a lower value.
   // please note the ISR (onces per second) we increment motor_offset to take care of sensor instability
   // in this code we decrement the value again
   if (motor<motor_offset) {
      if (motor<MOTOR_OFFSET_LOW) {
         motor_offset=MOTOR_OFFSET_START; // no longer to be trusted
         Serial.println("Reset motor_offset");
         motor_spike_cnt = 0;
      } else {
         if (motor_spike_cnt>=3) {
            motor_offset = motor; // new offset accepted after at least 3 measurements below the current
            motor_spike_cnt = 0;
            Serial.print("Set motor offset ");
            Serial.println(motor_offset);
         } else motor_spike_cnt++; // wait if next value is also lower
      }
      motor = 0; // consider current to be zero
   } else {
      motor_spike_cnt = 0;      // value is higher anyway, so reset the spike counter
      motor -= motor_offset;    // calculate actual peak current value
      if (motor<=6) motor=0;    // ignore jitter close to zero (ignore values below 50 Watt)
   }
   if ((motorcnt==0 && motor!=lastmotor) || motorcntmax==0 || !initdone) {
      if (send(motor_msg.set(((float) motor) * MOTOR_FACTOR,0))) {
         motorcnt = MIN_SEND_MOTOR;
         motorcntmax = MAX_SEND_FREQ;
         lastmotor = motor;
      } else errcnt++;
      w();
      // also send motor on/off status to the BasementCheck sketch
      stat = ((((float) motor) * MOTOR_FACTOR) > MOTOR_ON_WATT);
      Serial.print("Send motor status "); Serial.println((stat ? "ON" : "OFF"));
      check_motor_msg.setDestination(CHECK_NODE_ID);   // send to node instead of gateway
      send(check_motor_msg.set(stat));
      w();
   }

   // handle water leak circuit
   stat = (analogRead(LEAK_PIN) < LEAK_THRESHOLD);
   if (stat!=lastleak || !initdone || leakcntmax==0) {
      if (send(leak_msg.set(stat))) {
          lastleak = stat;
          leakcntmax = MAX_SEND_FREQ;
      } else errcnt++;
      w();
   }
 
   // now send keep alive stuff
   if (pingsendcnt==0 || !initdone) {
      pingcnt++;
      send(ping_msg.set(pingcnt));
      w();
      pingsendcnt = MIN_PINGSEND_FREQ;
   }

   if (errcnt>=MAXERRCNT) resetFunc(); //call reboot

   // we told the gateway at least once all we know
   initdone = true;
   
   // ready with all. Now wait for a while
   unsigned long end_loop = millis();
   // Serial.print(end_loop-start_loop);
   if (end_loop-start_loop<=CHECK_FREQUENCY)
      wait(CHECK_FREQUENCY - (end_loop-start_loop));
}

// receive command from Domoticz gateway (relais 1/2/3)
void receive(const MyMessage &message) {
   bool val;
   // reset the motor offset calculation if we provide command
   if (message.type==V_STATUS)  motor_offset=MOTOR_OFFSET_START;
   
   if (message.type==V_STATUS && message.sensor==RELAIS1_ID) {
      Serial.print("Relais 1&2 (heat enabled): ");
      relais1_enabled = message.getBool();
      Serial.println(relais1_enabled);
   } 
   if (message.type==V_STATUS && message.sensor==RELAIS2_ID) {
      Serial.print("Relais 2 (valve): ");
      relais2_status1 = relais2_status2 = message.getBool();
      relais2_cnt = RELAIS2_DELAYM;
      Serial.println(relais2_status2);
      digitalWrite(RELAIS2_PIN, (relais2_status2 ? RELAY_ON : RELAY_OFF));
   } 
   if (message.type==V_STATUS && message.sensor==RELAIS3_ID) {
      Serial.print("Relais 3 (ventilator): ");
      val = message.getBool();
      digitalWrite(RELAIS3_PIN, val);
      Serial.println(val);
   } 
}

// ISR routine calls this once per second
// We handle thermostat and relais 1+2 code here to guarantee this continues to run even if domoticz and mysensors gateways are down
void onesec(void) {
   if (seconds==0) {
      motor_offset++; // increment to handle instability current sensor (once per minute)
      seconds = 60;
   } else seconds--;
   if (relais2_cnt>0) relais2_cnt--;
   if (relais1_enabled) {
      if (digitalRead(THERMOSTAT_PIN)) {
         // pin high, thermostat not engaged, handle heat request and timeout valve
         digitalWrite(RELAIS1_PIN, RELAY_OFF);    // turn heat request off
         if (relais2_cnt==0 and relais2_status1) {
            digitalWrite(RELAIS2_PIN, RELAY_OFF); // also turn valve off
            relais2_status1 = false;
         }
      } else {
         // pin low, thermostat engaged, also engage RELAIS1 & RELAIS2
         digitalWrite(RELAIS1_PIN, RELAY_ON); // engage heat request
         digitalWrite(RELAIS2_PIN, RELAY_ON); // engage valve
         relais2_cnt = RELAIS2_DELAY;
         relais2_status1 = true;
      }
   } else {
      // switch all off when heating mode is disabled from domoticz
      digitalWrite(RELAIS1_PIN, RELAY_OFF);  // heat request switched off
      digitalWrite(RELAIS2_PIN, RELAY_OFF);  // turn valve off
      relais2_status1 = false;
   }
}

void checknow(void) {
   if (lightcnt>0) lightcnt--;
   if (lightcntmax>0) lightcntmax--;
   if (dhtcnt>0) dhtcnt--;
   if (pingsendcnt>0) pingsendcnt--;
   if (leakcntmax>0) leakcntmax--;
   if (motorcnt>0) motorcnt--;
   if (motorcntmax>0) motorcntmax--;
   if (thermoscntmax>0) thermoscntmax--;
   if (relais2cntmax>0) relais2cntmax--;
}

// Interrupt on timer0 - called as part of timer0 - already running at 1ms intervals
// we use the COMPA interrupt vector to also get a 1 ms trigger here and call onesec() once every 1000 times
SIGNAL(TIMER0_COMPA_vect) {
   if (thousant>0) thousant--; else {
      thousant = 1000;
      onesec();
   }
   if (checkcnt>0) checkcnt--; else {
      checkcnt = CHECK_FREQUENCY;
      checknow();
   }
}

// wait (6 ms) routine called after each send() call to ensure the protocol also works when more than 2 hops
// are between this device and the domoticz server
void w(void) { 
   wait(6); 
}

