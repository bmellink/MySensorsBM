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


// BOARD: PRO MINI 3.3V V/ 8Mhz ATMEGA328 16Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 1

// Enable and select radio type attached
#define MY_RADIO_RF24
// No repeater


// node ID's
#define MY_NODE_ID 30                // (hex 1E) start naming my own nodes at number 10
#define AIRQUAL_ID 2                 // Airquality (start here)
#define GAS_ID 3                     // gas saturation 
#define PM1_ID 4                     // concentration PM1
#define PM25_ID 5
#define PM10_ID 6

#include <MySensors.h>               // Mysensors library
#include <SoftwareSerial.h>          // PMS1003 communicates using UART
#include <PMserial.h>                // Library for PMS1003 sensor with serial interface https://github.com/avaldebe/PMserial
#include <FastLED.h>                 // RGB LED library for WS8111


// PIN connections
#define PMS_RX 3                     // PMS dust sensor TX
#define PMS_TX 2                     // PMS dust sensor, reserved, not used (RX) (was 2)
#define PIN_LED_DATA 6               // color led data fastled
#define PIN_GAS A0                   // analog input from MP135 sensor

// delay times
#define CHECK_FREQUENCY 1000         // time in milliseconds between loop (where we check the sensor) - 200ms   
#define MIN_SEND_FREQ 10             // Minimum time between send (in seconds). We don't want to spam the gateway (30 seconds)
#define MAX_SEND_FREQ 600            // Maximum time between send (in seconds). We need to show we are alive (600 sec/10 min)

// configs
#define NUM_LEDS 2                   // RGD LEDs
#define BRIGHTNESS  40               // RGS LED brighness
#define MAXERRCNT 20                 // max error count before we do reboot

// Mysensors message types
MyMessage airqualmsg(AIRQUAL_ID, V_LEVEL);
MyMessage airunitmsg(AIRQUAL_ID, V_UNIT_PREFIX);
MyMessage gasmsg(GAS_ID, V_LEVEL);
MyMessage gasunitmsg(GAS_ID, V_UNIT_PREFIX);
MyMessage pm1_msg(PM1_ID, V_DISTANCE);
MyMessage pm25_msg(PM25_ID, V_DISTANCE);
MyMessage pm10_msg(PM10_ID, V_DISTANCE);

// Other library vars
SoftwareSerial SWSerial(PMS_RX,PMS_TX);
CRGB leds[NUM_LEDS];    // fastled
SerialPM pms(PMS1003); // Dust sensor

// Variables
uint16_t errcnt = 0;                        // send message error counter
uint16_t caqisave = 1000;
uint16_t gassave = 101;

// counters updated in ISR routines
volatile uint16_t thousant = 1000;          // counter from ms to seconds
volatile uint16_t airsendcnt = 0;           // trigger sending to host
volatile uint16_t gassendcnt = 0;           // trigger sending to host

// reboot function
void(* resetFunc) (void) = 0;               // declare reset function at address 0

void setup() {
  // serio IO baud rate 115200 baud
  Serial.println("setup()");
  errcnt = 0;
  pms.begin(SWSerial);                      // will block if no device connected, also calls init()
  pms.init();                               // seems to be overkill as begin() also does this
  Serial.println("Dust Init done");
}

void before() { 
  // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the TIMER0_COMPA_vect interrupt
  // we need to process the RELAIS2 code early to ensure it always runs, also in case mysensor can not communicate with the server
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  pinMode(PIN_GAS, INPUT);
  delay(500);   
  FastLED.addLeds<WS2811, PIN_LED_DATA, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness( BRIGHTNESS );
  Serial.println("Fastled Init done");
  for (int i=0; i<=100; i++) {
    ledperc(0,i);
    ledperc(1,100-i);
    delay(20);
  }
  leds[0] = 0;
  leds[1] = 0;
  FastLED.show();
}


void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Air Quality sensor", "1.2");

  // Register all sensors to gw (they will be created as child devices)
  present(AIRQUAL_ID, S_AIR_QUALITY);  w();
  send(airunitmsg.set("CAQI")); w();
  present(GAS_ID, S_AIR_QUALITY);  w();
  send(gasunitmsg.set("%")); w();
  present(PM1_ID, S_DISTANCE); w();
  present(PM25_ID, S_DISTANCE); w();
  present(PM10_ID, S_DISTANCE); w();
}


// we come here every 200 ms (defined in CHECK_FREQUENCY)
void loop() {

  pms.read();     // will wait until we get value from dust sensor
  uint16_t aqi = getaqi(pms.pm[1], pms.pm[2]);
  uint16_t caqi = getcaqi(pms.pm[1], pms.pm[2]);
  char buf[6];

  uint16_t gasval = analogRead(PIN_GAS);
  // print the results
  Serial.print(F("PM1 "));  Serial.print(pms.pm[0]); Serial.print(F(", "));
  Serial.print(F("PM2.5 "));Serial.print(pms.pm[1]); Serial.print(F(", "));
  Serial.print(F("PM10 ")); Serial.print(pms.pm[2]); Serial.print(F(" [ug/m3] "));
  Serial.print(F("Gas "));  Serial.print(gasval);    Serial.print(F(", "));
  Serial.print(F("CAQI(EU) ")); Serial.print(caqi);  Serial.print(F(" (=")); Serial.print(caqiqual(caqi)); Serial.println(F(")"));

  // translate caqi perc into led color on led 1
  ledperc(1,caqi);
  gasval = gasval/10;
  ledperc(0,gasval);  // gas

  if (airsendcnt==0 && caqi!=caqisave) {
    airsendcnt = MIN_SEND_FREQ;    // try again in min period
  
    if (send(airqualmsg.setSensor(AIRQUAL_ID).set(caqi))) {
       // Save new value for next compare
       caqisave = caqi;
       errcnt = 0;
       w();
       // also send the underlying data for PM1, PM2.5 and PM10
       send(pm1_msg.set(pms.pm[0])); w();
       send(pm25_msg.set(pms.pm[1])); w();
       send(pm10_msg.set(pms.pm[2])); w();
    } else errcnt++;
  }

  if (gassendcnt==0 && gasval!=gassave) {
    gassendcnt = MIN_SEND_FREQ;    // try again in min period
  
    if (send(gasmsg.setSensor(GAS_ID).set(gasval))) {
       // Save new value for next compare
       gassave = gasval;
       errcnt = 0;
    } else errcnt++;
  }

  if (errcnt>=MAXERRCNT) resetFunc(); //call reboot
  // wait
  wait(CHECK_FREQUENCY);
}

// receive data from gateway
void receive(const MyMessage &message) {
}

void onesec() {
  if (airsendcnt>0) airsendcnt--;
  if (gassendcnt>0) gassendcnt--;
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

// show a percentage on LED (ledno 0/1, perc 0/100)
void ledperc(uint8_t ledno, uint8_t perc) {
  if (perc<=50) {
    // between greeen and orange
    leds[ledno] = CRGB(perc*5, 250,0);
  } else {
    // between orange and red
    leds[ledno] = CRGB(250, (100-perc)*5, 0);
  }
  FastLED.show();
}


// Convert pm25 and pm10 to USA based AQI scale
// https://en.wikipedia.org/wiki/Air_quality_index
#define AQILEN  8
uint16_t 
  aqi_pm10_scale[AQILEN] = {0, 55, 155, 255, 355, 425, 505, 605},
  aqi_pm25_scale[AQILEN] = {0, 13,  36,  56, 151, 251, 351, 500},
  aqi_scale[AQILEN]      = {0, 51, 101, 151, 201, 301, 401, 501};

uint16_t pm10aqi(uint16_t pm10) {
  return scale2aqi(pm10, aqi_pm10_scale, aqi_scale, AQILEN, 500);
}

uint16_t pm25aqi(uint16_t pm25) {
  return scale2aqi(pm25, aqi_pm25_scale, aqi_scale, AQILEN, 500);
}

uint16_t getaqi(uint16_t pm25, uint16_t pm10) {
  pm25 = pm25aqi(pm25);
  pm10 = pm10aqi(pm10);
  return max(pm10,pm25);
}

// returns qualitative value of AQI: 0=Good, 1=Moderate, 2=Unhealthy for sensitive groups, 3=Unhealthy, 4=Very unhealthy, 5=Hazardous
uint8_t aqiqual(uint16_t aqi) {
  uint8_t i=1;
  while (i<AQILEN && aqi>=aqi_scale[i]) i++;
  return i-1;
}

// European CAQI standard
#define CAQILEN  5
uint16_t 
  caqi_pm10_scale[CAQILEN] = {0, 25, 50, 90, 180},
  caqi_pm25_scale[CAQILEN] = {0, 15, 30, 55, 110},
  caqi_scale[CAQILEN]      = {0, 25, 50, 75, 100};

uint16_t pm10caqi(uint16_t pm10) {
  return scale2aqi(pm10, caqi_pm10_scale, caqi_scale, CAQILEN, 100);
}

uint16_t pm25caqi(uint16_t pm25) {
  return scale2aqi(pm25, caqi_pm25_scale, caqi_scale, CAQILEN, 100);
}

// returns the max of all underlying particals for caqi
uint16_t getcaqi(uint16_t pm25, uint16_t pm10) {
  pm25 = pm25caqi(pm25);
  pm10 = pm10caqi(pm10);
  return max(pm10,pm25);
}

// returns qualitative value of CAQI: 0=very low, 1=Low, 2=Medium, 3=High, 4=Very high
uint8_t caqiqual(uint16_t caqi) {
  uint8_t i=1;
  while (i<CAQILEN && caqi>=caqi_scale[i]) i++;
  return i-1;
}

// generic interpolation function

uint16_t scale2aqi(uint16_t val, uint16_t scale[], uint16_t targetscale[], uint8_t scalesize, uint16_t maxval) {
  uint8_t i;
  uint16_t aqi = maxval;
  for (i=0; i<scalesize-1; i++) {
    if (val < scale[i+1] && val>= scale[i]) {
      aqi = (targetscale[i+1] - targetscale[i] - 1) * (val - scale[i]) / (scale[i+1] - scale[i] - 1) + targetscale[i];
    }
  }
  return aqi;
}
