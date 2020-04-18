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
 * Functions BASEMENT CHECK
 * - Temperature (with Dallas temp) kamer Mark
 * - Temperature shower water
 * - Light sensor kamer Mark (LDR)
 * - LED status lights pump (2x)
 * - Microphone noise sensor pump --> use fourier funcion lib FHT to understand what happens
 * 
 * Also communicates directly with BasementHub (node 22): When MessageID=20 comes from BasementHub we know
 * the motor is running (based on current)
 * 
 * Note: requires patch of core serial library. See http://www.hobbytronics.co.uk/arduino-serial-buffer-size
 * - Make copy of lib file at C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino to C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino16
 * - Modify HardwareSerial.h and add in the beginning: #define SERIAL_TX_BUFFER_SIZE 16 and #define SERIAL_RX_BUFFER_SIZE 16
 * - Modify C:\Program Files (x86)\Arduino\hardware\arduino\avr\boards.txt and copy pro section to pro2 section (new board name) with pro2.build.core=arduino16
 * ---> Jan 2017. No longer needed
 * 
 *  Show the symbol table after compiling:
 *  "C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avr-nm" -Crtd --size-sort \users\bmellink\AppData\Local\Temp\arduino_build_651097\BasementHub.ino.elf
 *******************************  */

// BOARD: PRO MINI 5V V/ 16Mhz ATMEGA328 8Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 1

// Enable and select radio type attached
#define MY_RADIO_NRF24
// #define MY_REPEATER_FEATURE       // no repeater
// #define MY_GATEWAY_SERIAL         // enable this line to allow test without signal coverage

// node ID's
#define MY_NODE_ID 23             // HEX 17. start naming my own nodes at number 10

#define ROOM_TEMP_ID 1            // Room temperature
#define WATER_TEMP_ID 2           // Shower water temperature
#define ROOM_LIGHT_ID 4           // Room light level
#define PUMP_ERROR_ID 7           // Pump status (0=normal, 1=error)
#define PUMP_STATUSTXT_ID 8       // Pump status message text
#define PUMP_BITS_ID 9            // Individual status bits (0=PowerLED on, 1=PowerLED blink, 2=ErrLED on, 3=ErrLED blink, 4=Alarm beep, 5=Pumprun current, 6=Pumprun too long, 7=sound motor)
#define PING_ID 15                // ID of ping counter
#define CHECK_PUMP_ID 20          // ID of CHECK_PUMP_ID in BasementHub sketch where we will receive the motor on/off signal

#include <SPI.h>
#include <MySensors.h>  
#include <DallasTemperature.h>
#include <OneWire.h>

// Fourier analysis functions
#define LOG_OUT 1 // use the log output function
#define FHT_N 128 // set to 128 data point sample fht, resulting in 64 data buckets
#include <FHT.h> // include the library

// PIN connections
#define TEMP_PIN 3                // Temp sensors pin (Dallas) - Shower water + Room
#define PUMP_MIC_PIN A0           // Pump Microphone signal
#define ROOM_LDR_PIN A1           // Room Light LDR (lower voltage is more light)
#define PUMP_ERR_LDR_PIN A2       // Pump Error LED LDR (lower voltage, Error LED is on)
#define PUMP_PWR_LRD_PIN A3       // Pump Power LED LDR (lower voltage, Power LED is on)

// delay times
#define CHECK_FREQUENCY 100       // time in milliseconds between loop (where we check the sensor) - 100ms - 0.1 sec
#define MIN_SEND_FREQ 150         // Minimum time between send (in multiplies of CHECK_FREQUENCY). We don't want to spam the gateway (15 seconds)
#define MIN_SEND_FREQ_ERR 100     // Minimum time between sending our last data again in case of comm error (multiplied by CHECK_FREQUENCY) - 10 sec
#define MIN_SEND_FREQ_PUMP 10     // Pump updated minimal once per second (max is MAX_SEND_FREQ)
#define MAX_SEND_FREQ 3000        // Maximum time between send (in multiplies of CHECK_FREQUENCY). We need to show we are alive (300 sec/5 min)
#define MIN_PINGSEND_FREQ 600     // Time between pings (60 secs)
#define SOUND_CNT_ON 2            // number of repeat samples we need from sound to consider we hear a sound
#define SOUND_CNT_M 30            // When we hear the motor a sound, we assume it to stay on at least this time (3 seconds) counter is soundcntmotor2
#define CNT_M_LONG 300            // When the motor runs too long (>30 seconds)
#define SOUND_CNT_B 300           // When we hear the beep sound, there may be pauses in between of 20 sec, so asume it to be on for 30 seconds (counter is soundcntbeep2)
#define AUDIO_FREQ 5              // every 0.5 sec we sample audio
#define LED_CNT_BLINK 40          // max time between two led off/on transitions to consider the led blinking (4 sec)

// one wire config
#define ONE_WIRE_BUS TEMP_PIN
#define MAX_ATTACHED_DS18B20 16

// configs
#define MAXERRCNT 20                // max error count before we do reboot
#define WATER_SENSOR_TEMP_OFFSET 0  // offset sensors shower temp
#define ROOM_SENSOR_TEMP_OFFSET 1.1  // offset sensors shower temp
#define LED_ON_THRESHOLD 850        // ADS threshold value for LED to be considered on or off (higher = 0)

// Message types - Please note each message requires 51 byte RAM storage
// pumpStatus = 0=normal (quiet or motor run), 1=error 
MyMessage roomTemp_msg(ROOM_TEMP_ID, V_TEMP); 
MyMessage waterTemp_msg(WATER_TEMP_ID, V_TEMP); 
MyMessage roomLight_msg(ROOM_LIGHT_ID, V_LIGHT_LEVEL);
MyMessage pumpError_msg(PUMP_ERROR_ID, V_STATUS);
MyMessage pumpText_msg(PUMP_STATUSTXT_ID, V_TEXT);
MyMessage pumpBits_msg(PUMP_BITS_ID, V_TEXT);
MyMessage ping_msg(PING_ID, V_DISTANCE);

// sensor storage
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

// counters updated in ISR routines
volatile unsigned int checkcnt = CHECK_FREQUENCY;      // counter from ms to CHECK_FREQUENCY interval
volatile unsigned int tempsendcnt = MIN_SEND_FREQ;  
volatile unsigned int tempsendcntmax = MAX_SEND_FREQ;
volatile unsigned int lightcnt = MIN_SEND_FREQ;
volatile unsigned int lightcntmax = MAX_SEND_FREQ;
volatile unsigned int pingsendcnt = MIN_PINGSEND_FREQ;
volatile unsigned int audiocnt = AUDIO_FREQ;
volatile unsigned int soundcntmotor1 = 0;              // when we think we hear the motor we need to hear that at least SOUND_CNT_ON times before we think it is true
volatile unsigned int soundcntmotor2 = SOUND_CNT_M;    // when we hear motor set soundmotor, keep on as long as this counter runs
volatile unsigned int soundcntbeep1 = 0;               // when we think we hear a beep we need to hear that at least SOUND_CNT_ON times before we think it is true
volatile unsigned int soundcntbeep2 = SOUND_CNT_B;     // when we hear beep set soundbeep, keep on as long as this counter runs
volatile unsigned int pumpcnt = MIN_SEND_FREQ_PUMP;
volatile unsigned int pumpcntmax = MAX_SEND_FREQ;
volatile unsigned int pwrledcnt = 0;                   // number of CHECK_FREQUENCY ticks since we saw the pwr led state transition
volatile unsigned int errledcnt = 0;                   // number of CHECK_FREQUENCY ticks since we saw the err led state transition
volatile unsigned int cnt_moterOn = 0;                 // increments when the motor runs - test for too long (CNT_M_LONG)


// other counters and status vars
unsigned int numSensors = 0;
unsigned int errcnt = 0;                  // send message error counter (reboot when too many errors)
bool initdone = false;                    // have sent all sensor values at least once
int RoomTempIx = -1;                      // index of Water temp Dallas sensor on 1-wire
int WaterTempIx = -1;                     // index of Water temp Dallas sensor on 1-wire
bool soundbeep = false;                   // true when we hear the error beeping sound (cleared by timer)
bool soundmotor = false;                  // true when we hear the motor sound (cleared by timer)
volatile bool motorrun = false;           // true when motor uses current (value comes from BasementHub scetch)
bool errled_nowon = false;                // error LED is now on (real time value)
bool errled_on = false;                   // error LED is on or blinking (status)
bool errled_blink = false;                // error LED is blinking (status)
bool pwrled_nowon = true;                 // power LED is on or blinking (status)
bool pwrled_on = true;                    // power LED is now on (real time value)
bool pwrled_blink = false;                // power LED is blinking (status)

// sensor value intermediate storage
unsigned int lastRoomlight = 0;           // save value for room light
unsigned int lastShowerlight = 0;         // save value for shower light
float lastRoomTemp = 0;                   // save value for temperature room
float lastWaterTemp = 0;                  // save value for temperature water
unsigned long pingcnt = 0;                // ping counter (increments after each cycle, reset at reboot)
unsigned int lastPumpBits = 0;            // save value pump status bits

void(* resetFunc) (void) = 0;             //declare reboot function at address 0

void before() { 
   // before comm library is started
   errcnt = 0;
   pingcnt = 0;

   // analog inputs  
   pinMode(ROOM_LDR_PIN, INPUT); 
   pinMode(PUMP_ERR_LDR_PIN, INPUT); 
   pinMode(PUMP_PWR_LRD_PIN, INPUT); 
   pinMode(PUMP_MIC_PIN, INPUT); 
   analogReference(DEFAULT);
   analogRead(ROOM_LDR_PIN); // at least one measurement to stabilize
 
   // Startup up the OneWire library
   sensors.begin();

   // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the TIMER0_COMPA_vect interrupt
   // we need to process the RELAIS2 code early to ensure it always runs, also in case mysensor can not communicate with the server
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
}

void setup() {
   // please note that setup happens before presentation(), so we cannot call send()
   sensors.setWaitForConversion(true);
   wait(500); // wait to settle DHT sensors
}

void presentation()  {
   // Send the sketch version information to the gateway and Controller
   sendSketchInfo("Basement Check", "1.0"); w();

   // Register all sensors to gw (they will be created as child devices)
  
   present(ROOM_LIGHT_ID, S_LIGHT_LEVEL); w();
   present(PUMP_BITS_ID, S_INFO); w();
   present(PUMP_ERROR_ID, S_BINARY); w();
   present(PUMP_STATUSTXT_ID, S_INFO); w();
   present(PING_ID, S_DISTANCE); w();

   numSensors = sensors.getDeviceCount();
   Serial.print("# temp sensors: ");
   Serial.println(numSensors);
   if (numSensors>0) {
      present(ROOM_TEMP_ID, S_TEMP); w();
      present(WATER_TEMP_ID, S_TEMP); w();
      RoomTempIx = 0;
      WaterTempIx = 1;
   }
}

void loop() {
   // we come here every 100 ms (defined in CHECK_FREQUENCY)
   unsigned long start_loop = millis();    // to allow adjusting wait time
  
   // now handle light sensor in room
   unsigned int light = (1023 - analogRead(ROOM_LDR_PIN)) / 10;
   if ((lightcnt==0 && light!=lastRoomlight) || lightcntmax==0 || !initdone) {
      // Serial.print("Room Light: "); Serial.println(light);
      if (send(roomLight_msg.set(light))) {
         lastRoomlight = light;
         lightcnt = MIN_SEND_FREQ;
         lightcntmax = MAX_SEND_FREQ;
      } else errcnt++;
      w();
   }

   // now handle Dallas temperature sensors (if they are connected)
   if (tempsendcnt == 0 || !initdone) {
      float temperature;
      sensors.requestTemperatures();

      if (RoomTempIx>=0) {
      // Fetch and round temperature to one decimal
         temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(RoomTempIx) * 10.)) / 10.;
    
         // Only send data if temperature has changed and no error or we need to do it anyway
         if ((lastRoomTemp != temperature || tempsendcntmax==0) && temperature != -127.00 && temperature != 85.00) {
            // Send in the new temperature
            if (send(roomTemp_msg.set(temperature+ROOM_SENSOR_TEMP_OFFSET,1))) {
               // Save new temperature for next compare if we were able to send data
               lastRoomTemp=temperature;
               tempsendcnt = MIN_SEND_FREQ;  
            } else {
               tempsendcnt = MIN_SEND_FREQ_ERR;
               errcnt++;
            }
            w();
         }
      }
      if (WaterTempIx>=0) {
      // Fetch and round temperature to one decimal
         temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(WaterTempIx) * 10.)) / 10.;
    
         // Only send data if temperature has changed and no error or we need to do it anyway
         if ((lastWaterTemp != temperature || tempsendcntmax==0) && temperature != -127.00 && temperature != 85.00) {
            // Send in the new temperature
            if (send(waterTemp_msg.set(temperature+WATER_SENSOR_TEMP_OFFSET,1))) {
               // Save new temperature for next compare if we were able to send data
               lastWaterTemp=temperature;
               tempsendcnt = MIN_SEND_FREQ;  
            } else {
               tempsendcnt = MIN_SEND_FREQ_ERR;
               errcnt++;
            }
            w();
         }
      }
      tempsendcntmax = MAX_SEND_FREQ; 
   }

   // handle listening to audio input
   if (audiocnt==0) {
      audioread();            // sound analysis and sets vars: soundmotor and soundbeep
      audiocnt = AUDIO_FREQ;
   }

   handleLEDs();  // reflect status in errled_blink, errled_on, pwrled_blink, pwrled_on

   // Create status bits (lastPumpBits). Bit numbers: (0=PowerLED on, 1=PowerLED blink, 2=ErrLED on, 3=ErrLED blink, 4=Alarm beep, 5=Pumprun current, 6=Pumprun too long, 7=sound motor)
   unsigned int bits = (pwrled_on ? 1 : 0) |
                       (pwrled_blink ? 2 : 0) |
                       (errled_on ? 4 : 0) |
                       (errled_blink ? 8 : 0) |
                       (motorrun ? 16: 0) |
                       (soundbeep ? 32 : 0) |
                       (cnt_moterOn >= CNT_M_LONG ? 64 : 0) |
                       (soundmotor ? 128 : 0);
;

   // now handle sending of pump status to gateway
   // error codes:
   //   errled_blink + pwrled + soundbeep = faulty detection system
   //   errled + pwrdled + soundbeep = blocked pipe
   //   errled + pwrled_blink = power failure (with or without sound)
   //   errled + pwrled = past power failure or blocked pump
   //   !errled + !pwrled = power failure and batt empty
   //   cnt_moterOn >= CNT_M_LONG = pump on too long
   // normal operation codes:
   //   !errled + pwrled + !pwrled_blink + !motorrun = normal rest
   //   !errled + pwrled + !pwrled_blink + motorrun = normal pumping
   if (!initdone || pumpcntmax==0 || (pumpcnt==0 && bits!=lastPumpBits)) {
      bool e;
      Serial.print("Status (ML Mo Be Eb Er Pb Pw)=");
      Serial.println(bits, BIN);
      e = send(pumpError_msg.set((!errled_on && pwrled_on && !pwrled_blink && cnt_moterOn<CNT_M_LONG ? 0 : 1))); w();
      String bitsStr = String(bits, BIN);
      bitsStr = "0000000" + bitsStr;
      bitsStr = bitsStr.substring(bitsStr.length()-8);
      char buf[10];
      bitsStr.toCharArray(buf, 9);
      if (!send(pumpBits_msg.set((const char*) buf))) e=false; w();
      // String hstb = String(bits, BIN);
      if (!send(pumpText_msg.set( (const char*)
          (errled_blink && pwrled_on && soundbeep ?  "Faulty Detection" : 
           (errled_on && pwrled_blink ?  "Power Failure" : 
            (errled_on && pwrled_on && !soundbeep ?  "Error or Power Restored" : 
             (errled_on && pwrled_on && soundbeep ?  "Blocked Pipe" : 
              (!errled_on && !pwrled_on ?  "Battery Empty/ Dead" : 
               (cnt_moterOn >= CNT_M_LONG ?  "Pump Runs Too Long" : 
                (!errled_on && pwrled_on && !pwrled_blink ?  (motorrun ? "Normal Pumping" : "Normal Rest") : 
                  "Unknown Pump Status"
                )
               )
              )
             )
            )
           )
          )
          ))) e=false; w();
      if (e) {
         pumpcnt = MIN_SEND_FREQ_PUMP;
         pumpcntmax = MAX_SEND_FREQ;
         lastPumpBits = bits;
      } else errcnt++;
   }

   if (pingsendcnt==0 || !initdone) {
      pingcnt++;
      send(ping_msg.set(pingcnt));
      w();
      pingsendcnt = MIN_PINGSEND_FREQ;
   }

   if (errcnt>=MAXERRCNT) resetFunc(); //call reboot
   initdone = true;
   
   // ready with all. Now wait for a while
   unsigned long end_loop = millis();
   // Serial.print(end_loop-start_loop);
   if (end_loop-start_loop<CHECK_FREQUENCY)
      wait(CHECK_FREQUENCY - (end_loop-start_loop));
}

// will sample the status of the LEDs based on LDR light resistors
// also figure out if LED is steady on/off or blinking
// reflect status in errled_blink, errled_on, pwrled_blink, pwrled_on
void handleLEDs(void) {
   // handle ERROR led
   unsigned int light = 1023 - analogRead(PUMP_ERR_LDR_PIN);
   // Serial.print(light);
   bool stat = (light>LED_ON_THRESHOLD);
   if (stat) {
      // led is now on, set status depending on timing
      if (!errled_on) pumpcnt = pumpcntmax = MIN_SEND_FREQ_PUMP; // if error led just came on, do not send status just yet
      errled_on = true;
      errled_blink = (errledcnt>0);
   } else if (errledcnt==0) {
      // led is now off and has been for a while, so reset status
      errled_on = errled_blink = false;
   }
   // set counter if state change just happened
   if (errled_nowon != stat && !stat) errledcnt = LED_CNT_BLINK;
   errled_nowon = stat;
   
   // handle PWR led
   light = 1023 - analogRead(PUMP_PWR_LRD_PIN);
   stat = (light>LED_ON_THRESHOLD);
   if (stat) {
      // led is now on, set status depending on timing
      pwrled_on = true;
      pwrled_blink = (pwrledcnt>0);
   } else if (pwrledcnt==0) {
      // led is now off and has been for a while, so reset status
      pwrled_on = pwrled_blink = false;
   }
   // set counter if state change just happened
   if (pwrled_nowon != stat) pwrledcnt = LED_CNT_BLINK;
   pwrled_nowon = stat;
}

// will listen for motor and beep sound using microphone
// sets vars: soundmotor and soundbeep
void audioread(void) {
   // perform audio sample of 128 samples then perform fourier analysis
   noInterrupts();  
   for (int i = 0 ; i < FHT_N ; i++) { // save 128 samples
      int k = analogRead(PUMP_MIC_PIN) - 0x0200;
      k <<= 6; // form into a 16b signed int
      fht_input[i] = k; // put real data into bins
   }
   interrupts();
   fht_window();          // window the data for better frequency response
   fht_reorder();         // reorder the data before doing the fht
   fht_run();             // process the data in the fht
   fht_mag_log();         // take the output of the fht
   // fourier analysis ready, we will now have an array of 64 values where each value represents
   // the volume at a certain frequency band
   // We are looking at 2 tones: 
   // - the sound of the motor running - this is a low frequency around bands F3/F4/F5/F6
   // - the sound of the two error beeps - high frequencies around F30-F33 and F34-F37
   // - the frequency bands between F16-F27 stay more or less stable and are used as reference for background noise
   //
   // #define debugfft
   #ifdef debugfft
      for (byte i = 0 ; i < FHT_N/2 ; i++) {
         Serial.print(fht_log_out[i]); // send out the data
         Serial.print(char(9));
      }
      Serial.println();
   #endif

   // if we have not heard anything for a while we clear settings
   if (soundcntmotor2==0) soundmotor = false;
   if (soundcntbeep2==0) soundbeep = false;
   
   // first we calculate the AVG over F16-F27. These are 12 points. 
   // We do not divide by 12, because we compare with 4 points later on and want the average
   // of these 4 points to be at least 3 times louder.
   unsigned int avg=0;
   for (byte i = 16 ; i <= 27 ; i++) avg += fht_log_out[i];
   // now compare with F4/5/6/7 which is the motor sound if at least 3 times louder
   Serial.print("Base= "); Serial.print(avg);
   unsigned int snd=0;
   for (byte i = 4 ; i <= 7 ; i++) snd += fht_log_out[i];
   if (snd>avg) {
      // average of F4/5/6/7 is at least 3 times louder than average of f16-f27 -> may be motor sound
      soundcntmotor1++;
      if (soundcntmotor1>=SOUND_CNT_ON) {
         Serial.print(" Motor run");
         soundmotor = true;
         soundcntmotor2 = SOUND_CNT_M;
      }
   } else soundcntmotor1 = 0;
   Serial.print(" Motor="); Serial.print(snd); 
   snd=0;
   for (byte i = 30 ; i <= 33 ; i++) snd += fht_log_out[i];
   if (snd>avg) {
      // average of F30/31/32/33 at least 3 times louder than average of f16-f27 -> beep sound part 1
      soundcntbeep1++;
      if (soundcntbeep1>=SOUND_CNT_ON) {
         Serial.print(" Beep1");
         soundbeep = true;
         soundcntbeep2 = SOUND_CNT_B;
      }
   }
   Serial.print(" Beep1="); Serial.print(snd); 
   unsigned int snd2=0;
   for (byte i = 34 ; i <= 37 ; i++) snd2 += fht_log_out[i];
   if (snd2>avg) {
      // average of F34/35/36/37 at least 3 times louder than average of f16-f27 -> beep sound part 1
      if (soundcntbeep1>=SOUND_CNT_ON) {
         Serial.print(" Beep2");
         soundbeep = true;
         soundcntbeep2 = SOUND_CNT_B;
      }
   } 
   if (snd<=avg && snd2<=avg) soundcntbeep1 = 0;  // reset counter again
   Serial.print(" Beep2="); Serial.println(snd2); 
}

// receive command from Domoticz gateway (none) and from BasementHub (motor status)
void receive(const MyMessage &message) {
   if (message.type==V_STATUS && message.sensor==CHECK_PUMP_ID) {
      Serial.print("Motor current: ");
      motorrun = message.getBool();
      Serial.println(motorrun);
   }
}

// wait (6 ms) routine called after each send() call to ensure the protocol also works when more than 2 hops
// are between this device and the domoticz server
void w(void) { 
   wait(6); 
}

// called from ISR, decrement counters so loop() wil know when to trigger stuff
void checknow(void) {
   // decrement waiting counters (set in loop())
   if (lightcnt>0) lightcnt--;
   if (lightcntmax>0) lightcntmax--;
   if (tempsendcnt>0) tempsendcnt--;
   if (tempsendcntmax>0) tempsendcntmax--;
   if (audiocnt>0) audiocnt--;
   if (pingsendcnt>0) pingsendcnt--;
   if (soundcntmotor2>0) soundcntmotor2--;
   if (soundcntbeep2>0) soundcntbeep2--;
   if (pumpcntmax>0) pumpcntmax--;
   if (pumpcnt>0) pumpcnt--;
   
   // decrement the led counters since we saw last state transition (counters set in loop()
   if (errledcnt>0) errledcnt--;
   if (pwrledcnt>0) pwrledcnt--;

   // increment sound motor on counter
   if (motorrun) {
      if (cnt_moterOn<CNT_M_LONG) cnt_moterOn++;
   } else cnt_moterOn = 0;
}

// Interrupt on timer0 - called as part of timer0 - already running at 1ms intervals
// we use the COMPA interrupt vector to also get a 1 ms trigger here and call checknow() when needed
SIGNAL(TIMER0_COMPA_vect) {
   if (checkcnt>0)
      checkcnt--;
   else {
      checkcnt = CHECK_FREQUENCY;
      checknow();
   }
}
