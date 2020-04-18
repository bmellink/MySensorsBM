
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
 * Sprinker module for 2 separate Gardena sprinkler valves. Any type would do as we remove the computer
 * part that came with it and use the tulip connector as interface: exemple:
 * https://www.gardena.com/int/products/watering/water-controls/water-control-flex/967927201/
 *  
 * The module has its own internal RTC (based on DS3231), so the sprinkler can work without a connection
 * to the home automation system. This means we need to store (in EEPROM) the timer start/stop values
 * Also added is a ground huminidy sensor (Vegetronix VG400) and rain sensor
 * 
 * Each timer entry has a start time (24 hour format), the number of minutes the sprinkler needs to be
 * on and a flag. The flag works as follows:
 * - flag is set to A (Always): sprinkler always on, independent on humidity value and rain sensor
 * - flag is set to N (Normal): 
 *     * if humidity is below GROUNDMINHUM value and no rain: full time as defined in minutes
 *     * if humidity is below GROUNDMINHUM and it is raining: half the time as defined in minutes
 *     * if humidity is above GROUNDMINHUM: spinkler will not turn on
 * 
 * We setup timer2 for all internal timing and handle all switching in the interrupt service routines
 * so we are independent of MySensors
 * 
 * LED indicators:
 * - LED_ON - indicates if sprinkler valve 1 or 2 are on
 * - LED_NETWORK 
 *    - fast blinking when no connection (yet) with Mysensors gateway
 *    - slow blinking we have connection, but no timers are set
 *    - steady on: we have connection and at least one timer is set
 * 
 * How to set RTC and Timer values from Domoticz.
 * ==============================================
 * Also see https://www.do moticz.com/forum/viewtopic.php?t=21017 on a discussion how to use text updates from Domoticz in the ast:
 * As of Domotics v4.97 we can update a text sensor directly. There are 2 steps
 * Step 1: figure out the right string format to set the RTC or timer.
 *   For RTC the format is "0: DD-MM-YYYY HH:MM:SS". Example is "0: 12-04-2020 19:12:55"
 *   For a timer the format is: "T:S HH:MM-OO X" where T=timer id 1-6, S=sprinkler 1/2, HH:MM start time, OO-run time in minutes, X-A/N (Always/Not)
 *   (the X allows you to allow sprinkler to depend on the rain sensor value).
 *   Example string is "1:2 19:00-10 A"
 * Step 2: Set the formatted text value in the text based sensor TIMER_ID (=6). In my setup this sensor has idx 468 in domoticz (idx to be found in setup->devices)
 *    The easiest way is to use the build in json.htm function of Domotics. So in my case
 *        http://192.168.22.133:8080/json.htm?type=command&param=udevice&idx=468&nvalue=2&svalue=0: 12-04-2020 19:12:55
 *        It is best to replce spaces with %20, so the url is:
 *        http://192.168.22.133:8080/json.htm?type=command&param=udevice&idx=468&nvalue=2&svalue=0:%2012-04-2020%2019:12:55
 *    Example of setting timer 1 for sprinkler 1 at 6:00 for 5 minutes when it is not raining "1:1 06:00-05 N"
 *         http://192.168.22.133:8080/json.htm?type=command&param=udevice&idx=468&nvalue=2&svalue=1:1%2006:00-05%20N
 *    Example of setting timer 2 for sprinkler 2 at 7:00 for 10 minutes allways "2:2 07:00-10 A"
 *         http://192.168.22.133:8080/json.htm?type=command&param=udevice&idx=468&nvalue=2&svalue=2:2%2007:00-10%20A
 *    
 * PLEASE NOTE: This sketch needs to be compiled using the setting in MyConfig.h
 * #define MY_RF24_PA_LEVEL (RF24_PA_MAX) // was: (RF24_PA_HIGH)
 * 
 *******************************  */

// BOARD: PRO MINI 3.3V/ 8Mhz ATMEGA328 8Mhz

// Enable debug prints to serial monitor
#define MY_DEBUG 1

// Enable and select radio type attached
#define MY_RADIO_RF24

// node ID's
#define MY_NODE_ID 29     // start naming my own nodes at number 10
#define SPRINKLER1_ID 1   // used to show status of sprinkler on/off #1
#define SPRINKLER2_ID 8   // used to show status of sprinkler on/off #2
#define GROUND_HUM_ID 2   // ground humidity percentage
#define GROUND_HUMOK_ID 3 // ground humidity is sufficient (no sprinkler needed)
#define RAIN_HUM_ID 4     // rain percentage
#define RAINING_ID 5      // is it currently raining (on/off)
#define TIMER_ID 6        // text based timer setting and provider
#define TRIGGER_ID 7      // trigger a full status dump into the TIMER_ID text sensor
#define PING_ID 15        // ID of ping counter

#include <DS3231.h>       // RTC library http://www.rinkydinkelectronics.com/library.php?id=73
#include <SPI.h>
#include <MySensors.h>    // comm library
#include <MsTimer2.h>

// PIN connections
#define AGROUND  A0       // Humidity of the ground(Vegetronix VG400)
#define ARAIN A1          // rain sensor analog input
#define DRAIN 4           // rain sensor digital input
#define PIN_ONOFF1 2      // pin to manually switch on/off 1
#define PIN_ONOFF2 3      // pin to manually switch on/off 2
#define MOTA_IA  7        // motor shield A-IA input L91110 bridge for sprinkler 1
#define MOTA_IB  8        // motor shield A-IB input
#define MOTB_IA  A2       // motor shield B-IA input L91110 bridge for sprinkler 1
#define MOTB_IB  A3       // motor shield B-IB input
#define LED_ON  5         // LED valve is on (1 or 2)
#define LED_NETWORK 6     // LED network established

// delay times
#define CHECK_FREQUENCY 1000         // time in milliseconds between loop (where we check the sensor) - 200ms   
#define MIN_SEND_FREQ 1              // Minimum time between send (in seconds). We don't want to spam the gateway (1 second)
#define MAX_SEND_FREQ 120            // Maximum time between send (in seconds). We need to show we are alive (300 sec/2 min)
#define MAX_SEND_TIMER_FREQ 21600    // Maximum time between send for timer data (21600 = 6 hours = 4 times/day)
#define DENDERDELAY 1500             // 1500 ms (1.5 sec) denderdelay on manual switch
#define MANUALTIME 1200              // in seconds (1200 = 20 min) maximum time to manually switch on sprinker

// pulse length for old valve is 160ms on (-7V) and 140 ms off (+2.4V). Timing for big Gardena valve is 250/62 ms
#define PULSE_ON  160                // number of ms for the on time pulse
#define PULSE_OFF 120                // number of ms for the off time pulse

// parameters
#define GROUNDMINHUM 40              // threshhold of humidy ground when we no longer turn on sprinkler
#define MAXERRCNT 20                 // max error count before we do reboot
#define BLINK_FAST 100               // 100 ms (200ms duty cycle, 5x /sec)
#define BLINK_SLOW 500               // 500 ms (1s duty cycle)

// Message types
MyMessage sprinkler1_msg(SPRINKLER1_ID,V_STATUS);   // setting/status sprinkler 1
MyMessage sprinkler2_msg(SPRINKLER2_ID,V_STATUS);   // setting/status sprinkler 2
MyMessage humidity_msg(GROUND_HUM_ID, V_HUM);       // humidity of the ground
MyMessage humidityok_msg(GROUND_HUMOK_ID, V_STATUS);// if humidity is above GROUNDMINHUM
MyMessage rain_msg(RAIN_HUM_ID, V_HUM);             // rain intensity (value not used at this moment)
MyMessage raining_msg(RAINING_ID, V_STATUS);        // if it is raining
MyMessage ping_msg(PING_ID, V_DISTANCE);            // increment value since reset (to see when sensor resets)
MyMessage timer_msg(TIMER_ID, V_TEXT);              // log info which will keep text log of events (also used for setting RTC and timers)
MyMessage trigger_msg(TRIGGER_ID, V_STATUS);        // force log entries with all settings

// Init the DS3231 using the hardware interface
DS3231  rtc(SDA, SCL);

unsigned int errcnt = 0;  // send message error counter (reboot when too many errors)
unsigned long pingcnt = 0;// ping counter (increments after each cycle, reset at reboot)

#define MAXTIMES 6        // number of time slots for sprinkler on/off
typedef struct {
    uint8_t   sprinkler_num; // sprinkler number
    unsigned int startmin;// starting minute (start at midnight)
    uint8_t   nummin;     // number of minutes (0=disabled)
    bool      allways;    // overrule even if if raining and moisture is ok
  } sprinkler_struct;
volatile sprinkler_struct sprinklers[MAXTIMES];

// updated in ISR
volatile bool sprinkler1_on = false;   // sprinkler 1 is actually on or off now
volatile bool sprinkler2_on = false;   // sprinkler 1 is actually on or off now
volatile unsigned int pulse1_cnt = 0;  // pulse count (dec in 1 ms timer) for sprinkler 1
volatile unsigned int pulse2_cnt = 0;  // pulse count (dec in 1 ms timer) for sprinkler 1
volatile bool raining = false;         // is now raining (no sprinkler - if moisture low than half the time)
volatile bool moisture_ok = false;     // no sprinkler 1 if moisture is ok
volatile unsigned int rain_hum = 0;
volatile unsigned int humidity_hum = 0;

volatile unsigned int sendcnt = 0;     // how often we send all our status to the gateway in seconds (MAX_SEND_FREQ)
volatile unsigned int sendcnttimer = 0;// how often we send our RTC and status (MAX_SEND_TIMER_FREQ)
volatile unsigned int wait1000 = 1000;
volatile unsigned int oneminute = 30;  // count seconds until we check all timers
volatile unsigned int dendercnt1 = 0;  // dender of input switch 1
volatile unsigned int dendercnt2 = 0;  // dender of input switch 2
volatile unsigned int manualon1 = 0;   // >0 if currently manual on sprinkler 1 switch
volatile unsigned int manualon2 = 0;   // >0 if currently manual on sprinkler 2 switch
volatile unsigned int blinkcnt = 0;    // led blinking counter
volatile unsigned int blinkfreq = BLINK_FAST; // led blink frequency
volatile bool mustsendtimers = true;

void(* resetFunc) (void) = 0;          // declare reboot function at address 0

void before() { 
   // this is always executed first
   Serial.begin(115200);
   Serial.println("before();");
   pinMode(LED_ON, OUTPUT); 
   digitalWrite(LED_ON, 0);
   pinMode(LED_NETWORK, OUTPUT); 
   digitalWrite(LED_NETWORK, 0);
   pinMode(MOTA_IA, OUTPUT);
   digitalWrite(MOTA_IA, 1); 
   pinMode(MOTA_IB, OUTPUT);
   digitalWrite(MOTA_IB, 1); 
   pinMode(MOTB_IA, OUTPUT);
   digitalWrite(MOTB_IA, 1); 
   pinMode(MOTB_IB, OUTPUT);
   digitalWrite(MOTB_IB, 1); 
   pinMode(AGROUND, INPUT);
   pinMode(ARAIN, INPUT);
   pinMode(DRAIN, INPUT);
   pinMode(PIN_ONOFF1, INPUT); 
   pinMode(PIN_ONOFF2, INPUT); 

   // Initialize the rtc object
   rtc.begin();
   analogReference(DEFAULT);
   analogRead(AGROUND); // to stabilize ADC
   getrainandmoisture(); // initial values
   errcnt = 0;
   pingcnt = 0;
   uint8_t i;

   // get sprinkler data from EEPROM and reset to 0 if it is not valid
   sprinkler_struct tmp;
   for (i=0; i<MAXTIMES; i++) {
      tmp = loadsprinkler(i);
      if (tmp.startmin >= 24*60 || tmp.nummin>=60 || (tmp.startmin==0 && tmp.nummin==0) || 
          (tmp.sprinkler_num!=1 && tmp.sprinkler_num!=2)) {
         // invalid data in EEPROM: reset, set default values for sprinkler 1 and 2
         if (i<=1) {
            tmp.startmin = 7*60+(i*10); // 7:00 for sprinkler 1 and 7:10 for sprinkler 2
            tmp.nummin = 10;
            tmp.sprinkler_num = i+1;
         } else {
            tmp.startmin = 0;
            tmp.nummin = 0;
            tmp.sprinkler_num = 0;
         }
         tmp.allways = false;
         savesprinkler(i, tmp);
      }
      sprinklers[i] = tmp; 
   }
   
   // start timer2 too ensure we are active even if comms do not work
   MsTimer2::set(1, milli); // 10ms period
   MsTimer2::start();

   // force sprinklers off after 300 ms delay
   delay(300);
   turn1_off(true);
   delay(300);
   turn2_off(true);

   Serial.println(rtc.getDateStr());
   Serial.println(rtc.getTimeStr());
 
}

void setup() {
   // this requires communication to work - called just before presentation()
   uint8_t i;
   Serial.println("setup()");   
}

void presentation()  {
   Serial.println("presentation");
   // Send the sketch version information to the gateway and Controller
   sendSketchInfo("Sprinkler", "1.0");

   // Register all sensors to gw (they will be created as child devices)
   present(SPRINKLER1_ID, S_SPRINKLER); w();
   present(SPRINKLER2_ID, S_SPRINKLER); w();
   present(GROUND_HUM_ID, S_HUM); w();
   present(GROUND_HUMOK_ID, S_BINARY); w();
   present(RAIN_HUM_ID, S_HUM); w();      // !!! maybe S_RAIN
   present(RAINING_ID, S_BINARY); w();
   present(PING_ID, S_DISTANCE); w();
   present(TIMER_ID, S_INFO); w();
   present(TRIGGER_ID, S_BINARY); w();
}

void loop() {
   // we come here every 1000 ms (defined in CHECK_FREQUENCY)
   uint8_t i;
   // check if we have a timer config, adjust LED blinking accordingly
   bool configok = false;
   for (i=0; i<MAXTIMES; i++) {
      if (sprinklers[i].nummin>0) configok = true; // at least one timer active
   }
   blinkfreq = (configok ? 0 : BLINK_SLOW);

   if (sendcnt==0) {
      // typically run this every 5 minutes, except when something changes
      // get RTC date
      Serial.print("Loop: ");
      Serial.print(rtc.getDateStr());
      Serial.print(" ");
      // get RTC time
      Serial.print(rtc.getTimeStr());
      Serial.print((sprinkler1_on ? " 1=ON": " 1=OFF"));
      Serial.println((sprinkler2_on ? " 2=ON": " 2=OFF"));
  
      // here communicate status of sprinkler and send status if changed
      // hum, hum status, sprinkler status, rainval, rain status

      mysend(sprinkler1_msg.set(sprinkler1_on));
      mysend(sprinkler2_msg.set(sprinkler2_on));
      mysend(humidity_msg.set(humidity_hum));
      mysend(humidityok_msg.set(moisture_ok));
      mysend(rain_msg.set(rain_hum));
      mysend(raining_msg.set(raining));

      pingcnt++;
      mysend(ping_msg.set(pingcnt)); 

      sendcnt = MAX_SEND_FREQ;
 //    if (!sprinkler1_on && !sprinkler2_on && errcnt>=MAXERRCNT) resetFunc(); //call reboot on errors and if we are not turned on
   }
   char buf[30];
   if (sendcnttimer==0 || mustsendtimers) {
      sprintf(buf, "RTC=%s %s", rtc.getDateStr(), rtc.getTimeStr());
      mysend(timer_msg.set((char *) buf));
      sprintf(buf, "Sprinkler1=%s 2=%s", (sprinkler1_on ? "On" : "Off"), (sprinkler2_on ? "On" : "Off"));
      mysend(timer_msg.set((char *) buf));
      sendcnttimer = MAX_SEND_TIMER_FREQ;
   }
   if (mustsendtimers) {
      uint8_t h, m;
      for (i=0; i<MAXTIMES; i++) {
         if (sprinklers[i].nummin>0) {
            // format: "1: 19:00-10 A"
            h = sprinklers[i].startmin / 60;
            m = sprinklers[i].startmin % 60;
            sprintf(buf, "T%d-S%d %02d:%02d-%d %c", i+1, sprinklers[i].sprinkler_num, h, m, sprinklers[i].nummin, (sprinklers[i].allways ? 'A' : 'N'));
            mysend(timer_msg.set((char *) buf));
         }
      }
      mustsendtimers = false;
   }
   // wait again
   wait(CHECK_FREQUENCY);
}

void receive(const MyMessage &message) {
   if (message.type==V_STATUS && message.sensor==SPRINKLER1_ID) {
      bool st=message.getBool();
      if (st) {
         Serial.println(" Sprinkler 1 on"); 
         turn1_on(true);
      } else {
         Serial.println(" Sprinkler 1 off"); 
         turn1_off(true);
      }
      // statsave = st;
      sendcnt = MIN_SEND_FREQ; // look soon to status
   }

   if (message.type==V_STATUS && message.sensor==SPRINKLER2_ID) {
      bool st=message.getBool();
      if (st) {
         Serial.println(" Sprinkler 2 on"); 
         turn2_on(true);
      } else {
         Serial.println(" Sprinkler 2 off"); 
         turn2_off(true);
      }
      // statsave = st;
      sendcnt = MIN_SEND_FREQ; // look soon to status
   }

   // need here code for: set sprinkler timers (5x), set RTC clock
   if (message.type==V_STATUS && message.sensor==TRIGGER_ID) {
      Serial.println("Get status");
      // request(TIMER_ID, V_TEXT);  // this was needed before Domoticz 4.97
      mysend(trigger_msg.set(0));   // set off again
      mustsendtimers = true;
   }
   if (message.sensor==TIMER_ID && message.type==V_TEXT) {
      // ....
      char buf[35];
      uint8_t t;
      unsigned int d,m,yr,hr,mi,se,sp;
      char c;
      message.getString(buf);
      Serial.print("Setting:");
      Serial.println(buf);
      // format timer: "1:2 19:00-10 A"  -> set timer 1 for sprinkler 2 at 19:00 always on
      // format time: "0: 22-08-2018 19:12:55"
      if (strlen(buf)>10 && buf[1]==':') {
         t = (uint8_t) buf[0];
         if (t == byte('0') && buf[2]==' ') {
            // set the Real time clock
            Serial.println("Set RTC");
            sscanf(&buf[3], "%02d-%02d-%04d %02d:%02d:%02d", &d, &m, &yr, &hr, &mi, &se);
            Serial.println(d);
            Serial.println(m);
            Serial.println(yr);
            Serial.println(hr);
            Serial.println(mi);
            Serial.println(se);
            if (d>0 && d<=31 && m>0 && m<=12 && yr>2017 && hr<24 && mi<60 && se<60) {
               rtc.setTime(hr, mi, se); // Set the time (24hr format)
               rtc.setDate(d, m, yr);   // Set the date to 
               sendcnttimer = MIN_SEND_FREQ;
            }
         } else {
            t -= byte('1'); // t had hex value 0x31, 0x32,... and we make that 0,1,2...
            if (t<MAXTIMES && buf[3]==' ') {
                Serial.print("Set timer "); Serial.print(t+1);
                sscanf(&buf[2], "%01d %02d:%02d-%d %c", &sp, &hr, &mi, &se, &c);
                Serial.print(" sprinkler "); Serial.print(sp);
                Serial.print(" at "); Serial.print(hr);
                Serial.print(":"); Serial.print(mi);
                Serial.print(" len="); Serial.print(se);
                Serial.print(" cont="); Serial.println(c);
                // no longer than 1 hour
                if (hr<24 && mi<60 && se<=60) {
                   sprinkler_struct tmp;
                   tmp.sprinkler_num = sp;
                   tmp.startmin = hr*60 + mi;
                   tmp.nummin = se;
                   tmp.allways = (c=='A');
                   sprinklers[t] = tmp;
                   savesprinkler(t, tmp);
                   sendcnttimer = MIN_SEND_FREQ;
                   mustsendtimers = true;
                }
            }
         }
      }
      
      mysend(trigger_msg.set(false)); // reset trigger
      sendcnt = MIN_SEND_FREQ; // look soon to status
   }
}

void sprinklercheck() {
   // called once/30 seconds from interrupt routine to check if we need to turn on/off the sprinkler
   
   Time t;
   t = rtc.getTime();
   unsigned int rtc_min = t.hour*60 + t.min;
   bool must_be_on1 = false;
   bool must_be_on2 = false;
   uint8_t i;

   // 
    
   for (i=0; i<MAXTIMES; i++) {
     if (sprinklers[i].nummin>0 && (!moisture_ok || sprinklers[i].allways)) {
        // we may need to do something for this timer, only if hour and min are in range
        if (rtc_min >= sprinklers[i].startmin && rtc_min < sprinklers[i].startmin + (raining && !sprinklers[i].allways ? sprinklers[i].nummin/2 : sprinklers[i].nummin)) {
          if (sprinklers[i].sprinkler_num==1) must_be_on1 = true;
          if (sprinklers[i].sprinkler_num==2) must_be_on2 = true;          
        }
     }
   }

   // only handle the sprinklers if not on manual
   if (manualon1==0) {
     if (must_be_on1) turn1_on(false); else turn1_off(false);  
   }
   if (manualon2==0) {
     if (must_be_on2) turn2_on(false); else turn2_off(false);  
   }
}


void switchcheck(void) {
   // called from timer IRQ once every 1 ms
   if (dendercnt1>0) dendercnt1--;
   if (digitalRead(PIN_ONOFF1) && dendercnt1==0) {
      // switch 1 pressed
      if (sprinkler1_on) turn1_off(true); else turn1_on(true);
      dendercnt1 = DENDERDELAY;  // no reaction on switch during dender
   }
   if (dendercnt2>0) dendercnt2--;
   if (digitalRead(PIN_ONOFF2) && dendercnt2==0) {
      // switch pressed
      if (sprinkler2_on) turn2_off(true); else turn2_on(true);
      dendercnt2 = DENDERDELAY;  // no reaction on switch during dender
   }
}

void getrainandmoisture(void) {
   // get rain sensor data and moisture data, decide if sprinkler needs to be on
   // called once per minute from timer irq - so no console output here
   raining = !digitalRead(DRAIN); 
   rain_hum = (1023-analogRead(ARAIN))/10;
   humidity_hum = analogRead(AGROUND)/10;
   moisture_ok = (humidity_hum>GROUNDMINHUM);
}

// once per second
void onesec() {
   if (sendcnt>0) sendcnt--;
   if (sendcnttimer>0) sendcnttimer--;
   if (manualon1>0) {
       manualon1--;
       if (manualon1==0) turn1_off(true);   // turn off manual sprinkler 1
   }
   if (manualon2>0) {
       manualon2--;
       if (manualon2==0) turn2_off(true);   // turn off manual sprinkler 2
   }
   if (oneminute>0) oneminute--;
   if (oneminute==0) {
      // come here once/30 second
      oneminute = 30;
      interrupts();      // allow interrupts to run inside the following code (NOT SURE)
      getrainandmoisture();
      sprinklercheck(); 
   }
}

// timer2 interrupt (1ms)
void milli(void) {
   if (pulse1_cnt>0) {  // sprinkler 1 off check
      pulse1_cnt--;
      if (pulse1_cnt==0) {
         digitalWrite(MOTA_IA, 1); // pulse off
         digitalWrite(MOTA_IB, 1); // pulse off
      }
   }
   if (pulse2_cnt>0) {  // sprinkler 2 off check
      pulse2_cnt--;
      if (pulse2_cnt==0) {
         digitalWrite(MOTB_IA, 1); // pulse off
         digitalWrite(MOTB_IB, 1); // pulse off
      }
   }
   // look if we need to manually turn on/off
   switchcheck();

   // LED blinking
   if (blinkcnt>0) blinkcnt--;
   if (blinkcnt==0) {
      blinkcnt = blinkfreq;
      if (blinkfreq==0)
         digitalWrite(LED_NETWORK, 1); // just steady on
      else
         digitalWrite(LED_NETWORK, !digitalRead(LED_NETWORK));
   }
   if (wait1000>0) wait1000--;
   if (wait1000==0) {
      // come here once/second
      wait1000 = 1000;
      onesec();
   }
}

void turn1_on(bool manualmode) {
   if (!sprinkler1_on) {
      sprinkler1_on = true;      // now provide on pulse
      pulse1_cnt = PULSE_ON;
      digitalWrite(MOTA_IA, 0); 
      digitalWrite(LED_ON, 1);
      sendcnttimer = sendcnt = MIN_SEND_FREQ; // look soon to status
      manualon1 = (manualmode ? MANUALTIME : 0);
   }
}

void turn1_off(bool forceoff) {
   if (sprinkler1_on || forceoff) {
      sprinkler1_on = false;      // now provide off pulse 
      pulse1_cnt = PULSE_OFF;
      digitalWrite(MOTA_IB, 0); 
      digitalWrite(LED_ON, (sprinkler2_on ? 1 : 0));  // led only off if sprinkler 2 also off
      sendcnttimer = sendcnt = MIN_SEND_FREQ; // look soon to status
      manualon1 = 0;
   }
}

void turn2_on(bool manualmode) {
   if (!sprinkler2_on) {
      sprinkler2_on = true;      // now provide on pulse
      pulse2_cnt = PULSE_ON;
      digitalWrite(MOTB_IA, 0); 
      digitalWrite(LED_ON, 1);
      sendcnttimer = sendcnt = MIN_SEND_FREQ; // look soon to status
      manualon2 = (manualmode ? MANUALTIME : 0);
   }
}

void turn2_off(bool forceoff) {
   if (sprinkler2_on || forceoff) {
      sprinkler2_on = false;      // now provide off pulse 
      pulse2_cnt = PULSE_OFF;
      digitalWrite(MOTB_IB, 0); 
      digitalWrite(LED_ON, (sprinkler1_on ? 1 : 0));    // led only off if sprinkler 1 also off
      sendcnttimer = sendcnt = MIN_SEND_FREQ; // look soon to status
      manualon2 = 0;
   }
}

// aux send function with internal repeat
void mysend(MyMessage &msg) {
   bool ok=false;
   uint8_t cnt=5;
   do {
      ok = send(msg);
      w(); w();
      if (!ok) wait(50);  // wait longer on error
      cnt--;
   } while (!ok && cnt>0);
   if (!ok)  errcnt++;
}

// wait (6 ms) routine called after each send() call to ensure the protocol also works when more than 2 hops
// are between this device and the domoticz server
void w(void) { 
   wait(6); 
}

void savesprinkler(uint8_t pos, sprinkler_struct data) {
    // function for saving the sprinkler values to the internal EEPROM
    // data = the value to be stored (as struct)
    // pos = the position to store the value in (0=first available position)
    uint8_t i;
    uint8_t *p = (uint8_t *) &data;
    for (i=0; i<sizeof(sprinkler_struct); i++) {
       saveState(i + pos*sizeof(sprinkler_struct), *p++);
    }
}

sprinkler_struct loadsprinkler(uint8_t pos) {
    // function for reading sprinkler values from the internal EEPROM
    // pos = the position to store the value in (0=first available position)
    // returns sprinkler_struct
    sprinkler_struct data;
    uint8_t i;
    uint8_t *p = (uint8_t *) &data;
    for (i=0; i<sizeof(sprinkler_struct); i++) {
       *p++ = loadState(i + pos*sizeof(sprinkler_struct));
    }
    return data;
}
