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
 * Version 1.0 - Henrik EKblad
 * Contribution by a-lurker and Anticimex,
 * Contribution by Norbert Truchsess <norbert.truchsess@t-online.de>
 * Contribution by Tomas Hozza <thozza@gmail.com>
 *
 *
 * DESCRIPTION
 * The EthernetGateway sends data received from sensors to the ethernet link.
 * The gateway also accepts input on ethernet interface, which is then sent out to the radio network.
 *
 * The GW code is designed for Arduino 328p / 16MHz.  ATmega168 does not have enough memory to run this program.
 *
 * LED purposes:
 * - To use the feature, uncomment WITH_LEDS_BLINKING in MyConfig.h
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error
 *
 * See http://www.mysensors.org/build/ethernet_gateway for wiring instructions.
 *
 */

// BOARD TYPE: Arduino Ethernet

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_RF24  // was MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enable gateway ethernet module type 
#define MY_GATEWAY_W5100

// W5100 Ethernet module SPI enable (optional if using a shield/module that manages SPI_EN signal)
//#define MY_W5100_SPI_EN 4  

// Enable Soft SPI for NRF radio (note different radio wiring is required)
// The W5100 ethernet module seems to have a hard time co-operate with 
// radio on the same spi bus.
#if !defined(MY_W5100_SPI_EN) && !defined(ARDUINO_ARCH_SAMD)
  #define MY_SOFTSPI
  #define MY_SOFT_SPI_SCK_PIN 14
  #define MY_SOFT_SPI_MISO_PIN 16
  #define MY_SOFT_SPI_MOSI_PIN 15
#endif  

// When W5100 is connected we have to move CE/CSN pins for NRF radio
#ifndef MY_RF24_CE_PIN 
  #define MY_RF24_CE_PIN 5
#endif
#ifndef MY_RF24_CS_PIN 
  #define MY_RF24_CS_PIN 6
#endif

// Enable to UDP           
//#define MY_USE_UDP

//#define MY_IP_ADDRESS 192,168,178,66   // If this is disabled, DHCP is used to retrieve address
// Renewal period if using DHCP
//#define MY_IP_RENEWAL_INTERVAL 60000
// The port to keep open on node server mode / or port to contact in client mode
#define MY_PORT 5003      

// Controller ip address. Enables client mode (default is "server" mode). 
// Also enable this if MY_USE_UDP is used and you want sensor data sent somewhere. 
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 178, 254   
 
// The MAC address can be anything you want but should be unique on your network.
// Newer boards have a MAC address printed on the underside of the PCB, which you can (optionally) use.
// Note that most of the Ardunio examples use  "DEAD BEEF FEED" for the MAC address.
// changed to FEEF BEEB DEED
#define MY_MAC_ADDRESS 0xDE, 0xED, 0xBE, 0xEB, 0xFE, 0xEF

// Flash leds on rx/tx/err
//#define MY_LEDS_BLINKING_FEATURE
// Set blinking period 
#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Enable inclusion mode
//#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
//#define MY_INCLUSION_MODE_DURATION 60 
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3 

// Uncomment to override default HW configurations
//#define MY_DEFAULT_ERR_LED_PIN 7  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  8  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  9  // the PCB, on board LED

#include <SPI.h>

#if defined(MY_USE_UDP)
  #include <EthernetUdp.h>
#endif
#include <Ethernet.h>


// Enable repeater functionality for this node (relay)
// #define MY_REPEATER_FEATURE

#include <MySensors.h>

#include <MsTimer2.h>

#define TRIP_TIME 20; // Number of seconds we wait until we send the no movement code after movement has stopped
#define ALIVE_TIME 60; // Number of seconds we wait until we send our data anyway
#define DIGITAL_INPUT_SENSOR 4   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define SENSOR_ID 1   // Id of the sensor child
#define RELAY_ID 2   // Id of the sensor child

#define RELAY_PIN 7
#define RELAY_ON 0  // GPIO value to write to turn on attached relay
#define RELAY_OFF 1

// Initialize motion message
MyMessage msg(SENSOR_ID, V_TRIPPED);
bool tripped = false;
unsigned int countdown = 0; 
unsigned int alive = ALIVE_TIME;

void before() { 
    // Then set relay pins in output mode
    pinMode(RELAY_PIN, OUTPUT);   
    // Set relay to last known state (using eeprom storage) 
    digitalWrite(RELAY_PIN, loadState(1)?RELAY_ON:RELAY_OFF);
}


void setup() { 
  Serial.println("setup()");
  pinMode(DIGITAL_INPUT_SENSOR, INPUT);      // sets the motion sensor digital pin as input
  MsTimer2::set(1000, onesec); // 1000ms period
  MsTimer2::start();
  send(msg.set("0")); // motion sensor is not engaged
}

void onesec(void) {
  if (countdown>0) countdown--;
  if (alive>0) alive--;
}


void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Gateway Sensor", "1.1");

  // Register all sensors to gw (they will be created as child devices)
  present(SENSOR_ID, S_MOTION);
  present(RELAY_ID, S_BINARY);
}

void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type==V_STATUS) {
     // Change relay state
     digitalWrite(RELAY_PIN, message.getBool()?RELAY_ON:RELAY_OFF);
     // Store state in eeprom
     saveState(1, message.getBool());
     // Write some debug info (please note: ID coming back is RELAY_ID == 2)
     Serial.print("Incoming change for relay:");
     Serial.print(message.sensor);
     Serial.print(", New status: ");
     Serial.println(message.getBool());
   } 
}


void loop() {     
  // Read digital motion value  
  bool tripval = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH;
  if (tripval) {
    // we tripped now, set time and send message if this is the first time
    countdown = TRIP_TIME;
    if (!tripped) {
      Serial.println(1);
      send(msg.set("1"));  // send trip msg
      tripped = true;
    }
  } else if (countdown==0) {
    // send message clear only if timer has expired
    if (tripped || alive==0) {
      Serial.println(0);
      send(msg.set("0"));
      tripped = false;
      alive = ALIVE_TIME;
    }
  }
}
