# Domotica sensors based on Arduino hardware and MySensors for my house
This repository contains all documentation of Arduino based sensors in my house. The sensors use the MySensors protocol 
(see https://www.mysensors.org/) which is based on a 2.4 Ghz mesh network using NRF24L01 modules. The main domotica system used is Domoticz (https://www.domoticz.com/) running on a Raspberry Pi. Most Arduino based hardware is based on the Arduino PRO MINI with ATMega328 chip (both the 5V/16Mhz and 3.3V/8Mhz versions are used).

Most standard MySensors examples have a single sensor as part of a sketch. However, the MySensors library can handle multiple sensors, such as a light sensor combined with temperature, humidiy and PIR (movement) sensors and actuators like relais. This multi sensor capability is used extensively.

This repository is for my house only, but it can be used as example how to build more complex MySensors system.

### Folder structure
- **src** - contains all Arduino sketch code
- **Schematics** - contains electronic schematics files (created by Kicad-eeschema) -- NOT ALL SCHEMATICS COMPLETED --
- **Scematics/PDF versions** - PDF files of schematics
- **Schematics/kicad symbols** - contains kicad symbols of many of the Arduino modules I use which are not part of the standard Kicad libraries
- **tools** - Arduino sketches for testing and clearning EEPROM
- **pictures** - pictures of various sensors to clarify how I create the electronics -- TO BE COMPLETED --

## Sensors in use
### Airquality
Outside sensor for air quality. Sensor functions:
- Air quality sensor using MQ135
- Dust sensor using PMS1003 which provides PM1, PM2.5 and PM10 dust partical densities
- CAQI value is calculated based on these values (see https://en.wikipedia.org/wiki/Air_quality_index)
- Air quality is also displayed using RGB colored LEDs
 
### Basementhub
This set of sensors measure temperature, humidity of the basement area. This sensor module works together with **basementcheck** to communicate the waste water pump motor current value to that module. The reason there are
two separate modules (basementhub and basementcheck) is that the total code size was too large for a single Arduino
model (Pro Mini using ATMEGA328). Functions included in **basementhub**:
 - Temperature + Humidity sensor pump area
 - Temperature + Humidity sensor bathroom area
 - Light sensor bathroom
 - Water leak sensor pump area
 - Current (power) usage pump -> will send on/off switch value to BasementCheck sketch
 - 1 relais to force on basement air ventilator 
 - 2 relais to override basement thermostat (setting auto/manual and setting manual on/off)
 
### Basementcheck
This set of sensors is used to report temperature, light and various status signals from a wast water pump. Since I did not want to change the pump itself, I use LDR (light sensors) to see which status LEDs are on, a Microphone to pick up alarm buzzer signals and a current sensor to see if the pump actually runs. The basementcheck module sits next to the **basementhub** module and the 2 communicate to each other using the MySensors protocol. The current sensor is actually connected to the basementhub module and the value is collected from there. Sensors functions included in the hardware of **basementcheck**:
- Temperature of room "Mark" (using Dallas sensor)
- Temperature shower water in basement (using Dallas sensor)
- Light sensor of room "Mark" (based on LDR)
- A LDR to report the status LED of the waste water pump
- A LDR to report the error LED of the waste water pump
- A microphone noise sensor mounted on the pump. We use a fourier funcion lib FHT to understand what happens

### Basementplayroom and Shower
These sets of sensors have identical hardware and are battery powered devices to measure:
- humidity (we use also the humidity part of the DHT11 for this and not the temp part as this is not accurate)
- room temperature based on a Dallas sensor (more accurate than the DHT11)
- light level in the room (based on LDR)

### Dining
This set of sensors look for movement, light level and temperature. Also contains a 'smart' relais to switch 2 sets of lights on or off. Since the lights are also part of a traditional multiway switch, the sensor measures if the lights are 
actually already on or off using a opto coupler before operating the switch relais. Functions:
- PIR movement in dining room
- Temperature (with Dallas sensor)
- Status signal if the 2 lights are on or off (using opto coupler)
- Light switches for the 2 lights (using relais) to either switch the light on or off
- light level in the room (based on LDR)

### Doorbell_sensor
This set of sensors are located in the living room and connected to a RING.COM external 'chime'. The chime module has some additional circuit added to create a digital signal when the doorbell is activated. The chime module also has a 5V power
supply we use to power the doorbell_sensor. Sensor functions: 
- ring.com doorbell signal
- Infra red (IR LED) to send on/off signal to the TV located in the living room
- PIR movement
- light level in the room (based on LDR)

### Fireplace
This set of sensors is used to get status and remote operate the natural gas fireplace. The fireplace uses a Mertik Maxitrol GV60 remote ignition and control system. Functions:
- Ignite the fireplace
- Set fireplace higher
- Set fireplace lower (or turn off)
- Report back current status

### Fronthall_door and Sidehall_door
These sets of sensors are identical and monitor a door with mechanical lock. A megnetic switch (same as with traditional alarm systems) is used to see if the door is closed. A IR LED with photo transistor located inside the door frame is used to
see if the door is actually locked. These sensors are battery powered. Functions:
- PIR movement
- Door is closed
- Door is locked
- temperature (with Dallas sensor)

### Gateway_W5100
Ethernet gateway to route all NRF24L01 radio signals to the domoticz server. Uses standard code from MySensors and added a relay (output) and PIR sensor

### Ketel_relais
Sets of sensors close to the central heating boiler. The central heating boiler is controlled by a NEST thermostat that uses the Opentherm protocol. However the NEST thermostat is only situated in the living room. A specific relais is added in this unit to override the NEST value and force heating of a part of the house while closing a valve to the living room. Functions in this module:
- relais for boiler override
- input from NEST for living room valve
- relais for living room valve (normally taking the value from the NEST input, but can be overridden)
- temperature sensor for shower water

### Sprinkler
Sprinker module for 2 separate Gardena sprinkler valves. Any type would do as we remove the computer
part that came with it and use the tulip connector as interface. Example Gardena product used: https://www.gardena.com/int/products/watering/water-controls/water-control-flex/967927201/

The module has its own internal RTC (based on DS3231), so the sprinkler can work without a connection to the home automation system. This means we need to store (in EEPROM) the timer start/stop values. Sensor functions included:
- ground huminidy sensor based on the Vegetronix VG400
- rain sensor

### Wtw_flow and Wtw_temp
These two sensor modules work together to handle all sensors from a WTW (ductch for Warmte-Terug-Win) or Water Heat Recycling installation (see https://en.wikipedia.org/wiki/Water_heat_recycling). The (still warm) shower waste water is cooled down by flowing in a concentric pipe next to the cold water towards the boiler. The specific WTW used (https://www.technea.nl/product/opvolger-rv20dv4-douchepijp-wtw) has two double pipes to recover heat from the shower waste water. A total of 2 flow sensors and 7 temperature sensors are used to measure the waterflow to the shower (hot and cold) and water temperature (cold entry, pre-warmed cold, shower hot, waste left hot, waste right hot, waste left cold, waste right cold). The flow sensors are based on the Caleffi 316 which has a hall sensor that pulses at 8.8 Hz per liter/min.

- the wtw_flow module connects to 2 flow sensors (shower cold water and boiler cold water entry) and 2 temp sensors
- the wtw_temp module connects to 5 temperature sensors and an analog input to measure the power usage of the Hydrofor (pressure) pump.
