# MySensorsBM Domotica Arduino semsors based on MySensors for my house
This repository contains all documentation of Arduino based sensors in my house. The sensors use the MySensors protocol 
(see https://www.mysensors.org/) which is based on a 2.4 Ghz mesh network using NRF24L01 modules.

Most MySensors examples have a single sensor as part of a scetch. In my case I often have more sensors by sketch,
such as a light sensor combined with temperature, humidiy and PIR (movement) sensors and actuators like relais.

### Folder structure
- **src** - contains all Arduino sketch code
- **Schematics** - contains electronic schematics files (created by Kicad-eeschema). PDF version are also included
- **Schematics/kicad symbols** - contains kicad symbols of many of the Arduino modules I use which are not part of the standard Kicad libraries
- **tools** - Arduino sketches for testing and clearning EEPROM
- **pictures** - pictures of various sensors to clarify how I create the electronics

### Sensors in use
**airquality**
Outside sensor for air quality. Sensor functions:
- Air quality sensor using MQ135
- Dust sensor using PMS1003 which provides PM1, PM2.5 and PM10 dust partical densities
- CAQI value is calculated based on these values (see https://en.wikipedia.org/wiki/Air_quality_index)
- Air quality is also displayed using RGB colored LEDs
 
**basementcheck**
This set of sensors is used to report temperature, light and various status signals from a wast water pump. Since I did not want to change the pump itself, I use LDR (light sensors) to see which status LEDs are on, a Microphone to pick up alarm buzzer signals and a current sensor to see if the pump actually runs. The basementcheck module sits next to the **basementhub** module and the 2 communicate to each other using the MySensors protocol. The current sensor is actually connected to the basementhub module and the value is collected from there. Sensors functions included in hardware:
- Temperature of room "Mark" (using Dallas sensor)
- Temperature shower water in basement (using Dallas sensor)
- Light sensor of room "Mark" (based on LDR)
- A LDR to report the status LED of the waste water pump
- A LDR to report the error LED of the waste water pump
- A microphone noise sensor mounted on the pump. We use a fourier funcion lib FHT to understand what happens

**basementhub**
This set of sensors measure temperature, humidity of the basement area. This sensor module works together with **basementcheck** to communicate the waste water pump motor current value to that module. Functions included in basementhub:
 - Temperature + Humidity sensor pump area
 - Temperature + Humidity sensor bathroom area
 - Light sensor bathroom
 - Water leak sensor pump area
 - Current (power) usage pump -> will send on/off switch value to BasementCheck sketch
 - 1 relais to force on basement air ventilator 
 - 2 relais to override basement thermostat (setting auto/manual and setting manual on/off)
 
**basementplayroom** and **shower**
These sets of sensors have identical hardware and are battery powered devices to measure:
- humidity (we use also the humidity part of the DHT11 for this and not the temp part as this is not accurate)
- temperature based on a Dallas sensor (more accurate than the DHT11)
- light level in the room (based on LDR)

**dining**
This set of sensors look for movement, light level and temperature. Also contains a 'smart' relais to switch 2 sets of lights on or off. Since the lights are also part of a traditional multiway switch, the sensor measures if the lights are 
actually already on or off using a opto coupler before operating the switch relais. Functions:
- PIR movement in dining room
- Temperature (with Dallas sensor)
- Status signal if the 2 lights are on or off (using opto coupler)
- Light switches for the 2 lights (using relais) to either switch the light on or off
- light level in the room (based on LDR)

**doorbell_sensor**
This set of sensors are located in the loving room and connected to a RING.COM external 'chime'. The chime module has some additional circuit added to create a digital signal when the doorbell is activated. The chime module also has a 5V power
supply we use to power the doorbell_sensor. Sensor functions: 
- ring.com doorbell signal
- Infra red (IR LED) to send on/off signal to the TV located in the living room
- PIR movement
- light level in the room (based on LDR)

**fireplace**
This set of sensors is used to get status and remote operate the natural gas fireplace. The fireplace uses a Mertik Maxitrol GV60 remote ignition and control system. Functions:
- Ignite the fireplace
- Set fireplace higher
- Set fireplace lower (or turn off)
- Report back current status

**fronthall_door** and **sidehall_door**
These sets of sensors are identical and monitor a door with mechanical lock. A megnetic switch (same as with traditional alarm systems) is used to see if the door is closed. A IR LED with photo transistor located inside the door frame is used to
see if the door is actually locked. These sensors are battery powered. Functions:
- PIR movement
- Door is closed
- Door is locked
- temperature (with Dallas sensor)

**gateway_W5100**
Todo

**ketel_relais**
Todo

**sprinkler**
Todo

**wtw_flow** and **wtw_temp**
Todo

