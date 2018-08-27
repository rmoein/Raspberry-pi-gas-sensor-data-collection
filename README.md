# Raspberry pi gas sensor data collection


This program is written in Python 2. It collects and stores sensor data (O2, CO2, and Humidity) attached to a Raspberry Pi. It also controls a motor pump through a relay switch. By running AnalyzeGasSample class, you can begin an experiment where a pump pushes desired gas towards sensors and sensor data is collected and appended to a text file.

### Sensors:
- COZIR CM-0123 (CO2 sensor)
- KE-25 (O2 sensor)
- DHT22 (Humidity sensor)

### Dependencies:
- Adafruit_ADS1x15 library (it's an analog to digital converter for reading the O2 sensor)
- Adafruit_DHT library (for reading the humidity sensor values)


### Examples:

By default, the program assumes that the relay switch is connected to pin 7, while the humidity sensor is connected to GPIO27. __*Remember to change those settings if your setup varies*__. 

#### To start a gas collection and composition analysis experiment, run the code below. 
AnalyzeGasSample().begin_experiment() 

#### To view raw sensor outputs:

Sensors().print_o2_co2_humidity_values()
