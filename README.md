# IoT Weather Station

## Overview
This project implements a low‑cost Internet of Things (IoT) weather station capable of measuring and publishing environmental data in real time.  
The system was designed and built as part of a university IoT course project and focuses on collecting wind speed, wind direction, temperature, and humidity data and transmitting it to an online server.

The device is designed to be:
- Low cost
- Battery powered with solar charging
- Internet connected
- Deployable in remote locations

## Features
- Wind speed measurement using an anemometer
- Wind direction sensing
- Temperature and humidity sensing
- Battery voltage monitoring
- Ambient light monitoring
- WiFi data transmission to a web server
- Low‑power operation using sleep cycles

## System Architecture
The system consists of:

**Hardware**
- ATMega328P microcontroller
- ESP8266 WiFi module
- Wind speed sensor (anemometer)
- Wind direction sensor
- Temperature and humidity sensor
- Solar panel
- LiPo battery
- Solar charging and power management module

**Software**
- Arduino firmware running on the MCU
- ESP8266 controlled via AT commands
- Python Flask web server
- SQLite database for storing measurements

The device periodically:
1. Powers sensors and collects measurements.
2. Connects to WiFi.
3. Sends data via HTTP POST to a web server.
4. Enters a low‑power sleep mode.

## Hardware Components
Approximate system components:

- ATMega328P MCU
- ESP8266‑01 WiFi module
- DHT temperature/humidity sensor
- Wind speed and direction sensors
- Solar panel (≈1.5W)
- LiPo battery (~2000mAh)
- DC‑DC converter
- Solar battery charging module
- Weatherproof enclosure

## Firmware
The microcontroller firmware is written using the Arduino framework.

Main tasks performed by the firmware:
- Sensor data acquisition
- Wind speed interrupt handling
- Wind direction sampling
- Power management
- WiFi communications using AT commands
- HTTP POST data transmission

The MCU also uses:
- Watchdog timer protection
- Deep sleep cycles for energy efficiency

## Server Side
Sensor data is transmitted to a web server running a Python Flask application.

The server:
- Receives sensor data via HTTP POST
- Stores measurements in an SQLite database
- Allows retrieval and visualization of weather data

## Power Management
The system is designed for long‑term outdoor deployment.

Power is supplied by:
- 3.7V LiPo battery
- 5V solar panel

Energy usage is minimized by:
- Turning sensors on only during measurement
- Powering WiFi only during data transmission
- Putting the MCU into deep sleep between measurement cycles

## Example Data Collected
Typical measurements include:

- Wind speed
- Wind direction
- Temperature
- Humidity
- Battery level
- Ambient light level
- WiFi signal strength

## Cost
The prototype system was built for approximately **$140**, with the wind sensors representing the largest cost component.

## Possible Improvements
Potential future improvements include:

- Custom PCB for reduced power consumption
- Cellular connectivity for wider deployment
- Improved enclosure and mounting system
- Real‑time alerts and dashboards
- Predictive weather modelling using historical data

## Authors
- Nathan Scott  
- Tracy Hong  
- Guobei Zhang

## License
This project was developed for academic purposes and is provided for educational and demonstration use.
