# About
This repository contains codes to realize an expensive espDrone

# espDrone

The espDrone works with Drone Apps for both Android and IOs

Use uptodate Arduino IDE, or Platformio in VS Code 

Devices:
1. MPU6050
2. ESP8266 (Any Firmware)

The rest is optional, it depends on your design
1. MOSFET (IRFZ/ IRFL etc)
2. SmallBrushed Motor
3. Propellers
4. Frame
5. etc

[env:esp12e]
platform = espressif8266
board = esp12e
framework = arduino

Libdaries 
1. 7364 ;Modbus IP
2. 11 ;I2Cdev.h
3. PID
4. EspSoftwareSerial
5. MPU6050 library

# To set up environment
1. Download the libraries from github, put/extract them into arduino library folder
2. Clone this repository and open it with arduino IDE
3. Go to Drone Controller app
3. Contribute, open issues, requests, etc
    
 ***STILL IN DEVELOPMENT***
