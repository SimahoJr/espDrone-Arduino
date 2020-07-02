# espDrone

The espDrone works with Drone Controller https://github.com/Simaho99/espDroneController.git

Use uptodate Arduino IDE, or Platformio in VS Code 

[env:esp12e]
platform = espressif8266
board = esp12e
framework = arduino

lib_deps =
    7364 ;Modbus IP
    11 ;I2Cdev.h
    PID
    EspSoftwareSerial
    
 ***STILL IN DEVELOPMENT***
