# Drone using ESP32 and MPU6050

A quadcopter project focused on maintaining balance using a PID controller.

## Requirements

- 4 ESCs (Electronic Speed Controllers)
- 4 brushless motors
- MPU6050 IMU (Inertial Measurement Unit)
- ESP32 microcontroller
- PlatformIO for development

## Libraries/Modules

The project utilizes various libraries and modules. Here's an overview of the project folder structure:

- src/
  - main.cpp
  - main.h
- lib/
  - Brushless_controlling/
    - BLC.cpp
    - BLC.h
    - ...
  - PID_controller/
    - PIDcontroller.cpp
    - PIDcontroller.h
    - ...
  - MPU6050_axis_handling/
    - axisHandler.cpp
    - axisHandler.h
  - Remote_handling/
    - BTremote.cpp
    - BTremote.h
- platformio.ini
- README.md

Feel free to modify the folder structure based on your specific project requirements.