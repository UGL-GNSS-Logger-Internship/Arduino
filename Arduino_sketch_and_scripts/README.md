# Arduino Sensor Data Logger of UGL GNSS Logger Internship

## Overview

This project involves an Arduino-based system that reads data from various sensors (MPU6050, IR sensor, and GPS module), logs the data, and communicates with a Raspberry Pi. The data could plotted in real-time using a Python script.

## Components

- **Arduino MEGA**: The main microcontroller board used in the project.
- **MPU6050**: A 3-dimension motion tracking device for measuring acceleration and gyroscopic data.
- **IR Sensor**: An infrared sensor used for detecting obstacles or measuring distances.
- **KS0319 keyestudio GPS module**: A GPS module for obtaining location data.
- **Raspberry Pi 4B**: Used for receiving data from the Arduino and further processing.

## Files

- **main.ino**: The main Arduino sketch that integrates all the sensors and handles communication with the Raspberry Pi.
- **MPU.h / MPU.cpp**: Contains functions for setting up and processing data from the MPU6050 sensor.
- **IR.h / IR.cpp**: Contains functions for setting up and processing data from the IR sensor.
- **GPS.h / GPS.cpp**: Contains functions for setting up and processing data from the GPS module.
- **log_gps.py**: A Python script for reading GPS data from the Arduino, logging it to a file, and plotting it in real-time.
- **gps_plotter.py**: Contains the `GPSPlotter` class for plotting GPS data.

## Setup

1. **Arduino Setup**:
    - Connect the MPU6050, IR sensor, GPS module, and Raspberry Pi to the Arduino.
    - Install `TinyGPSPlus` and `MPU6050_light` libraries in Arduino IDE.
    - Upload the `main.ino` sketch to the Arduino.

2. **Python Environment**:
    - Ensure you have Python installed.
    - Install the required Python libraries:
      ```sh
      pip install pyserial matplotlib cartopy
      ```

## Usage

1. **Arduino**:
    - Connect the Arduino to your computer.
    - Open the Serial Monitor to view the sensor data.

2. **Python Script**:
    - Run the `log_gps.py` script to start logging GPS data and plotting it in real-time:
      ```sh
      python log_gps.py
      ```
    - Note that the Serial Monitor and Python script cannot run at the same time.

## Communication

- The Arduino communicates with the Raspberry Pi via serial communication.
- The `listenToPi` function in `main.ino` handles incoming messages from the Raspberry Pi.
  - This function is commented out by default.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgements

- The project uses the `TinyGPSPlus` library for GPS data parsing.
- The `MPU6050_light` library is used for interfacing with the MPU6050 sensor.