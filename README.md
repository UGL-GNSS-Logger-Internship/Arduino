# GPS Module Scripts

This repository contains scripts for working with a GPS module. The scripts include:

1. `GPS.ino` - Arduino sketch for reading GPS data.
2. `log_gps.py` - Python script for logging GPS data.
3. `GPS_plotter.py` - Python script for plotting GPS data.

## GPS.ino

This Arduino sketch reads data from a GPS module and outputs it via serial communication.

### Requirements

- Install the `TinyGPSPlus` library in the Arduino IDE.
- Use Arduino MEGA.

### Usage

1. Connect the GPS module to the Arduino MEGA.
2. Upload the `GPS.ino` sketch to the Arduino MEGA.
3. The GPS module will output data in the following format via serial 3:
    ```CPP
    '<Timestamp>,<Easting>,<Northing>,<Altitude>,<Latitude>,<Longitude>,<Altitude>'
    ```

## log_gps.py

This Python script logs GPS data from the Arduino to a file and plots it in real-time.

### Requirements

- Install the `pyserial` package in Python.

### Usage

1. Change the port in **line 4** to match your Arduino's port:
    ```python
    arduino_port = "COM3" # Change to your port
    ```
2. Run the script:
    ```bash
    python log_gps.py
    ```
3. The script will save the GPS data to a file and plot it in real-time.

## GPS_plotter.py

This Python script plots GPS data on a map.

### Requirements

- Install the `matplotlib` and `cartopy` packages in Python.

### Usage

The `GPSPlotter` class provides methods to plot GPS data.

1. Initial GPSPlotter:
    ```python
    plotter = GPSPlotter.plotter()
    ```
2. Input data into plotter and plot the position:
    ```python
    plotter.plot(data)
    ```
3. Save image:
    ```python
    plotter.save('filename')
    ```
4. Show and close image:
    ```python
    plotter.show()
    plotter.close()
    ```

### Note

- The GPS module can only be used outdoors and cannot position indoors.
- The module needs several minutes to scan GPS signals and initialize the position. This may take from 1 to 5 minutes.
- The output frequency is 1 Hz. It can be changed in `GPS.ino` by modifying the `ubxCfgRate` array:
    ```cpp
    u8 ubxCfgRate[] {
      0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, // output@1Hz
      // 0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77, // output@2Hz
    };
    ```

# To-do
- [ ] Modify the output structure of GPS module for collaborating with other modules.