# Script for GPS module

Install `TinyGPSPlus` library in Arduino IDE before using `GPS.ino`.


The structure of the message output via serial3 is:
```
Timestamp, Easting, Northing, Altitude, Latitude, Longitude, Altitude
```

# Log GPS info

Install `pyserial` package in Python before using `log_gps.py`.

Change the port that you use at **line 4**:
```python
arduino_port = "COM3" # Change to your port
```

# Note

The GPS module can only for outdoors use, cannot positioning indoors.

The module needs several minutes to scan GPS signals and initialize the position.
The time is about 1 minute and up to 5 minutes.

The output frequency is 2 Hz.
It could be changed at `ubxCfgRate` in `GPS.ino`:
``` CPP
u8 ubxCfgRate[] {
  // 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, // output@1Hz
  0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77, // output@2Hz
};
```
