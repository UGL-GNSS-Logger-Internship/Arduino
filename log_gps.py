"""
File: log_gps.py
Date: 21/01/2025
Version: 1.0
Description:
    This script reads GPS data from an Arduino device connected via a serial port,
    logs the data to a file, and plots the data in real-time using the GPSPlotter class.
Modules:
    serial: For serial communication with the Arduino device.
    time: For timestamping the log file.
    plot: Contains the GPSPlotter class for plotting GPS data.
Usage:
    Run the script to start reading GPS data from the Arduino, log the data to a file,
    and plot the data in real-time. Press Ctrl+C to stop data capture and close the serial port.
"""
import serial
import time
from GPS_plotter import GPSPlotter

arduino_port = "COM3" # Change to your port
baud_rate = 9600
time_start = time.time()
output_file = f"logs/GPS_Log_{time_start}.txt"

print(f"Connecting to {arduino_port} at {baud_rate} baud.")
ser = serial.Serial(arduino_port, baud_rate)
print("Connected to serial port.")

plotter = GPSPlotter(
    area=[151.231, 151.2325, -33.9185, -33.9175],
    add_feature=False
)

try:
    print("Saving data to file...")
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode("utf-8").strip()
            # <Timestamp>,<Easting>,<Northing>,<Altitude>,<Latitude>,<Longitude>,<Altitude>
            lat, lon = data.split(",")[4:6]
            plotter.plot(lat, lon)
            print(data)
            with open(output_file, "a", encoding="utf-8") as file:
                file.write(data+"\n")
except KeyboardInterrupt:
    print("\nStopping data capture.")
finally:
    ser.close()
    print("Serial port closed.")
    plotter.save(f"logs/GPS_Plot_{time_start}.png")
    plotter.show()
    # plotter.close()
