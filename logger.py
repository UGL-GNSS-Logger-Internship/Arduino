"""
File: logger.py
Date: 23/01/2025
Version: 1.1
Based on: log_gps.py v1.0
Description:
    This script reads sensor data (GPS, MPU, and IR) from an Arduino device connected via a serial port,
    logs the data to a file, and plots the GPS data in real-time using the GPSPlotter class.
Modules:
    serial: For serial communication with the Arduino device.
    time: For timestamping the log file.
    sys: For writing data to the log file.
    os: For creating directories to store log files.
    gps_plotter: Contains the GPSPlotter class for plotting GPS data.
Usage:
    Run the script to start reading sensor data from the Arduino, log the data to a file,
    and plot the GPS data in real-time. Press Ctrl+C to stop data capture and close the serial port.
"""
import serial
import time
import sys, os
from gps_plotter import GPSPlotter

def main():
    """
    Main function to connect to an Arduino via a serial port, read data, and log it to files.
    This function performs the following tasks:
    1. Connects to the specified Arduino serial port.
    2. Initializes a GPS plotter for a specified area.
    3. Creates log directories and files for main data, GPS data, MPU data, and IR data.
    4. Continuously reads data from the serial port and logs it to the appropriate files.
    5. Plots GPS data on a map.
    6. Handles keyboard interruption to stop data capture and close the serial port.
    Data structure:
    - GPS data: "0,<Easting>,<Northing>,<Altitude>,<Latitude>,<Longitude>,<Altitude>"
    - MPU data: "1,<x-axis>,<y-axis>,<z-axis>"
    - IR data: "2,<IR value>"
    Raises:
        KeyboardInterrupt: If the user interrupts the data capture process.
    """
    arduino_port = "COM3" # Change to the port where your Arduino is connected
    baud_rate = 9600
    time_start = time.strftime("%Y%m%d-%H%M%S")

    print(f"Connecting to {arduino_port} at {baud_rate} baud.")
    ser = serial.Serial(arduino_port, baud_rate)
    print("Connected to serial port.")

    # area parameter: [min_longitude, max_longitude, min_latitude, max_latitude]
    plotter = GPSPlotter(
        area=[151.231, 151.2325, -33.9185, -33.9175],
        add_feature=False
    )

    log_dir = f"logs/{time_start}"
    os.makedirs(log_dir, exist_ok=True)
    log_main = f'{log_dir}/main.txt'
    log_gps  = f'{log_dir}/gps.txt'
    log_mpu  = f'{log_dir}/mpu.txt'
    log_ir   = f'{log_dir}/ir.txt'

    with open(log_main, 'w') as f:
        f.write(f"Log file for main data capture.\n")
    with open(log_gps, 'w') as f:
        f.write(f"Timestamp,Easting,Northing,Altitude,Latitude,Longitude,Altitude\n")
    with open(log_mpu, 'w') as f:
        f.write(f"Timestamp,x-axis,y-axis,z-axis\n")
    with open(log_ir, 'w') as f:
        f.write(f"Timestamp,IR value\n")

    try:
        # print("Saving data to file and ploting position...")
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode("utf-8").strip()
                sys.stdout = open(log_main, 'a')
                print(data)
                if data.startswith("0") or data.split(",")[0] == "0":
                    # gps_data = data.split(",")[1:]
                    gps_data = data.split(",")[2:]
                    sys.stdout = open(log_gps, 'a')
                    print(f"{time.strftime("%Y%m%d-%H%M%S")},{','.join(gps_data)}")
                    # <Timestamp>,<Easting>,<Northing>,<Altitude>,<Latitude>,<Longitude>,<Altitude>
                    lat, lon = gps_data.split(",")[4:6]
                    plotter.plot(lat, lon)
                elif data.startswith("1") or data.split(",")[0] == "1":
                    mpu_data = data.split(",")[1:]
                    sys.stdout = open(log_mpu, 'a')
                    print(f"{time.strftime("%Y%m%d-%H%M%S")},{','.join(mpu_data)}")
                    # <Timestamp>,<x-axis>,<y-axis>,<z-axis>
                elif data.startswith("2") or data.split(",")[0] == "2":
                    ir_data = data.split(",")[1:]
                    sys.stdout = open(log_ir, 'a')
                    print(f"{time.strftime("%Y%m%d-%H%M%S")},{','.join(ir_data)}")
                    # <Timestamp>,<IR value>

        sys.stdout = sys.__stdout__
    except KeyboardInterrupt:
        sys.stdout = sys.__stdout__
        print("\nStopping data capture.")
    finally:
        sys.stdout = sys.__stdout__
        print("Serial port closed.")
        ser.close()
        plotter.save(f"{log_dir}/GPS_Plot.png")
        plotter.show()
        plotter.close()

if __name__ == "__main__":
    main()