#!/usr/bin/env python3
"""
File: main.py
Author: UNSW UGL GNSS Logger Internship Team
Version: 1.1.0.beta
Description:
    Main script for the GNSS Logger.
    This script is responsible for reading data from the GPS modules, recording video from the camera module, and switching between GPS modules.
    It also provides a GUI for user interaction.
Dependencies:
    - Python >= 3.9
    - OpenCV
    - numpy
    - RPi.GPIO
    - picamera2
    - pyserial
Setup:
    1. Install the required dependencies using pip:
        ``` bash
        pip install opencv-python numpy picamera2 pyserial tkinter
        ```
    2. Connect the GPS modules, camera module, and button to the Raspberry Pi.
    3. Run the script with the following command:
        ``` bash
        python3 main.py
        ```
        The script will start reading data from the GPS modules and auto recording video from the camera module after the GPS data is valid.
    4. (Option) Press the button to capture and save an image to the specified directory.
    5. (Option) Touch 'Start Recording' to start recording video if it is not auto recording.
GUI Layout:
    - GPS Label: Display the current GPS module being used.
    - Switch GPS: Switch GPS modules between using internal antenna and external antenna.
    - Start Recording: Start recording video if it is not auto recording.
    - Capture an image: Capture an image and save it to the specified directory.
    - Stop logging: Stop logging data and video, and exit the program.
"""
import os
import sys
import serial
import cv2
from picamera2 import Picamera2
import datetime
import time
import numpy as np
import threading
import RPi.GPIO as GPIO
import subprocess
import tkinter as tk
import signal

BUTTON = 16
GPS1 = 6
GPS2 = 5
SAVE_ERROR = False

def converter(Latitude, Longitude):
    '''
    This function converts latitude and longitude to easting and northing.
    '''
    a = 6378137.000
    FalseOriginE = 500000.0000
    FalseOriginN = 10000000.0000
    K0 = 0.9996
    e2 = 0.006694380023
    e4 = e2 * e2
    e6 = e4 * e2

    radLat = Latitude * np.pi / 180
    radLng = Longitude * np.pi / 180
    diffLng = Longitude - 153.000000
    radDiffLng = diffLng * np.pi / 180

    SinLat1 = np.sin(radLat)
    SinLat2 = np.sin(2*radLat)
    SinLat4 = np.sin(4*radLat)
    SinLat6 = np.sin(6*radLat)

    A0 = 1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256
    A2 = 3 * (e2 + e4 / 4 + 15 * e6 / 128) / 8
    A4 = 15 * (e4 + 3 * e6 / 4) / 256
    A6 = 35 * e6 / 3072

    M1 = a * A0 * radLat
    M2 = - a * A2 * SinLat2
    M3 = a * A4 * SinLat4
    M4 = - a * A6 * SinLat6
    M = M1 + M2 + M3 + M4

    Pho = a * (1 - e2) / (1 - e2 * SinLat1 * SinLat1) ** 1.5
    Nu = a / (1 - (e2 * SinLat1 * SinLat1)) ** 0.5

    CosLat1 = np.cos(radLat)
    CosLat2 = CosLat1 * CosLat1
    CosLat3 = CosLat1 * CosLat2
    CosLat4 = CosLat2 * CosLat2
    CosLat5 = CosLat2 * CosLat3
    CosLat6 = CosLat3 * CosLat3
    CosLat7 = CosLat3 * CosLat4

    omega1 = radDiffLng
    omega2 = omega1 * omega1
    omega3 = omega1 * omega2
    omega4 = omega2 * omega2
    omega5 = omega2 * omega3
    omega6 = omega3 * omega3
    omega7 = omega3 * omega4
    omega8 = omega4 * omega4

    TanLat1 = np.tan(radLat)
    TanLat2 = TanLat1 * TanLat1
    TanLat4 = TanLat2 * TanLat2
    TanLat6 = TanLat2 * TanLat4

    Psi1 = Nu / Pho
    Psi2 = Psi1 * Psi1
    Psi3 = Psi1 * Psi2
    Psi4 = Psi2 * Psi2

    # Easting
    E1 = Nu * omega1 * CosLat1
    E2 = Nu * omega3 * CosLat3 * (Psi1 - TanLat2) / 6
    E3 = Nu * omega5 * CosLat5 * (4 * Psi3 * (1 - 6 * TanLat2) + Psi1 * (1 + 8 * TanLat2) - Psi1 * (2 * TanLat2) + TanLat4) / 120
    E4 = Nu * omega7 * CosLat7 * (61 - 479 * TanLat2 + 179 * TanLat4 - TanLat6) / 5040

    SumE = E1 + E2 + E3 + E4
    SumEK0 = K0 * SumE
    Easting = SumEK0 + FalseOriginE

    # Northing
    N1 = Nu * SinLat1 * omega2 * CosLat1 / 2
    N2 = Nu * SinLat1 * omega4 * CosLat3 * (4 * Psi2 + Psi1 - TanLat2) / 24
    N3 = Nu * SinLat1 * omega6 * CosLat5 * (61 - 58 * Psi4 + Psi2 * (4 * Psi4 + 9 * Psi2) - 4 * Psi1 * (7 * Psi4 + 11 * Psi2) - 7 * Psi1 * TanLat2) / 720
    N4 = Nu * SinLat1 * omega8 * CosLat7 * (1385 - 3111 * Psi2 + 543 * Psi4 - TanLat6) / 40320

    SumN = M + N1 + N2 + N3 + N4
    SumNK0 = K0 * SumN
    Northing = SumNK0 + FalseOriginN

    return Easting, Northing

def cameraRecord():
    '''
    This function handles recording from either a Raspberry Pi Camera Module or a USB camera.
    '''
    CAPTURE_FOLDER = f'{logger_folder}/captured_img'
    os.makedirs(CAPTURE_FOLDER, exist_ok=True)
    CAPTURED_TIME_FILE = f'{logger_folder}/captured_time.txt'
    VIDEO_FILE = f'{logger_folder}/logger.mp4'
    VIDEO_TIMESTAMP_FILE = f'{logger_folder}/video_timestamp.csv'

    with open(VIDEO_TIMESTAMP_FILE, 'w') as f:
        f.write('timestamp\n')
    with open(CAPTURED_TIME_FILE, 'w') as f:
        f.write('timestamp\n')

    # Attempt to initialize the Pi Camera
    try:
        picam2 = Picamera2()
        picam2.preview_configuration.main.size = (640, 480)
        picam2.preview_configuration.main.format = 'RGB888'
        picam2.configure('preview')
        picam2.start()
        camera_type = "pi"
    except Exception as e:
        print(f"Pi Camera initialization failed: {e}. Trying USB camera.")
        # If Pi Camera fails, try USB camera
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Could not open USB camera.")
            return -1  # Indicate an error
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera_type = "usb"

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(VIDEO_FILE, fourcc, 30.0, (640, 480))

    captured = False
    while True:
        date_time = str(datetime.datetime.now())

        if camera_type == "pi":
            frame = picam2.capture_array()
        elif camera_type == "usb":
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame from USB camera.")
                break
        else:
            break

        font = cv2.FONT_HERSHEY_SIMPLEX
        image = cv2.putText(frame, date_time, (20, 50), font, 1, (1, 255, 255), 4)

        if time_set.is_set() and camera_start.is_set():
            try:
                out.write(image)
                with open(VIDEO_TIMESTAMP_FILE, 'a') as f:
                    f.write(date_time + '\n')
                cv2.imshow('Live Video Recording', image)

                if cv2.waitKey(1) != -1:
                    break

            except KeyboardInterrupt:
                break

        if GPIO.input(BUTTON) == GPIO.HIGH or cap_image.is_set():
            if not captured:
                cv2.imwrite(f'{CAPTURE_FOLDER}/{date_time}.jpg', image)
                with open(CAPTURED_TIME_FILE, 'a') as f:
                    f.write(date_time + '\n')
                captured = True
                cap_image.clear()
        else:
            captured = False

        time.sleep(0.1)  # Add a small sleep interval to reduce CPU load
        if logging_stop.is_set():
            break

    if camera_type == "pi":
        picam2.close()
    else:
        cap.release() #release the usb camera.

    out.release() #release the video writer.
    cv2.destroyAllWindows()

    return 0

def serialRead():
    '''
    This function reads data from the GPS modules and writes the data to a file.
    '''
    def cat(process):
        try:
            process = subprocess.Popen(
                ['stdbuf', '-o0', 'cat', '/dev/ttyAMA0'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
            )
            return process
        except subprocess.CalledProcessError as e:
            print(f"Error starting cat: {e}")
            time.sleep(1)
            return None
        except Exception as e:
            print(f"Unexpected error starting cat: {e}")
            time.sleep(1)
            return None

    try:
        ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
        ser.flush()
        serial_available = True
    except serial.SerialException:
        print("Serial port /dev/ttyAMA0 not available. Using subprocess.")
        serial_available = False

    RAW_DATA_FILE = f'{logger_folder}/raw_data.txt'
    LOCATION_FILE = f'{logger_folder}/location.txt'

    with open(RAW_DATA_FILE, 'w') as f:
        f.write(f"timestamp,raw data\n")
    with open(LOCATION_FILE, 'w') as f:
        f.write(f"timestamp,latitude,longitude,altitude,easting,northing\n")

    gps_select = 0
    GPIO.output(GPIO.GPS1, GPIO.HIGH)  # Assuming GPIO.GPS1 and GPIO.GPS2 are defined
    GPIO.output(GPIO.GPS2, GPIO.LOW)

    process = None
    rmc_data = None
    gga_data = None

    while True:
        if gps_switch.is_set():
            gps_select = (gps_select + 1) % 2
            if gps_select == 0:
                GPIO.output(GPIO.GPS1, GPIO.HIGH)
                GPIO.output(GPIO.GPS2, GPIO.LOW)
            elif gps_select == 1:
                GPIO.output(GPIO.GPS1, GPIO.LOW)
                GPIO.output(GPIO.GPS2, GPIO.HIGH)
            gps_switch.clear()

        try:
            if serial_available:
                try:
                    if ser.in_waiting > 0:
                        try:
                            data = ser.readline().decode("utf-8").strip()
                        except UnicodeDecodeError:
                            data = ""
                except serial.SerialException:
                    print("Serial port error, switching to subprocess.")
                    serial_available = False
                    continue

            if not serial_available:
                if process is None or process.poll() is not None:
                    if process is not None:
                        print("Restarting cat /dev/ttyAMA0...")

                    process = cat(process)
                    if process is None:
                        continue

                try:
                    output = process.stdout.readline()
                    if output:
                        data = output.decode().strip()
                    else:
                        continue
                except IOError as e:
                    print(f"IO Error reading output: {e}")
                    time.sleep(0.1)
                    continue
                except Exception as e:
                    print(f"Unexpected error in cat: {e}")
                    process = cat(process)
                    if process is None:
                        continue

            fields = data.split(',')
            with open(RAW_DATA_FILE, 'a') as f:
                f.write(f"{str(datetime.datetime.now())},{data}\n")

            if fields and fields[0] == '$GPRMC':
                rmc_data = fields[:]
                if rmc_data[2] == 'A': # Valid data
                    if not time_set.is_set():
                        time_str = rmc_data[1]
                        date_str = rmc_data[9]
                        if time_str != '' and date_str != '':
                            time = f"{time_str[0:2]}:{time_str[2:4]}:{time_str[4:]}"
                            date = f"20{date_str[4:6]}-{date_str[2:4]}-{date_str[0:2]}"
                            result = subprocess.run(
                                ["sudo", "date", "-u", "--set", f"{date} {time}"],
                                capture_output=True,
                                text=True
                            )
                            print(result)
                            time_set.set()
                    elif time_set.is_set():
                        if not camera_start.is_set():
                            camera_start.set()
                else:
                    rmc_data = None

            elif fields and fields[0] == '$GPGGA':
                gga_data = fields[:]
                try:
                    if rmc_data and rmc_data[2] == 'A':
                        altitude = gga_data[9]
                        if altitude == '':
                            altitude = "N/A"
                        latitude = gga_data[2]
                        direction_ns = gga_data[3]
                        longitude = gga_data[4]

                        if time_set.is_set():
                            date_time = str(datetime.datetime.now())
                            latitude_deg = float(latitude[0:2]) + float(latitude[2:]) / 60
                            longitude_deg = float(longitude[0:3]) + float(longitude[3:]) / 60
                            if direction_ns == 'S':
                                latitude_deg = -latitude_deg
                            easting, northing = converter(float(latitude_deg), float(longitude_deg))
                            with open(LOCATION_FILE, 'a') as f:
                                f.write(f"{date_time},{latitude},{longitude},{altitude},{easting},{northing}\n")

                except IndexError:
                    altitude = "N/A"

            rmc_data = None
            gga_data = None

            if logging_stop.is_set():
                break

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"An error occurred: {e}")

        if logging_stop.is_set():
            break

    if serial_available:
        try:
            ser.close()
        except Exception:
            print("Error closing serial port")
    if process and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            process.kill()
        except Exception as e:
            print(f"Error during process termination: {e}")
    return 0

class GNSSLoggerUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GNSS LOGGER")

        self.gps_label = tk.Label(self.root, text='Internal GPS')
        self.gps_label.pack(padx=20, pady=10)

        self.switch_id = 0
        self.switch_button = tk.Button(
            self.root,
            text='Switch GPS',
            command=self.switch_gps
        )
        self.switch_button.pack(padx=0, pady=10)

        self.start_cap_button = tk.Button(
            self.root,
            text='Start recording',
            command=self.start_cap
        )
        self.start_cap_button.pack(padx=10, pady=10)

        self.cap_now_button = tk.Button(
            self.root,
            text='Capture an image',
            command=self.cap_now
        )
        self.cap_now_button.pack(padx=10, pady=10)

        self.stop_button = tk.Button(
            self.root,
            text='Stop logging',
            command=self.stop_logging
        )
        self.stop_button.pack(padx=10, pady=10)

    def switch_gps(self):
        gps_switch.set()
        self.switch_id = (self.switch_id + 1) % 2
        if self.switch_id == 0:
            self.gps_label.config(text='Internal GPS')
        elif self.switch_id == 1:
            self.gps_label.config(text='External GPS')

    def start_cap(self):
        time_set.set()
        camera_start.set()

    def cap_now(self):
        cap_image.set()

    def stop_logging(self):
        logging_stop.set()
        GPIO.cleanup()
        self.root.quit()

if __name__ == '__main__':
    start_time = str(datetime.datetime.now())
    logger_folder = f'logs/{start_time}'
    os.makedirs(logger_folder)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(GPS1, GPIO.OUT)
    GPIO.setup(GPS2, GPIO.OUT)

    if SAVE_ERROR:
        sys.stderr = open(f'{logger_folder}/err.txt', "w")

    time_set = threading.Event()
    camera_start = threading.Event()
    gps_switch = threading.Event()
    cap_image = threading.Event()
    logging_stop = threading.Event()

    camera_thread = threading.Thread(target=cameraRecord, args=())
    serial_thread = threading.Thread(target=serialRead, args=())

    camera_thread.start()
    serial_thread.start()

    root = tk.Tk()
    app = GNSSLoggerUI(root)
    root.mainloop()

    camera_thread.join()
    serial_thread.join()

    print("Program exited cleanly.")
