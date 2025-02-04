#!/usr/bin/env python3
"""
File: main.py
Author: UNSW UGL GNSS Logger Internship Team
Version: 1.0.0
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
        pip install opencv-python numpy picamera2 pyserial
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
    - Label
        - GPS label: Show which GPS module is now using.
    - Button
        - Switch GPS: Switch GPS modules between using internal anteena and external antenna.
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

BUTTON = 6
GPS1 = 23
GPS2 = 24

def converter(Latitude, Longitude):
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
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = 'RGB888'
    picam2.configure('preview')
    picam2.start()

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(f'{logger_folder}/logger.mp4', fourcc, 30.0, (640,480))

    capture_folder = f'{logger_folder}/captured_img'
    os.makedirs(capture_folder)

    with open(f'{logger_folder}/captured_time.txt', 'w') as f:
        f.write('timestamp\n')

    while True:
        date_time = str(datetime.datetime.now())

        frame = picam2.capture_array()
        font = cv2.FONT_HERSHEY_SIMPLEX
        image = cv2.putText(frame, date_time, (20, 50), font, 1, (1, 255, 255), 4)

        if time_set.is_set() and camera_start.is_set():
            try:
                out.write(image)
                cv2.imshow('Live Video Recording (UGL)', image)

                if cv2.waitKey(1) != -1:
                    break

            except KeyboardInterrupt:
                break

        if GPIO.input(BUTTON) == GPIO.HIGH or cap_image.is_set():
            if not captured:
                cv2.imwrite(f'{capture_folder}/{date_time}.jpg', image)
                with open(f'{logger_folder}/captured_time.txt', 'a') as f:
                    f.write(date_time+'\n')
                captured = True
                cap_image.clear()
        else:
            captured = False

        if logging_stop.is_set():
            break

    picam2.close()
    cv2.destroyAllWindows()

    return 0

def serialRead():
    ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
    with open(f'{logger_folder}/gps.txt', 'w') as f:
        f.write(f"timestamp,UTC_time,valid,latitude,direction_ns,longitude,direction_ew,speed,angle,UTC_date,magnetic_var,magnetic_dir,mode_ind\n")
    with open(f'{logger_folder}/logger.txt', 'w') as f:
        f.write(f"timestamp,UTC_date,UTC_time,latitude,longitude,easting,northing\n")

    gps_select = 0
    GPIO.output(GPS1, GPIO.HIGH)
    GPIO.output(GPS2, GPIO.LOW)

    while True:
        if gps_switch.is_set():
            gps_select = (gps_select+1) % 2
            if gps_select == 0:
                GPIO.output(GPS1, GPIO.HIGH)
                GPIO.output(GPS2, GPIO.LOW)
            elif gps_select == 1:
                GPIO.output(GPS1, GPIO.LOW)
                GPIO.output(GPS2, GPIO.HIGH)
            gps_switch.clear()

        try:
            if ser.in_waiting > 0:
                try:
                    data = ser.readline().decode("utf-8").strip()
                except:
                    data = ""

                fields = data.split(',')

                if fields[0] == '$GPRMC':
                    time_str = fields[1]
                    valid = fields[2]
                    latitude = fields[3]
                    direction_ns = fields[4]
                    longitude = fields[5]
                    direction_ew = fields[6]
                    speed = fields[7]
                    angle = fields[8]
                    date_str = fields[9]
                    magnetic_var = fields[10]
                    magnetic_dir = fields[11]
                    mode_ind = fields[12]

                    if time_str != '' and date_str != '' and valid == 'A':
                        if not time_set.is_set():
                            time = f"{time_str[0:2]}:{time_str[2:4]}:{time_str[4:]}"
                            date = f"20{date_str[4:6]}-{date_str[2:4]}-{date_str[0:2]}"
                            result = subprocess.run(
                                ["sudo", "date", "-u", "--set", f"{date} {time}"],
                                capture_output = True,
                                text = True
                            )
                            print(result)
                            print(datetime.datetime.now())
                            time_set.set()
                        # elif time_set.is_set():
                        #     camera_start.set()
                    if time_set.is_set():
                        date_time = str(datetime.datetime.now())
                        with open(f'{logger_folder}/gps.txt', 'a') as f:
                            f.write(f"{date_time},{time_str},{valid},{latitude},{direction_ns},{longitude},{direction_ew},{speed},{angle},{date_str},{magnetic_var},{magnetic_dir},{mode_ind}\n")

                        if valid == 'A':
                            if not camera_start.is_set():
                                camera_start.set()
                            latitude_deg = float(latitude[0:2])+float(latitude[2:])/60
                            longitude_deg = float(longitude[0:3])+float(longitude[3:])/60
                            if direction_ns == 'S':
                                latitude_deg = -latitude_deg
                            easting, northing = converter(float(latitude_deg), float(longitude_deg))
                            with open(f'{logger_folder}/logger.txt', 'a') as f:
                                f.write(f"{date_time},{date_str},{time_str},{latitude_deg},{longitude_deg},{easting},{northing}\n")

                if logging_stop.is_set():
                    break

            if logging_stop.is_set():
                break

        except KeyboardInterrupt:
            break

    ser.close()
    return 0

def ui():
    root = tk.Tk()
    root.title("GNSS LOGGER")

    gps_label = tk.Label(
        root,
        text='GPS 1'
    )
    gps_label.pack(padx=20, pady=10)

    switch_id = 0
    def switch_gps(switch_id):
        gps_switch.set()
        switch_id = (switch_id + 1) %2
        print
        if switch_id == 0:
            gps_label.config(text='GPS 1')
        if switch_id == 1:
            gps_label.config(text='GPS 2')

    switch_button = tk.Button(
        root,
        text='Switch GPS',
        command=lambda:switch_gps(switch_id),
    )
    switch_button.pack(padx=0, pady=10)

    def start_cap():
        time_set.set()
        camera_start.set()

    start_cap_button = tk.Button(
        root,
        text='Start recording',
        command=start_cap,
    )
    start_cap_button.pack(padx=10, pady=10)

    def cap_now():
        cap_image.set()

    cap_now_button = tk.Button(
        root,
        text='Capture an image',
        command=cap_now,
    )
    cap_now_button.pack(padx=10, pady=10)

    def stop_logging():
        logging_stop.set()
        GPIO.cleanup()
        root.quit()

    stop_button = tk.Button(
        root,
        text='Stop logging',
        command=stop_logging,
    )
    stop_button.pack(padx=10, pady=10)

    root.mainloop()

if __name__ == '__main__':
    start_time = str(datetime.datetime.now())
    logger_folder = f'logs/{start_time}'
    os.makedirs(logger_folder)

    GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
    GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(GPS1, GPIO.OUT)
    GPIO.setup(GPS2, GPIO.OUT)

    # Creates a log file and redirects errors to it
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

    ui()

    camera_thread.join()
    serial_thread.join()

    print("Program exited cleanly.")
