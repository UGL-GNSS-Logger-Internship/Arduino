#!/usr/bin/env python3
import os
import sys
import serial
import cv2
from picamera2 import Picamera2
import datetime
import time
import numpy as np
import webbrowser
import threading
import RPi.GPIO as GPIO
import subprocess

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

def cameraRecord(logger_folder):
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640, 480)
    picam2.preview_configuration.main.format = 'RGB888'
    picam2.configure('preview')
    picam2.start()

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(f'{logger_folder}/logger.mp4', fourcc, 30.0, (640, 480))

    while True:
        try:
            date_time = str(datetime.datetime.now())

            frame = picam2.capture_array()
            font = cv2.FONT_HERSHEY_SIMPLEX
            image = cv2.putText(image, date_time, (20, 50), font, 1, (1, 255, 255), 4)
            cv2.imshow('Live Video Recording (UGL)', image)
            out.write(image)

            if cv2.waitKey(1) != -1:
                break
        except KeyboardInterrupt:
            break

    picam2.close()
    cv2.destroyAllWindows()

    return 0

def serialRead(logger_folder):
    ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

    while True:
        try:
            if ser.in_waiting > 0:
                try:
                    data = ser.readline().decode("utf-8").strip()
                except:
                    data = ""

                fields = data.split(',')

                if fields[0] == '$GPRMC':
                    date_time = str(datetime.datetime.now())
                    time_srt = fields[1]
                    valid = fields[2]
                    latitude = fields[3]
                    direction_ns = fields[4]
                    longitude = fields[5]
                    direction_ew = fields[6]
                    speed = fields[7]
                    angle = fields[8]
                    date_srt = fields[9]
                    magnetic_var = fields[10]
                    magnetic_dir = fields[11]
                    mode_ind = fields[12]

                    with open(f'{logger_folder}/gps.txt', 'a') as f:
                        f.write(f"{date_time},{time_srt},{valid},{latitude},{direction_ns},{longitude},{direction_ew},{speed},{angle},{date_srt},{magnetic_var},{magnetic_dir},{mode_ind}\n")
                    easting, northing = converter(latitude, longitude)
                    with open(f'{logger_folder}/logger.txt', 'a') as f:
                        f.write(f"{date_srt},{time_srt},{latitude},{longitude},{easting},{northing}\n")
        except KeyboardInterrupt:
            break

    ser.close()
    return 0

if __name__ == '__main__':
    start_time = str(datetime.datetime.now())
    logger_folder = start_time
    log_file = start_time / 'logger.log'
    os.makedirs(logger_folder)

    # Creates a log file and redirects errors to it
    sys.stderr = open(f'{logger_folder}/err.txt', "w")

    camera_thread = threading.Thread(target=cameraRecord, args=(logger_folder))
    serial_thread = threading.Thread(target=serialRead, args=(logger_folder))

    camera_thread.start()
    serial_thread.start()

    camera_thread.join()
    serial_thread.join()

    print("Program exited cleanly.")