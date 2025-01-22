/**
  @file GPS.h
  @date 22/01/2025
  @version 1.0
  @brief This file contains the function prototypes and global variables for the GPS module.

  This file contains the function prototypes and global variables for the GPS module.
  The GPS module is used to read location data (latitude, longitude, date, time, altitude, speed, and satellites in view)
  from the GPS module and send the data to the Raspberry Pi.

  This file uses TinyGPSPlus library to parse the GPS data.
*/

#ifndef GPS_H
#define GPS_H

#include <TinyGPSPlus.h>
#include <math.h>
#include <stdint.h>

typedef uint32_t u32;
typedef uint8_t  u8;

extern TinyGPSPlus gps;
// #define PPS_PIN 38
extern u8 ubxCfgRate[];
extern int idxGPS;
extern bool outputFlagGPS;

void setupGPS(HardwareSerial &gpsSerial, HardwareSerial &piSerial);
void loopGPS(HardwareSerial &gpsSerial, HardwareSerial &piSerial, bool &outputFlagGPS);
// void handlePPS();
void printGPSData(HardwareSerial &piSerial, bool &outputFlagGPS);
void dec2dms(double dec);
void converter(double Latitude, double Longitude, double &Easting, double &Northing);

#endif // GPS_H
