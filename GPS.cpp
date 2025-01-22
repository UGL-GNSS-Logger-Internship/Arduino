/**
  @file GPS.cpp
  @date 22/01/2025
  @version 1.0
  @brief This file contains the function definitions for the GPS module.

  This file contains the function definitions for the GPS module.
  The GPS module is used to read location data (latitude, longitude, date, time, altitude, speed, and satellites in view)
  from the GPS module and send the data to the Raspberry Pi.

*/

#include "GPS.h"

TinyGPSPlus gps;

u8 ubxCfgRate[]{
    // 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, // output@1Hz
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4, 0x01, 0x01, 0x00, 0x01, 0x00, 0x0B, 0x77, // output@2Hz
};

int idxGPS = 0;

void setupGPS(HardwareSerial &gpsSerial, HardwareSerial &piSerial)
{
  // Serial.begin(9600);    // For debugging (PC communication)
  gpsSerial.begin(9600); // For GPS communication
  // piSerial.begin(9600);  // For Arduino - Raspberry Pi communication

  // pinMode(PPS_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(PPS_PIN), handlePPS, RISING);

  gpsSerial.write(ubxCfgRate, sizeof(ubxCfgRate));
  Serial.println("GPS module initialized.");
  piSerial.write("GPS module initialized.");
}

void loopGPS(HardwareSerial &gpsSerial, HardwareSerial &piSerial, bool &outputFlagGPS)
{
  // Read data from GPS
  if (gpsSerial.available() > 0)
  {
    char gpsdata = gpsSerial.read();
    // Serial.print(gpsdata);
    gps.encode(gpsdata);

    if (gps.location.isUpdated()) {
      // Print GPS data
      printGPSData(piSerial, outputFlagGPS);
    }
  }
}

// void handlePPS() {
//   Serial.println("PPS signal");
// }

void printGPSData(HardwareSerial &piSerial, bool &outputFlagGPS)
{
  double Latitude, Longitude, Easting, Northing;
  // Print latitude and longitude
  if (gps.location.isValid())
  {
    Latitude = gps.location.lat();
    Longitude = gps.location.lng();
    converter(Latitude, Longitude, Easting, Northing);
    if (outputFlagGPS)
    {
      Serial.print("Latitude: ");
      Serial.print(Latitude, 6);
      Serial.print(", Longitude: ");
      Serial.println(Longitude, 6);
      Serial.print("Easting: ");
      Serial.print(Easting, 3);
      Serial.print(", Northing: ");
      Serial.println(Northing, 3);
    }
  }
  else
  {
    Latitude = NULL;
    Longitude = NULL;
    Easting = NULL;
    Northing = NULL;
    if (outputFlagGPS)
    {
      Serial.println("Location not available");
    }
  }

  // Print date and time
  u32 Date, Time;
  if (gps.date.isValid() && gps.time.isValid())
  {
    Date = gps.date.value();
    Time = gps.time.value();
    if (outputFlagGPS)
    {
      Serial.print("Date: ");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.println(gps.date.year());
      Serial.print("Time: ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.print(gps.time.second());
      Serial.println(" UTC");
    }
  }
  else
  {
    Date = NULL;
    Time = NULL;
    if (outputFlagGPS)
    {
      Serial.println("Date/Time not available");
    }
  }

  // Print altitude
  double Altitude;
  if (gps.altitude.isValid())
  {
    Altitude = gps.altitude.meters();
    if (outputFlagGPS)
    {
      Serial.print("Altitude: ");
      Serial.println(" meters");
    }
  }
  else
  {
    Altitude = NULL;
    if (outputFlagGPS)
    {
      Serial.println("Altitude not available");
    }
  }

  // Print speed
  double Speed;
  if (gps.speed.isValid())
  {
    Speed = gps.speed.kmph();
    if (outputFlagGPS)
    {
      Serial.print("Speed: ");
      Serial.print(Speed);
      Serial.println(" km/h");
    }
  }
  else
  {
    Speed = NULL;
    if (outputFlagGPS)
    {
      Serial.println("Speed not available");
    }
  }

  // Print satellites in view
  if (gps.satellites.isValid())
  {
    if (outputFlagGPS)
    {
      Serial.print("Satellites in view: ");
      Serial.println(gps.satellites.value());
    }
  }
  else
  {
    if (outputFlagGPS)
    {
      Serial.println("Satellite data not available");
    }
  }

  if (outputFlagGPS)
  {
    Serial.println();
  }

  // piSerial.print(idxGPS);
  // piSerial.print(',');
  // piSerial.print(Date);
  // piSerial.print(Time);
  // piSerial.print(',');
  // piSerial.print(Easting);
  // piSerial.print(',');
  // piSerial.print(Northing);
  // piSerial.print(',');
  // piSerial.print(Altitude);
  // piSerial.print(',');
  // piSerial.print(Latitude, 6);
  // piSerial.print(',');
  // piSerial.print(Longitude, 6);
  // piSerial.print(',');
  // piSerial.println(Altitude);

  Serial.print(idxGPS);
  Serial.print(',');
  Serial.print(Date);
  Serial.print(Time);
  Serial.print(',');
  Serial.print(Easting);
  Serial.print(',');
  Serial.print(Northing);
  Serial.print(',');
  Serial.print(Altitude);
  Serial.print(',');
  Serial.print(Latitude, 6);
  Serial.print(',');
  Serial.print(Longitude, 6);
  Serial.print(',');
  Serial.println(Altitude);
}

void dec2dms(double dec)
{
  int degrees = int(dec);
  int minutes = int(abs(dec - degrees) * 60);
  double seconds = ((abs(dec - degrees) * 60) - minutes) * 60;

  Serial.print(degrees);
  Serial.print("\u00B0");
  Serial.print(minutes);
  Serial.print("'");
  Serial.print(seconds, 3);
  Serial.print("\"");
}

void converter(double Latitude, double Longitude, double &Easting, double &Northing)
{
  const double a = 6378137.000;
  const double FalseOriginE = 500000.0000;
  const double FalseOriginN = 10000000.0000;
  const double K0 = 0.9996;
  const double e2 = 0.006694380023;
  const double e4 = e2 * e2;
  const double e6 = e4 * e2;

  double radLat = Latitude * M_PI / 180;
  double radLng = Longitude * M_PI / 180;
  double diffLng = Longitude - 153.000000;
  double radDiffLng = diffLng * M_PI / 180;

  double SinLat1 = sin(radLat);
  double SinLat2 = sin(2 * radLat);
  double SinLat4 = sin(4 * radLat);
  double SinLat6 = sin(6 * radLat);

  double A0 = 1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256;
  double A2 = 3 * (e2 + e4 / 4 + 15 * e6 / 128) / 8;
  double A4 = 15 * (e4 + 3 * e6 / 4) / 256;
  double A6 = 35 * e6 / 3072;

  double M1 = a * A0 * radLat;
  double M2 = -a * A2 * SinLat2;
  double M3 = a * A4 * SinLat4;
  double M4 = -a * A6 * SinLat6;
  double M = M1 + M2 + M3 + M4;

  double Pho = a * (1 - e2) / pow((1 - e2 * SinLat1 * SinLat1), 1.5);
  double Nu = a / pow((1 - (e2 * SinLat1 * SinLat1)), 0.5);

  double CosLat1 = cos(radLat);
  double CosLat2 = CosLat1 * CosLat1;
  double CosLat3 = CosLat1 * CosLat2;
  double CosLat4 = CosLat2 * CosLat2;
  double CosLat5 = CosLat2 * CosLat3;
  double CosLat6 = CosLat3 * CosLat3;
  double CosLat7 = CosLat3 * CosLat4;

  double omega1 = radDiffLng;
  double omega2 = omega1 * omega1;
  double omega3 = omega1 * omega2;
  double omega4 = omega2 * omega2;
  double omega5 = omega2 * omega3;
  double omega6 = omega3 * omega3;
  double omega7 = omega3 * omega4;
  double omega8 = omega4 * omega4;

  double TanLat1 = tan(radLat);
  double TanLat2 = TanLat1 * TanLat1;
  double TanLat4 = TanLat2 * TanLat2;
  double TanLat6 = TanLat2 * TanLat4;

  double Psi1 = Nu / Pho;
  double Psi2 = Psi1 * Psi1;
  double Psi3 = Psi1 * Psi2;
  double Psi4 = Psi2 * Psi2;

  // Easting

  double E1 = Nu * omega1 * CosLat1;
  double E2 = Nu * omega3 * CosLat3 * (Psi1 - TanLat2) / 6;
  double E3 = Nu * omega5 * CosLat5 * (4 * Psi3 * (1 - 6 * TanLat2) + Psi1 * (1 + 8 * TanLat2) - Psi1 * (2 * TanLat2) + TanLat4) / 120;
  double E4 = Nu * omega7 * CosLat7 * (61 - 479 * TanLat2 + 179 * TanLat4 - TanLat6) / 5040;

  double SumE = E1 + E2 + E3 + E4;
  double SumEK0 = K0 * SumE;
  Easting = SumEK0 + FalseOriginE;

  // Northing
  double N1 = Nu * SinLat1 * omega2 * CosLat1 / 2;
  double N2 = Nu * SinLat1 * omega4 * CosLat3 * (4 * Psi2 + Psi1 - TanLat2) / 24;
  double N3 = Nu * SinLat1 * omega6 * CosLat5 * (61 - 58 * Psi4 + Psi2 * (4 * Psi4 + 9 * Psi2) - 4 * Psi1 * (7 * Psi4 + 11 * Psi2) - 7 * Psi1 * TanLat2) / 720;
  double N4 = Nu * SinLat1 * omega8 * CosLat7 * (1385 - 3111 * Psi2 + 543 * Psi4 - TanLat6) / 40320;

  double SumN = M + N1 + N2 + N3 + N4;
  double SumNK0 = K0 * SumN;
  Northing = SumNK0 + FalseOriginN;
}
