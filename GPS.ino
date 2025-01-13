#include <TinyGPSPlus.h>
#include <math.h>

TinyGPSPlus gps;
#define GPS_TX_PIN 18
#define GPS_RX_PIN 19
#define PPS_PIN 38

void setup() {
  Serial.begin(9600);    // For debugging (PC communication)
  Serial1.begin(9600);   // For GPS communication

  pinMode(PPS_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(PPS_PIN), handlePPS, RISING);

  Serial.print("GPS module initialized.");
}

void loop() {
  // Read data from GPS
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    // Serial.print(c);
    gps.encode(c); 

    if (gps.location.isUpdated()) {
      // Print GPS data
      printGPSData();
    }
  }
}

void handlePPS() {
  Serial.println("PPS signal");
}
void printGPSData() {
  // Print latitude and longitude
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    double Latitude = gps.location.lat();
    // Serial.print(Latitude, 6);
    dec2dms(Latitude);
    Serial.print(", Longitude: ");
    double Longitude = gps.location.lng();
    // Serial.println(Longitude, 6);
    dec2dms(Longitude);
    // converter(Latitude, Longitude);
  } else {
    Serial.println("Location not available");
  }

  // Print date and time
  if (gps.date.isValid() && gps.time.isValid()) {
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
  } else {
    Serial.println("Date/Time not available");
  }

  // Print altitude
  if (gps.altitude.isValid()) {
    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" meters");
  } else {
    Serial.println("Altitude not available");
  }

  // Print speed
  if (gps.speed.isValid()) {
    Serial.print("Speed: ");
    Serial.print(gps.speed.kmph());
    Serial.println(" km/h");
  } else {
    Serial.println("Speed not available");
  }

  // Print satellites in view
  if (gps.satellites.isValid()) {
    Serial.print("Satellites in view: ");
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("Satellite data not available");
  }

  Serial.println(); 
}

void dec2dms(double dec) {
  int degrees = int(dec);  
  int minutes = int(abs(dec - degrees) * 60);
  double seconds = ((abs(dec - degrees) * 60) - minutes) * 60;
  
  Serial.print(degrees);
  Serial.print((char)248);
  Serial.print(minutes);
  Serial.print("' ");
  Serial.print(seconds, 6);
  Serial.println("\"");
}

void converter(double Latitude, double Longitude) {
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
  double SinLat2 = sin(2*radLat);
  double SinLat4 = sin(4*radLat);
  double SinLat6 = sin(6*radLat);

  double A0 = 1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256;
  double A2 = 3 * (e2 + e4 / 4 + 15 * e6 / 128) / 8;
  double A4 = 15 * (e4 + 3 * e6 / 4) / 256;
  double A6 = 35 * e6 / 3072;

  double M1 = a * A0 * radLat;
  double M2 = - a * A2 * SinLat2;
  double M3 = a * A4 * SinLat4;
  double M4 = - a * A6 * SinLat6;
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
  double Easting = SumEK0 + FalseOriginE;

  // Northing
  double N1 = Nu * SinLat1 * omega2 * CosLat1 / 2;
  double N2 = Nu * SinLat1 * omega4 * CosLat3 * (4 * Psi2 + Psi1 - TanLat2) / 24;
  double N3 = Nu * SinLat1 * omega6 * CosLat5 * (61 - 58 * Psi4 + Psi2 * (4 * Psi4 + 9 * Psi2) - 4 * Psi1 * (7 * Psi4 + 11 * Psi2) - 7 * Psi1 * TanLat2) / 720;
  double N4 = Nu * SinLat1 * omega8 * CosLat7 * (1385 - 3111 * Psi2 + 543 * Psi4 - TanLat6) / 40320;

  double SumN = M + N1 + N2 + N3 + N4;
  double SumNK0 = K0 * SumN;
  double Northing = SumNK0 + FalseOriginN;

  Serial.print("Easting: ");
  Serial.print(Easting, 10);
  Serial.print("Northing: ");
  Serial.println(Northing, 10);
}