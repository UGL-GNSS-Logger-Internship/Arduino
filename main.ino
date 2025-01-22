// main.ino

#include "MPU.h"
#include "IR.h"
#include "GPS.h"

int sensorPin = A0;
HardwareSerial &gpsSerial = Serial2;
HardwareSerial &piSerial = Serial3;

bool outputFlagGPS = false;
bool outputFlagIR = false;
bool outputFlagMPU = false;

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  piSerial.begin(9600);

  // Setup GPS
  setupGPS(gpsSerial, piSerial);

  // Setup sensor
  setupSensor(sensorPin, piSerial);

  // Setup MPU
  setupMPU(piSerial);
}

void loop() {
  // Process GPS data
  loopGPS(gpsSerial, piSerial, outputFlagGPS);

  // Process sensor data
  processSensor(sensorPin, piSerial, outputFlagIR);

  // Process MPU data
  processMPU(piSerial, outputFlagMPU);

  // Listen to Raspberry Pi
  // listenToPi();
}

void listenToPi() {
  if (piSerial.available() > 0) {
    String incomingByte = piSerial.readStringUntil('\n');  // Read one byte at a time
    Serial.print("Received from piSerial: ");
    Serial.println(incomingByte);
  }
}