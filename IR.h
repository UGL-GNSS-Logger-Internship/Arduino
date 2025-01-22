#ifndef IR_H
#define IR_H

#include <Arduino.h>

extern unsigned long firstTimestamp;
extern bool flag;
extern int idxSensor;
extern bool outputFlagIR;

void setupSensor(int &sensorPin, HardwareSerial &piSerial);
void processSensor(int &sensorPin, HardwareSerial &piSerial, bool &outputFlagGPS);

#endif