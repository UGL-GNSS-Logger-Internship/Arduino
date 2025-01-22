
#ifndef MPU_H
#define MPU_H

#include <MPU6050_light.h>
#include "Wire.h"

extern MPU6050 mpu;
extern int idxMPU;
extern bool outputFlagMPU;

void setupMPU(HardwareSerial &piSerial);
void processMPU(HardwareSerial &piSerial, bool &outputFlagGPS);

#endif