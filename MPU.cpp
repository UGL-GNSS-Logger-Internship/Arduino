#include "MPU.h"

MPU6050 mpu(Wire);
int idxMPU = 1;

void setupMPU(HardwareSerial &piSerial)
{
  Wire.begin();
  // Set up the IMU
  byte status = mpu.begin();
  // Serial.print(F("MPU6050 status: "));
  // Serial.println(status);
  while (status != 0)
  { // stop everything if could not connect to MPU6050
  }
  // Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);
  // Serial.begin(9600);
  Serial.println("MPU initialized.");
  piSerial.write("MPU initialized.");
}

void processMPU(HardwareSerial &piSerial, bool &outputFlagMPU)
{
  mpu.update();
  if (outputFlagMPU)
  {
    Serial.print("Angle X: ");
    Serial.print(mpu.getAngleX());
    Serial.print(", Angle Y: ");
    Serial.print(mpu.getAngleY());
    Serial.print(", Angle Z: ");
    Serial.println(mpu.getAngleZ());
  }

  // piSerial.print(idxMPU);
  // piSerial.print(',');
  // piSerial.print(mpu.getAngleX());
  // piSerial.print(',');
  // piSerial.print(mpu.getAngleY());
  // piSerial.print(',');
  // piSerial.println(mpu.getAngleZ());

  Serial.print(idxMPU);
  Serial.print(',');
  Serial.print(mpu.getAngleX());
  Serial.print(',');
  Serial.print(mpu.getAngleY());
  Serial.print(',');
  Serial.println(mpu.getAngleZ());
}
