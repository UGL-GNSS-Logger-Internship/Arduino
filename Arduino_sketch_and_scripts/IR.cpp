#include "IR.h"

unsigned long firstTimestamp = -1;
bool flag = false;
int idxSensor = 2;

void setupSensor(int &sensorPin, HardwareSerial &piSerial) {
  // pinMode(sensorPin, INPUT);
  // Serial.begin(9600);
  Serial.println("Sensor initialized.");
  piSerial.write("Sensor initialized.");
}

void processSensor(int &sensorPin, HardwareSerial &piSerial, bool &outputFlagIR) {
  unsigned long currentMillis = millis();

  int val = analogRead(sensorPin);
  float voltage = val * (5.0 / 1023.0);
  if (voltage < 1) {
    if (!flag) {  // If flag is not set (i.e., we're in > 3.5V state)
      // If flag is set (i.e., we are in < 3.5V state)
      if (firstTimestamp != -1) {
        // Calculate the output ((1/3) / (time in seconds since the first < 3.5V reading))
        float timeInSeconds = (currentMillis - firstTimestamp) / 1000.0;
        float result = (1.0 / 4.0) * 2 * PI / timeInSeconds;
        // Output the result
        if (outputFlagIR) {
          Serial.print("IR Output: ");
          Serial.print(result);
          Serial.println(", ");
        }

        // piSerial.print(idxSensor);
        // piSerial.print(',');
        // piSerial.println(result);

        Serial.print(idxSensor);
        Serial.print(',');
        Serial.println(result);
      }
      firstTimestamp = currentMillis;  // Store timestamp of the first < 3.5V reading
      flag = true;                     // Set the flag to true
    }
  } else {

    flag = false;  // Set the flag to false when we get the next > 3.5V reading
  }
}