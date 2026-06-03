#include <Arduino.h>
#include <Wire.h>

#include "MPU6050.h"
#include "Calibration.h"

AccelBias accelBias;
GyroBias gyroBias;

const uint32_t LOOP_PERIOD = 10000; // Sample rate = 100 Hz (10,000 microseconds per loop)
uint32_t lastLoopTime = 0;

void setup()
{
  Serial.begin(115200); // Begin serial communication with a 115200 bps baud rate
  Wire.begin(); // Begin I2C communication
  while(!Serial) // Wait until connection is secured
  {
  };

  Serial.println("Initializing MPU6050...");
  initializeMPU6050();

  Serial.println("Calibrating accelerometer...");
  accelBias = calibrateAccel();

  Serial.println("Calibrating gyroscope...");
  gyroBias = calibrateGyro();

  Serial.println("Calibration complete.");
  
  Serial.println("MPU6050 IMU awake.\n");
}

// bool hasRan = false;

void loop()
{
  uint32_t currentTime = micros();

  // If the time elapsed since the last loop is less than the sample rate, skip this loop
  if (currentTime - lastLoopTime < LOOP_PERIOD)
  {
    return;
  }

  // Convert time elapsed to seconds
  float dt = (currentTime - lastLoopTime) / 1e6f;
  lastLoopTime = currentTime;

  AccelData accel = readRawAccel();
  GyroData gyro = readRawGyro();

  float axG = rawAccelToG(accel.x, accelBias.x);
  float ayG = rawAccelToG(accel.y, accelBias.y);
  float azG = rawAccelToG(accel.z, accelBias.z);

  float gxDPS = rawGyroToDPS(gyro.x, gyroBias.x);
  float gyDPS = rawGyroToDPS(gyro.y, gyroBias.y);
  float gzDPS = rawGyroToDPS(gyro.z, gyroBias.z);

  // Serial.print("AX (g): ");
  // Serial.print(axG);

  // Serial.print(" AY (g): ");
  // Serial.print(ayG);

  // Serial.print(" AZ (g): ");
  // Serial.println(azG);

  // Serial.print("GX (dps): ");
  // Serial.print(gxDPS);

  // Serial.print(" GY (dps): ");
  // Serial.print(gyDPS);

  // Serial.print(" GZ (dps): ");
  // Serial.println(gzDPS);

  // hasRan = true;
}