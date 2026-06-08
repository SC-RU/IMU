#include <Arduino.h>
#include <Wire.h>

#include "MPU6050.h"
#include "Calibration.h"
#include "Attitude.h"

// ----- Calibration data -----

AccelBias accelBias;
GyroBias gyroBias;

// ----- Attitude estimates -----

Attitude accelAttitude;
Attitude gyroAttitude;
Attitude filteredAttitude;

// ----- Timing and filter configuration -----

constexpr uint32_t LOOP_PERIOD_US = 500000;
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

uint32_t lastLoopTime = 0;

void setup()
{
  // ----- Hardware initialization -----
  
  Serial.begin(115200);
  Wire.begin();
  
  while (!Serial)
  {
  };
  
  // ----- Sensor initialization -----

  Serial.println("Initializing MPU6050...");
  initializeMPU6050();

  Serial.println("Calibrating accelerometer...");
  accelBias = calibrateAccel();

  Serial.println("Calibrating gyroscope...");
  gyroBias = calibrateGyro();

  // ----- Attitude initialization -----

  gyroAttitude = initializeAttitude(accelBias);
  filteredAttitude = complementaryFilter(gyroAttitude, accelAttitude, COMPLEMENTARY_ALPHA);

  Serial.println("Calibration complete.");

  lastLoopTime = micros(); // Initialize last loop time

  Serial.println("MPU6050 IMU awake.\n");
}

// bool hasRan = false;

void loop()
{
  // ----- Timing control -----

  // if (hasRan)
  // {
  //   return;
  // }

  uint32_t currentTime = micros();

  // If the time elapsed since the last loop is less than the sample rate, skip this loop
  if (currentTime - lastLoopTime < LOOP_PERIOD_US)
  {
    return;
  }

  float dt = (currentTime - lastLoopTime) / 1000000.0f; // Convert microseconds to seconds
  lastLoopTime = currentTime;

  // ----- Sensor acquisition & unit conversion -----

  AccelData accel = readRawAccel();
  GyroData gyro = readRawGyro();

  float axG = rawAccelToG(accel.x, accelBias.x);
  float ayG = rawAccelToG(accel.y, accelBias.y);
  float azG = rawAccelToG(accel.z, accelBias.z);

  float gxDPS = rawGyroToDPS(gyro.x, gyroBias.x);
  float gyDPS = rawGyroToDPS(gyro.y, gyroBias.y);
  float gzDPS = rawGyroToDPS(gyro.z, gyroBias.z);
  
  // ----- Attitude estimation -----

  accelAttitude = calculateAccelAttitude(axG, ayG, azG);
  gyroAttitude = updateGyroAttitude(gyroAttitude, gxDPS, gyDPS, gzDPS, dt);
  filteredAttitude = complementaryFilter(gyroAttitude, accelAttitude, COMPLEMENTARY_ALPHA);
  gyroAttitude = filteredAttitude; // Update gyro attitude to be the corrected estimate

  // ----- Telemetry Output -----

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
  
  Serial.print("Accel Roll: ");
  Serial.print(accelAttitude.roll);

  Serial.print(" Gyro Roll: ");
  Serial.print(gyroAttitude.roll);

  Serial.print(" Filtered Roll: ");
  Serial.println(filteredAttitude.roll);

  Serial.print("Accel Pitch: ");
  Serial.print(accelAttitude.pitch);

  Serial.print(" Gyro Pitch: ");
  Serial.print(gyroAttitude.pitch);

  Serial.print(" Filtered Pitch: ");
  Serial.println(filteredAttitude.pitch);

  // hasRan = true;
}