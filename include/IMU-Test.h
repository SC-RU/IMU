#ifndef IMU-TEST.H
#define IMU-TEST.H

#include <Arduino.h>
#include <Wire.h>

void confAdd(); // Confirm the devices address
void readAccelConfig(); // Reads the device's accelerometer configuration
void readGyroConfig(); // Reads the device's gyroscope configuration

void wakeUpIMU(); // Wakes up the IMU (plus a few more things)
void redNoise(); // Reduces noise
void setAccelFS(); // Sets the accelerometer's full scale range (sensitivity) to +- 2g
void setGyroFS(); // Sets the gyroscope's full-scale range (sensitivity) to +-250 deg/s
void calibrateAccel(); // Calibrates the accelerometer
void calibrateGyro(); // Calibrates the gyroscope

void readRawAccel(int16_t &ax, int16_t &ay, int16_t &az); // Gets the raw accelerometer data
void readRawGyro(int16_t &gx, int16_t &gy, int16_t &gz); // Gets the raw gyroscope data

float rawAToG(int16_t rawA, float bias); // Converts raw accelerometer data into g's, taking into account the accelerometer's biases
float aGToMs2(float aG); // Converts accelerometer data in g's into m/s^2
float rawGToDPS(int16_t rawG, float bias); // Converts raw gyroscope data into degrees per second, taking into account the gyroscope's biases

#endif