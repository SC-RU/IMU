#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>

constexpr int CALIBRATION_SAMPLES = 500;

constexpr float ACCEL_SENSITIVITY = 16384.0f;
constexpr float GYRO_SENSITIVITY  = 131.0f;

constexpr float GRAVITY = 9.80665f;

struct AccelBias
{
    float x;
    float y;
    float z;
};

struct GyroBias
{
    float x;
    float y;
    float z;
};

AccelBias calibrateAccel();
GyroBias calibrateGyro();

float rawAccelToG(int16_t raw, float bias);
float rawGyroToDPS(int16_t raw, float bias);
float gToMetersPerSecondSquared(float g);

#endif // CALIBRATION_H