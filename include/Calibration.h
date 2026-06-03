#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>

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