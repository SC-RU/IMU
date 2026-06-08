#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <Arduino.h>
#include "Calibration.h"

constexpr int STARTUP_SAMPLES = 100; // Number of samples to take at startup for attitude initialization

struct Attitude
{
    float roll;
    float pitch;
};

Attitude calculateAccelAttitude(
    float ax,
    float ay,
    float az);

Attitude updateGyroAttitude(
    Attitude previous,
    float gx,
    float gy,
    float gz,
    float dt);

Attitude complementaryFilter(
    Attitude gyro,
    Attitude accel,
    float alpha);

Attitude initializeAttitude(AccelBias accelBias);

#endif // ATTITUDE_H