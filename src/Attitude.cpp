#include "Attitude.h"
#include "MPU6050.h"
#include <math.h>

Attitude calculateAccelAttitude(float ax, float ay, float az)
{
    Attitude attitude;

    attitude.roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    attitude.pitch = atan2(ay, az) * 180.0f / PI;

    return attitude;
}

Attitude updateGyroAttitude(Attitude previous, float gx, float gy, float gz, float dt)
{
    previous.roll += gy * dt;
    previous.pitch += gx * dt;

    return previous;
}

Attitude complementaryFilter(Attitude gyro, Attitude accel, float alpha)
{
    Attitude filtered;

    filtered.roll = alpha * gyro.roll + (1.0f - alpha) * accel.roll;
    filtered.pitch = alpha * gyro.pitch + (1.0f - alpha) * accel.pitch;

    return filtered;
}

Attitude initializeAttitude(AccelBias accelBias)
{
    float axSum = 0.0f;
    float aySum = 0.0f;
    float azSum = 0.0f;

    for (int i = 0; i < STARTUP_SAMPLES; i++)
    {
        AccelData accel = readRawAccel();

        axSum += rawAccelToG(accel.x, accelBias.x);
        aySum += rawAccelToG(accel.y, accelBias.y);
        azSum += rawAccelToG(accel.z, accelBias.z);

        delay(5);
    }

    return calculateAccelAttitude(
        axSum / STARTUP_SAMPLES,
        aySum / STARTUP_SAMPLES,
        azSum / STARTUP_SAMPLES);
}