#include "Calibration.h"
#include "MPU6050.h"

AccelBias calibrateAccel()
{
    long sumX = 0;
    long sumY = 0;
    long sumZ = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        AccelData accel = readRawAccel();

        sumX += accel.x;
        sumY += accel.y;
        sumZ += accel.z;

        delay(5);
    }

    AccelBias bias;

    bias.x = (float)sumX / CALIBRATION_SAMPLES;
    bias.y = (float)sumY / CALIBRATION_SAMPLES;

    bias.z = ((float)sumZ / CALIBRATION_SAMPLES) - ACCEL_SENSITIVITY;

    return bias;
}

GyroBias calibrateGyro()
{
    long sumX = 0;
    long sumY = 0;
    long sumZ = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        GyroData gyro = readRawGyro();

        sumX += gyro.x;
        sumY += gyro.y;
        sumZ += gyro.z;

        delay(5);
    }

    GyroBias bias;

    bias.x = (float)sumX / CALIBRATION_SAMPLES;
    bias.y = (float)sumY / CALIBRATION_SAMPLES;
    bias.z = (float)sumZ / CALIBRATION_SAMPLES;

    return bias;
}

float rawAccelToG(int16_t raw, float bias)
{
    return ((float)raw - bias) / ACCEL_SENSITIVITY;
}

float rawGyroToDPS(int16_t raw, float bias)
{
    return ((float)raw - bias) / GYRO_SENSITIVITY;
}

float gToMetersPerSecondSquared(float g)
{
    return g * GRAVITY;
}