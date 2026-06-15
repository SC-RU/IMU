/******************************************************************************
 * @file    Calibration.cpp
 * @brief   Implementation of sensor calibration and unit conversion.
 *
 * @details This module estimates accelerometer and gyroscope
 *          bias values by averaging multiple stationary
 *          sensor samples during startup.
 *
 *          The resulting biases are used to compensate
 *          future measurements and convert raw sensor
 *          values into meaningful physical units.
 *
 * @author  Sumedh Camarushi
 * @date    June 10, 2026
 ******************************************************************************/

#include "Calibration.h"
#include "MPU6050.h"

// -----------------------------------------------------------------------------
// Accelerometer calibration
// -----------------------------------------------------------------------------

AccelBias calibrateAccel()
{
    // Accumulate a large number of stationary
    // samples to estimate sensor offset.

    int32_t sumX = 0;
    int32_t sumY = 0;
    int32_t sumZ = 0;

    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        AccelData accel = readRawAccel();

        sumX += accel.x;
        sumY += accel.y;
        sumZ += accel.z;

        // Allow time between samples so that
        // measurements are evenly spaced.

        delay(CALIBRATION_DELAY_MS);
    }

    AccelBias bias;

    // Compute the average offset for each axis.

    bias.x = (float)sumX / CALIBRATION_SAMPLES;
    bias.y = (float)sumY / CALIBRATION_SAMPLES;

    // When the sensor is stationary, the Z-axis
    // measures approximately +1 g due to gravity.
    //
    // Remove this expected value so that the
    // stored bias represents only sensor error.

    bias.z = ((float)sumZ / CALIBRATION_SAMPLES) - ACCEL_SENSITIVITY_2G;

    return bias;
}

// -----------------------------------------------------------------------------
// Gyroscope calibration
// -----------------------------------------------------------------------------

GyroBias calibrateGyro()
{
    // While stationary, the gyroscope should
    // ideally report zero angular velocity.
    //
    // Any measured rotation is treated as
    // sensor bias.

    int32_t sumX = 0;
    int32_t sumY = 0;
    int32_t sumZ = 0;

    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        GyroData gyro = readRawGyro();

        sumX += gyro.x;
        sumY += gyro.y;
        sumZ += gyro.z;

        delay(CALIBRATION_DELAY_MS);
    }

    GyroBias bias;

    // Compute the average stationary offset.

    bias.x = (float)sumX / CALIBRATION_SAMPLES;
    bias.y = (float)sumY / CALIBRATION_SAMPLES;
    bias.z = (float)sumZ / CALIBRATION_SAMPLES;

    return bias;
}

// -----------------------------------------------------------------------------
// Unit conversion
// -----------------------------------------------------------------------------

float rawAccelToG(int16_t raw, float bias)
{
    // Remove sensor bias and convert from
    // least-significant bits (LSB) to g.

    return ((float)raw - bias) / ACCEL_SENSITIVITY_2G;
}

float gToMetersPerSecondSquared(float g)
{
    // Convert acceleration from units of g
    // into SI units.

    return g * STANDARD_GRAVITY;
}

float rawGyroToDPS(int16_t raw, float bias)
{
    // Remove sensor bias and convert from
    // least-significant bits (LSB) to
    // degrees per second.

    return ((float)raw - bias) / GYRO_SENSITIVITY_250DPS;
}