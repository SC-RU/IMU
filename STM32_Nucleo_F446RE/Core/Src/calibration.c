/******************************************************************************
 * @file    calibration.c
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
 * @date    June 25, 2026
 ******************************************************************************/

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------

#include "stm32f4xx_hal.h"
#include "calibration.h"
#include "mpu6050.h"

// -----------------------------------------------------------------------------
// Accelerometer calibration
// -----------------------------------------------------------------------------

AccelBias calibrateAccel(void)
{
    // Accumulate stationary accelerometer samples
    // until enough valid readings have been collected
    // or the maximum number of read attempts is reached.

    int32_t sumX = 0;
    int32_t sumY = 0;
    int32_t sumZ = 0;
    uint16_t validSamples = 0;
    uint16_t attempts = 0;

    while ((validSamples < CALIBRATION_SAMPLES) && (attempts < CALIBRATION_MAX_ATTEMPTS))
    {
        AccelData accel = {0};

        attempts++;

        // Only successful MPU6050 reads are included
        // in the accumulated bias estimate.

        if (readRawAccel(&accel) == HAL_OK)
        {
            sumX += accel.x;
            sumY += accel.y;
            sumZ += accel.z;
            validSamples++;
        }

        // Allow time between samples so that
        // measurements are evenly spaced.

        HAL_Delay(CALIBRATION_DELAY_MS);
    }

    AccelBias bias = {0};

    // If no valid samples were collected,
    // return a zero-initialized bias.

    if (validSamples == 0U)
    {
        return bias;
    }

    // Compute the average offset using only
    // the valid samples that were collected.

    bias.x = (float)sumX / validSamples;
    bias.y = (float)sumY / validSamples;

    // When the sensor is stationary, the Z-axis
    // measures approximately +1 g due to gravity.
    //
    // Remove this expected value so that the
    // stored bias represents only sensor error.

    bias.z = ((float)sumZ / validSamples) - ACCEL_SENSITIVITY_2G;

    return bias;
}

// -----------------------------------------------------------------------------
// Gyroscope calibration
// -----------------------------------------------------------------------------

GyroBias calibrateGyro(void)
{
    // Accumulate stationary gyroscope samples
    // until enough valid readings have been collected
    // or the maximum number of read attempts is reached.

    int32_t sumX = 0;
    int32_t sumY = 0;
    int32_t sumZ = 0;
    uint16_t validSamples = 0;
    uint16_t attempts = 0;

    while ((validSamples < CALIBRATION_SAMPLES) && (attempts < CALIBRATION_MAX_ATTEMPTS))
    {
        GyroData gyro = {0};

        attempts++;

        // Only successful MPU6050 reads are included
        // in the accumulated bias estimate.

        if (readRawGyro(&gyro) == HAL_OK)
        {
            sumX += gyro.x;
            sumY += gyro.y;
            sumZ += gyro.z;
            validSamples++;
        }

        HAL_Delay(CALIBRATION_DELAY_MS);
    }

    GyroBias bias = {0};

    // If no valid samples were collected,
    // return a zero-initialized bias.

    if (validSamples == 0U)
    {
        return bias;
    }

    // Compute the average stationary offset
    // using only the valid samples that were collected.

    bias.x = (float)sumX / validSamples;
    bias.y = (float)sumY / validSamples;
    bias.z = (float)sumZ / validSamples;

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
