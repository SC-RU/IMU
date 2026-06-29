/******************************************************************************
 * @file    attitude.c
 * @brief   Implementation of attitude estimation and sensor fusion.
 *
 * @details This module computes sensor orientation using
 *          both accelerometer and gyroscope measurements.
 *
 *          The implementation includes:
 *
 *          - Accelerometer-based roll and pitch estimation
 *          - Gyroscope attitude integration
 *          - Complementary filter sensor fusion
 *          - Startup attitude initialization
 *
 * @author  Sumedh Camarushi
 * @date    June 25, 2026
 ******************************************************************************/

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------

#include "attitude.h"
#include "mpu6050.h"
#include <math.h>

// -----------------------------------------------------------------------------
// Accelerometer attitude estimation
// -----------------------------------------------------------------------------

HAL_StatusTypeDef calculateAccelAttitude(
    float ax,
    float ay,
    float az,
    Attitude *attitude)
{
    // Verify that the output pointer is valid
    // before computing the attitude estimate.

    if (attitude == 0)
    {
        return HAL_ERROR;
    }

    // These equations estimate orientation
    // from the direction of the measured
    // gravity vector.
    //
    // The accelerometer provides an
    // absolute reference that does not
    // accumulate drift. However, it is sensitive
    // to linear acceleration and rapid motion
    // because the accelerometer measures all applied
    // acceleration, not gravity alone.

    attitude->roll = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    attitude->pitch = atan2f(ay, az) * RAD_TO_DEG;

    return HAL_OK;
}


// -----------------------------------------------------------------------------
// Gyroscope attitude estimation
// -----------------------------------------------------------------------------

HAL_StatusTypeDef updateGyroAttitude(
    const Attitude *previous,
    float gx,
    float gy,
    float gz,
    float dt,
    Attitude *attitude)
{
    // Verify that the input and output pointers
    // are valid before updating the attitude.

    if ((previous == 0) || (attitude == 0))
    {
        return HAL_ERROR;
    }

    // Integrate angular velocity over time.
    //
    // angle = previous_angle + rate * dt
    //
    // The previous attitude estimate is provided
    // as a read-only input, while the updated
    // estimate is written to the output structure.
    //
    // The current implementation estimates
    // only roll and pitch.

    attitude->roll = previous->roll + gy * dt;
    attitude->pitch = previous->pitch + gx * dt;

    // gz is currently unused because
    // yaw estimation is not implemented.
    //
    // The parameter is retained so that
    // the interface naturally extends to
    // future three-axis attitude estimation.

    (void)gz;

    return HAL_OK;
}


// -----------------------------------------------------------------------------
// Sensor fusion
// -----------------------------------------------------------------------------

HAL_StatusTypeDef complementaryFilter(
    const Attitude *gyro,
    const Attitude *accel,
    float alpha,
    Attitude *attitude)
{
    // Verify that the input and output pointers
    // are valid before fusing the estimates.

    if ((gyro == 0) || (accel == 0) || (attitude == 0))
    {
        return HAL_ERROR;
    }

    // Combine the short-term responsiveness of the gyroscope estimate
    // with the long-term drift correction provided by the accelerometer.
    //
    // The input estimates are treated as read-only inputs
    // to the fusion step. The filter computes a new
    // fused attitude estimate without altering either
    // source estimate.

    attitude->roll = alpha * gyro->roll + (1.0f - alpha) * accel->roll;
    attitude->pitch = alpha * gyro->pitch + (1.0f - alpha) * accel->pitch;

    return HAL_OK;
}


// -----------------------------------------------------------------------------
// Startup attitude initialization
// -----------------------------------------------------------------------------

HAL_StatusTypeDef initializeAttitude(
    const AccelBias *accelBias,
    Attitude *attitude)
{
    // Verify that the input and output pointers
    // are valid before computing the initial attitude.

    if ((accelBias == 0) || (attitude == 0))
    {
        return HAL_ERROR;
    }

    // Average multiple accelerometer samples
    // to reduce startup noise.
    //
    // Only valid MPU6050 reads are accumulated.
    // Sampling continues until enough valid
    // samples have been collected or the
    // maximum number of attempts is reached.

    float axSum = 0.0f;
    float aySum = 0.0f;
    float azSum = 0.0f;
    uint16_t validSamples = 0;
    uint16_t attempts = 0;

    while ((validSamples < STARTUP_SAMPLES) &&
           (attempts < ATTITUDE_INIT_MAX_ATTEMPTS))
    {
        AccelData accel = {0};

        attempts++;

        if (readRawAccel(&accel) == HAL_OK)
        {
            axSum += rawAccelToG(accel.x, accelBias->x);
            aySum += rawAccelToG(accel.y, accelBias->y);
            azSum += rawAccelToG(accel.z, accelBias->z);
            validSamples++;
        }

        // Space measurements evenly
        // during initialization.

        HAL_Delay(ATTITUDE_INIT_DELAY_MS);
    }

    // Return an error if no valid samples
    // were collected during initialization.

    if (validSamples == 0U)
    {
        return HAL_ERROR;
    }

    // Compute the average gravity vector
    // and estimate the initial orientation.

    return calculateAccelAttitude(
        axSum / validSamples,
        aySum / validSamples,
        azSum / validSamples,
        attitude);
}
