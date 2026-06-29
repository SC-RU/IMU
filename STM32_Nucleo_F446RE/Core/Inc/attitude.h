/******************************************************************************
 * @file    attitude.h
 * @brief   Attitude estimation and sensor fusion functions.
 *
 * @details This module provides:
 *          - Accelerometer-based attitude estimation
 *          - Gyroscope attitude integration
 *          - Complementary filter sensor fusion
 *          - Initial attitude estimation during startup
 *
 *          The current implementation estimates two Euler angles:
 *
 *          - Roll
 *          - Pitch
 *
 *          Yaw is not estimated because the MPU6050 does not
 *          include a magnetometer, and gyroscope-only yaw
 *          integration would accumulate drift over time.
 *
 * @author  Sumedh Camarushi
 * @date    June 25, 2026
 ******************************************************************************/

// -----------------------------------------------------------------------------
// Define to prevent recursive inclusion
// -----------------------------------------------------------------------------

#ifndef ATTITUDE_H
#define ATTITUDE_H

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "calibration.h"

// -----------------------------------------------------------------------------
// Attitude estimation constants
// -----------------------------------------------------------------------------

#define STARTUP_SAMPLES               100U                       ///< Valid samples used for initial attitude estimate
#define ATTITUDE_INIT_DELAY_MS        5U                         ///< Delay between startup samples in milliseconds
#define ATTITUDE_INIT_MAX_ATTEMPTS    (STARTUP_SAMPLES * 2U)     ///< Maximum startup attitude read attempts

// -----------------------------------------------------------------------------
// Math constants
// -----------------------------------------------------------------------------

#define PI_F                          3.14159265358979323846f    ///< Pi constant in single precision
#define RAD_TO_DEG                    (180.0f / PI_F)            ///< Radians-to-degrees conversion factor

// -----------------------------------------------------------------------------
// Attitude data structure
// -----------------------------------------------------------------------------

/**
 * @brief   Estimated sensor orientation.
 *
 * @details Stores the current roll and pitch angles
 *          expressed in degrees.
 *
 *          These values may represent:
 *          - Accelerometer attitude
 *          - Gyroscope attitude
 *          - Complementary filter output
 *
 *          Coordinate convention:
 *
 *          - Roll:  rotation about the IMU Y-axis
 *          - Pitch: rotation about the IMU X-axis
 *
 *          Angles are expressed in degrees.
 */
typedef struct
{
    float roll;  ///< Roll angle in degrees
    float pitch; ///< Pitch angle in degrees
} Attitude;

// -----------------------------------------------------------------------------
// Accelerometer attitude estimation
// -----------------------------------------------------------------------------

/**
 * @brief             Computes roll and pitch from accelerometer data.
 *
 * @details           Uses the measured gravity vector to estimate
 *                    sensor orientation.
 *
 *                    This method does not accumulate drift, but
 *                    becomes less accurate during rapid motion
 *                    or when linear acceleration is present.
 *
 * @param ax:         Acceleration along the X-axis (g).
 * @param ay:         Acceleration along the Y-axis (g).
 * @param az:         Acceleration along the Z-axis (g).
 * @param attitude:   Output attitude estimate.
 *
 * @retval HAL_OK     Accelerometer attitude was computed successfully.
 * @retval HAL_ERROR  Output pointer was invalid.
 */
HAL_StatusTypeDef calculateAccelAttitude(
    float ax,
    float ay,
    float az,
    Attitude *attitude);

// -----------------------------------------------------------------------------
// Gyroscope attitude estimation
// -----------------------------------------------------------------------------

/**
 * @brief             Updates attitude using gyroscope integration.
 *
 * @details           Angular velocity measurements are integrated
 *                    over the elapsed time interval to estimate
 *                    orientation.
 *
 *                    This method provides smooth short-term
 *                    tracking but gradually accumulates drift.
 *
 *                    The previous attitude estimate is provided
 *                    as a read-only input, and the updated result
 *                    is written to the output structure.
 *
 * @param previous:   Previous attitude estimate.
 * @param gx:         X-axis angular velocity (degrees/second).
 * @param gy:         Y-axis angular velocity (degrees/second).
 * @param gz:         Z-axis angular velocity (degrees/second).
 * @param dt:         Elapsed time since the previous update (seconds).
 * @param attitude:   Output gyroscope attitude estimate.
 *
 * @retval HAL_OK     Gyroscope attitude was computed successfully.
 * @retval HAL_ERROR  Input or output pointer was invalid.
 */
HAL_StatusTypeDef updateGyroAttitude(
    const Attitude *previous,
    float gx,
    float gy,
    float gz,
    float dt,
    Attitude *attitude);

// -----------------------------------------------------------------------------
// Sensor fusion
// -----------------------------------------------------------------------------

/**
 * @brief             Combines gyroscope and accelerometer estimates.
 *
 * @details           Implements a complementary filter:
 *
 *                    filtered = alpha * gyro + (1 - alpha) * accel
 *
 *                    where alpha is the filter weighting coefficient.
 *
 *                    The gyroscope contributes short-term stability,
 *                    while the accelerometer corrects long-term drift.
 *
 *                    Both input attitudes are treated as read-only,
 *                    and the fused estimate is written to the
 *                    output structure.
 *
 * @param gyro:       Gyroscope attitude estimate.
 * @param accel:      Accelerometer attitude estimate.
 * @param alpha:      Complementary filter weighting coefficient.
 * @param attitude:   Output filtered attitude estimate.
 *
 * @retval HAL_OK     Attitude estimates were fused successfully.
 * @retval HAL_ERROR  Input or output pointer was invalid.
 */
HAL_StatusTypeDef complementaryFilter(
    const Attitude *gyro,
    const Attitude *accel,
    float alpha,
    Attitude *attitude);

// -----------------------------------------------------------------------------
// Startup attitude initialization
// -----------------------------------------------------------------------------

/**
 * @brief               Computes the initial attitude estimate.
 *
 * @details             Collects accelerometer samples until either:
 *                      - STARTUP_SAMPLES valid samples have been collected, or
 *                      - ATTITUDE_INIT_MAX_ATTEMPTS total read attempts have been made.
 *
 *                      Only successful MPU6050 reads are accumulated
 *                      into the startup average. Failed reads are
 *                      ignored and do not contribute to the result.
 *
 *                      The averaged acceleration vector is converted
 *                      into an initial roll and pitch estimate that
 *                      provides a stable starting point for subsequent
 *                      gyroscope integration.
 *
 * @param accelBias:    Accelerometer calibration data.
 * @param attitude:     Output initial attitude estimate.
 *
 * @retval HAL_OK       Initial attitude estimate was computed successfully.
 * @retval HAL_ERROR    Output pointer was invalid or no valid samples were collected.
 */
HAL_StatusTypeDef initializeAttitude(
    const AccelBias *accelBias,
    Attitude *attitude);

#endif /* ATTITUDE_H */
