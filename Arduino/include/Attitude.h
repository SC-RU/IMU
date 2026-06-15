/******************************************************************************
 * @file    Attitude.h
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
 * @date    June 10, 2026
 ******************************************************************************/

#ifndef ATTITUDE_H
#define ATTITUDE_H

// -----------------------------------------------------------------------------
// Include files
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include "Calibration.h"

// -----------------------------------------------------------------------------
// Attitude estimation constants
// -----------------------------------------------------------------------------

constexpr uint16_t STARTUP_SAMPLES          = 100;              ///< Samples used for initial attitude estimate
constexpr uint8_t ATTITUDE_INIT_DELAY_MS    = 5;                ///< Delay between startup samples

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
struct Attitude
{
    float roll;
    float pitch;
};

// -----------------------------------------------------------------------------
// Accelerometer attitude estimation
// -----------------------------------------------------------------------------

/**
 * @brief   Computes roll and pitch from accelerometer data.
 *
 * @details  Uses the measured gravity vector to estimate
 *           sensor orientation.
 *
 *           This method does not accumulate drift, but
 *           becomes less accurate during rapid motion
 *           or when linear acceleration is present.
 *
 * @param ax Acceleration along the X-axis (g).
 * @param ay Acceleration along the Y-axis (g).
 * @param az Acceleration along the Z-axis (g).
 *
 * @return   Accelerometer attitude estimate.
 */
Attitude calculateAccelAttitude(
    float ax,
    float ay,
    float az);

// -----------------------------------------------------------------------------
// Gyroscope attitude estimation
// -----------------------------------------------------------------------------

/**
 * @brief          Updates attitude using gyroscope integration.
 *
 * @details        Angular velocity measurements are integrated
 *                 over the elapsed time interval to estimate
 *                 orientation.
 *
 *                 This method provides smooth short-term
 *                 tracking but gradually accumulates drift.
 * 
 *                 The previous attitude estimate is passed by const
 *                 reference because it is a read-only input to the
 *                 integration step. The function does not modify the
 *                 caller's object in place; instead, it computes and
 *                 returns a new Attitude estimate.
 *
 * @param previous Previous attitude estimate, read-only.
 * @param gx       X-axis angular velocity (degrees/second).
 * @param gy       Y-axis angular velocity (degrees/second).
 * @param gz       Z-axis angular velocity (degrees/second).
 * @param dt       Elapsed time since the previous update (seconds).
 *
 * @return         Updated gyroscope attitude estimate.
 */
Attitude updateGyroAttitude(
    const Attitude& previous,
    float gx,
    float gy,
    float gz,
    float dt);

// -----------------------------------------------------------------------------
// Sensor fusion
// -----------------------------------------------------------------------------

/**
 * @brief       Combines gyroscope and accelerometer estimates.
 *
 * @details     Implements a complementary filter:
 *
 *              filtered = α × gyro + (1 - α) × accel,
 * 
 *              where α is the filter weighting coefficient.
 *
 *              The gyroscope contributes short-term stability,
 *              while the accelerometer corrects long-term drift.
 * 
 *              Both input attitudes are passed by const reference because
 *              they are read-only inputs to the fusion step. The function 
 *              computes and returns a new fused estimate without modifying
 *              either caller-owned input object.
 *
 * @param gyro  Gyroscope attitude estimate, read-only.
 * @param accel Accelerometer attitude estimate, read-only.
 * @param alpha Complementary filter weighting coefficient.
 *
 * @return      Filtered attitude estimate.
 */
Attitude complementaryFilter(
    const Attitude& gyro,
    const Attitude& accel,
    float alpha);

// -----------------------------------------------------------------------------
// Startup attitude initialization
// -----------------------------------------------------------------------------

/**
 * @brief           Computes the initial attitude estimate.
 *
 * @details         Collects multiple accelerometer samples,
 *                  averages the measurements, and computes
 *                  the initial roll and pitch angles.
 *
 *                  This provides a stable starting point for
 *                  subsequent gyroscope integration.
 * 
 *                  The accelerometer bias is passed by const reference
 *                  because it is input-only configuration data.
 *                  The function does not modify the supplied
 *                  calibration values.
 *
 * @param accelBias Accelerometer calibration data, read-only.
 *
 * @return          Initial attitude estimate.
 */
Attitude initializeAttitude(const AccelBias& accelBias);

#endif // ATTITUDE_H