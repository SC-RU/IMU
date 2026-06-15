/******************************************************************************
 * @file    Calibration.h
 * @brief   Sensor calibration and unit conversion functions.
 *
 * @details This module provides:
 *          - Accelerometer bias estimation
 *          - Gyroscope bias estimation
 *          - Raw-to-physical unit conversion
 *
 *          Calibration is performed while the IMU remains
 *          stationary during startup. The computed biases
 *          are then applied to future sensor measurements
 *          to improve accuracy.
 *
 * @author  Sumedh Camarushi
 * @date    June 10, 2026
 ******************************************************************************/

#ifndef CALIBRATION_H
#define CALIBRATION_H

// -----------------------------------------------------------------------------
// Include files
// -----------------------------------------------------------------------------

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Calibration constants
// -----------------------------------------------------------------------------

constexpr uint16_t CALIBRATION_SAMPLES  = 500;        ///< Number of startup samples
constexpr uint8_t CALIBRATION_DELAY_MS  = 5;          ///< Delay between calibration samples

// -----------------------------------------------------------------------------
// MPU6050 conversion factors
// -----------------------------------------------------------------------------

// Accelerometer conversion
constexpr float ACCEL_SENSITIVITY_2G    = 16384.0f;   ///< LSB/g
constexpr float STANDARD_GRAVITY        = 9.80665f;   ///< m/s²

// Gyroscope conversion
constexpr float GYRO_SENSITIVITY_250DPS = 131.0f;     ///< LSB/(°/s)

// -----------------------------------------------------------------------------
// Calibration data structures
// -----------------------------------------------------------------------------

/**
 * @brief   Accelerometer bias values.
 *
 * @details Stores the average sensor offset for each
 *          accelerometer axis.
 *
 *          The Z-axis bias is computed relative to
 *          Earth's gravitational acceleration.
 */
struct AccelBias
{
    float x;     ///< X-axis bias
    float y;     ///< Y-axis bias
    float z;     ///< Z-axis bias
};

/**
 * @brief   Gyroscope bias values.
 *
 * @details Stores the average stationary angular
 *          velocity measured by the gyroscope.
 *
 *          These offsets are subtracted from future
 *          measurements to reduce drift.
 */
struct GyroBias
{
    float x;     ///< X-axis bias
    float y;     ///< Y-axis bias
    float z;     ///< Z-axis bias
};

// -----------------------------------------------------------------------------
// Calibration routines
// -----------------------------------------------------------------------------

/**
 * @brief   Computes accelerometer bias values.
 *
 * @details Collects a series of stationary measurements
 *          and computes the average offset for each axis.
 *
 *          The Z-axis measurement is corrected for the
 *          presence of gravity.
 *
 * @return  Accelerometer calibration data.
 */
AccelBias calibrateAccel();

/**
 * @brief   Computes gyroscope bias values.
 *
 * @details Collects a series of stationary measurements
 *          and computes the average angular velocity
 *          offset for each axis.
 *
 * @return  Gyroscope calibration data.
 */
GyroBias calibrateGyro();

// -----------------------------------------------------------------------------
// Unit conversion
// -----------------------------------------------------------------------------

/**
 * @brief      Converts raw accelerometer data to g.
 *
 * @param raw  Raw accelerometer reading.
 * @param bias Accelerometer bias.
 *
 * @return     Acceleration in g.
 */
float rawAccelToG(int16_t raw, float bias);

/**
 * @brief   Converts acceleration from g to m/s².
 *
 * @param g Acceleration in units of g.
 *
 * @return  Acceleration in meters per second squared.
 */
float gToMetersPerSecondSquared(float g);

/**
 * @brief      Converts raw gyroscope data to degrees/second.
 *
 * @param raw  Raw gyroscope reading.
 * @param bias Gyroscope bias.
 *
 * @return     Angular velocity in degrees per second.
 */
float rawGyroToDPS(int16_t raw, float bias);

#endif // CALIBRATION_H