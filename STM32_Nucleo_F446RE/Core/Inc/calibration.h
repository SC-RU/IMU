/******************************************************************************
 * @file    calibration.h
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
 * @date    June 24, 2026
 ******************************************************************************/

// -----------------------------------------------------------------------------
// Define to prevent recursive inclusion
// -----------------------------------------------------------------------------

#ifndef CALIBRATION_H
#define CALIBRATION_H

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------

#include <stdint.h>

// -----------------------------------------------------------------------------
// Calibration constants
// -----------------------------------------------------------------------------

#define CALIBRATION_SAMPLES         500U                        ///< Number of startup samples
#define CALIBRATION_DELAY_MS        5U                          ///< Delay between calibration samples in milliseconds
#define CALIBRATION_MAX_ATTEMPTS    (CALIBRATION_SAMPLES * 2U)  ///< Maximum calibration read attempts

// -----------------------------------------------------------------------------
// MPU6050 conversion factors
// -----------------------------------------------------------------------------

// Accelerometer conversion
#define ACCEL_SENSITIVITY_2G     16384.0f   ///< LSB/g
#define STANDARD_GRAVITY         9.80665f   ///< m/s²

// Gyroscope conversion
#define GYRO_SENSITIVITY_250DPS  131.0f     ///< LSB/(°/s)

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
typedef struct
{
    float x; ///< X-axis bias
    float y; ///< Y-axis bias
    float z; ///< Z-axis bias
} AccelBias;

/**
 * @brief   Gyroscope bias values.
 *
 * @details Stores the average stationary angular
 *          velocity measured by the gyroscope.
 *
 *          These offsets are subtracted from future
 *          measurements to reduce drift.
 */
typedef struct
{
    float x; ///< X-axis bias
    float y; ///< Y-axis bias
    float z; ///< Z-axis bias
} GyroBias;

// -----------------------------------------------------------------------------
// Calibration routines
// -----------------------------------------------------------------------------

/**
 * @brief   Computes accelerometer bias values.
 *
 * @details Repeatedly collects stationary accelerometer
 *          measurements until either:
 *          - CALIBRATION_SAMPLES valid samples have been collected, or
 *          - CALIBRATION_MAX_ATTEMPTS total read attempts have been made.
 *
 *          Only successful MPU6050 reads are accumulated
 *          into the bias estimate. Failed reads are ignored
 *          and do not contribute to the final average.
 *
 *          The Z-axis measurement is corrected for the
 *          presence of gravity so that the stored bias
 *          represents sensor offset only.
 *
 * @return  Accelerometer calibration data.
 *
 * @note    If no valid samples are collected, a
 *          zero-initialized bias structure is returned.
 */
AccelBias calibrateAccel(void);


/**
 * @brief   Computes gyroscope bias values.
 *
 * @details Repeatedly collects stationary gyroscope
 *          measurements until either:
 *          - CALIBRATION_SAMPLES valid samples have been collected, or
 *          - CALIBRATION_MAX_ATTEMPTS total read attempts have been made.
 *
 *          Only successful MPU6050 reads are accumulated
 *          into the bias estimate. Failed reads are ignored
 *          and do not contribute to the final average.
 *
 *          While the IMU is stationary, any measured
 *          angular velocity is treated as gyroscope bias.
 *
 * @return  Gyroscope calibration data.
 *
 * @note    If no valid samples are collected, a
 *          zero-initialized bias structure is returned.
 */
GyroBias calibrateGyro(void);

// -----------------------------------------------------------------------------
// Unit conversion
// -----------------------------------------------------------------------------

/**
 * @brief       Converts raw accelerometer data to g.
 *
 * @param raw:  Raw accelerometer reading.
 * @param bias: Accelerometer bias.
 *
 * @retval      Acceleration in g.
 */
float rawAccelToG(int16_t raw, float bias);

/**
 * @brief		Converts acceleration from g to m/s².
 *
 * @param g:	Acceleration in units of g.
 *
 * @retval		Acceleration in meters per second squared.
 */
float gToMetersPerSecondSquared(float g);

/**
 * @brief       Converts raw gyroscope data to degrees/second.
 *
 * @param raw:  Raw gyroscope reading.
 * @param bias: Gyroscope bias.
 *
 * @retval      Angular velocity in degrees per second.
 */
float rawGyroToDPS(int16_t raw, float bias);

#endif /* CALIBRATION_H */
