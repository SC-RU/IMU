/******************************************************************************
 * @file   MPU6050.h
 * @brief  Header file for the MPU6050 driver.
 * 
* @details This module provides:
 *          - Register-level I2C communication
 *          - Device initialization
 *          - Sensor configuration
 *          - Connection verification
 *          - Raw accelerometer data acquisition
 *          - Raw gyroscope data acquisition
 *
 *          The driver communicates directly with the MPU6050
 *          without relying on third-party libraries.
 * 
 * @author  Sumedh Camarushi
 * @date    June 10, 2026
 ******************************************************************************/

#ifndef MPU6050_H
#define MPU6050_H

// -----------------------------------------------------------------------------
// Include files
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

// -----------------------------------------------------------------------------
// MPU6050 register definitions
// -----------------------------------------------------------------------------

constexpr uint8_t MPU6050_ADDR      = 0x68;   ///< Default I2C address
constexpr uint8_t WHO_AM_I          = 0x75;   ///< Device identification register
constexpr uint8_t PWR_MGMT_1        = 0x6B;   ///< Primary power management register
constexpr uint8_t PWR_MGMT_2        = 0x6C;   ///< Secondary power management register
constexpr uint8_t ACCEL_CONFIG      = 0x1C;   ///< Accelerometer configuration register
constexpr uint8_t GYRO_CONFIG       = 0x1B;   ///< Gyroscope configuration register
constexpr uint8_t CONFIG_REG        = 0x1A;   ///< Digital low-pass filter configuration
constexpr uint8_t SMPRT_DIV         = 0x19;   ///< Sample-rate divider register
constexpr uint8_t ACCEL_XOUT_H      = 0x3B;   ///< First accelerometer output register
constexpr uint8_t GYRO_XOUT_H       = 0x43;   ///< First gyroscope output register

// -----------------------------------------------------------------------------
// MPU6050 configuration values
// -----------------------------------------------------------------------------

// PWR_MGMT_1
constexpr uint8_t CLOCK_INTERNAL            = 0x00;   ///< Internal oscillator
constexpr uint8_t CLOCK_PLL_XGYRO           = 0x01;   ///< PLL clock (X gyro)

// PWR_MGMT_2
constexpr uint8_t ENABLE_ALL_SENSOR_AXES    = 0x00;   ///< Power management value to enable all axes

// ACCEL_CONFIG
constexpr uint8_t ACCEL_RANGE_2G            = 0x00;   ///< ±2 g range

// GYRO_CONFIG
constexpr uint8_t GYRO_RANGE_250DPS         = 0x00;   ///< ±250 deg/s range

// CONFIG_REG
constexpr uint8_t DLPF_BANDWIDTH_44HZ       = 0x03;   ///< DLPF configuration

// SMPRT_DIV
constexpr uint8_t SAMPLE_RATE_DIV_100HZ     = 0x09;   ///< 100 Hz output rate

// -----------------------------------------------------------------------------
// Driver constants
// -----------------------------------------------------------------------------

constexpr uint8_t AXIS_DATA_SIZE            = 6;      ///< X,Y,Z (2 bytes each)
constexpr uint8_t STARTUP_DELAY_MS          = 10;     ///< Hardware settle delay

// -----------------------------------------------------------------------------
// Sensor data structures
// -----------------------------------------------------------------------------

/**
 * @brief   Raw accelerometer measurement.
 *
 * @details Stores the three accelerometer axes exactly as
 *          reported by the MPU6050 registers before any
 *          calibration or unit conversion.
 */
struct AccelData
{
    int16_t x;   ///< Raw X-axis acceleration
    int16_t y;   ///< Raw Y-axis acceleration
    int16_t z;   ///< Raw Z-axis acceleration
};

/**
 * @brief   Raw gyroscope measurement.
 *
 * @details Stores the three gyroscope axes exactly as
 *          reported by the MPU6050 registers before any
 *          calibration or unit conversion.
 */
struct GyroData
{
    int16_t x;   ///< Raw X-axis angular velocity
    int16_t y;   ///< Raw Y-axis angular velocity
    int16_t z;   ///< Raw Z-axis angular velocity
};

// -----------------------------------------------------------------------------
// Low-level register interface
// -----------------------------------------------------------------------------

/**
 * @brief       Writes a value to an MPU6050 register.
 *
 * @details     Uses the I2C bus to transmit an 8-bit value
 *              to the specified register address.
 *
 * @param reg   Register address.
 * @param value Value to write.
 */
void writeRegister(uint8_t reg, uint8_t value);

/**
 * @brief     Reads a single MPU6050 register.
 *
 * @param reg Register address.
 *
 * @return    Register contents.
 */
uint8_t readRegister(uint8_t reg);

// -----------------------------------------------------------------------------
// Device initialization
// -----------------------------------------------------------------------------

/**
 * @brief   Verifies communication with the MPU6050.
 *
 * @details Reads the WHO_AM_I register and compares
 *          the returned value against the expected
 *          device address.
 *
 * @return  true if device detected, false if device not detected.
 */
bool verifyConnection();

/**
 * @brief   Initializes the MPU6050.
 *
 * @details Performs:
 *          - Device verification
 *          - Wake-up sequence
 *          - Accelerometer configuration
 *          - Gyroscope configuration
 *          - Noise reduction configuration
 */
void initializeMPU6050();

/**
 * @brief   Wakes the MPU6050 from sleep mode.
 *
 * @details Configures the internal clock source and
 *          enables all accelerometer and gyroscope axes.
 */
void wakeIMU();

/**
 * @brief   Configures the accelerometer.
 *
 * @details Sets the full-scale measurement range
 *          to ±2 g.
 */
void setAccelConfig();

/**
 * @brief   Configures the gyroscope.
 *
 * @details Sets the full-scale measurement range
 *          to ±250 degrees per second.
 */
void setGyroConfig();

/**
 * @brief   Configures the MPU6050 digital filters.
 *
 * @details Sets the Digital Low-Pass Filter (DLPF)
 *          and sample-rate divider to reduce
 *          measurement noise.
 */
void reduceNoise();

// -----------------------------------------------------------------------------
// Sensor acquisition
// -----------------------------------------------------------------------------

/**
 * @brief   Reads raw accelerometer measurements.
 *
 * @details Retrieves the current X, Y, and Z
 *          accelerometer register values.
 *
 * @return  Raw accelerometer data.
 */
AccelData readRawAccel();

/**
 * @brief   Reads raw gyroscope measurements.
 *
 * @details Retrieves the current X, Y, and Z
 *          gyroscope register values.
 *
 * @return  Raw gyroscope data.
 */
GyroData readRawGyro();

#endif // MPU6050_H