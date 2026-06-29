/******************************************************************************
 * @file   mpu6050.h
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
 * @date    June 22, 2026
 ******************************************************************************/

// -----------------------------------------------------------------------------
// Define to prevent recursive inclusion
// -----------------------------------------------------------------------------

#ifndef MPU6050_H
#define MPU6050_H

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

// -----------------------------------------------------------------------------
// MPU6050 register definitions
// -----------------------------------------------------------------------------

#define MPU6050_ADDR             0x68U					///< Default I2C address
#define MPU6050_ADDR_HAL         (MPU6050_ADDR << 1)	///< Default I2C address shifted for HAL
#define WHO_AM_I                 0x75U					///< Device identification register
#define PWR_MGMT_1               0x6BU					///< Primary power management register
#define PWR_MGMT_2               0x6CU					///< Secondary power management register
#define ACCEL_CONFIG             0x1CU					///< Accelerometer configuration register
#define GYRO_CONFIG              0x1BU					///< Gyroscope configuration register
#define CONFIG_REG               0x1AU					///< Digital low-pass filter configuration
#define SMPRT_DIV                0x19U					///< Sample-rate divider register
#define ACCEL_XOUT_H             0x3BU					///< First accelerometer output register
#define GYRO_XOUT_H              0x43U					///< First gyroscope output register

// -----------------------------------------------------------------------------
// MPU6050 configuration values
// -----------------------------------------------------------------------------

// PWR_MGMT_1
#define CLOCK_INTERNAL           0x00U	///< Internal oscillator
#define CLOCK_PLL_XGYRO          0x01U	///< PLL clock (X gyro)

// PWR_MGMT_2
#define ENABLE_ALL_SENSOR_AXES   0x00U	///< Power management value to enable all axes

// ACCEL_CONFIG
#define ACCEL_RANGE_2G           0x00U	///< ±2 g range

// GYRO_CONFIG
#define GYRO_RANGE_250DPS        0x00U	///< ±250 deg/s range

// CONFIG_REG
#define DLPF_BANDWIDTH_44HZ      0x03U	///< DLPF configuration

// SMPRT_DIV
#define SAMPLE_RATE_DIV_100HZ    0x09U	///< 100 Hz output rate

// -----------------------------------------------------------------------------
// Driver constants
// -----------------------------------------------------------------------------

#define AXIS_DATA_SIZE           6U		///< X,Y,Z (2 bytes each)
#define STARTUP_DELAY_MS         10U	///< Hardware settle delay in milliseconds

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
typedef struct
{
	int16_t x;	///< Raw X-axis acceleration
    int16_t y;	///< Raw Y-axis acceleration
    int16_t z;	///< Raw Z-axis acceleration
} AccelData;


/**
 * @brief   Raw gyroscope measurement.
 *
 * @details Stores the three gyroscope axes exactly as
 *          reported by the MPU6050 registers before any
 *          calibration or unit conversion.
 */
typedef struct
{
	int16_t x;	///< Raw X-axis angular velocity
    int16_t y;	///< Raw Y-axis angular velocity
    int16_t z;	///< Raw Z-axis angular velocity
} GyroData;

// -----------------------------------------------------------------------------
// Low-level register interface
// -----------------------------------------------------------------------------

/**
 * @brief           Writes a value to an MPU6050 register.
 *
 * @details         Uses the I2C bus to transmit an 8-bit value
 *                  to the specified register address.
 *
 * @param reg:      Register address.
 * @param value:    Value to write.
 * 
 * @retval			HAL status of the I2C write operation.
 */
HAL_StatusTypeDef writeRegister(uint8_t reg, uint8_t value);

/**
 * @brief           Reads a single MPU6050 register.
 *
 * @param reg:      Register address.
 * @param value:    Pointer to store the read value.
 *
 * @retval          HAL status of the I2C read operation. The read value is stored in the provided pointer.
 */
HAL_StatusTypeDef readRegister(uint8_t reg, uint8_t *value);

// -----------------------------------------------------------------------------
// Device initialization
// -----------------------------------------------------------------------------

/**
 * @brief       Stores the HAL I2C handle for MPU6050.
 *
 * @param hi2c: I2C handle pointer.
 */
void MPU6050_SetI2C(I2C_HandleTypeDef *hi2c);

/**
 * @brief   Verifies communication with the MPU6050.
 *
 * @details Reads the WHO_AM_I register and compares
 *          the returned value against the expected
 *          device address.
 *
 * @retval  true if device detected, false if device not detected.
 */
bool verifyConnection(void);

/**
 * @brief   Initializes the MPU6050.
 *
 * @details Performs:
 *          - Device verification
 *          - Wake-up sequence
 *          - Accelerometer configuration
 *          - Gyroscope configuration
 *          - Noise reduction configuration
 * 
 * @retval  true if initialization successful, false if initialization failed.
 */
bool initializeMPU6050(void);

/**
 * @brief   Wakes the MPU6050 from sleep mode.
 *
 * @details Configures the internal clock source and
 *          enables all accelerometer and gyroscope axes.
 * 
 * @retval  HAL status of the wake-up process.
 */
HAL_StatusTypeDef wakeIMU(void);

/**
 * @brief   Configures the accelerometer.
 *
 * @details Sets the full-scale measurement range
 *          to ±2 g.
 * 
 * @retval  HAL status of the accelerometer configuration.
 */
HAL_StatusTypeDef setAccelConfig(void);

/**
 * @brief   Configures the gyroscope.
 *
 * @details Sets the full-scale measurement range
 *          to ±250 degrees per second.
 * 
 * @retval  HAL status of the gyroscope configuration.
 */
HAL_StatusTypeDef setGyroConfig(void);

/**
 * @brief   Configures the MPU6050 digital filters.
 *
 * @details Sets the Digital Low-Pass Filter (DLPF)
 *          and sample-rate divider to reduce
 *          measurement noise.
 * 
 * @retval  HAL status of the noise reduction configuration.
 */
HAL_StatusTypeDef reduceNoise(void);

// -----------------------------------------------------------------------------
// Sensor acquisition
// -----------------------------------------------------------------------------

/**
 * @brief   Reads raw accelerometer measurements.
 *
 * @details Retrieves the current X, Y, and Z
 *          accelerometer register values.
 *
 * @retval  HAL status of the accelerometer read operation. The read values are stored in the provided pointer.
 */
HAL_StatusTypeDef readRawAccel(AccelData *accel);

/**
 * @brief   Reads raw gyroscope measurements.
 *
 * @details Retrieves the current X, Y, and Z
 *          gyroscope register values.
 *
 * @retval  HAL status of the gyroscope read operation. The read values are stored in the provided pointer.
 */
HAL_StatusTypeDef readRawGyro(GyroData *gyro);

#endif /* MPU6050_H */
