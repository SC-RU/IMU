/******************************************************************************
 * @file    mpu6050.c
 * @brief   Implementation of the MPU6050 driver.
 *
 * @details This module performs low-level I2C communication
 *          with the MPU6050 and provides functions for:
 *
 *          - Register reads and writes
 *          - Device verification
 *          - Sensor initialization
 *          - Accelerometer configuration
 *          - Gyroscope configuration
 *          - Noise reduction configuration
 *          - Raw sensor data acquisition
 *
 * @author  Sumedh Camarushi
 * @date    June 24, 2026
 ******************************************************************************/

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------

#include "MPU6050.h"

// -----------------------------------------------------------------------------
// Driver handle
// -----------------------------------------------------------------------------

static I2C_HandleTypeDef *mpu6050_i2c = 0;

// -----------------------------------------------------------------------------
// Low-level register interface
// -----------------------------------------------------------------------------

HAL_StatusTypeDef writeRegister(uint8_t reg, uint8_t value)
{
    // Send a register address followed by the
    // value that should be written to it.

    if (mpu6050_i2c == 0)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(mpu6050_i2c,
                             MPU6050_ADDR_HAL,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             HAL_MAX_DELAY);
}


HAL_StatusTypeDef readRegister(uint8_t reg, uint8_t *value)
{
    // Read a single register from the MPU6050
    // through the configured HAL I2C peripheral.

    if ((mpu6050_i2c == 0) || (value == 0))
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(mpu6050_i2c,
                            MPU6050_ADDR_HAL,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1,
                            HAL_MAX_DELAY);
}


// -----------------------------------------------------------------------------
// Device initialization
// -----------------------------------------------------------------------------

void MPU6050_SetI2C(I2C_HandleTypeDef *hi2c)
{
    // Store the HAL I2C handle so that the driver
    // can communicate with the MPU6050.

    mpu6050_i2c = hi2c;
}


bool verifyConnection(void)
{
    uint8_t whoami = 0;

    // The WHO_AM_I register should contain
    // the MPU6050 I2C address.

    if (readRegister(WHO_AM_I, &whoami) != HAL_OK)
    {
        return false;
    }

    return (whoami == MPU6050_ADDR);
}


bool initializeMPU6050(void)
{
    // Verify that the sensor is responding before
    // attempting any configuration.

    if (!verifyConnection())
    {
        return false;
    }

    // Configure the device.

    if (wakeIMU() != HAL_OK)
    {
        return false;
    }

    if (setAccelConfig() != HAL_OK)
    {
        return false;
    }

    if (setGyroConfig() != HAL_OK)
    {
        return false;
    }

    if (reduceNoise() != HAL_OK)
    {
        return false;
    }

    return true;
}


HAL_StatusTypeDef wakeIMU(void)
{
    HAL_StatusTypeDef status;

    // The MPU6050 powers up in sleep mode.
    // This sequence:
    //
    // 1. Wakes the device.
    // 2. Enables all sensor axes.
    // 3. Selects the PLL clock source.
    //
    // Short delays allow the internal clock
    // to stabilize between configuration steps.

    status = writeRegister(PWR_MGMT_1, CLOCK_INTERNAL);

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(STARTUP_DELAY_MS);

    status = writeRegister(PWR_MGMT_2, ENABLE_ALL_SENSOR_AXES);

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(STARTUP_DELAY_MS);

    status = writeRegister(PWR_MGMT_1, CLOCK_PLL_XGYRO);

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(STARTUP_DELAY_MS);

    return HAL_OK;
}


// -----------------------------------------------------------------------------
// Sensor configuration & noise reduction
// -----------------------------------------------------------------------------

HAL_StatusTypeDef setAccelConfig(void)
{
    // Configure the accelerometer for
    // a ±2 g full-scale range.

    return writeRegister(ACCEL_CONFIG, ACCEL_RANGE_2G);
}


HAL_StatusTypeDef setGyroConfig(void)
{
    // Configure the gyroscope for
    // a ±250 degrees/second full-scale range.

    return writeRegister(GYRO_CONFIG, GYRO_RANGE_250DPS);
}


HAL_StatusTypeDef reduceNoise(void)
{
    HAL_StatusTypeDef status;

    // Configure the Digital Low-Pass Filter (DLPF)
    // to reduce measurement noise and high-frequency
    // vibration.
    //
    // Configure the sample-rate divider so that
    // the effective output data rate is 100 Hz.

    status = writeRegister(CONFIG_REG, DLPF_BANDWIDTH_44HZ);

    if (status != HAL_OK)
    {
        return status;
    }

    status = writeRegister(SMPRT_DIV, SAMPLE_RATE_DIV_100HZ);

    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}


// -----------------------------------------------------------------------------
// Sensor acquisition
// -----------------------------------------------------------------------------

HAL_StatusTypeDef readRawAccel(AccelData *accel)
{
    uint8_t buffer[AXIS_DATA_SIZE];

    // Verify that the output pointer is valid
    // before attempting the I2C transaction.

    if ((mpu6050_i2c == 0) || (accel == 0))
    {
        return HAL_ERROR;
    }

    // Read six consecutive bytes beginning at the
    // first accelerometer output register.

    if (HAL_I2C_Mem_Read(mpu6050_i2c,
                         MPU6050_ADDR_HAL,
                         ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT,
                         buffer,
                         AXIS_DATA_SIZE,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        // Return a zero-initialized structure if
        // the I2C transaction was incomplete.

        accel->x = 0;
        accel->y = 0;
        accel->z = 0;

        return HAL_ERROR;
    }

    // Combine the high and low bytes into
    // signed 16-bit measurements.

    accel->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    accel->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    accel->z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return HAL_OK;
}


HAL_StatusTypeDef readRawGyro(GyroData *gyro)
{
    uint8_t buffer[AXIS_DATA_SIZE];

    // Verify that the output pointer is valid
    // before attempting the I2C transaction.

    if ((mpu6050_i2c == 0) || (gyro == 0))
    {
        return HAL_ERROR;
    }

    // Read six consecutive bytes beginning at the
    // first gyroscope output register.

    if (HAL_I2C_Mem_Read(mpu6050_i2c,
                         MPU6050_ADDR_HAL,
                         GYRO_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT,
                         buffer,
                         AXIS_DATA_SIZE,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        // Return a zero-initialized structure if
        // the I2C transaction was incomplete.

        gyro->x = 0;
        gyro->y = 0;
        gyro->z = 0;

        return HAL_ERROR;
    }

    // Combine the high and low bytes into
    // signed 16-bit measurements.

    gyro->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    gyro->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    gyro->z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return HAL_OK;
}
