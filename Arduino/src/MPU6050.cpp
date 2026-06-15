/******************************************************************************
 * @file    MPU6050.cpp
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
 * @date    June 10, 2026
 ******************************************************************************/

#include "MPU6050.h"

// -----------------------------------------------------------------------------
// Low-level register interface
// -----------------------------------------------------------------------------

void writeRegister(uint8_t reg, uint8_t value)
{
    // Send a register address followed by the
    // value that should be written to it.

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);

    // Use a repeated START condition so that the bus
    // remains under control of the current master
    // while switching from write mode to read mode.

    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, (uint8_t) 1);

    if (Wire.available())
    {
        return Wire.read();
    }

    // Return zero if the transaction failed.
    return 0;
}

// -----------------------------------------------------------------------------
// Device initialization
// -----------------------------------------------------------------------------

bool verifyConnection()
{
    // The WHO_AM_I register should contain
    // the MPU6050 I2C address.

    return readRegister(WHO_AM_I) == MPU6050_ADDR;
}

void initializeMPU6050()
{
    // Verify that the sensor is responding before
    // attempting any configuration.

    if (!verifyConnection())
    {
        Serial.println("MPU6050 not detected.");

        // Halt execution because the remainder
        // of the application depends on the IMU.

        while (true)
        {
        }
    }

    // Configure the device.

    wakeIMU();
    setAccelConfig();
    setGyroConfig();
    reduceNoise();
}

void wakeIMU()
{
    // The MPU6050 powers up in sleep mode.
    // This sequence:
    //
    // 1. Wakes the device.
    // 2. Enables all sensor axes.
    // 3. Selects the PLL clock source.
    //
    // Short delays allow the internal clock
    // to stabilize between configuration steps.

    writeRegister(PWR_MGMT_1, CLOCK_INTERNAL);

    delay(STARTUP_DELAY_MS);

    writeRegister(PWR_MGMT_2, ENABLE_ALL_SENSOR_AXES);

    delay(STARTUP_DELAY_MS);

    writeRegister(PWR_MGMT_1, CLOCK_PLL_XGYRO);

    delay(STARTUP_DELAY_MS);
}

// -----------------------------------------------------------------------------
// Sensor configuration & noise reduction
// -----------------------------------------------------------------------------

void setAccelConfig()
{
    // Configure the accelerometer for
    // a ±2 g full-scale range.
    writeRegister(ACCEL_CONFIG, ACCEL_RANGE_2G);
}

void setGyroConfig()
{
    // Configure the gyroscope for
    // a ±250 degrees/second full-scale range.
    writeRegister(GYRO_CONFIG, GYRO_RANGE_250DPS);
}

void reduceNoise()
{
    // Configure the Digital Low-Pass Filter (DLPF)
    // to reduce measurement noise and high-frequency
    // vibration.
    //
    // Configure the sample-rate divider so that
    // the effective output data rate is 100 Hz.

    writeRegister(CONFIG_REG, DLPF_BANDWIDTH_44HZ);
    writeRegister(SMPRT_DIV, SAMPLE_RATE_DIV_100HZ);
}

// -----------------------------------------------------------------------------
// Sensor acquisition
// -----------------------------------------------------------------------------

AccelData readRawAccel()
{
    AccelData accel = {};

    // Read six consecutive bytes beginning at the
    // first accelerometer output register.

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);

    // Request all three sensor axes (2 bytes each).
    Wire.requestFrom(MPU6050_ADDR, AXIS_DATA_SIZE);

    if (Wire.available() < AXIS_DATA_SIZE)
    {
        // Return a zero-initialized structure if
        // the I2C transaction was incomplete.
        
        return accel;
    }

    // Combine the high and low bytes into
    // signed 16-bit measurements.

    accel.x = Wire.read() << 8 | Wire.read();
    accel.y = Wire.read() << 8 | Wire.read();
    accel.z = Wire.read() << 8 | Wire.read();

    return accel;
}

GyroData readRawGyro()
{
    GyroData gyro = {};

    // Read six consecutive bytes beginning at the
    // first gyroscope output register.

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);

    // Request all three sensor axes (2 bytes each).
    Wire.requestFrom(MPU6050_ADDR, AXIS_DATA_SIZE);

    if (Wire.available() < AXIS_DATA_SIZE)
    {
        // Return a zero-initialized structure if
        // the I2C transaction was incomplete.
        
        return gyro;
    }

    // Combine the high and low bytes into
    // signed 16-bit measurements.

    gyro.x = Wire.read() << 8 | Wire.read();
    gyro.y = Wire.read() << 8 | Wire.read();
    gyro.z = Wire.read() << 8 | Wire.read();

    return gyro;
}