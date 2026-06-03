#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t MPU6050_ADDR = 0x68;
constexpr uint8_t WHO_AM_I     = 0x75;
constexpr uint8_t PWR_MGMT_1   = 0x6B;
constexpr uint8_t PWR_MGMT_2   = 0x6C;
constexpr uint8_t ACCEL_CONFIG = 0x1C;
constexpr uint8_t GYRO_CONFIG  = 0x1B;
constexpr uint8_t CONFIG_REG      = 0x1A;
constexpr uint8_t SMPRT_DIV       = 0x19;
constexpr uint8_t ACCEL_XOUT_H    = 0x3B;
constexpr uint8_t GYRO_XOUT_H     = 0x43;

struct AccelData
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct GyroData
{
    int16_t x;
    int16_t y;
    int16_t z;
};

void writeRegister(uint8_t reg, uint8_t value); // Helper function to write to a register
uint8_t readRegister(uint8_t reg); // Helper function to read from a register.

bool verifyConnection();

void initializeMPU6050();

void wakeIMU();
void setAccelConfig();
void setGyroConfig();
void reduceNoise();

AccelData readRawAccel();

GyroData readRawGyro();

#endif // MPU6050_H