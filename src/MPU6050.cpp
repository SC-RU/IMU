#include "MPU6050.h"

void writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false); // repeated START

    Wire.requestFrom(MPU6050_ADDR, (uint8_t)1);

    if (Wire.available())
    {
        return Wire.read();
    }

    return 0;
}

bool verifyConnection()
{
    return readRegister(WHO_AM_I) == MPU6050_ADDR;
}

void initializeMPU6050()
{
    if (!verifyConnection())
    {
        Serial.println("MPU6050 not detected.");
        while (true)
        {
        }
    }

    wakeIMU();
    setAccelConfig();
    setGyroConfig();
    reduceNoise();
}

void wakeIMU() // Wakes up the IMU (plus a few more things)
{
    // Wake up IMU on default settings - enables gyroscope
    writeRegister(PWR_MGMT_1, 0x00);

    delay(10); // Let clock stabilize

    writeRegister(PWR_MGMT_2, 0x00); // Enable all accelerometer and gyroscope axes (turning bits to 0 makes sure they are not on standby)

    delay(10);

    writeRegister(PWR_MGMT_1, 0x01); // Set CLKSEL to 1 (switch clock to PLL with X axis gyroscope reference)

    delay(10);
}

void setAccelConfig() // Sets the accelerometer's full scale range (sensitivity) to +- 2g
{
    writeRegister(ACCEL_CONFIG, 0x00); // Turn it to 0 (default to +- 2g)
}

void setGyroConfig() // Sets the gyroscope's full-scale range (sensitivity) to +-250 deg/s
{
    writeRegister(GYRO_CONFIG, 0x00); // Set FS_SEL to 0 (+-250 deg/s full-scale range)
}

void reduceNoise() // Reduces noise
{
    writeRegister(CONFIG_REG, 0x03); // Set DLPF to 3 (reduces noise)
    writeRegister(SMPRT_DIV, 0x09); // Set SMPLRT_DIV to 9 (sets Sample Rate to 100 Hz)
}

AccelData readRawAccel()
{
    AccelData accel = {0, 0, 0};

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, (uint8_t) 6);

    if (Wire.available() < 6)
    {
        return accel;
    }

    accel.x = Wire.read() << 8 | Wire.read();
    accel.y = Wire.read() << 8 | Wire.read();
    accel.z = Wire.read() << 8 | Wire.read();

    return accel;
}

GyroData readRawGyro()
{
    GyroData gyro = {0, 0, 0};

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, (uint8_t) 6);

    if (Wire.available() < 6)
    {
        return gyro;
    }

    gyro.x = Wire.read() << 8 | Wire.read();
    gyro.y = Wire.read() << 8 | Wire.read();
    gyro.z = Wire.read() << 8 | Wire.read();

    return gyro;
}