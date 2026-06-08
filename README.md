# MPU6050 IMU Bring-Up, Calibration, and Data Acquisition

## Overview

This project focuses on bringing up and validating an MPU6050 inertial measurement unit (IMU) using an Arduino Nano Every. The goal was to communicate with the sensor at the register level, configure its operating modes, calibrate both the accelerometer and gyroscope, and convert raw sensor readings into meaningful physical units.

Rather than relying on third-party MPU6050 libraries, all sensor communication, configuration, and data acquisition were implemented directly through I2C register reads and writes using the MPU6050 datasheet and register map.

Current capabilities include:

* Register-level MPU6050 configuration
* Accelerometer calibration
* Gyroscope calibration
* Raw sensor data acquisition
* Conversion of accelerometer readings to g and m/s²
* Conversion of gyroscope readings to degrees per second (dps)
* Noise reduction using the MPU6050 digital low-pass filter (DLPF)
* Verification of sensor stability and bias correction

---

## Hardware

### Components

* Arduino Nano Every
* MPU6050 IMU Module
* Breadboard
* Jumper Wires

### Wiring

| MPU6050 | Arduino Nano Every |
| ------- | ------------------ |
| VCC     | 3V                 |
| GND     | GND                |
| SDA     | SDA                |
| SCL     | SCL                |

---

## Features

### Accelerometer

The accelerometer is configured for a ±2 g measurement range.

During startup, the system:

1. Collects 500 stationary samples.
2. Computes average offsets for each axis.
3. Accounts for gravity on the Z-axis.
4. Stores calibration biases.
5. Converts readings into:

   * g
   * m/s²

Example output:

AX: 112  
AY: -48  
AZ: 16421  

AX (g): 0.007  
AY (g): -0.003  
AZ (g): 1.002  

AX (m/s²): 0.069  
AY (m/s²): -0.029  
AZ (m/s²): 9.826  

---

### Gyroscope

The gyroscope is configured for a ±250°/s measurement range.

During startup, the system:

1. Collects 500 stationary samples.
2. Computes average bias values.
3. Stores the offsets.
4. Converts readings into degrees per second.

Example output:

gxBias: 41.16  
gyBias: 207.01  
gzBias: -197.99  

GX: 48  
GY: 205  
GZ: -204  

GX (dps): 0.052  
GY (dps): -0.015  
GZ (dps): -0.046  

Values close to zero while stationary indicate successful calibration.

---

## Sensor Configuration

### Accelerometer

| Setting          | Value       |
| ---------------- | ----------- |
| Full Scale Range | ±2 g        |
| Sensitivity      | 16384 LSB/g |

### Gyroscope

| Setting          | Value         |
| ---------------- | ------------- |
| Full Scale Range | ±250 °/s      |
| Sensitivity      | 131 LSB/(°/s) |

### Filtering

| Register              | Value  |
| --------------------- | ------ |
| DLPF_CFG              | 3      |
| Sample Rate Divider   | 9      |
| Effective Sample Rate | 100 Hz |

---

## Lessons Learned

This project provided practical experience with:

* Reading technical datasheets
* Register-level embedded programming
* I2C communication
* Sensor calibration techniques
* Noise reduction and filtering
* Converting raw digital measurements into physical units
* Debugging embedded hardware and software systems

It also served as a foundation for future projects involving sensor fusion, attitude estimation, and embedded flight software.

---

## Future Work

Planned extensions include:

* Roll and pitch estimation
* Real-time orientation tracking
* Complementary filtering
* Sensor fusion
* Kalman filtering
* Attitude estimation
* Migration to STM32 hardware
* FreeRTOS integration

---

## References

MPU6050 Product Specification:
https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

MPU6050 Register Map:
https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

Arduino Wire Library Documentation:
https://www.arduino.cc/reference/en/language/functions/communication/wire/
