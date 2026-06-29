# MPU6050 Attitude Estimator

## Overview

This project implements register-level MPU6050 bring-up, calibration, sensor data acquisition, and roll/pitch attitude estimation.

The repository currently covers two versions of the same project:

- An Arduino Nano Every version used for initial bring-up, calibration, validation, and attitude-estimation development
- An STM32 version used to port the same pipeline to a more industry-standard embedded platform with improved timing control and a cleaner path toward higher-rate sampling and RTOS-based expansion

Rather than relying on third-party MPU6050 libraries, all sensor communication, configuration, calibration, and attitude estimation are implemented from scratch using direct I2C register access and the MPU6050 datasheet.

Current capabilities include:

- Register-level MPU6050 configuration
- Accelerometer calibration
- Gyroscope calibration
- Raw sensor data acquisition
- Physical unit conversion
- Roll and pitch estimation
- Gyroscope attitude integration
- Complementary filtering
- Fixed-rate sampling and timing control
- Modular software organization

***

## Project Status

Active development.

***

## Hardware

### Arduino Version

- Arduino Nano Every
- MPU6050 IMU Module
- Breadboard
- Jumper Wires

### STM32 Version

- STM32 Development Board
- MPU6050 IMU Module
- Breadboard
- Jumper Wires

### Wiring

| MPU6050 | Arduino Nano Every | STM32 Version |
| ------- | ------------------ | ------------- |
| VCC     | 3.3V               | 3.3V          |
| GND     | GND                | GND           |
| SDA     | SDA                | I2C SDA       |
| SCL     | SCL                | I2C SCL       |

***

## Features

### Accelerometer

The accelerometer is configured for a ±2 g measurement range.

During startup, the system:

1. Collects 500 stationary samples.
2. Computes average offsets for each axis.
3. Accounts for gravity on the Z-axis.
4. Stores calibration biases.
5. Converts readings into g and m/s².

***

### Gyroscope

The gyroscope is configured for a ±250 °/s measurement range.

During startup, the system:

1. Collects 500 stationary samples.
2. Computes average bias values.
3. Stores the offsets.
4. Converts readings into degrees per second.

Values close to zero while stationary indicate successful calibration.

***

## Attitude Estimation

Roll and pitch are estimated from accelerometer measurements using trigonometric relationships.

A gyroscope attitude estimate is maintained by integrating angular velocity over time.

A complementary filter combines both estimates:

```text
Filtered Angle = α × Gyroscope Estimate + (1 - α) × Accelerometer Estimate
```

This provides:

- Short-term stability from the gyroscope
- Long-term drift correction from the accelerometer

***

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

***

## Telemetry Output

The current firmware outputs one compact CSV telemetry line per sample over the serial/UART interface. This format is intended for logging, plotting, and host-side parsing.

### CSV Format

```text
time_MS,accel.roll,accel.pitch,gyro.roll,gyro.pitch,filtered.roll,filtered.pitch
```

Each field represents:

1. `time_MS` — loop timestamp in milliseconds
2. `accel.roll` — roll estimate from the accelerometer
3. `accel.pitch` — pitch estimate from the accelerometer
4. `gyro.roll` — roll estimate from gyroscope integration only
5. `gyro.pitch` — pitch estimate from gyroscope integration only
6. `filtered.roll` — complementary-filtered roll estimate
7. `filtered.pitch` — complementary-filtered pitch estimate

### Example Output

```text
1250,0.842,-1.316,0.905,-1.271,0.904,-1.272
1260,0.851,-1.308,0.914,-1.262,0.913,-1.263
1270,0.847,-1.311,0.923,-1.254,0.921,-1.255
```

***

## Building and Running

### Arduino Version

1. Open the `Arduino/` project in PlatformIO.
2. Build and upload the firmware to the Arduino Nano Every.
3. Open the serial monitor to view the telemetry stream.

### STM32 Version

1. Open the `STM32_Nucleo_F446RE/` project in your STM32 development environment.
2. Build and flash the firmware to the STM32 board.
3. Open the configured serial/UART output to view the telemetry stream.

***

## Repository Layout

```text
Arduino/
  docs/
    architecture.md
    attitude_estimation.md
    calibration.md
    roadmap.md
    testing.md

  include/
    Attitude.h
    Calibration.h
    MPU6050.h

  src/
    Attitude.cpp
    Calibration.cpp
    main.cpp
    MPU6050.cpp

  .gitignore
  platformio.ini

STM32_Nucleo_F446RE/
  Core/
    Inc/
      attitude.h
      calibration.h
      main.h
      mpu6050.h
      stm32f4xx_hal_conf.h
      stm32f4xx_it.h

    Src/
      attitude.c
      calibration.c
      main.c
      mpu6050.c
      stm32f4xx_hal_msp.c
      stm32f4xx_it.c
      syscalls.c
      sysmem.c
      system_stm32f4xx.c

.gitignore
README.md
```

***

## Lessons Learned

This project provided practical experience with:

- Reading technical datasheets
- Register-level embedded programming
- I2C communication
- Sensor calibration techniques
- Noise reduction and filtering
- Converting raw digital measurements into physical units
- Debugging embedded hardware and software systems
- Organizing embedded software into reusable modules

It also served as a foundation for future projects involving sensor fusion, attitude estimation, STM32-based embedded development, and real-time control software.

***

## Future Work

### Immediate

- Improved attitude initialization
- Real-time roll and pitch visualization
- Data logging

### Near-term

- Higher-rate sampling and performance optimization
- STM32 refinement and cleanup
- FreeRTOS task-based architecture

### Long-term

- Quaternion attitude representation
- Madgwick or Mahony sensor fusion
- Kalman filtering
- 3D orientation tracking
- Motor control integration

***

## References

MPU6050 Product Specification:  
[https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)

MPU6050 Register Map:  
[https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

Arduino Wire Library Documentation:  
[https://www.arduino.cc/reference/en/language/functions/communication/wire/](https://www.arduino.cc/reference/en/language/functions/communication/wire/)