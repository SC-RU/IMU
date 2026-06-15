# Software Architecture

## Data Flow

```
+-----------+
|  MPU6050  |
+-----------+
      |
      v
+-------------+
| Calibration |
+-------------+
      |
      v
+------------+
|  Attitude  |
| Estimation |
+------------+
      |
      v
+---------------+
| Complementary |
|    Filter     |
+---------------+
      |
      v
+-----------+
| Telemetry |
+-----------+
```

Sensor data flows from the MPU6050 module through calibration and attitude estimation before being fused by the complementary filter and transmitted through the telemetry interface.

The project is divided into three modules:

## MPU6050

Responsible for:

- Register-level communication
- Device initialization
- Sensor configuration
- Raw data acquisition

Files:

- MPU6050.h
- MPU6050.cpp

## Calibration

Responsible for:

- Accelerometer bias estimation
- Gyroscope bias estimation
- Unit conversion

Files:

- Calibration.h
- Calibration.cpp

## Attitude

Responsible for:

- Accelerometer attitude estimation
- Gyroscope integration
- Complementary filtering
- Initial attitude estimation

Files:

- Attitude.h
- Attitude.cpp

## Main Application

Responsible for:

- System startup
- Calibration execution
- Sensor polling
- Data output

Files:

- main.cpp