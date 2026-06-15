# Attitude Estimation

## Overview

This module estimates the orientation of the MPU6050 using both accelerometer and gyroscope measurements.

The current implementation estimates two Euler angles:

* Roll
* Pitch

Yaw is not estimated because the MPU6050 does not contain a magnetometer and gyroscope integration alone would drift over time.

---

## Accelerometer Attitude

When the IMU is stationary, gravity provides a fixed reference vector.

After calibration and conversion to physical units, roll and pitch can be estimated from the accelerometer measurements.

### Roll

```
roll = atan2(-ax, sqrt(ay² + az²))
```

### Pitch

```
pitch = atan2(ay, az)
```

The resulting angles are converted from radians to degrees.

### Advantages

* Does not accumulate drift
* Simple to compute

### Limitations

* Sensitive to vibration
* Sensitive to linear acceleration
* Noisy during rapid motion

---

## Gyroscope Attitude

The gyroscope measures angular velocity in degrees per second.

Orientation is estimated by integrating angular velocity over time.

```
angle = previous_angle + angular_velocity × dt
```

where

* angular_velocity is measured in degrees per second
* dt is the elapsed loop time in seconds

### Advantages

* Smooth output
* Excellent short-term tracking
* Handles rapid motion well

### Limitations

* Small bias errors accumulate over time
* Long-term drift is unavoidable

---

## Loop Timing

The integration interval is computed from the elapsed time between iterations.

```
dt = (currentTime - lastLoopTime) / 1000000.0
```

Microseconds are converted into seconds because gyroscope measurements are expressed in degrees per second.

Accurate timing is essential because integration error directly affects attitude estimation.

---

## Complementary Filter

The accelerometer and gyroscope each have strengths and weaknesses.

A complementary filter combines both estimates.

```
filtered = α × gyro_estimate + (1 - α) × accel_estimate
```

The current implementation uses

```
α = 0.98
```

This means:

* 98% of the estimate comes from the gyroscope
* 2% comes from the accelerometer

The gyroscope provides smooth short-term motion tracking while the accelerometer continuously corrects long-term drift.

After filtering, the corrected estimate becomes the starting point for the next gyroscope integration step.

---

## Current Limitations

The present implementation estimates only roll and pitch.

Future improvements include:

* Quaternion attitude representation
* Madgwick sensor fusion
* Kalman filtering
* Full 3D orientation estimation
* STM32 implementation
* Real-time visualization