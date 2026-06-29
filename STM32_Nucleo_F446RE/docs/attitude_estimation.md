# Attitude Estimation

## Overview

This module estimates the orientation of the MPU6050 using accelerometer and gyroscope measurements.

The current implementation estimates two Euler angles:

- Roll
- Pitch

Yaw is not estimated in the current implementation. The MPU6050 provides a 3-axis accelerometer and 3-axis gyroscope, but it does not include an onboard magnetometer, so yaw cannot be corrected against a fixed heading reference and would drift if estimated from gyroscope integration alone.

---

## Estimation Strategy

The attitude estimator combines two sources of information:

- **Accelerometer data**, which provides a gravity-based estimate of roll and pitch
- **Gyroscope data**, which provides short-term angular rate information

Each sensor has different strengths and weaknesses, so the final estimate is produced using a complementary filter.

---

## Accelerometer Estimate

When the IMU is stationary or moving slowly, gravity provides a reference vector that can be used to estimate orientation.

After calibration and conversion to physical units, roll and pitch are computed from the accelerometer measurements.

### Roll

```text
roll = atan2(-ax, sqrt(ay² + az²))
```

### Pitch

```text
pitch = atan2(ay, az)
```

The resulting angles are converted from radians to degrees.

### Strengths

- Does not accumulate drift
- Simple to compute
- Provides a long-term reference for roll and pitch

### Limitations

- Sensitive to vibration
- Sensitive to linear acceleration
- Noisy during rapid motion

---

## Gyroscope Estimate

The gyroscope measures angular velocity in degrees per second.

Orientation is estimated by integrating angular velocity over time:

```text
angle = previous_angle + angular_velocity × dt
```

where:

- `angular_velocity` is measured in degrees per second
- `dt` is the elapsed loop time in seconds

### Strengths

- Smooth output
- Good short-term motion tracking
- Responds well to rapid motion

### Limitations

- Small bias errors accumulate over time
- Long-term drift is unavoidable

---

## Loop Timing

Accurate loop timing is essential because gyroscope integration depends directly on the value of `dt`.

In this project, `dt` is computed from the elapsed time between iterations using `HAL_GetTick()`. Since `HAL_GetTick()` returns elapsed time in milliseconds, the difference between successive timestamps is divided by 1000.0 to convert the interval into seconds.

A generic form is:

```text
dt = (currentTime - lastLoopTime) / 1000.0
```

If the timing interval is inconsistent or scaled incorrectly, the gyroscope estimate will accumulate error even when the angular-rate measurements are correct.

---

## Complementary Filter

The accelerometer provides long-term stability, while the gyroscope provides smooth short-term tracking.

A complementary filter combines both estimates:

```text
filtered = α × gyro_estimate + (1 - α) × accel_estimate
```

The current implementation uses:

```text
α = 0.98
```

This means:

- 98% of the estimate comes from the gyroscope
- 2% comes from the accelerometer

The filtered result is then used as the starting point for the next gyroscope integration step. This allows the gyroscope to dominate short-term motion while the accelerometer continuously corrects long-term drift.

---

## Current Scope

The current implementation estimates roll and pitch only.

The estimator is designed as a lightweight attitude solution for the STM32 Nucleo-F446RE platform and serves as a foundation for future improvements such as:

- Real-time visualization
- Higher-rate sampling and timing refinement
- Quaternion attitude representation
- Madgwick or Mahony sensor fusion
- Kalman filtering
- Full 3D orientation estimation

---

## References

- [MPU-6000 and MPU-6050 Product Specification](https://www.invensense.tdk.com/en-us/search-result?query=PS-MPU-6000A-00+%E2%80%93+MPU-6000+and+MPU-6050+Datasheet)
- [UM1725 - Description of STM32F4 HAL and low-layer drivers](https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)
- [HAL I2C APIs - STMicroelectronics](https://dev.st.com/stm32cube-docs/stm32u5-hal2/2.0.0-beta.1.1/docs/drivers/hal_drivers/i2c/hal_i2c_apis.html)
- [STM32F4xx HAL Control Functions](https://mikrocontroller.ti.bfh.ch/halDoc/group__HAL__Exported__Functions__Group2.html)
- [HAL TIM How to Use - STMicroelectronics](https://dev.st.com/stm32cube-docs/stm32u5-hal2/2.0.0-beta.1.1/docs/drivers/hal_drivers/tim/hal_tim_how_to_use.html)