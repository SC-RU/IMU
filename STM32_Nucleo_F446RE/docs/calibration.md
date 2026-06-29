# Calibration Process

## Overview

This module estimates sensor bias during startup and converts raw MPU6050 measurements into physical units.

Calibration is performed while the IMU remains stationary. The computed accelerometer and gyroscope biases are then subtracted from future measurements before attitude estimation.

---

## Accelerometer Calibration

The accelerometer calibration routine collects 500 stationary samples during startup.

Bias is computed by averaging the raw sensor measurements along each axis:

```text
bias = average(raw readings)
```

For the X- and Y-axes, the average raw value is stored directly as the sensor bias.

For the Z-axis, gravity must be removed from the average measurement:

```text
bias_z = average(z) - 16384
```

because the MPU6050 is configured for a ±2 g accelerometer range, and in that mode the sensitivity is 16384 LSB/g.

This means the stored Z-axis bias represents only sensor offset, not Earth’s gravitational acceleration.

---

## Gyroscope Calibration

The gyroscope calibration routine also collects 500 stationary samples during startup.

While the IMU is motionless, the ideal gyroscope output is zero angular velocity on all three axes. Any nonzero average reading is treated as sensor bias.

Bias is computed as:

```text
bias = average(raw readings)
```

These average offsets are then subtracted from future gyroscope measurements.

After calibration, stationary gyroscope output should be near 0 dps on all axes, though small residual noise may still remain.

---

## Output Conversion

After calibration, raw measurements are converted into physical units.

### Accelerometer

Accelerometer readings are converted from raw least-significant bits to g using:

```text
g = (raw - bias) / 16384
```

where 16384 LSB/g is the MPU6050 sensitivity at the configured ±2 g full-scale range.

If needed, acceleration can also be converted from g into meters per second squared:

```text
m/s² = g × 9.80665
```

### Gyroscope

Gyroscope readings are converted from raw least-significant bits to degrees per second using:

```text
dps = (raw - bias) / 131
```

where 131 LSB/(°/s) is the MPU6050 sensitivity at the configured ±250 °/s full-scale range.

---

## Assumptions

This calibration process assumes:

- The IMU is stationary during startup
- The board begins in a known resting orientation for accelerometer Z-axis gravity compensation
- The MPU6050 is configured for:
  - ±2 g accelerometer range
  - ±250 °/s gyroscope range

If these sensor ranges are changed later, the calibration conversion constants must also be updated.

---

## Current Scope

The current calibration implementation provides:

- Accelerometer bias estimation
- Gyroscope bias estimation
- Raw-to-g conversion
- Raw-to-dps conversion
- Optional g-to-m/s² conversion

This is sufficient for the current roll and pitch attitude estimator and complementary filter pipeline.

Future improvements may include:

- Calibration result logging and visualization
- Bias stability characterization over longer runs
- Runtime recalibration support
- Temperature-aware bias compensation