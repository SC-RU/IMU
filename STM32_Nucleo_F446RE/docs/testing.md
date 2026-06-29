# Testing Results

## Connection Test

The MPU6050 connection was verified by reading the `WHO_AM_I` register.

Observed result:

```text
WHO_AM_I = 0x68
```

Expected result:

```text
0x68
```

Status: PASS

---

## Accelerometer Test

After accelerometer calibration, the IMU was held stationary and level.

Observed readings:

```text
AX (g): 0.000 AY (g): 0.000 AZ (g): 1.000
```

Expected behavior:

- X-axis and Y-axis remain near 0 g
- Z-axis remains near 1 g while level and stationary

Status: PASS

---

## Gyroscope Test

After gyroscope calibration, the IMU was held stationary.

Observed readings:

```text
GX (dps): 0.020 GY (dps): 0.010 GZ (dps): 0.020
```

Expected behavior:

- All axes remain near 0 dps while stationary
- Small residual noise is acceptable

Status: PASS

---

## Attitude Estimation Test

### Accelerometer Estimate

Observed behavior:

- Roll changes during side-to-side tilting
- Pitch changes during forward/backward tilting
- Estimates return near their original values when the board is level again

Status: PASS

### Gyroscope Integration

Observed behavior:

- Roll and pitch change smoothly during motion
- Angle estimates continue changing while the sensor rotates
- Small long-term drift is present, as expected

Status: PASS

### Complementary Filter

Observed behavior:

- Gyroscope contribution provides smooth short-term motion tracking
- Accelerometer contribution corrects long-term drift
- Filtered estimate remains stable while stationary

Status: PASS