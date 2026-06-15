# Testing Results

## Connection Test

WHO_AM_I = 0x68

PASS

## Accelerometer

Stationary readings:

AX = 0.00 g
AY = 0.00 g
AZ = 1.00 g

PASS

## Gyroscope

Stationary readings:

GX = 0.02 dps
GY = 0.01 dps
GZ = 0.02 dps

PASS

## Attitude Estimation

### Accelerometer Estimate

Observed behavior:

* Roll changes during side-to-side tilting.
* Pitch changes during forward/backward tilting.
* Estimates return to their original values when the board is level.

PASS

### Gyroscope Integration

Observed behavior:

* Roll and pitch change smoothly during motion.
* Angle estimates continue changing while the sensor rotates.
* Small long-term drift is present, as expected.

PASS

### Complementary Filter

Observed behavior:

* Gyroscope provides smooth short-term motion tracking.
* Accelerometer corrects long-term drift.
* Filtered estimate remains stable while stationary.

PASS