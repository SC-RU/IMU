# Calibration Process

## Accelerometer

500 stationary samples are collected.

Biases are computed as:

bias = average(raw readings)

For the Z-axis:

bias_z = average(z) - 16384

because 16384 LSB corresponds to 1 g.

## Gyroscope

500 stationary samples are collected.

Biases are computed as:

bias = average(raw readings)

Expected output after calibration is near 0 dps.

## Output Conversion

Accelerometer:

raw -> g

Gyroscope:

raw -> dps (degrees per second)