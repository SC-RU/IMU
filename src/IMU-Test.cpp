#include <Wire.h>

void setup()
{
  Serial.begin(115200); // Begin serial communication with a 115200 bps baud rate
  Wire.begin(); // Begin I2C communication
  while(!Serial); // Wait until connection is secured

  // confAdd(); // Debug point: Confirm the devices address to be 0x68
  wakeUpIMU(); // Wake up the IMU
  setAccelFS(); // Default accelerometer sens. to +- 2g
  setGyroFS(); // Default gyroscope sens. to +- 250 deg/s
  redNoise(); // Reduce noise
  // readAccelConfig(); // Debug point: Is ACCEL_CONFIG being defaulted to +-2g or not? (Should be 0x0)
  // readGyroConfig(); // Debug point: Is GYRO_CONFIG being defaulted to +- 250 deg/s or not? (Should be 0x0)
  calibrateAccel(); // Calibrate the accelerometer
  calibrateGyro(); // calibrate the gyroscope

  Serial.println("MPU6050 IMU awake.\n");
}

// void confAdd() // Confirm the devices address
// {
//   Wire.beginTransmission(0x68); // Start talking to IMU
//   Wire.write(0x75); // Write to its WHO_AM_I register (0x75)
//   Wire.endTransmission(false); // Send a repeated START condition

//   Wire.requestFrom(0x68, 1); // Request to read the WHO_AM_I data (device address)
//   byte add = Wire.read(); // Save the device's address to a variable

//   // Print the device's address
//   Serial.print("WHO_AM_I = 0x");
//   Serial.println(add, HEX);
// }

void wakeUpIMU() // Wakes up the IMU (plus a few more things)
{
  // Wake up IMU on default settings - enables gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(10); // Let clock stabilize

  Wire.beginTransmission(0x68); // Begin talking to IMU again
  Wire.write(0x6C); // Write to its PWR_MGMT_2 register (0x6C)
  Wire.write(0x00); // Enable all accelerometer and gyroscope axes (turning bits to 0 makes sure they are not on standby)
  Wire.endTransmission(); // Stop talking to IMU

  delay(10);

  Wire.beginTransmission(0x68); // Begin talking to IMU
  Wire.write(0x6B); // Write to its PWR_MGMT_1 register (0x6B)
  Wire.write(0x01); // Set CLKSEL to 1 (switch clock to PLL with X axis gyroscope reference)
  Wire.endTransmission(); // Stop talking to IMU

  delay(10);
}

void redNoise() // Reduces noise
{
  Wire.beginTransmission(0x68); // Start talking to IMU
  Wire.write(0x1A); // Write to its CONFIG register (0x1A)
  Wire.write(0x03); // Set DLPF to 3 (reduces noise)
  Wire.endTransmission(); // Stop talking to IMU

  Wire.beginTransmission(0x68); // Start talking to IMU again
  Wire.write(0x19); // Write to its SMPRT_DIV register (0x19)
  Wire.write(0x09); // Set SMPLRT_DIV to 9 (sets Sample Rate to 100 Hz)
  Wire.endTransmission(); // Stop talking to IMU
}

void setAccelFS() // Sets the accelerometer's full scale range (sensitivity) to +- 2g
{
  Wire.beginTransmission(0x68); // Begin talking to IMU
  Wire.write(0x1C); // Write to its ACCEL_CONFIG register (0x1C)
  Wire.write(0x00); // Turn it to 0 (default to +- 2g)
  Wire.endTransmission(); // Stop talking to IMU
}

void setGyroFS() // Sets the gyroscope's full-scale range (sensitivity) to +-250 deg/s
{
  Wire.beginTransmission(0x68); // Begin talking to IMU
  Wire.write(0x1B); // Write to its GYRO_CONFIG register (0x1B)
  Wire.write(0x00); // Set FS_SEL bits to 0 (+-250 deg/s full-scale range)
  Wire.endTransmission(); // Stop talking to IMU
}

// void readAccelConfig() // Reads the device's accelerometer configuration
// {
  // Wire.beginTransmission(0x68); // Start talking to IMU
  // Wire.write(0x1C); // Write to its ACCEL_CONFIG register (0x1C)
  // Wire.endTransmission(false); // Send a repeated START condition

  // Wire.requestFrom(0x68, 1); // Request to read its AFS_SEL data
  // byte afs = Wire.read(); // Save AFS_SEL data to variable

  // // Print the accelerometer's full scale data (0x00 means a +- 2g range)
  // Serial.print("ACCEL_CONFIG = 0x");
  // Serial.println(afs, HEX);
  // Serial.println();
// }

// void readGyroConfig() // Reads the device's gyroscope configuration
// {
//   Wire.beginTransmission(0x68); // Start talking to IMU
//   Wire.write(0x1B); // Write to its GYRO_CONFIG register (0x1B)
//   Wire.endTransmission(false); // Send a repeated START condition

//   Wire.requestFrom(0x68, 1); // Request to read its FS_SEL data
//   byte fs = Wire.read(); // Save FS_SEL data to variable

//   // Print the gyroscope's full-scale data (0x00 means a +- 250 deg/s range)
//   Serial.print("GYRO_CONFIG = 0x");
//   Serial.println(fs, HEX);
//   Serial.println();
// }

// Global variables to track the accelerometer's biases for ax, ay, and az
float axBias = 0, ayBias = 0, azBias = 0;
void calibrateAccel() // Calibrates the accelerometer
{
  const int N = 500;
  long sx = 0, sy = 0, sz = 0; // Numbers to represent the sum of the readings

  int16_t ax, ay, az;

  // Take a sampling of 500 readings of the raw ax, ay, and az values and sum them up
  for (int i = 0; i < N; i++)
  {
    readRawAccel(ax, ay, az);
    sx += ax;
    sy += ay;
    sz += az;
    delay(5);
  }

  // Average the sums
  float axAvg = sx / (float)N;
  float ayAvg = sy / (float)N;
  float azAvg = sz / (float)N;

  // This average represents their bias (Offset in the data readings)
  axBias = axAvg;
  ayBias = ayAvg;
  azBias = azAvg - 16384.0; // The bias means that the accelerometer is azAvg away from 0. When accounting for this bias you calculate (actual reading) - (bias), which should = 0. Thus, by subtracting 16384.0 (which is 1g), you essentially "force" the accelerometer's base reading of az to be +1g (which is what it should be)

  // Print the accelerometer's biases
  // Serial.print("axBias: ");
  // Serial.println(axBias);
  
  // Serial.print("ayBias: ");
  // Serial.println(ayBias);
  
  // Serial.print("azBias: ");
  // Serial.println(azBias);
  // Serial.println();
}

// Global variables to track the gyroscope's biases for gx, gy, and gz
float gxBias = 0, gyBias = 0, gzBias = 0;
void calibrateGyro() // calibrates the gyroscope
{
  const int N = 500;
  long sx = 0, sy = 0, sz = 0; // Numbers to represent the sum of the readings

  int16_t gx, gy, gz;

  // Take a sampling of 500 readings of the raw gx, gy, and gz values and sum them up
  for (int i = 0; i < N; i++)
  {
    readRawGyro(gx, gy, gz);
    sx += gx;
    sy += gy;
    sz += gz;
    delay(5);
  }

  // The average of these readings represents their bias
  gxBias = sx / (float)N;
  gyBias = sy / (float)N;
  gzBias = sz / (float)N;

  // Print the accelerometer's biases
  Serial.print("gxBias: ");
  Serial.println(gxBias);
  
  Serial.print("gyBias: ");
  Serial.println(gyBias);
  
  Serial.print("gzBias: ");
  Serial.println(gzBias);
  Serial.println();
}

void readRawAccel(int16_t &ax, int16_t &ay, int16_t &az) // Gets the raw accelerometer data
{
  Wire.beginTransmission(0x68); // Start talking to IMU
  Wire.write(0x3B); // Write to its ACCEL_XOUT_H register (0x3B) (ax high byte)
  Wire.endTransmission(false); // Keep in the same bus (repeated START condition)

  Wire.requestFrom(0x68, 6); // Request 6 bytes of data (accelerometer data)

  // Check if the 6 bytes are available. If not, set ax, ay, and az to 0 and skip this reading
  if (Wire.available() < 6)
  {
    ax = ay = az = 0;
    return;
  }

  ax = Wire.read() << 8 | Wire.read(); // Read first byte, shift up by 1 byte, and then read 2nd byte. This is the 16-bit data of ax
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
}

void readRawGyro(int16_t &gx, int16_t &gy, int16_t &gz) // Gets the raw gyroscope data
{
  Wire.beginTransmission(0x68); // Start talking to IMU
  Wire.write(0x43); // Write to its GYRO_XOUT_H register (0x43) (gx high byte)
  Wire.endTransmission(false); // Keep in the same bus (repeated START condition)

  Wire.requestFrom(0x68, 6); // Request 6 bytes of data (gyroscope data)

  // Check if the 6 bytes are available. If not, set ax, ay, and az to 0 and skip this reading
  if (Wire.available() < 6)
  {
    gx = gy = gz = 0;
    return;
  }

  gx = Wire.read() << 8 | Wire.read(); // Read first byte, shift up by 1 byte, and then read 2nd byte. This is the 16-bit data of ax
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

float rawAToG(int16_t a, float bias)
{
  return (a - bias) / 16384.0; // LSB sensitivity (+- 2g range sens.) is 16384 LSB/g (not grams)
}

float rawGToDPS(int16_t g, float bias)
{
  return (g - bias) / 131.0; // LSB sensitivity (+- 250 dps range sens.) is 131 LSB/dps
}

float aGToMs2(float aG)
{
  const float GRAV = 9.80665;
  
  float mS2 = aG * GRAV;
  return mS2; 
}

bool hasRan = false;

void loop()
{
  // if (hasRan)
  // {
  //   return;
  // }
  
  int16_t ax, ay, az; // Initialize 16-bit signed integers for ax, ay, and az

  readRawAccel(ax, ay, az); // Read the raw accelerometer data and save it in the respective variables (ax, ay, and az)

  // Print ax, ay, and az raw values
  Serial.print("AX: ");
  Serial.println(ax);

  Serial.print("AY: ");
  Serial.println(ay);

  Serial.print("AZ: ");
  Serial.println(az);

  Serial.println();

  // Convert raw values into g's, taking into account the accelerometer's biases, and save in respective variables
  float axG = rawAToG(ax, axBias);
  float ayG = rawAToG(ay, ayBias);
  float azG = rawAToG(az, azBias);

  // Print ax, ay, and az in g's
  Serial.print("AX (g): ");
  Serial.println(axG, 3);

  Serial.print("AY (g): ");
  Serial.println(ayG, 3);

  Serial.print("AZ (g): ");
  Serial.println(azG, 3);

  Serial.println();

  // Convert g values into m/s^2
  float ax_ms2 = aGToMs2(axG);
  float ay_ms2 = aGToMs2(ayG);
  float az_ms2 = aGToMs2(azG);

  // Print ax, ay, and az in m/s^2
  Serial.print("AX (m/s^2): ");
  Serial.println(ax_ms2, 3);

  Serial.print("AY (m/s^2): ");
  Serial.println(ay_ms2, 3);

  Serial.print("AZ (m/s^2): ");
  Serial.println(az_ms2, 3);

  Serial.println();

  int16_t gx, gy, gz; // Initialize 16-bit signed integers for gx, gy, and gz

  readRawGyro(gx, gy, gz); // Read the raw gyroscope data and save it in the respective variables (gx, gy, and gz)

  // Print gx, gy, and gz raw values
  Serial.print("GX: ");
  Serial.println(gx);

  Serial.print("GY: ");
  Serial.println(gy);

  Serial.print("GZ: ");
  Serial.println(gz);

  Serial.println();

  // Convert raw values into dps, taking into account the gyroscope's biases, and save in respective variables
  float gxDPS = rawGToDPS(gx, gxBias);
  float gyDPS = rawGToDPS(gy, gyBias);
  float gzDPS = rawGToDPS(gz, gzBias);

  // Print ax, ay, and az in g's
  Serial.print("GX (dps): ");
  Serial.println(gxDPS, 3);

  Serial.print("GY (dps): ");
  Serial.println(gyDPS, 3);

  Serial.print("GZ (dps): ");
  Serial.println(gzDPS, 3);

  Serial.println();

  delay(200);
  
  // hasRan = true;
}