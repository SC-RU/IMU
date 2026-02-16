#include <Wire.h> // I2C Library

void setup()
{
  Serial.begin(115200); // Starts serial communication with 115200 bps (baud rate). This is one of the default baud rates.
  Wire.begin(); // Initializes the Nano Every as the master device in I2C communication, enables I2C hardware, and sets up SCL + SDA pins.
  while(!Serial); // Makes sure the serial connection is ready.
  Serial.println("I2C Scanner\n");
}

bool hasRun = false; // Boolean to see if the code has run.

void loop()
{
  if (hasRun)
  {
    return; // Stops code forever.
  }

  byte address, res; // Bytes to represent all possible addresses and the results of trying to communicate with said address.
  int nDevices = 0; // Number of devices connected.

  for (address = 1; address < 127; address++) // Loops through all possible addresses except 0X00 and 0x7E, as those are reserved.
  {
    Wire.beginTransmission(address); // Sends a START condition to specified slave address.
    res = Wire.endTransmission(); // Saves the result after sending a STOP condition (0 if acknowledged, 1 if not) to res.

    if (res == 0) // If a device is found, print "I2C device found at " + address of device and add 1 to nDevices.
    {
      Serial.print("I2C device found at 0x");
      
      if (address < 16)
      {
        Serial.print("0");
      }
      
      Serial.println(address, HEX);
    }

    nDevices++;
  }

  if (nDevices == 0)
  {
    Serial.println("No I2C devices found.");
  }

  else
  {
    Serial.println("Done.");
  }

  delay(2000);

  hasRun = true;
}
