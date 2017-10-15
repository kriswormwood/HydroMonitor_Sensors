/*
  BMP180.cpp
  Bosch BMP180 pressure sensor library for the Arduino microcontroller
  Mike Grusin, SparkFun Electronics

  Uses floating-point equations from the Weather Station Data Logger project
  http://wmrx00.sourceforge.net/
  http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf

  Forked from BMP085 library by M.Grusin

  version 1.0 2013/09/20 initial version
  Verison 1.1.2 - Updated for Arduino 1.6.4 5/2015

  Our example code uses the "beerware" license. You can do anything
  you like with this code. No really, anything. If you find it useful,
  buy me a (root) beer someday.
*/

/******************************************************************************
Modified by City Hydroponics for use with HydroMonitor.
******************************************************************************/

#include <BMP180.h>
#include <Wire.h>
#include <stdio.h>
#include <math.h>

//The constructor.
BMP180::BMP180()
{
}

// Initialize library for subsequent pressure measurements
char BMP180::begin()
{
  double c3,c4,b1;
  
  // Start up the Arduino's "wire" (I2C) library:
  
  Wire.begin();

  // The BMP180 includes factory calibration data stored on the device.
  // Each device has different numbers, these must be retrieved and
  // used in the calculations when taking pressure measurements.

  // Retrieve calibration data from device:
  
  if (readInt(0xAA, AC1) &&
    readInt(0xAC, AC2) &&
    readInt(0xAE, AC3) &&
    readUInt(0xB0, AC4) &&
    readUInt(0xB2, AC5) &&
    readUInt(0xB4, AC6) &&
    readInt(0xB6, VB1) &&
    readInt(0xB8, VB2) &&
    readInt(0xBA, MB) &&
    readInt(0xBC, MC) &&
    readInt(0xBE, MD))
  {

    // All reads completed successfully!
    // Compute floating-point polynominals:
    c3 = 160.0 * pow(2,-15) * AC3;
    c4 = pow(10,-3) * pow(2,-15) * AC4;
    b1 = pow(160,2) * pow(2,-30) * VB1;
    c5 = (pow(2,-15) / 160) * AC5;
    c6 = AC6;
    mc = (pow(2,11) / pow(160,2)) * MC;
    md = MD / 160.0;
    x0 = AC1;
    x1 = 160.0 * pow(2,-13) * AC2;
    x2 = pow(160,2) * pow(2,-25) * VB2;
    y0 = c4 * pow(2,15);
    y1 = c4 * c3;
    y2 = c4 * b1;
    p0 = (3791.0 - 8.0) / 1600.0;
    p1 = 1.0 - 7357.0 * pow(2,-20);
    p2 = 3038.0 * 100.0 * pow(2,-36);

    // Success!
    return(1);
  }
  else
  {
    // Error reading calibration data; bad component or connection?
    return(0);
  }
}

// Read a signed integer (two bytes) from device.
// address: register to start reading (plus subsequent register).
// value: external variable to store data (function modifies value).
char BMP180::readInt(char address, int16_t &value)
{
  unsigned char data[2];
  data[0] = address;
  if (readBytes(data,2))
  {
    value = (int16_t)((data[0] << 8) | data[1]);
    return 1;
  }
  value = 0;
  return 0;
}

// Read an unsigned integer (two bytes) from device.
// address: register to start reading (plus subsequent register).
// value: external variable to store data (function modifies value).
char BMP180::readUInt(char address, uint16_t &value)
{
  unsigned char data[2];
  data[0] = address;
  if (readBytes(data,2))
  {
    value = (((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    return 1;
  }
  value = 0;
  return 0;
}

// Read an array of bytes from device.
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read.
char BMP180::readBytes(unsigned char *values, char length)
{
  char x;
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(values[0]);
  _error = Wire.endTransmission();
  if (_error == 0)
  {
    Wire.requestFrom(BMP180_ADDR,length);
    while(Wire.available() != length) ; // wait until bytes are ready
    for(x=0;x<length;x++) {
      values[x] = Wire.read();
    }
    return 1;
  }
  return 0;
}

// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
char BMP180::writeBytes(unsigned char *values, char length)
{
  char x;
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(values,length);
  _error = Wire.endTransmission();
  if (_error == 0)
    return 1;
  else
    return 0;
}

// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.
float BMP180::readTemperature()
{
  unsigned char data[2], result;
  double tu, a;
  float T;

  // Start the temperature reading.
  data[0] = BMP180_REG_CONTROL;
  data[1] = BMP180_COMMAND_TEMPERATURE;
  result = writeBytes(data, 2);
  if (result == 0) 
    return -1;
  
  // Wait for conversion to complete.
  delay(5);

  // Read the conversion result.
  data[0] = BMP180_REG_RESULT;
  result = readBytes(data, 2);
  if (result) { // good read, calculate temperature
    tu = (data[0] * 256.0) + data[1];
    a = c5 * (tu - c6);
    T = a + (mc / (a + md));
    return T;
  }
  else
    return -1;
}

// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// For HydroMonitor always use 3.

// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.

// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
float BMP180::readPressure(float T)
{
  unsigned char data[3];
  char result;
  double pu, s, x, y, z;
  float P;

  // Start pressure conversion with 3x oversampling.
  data[0] = BMP180_REG_CONTROL;
  data[1] = BMP180_COMMAND_PRESSURE3;

  result = writeBytes(data, 2);
  if (result == 0) // write failed?
    return -1;

  // Wait for conversion to complete.
  delay(26); 
  data[0] = BMP180_REG_RESULT;
  result = readBytes(data, 3);
  if (result) { // good read, calculate pressure
    pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);
    s = T - 25.0;
    x = (x2 * pow(s,2)) + (x1 * s) + x0;
    y = (y2 * pow(s,2)) + (y1 * s) + y0;
    z = (pu - x) / y;
    P = (p2 * pow(z,2)) + (p1 * z) + p0;
    return P;
  }
  else
    return -1;
}


// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
float BMP180::sealevel(float P, float A) {
  return(P/pow(1-(A/44330.0),5.255));
}

// If any library command fails, you can retrieve an extended
// error code using this command. Errors are from the wire library: 
// 0 = Success
// 1 = Data too long to fit in transmit buffer
// 2 = Received NACK on transmit of address
// 3 = Received NACK on transmit of data
// 4 = Other error
char BMP180::getError(void) {
  return(_error);
}

