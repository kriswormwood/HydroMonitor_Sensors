/**************************************************************************/
/*!
    @file     TSL2591.cpp
    @author   KT0WN (adafruit.com)

    This is a library for the Adafruit TSL2591 breakout board
    This library works with the Adafruit TSL2591 breakout
    ----> https://www.adafruit.com/products/1980

    Check out the links above for our tutorials and wiring diagrams
    These chips use I2C to communicate

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2014 Adafruit Industries
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

/******************************************************************************
Modified and stripped down by City Hydroponics for use with HydroMonitor.
******************************************************************************/

#include "TSL2591.h"

/*
 * The constructor.
 * Sets default values.
 */
TSL2591::TSL2591()
{
  initialized = false;
  integration = 1;
  gain        = 0;
}

/*
 * Set up the sensor.
 */
boolean TSL2591::begin(void)
{
  uint8_t id = read8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_ID);
  if (id == 0x50 ) {} // found sensor!
  else
    return false;

  initialized = true;

  // Set default integration time and gain.
  setTiming(integration);
  setGain(gain);

  // Note: by default, the device is in power down mode on bootup.
  disable();

  return true;
}


/*
 * Switch on the sensor.
 */
void TSL2591::enable(void)
{
  if (!initialized)
  {
    if (!begin())
    {
      return;
    }
  }

  // Enable the device by setting the control bit to 0x01.
  write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE, TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN | TSL2591_ENABLE_NPIEN);
}

/*
 * Switch off the sensor.
 */
void TSL2591::disable(void)
{
  if (!initialized)
  {
    if (!begin())
    {
      return;
    }
  }

  // Disable the device by setting the control bit to 0x00.
  write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE, TSL2591_ENABLE_POWEROFF);
}

/*
 * Take a lux reading.
 * This function has basic autogain: if lux levels are low, the gain is set to Medium (25x).
 * Higher gains of the sensor are not used.
 */
int32_t TSL2591::readSensor() {

  if (!initialized)
  {
    if (!begin())
    {
      return 0;
    }
  }
  
  int32_t lux;
  for (int8_t i=0; i<5; i++) {
     lux = takeReading();
  
    // Check wether we have a valid reading, if not we get back a huge number.
    if (lux > 0 && lux < 1000000)
      break;
  }

  if (lux < 0 || lux > 1000000)
    return -1;
 
  // Overflow detected! Lower gain & redo reading. 
  if (lux < 0 && gain == 1) {
    setGain(0);
    lux = takeReading();
  }
  
  // Set the gain based on the lux level.
  else if (lux < 500 && gain == 0) {
    setGain(1);
    lux = takeReading();
  }
  
  else if (lux > 1000) {
    gain = 0;
    setGain(gain);
  }
  return lux;
}
  
/*
 * Take the actual reading from the sensor.
 */
uint32_t TSL2591::takeReading() {

  // Enable the device
  enable();

  uint32_t luminosity;

  // Wait x ms for ADC to complete
  for (uint8_t d=0; d<=integration; d++)
  {
    delay(120);
  }

  // Read the values from the two sensor channels.
  uint16_t ch1 = read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW);
  uint16_t ch0 = read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);
  disable();
  
  // Check for overflow conditions first.
  if ((ch0 == 0xFFFF) | (ch1 == 0xFFFF))
  {
    // Signal an overflow.
    return -1;
  }
  float    atime, again;
  float    cpl, lux1, lux2, lux;

  // Note: This algorithm is based on preliminary coefficients
  // provided by AMS and may need to be updated in the future.
  atime = (float)integration * 100.0F;
  if (gain == 0)
    again = 1.0F;
  else
    again = 25.0F;

  // cpl = (ATIME * AGAIN) / DF
  cpl = (atime * again) / TSL2591_LUX_DF;

  // Calculate lux value.
  lux1 = ( (float)ch0 - (TSL2591_LUX_COEFB * (float)ch1) ) / cpl;
  lux2 = ( ( TSL2591_LUX_COEFC * (float)ch0 ) - ( TSL2591_LUX_COEFD * (float)ch1 ) ) / cpl;
  lux = lux1 > lux2 ? lux1 : lux2;

  // Alternate lux calculation
  //lux = ( (float)ch0 - ( 1.7F * (float)ch1 ) ) / cpl;

  // Signal I2C had no errors
  return (uint32_t)lux;
}

/*
 * Set the gain of the sensor.
 */
void TSL2591::setGain(uint8_t g)
{
  if (!initialized)
  {
    if (!begin())
    {
      return;
    }
  }
  enable();
  gain = g;
  write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, integration | (gain << 4));
  disable();
}

/*
 * Set the integration time of the sensor.
 */
void TSL2591::setTiming(uint8_t i)
{
  if (!initialized)
  {
    if (!begin())
    {
      return;
    }
  }

  enable();
  integration = i+1;
  write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, integration | (gain << 4));
  disable();
}

/*
 * Read an 8-bit (1-byte) from the sensor.
 */
uint8_t TSL2591::read8(uint8_t reg)
{
  uint8_t x;

  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(TSL2591_ADDR, 1);
  x = Wire.read();
  // while (! Wire.available());
  // return Wire.read();
  return x;
}

/*
 * Read a 16-bit (2-byte) value from the sensor.
 */
uint16_t TSL2591::read16(uint8_t reg)
{
  uint16_t x;
  uint16_t t;

  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(TSL2591_ADDR, 2);
  t = Wire.read();
  x = Wire.read();
  x <<= 8;
  x |= t;
  return x;
}

/*
 * Write an 8-bit (1-byte) value to a register of the sensor.
 */
void TSL2591::write8 (uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

/*
 * Write a command to the sensor.
 */
void TSL2591::write8 (uint8_t reg)
{
  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
}

