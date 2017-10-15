/* Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library
------------------------------------------------------------
 
Title: Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library

Description: This library provides utilities to communicate with and to
read data from the Measurement Specialties MS5837-30BA pressure/temperature 
sensor.

Authors: Rustom Jehangir, Blue Robotics Inc.
         Adam Šimko, Blue Robotics Inc.

-------------------------------
The MIT License (MIT)

Copyright (c) 2015 Blue Robotics Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/ 

#ifndef MS5837_H
#define MS5837_H

#include "Arduino.h"

#define MS5837_ADDR         0x76  
#define MS5837_RESET        0x1E
#define MS5837_ADC_READ     0x00
#define MS5837_PROM_READ    0xA0
#define MS5837_CONVERT_D1   0x4A
#define MS5837_CONVERT_D2   0x5A
#define MIN_INTERVAL 2000


class MS5837 {
public:
	MS5837();

	bool begin();

	/** Provide the density of the working fluid in kg/m^3. Default is for 
	 * seawater. Should be 997 for freshwater.
	 */
	void setFluidDensity(float);

	/** Pressure returned in mbar.
	 */
	float readPressure(void);

	/** Temperature returned in deg C.
	 */
	float readTemperature(void);

	/** Depth returned in meters (valid for operation in incompressible
	 *  liquids only. Uses density that is set for fresh or seawater.
	 * 
	 */
	float readWaterLevel(float);

private:
	/** The read from I2C takes up to 40 ms, so use sparingly is possible.
	 * There's also a limit of one reading per MIN_INTERVAL ms.
	 */
	void read();

	uint16_t C[8];
	int32_t temperature;
  int32_t pressure;

	uint32_t lastReadTime;

	float fluidDensity;

	uint8_t crc4(uint16_t[]);
};

#endif
