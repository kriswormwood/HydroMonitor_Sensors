/******************************************************************************
SparkFunBME280.cpp
BME280 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/BME280_Breakout

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
//See SparkFunBME280.h for additional topology notes.

#include "BME280.h"
#include "stdint.h"
#include <math.h>

#include "Wire.h"

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//

//Constructor -- Specifies default configuration
BME280::BME280(void)
{
  //***Operation settings*****************************//

  //runMode can be:
  //  0, Sleep mode
  //  1 or 2, Forced mode
  //  3, Normal mode
  settings.runMode = 3; 

  //tStandby can be:
  //  0, 0.5ms
  //  1, 62.5ms
  //  2, 125ms
  //  3, 250ms
  //  4, 500ms
  //  5, 1000ms
  //  6, 10ms
  //  7, 20ms
  settings.tStandby = 0;

  //filter can be off or number of FIR coefficients to use:
  //  0, filter off
  //  1, coefficients = 2
  //  2, coefficients = 4
  //  3, coefficients = 8
  //  4, coefficients = 16
  settings.filter = 0;

  //tempOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  settings.tempOverSample = 3;

  //pressOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  settings.pressOverSample = 3;

  //humidOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  settings.humidOverSample = 3;
  
  lastReadTime = -MIN_INTERVAL;
  temperature = -1;
  pressure = -1;
  humidity = -1;
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to 
//  configure before calling .begin();
//
//****************************************************************************//
uint8_t BME280::begin()
{
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable
  Wire.begin();

	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	
	calibration.dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));

	calibration.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
	calibration.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
	calibration.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
	calibration.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
	calibration.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	calibration.dig_H6 = ((uint8_t)readRegister(BME280_DIG_H6_REG));

	//Set the oversampling control words.
	//config will only be writeable in sleep mode, so first insure that.
	writeRegister(BME280_CTRL_MEAS_REG, 0x00);
	
	//Set the config word
	dataToWrite = (settings.tStandby << 0x5) & 0xE0;
	dataToWrite |= (settings.filter << 0x02) & 0x1C;
	writeRegister(BME280_CONFIG_REG, dataToWrite);
	
	//Set ctrl_hum first, then ctrl_meas to activate ctrl_hum
	dataToWrite = settings.humidOverSample & 0x07; //all other bits can be ignored
	writeRegister(BME280_CTRL_HUMIDITY_REG, dataToWrite);
	
	//set ctrl_meas
	//First, set temp oversampling
	dataToWrite = (settings.tempOverSample << 0x5) & 0xE0;
	//Next, pressure oversampling
	dataToWrite |= (settings.pressOverSample << 0x02) & 0x1C;
	//Last, set mode
	dataToWrite |= (settings.runMode) & 0x03;
	//Load the byte
	writeRegister(BME280_CTRL_MEAS_REG, dataToWrite);
	
	return readRegister(0xD0);
}

//Strictly resets.  Run .begin() afterwards
void BME280::reset(void)
{
	writeRegister(BME280_RST_REG, 0xB6);
	
}

//****************************************************************************//
//
//  Readings Section
//
//****************************************************************************//
float BME280::readPressure(void) {
  read();
  return pressure;
}

float BME280::readTemperature(void) {
  read();
  return temperature;
}

float BME280::readHumidity(void) {
  read();
  return humidity;
}

// Calculate the sealevel pressure out of the measured pressure P in mbar and altitude A in meters.
float BME280::sealevel(float P, float A) {
  return (P / pow(1 - (A / 44330.0) , 5.255));
}


// Read the values from the sensor.
// Readings are limited to one every MIN_INTERVAL ms.
void BME280::read() {

  // Check if sensor was read less than MIN_INTERVAL milliseconds ago. If so, don't read it again
  // at this time.  
  if ((millis() - lastReadTime) < MIN_INTERVAL) return;
  lastReadTime = millis();
  
  // Read the temperature.

	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// t_fine carries fine temperature as global value

	//get the reading (adc_T);
	int32_t adc_T = ((uint32_t)readRegister(BME280_TEMPERATURE_MSB_REG) << 12) | ((uint32_t)readRegister(BME280_TEMPERATURE_LSB_REG) << 4) | ((readRegister(BME280_TEMPERATURE_XLSB_REG) >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;
	temperature = output / 100;
	
	// Read the pressure.
	
	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
	int32_t adc_P = ((uint32_t)readRegister(BME280_PRESSURE_MSB_REG) << 12) | ((uint32_t)readRegister(BME280_PRESSURE_LSB_REG) << 4) | ((readRegister(BME280_PRESSURE_XLSB_REG) >> 4) & 0x0F);
	
	int64_t p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	  pressure = -1;
	  
	else {
	  p_acc = 1048576 - adc_P;
	  p_acc = (((p_acc<<31) - var2)*3125)/var1;
	  var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	  var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
	  p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
	  pressure = p_acc / 256.0 / 100;
	}
	
  // Read humidity.

	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
	int32_t adc_H = ((uint32_t)readRegister(BME280_HUMIDITY_MSB_REG) << 8) | ((uint32_t)readRegister(BME280_HUMIDITY_LSB_REG));
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)calibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);
	humidity = (float)(var1>>12) / 1024.0;
  return;
}


//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void BME280::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	//define pointer that will point to the external space
	uint8_t i = 0;
	char c = 0;

	Wire.beginTransmission(ADDRESS);
	Wire.write(offset);
	Wire.endTransmission();

	// request bytes from slave device
	Wire.requestFrom((uint8_t)ADDRESS, length);
	while ( (Wire.available()) && (i < length))  // slave may send less than requested
	{
		c = Wire.read(); // receive a byte as character
		*outputPointer = c;
		outputPointer++;
		i++;
	}

}

uint8_t BME280::readRegister(uint8_t offset)
{
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;
	Wire.beginTransmission(ADDRESS);
	Wire.write(offset);
	Wire.endTransmission();

	Wire.requestFrom((uint8_t)ADDRESS, numBytes);
	while ( Wire.available() ) // slave may send less than requested
	{
		result = Wire.read(); // receive a byte as a proper uint8_t
	}
	return result;
}

int16_t BME280::readRegisterInt16( uint8_t offset )
{
	uint8_t myBuffer[2];
	readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
	
	return output;
}

void BME280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	//Write the byte
	Wire.beginTransmission(ADDRESS);
	Wire.write(offset);
	Wire.write(dataToWrite);
	Wire.endTransmission();
}
