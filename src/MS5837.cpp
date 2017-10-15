#include "MS5837.h"
#include <Wire.h>

MS5837::MS5837() {
	fluidDensity = 997;
}

bool MS5837::begin() {

  // Send reset command, and wait 10 ms for it to compelte.
  Wire.beginTransmission(MS5837_ADDR);
  Wire.write(MS5837_RESET);
  Wire.endTransmission();
  delay(10);

  // Read coefficients values stored in PROM of the device
  for(int i = 0; i < 7; i++)
  {
    // Select data register to read.
    Wire.beginTransmission(MS5837_ADDR);
    Wire.write(MS5837_PROM_READ + (2 * i));
    Wire.endTransmission();
  
    // Request 2 bytes of data
    Wire.requestFrom(MS5837_ADDR, 2);
      
    // Read 2 bytes of data: coeff msb, coeff lsb.
    C[i] = (Wire.read() << 8) | Wire.read();
  }

	// Verify that data is correct with CRC
	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);
	
	// Make sure the sensor will be read right away.
  lastReadTime = -MIN_INTERVAL;

	if ( crcCalculated == crcRead ) {
		return true; // Initialization success
	}

	return false; // CRC fail
}

void MS5837::setFluidDensity(float density) {
	fluidDensity = density;
}

void MS5837::read() {

  // Check if sensor was read less than MIN_INTERVAL milliseconds ago. If so, don't read it again
  // at this time.  
  if ((millis() - lastReadTime) < MIN_INTERVAL) return;
  lastReadTime = millis();

  // Request D1 (pressure) conversion using OSR = 8192
  Wire.beginTransmission(MS5837_ADDR);
  Wire.write(MS5837_CONVERT_D1);
  Wire.endTransmission();
  delay(20);  // Wait for conversion to complete, maximum 17.2 ms according to the datasheet.
  
  // Request the ADC to provide us with the result.
  Wire.beginTransmission(MS5837_ADDR);
  Wire.write(MS5837_ADC_READ);
  Wire.endTransmission();
  
  // Request the 3 bytes of data that should be ready for us.
  Wire.requestFrom(MS5837_ADDR, 3);
  uint32_t D1 = 0;
	D1 = Wire.read();
	D1 = (D1 << 8) | Wire.read();
	D1 = (D1 << 8) | Wire.read();

  // Request D2 (temperature) conversion using OSR = 8192
  Wire.beginTransmission(MS5837_ADDR);
  Wire.write(MS5837_CONVERT_D2);
  Wire.endTransmission();
  delay(20);  // Wait for conversion to complete, maximum 17.2 ms according to the datasheet.
  
  // Request the ADC to provide us with the result.
  Wire.beginTransmission(MS5837_ADDR);
  Wire.write(MS5837_ADC_READ);
  Wire.endTransmission();
  
  // Request the 3 bytes of data that should be ready for us.
  Wire.requestFrom(MS5837_ADDR, 3);
  uint32_t D2 = 0;
	D2 = Wire.read();
	D2 = (D2 << 8) | Wire.read();
	D2 = (D2 << 8) | Wire.read();

  // Calculate temperature (first order compensation).
  int32_t dT = D2 - ((int32_t)C[5] << 8);
  temperature = 2000 + ((int64_t(dT) * C[6]) >> 23);
  
  // Calculate temperature compensated pressure (first order compensation).
  int64_t off = (int64_t(C[2]) << 17) + ((int64_t(C[4]) * dT) >> 6);    // Offset at actual temperature.
  int64_t sens = (int64_t(C[1]) << 16) + ((int64_t(C[3]) * dT) >> 7);   // Sensitivity at actual temperature.
  pressure = (((D1 * sens) >> 21) - off) >> 15;      // Temperature compensated pressure x100.

  // If the temperature is < 20 C, calculate the second order compensation.
  if (temperature < 2000) {
    uint64_t ti = (11 * dT * dT) >> 35;
    int64_t offi = (31 * (temperature - 2000) * (temperature - 2000)) >> 3;
    int64_t sensi = (63 * (temperature - 2000) * (temperature - 2000)) >> 5;
    off -= offi;
    sens -= sensi;
    temperature -= ti;
    pressure = (((D1 * sens) >> 21) - off) >> 15;
  }

  // Crude correction: make sure it's above the pressure the BMP280 returns.  
  pressure += 700;
  
}

float MS5837::readTemperature() {
  read();
	return temperature/100.0f;
}

float MS5837::readPressure() {
  read();
	return pressure/100.0f;
}

float MS5837::readWaterLevel(float atmPressure) {
  read();

  // Returns water level in cm.  
	return 100 * ((float)pressure-atmPressure * 100)/(fluidDensity*9.80665);
}

uint8_t MS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	
	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}
