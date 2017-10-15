/* DHT library

MIT license
written by Adafruit Industries - stripped down by City Hydroponics.
*/

#include "DHT22.h"

#define MIN_INTERVAL 2000

DHT22::DHT22() {
  maxcycles = microsecondsToClockCycles(1000);  // 1 millisecond timeout for
                                                 // reading pulses from DHT sensor.
}

void DHT22::begin(uint8_t p) {
  pin = p;

  // set up the pins!
  pinMode(pin, INPUT_PULLUP);

  // Using this value makes sure that millis() - lastreadtime will be
  // >= MIN_INTERVAL right away. Note that this assignment wraps around,
  // but so will the subtraction.
  lastreadtime = -MIN_INTERVAL;
}

//boolean S == Scale.  True == Fahrenheit; False == Celcius
float DHT22::readTemperature(bool S) {
  float f = -1;

  if (read()) {
    f = ((data[2] & 0x7F) << 8);
    f += data[3];
    f *= 0.1;
    if (data[2] & 0x80) {
      f *= -1;
    }
    if(S) {
      f = f * 1.8 + 32; // convert from C to F.
    }
  }
  else 
    return -1;
  if (isnan(f)) return -1; // Invalid reading: return -1.
  return f; 
}

float DHT22::readHumidity() {
  float f = -1;
  if (read()) {
    f = (data[0] << 8);
    f += data[1];
    f *= 0.1;
  }
  else 
    return -1;
  if (isnan(f)) return -1; // Invalid reading: return -1.
  return f; 
}

boolean DHT22::read(bool force) {

  // Check if sensor was read less than MIN_INTERVAL milliseconds ago and if so return early
  // to use last reading.
  if (!force && ((millis() - lastreadtime) < MIN_INTERVAL)) return lastresult; // return last correct measurement
  lastreadtime = millis();

  // Reset 40 bits of received data to zero.
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  digitalWrite(pin, HIGH);
  delay(250);

  // First set data line low for 20 milliseconds.
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(20);

  uint32_t cycles[80];
  {
    // Turn off interrupts temporarily because the next sections are timing critical
    // and we don't want any interruptions.
    InterruptLock lock;

    // End the start signal by setting data line high for 40 microseconds.
    digitalWrite(pin, HIGH);
    delayMicroseconds(40);

    // Now start reading the data line to get the value from the DHT sensor.
    pinMode(pin, INPUT_PULLUP);
    delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == 0) {
      lastresult = false;
      return lastresult;
    }
    if (expectPulse(HIGH) == 0) {
      lastresult = false;
      return lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (uint8_t i=0; i<80; i+=2) {
      cycles[i]   = expectPulse(LOW);
      cycles[i+1] = expectPulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (uint8_t i=0; i<40; ++i) {
    uint32_t lowCycles  = cycles[2*i];
    uint32_t highCycles = cycles[2*i+1];
    if ((lowCycles == 0) || (highCycles == 0)) {
      lastresult = false;
      return lastresult;
    }
    data[i/8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
    
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    lastresult = true;
    return lastresult;
  }
  else {
    lastresult = false;
    return lastresult;
  }
}

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint32_t DHT22::expectPulse(bool level) {
  uint32_t count = 0;
  
  // Otherwise fall back to using digitalRead (this seems to be necessary on ESP8266
  // right now, perhaps bugs in direct port access functions?).
  while (digitalRead(pin) == level) {
    if (count++ >= maxcycles) {
      return 0; // Exceeded timeout, fail.
    }
  }
  return count;
}
