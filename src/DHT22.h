/* DHT library

MIT license
written by Adafruit Industries - stripped down by City Hydroponics.
*/
#ifndef DHT22_H
#define DHT22_H

#include "Arduino.h"

class DHT22 {
  public:
    DHT22(void);
    void begin(uint8_t pin);
    float readTemperature(bool S=false);
    float convertCtoF(float);
    float readHumidity();

  private:
    uint8_t data[5];
    uint8_t pin, type;
    uint32_t lastreadtime, maxcycles;
    bool lastresult;
    uint32_t expectPulse(bool level);
    boolean read(bool force=false);

};

class InterruptLock {
  public:
   InterruptLock() {
    noInterrupts();
   }
   ~InterruptLock() {
    interrupts();
   }

};

#endif
