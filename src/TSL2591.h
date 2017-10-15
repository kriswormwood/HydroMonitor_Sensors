/**************************************************************************/
/*! 
    @file     TSL2591.h
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

#ifndef _TSL2591_H_
#define _TSL2591_H_

#include <Arduino.h>
#include <Wire.h>

#define TSL2591_VISIBLE           (2)       // channel 0 - channel 1
#define TSL2591_INFRARED          (1)       // channel 1
#define TSL2591_FULLSPECTRUM      (0)       // channel 0

#define TSL2591_ADDR              (0x29)
#define TSL2591_READBIT           (0x01)

#define TSL2591_COMMAND_BIT       (0xA0)    // 1010 0000: bits 7 and 5 for 'command normal'
#define TSL2591_CLEAR_INT         (0xE7)
#define TSL2591_TEST_INT          (0xE4)
#define TSL2591_WORD_BIT          (0x20)    // 1 = read/write word (rather than byte)
#define TSL2591_BLOCK_BIT         (0x10)    // 1 = using block read/write

#define TSL2591_ENABLE_POWEROFF   (0x00)
#define TSL2591_ENABLE_POWERON    (0x01)
#define TSL2591_ENABLE_AEN        (0x02)    // ALS Enable. This field activates ALS function. Writing a one activates the ALS. Writing a zero disables the ALS.
#define TSL2591_ENABLE_AIEN       (0x10)    // ALS Interrupt Enable. When asserted permits ALS interrupts to be generated, subject to the persist filter.
#define TSL2591_ENABLE_NPIEN      (0x80)    // No Persist Interrupt Enable. When asserted NP Threshold conditions will generate an interrupt, bypassing the persist filter

#define TSL2591_LUX_DF            (408.0F)
#define TSL2591_LUX_COEFB         (1.64F)  // CH0 coefficient 
#define TSL2591_LUX_COEFC         (0.59F)  // CH1 coefficient A
#define TSL2591_LUX_COEFD         (0.86F)  // CH2 coefficient B

enum
{
  TSL2591_REGISTER_ENABLE             = 0x00,
  TSL2591_REGISTER_CONTROL            = 0x01,
  TSL2591_REGISTER_THRESHOLD_AILTL    = 0x04, // ALS low threshold lower byte
  TSL2591_REGISTER_THRESHOLD_AILTH    = 0x05, // ALS low threshold upper byte
  TSL2591_REGISTER_THRESHOLD_AIHTL    = 0x06, // ALS high threshold lower byte
  TSL2591_REGISTER_THRESHOLD_AIHTH    = 0x07, // ALS high threshold upper byte
  TSL2591_REGISTER_THRESHOLD_NPAILTL  = 0x08, // No Persist ALS low threshold lower byte
  TSL2591_REGISTER_THRESHOLD_NPAILTH  = 0x09, // etc
  TSL2591_REGISTER_THRESHOLD_NPAIHTL  = 0x0A,
  TSL2591_REGISTER_THRESHOLD_NPAIHTH  = 0x0B,
  TSL2591_REGISTER_PERSIST_FILTER     = 0x0C,
  TSL2591_REGISTER_PACKAGE_PID        = 0x11,
  TSL2591_REGISTER_DEVICE_ID          = 0x12,
  TSL2591_REGISTER_DEVICE_STATUS      = 0x13,
  TSL2591_REGISTER_CHAN0_LOW          = 0x14,
  TSL2591_REGISTER_CHAN0_HIGH         = 0x15,
  TSL2591_REGISTER_CHAN1_LOW          = 0x16,
  TSL2591_REGISTER_CHAN1_HIGH         = 0x17
};


class TSL2591
{
 public:
  TSL2591();
  
  boolean   begin   ( void );
  void      enable  ( void );
  void      disable ( void );
  void      write8  ( uint8_t r);
  void      write8  ( uint8_t r, uint8_t v );
  uint16_t  read16  ( uint8_t reg );
  uint8_t   read8   ( uint8_t reg );

  void      setGain       ( uint8_t gain );
  void      setTiming     ( uint8_t timing );
  
  int32_t   readSensor(void);

 private:
  uint8_t integration;
  uint8_t gain;
  boolean initialized;
  uint32_t  takeReading ( );
};
#endif
