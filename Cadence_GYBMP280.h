/*
  File:         Cadence_GYBMP280.h
  Version:      1.0.1
  Date:         4-Jun-2019
  Revision:     6-Jun-2019
  Author:       Jerome Drouin
  
  https://github.com/newEndeavour/Cadence_GYBMP280
  library for the GY_BMP280 Pressure/Temperature Sensor
  Sensors use I2C to communicate, 2 pins are required to interface

  Estimattion of Pressure Altitude.

  Source: 	A Quick Derivation relating altitude to air pressure
  		© 2004 Portland State Aerospace Society <http://www.psas.pdx.edu>
  		Redistribution allowed under the terms of the GNU General Public License version 2 or later.

  Abstract: 	Using a barometer to measure altitude is a well established technique. 
  		The idealized theory for doing this is easily expressed. This derivation aims to 
  		make the concepts involved easily and rapidly available to a technical audience.

		Atmospheric parameters used in this Calculation (SI units):
		Symbol	Value		Unit		Description
		P0	101325		Pa		Pressure at Zero Altitude (Sea level)	
		T0	288.15		K		Kelvins. Temp at Zero Altitude	
		g	9.80665		m/s^2		acceleration due to gravity (standard uniform)
		L	-6.5.10-3	K/m		Kelvins per metre. Temp lapse rate
		R	287.053		J/(kg.K)	Gas constant of Air
		Rh	0%		-		Relative humidity
		z					Elevation/Pressure Altitude

		(1). Hypsometric equation
			
			z = - R.T/g x ln(P/P0)	(Equ.4)

			Constant Temp and gravity (zero lapse rate). The hypsometric equation is not 
			very satisfactory because it assumes zero lapse rate. It is mentioned here because 
			it is often cited in the literature. Don’t use it for altitude determination 
			unless the approximation of constant temperature is an acceptable one.


		(2). Simplified expression for altitude in terms of atmospheric pressure. 


			Z = T0/L x ((P/P0)^(-L.R/g)-1)	(Equ.8)

  Credits: 
  Library initially inspired by K.Townsend (Adafruit Industries) and based on Adafruit_BMP280. Thanks.

  Copyright (c) 2018-2019 Jerome Drouin  All rights reserved.  

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Versions:
  1.0.0		: Initial version
  1.0.1		: Introduced Default_ISA_QNH
		  Added Altitude in Metre and Feet

	
*/


#ifndef __BMP280_H__
#define __BMP280_H__

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>

//
//  I2C ADDRESS/BITS/SETTINGS
//#define BMP280_ADDRESS 	(0x77) 	//Default I2C address for the sensor
#define BMP280_ADDRESS 		(0x76) 	//I2C address for the GY_BMP280 sensor

#define BMP280_ADDRESS_ALT 	(0x76) 	//Alternative I2C address for the sensor
#define BMP280_CHIPID 		(0x58) 	//Default chip ID


#define DEFAULT_ISA_QNH 	1013.25	//Default Pressure Altitude at Sea Level in ISA Atmosphere


//  Forward declarations of Wire and SPI for board/variant combinations that don't have a default 'Wire' or 'SPI' 
extern TwoWire Wire;  			//Forward declaration of Wire object
extern SPIClass SPI;  			//Forward declaration of SPI object


// Registers available on the sensor.
enum {
  BMP280_REGISTER_DIG_T1 	= 0x88,
  BMP280_REGISTER_DIG_T2 	= 0x8A,
  BMP280_REGISTER_DIG_T3 	= 0x8C,
  BMP280_REGISTER_DIG_P1 	= 0x8E,
  BMP280_REGISTER_DIG_P2 	= 0x90,
  BMP280_REGISTER_DIG_P3 	= 0x92,
  BMP280_REGISTER_DIG_P4 	= 0x94,
  BMP280_REGISTER_DIG_P5 	= 0x96,
  BMP280_REGISTER_DIG_P6 	= 0x98,
  BMP280_REGISTER_DIG_P7 	= 0x9A,
  BMP280_REGISTER_DIG_P8 	= 0x9C,
  BMP280_REGISTER_DIG_P9 	= 0x9E,
  BMP280_REGISTER_CHIPID 	= 0xD0,
  BMP280_REGISTER_VERSION 	= 0xD1,
  BMP280_REGISTER_SOFTRESET 	= 0xE0,
  BMP280_REGISTER_CAL26 	= 0xE1, //R calibration = 0xE1-0xF0
  BMP280_REGISTER_CONTROL 	= 0xF4,
  BMP280_REGISTER_CONFIG 	= 0xF5,
  BMP280_REGISTER_PRESSUREDATA 	= 0xF7,
  BMP280_REGISTER_TEMPDATA 	= 0xFA,
};


//  Struct to hold calibration data.
typedef struct {
  uint16_t dig_T1; 		//< dig_T1 cal register
  int16_t dig_T2;  		//< dig_T2 cal register
  int16_t dig_T3;  		//< dig_T3 cal register

  uint16_t dig_P1; 		//< dig_P1 cal register
  int16_t dig_P2;  		//< dig_P2 cal register
  int16_t dig_P3;  		//< dig_P3 cal register
  int16_t dig_P4;  		//< dig_P4 cal register
  int16_t dig_P5;  		//< dig_P5 cal register
  int16_t dig_P6;  		//< dig_P6 cal register
  int16_t dig_P7;  		//< dig_P7 cal register
  int16_t dig_P8;  		//< dig_P8 cal register
  int16_t dig_P9;  		//< dig_P9 cal register

  uint8_t dig_H1; 		//< dig_H1 cal register
  int16_t dig_H2; 		//< dig_H2 cal register
  uint8_t dig_H3; 		//< dig_H3 cal register
  int16_t dig_H4; 		//< dig_H4 cal register
  int16_t dig_H5; 		//< dig_H5 cal register
  int8_t dig_H6;  		//< dig_H6 cal register
} bmp280_calib_data;


//
// Driver for the Adafruit BMP280 barometric pressure sensor.
class Cadence_GYBMP280 {
public:
  
  enum sensor_sampling {	// Oversampling rate for the sensor.
    SAMPLING_NONE = 0x00,    	// No over-sampling 
    SAMPLING_X1 = 0x01,    	// 1x over-sampling 
    SAMPLING_X2 = 0x02,		// 2x over-sampling  
    SAMPLING_X4 = 0x03,		// 4x over-sampling 
    SAMPLING_X8 = 0x04,		// 8x over-sampling
    SAMPLING_X16 = 0x05		// 16x over-sampling 
  };

  enum sensor_mode {		// Operating mode for the sensor.
    MODE_SLEEP = 0x00,    	// Sleep mode
    MODE_FORCED = 0x01,    	// Forced mode 
    MODE_NORMAL = 0x03,		// Normal mode 
    MODE_SOFT_RESET_CODE = 0xB6	// Software reset 
  };

  enum sensor_filter {		// Filtering level for sensor data. 
    FILTER_OFF = 0x00,    	// No filtering
    FILTER_X2 = 0x01,    	// 2x filtering 
    FILTER_X4 = 0x02,    	// 4x filtering 
    FILTER_X8 = 0x03,    	// 8x filtering 
    FILTER_X16 = 0x04    	// 16x filtering 
  };

  enum standby_duration {	// Standby duration in ms    
    STANDBY_MS_1 = 0x00,	// 1 ms standby 
    STANDBY_MS_63 = 0x01,	// 63 ms standby 
    STANDBY_MS_125 = 0x02,	// 125 ms standby 
    STANDBY_MS_250 = 0x03,	// 250 ms standby 
    STANDBY_MS_500 = 0x04,	// 500 ms standby 	
    STANDBY_MS_1000 = 0x05,	// 1000 ms standby 
    STANDBY_MS_2000 = 0x06,	// 2000 ms standby 
    STANDBY_MS_4000 = 0x07	// 4000 ms standby 
  };

  //Constructors
  Cadence_GYBMP280(TwoWire *theWire = &Wire);
  Cadence_GYBMP280(int8_t cspin, SPIClass *theSPI = &SPI);
  Cadence_GYBMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

  //Methods
  bool 		begin(uint8_t addr = BMP280_ADDRESS, uint8_t chipid = BMP280_CHIPID);
  float 	readTemperatureC();
  float 	readTemperatureF();
  float 	seaLevelForAltitude(float altitude, float atmospheric);
  void	        setDefaultQNH(float);
  float	        getDefaultQNH();
  float 	readPressure(void);
  float 	readAltitudeMetre(void);
  float 	readAltitudeMetre(float);
  float 	readAltitudeFeet(void);
  float 	readAltitudeFeet(float);
  void 		setSampling(	sensor_mode mode = MODE_NORMAL,
                   		sensor_sampling tempSampling = SAMPLING_X16,
                   		sensor_sampling pressSampling = SAMPLING_X16,
                   		sensor_filter filter = FILTER_OFF,
                   		standby_duration duration = STANDBY_MS_1);
  
  //void takeForcedMeasurement();

  TwoWire *_wire; 		// Wire object
  SPIClass *_spi; 		// SPI object


private:
  struct config {		// Encapsulates the config register
    unsigned int t_sb : 3; 	// Inactive duration (standby time) in normal mode
    unsigned int filter : 3;	// Filter settings
    unsigned int none : 1;	// Unused - don't set
    unsigned int spi3w_en : 1;	// Enables 3-wire SPI
    unsigned int get() { return (t_sb << 5) | (filter << 2) | spi3w_en; } // Used when Retrieving the assembled config register's byte value.
  };

  struct ctrl_meas {		// Encapsulates the ctrl_meas register
    unsigned int osrs_t : 3;    // Temperature oversampling. 
    unsigned int osrs_p : 3;	// Pressure oversampling. 
    unsigned int mode : 2;    	// Device mode 
    unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }   // Used when Retrieving the assembled ctrl_meas register's byte value.
  };

  void 			readCoefficients(void);
  uint8_t 		spixfer(uint8_t x);
  void 			write8(byte reg, byte value);
  uint8_t 		read8(byte reg);
  uint16_t 		read16(byte reg);
  uint32_t 		read24(byte reg);
  int16_t 		readS16(byte reg);
  uint16_t 		read16_LE(byte reg);
  int16_t 		readS16_LE(byte reg);
  uint8_t 		_i2caddr;
  int32_t 		_sensorID;
  int32_t 		t_fine;
  int8_t 		_cs;
  int8_t		_mosi;
  int8_t		_miso;
  int8_t 		_sck;
  bmp280_calib_data 	_bmp280_calib;
  config 		_configReg;
  ctrl_meas 		_measReg;

  float 		default_QNH;

};

#endif