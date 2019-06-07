/*
  File:         Cadence_GYBMP280.cpp
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


			z = T0/L x ((P/P0)^(-L.R/g)-1)	(Equ.8)

		Which gives:		
	
			z = 44330.76 * (1.0 - pow(localhPa / seaLevelhPa, 0.190263));
			
		

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

*/


#include "Cadence_GYBMP280.h"
#include "Arduino.h"
#include <Wire.h>

 
// BMP280 constructor using i2c
// @param  *theWire
Cadence_GYBMP280::Cadence_GYBMP280(TwoWire *theWire)

	: _cs(-1), _mosi(-1), _miso(-1), _sck(-1) {
  	_wire = theWire;

}



// BMP280 constructor using hardware SPI
// @param  cspin
//         cs pin number
// @param  theSPI
//         optional SPI object
Cadence_GYBMP280::Cadence_GYBMP280(int8_t cspin, SPIClass *theSPI)
    
	: _cs(cspin), _mosi(-1), _miso(-1), _sck(-1) {
  	*_spi = *theSPI;

}



// BMP280 constructor using bitbang SPI
// @param  cspin
//         The pin to use for CS/SSEL.
// @param  mosipin
//         The pin to use for MOSI.
// @param  misopin
//         The pin to use for MISO.
// @param  sckpin
//         The pin to use for SCK.
Cadence_GYBMP280::Cadence_GYBMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
	
	: _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin) {

}



//  Initialises the sensor.
//  @param addr
//         The I2C address to use (default = 0x77)
//  @param chipid
//         The expected chip ID (used to validate connection).
//  @return True if the init was successful, otherwise false.
bool Cadence_GYBMP280::begin(uint8_t addr, uint8_t chipid) {
  _i2caddr = addr;

  if (_cs == -1) {
    // i2c
    _wire->begin();
  } else {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);

    if (_sck == -1) {
      // hardware SPI
      _spi->begin();
    } else {
      // software SPI
      pinMode(_sck, OUTPUT);
      pinMode(_mosi, OUTPUT);
      pinMode(_miso, INPUT);
    }
  }

  if (read8(BMP280_REGISTER_CHIPID) != chipid)
    return false;

  readCoefficients();
  // write8(BMP280_REGISTER_CONTROL, 0x3F); /* needed? */
  setSampling();

  //Set default QNH
  default_QNH = DEFAULT_ISA_QNH;

  delay(100);
  return true;
}


// Sets the sampling config for the device.
// @param mode
//        The operating mode of the sensor.
// @param tempSampling
//        The sampling scheme for temp readings.
// @param pressSampling
//        The sampling scheme for pressure readings.
// @param filter
//        The filtering mode to apply (if any).
// @param duration
//        The sampling duration.
void Cadence_GYBMP280::setSampling(sensor_mode mode,
                                  sensor_sampling tempSampling,
                                  sensor_sampling pressSampling,
                                  sensor_filter filter,
                                  standby_duration duration) {
  _measReg.mode = mode;
  _measReg.osrs_t = tempSampling;
  _measReg.osrs_p = pressSampling;

  _configReg.filter = filter;
  _configReg.t_sb = duration;

  write8(BMP280_REGISTER_CONFIG, _configReg.get());
  write8(BMP280_REGISTER_CONTROL, _measReg.get());
}


uint8_t Cadence_GYBMP280::spixfer(uint8_t x) {
  if (_sck == -1)
    return _spi->transfer(x);

  // software spi
  // Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--) {
    reply <<= 1;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, x & (1 << i));
    digitalWrite(_sck, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
}


// Writes an 8 bit value over I2C/SPI
void Cadence_GYBMP280::write8(byte reg, byte value) {
  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->write((uint8_t)value);
    _wire->endTransmission();
  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg & ~0x80); // write, bit 7 low
    spixfer(value);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }
}


// Reads an 8 bit value over I2C/SPI
// @param  reg
//         selected register
// @return value from selected register
uint8_t Cadence_GYBMP280::read8(byte reg) {
  uint8_t value;

  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom((uint8_t)_i2caddr, (byte)1);
    value = _wire->read();

  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }
  return value;
}


// Reads a 16 bit value over I2C/SPI
uint16_t Cadence_GYBMP280::read16(byte reg) {
  uint16_t value;

  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom((uint8_t)_i2caddr, (byte)2);
    value = (_wire->read() << 8) | _wire->read();

  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = (spixfer(0) << 8) | spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }

  return value;
}


uint16_t Cadence_GYBMP280::read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}


//   @brief  Reads a signed 16 bit value over I2C/SPI
int16_t Cadence_GYBMP280::readS16(byte reg) { 
  return (int16_t)read16(reg); 
}


int16_t Cadence_GYBMP280::readS16_LE(byte reg) {
  return (int16_t)read16_LE(reg);
}



// Reads a 24 bit value over I2C/SPI
uint32_t Cadence_GYBMP280::read24(byte reg) {
  uint32_t value;

  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom((uint8_t)_i2caddr, (byte)3);

    value = _wire->read();
    value <<= 8;
    value |= _wire->read();
    value <<= 8;
    value |= _wire->read();

  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high

    value = spixfer(0);
    value <<= 8;
    value |= spixfer(0);
    value <<= 8;
    value |= spixfer(0);

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }

  return value;
}


// Reads the factory-set coefficients
void Cadence_GYBMP280::readCoefficients() {
  _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
  _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
  _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

  _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
  _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
  _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
  _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
  _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
  _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
  _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
  _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
  _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}


//Reads the temperature from the device. 
//Returns temperature in degrees celcius.
float Cadence_GYBMP280::readTemperatureC() {
  int32_t var1, var2;

  int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
          ((int32_t)_bmp280_calib.dig_T2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
           12) *
          ((int32_t)_bmp280_calib.dig_T3)) >>
         14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}


//Reads the temperature from the device.
//Return temperature in degrees Farenheit.
float Cadence_GYBMP280::readTemperatureF() {

  return (readTemperatureC() * 1.8 +32.0);

}



// Reads the barometric pressure from the device.
// Return Barometric pressure in hPa.
float Cadence_GYBMP280::readPressure() {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  readTemperatureC();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
  return (float)p / 256;
}


// Calculates the approximate altitude using barometric pressure and the
// supplied sea level hPa as a reference.
// Returns The approximate altitude above sea level in meters.
float Cadence_GYBMP280::readAltitudeMetre(float seaLevelhPa) {
  float altitude;

  float localhPa = readPressure(); // in Si units for Pascal
  localhPa /= 100;

  altitude = 44330.76 * (1.0 - pow(localhPa / seaLevelhPa, 0.190263));

  return altitude;
}


// Calculates the approximate altitude using barometric pressure and the
// supplied sea level hPa as a reference.
// Returns The approximate altitude above sea level in Feet.
float Cadence_GYBMP280::readAltitudeFeet(float seaLevelhPa) {
  float altitude;

  float localhPa = readPressure(); // in Si units for Pascal
  localhPa /= 100;

  altitude = 145366.45 * (1.0 - pow(localhPa / seaLevelhPa, 0.190263));

  return altitude;
}


//set the default QNH
void Cadence_GYBMP280::setDefaultQNH(float _QNH) {

  if (_QNH>0) default_QNH = _QNH;

}


//Returns the default QNH
float Cadence_GYBMP280::getDefaultQNH() {

  return default_QNH;

}


// Calculates the approximate altitude using barometric pressure and the
// default default_QNH (sea level hPa) as a reference
// Returns The approximate altitude above sea level in meters.
float Cadence_GYBMP280::readAltitudeMetre() {
  float altitude;

  float localhPa = readPressure(); // in Si units for Pascal
  localhPa /= 100;

  altitude = 44330.76 * (1.0 - pow(localhPa / default_QNH, 0.190263));

  return altitude;
}


// Calculates the approximate altitude using barometric pressure and the
// default default_QNH (sea level hPa) as a reference
// Returns The approximate altitude above sea level in Feet.
float Cadence_GYBMP280::readAltitudeFeet() {
  float altitude;

  float localhPa = readPressure(); // in Si units for Pascal
  localhPa /= 100;

  altitude = 145366.45 * (1.0 - pow(localhPa / default_QNH, 0.190263));

  return altitude;
}


// Calculates the pressure at sea level (in hPa) from the specified altitude// 
// (in meters), and atmospheric pressure (in hPa).
// @param  altitude      Altitude in meters
// @param  atmospheric   Atmospheric pressure in hPa
// @return The approximate pressure
float Cadence_GYBMP280::seaLevelForAltitude(float altitude, float atmospheric) {

  return atmospheric / pow(1.0 - (altitude / 44330.76), 5.255882);

}


// INACTIVE
// Take a new measurement (only possible in forced mode)
/*
void Cadence_GYBMP280::takeForcedMeasurement()
{
    // If we are in forced mode, the BME sensor goes back to sleep after each
    // measurement and we need to set it to forced mode once at this point, so
    // it will take the next measurement and then return to sleep again.
    // In normal mode simply does new measurements periodically.
    if (_measReg.mode == MODE_FORCED) {
        // set to forced mode, i.e. "take next measurement"
        write8(BMP280_REGISTER_CONTROL, _measReg.get());
        // wait until measurement has been completed, otherwise we would read
        // the values from the last measurement
        while (read8(BMP280_REGISTER_STATUS) & 0x08)
                delay(1);
    }
}
*/
