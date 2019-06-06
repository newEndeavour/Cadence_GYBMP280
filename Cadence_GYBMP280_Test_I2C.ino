/*
  File:         Cadence_GYBMP280
  Version:      1.0.0
  Date:         4-Jun-2019
  Revision:     4-Jun-2019
  Author:       Jerome Drouin
  
  https://github.com/newEndeavour/Cadence_GYBMP280
  Pressure & Temperature Sensor Library for 'duino
  Warning: BMP280 is a 3.3V device. Use level shifters to avoid damaging your Sensor

  Credits: 
        - Library initially inspired by/ derived from Adafruit_BMP280. Thanks.

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


#include <Wire.h>
#include <SPI.h>
#include <Cadence_GYBMP280.h>

Cadence_GYBMP280 Sensor; // I2C
//Cadence_GYBMP280 Sensor(BMP_CS); // hardware SPI
//Cadence_GYBMP280 Sensor(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
  
void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));
  
  if (!Sensor.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  //Set QNH
  Sensor.setDefaultQNH(1013.25);
  
}
  
void loop() {
    Serial.print("Temperature = ");
    Serial.print(Sensor.readTemperatureC());
    Serial.println("*C");
    
    Serial.print("Temperature = ");
    Serial.print(Sensor.readTemperatureF());
    Serial.println("*F");

    Serial.print("Pressure = ");
    Serial.print(Sensor.readPressure()/100.0);
    Serial.println("hPa");

    float localPress = 1017.80;
    Serial.print("Press. altitude (hPa=");
    Serial.print(localPress,2); 
    Serial.print(")=");
    Serial.print(Sensor.readAltitude(localPress));
    Serial.println(" m");
    
    Serial.print("Press. altitude (QNH=");
    Serial.print(Sensor.getDefaultQNH(),2); 
    Serial.print(")=");
    Serial.print(Sensor.readAltitude()); 
    Serial.println(" m");

    Serial.println();
    delay(2000);
}
