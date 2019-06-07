#Cadence_GYBMP280 library#
Driver for GY_BMP280 Barometric Pressure Sensor from Bosch (https://github.com/newEndeavour/Cadence_GYBMP280/blob/master/GY-BMP280-3V3-High-Precision-Atmospheric-Pressure-Sensor-Module-for-Arduino.jpg)

Credits:
The Cadence_GYBMP280 library was originally inspired from Adafruit_BMP280 by K.Townsend (Adafruit Industries) and is maintained by Jerome Drouin.

About the GY_BMP280 Sensor:
This precision sensor from Bosch is the best low-cost sensing solution for measuring barometric pressure and temperature. 
Because pressure changes with altitude you can also use it as an altimeter using the ISA standard atmosphere characteristics.

WARNING:
The sensor is a 3.3V device: a Logic Level shifter must be used to avoid damaging the sensor with 5V.
In some cases, 5V may be used and may not damage the sensor, but the behaviour of the sensor may not be predictable.  


Library Compatibility:

MCU                | Tested Works | Doesn't Work | Not Tested  | Notes
------------------ | :----------: | :----------: | :---------: | -----
Atmega328 @ 16MHz  |      X       |		 |             |
Atmega328 @ 12MHz  |      X       |              |             |
Atmega32u4 @ 16MHz |      X       |              |             | (1)
Atmega32u4 @ 8MHz  |      X       |              |             | (2)
ESP8266            |      X       |              |             | (3)
Atmega2560 @ 16MHz |      X       |              |             | (4)
ATSAM3X8E          |      X       |              |             | (5)
ATSAM21D           |      X       |              |             |
ATtiny85 @ 16MHz   |              |      X       |             |
ATtiny85 @ 8MHz    |              |      X       |             |
Intel Curie @ 32MHz|              |              |     X       |
STM32F2            |              |              |     X       |

  (1): Use SDA/SCL on pins D2 &amp; D3
  (2): Use SDA/SCL on pins D2 &amp; D3
  (3): SDA/SCL default to pins 4 &amp; 5 but any two pins can be assigned as SDA/SCL using Wire.begin(SDA,SCL)
  (4): Use SDA/SCL on pins 20 &amp; 21
  (5): Use SDA/SCL on pins 20 &amp; 21


  * ATmega328 @ 16MHz 	: Arduino UNO, Arduino Nano, Adafruit Pro Trinket 5V, 
			  Adafruit Metro 328, Adafruit Metro Mini
  * ATmega328 @ 12MHz 	: Adafruit Pro Trinket 3V
  * ATmega32u4 @ 16MHz 	: Arduino Leonardo, Arduino Micro, Arduino Yun, Teensy 2.0
  * ATmega32u4 @ 8MHz 	: Adafruit Flora, Bluefruit Micro
  * ESP8266 		: Adafruit Huzzah
  * ATmega2560 @ 16MHz 	: Arduino Mega
  * ATSAM3X8E 		: Arduino Due
  * ATSAM21D 		: Arduino Zero, M0 Pro
  * ATtiny85 @ 16MHz 	: Adafruit Trinket 5V
  * ATtiny85 @ 8MHz 	: Adafruit Gemma, Arduino Gemma, Adafruit Trinket 3V

