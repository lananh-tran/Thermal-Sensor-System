# Thermal Sensor System

by Lan-Anh Tran

_Many thanks to Carter Trudeau and Kyle Chen for their help in this project._

## Introduction

The purpose of this module is to take measurements of the surrounding ambient temperature and inner surface temperature of the rocket before and during flight for simulation purposes. The components used in this project can be viewed in the [Bill of Materials](https://github.com/lananh-tran/Thermal-Sensor-System/blob/1d724febd7ca4944d8594dc0ccbd3dcf55217213/Thermal%20Sensor%20System%20BOM%202025.xlsx), or shown below:
- [Adafruit MAX31865 PT100 RTD Temperature Sensor Amplifier](https://www.adafruit.com/product/3328)
- [Adafruit MCP9808 I2C Temperature Sensor Breakout Board](https://www.adafruit.com/product/1782)
- [MPU6050 Analog Gyro Sensors and Accelerometer Module](https://www.amazon.com/Gy-521-MPU-6050-MPU6050-Sensors-Accelerometer/dp/B008BOPN40?sr=8-2)
- [HiLetgo Micro SD TF Card Adapter Reader Module](https://www.amazon.com/HiLetgo-Adater-Interface-Conversion-Arduino/dp/B07BJ2P6X6?sr=8-1)
- [Battery Charger Module](https://www.amazon.com/dp/B098989NRZ?amp=&amp=)
- [Samsung 35E 18650 3500mAh 8A - Button Top Battery](https://www.18650batterystore.com/products/samsung-35e-button-top)
- [Arduino Nano Microcontroller](https://www.amazon.com/LAFVIN-Board-ATmega328P-Micro-Controller-Arduino/dp/B07G99NNXL?sr=8-1)
- [Surface Mount RTD PT100 Temperature Sensor](https://evosensors.com/products/p3a-tape-rec-px-1-pfxx-40-stwl?variant=29631284299&country=US&currency=USD&gQT=1)

## Design Procedure

Below is the schematic for the sensor system:
![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/3b80351effe68cb9232d9420fbba5fe1baf3be82/Design%20schematic.PNG)

Except for the RTD sensor, the battery charger module, and the 18650 battery, the rest are mounted on a PCB.

The MCP9808 and the MPU6050 both utilize the I2C communication protocol, while the MAX31865 and the microSD Card Adapter use the SPI protocol. Though we couldn't find a particular reason why there aren't any I2C breakout boards for the MAX31865, we assume the microSD Card Adapter has to use SPI due to its need for speed. This resulted in a communication issue, which will be discussed in the Results section.

## Assembly Process



## Results



## References



## Appendix
