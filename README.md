# Thermal Sensor System

by Lan-Anh Tran

_Many thanks to Carter Trudeau and Kyle Chen for their help in this project._

## Introduction

The purpose of this module is to take measurements of the surrounding ambient temperature and inner surface temperature of the rocket before and during flight for simulation purposes. The components used in this project can be viewed in the [Bill of Materials](https://github.com/lananh-tran/Thermal-Sensor-System/blob/1d724febd7ca4944d8594dc0ccbd3dcf55217213/Thermal%20Sensor%20System%20BOM%202025.xlsx) or shown below:
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

I2C devices communicate through two lines - the clock (SCL) and the data line (SDA). An I2C master can be connected to different slaves, as long as their SDA lines are connected and their SCL lines are connected.

SPI devices require more connections, with a clock line (SCK or CLK), Master-In-Slave-Out (MISO or SDO), Master-Out-Slave-In (MOSI or SDI), and a Chip Select line (CS). Multiple SPI slaves can have their CLK, MISO, and MOSI lines connected to the corresponding lines of their master, but each slave has to be connected to a different CS pin on their master.

In order to mitigate the potential wiring errors, we opt to design and fabricate a custom PCB of size 6 x 1.6 in, with a drill hole diameter of 0.1 in. The design is done in KiCad, which is a powerful and free PCB design software that is quite popular. KiCad also allows 3D modelling, which is quite convenient if you want to make sure the electrical design is compatible with the mechanical structure.

![image]()
![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/2986bce716af941466dec6310956a8af12dea283/PCB%20Layout.jpeg)
![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/2986bce716af941466dec6310956a8af12dea283/PCB%203D%20Model.jpeg)

## Assembly Process

The assembly process is quite straightforward. After soldering all the components onto the PCB, connect the RTD sensor to the MAX31865 breakout board as shown below, then connect the battery and charger module as demonstrated in the system schematic posted above.

![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/8d93a0bf2f5ad6d971b894c0a99290d112e9a91c/MAX31865%203_wired.jpg)

More details for the MAX31865 wiring can be viewed on [this site](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier/rtd-wiring-config).

## Results

Below is the final product before it was sent to IREC 2025 for competition:
![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/2986bce716af941466dec6310956a8af12dea283/PCB%20Fabricated.jpg)
![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/2986bce716af941466dec6310956a8af12dea283/PCB%20Soldered%20components.jpg)

## References



## Appendix
