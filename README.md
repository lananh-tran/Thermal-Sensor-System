# Thermal Sensor System

by Lan-Anh Tran

_Many thanks to Carter Trudeau and Kyle Chen for their help in this project._

## Introduction

The purpose of this module is to take measurements of the surrounding ambient temperature and inner surface temperature of our rocket - the Wildcat V - before and during flight for simulation purposes. The components used in this project can be viewed in the [Bill of Materials](https://github.com/lananh-tran/Thermal-Sensor-System/blob/1d724febd7ca4944d8594dc0ccbd3dcf55217213/Thermal%20Sensor%20System%20BOM%202025.xlsx) or shown below:

- [Adafruit MAX31865 PT100 RTD Temperature Sensor Amplifier](https://www.adafruit.com/product/3328)
- [Adafruit MCP9808 I2C Temperature Sensor Breakout Board](https://www.adafruit.com/product/1782)
- [MPU6050 Analog Gyro Sensors and Accelerometer Module](https://www.amazon.com/Gy-521-MPU-6050-MPU6050-Sensors-Accelerometer/dp/B008BOPN40?sr=8-2)
- [HiLetgo Micro SD TF Card Adapter Reader Module](https://www.amazon.com/HiLetgo-Adater-Interface-Conversion-Arduino/dp/B07BJ2P6X6?sr=8-1)
- [Battery Charger Module](https://www.amazon.com/dp/B098989NRZ?amp=&amp=)
- [Samsung 35E 18650 3500mAh 8A - Button Top Battery](https://www.18650batterystore.com/products/samsung-35e-button-top)
- [Arduino Nano Microcontroller](https://www.amazon.com/LAFVIN-Board-ATmega328P-Micro-Controller-Arduino/dp/B07G99NNXL?sr=8-1)
- [Surface Mount RTD PT100 Temperature Sensor](https://evosensors.com/products/p3a-tape-rec-px-1-pfxx-40-stwl?variant=29631284299&country=US&currency=USD&gQT=1)

## Design Procedure

Below is the rough design for the sensor system. See [design schematic](https://github.com/lananh-tran/Thermal-Sensor-System/blob/db7faa17b55e4ecaf4507a09ca1c67294a6be1d8/Thermal%20System%20Schematic.pdf) and [PCB layout](https://github.com/lananh-tran/Thermal-Sensor-System/blob/db7faa17b55e4ecaf4507a09ca1c67294a6be1d8/Thermal%20System%20PCB.pdf) for more details.

![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/3b80351effe68cb9232d9420fbba5fe1baf3be82/Design%20schematic.PNG)

Except for the RTD sensor, the battery charger module, and the 18650 battery, the rest are mounted on a PCB.

The MCP9808 and the MPU6050 both utilize the I2C communication protocol, while the MAX31865 and the microSD Card Adapter use the SPI protocol. Though we couldn't find a particular reason why there aren't any I2C breakout boards for the MAX31865, we assume the microSD Card Adapter has to use SPI due to its need for speed. This resulted in a communication issue, which will be discussed in the Results section.

I2C devices communicate through two lines - the clock (SCL) and the data line (SDA). An I2C master can be connected to different slaves, as long as their SDA lines are connected and their SCL lines are connected.

SPI devices require more connections, with a clock line (SCK or CLK), Master-In-Slave-Out (MISO or SDO), Master-Out-Slave-In (MOSI or SDI), and a Chip Select line (CS). Multiple SPI slaves can have their CLK, MISO, and MOSI lines connected to the corresponding lines of their master, but each slave has to be connected to a different CS pin on their master.

In order to mitigate the potential wiring errors, we opt to design and fabricate a custom PCB of size 6 x 1.6 in, with a drill hole diameter of 0.1 in. The design is done in KiCad, which is a powerful and free PCB design software. KiCad also allows 3D modelling, which is quite convenient if you want to make sure the electrical design is compatible with the mechanical structure.

![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/2986bce716af941466dec6310956a8af12dea283/PCB%203D%20Model.jpeg)

## Assembly Process

The assembly process is quite straightforward. After soldering all the components onto the PCB, connect the RTD sensor to the MAX31865 breakout board as shown below, then connect the battery and charger module as demonstrated in the system schematic posted above.

![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/8d93a0bf2f5ad6d971b894c0a99290d112e9a91c/MAX31865%203_wired.jpg)

More details about the MAX31865 wiring can be viewed on [Adafruit's official guide](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier/rtd-wiring-config).

## Results

Below is the final product before it was sent to IREC 2025 for competition:

![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/2986bce716af941466dec6310956a8af12dea283/PCB%20Fabricated.jpg)
![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/2986bce716af941466dec6310956a8af12dea283/PCB%20Soldered%20components.jpg)

A critical issue we have with this module is the design of the microSD Adapter Module, which leads to the SPI bus not getting off the SPI bus. We utilized the logic analyzer function of the ADALM2000 module to observe the behavior of the system.

Results when the SD module is connected:
![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/631a0708d12d0355a09fba3c9a527eef10d72604/SD%20reader%20connected.png)

When the SD module isn't connected:
![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/631a0708d12d0355a09fba3c9a527eef10d72604/SD%20reader%20not%20connected.png)

After a close inspection of the SD module schematic, we suspect that the MISO might be pulled to GND by default by the buffer, leading to the issue with the SPI bus.
![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/631a0708d12d0355a09fba3c9a527eef10d72604/SD%20reader%20schematic.png)

We decided to follow a modification suggested on the Arduino Forum (source lost), as shown below.
![image](https://github.com/lananh-tran/Thermal-Sensor-System/blob/631a0708d12d0355a09fba3c9a527eef10d72604/microSD%20card%20reader%20modification.png)

## Code

```
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_MAX31865.h>

// --- CONFIGURATION ---
int timeStamp = 1;
#define SD_CS 4
//SdFat SD;
File dataLog;

float acceleration;
Adafruit_MPU6050 mpu;

float ambient;
Adafruit_MCP9808 ambientTemp = Adafruit_MCP9808();

#define RREF 430.0 // Rref resistor for PT100
#define RNOMINAL 100.0 // Nominal resistance for PT100
#define RTD_CS 10
#define MOSI 11
#define MISO 12
#define CLK 13
float surface;
Adafruit_MAX31865 surfaceTemp = Adafruit_MAX31865(RTD_CS);

void setup() {
  // set serial monitor baud rate
  Serial.begin(9600);
  while(!Serial) {;}

  // MCP9808 initialization
  MCP_init();

  // MPU6050 initialization
  MPU_init();

  // SD card initialization
  SD_init();

  // MAX31865 initialization
  pinMode(RTD_CS, OUTPUT);
  digitalWrite(RTD_CS, HIGH);
  surfaceTemp.begin(MAX31865_3WIRE);
  delay(1000);

  // SPI communication setup
  SPI.begin();

  Serial.println("Begin testing:");
}

void loop() {
  // --- MCP9808 Sampling ---
  MCP_sample();

  // testing
  Serial.print("Ambient temp: "); 
  Serial.print(ambient, 3);
  Serial.println("*C");

  // --- MAX31865 Sampling ---
  MAX_sample();

  // testing
  Serial.print("Surface temp: "); 
  Serial.print(surface, 3);
  Serial.println("*C");

  // --- MPU6050 Sampling ---
  MPU_sample();

  // testing
  Serial.print("Acceleration: "); 
  Serial.print(acceleration, 3);
  Serial.println("m/s^2");

  // --- SD card reader ---
  SD_write();

  delay(1000); // take readings every 1s
}

void MCP_init(void) {
  //  A2 A1 A0 address
  //  0  0  0   0x18  this is the default address
  //  0  0  1   0x19
  //  0  1  0   0x1A
  //  0  1  1   0x1B
  //  1  0  0   0x1C
  //  1  0  1   0x1D
  //  1  1  0   0x1E
  //  1  1  1   0x1F
  if(!ambientTemp.begin(0x18)) {
	  Serial.println("Unable to connect to the MCP9808!");
	  Serial.println("Check your connections and verify the address is correct.");
	  while (1);
  }

  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
  ambientTemp.setResolution(3);
  //Serial.println("MCP9808 initialized!");
  delay(1000); // Give sensor time to stabilize
}

void MPU_init(void) {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000); // Give sensor time to stabilize
}

void SD_init(void) {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  //Serial.println("Initializing SD card...");
  if(!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    //while(1) {;}
  }
  //Serial.println("Card initialized!");

  dataLog = SD.open("DATALOG.csv", FILE_WRITE);
  if(!dataLog) {
    Serial.println("Failed to write!");
    //while(1) {;}
  }
  dataLog.println("no.,ambientTemp(C),surfaceTemp(C),acceleration(m/s^2)");
}

void MCP_sample(void) {
  ambientTemp.wake();
  delay(200);
  ambient = ambientTemp.readTempC();
  ambientTemp.shutdown_wake(1);
  delay(200);
}

void SD_write(void) {
  digitalWrite(RTD_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  
  if(dataLog) {
    dataLog.print(timeStamp);
    dataLog.print(",");
    dataLog.print(ambient, 3);
    dataLog.print(",");
    dataLog.print(surface, 3);
    dataLog.print(",");
    dataLog.println(acceleration, 3);
    
    dataLog.flush();
    delay(200);
  }

  digitalWrite(SD_CS, HIGH);
  timeStamp++;
  Serial.println("Writing done");
}

void MAX_sample(void) {
  digitalWrite(SD_CS, HIGH);
  digitalWrite(RTD_CS, LOW);
  surface = surfaceTemp.temperature(RNOMINAL, RREF);
  delay(200);
  digitalWrite(RTD_CS, HIGH);
}

void MPU_sample(void) {
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  acceleration = sqrt(a.acceleration.x * a.acceleration.x +
                        a.acceleration.y * a.acceleration.y +
                        a.acceleration.z * a.acceleration.z) - 9.81; // account for Earth stationary acceleration
}
```
