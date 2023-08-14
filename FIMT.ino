#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3MDL.h>

#define DHTPIN 6 
#define DHTTYPE DHT11

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
Adafruit_LIS3MDL lis3mdl;
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);

  if (!bmp.begin()) {
  Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                    "try a different address!"));
  Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
  Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  Serial.print("        ID of 0x60 represents a BME 280.\n");
  Serial.print("        ID of 0x61 represents a BME 680.\n");
  while (1) delay(10);
  }

  dht.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  if (! lis3mdl.begin_I2C()) { 
 
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }

  // Settings

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
  case MPU6050_RANGE_4_G:
    break;
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_0_625_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
  
  delay(100);
}

void loop() {
  delay(3000);

  lis3mdl.read();
  sensors_event_t a, g, temp;
  sensors_event_t event;
  mpu.getEvent(&a, &g, &temp);
  lis3mdl.getEvent(&event);
  
  float DHTHumidity = dht.readHumidity();
  float DHTTemp = dht.readTemperature();
  float DHTIndex = dht.computeHeatIndex(DHTTemp, DHTHumidity, false);
  float BMPTemp = bmp.readTemperature();
  float BMPPres = bmp.readPressure();
  float BMPAlt = bmp.readAltitude(1013.25);
  float IMUAcc_x = a.acceleration.x;
  float IMUAcc_y = a.acceleration.y;
  float IMUAcc_z = a.acceleration.z;
  float IMUGyro_x = g.gyro.x;
  float IMUGyro_y = g.gyro.y;
  float IMUGyro_z = g.gyro.z;
  float IMUTemp = temp.temperature;
  float MAGRaw_x = lis3mdl.x;
  float MAGRaw_y = lis3mdl.y;
  float MAGRaw_z = lis3mdl.z;
  float MAGTesla_x = event.magnetic.x;
  float MAGTesla_y = event.magnetic.y;
  float MAGTesla_z = event.magnetic.z;

  Serial.print(DHTHumidity);
  Serial.print(",");
  Serial.print(DHTTemp);
  Serial.print(",");
  Serial.print(DHTIndex);
  Serial.print(",");
  Serial.print(BMPTemp);
  Serial.print(",");
  Serial.print(BMPPres);
  Serial.print(",");
  Serial.print(BMPAlt);
  Serial.print(",");
  Serial.print(IMUAcc_x);
  Serial.print(",");
  Serial.print(IMUAcc_y);
  Serial.print(",");
  Serial.print(IMUAcc_z);
  Serial.print(",");
  Serial.print(IMUGyro_x);
  Serial.print(",");
  Serial.print(IMUGyro_y);
  Serial.print(",");
  Serial.print(IMUGyro_z);
  Serial.print(",");
  Serial.print(IMUTemp);
  Serial.print(",");
  Serial.print(MAGRaw_x);
  Serial.print(",");
  Serial.print(MAGRaw_y);
  Serial.print(",");
  Serial.print(MAGRaw_z);
  Serial.print(",");
  Serial.print(MAGTesla_x);
  Serial.print(",");
  Serial.print(MAGTesla_y);
  Serial.print(",");
  Serial.println(MAGTesla_z);
}
