/*
CalibrateMPU9250.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//////////////////////////////////////////////////////////////////////////////////////
Calibration Instructions and Notes.

Gyro bias calibration is typically accomplished each time the IMU initializes (powers on).
Accelerometer and magnetometer bias and scale factor calibration is typically accomplished 
prior to IMU operations, and the values are stored in EEPROM. 

This program performs calibration for gyro bias, accelerometer bias and scale factor, and
magnetometer bias and scale factor. The gyro bias calibration is really just provided so
that user can verify it will perform as expected upon IMU startup. The accel and mag
calibrations are written to EEPROM where they can be retrieved upon IMU startup.
////////////////////////////////////////////////////////////////////////////////////
*/
//for giga
#include <LibPrintf.h>
#include <avr/dtostrf.h>

//Select BFS IMU
//#define MPU9250
#define ICM20948
//#define ICM20649
//#define MPU6500
//#define MPU6050

//Separate Magnetometer test
//Uncomment EXTMAG and one of the magnetometometers upported
//#define EXTMAG
//#define HMC5983A
//#define LIS3MDLA

#if defined(MPU9250)
#include "mpu9250.h"
#define IMU_TYPE bfs::Mpu9250
#define IMU_ADDR I2C_ADDR_PRIM
bfs::Mpu9250 Imu;

#elif defined(ICM20948)
#include "icm20948.h"
#include "ak09916.h"
#define IMU_TYPE bfs::Icm20948
#define IMU_ADDR I2C_ADDR_SEC //Sparkfun Breakout is on Secondary
bfs::Icm20948 Imu;
bfs::Ak09916 mag;

#elif defined(ICM20649)
#include "icm20649.h"
#define IMU_TYPE bfs::Icm20649
#define IMU_ADDR I2C_ADDR_PRIM
bfs::Icm20649 Imu;

#elif defined(MPU6500)
#include "mpu6500.h"
#define IMU_ADDR I2C_ADDR_PRIM
#define IMU_TYPE bfs::Mpu6500
bfs::Mpu6500 Imu;

#elif defined(MPU6050)
#include "mpu6050.h"
#define IMU_TYPE bfs::Mpu6050
#define IMU_ADDR I2C_ADDR_PRIM
bfs::Mpu6050 Imu;
#endif

#if defined(HMC5983A)
#include "HMC5983.h"
HMC5983 mag(&Wire);
#elif defined(LIS3MDLA)
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
Adafruit_LIS3MDL mag;
#endif

// IMU Declares
#define IMU_BUS       Wire  //Wire // SPI
#define IMU_SPD       400000  //1000000 // 0==NULL or other
  /* Set the sample rate divider for both accelerometer and gyroscope*/
  /* these settings apply to ICM20948
    rate = 1125/(SRD + 1) HZ
    ==========================
    SRD 4  => rate = 225 HZ
    SRD 5  => rate = 187.5 HZ
    SRD 8  => rate = 125 HZ
    SRD 9  => rate = 112.5 HZ
    SRD 10 => rate = 102.3 HZ
    SRD 11 => rate = 93.75 HZ
    SRD 19 => rate = 56.25 HZ
    SRD 20 => rate = 53,57 HZ
    SRD 21 => rate = 51,14 HZ
    SRD 22 => rate = 48.91 HZ
    when using SRD to set sampling rate magnetometer is 
    automatically set to 50Hz for SRD's > 10 otherwise it
    is set to 100Hz.
   */
#define IMU_SRD       9   // Used in initIMU setSRD() - setting SRD to 9 for a 100 Hz update rate

int status;

// EEPROM buffer and variable to save accel and mag bias and scale factors
uint8_t EepromBuffer[48];
float value;  // temp variable for loading into EEPROM buffer
char rx_byte = 0;
float axb, axs=1.0f, ayb, ays=1.0f, azb, azs=1.0f;
float hxb, hxs=1.0f, hyb, hys=1.0f, hzb, hzs=1.0f;
float gxb, gyb, gzb;
bool serialPrintFlag = false;
const float G = -9.80665;

// Set the values below for the circular buffer size (bufSize) and
// the number of loops to execute for passing data to the circular buffer.
const int bufSize = 2048; //64; // Must be power of 2
int numIter = 2070; //70; // Must be greater than bufSize

// Magnetic Field Strength in NED frame for Whitestone, NY
//   Reference: ngdc.noaa.gov
const float hNorth = 20.4631;
const float hEast = -4.5981;
const float hDown = 46.5087 ;

void setup() {
  // initialize serial to display instructions
  Serial.begin(115200);
  while(!Serial && millis() < 5000) {}
  // start communication with IMU 
  Serial.println(" ");
  /* Start the I2C bus */
  IMU_BUS.begin();
  IMU_BUS.setClock(IMU_SPD);

  Serial.println("Beginning IMU Initialization...");

  Imu.Config(&IMU_BUS, IMU_TYPE::IMU_ADDR);
  #if defined(ICM20948)

  mag.Config(&IMU_BUS);
  #endif

  #if defined(ICM20948)
  if (!Imu.Begin(bfs::Icm20948::MAG_PASSTHROUGH)) {
  #else
  if (!Imu.Begin()) {
  #endif
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);    while(1) {}
  }
  Serial.println("IMU Initialization Complete");
  Serial.println(" ");

  // provide warning if IMU does not initialize properly
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    while(1) {}
  }

  // set IMU operating parameters
  #if defined(MPU9250)
  // setting a 20 Hz DLPF bandwidth
  Imu.ConfigDlpfBandwidth(IMU_TYPE::DLPF_BANDWIDTH_20HZ);
  Imu.ConfigAccelRange(IMU_TYPE::ACCEL_RANGE_8G);
  Imu.ConfigGyroRange(IMU_TYPE::GYRO_RANGE_250DPS);

  #elif defined(ICM20948)
  Imu.ConfigGyroDlpfBandwidth(IMU_TYPE::GYRO_DLPF_BANDWIDTH_23HZ);
  Imu.ConfigAccelDlpfBandwidth(IMU_TYPE::ACCEL_DLPF_BANDWIDTH_23HZ);
  Imu.ConfigAccelRange(IMU_TYPE::ACCEL_RANGE_8G);
  Imu.ConfigGyroRange(IMU_TYPE::GYRO_RANGE_250DPS);

  #elif defined(ICM20649)
  Imu.ConfigGyroDlpfBandwidth(IMU_TYPE::GYRO_DLPF_BANDWIDTH_23HZ);
  Imu.ConfigAccelDlpfBandwidth(IMU_TYPE::ACCEL_DLPF_BANDWIDTH_23HZ);
  Imu.ConfigAccelRange(IMU_TYPE::ACCEL_RANGE_8G);
  Imu.ConfigGyroRange(IMU_TYPE::GYRO_RANGE_500DPS);

  #elif defined(MPU6050)
  Imu.ConfigAccelRange(IMU_TYPE::ACCEL_RANGE_8G);
  Imu.ConfigGyroRange(IMU_TYPE::GYRO_RANGE_250DPS);
  Imu.ConfigDlpfBandwidth(IMU_TYPE::DLPF_BANDWIDTH_20HZ);
  #endif

  #if defined(HMC5983)
  if(!mag.Begin()) {
    Serial.println("Mag failed to start!!");
    while(1) {}
  };
  mag.setRange(HMC5983_RANGE_8_1GA);
  mag.setMeasurementMode(HMC5983_CONTINOUS);
  mag.setSampleAverages(HMC5983_SAMPLEAVERAGE_8);
  mag.setDataRate(HMC5983_DATARATE_75HZ);

  #elif defined(LIS3MDLA)
  if (! mag.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");
  mag.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
  mag.setRange(LIS3MDL_RANGE_4_GAUSS);
  mag.setIntThreshold(500);
  mag.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
  #endif
  
  // setting SRD to 9 for a 100 Hz update rate
  Imu.ConfigSrd(IMU_SRD);

  #if defined(ICM20948)
  /* MAG */
  if (!mag.Begin()) {
    Serial.println("Error initializing communication with MAG");
    while (1) {}
  }
  #endif

  // Read and print current calibration values
  printIMUBiases();
  
  // tell user the command options
  Serial.println("Enter one of the following commands using the serial terminal: ");
  Serial.println("  Enter 'a' to preform MPU library accel calibrations");
  Serial.println("  Enter 'm' to perform MPU library mag calibrations");
  Serial.println("  Enter 'g' to perform MPU library gyro calibrations");
  Serial.println("  Enter 's' to perform static IMU bias calibrations");
  Serial.println("  Enter 'd' to display all calibration values");
  Serial.println("  Enter 'z' to reset all calibration values to zero");
  Serial.println("  Enter 'p' to print corrected IMU readings to serial");
  Serial.println("  Enter 'i' to load static cal values to IMU");
  Serial.println("  Enter 'r' to calculate IMU sensor noise sigmas");
  Serial.println(" ");
} // end setup loop

void loop() {
  
  // wait for user command
  if (Serial.available()>0) {
    rx_byte = Serial.read();

    // read user command and execute proper routine
    if (rx_byte == 'a'){
      accelerometerCal();
    }
    else if (rx_byte == 'm'){
      magnetometerCal();
    }
    else if (rx_byte == 'g'){
      gyroCal();
    }
    else if (rx_byte == 'd'){
      Serial.println("Printing IMU:"); printIMUBiases();
    }
    else if (rx_byte == 's'){
      staticCal();
    }
    else if (rx_byte == 'p'){
      serialPrintFlag = !serialPrintFlag;
    }
    else if (rx_byte == 'i'){
      loadBiasesIMU(); // load static biases into IMU
    }
    else if (rx_byte == 'r'){
      noiseLevelsIMU(); // calculate noise sigma for all IMU sensors
    }
  }

  if (serialPrintFlag){
    Imu.Read();
    serialPrint();  
    delay(200); // Update rate for streaming to serial
  }
} // End main void loop
