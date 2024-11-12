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

#include "mpu9250.h"
#include "EEPROM.h"

// IMU Declares
#define IMU_BUS       Wire  //Wire // SPI
#define IMU_SCL       19  //47 // 0x255
#define IMU_SDA       18  //48 // 0x255
#define IMU_SPD       400000  //1000000 // 0==NULL or other
#define IMU_SRD        9   // Used in initIMU setSRD() - setting SRD to 9 for a 100 Hz update rate
#define IMU_INT_PIN    1  //50 // 1 // 14
//#include <Wire.h>

bfs::Mpu9250 Imu;


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
  Wire2.begin();
  Wire2.setClock(400000);

  Serial.println("Beginning IMU Initialization...");
  Imu.Config(&Wire2, bfs::Mpu9250::I2C_ADDR_PRIM);

  if (!Imu.Begin()) {
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
  // setting a 20 Hz DLPF bandwidth
  Imu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 9 for a 100 Hz update rate
  Imu.ConfigSrd(IMU_SRD);

  // Read and print current calibration values
  printEEPROMBiases();
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
  Serial.println("  Enter 'e' to load static cal values to EEPROM");
  Serial.println("  Enter 'i' to load static cal values to IMU");
  Serial.println("  Enter 'l' to load MPU Library cal values to EEPROM");
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
      Serial.println("Printing EEPROM:"); printEEPROMBiases();
      Serial.println("Printing IMU:"); printIMUBiases();
    }
    else if (rx_byte == 'z'){
      zeroCalValues();
    }
    else if (rx_byte == 's'){
      staticCal();
    }
    else if (rx_byte == 'p'){
      serialPrintFlag = !serialPrintFlag;
    }
    else if (rx_byte == 'e'){
      loadSCBiasesEEPROM(); // load static biases into EEPROM
    }
    else if (rx_byte == 'i'){
      loadBiasesIMU(); // load static biases into IMU
    }
    else if (rx_byte == 'l'){
      loadLibBiasesEEPROM(); // load MPU Lib biases into EEPROM
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
