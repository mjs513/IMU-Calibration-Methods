////////////////////////////////////////////////////////////////////
//
// IMU calibration (static calibration method)
//
// Note: IMU must be level-set, and facing magnetic North
//
////////////////////////////////////////////////////////////////////

// These lines set up the circular buffers. Must be outside of the
// void serialPrintMag routine.
//#include <SPI_MSTransfer.h>
#include "circular_buffer.h"
//const int bufSize = 64; // Must be power of 2
//int numIter = 70; // Must be greater than bufSize
Circular_Buffer<float, bufSize> cbaX;
Circular_Buffer<float, bufSize> cbaY;
Circular_Buffer<float, bufSize> cbaZ;
Circular_Buffer<float, bufSize> cbgX;
Circular_Buffer<float, bufSize> cbgY;
Circular_Buffer<float, bufSize> cbgZ;
Circular_Buffer<float, bufSize> cbhX;
Circular_Buffer<float, bufSize> cbhY;
Circular_Buffer<float, bufSize> cbhZ;

void staticCal(){

  Serial.println("Wait... Conducting IMU static calibration...");
  for (int i=0; i<numIter; i++){
    Imu.Read();

    cbaX.push_back(Imu.accel_x_mps2());
    cbaY.push_back(Imu.accel_y_mps2());
    cbaZ.push_back(Imu.accel_z_mps2()-G);
    cbgX.push_back(Imu.gyro_x_radps());
    cbgY.push_back(Imu.gyro_y_radps());
    cbgZ.push_back(Imu.gyro_z_radps());
    
    #if defined(MPU9250)
    cbhX.push_back(Imu.mag_x_ut()-hNorth);
    cbhY.push_back(Imu.mag_y_ut()-hEast);
    cbhZ.push_back(Imu.mag_z_ut()-hDown);
    
    #elif defined(ICM20948)
    mag.Read();
    cbhX.push_back(mag.mag_x_ut()-hNorth);
    cbhY.push_back(mag.mag_y_ut()-hEast);
    cbhZ.push_back(mag.mag_z_ut()-hDown);
    
    #elif defined(HMC5983A)
    float mag_val[3];
    mag.getMagScaled(mag_val);
    cbhX.push_back(-mag_val[0]-hNorth);
    cbhY.push_back(mag_val[1]-hEast);
    cbhZ.push_back(-mag_val[2]-hDown);

    #elif defined(LIS3MDLA)
  /* Or....get a new sensor event, normalized to uTesla */
    sensors_event_t event; 
    mag.getEvent(&event);
    cbhX.push_back(event.magnetic.x-hNorth);
    cbhY.push_back(event.magnetic.y-hEast);
    cbhZ.push_back(event.magnetic.z-hDown);

    #else
    cbhX.push_back(0);
    cbhY.push_back(0);
    cbhZ.push_back(0);
    #endif
    delayMicroseconds(200);
  }
  
  axb = cbaX.mean();
  ayb = cbaY.mean();
  azb = cbaZ.mean();
  gxb = cbgX.mean();
  gyb = cbgY.mean();
  gzb = cbgZ.mean();
  hxb = cbhX.mean();
  hyb = cbhY.mean();
  hzb = cbhZ.mean();

  axs = 1.0f; ays = 1.0f; azs = 1.0f;
  hxs = 1.0f; hys = 1.0f; hzs = 1.0f;
  
  Serial.println("Static Calibration Bias Values:");
  Serial.print("axb: "); Serial.println(axb,6);
  Serial.print("ayb: "); Serial.println(ayb,6);
  Serial.print("azb: "); Serial.println(azb,6);
  Serial.print("gxb: "); Serial.println(gxb,6);
  Serial.print("gyb: "); Serial.println(gyb,6);
  Serial.print("gzb: "); Serial.println(gzb,6);
  Serial.print("hxb: "); Serial.println(hxb,6);
  Serial.print("hyb: "); Serial.println(hyb,6);
  Serial.print("hzb: "); Serial.println(hzb,6);
  Serial.println(" ");
}

void noiseLevelsIMU(){

  float axbSigma, aybSigma, azbSigma;
  float gxbSigma, gybSigma, gzbSigma;
  float hxbSigma, hybSigma, hzbSigma;
  
  Serial.println("Wait... Conducting IMU noise sigma calculation...");
  for (int i=0; i<numIter; i++){
    Imu.Read();

    cbaX.push_back(Imu.accel_x_mps2());
    cbaY.push_back(Imu.accel_y_mps2());
    cbaZ.push_back(Imu.accel_z_mps2());
    cbgX.push_back(Imu.gyro_x_radps());
    cbgY.push_back(Imu.gyro_y_radps());
    cbgZ.push_back(Imu.gyro_z_radps());
    
    #if defined(MPU9250)
    cbhX.push_back(Imu.mag_x_ut());
    cbhY.push_back(Imu.mag_y_ut());
    cbhZ.push_back(Imu.mag_z_ut());
    
    #elif defined(ICM20948)
    mag.Read();
    cbhX.push_back(mag.mag_x_ut());
    cbhY.push_back(mag.mag_y_ut());
    cbhZ.push_back(mag.mag_z_ut());
    
    #elif defined(HMC5983A)
    float mag_val[3];
    mag.getMagScaled(mag_val);
    cbhX.push_back(-mag_val[0]);
    cbhY.push_back(mag_val[1]);
    cbhZ.push_back(-mag_val[2]);

    #elif defined(LIS3MDLA)
  /* Or....get a new sensor event, normalized to uTesla */
    sensors_event_t event; 
    mag.getEvent(&event);
    cbhX.push_back(event.magnetic.x-hNorth);
    cbhY.push_back(event.magnetic.y-hEast);
    cbhZ.push_back(event.magnetic.z-hDown);
    
    #else
    cbhX.push_back(0);
    cbhY.push_back(0);
    cbhZ.push_back(0);
    #endif

    delayMicroseconds(200);
  }
  
  axbSigma = cbaX.deviation();
  aybSigma = cbaY.deviation();
  azbSigma = cbaZ.deviation();
  gxbSigma = cbgX.deviation();
  gybSigma = cbgY.deviation();
  gzbSigma = cbgZ.deviation();
  hxbSigma = cbhX.deviation();
  hybSigma = cbhY.deviation();
  hzbSigma = cbhZ.deviation();
  
  Serial.println("Static IMU Sensor Sigma Values:");
  Serial.print("axb Sigma (m/ss):  "); Serial.print(axbSigma,6);
  Serial.print("  axMax: ");Serial.print(cbaX.max(),6);
  Serial.print("  axMin: ");Serial.println(cbaY.min(),6);
  
  Serial.print("ayb Sigma (m/ss):  "); Serial.print(aybSigma,6);
  Serial.print("  ayMax: ");Serial.print(cbaY.max(),6);
  Serial.print("  ayMin: ");Serial.println(cbaY.min(),6);
  
  Serial.print("azb Sigma (m/ss):  "); Serial.print(azbSigma,6);
  Serial.print("  azMax: ");Serial.print(cbaZ.max(),6);
  Serial.print("  azMin: ");Serial.println(cbaZ.min(),6);
  
  Serial.print("gxb Sigma (deg/s): "); Serial.print(gxbSigma,6);
  Serial.print("  gxMax: ");Serial.print(cbgX.max(),6);
  Serial.print("  gxMin: ");Serial.println(cbgX.min(),6);
  
  Serial.print("gyb Sigma (deg/s): "); Serial.print(gybSigma,6);
  Serial.print("  gyMax: ");Serial.print(cbgY.max(),6);
  Serial.print("  gyMin: ");Serial.println(cbgY.min(),6);
  
  Serial.print("gzb Sigma (deg/s): "); Serial.print(gzbSigma,6);
  Serial.print("  gzMax: ");Serial.print(cbgZ.max(),6);
  Serial.print("  gzMin: ");Serial.println(cbgZ.min(),6);
  
  Serial.print("hxb Sigma (uT):    "); Serial.print(hxbSigma,6);
  Serial.print("  hxMax: ");Serial.print(cbhX.max(),6);
  Serial.print("  hxMin: ");Serial.println(cbhX.min(),6);
  
  Serial.print("hyb Sigma (uT):    "); Serial.print(hybSigma,6);
  Serial.print("  hyMax: ");Serial.print(cbhY.max(),6);
  Serial.print("  hyMin: ");Serial.println(cbhY.min(),6);
  
  Serial.print("hzb Sigma (uT):    "); Serial.print(hzbSigma,6);
  Serial.print("  hzMax: ");Serial.print(cbhZ.max(),6);
  Serial.print("  hzMin: ");Serial.println(cbhZ.min(),6);
  
  Serial.println(" ");
}


