// Include Modified Bolderflight invensense-imu library
#include "mpu9250.h"

/* Mpu9250 object */
bfs::Mpu9250 imu;
uint32_t time_old = 0;

//Freeimu calibration
int cal_acc_off[] = {-37, 30, 130};
int cal_acc_scale[]  = {2047, 2060, 2065};
int cal_magn_off[]  = {99, 16, -186};
int cal_magn_scale[]  = {239, 246, 263};

//Magneto 1.2 Calibration 
float MagOffset1[3] = {17.913548, 3.018422, -32.074879}; 
float mCal1[3][3] = 
{
  {1.038134, 0.001051, -0.007501},
  {0.001051, +1.039582, 0.007319},
  {-0.007501, 0.007319, +1.008515}
};

float AccOffset1[3] = {-0.008955, 0.012518, 0.062580}; 
float aCal1[3][3] = 
{
  {-0.008955, 0.010505, 0.004729},
  {0.010505, 1.012468, 0.002124},
  {0.004729, 0.002124, 0.995591}
};

//MotionCal (PJRC) calibration
float MagOffset[3] = {17.791,3.680,-32.655};
float mCal[3][3] = 
{
  {1.0034,-0.0051,-0.0025},
  {-0.0051,1.0065,0.0065},
  {-0.0025,0.0065,0.9903}
};

//Min - max
float acc_off[] = {0.093902, 0.093902, -0.310150};
float acc_scale[]  = {1.001197, 0.999423, 1.029498};
float magn_off[]  = {18.221668, 2.135885, -31.471630};
float magn_scale[]  = {1.013987, 1.013153, 0.973922};

//Gravity
#define   G   9.80665
#define   data_count  100

// scale factors
float accelScale, gyroScale;
float magScale[3];

//Averages
float ax_avg, ay_avg, az_avg, gx_avg, gy_avg, gz_avg, mx_avg, my_avg, mz_avg;

void setup() {
  /* Serial to display data */
  while(!Serial && millis() < 5000) {}
  Serial.begin(115200);
  
  // If Teensy 4.x fails print Crashreport to serial monitor
  if (CrashReport) {
      Serial.print(CrashReport);
      Serial.println("Press any key to continue");
      while (Serial.read() != -1) {
      }
      while (Serial.read() == -1) {
      }
      while (Serial.read() != -1) {
      }
  }

  /* Start the I2C bus */
  Wire2.begin();
  Wire2.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire2, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  // rate = 1000 / (srd + 1)
  // = 1000/20 = 50 hz
  // = 100 hz
  if (!imu.ConfigSrd(9)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }

  /* Accelerometer options
    ACCEL_RANGE_2G
    ACCEL_RANGE_4G
    ACCEL_RANGE_8G
    ACCEL_RANGE_16G (Default)
  */
  //imu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_16G);
  /* Gyroscope Options
    GYRO_RANGE_250DPS
    GYRO_RANGE_500DPS
    GYRO_RANGE_1000DPS
    GYRO_RANGE_2000DPS
  */
  //imu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_2000DPS);

  //Get MPU sensitivity values
  imu.getScales(&accelScale, &gyroScale, magScale);
  Serial.print("Accelerometer Scale: "); Serial.println(accelScale, 5);
  Serial.print("Gyro Scale: "); Serial.println(gyroScale, 5);
  Serial.println("Magnetometer Scales:");
  Serial.print("\tMagx: "); Serial.println(magScale[0],5);
  Serial.print("\tMagx: "); Serial.println(magScale[1],5);
  Serial.print("\tMagx: "); Serial.println(magScale[2],5);

}

//The command from the PC
char cmd;
float values[9];
float values1[10];
int16_t raw_values[9];
char str[128];
char str1[128];

void loop() {
  char cmd = '0';
  if(Serial.available()) {
    cmd = Serial.read();
    switch(cmd) {
      case 'v':  // Send handshake
        {
          //sprintf(str, "FreeIMU library by FREQ: LIB_VERSION: %s", FREEIMU_LIB_VERSION);
          Serial.print("OK....");
          Serial.print('\n');
          break;
        }
      case 'b': // Send packed values to GUI
        {
          uint8_t count = serial_busy_wait();
          uint8_t i_count = 0;
          while(i_count < count) {
            imu.Read_raw(raw_values);
            if(imu.new_imu_data()) {
              i_count += 1;
              writeArr(raw_values, 9, sizeof(int16_t));
              //writeArr(raw_values, 6, sizeof(int));
              Serial.println();
            }
          }
          break;
        }
      case 'w': //Send word size to GUI
        {
          //Serial.println(sizeof(int16_t));
          Serial.write(sizeof(int16_t));  //in this case the size of an int16.
          break;
        }
      case 'r':  // Option to print raw values to serial monitor
        {
          uint8_t count = serial_busy_wait();
          uint8_t i_count = 0;
          //for(uint8_t i=0; i<count; i++) {
          while(i_count < count) {
            //my3IMU.getUnfilteredRawValues(raw_values);
            if(imu.Read_raw(raw_values)){
              if(imu.new_imu_data()) {
                i_count += 1;
                sprintf(str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5], raw_values[6], raw_values[7], raw_values[8]);
                Serial.print(str);
                Serial.print(millis()); Serial.print(",");
                Serial.println("\r\n");
                }
            }
          }
          break;
        }
      case 'y':  //FreeIMU_GUI
        {
          uint8_t count = data_count;
          uint8_t i_count = 0;
          //for(uint8_t i=0; i<count; i++) {
          while(i_count < count) {
            if(imu.Read_raw(raw_values)){
              if(imu.new_imu_data()){
                i_count += 1;
                values[1] = ((float)(raw_values[0] - cal_acc_off[0]) / (float)cal_acc_scale[0]);
                values[0] = ((float)(raw_values[1] - cal_acc_off[1]) / (float)cal_acc_scale[1]) ;
                values[2] = -((float)(raw_values[2] - cal_acc_off[2])  / (float)cal_acc_scale[2]) ;

                values[6] = ((float)(raw_values[6] - cal_magn_off[0]) / (float)cal_magn_scale[0]);
                values[7] = ((float)(raw_values[7] - cal_magn_off[1]) / (float)cal_magn_scale[1]) ;
                values[8] = ((float)(raw_values[8] - cal_magn_off[2])  / (float)cal_magn_scale[2]) ;

                ax_avg += values[0]; ay_avg += values[1]; az_avg += values[2];   
                mx_avg += values[6]; my_avg += values[7]; mz_avg += values[8];                        
                //sprintf(str, "%f,%f,%f,%f,%f,%f,%f,%f,%f,", values[0], values[1], -values[2], values[3], values[4], values[5], values[6], values[7], values[8]);
                //Serial.println(str);
              }
            }
          }
          Serial.printf("FreeIMU Avg: %f, %f, %f, %f, %f, %f\n", ax_avg/count, ay_avg/count, az_avg/count, mx_avg/count, my_avg/count, mz_avg/count);
          ax_avg = 0; ay_avg  = 0; az_avg = 0; mx_avg = 0; my_avg  = 0; mz_avg = 0;
          break;
        }
      case 'x': //No calibration
        {
          uint8_t count = data_count;
          uint8_t i_count = 0;
          //for(uint8_t i=0; i<count; i++) {
          while(i_count < count) {
            if(imu.Read(values1)){
                if(imu.new_imu_data()){
                i_count += 1;
                ax_avg += values1[0]; ay_avg += values1[1]; az_avg += values1[2];   
                mx_avg += values1[6]; my_avg += values1[7]; mz_avg += values1[8]; 
                //sprintf(str1, "%f,%f,%f,%f,%f,%f,%f,%f,%f,", values1[0], values1[1], values1[2], values1[3], values1[4], values1[5], values1[6], values1[7], values1[8]);
                //Serial.println(str1);
              }
            }
          }
          float m_normal = sqrt((mx_avg/count)*(mx_avg/count) + (my_avg/count)*(my_avg/count) + (mz_avg/count)*(mz_avg/count));
          Serial.printf("No Cal: %f, %f, %f, %f, %f, %f\n", ax_avg/count, ay_avg/count, az_avg/count, (mx_avg/count)/m_normal, (my_avg/count)/m_normal, (mz_avg/count)/m_normal);
          ax_avg = 0; ay_avg  = 0;; az_avg = 0; mx_avg = 0; my_avg  = 0; mz_avg = 0;
          break;
        }
      case 'z':  //MotionCal by PJRC
        {
          uint8_t count = data_count;
          uint8_t i_count = 0;
          //for(uint8_t i=0; i<count; i++) {
          while(i_count < count) {
            if(imu.Read_raw(raw_values)){
              if(imu.new_imu_data()){
                i_count += 1;
                float x = (float)raw_values[6] * magScale[0];
                float y = (float)raw_values[7] * magScale[1];
                float z = (float)raw_values[8] * magScale[2];
                float magxc = mCal[0][0]*(x-MagOffset[0])+ mCal[0][1]*(y-MagOffset[1]) + mCal[0][2]*(z-MagOffset[2]);
                float magyc = mCal[1][0]*(x-MagOffset[0])+ mCal[1][1]*(y-MagOffset[1]) + mCal[1][2]*(z-MagOffset[2]);
                float magzc = mCal[2][0]*(x-MagOffset[0])+ mCal[2][1]*(y-MagOffset[1]) + mCal[2][2]*(z-MagOffset[2]);
                //magxc *= magScale[0];
                ///magyc *= magScale[1];
                //magzc *= magScale[2];
                ax_avg += 0; ay_avg += 0; az_avg += 0;   
                mx_avg += magxc; my_avg += magyc; mz_avg += magzc; 
                //sprintf(str1, "%d,%d,%d,%d,%d,%f,%f,%f,", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5], magxc, magyc, magzc);
                //Serial.println(str1);
              }
            }
          }
          float m_normal = sqrt((mx_avg/count)*(mx_avg/count) + (my_avg/count)*(my_avg/count) + (mz_avg/count)*(mz_avg/count));
          Serial.printf("MotionCal: %f, %f, %f, %f, %f, %f\n", ax_avg/count, ay_avg/count, az_avg/count,(mx_avg/count)/m_normal, (my_avg/count)/m_normal, (mz_avg/count)/m_normal);
          ax_avg = 0; ay_avg  = 0;; az_avg = 0; mx_avg = 0; my_avg  = 0; mz_avg = 0;
          break;
        }
      case '1':  //Magneto 1.2
        {
          uint8_t count = data_count;
          uint8_t i_count = 0;
          //for(uint8_t i=0; i<count; i++) {
          while(i_count < count) {
            if(imu.Read_raw(raw_values)){
              if(imu.new_imu_data()){
                i_count += 1;
                float x = (float)raw_values[6] * magScale[0];
                float y = (float)raw_values[7] * magScale[1];
                float z = (float)raw_values[8] * magScale[2];
                float magxc = mCal1[0][0]*(x-MagOffset1[0])+ mCal1[0][1]*(y-MagOffset1[1]) + mCal1[0][2]*(z-MagOffset1[2]);
                float magyc = mCal1[1][0]*(x-MagOffset1[0])+ mCal1[1][1]*(y-MagOffset1[1]) + mCal1[1][2]*(z-MagOffset1[2]);
                float magzc = mCal1[2][0]*(x-MagOffset1[0])+ mCal1[2][1]*(y-MagOffset1[1]) + mCal1[2][2]*(z-MagOffset1[2]);

                float ax = (float)raw_values[0] * accelScale;
                float ay = (float)raw_values[1] * accelScale;
                float az = (float)raw_values[2] * accelScale;
                float agxc = aCal1[0][0]*(ax-AccOffset1[0])+ aCal1[0][1]*(ay-AccOffset1[1]) + aCal1[0][2]*(az-AccOffset1[2]);
                float agyc = aCal1[1][0]*(ax-AccOffset1[0])+ aCal1[1][1]*(ay-AccOffset1[1]) + aCal1[1][2]*(az-AccOffset1[2]);
                float agzc = aCal1[2][0]*(ax-AccOffset1[0])+ aCal1[2][1]*(ay-AccOffset1[1]) + aCal1[2][2]*(az-AccOffset1[2]);
                
                ax_avg += agyc; ay_avg += agxc; az_avg += agzc;   
                mx_avg += magxc; my_avg += magyc; mz_avg += magzc; 
                //sprintf(str1, "%f,%f,%f,%d,%d,%d,%f,%f,%f,",  agyc, agxc, -agzc, raw_values[3], raw_values[4], raw_values[5], magxc, magyc, magzc);
                //Serial.println(str1);
              }
            }
          }
          float m_normal = sqrt((mx_avg/count)*(mx_avg/count) + (my_avg/count)*(my_avg/count) + (mz_avg/count)*(mz_avg/count));
          Serial.printf("Magneto1.2: %f, %f, %f, %f, %f, %f\n", ay_avg/count, ax_avg/count, -az_avg/count, (mx_avg/count)/m_normal, (my_avg/count)/m_normal, (mz_avg/count)/m_normal);
          ax_avg = 0; ay_avg  = 0;; az_avg = 0; mx_avg = 0; my_avg  = 0; mz_avg = 0;
          break;
        }
      case '2': // min max
        {
          uint8_t count = data_count;
          uint8_t i_count = 0;
          //for(uint8_t i=0; i<count; i++) {
          while(i_count < count) {
            if(imu.Read_raw(raw_values)){
              if(imu.new_imu_data()){
                i_count += 1;
                float magxc = (((float)raw_values[6] * magScale[0]) - magn_off[0]) * magn_scale[0];
                float magyc = (((float)raw_values[7] * magScale[1]) - magn_off[1]) * magn_scale[1];
                float magzc = (((float)raw_values[8] * magScale[2]) - magn_off[2]) * magn_scale[2];
                float ax = (((float)raw_values[1] * accelScale) - acc_off[0]/G) * acc_scale[0] ;
                float ay = (((float)raw_values[0] * accelScale) - acc_off[1]/G) * acc_scale[1] ;
                float az = ((-(float)raw_values[2] * accelScale) - acc_off[2]/G) * acc_scale[2];
                ax_avg += ax; ay_avg += ay; az_avg += az;   
                mx_avg += magxc; my_avg += magyc; mz_avg += magzc; 
                //sprintf(str1, "%f,%f,%f,%d,%d,%d,%f,%f,%f,",  ax, ay, az, raw_values[3], raw_values[4], raw_values[5], magxc, magyc, magzc);
                //Serial.println(str1);
              }
            }
          }
          float m_normal = sqrt((mx_avg/count)*(mx_avg/count) + (my_avg/count)*(my_avg/count) + (mz_avg/count)*(mz_avg/count));
          Serial.printf("Min/Max: %f, %f, %f, %f, %f, %f\n", ax_avg/count, ay_avg/count, az_avg/count, (mx_avg/count)/m_normal, (my_avg/count)/m_normal, (mz_avg/count)/m_normal);
          ax_avg = 0; ay_avg  = 0;; az_avg = 0; mx_avg = 0; my_avg  = 0; mz_avg = 0;
          break;
        }
      default:
        break;
    }
    while (Serial.read() != -1)
        ;  // lets strip the rest out
  }
}

char serial_busy_wait() {
  while(!Serial.available()) {
    ; // do nothing until ready
  }
  return Serial.read();
}

//From FreeIMU library
void writeArr(void * varr, uint8_t arr_length, uint8_t type_bytes) {
  byte * arr = (byte*) varr;
  for(uint8_t i=0; i<arr_length; i++) {
    writeVar(&arr[i * type_bytes], type_bytes);
  }
}


// thanks to Francesco Ferrara and the Simplo project for the following code!
void writeVar(void * val, uint8_t type_bytes) {
  byte * addr=(byte *)(val);
  for(uint8_t i=0; i<type_bytes; i++) { 
    Serial.write(addr[i]);
  }
}


