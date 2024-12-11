#include <Wire.h>
#include <LSM303.h>        // Click here to get the library: https://github.com/pololu/lsm303-arduino
#include <L3G.h>           // Click here to get the library: https://github.com/pololu/l3g-arduino

#include <MadgwickAHRS.h>  // Click here to get the library: https://github.com/arduino-libraries/MadgwickAHRS
#include <MahonyAHRS.h>    // Click here to get the library: https://github.com/PaulStoffregen/MahonyAHRS

LSM303 compass;
L3G gyro;

int period = 100;
unsigned long time_now = 0;

//Mahony filter;   // Decide which filter. Mahony is not as accurate but is faster than Madgwick. Use Mahony if MCU is slow.
Madgwick filter;

void setup() {
Serial.begin(9600);
Wire.begin();
compass.init();
compass.enableDefault();
gyro.init();
gyro.enableDefault();

filter.begin(1000/period); // Sampling rate in Hz.
}

void loop()
{
compass.read();
gyro.read();
float ax_off, ay_off, az_off, ax_cal, ay_cal, az_cal, ax, ay, az, gx, gy, gz, mx_off, my_off, mz_off, mx_cal, my_cal, mz_cal, mx, my, mz;

if(millis() >= time_now + period){
   time_now += period;

// Accelerometer calibration
ax_off = compass.a.x/16.0 + 7.119589;
ay_off = compass.a.y/16.0 + 23.882226;
az_off = compass.a.z/16.0 + 122.388174;
ax_cal =  0.979353*ax_off + 0.002652*ay_off + 0.000235*az_off;
ay_cal =  0.002652*ax_off + 0.967917*ay_off - 0.002283*az_off;
az_cal =  0.000235*ax_off - 0.002283*ay_off + 1.001514*az_off;

// Accelerometer mGal to m/s²

ax = -1*ax_cal/100.00;
ay = ay_cal/100.00;
az = az_cal/100.00;

// Gyroscope calibration
gx = (gyro.g.x -420)* 8.75/1000.00;     // Check the offset of the gyroscope at rest.
gy = -1*(gyro.g.y -540)* 8.75/1000.00;  // Once gyro is at 0, multiply with 8.75/1000 dps/LSB
gz = -1*(gyro.g.z - 15)* 8.75/1000.00;

// Magnetometer calibration
mx_off = compass.m.x*(100000.0/1100.0) + 1435.229315;
my_off = compass.m.y*(100000.0/1100.0) + 11849.930610;
mz_off = compass.m.z*(100000.0/980.0 ) + 14156.784119;
mx_cal =  0.959250*mx_off + 0.002153*my_off + 0.017237*mz_off;
my_cal =  0.002153*mx_off + 0.942457*my_off - 0.021010*mz_off;
mz_cal =  0.017237*mx_off - 0.021010*my_off + 0.982433*mz_off;

// Magnetometer from nT to µT
mx = -1* mx_cal/1000.00;
my = my_cal/1000.00;
mz = mz_cal/1000.00;

// Update the filter
    
//filter.updateIMU(gx, gy, gz, ax, ay, az);
filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

// Print the orientation filter output
float roll = filter.getRoll();
float pitch = filter.getPitch();
float heading = filter.getYaw();
Serial.print(millis());
Serial.print(" - Orientation: ");
Serial.print(heading);
Serial.print(" ");
Serial.print(pitch);
Serial.print(" ");
Serial.print(roll);

Serial.println();

}
}
