////////////////////////////////////////////////////////////////////
//
// Routine to stream IMU raw data and heading to serial
//
////////////////////////////////////////////////////////////////////

void serialPrint(){

  //Needed for Tviewer
  int textIndex;
  textIndex = 10 * 31;
  char text[textIndex];
  //float headingVal, hxVal, hyVal, hzVal;
  #if defined(MPU9250) || defined(ICM20648)
  float hxVal = Imu.mag_x_ut();
  float hyVal = Imu.mag_y_ut();
  float hzVal = Imu.mag_z_ut();
  #else
  float hxVal = 0;
  float hyVal = 0;
  float hzVal = 0;
  #endif
  float headingVal;
  
  if (-hyVal > 0){
    headingVal = (PI/2.0f - ( atanf(hxVal/(-hyVal)) ) ) * 180.0f/PI;
  }
  else {
    headingVal = (3.0f*PI/2.0f - ( atanf(hxVal/(-hyVal)) ) ) * 180.0f/PI;
  }
   
  char ax_text[30];
  char ay_text[30];
  char az_text[30];
  char gx_text[30];
  char gy_text[30];
  char gz_text[30];
  char hx_text[30];
  char hy_text[30];
  char hz_text[30];
  char heading_text[30];

  dtostrf(Imu.accel_x_mps2(), 10,4, ax_text);
  dtostrf(Imu.accel_y_mps2(), 10,4, ay_text);
  dtostrf(Imu.accel_z_mps2(), 10,4, az_text);
  dtostrf(Imu.gyro_x_radps(), 10,4, gx_text);
  dtostrf(Imu.gyro_y_radps(), 10,4, gy_text);
  dtostrf(Imu.gyro_z_radps(), 10,4, gz_text);
  dtostrf(hxVal, 10,4, hx_text);
  dtostrf(hyVal, 10,4, hy_text);
  dtostrf(hzVal, 10,4, hz_text);
  dtostrf(headingVal, 10,4, heading_text);

  snprintf(text, textIndex, "%s,%s,%s,%s,%s, %s,%s,%s,%s,%s", 
        ax_text, ay_text, az_text,
        gx_text, gy_text, gz_text, 
        hx_text, hy_text, hz_text, heading_text );
  Serial.println(text);
}

