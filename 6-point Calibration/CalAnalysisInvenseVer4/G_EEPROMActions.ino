////////////////////////////////////////////////////////////////////
//
// EEPROM Routines
//
////////////////////////////////////////////////////////////////////
void printEEPROMBiases(){

  float axbE, aybE, azbE, axsE, aysE, azsE;
  float hxbE, hybE, hzbE, hxsE, hysE, hzsE;
  
  // read entire memory buffer, where gyro and accel bias and scale factors stored
  for (size_t i=0; i < sizeof(EepromBuffer); i++) {
    EepromBuffer[i] = EEPROM.read(i);
  }
  
  // assign biases and scale factors to proper variables
  Serial.println("Teensy Accel and Gyro EEPROM Biases:"); 
  memcpy(&axbE,EepromBuffer+0,4);
  memcpy(&axsE,EepromBuffer+4,4);
  memcpy(&aybE,EepromBuffer+8,4);
  memcpy(&aysE,EepromBuffer+12,4);
  memcpy(&azbE,EepromBuffer+16,4);
  memcpy(&azsE,EepromBuffer+20,4);
  memcpy(&hxbE,EepromBuffer+24,4);
  memcpy(&hxsE,EepromBuffer+28,4);
  memcpy(&hybE,EepromBuffer+32,4);
  memcpy(&hysE,EepromBuffer+36,4);
  memcpy(&hzbE,EepromBuffer+40,4);
  memcpy(&hzsE,EepromBuffer+44,4);
  //Serial.println("Accel and Mag Biases and Scale Factors Read"); 

  // print gyro, accel, and mag calibration values to serial terminal
  //Serial.println(" "); 
  //Serial.println("Current Gyro Biases (rads): ");
  //Serial.println( gxb,6); 
  //Serial.println( gyb,6); 
  //Serial.println( gzb,6); 
  //Serial.println(" "); 
  Serial.println("Current Accel Biases (mss) and Scale Factors: ");
  Serial.print( axbE,6); Serial.print(", "); Serial.println(axsE,6);
  Serial.print( aybE,6); Serial.print(", "); Serial.println(aysE,6);
  Serial.print( azbE,6); Serial.print(", "); Serial.println(azsE,6);
  //Serial.println(" "); 
  Serial.println("Current Mag Biases (uT) and Scale Factors: ");
  Serial.print( hxbE,6); Serial.print(", "); Serial.println(hxsE,6);
  Serial.print( hybE,6); Serial.print(", "); Serial.println(hysE,6);
  Serial.print( hzbE,6); Serial.print(", "); Serial.println(hzsE,6);
  Serial.println(" ");
}

void loadSCBiasesEEPROM() {

  // save to memory
  memcpy(EepromBuffer+0,&axb,4);
  memcpy(EepromBuffer+4,&axs,4);
  memcpy(EepromBuffer+8,&ayb,4);
  memcpy(EepromBuffer+12,&ays,4);
  memcpy(EepromBuffer+16,&azb,4);
  memcpy(EepromBuffer+20,&azs,4);
  memcpy(EepromBuffer+24,&hxb,4);
  memcpy(EepromBuffer+28,&hxs,4);
  memcpy(EepromBuffer+32,&hyb,4);
  memcpy(EepromBuffer+36,&hys,4);
  memcpy(EepromBuffer+40,&hzb,4);
  memcpy(EepromBuffer+44,&hzs,4);

  // write memory to EEPROM
  for (size_t i=0; i < sizeof(EepromBuffer); i++) {
    EEPROM.write(i,EepromBuffer[i]);
  }
  Serial.println("Static Calibration Values Saved to EEPROM");
  Serial.println(" ");
}

void loadLibBiasesEEPROM() {
/*
  // saving MPU9250 library cal values to temp memory
  value = Imu.getAccelBiasX_mss();
  memcpy(EepromBuffer,&value,4);
  value = Imu.getAccelScaleFactorX();
  memcpy(EepromBuffer+4,&value,4);
  value = Imu.getAccelBiasY_mss();
  memcpy(EepromBuffer+8,&value,4);
  value = Imu.getAccelScaleFactorY();
  memcpy(EepromBuffer+12,&value,4);
  value = Imu.getAccelBiasZ_mss();
  memcpy(EepromBuffer+16,&value,4);
  value = Imu.getAccelScaleFactorZ();
  memcpy(EepromBuffer+20,&value,4);
  value = Imu.getMagBiasX_uT();
  memcpy(EepromBuffer+24,&value,4);
  value = Imu.getMagScaleFactorX();
  memcpy(EepromBuffer+28,&value,4);
  value = Imu.getMagBiasY_uT();
  memcpy(EepromBuffer+32,&value,4);
  value = Imu.getMagScaleFactorY();
  memcpy(EepromBuffer+36,&value,4);
  value = Imu.getMagBiasZ_uT();
  memcpy(EepromBuffer+40,&value,4);
  value = Imu.getMagScaleFactorZ();
  memcpy(EepromBuffer+44,&value,4);

  // write memory to EEPROM
  for (size_t i=0; i < sizeof(EepromBuffer); i++) {
    EEPROM.write(i,EepromBuffer[i]);
  }
  */
  Serial.println("MPU9250 Lib Calibration Values Saved to EEPROM");
  Serial.println(" ");
}
