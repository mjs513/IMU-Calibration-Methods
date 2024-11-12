////////////////////////////////////////////////////////////////////
//
// Zeros out calibrations in EEPROM and MPU9250 Library
//
////////////////////////////////////////////////////////////////////

void zeroCalValues(){
  axb = 0.0f;
  ayb = 0.0f;
  azb = 0.0f;  
  axs = 1.0f;
  ays = 1.0f;
  azs = 1.0f;
  hxb = 0.0f;
  hyb = 0.0f;
  hzb = 0.0f;  
  hxs = 1.0f;
  hys = 1.0f;
  hzs = 1.0f;
  gxb = 0.0f;
  gyb = 0.0f;
  gzb = 0.0f;

  // saving to memory
  value = axb;
  memcpy(EepromBuffer,&value,4);
  value = axs;
  memcpy(EepromBuffer+4,&value,4);
  value = ayb;
  memcpy(EepromBuffer+8,&value,4);
  value = ays;
  memcpy(EepromBuffer+12,&value,4);
  value = azb;
  memcpy(EepromBuffer+16,&value,4);
  value = azs;
  memcpy(EepromBuffer+20,&value,4);
  value = hxb;
  memcpy(EepromBuffer+24,&value,4);
  value = hxs;
  memcpy(EepromBuffer+28,&value,4);
  value = hyb;
  memcpy(EepromBuffer+32,&value,4);
  value = hys;
  memcpy(EepromBuffer+36,&value,4);
  value = hzb;
  memcpy(EepromBuffer+40,&value,4);
  value = hzs;
  memcpy(EepromBuffer+44,&value,4);

  // write memory to EEPROM
  for (size_t i=0; i < sizeof(EepromBuffer); i++) {
    EEPROM.write(i,EepromBuffer[i]);
  }
  Serial.println("Calibration Values Reset in EEPROM");
  Serial.println(" ");
  
  //readAndPrintCalValues();

  zeroIMUBiases();
}

