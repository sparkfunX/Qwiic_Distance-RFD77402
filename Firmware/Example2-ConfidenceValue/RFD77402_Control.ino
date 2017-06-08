#define RFD77402_ADDR 0x4C //7-bit unshifted default I2C Address

//Register addresses
#define RFD77402_ICSR 0x00
#define RFD77402_INTERRUPTS 0x02
#define RFD77402_COMMAND 0x04
#define RFD77402_DEVICE_STATUS 0x06
#define RFD77402_RESULT 0x08
#define RFD77402_RESULT_CONFIDENCE 0x0A
#define RFD77402_CONFIGURE_A 0x0C
#define RFD77402_CONFIGURE_B 0x0E
#define RFD77402_HOST_TO_MCPU_MAILBOX 0x10
#define RFD77402_MCPU_TO_HOST_MAILBOX 0x12
#define RFD77402_CONFIGURE_PMU 0x14
#define RFD77402_CONFIGURE_I2C 0x1C
#define RFD77402_CONFIGURE_HW_0 0x20
#define RFD77402_CONFIGURE_HW_1 0x22
#define RFD77402_CONFIGURE_HW_2 0x24
#define RFD77402_CONFIGURE_HW_3 0x26
#define RFD77402_MOD_CHIP_ID 0x28

#define RFD77402_MODE_MEASUREMENT 0x01
#define RFD77402_MODE_STANDBY 0x10
#define RFD77402_MODE_OFF 0x11
#define RFD77402_MODE_ON 0x12

#define CODE_VALID_DATA 0x00
#define CODE_FAILED_PIXELS 0x01
#define CODE_FAILED_SIGNAL 0x02
#define CODE_FAILED_SATURATED 0x03
#define CODE_FAILED_NOT_NEW 0x04
#define CODE_FAILED_TIMEOUT 0x05

//Global values
uint16_t distance;
uint8_t validPixels;
uint16_t confidenceValue;
uint8_t calibrationData[54]; //Loaded by the 0x006 mailbox command

//Sets up the sensor for constant read
//Returns true if sensor responded
boolean RFD77402setup(void)
{
  unsigned int hwConfig;

  uint16_t moduleChipID = readRegister16(RFD77402_MOD_CHIP_ID);
  if (moduleChipID < 0xAD00)
  {
    Serial.print("ID (should be 0xAD00 or greater): 0x");
    Serial.println(moduleChipID, HEX);
    return (false);
  }

  //Put chip into standby
  if (goToStandbyMode() == false) return (false); //Chip timed out before going to standby

  //Drive INT_PAD high
  byte setting = readRegister(RFD77402_ICSR);
  setting |= (1 << 2); //Set the bit
  writeRegister(RFD77402_ICSR, setting);

  //Configure I2C Interface
  writeRegister(RFD77402_CONFIGURE_I2C, 0x65); //0b.0110.0101 = Address increment, auto increment, host debug, MCPU debug

  //Set initialization - Magic from datasheet. Write 0x05 to 0x15 location.
  writeRegister16(RFD77402_CONFIGURE_PMU, 0x0500); //Patch_code_id_en, Patch_mem_en

  if (goToOffMode() == false) return (false); //MCPU never turned off

  //Read Module ID
  //Skipped

  //Read Firmware ID
  //Skipped

  //Set initialization - Magic from datasheet. Write 0x06 to 0x15 location.
  writeRegister16(RFD77402_CONFIGURE_PMU, 0x0600); //MCPU_Init_state, Patch_mem_en

  if (goToOnMode() == false) return (false); //MCPU never turned on

  //ToF Configuration
  //writeRegister16(RFD77402_CONFIGURE_A, 0xE100); //0b.1110.0001 = Peak is 0x0E, Threshold is 1.
  setPeak(0x0E); //Suggested values from page 20
  setThreshold(0x01);

  writeRegister16(RFD77402_CONFIGURE_B, 0x10FF); //Set valid pixel. Set MSP430 default config.
  writeRegister16(RFD77402_CONFIGURE_HW_0, 0x07D0); //Set saturation threshold = 2,000.
  writeRegister16(RFD77402_CONFIGURE_HW_1, 0x5008); //Frequecy = 5. Low level threshold = 8.
  writeRegister16(RFD77402_CONFIGURE_HW_2, 0xA041); //Integration time = 10 * (6500-20)/15)+20 = 4.340ms. Plus reserved magic.
  writeRegister16(RFD77402_CONFIGURE_HW_3, 0x45D4); //Enable harmonic cancellation. Enable auto adjust of integration time. Plus reserved magic.

  if (goToStandbyMode() == false) return (false); //Error - MCPU never went to standby

  //Whew! We made it through power on configuration

  //Get the calibration data via the 0x0006 mailbox command
  getCalibrationData(); //Load the calibration array

  //Put device into Standby mode
  if (goToStandbyMode() == false) return (false); //Error - MCPU never went to standby

  //Now assume user will want sensor in measurement mode
  //Go from IDLE to MCPU On and be ready to take single shot measurement

  //Set initialization - Magic from datasheet. Write 0x05 to 0x15 location.
  writeRegister16(RFD77402_CONFIGURE_PMU, 0x0500); //Patch_code_id_en, Patch_mem_en

  if (goToOffMode() == false) return (false); //Error - MCPU never turned off

  //Write calibration data
  //This may be the 54 bytes read from the mailbox command

  //Set initialization - Magic from datasheet. Write 0x06 to 0x15 location.
  writeRegister16(RFD77402_CONFIGURE_PMU, 0x0600); //MCPU_Init_state, Patch_mem_en

  if (goToOnMode() == false) return (false); //Error - MCPU never turned on

  return (true); //Success! Sensor is ready for measurements
}

//Takes a single measurement and sets the global variables with new data
//Returns zero if reading is good, otherwise return the errorCode from the result register.
uint8_t takeMeasurement(void)
{
  uint8_t errorCode;

  if (goToMeasurementMode() == false) return (CODE_FAILED_TIMEOUT); //Error - Timeout
  //New data is now available!

  //Read result
  uint16_t resultRegister = readRegister16(RFD77402_RESULT);

  if (resultRegister & 0x7FFF) //Reading is valid
  {
    errorCode = (resultRegister >> 13) & 0x03;

    if (errorCode == 0)
    {
      distance = (resultRegister >> 2) & 0x07FF; //Distance is good. Read it.
      //Serial.println("Distance field valid");

      //Read confidence register
      uint16_t confidenceRegister = readRegister16(RFD77402_RESULT_CONFIDENCE);
      validPixels = confidenceRegister & 0x0F;
      confidenceValue = (confidenceRegister >> 4) & 0x07FF;
    }

    return (errorCode);

  }
  else
  {
    //Reading is not vald
    return (CODE_FAILED_NOT_NEW); //Error code for reading is not new
  }

}

uint16_t getDistance()
{
  return (distance);
}

uint8_t getValidPixels()
{
  return (validPixels);
}

uint16_t getConfidenceValue()
{
  return (confidenceValue);
}

//Read the command opcode and covert to mode
uint8_t getMode()
{
  return (readRegister(RFD77402_COMMAND) & 0x3F);
}

//Tell MCPU to go to standby mode
//Return true if successful
boolean goToStandbyMode()
{
  //Set Low Power Standby
  writeRegister(RFD77402_COMMAND, 0x90); //0b.1001.0000 = Go to standby mode. Set valid command.

  //Check MCPU_ON Status
  for (uint8_t x = 0 ; x < 10 ; x++)
  {
    if ( (readRegister16(RFD77402_DEVICE_STATUS) & 0x001F) == 0x0000) return (true); //MCPU is now in standby
    delay(10); //Suggested timeout for status checks from datasheet
  }

  return (false); //Error - MCPU never went to standby
}

//Tell MCPU to go to off state
//Return true if successful
boolean goToOffMode()
{
  //Set MCPU_OFF
  writeRegister(RFD77402_COMMAND, 0x91); //0b.1001.0001 = Go MCPU off state. Set valid command.

  //Check MCPU_OFF Status
  for (uint8_t x = 0 ; x < 10 ; x++)
  {
    if ( (readRegister16(RFD77402_DEVICE_STATUS) & 0x001F) == 0x0010) return (true); //MCPU is now off
    delay(10); //Suggested timeout for status checks from datasheet
  }

  return (false); //Error - MCPU never turned off
}

//Tell MCPU to go to on state
//Return true if successful
boolean goToOnMode()
{
  //Set MCPU_ON
  writeRegister(RFD77402_COMMAND, 0x92); //0b.1001.0010 = Wake up MCPU to ON mode. Set valid command.

  //Check MCPU_ON Status
  for (uint8_t x = 0 ; x < 10 ; x++)
  {
    if ( (readRegister16(RFD77402_DEVICE_STATUS) & 0x001F) == 0x0018) return (true); //MCPU is now on
    delay(10); //Suggested timeout for status checks from datasheet
  }

  return (false); //Error - MCPU never turned on
}

//Tell MCPU to go to measurement mode
//Takes a measurement. If measurement data is ready, return true
boolean goToMeasurementMode()
{
  //Single measure command
  writeRegister(RFD77402_COMMAND, 0x81); //0b.1000.0001 = Single measurement. Set valid command.

  //Read ICSR Register - Check to see if measurement data is ready
  for (uint8_t x = 0 ; x < 10 ; x++)
  {
    if ( (readRegister(RFD77402_ICSR) & (1 << 4)) != 0) return (true); //Data is ready!
    delay(10); //Suggested timeout for status checks from datasheet
  }

  return (false); //Error - Timeout
}

//Returns the VCSEL peak 4-bit value
uint16_t getPeak(void)
{
  uint16_t configValue = readRegister16(RFD77402_CONFIGURE_A);
  return (configValue >> 12);
}

//Sets the VCSEL peak 4-bit value
void setPeak(uint16_t peakValue)
{
  uint16_t configValue = readRegister16(RFD77402_CONFIGURE_A); //Read
  configValue &= ~0xF000;// Zero out the peak configuration bits
  configValue |= peakValue << 12; //Mask in user's settings
  writeRegister16(RFD77402_CONFIGURE_A, configValue); //Write in this new value
}

//Returns the VCSEL Threshold 4-bit value
uint16_t getThreshold(void)
{
  uint16_t configValue = readRegister16(RFD77402_CONFIGURE_A);
  return ((configValue >> 8) & 0x0F);
}

//Sets the VCSEL Threshold 4-bit value
void setThreshold(uint16_t thresholdValue)
{
  uint16_t configValue = readRegister16(RFD77402_CONFIGURE_A); //Read
  configValue &= ~0x0F00;// Zero out the threshold configuration bits
  configValue |= thresholdValue << 8; //Mask in user's settings
  writeRegister16(RFD77402_CONFIGURE_A, configValue); //Write in this new value
}

//Returns the VCSEL Frequency 4-bit value
uint16_t getFrequency(void)
{
  uint16_t configValue = readRegister16(RFD77402_CONFIGURE_HW_1);
  return ((configValue >> 12) & 0x0F);
}

//Sets the VCSEL Frequency 4-bit value
void setFrequency(uint16_t thresholdValue)
{
  uint16_t configValue = readRegister16(RFD77402_CONFIGURE_HW_1); //Read
  configValue &= ~0xF000;// Zero out the threshold configuration bits
  configValue |= thresholdValue << 12; //Mask in user's settings
  writeRegister16(RFD77402_CONFIGURE_HW_1, configValue); //Write in this new value
}

//Gets whatever is in the 'MCPU to Host' mailbox
//Check ICSR bit 5 before reading
uint16_t getMailbox(void)
{
  return (readRegister16(RFD77402_MCPU_TO_HOST_MAILBOX));
}

//Retreive 2*27 bytes from MCPU for computation of calibration parameters
//This is 9.2.2 from datasheet
//Reads 54 bytes into the calibration[] array
//Returns true if new cal data is loaded
boolean getCalibrationData(void)
{
  if (goToOnMode() == false) return (false); //Error - sensor timed out before getting to On Mode

  //Check ICSR Register and read Mailbox until it is empty
  uint8_t messages = 0;
  while (1)
  {
    if ( (readRegister(RFD77402_ICSR) & (1 << 5)) == 0) break; //Mailbox interrupt is cleared

    //Mailbox interrupt (Bit 5) is set so read the M2H mailbox register
    getMailbox(); //Throw it out. Just read to clear the register.

    if (messages++ > 27) return (false); //Error - Too many messages

    delay(10); //Suggested timeout for status checks from datasheet
  }

  //Issue mailbox command
  writeRegister16(RFD77402_HOST_TO_MCPU_MAILBOX, 0x0006); //Send 0x0006 mailbox command

  //Check to see if Mailbox can be read
  //Read 54 bytes of payload into the calibration[54] array
  for (uint8_t message = 0 ; message < 27 ; message++)
  {
    //Wait for bit to be set
    uint8_t x = 0;
    while (1)
    {
      uint8_t icsr = readRegister(RFD77402_ICSR);
      if ( (icsr & (1 << 5)) != 0) break; //New message in available

      if (x++ > 10) return (false); //Error - Timeout

      delay(10); //Suggested timeout for status checks from datasheet
    }

    uint16_t incoming = getMailbox(); //Get 16-bit message

    //Put message into larger calibrationData array
    calibrationData[message * 2] = incoming >> 8;
    calibrationData[message * 2 + 1] = incoming & 0xFF;
  }

  /*Serial.println("Calibration data:");
    for (int x = 0 ; x < 54 ; x++)
    {
    Serial.print("[");
    Serial.print(x);
    Serial.print("]=0x");
    if (calibrationData[x] < 0x10) Serial.print("0"); //Pretty print
    Serial.println(calibrationData[x], HEX);
    }*/
}

//Reads two bytes from a given location from the RFD77402
uint16_t readRegister16(uint8_t addr)
{
  Wire.beginTransmission(RFD77402_ADDR);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(RFD77402_ADDR, 2);

  if (Wire.available() != 2) return (0xFFFF); //Error

  uint8_t lower = Wire.read();
  uint8_t higher = Wire.read();

  return ((uint16_t)higher << 8 | lower);
}

//Reads from a given location from the RFD77402
uint8_t readRegister(uint8_t addr)
{
  Wire.beginTransmission(RFD77402_ADDR);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(RFD77402_ADDR, 1);
  if (Wire.available()) return (Wire.read());

  Serial.println("I2C Error");
  return (0xFF); //Error
}

//Write a 16 bit value to a spot in the RFD77402
void writeRegister16(uint8_t addr, uint16_t val)
{
  Wire.beginTransmission(RFD77402_ADDR);
  Wire.write(addr);
  Wire.write(val & 0xFF); //Lower byte
  Wire.write(val >> 8); //Uper byte
  Wire.endTransmission();
}

//Write a value to a spot in the RFD77402
void writeRegister(uint8_t addr, uint8_t val)
{
  Wire.beginTransmission(RFD77402_ADDR);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission();
}

