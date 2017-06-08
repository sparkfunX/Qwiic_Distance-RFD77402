/*
  Getting distance from the RFD77402 Time of Flight Sensor
  By: Nathan Seidle
  SparkFun Electronics
  Date: June 6th, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!

  Read the raw distance values from the sensor.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield (any port)
  Open the serial monitor at 9600 baud

  Available:
  void takeMeasurements()
  void takeMeasurementsWithBulb()
  int getViolet() Blue() Green(01) Yellow() Orange() Red()
  float getCalibratedViolet(), Blue, Green, Yellow, Orange, Red
  void setMeasurementMode(byte mode)
  boolean dataAvailable()
  boolean as726xSetup()
  byte getTemperature()
  byte getTemperatureF()
  void setIndicatorCurrent(byte)
  void enableIndicator()
  void disableIndicator()
  void setBulbCurrent(byte)
  void enableBulb()
  void disableBulb()
  void softReset()
  setGain(byte gain)
  setIntegrationTime(byte integrationValue)
  enableInterrupt()
  disableInterrupt()

  TODO:
  getPeak/setPeak(byte) - Gets/sets the vertical-cavity surface-emitting laser (VCSEL) peak
  getThreshold/setThreshold(byte) - Gets/sets the VCSEL threshold
  getFrequency/setFrequency(byte) - Gets/sets the modulation frequency

  goToOffMode() - Turn off MCPU
  goToOnMode() - Wake MCPU to ON Mode
  goToStandbyMode() - Low power standby mode
  getMeasurement() - Once sensor is configured, issue this command to take measurement

  getCalibrationData - reads 27 messages of MPU mailbox data and loads calibration data array
  getMailbox() - returns the 16-bits in the MPU mailbox

  Sleep - power control register. Unknown. Page 28

  getValidPixels Result_Confidence_Register

*/

#include <Wire.h>

#define CODE_VALID_DATA 0x00
#define CODE_FAILED_PIXELS 0x01
#define CODE_FAILED_SIGNAL 0x02
#define CODE_FAILED_SATURATED 0x03
#define CODE_FAILED_NOT_NEW 0x04
#define CODE_FAILED_TIMEOUT 0x05

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println("RFD77402 Read Example");

  Wire.begin();

  if (RFD77402setup() == false)
  {
    Serial.println("Sensor failed to respond. Check wiring.");
    while (1); //Freeze!
  }
  Serial.println("Sensor online!");
}

void loop()
{
  byte errorCode = takeMeasurement();
  if (errorCode == CODE_VALID_DATA)
  {
    unsigned int distance = getDistance();
    byte pixels = getValidPixels();
    unsigned int confidence = getConfidenceValue();

    Serial.print("distance: ");
    Serial.print(distance);
    Serial.print(" pixels: ");
    Serial.print(pixels);
    Serial.print(" confidence: ");
    Serial.print(confidence);
  }
  else if (errorCode == CODE_FAILED_PIXELS)
  {
    Serial.print("Not enough pixels valid");
  }
  else if (errorCode == CODE_FAILED_SIGNAL)
  {
    Serial.print("Not enough signal");
  }
  else if (errorCode == CODE_FAILED_SATURATED)
  {
    Serial.print("Sensor pixels saturated");
  }
  else if (errorCode == CODE_FAILED_NOT_NEW)
  {
    Serial.print("New measurement failed");
  }
  else if (errorCode == CODE_FAILED_TIMEOUT)
  {
    Serial.print("Sensors timed out");
  }

  Serial.println();

  //delay(25);
}
