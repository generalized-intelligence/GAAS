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
*/

#include "MPU9250.h"
#include "EEPROM.h"

// an MPU-9250 object on SPI bus 0 with chip select 10
MPU9250 Imu(SPI,10);
int status;

// EEPROM buffer and variable to save accel and mag bias 
// and scale factors
uint8_t EepromBuffer[48];
float value;

void setup() {
  // serial to display instructions
  Serial.begin(115200);
  while(!Serial) {}
  // start communication with IMU 
  status = Imu.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // calibrating accelerometer
  Serial.println("Starting Accelerometer Calibration");
  Imu.calibrateAccel();
  Serial.println("Switch");
  delay(5000);
  Imu.calibrateAccel();
  Serial.println("Switch");
  delay(5000);
  Imu.calibrateAccel();
  Serial.println("Switch");
  delay(5000);
  Imu.calibrateAccel();
  Serial.println("Switch");
  delay(5000);
  Imu.calibrateAccel();
  Serial.println("Switch");
  delay(5000);
  Imu.calibrateAccel();
  Serial.println("Done");
  Serial.println("Starting Magnetometer Calibration");
  delay(5000);
  // calibrating magnetometer
  Imu.calibrateMag();
  // saving to EEPROM
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
  for (size_t i=0; i < sizeof(EepromBuffer); i++) {
    EEPROM.write(i,EepromBuffer[i]);
  }
  Serial.println("Done");
}

void loop() {}
