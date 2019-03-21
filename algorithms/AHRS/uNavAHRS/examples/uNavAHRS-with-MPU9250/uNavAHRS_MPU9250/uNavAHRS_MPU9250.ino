/*
uNavAHRS_MPU9250.ino
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

#include "uNavAHRS.h"
#include "MPU9250.h"
#include "EEPROM.h"

// an MPU-9250 object on SPI bus 0 with chip select 10
MPU9250 Imu(SPI,10);
int status;
// a uNavAHRS object
uNavAHRS Filter;
// a flag for when the MPU-9250 has new data
volatile int newData;
// EEPROM buffer and variables to load accel and mag bias 
// and scale factors from CalibrateMPU9250.ino
uint8_t EepromBuffer[48];
float axb, axs, ayb, ays, azb, azs;
float hxb, hxs, hyb, hys, hzb, hzs;
// timers to measure performance
unsigned long tstart, tstop;

void setup() {
  // serial to display data
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
  // load and set accel and mag bias and scale
  // factors from CalibrateMPU9250.ino 
  for (size_t i=0; i < sizeof(EepromBuffer); i++) {
    EepromBuffer[i] = EEPROM.read(i);
  }
  memcpy(&axb,EepromBuffer+0,4);
  memcpy(&axs,EepromBuffer+4,4);
  memcpy(&ayb,EepromBuffer+8,4);
  memcpy(&ays,EepromBuffer+12,4);
  memcpy(&azb,EepromBuffer+16,4);
  memcpy(&azs,EepromBuffer+20,4);
  memcpy(&hxb,EepromBuffer+24,4);
  memcpy(&hxs,EepromBuffer+28,4);
  memcpy(&hyb,EepromBuffer+32,4);
  memcpy(&hys,EepromBuffer+36,4);
  memcpy(&hzb,EepromBuffer+40,4);
  memcpy(&hzs,EepromBuffer+44,4);

  Imu.setAccelCalX(axb,axs);
  Imu.setAccelCalY(ayb,ays);
  Imu.setAccelCalZ(azb,azs);

  Imu.setMagCalX(hxb,hxs);
  Imu.setMagCalY(hyb,hys);
  Imu.setMagCalZ(hzb,hzs);

  // setting a 41 Hz DLPF bandwidth
  Imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 9 for a 100 Hz update rate
  Imu.setSrd(9);
  // enabling the data ready interrupt
  Imu.enableDataReadyInterrupt();
  // attaching the interrupt to microcontroller pin 1
  pinMode(1,INPUT);
  attachInterrupt(1,runFilter,RISING);
}

void loop() {
  if (newData == 1) {
    newData = 0;
    tstart = micros();
    // read the sensor
    Imu.readSensor();
    // update the filter
    if (Filter.update(Imu.getGyroX_rads(),Imu.getGyroY_rads(),Imu.getGyroZ_rads(),Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss(),Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT())) {
      tstop = micros();
      Serial.print(Filter.getPitch_rad()*180.0f/PI);
      Serial.print("\t");
      Serial.print(Filter.getRoll_rad()*180.0f/PI);
      Serial.print("\t");
      Serial.print(Filter.getYaw_rad()*180.0f/PI);
      Serial.print("\t");
      Serial.print(Filter.getHeading_rad()*180.0f/PI);
      Serial.print("\t");
      Serial.print(Filter.getGyroBiasX_rads(),6);
      Serial.print("\t");
      Serial.print(Filter.getGyroBiasY_rads(),6);
      Serial.print("\t");
      Serial.print(Filter.getGyroBiasZ_rads(),6);
      Serial.print("\t");
      Serial.println(tstop - tstart);
    }
  }
}

void runFilter() {
  newData = 1;
}
