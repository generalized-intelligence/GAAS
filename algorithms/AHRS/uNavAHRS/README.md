# uNavAHRS
uNav Attitude and Heading Reference System 7 State EKF Arduino Library.

***Developing with the [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) or [LC](https://www.pjrc.com/store/teensylc.html)? Consider buying our [Teensy Motion Backpack](http://bolderflight.com/products/teensy/motion/), which integrates an MPU-9250 and BME-280 into a stackable add-on module, thoughtfully designed to integrate perfectly with the Teensy. Check out all of our wonderfully small and powerful [Teensy Backpacks](http://bolderflight.com/products/teensy/)***

# Description
The uNav Attitude and Heading Reference System (AHRS) is a 7 state Extended Kalman Filter (EKF) to estimate attitude and heading from IMU data. The 7 states comprise a quaternion and three gyro biases. It uses gyro measurements to propogate the state; accelerometers are used as a measurement update on the pitch and roll channels and magnetometers as a measurement update on the yaw channel to constrain drift.

# Usage

## Installation
This library requires [Eigen](https://github.com/bolderflight/Eigen) to compile. First download or clone [Eigen](https://github.com/bolderflight/Eigen) into your Arduino/libraries folder, then download or clone this library into your Arduino/libraries folder. Additionally, this library requires IMU measurements. For the included examples, an [MPU-9250 IMU](https://github.com/bolderflight/MPU9250) is used, and its library will need to be installed as well. Finally, because this library is using accelerometers and magnetometers as a measurement update, the IMU used should be well calibrated.

## Function Description

### Object Declaration
This library uses the default constructor. The following is an example of declaring a uNavAHRS object called *Filter*.

```C++
uNavAHRS Filter;
```

### Setup Functions
**(optional) void setInitializationDuration(uint32_t duration)**
Statistical data are gathered from the IMU measurements for automatically setting up the filter. By default this initialization takes 2 minutes. Optionally, this function can be used to set initialization durations other than the default value. The input is initialization time in microseconds.

```C++
Filter.setInitializationDuration(60000000);
```

### Data Collection Functions

**bool update(float ias,float p,float q,float r,float ax,float ay,float az,float hx, float hy, float hz)** updates the filter with new IMU measurements. Inputs are:

* float p: gyro measurement in the x direction, units are rad/s.
* float q: gyro measurement in the y direction, units are rad/s.
* float r: gyro measurement in the z direction, units are rad/s.
* float ax: accelerometer measurement in the x direction, units need to be consistant across all accelerometer measurements used
* float ay: accelerometer measurement in the y direction, units need to be consistant across all accelerometer measurements used
* float az: accelerometer measurement in the z direction, units need to be consistant across all accelerometer measurements used
* float hx: magnetometer measurement in the x direction, units need to be consistant across all magnetometer measurements used.
* float hy: magnetometer measurement in the y direction, units need to be consistant across all magnetometer measurements used.
* float hz: magnetometer measurement in the z direction, units need to be consistant across all magnetometer measurements used.

Please note that all measurements need to be given in the [defined axis system](#axis-system). Measurements need to be provided in integer rates of each other, which the gyro measurements always updated. The *update* function checks each set of measurements to see if they've been updated and uses the following logic:

* Gyro, accelerometer, and magnetometer measurements updated: filter time update, accelerometer and magnetometer measurement update.
* Gyro and accelerometer measurements updated: filter time update, accelerometer measurement update.
* Gyro measurements updated: filter time update.

The filter automatically initializes itself. Calls to *update* return false if the filter has not completed its initialization and return true after initialization is complete. The duration of initialization can be optionally set with the *setInitializationDuration* function, otherwise, the filter takes 2 minutes to initialize by default.

```C++
// read the sensor
Imu.readSensor();
// update the filter
Filter.update(Imu.getGyroX_rads(),Imu.getGyroY_rads(),Imu.getGyroZ_rads(),Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss(),Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT());
```

**float getRoll_rad()** returns the roll angle in units of rad.

```C++
float roll;
roll = Filter.getRoll_rad();
```

**float getPitch_rad()** returns the pitch angle in units of rad.

```C++
float pitch;
pitch = Filter.getPitch_rad();
```

**float getYaw_rad()** returns the yaw angle in units of rad.

```C++
float yaw;
yaw = Filter.getYaw_rad();
```

**float getHeading_rad()** returns the heading angle in units of rad.

```C++
float heading;
heading = Filter.getHeading_rad();
```

**float getGyroBiasX_rads** returns the current gyro bias in the x direction in units of rad/s.

```C++
float gxb;
gxb = Filter.getGyroBiasX_rads();
```

**float getGyroBiasY_rads** returns the current gyro bias in the y direction in units of rad/s.

```C++
float gyb;
gyb = Filter.getGyroBiasY_rads();
```

**float getGyroBiasZ_rads** returns the current gyro bias in the z direction in units of rad/s.

```C++
float gzb;
gzb = Filter.getGyroBiasZ_rads();
```

## <a name="axis-system"></a>Axis System
This library expects IMU data to be input in a defined axis system, which is shown below. It is a right handed coordinate system with x-axis pointed forward, the y-axis to the right, and the z-axis positive down, common in aircraft dynamics. Pitch is defined as a rotation angle around the y-axis with level as zero and roll is defined as a rotation angle around the x-axis with level as zero. Yaw is defined as a rotation angle around the z-axis with zero defined as the starting orientation. Heading is defined as a rotation angle around the z-axis with zero defined as magnetic north.

<img src="https://github.com/bolderflight/MPU9250/blob/master/docs/MPU-9250-AXIS.png" alt="Common Axis System" width="250">

## Example List
* **uNavAHRS-with-MPU9250**: demonstrates using this filter with an MPU-9250 IMU. *CalibrateMPU9250.ino* is used to calibrate the MPU-9250 IMU and store the calibration coefficients in EEPROM. *uNavAHRS_MPU9250.ino* uses the MPU-9250 IMU as measurement input to the uNav AHRS filter, which is run at a rate of 100 Hz. 
