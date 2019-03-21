/*
uNavAHRS.h
Brian R Taylor
brian.taylor@bolderflight.com
Bolder Flight Systems
Copyright 2017 Bolder Flight Systems

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

#ifndef UNAVAHRS_h
#define UNAVAHRS_h

#include "Arduino.h"
#include "Eigen.h"
#include <Eigen/Dense>

class uNavAHRS {
	public:
    void setInitializationDuration(uint32_t duration);
    bool update(float gx,float gy,float gz,float ax,float ay,float az,float hx, float hy, float hz);
    float getRoll_rad();
    float getPitch_rad();
    float getYaw_rad();
    float getHeading_rad();
    void getQuaternion(float *qw, float *qi, float *qj, float *qk);
    float getGyroBiasX_rads();
    float getGyroBiasY_rads();
    float getGyroBiasZ_rads();
  private:
    // initialized
    bool _initialized = false;
    // timing
    float _tnow, _tprev, _dt;
    // store previous sensor values to see when updated
    float gx_, gy_, gz_;
    float ax_, ay_, az_;
    float hx_, hy_, hz_;
    bool gyroUpdated_, accelUpdated_, magUpdated_;
    /* sensor statistics */
    // latch the starting time
    bool _timeLatch = false;
    uint32_t _startingTime;
    uint32_t _duration = 120*1000*1000;
    uint32_t _gyroCount, _accelCount, _magCount, _quatCount;
    Eigen::Matrix<float,3,1> _gyroBias;
    Eigen::Matrix<float,3,1> _gyroVariance;
    Eigen::Matrix<float,3,1> _gyroDelta;
    Eigen::Matrix<float,3,1> _gyroDelta2;
    Eigen::Matrix<float,3,1> _gyroM2;
    Eigen::Matrix<float,3,1> _accelMean;
    Eigen::Matrix<float,3,1> _accelVariance;
    Eigen::Matrix<float,3,1> _accelDelta;
    Eigen::Matrix<float,3,1> _accelDelta2;
    Eigen::Matrix<float,3,1> _accelM2;
    Eigen::Matrix<float,3,1> _magMean;
    Eigen::Matrix<float,3,1> _magVariance; 
    Eigen::Matrix<float,3,1> _magDelta;
    Eigen::Matrix<float,3,1> _magDelta2;
    Eigen::Matrix<float,3,1> _magM2;
    Eigen::Matrix<float,4,1> _quatMean;
    Eigen::Matrix<float,4,1> _quatVariance; 
    Eigen::Matrix<float,4,1> _quatDelta;
    Eigen::Matrix<float,4,1> _quatDelta2;
    Eigen::Matrix<float,4,1> _quatM2;
    /* starting orientation */
    Eigen::Matrix<float,3,1> _initialEuler;
    /* temporary values */
    Eigen::Matrix<float,3,1> _accel;
    Eigen::Matrix<float,3,1> _mag;
    Eigen::Matrix<float,3,1> _Euler;
    Eigen::Matrix<float,4,1> _Quat;
    float theta, phi;
    /* kalman filter time update */
    // state matrix
    Eigen::Matrix<float,7,1> x_;
    // state transition matrix
    Eigen::Matrix<float,7,7> F_;
    // state covariance matrix
    Eigen::Matrix<float,7,7> P_;
    // gyro covariance matrix
    Eigen::Matrix<float,3,3> Q_;
    // process noise matrix
    Eigen::Matrix<float,7,3> L_;
    /* kalman filter accel and mag measurement update */
    // measurement jacobian
    Eigen::Matrix<float,4,7> H_;
    // measurement covariance matrix
    Eigen::Matrix<float,6,6> R_;
    // measurement output matrix
    Eigen::Matrix<float,4,6> M_;
    // kalman gain
    Eigen::Matrix<float,7,4> K_;
    // estimated output
    Eigen::Matrix<float,4,1> h_;
    // measured output
    Eigen::Matrix<float,4,1> y_;
    /* kalman filter accel only measurement update */
    // measurement jacobian
    Eigen::Matrix<float,3,7> Ha_;
    // measurement covariance matrix
    Eigen::Matrix<float,3,3> Ra_;
    // measurement output matrix
    Eigen::Matrix<float,3,3> Ma_;
    // kalman gain
    Eigen::Matrix<float,7,3> Ka_;
    // estimated output
    Eigen::Matrix<float,3,1> ha_;
    // measured output
    Eigen::Matrix<float,3,1> ya_;
    // functions to transform attitude
    Eigen::Matrix<float,4,1> Eul2Quat(Eigen::Matrix<float,3,1> eul);
    Eigen::Matrix<float,3,1> Quat2Eul(Eigen::Matrix<float,4,1> q);
    float constrainAngle180(float dta);
    float constrainAngle360(float dta);
};

#endif
