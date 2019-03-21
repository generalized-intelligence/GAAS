/*
uNavAHRS.cpp
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

#include "Arduino.h"
#include "uNavAHRS.h"

/* sets the duration for gathering IMU statistics, us */
void uNavAHRS::setInitializationDuration(uint32_t duration) {
  _duration = duration;
}

// time and measurement update for uNavAHRS
// gx, gy, gz input in rad/s
// ax, ay, az input in consistant units
// hx, hy, hz input in consistant units
bool uNavAHRS::update(float gx,float gy,float gz,float ax,float ay,float az,float hx, float hy, float hz) {

  // checking if gyro data updated
  if ((gx!=gx_)||(gy!=gy_)||(gz!=gz_)) {
    gyroUpdated_ = true;
    gx_ = gx;
    gy_ = gy;
    gz_ = gz;
  } else {
    gyroUpdated_ = false;
  }

  // checking if accel data updated
  if ((ax!=ax_)||(ay!=ay_)||(az!=az_)) {
    accelUpdated_ = true;
    ax_ = ax;
    ay_ = ay;
    az_ = az;
  } else {
    accelUpdated_ = false;
  }

  // checking if magnetometer data updated
  if ((hx!=hx_)||(hy!=hy_)||(hz!=hz_)) {
    magUpdated_ = true;
    hx_ = hx;
    hy_ = hy;
    hz_ = hz;
  } else {
    magUpdated_ = false;
  }

  if (!_initialized) {
    /* gather sensor statistics */
    if (!_timeLatch) {
      _startingTime = micros();
      _timeLatch = true;
    }
    if ((micros() - _startingTime) < _duration) {
      // Welfords algorithm for mean and variance

      if (gyroUpdated_) {
        _gyroCount++;
        _gyroDelta(0,0) = gx - _gyroBias(0,0);
        _gyroBias(0,0) = _gyroBias(0,0) + _gyroDelta(0,0) / ((float)_gyroCount);
        _gyroDelta2(0,0) = gx - _gyroBias(0,0);
        _gyroM2(0,0) = _gyroM2(0,0) + _gyroDelta(0,0) * _gyroDelta2(0,0);
        _gyroDelta(1,0) = gy - _gyroBias(1,0);
        _gyroBias(1,0) = _gyroBias(1,0) + _gyroDelta(1,0) / ((float)_gyroCount);
        _gyroDelta2(1,0) = gy - _gyroBias(1,0);
        _gyroM2(1,0) = _gyroM2(1,0) + _gyroDelta(1,0) * _gyroDelta2(1,0);
        _gyroDelta(2,0) = gz - _gyroBias(2,0);
        _gyroBias(2,0) = _gyroBias(2,0) + _gyroDelta(2,0) / ((float)_gyroCount);
        _gyroDelta2(2,0) = gz - _gyroBias(2,0);
        _gyroM2(2,0) = _gyroM2(2,0) + _gyroDelta(2,0) * _gyroDelta2(2,0);
        if (_gyroCount > 2) {
          _gyroVariance(0,0) = _gyroM2(0,0)/((float)(_gyroCount-1));
          _gyroVariance(1,0) = _gyroM2(1,0)/((float)(_gyroCount-1));
          _gyroVariance(2,0) = _gyroM2(2,0)/((float)(_gyroCount-1));
        }
      }

      if (accelUpdated_) {
        _accelCount++;
        _accelDelta(0,0) = ax - _accelMean(0,0);
        _accelMean(0,0) = _accelMean(0,0) + _accelDelta(0,0) / ((float)_accelCount);
        _accelDelta2(0,0) = ax - _accelMean(0,0);
        _accelM2(0,0) = _accelM2(0,0) + _accelDelta(0,0) * _accelDelta2(0,0);
        _accelDelta(1,0) = ay - _accelMean(1,0);
        _accelMean(1,0) = _accelMean(1,0) + _accelDelta(1,0) / ((float)_accelCount);
        _accelDelta2(1,0) = ay - _accelMean(1,0);
        _accelM2(1,0) = _accelM2(1,0) + _accelDelta(1,0) * _accelDelta2(1,0);
        _accelDelta(2,0) = az - _accelMean(2,0);
        _accelMean(2,0) = _accelMean(2,0) + _accelDelta(2,0) / ((float)_accelCount);
        _accelDelta2(2,0) = az - _accelMean(2,0);
        _accelM2(2,0) = _accelM2(2,0) + _accelDelta(2,0) * _accelDelta2(2,0);
        if (_accelCount > 2) {
          _accelVariance(0,0) = _accelM2(0,0)/((float)(_accelCount-1));
          _accelVariance(1,0) = _accelM2(1,0)/((float)(_accelCount-1));
          _accelVariance(2,0) = _accelM2(2,0)/((float)(_accelCount-1));
        }
      }

      if (magUpdated_) {
        _magCount++;
        _magDelta(0,0) = hx - _magMean(0,0);
        _magMean(0,0) = _magMean(0,0) + _magDelta(0,0) / ((float)_magCount);
        _magDelta2(0,0) = hx - _magMean(0,0);
        _magM2(0,0) = _magM2(0,0) + _magDelta(0,0) * _magDelta2(0,0);
        _magDelta(1,0) = hy - _magMean(1,0);
        _magMean(1,0) = _magMean(1,0) + _magDelta(1,0) / ((float)_magCount);
        _magDelta2(1,0) = hy - _magMean(1,0);
        _magM2(1,0) = _magM2(1,0) + _magDelta(1,0) * _magDelta2(1,0);
        _magDelta(2,0) = hz - _magMean(2,0);
        _magMean(2,0) = _magMean(2,0) + _magDelta(2,0) / ((float)_magCount);
        _magDelta2(2,0) = hz - _magMean(2,0);
        _magM2(2,0) = _magM2(2,0) + _magDelta(2,0) * _magDelta2(2,0);
        if (_magCount > 2) {
          _magVariance(0,0) = _magM2(0,0)/((float)(_magCount-1));
          _magVariance(1,0) = _magM2(1,0)/((float)(_magCount-1));
          _magVariance(2,0) = _magM2(2,0)/((float)(_magCount-1));
        }
      }

      if ((accelUpdated_)&&(magUpdated_)) {
        _quatCount++;
        // estimate the quaternion to get an idea of the initial state covariance
        _accel(0,0) = ax;
        _accel(1,0) = ay;
        _accel(2,0) = az;
        _mag(0,0) = hx;
        _mag(1,0) = hy;
        _mag(2,0) = hz;
        _accel.normalize();
        _mag.normalize();
        _Euler(1,0) = asin(_accel(0,0));
        _Euler(0,0) = asin(-_accel(1,0)/(cos(_Euler(1,0))));
        _Euler(2,0) = atan2(_mag(2,0)*sin(_Euler(0,0))-_mag(1,0)*cos(_Euler(0,0)),_mag(0,0)*cos(_Euler(1,0))+_mag(1,0)*sin(_Euler(1,0))*sin(_Euler(0,0))+_mag(2,0)*sin(_Euler(1,0))*cos(_Euler(0,0)));
        _Quat = Eul2Quat(_Euler);
        _quatDelta(0,0) = _Quat(0,0) - _quatMean(0,0);
        _quatMean(0,0) = _quatMean(0,0) + _quatDelta(0,0) / ((float)_quatCount);
        _quatDelta2(0,0) = _Quat(0,0) - _quatMean(0,0);
        _quatM2(0,0) = _quatM2(0,0) + _quatDelta(0,0) * _quatDelta2(0,0);
        _quatDelta(1,0) = _Quat(1,0) - _quatMean(1,0);
        _quatMean(1,0) = _quatMean(1,0) + _quatDelta(1,0) / ((float)_quatCount);
        _quatDelta2(1,0) = _Quat(1,0) - _quatMean(1,0);
        _quatM2(1,0) = _quatM2(1,0) + _quatDelta(1,0) * _quatDelta2(1,0);
        _quatDelta(2,0) = _Quat(2,0) - _quatMean(2,0);
        _quatMean(2,0) = _quatMean(2,0) + _quatDelta(2,0) / ((float)_quatCount);
        _quatDelta2(2,0) = _Quat(2,0) - _quatMean(2,0);
        _quatM2(2,0) = _quatM2(2,0) + _quatDelta(2,0) * _quatDelta2(2,0);
        _quatDelta(3,0) = _Quat(3,0) - _quatMean(3,0);
        _quatMean(3,0) = _quatMean(3,0) + _quatDelta(3,0) / ((float)_quatCount);
        _quatDelta2(3,0) = _Quat(3,0) - _quatMean(3,0);
        _quatM2(3,0) = _quatM2(3,0) + _quatDelta(3,0) * _quatDelta2(3,0);
        if (_quatCount > 2) {
          _quatVariance(0,0) = _quatM2(0,0)/((float)(_quatCount-1));
          _quatVariance(1,0) = _quatM2(1,0)/((float)(_quatCount-1));
          _quatVariance(2,0) = _quatM2(2,0)/((float)(_quatCount-1));
          _quatVariance(3,0) = _quatM2(3,0)/((float)(_quatCount-1));
        }
      }
    } else {
      /* initialize state */
      x_(0,0) = _quatMean(0,0);
      x_(1,0) = _quatMean(1,0);
      x_(2,0) = _quatMean(2,0);
      x_(3,0) = _quatMean(3,0);
      x_(4,0) = _gyroBias(0,0);
      x_(5,0) = _gyroBias(1,0);
      x_(6,0) = _gyroBias(2,0);
      _Quat = x_.block(0,0,4,1);
      _Quat.normalize();
      _initialEuler = Quat2Eul(_Quat);

      /* initialize process covariance */
      P_(0,0) = _quatVariance(0,0);
      P_(1,1) = _quatVariance(1,0);
      P_(2,2) = _quatVariance(2,0);
      P_(3,3) = _quatVariance(3,0);
      P_(4,4) = _gyroVariance(0,0);
      P_(5,5) = _gyroVariance(1,0);
      P_(6,6) = _gyroVariance(2,0);

      /* initialize gyro covariance matrix */
      Q_(0,0) = _gyroVariance(0,0);
      Q_(1,1) = _gyroVariance(1,0);
      Q_(2,2) = _gyroVariance(2,0);

      /* initialize measurement covariance matrix */
      R_(0,0) = _accelVariance(0,0);
      R_(1,1) = _accelVariance(1,0);
      R_(2,2) = _accelVariance(2,0);
      R_(3,3) = _magVariance(0,0);
      R_(4,4) = _magVariance(1,0);
      R_(5,5) = _magVariance(2,0);

      Ra_(0,0) = _accelVariance(0,0);
      Ra_(1,1) = _accelVariance(1,0);
      Ra_(2,2) = _accelVariance(2,0);

      /* initialize output matrix */
      M_.block(0,0,3,3) = Eigen::Matrix<float,3,3>::Identity();
      Ma_ = Eigen::Matrix<float,3,3>::Identity();

      /* initialize the time */
      _tprev = (float) micros()/1000000.0;

      /* initialization complete */
      _initialized = true;
    }
  } else {

    /* time update */
    if (gyroUpdated_) {
      // get the change in time
      _tnow = (float) micros()/1000000.0;
      _dt = _tnow - _tprev;
      _tprev = _tnow;
      // state transition matrix
      F_(0,0) = 1.0;                   F_(0,1) = -0.5*_dt*(gx-x_(4,0)); F_(0,2) = -0.5*_dt*(gy-x_(5,0)); F_(0,3) = -0.5*_dt*(gz-x_(6,0));  
      F_(1,0) = 0.5*_dt*(gx-x_(4,0));  F_(1,1) = 1.0;                   F_(1,2) = 0.5*_dt*(gz-x_(6,0));  F_(1,3) = -0.5*_dt*(gy-x_(5,0));                    
      F_(2,0) = 0.5*_dt*(gy-x_(5,0));  F_(2,1) = -0.5*_dt*(gz-x_(6,0)); F_(2,2) = 1.0;                   F_(2,3) = 0.5*_dt*(gx-x_(4,0));
      F_(3,0) = 0.5*_dt*(gz-x_(6,0));  F_(3,1) = 0.5*_dt*(gy-x_(5,0));  F_(3,2) = -0.5*_dt*(gx-x_(4,0)); F_(3,3) = 1.0;
      F_(0,4) = 0.5*_dt*x_(1,0);       F_(0,5) = 0.5*_dt*x_(2,0);       F_(0,6) = 0.5*_dt*x_(3,0);
      F_(1,4) = -0.5*_dt*x_(0,0);      F_(1,5) = -0.5*_dt*x_(2,0);      F_(1,6) = 0.5*_dt*x_(3,0);
      F_(2,4) = -0.5*_dt*x_(3,0);      F_(2,5) = -0.5*_dt*x_(0,0);      F_(2,6) = 0.5*_dt*x_(1,0);
      F_(3,4) = 0.5*_dt*x_(2,0);       F_(3,5) = -0.5*_dt*x_(1,0);      F_(3,6) = 0.5*_dt*x_(0,0);
      F_(4,4) = 1.0;                   F_(5,5) = 1.0;                   F_(6,6) = 1.0;
      // process noise matrix
      L_(0,0) = -0.5*_dt*x_(1,0);  L_(0,1) = -0.5*_dt*x_(2,0);  L_(0,2) = -0.5*_dt*x_(3,0);
      L_(1,0) = 0.5*_dt*x_(0,0);   L_(1,1) = -0.5*_dt*x_(3,0);  L_(1,2) = 0.5*_dt*x_(2,0);
      L_(2,0) = 0.5*_dt*x_(3,0);   L_(2,1) = 0.5*_dt*x_(0,0);   L_(2,2) = -0.5*_dt*x_(1,0); 
      L_(3,0) = -0.5*_dt*x_(2,0);  L_(3,1) = 0.5*_dt*x_(1,0);   L_(3,2) = 0.5*_dt*x_(0,0);
      L_(4,0) = 1.0;               L_(4,1) = 0.0;               L_(4,2) = 0.0;
      L_(5,0) = 0.0;               L_(5,1) = 1.0;               L_(5,2) = 0.0;
      L_(6,0) = 0.0;               L_(6,1) = 0.0;               L_(6,2) = 1.0;
      // covariance
      P_ = F_*P_*F_.transpose() + L_*Q_*L_.transpose();
      // state
      x_ = F_*x_;

      /* accel and magnetometer measurement update */
      if ((accelUpdated_)&&(magUpdated_)) {
        _accel(0,0) = ax;
        _accel(1,0) = ay;
        _accel(2,0) = az;
        _accel.normalize();
        _mag(0,0) = hx;
        _mag(1,0) = hy;
        _mag(2,0) = hz;
        _mag.normalize();
        // measurement jacobian
        H_(0,0) = 2.0*x_(2,0);  H_(0,1) = -2.0*x_(3,0);  H_(0,2) = 2.0*x_(0,0);   H_(0,3) = -2.0*x_(1,0);
        H_(1,0) = -2.0*x_(1,0); H_(1,1) = -2.0*x_(0,0);  H_(1,2) = -2.0*x_(3,0);  H_(1,3) = -2.0*x_(2,0);
        H_(2,0) = -2.0*x_(0,0); H_(2,1) = 2.0*x_(1,0);   H_(2,2) = 2.0*x_(2,0);   H_(2,3) = -2.0*x_(3,0);
        H_(3,0) = (-4.0*x_(0,0)*x_(0,0)*x_(3,0)+4.0*x_(1,0)*x_(1,0)*x_(3,0)-2.0*x_(3,0)-8.0*x_(1,0)*x_(2,0)*x_(0,0))/(pow(2.0*x_(1,0)*x_(2,0)+2.0*x_(0,0)*x_(3,0),2.0)+pow((2.0*x_(1,0)*x_(1,0)+2.0*x_(0,0)*x_(0,0)-1.0),2.0));
        H_(3,1) = (4.0*x_(2,0)*x_(0,0)*x_(0,0)-4.0*x_(1,0)*x_(1,0)*x_(2,0)-2.0*x_(2,0)-8.0*x_(1,0)*x_(0,0)*x_(3,0))/(pow(2.0*x_(1,0)*x_(2,0)+2.0*x_(0,0)*x_(3,0),2.0)+pow((2.0*x_(1,0)*x_(1,0)+2.0*x_(0,0)*x_(0,0)-1.0),2.0));
        H_(3,2) = 2.0*x_(1,0)*(2.0*x_(1,0)*x_(1,0)+2.0*x_(0,0)*x_(0,0)-1.0)/(pow(2.0*x_(1,0)*x_(2,0)+2.0*x_(0,0)*x_(3,0),2.0)+pow((2.0*x_(1,0)*x_(1,0)+2.0*x_(0,0)*x_(0,0)-1.0),2.0));
        H_(3,3) = 2.0*x_(0,0)*(2.0*x_(0,0)*x_(0,0)+2.0*x_(1,0)*x_(1,0)-1.0)/(pow(2.0*x_(1,0)*x_(2,0)+2.0*x_(0,0)*x_(3,0),2.0)+pow((2.0*x_(1,0)*x_(1,0)+2.0*x_(0,0)*x_(0,0)-1.0),2.0));
        // noise jacobian
        theta = asin(_accel(0,0));
        phi = asin(-_accel(1,0)/(cos(theta)));
        M_(3,3) = (-cos(theta)*(_mag(2,0)*sin(phi)-_mag(1,0)*cos(phi)))/(pow(_mag(2,0)*sin(phi)-_mag(1,0)*cos(phi),2.0)+pow(_mag(0,0)*cos(theta)+_mag(2,0)*cos(phi)*sin(theta)+_mag(1,0)*sin(phi)*sin(theta),2.0));
        M_(3,4) = (-_mag(2,0)*cos(phi)*cos(phi)*sin(theta)-_mag(0,0)*cos(theta)*cos(phi)-_mag(2,0)*sin(phi)*sin(phi)*sin(theta))/(pow(_mag(2,0)*sin(phi)-_mag(1,0)*cos(phi),2.0)+pow(_mag(0,0)*cos(theta)+_mag(2,0)*cos(phi)*sin(theta)+_mag(1,0)*sin(phi)*sin(theta),2.0));
        M_(3,5) = (_mag(0,0)*sin(phi)*cos(theta)+_mag(1,0)*sin(phi)*sin(phi)*sin(theta)+_mag(1,0)*cos(phi)*cos(phi)*sin(theta))/(pow(_mag(2,0)*sin(phi)-_mag(1,0)*cos(phi),2.0f)+pow(_mag(0,0)*cos(theta)+_mag(2,0)*cos(phi)*sin(theta)+_mag(1,0)*sin(phi)*sin(theta),2.0));
        // kalman gain
        K_ = P_*H_.transpose()*(H_*P_*H_.transpose()+M_*R_*M_.transpose()).inverse();
        // estimated output
        h_(0,0) = -2.0*(x_(1,0)*x_(3,0)-x_(0,0)*x_(2,0));
        h_(1,0) = -2.0*(x_(0,0)*x_(1,0)+x_(2,0)*x_(3,0));
        h_(2,0) = -(x_(0,0)*x_(0,0)-x_(1,0)*x_(1,0)-x_(2,0)*x_(2,0)+x_(3,0)*x_(3,0));
        h_(3,0) = atan2(2.0*x_(1,0)*x_(2,0)+2.0*x_(0,0)*x_(3,0),2.0*x_(0,0)*x_(0,0)+2.0*x_(1,0)*x_(1,0)-1.0);
        // measured output
        y_(0,0) = _accel(0,0);
        y_(1,0) = _accel(1,0);
        y_(2,0) = _accel(2,0);
        y_(3,0) = atan2(_mag(2,0)*sin(phi)-_mag(1,0)*cos(phi),_mag(0,0)*cos(theta)+_mag(1,0)*sin(theta)*sin(phi)+_mag(2,0)*sin(theta)*cos(phi));
        // state
        x_ = x_ + K_*(y_-h_);
        // covariance
        P_ = (Eigen::Matrix<float,7,7>::Identity()-K_*H_)*P_;  

        /* accel only measurement update */
      } else if (accelUpdated_) {
        _accel(0,0) = ax;
        _accel(1,0) = ay;
        _accel(2,0) = az;
        _accel.normalize();
        // measurement jacobian
        Ha_(0,0) = 2.0*x_(2,0);  Ha_(0,1) = -2.0*x_(3,0);  Ha_(0,2) = 2.0*x_(0,0);   Ha_(0,3) = -2.0*x_(1,0);
        Ha_(1,0) = -2.0*x_(1,0); Ha_(1,1) = -2.0*x_(0,0);  Ha_(1,2) = -2.0*x_(3,0);  Ha_(1,3) = -2.0*x_(2,0);
        Ha_(2,0) = -2.0*x_(0,0); Ha_(2,1) = 2.0*x_(1,0);   Ha_(2,2) = 2.0*x_(2,0);   Ha_(2,3) = -2.0*x_(3,0);
        // kalman gain
        Ka_ = P_*Ha_.transpose()*(Ha_*P_*Ha_.transpose()+Ma_*Ra_*Ma_.transpose()).inverse();
        // estimated output
        ha_(0,0) = -2.0*(x_(1,0)*x_(3,0)-x_(0,0)*x_(2,0));
        ha_(1,0) = -2.0*(x_(0,0)*x_(1,0)+x_(2,0)*x_(3,0));
        ha_(2,0) = -(x_(0,0)*x_(0,0)-x_(1,0)*x_(1,0)-x_(2,0)*x_(2,0)+x_(3,0)*x_(3,0));
        // measured output
        ya_(0,0) = _accel(0,0);
        ya_(1,0) = _accel(1,0);
        ya_(2,0) = _accel(2,0);
        // state
        x_ = x_ + Ka_*(ya_-ha_);
        // covariance
        P_ = (Eigen::Matrix<float,7,7>::Identity()-Ka_*Ha_)*P_;
      }

      // transform best estimate of attitude quaternion to euler angles
      _Quat = x_.block(0,0,4,1);
      _Quat.normalize();
      _Euler = Quat2Eul(_Quat);
    }
  }

  if (_initialized) {
    return true;
  } else {
    return false;
  }
}

/* Returns the roll angle, rad */
float uNavAHRS::getRoll_rad() {
  return _Euler(0,0);
}

/* Returns the pitch angle, rad */
float uNavAHRS::getPitch_rad() {
  return _Euler(1,0);
}

/* Returns the yaw angle, rad */
float uNavAHRS::getYaw_rad() {
  return constrainAngle180(_Euler(2,0)-_initialEuler(2,0));
}

/* Returns the heading angle, rad */
float uNavAHRS::getHeading_rad() {
  return constrainAngle360(_Euler(2,0));
}

/* Returns the unit quaternion */
void uNavAHRS::getQuaternion(float *qw, float *qi, float *qj, float *qk) {
  *qw = _Quat(0,0);
  *qi = _Quat(1,0);
  *qj = _Quat(2,0);
  *qk = _Quat(3,0);
}

/* Returns the gyro bias in the X axis, rad/s */
float uNavAHRS::getGyroBiasX_rads() {
  return x_(4,0);
}

/* Returns the gyro bias in the Y axis, rad/s */
float uNavAHRS::getGyroBiasY_rads() {
  return x_(5,0);
}

/* Returns the gyro bias in the Z axis, rad/s */
float uNavAHRS::getGyroBiasZ_rads() {
  return x_(6,0);
}

/* Transforms euler angles (phi, theta, psi) to quaternion (q[1,0,0,0]) */
Eigen::Matrix<float,4,1> uNavAHRS::Eul2Quat(Eigen::Matrix<float,3,1> eul) {
  Eigen::Matrix<float,4,1> q;
  q(0,0) = cos(eul(2,0)/2.0)*cos(eul(1,0)/2.0)*cos(eul(0,0)/2.0) + sin(eul(2,0)/2.0)*sin(eul(1,0)/2.0)*sin(eul(0,0)/2.0);  
  q(1,0) = cos(eul(2,0)/2.0)*cos(eul(1,0)/2.0)*sin(eul(0,0)/2.0) - sin(eul(2,0)/2.0)*sin(eul(1,0)/2.0)*cos(eul(0,0)/2.0);
  q(2,0) = cos(eul(2,0)/2.0)*sin(eul(1,0)/2.0)*cos(eul(0,0)/2.0) + sin(eul(2,0)/2.0)*cos(eul(1,0)/2.0)*sin(eul(0,0)/2.0);  
  q(3,0) = sin(eul(2,0)/2.0)*cos(eul(1,0)/2.0)*cos(eul(0,0)/2.0) - cos(eul(2,0)/2.0)*sin(eul(1,0)/2.0)*sin(eul(0,0)/2.0);
  return q;
}

/* Transforms quaternion (q[1,0,0,0]) to euler angles (phi, theta, psi) */
Eigen::Matrix<float,3,1> uNavAHRS::Quat2Eul(Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,3,1> eul;
  eul(0,0) = atan2(2.0*q(2,0)*q(3,0)+2.0*q(0,0)*q(1,0),2.0*q(0,0)*q(0,0)+2.0l*q(3,0)*q(3,0)-1.0);
  eul(1,0) = asin(-2.0*q(1,0)*q(3,0)+2.0*q(0,0)*q(2,0));
  eul(2,0) = atan2(2.0*q(1,0)*q(2,0)+2.0*q(0,0)*q(3,0),2.0*q(0,0)*q(0,0)+2.0l*q(1,0)*q(1,0)-1.0);
  return eul;
}

/* Bound angle between -180 and 180 */
float uNavAHRS::constrainAngle180(float dta) {
  if(dta >  PI) dta -= (PI*2.0);
  if(dta < -PI) dta += (PI*2.0);
  return dta;
}

/* Bound angle between 0 and 360 */
float uNavAHRS::constrainAngle360(float dta) {
  dta = fmod(dta,2.0*PI);
  if (dta < 0.0)
    dta += 2.0*PI;
  return dta;
}
