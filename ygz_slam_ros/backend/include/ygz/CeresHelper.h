#ifndef CERESHELPER_H
#define CERESHELPER_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace ygz{
    
    // limit
    template <typename T>
    T NormalizeAngle(const T& angle_degrees) {
    if (angle_degrees > T(180.0))
        return angle_degrees - T(360.0);
    else if (angle_degrees < T(-180.0))
        return angle_degrees + T(360.0);
    else
        return angle_degrees;
    };

    class AngleLocalParameterization {
    public:

    template <typename T>
    bool operator()(const T* theta_radians, const T* delta_theta_radians,
                    T* theta_radians_plus_delta) const {
        *theta_radians_plus_delta =
            NormalizeAngle(*theta_radians + *delta_theta_radians);

        return true;
    }

    static ceres::LocalParameterization* Create() {
        return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                        1, 1>);
    }
    };

    template <typename T> 
    void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
    {
        T y = yaw / T(180.0) * T(M_PI);
        T p = pitch / T(180.0) * T(M_PI);
        T r = roll / T(180.0) * T(M_PI);

        R[0] = cos(y) * cos(p);
        R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
        R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
        R[3] = sin(y) * cos(p);
        R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
        R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
        R[6] = -sin(p);
        R[7] = cos(p) * sin(r);
        R[8] = cos(p) * cos(r);
    };

    template <typename T> 
    void RotationMatrixTranspose(const T R[9], T inv_R[9])
    {
        inv_R[0] = R[0];
        inv_R[1] = R[3];
        inv_R[2] = R[6];
        inv_R[3] = R[1];
        inv_R[4] = R[4];
        inv_R[5] = R[7];
        inv_R[6] = R[2];
        inv_R[7] = R[5];
        inv_R[8] = R[8];
    };

    template <typename T> 
    void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
    {
        r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
        r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
        r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
    };

    struct FourDOFError
    {
        FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
                    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){}

        template <typename T>
        bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
        {
            T t_w_ij[3];
            t_w_ij[0] = tj[0] - ti[0];
            t_w_ij[1] = tj[1] - ti[1];
            t_w_ij[2] = tj[2] - ti[2];

            // euler to rotation
            T w_R_i[9];
            YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
            // rotation transpose
            T i_R_w[9];
            RotationMatrixTranspose(w_R_i, i_R_w);
            // rotation matrix rotate point
            T t_i_ij[3];
            RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

            residuals[0] = (t_i_ij[0] - T(t_x));
            residuals[1] = (t_i_ij[1] - T(t_y));
            residuals[2] = (t_i_ij[2] - T(t_z));
            residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));

            return true;
        }

        static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                        const double relative_yaw, const double pitch_i, const double roll_i) 
        {
        return (new ceres::AutoDiffCostFunction<FourDOFError, 4, 1, 3, 1, 3>(
                    new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
        }

        double t_x, t_y, t_z;
        double relative_yaw, pitch_i, roll_i;
        
    };

    struct FourDOFWeightError
    {
        FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
                    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
                        weight = 1;
                    }

        template <typename T>
        bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
        {
            T t_w_ij[3];
            t_w_ij[0] = tj[0] - ti[0];
            t_w_ij[1] = tj[1] - ti[1];
            t_w_ij[2] = tj[2] - ti[2];

            // euler to rotation
            T w_R_i[9];
            YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
            // rotation transpose
            T i_R_w[9];
            RotationMatrixTranspose(w_R_i, i_R_w);
            // rotation matrix rotate point
            T t_i_ij[3];
            RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

            residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
            residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
            residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
            residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);

            return true;
        }

        static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                        const double relative_yaw, const double pitch_i, const double roll_i) 
        {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError, 4, 1, 3, 1, 3>(
                    new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
        }

        double t_x, t_y, t_z;
        double relative_yaw, pitch_i, roll_i;
        double weight;

    };

}

#endif
