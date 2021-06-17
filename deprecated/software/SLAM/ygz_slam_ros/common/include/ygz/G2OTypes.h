#ifndef YGZ_G2OTYPES_H
#define YGZ_G2OTYPES_H

#include "ygz/NumTypes.h"
#include "ygz/Settings.h"
#include "ygz/IMUPreIntegration.h"
#include "ygz/Camera.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/edge_pointxyz.h>

using namespace Eigen;

using namespace g2o;
// 在优化中要用到的g2o点和边

namespace ygz {

    struct CameraParam;

    struct Frame;

    // ---------------------------------------------------------------------------------------------------------
    // 各种顶点
    /**
     * VIO的Pose
     * 参数化为P+R，右乘更新，P在前
     */
    class VertexPR : public BaseVertex<6, Vector6d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexPR() : BaseVertex<6, Vector6d>() {}

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void setToOriginImpl() override {
            _estimate.setZero();
        }

        virtual void oplusImpl(const double *update_) override {
            // P直接加，R右乘
            _estimate.segment<3>(0) += Vector3d(update_[0], update_[1], update_[2]);
            _estimate.segment<3>(3) = SO3d::log(
                    SO3d::exp(_estimate.segment<3>(3)) *
                    SO3d::exp(Vector3d(update_[3], update_[4], update_[5])));
        }

        Matrix3d R() const {
            return SO3d::exp(_estimate.segment<3>(3)).matrix();
        }

        Vector3d t() const {
            return _estimate.head<3>();
        }
    };
    
    
    // TODO GPS vertex--------------------------------------------------
    typedef g2o::VertexPointXYZ VertexGPS;
    //------------------------------------------------------------------
    
    //TODO Attitude vertex
    
    ///typedef g2o::VertexPointXYZ VertexAtti;
    
    
    // Speed
    typedef g2o::VertexPointXYZ VertexSpeed;

    // Bias Acce
    typedef g2o::VertexPointXYZ VertexAcceBias;

    // Bias Gyro
    typedef g2o::VertexPointXYZ VertexGyrBias;

    /**
     * @brief The VertexGravityW class
     * 重力方向的顶点，估计的是重力的旋转
     */
    class VertexGravityW : public BaseVertex<2, Vector3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexGravityW();

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        virtual void setToOriginImpl() {
            _estimate = Vector3d(0, 0, setting::gravity);
        }

        virtual void oplusImpl(const double *update_) {
            _estimate = SO3d::exp(Vector3d(update_[0], update_[1], 0)) * _estimate;
        }
    };

    /**
     * 逆深度地图点
     * _estimate 为逆深度
     */
    class VertexPointInvDepth : public BaseVertex<1, double> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexPointInvDepth() : BaseVertex<1, double>() {};

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        virtual void setToOriginImpl() {
            _estimate = 1.0;
        }

        virtual void oplusImpl(const double *update) {
            _estimate += update[0];
        }
    };


    // ---------------------------------------------------------------------------------------------------------
    /**
     * 各种边
     */

    /**
     * Edge of inverse depth prior for stereo-triangulated mappoints
     * Vertex: inverse depth map point
     *
     * Note: User should set the information matrix (inverse covariance) according to feature position uncertainty and baseline
     */
    class EdgeIDPPrior : public BaseUnaryEdge<1, double, VertexPointInvDepth> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeIDPPrior() : BaseUnaryEdge<1, double, VertexPointInvDepth>() {}

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

        virtual void linearizeOplus() override;
    };

    /**
    * Edge of reprojection error in one frame.
    * Vertex 0: inverse depth map point
    * Veretx 1: reference KF PR
    * Vertex 2: current frame PR
    * Vertex 3: extrinsic pose Tbc(or Tcb)
    **/
    class EdgePRIDP : public BaseMultiEdge<2, Vector2d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // give the normalized x, y and camera intrinsics
        EdgePRIDP(double x, double y, CameraParam *cam) : BaseMultiEdge<2, Vector2d>() {
            resize(4);
            this->x = x;
            this->y = y;
            this->mpCam = cam;
        }

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

        virtual void linearizeOplus() override;

        bool isDepthValid() {
            return dynamic_cast<const VertexPointInvDepth *>( _vertices[0])->estimate() > 0;
        }

    protected:
        // [x,y] in normalized image plane in reference KF
        double x = 0, y = 0;
        CameraParam *mpCam = nullptr;

    };

    /**
     * XYZ 的投影误差
     * 0号点为PR，相机位姿
     * 1号点为XYZ
     */
    class EdgePRXYZ : public BaseBinaryEdge<2, Vector2d, VertexPR, VertexPointXYZ> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgePRXYZ(CameraParam *cam) : fx(cam->fx), fy(cam->fy), cx(cam->cx), cy(cam->cy) {}

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

        virtual void linearizeOplus() override;

        bool isDepthValid() {
            return depth > 0;
        }

    protected:
        double fx, fy, cx, cy;
        double depth = 0;
    };
    
    
    class EdgePRGPS : public BaseUnaryEdge<3, Vector3d, VertexPR> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      
      EdgePRGPS(): BaseUnaryEdge<3, Vector3d, VertexPR>()
      {
          //LOG(WARNING) << " EdgePRGPS instantialized" << endl;
      }
      
      bool read(std::istream &is) override{return true; }

      bool write(std::ostream &os) const override{ return true; }
      
      virtual void computeError()override;

      // virtual void linearizeOplus() override;
    };
    
    typedef Vector3d SO3LieGroup;
    class EdgeAttitude : public BaseUnaryEdge<3,SO3LieGroup,VertexPR> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeAttitude():BaseUnaryEdge<3,SO3LieGroup,VertexPR >()
      {
          //LOG(WARNING)<<"EdgeAttitude instantialized!"<<endl;
      }
      
      bool read(std::istream &is) override{return true;}
      
      bool write(std::ostream &os) const override { return true;}
      
      virtual void computeError() override;
      
      virtual void linearizeOplus() override;
    };
    
    /**
     * The pre-integration IMU motion constraint
     * Connect 6 vertex: PR0, V0, biasG0, bias A0 and PR1, V1
     * Vertex 0: PR0
     * Vertex 1: PR1
     * Vertex 2: V0
     * Vertex 3: V1
     * Vertex 4: biasG0
     * Vertex 5: biasA0
     * Error order: error_P, error_R, error_V
     *      different from PVR edge
     */
    //important. this edge merges imu preint and image pr estimation.
    //BaseMultiEdge<D(dim of E),E(measurement)>  multi measurement,pushed into a vector of edge.
    
    class EdgePRV : public BaseMultiEdge<9, IMUPreIntegration> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgePRV(const Vector3d &gw) : BaseMultiEdge<9, IMUPreIntegration>(), GravityVec(gw) {
            resize(6);
        }

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

        virtual void linearizeOplus() override;

    protected:
        // Gravity vector in 'world' frame
        Vector3d GravityVec;
    };

    
    
    /**
     * BiasG上的随机游走
     * 顺序：i帧的Bgi, j帧的Bgj
     * Error = Bgj - Bgi
     */
    typedef g2o::EdgePointXYZ EdgeBiasG;

    /**
     * BiasA基本和Bg相同
     * 顺序：i帧的Bai, j帧的Baj
     * Error = Baj - Bai
     */
    typedef g2o::EdgePointXYZ EdgeBiasA;

    /**
     * 投影方程，用于估计P+R
     * 需要指定Rcb, tcb, 相机模型和被投影点
     * 这里不优化Rcb和tcb，所以不从setting里取
     */
    class EdgeProjectPoseOnly : public BaseUnaryEdge<2, Vector2d, VertexPR> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectPoseOnly(const CameraParam *cam, const Vector3d &pw_)
                : BaseUnaryEdge<2, Vector2d, VertexPR>(), pw(pw_) {
            fx = cam->fx;
            fy = cam->fy;
            cx = cam->cx;
            cy = cam->cy;
        }

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

        virtual void linearizeOplus() override;

    private:
        double fx = 0, fy = 0, cx = 0, cy = 0;
        Vector3d pw;    // world 3d position

    };

    /**
     * @brief The EdgeGyrBias class
     * For gyroscope bias compuation in Visual-Inertial initialization
     */

    class EdgeGyrBias : public BaseUnaryEdge<3, Vector3d, VertexGyrBias> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeGyrBias() : BaseUnaryEdge<3, Vector3d, VertexGyrBias>() {}

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        Matrix3d dRbij;
        Matrix3d J_dR_bg;
        Matrix3d Rwbi;
        Matrix3d Rwbj;

        void computeError() override {
            const VertexGyrBias *v = static_cast<const VertexGyrBias *>(_vertices[0]);
            Vector3d bg = v->estimate();
            Matrix3d dRbg = SO3d::exp(J_dR_bg * bg).matrix();
            SO3d errR((dRbij * dRbg).transpose() * Rwbi.transpose() * Rwbj); // dRij^T * Riw * Rwj
            _error = errR.log();
        }

        virtual void linearizeOplus() override {
            SO3d errR(dRbij.transpose() * Rwbi.transpose() * Rwbj); // dRij^T * Riw * Rwj
            Matrix3d Jlinv = SO3d::JacobianLInv(errR.log());

            _jacobianOplusXi = -Jlinv * J_dR_bg;
        }
    };

}

#endif
