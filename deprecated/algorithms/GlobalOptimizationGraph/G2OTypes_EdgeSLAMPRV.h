#ifndef G2OTYPES_EDGESLAMPRV_H
#define G2OTYPES_EDGESLAMPRV_H
//Part1. IMUPreIntegration structure declaration.
class IMUPreIntegration {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        IMUPreIntegration() {
            ;//reset();
        }
/*
        // reset to initial state
        inline void reset() {
            // delta measurements, position/velocity/rotation(matrix)
            _delta_P.setZero();    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
            _delta_V.setZero();    // V_k+1 = V_k + R_k*a_k*dt
            _delta_R.setIdentity();    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

            // jacobian of delta measurements w.r.t bias of gyro/acc
            _J_P_Biasg.setZero();     // position / gyro
            _J_P_Biasa.setZero();     // position / acc
            _J_V_Biasg.setZero();     // velocity / gyro
            _J_V_Biasa.setZero();     // velocity / acc
            _J_R_Biasg.setZero();   // rotation / gyro

            // noise covariance propagation of delta measurements
            _cov_P_V_Phi.setZero();

            _delta_time = 0;
        }

        // incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
        void update(const Vector3d &omega, const Vector3d &acc, const double &dt) {
            float dt2 = dt * dt;
            Matrix3d dR = SO3d::exp(omega * dt).matrix();
            Matrix3d Jr = SO3d::JacobianR(omega * dt).matrix();
            Matrix3d I3x3 = Matrix3d::Identity();
            Matrix9d A = Matrix9d::Identity();
            Matrix3d acc_h = SO3d::hat(acc).matrix();

            // noise propagation, see Forster's paper APPENDIX A.
            A.block<3, 3>(6, 6) = dR.transpose();
            A.block<3, 3>(3, 6) = -_delta_R * acc_h * dt;
            A.block<3, 3>(0, 6) = -0.5 * _delta_R * acc_h * dt2;
            A.block<3, 3>(0, 3) = I3x3 * dt;

            Matrix<double, 9, 3> Bg;
            Bg.setZero();
            Bg.block<3, 3>(6, 0) = Jr * dt;

            Matrix<double, 9, 3> Ca;
            Ca.setZero();
            Ca.block<3, 3>(3, 0) = -_delta_R * dt;
            Ca.block<3, 3>(0, 0) = 0.5 * _delta_R * dt2;
            _cov_P_V_Phi = A * _cov_P_V_Phi * A.transpose()
                           + Bg * IMUData::mfGyrMeasCov * Bg.transpose()
                           + Ca * IMUData::mfAccMeasCov * Ca.transpose();

            // calculate the jacobians
            // NOTE see Forster's paper appendix B. Bias correction via first order updates
            _J_P_Biasa += _J_V_Biasa * dt - 0.5 * _delta_R * dt2;
            _J_P_Biasg += _J_V_Biasg * dt - 0.5 * _delta_R * acc_h * _J_R_Biasg * dt2;
            _J_V_Biasa += -_delta_R * dt;
            _J_V_Biasg += -_delta_R * acc_h * _J_R_Biasg * dt;
            _J_R_Biasg = dR.transpose() * _J_R_Biasg - Jr * dt;

            // delta measurements, position/velocity/rotation(matrix)
            // update P first, then V, then R. because P's update need V&R's previous state
            _delta_P += _delta_V * dt + 0.5 * _delta_R * acc * dt2;    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
            _delta_V += _delta_R * acc * dt;
            _delta_R = normalizeRotationM(
                    _delta_R * dR);  // normalize rotation, in case of numerical error accumulation

            // delta time
            _delta_time += dt;
        }
*/
        // 归一化矩阵
        inline Matrix3d normalizeRotationM(const Matrix3d &R) {
            Quaterniond qr(R);
            return normalizeRotationQ(qr).toRotationMatrix();
        }

        inline Quaterniond normalizeRotationQ(const Quaterniond &r) {
            Quaterniond _r(r);
            if (_r.w() < 0) {
                _r.coeffs() *= -1;
            }
            return _r.normalized();
        }

        // accessors
        // delta measurements, position/velocity/rotation(matrix)
        inline Eigen::Vector3d getDeltaP() const    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
        {
            return _delta_P;
        }
        inline void setDP(const Eigen::Vector3d dp)
        {
            this->_delta_P = dp;
        }

        inline Eigen::Vector3d getDeltaV() const    // V_k+1 = V_k + R_k*a_k*dt
        {
            return _delta_V;
        }
        inline void setDeltaV(const Eigen::Vector3d dv)
        {
            this->_delta_V = dv;
        }

        inline Eigen::Matrix3d getDeltaR() const   // R_k+1 = R_k*exp(w_k*dt).     NOTE: Rwc, Rwc'=Rwc*[w_body]x
        {
            return _delta_R;
        }
        inline void setDR(const Eigen::Matrix3d dr)
        {
            this->_delta_R = dr;
        }

        // jacobian of delta measurements w.r.t bias of gyro/acc
        inline Eigen::Matrix3d getJPBiasg() const     // position / gyro
        {
            return _J_P_Biasg;
        }

        inline Eigen::Matrix3d getJPBiasa() const     // position / acc
        {
            return _J_P_Biasa;
        }

        inline Eigen::Matrix3d getJVBiasg() const     // velocity / gyro
        {
            return _J_V_Biasg;
        }

        inline Eigen::Matrix3d getJVBiasa() const     // velocity / acc
        {
            return _J_V_Biasa;
        }

        inline Eigen::Matrix3d getJRBiasg() const     // rotation / gyro
        {
            return _J_R_Biasg;
        }

        // noise covariance propagation of delta measurements
        // note: the order is rotation-velocity-position here

        inline Matrix9d getCovPVPhi() const {
            return _cov_P_V_Phi;
        }

        inline double getDeltaTime() const {
            return _delta_time;
        }
/*
        
         * NOTE:
         * don't add pointer as member variable.
         * operator = is used in g2o
        */

        // delta measurements, position/velocity/rotation(matrix)
        Eigen::Vector3d _delta_P;    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
        Eigen::Vector3d _delta_V;    // V_k+1 = V_k + R_k*a_k*dt
        Eigen::Matrix3d _delta_R;    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

        // jacobian of delta measurements w.r.t bias of gyro/acc
        Eigen::Matrix3d _J_P_Biasg;     // position / gyro
        Eigen::Matrix3d _J_P_Biasa;     // position / acc
        Eigen::Matrix3d _J_V_Biasg;     // velocity / gyro
        Eigen::Matrix3d _J_V_Biasa;     // velocity / acc
        Eigen::Matrix3d _J_R_Biasg;   // rotation / gyro

        // noise covariance propagation of delta measurements
        Matrix9d _cov_P_V_Phi;

        double _delta_time;
};




//Part2.SLAM_PRV g2o edge defination.

 class EdgeSLAMPRV : public BaseMultiEdge<9, IMUPreIntegration> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeSLAMPRV(const Vector3d &gw) : BaseMultiEdge<9, IMUPreIntegration>(), GravityVec(gw) {
            resize(4);
        }/*
        EdgeSLAMPRV(): BaseMultiEdge<9, IMUPreIntegration>()
        {
            resize(6);
        }*/

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

        virtual void linearizeOplus() override;

    protected:
        // Gravity vector in 'world' frame
        Vector3d GravityVec;
    };
    void EdgeSLAMPRV::computeError() {
        
        const VertexPR *vPRi = static_cast<const VertexPR *>(_vertices[0]);
        const VertexPR *vPRj = static_cast<const VertexPR *>(_vertices[1]);
        const VertexSpeed *vVi = static_cast<const VertexSpeed *>(_vertices[2]);
        const VertexSpeed *vVj = static_cast<const VertexSpeed *>(_vertices[3]);
        //const VertexGyrBias *vBiasGi = static_cast<const VertexGyrBias *>(_vertices[4]);
        //const VertexAcceBias *vBiasAi = static_cast<const VertexAcceBias *>(_vertices[5]);

        // terms need to computer error in vertex i, except for bias error
        const Vector3d Pi = vPRi->t();
        const Matrix3d Ri = vPRi->R();

        const Vector3d Vi = vVi->estimate();

        // Bias from the bias vertex
        const Vector3d dBgi = Vector3d(0,0,0);//vBiasGi->estimate();
        const Vector3d dBai = Vector3d(0,0,0);//vBiasAi->estimate();

        // terms need to computer error in vertex j, except for bias error
        const Vector3d Pj = vPRj->t();
        const Matrix3d Rj = vPRj->R();

        const Vector3d Vj = vVj->estimate();

        // IMU Preintegration measurement
        const IMUPreIntegration &M = _measurement;

        const double dTij = M.getDeltaTime();   // Delta Time
        const double dT2 = dTij * dTij;
        const Vector3d dPij = M.getDeltaP().cast<double>();    // Delta Position pre-integration measurement
        const Vector3d dVij = M.getDeltaV().cast<double>();    // Delta Velocity pre-integration measurement
        const Matrix3d dRij = M.getDeltaR().cast<double>();    // Delta Rotation pre-integration measurement

        // tmp variable, transpose of Ri
        const Matrix3d RiT = Ri.inverse();
        // residual error of Delta Position measurement
        const Vector3d rPij = RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2);  
                              //- (dPij + M.getJPBiasg() * dBgi +
                              //   M.getJPBiasa() * dBai);  
            // this line includes correction term of bias change.

        // residual error of Delta Velocity measurement
        const Vector3d rVij = RiT * (Vj - Vi - GravityVec * dTij);
                              //- (dVij + M.getJVBiasg() * dBgi +
                              //   M.getJVBiasa() * dBai);
            //this line includes correction term of bias change

        // residual error of Delta Rotation measurement
        //const Matrix3d dR_dbg = SO3d::exp(M.getJRBiasg() * dBgi).matrix();
        const Matrix3d dR_dbg = Matrix3d::Identity(3,3);
        const Matrix3d rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
        const Vector3d rPhiij = SO3d::log(rRij);

        // 9-Dim error vector order:
        // position-velocity-rotation
        // rPij - rPhiij - rVij
        _error.segment<3>(0) = rPij;       // position error
        _error.segment<3>(3) = rPhiij;     // rotation phi error
        _error.segment<3>(6) = rVij;       // velocity error
    }

    void EdgeSLAMPRV::linearizeOplus() {

        const VertexPR *vPRi = static_cast<const VertexPR *>(_vertices[0]);
        const VertexPR *vPRj = static_cast<const VertexPR *>(_vertices[1]);
        const VertexSpeed *vVi = static_cast<const VertexSpeed *>(_vertices[2]);
        const VertexSpeed *vVj = static_cast<const VertexSpeed *>(_vertices[3]);
        //const VertexGyrBias *vBiasGi = static_cast<const VertexGyrBias *>(_vertices[4]);
        //const VertexAcceBias *vBiasAi = static_cast<const VertexAcceBias *>(_vertices[5]);

        // terms need to computer error in vertex i, except for bias error
        const Vector3d Pi = vPRi->t();
        const Matrix3d Ri = vPRi->R();

        const Vector3d Vi = vVi->estimate();

        // Bias from the bias vertex
        const Vector3d dBgi = Vector3d(0,0,0);//vBiasGi->estimate();
        const Vector3d dBai = Vector3d(0,0,0);//vBiasAi->estimate();

        // terms need to computer error in vertex j, except for bias error
        const Vector3d Pj = vPRj->t();
        const Matrix3d Rj = vPRj->R();

        const Vector3d Vj = vVj->estimate();

        // IMU Preintegration measurement
        const IMUPreIntegration &M = _measurement;
        const double dTij = M.getDeltaTime();   // Delta Time
        const double dT2 = dTij * dTij;

        // some temp variable
        Matrix3d O3x3 = Matrix3d::Zero();       // 0_3x3
        Matrix3d RiT = Ri.transpose();          // Ri^T
        Matrix3d RjT = Rj.transpose();          // Rj^T
        Vector3d rPhiij = _error.segment<3>(3); // residual of rotation, rPhiij
        Matrix3d JrInv_rPhi = SO3d::JacobianRInv(rPhiij);    // inverse right jacobian of so3 term #rPhiij#
        //Matrix3d J_rPhi_dbg = M.getJRBiasg();              // jacobian of preintegrated rotation-angle to gyro bias i

        // this is really messy
        // 1.
        // increment is the same as Forster 15'RSS
        // pi = pi + dpi,    pj = pj + dpj
        // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
        // vi = vi + dvi,       vj = vj + dvj
        //      Note: the optimized bias term is the 'delta bias'
        // dBgi = dBgi + dbgi_update,    dBgj = dBgj + dbgj_update
        // dBai = dBai + dbai_update,    dBaj = dBaj + dbaj_update

        // 2.
        // 9-Dim error vector order in PVR:
        // position-velocity-rotation
        // rPij - rPhiij - rVij
        //      Jacobian row order:
        // J_rPij_xxx
        // J_rPhiij_xxx
        // J_rVij_xxx

        // 3.
        // order in 'update_' in PR
        // Vertex_i : dPi, dPhi_i
        // Vertex_j : dPj, dPhi_j
        // 6-Dim error vector order in Bias:
        // dBiasg_i - dBiasa_i

        // Jacobians:
        // dP/dPR0, dP/dPR1, dP/dV0, dP/dV1, dP/dBiasG, dP/dBiasG
        // dR/dPR0, dR/dPR1, dR/dV0, dR/dV1, dR/dBiasG, dR/dBiasG
        // dV/dPR0, dV/dPR1, dV/dV0, dV/dV1, dV/dBiasG, dV/dBiasG

        // 4. PR0 & V0    
        // For Vertex_PR_i, J [dP;dR;dV] / [dP0 dR0]
        Matrix<double, 9, 6> JPRi;
        JPRi.setZero();
        // J_rPij_xxx_i for Vertex_PR_i
        JPRi.block<3, 3>(0, 0) = -RiT;      //J_rP_dpi
        JPRi.block<3, 3>(0, 3) = SO3d::hat(
                RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2));    //J_rP_dPhi_i
        // J_rPhiij_xxx_i for Vertex_PR_i                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        Matrix3d ExprPhiijTrans = SO3d::exp(rPhiij).inverse().matrix();
        //Matrix3d JrBiasGCorr = SO3d::JacobianR(J_rPhi_dbg * dBgi);
        JPRi.block<3, 3>(3, 0) = O3x3;    //dpi
        JPRi.block<3, 3>(3, 3) = -JrInv_rPhi * RjT * Ri;    //dphi_i
        // J_rVij_xxx_i for Vertex_PVR_i
        JPRi.block<3, 3>(6, 0) = O3x3;    //dpi
        JPRi.block<3, 3>(6, 3) = SO3d::hat(RiT * (Vj - Vi - GravityVec * dTij));    //dphi_i

        // For Vertex_V_i, J [dP;dR;dV] / dV0
        Matrix<double, 9, 3> JVi;
        JVi.setZero();
        JVi.block<3, 3>(0, 0) = -RiT * dTij;  //J_rP_dvi
        JVi.block<3, 3>(3, 0) = O3x3;    //rR_dvi
        JVi.block<3, 3>(6, 0) = -RiT;    //rV_dvi

        // 5. PR1 & V1
        // For Vertex_PR_j, J [dP;dR;dV] / [dP1 dR1]
        Matrix<double, 9, 6> JPRj;
        JPRj.setZero();
        // J_rPij_xxx_j for Vertex_PR_j
        JPRj.block<3, 3>(0, 0) = RiT;  //rP_dpj
        JPRj.block<3, 3>(0, 3) = O3x3;    //rP_dphi_j
        // J_rPhiij_xxx_j for Vertex_PR_j
        JPRj.block<3, 3>(3, 0) = O3x3;    //rR_dpj
        JPRj.block<3, 3>(3, 3) = JrInv_rPhi;    //rR_dphi_j
        // J_rVij_xxx_j for Vertex_PR_j
        JPRj.block<3, 3>(6, 0) = O3x3;    //rV_dpj
        JPRj.block<3, 3>(6, 3) = O3x3;    //rV_dphi_j

        // For Vertex_V_i, J [dP;dR;dV] / dV1
        Matrix<double, 9, 3> JVj;
        JVj.setZero();
        JVj.block<3, 3>(0, 0) = O3x3;    //rP_dvj
        JVj.block<3, 3>(3, 0) = O3x3;    //rR_dvj
        JVj.block<3, 3>(6, 0) = RiT;    //rV_dvj

        // 6.
        // For Vertex_Bias_i
        //Matrix<double, 9, 3> JBiasG;
        //Matrix<double, 9, 3> JBiasA;
        //JBiasG.setZero();
        //JBiasA.setZero();

        // bias
        //JBiasG.block<3, 3>(0, 0) = -M.getJPBiasg();     //J_rP_dbgi
        //JBiasG.block<3, 3>(3, 0) = -JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    //dbg_i
        //JBiasG.block<3, 3>(6, 0) = -M.getJVBiasg();    //dbg_i

        //JBiasA.block<3, 3>(0, 0) = -M.getJPBiasa();     //J_rP_dbai
        //JBiasA.block<3, 3>(3, 0) = O3x3;    //dba_i
        //JBiasA.block<3, 3>(6, 0) = -M.getJVBiasa();    //dba_i

        // set all jacobians
        _jacobianOplus[0] = JPRi;
        _jacobianOplus[1] = JPRj;
        _jacobianOplus[2] = JVi;
        _jacobianOplus[3] = JVj;
        //_jacobianOplus[4] = JBiasG;
        //_jacobianOplus[5] = JBiasA;
    }
#endif
