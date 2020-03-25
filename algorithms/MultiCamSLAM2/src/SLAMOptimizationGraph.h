#ifndef SLAM_OPTIMIZATION_GRAPH_H
#define SLAM_OPTIMIZATION_GRAPH_H
#include <glog/logging.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>


#include <gtsam/inference/Ordering.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/PCGSolver.h>

#include <gtsam/nonlinear/DoglegOptimizer.h>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

#include <gtsam/slam/dataset.h> 
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/SmartFactorParams.h> //like chi2 outlier select.
#include <gtsam/slam/GeneralSFMFactor.h>

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <gtsam_unstable/nonlinear/ConcurrentBatchFilter.h>
#include <gtsam_unstable/nonlinear/ConcurrentBatchSmoother.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>


#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>
//#include <gtsam/geometry/Rot3.h>
//#include <gtsam/geometry/Point3.h>
//#include <gtsam/geometry/Pose3.h>


#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d.hpp"


#include "Frame.h"
#include "FrameManager.h"

#include <thread>
#include <opencv2/core/eigen.hpp>
#include "IMU_Preint_GTSAM.h"
#include <iostream>
#include "Visualization.h"
#include <mutex>
#include "utils/mcs_utils.h"
using namespace gtsam;
using namespace std;
using namespace cv;



const int DEBUG_USE_LM = 1;
const int ENABLE_INERTIAL_MEASUREMENT = 0;
const int DEBUG_VISUALIZE = 0;
namespace mcs
{

//    Eigen::Matrix4d get_pose3_matrix(Pose3 pose3)
//    {
//        static const auto A14 = Eigen::RowVector4d(0,0,0,1);
//        Eigen::Matrix4d mat;
//        mat << pose3.rotation().matrix(), pose3.translation().vector(), A14;
//        return mat;
//    }



    struct landmark_properties;


    struct landmark_properties
    {
        int landmark_reference_time = 0;
        weak_ptr<Frame> pCreatedByFrame;
        //weak_ptr<Frame> pLastObservedByFrame;
        boost::shared_ptr<SmartStereoProjectionPoseFactor> pRelativeStereoSmartFactor;//目前还没实现处理无穷远.
        std::vector<weak_ptr <Frame> > observed_by_frames;

    };
    //void doFrontEndTrackingForOrdinaryFrames
    //跟踪方法挪到FrameWiseGeometry里面去.
    class SLAMOptimizationGraph
    {
    private:
        ISAM2Params parameters;
        //parameters.relinearizeThreshold = 0.1;
        ISAM2 isam;//(parameters);

        NonlinearFactorGraph graph;
        Values initialEstimate;
        Values currentEstimate;

        double fixed_lag_val_sec = 10.0;
        // Create a Concurrent Filter and Smoother
        ConcurrentBatchFilter concurrentFilter;
        ConcurrentBatchSmoother concurrentSmoother;
        // And a fixed lag smoother with a short lag
        BatchFixedLagSmoother fixedlagSmoother;
        //BatchFixedLagSmoother fixedlagSmoother(lag);

        //下面这两个是用来处理新的节点用的.每当发生marginalize时更新这两个东西.
        NonlinearFactorGraph newFactors;
        Values newValues;



        cv::FileStorage* pfSettings;
        vector<StereoCamConfig> stereo_config;
        vector<CamInfo> rgbd_config;
        int landmark_id = 0;
        vector<landmark_properties> vlandmark_properties;//路标点的管理.
        std::mutex vlandmark_properties_mutex;
        int frame_id = 0;
        vector<boost::shared_ptr<Cal3_S2> > v_pcams_gtsam_config_stereo_left;
        vector<boost::shared_ptr<Cal3_S2Stereo> > v_pcams_gtsam_config_stereo;
        vector<Cal3_S2> v_cams_gtsam_config_depth;
        //IMUHelper slam_imu_helper;


//这里参考Kimera删除节点的实现.
//        // State.
//        gtsam::Values state_;  //!< current state of the system.
//        // GTSAM:
//        std::shared_ptr<Smoother> smoother_;
//        // Values
//        gtsam::Values new_values_;  //!< new states to be added
//        // Factors.
//        gtsam::NonlinearFactorGraph
//            new_imu_prior_and_other_factors_;  //!< new factors to be added
//        LandmarkIdSmartFactorMap
//            new_smart_factors_;  //!< landmarkId -> {SmartFactorPtr}
//        SmartFactorMap
//            old_smart_factors_;  //!< landmarkId -> {SmartFactorPtr, SlotIndex}
//        // if SlotIndex is -1, means that the factor has not been inserted yet in the
//        // graph


    public:
        SLAMOptimizationGraph(cv::FileStorage& fSettings)
        {//填充初始化函数.
            fixedlagSmoother = BatchFixedLagSmoother(fixed_lag_val_sec);
            this->pfSettings = &fSettings;//现用现取.
            parameters.relinearizeSkip = 0;
            //parameters.relinearizeThreshold = 0.01;
            //parameters.relinearizeSkip = 1;
            parameters.cacheLinearizedFactors = false;
            parameters.enableDetailedResults = true;
            parameters.setEnableRelinearization(false);

            this->isam = ISAM2(parameters);
            //this->slam_imu_helper = IMUHelper(true);
        }
        int getFrameID()
        {
            return this->frame_id;
        }
        void initCamsStereo(vector<StereoCamConfig>& cams)
        {
            this->stereo_config = cams;
            for(auto c:cams)
            {
                float fx,fy,cx,cy;
                c.getLCamMatFxFyCxCy(fx,fy,cx,cy);
                auto k_l = boost::shared_ptr<Cal3_S2>(new Cal3_S2(fx,fy,0,cx,cy));//这种形式的相机标定是可以优化的.如果需要,可以后续在重投影过程中附加进行一个内参优化.
                //noiseModel::Diagonal::shared_ptr calNoise = noiseModel::Diagonal::Sigmas((Vector(5) << 500, 500, 0.1, 100, 100).finished());
                //就像这样.具体参考examples/SelfCalibrationExample.cpp
                v_pcams_gtsam_config_stereo_left.push_back(k_l);//先建立主视觉的K.

                //double fx, double fy, double s, double u0, double v0, double b//这要求双目rectify之后共享一组fx,fy cx,cy且必须旋转也对齐.需要预处理.
                double b = c.getBaseLine();
                LOG(ERROR)<<"DEBUG ONLY!!!!"<<endl;
                b = 0.12;
                cout<<"setting baseline:"<<b<<endl;
                auto k_stereo = boost::shared_ptr<Cal3_S2Stereo>(new Cal3_S2Stereo(fx,fy,0,cx,cy,b));
                this->v_pcams_gtsam_config_stereo.push_back(k_stereo);
            }
        }
        void initCamsDepth(vector<CamInfo>& cams)
        {
            for(auto c:cams)
            {
                float fx,fy,cx,cy;
                c.getCamMatFxFyCxCy(fx,fy,cx,cy);
                Cal3_S2 k_(fx,fy,0,cx,cy);
                v_cams_gtsam_config_depth.push_back(k_);
            }
        }
        shared_ptr<Frame> getLastKF();
        void addSingleCamObservationFactor(Matrix3d camR,Vector3d camt,vector<p2dT> observation2d,vector<shared_ptr<MapPoint> > coresponding_map_pts);
        void generateMapPointsCorespondingToStereoKFObservation(Matrix3d camR,Vector3d camt,
                                                                vector<p3dT> observation3d,vector<p2dT> observation2d,
                                                                vector<shared_ptr<MapPoint> > &output_map_points);//根据关键帧观测的p2d,p3d生成对应的Map.

        const static int METHOD_SIMPLE_MONOCULAR_REPROJECTION_ERROR = 0;
        const static int METHOD_SIMPLE_STEREO_REPROJECTION_ERROR = 1;
        const static int METHOD_SMARTFACTOR_MONOCULAR_REPROJECTION_ERROR = 2;
        const static int METHOD_SMARTFACTOR_STEREO_REPROJECTION_ERROR = 3;//可以做位操作.




        void addOrdinaryStereoFrame_MONO_PART_ToBackendAndOptimize(shared_ptr<Frame> pFrame,shared_ptr<Frame> pKeyFrameReference,int& tracked_pts)//加入mono追踪的部分.
        {
            //要支持depth和stereo两种.
            //Step<1>对每个相机,查询没有p3d的p2d.
            //Step<2>对每个p2d,尝试追踪.保留成功的点.
            //Step<3>对追踪成功的点加入smart factor.smart factor自己尝试三角化.

        }
        void addOrdinaryDepthFrameToBackendAndOptimize(shared_ptr<Frame> pFrame,shared_ptr<Frame> pKeyFrameReference,int& pts_tracked_output);
        void addOrdinaryStereoFrameToBackendAndOptimize(shared_ptr<Frame> pFrame,shared_ptr<Frame> pKeyFrameReference,int& optimization_pts_tracked,int method = METHOD_SIMPLE_STEREO_REPROJECTION_ERROR)
        {
            if(pKeyFrameReference == nullptr)
            {
                cout<<"in addOrdinaryStereoFrameToBackendAndOptimize():pKeyRef == nullptr!!!"<<endl;
            }
            else
            {
                LOG(INFO)<<"tracking frame:"<<frame_id<<",isKeyFrame?"<<pFrame->isKeyFrame<<"."<<endl;
            }

            optimization_pts_tracked = 0;
            ScopeTimer t1("addOrdinaryStereoFrameToBackend() timer");
            cout<<"in addOrdinaryStereoFrameToBackend() stage2"<<endl;

            //auto pts_vv = pFrame->p2d_vv;
            //pts_vv = OptFlowForFrameWiseTracking(*pframe2);//track frame2 获取对应位置的点...

            if(pFrame->frame_id != -1)
            {
                LOG(ERROR)<<"ERROR: in addOrdinaryStereoFrameToBackendAndOptimize():frame id exists!"<<endl;
                return;
            }
            pFrame->frame_id = frame_id;
            cout<<"In addOrdinaryStereoFrameToBackendAndOptimize(): step<1> create pose for each camera.frame_id:"<<frame_id<<endl;
            int cam_count = pFrame->get_cam_num();
            for(int i = 0;i < cam_count;i++)//对每个相机,先创建它的Pose3优化节点.
            {
                if(frame_id ==0 )//第一个frame.
                {
                    cout<<"initializing optimization graph for the first frame,insert X0-Xn."<<endl;
                    Eigen::Matrix4d cam_to_body_rt_mat;
                    cv2eigen(pFrame->cam_info_stereo_vec[i].getRTMat(),cam_to_body_rt_mat);

                    //graph.add(Symbol('X',frame_id*cam_count + i),Pose3(cam_to_body_rt_mat));
                    if(i == 0)//就算是这种情况,也只对第一组摄像头定绝对位置.
                    {//对初始所有摄像头定位置.
//                        gtsam::noiseModel::Diagonal::shared_ptr priorFrame_Cam0_noisemodel = gtsam::noiseModel::Diagonal::Variances (
//                                    ( gtsam::Vector ( 6 ) <<0.1, 0.1, 0.1, 1e-6, 1e-6, 1e-6 ).finished()
//                                );
                        gtsam::noiseModel::Diagonal::shared_ptr priorFrame_Cam0_noisemodel = gtsam::noiseModel::Diagonal::Variances (
                                    ( gtsam::Vector ( 6 ) <<1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 ).finished()
                                );
                        //添加初始绝对位置约束.Rotation可变(重力对齐),translation不可变(绝对坐标系创建的点).
                        //graph.emplace_shared<PriorFactor<Pose3> > (Symbol('X',0), Pose3(Rot3(1,0,0,0),Point3(0,0,0)),priorFrame_Cam0_noisemodel);

                        //Pose3 first_pose = initialEstimate.at<Pose3>(Symbol('X',i));
                        Pose3 first_pose(cam_to_body_rt_mat);
                        //constrain the first pose such that it cannot change from its original value during optimization
                        // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
                        // QR is much slower than Cholesky, but numerically more stable
                        //graph.emplace_shared<NonlinearEquality<Pose3> >(Symbol('X',i),first_pose);
                        LOG(INFO)<<"NonlinearEquality Constrained symbol X"<<i<<" at first pose:"<<first_pose.matrix()<<endl;
                        initialEstimate.insert(Symbol('X',i),first_pose);
                        graph.add(PriorFactor<Pose3>  (Symbol('X',i), first_pose,priorFrame_Cam0_noisemodel));
                    }
                    else
                    {
                        Pose3 first_pose(cam_to_body_rt_mat);
                        initialEstimate.insert(Symbol('X',i),first_pose);
                    }

                    LOG(INFO)<<"Cam "<<i<<" initiated at first pose:"<<initialEstimate.at<Pose3>(Symbol('X',i)).matrix()<<endl;
                    cout<<"Cam "<<i<<" initiated at first pose:"<<initialEstimate.at<Pose3>(Symbol('X',i)).matrix()<<endl;
                }
                else
                {
                    if(frame_id >= 1)//第二个frame没法看第一个的值.
                    {
                        cout<<"creating cam node X:frame id:"<<frame_id<<",cam_count:"<<cam_count<<",current_i:"<<i<<endl;
                        //currentEstimate = isam.calculateEstimate();
                        //currentEstimate.print();

                        //const Pose3* p_last = &(initialEstimate.at(Symbol('X',(frame_id - 1)*cam_count + i)).cast<Pose3>());
                        //cout<<"last pose:"<<p_last->translation()<<endl;

                        //initialEstimate.insert(Symbol('X',frame_id*cam_count + i),Pose3(*p_last)); // 用上一帧的对应相机优化结果作为初始估计.//有了ransac估计就不用这个了.
                    }
                    else
                    {//test.
                        const Pose3* p_last = &(currentEstimate.at(Symbol('X',(frame_id - 1)*cam_count + i)).cast<Pose3>());
                        initialEstimate.insert(Symbol('X',frame_id*cam_count + i),Pose3(*p_last)); // 用上一帧的对应相机优化结果作为初始估计.
                    }
                }
                //加入相机位置约束.
                if(i != 0)
                {
                    noiseModel::Diagonal::shared_ptr noise_model_between_cams = gtsam::noiseModel::Diagonal::Variances (
                                ( gtsam::Vector ( 6 ) <<0.00001, 0.00001, 0.00001, 0.0001, 0.0001, 0.0001 ).finished()); //1mm,0.1度.//放松一些
                    Eigen::Matrix4d cam_to_cam0_rt_mat;
                    cv2eigen(pFrame->cam_info_stereo_vec[0].getRTMat().inv()* pFrame->cam_info_stereo_vec[i].getRTMat(),cam_to_cam0_rt_mat);
                    graph.emplace_shared<BetweenFactor<Pose3> >(Symbol('X',frame_id*cam_count),Symbol('X',frame_id*cam_count + i),Pose3(cam_to_cam0_rt_mat),
                                                                  noise_model_between_cams);//用非常紧的约束来限制相机间位置关系.
                    LOG(INFO)<<"added constrain betweed cam "<<i<<" and cam 0,mat:"<<cam_to_cam0_rt_mat<<endl;
                    cout<<"added constrain betweed cam "<<i<<" and cam 0,mat:"<<cam_to_cam0_rt_mat<<endl;
                }
            }
            if(frame_id == 0)//&& method==METHOD_SMARTFACTOR_STEREO_REPROJECTION_ERROR)
            {
                cout<<"initialized the 1st frame successfully!"<<endl;
                frame_id++;//记录这是第几个frame.
                cout<<"Before calling isam2 update:"<<endl;
                initialEstimate.print("initial estimate:(not optimized)");
                this->currentEstimate = this-> initialEstimate;


                /*
                if(DEBUG_USE_LM == 1)
                {
                    cout<<"Debug:For frame 1, call lm optimizer!"<<endl;
                    LevenbergMarquardtParams params;
                    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
                    params.verbosity = NonlinearOptimizerParams::ERROR;

                    cout << "Optimizing" << endl;
                    //create Levenberg-Marquardt optimizer to optimize the factor graph
                    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
                    Values result = optimizer.optimize();
                    cout<<"optimized result:"<<endl;
                    result.print();
                }*/

                //isam.update(graph, initialEstimate);//优化.
                //this->currentEstimate = isam.calculateEstimate();
                //currentEstimate.print("before resize,values:");
                //graph.resize(0);
                //initialEstimate.clear();//这里不能做任何操作.否则引起indetermined linear system exception.
                return;
            }

            if(ENABLE_INERTIAL_MEASUREMENT)
            {
                //TODO:加入imu preint计算的关联关系.
            }
            auto imgs_prev = pKeyFrameReference->getMainImages();
            auto imgs_next = pFrame->getMainImages();
            auto imgs_next_right = pFrame->getSecondaryImages();
            cout<<"in addOrdinaryStereoFrameToBackend() stage3"<<endl;

            auto p3ds_vv = pKeyFrameReference->p3d_vv;
            //多线程实现

            vector<std::thread> threads_frontend;
            vector<vector<Point2f> > left_p2f_vv,right_p2f_vv,output_to_track_kf_p2d_vv;
            vector<map<int,int> > p2f_to_p3d_maps;
            vector<char> track_success_vcams;
            vector<int> success_track_pts_count_vec;
            track_success_vcams.resize(cam_count);
            left_p2f_vv.resize(cam_count);right_p2f_vv.resize(cam_count);output_to_track_kf_p2d_vv.resize(cam_count);
            p2f_to_p3d_maps.resize(cam_count);track_success_vcams.resize(cam_count);success_track_pts_count_vec.resize(cam_count);
            for(int i = 0;i<cam_count;i++)
            {
                int cam_index = i;
                auto& lp2fv = left_p2f_vv.at(cam_index);auto& rp2fv = right_p2f_vv.at(cam_index);
                auto& map__ = p2f_to_p3d_maps.at(cam_index);
                auto& tracked_p2d = output_to_track_kf_p2d_vv.at(cam_index);

                //threads_frontend.push_back(std::thread(
                //                               doFrontEndTrackingForOrdinaryFrames,std::ref(pFrame),std::ref(pKeyFrameReference),cam_index,
                //                               std::ref(lp2fv),std::ref(rp2fv),
                //                               std::ref(map__),std::ref(tracked_p2d),&(track_success_vcams[cam_index]),0
                //                               ));

                cout<<"before calling doFrontEndTrackingForOrd:  pFrame:"<<pFrame<<",pFrameRef:"<<pKeyFrameReference<<",cam_index:"<<cam_index<<",lp2fv.size():"<<lp2fv.size()<<endl;
                try{
                    doTrackLastKF
                    //doFrontEndTrackingForOrdinaryFrames
			(pFrame,pKeyFrameReference,cam_index,
                        lp2fv,rp2fv,
                        map__,tracked_p2d,&(track_success_vcams[cam_index]),&(success_track_pts_count_vec[cam_index]),0);
                }
                catch (Exception e)
                {
                    
                    LOG(ERROR)<<"Caught error in doFrontEndTrackingForOrdinaryFrames()"<<e.what()<<endl;
                    cout<<"Caught error in doFrontEndTrackingForOrdinaryFrames()"<<e.what()<<endl;
                    throw e;
                }

                //doFrontEndTrackingForOrdinaryFrames(std::ref(pFrame),std::ref(pKeyFrameReference),cam_index,
                //                                                                std::ref(lp2fv),std::ref(rp2fv),
                //                                                                std::ref(map__),std::ref(tracked_p2d),&(track_success_vcams[cam_index]),0);

            }
            for(auto& thread_:threads_frontend)
            {
                thread_.join();
            }
            //分成两个部分.上一部分是前端操作.下一部分是后端操作.
            auto mono_reprojection_noise_model = noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v
            auto stereo_reprojection_noise_model = noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

            auto gaussian__ = noiseModel::Isotropic::Sigma(3, 1.0);
            //auto robust_kernel = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(15), gaussian__); //robust
            //auto robust_kernel = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian__); //robust

            gtsam::SharedNoiseModel robust_kernel = gtsam::noiseModel::Robust::Create(
                                                        gtsam::noiseModel::mEstimator::Huber::Create(
                                                        1.345*20,
                                                        gtsam::noiseModel::mEstimator::Huber::Scalar),  // Default is
                                                                            // Block
                                                        gaussian__);
            //auto robust_kernel = gaussian__;//DEBUG ONLY!


            int cam_pair_invalid_tracking_count = 0;
            for(int i = 0;i < cam_count;i++)
            {
                //prepare for pnp ransac. //OpenCV pnp ransac accepts vector of point3d and point2d only.
                std::vector<cv::Point3d> p3d_pts_matched;
                std::vector<cv::Point2d> p2d_pts_matched;

                map<int,int> tracked_kps_to_original_kps_map_output;//关键帧的p3d到追踪到的p2d的关系
                vector<p3dT> vp3d_pts = p3ds_vv.at(i);
                auto& vp2d_pts = output_to_track_kf_p2d_vv.at(i);
                for(auto iter = p2f_to_p3d_maps.at(i).begin(); iter != p2f_to_p3d_maps.at(i).end(); ++iter)
                {//交换键值关系
                    auto key = iter->first;
                    auto val = iter->second;
                    tracked_kps_to_original_kps_map_output[val] = key;
                }
                if(success_track_pts_count_vec.at(i) == 0) // 在这组摄像机未能成功追踪任何点
                {
                    LOG(WARNING)<<"No stereo point tracked at cam_index:"<<i<<",frame_id:"<<frame_id<<"!"<<endl;
                    cam_pair_invalid_tracking_count++;
                    if(cam_pair_invalid_tracking_count == cam_count)
                    {
                        LOG(ERROR)<<"[ERROR] All cams track failure!"<<endl;
                        cout<<"[ERROR] All cams track failure!"<<endl;
                        throw "[ERROR] All cams track failure!";
                    }
                    const Pose3* p_last = &(currentEstimate.at(Symbol('X',(frame_id - 1)*cam_count + i)).cast<Pose3>());
                    cout<<"last pose:"<<p_last->translation()<<endl;
                    initialEstimate.insert(Symbol('X',frame_id*cam_count + i),Pose3(*p_last)); // 用上一帧的对应相机优化结果作为初始估计.//有了ransac估计就不用这个了,仅在跟踪失败时使用.
                    continue;
                }

                for(int index_p3d = 0;index_p3d<vp3d_pts.size();index_p3d++)
                {
                    int p2d_index = -1;
                    if(tracked_kps_to_original_kps_map_output.count(index_p3d))
                    {
                        p2d_index = tracked_kps_to_original_kps_map_output[index_p3d];//原始关键帧要追踪的2d点的index(同时也是KeyFrame p3d的真实index)失败是-1.
                    }
                    //int p2d_index = tracked_kps_to_original_kps_map_output[index_p3d];
                    //注意:下面这段代码不可重入.
                    if(p2d_index>=0)//a match has been found.
                    {
                        //TODO:
                        //1.检查这个map_point是否有这个对应的landmark;没有就先创建点.
                        this->vlandmark_properties_mutex.lock();
                        int map_point_relavent_landmark_id = pKeyFrameReference->get_p3dindex_to_landmark_index(p2d_index,i);//获取对应Landmark的id.失败返回-1.
                        if(map_point_relavent_landmark_id == -1) // 第一次被另一个帧观测,添加到vLandmark记录中.
                        {

                            int reference_kf_id = pKeyFrameReference->frame_id;
                            const Pose3* p_kf_pose3 = &(currentEstimate.at(Symbol('X',reference_kf_id*cam_count + i)).cast<Pose3>());
                            Matrix3d kf_rot = p_kf_pose3->rotation().matrix();
                            Vector3d kf_trans = p_kf_pose3->translation().vector();
                            Vector3d p3d_relative_to_cam;
                            Vector2d reprojected_p2d;
                            //cv2eigen(vp3d_pts.at(p2d_index),p3d_relative_to_cam);
                            //cv2eigen(vp2d_pts.at(p2d_index),reprojected_p2d);
                            p3d_relative_to_cam = Vector3d(vp3d_pts.at(p2d_index).x,vp3d_pts.at(p2d_index).y,vp3d_pts.at(p2d_index).z);
                            cout<<"DEBUG:backend vp2d_pts.size():"<<vp2d_pts.size()<<endl;
                            reprojected_p2d = Vector2d(vp2d_pts.at(p2d_index).x,vp2d_pts.at(p2d_index).y);

                            Vector3d point_pos_initial_guess = kf_rot*p3d_relative_to_cam + kf_trans; //TODO:通过对应关键帧计算其初始位置估计,所有都变换到一个坐标系下面.
                            //pKeyFrameReference->set_p3d_landmark_index(p2d_index,landmark_id,i);  //绑定对应关系.
                            pKeyFrameReference->set_p3d_landmark_index(p2d_index,vlandmark_properties.size(),i);
                            map_point_relavent_landmark_id = vlandmark_properties.size();//landmark_id;
                            //if(landmark_id != this->vlandmark_properties.size())
                            //{
                            //    LOG(ERROR)<<"landmark_id != vlandmark_properties.size()!Error occured!landmark_id:"<<landmark_id<<",properties.size:"<<vlandmark_properties.size()<<endl;
                            //    throw "size not equal!";
                            //}
                            //创建在参考关键帧的投影约束.注意:这里没有,也不应该对'L'[landmark_id]进行任何的prior约束!

                            //这个不行.这个没有引入尺度信息,仅仅靠摄像机间的尺度信息和初始位置估计太不可靠了,必须有约束.
                            //这里应该是只能选是否用smart factor,factor的类型必须是Cal3_S2Stereo,而不能是GenericProjectionFactor<...,...,Cal3_S2>.
                            //除非把每组双目认为是两个相机,两个相机之间再用prior约束一次,有点太鸡肋了,误差也不好控制.
                            //method<1> 最普通的投影误差,没有任何技巧.
                            //graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
                            //            measurement, measurementNoise, Symbol('X',reference_kf_id*cam_count + i), Symbol('L', map_point_relavent_landmark_id), this->v_pcams_gtsam_config_stereo_left[i]);
                            //method<2>.GenericStereoFactor
                            p2dT point_at_kf_left = pKeyFrameReference->p2d_vv.at(i).at(pKeyFrameReference->map3d_to_2d_pt_vec.at(i).at(p2d_index));
                            double xl_ = point_at_kf_left.x;double y_ = point_at_kf_left.y;
                            double disp = pKeyFrameReference->disps_vv.at(i).at(p2d_index);
                            double xr_ = xl_-disp;//Take care:xr_ shall be smaller than xl_!!!
                            LOG(INFO)<<"xl,xr,disp:"<<xl_<<","<<xr_<<","<<disp<<endl;

                            if(method == METHOD_SIMPLE_STEREO_REPROJECTION_ERROR)
                            {//加入在关键帧上面的观测.
                                //创建待优化点.这个不要了.只用SmartFactor.
                                //initialEstimate.insert(Symbol('L',landmark_id),
                                //                       gtsam::Point3(point_pos_initial_guess)
                                //                       );//landmark.
                                auto p3__ = pKeyFrameReference->p3d_vv.at(i).at(p2d_index);
                                Pose3 relative_cam_pose = initialEstimate.at(Symbol('X',reference_kf_id*cam_count + i)).cast<Pose3>();
                                Vector3d position;

                                {
                                    position[0] = p3__.x;position[1] = p3__.y;position[2] = p3__.z;
                                    Eigen::Matrix3d mRotate = relative_cam_pose.rotation().matrix();
//                                    position = mRotate.inverse() * position - relative_cam_pose.translation().vector();
                                    position = mRotate * position + relative_cam_pose.translation().vector();

                                    //Point3 initial_estimation_of_new_landmark =
                                        //initialEstimate.at(Symbol('X',reference_kf_id*cam_count + i)).cast<Pose3>().transformFrom(Point3(p3__.x,p3__.y,p3__.z));
                                }
                                initialEstimate.insert(Symbol('L',map_point_relavent_landmark_id),//Point3(0,0,0)
                                                       //initial_estimation_of_new_landmark
                                                       Point3(position)
                                                       );
                                graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(xl_,//左目的u
                                                                                                      xr_,//右目的u
                                                                                                      y_),//观测的v.
                                                                                         robust_kernel,
                                                                                         Symbol('X',reference_kf_id*cam_count + i),
                                                                                         Symbol('L', map_point_relavent_landmark_id),
                                                                                         this->v_pcams_gtsam_config_stereo[i]);//创建双目观测约束.
                                landmark_properties lp_;
                                lp_.pRelativeStereoSmartFactor = nullptr;
                                lp_.pCreatedByFrame = weak_ptr<Frame>(pKeyFrameReference);
                                lp_.observed_by_frames.push_back(weak_ptr<Frame>(pKeyFrameReference));
                                this->vlandmark_properties.push_back(lp_);
                                LOG(INFO)<<"KeyFrame: cam_id"<<i<<",create landmark and link between Landmark L"<<landmark_id<<" at "<<p3__.x<<","<<p3__.y<<","<<p3__.z<<" and KeyFrame X"<<reference_kf_id*cam_count + i<<endl;
                                LOG(INFO)<<"    Transformed pt 3d coordinate:"<<position[0]<<","<<position[1]
                                            <<","<<position[2]<<endl;

                            }
                            //method<4>.SmartFactor of GenericStereoFactor

                            //这种情况下,smartFactor本身就是landmark,无需额外在创建了
                            else if(method == METHOD_SMARTFACTOR_STEREO_REPROJECTION_ERROR)
                            {
                                SmartProjectionParams params(HESSIAN, ZERO_ON_DEGENERACY);
                                auto smart_stereo_factor = SmartStereoProjectionPoseFactor::shared_ptr(
                                            new SmartStereoProjectionPoseFactor(robust_kernel, params));
                                graph.push_back(smart_stereo_factor);
                                //第一次使用smart factor::add(),加入关键帧处的观测约束.
                                smart_stereo_factor->add(StereoPoint2(xl_, xr_, y_),Symbol('X',reference_kf_id*cam_count + i),this->v_pcams_gtsam_config_stereo[i]);//这个v_pcams里面是空的...
                                landmark_properties lp_;
                                //TODO:填上其他属性.这个lp_如果不用smart factor的话现在看来不一定要用.
                                lp_.pRelativeStereoSmartFactor = smart_stereo_factor;
                                lp_.pCreatedByFrame = weak_ptr<Frame>(pKeyFrameReference);
                                lp_.observed_by_frames.push_back(weak_ptr<Frame>(pKeyFrameReference));
                                vlandmark_properties.push_back(lp_);
                                LOG(INFO)<<"KeyFrame: cam_id"<<i<<",create landmark and link between Landmark L"<<landmark_id<<" and KeyFrame X"<<reference_kf_id*cam_count + i<<endl;
                            }
                            landmark_id++;

                        }
                        //2.不管是不是第一次被观测,都要创建在当前帧map_point对应的landmark 与 2d投影点 的gtsam factor约束.
                        //smartFactors.at(map_point_relavent_landmark_id)->add(StereoPoint2(xl, xr, y), X(frame), K);//使用SmartStereo的方法.这里使用Mono(参考ISAM2Example_SmartFactor.cpp).
                        //method <1> 创建最普通的投影误差.
                        this->vlandmark_properties_mutex.unlock();
                        auto p3__ = pKeyFrameReference->p3d_vv.at(i).at(p2d_index);
                        p3d_pts_matched.push_back(cv::Point3d(p3__.x,p3__.y,p3__.z));
                        p2d_pts_matched.push_back(cv::Point2d(left_p2f_vv.at(i).at(p2d_index).x,left_p2f_vv.at(i).at(p2d_index).y));
                        if(method == METHOD_SIMPLE_STEREO_REPROJECTION_ERROR)
                        {
                            //graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
                            //      Point2(...), mono_reprojection_noise_model, Symbol('X',frame_id*cam_count + i), Symbol('L', map_point_relavent_landmark_id),
                            //      this->v_pcams_gtsam_config_stereo_left[i]);
                            Eigen::Matrix4d cam_to_body;
                            cv2eigen(pFrame->cam_info_stereo_vec.at(i).getRTMat().inv(),cam_to_body);
                            graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(
                                        //1.stereo point
                                            StereoPoint2(left_p2f_vv.at(i).at(p2d_index).x,//左目的u
                                                            right_p2f_vv.at(i).at(p2d_index).x,//右目的u
                                                            left_p2f_vv.at(i).at(p2d_index).y),
                                        //2.stereo noise.
                                            robust_kernel,
                                            Symbol('X',frame_id*cam_count+i),
                                            Symbol('L',map_point_relavent_landmark_id),
                                            this->v_pcams_gtsam_config_stereo.at(i)//,
                                            //false,true,
                                            //cam_to_body
                                        );
                            auto& lp_ = this->vlandmark_properties.at(map_point_relavent_landmark_id);
                            lp_.observed_by_frames.push_back(weak_ptr<Frame>(pKeyFrameReference));
                            cout<<"landmark L"<<map_point_relavent_landmark_id<<"at"<<left_p2f_vv.at(i).at(p2d_index).x<<","<<right_p2f_vv.at(i).at(p2d_index).x
                                                            <<" observed by "<<lp_.observed_by_frames.size()<<" frames at image pair "<<i<<"!"<<endl;
                            LOG(INFO)<<"OrdinaryFrame: cam_id:"<<i<<",create link between Landmark L"<<map_point_relavent_landmark_id<<" and X"<<frame_id*cam_count+i<<",disparity:"<<left_p2f_vv.at(i).at(p2d_index).x - right_p2f_vv.at(i).at(p2d_index).x<<endl;
                        }
                        //method <2> 创建smart stereo factor上面的约束.
                        else if(method == METHOD_SMARTFACTOR_STEREO_REPROJECTION_ERROR)
                        {
                            auto &lp_ = this->vlandmark_properties.at(map_point_relavent_landmark_id);
                            auto smart_factor_ = lp_.pRelativeStereoSmartFactor;//查找老的点.
                            smart_factor_->add(StereoPoint2(left_p2f_vv.at(i).at(p2d_index).x,//左目的u
                                                            right_p2f_vv.at(i).at(p2d_index).x,//右目的u
                                                            left_p2f_vv.at(i).at(p2d_index).y),
                                                    Symbol('X',frame_id*cam_count+i),//
                                                    //Symbol('L',map_point_relavent_landmark_id),
                                                    this->v_pcams_gtsam_config_stereo[i]
                                               );//模仿上面的.TODO.这种的缺点是没法再用mono约束这个点了,视野会比较窄...而且还需要在非关键帧上再进行一次opt track.
                            //可能对不同的点需要做不同的方式处理.需要一个判断逻辑.
                            LOG(INFO)<<"OrdinaryFrame: cam_id:"<<i<<",create link between Landmark L"<<map_point_relavent_landmark_id<<" and X"<<frame_id*cam_count+i<<endl;
                            lp_.observed_by_frames.push_back(weak_ptr<Frame>(pKeyFrameReference));
                            cout<<"landmark L"<<map_point_relavent_landmark_id<<"at"<<left_p2f_vv.at(i).at(p2d_index).x<<","<<right_p2f_vv.at(i).at(p2d_index).x
                                                            <<" observed by "<<lp_.observed_by_frames.size()<<" frames at image pair "<<i<<"!"<<endl;
                            LOG(INFO)<<"OrdinaryFrame: cam_id:"<<i<<",create link between Landmark L"<<map_point_relavent_landmark_id<<" and X"<<frame_id*cam_count+i<<",disparity:"<< right_p2f_vv.at(i).at(p2d_index).x-left_p2f_vv.at(i).at(p2d_index).x<<endl;
                        }
                        //TODO:处理其他类型...
                        //input_p2d_to_optimize.push_back(vp2d_pts[p2d_index]);
                        //input_p3d_to_optimize.push_back(vp3d_pts[p2d_index]);
                    }
                }

                cv::Mat pnp_R_vec,pnp_R;
                cv::Mat pnp_t;

                vector<unsigned char> pnp_ransac_is_inlier;
                bool pnp_estimate_success = cv::solvePnPRansac(p3d_pts_matched,p2d_pts_matched,this->stereo_config.at(i).getCamMat(),cv::Mat(),pnp_R_vec,pnp_t,false,100,2.0,0.99,pnp_ransac_is_inlier);
                int pnp_success_count = 0;
                for (auto u:pnp_ransac_is_inlier)
                {
                    if(u)
                    {
                        pnp_success_count++;
                    }
                }
                LOG(INFO)<<"pnp ransac matched pts count:"<<pnp_success_count<<endl;
                cout<<"pnp ransac matched pts count:"<<pnp_success_count<<endl;
                if(pnp_estimate_success && pnp_success_count>15 && !is_obvious_movement(pnp_R_vec,pnp_t) && i == 0)
                {
                    cv::Rodrigues(pnp_R_vec,pnp_R);


                    Eigen::Matrix3d Rotation_;
                    cv::cv2eigen(pnp_R,Rotation_);
                    Eigen::Vector3d translation_(pnp_t.at<double>(0),pnp_t.at<double>(1),pnp_t.at<double>(2));
                    //建立从关键帧到对应关系.
                    //gtsam::Pose3 relative_pose_(
                    //                                gtsam::Rot3(Rotation_),
                    //                                gtsam::Point3(translation)
                    //                            );//between pKeyFrameReference and pFrame;
                    Rot3 r_tmp(Rotation_);
                    Point3 t_tmp(pnp_t.at<double>(0),pnp_t.at<double>(1),pnp_t.at<double>(2));
                    gtsam::Pose3 relative_pose_(r_tmp,t_tmp);

                    Eigen::Matrix4d relative_pose_estimation;
                    static const auto A14 = Eigen::RowVector4d(0,0,0,1);
                    relative_pose_estimation<<relative_pose_.rotation().matrix(), relative_pose_.translation().vector(), A14;

                    Eigen::Matrix4d cam_body_relative,cam_body_relative_inv;
                    cv::cv2eigen( this->stereo_config.at(i).getRTMat(),cam_body_relative);
                    cam_body_relative_inv = cam_body_relative.inverse();

                    relative_pose_estimation = relative_pose_estimation*cam_body_relative_inv;
                    LOG(INFO)<<"relative_pose_estimation:"<<endl<<relative_pose_estimation<<endl;
                    cout<<"relative_pose_estimation:"<<endl<<relative_pose_estimation<<endl;




                    //Eigen::Matrix4d
                    auto keyframe_pose_estimation_pose3 = initialEstimate.at(Symbol('X',( pKeyFrameReference->frame_id*cam_count + i)) ).cast<Pose3>();
                    Eigen::Matrix4d kf_pose_estimation;
                    kf_pose_estimation<<keyframe_pose_estimation_pose3.rotation().matrix(),keyframe_pose_estimation_pose3.translation().vector(),A14;
                    //Eigen::Matrix4d
                    //Eigen::Matrix4d final_pose_estimation = kf_pose_estimation.inverse() * relative_pose_estimation.inverse();
                    //Eigen::Matrix4d final_pose_estimation = kf_pose_estimation * relative_pose_estimation;
                    Eigen::Matrix4d final_pose_estimation = relative_pose_estimation* kf_pose_estimation;

                    LOG(INFO)<<"kf_pose_estimation:"<<endl<<kf_pose_estimation<<endl;
                    cout<<"kf_pose_estimation:"<<endl<<kf_pose_estimation<<endl;

                    LOG(INFO)<<"final_pose_estimation:"<<endl<<final_pose_estimation<<endl;
                    cout<<"final_pose_estimation:"<<endl<<final_pose_estimation<<endl;

                    initialEstimate.insert(Symbol('X',pFrame->frame_id*cam_count + i),Pose3(final_pose_estimation));
                    LOG(INFO)<<"PNP ransac result: t:"<<translation_<<endl;

                }
                else
                {
                    Pose3 prev_pose = initialEstimate.at(Symbol('X', (pFrame->frame_id-1)*cam_count +i  ) ).cast<Pose3>();
                    initialEstimate.insert(Symbol('X',pFrame->frame_id*cam_count + i),prev_pose);
                    LOG(INFO)<<"pnp ransac failed;using last pose to init!"<<endl;
                }




            }
//            if(frame_id == 0)//不对,这个还不能放这里,会导致pKeyFrameRef是空.
//            {
//                cout<<"initialized the 1st frame successfully!"<<endl;
//                frame_id++;//记录这是第几个frame.
//                cout<<"Before calling isam2 update:"<<endl;
//                initialEstimate.print("initial estimate:(not optimized)");
//                this->currentEstimate = this-> initialEstimate;
//                //isam.update(graph, initialEstimate);//优化.
//                //this->currentEstimate = isam.calculateEstimate();
//                //currentEstiate.print("before resize,values:");
//                //graph.resize(0);
//                //initialEstimate.clear();//这里不能做任何操作.否则引起indetermined linear system exception.
//                return;
//            }
            //step<2>.process result.
            //TODO:记录跟踪点.
            if(frame_id>=2)//||method == )
            {
                //cout<<"Before calling isam2 update, print initialEstimate:"<<endl;
                //initialEstimate.print();

                if(DEBUG_USE_LM ==1)
                {
                    //if(this->frame_id%3 == 0)
                    if(this->frame_id%600 == 0)
                    {
                        for(int i = 0;i<frame_id;i++)
                        {
                            Pose3 var_x = initialEstimate.at(Symbol('X',i*2)).cast<Pose3>();
                            LOG(INFO)<<"    [ESTIMATED RESULT] node_id:"<<i<<";xyz:"<<var_x.translation().x()<<","<<var_x.translation().y()<<","<<var_x.translation().z()<<endl;

                        }
                        cout<<"print initialEstimate:"<<endl;
                        initialEstimate.print();
                        cout<<"debug:call lm optimizer!"<<endl;
                        LevenbergMarquardtParams params;
                        params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
                        params.verbosity = NonlinearOptimizerParams::ERROR;


                        params.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;
                        //params.linearSolverType = NonlinearOptimizerParams::SEQUENTIAL_QR;
                        //params.absoluteErrorTol = 1e-10;
                        //params.relativeErrorTol = 1e-10;
                        //params.ordering = Ordering::Create(Ordering::METIS, graph);

                        cout << "Optimizing" << endl;
                        //create Levenberg-Marquardt optimizer to optimize the factor graph


                        LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
                        Values result = optimizer.optimize();

                        //Values result = DoglegOptimizer(graph, initialEstimate).optimize();

                        //Values result = GaussNewtonOptimizer(graph,initialEstimate).optimize();

                        //initialEstimate.
                        cout<<"optimized result:"<<endl;
                        result.print();
                        graph.printErrors(result);
                        if(frame_id%600== 0 )
                        {
                            LOG(INFO)<<"Output result for visualization:"<<endl;

                            for(int i = 0;i<frame_id;i++)
                            {
                                Pose3 var_x = result.at(Symbol('X',i*2)).cast<Pose3>();
                                LOG(INFO)<<"    [OFFLINE OPTIMIZED RESULT] node_id:"<<i<<";xyz:"<<var_x.translation().x()<<","<<var_x.translation().y()<<","<<var_x.translation().z()<<endl;

                            }
                            for(int i = 0;i<this->vlandmark_properties.size();i++)
                            {
                                //auto p_frame = vlandmark_properties.at(i).pCreatedByFrame.lock();
                                //if(p_frame!=nullptr&&p_frame->frame_id%2 == 0)//只看第一个摄像头看到的点.
                                if(true)//先不筛选摄像头.看一下大概分布.
                                {
                                    Point3 landmark_l = result.at(Symbol('L',i)).cast<Point3>();
                                    LOG(INFO)<<"  [OFFLINE OPTIMIZED LANDMARK RESULT] landmark_id:"<<i<<";xyz:"<<landmark_l.x()<<","<<landmark_l.y()<<","<<landmark_l.z()<<endl;
                                }
                            }
                        }
                        std::string output_file_name = "info_graph_g2o";
                        stringstream ss;
                        ss<<output_file_name<<(frame_id/3)<<".g2o";
                        cout<<"saving g2o file:"<<ss.str()<<"."<<endl;
                        stringstream ss2;
                        ss2<<"Pose2SLAMExample"<<(frame_id/3)<<".dot";
                        ofstream os(ss2.str());
                        //if(frame_id == 20)
                        //{
                        graph.saveGraph(os,result);
                        //}
                        writeG2o(graph, result, ss.str());
                        cout<<"g2o file saved!"<<endl;
                        exit(-1);
                    }
                    currentEstimate = initialEstimate;
                }
                else
                {
                    cout<<"will call isam2::update()"<<endl;
                    isam.update(graph, initialEstimate);//优化.
                    this->currentEstimate = isam.calculateEstimate();
                    graph.resize(0);
                    initialEstimate.clear();
                }
                if(method == METHOD_SMARTFACTOR_STEREO_REPROJECTION_ERROR)
                {
                    for(int sf_id = 0;sf_id<this->vlandmark_properties.size();sf_id++)
                    {
                        cout<<"for landmark "<<sf_id<<" ";
                        auto u = this->vlandmark_properties.at(sf_id);
                        auto is_valid = u.pRelativeStereoSmartFactor->isValid();
                        auto is_outl = u.pRelativeStereoSmartFactor->isOutlier();
                        auto is_farpoint = u.pRelativeStereoSmartFactor->isFarPoint();
                        auto is_degenerate = u.pRelativeStereoSmartFactor->isDegenerate();
                        auto is_behindcam = u.pRelativeStereoSmartFactor->isPointBehindCamera();
                        cout<<" is valid:"<<is_valid<<" ,is outlier:"<<is_outl<<" ,is far point:"<<is_farpoint<<" ,is degen:"<<is_degenerate<<" ,is behind cam:"<<is_behindcam<<"."<<endl;
                    }
                }
            }
            if(frame_id == 1)
            {
                currentEstimate = initialEstimate;
            }


            cout<<"in OptFlowForFrameWiseTracking stage7"<<endl;
            //TODO:处理每个摄像头的状态.
            //if(success_count>=3)
            //{
            //    output_frame_pnp_ransac_success = true;
            //}
            //cout<<"For all cams:total tracked points count:"<<total_success_tracked_point_count;
            frame_id++;//记录这是第几个frame.
            return;
        }

        void addStereoKeyFrameToBackEndAndOptimize(shared_ptr<Frame> pKeyFrame,shared_ptr<Frame> pReferenceKeyFrame,int& optimization_pts_tracked,int method = METHOD_SIMPLE_MONOCULAR_REPROJECTION_ERROR)
        {
            if(pReferenceKeyFrame == nullptr)
            {
                cout<<"Reference frame is null.Initializing."<<endl;
            }
            if(!pKeyFrame->isKeyFrame)
            {
                LOG(ERROR)<<"ERROR:In insertKeyFrameToBackEnd: frame is not keyframe!"<<endl;
                return;
            }
            this->addOrdinaryStereoFrameToBackendAndOptimize(pKeyFrame,pReferenceKeyFrame,optimization_pts_tracked,method);//先像处理普通帧一样,处理跟踪和定位问题.

            //从这里开始,这一帧的位置已经被初步优化.
            /*
            pKeyFrame->map_points = generateMapPointsCorespondingToStereoKFObservation();//创建新地图点.

            //add vertex for frame.
            //add IDP Error in Frame.
            //convert pKeyFrame->p3d_vv to MapPoints;
            //TODO:是否先尝试一下pnpransac,再进行新关键帧创建?
            //mcs::trackAndDoSolvePnPRansacMultiCam(...)
            //对每组摄像头:
            int cam_count = pKeyFrame->map2d_to_3d_pt_vec.size();

            //挪到addOrdinaryStereoFrameToBackendAndOptimize()里面去.关键帧不要做这个操作 .
            //  //2.add reprojection factor between this frame and last frame.
            //shared_ptr<Frame> pLastFrame = this->getLastKF();
            //int image_count = pFrame->cam_info_stereo_vec.size();
            //isam.update(graph, initialEstimate);
            */
            //TODO:加入marginalize策略.
        }
        void doMarginalize()
        {
            ;
        }
        void addPose();
        void addPoint3d();
        void addPoint2d_3d_projection();

        //
        //void addCamObservation(int cam_index,vector<...> p2ds,vector<...> p3ds)
        //{
        //    graph.emplace_shared(GeneralICPFactor<Pose3,Point3>(Symbol('c',cam_index + cam_num*frame_index),p3d,Noise ....));//bind with relative camera.
        //}

    };
    /*
        void initCams();
        {
            noiseModel bind_model(0,0,0,0,0,0);
            //Symbol('c',0) stands for original pose.
            for(caminfo = CamInfoList[i],.....)
            {
                Pose3 CamPose = ....
                graph.addPrior<BetweenFactor>(Symbol('c',i),Symbol('c',0),CamPose,bind_model);

            }

        }
            void insertFrame();
            {
                noiseModel bind_model(0,0,0,0,0,0);
                //Symbol('c',0) stands for original pose.
                for(caminfo = CamInfoList[i],.....)
                {
                    Pose3 CamPose = ....
                    graph.addPrior<BetweenFactor>(Symbol('c',i+cam_num*frame_index),Symbol('c',0),CamPose,bind_model);

                }

            }
    */

}
#endif
