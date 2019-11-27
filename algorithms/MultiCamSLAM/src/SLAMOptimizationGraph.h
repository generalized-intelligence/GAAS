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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
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


#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>


#include "Frame.h"
#include "FrameManager.h"
#include <thread>
#include <opencv2/core/eigen.hpp>

using namespace gtsam;
using namespace std;
using namespace cv;

namespace mcs
{
    struct landmark_properties;


    struct landmark_properties
    {
        int landmark_reference_time = 0;
        weak_ptr<Frame> pCreatedByFrame;
        weak_ptr<Frame> pLastObservedByFrame;
        boost::shared_ptr<SmartStereoProjectionPoseFactor> pRelativeStereoSmartFactor;//目前还没实现处理无穷远.

    };
    void doFrontEndTrackingForOrdinaryFrames(shared_ptr<Frame> pFrame,shared_ptr<Frame> pKeyFrameReference,int cam_index,
                                             vector<Point2f>& output_tracked_pts_left,vector<Point2f>& output_tracked_pts_right,
                                             map<int,int>& map_point2f_to_kf_p3ds,vector<p2dT>& output_to_track_kf_p2d,char* p_output_track_success,int method = 0)
    {//这个函数要支持多线程.
        //step<1> 追踪左目.
        const int& i = cam_index;
        *p_output_track_success = false;
        auto p_origin_img = pKeyFrameReference->getMainImages().at(i);
        auto p_left = pFrame->getMainImages().at(i);
        auto p_right= pFrame->getSecondaryImages().at(i);
        vector<p3dT> vp3d_pts = pKeyFrameReference->p3d_vv.at(i);
        vector<p2dT>& to_track_vp2d_pts = output_to_track_kf_p2d;//要追踪的参考帧 kps.
        to_track_vp2d_pts.clear();
        for(int index_p3d=0;index_p3d<vp3d_pts.size();index_p3d++)
        {
            to_track_vp2d_pts.push_back(pKeyFrameReference->p2d_vv[i][pKeyFrameReference->map3d_to_2d_pt_vec[i][index_p3d] ]);//查找对应关键帧的p2d.只追踪成功三角化的点.
        }
        vector<unsigned char> left_track_success;
        vector<Point2f> left_tracked_pts;
        {//call cv::OptFlow for left img.
            vector<float> err;
            cv::calcOpticalFlowPyrLK(*p_origin_img,*p_left,to_track_vp2d_pts,left_tracked_pts,left_track_success,err,cv::Size(21, 21), 3,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        }

        //if(method == 0)
        //{
        //    just return the coresponding pts.
        //}
        //else if(method == 1)
        {//双目性质的追踪.
            vector<unsigned char> right_track_success;
            vector<Point2f> right_tracked_pts;
            vector<float> err2;
            cv::calcOpticalFlowPyrLK(*p_left,*p_right,left_tracked_pts,right_tracked_pts,err2,right_track_success,cv::Size(21,21),3,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
            //step<2> 筛选关键帧和左侧能匹配,且左侧和右侧能匹配的.
            for(int pt_index = 0;pt_index<left_tracked_pts.size();pt_index++)
            {
                //step<3> 生成最终结果并产生map.
                if(left_track_success.at(pt_index) && right_track_success.at(pt_index) )
                {//双目成功//TODO:check diff v in left and right.
                    map_point2f_to_kf_p3ds[output_tracked_pts_left.size()] = pt_index;
                    output_tracked_pts_left.push_back(left_tracked_pts.at(pt_index));
                    output_tracked_pts_right.push_back(right_tracked_pts.at(pt_index));
                }
                else if(left_track_success.at(pt_index) && !right_track_success.at(pt_index))
                {//只有单目追踪成功.
                    LOG(WARNING)<<"This mono strategy not implemented yet."<<endl;
                }
            }
            int success_stereo_tracked_count = output_tracked_pts_left.size();
            if(success_stereo_tracked_count>15)
            {
                LOG(INFO)<<"stereo track success!"<<endl;
                *p_output_track_success = true;
            }
        }
    }
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
        vector<landmark_properties> vlandmark_properties;//暂时用不到.用到时候可以直接拿.
        int frame_id = 0;
        vector<boost::shared_ptr<Cal3_S2> > v_pcams_gtsam_config_stereo_left;
        vector<boost::shared_ptr<Cal3_S2Stereo> > v_pcams_gtsam_config_stereo;
        vector<Cal3_S2> v_cams_gtsam_config_depth;

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
        }

        void initCamsStereo(vector<StereoCamConfig>& cams)
        {
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
        void addOrdinaryStereoFrameToBackendAndOptimize(shared_ptr<Frame> pFrame,shared_ptr<Frame> pKeyFrameReference,int& optimization_pts_tracked,int method = METHOD_SIMPLE_MONOCULAR_REPROJECTION_ERROR)
        {
            optimization_pts_tracked = 0;
            ScopeTimer t1("addOrdinaryStereoFrameToBackend() timer");
            LOG(INFO)<<"in addOrdinaryStereoFrameToBackend() stage2"<<endl;

            //auto pts_vv = pFrame->p2d_vv;
            //pts_vv = OptFlowForFrameWiseTracking(*pframe2);//track frame2 获取对应位置的点...

            if(pFrame->frame_id != -1)
            {
                LOG(ERROR)<<"ERROR: in addOrdinaryStereoFrameToBackendAndOptimize():frame id exists!"<<endl;
                return;
            }
            pFrame->frame_id = frame_id;
            LOG(INFO)<<"In addOrdinaryStereoFrameToBackendAndOptimize(): step<1> create pose for each camera."<<endl;
            int cam_count = pFrame->get_cam_num();
            for(int i = 0;i < cam_count;i++)//对每个相机,先创建它的Pose3优化节点.
            {
                if(frame_id ==0 )//第一个frame.
                {
                    Eigen::Matrix4d cam_to_body_rt_mat;
                    cv2eigen(pFrame->cam_info_stereo_vec[i].getRTMat(),cam_to_body_rt_mat);
                    initialEstimate.insert(Symbol('X',frame_id*cam_count + i),Pose3(cam_to_body_rt_mat));
                    if(i == 0)//就算是这种情况,也只对第一组摄像头定绝对位置.
                    {
                        gtsam::noiseModel::Diagonal::shared_ptr priorFrame_Cam0_noisemodel = gtsam::noiseModel::Diagonal::Variances (
                                    ( gtsam::Vector ( 6 ) <<0.1, 0.1, 0.1, 1e-6, 1e-6, 1e-6 ).finished()
                                );
                        //添加初始绝对位置约束.Rotation可变(重力对齐),translation不可变(绝对坐标系创建的点).
                        graph.emplace_shared<PriorFactor<Pose3> > (Symbol('X',0), Pose3(Rot3(1,0,0,0),Point3(0,0,0)),priorFrame_Cam0_noisemodel);
                    }
                }
                else
                {
                    const Pose3* p_last = &(currentEstimate.at(Symbol('x',(frame_id - 1)*cam_count + i)).cast<Pose3>());
                    initialEstimate.insert(Symbol('X',frame_id*cam_count + i),Pose3(*p_last)); // 用上一帧的对应相机优化结果作为初始估计.
                }
                //加入相机位置约束.
                if(i != 0)
                {
                    noiseModel::Diagonal::shared_ptr noise_model_between_cams = gtsam::noiseModel::Diagonal::Variances (
                                ( gtsam::Vector ( 6 ) <<1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6 ).finished()); //1mm,0.1度.
                    Eigen::Matrix4d cam_to_cam0_rt_mat;
                    cv2eigen(pFrame->cam_info_stereo_vec[0].getRTMat().inv()* pFrame->cam_info_stereo_vec[i].getRTMat(),cam_to_cam0_rt_mat);
                    graph.emplace_shared<BetweenFactor<Pose3> >(Symbol('x',frame_id*cam_count),Symbol(frame_id*cam_count + i),Pose3(cam_to_cam0_rt_mat),
                                                                  noise_model_between_cams);//用非常紧的约束来限制相机间位置关系.
                }
            }
            if(frame_id == 0)
            {
                LOG(INFO)<<"initializing!"<<endl;
                frame_id++;//记录这是第几个frame.
                return;
            }
            auto imgs_prev = pKeyFrameReference->getMainImages();
            auto imgs_next = pFrame->getMainImages();
            auto imgs_next_right = pFrame->getSecondaryImages();
            LOG(INFO)<<"in addOrdinaryStereoFrameToBackend() stage3"<<endl;

            auto p3ds_vv = pKeyFrameReference->p3d_vv;
            //多线程实现

            vector<std::thread> threads_frontend;
            vector<vector<Point2f> > left_p2f_vv,right_p2f_vv,output_to_track_kf_p2d_vv;
            vector<map<int,int> > p2f_to_p3d_maps;
            vector<char> track_success_vcams;
            track_success_vcams.resize(cam_count);
            left_p2f_vv.resize(cam_count);right_p2f_vv.resize(cam_count);output_to_track_kf_p2d_vv.resize(cam_count);
            p2f_to_p3d_maps.resize(cam_count);track_success_vcams.resize(cam_count);
            for(int i = 0;i<cam_count;i++)
            {
                int cam_index = i;
                auto lp2fv = left_p2f_vv.at(cam_index);auto rp2fv = right_p2f_vv.at(cam_index);
                auto map__ = p2f_to_p3d_maps.at(cam_index);
                auto tracked_p2d = output_to_track_kf_p2d_vv.at(cam_index);

                threads_frontend.push_back(std::thread(
                                               doFrontEndTrackingForOrdinaryFrames,std::ref(pFrame),std::ref(pKeyFrameReference),cam_index,
                                               std::ref(lp2fv),std::ref(rp2fv),
                                               std::ref(map__),std::ref(tracked_p2d),&(track_success_vcams[cam_index]),0
                                               ));
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

            for(int i = 0;i < cam_count;i++)
            {
                map<int,int> tracked_kps_to_original_kps_map_output;//关键帧的p3d到追踪到的p2d的关系
                vector<p3dT> vp3d_pts = p3ds_vv.at(i);
                auto& vp2d_pts = output_to_track_kf_p2d_vv.at(i);
                for(auto iter = p2f_to_p3d_maps.at(i).begin(); iter != p2f_to_p3d_maps.at(i).end(); ++iter)
                {//交换键值关系
                    auto key = iter->first;
                    auto val = iter->second;
                    tracked_kps_to_original_kps_map_output[val] = key;
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
                        int map_point_relavent_landmark_id = pKeyFrameReference->get_p3dindex_to_landmark_index(p2d_index);//获取对应Landmark的id.失败返回-1.
                        if(map_point_relavent_landmark_id == -1) // 第一次被另一个帧观测.
                        {
                            int reference_kf_id = pKeyFrameReference->frame_id;
                            const Pose3* p_kf_pose3 = &(currentEstimate.at(Symbol('x',reference_kf_id*cam_count + i)).cast<Pose3>());
                            Matrix3d kf_rot = p_kf_pose3->rotation().matrix();
                            Vector3d kf_trans = p_kf_pose3->translation().vector();
                            Vector3d p3d_relative_to_cam;
                            Vector2d reprojected_p2d;
                            //cv2eigen(vp3d_pts.at(p2d_index),p3d_relative_to_cam);
                            //cv2eigen(vp2d_pts.at(p2d_index),reprojected_p2d);
                            p3d_relative_to_cam = Vector3d(vp3d_pts.at(p2d_index).x,vp3d_pts.at(p2d_index).y,vp3d_pts.at(p2d_index).z);
                            reprojected_p2d = Vector2d(vp2d_pts.at(p2d_index).x,vp2d_pts.at(p2d_index).y);

                            Vector3d point_pos_initial_guess = kf_rot*p3d_relative_to_cam + kf_trans; //TODO:通过对应关键帧计算其初始位置估计,所有都变换到一个坐标系下面.
                            //创建待优化点.
                            initialEstimate.insert(Symbol('L',landmark_id),
                                                   gtsam::Point3(point_pos_initial_guess)
                                                   );//landmark.

                            //
                            pKeyFrameReference->set_p3d_landmark_index(p2d_index,landmark_id);  //绑定对应关系.
                            map_point_relavent_landmark_id = landmark_id;
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
                            double xr_ = xl_+disp;

                            if(method == 0)
                            {//加入在关键帧上面的观测.
                                graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(xl_,//左目的u
                                                                                                      xr_,//右目的u
                                                                                                      y_),//观测的v.
                                                                                         stereo_reprojection_noise_model,
                                                                                         Symbol('X',reference_kf_id*cam_count + i),
                                                                                         Symbol('L', map_point_relavent_landmark_id),
                                                                                         this->v_pcams_gtsam_config_stereo[i]);//创建双目观测约束.
                            }
                            //method<4>.SmartFactor of GenericStereoFactor

                            //这种情况下,smartFactor本身就是landmark,无需额外在创建了
                            else if(method == 1)
                            {
                                auto gaussian = noiseModel::Isotropic::Sigma(3, 1.0);
                                SmartProjectionParams params(HESSIAN, ZERO_ON_DEGENERACY);
                                auto smart_stereo_factor = SmartStereoProjectionPoseFactor::shared_ptr(
                                            new SmartStereoProjectionPoseFactor(gaussian, params));
                                graph.push_back(smart_stereo_factor);
                                //第一次使用smart factor::add(),加入关键帧处的观测约束.
                                smart_stereo_factor->add(StereoPoint2(xl_, xr_, y_),Symbol('X',reference_kf_id*cam_count + i),this->v_pcams_gtsam_config_stereo[i]);
                                landmark_properties lp_;
                                //TODO:填上其他属性.这个lp_如果不用smart factor的话现在看来不一定要用.
                                lp_.pRelativeStereoSmartFactor = smart_stereo_factor;
                                vlandmark_properties.push_back(lp_);
                            }
                            landmark_id++;
                        }
                        //2.不管是不是第一次被观测,都要创建在当前帧map_point对应的landmark 与 2d投影点 的gtsam factor约束.
                        //smartFactors.at(map_point_relavent_landmark_id)->add(StereoPoint2(xl, xr, y), X(frame), K);//使用SmartStereo的方法.这里使用Mono(参考ISAM2Example_SmartFactor.cpp).
                        //method <1> 创建最普通的投影误差.
                        if(method == 0)
                        {
                            //graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
                            //      Point2(...), mono_reprojection_noise_model, Symbol('X',frame_id*cam_count + i), Symbol('L', map_point_relavent_landmark_id), this->v_pcams_gtsam_config_stereo_left[i]);
                        }
                        //method <2> 创建smart stereo factor上面的约束.
                        else if(method == 1)
                        {
                            auto smart_factor_ = this->vlandmark_properties[map_point_relavent_landmark_id].pRelativeStereoSmartFactor;
                            smart_factor_->add(StereoPoint2(left_p2f_vv.at(i).at(p2d_index).x,//左目的u
                                                            right_p2f_vv.at(i).at(p2d_index).x,//右目的u
                                                            left_p2f_vv.at(i).at(p2d_index).y),
                                               Symbol('X',frame_id*cam_count+i),//
                                               //Symbol('L',map_point_relavent_landmark_id),
                                               this->v_pcams_gtsam_config_stereo[i]
                                               );//模仿上面的.TODO.这种的缺点是没法再用mono约束这个点了,视野会比较窄...而且还需要在非关键帧上再进行一次opt track.
                            //可能对不同的点需要做不同的方式处理.需要一个判断逻辑.
                        }
                        //TODO:处理其他类型...
                        //input_p2d_to_optimize.push_back(vp2d_pts[p2d_index]);
                        //input_p3d_to_optimize.push_back(vp3d_pts[p2d_index]);
                    }
                }
            }
            //step<2>.process result.
            //TODO:记录跟踪点.
            isam.update(graph, initialEstimate);//优化.
            this->currentEstimate = isam.calculateEstimate();

            LOG(INFO)<<"in OptFlowForFrameWiseTracking stage7"<<endl;
            //TODO:处理每个摄像头的状态.
            //if(success_count>=3)
            //{
            //    output_frame_pnp_ransac_success = true;
            //}
            //LOG(INFO)<<"For all cams:total tracked points count:"<<total_success_tracked_point_count;
            frame_id++;//记录这是第几个frame.
            return;
        }

        void addStereoKeyFrameToBackEndAndOptimize(shared_ptr<Frame> pKeyFrame,shared_ptr<Frame> pReferenceKeyFrame,int& optimization_pts_tracked,int method = METHOD_SIMPLE_MONOCULAR_REPROJECTION_ERROR)
        {
            if(!pKeyFrame->isKeyFrame)
            {
                LOG(ERROR)<<"ERROR:In insertKeyFrameToBackEnd: frame is not keyframe!"<<endl;
                return;
            }
            this->addOrdinaryStereoFrameToBackendAndOptimize(pKeyFrame,pReferenceKeyFrame,optimization_pts_tracked,method);//先像处理普通帧一样,处理跟踪和定位问题.
            if(pReferenceKeyFrame == nullptr)
            {
                LOG(INFO)<<"Reference frame is null.Initializing."<<endl;
            }
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
