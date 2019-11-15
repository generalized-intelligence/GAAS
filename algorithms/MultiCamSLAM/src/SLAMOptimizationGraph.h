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


#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>


#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>


#include "Frame.h"
#include "FrameManager.h"

using namespace gtsam;
using namespace std;

namespace mcs
{
    struct landmark_properties;


    struct landmark_properties
    {
        int landmark_reference_time = 0;
        weak_ptr<Frame> pCreatedByFrame;
        weak_ptr<Frame> pLastObservedByFrame;
        shared_ptr<SmartStereoProjectionPoseFactor> pRelativeStereoSmartFactor;//目前还没实现处理无穷远.

    }
    class SLAMOptimizationGraph
    {
    private:
        ISAM2Params parameters;
        //parameters.relinearizeThreshold = 0.1;
        ISAM2 isam;//(parameters);
        NonlinearFactorGraph graph;
        Values initialEstimate;
        cv::FileStorage* pfSettings;
        vector<StereoCamConfig> stereo_config;
        vector<CamInfo> rgbd_config;
        int landmark_id = 0;
        vector<landmark_properties> vlandmark_properties;//暂时用不到.用到时候可以直接拿.
        int frame_id = 0;
        vector<Cal3_S2> v_cams_gtsam_config_stereo_left;
        vector<Cal3_S2Stereo> v_cams_gtsam_config_stereo;
        vector<Cal3_S2> v_cams_gtsam_config_depth;




    public:
        SLAMOptimizationGraph(cv::FileStorage fSettings)
        {
            //...
        }

        void initCamsStereo(vector<StereoCamConfig>& cams)
        {
            for(auto c:cams)
            {
                float fx,fy,cx,cy;
                c.getLCamMatFxFyCxCy(fx,fy,cx,cy);
                Cal3_S2 k_l(fx,fy,0,cx,cy);//这种形式的相机标定是可以优化的.如果需要,可以后续在重投影过程中附加进行一个内参优化.
                //noiseModel::Diagonal::shared_ptr calNoise = noiseModel::Diagonal::Sigmas((Vector(5) << 500, 500, 0.1, 100, 100).finished());
                //就像这样.具体参考examples/SelfCalibrationExample.cpp
                v_cams_gtsam_config_stereo_left.push_back(k_l);//先建立主视觉的K.

                //double fx, double fy, double s, double u0, double v0, double b//这要求双目rectify之后共享一组fx,fy cx,cy且必须旋转也对齐.需要预处理.
                double b = c.getBaseLine();
                Cal3_S2Stereo k_stereo(fx,fy,0,cx,cy,b);
                this->v_cams_gtsam_config_stereo.push_back(k_stereo);
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

        const int METHOD_SIMPLE_MONOCULAR_REPROJECTION_ERROR = 0;
        const int METHOD_SIMPLE_STEREO_REPROJECTION_ERROR = 1;
        const int METHOD_SMARTFACTOR_MONOCULAR_REPROJECTION_ERROR = 2;
        const int METHOD_SMARTFACTOR_STEREO_REPROJECTION_ERROR = 3;//可以做位操作.
        void addOrdinaryStereoFrameToBackendAndOptimize(shared_ptr<Frame> pFrame,shared_ptr<Frame> pKeyFrameReference,int method = METHOD_SIMPLE_MONOCULAR_REPROJECTION_ERROR)
        {
            int total_success_tracked_point_count = 0;
            ScopeTimer t1("addOrdinaryStereoFrameToBackend() timer");
            int success_count = 0;//TODO:加入一个策略,完成多摄像头之间的筛选.
            //doSolvePnPRansac
            LOG(INFO)<<"in addOrdinaryStereoFrameToBackend() stage2"<<endl;
            auto imgs_prev = pKeyFrameReference->getMainImages();
            auto imgs_next = pFrame->getMainImages();
            LOG(INFO)<<"in addOrdinaryStereoFrameToBackend() stage3"<<endl;

            auto p3ds_vv = pKeyFrameReference->p3d_vv;
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
                    Eigen::Matrix4d cam_to_body_rt_mat,final_rt_mat;
                    cv2eigen(pFrame->cam_info_stereo_vec[i].getRTMat(),cam_to_body_rt_mat);
                    final_rt_mat = cam_to_body_rt_mat;
                    final_rot = cam_to_body_rot;
                    initialEstimate.insert(Symbol('X',frame_id*cam_count + i),Pose3(final_rot));
                    //添加初始绝对位置约束.Rotation可变(重力对齐),translation不可变(绝对坐标系创建的点).
                    if(i == 0)//就算是这种情况,也只对第一组摄像头定绝对位置.
                    {
                        gtsam::noiseModel::Diagonal::shared_ptr priorFrame_Cam0_noisemodel = gtsam::noiseModel::Diagonal::Variances (
                                    ( gtsam::Vector ( 6 ) <<0.1, 0.1, 0.1, 1e-6, 1e-6, 1e-6 ).finished()
                                );
                        graph.emplace_shared<PriorFactor<Pose3> > (Symbol('X',0), Pose3(Rot3(1,0,0,0),Point3(0,0,0)),priorFrame_Cam0_noisemodel);
                    }
                }
                else
                {
                    const Pose3* p_last = &(currentEstimate.at(Symbol('x',(frame_id - 1)*cam_count + i).cast<Pose3>());
                    initialEstimate.insert(Symbol('X',frame_id*cam_count + i),Pose3(*p_last)); // 用上一帧的对应相机优化结果作为初始估计.
                }
                //加入相机位置约束.
                if(i != 0)
                {
                    noiseModel::Diagonal::shared_ptr noise_model_between_cams = gtsam::noiseModel::Diagonal::Variances (
                                ( gtsam::Vector ( 6 ) <<1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6 ).finished()); //1mm,0.1度.
                    Eigen::Matrix4d cam_to_cam0_rt_mat;
                    cv2eigen(pFrame->cam_info_stereo_vec[0].getRTMat().inv()* cam_info_stereo_vec[i].getRTMat(),cam_to_cam0_rt_mat);
                    graph.emplace_shared<BetweenFactor<Pose3> >(Symbol('x',frame_id*cam_count)),Symbol(frame_id*cam_count + i),Pose3(cam_to_cam0_rt_mat),
                                                                  noise_model_between_cams);//用非常紧的约束来限制相机间位置关系.
                }
            }
            for(int i = 0;i < cam_count;i++)
            {
                LOG(INFO)<<"Ref keyframe p3ds_vv.size():"<<p3ds_vv.size()<<";new frame p2ds_vv.size():"<<pts_vv.size()<<endl;

                LOG(INFO)<<"in OptFlowForFrameWiseTracking stage4.1"<<endl;
                cv::Mat camMat;
                if(isStereoMode)
                {
                    camMat = pKeyFrameReference->get_stereo_cam_info()[i].getCamMat();
                }
                else
                {
                    camMat = pKeyFrameReference->get_cam_info()[i].getCamMat();
                }
                LOG(INFO)<<"in OptFlowForFrameWiseTracking stage4.2"<<endl;
                output_r_mat = cv::Mat();
                output_t_vec = cv::Mat();
                vector<unsigned char> match_success;
                //TODO:multi_thread implementation.
                vector<p3dT> input_p3d_to_optimize;
                vector<p2dT> input_p2d_to_optimize;
                {//step<1> form optimization problem.
                    vector<p3dT> vp3d_pts = p3ds_vv[i];
                    vector<p2dT> vp2d_pts;//要追踪的参考帧 kps.
                    for(int index_p3d=0;index_p3d<vp3d_pts.size();index_p3d++)
                    {
                        vp2d_pts.push_back(pKeyFrameReference->p2d_vv[i][pKeyFrameReference->map3d_to_2d_pt_vec[i][index_p3d] ]);//查找对应关键帧的p2d.只追踪成功三角化的点.
                    }
                    vector<cv::KeyPoint> toTrack_2dpts;
                    cv::KeyPoint::convert(vp2d_pts,toTrack_2dpts);//要跟踪的点.
                    vector<Point2f> original_remaining_2dpts,tracked_nextimg_2dpts;//原始关键帧跟踪成功的点,在当前帧跟踪得到的下一个点.
                    vector<unsigned char> v_pt_track_success;
                    bool output_track_success;
                    map<int,int> tracked_kps_to_original_kps_map_output;//跟踪到的2d点到原始2d点的id to id map.
                    //step <1>. do opt flow.
                    OptFlowForFrameWiseTracking(*(imgs_prev[i]),*(imgs_next[i]),toTrack_2dpts,original_remaining_2dpts,tracked_nextimg_2dpts,tracked_kps_to_original_kps_map_output,v_pt_track_success,output_track_success);
                    for(int index_p3d = 0;index_p3d<vp3d_pts.size();index_p3d++)
                    {
                        int p2d_index = tracked_kps_to_original_kps_map_output[index_p3d];//原始关键帧要追踪的2d点的index(同时也是p3d的真实index)失败是-1.
                        if(p2d_index>=0)//a match has been found.
                        {
                            //TODO:
                            //1.检查这个map_point是否有这个对应的landmark;没有就先创建点.
                            int map_point_relavent_landmark_id = pKeyFrameReference->get_p3dindex_to_landmark_id(p2d_index);//获取对应Landmark的id.失败返回-1.
                            if(map_point_relavent_landmark_id == -1) // 第一次被另一个帧观测.
                            {


                                int reference_kf_id = pKeyFrameReference->frame_id;
                                const Pose3* p_kf_pose3 = &(currentEstimate.at(Symbol('x',reference_kf_id*cam_count + i).cast<Pose3>());
                                Matrix3d kf_rot = p_kf_pose3->rotation().matrix();
                                Vector3d kf_trans = p_kf_pose3->translation().vector();
                                Vector3d p3d_relative_to_cam;
                                Vector2d reprojected_p2d;
                                cv2eigen(vp3d_pts.at(p2d_index),p3d_relative_to_cam);
                                cv2eigen(vp2d_pts.at(p2d_index),reprojected_p2d);

                                Vector3d point_pos_initial_guess = kf_rot*p3d_relative_to_cam + kf_trans; //TODO:通过对应关键帧计算其初始位置估计,所有都变换到一个坐标系下面.
                                //创建待优化点.
                                initialEstimate.insert(Symbol('L',landmark_id),
                                                       gtsam::Point3(point_pos_initial_guess)
                                                       );//landmark.

                                //
                                pKeyFrameReference->set_p3d_landmark_id(p2d_index,landmark_id);  //绑定对应关系.
                                map_point_relavent_landmark_id = landmark_id;
                                //创建在参考关键帧的投影约束.注意:这里没有,也不应该对'L'[landmark_id]进行任何的prior约束!

                                //这个不行.这个没有引入尺度信息,仅仅靠摄像机间的尺度信息和初始位置估计太不可靠了,必须有约束.
                                //这里应该是只能选是否用smart factor,factor的类型必须是Cal3_S2Stereo,而不能是GenericProjectionFactor<...,...,Cal3_S2>.
                                //method<1> 最普通的投影误差,没有任何技巧.
                                //graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
                                //            measurement, measurementNoise, Symbol('X',reference_kf_id*cam_count + i), Symbol('L', map_point_relavent_landmark_id), this->v_cams_gtsam_config_stereo_left[i]);
                                //method<2>.GenericStereoFactor
                                graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(520,//左目的u
                                                                                                      480,//右目的u
                                                                                                      440),//观测的v.
                                                                                         model,
                                                                                         Symbol('X',reference_kf_id*cam_count + i),
                                                                                         Symbol('L', map_point_relavent_landmark_id),
                                                                                         this->v_cams_gtsam_config_stereo[i]);//创建双目观测约束.
                                //method<4>.SmartFactor of GenericStereoFactor

                                //这种情况下,smartFactor本身就是landmark,无需额外在创建了
                                auto smart_stereo_factor = SmartStereoProjectionPoseFactor::shared_ptr(
                                                                new SmartStereoProjectionPoseFactor(gaussian, params));
                                graph.push_back(smart_stereo_factor);
                                smart_stereo_factor->add(StereoPoint2(xl, xr, y),Symbol('X',reference_kf_id*cam_count + i),this->v_cams_gtsam_config_stereo[i]);
                                landmark_properties lp_;
                                //TODO:填上其他属性.这个lp_如果不用smart factor的话现在看来不一定要用.
                                lp_.pRelativeStereoSmartFactor = smart_stereo_factor;
                                vlandmark_properties.push_back(lp_);
                                landmark_id++;
                            }
                            //2.不管是不是第一次被观测,都要创建在当前帧map_point对应的landmark 与 2d投影点 的gtsam factor约束.
                            //smartFactors.at(map_point_relavent_landmark_id)->add(StereoPoint2(xl, xr, y), X(frame), K);//使用SmartStereo的方法.这里使用Mono(参考ISAM2Example_SmartFactor.cpp).
                            //method <1> 创建最普通的投影误差.
                            if(method == 0)
                            {
                                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
                                      measurement, measurementNoise, Symbol('X',reference_kf_id*cam_count + i), Symbol('L', map_point_relavent_landmark_id), this->v_cams_gtsam_config_stereo_left[i]);
                            }
                            //method <2> 创建smart stereo factor上面的约束.
                            if(method == 1)
                            {
                                smart_factor_ = this->vlandmark_properties[map_point_relavent_landmark_id];
                                smart_factor_.add();//模仿上面的.TODO.这种的缺点是没法再用mono约束这个点了,视野会比较窄...
                                //可能对不同的点需要做不同的方式处理.需要一个判断逻辑.
                            }
                            //TODO:处理其他类型...
                            //input_p2d_to_optimize.push_back(vp2d_pts[p2d_index]);
                            //input_p3d_to_optimize.push_back(vp3d_pts[p2d_index]);
                        }
                    }
                }
                //step<2>.process result.
                LOG(INFO)<<"in OptFlowForFrameWiseTracking stage5: success tracked vp3d_pts.size:"<<pnp_ransac_input_p2d.size()<<",tracked p2d_pts.size:"<<pnp_ransac_input_p2d.size()<<endl;
                LOG(INFO)<<"cam mat rows and cols:"<<camMat.rows<<","<<camMat.cols;
                //solve pnp ransac.

                //bool success = solvePnPRansac(input_p3d_to_optimize,input_p2d_to_optimize,camMat,cv::Mat(),output_r_mat,output_t_vec,false,100,8.0,0.99,match_success,SOLVEPNP_ITERATIVE);

                Matrix3d R_cam;
                Vector3d t_cam;
                cv::Mat rt_mat_cam = pFrame->cam_info_stereo_vec[i].getRTMat();
                cv2eigen(rt_mat_cam.colRange(0,3).rowRange(0,3),R_cam);
                cv2eigen(rt_mat_cam.colRange(3,4).rowRange(0,3),t_cam);

                auto map_points = pKeyFrameReference->fetchMapPoints();
                //generateMapPointsCorespondingToStereoKFObservation(R_cam,t_cam,....);//在关键帧创建时调用这个.//已添加.

                addSingleCamObservationFactor(R_cam,t_cam,input_p2d_to_optimize,map_points);
                //
                //for( each image)
                //{
                //    for(each_2d_3d_track)
                //    {
                //        if(method == ....)//TODO.
                //        {...}
                //        else
                //        {...}
                //        addFactorReprojection()
                //        addSingleCamObservationFactor()
                //    }
                //
                //}
                isam.update(graph, initialEstimate);//优化.
                cv::Mat R_output;
                cv::Rodrigues(output_r_mat,R_output);
                //LOG(INFO)<<"in OptFlowForFrameWiseTracking stage6,output R_mat:\n"<<R_output<<"\n output t vec"<<output_t_vec<<endl;
                output_cam_match_success[i] = success;
                if(success)//一组摄像头成功
                {
                    int pnpransac_success_count = 0;
                    for(int j = 0;j<match_success.size();j++)
                    {
                        if(match_success[j])
                        {
                            pnpransac_success_count+=1;
                            total_success_tracked_point_count+=1; //只计算整组成功以后的成功点个数.失败的摄像头组整组都不算了.
                        }
                    }
                    //LOG(INFO)<<"pnp_ransac_match_success!!matched:"<< pnpransac_success_count<<"/"<<pnp_ransac_input_p2d.size()<<endl;
                    success_count+=1;
                }
                else
                {
                    //LOG(WARNING)<<"Cam "<<i<<" has caught into pnp ransac failure!"<<endl;
                }
            }
            LOG(INFO)<<"in OptFlowForFrameWiseTracking stage7"<<endl;
            //if(success_count>=3)
            //{
            //    output_frame_pnp_ransac_success = true;
            //}
            LOG(INFO)<<"For all cams:total tracked points count:"<<total_success_tracked_point_count;
            if(ptotal_tracked_points_count != nullptr)
            {
                *ptotal_tracked_points_count = total_success_tracked_point_count;
            }
            frame_id++;//记录这是第几个frame.
            return;
/*
 *
 *
 *
 *  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
 *
 * // Define the camera observation noise model, 1 pixel stddev
 * auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);
 *
 * //创建相机pose点.
 *
 *
 *
 *initialEstimate.insert(Symbol('x', i), Pose3(....));
 * graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
 *           measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
 *
 *
            // 1. track referring pKeyFrame to get reprojection error.
            int image_count = pFrame->cam_info_stereo_vec.size();
            vector<std::pair<p2dT,p2dT> > vpoints;
            for(int i = 0;i<image_count;i++)
            {
                OptFlowForFrameWiseTracking(*(pFrame->pLRImgs->at(i).first),*(pKeyFrameReference->pLRImgs->at(i).first),)
                //(pKeyFrameReference)
                addFactorReprojection(...);
            }
            // 2. do optimization and solve current pose.

            // *3. check with pnp ransac.
*/
        }
        void addStereoKeyFrameToBackEndAndOptimize(shared_ptr<Frame> pKeyFrame,int method = METHOD_SIMPLE_MONOCULAR_REPROJECTION_ERROR)
        {
            if(!pKeyFrame->isKeyFrame)
            {
                LOG(ERROR)<<"ERROR:In insertKeyFrameToBackEnd: frame is not keyframe!"<<endl;
                return;
            }
            this->addOrdinaryStereoFrameToBackendAndOptimize(pKeyFrame,getLastKF(),method);//先像处理普通帧一样,处理跟踪和定位问题.
            //从这里开始,这一帧的位置已经被初步优化.
            pKeyFrame->map_points = generateMapPointsCorespondingToStereoKFObservation();//创建新地图点.

            //add vertex for frame.
            //add IDP Error in Frame.
            //convert pKeyFrame->p3d_vv to MapPoints;
            //TODO:是否先尝试一下pnpransac,再进行新关键帧创建?
            //mcs::trackAndDoSolvePnPRansacMultiCam(...)
            //对每组摄像头:
            int cam_count = pKeyFrame->map2d_to_3d_pt_vec.size();
            for(int cam_index = 0;cam_index<cam_count;cam_index++)
            {
                //1.create backend MapPoint node.
                //vector<shared_ptr<MapPoint> >& vMPs = pKeyFrame->map_points[cam_index];
                //for(auto &pMP :vMPs)
                //{
                //    gtsam::Point3();
                //    pMP->pos
                //}
                vector<p2dT>& vP2ds = pKeyFrame->p2d_vv[cam_index];
                for(p2dT& p2d_:vP2ds)
                {
                    //add relative perspective factor
                    if(method == METHOD_SIMPLE_MONOCULAR_REPROJECTION_ERROR) //....处理对应方法...TODO.
                    {
                        if (smartFactors.count(landmark_id) == 0) {//只能对已经观测一次,这里又观测一次的点做优化.只有一次观测会引发异常.
                          auto gaussian_reprojection_error = noiseModel::Isotropic::Sigma(3, 1.0);

                          SmartProjectionParams params(HESSIAN, ZERO_ON_DEGENERACY);

                          smartFactors[landmark_id] = SmartStereoProjectionPoseFactor::shared_ptr(
                              new SmartStereoProjectionPoseFactor(gaussian_reprojection_error, params));
                          graph.push_back(smartFactors[landmark_id]);
                        }
                        smartFactors[landmark_id]->add(StereoPoint2(xl, xr, y), X(frame), K);
                    }
                    else
                    {
                        auto gaussian_reprojection_error = noiseModel::Isotropic::Sigma(3, 1.0);
                        noiseModel::Robust::shared_ptr velocityNoiseModel = noiseModel::Robust::Create(
                                                                            noiseModel::mEstimator::Huber::Create(1.345), gaussian);
                        initialEstimate.insert(Symbol('l', xxx), Pose3(....));
                        graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
                                  measurement, velocityNoiseModel, Symbol('x', i), Symbol('l', xxx), K);
                            }
                    }

                }

            }
            /*挪到addOrdinaryStereoFrameToBackendAndOptimize()里面去.关键帧不要做这个操作 .
             * //2.add reprojection factor between this frame and last frame.
            shared_ptr<Frame> pLastFrame = this->getLastKF();
            int image_count = pFrame->cam_info_stereo_vec.size();
            for(int i = 0;i<image_count;i++)
            {
                OptFlowForFrameWiseTracking();
                addFactorReprojection();
            }
            //...
            */
            isam.update(graph, initialEstimate);
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
