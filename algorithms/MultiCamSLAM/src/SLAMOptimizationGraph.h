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
    class SLAMOptimizationGraph
    {
    private:
        ISAM2Params parameters;
        //parameters.relinearizeThreshold = 0.1;
        ISAM2 isam;//(parameters);
        NonlinearFactorGraph graph;
        Values initialEstimate;



    public:
        SLAMOptimizationGraph(cv::FileStorage fSettings)
        {
            //...
        }

        void initCams();
        shared_ptr<Frame> getLastKF();
        void addSingleCamObservationFactor(Matrix3d camR,Vector3d camt,vector<p2dT> observation2d,vector<shared_ptr<MapPoint> > coresponding_map_pts);
        void generateMapPointsCorespondingToStereoKFObservation(Matrix3d camR,Vector3d camt,
                                                                vector<p3dT> observation3d,vector<p2dT> observation2d,
                                                                vector<shared_ptr<MapPoint> > &output_map_points);//根据关键帧观测的p2d,p3d生成对应的Map.

        void addOrdinaryStereoFrameToBackendAndOptimize(shared_ptr<Frame> pFrame,shared_ptr<Frame> pKeyFrameReference,bool useSmartFactor = false)
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
            auto pts_vv = pFrame->p2d_vv;
            //pts_vv = OptFlowForFrameWiseTracking(*pframe2);//track frame2 获取对应位置的点...
            LOG(INFO)<<"in OptFlowForFrameWiseTracking stage4"<<endl;
            for(int i = 0;i < imgs_prev.size();i++)
            {
                LOG(INFO)<<"Ref keyframe p3ds_vv.size():"<<p3ds_vv.size()<<";new frame p2ds_vv.size():"<<pts_vv.size()<<endl;

                LOG(INFO)<<"in OptFlowForFrameWiseTracking stage4.1"<<endl;
                cv::Mat camMat;
                if(isStereoMode)
                {
                    camMat = f2.get_stereo_cam_info()[i].getCamMat();
                }
                else
                {
                    camMat = f2.get_cam_info()[i].getCamMat();
                }
                LOG(INFO)<<"in OptFlowForFrameWiseTracking stage4.2"<<endl;
                output_r_mat = cv::Mat();
                output_t_vec = cv::Mat();
                vector<unsigned char> match_success;
                //TODO:multi_thread implementation.
                vector<p3dT> input_p3d_to_optimize;
                vector<p2dT> input_p2d_to_optimize;
                {//step<1>.track 2d pts.
                    vector<p3dT> vp3d_pts = p3ds_vv[i];
                    vector<p2dT> vp2d_pts;
                    for(int index_p3d=0;index_p3d<vp3d_pts.size();index_p3d++)
                    {
                        vp2d_pts.push_back(f1.p2d_vv[i][f1.map3d_to_2d_pt_vec[i][index_p3d] ]);
                    }
                    vector<cv::KeyPoint> toTrack_2dpts;
                    cv::KeyPoint::convert(vp2d_pts,toTrack_2dpts);
                    vector<Point2f> original_remaining_2dpts,tracked_nextimg_2dpts;
                    vector<unsigned char> v_pt_track_success;
                    bool output_track_success;
                    map<int,int> tracked_kps_to_original_kps_map_output;
                    //step <1>. do opt flow.
                    OptFlowForFrameWiseTracking(*(imgs_prev[i]),*(imgs_next[i]),toTrack_2dpts,original_remaining_2dpts,tracked_nextimg_2dpts,tracked_kps_to_original_kps_map_output,v_pt_track_success,output_track_success);


                    for(int index_p3d = 0;index_p3d<vp3d_pts.size();index_p3d++)
                    {
                        int p2d_index = tracked_kps_to_original_kps_map_output[index_p3d];
                        if(p2d_index>=0)//a match has been found.
                        {
                            input_p2d_to_optimize.push_back(vp2d_pts[p2d_index]);
                            input_p3d_to_optimize.push_back(vp3d_pts[p2d_index]);
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
                //        if(useSmartFactor)
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
        void addStereoKeyFrameToBackEndAndOptimize(shared_ptr<Frame> pKeyFrame,bool useSmartFactor = false)
        {
            if(!pKeyFrame->isKeyFrame)
            {
                LOG(ERROR)<<"ERROR:In insertKeyFrameToBackEnd: frame is not keyframe!"<<endl;
                return;
            }
            this->addOrdinaryStereoFrameToBackendAndOptimize(pKeyFrame,getLastKF(),useSmartFactor);//先像处理普通帧一样,处理跟踪和定位问题.
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
                    if(useSmartFactor)
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
    private:
        cv::FileStorage* pfSettings;
        vector<StereoCamConfig> stereo_config;
        vector<CamInfo> rgbd_config;
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
