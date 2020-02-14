#ifndef REPROJECTION_INFO_DATABASE
#define REPROJECTION_INFO_DATABASE
#include <deque>
#include <memory>
#include <mutex>
#include "Frame.h"
#include "FrameWiseGeometry.h"
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>

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

#include <glog/logging.h>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d.hpp"


#include "FrameManager.h"

#include <thread>
#include <opencv2/core/eigen.hpp>
#include "IMU_Preint_GTSAM.h"
#include <iostream>
#include "Visualization.h"
#include<algorithm>
#include "utils/mcs_utils.h"

namespace mcs {
using namespace gtsam;

class FrameTable;
class LandmarkTable;
class RelationTableT;

struct LandmarkProperties;
struct RelationT;

//struct ObservationInfo
//{
//    //weak_ptr<Frame> pObservedByFrame;
//    int observed_by_frame_id;
//    int relative_p2d_index;
//    int relative_p3d_index;
//};


struct LandmarkProperties
{
    //mutex propertiesMutex;
    mutex writeMutex;
    Point3 estimatedPosition;
    bool triangulated = false;
    void setTriangulated()
    {
        this->triangulated = true;
    }
    inline bool everTriangulated()
    {
        return this->triangulated;
    }
    bool position_ever_init = false;
    Point3 getEstimatedPosition(bool& valid)
    {
        valid = position_ever_init;
        if(!valid)//DEBUG.
        {
            //throw "trying to get an invalid pose estimation!";
            ;//DEBUG ONLY.
        }
        return estimatedPosition;
    }
    void setEstimatedPosition(Point3 pose)
    {
        this->estimatedPosition = pose;
        LOG(INFO)<<"landmark_id:"<<this->landmark_id<<" estimated!"<<endl;
        position_ever_init = true;
    }
    int landmark_reference_time = 0;
    int cam_id;
    //weak_ptr<Frame> pCreatedByFrame;
    int created_by_kf_id;
    int relative_kf_p2d_id;//在created_by_kf_id这个帧里对应的p2d id.
    int landmark_id;
    //weak_ptr<Frame> pLastObservedByFrame;
    //boost::shared_ptr<SmartStereoProjectionPoseFactor> pRelativeStereoSmartFactor;//目前还没实现处理无穷远.//不用这个垃圾东西了.
    //std::vector<ObservationInfo> vObservationInfo;//这个和relationT选一个?
    std::set<int> ObservedByFrameIDSet;
};
struct RelationT//定义一次追踪关系.
{
    int cam_id;
    int landmarkID;
    int referedByKFID;//当前跟踪关系 被跟踪的那个关键帧.
    int trackedByFrameID;//这个不是列表,这个直接就是第二次被追踪的帧id;多次追踪就创建多个RelationShip.
    char trackType; //跟踪关系类型.
};
class FrameTable
{
private:
    mutex writeMutex;
    //存储索引关系用于快查.
public:
    int currentID = 0;
    map<int,shared_ptr<Frame> > dataTable;
    void insertFrame(shared_ptr<Frame> pFrame);
    inline shared_ptr<Frame> query(int id);
    //vector<shared_ptr<Frame> > queryByRefKFID(int refKFID);//查询函数都按这个模型写.
    void marginalizeKFDeleteEverythingRecursive(int KFID);
//    void withdrawAFrame()
//    {
//        this->currentID--;
//    }
};
void FrameTable::insertFrame(shared_ptr<Frame> pFrame)
{
    this->dataTable[currentID] = pFrame;
    pFrame->frame_id = currentID;
    currentID++;
}
shared_ptr<Frame> FrameTable::query(int id)
{
    if(this->dataTable.count(id) ==0)
    {
        LOG(WARNING)<<"Access Violation in FrameTable.query()!"<<endl;
        return nullptr;
    }
    return this->dataTable.at(id);
}
//vector<int> FrameTable::queryByRefKFID(int refKFID)
//{//查询所有参考这个帧的id


//}

class LandmarkTable
{
private:
    mutex writeMutex;
    map<int,
        map<int,
            map<int,int>
        >
    > kfid_camid_p2did_to_landmark_id_mapping;
public:
    int currentID = 0;
    map<int,shared_ptr<LandmarkProperties> > dataTable;
    void insertLandmark(shared_ptr<LandmarkProperties> pLandmark);
    inline shared_ptr<LandmarkProperties> query(int id)
    {
        return this->dataTable.at(id);
    }
    vector<shared_ptr<LandmarkProperties> > queryByCreatedByKF(int createdByKFID);
    shared_ptr<LandmarkProperties> queryByKF_camID_p2dID(int kfid,int cam_id,int p2d_id)
    {
        if(this->kfid_camid_p2did_to_landmark_id_mapping.count(kfid) == 0 ||
                this->kfid_camid_p2did_to_landmark_id_mapping.at(kfid).count(cam_id) == 0||
                this->kfid_camid_p2did_to_landmark_id_mapping.at(kfid).at(cam_id).count(p2d_id) == 0)
        {
            LOG(ERROR)<<"Access Violation in LandmarkTable::queryByKF_camID_p2dID!"<<endl;
            return nullptr;
        }
        return this->dataTable.at( this->kfid_camid_p2did_to_landmark_id_mapping.at(kfid).at(cam_id).at(p2d_id));
    }
};
void LandmarkTable::insertLandmark(shared_ptr<LandmarkProperties> pLandmark)
{
    //插入.
    this->dataTable.insert(std::make_pair(currentID,pLandmark)); //.at(this->currentID) = pLandmark;
    pLandmark->landmark_id = currentID;
    //加索引.
    if(this->kfid_camid_p2did_to_landmark_id_mapping.count(pLandmark->created_by_kf_id)  == 0)//不存在,创建:
    {
        this->kfid_camid_p2did_to_landmark_id_mapping.insert(std::make_pair(pLandmark->created_by_kf_id,map<int ,map<int,int> >()));//.at(pLandmark->created_by_kf_id) = map<int ,map<int,int> >();
    }
    if(this->kfid_camid_p2did_to_landmark_id_mapping.at(pLandmark->created_by_kf_id).count(pLandmark->cam_id) == 0)
    {
        this->kfid_camid_p2did_to_landmark_id_mapping.at(pLandmark->created_by_kf_id).insert(std::make_pair(pLandmark->cam_id,map<int,int>() ) );//.at(pLandmark->cam_id) = map<int,int>();
    }
    this->kfid_camid_p2did_to_landmark_id_mapping.at(pLandmark->created_by_kf_id).at(pLandmark->cam_id).insert(make_pair(pLandmark->relative_kf_p2d_id, this->currentID));
    this->currentID++;
}
class RelationTableT
{
private:
    mutex writeMutex;
public:
    int currentID = 0;
    map<int,shared_ptr<RelationT> > dataTable;
    void insertRelation(shared_ptr<RelationT> pReprojRelation);
    inline shared_ptr<RelationT> query(int id);
    vector<shared_ptr<RelationT> > queryByCreatedByKF(int KFID);
    vector<shared_ptr<RelationT> > queryRelativeWithFrameID(int frameID);
};



class ReprojectionInfoDatabase{
private:
    mutex writeMutex;
public:
    //TODO:考虑table里只存储索引.
    FrameTable frameTable;
    LandmarkTable landmarkTable;
    RelationTableT relationTable;


    vector<shared_ptr<RelationT> > queryAllRelatedRelationsByKF(shared_ptr<Frame> pKF);//查询所有相关的点.用于创建投影关系和marginalize;
//    void withdrawAFrame()
//    {
//        this->frameTable.withdrawAFrame();
//    }
    void insertFramePoseEstimation(int frameID,shared_ptr<NonlinearFactorGraph> pGraph,shared_ptr<Values> pInitialEstimate)
    {//插入对应的frame Fi, Cam Xi,并创建对应的约束关系.
        noiseModel::Diagonal::shared_ptr noise_model_between_cams = gtsam::noiseModel::Diagonal::Variances (
                    ( gtsam::Vector ( 6 ) <<0.00001, 0.00001, 0.00001, 0.0001, 0.0001, 0.0001 ).finished()); //1mm,0.1度.//放松一些
        if(frameID > 0)//初始化完成后的帧
        {
            Rot3 rot;Point3 trans;
            bool valid;

            auto pCurrentFrame = this->frameTable.query(frameID);
            pCurrentFrame->getRotationAndTranslation(rot,trans,valid);
            const int cam_count =pCurrentFrame->get_cam_num();

            auto xi_array = pCurrentFrame->getXiArray();
            if(valid)
            {//当前帧曾经优化过.那就直接填写老值.
                auto Fi_XiArray = pCurrentFrame->getFiAndXiArray();
                LOG(WARNING)<<"Insert F"<<frameID<<endl;
                pInitialEstimate->insert(Symbol('F',frameID),Fi_XiArray.first);
                for(int i = 0;i<cam_count;i++)
                {
                    //
                    pInitialEstimate->insert(Symbol('X',frameID*cam_count + i),
                                             Pose3(
                                                 //Fi_XiArray.second.at(i).matrix().inverse()*
                                                 //pInitialEstimate->at(Symbol('F',frameID)).cast<Pose3>().matrix()

                                                 //  pInitialEstimate->at(Symbol('F',frameID)).cast<Pose3>().matrix().inverse()
                                                   //pInitialEstimate->at(Symbol('F',frameID)).cast<Pose3>().matrix()

                                                   pInitialEstimate->at(Symbol('F',frameID)).cast<Pose3>().matrix()*
                                                   //Fi_XiArray.second.at(i).matrix().inverse()
                                                   Fi_XiArray.second.at(i).matrix()


                                                      )
                                                );
                    //Pose3 relative_pose = Pose3(xi_array.at(i).matrix().inverse());//TODO.
                    Pose3 relative_pose = Pose3(xi_array.at(i).matrix());//TODO.
                    pGraph->emplace_shared<BetweenFactor<Pose3> >(Symbol('F',frameID),Symbol('X',frameID*cam_count + i),
                                                                  relative_pose,
                                                                  noise_model_between_cams
                                                                  );//加入约束关系.
                    LOG(WARNING)<<"X"<<frameID*cam_count + i<<"already constrained!"<<endl;
                }
            }
            else
            {//新插入的帧,没有位置估计.
                this->frameTable.query(frameID-1)->getRotationAndTranslation(rot,trans,valid);
                if(valid)
                {
                    auto Fi_XiArray = this->frameTable.query(frameID -1)->getFiAndXiArray();//查上一个老帧.
                    LOG(WARNING)<<"Insert F"<<frameID<<endl;
                    pInitialEstimate->insert(Symbol('F',frameID),Fi_XiArray.first);
                    for(int i = 0;i<cam_count;i++)
                    {//对每个摄像头,创建约束关系.
                        int x_index = frameID*cam_count + i;
//                        pInitialEstimate->insert(Symbol('X',x_index),
//                                                 Pose3(Fi_XiArray.second.at(i).matrix().inverse()*
//                                                                        pInitialEstimate->at(Symbol('F',frameID)).cast<Pose3>().matrix()
//                                                       )//两次变换.
//                                                 );
                        pInitialEstimate->insert(Symbol('X',x_index),
                                                 Pose3(
                                                      pInitialEstimate->at(Symbol('F',frameID)).cast<Pose3>().matrix()*
                                                      //Fi_XiArray.second.at(i).matrix().inverse()
                                                     Fi_XiArray.second.at(i).matrix()
                                                       )//两次变换.
                                                 );


                        Matrix3d Rot;
                        cv::cv2eigen(pCurrentFrame->cam_info_stereo_vec.at(i).get_RMat(),Rot);
                        float tx,ty,tz;
                        pCurrentFrame->cam_info_stereo_vec.at(i).get_tMat(tx,ty,tz);
                        Vector3d translation(tx,ty,tz);
                        //Pose3 relative_pose = xi_array.at(i);
                        //Pose3 relative_pose = Pose3(xi_array.at(i).matrix().inverse());

                        Pose3 relative_pose = Pose3(xi_array.at(i).matrix());
                        pGraph->emplace_shared<BetweenFactor<Pose3> >(Symbol('F',frameID),Symbol('X',x_index),
                                                                      relative_pose,
                                                                      noise_model_between_cams
                                                                      );//加入约束关系.
                        LOG(WARNING)<<"X"<<x_index<<"already constrained!"<<endl;
                    }
                }
                else
                {//这个应该是出错了才会有.
                    LOG(ERROR)<<"last frame estimation invalid!!"<<endl;
                    throw "frame pose estimation invalid!";
                    exit(-1);
                }
            }
        }
        else// frameID = 0,固定第一帧的位置.
        {
            auto pCurrentFrame = this->frameTable.query(frameID);
            pCurrentFrame->setRotationAndTranslation(Rot3().matrix(),Vector3d());

            Rot3 rot;Point3 trans;
            bool valid;
            pCurrentFrame->getRotationAndTranslation(rot,trans,valid);
            const int cam_count =pCurrentFrame->get_cam_num();
//            pInitialEstimate->insert();
            //auto Fi_XiArray = pCurrentFrame->getFiAndXiArray();
            auto xi_array = pCurrentFrame->getXiArray();
            LOG(WARNING)<<"Insert F"<<frameID<<endl;
            pInitialEstimate->insert(Symbol('F',frameID),Pose3());
            for(int i = 0;i<cam_count;i++)
            {
                //
                pInitialEstimate->insert(Symbol('X',frameID*cam_count + i),
                                            Pose3(
                                             //xi_array.at(i).matrix().inverse()*
                                             xi_array.at(i).matrix()*
                                               pInitialEstimate->at(Symbol('F',frameID)).cast<Pose3>().matrix())
                                            );
                //Pose3 relative_pose = xi_array.at(i);//TODO.
                //Pose3 relative_pose = Pose3(xi_array.at(i).matrix().inverse());

                Pose3 relative_pose = Pose3(xi_array.at(i).matrix());
                pGraph->emplace_shared<BetweenFactor<Pose3> >(Symbol('F',0),Symbol('X',i),
                                                              relative_pose,
                                                              noise_model_between_cams
                                                              );//加入约束关系.
                LOG(WARNING)<<"X"<<i<<"already constrained!"<<endl;
            }
        }
    }

    shared_ptr<NonlinearFactorGraph> generateLocalGraphByFrameID(int frameID, shared_ptr<Values>& pInitialEstimate_output)//,bool& local_graph_track_valid)
    {//创建"局部"优化图.只参考上一关键帧,优化本帧位置.

        LOG(WARNING)<<"Generating local graph for frame:"<<frameID<<endl;
        shared_ptr<NonlinearFactorGraph> pGraph = shared_ptr<NonlinearFactorGraph>(new NonlinearFactorGraph());
        pInitialEstimate_output = shared_ptr<Values>(new Values());
        auto gaussian__ = noiseModel::Isotropic::Sigma(3, 1.0);
        //auto robust_kernel = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(15), gaussian__); //robust
        //auto robust_kernel = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian__); //robust
        gtsam::SharedNoiseModel robust_kernel = gtsam::noiseModel::Robust::Create(
                                                    //gtsam::noiseModel::mEstimator::Huber::Create(
                                                    //1.345,gtsam::noiseModel::mEstimator::Huber::Scalar),  // Default is
                                                    gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                                    gaussian__);
        auto gaussian__2d = noiseModel::Isotropic::Sigma(2, 1.0);
        gtsam::SharedNoiseModel robust_kernel_2d = gtsam::noiseModel::Robust::Create(
                                                    //gtsam::noiseModel::mEstimator::Huber::Create(
                                                    //1.345,gtsam::noiseModel::mEstimator::Huber::Scalar),  // Default is
                                                    gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                                    gaussian__2d);
        Cal3_S2Stereo::shared_ptr K_stereo(new Cal3_S2Stereo(1000, 1000, 0, 320, 240, 0.2));//TODO edit this.
        Cal3_S2::shared_ptr K_mono(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

        shared_ptr<Frame> pFrame = this->frameTable.query(frameID);
        //获取跟踪的点.

        //创建Fframe,Xframe,Xframe+i.建立约束关系.
        LOG(WARNING)<<"Insert pose estimation of frame:"<<frameID<<endl;
        this->insertFramePoseEstimation(frameID,pGraph,pInitialEstimate_output);
        if(frameID == 0)
        {
            LOG(WARNING)<<"Initialized frame X0."<<endl;
            return pGraph;//对第一帧 只设定约束即可.
        }

        //shared_ptr<Frame> pRefKF = this->frameTable.query(pRefKF->frame_id);//TODO:改成选择最好的那个.


        int ref_kf_best_id;double best_score;
        bool best_kf_valid = pFrame->track_states.getBestRefKFid(ref_kf_best_id,best_score);
        if(best_kf_valid == false )//获取的最优id无效.
        {
            //local_graph_track_valid = false;//无法生成优化图.
            LOG(ERROR)<<"For frame F"<<frameID<<" all track invalid!"<<endl;
            //return;//DEBUG ONLY.
        }
        LOG(WARNING)<<"Insert pose estimation of referring frame:"<<ref_kf_best_id<<",score:"<<best_score<<endl;
        shared_ptr<Frame> pRefKF = this->frameTable.query(ref_kf_best_id);


        this->insertFramePoseEstimation(ref_kf_best_id//pRefKF->frame_id
                                        ,pGraph,pInitialEstimate_output);
        //对refkf进行set fixed.
        //pGraph->emplace_shared<NonlinearEquality<Pose3> >(Symbol('F',pRefKF->frame_id),pInitialEstimate_output->at(Symbol('F',pRefKF->frame_id)).cast<Pose3>());
        LOG(WARNING)<<"Constrained frame: F"<<pRefKF->frame_id<<endl;
        noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-7, 1e-7, 1e-7).finished());
        pGraph->add(PriorFactor<Pose3>(Symbol('F',pRefKF->frame_id),pInitialEstimate_output->at(Symbol('F',pRefKF->frame_id)).cast<Pose3>(),priorModel));//加入约束.

        const int cam_count = pFrame->get_cam_num();

        //创建Xref,Xref+i.
        //
        //  step<1>.查询对应的Frame.//暂时只用getLastKFID();将来可以把所有track过的都加入进去.
        //  step<2>.生成对应的Symbol.
//        this->insertFramePoseEstimation(pRefKF->frame_id,pGraph,pInitialEstimate_output);
//        if(frameID == 0)
//        {
//            return pGraph;
//        }
        for(int i = 0;i<cam_count;i++)
        {
            float camfx,camfy,camcx,camcy;
            pFrame->cam_info_stereo_vec.at(i).getCamMatFxFyCxCy(camfx,camfy,camcx,camcy);
            auto p2d_vec = pFrame->reproj_map.at(pRefKF->frame_id).at(i);//仅跟踪getLastKFID()这一关键帧.
            //for(auto &tracked_pt:p2d_vec)
            for(int reprojected_p2d_index = 0;reprojected_p2d_index < p2d_vec.size();reprojected_p2d_index++)
            {
                auto & tracked_pt = p2d_vec.at(reprojected_p2d_index);
                int originalKFp2dID = tracked_pt.ref_p2d_id; //判断对应投影类型,创建对应投影关系约束.
                cv::Point2f& p2d_ = tracked_pt.current_frame_p2d;
                //查找对应disp.//disp直接存在里面.
                float disp = tracked_pt.disp;
                char tracking_state = tracked_pt.tracking_state;

                int relative_landmark_id = pRefKF->getLandmarkIDByCamIndexAndp2dIndex(i,originalKFp2dID);//查询对应的landmark.如果没有就创建.
                if(relative_landmark_id<0)//Landmark invalid.
                {//create a landmark;
                    LOG(ERROR)<<"in generateLocalGraphByFrameID(): landmark id index access violation!"<<endl;
                    throw "access violation.";
                    exit(-1);
                }
                auto pLandmark = this->landmarkTable.query(relative_landmark_id);
                if(tracking_state == TRACK_STEREO2MONO)
                {
                    //创建landmark并加入初始估计.
                    if(pInitialEstimate_output->find(Symbol('L',relative_landmark_id)) == pInitialEstimate_output->end())
                    {//没有initial Estimate.
//                        cv::Point3f p3d_relative = pRefKF->p3d_vv.at(i).at(pRefKF->map2d_to_3d_pt_vec.at(i).at(originalKFp2dID));
//                        Eigen::Vector4d v4d(p3d_relative.x,p3d_relative.y,p3d_relative.z,1);
                        auto v4d = triangulatePoint(pRefKF->disps_vv.at(i).at(originalKFp2dID),pRefKF->p2d_vv.at(i).at(originalKFp2dID).x,
                                                    pRefKF->p2d_vv.at(i).at(originalKFp2dID).y,
                                                    0.12,//DEBUG ONLY!b=0.12
                                         camfx,camfy,camcx,camcy
                                         );
                        //v4d = pInitialEstimate_output->at(Symbol('X',pRefKF->frame_id*cam_count + i)).cast<Pose3>().matrix().inverse() * v4d;
                        v4d = pInitialEstimate_output->at(Symbol('X',pRefKF->frame_id*cam_count + i)).cast<Pose3>().matrix() * v4d;//DEBUG.
                        LOG(INFO)<<"Landmark L"<<relative_landmark_id<<" estimated at "<<v4d[0]<<","<<v4d[1]<<","<<v4d[2]<<";STEREO_TO_MONO."<<endl;
                        pInitialEstimate_output->insert(Symbol('L',relative_landmark_id),Point3(v4d[0],v4d[1],v4d[2]
                                                            ));//创建initial estimate,根据对应的那一次双目观测.
                        pLandmark->setEstimatedPosition(Point3(v4d[0],v4d[1],v4d[2]));
                    }
                    //加入关键帧位置的观测.
                    auto kf_p2d = pRefKF->p2d_vv.at(i).at(tracked_pt.ref_p2d_id);
                    pGraph->emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(kf_p2d.x,kf_p2d.x-pRefKF->disps_vv.at(i).at(tracked_pt.ref_p2d_id),kf_p2d.y
                                                                                            ),robust_kernel,
                                Symbol('X',pRefKF->frame_id*cam_count + i ),Symbol('L',relative_landmark_id),K_stereo);
                    //加入当前帧的观测.
                    pGraph->emplace_shared<GenericProjectionFactor<Pose3,Point3,Cal3_S2> > (Point2(tracked_pt.current_frame_p2d.x,tracked_pt.current_frame_p2d.y),robust_kernel_2d,
                                Symbol('X',frameID*cam_count+i),Symbol('L',relative_landmark_id),K_mono);
                }
                else if(tracking_state == TRACK_MONO2STEREO)
                {
                    if(pInitialEstimate_output->find(Symbol('L',relative_landmark_id)) == pInitialEstimate_output->end())
                    {//没有initial Estimate.
                        //这种要根据当前帧做计算.
                        auto v4d = triangulatePoint(tracked_pt.disp,tracked_pt.current_frame_p2d.x,tracked_pt.current_frame_p2d.y,
                                                    0.12,//DEBUG ONLY.
                                                    camfx,camfy,camcx,camcy);
                        //v4d = pInitialEstimate_output->at(Symbol('X',pFrame->frame_id*cam_count + i)).cast<Pose3>().matrix().inverse() * v4d;
                        v4d = pInitialEstimate_output->at(Symbol('X',pFrame->frame_id*cam_count + i)).cast<Pose3>().matrix() * v4d;
                        LOG(INFO)<<"Landmark L"<<relative_landmark_id<<" estimated at "<<v4d[0]<<","<<v4d[1]<<","<<v4d[2]<<";MONO_TO_STEREO."<<endl;
                        pInitialEstimate_output->insert(Symbol('L',relative_landmark_id),Point3(v4d[0],v4d[1],v4d[2]
                                                                ));//创建initial estimate,根据对应的那一次双目观测.
                        pLandmark->setEstimatedPosition(Point3(v4d[0],v4d[1],v4d[2]));
                    }
                    //加入关键帧位置的观测.
                    auto kf_p2d = pRefKF->p2d_vv.at(i).at(tracked_pt.ref_p2d_id);
                    pGraph->emplace_shared<GenericProjectionFactor<Pose3,Point3,Cal3_S2> >(Point2(kf_p2d.x,kf_p2d.y),robust_kernel_2d,
                        Symbol('X',pRefKF->frame_id*cam_count + i ),Symbol('L',relative_landmark_id),K_mono);
                    //加入当前帧的观测.
                    pGraph->emplace_shared<GenericStereoFactor<Pose3,Point3> > (StereoPoint2(tracked_pt.current_frame_p2d.x,
                        tracked_pt.current_frame_p2d.x - tracked_pt.disp,tracked_pt.current_frame_p2d.y),robust_kernel,
                        Symbol('X',frameID*cam_count+i),Symbol('L',relative_landmark_id),K_stereo);
                }
                else if(tracking_state == TRACK_STEREO2STEREO)
                {
                    if(pInitialEstimate_output->find(Symbol('L',relative_landmark_id)) == pInitialEstimate_output->end())
                    {//没有initial Estimate.
//                        cv::Point3f p3d_relative = pRefKF->p3d_vv.at(i).at(pRefKF->map2d_to_3d_pt_vec.at(i).at(originalKFp2dID));
//                        Eigen::Vector4d v4d(p3d_relative.x,p3d_relative.y,p3d_relative.z,1);
                        auto v4d = triangulatePoint(pRefKF->disps_vv.at(i).at(originalKFp2dID),pRefKF->p2d_vv.at(i).at(originalKFp2dID).x,
                                                    pRefKF->p2d_vv.at(i).at(originalKFp2dID).y,
                                                    0.12,//DEBUG ONLY!b=0.12
                                         camfx,camfy,camcx,camcy
                                         );
                        //v4d = pInitialEstimate_output->at(Symbol('X',pRefKF->frame_id*cam_count + i)).cast<Pose3>().matrix().inverse() * v4d;
                        v4d = pInitialEstimate_output->at(Symbol('X',pRefKF->frame_id*cam_count + i)).cast<Pose3>().matrix() * v4d;
                        LOG(INFO)<<"Landmark L"<<relative_landmark_id<<" estimated at "<<v4d[0]<<","<<v4d[1]<<","<<v4d[2]<<";STEREO_TO_STEREO."<<endl;
                        pInitialEstimate_output->insert(Symbol('L',relative_landmark_id),Point3(v4d[0],v4d[1],v4d[2]
                                                            ));//创建initial estimate,根据对应的那一次双目观测.和stereo2mono相同.
                        pLandmark->setEstimatedPosition(Point3(v4d[0],v4d[1],v4d[2]));
                    }

                    //加入关键帧位置的观测.
                    auto kf_p2d = pRefKF->p2d_vv.at(i).at(tracked_pt.ref_p2d_id);
                    pGraph->emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(kf_p2d.x,kf_p2d.x-pRefKF->disps_vv.at(i).at(tracked_pt.ref_p2d_id),kf_p2d.y
                                                                                            ),robust_kernel,
                                Symbol('X',pRefKF->frame_id*cam_count + i ),Symbol('L',relative_landmark_id),K_stereo);
                    //加入当前帧的观测.
                    pGraph->emplace_shared<GenericStereoFactor<Pose3,Point3> > (StereoPoint2(tracked_pt.current_frame_p2d.x,
                        tracked_pt.current_frame_p2d.x - tracked_pt.disp,tracked_pt.current_frame_p2d.y),robust_kernel,
                        Symbol('X',frameID*cam_count+i),Symbol('L',relative_landmark_id),K_stereo);

                }//MONO2MONO暂时不处理.
                //创建投影关系.
                /*类似方法.双目用这个.
                pGraph->emplace_shared<GenericStereoFactor<Pose3,Point3> >(
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
                                                                );*/
                //单目用这个.
                /*
                Point2 measurement = camera.project(points[j]);
                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
                */
            }
        }
        return pGraph;
    }
    shared_ptr<NonlinearFactorGraph> generateCurrentGraphByKFIDVector(vector<int> kf_ids,shared_ptr<Values>&pInitialEstimate_output)//查数据库,仅根据当前kfid的列表,生成一个优化图.默认id是排序后的,第一个id被固定.
    {

        set<int> relativeOrdinaryFrameIDs;
        for(auto& id:kf_ids)
        {
            relativeOrdinaryFrameIDs.insert(id);
        }
        /*
        //排序 逐个查投影关系.
        for(auto iter = relativeOrdinaryFrameIDs.begin();iter!=relativeOrdinaryFrameIDs.end();++iter)
        {//对每个帧(id从小到大).

        }*/

        //Pose加入优化图.
        shared_ptr<Frame> pLastKF = this->frameTable.query(kf_ids.back());
        const int cam_count = pLastKF->get_cam_num();
        shared_ptr<NonlinearFactorGraph> pGraph = std::make_shared<NonlinearFactorGraph>();
        pInitialEstimate_output = std::make_shared<Values>();

        auto gaussian__ = noiseModel::Isotropic::Sigma(3, 1.0);
        gtsam::SharedNoiseModel robust_kernel = gtsam::noiseModel::Robust::Create(
                                                    //gtsam::noiseModel::mEstimator::Huber::Create(
                                                    //1.345,gtsam::noiseModel::mEstimator::Huber::Scalar),  // Default is
                                                    gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                                    gaussian__);

        auto gaussian__2d = noiseModel::Isotropic::Sigma(2, 1.0);
        gtsam::SharedNoiseModel robust_kernel_2d = gtsam::noiseModel::Robust::Create(
                                                    //gtsam::noiseModel::mEstimator::Huber::Create(
                                                    //1.345,gtsam::noiseModel::mEstimator::Huber::Scalar),  // Default is
                                                    gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                                    gaussian__2d);
        Cal3_S2Stereo::shared_ptr K_stereo(new Cal3_S2Stereo(1000, 1000, 0, 320, 240, 0.2));//TODO edit this.
        Cal3_S2::shared_ptr K_mono(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

        set<int> landmark_index_set;//避免重复创建.
        //先加X.
        for(int kf_index = 0;kf_index<kf_ids.size();kf_index++)
        {
            int kfid = kf_ids.at(kf_index);
            auto pCurrent = this->frameTable.query(kfid);
            //Pose3 frame_pose;
//            Matrix3d rotation_frame_estimated;
//            Vector3d translation_frame_estimated;
//            bool pose_estimation_valid;
//            pCurrent->getRotationAndTranslation(rotation_frame_estimated,translation_frame_estimated,pose_estimation_valid);
//            if(!pose_estimation_valid)
//            {
//                LOG(ERROR)<<"Pose estimation invalid!!!Check Optimization procedure of this kf!KF_id:"<<pCurrent->frame_id<<endl;
//                exit(-1);
//            }
//            for(int i = 0;i<cam_count;i++)
//            {
//                //获取帧位姿估计.
//                Pose3 estimated_cam_pose;
//                //构造一个Pose3,并对其在estimated pose基础上根据camRT进行变换.
//                //TODO:
//                //estimated_cam_pose  = Pose3(Rot3(rotation_frame_estimated ....),Point3(translation_frame_estimated ....))
//                pGraph->emplace_shared(Symbol('X',kfid*cam_count+i), estimated_cam_pose);
//            }
            insertFramePoseEstimation(kfid,pGraph,pInitialEstimate_output);
            noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances((Vector(6) << 1e-3, 1e-3, 1e-3, 1e-1, 1e-1, 1e-1).finished());
            pGraph->add(PriorFactor<Pose3>(Symbol('F',kfid),pInitialEstimate_output->at(Symbol('F',kfid)).cast<Pose3>(),priorModel));//加入约束.
            cout<<"In generateCurrentGraphByKFIDVector():added prior!"<<endl;
            //if(kf_index ==0)
            //{
            //    cout<<"In generateCurrentGraphByKFIDVector():added NonlinearEquality!"<<endl;
            //    pGraph->emplace_shared<NonlinearEquality<Pose3> >(Symbol('F',kfid),pInitialEstimate_output->at(Symbol('F',kfid)).cast<Pose3>());//固定滑动窗口第一帧 DEBUG ONLY!!
            //}
        }
        for(int kf_index = 0;kf_index<kf_ids.size();kf_index++)
        {
            int kfid = kf_ids.at(kf_index);
            auto pCurrent = this->frameTable.query(kfid);
            //再加L.
            for(auto iter_ = pCurrent->reproj_map.begin();iter_!=pCurrent->reproj_map.end();++iter_)//查找当前KF作为普通帧时,每个reprojection map.
            {
                int referring_kfid = iter_->first;
                ReprojectionRecordT& rec = iter_ ->second;
                if(relativeOrdinaryFrameIDs.count(referring_kfid))//存在这一项.
                {//两个帧之间可以投影.因为都在sliding window中.

                    shared_ptr<Frame> pRefKF = this->frameTable.query(referring_kfid);
                    for(int cam_index = 0;cam_index<cam_count;cam_index++)
                    {
                        auto& p2d_reproj_vec = rec.at(cam_index);
                        for(int p2d_reproj_index = 0;p2d_reproj_index<p2d_reproj_vec.size();p2d_reproj_index++)
                        {
                            auto proj_ = p2d_reproj_vec.at(p2d_reproj_index);
                            if(proj_.tracking_state != TRACK_MONO2MONO)//检查对应的跟踪状态.只要不是mono to mono就行.
                            {
                                //SingleProjectionT& proj_ = p2d_reproj_vec.at(p2d_reproj_index);
                                cv::Point2f referring_p2d = pRefKF->p2d_vv.at(cam_index).at(proj_.ref_p2d_id);
                                float referring_disp = pRefKF->disps_vv.at(cam_index).at(proj_.ref_p2d_id);
                                int proj_referring_landmark_id = pRefKF->getLandmarkIDByCamIndexAndp2dIndex(cam_index,proj_.ref_p2d_id);
                                if(proj_referring_landmark_id>=0)//存在对应的landmark;不存在不处理.
                                {
                                    shared_ptr<LandmarkProperties> pLandmark = this->landmarkTable.query(proj_referring_landmark_id);
                                    if(!landmark_index_set.count(proj_referring_landmark_id))
                                    {//优化图中不存在对应的landmark.先创建Symbol并插入创建这一个landmark关键帧处的观测.landmark的估计之前已经做过了.
                                        //pGraph->emplace_shared<Point3>
                                        set<int> intersec;
                                        Intersection(relativeOrdinaryFrameIDs,pLandmark->ObservedByFrameIDSet,intersec);//小的在前,大的在后,时间复杂度有保证.
                                        if(intersec.size()<1)//窗口中观测次数,自己那次不算.
                                        {
                                            //观测次数不足.舍弃.
                                            continue;
                                        }
                                        landmark_index_set.insert(proj_referring_landmark_id);
                                        bool estimate_position_valid;
                                        auto p_ = pLandmark->getEstimatedPosition(estimate_position_valid);
                                        if(estimate_position_valid)
                                        {
                                            pInitialEstimate_output->insert(Symbol('L',proj_referring_landmark_id),p_);
                                            //加入初始位置估计.
                                            auto kf_p2d = pRefKF->p2d_vv.at(cam_index).at(proj_.ref_p2d_id);
                                            if(pRefKF->map2d_to_3d_pt_vec.at(cam_index).count(proj_.ref_p2d_id)>0)
                                            {//在被创建的关键帧,它已经三角化成功了.
                                                auto kf_disp__ = pRefKF->disps_vv.at(cam_index).at(proj_.ref_p2d_id);//disp
                                                pGraph->emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(kf_p2d.x,kf_p2d.x - kf_disp__,kf_p2d.y),robust_kernel,
                                                    Symbol('X',referring_kfid*cam_count + cam_index ),Symbol('L',proj_referring_landmark_id),K_stereo);
                                            }
                                            else
                                            {
                                                pGraph->emplace_shared<GenericProjectionFactor<Pose3,Point3,Cal3_S2> > (Point2(kf_p2d.x,kf_p2d.y),robust_kernel_2d,
                                                    Symbol('X',kfid*cam_count+cam_index),Symbol('L',proj_referring_landmark_id),K_mono);
                                            }
                                        }
                                        else
                                        {
                                            LOG(ERROR)<<"Invalid estimation of landmark position!"<<endl;
                                            throw "bad estimation of landmark position!";
                                            //exit(-1);//DEBUG ONLY.
                                        }
                                    }
                                    //cv::Point2f proj_.current_frame_p2d;
                                    //float proj_.disp
                                    //分情况,创建对应的投影关系约束.
                                    //只需要约束后面的观测就行了.不需要再去追加.
                                    const int i = cam_index;

                                    if(proj_.tracking_state == TRACK_STEREO2MONO)
                                    {
                                        pGraph->emplace_shared<GenericProjectionFactor<Pose3,Point3,Cal3_S2> > (Point2(proj_.current_frame_p2d.x,proj_.current_frame_p2d.y),robust_kernel_2d,
                                                    Symbol('X',kfid*cam_count+i),Symbol('L',proj_referring_landmark_id),K_mono);
                                        cout<<"added a mono projection into kf graph!"<<endl;
                                    }
                                    else if(proj_.tracking_state == TRACK_MONO2STEREO||proj_.tracking_state == TRACK_STEREO2STEREO)
                                    {
                                        auto current_p2d = proj_.current_frame_p2d;
                                        pGraph->emplace_shared<GenericStereoFactor<Pose3,Point3> > (StereoPoint2(current_p2d.x,current_p2d.x - proj_.disp,current_p2d.y),robust_kernel,
                                                    Symbol('X',kfid*cam_count+i),Symbol('L',proj_referring_landmark_id),K_stereo);
                                        cout<<"added a stereo reprojection into kf graph!"<<endl;
                                    }//MONO2MONO暂时不处理.
                                }
                            }
                        }
                    }
                }
            }
        }
        return pGraph;
    }
    double evaluateTrackingQualityScoreOfFrame(int frame_id);//查询重投影/跟踪质量(从数据库图结构的角度),评估是否需要创建新的关键帧.
    double analyzeTrackQualityOfCamID(int frame_id,int cam_id);//分析某一组摄像头的投影质量.
    void DEBUG_showLossForeachFactor(NonlinearFactorGraph& graph,Values& values)
    {
        //graph.error()
    }
};

}



#endif
