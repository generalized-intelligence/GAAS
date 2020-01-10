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

namespace mcs {


void Intersection(set<int> &A,set<int> &B,set<int> &result){//保证a小b大即可.
    set<int>::iterator it;
    it = A.begin();
    while(it != A.end()){
        if(B.find(*it) != B.end()) result.insert(*it);
        it++;
    }
}


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
    Pose3 getEstimatedPosition(bool& valid)
    {
        valid = position_ever_init;
        if(!valid)//DEBUG.
        {
            throw "trying to get an invalid pose estimation!";
        }
        return estimatedPosition;
    }
    void setEstimatedPosition(Point3 pose)
    {
        this->estimatedPosition = pose;
        position_ever_init = true;
    }
    int landmark_reference_time = 0;
    int cam_id;
    //weak_ptr<Frame> pCreatedByFrame;
    int created_by_kf_id;
    int relative_kf_p2d_id;//在created_by_kf_id这个帧里对应的p2d id.
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
    vector<shared_ptr<Frame> > queryByRefKFID(int refKFID);//查询函数都按这个模型写.
    void marginalizeKFDeleteEverythingRecursive(int KFID);
};
void FrameTable::insertFrame(shared_ptr<Frame> pFrame)
{
    this->dataTable[currentID] = pFrame;
    currentID++;
}
shared_ptr<Frame> FrameTable::query(int id)
{
    if(this->dataTable.count(id) ==0)
    {
        LOG(WARNING)<<"Access Violation in FrameTable.query()!"<<endl;
        return nullptr;
    }
    return this->dataTable.count(id);
}
vector<int> FrameTable::queryByRefKFID(int refKFID)
{//查询所有参考这个帧的id

}

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
    inline shared_ptr<LandmarkProperties> query(int id);
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
        return this->dataTable.at( this->kf_camid_p2did_to_landmark_id_mapping.at(kfid).at(cam_id).at(p2d_id));
    }
};
void LandmarkTable::insertLandmark(shared_ptr<LandmarkProperties> pLandmark)
{
    //插入.
    this->dataTable.(this->currentID) = pLandmark;
    //加索引.
    if(this->kfid_camid_p2did_to_landmark_id_mapping.count(pLandmark->created_by_kf_id_)  == 0)//不存在,创建:
    {
        this->kfid_camid_p2did_to_landmark_id_mapping.at(pLandmark->created_by_kf_id_) = map<int ,map<int,int> >();
    }
    if(this->kfid_camid_p2did_to_landmark_id_mapping.at(pLandmark->created_by_kf_id_).count(pLandmark->cam_id) == 0)
    {
        this->kfid_camid_p2did_to_landmark_id_mapping.at(pLandmark->created_by_kf_id_).at(pLandmark->cam_id) = map<int,int>();
    }
    this->kfid_camid_p2did_to_landmark_id_mapping.at(pLandmark->created_by_kf_id).at(pLandmark->cam_id).at(pLandmark->relative_kf_p2d_id) = this->currentID;
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

    shared_ptr<NonlinearFactorGraph> generateLocalGraphByFrameID(int frameID, shared_ptr<Values>& pInitialEstimate_output)
    {//创建"局部"优化图.只参考上一关键帧,优化本帧位置.
        //创建Xframe,Xframe+i.
        //创建Xref,Xref+i.
        //
        auto pGraph = shared_ptr<NonlinearFactorGraph>(new NonlinearFactorGraph());
        pInitialEstimate_output = shared_ptr<Values>(new Values());

        shared_ptr<Frame> pFrame = this->frameTable.query(frameID);
        //获取跟踪的点.
        shared_ptr<Frame> pRefKF = this->frameTable.query();
        int cam_count = pFrame->get_cam_num();
        for(int i = 0;i<cam_count;i++)
        {
            auto p2d_vec = pFrame->reproj_map.at(pFrame->getLastKFID()).at(i);//仅跟踪getLastKFID()这一关键帧.
            //for(auto &tracked_pt:p2d_vec)
            for(int reprojected_p2d_index = 0;reprojected_p2d_index < p2d_vec.size();reprojected_p2d_index++)
            {
                auto & tracked_pt = p2d_vec.at(reprojected_p2d_index);
                int originalKFp2dID = tracked_pt.ref_p2d_id; //判断对应投影类型,创建对应投影关系约束.
                cv::Point2f& p2d_ = tracked_pt.current_frame_p2d;
                //查找对应disp.//disp直接存在里面.
                float disp = tracked_pt.disp;
                char tracking_state = tracked_pt.tracking_state;

                int relative_landmark_id = getLandmarkIDByCamIndexAndp2dIndex(i,originalKFp2dID);//查询对应的landmark.如果没有就创建.
                if(relative_landmark_id<0)//Landmark invalid.
                {//create a landmark;
                    LOG(ERROR)<<"in generateLocalGraphByFrameID(): landmark id index access violation!"<<endl;
                    exit(-1);
                }
                auto pLandmark = this->landmarkTable.query(relative_landmark_id);
                if(tracking_state == TRACK_STEREO2MONO)
                {

                }
                else if(tracking_state == TRACK_MONO2STEREO)
                {

                }
                else if(tracking_state == TRACK_STEREO2STEREO)
                {

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


        set<int> landmark_index_set;//避免重复创建.



        //先加X.
        for(int kf_index = 0;kf_index<kf_ids.size();kf_index++)
        {
            int kfid = kf_ids.at(kf_index);
            auto pCurrent = this->frameTable.query(kfid);
            //Pose3 frame_pose;
            Matrix3d rotation_frame_estimated;
            Vector3d translation_frame_estimated;
            bool pose_estimation_valid;
            pCurrent->getRotationAndTranslation(rotation_frame_estimated,translation_frame_estimated,pose_estimation_valid);
            if(!pose_estimation_valid)
            {
                LOG(ERROR)<<"Pose estimation invalid!!!Check Optimization procedure of this kf!KF_id:"<<pCurrent->frame_id<<endl;
                exit(-1);
            }
            for(int i = 0;i<cam_count;i++)
            {
                //获取帧位姿估计.
                Pose3 estimated_cam_pose;
                //构造一个Pose3,并对其在estimated pose基础上根据camRT进行变换.
                //TODO:
                //estimated_cam_pose  = Pose3(Rot3(rotation_frame_estimated ....),Point3(translation_frame_estimated ....))
                pGraph->emplace_shared(Symbol('X',kfid*cam_count+i), estimated_cam_pose);
            }
            //再加L.

            for(auto iter_ = pCurrent->reproj_map.begin();iter_!=pCurrent->reproj_map.end();++iter_)//查找每个reprojection map.
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
                            if(proj_.tracking_state == )//检查对应的跟踪状态.
                            {
                                SingleProjectionT& proj_ = p2d_reproj_vec.at(p2d_reproj_index);

                                cv::Point2f referring_p2d = pRefKF->p2d_vv.at(cam_index).at(proj_.ref_p2d_id);
                                float referring_disp = pRefKF->disps_vv.at(cam_index).at(proj_.ref_p2d_id);
                                int proj_referring_landmark_id = pRefKF->getLandmarkIDByCamIndexAndp2dIndex(cam_index,proj_.ref_p2d_id);
                                if(proj_referring_landmark_id>=0)//存在对应的landmark;不存在不处理.
                                {
                                    if(!landmark_index_set.count(proj_referring_landmark_id))
                                    {//优化图中不存在对应的landmark.先创建.

                                        //pGraph->emplace_shared<Point3>
                                        shared_ptr<LandmarkProperties> pLandmark = this->landmarkTable.query(proj_referring_landmark_id);
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
                                        }
                                        else
                                        {
                                            LOG(ERROR)<<"Invalid estimation of landmark position!"<<endl;
                                            exit(-1);//DEBUG ONLY.
                                        }
                                    }
                                    //cv::Point2f proj_.current_frame_p2d;
                                    //float proj_.disp
                                    //TODO:分情况,创建对应的投影关系约束.
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    double evaluateTrackingQualityScoreOfFrame(int frame_id);//查询重投影/跟踪质量(从数据库图结构的角度),评估是否需要创建新的关键帧.

    double analyzeTrackQualityOfCamID(int frame_id,int cam_id);//分析某一组摄像头的投影质量.
};

}



#endif
