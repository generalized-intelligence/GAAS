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

namespace mcs {

class FrameTable;
class LandmarkTable;
class RelationTableT;

struct LandmarkProperties;
struct RelationT;

struct ObservationInfo
{
    //weak_ptr<Frame> pObservedByFrame;
    int observed_by_frame_id;
    int relative_p2d_index;
    int relative_p3d_index;
};


struct LandmarkProperties
{
    //mutex propertiesMutex;
    mutex writeMutex;
    Pose3 estimatedPose;
    bool pose_ever_init = false;
    Pose3 getEstimatedPose(bool& valid)
    {
        if(pose_ever_init)
        {
            valid = true;
            return estimatedPose;
        }
        valid = false;
    }
    int landmark_reference_time = 0;
    int cam_id;
    //weak_ptr<Frame> pCreatedByFrame;
    int created_by_kf_id;
    int relative_kf_p2d_id;//在created_by_kf_id这个帧里对应的p2d id.
    //weak_ptr<Frame> pLastObservedByFrame;
    //boost::shared_ptr<SmartStereoProjectionPoseFactor> pRelativeStereoSmartFactor;//目前还没实现处理无穷远.//不用这个垃圾东西了.
    std::vector<ObservationInfo> vObservationInfo;//这个和relationT选一个?
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
            for(auto &tracked_pt:p2d_vec)
            {
                int originalKFp2dID = tracked_pt.ref_p2d_id; //判断对应投影类型,创建对应投影关系约束.
                cv::Point2f& p2d_ = tracked_pt.current_frame_p2d;
                //查找对应disp.//disp直接存在里面.
                float disp = tracked_pt.disp;
                char tracking_state = tracked_pt.tracking_state;
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
        //排序 逐个查投影关系.
        //加入优化图.

    }

    double evaluateTrackingQualityScoreOfFrame(int frame_id);//查询重投影/跟踪质量(从数据库图结构的角度),评估是否需要创建新的关键帧.

    double analyzeTrackQualityOfCamID(int frame_id,int cam_id);//分析某一组摄像头的投影质量.
};

}



#endif
