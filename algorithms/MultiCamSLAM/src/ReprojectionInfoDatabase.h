#ifndef REPROJECTION_INFO_DATABASE
#define REPROJECTION_INFO_DATABASE
#include <deque>
#include <memory>
#include <mutex>
#include "Frame.h"
#include "FrameWiseGeometry.h"

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
    weak_ptr<Frame> pObservedByFrame;
    int relative_p2d_index;
    int relative_p3d_index;
};
struct LandmarkProperties
{
    //mutex propertiesMutex;
    mutex writeMutex;


    int landmark_reference_time = 0;
    int cam_id;
    weak_ptr<Frame> pCreatedByFrame;
    //weak_ptr<Frame> pLastObservedByFrame;
    //boost::shared_ptr<SmartStereoProjectionPoseFactor> pRelativeStereoSmartFactor;//目前还没实现处理无穷远.//不用这个垃圾东西了.
    std::vector<ObservationInfo> vObservationInfo;
};
struct RelationT//定义一次追踪关系.
{
    int cam_id;
    int landmarkID;
    int referedByKFID;//当前跟踪关系 被跟踪的那个关键帧.
    int trackedByFrameID;//这个不是列表,这个直接就是第二次被追踪的帧id;多次追踪就创建多个RelationShip.
};
class FrameTable
{
private:
    mutex writeMutex;
public:
    int currentID = 0;
    map<int,shared_ptr<Frame> > dataTable;
    void insertFrame(shared_ptr<Frame> pFrame);
    inline shared_ptr<Frame> query(int id);
    vector<shared_ptr<Frame> > queryByRefKFID(int refKFID);//查询函数都按这个模型写.
    void marginalizeKFDeleteEverythingRecursive(int KFID);
};
class LandmarkTable
{
private:
    mutex writeMutex;
public:
    int currentID = 0;
    map<int,shared_ptr<LandmarkProperties> > dataTable;
    void insertLandmark(shared_ptr<LandmarkProperties> pLandmark);
    inline shared_ptr<LandmarkProperties> query(int id);
    vector<shared_ptr<LandmarkProperties> > queryByCreatedByKF(int createdByKFID);
};
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
//跟踪用的函数.


class ReprojectionInfoDatabase{
private:
    mutex writeMutex;
public:
    FrameTable frameTable;
    LandmarkTable landmarkTable;
    RelationTableT relationTable;


    vector<shared_ptr<RelationT> > queryAllRelatedRelationsByKF(shared_ptr<Frame> pKF);//查询所有相关的点.用于创建投影关系和marginalize;
    shared_ptr<NonlinearFactorGraph> generateCurrentGraphByKFIDVector(vector<int> kf_ids);//查数据库,仅根据当前kfid的列表,生成一个优化图.默认id是排序后的,第一个id被固定.
    double evaluateTrackingQualityScoreOfFrame(int frame_id);//查询重投影/跟踪质量(从数据库图结构的角度),评估是否需要创建新的关键帧.
};

}



#endif
