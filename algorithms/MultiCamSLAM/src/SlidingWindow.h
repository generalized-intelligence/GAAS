#ifndef MCS_SLIDING_WINDOW_H
#define MCS_SLIDING_WINDOW_H
#include <deque>
#include <memory>
#include <mutex>
#include "Frame.h"
#include "FrameWiseGeometry.h"

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


#include "FrameManager.h"

#include <thread>
#include <opencv2/core/eigen.hpp>
#include "IMU_Preint_GTSAM.h"
#include <iostream>
#include "Visualization.h"
#include "ReprojectionInfoDatabase.h"

using namespace gtsam;
using namespace std;
using namespace cv;

namespace mcs
{


//滑动窗口类
class SlidingWindow
{
private:
    //deque<shared_ptr<Frame> > ordinaryFrameQueue;
    //deque<shared_ptr<Frame> > KF_queue;
    //vector<weak_ptr<LandmarkProperties> vLandmarks;//shared_ptr保存在对应的kf里.

    //扩展kf的内容.这里不需要外部Frame结构知道这种扩展.
    //map<int,LandmarkProperties> KF_landmark_extension_map;
    ReprojectionInfoDatabase reproj_db;
    const int max_kf_count = 5;


public:
    SlidingWindow()
    {
    }
    inline shared_ptr<Frame> getLastKF()
    {
        //return KF_queue.end();
    }
    shared_ptr<Values> optimizeFactorGraph(shared_ptr<NonlinearFactorGraph> pGraph,shared_ptr<Values> pInitialEstimate)
    {
        //初始化一个Levenburg-Marquardt优化器,解优化图.返回估计值.
    }
    void trackAndKeepReprojectionDBForFrame(shared_ptr<Frame> pFrame)//这是普通帧和关键帧公用的.
    {
        for(const int& ref_kf_id:this->getInWindKFidVec())//对当前帧,追踪仍在窗口内的关键帧.
        {
            //第一步 跟踪特征点,创建关联关系.
            const int cam_count = pFrame->get_cam_num();
            vector<vector<Point2f> > vvLeftp2d,vvRightp2d;
            vvLeftp2d.resize(cam_count);vvRightp2d.resize(cam_count);
            vector<map<int,int> > v_p2d_to_kf_p3d_index;
            v_p2d_to_kf_p3d_index.resize(cam_count);
            vector<vector<Point2f> >  v_originalKFP2d_relative;//对应的关键帧p2d.
            v_originalKFP2d_relative.resize(cam_count);
            vector<char> v_track_success;
            v_track_success.resize(cam_count);
            vector<int> v_track_success_count;
            v_track_success_count.resize(cam_count);
            for(int i = 0;i<cam_count;i++)//这里可以多线程.暂时不用.
            {
                //doTrackLastKF(pFrame,getLastKF(),i,vvLeftp2d.at(i),vvRightp2d.at(i),v_p2d_to_kf_p3d_index.at(i),v_originalKFP2d_relative.at(i),&v_track_success.at(i),&v_track_success_count.at(i));
                doTrackLastKF(...);//跟踪窗口里的所有kf.处理所有情况(mono2mono,mono2stereo,stereo2mono,stereo2stereo.)
            }
            //threads.join();//合并结果集.

            pFrame->reproj_map.at(ref_kf_id) = reprojectionRecordT();
            pFrame->reproj_map.at(ref_kf_id).resize(cam_count);

            //第二步 建立地图点跟踪关系 根据结果集维护数据库.
            for(int i = 0 ;i<cam_count,i++)//这是一个不可重入过程.数据库本身访问暂时没有锁.暂时估计应该不需要.
            {
                if(v_track_success.at(i))
                {
                    //反复查表,改数据库.
                    pFrame->reproj_map.at(ref_kf_id).at(i).push_back();
                            this->reproj_db->table_xxx.insert(...);
                }
                else
                {//防止出现空结构无法访问.

                }
            }
        }
    }
    void insertKFintoSlidingWindow(shared_ptr<Frame> pCurrentKF)
    {
        //第一步 跟踪特征点,创建关联关系.
        //第二步 建立地图点跟踪关系 根据结果集维护数据库.
        trackAndKeepReprojectionDBForFrame(pCurrentKF);
        //第三步 将当前普通帧升级成一个关键帧.
        upgradeOrdinaryFrameToKeyFrameStereos(pCurrentKF);//升级.
        //第四步 创建局部优化图 第一次优化.
        //method<1>.pnp初始位置估计.
        //  stage<1>.选取最优相机组,估计初始位置.
        //method<2>.继承上一帧初始位置.直接图优化和上一帧的关系.
//        for(int i = 0;i<cam_count;i++)
//        {//创建对应的X,L;插入初始值(X位姿估计在上一个估计点;L位置估计在对应X的坐标变换后位置);Between Factor<>,每次对滑窗里最早的帧约束位置关系.
//            int pose_index = x*pCurrentKF->frame_id + i;
//            localGraph.emplace_shared<Pose3>(...);
//            localGraph.add();
//            localInitialEstimate.insert(...);
//        }

        shared_ptr<Values> pLocalInitialEstimate,pLocalRes;
        shared_ptr<NonlinearFactorGraph> pLocalGraph = this->reproj_db.generateLocalGraphByFrameID(pCurrentKF->frame_id,pLocalInitialEstimate);//优化当前帧.
        pLocalRes = optimizeFactorGraph(pLocalGraph,pLocalInitialEstimate);//TODO:这种"优化" 可以考虑多线程实现.
        //优化这个图.
        //第五步 对当前帧,跟踪滑窗里的所有关键帧(地图点向当前帧估计位置重投影).创建新优化图.

        shared_ptr<Values> pSWRes,pSWInitialEstimate;
        shared_ptr<NonlinearFactorGraph> pSlidingWindGraph = this->reproj_db.generateCurrentGraphByKFIDVector(this->getInWindKFidVec(),pSWInitialEstimate);
        pSWRes = optimizeFactorGraph(pSlidingWindGraph,pSWInitialEstimate);
        //第六步 第二次优化.

        //第七步 进行Marginalize,分析优化图并选择要舍弃的关键帧和附属的普通帧,抛弃相应的信息.
        shared_ptr<Frame> pToMarginalizeKF = this->proposalMarginalizationKF();
        if(pToMarginalize!= nullptr)
        {
            removeKeyFrameAndItsProperties(pToMarginalizeKF);
            removeOrdinaryFrame(pToMarginalizeKF);
            //TODO:对它的每一个从属OrdinaryFrame进行递归删除.
        }
    }

    void insertOrdinaryFrameintoSlidingWindow(shared_ptr<Frame> pCurrentFrame)
    //普通帧和关键帧的区别是:普通帧没有附属landmark properties;普通帧与普通帧之间不考虑关联追踪.
    {
        //第一步 跟踪特征点.
        //第二步 建立地图点跟踪关系. 修改/创建LandmarkProperties.
        trackAndKeepReprojectionDBForFrame(pOrdinaryFrame);
        //第三步 创建局部优化图.第一次优化.
        shared_ptr<NonlinearFactorGraph> pLocalGraph = this->reproj_db.generateLocalGraphByFrameID(pOriginaryFrame->frame_id);


    }

    void removeOrdinaryFrame(shared_ptr<Frame>);
    void removeKeyFrameAndItsProperties(shared_ptr<Frame>);
    inline int getOrdinaryFrameSize();
    inline int getKFSize();
    vector<int> getInWindKFidVec();
    void proposalMarginalizationKF()//提议一个应该被marg的关键帧.
    {
    }
};








}
#endif
