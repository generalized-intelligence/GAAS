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
    void insertKFintoSlidingWindow(shared_ptr<Frame> pCurrentKF)
    {
        //第一步 跟踪特征点,创建关联关系.
        const int cam_count = pCurrentKF->get_cam_num();
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
            doTrackLastKF(pCurrentKF,getLastKF(),i,vvLeftp2d.at(i),vvRightp2d.at(i),v_p2d_to_kf_p3d_index.at(i),v_originalKFP2d_relative.at(i),&v_track_success.at(i),&v_track_success_count.at(i));
        }
        //threads.join();//合并结果集.


        //第二步 建立地图点跟踪关系 根据结果集维护数据库.
        for(int i = 0 ;i<cam_count,i++)//这是一个不可重入过程.数据库本身访问暂时没有锁.暂时估计应该不需要.
        {
            this->reproj_db->table_xxx.insert(...);
        }
        //第三步 创建局部优化图 第一次优化.
        for(int i = 0;i<cam_count;i++)
        {//创建对应的X,L;插入初始值(X位姿估计在上一个估计点;L位置估计在对应X的坐标变换后位置);Between Factor<>,每次对滑窗里最早的帧约束位置关系.

        }
        //第四步 对当前帧,跟踪滑窗里的所有关键帧(地图点向当前帧估计位置重投影).创建新优化图.

        //第五步 第二次优化.

        //第六步 进行Marginalize,分析优化图并选择要舍弃的关键帧和附属的普通帧,抛弃相应的信息.
        shared_ptr<Frame> pToMarginalizeKF = this->proposalMarginalizationKF();
        if(pToMarginalize!= nullptr)
        {
            removeKeyFrameAndItsProperties(pToMarginalizeKF);
            removeOrdinaryFrame(pToMarginalizeKF);
            //TODO:对它的每一个从属OrdinaryFrame进行递归删除.
        }
    }
    shared_ptr<Frame> generateKFFromOrdinaryFrame(shared_ptr<Frame> pOridinaryFrame);//把普通帧转化为关键帧.帧id不变.

    void insertOrdinaryFrameintoSlidingWindow(shared_ptr<Frame> pCurrentFrame)
    //普通帧和关键帧的区别是:普通帧没有附属landmark properties;普通帧与普通帧之间不考虑关联追踪.
    {
        //第一步 跟踪特征点.
        doTrackLastKF(pCurrentFrame,getLastKF(),i,...);
        //第二步 建立地图点跟踪关系. 修改/创建LandmarkProperties.
        //第三步 创建局部优化图.第一次优化.

    }

    void removeOrdinaryFrame(shared_ptr<Frame>);
    void removeKeyFrameAndItsProperties(shared_ptr<Frame>);
    inline int getOrdinaryFrameSize();
    inline int getKFSize();
    void proposalMarginalizationKF()//提议一个应该被marg的关键帧.
    {
    }
    void OptimizeLocalWindow();//局部窗口构造优化问题.
};








}
#endif
