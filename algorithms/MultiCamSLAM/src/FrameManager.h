#ifndef FRAME_MANAGER_H
#define FRAME_MANAGER_H

#include <glog/logging.h>



#include "SLAMOptimizationGraph.h"

using namespace std;

namespace mcs
{
    
    
    
    typedef cv::Mat Feature;
    typedef cv::Point2f p2dT;
    typedef cv::Point3f p3dT;
    typedef std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > StereoMatPtrPair;
    struct MapPoint
    {
        Feature feat;
        Point3 pos;
        bool state;
    };
    struct FeaturePoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Vector2f mPixel = Vector2f(0, 0);        // the pixel position
        shared_ptr<MapPoint> mpPoint = nullptr;  // the corresponding map point, nullptr if not associated
        float mfInvDepth = -1;                   // inverse depth, invalid if less than zero.

        // data used in ORB
        float mScore = 0;                        // score, maybe harris or other things
        float mAngle = 0;                        // angle of oriented FAST
        size_t mLevel = 0;                       // the pyramid level
        uchar mDesc[32] = {0};                   // 256 bits of ORB feature (32x8), ignored if using LK flow

        // flags
        bool mbOutlier = false;                  // true if it is an outlier
    };
    static const int FRAME_TYPE_STEREO = 0;
    static const int FRAME_TYPE_DEPTH = 1;
    struct Frame
    {
    public:
        Frame(vector<StereoMatPtrPair> pLRImgs);
        Frame(vector<shared_ptr<cv::Mat> > pOriginalImgs,pDepthImgs);
        
        vector<vector<p2dT> > p2d;
        vector<vector<p3dT> > p3d;
        vector<CamConfig> camconfig;
        vector<vector<shared_ptr<FeaturePoint> > > feature_points;
        vector<shared_ptr<MapPoint> > fetchMapPoints();
        void removeOriginalImages()
        {

        }
        shared_ptr<StereoMatPtrPair> pLRImgs;
        shared_ptr<vector<shared_ptr<cv::Mat> > > pOriginalImgs,pDepthImgs;
        int frame_type;
    };
    class KeyFrame:Frame
    {
    public:
    
    private:
    
    };
    class FrameSlidingWindow
    {
    public:
    
    private:
        deque<shared_ptr<Frame> > wind;
    };
    class FrameManager
    {
    public:
        FrameManager();
        Match2FrameAndTriangulate(shared_ptr<Frame> pf1,shared_ptr<Frame> pf2)
        {
        }
        Match2FrameAndTriangulateWithOptFlow(shared_ptr<Frame> pf1,shared_ptr pf2)
        {
            pts2d_gftt = pf1.extract
        }
        ProcessFrame(Frame frame_in)
        {
            ;
        }
    private:
        FrameSlidingWindow sliding_wind;
    };
    
    
    
}
#endif
