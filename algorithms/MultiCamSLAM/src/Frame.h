#ifndef FRAME_H_FILE_PROTECT
#define FRAME_H_FILE_PROTECT

#include "TypeDefs.h"
#include <glog/logging.h>
#include <vector>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>  
#include <Eigen/StdVector>

#include "stereo_cam.h"
namespace mcs
{
    using namespace std;
    using Eigen::Vector2f;

    typedef cv::Point2f p2dT;
    typedef cv::Point3f p3dT;
    typedef std::pair<shared_ptr<cvMat_T>,shared_ptr<cvMat_T> > StereoMatPtrPair;
    typedef cv::Mat Feature;
    struct MapPoint
    {
        Feature feat;
        cv::Point3d pos;
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
    typedef vector<double> IMU_Data_T;
    struct Frame
    {   
    public:
        Frame()
        {
            LOG(ERROR)<<"In null construction function of Frame()! Nothing will be done."<<endl;
        }
        Frame(vector<StereoMatPtrPair> pLRImgs);
        Frame(vector<shared_ptr<cvMat_T> > pOriginalImgs,vector<shared_ptr<cv::Mat>>pDepthImgs);
    
        vector<vector<p2dT> > p2d_vv;
        vector<vector<p3dT> > p3d_vv;
        vector<CamInfo> cam_info_vec;
        vector<vector<shared_ptr<FeaturePoint> > > feature_points;
        vector<shared_ptr<MapPoint> > fetchMapPoints();
        void removeOriginalImages()
        {
            this->pLRImgs = shared_ptr<vector<StereoMatPtrPair> >(nullptr);
            this->pOriginalImgs = shared_ptr<vector<shared_ptr<cvMat_T> > >(nullptr);
        }
        shared_ptr<vector<StereoMatPtrPair> > pLRImgs;
        shared_ptr<vector<shared_ptr<cvMat_T> > > pOriginalImgs,pDepthImgs;
        IMU_Data_T imu_info_vec;
        vector<shared_ptr<cvMat_T> > getMainImages()//for stereo frame: main images is the left ones of each pair;
                                                    //for depth frame: main images is the main rgb/grayscale cam.
        {
            if(this->frame_type == FRAME_TYPE_STEREO)
            {
                vector<shared_ptr<cvMat_T> > ret_vec;
                for(auto u:(*pLRImgs))
                {
                    ret_vec.push_back(std::get<0>(u));
                }
                return ret_vec;
            }
            else
            {
                LOG(ERROR)<<"not implemented yet."<<endl;
            }
        }
    
        int frame_type;
    };

}
#endif
