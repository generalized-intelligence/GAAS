#ifndef __TYPEDEFS_H_FILE
#define __TYPEDEFS_H_FILE
#include <Eigen/Core>
#include <Eigen/Dense>  
#include <Eigen/StdVector>
#include <glog/logging.h>
#include <thread>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>


#define USE_UMAT_AS_TYPE 0
namespace mcs 
{
    using namespace std;
    using namespace cv; 
    //part0 typedefs.
    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > VecVector2f;


    #if USE_UMAT_AS_TYPE == 1
    typedef cv::UMat cvMat_T; // use UMat to enable Transparent API and OpenCL acceleration.
    #else
    typedef cv::Mat cvMat_T;
    #endif


    cvMat_T IMREAD(const string& path)
    {
        #if USE_UMAT_AS_TYPE == 1
        cvMat_T res;
        cv::imread(path).copyTo(res);
        return res;
        #else
        return cv::imread(path);
        #endif
    }
    typedef struct
    {   
        std::vector<KeyPoint> kps;
        cvMat_T desp;
    }PointWithFeatureT;
    const static int KEYPOINT_METHOD_GFTT = 0;
    const static int KEYPOINT_METHOD_ORB = 1;

    const static int OPT_FLOW_FARNE_BACK = 0;
    const static int OPT_FLOW_PYRLK = 1;
}
#endif
