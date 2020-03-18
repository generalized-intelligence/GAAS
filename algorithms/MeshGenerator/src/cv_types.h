#ifndef CV_TYPES_MESHGEN_H
#define CV_TYPES_MESHGEN_H


#include <unordered_map>
#include "MeshObject.h"
//#include "imgproc.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>



#define USE_OPENCL_UMAT // 使用OpenCL加速.


#ifdef USE_OPENCL_UMAT
typedef cv::UMat cvMatT;
#else
typedef cv::Mat cvMatT;
#endif
//读取.
inline cvMatT IMREADcvMatT(const std::string& path)
{
    cv::Mat im = cv::imread(path);
#ifdef USE_OPENCL_UMAT
    return im.getUMat(cv::ACCESS_RW);
#else
    return im;
#endif
}
//互相转换.
inline cvMatT convertMatTocvMatT(cv::Mat& mat)
{
#ifdef USE_OPENCL_UMAT
    return mat.getUMat(cv::ACCESS_RW);
#else
    return mat;
#endif
}
inline cv::Mat getMatFromcvMatTReadOnly(const cvMatT& mat)
{
#ifdef USE_OPENCL_UMAT
    return mat.getMat(cv::ACCESS_READ);
#else
    return mat;
#endif
}



inline cv::Mat getMatFromcvMatT(cvMatT& mat)
{
#ifdef USE_OPENCL_UMAT
    return mat.getMat(cv::ACCESS_RW);
#else
    return mat;
#endif
}




#endif
