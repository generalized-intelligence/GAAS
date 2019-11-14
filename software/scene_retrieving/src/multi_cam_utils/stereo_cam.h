#ifndef STEREOCAM_UTIL_H
#define STEREOCAM_UTIL_H

#include "multi_cam_utils/cam_info.h"

class StereoCamConfig:CamInfo
{
public:
    virtual bool isStereo()
    {
        return true;
    }
    StereoCamConfig(const FileNode& cam_node)
    {
        //eg:
        //A = (int)node["A"];
        //X = (double)node["X"];
        //id = (string)node["id"];
    }
    virtual cv::Mat& getCamMat()
    {
        return (this->camera1_mat);
    }
    virtual cv::Mat& getRTMat()
    {
        return (this->rt_mat);
    }
    cv::Mat& getQMat()
    {
        return this->q_mat;
    }
private:
    cv::Mat camera1_mat;
    cv::Mat camera2_mat;
    cv::Mat rt_mat;//for camera 1 only.
    cv::Mat q_mat;
};



#endif
