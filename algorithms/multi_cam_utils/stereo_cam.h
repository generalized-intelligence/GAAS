#ifndef STEREOCAM_UTIL_H
#define STEREOCAM_UTIL_H

#include "cam_info.h"
#include <iostream>
class StereoCamConfig:CamInfo
{
public:
    virtual bool isStereo()
    {
        return true;
    }
    StereoCamConfig(const cv::FileNode& cam_node)
    {
        //eg:
        //A = (int)node["A"];
        //X = (double)node["X"];
        //id = (string)node["id"];
        LOG(INFO)<<"check stereo cam config:"<<std::endl;
        cam_node["camMat1"]>>this->camera1_mat;
        LOG(INFO)<<"check stereo cam config: 1"<<std::endl;
        cam_node["camMat2"]>>this->camera2_mat;
        LOG(INFO)<<"check stereo cam config: 2"<<std::endl;
        cam_node["camRTMat"]>>this->rt_mat;
        LOG(INFO)<<"check stereo cam config: 3"<<std::endl;
        cam_node["QMat"]>>this->q_mat;
    }
    virtual cv::Mat getCamMat()
    {
        return (this->camera1_mat);
    }
    virtual cv::Mat getRTMat()
    {
        return (this->rt_mat);
    }
    cv::Mat getQMat()
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
