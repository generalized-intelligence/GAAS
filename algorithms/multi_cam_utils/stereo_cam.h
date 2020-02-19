#ifndef STEREOCAM_UTIL_H
#define STEREOCAM_UTIL_H

#include "cam_info.h"
#include <iostream>
class StereoCamConfig:public CamInfo
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
        for(auto u:cam_node.keys())
        {
            std::cout<<"keys:"<<u<<std::endl;
        }
        for(auto iter = cam_node.begin();iter!=cam_node.end();++iter)
        {
            std::cout<<(*iter).string()<<std::endl;
        }


        LOG(INFO)<<"check stereo cam config:"<<std::endl;
        const cv::FileNode& cam_mat1 = cam_node["camMat1"];

        std::cout<<"cam_node[camMat1]:"<<cam_mat1.mat()<<std::endl;
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
        LOG(INFO)<<"in stereo cam getCamMat()"<<std::endl;
        return (this->camera1_mat);
    }
    virtual cv::Mat getRTMat()
    {
        return (this->rt_mat);
    }
    virtual cv::Mat get_RMat()
    {
        return this->rt_mat.colRange(0,3).rowRange(0,3);
    }
    virtual void get_tMat(float& x,float& y,float& z)
    {
        //return this->rt_mat.colRange(3,4).rowRange(0,3);
        x = this->rt_mat.at<float>(3,0);
        y = this->rt_mat.at<float>(3,1);
        z = this->rt_mat.at<float>(3,2);
    }
    void getLCamMatFxFyCxCy(float& fx,float& fy,float& cx,float& cy)
    {
        auto k = this->camera1_mat;
        fx = k.at<float>(0,0);
        fy = k.at<float>(1,1);
        cx = k.at<float>(0,2);
        cy = k.at<float>(1,2);
    }
    void getRCamMatFxFyCxCy(float& fx,float& fy,float& cx,float& cy)
    {
        auto k = this->camera2_mat;
        fx = k.at<float>(0,0);
        fy = k.at<float>(1,1);
        cx = k.at<float>(0,2);
        cy = k.at<float>(1,2);
    }
    double getBaseLine()
    {
        LOG(ERROR)<<"in StereoCamConfig::getBaseLine():Maybe incorrect!"<<std::endl;
        return (1.0/(this->q_mat.at<float>(2,3)* (double)1.0))/this->camera1_mat.at<float>(0,0); //q[2,3] is camera bf.
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
