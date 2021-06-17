#ifndef CAM_INFO_H
#define CAM_INFO_H
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/mat.hpp>

/*class CamInfo
{
publish:
    virtual bool isStereo() = 0;
    virtual cv::Mat* getCamMat() = 0;
    virtual cv::Mat* getRTMat() = 0;
    //{
    //    return this->isStereo;
    //}
//private:
//    bool isStereo;
};*/
class CamInfo
{
public:
    virtual bool isStereo()
    {
        return false;
    }
    CamInfo()
    {

    }
    CamInfo(const cv::FileNode& cam_node)
    {
        //eg:
        //A = (int)node["A"];
        //X = (double)node["X"];
        //id = (string)node["id"];
        cam_node["camMat"]>>this->camera_mat;
        cam_node["camRTMat"]>>this->rt_mat;
    }
    virtual cv::Mat getCamMat()
    {
        LOG(INFO)<<"in base cam class getCamMat()"<<std::endl;
        return (this->camera_mat);
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
    virtual void getCamMatFxFyCxCy(float& fx,float& fy,float& cx,float& cy)
    {
        auto k = this->getCamMat();
        fx = k.at<float>(0,0);
        fy = k.at<float>(1,1);
        cx = k.at<float>(0,2);
        cy = k.at<float>(1,2);
    }
private:
    cv::Mat camera_mat;
    cv::Mat rt_mat;
};



#endif
