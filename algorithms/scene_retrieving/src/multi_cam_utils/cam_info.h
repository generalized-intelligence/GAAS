#ifndef CAM_INFO_H
#define CAM_INFO_H
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
    CamInfo(const FileNode& cam_node)
    {
        //eg:
        //A = (int)node["A"];
        //X = (double)node["X"];
        //id = (string)node["id"];
    }
    virtual cv::Mat& getCamMat()
    {
        return &(this->camera_mat);
    }   
    virtual cv::Mat& getRTMat()
    {
        return &(this->rt_mat);
    } 
private:
    cv::Mat camera_mat;
    cv::Mat rt_mat;
};



#endif
