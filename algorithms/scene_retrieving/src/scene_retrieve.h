#include <iostream>
#include <vector>

// DBoW2/3
#include "DBoW3.h" // defines OrbVocabulary and OrbDatabase

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"

using namespace DBoW3;
using namespace std;
using namespace cv;

using namespace cv;

class Scene
{
public:
    Scene();
    void fromCVMat(const std::vector<cv::Point3d>& points_3d,const std::vector<Mat>& point_desps)
    {
      ;
    }
    void saveFile(const std::string &filename)
    {
      ;
    }
    void loadFile(const std::string &filename)
    {
      ;
    }
    void RotateAndTranslate(const Mat &RT)
    {
      ;
    }
private:
    bool hasScale = false;
    std::vector <cv::Point3d> vec_p3d;
    std::vector <cv::Mat> point_desps;
    cv::Mat m_RT = cv::Mat::eye(4,4,CV_32F);
};

class SceneRetriever
{
public:
    SceneRetriever();
    SceneRetriever(const Scene& original_scene);
    SceneRetriever(const std::string& scene_file);
    
private:
};
