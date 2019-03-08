#ifndef LOOPCLOSINGMANAGER_H
#define LOOPCLOSINGMANAGER_H

#include <iostream>
#include <vector>

// DBoW2/3
#include "../src/DBoW3.h" // defines OrbVocabulary and OrbDatabase
//#include "src/DBoW3.h"

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d.hpp"
#include <memory>
//#include<boost/smart_ptr.hp0p>
//#include "opencv2/xfeatures2d.hpp"

using namespace DBoW3;
using namespace std;
using namespace cv;
//using namespace boost;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<cv::Mat > > &features ,vector<vector<cv::KeyPoint> > &kp_list,vector<cv::Mat> &kdesp_list,std::string detect_method = "orb",std::string compute_method = "brief");
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);
void testVocCreation(const vector<vector<cv::Mat > > &features);
void testDatabase(const vector<vector<cv::Mat > > &features,const std::string db_path,vector<vector<cv::KeyPoint> > &kp_list,vector<cv::Mat> &kdesp_list);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
//const int NIMAGES = 4;
//const int NIMAGES = 1700;

//const int NIMAGES = 1798;
//const int NIMAGES=2250;
//const int NIMAGES=1470;


//const int NIMAGES = 40;

const int RET_QUERY_LEN = 4;
//const int TOO_CLOSE_THRES = 15;
const int TOO_CLOSE_THRES = 400;


//const float DB_QUERY_SCORE_THRES = 0.4;//0.5;//0.65;
const float DB_QUERY_SCORE_THRES = 0.0075;//0.015;//0.5;//0.65;
const int STEP1_KP_NUM = 8;//12;
const int STEP2_KP_NUM = 5;//8;

const double ORB_TH_HIGH = 20;//pretty good.//10; pretty good//5;


struct FrameInfo
{
    vector<cv::KeyPoint> keypoints;
    //cv::Mat descriptors;
    //vector<cv::Mat > descriptors;
    cv::Mat descriptors;
    //SE3
    //IMU_INFO
};

typedef std::shared_ptr<FrameInfo> ptr_frameinfo;

class LoopClosingManager
{
public:
    LoopClosingManager(const std::string &voc_path);
    LoopClosingManager(const std::string &voc_path,const std::string & frame_db_path);
    void addKeyFrame(const cv::Mat& image);
    void addKeyFrame(ptr_frameinfo info);
    QueryResults queryKeyFrames(ptr_frameinfo info);
    int detectLoopByKeyFrame(ptr_frameinfo info,std::vector<DMatch>& good_matches_output,bool current_frame_has_index);
    int loadVoc(const std::string& voc_path);
    int saveDB();
    void loadFromDB();
    static ptr_frameinfo extractFeature(const cv::Mat& image);
    inline ptr_frameinfo& getFrameInfoById(int i)
    {
      return frameinfo_list[i];
    }
private:
    
    Vocabulary voc;
    
private:
    Database frame_db;
    int frame_index;
    std::vector<ptr_frameinfo> frameinfo_list;
    int loop_id;//just for visualize.
};

#endif
