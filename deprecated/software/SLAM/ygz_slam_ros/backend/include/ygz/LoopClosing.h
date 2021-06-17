#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include <iostream>

#include <ygz/Frame.h>
#include <ygz/Tracker.h>
#include <ygz/System.h>
#include <ygz/Settings.h>
#include <ygz/ORBMatcher.h>
#include <ygz/Frame.h>
//#include <ygz/CeresHelper.h>
#include <ygz/utility.h>

//#include <DBoW2/BowVector.h>
//#include <DBoW2/FeatureVector.h>
//#include <DBoW2/TemplatedVocabulary.h>
//#include <DBoW2/TemplatedDatabase.h>
//#include <DBoW2/DBoW2.h>
#include "LoopClosingManager.h"

#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

//#include <DUtils/DUtils.h>



namespace ygz{
  
  using namespace std;
  using namespace DBoW3;
  //using namespace DVision;
  using namespace Eigen;
  
  class Tracker;
  
  struct Frame;
  
  class LoopClosing
  {
    
	      
  public:
    
    LoopClosing(string voc_path);
    
    int DetectLoop(shared_ptr<Frame> pFrame, int frame_idx);
    
    void addKeyFrame(shared_ptr<Frame> pFrame, bool flag_detect_loop);

    shared_ptr<Frame> getFrame(size_t index);
    
//     void optimized4DoF();
    
//     shared_ptr<Frame> mpCurrentKF = nullptr;
//     
//     vector<shared_ptr<Frame> > mvKF;
//      
//     shared_ptr<Tracker> mpTracker = nullptr;
//     
//     shared_ptr<Frame> mCurrentFrame = nullptr;
//     
//     vector<shared_ptr<Frame> > mvLoopClosureCandidates;
    
      
    //BriefDatabase mDB;
    //BriefVocabulary* mVoc;

    map<int, cv::Mat> image_pool;

    size_t global_index = 0;
    
    size_t earliest_loop_index = -1;
    
    list<shared_ptr<Frame> > mFrameList;
    
    std::mutex m_optimize_buf;
    std::mutex m_framelist;
    
    std::thread t_optimization;
    std::queue<int> optimize_buf;
    
    
  };
  
}

#endif
