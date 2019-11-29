#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/persistence.hpp>
#include "FeatureFrontEndCV.h"
#include "Timer.h"
#include "FrameWiseGeometry.h"
//#include "FrameManager.h"
using namespace mcs;
using namespace std;

int main(int argc,char** argv)
{
    google::InitGoogleLogging(argv[0]);
    LOG(INFO)<<"reading imgs..."<<endl;
    shared_ptr<cvMat_T> pL(new cvMat_T(mcs::IMREAD("data/left_400.png") ));
    shared_ptr<cvMat_T> pR(new cvMat_T(mcs::IMREAD("data/right_400.png") ));
    shared_ptr<cvMat_T> pL_next(new cvMat_T(mcs::IMREAD("data/left_402.png") ));
    shared_ptr<cvMat_T> pR_next(new cvMat_T(mcs::IMREAD("data/right_402.png") ));
    if(pL->empty())
    {
        LOG(ERROR)<<"pL is empty."<<endl;
    }
    //cv::FileStorage
    LOG(INFO)<<"reading file storage..."<<endl;
    cv::FileStorage fsSettings("data/stereoSettings1_test_create_stereo_frame.yaml",cv::FileStorage::READ);
    LOG(INFO)<<"setting timer."<<endl;
    ScopeTimer t1("keyFrame generation timer");
    auto lr_pair = std::make_pair(pL,pR);
    shared_ptr<vector<StereoMatPtrPair> > pVecStereo(new vector<StereoMatPtrPair>);
    pVecStereo->push_back(lr_pair);
    LOG(INFO)<<"size:"<<fsSettings["cams"].size()<<endl;
    StereoCamConfig conf(fsSettings["cams"][0]);
    //LOG(INFO)<<"Init fn"<<endl;
    //cv::FileNode fn = fsSettings["cams"][0];
    //auto keys_ = fn.name();
    //LOG(INFO)<<"keys_"<<keys_<<endl;
    //LOG(INFO)<<"is map:"<<fn.isMap()<<endl;
    //LOG(INFO)<<"size:"<<fn.size()<<endl;

    /*

    for(int i = 0;i<keys_.size();i++)
    {
        LOG(INFO)<<"keys:"<<keys_[i]<<";";
    }*/

    vector<StereoCamConfig> cams_conf;
    cams_conf.push_back(conf);
    bool create_frame_success_output;
    auto pframe1 = createFrameStereos(pVecStereo,cams_conf,create_frame_success_output,true);

    t1.watch("create_frame time cost:");
    LOG(INFO)<<"create frame success:"<<create_frame_success_output<<endl;
    LOG(INFO)<<"Create frame2 for trackLocalFrame()..."<<endl;
    auto lr_pair2 = std::make_pair(pL_next,pR_next);
    shared_ptr<vector<StereoMatPtrPair> > pVecStereo2(new vector<StereoMatPtrPair>);
    pVecStereo2->push_back(lr_pair2);
    auto pframe2 = createFrameStereos(pVecStereo2,cams_conf,create_frame_success_output,false);
    cv::Mat R_,t_;
    bool pnp_ransac_success_output;
    vector<bool> cam_match_success;

    int success_tracked_points_count;
    trackAndDoSolvePnPRansacMultiCam(*pframe1,*pframe2,R_,t_,pnp_ransac_success_output,cam_match_success,&success_tracked_points_count);
    LOG(INFO)<<"pnp_ransac success:"<<pnp_ransac_success_output<<endl;
    for(int i = 0;i<cam_match_success.size();i++)
    {
        LOG(INFO)<<"cam:"<<i<<" match success:"<<cam_match_success[i]<<endl;
    }

    return 0;
}
