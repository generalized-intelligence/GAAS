#include <glog/logging.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include "FeatureFrontEndCV.h"


using namespace mcs;
using namespace chrono;
int main(int argc,char** argv)
{
    //test optflow.
    //step<1>.test frame wise tracking.
    shared_ptr<cvMat_T> prev_img(new cvMat_T),next_img(new cvMat_T);
    #if USE_UMAT_AS_TYPE == 1
    cv::imread("data/prev.png").copyTo(*prev_img);
    cv::imread("data/next.png").copyTo(*next_img);
    #else
    prev_img =  shared_ptr<cvMat_T>(new cvMat_T(cv::imread("data/prev.png")));
    next_img =  shared_ptr<cvMat_T>(new cvMat_T(cv::imread("data/next.png")));
    #endif
    LOG(INFO)<<"col:"<<prev_img->cols<<",row:"<<prev_img->rows<<endl;

    std::chrono::time_point<std::chrono::steady_clock> t_start,t_start2,t_start3,t_1,t_2,t_3;
    t_start = std::chrono::steady_clock::now();

    shared_ptr<mcs::PointWithFeatureT> pPWFT = mcs::extractCamKeyPoints(*prev_img,mcs::KEYPOINT_METHOD_GFTT,false);
    t_1 = std::chrono::steady_clock::now();

    vector<cv::Point2f> kp_original_tracked;
    vector<cv::Point2f> kp_output_tracked;
    vector<unsigned char> kp_output_track_success;
    bool optflow_tracking_success;
    map<int,int> tracked_to_original_map;
    mcs::OptFlowForFrameWiseTracking(*prev_img,*next_img,pPWFT->kps,kp_original_tracked,
                                         kp_output_tracked,tracked_to_original_map,kp_output_track_success,optflow_tracking_success);

    t_2 = std::chrono::steady_clock::now();

    auto cost = duration_cast<std::chrono::duration<double> >(t_2 - t_1);
    auto total_cost = duration_cast<std::chrono::duration<double> >(t_2 - t_start);
    LOG(INFO)<<"Extract and Optflow Track1 time cost:"<<total_cost.count()*1000<<"ms."<<endl;
    LOG(INFO)<<"Optflow Track1 time cost:"<<cost.count()*1000<<"ms."<<endl;
    //step<2>. multithread_operation.

    //#if USE_UMAT_AS_TYPE ==1
    //auto vis_1_mat = prev_img->getMat(ACCESS_READ).clone();
    //#else
    auto vis_1_mat = prev_img->clone();
    //#endif
    const int RADIUS = 2;
    for(int i = 0; i<kp_output_track_success.size();i++)
    {
        cv::circle(vis_1_mat, kp_original_tracked[i], RADIUS, CV_RGB(255, 0, 0), CV_FILLED);
        cv::line(vis_1_mat, kp_output_tracked[i], kp_original_tracked[i], CV_RGB(0, 0, 255));
    }
    //cv::drawMatches (*prev_img, const std::vector< KeyPoint > &keypoints1,*next_img, const std::vector< KeyPoint > &keypoints2, const std::vector< DMatch > &matches1to2, vis_1_mat);
    cv::imshow("1",vis_1_mat);
    cv::imshow("2",*next_img);
    cv::waitKey(0);
    ////step<3>.test stereo frame.
    //cvMat_T stereo_left = cv::imread("left.jpg");
    //cvMat_T stereo_right = cv::imread("right.jpg");
    //mcs::OptFlowForStereoFrame(stereo_left,stereo_right);
    
    
    
    // const Scalar &matchColor=Scalar::all(-1), const Scalar &singlePointColor=Scalar::all(-1), const std::vector< char > &matchesMask=std::vector< char >(), int flags=DrawMatchesFlags::DEFAULT)
    return 0;
}







