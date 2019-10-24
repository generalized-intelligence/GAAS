#include "FeatureFrontEndCV.h"

int main(int argc,char** argv)
{
    //test optflow.
    //step<1>.test frame wise tracking.
    cvMat prev_img = cv::imread("prev.jpg");
    cvMat next_img = cv::imread("next.jpg");
    mcs::PointWithFeatureT* pPWFT = mcs::extractCamKeyPoints(prev_img,mcs::KEYPOINT_METHOD_BRIEF,false);
    vector<KeyPoint> kp_output_tracked;
    vector<unsigned char> kp_output_track_success;
    mcs::OptFlowForFrameWiseTracking(prev_img,next_img,pPWFT->kps,kp_output_tracked,kp_output_track_success);

    cv::drawMatches (InputArray img1, const std::vector< KeyPoint > &keypoints1, InputArray img2, const std::vector< KeyPoint > &keypoints2, const std::vector< DMatch > &matches1to2, InputOutputArray outImg);
    cv::imshow()...
    //step<2>.test stereo frame.
    cvMat stereo_left = cv::imread("left.jpg");
    cvMat stereo_right = cv::imread("right.jpg");
    mcs::OptFlowForStereoFrame(stereo_left,stereo_right);
    
    
    
    // const Scalar &matchColor=Scalar::all(-1), const Scalar &singlePointColor=Scalar::all(-1), const std::vector< char > &matchesMask=std::vector< char >(), int flags=DrawMatchesFlags::DEFAULT)
    return 0;
}







