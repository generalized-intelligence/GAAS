#ifndef FEATURE_FRONTEND_CV_H
#define FEATURE_FRONTEND_CV_H

using namespace std;
namespace mcs
{
    typedef std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> VecVector2f;
    typedef struct
    {
        Vector2f pt;
        cv::Mat desp;
    }PointWithFeatureT;
    const static int KEYPOINT_METHOD_BRIEF = 0;
    const static int KEYPOINT_METHOD_ORB = 1;

    const static int OPT_FLOW_FARNE_BACK = 0;
    const static int OPT_FLOW_PYRLK = 1;
    shared_ptr<PointWithFeatureT> extractCamKeyPoints(cv::Mat& Img,const int method = KEYPOINT_METHOD_BRIEF)
    {
        cv::Mat mask;
        auto pframeinfo = shared_ptr<FrameInfo>(new FrameInfo);

        cv::Ptr<cv::ORB> orb;
        orb = cv::ORB::create(1000);
        orb->detectAndCompute(image, mask, pframeinfo->keypoints, pframeinfo->descriptors);

    }

    vector<shared_ptr<PointWithFeatureT> > extractMultipleCamKeyPoints(vector<shared_ptr<cv::Mat> > pImgsL) // extract left images of all cam-pairs.
    {
        
    }
    vector<shared_ptr<cv::Mat> > extractMultipleCamKeyPointsMultiThread(vector<shared_ptr<cv::Mat> > pImgsL) // extract left images of all cam-pairs.
    {
        
    }

    
    void calcOptFlowFor1Pair(cv::Mat& imgl,cv::Mat& imgr,VecVector2f& refPts,VecVector2f& trackedPts) //just like ygz slam LKFlowCV().
    {
    }
    void cvOptFlowLR(vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > imgPairs,
                           shared_ptr<VecVector2f> vrefPts ,shared_ptr<VecVector2f> vtrackedPts)
    {
        
    }
    void cvOptFlowLRMultiThread(vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > imgPairs,
                           shared_ptr<VecVector2f> vrefPts ,shared_ptr<VecVector2f> vtrackedPts)
    {
        
    }
    void TriangulateImgsViaOptFlow(cv::Mat& refImg,cv::Mat& currentImg,bool& success,const int method = OPT_FLOW_PYRLK);







}


#endif
