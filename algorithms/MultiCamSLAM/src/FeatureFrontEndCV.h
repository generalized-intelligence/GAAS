#ifndef FEATURE_FRONTEND_CV_H
#define FEATURE_FRONTEND_CV_H

#include "Frame.h"
#include <thread>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#define USE_UMAT_AS_TYPE 1

namespace mcs
{
    using namespace std;
    using namespace cv;
    //part0 typedefs.
    typedef std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> > VecVector2f;


    #if USE_UMAT_AS_TYPE == 1
    typedef cv::UMat cvMat_T; // use UMat to enable Transparent API and OpenCL acceleration.
    #else
    typedef cv::Mat cvMat_T;
    #endif
    typedef struct
    {
        std::vector<KeyPoint> kps;
        cvMat_T desp;
    }PointWithFeatureT;
    const static int KEYPOINT_METHOD_GFTT = 0;
    const static int KEYPOINT_METHOD_ORB = 1;

    const static int OPT_FLOW_FARNE_BACK = 0;
    const static int OPT_FLOW_PYRLK = 1;
    const static cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
    //const static auto brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
    const static auto gftt = cv::GFTTDetector::create(500,  // maximum number of corners to be returned
                                         0.01, // quality level
10); // minimum allowed distance between points
    //part1 kps,desps.
    shared_ptr<PointWithFeatureT> extractCamKeyPoints(cvMat_T& Img,int method = KEYPOINT_METHOD_GFTT,bool compute_feature = false);
    //inline void extractCamKeyPoints_(cvMat_T& Img,PointWithFeatureT* feat ,int method = KEYPOINT_METHOD_GFTT,bool compute_feature = false);//for parallel

    vector<shared_ptr<PointWithFeatureT> > extractMultipleCamKeyPoints(vector<shared_ptr<cvMat_T> > pImgsL,int method = KEYPOINT_METHOD_GFTT,bool compute_feature = false); // extract left images of all cam-pairs.
    vector<shared_ptr<PointWithFeatureT> > extractMultipleCamKeyPointsMultiThread(vector<shared_ptr<cvMat_T> > pImgsL,int method = KEYPOINT_METHOD_GFTT,bool compute_feature = false); // extract left images of all cam-pairs.


    shared_ptr<PointWithFeatureT> extractCamKeyPoints(cvMat_T& Img,int method,bool compute_feature)
    {
        LOG(INFO)<<"method:"<<method<<";compute_feature:"<<compute_feature<<endl;
        cvMat_T mask;
        //auto pframeinfo = shared_ptr<FrameInfo>(new FrameInfo);
        //cv::Ptr<cv::ORB> orb;
        //orb = cv::ORB::create(1000);
        cv::Feature2D* pfeat;
        shared_ptr<PointWithFeatureT> pResult(new PointWithFeatureT); 
        if(method == KEYPOINT_METHOD_GFTT)
        {
            pfeat = gftt;
        }
        else if(method == KEYPOINT_METHOD_ORB)
        {
            pfeat = orb;
            //orb->detectAndCompute(image, mask, pframeinfo->keypoints, pframeinfo->descriptors);
        }

        else
        {
            LOG(ERROR)<<"method undefined."<<endl;
            return nullptr;
        }
        if(!compute_feature)
        {
            pfeat->detect(Img,pResult->kps,mask);
        }
        else
        {
            pfeat->detectAndCompute(Img,mask,pResult->kps,pResult->desp);
        }
        return pResult;
    }
    
    inline void extractCamKeyPoints_(cvMat_T& Img,shared_ptr<PointWithFeatureT>* feat ,int method,bool compute_feature)//for parallel
    {
        *feat = extractCamKeyPoints(Img,method,compute_feature);
    }
    
    vector<shared_ptr<PointWithFeatureT> > extractMultipleCamKeyPoints(vector<shared_ptr<cvMat_T> > pImgsL,int method,bool compute_feature ) // extract left images of all cam-pairs.
    {
        vector<shared_ptr<PointWithFeatureT> > retVal;
        for(int i = 0;i < pImgsL.size();i++)
        {
            retVal.push_back(extractCamKeyPoints(*(pImgsL[i])) );
        }
        return retVal;
    }
    vector<shared_ptr<PointWithFeatureT> > extractMultipleCamKeyPointsMultiThread(vector<shared_ptr<cvMat_T> > pImgsL,int method ,bool compute_feature) // extract left images of all cam-pairs.
    {
        vector<std::thread> threads;
        //vector<PointWithFeatureT* > feats_output;
        vector<shared_ptr<PointWithFeatureT> > feats_output;
        feats_output.resize(pImgsL.size());
        for(int i = 0;i<pImgsL.size();i++)
        {
            threads.push_back(std::thread(extractCamKeyPoints_,std::ref(*(pImgsL[i]) ), &(feats_output[i]) , std::ref(method), std::ref(compute_feature )));
        }
        for(int i = 0;i<pImgsL.size();i++)
        {
            threads[i].join();
        }
        return feats_output;
    }
    //part2 optflow.
    //-----

/*

    void TriangulateImgsViaOptFlow(cvMat_T& refImg,cvMat_T& currentImg,bool& success,const int method = OPT_FLOW_PYRLK);
    {

    }

    void calcOptFlowFor1Pair(cvMat_T* imgl,cvMat_T* imgr, // nothing about memory malloc,use basic ptr.
                               vector<KeyPoint>& output_kp_l,vector<KeyPoint>& output_kp_r)
    {
        auto pfeat_l = extractCamKeyPoints(*imgl);
        //auto pfeat_r = extractCamKeyPoints(*imgr);
        vector<KeyPoint> output_kps;
        vector<unsigned char> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(*imgl,*imgr,pfeat_l,output_kps,status,err,cv::Size(21, 21), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
    }
    void calcOpticalFlowPyrLKMultipleImgs(vector<shared_ptr<cvMat_T> > pImgsL,vector<shared_ptr<cvMat_T> > pImgsR,vector<vector<KeyPoint> >output_kp_ls,vector<vector<KeyPoint> > output_kp_rs);
    void calcOpticalFlowPyrLKAndInitiate3DMapPoint(cvMat_T* imgl,cvMat_T* imgr, // nothing about memory malloc,use basic ptr.
                               vector<KeyPoint>& output_kp_l,vector<KeyPoint>& output_kp_r,vector<Point3f>& output_map_point_3d);
    //void calcOptFlowFor1Pair(cvMat_T& imgl,cvMat_T& imgr,VecVector2f& refPts,VecVector2f& trackedPts) //just like ygz slam LKFlowCV().
    //{
    //    shared_ptr<PointWithFeatureT> pFeat = extractCamKeyPoints(imgl);
    //}
    //void cvOptFlowLR(vector<std::pair<shared_ptr<cvMat_T>,shared_ptr<cvMat_T> > > imgPairs,
    //                       shared_ptr<VecVector2f> vrefPts ,shared_ptr<VecVector2f> vtrackedPts)
    //{
    //    
    //}
    void cvOptFlowLRMultiThread(vector<std::pair<shared_ptr<cvMat_T>,shared_ptr<cvMat_T> > > imgPairs,
                           shared_ptr<VecVector2f> vrefPts ,shared_ptr<VecVector2f> vtrackedPts)
    {
        
    }
 
    void OptFlowForStereoFrame();
    void OptFlowForFrameWiseTracking(cvMat_T& prev_img,cvMat_T& next_img,vector<KeyPoint>& priorior_kps,vector<KeyPoint>& output_kps,vector<unsigned char>& v_track_success,bool& output_track_success);
    void OptFlowForFrameWiseTracking(vector<shared_ptr<cvMat_T> > prev_imgs,vector<shared_ptr<cvMat_T> > next_imgs,vector<vector<KeyPoint> >& priorior_kps_vec,vector<vector<KeyPoint> >& output_kps_vec,vector<vector<unsigned char> >& v_track_success_vec,vector<bool&> output_track_success_vec);

    void OptFlowForFrameWiseTracking(cvMat_T& prev_img,cvMat_T& next_img,vector<KeyPoint>& priorior_kps,vector<KeyPoint>& output_kps,vector<unsigned char>& v_track_success,bool& output_track_success)
    {
        output_track_success = false;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_img,next_img,priorior_kps,output_kps,v_track_success,err,cv::Size(21, 21), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        vector<Point2f> origin_pts_success;
        vector<Point2f> tracked_pts_success;
        for(int i=0;i<priorior_kps.size();i++)
        {
            if(v_track_success[i])
            {
                origin_pts_success.push_back(priorior_kps[i].pt)
                tracked_pts_success.push_back(output_kps[i].pt);
            }
        }
        vector<bool> match_success;
        cv::findFundamentalMat(origin_pts_success,tracked_pts_success,match_success);
        int goodMatches_count = 0;
        for(int i = 0;i<match_success.size();i++)
        {
            if(match_success[i])
            {
                goodMatches_count += 1;
            }
        }
        if(goodMatches_count>=10)
        {
            output_track_success = true;
            return;
        }
    }
    */


}


#endif
