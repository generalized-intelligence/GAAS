#ifndef FEATURE_FRONTEND_CV_H
#define FEATURE_FRONTEND_CV_H

#include "TypeDefs.h"
#include <glog/logging.h>
#include "Frame.h"
#include <thread>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d.hpp"

namespace mcs
{
    using namespace std;
    using namespace cv;
    /*
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
    */
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
    {//TODO:add kps distribution check.
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
            pfeat->detect(Img,pResult->kps,mask);
            orb->compute(Img,pResult->kps,pResult->desp);// calc orb feat.
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

    */
    void OptFlowForFrameWiseTracking(cvMat_T& prev_img,cvMat_T& next_img,vector<KeyPoint>& priorior_kps,vector<Point2f>& output_original_p2f, vector<Point2f>& output_tracked_p2f,map<int,int>& tracked_point_index_to_prev_kps_index,vector<unsigned char>& v_track_success,bool& output_track_success)
    {
        output_track_success = false;
        tracked_point_index_to_prev_kps_index.clear();
        vector<float> err;
        vector<Point2f>& origin_pts_success = output_original_p2f;
        vector<Point2f>& tracked_pts_success = output_tracked_p2f;
        vector<Point2f> origin_p2f;
        vector<Point2f> tracked_p2f;
        cv::KeyPoint::convert(priorior_kps,origin_p2f);
        cv::calcOpticalFlowPyrLK(prev_img,next_img,origin_p2f,tracked_p2f,v_track_success,err,cv::Size(21, 21), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

        for(int i=0;i<priorior_kps.size();i++)
        {
            if(v_track_success[i])
            {
                tracked_point_index_to_prev_kps_index[i] = origin_pts_success.size();
                origin_pts_success.push_back(origin_p2f[i]);
                tracked_pts_success.push_back(tracked_p2f[i]);
            }
        }
        vector<unsigned char> match_success;
        cv::Mat fundMat = cv::findFundamentalMat(origin_pts_success,tracked_pts_success,match_success);
        int goodMatches_count = 0;
        for(int i = 0;i<match_success.size();i++)
        {
            if(match_success[i])
            {
                goodMatches_count += 1;
            }
            else
            {
                tracked_point_index_to_prev_kps_index[i] = -1;
            }
        }
        LOG(INFO)<<"Good match count/Total match count:"<<goodMatches_count<<"/"<<match_success.size()<<", Fundamental Mat:\n"<<fundMat<<endl;
        if(goodMatches_count>=10)
        {
            output_track_success = true;
            return;
        }
    }



    //---------

    inline bool check_stereo_match(Point2f& p1,Point2f& p2)
    {
        //const float diff_v_max = 2.0;
        const float diff_v_max = 4.0;
        //const diff_u_max = 100;//TODO:
        if  (
              abs(p1.y - p2.y) < diff_v_max
                // && p1.x-p2.x <
            )
        {
            return true;
        }
    }
    void createStereoMatchViaOptFlowMatching(cvMat_T& l,cvMat_T& r,StereoCamConfig& cam_info,vector<p2dT>& p2d_output,vector<p3dT>& p3d_output,map<int,int>& map_2d_to_3d_pts,map<int,int>& map_3d_to_2d_pts,bool& output_create_success)//for this is a multi-thread usage function, we let it return void.
    {
        LOG(INFO)<<"In createStereoMatchViaOptFlowMatching:extracting features."<<endl;
        shared_ptr<mcs::PointWithFeatureT> pPWFT_l_img = mcs::extractCamKeyPoints(l,mcs::KEYPOINT_METHOD_GFTT,false);
        LOG(INFO)<<"In createStereoMatchViaOptFlowMatching:kp.size():"<<pPWFT_l_img->kps.size()<<endl;
        vector<Point2f> l_p2fs_original;
        cv::KeyPoint::convert(pPWFT_l_img->kps,l_p2fs_original);
        vector<Point2f> tracked_unchecked;
        vector<unsigned char> v_track_success;
        vector<float> err;
        LOG(INFO)<<"In createStereoMatchViaOptFlowMatching:starting OptFlow."<<endl;
        cv::calcOpticalFlowPyrLK(l,r,l_p2fs_original,tracked_unchecked,v_track_success,err,cv::Size(21, 21), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01)
                                 );
         
        //p2d_output = shared_ptr<vector<p2dT> >(new vector<p2dT>);
        //p3d_output = shared_ptr<vector<p3dT> >(new vector<p3dT>);
        vector<Point2f>& checked_l = p2d_output;
        vector<Point2f> checked_r;
        LOG(INFO)<<"checking vtrack_success"<<endl;
        for(int i = 0;i<v_track_success.size();i++)
        {
            if(v_track_success[i] && check_stereo_match(l_p2fs_original[i],tracked_unchecked[i]))// check v.diff(v) shall be smaller than 4
            {
                //LOG(INFO)<<"track success,dealing with 3d reconstruction."<<endl;
                checked_l.push_back(l_p2fs_original[i]);
                checked_r.push_back(tracked_unchecked[i]);
                float disparity = tracked_unchecked[i].x - l_p2fs_original[i].x;
                float x,y,z,scale;
                //calc p3d.
                const auto &QMat = cam_info.getQMat();
                //LOG(INFO)<<"Q_mat:\n"<<QMat<<endl;
                //LOG(INFO)<<"Q_mat dtype:"<<QMat.type()<<";cv_32f is:"<<CV_32F<<"cv_64f is:"<<CV_64F<<endl;
                scale = 1.0/(disparity*QMat.at<float>(3,2)+QMat.at<float>(3,3));
                x = (l_p2fs_original[i].x + QMat.at<float>(0,3))*scale;
                y = (l_p2fs_original[i].y + QMat.at<float>(1,3))*scale;
                z = QMat.at<float>(2, 3)*scale;
                //Point3f p3f_prev(x,y,z);
                cv::Mat matp1(4,1,CV_32F);
                matp1.at<float>(0,0)= x;
                matp1.at<float>(1,0)= y;
                matp1.at<float>(2,0)= z;
                matp1.at<float>(3,0)= 1.0;
                //LOG(INFO)<<"RT_mat:\n"<<cam_info.getRTMat()<<endl;
                matp1 = cam_info.getRTMat()*matp1;
                //Transformation of rt mat.
                x = matp1.at<float>(0,0);
                y = matp1.at<float>(1,0);
                z = matp1.at<float>(2,0);
                map_2d_to_3d_pts[i] = p3d_output.size();
                map_3d_to_2d_pts[p3d_output.size()] = i;
                p3d_output.push_back(Point3f(x,y,z));
            }
        }
        LOG(INFO)<<"In createStereoMatchViaOptFlowMatching:points matched success num:"<<checked_l.size()<<endl;
        if(checked_l.size()>15)
        {
            output_create_success = true;
        }
    }
    void createStereoMatchViaOptFlowMatching_MultiThread();//TODO
    shared_ptr<Frame> createFrameStereos(shared_ptr<vector<StereoMatPtrPair> >stereo_pair_imgs_vec,
                                      vector<StereoCamConfig>& cam_distribution_info_vec,
                                      bool& create_Frame_success,
                                      bool create_key_frame = true)
    {

        //method<1> single thread.
        shared_ptr<Frame> pF_ret(new Frame(stereo_pair_imgs_vec));
        pF_ret->frame_type = FRAME_TYPE_STEREO;
        pF_ret->cam_info_stereo_vec = cam_distribution_info_vec;
        pF_ret->p2d_vv.resize(stereo_pair_imgs_vec->size());
        pF_ret->p3d_vv.resize(stereo_pair_imgs_vec->size());
        for(int ci_index=0;ci_index<cam_distribution_info_vec.size();ci_index++)
        {
            pF_ret->cam_info_vec.push_back(static_cast<CamInfo&>(cam_distribution_info_vec[ci_index]));
        }
        for(int i = 0;i<stereo_pair_imgs_vec->size();i++ )
        {
             auto& p =  (*stereo_pair_imgs_vec)[i];
             cvMat_T& l = *(std::get<0>(p));
             cvMat_T& r = *(std::get<1>(p));
             //shared_ptr<vector<p2dT> > p2d_output_;
             //shared_ptr<vector<p3dT> > p3d_output_;
             bool create_stereo_successs = false;
             if(create_key_frame)
             {
                 map<int,int> kps_2d_to_3d,kps_3d_to_2d;
                 createStereoMatchViaOptFlowMatching(l,r,cam_distribution_info_vec[i],pF_ret->p2d_vv[i],pF_ret->p3d_vv[i],kps_2d_to_3d,kps_3d_to_2d,create_stereo_successs);
                 pF_ret->map2d_to_3d_pt_vec.push_back(kps_2d_to_3d);
                 pF_ret->map3d_to_2d_pt_vec.push_back(kps_3d_to_2d);
                 if(!create_stereo_successs)
                 {
                     create_Frame_success = false;
                 }
             }
             else
             {//maybe detect gftt for optflow check is needed.TODO.
                 LOG(ERROR)<<"TODO:"<<endl;
             }
        }
        return pF_ret;



        
    }
    //void createFrameStereos(shared_ptr<vector<StereoMatPtrPair> >stereo_pair_imgs_vec,
    //                        vector<StereoCamConfig>& cam_distribution_info_vec,
    //                        bool& create_Frame_success,
    //                        bool create_key_frame = true)
    //{
    //method<2> multiple thread.
    //
    // createStereoMatchViaOptFlowMatching(stereo_pair_imgs_vec);
    //不要打乱p2dT,p3dT的顺序!这里可能未来有一个摄像机整体信息的丢弃处理.

    //pF_ret->p2d_vv =
    shared_ptr<Frame> createFrameDepth(vector<shared_ptr<cv::Mat> > pOriginalImgs,
                                      vector<shared_ptr<cv::Mat>>pDepthImgs,
                                      vector<CamInfo>& cam_distribution_info_vec,
                                      bool create_Frame_success = false,
                                      bool create_key_frame = true);

}


#endif
