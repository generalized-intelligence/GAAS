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
#include "utils/Timer.h"

namespace mcs
{
    void do_cvPyrLK(InputOutputArray prev,InputOutputArray next,vector<Point2f>& p2d_v_prev,vector<Point2f>& p2d_v_next,
                    vector<unsigned char>& track_success_v,vector<float>& err,bool do_backward_check = true);

    using namespace std;
    using namespace cv;
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


    shared_ptr<PointWithFeatureT> extractCamKeyPoints_splited(cvMat_T& Img,int method,bool compute_feature);

    const bool do_img_split = true;
    //const bool do_img_split = false;
    shared_ptr<PointWithFeatureT> extractCamKeyPoints(cvMat_T& Img,int method,bool compute_feature)
    {//TODO:add kps distribution check.
        if(do_img_split)
        {
            return extractCamKeyPoints_splited(Img,method,compute_feature);
        }

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
            ;//pfeat = orb;
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
            LOG(INFO)<<"feats.size():"<<pResult->desp.rows<<","<<pResult->desp.cols<<endl;
        }
        return pResult;
    }

    const int EXTRACT_COUNT_EACH_BLOCK = 20;//40;
    shared_ptr<PointWithFeatureT> extractCamKeyPoints_splited(cvMat_T& Img,int method,bool compute_feature)
    {
        LOG(INFO)<<"in extractCamKeyPoints_splited(),method and compute_feat:"<<method<<","<<compute_feature<<endl;
        //cv::Feature2D* pfeat = gftt;
        auto gftt = cv::GFTTDetector::create(EXTRACT_COUNT_EACH_BLOCK,//50,  // maximum number of corners to be returned
                                                       0.01, // quality level
              10);
        auto orb_ex= orb;cv::ORB::create(1000);
        cv::Feature2D* pfeat;
        //auto orb_ex = orb;
        //LOG(INFO)<<"feat2d ptr created."<<endl;
        if(method == KEYPOINT_METHOD_GFTT)
        {
            pfeat = gftt;
        }
        else if(method == KEYPOINT_METHOD_ORB)
        {
            pfeat = orb_ex;
        }
        //const int rows_count = 5;
        //const int cols_count = 5;
        const int rows_count = 3;
        const int cols_count = 3;
        const int img_size_v = Img.rows;
        const int img_size_u = Img.cols;
        shared_ptr<PointWithFeatureT> pResult(new PointWithFeatureT);
        vector<KeyPoint> out_kps;
        cvMat_T out_feats;
        vector<cvMat_T> desps_vec;
        //LOG(INFO)<<"input image size:"<<Img.cols<<","<<Img.rows<<endl;
        for(int v_ = 0;v_ < rows_count;v_++)
        {
            for(int u_ = 0; u_ < cols_count;u_++)
            {
                //LOG(INFO)<<"        creating img crops.u,v:"<<u_<<","<<v_<<endl;
                auto img_part = Img.colRange(u_*(img_size_u/cols_count),((u_+1)*(img_size_u/cols_count))>(img_size_u-1)?(img_size_u-1):(u_+1)*(img_size_u/cols_count)
                                             ).rowRange(
                                             v_*(img_size_v/rows_count),((v_+1)*(img_size_v/rows_count))>(img_size_v-1)?(img_size_v-1):(v_+1)*(img_size_v/rows_count)
                                            );
                vector<KeyPoint> kps;
                cvMat_T feats;
                cvMat_T mask;
                //LOG(INFO)<<"        input_size:"<<img_part.cols<<","<<img_part.rows<<endl;
                //LOG(INFO)<<"        detect kps."<<endl;
                pfeat->detect(img_part,kps,mask);
                //LOG(INFO)<<"        kps.size:"<<kps.size()<<endl;
                if(compute_feature&&kps.size()>0)
                {
                    //LOG(INFO)<<"        compute desps."<<endl;
                    orb_ex->compute(img_part,kps,feats);
                }
                for(auto& kp:kps)
                {
                    //LOG(INFO)<<"original_pos_x"<<kp.pt.x<<endl;
                    kp.pt.x += u_*(img_size_u/cols_count);
                    kp.pt.y += v_*(img_size_v/rows_count);
                    //LOG(INFO)<<"transformed_pos_x"<<kp.pt.x<<endl;
                    out_kps.push_back(kp);
                }
                //extend cvmat feature:
                //LOG(INFO)<<"in extractCamKeyPoints_splited:u="<<u_<<" v="<<v_<<"input_size:"<<img_part.cols<<","<<img_part.rows<<";kps.size():"<<kps.size()<<",feats.size():"<<feats.cols<<","<<feats.rows<<endl;
                //cv::Mat feat_tmp;
                //cv::vconcat(out_feats,feats,feat_tmp);//TODO:vconcat()还是hconcat()?
                //out_feats = feat_tmp;


                //if(!feats.empty())
                //{
                //    //desps_vec.push_back(feats);
                //    cv::Mat feat_tmp;
                //    cv::vconcat(out_feats,feats,feat_tmp);
                //    out_feats = feat_tmp.clone();
                //}
                if(!feats.empty())
                {
                    desps_vec.push_back(feats);
                }
            }
        }
        cv::vconcat(desps_vec,out_feats);
        LOG(INFO)<<"output_feats.size():"<<out_feats.cols<<","<<out_feats.rows<<";kps.size():"<<out_kps.size()<<endl;
        pResult->kps = out_kps;
        pResult->desp= out_feats;
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

//正反向光流验证参考.
//        p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
//        p1, _st, _err = cv.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)//先正向,再反向.
//        p0r, _st, _err = cv.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
//        d = abs(p0-p0r).reshape(-1, 2).max(-1)
//        good = d < 1
//        new_tracks = []
//        for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
//            if not good_flag:
//                continue
//            tr.append((x, y))
//            if len(tr) > self.track_len:
//                del tr[0]
//            new_tracks.append(tr)
//            cv.circle(vis, (x, y), 2, (0, 255, 0), -1)
//        self.tracks = new_tracks
//        cv.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))

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



    const float diff_v_max = 2.0;
    //---------
    inline bool check_stereo_v(Point2f& p1,Point2f& p2)
    {
        if(abs(p1.y - p2.y) < diff_v_max && p2.x-p1.x >= 0.0)
        {
            return true;
        }
        return false;
    }
    inline bool check_stereo_match(Point2f& p1,Point2f& p2)
    {
        //const float diff_v_max = 2.0;

        //const diff_u_max = 100;//TODO:
        if  (
              abs(p1.y - p2.y) < diff_v_max
             &&  p1.x<p2.x
             //&&  p2.x-p1.x > 10.0 // minimum disparity :2 //TODO:set into config file.20 太大....10试试?
             &&  p2.x-p1.x > 4.0 // minimum disparity :2 //TODO:set into config file.10 有点大.4试试?
                // && p1.x-p2.x <
            )
        {
            return true;
        }
        if(abs(p1.y - p2.y) >= diff_v_max)
        {
            cout<<"    diff_v:"<<abs(p1.y - p2.y)<<" larger than vmax!"<<endl;
        }
        return false;
    }
    Eigen::Vector4d triangulatePoint(double disp,double u,double v,double b,double camfx,double camfy,double camcx,double camcy)
    {
        //b = 0.12;//DEBUG ONLY!!!
        double x,y,z;
        z = b*camfx/(disp);
        x = z*(u - camcx) / camfx;
        y = z*(v - camcy) / camfy;
        return Eigen::Vector4d(x,y,z,1.0);
    }
    void createStereoMatchViaOptFlowMatching(cvMat_T& l,cvMat_T& r,StereoCamConfig& cam_info,vector<p2dT>& p2d_output,
                                             vector<p3dT>& p3d_output,map<int,int>& map_2d_to_3d_pts,
                                             map<int,int>& map_3d_to_2d_pts,vector<double>& p3d_disparity,
                                             bool& output_create_success)//for this is a multi-thread usage function, we let it return void.
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
        //cv::calcOpticalFlowPyrLK(l,r,l_p2fs_original,tracked_unchecked,v_track_success,err,cv::Size(21, 21), 3,
        //                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01)
        //                         );
        do_cvPyrLK(l,r,l_p2fs_original,tracked_unchecked,v_track_success,err//,cv::Size(21, 21), 3,
                                 //cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01)
                                 );
        //p2d_output = shared_ptr<vector<p2dT> >(new vector<p2dT>);
        //p3d_output = shared_ptr<vector<p3dT> >(new vector<p3dT>);
        vector<Point2f>& checked_l = p2d_output;
        vector<Point2f> checked_r;
        LOG(INFO)<<"checking vtrack_success,l_p2fs_original.size:"<<l_p2fs_original.size()<<",tracked_unchecked.size:"<<tracked_unchecked.size() <<endl;
        for(int i = 0;i<v_track_success.size();i++)
        {
            if(v_track_success[i] && check_stereo_match(tracked_unchecked[i],l_p2fs_original[i]))// check v.diff(v) shall be smaller than 4
            {
                LOG(INFO)<<"        track success,dealing with 3d reconstruction."<<endl;
                checked_l.push_back(l_p2fs_original.at(i));
                checked_r.push_back(tracked_unchecked.at(i));
                float disparity = l_p2fs_original.at(i).x - tracked_unchecked.at(i).x;
                float x,y,z,scale;
                //calc p3d.
                const auto &QMat = cam_info.getQMat();
                float camfx,camfy,camcx,camcy;
                cam_info.getCamMatFxFyCxCy(camfx,camfy,camcx,camcy);
                //LOG(INFO)<<"Q_mat:\n"<<QMat<<endl;
                //LOG(INFO)<<"Q_mat dtype:"<<QMat.type()<<";cv_32f is:"<<CV_32F<<"cv_64f is:"<<CV_64F<<endl;
                LOG(INFO)<<"        disp,q_32,q_33:"<<disparity<<","<<QMat.at<float>(3,2)<<","<<QMat.at<float>(3,3)<<endl;
/*
                scale = 1.0/(disparity*QMat.at<float>(3,2)+QMat.at<float>(3,3));
                x = (l_p2fs_original.at(i).x + QMat.at<float>(0,3))*scale;
                y = (l_p2fs_original.at(i).y + QMat.at<float>(1,3))*scale;
                z = QMat.at<float>(2, 3)*scale;

                //Point3f p3f_prev(x,y,z);
                 cv::Mat matp1(4,1,CV_32F);
                matp1.at<float>(0,0)= x;
                matp1.at<float>(1,0)= y;
                matp1.at<float>(2,0)= z;
                matp1.at<float>(3,0)= 1.0;
                //LOG(INFO)<<"        RT_mat:\n"<<cam_info.getRTMat()<<endl;
                matp1 = cam_info.getRTMat()*matp1;
                //Transformation of rt mat.
                x = matp1.at<float>(0,0);
                y = matp1.at<float>(1,0);
                z = matp1.at<float>(2,0);
*/
                float b = 0.12;//DEBUG ONLY!!!
                z = b*camfx/(disparity);
                x = z*(l_p2fs_original.at(i).x - camcx) / camfx;
                y = z*(l_p2fs_original.at(i).y - camcy) / camfy;
                cv::Mat matp1(4,1,CV_32F);
                matp1.at<float>(0,0)= x;
                matp1.at<float>(1,0)= y;
                matp1.at<float>(2,0)= z;
                matp1.at<float>(3,0)= 1.0;
                LOG(INFO)<<"original triangulated 3d pt:"<<matp1<<endl;
                //LOG(INFO)<<"        RT_mat:\n"<<cam_info.getRTMat()<<endl;
                matp1 = cam_info.getRTMat()*matp1;

                x = matp1.at<float>(0,0);
                y = matp1.at<float>(1,0);
                z = matp1.at<float>(2,0);



                //fixed overflow.
                //map_2d_to_3d_pts.at(checked_l.size()-1) = p3d_output.size();
                map_2d_to_3d_pts.insert(std::pair<int,int>(checked_l.size()-1,p3d_output.size()));
                //map_3d_to_2d_pts.at(p3d_output.size()) = checked_l.size()-1;
                map_3d_to_2d_pts.insert(std::pair<int,int>(p3d_output.size(),checked_l.size()-1));
                p3d_disparity.push_back((double)disparity);
                p3d_output.push_back(p3dT(x,y,z));
            }
        }
        if(p3d_disparity.size()==0)
        {
            LOG(ERROR)<<"p3d is empty. check your condition of p3d calc!"<<endl;
        }
        LOG(INFO)<<"In createStereoMatchViaOptFlowMatching:points matched success num:"<<checked_l.size()<<endl;
        if(checked_l.size()>15)
        {
            output_create_success = true;
        }
    }
    void createStereoMatchViaOptFlowMatching_MultiThread();//TODO
    vector<vector<shared_ptr<MapPoint> > > createMapPointForKeyFrame(shared_ptr<Frame> pFrame)
    {
        vector<vector<shared_ptr<MapPoint> > > ret_;
        for(int cam_index = 0;cam_index<pFrame->map2d_to_3d_pt_vec.size();cam_index++)
        {
            vector<shared_ptr <MapPoint> > cam_mps;
            for(int i = 0;i<pFrame->p2d_vv.size();i++)//对每个2d点创建.
            {
                shared_ptr<MapPoint> pPoint(new MapPoint());
                pPoint->feat = cv::Mat();//mat为空,暂时不提取特征.
                //if(pFrame->map2d_to_3d_pt_vec[cam_index].find(i) != pFrame->map2d_to_3d_pt_vec[cam_index].end())
                if(pFrame->map2d_to_3d_pt_vec.at(cam_index).count(i))
                {

                    Point3f P3fMapPoint= pFrame->p3d_vv.at(cam_index).at(pFrame->map2d_to_3d_pt_vec.at(cam_index).at(i) );
                    pPoint->pos = Point3d(P3fMapPoint.x,P3fMapPoint.y,P3fMapPoint.z);//相对创建帧的位置.
                    pPoint->state = MAP_POINT_STATE_MATURE;
                }
                else//点尚未被三角化.
                {
                    pPoint->state = MAP_POINT_STATE_IMMATURE;
                }
                cam_mps.push_back(pPoint);
            }
            ret_.push_back(cam_mps);
        }
        return ret_;
    }
    void upgradeOrdinaryFrameToKeyFrameStereos(shared_ptr<Frame> pOrdinaryFrame)//,bool& create_stereo_success)
    {//升级成关键帧.补充提取gftt,并且加入左右之间的关联关系(p2d,p3d,各种Mapping.).
        //TODO.
        if(pOrdinaryFrame->isKeyFrame)
        {
            return;
        }
        bool create_stereo_success = false;
        pOrdinaryFrame->isKeyFrame = true;
        auto& pF_ret = pOrdinaryFrame;
        for(int i = 0;i<pF_ret->get_cam_num();i++ )//对每组摄像头//TODO:改成多线程.
        //TODO:追加一个策略,保留能保留的Landmark.
        {
             auto& p =  pF_ret->pLRImgs->at(i);
             cvMat_T& l = *(std::get<0>(p));
             cvMat_T& r = *(std::get<1>(p));
             //shared_ptr<vector<p2dT> > p2d_output_;
             //shared_ptr<vector<p3dT> > p3d_output_;
             pF_ret->isKeyFrame = true;
             map<int,int>& kps_2d_to_3d = pF_ret->map2d_to_3d_pt_vec.at(i);
             map<int,int>& kps_3d_to_2d = pF_ret->map3d_to_2d_pt_vec.at(i);
             vector<double>& disps = pF_ret->disps_vv.at(i);
             LOG(INFO)<<"will create kf_stereo :"<<i<<endl;
             createStereoMatchViaOptFlowMatching(l,r,pF_ret->cam_info_stereo_vec[i],pF_ret->p2d_vv.at(i),pF_ret->p3d_vv.at(i),kps_2d_to_3d,kps_3d_to_2d,disps,create_stereo_success);
             if(!create_stereo_success)
             {
                 LOG(WARNING)<<"Created Stereo KF in cam "<<i<<" Failed!"<<endl;//TODO.
             }
             LOG(INFO)<<"create kf_stereo "<<i<<" finished;p2d_size:"<<pF_ret->p2d_vv.at(i).size()<<endl;
             pF_ret->map_points =  createMapPointForKeyFrame(pF_ret);
        }
//        if(pimu_info!= nullptr)
//        {
//            pF_ret->imu_info_vec = *pimu_info;
//        }
//        //return pF_ret;
    }
    shared_ptr<Frame> createFrameStereos(shared_ptr<vector<StereoMatPtrPair> >stereo_pair_imgs_vec,
                                      vector<StereoCamConfig>& cam_distribution_info_vec,
                                      bool& create_Frame_success,
                                      bool create_key_frame = true,vector<IMU_Data_T> *pimu_info=nullptr)
    {
        ScopeTimer t_("createFrameStereos()");
        LOG(INFO)<<"in createFrameStereos():create_key_frame = "<<create_key_frame<<endl;
        if(cam_distribution_info_vec.size()!= stereo_pair_imgs_vec->size())
        {
            throw "in createFrameStereos:size of cam conf and image pairs differs!";
        }
        create_Frame_success = false;
        //method<1> single thread.
        shared_ptr<Frame> pF_ret(new Frame(stereo_pair_imgs_vec));
        pF_ret->frame_type = FRAME_TYPE_STEREO;
        pF_ret->cam_info_stereo_vec = cam_distribution_info_vec;
        pF_ret->p2d_vv.resize(stereo_pair_imgs_vec->size());
        pF_ret->p3d_vv.resize(stereo_pair_imgs_vec->size());
        pF_ret->disps_vv.resize(stereo_pair_imgs_vec->size());
        pF_ret->map2d_to_3d_pt_vec.resize(stereo_pair_imgs_vec->size());
        pF_ret->map3d_to_2d_pt_vec.resize(stereo_pair_imgs_vec->size());
        pF_ret->kf_p2d_to_landmark_id.resize(stereo_pair_imgs_vec->size());
        //for(int ci_index=0;ci_index<cam_distribution_info_vec.size();ci_index++)
        //{
        //    //pF_ret->cam_info_vec.push_back(static_cast<CamInfo&>(cam_distribution_info_vec[ci_index]));
        //    pF_ret->cam_info_stereo_vec.push_back(cam_distribution_info_vec[ci_index]);
        //}
        for(int i = 0;i<stereo_pair_imgs_vec->size();i++ )//对每组摄像头//TODO:改成多线程.
        //TODO:追加一个策略,保留能保留的Landmark.
        {
             auto& p =  stereo_pair_imgs_vec->at(i);
             cvMat_T& l = *(std::get<0>(p));
             cvMat_T& r = *(std::get<1>(p));
             //shared_ptr<vector<p2dT> > p2d_output_;
             //shared_ptr<vector<p3dT> > p3d_output_;
             bool create_stereo_successs = false;
             if(create_key_frame)
             {
                 pF_ret->isKeyFrame = true;
                 map<int,int>& kps_2d_to_3d = pF_ret->map2d_to_3d_pt_vec.at(i);
                 map<int,int>& kps_3d_to_2d = pF_ret->map3d_to_2d_pt_vec.at(i);
                 vector<double>& disps = pF_ret->disps_vv.at(i);
                 LOG(INFO)<<"will create kf_stereo :"<<i<<endl;
                 createStereoMatchViaOptFlowMatching(l,r,cam_distribution_info_vec[i],pF_ret->p2d_vv.at(i),pF_ret->p3d_vv.at(i),kps_2d_to_3d,kps_3d_to_2d,disps,create_stereo_successs);

                 LOG(INFO)<<"create kf_stereo "<<i<<" finished;p2d_size:"<<pF_ret->p2d_vv.at(i).size()<<endl;

                 pF_ret->map_points =  createMapPointForKeyFrame(pF_ret);
                 if(!create_stereo_successs)
                 {
                     create_Frame_success = false;
                 }
                 else
                 {
                     create_Frame_success = true;
                 }
             }
             else
             {//maybe detect gftt for optflow check is needed for opt check.TODO.
                 //LOG(ERROR)<<"TODO:"<<endl;
                 LOG(INFO)<<"TODO:"<<endl;
             }
        }
        if(pimu_info!= nullptr)
        {
            pF_ret->imu_info_vec = *pimu_info;
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

    float pt_calc_dist(const Point2f& p1,const Point2f& p2)
    {
        return pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2);
    }

    void do_cvPyrLK(InputOutputArray prev,InputOutputArray next,vector<Point2f>& p2d_v_prev,vector<Point2f>& p2d_v_next,vector<unsigned char>& track_success_v,vector<float>& err,bool do_backward_check)
    {
        ScopeTimer do_pyrlk_timer("        do_cvPyrLK()");
        cv::calcOpticalFlowPyrLK(prev,next,p2d_v_prev,p2d_v_next,track_success_v,err,cv::Size(21, 21), 3,
                                  cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        vector<unsigned char> backward_check_output = track_success_v;
        vector<unsigned char> final_output = track_success_v;

        LOG(INFO)<<"        To track 2d pts size:"<<p2d_v_prev.size();
        do_pyrlk_timer.watch("      forward pyrlk done...");
        if(do_backward_check)
        {
            //进行反向光流检查,有一个阈值限制.
            vector<Point2f> p2d_prev_backward;
            vector<float> backward_err;
            cv::calcOpticalFlowPyrLK(next,prev,p2d_v_next,p2d_prev_backward,backward_check_output,backward_err);
            for(int i = 0;i<backward_check_output.size();i++)
            {
                backward_check_output.at(i) = backward_check_output.at(i) && (pt_calc_dist(p2d_v_prev.at(i),p2d_prev_backward.at(i))<1.0);
            }
        }
        do_pyrlk_timer.watch("      backward pyrlk done...");
        vector<unsigned char> fundamental_match_success;
        cv::Mat fundMat = cv::findFundamentalMat(p2d_v_prev,p2d_v_next,fundamental_match_success);

        int goodMatches_count = 0;
        for(int i = 0;i<fundamental_match_success.size();i++)
        {
            if(fundamental_match_success[i]&&track_success_v[i]&&backward_check_output[i])
            {
                final_output[i] = 1;
            }
            else
            {
                final_output[i] = 0;
            }
        }
        track_success_v = final_output;

    }
}


#endif
