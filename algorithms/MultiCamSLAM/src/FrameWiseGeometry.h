#ifndef FRAMEWISE_GEOMETRY_H
#define FRAMEWISE_GEOMETRY_H
#include "Frame.h"
#include "FeatureFrontEndCV.h"
#include "Timer.h"



namespace mcs
{
    using namespace std;
    //void doFindEssentialMatrix(vector<KeyPoint>& prev,vector<KeyPoint>& next,cvMat& output_essential_mat);

    //void doFindEssentialMatrix(Frame& f_prev,Frame& f_next);
    void doFindFundamentalMatrix();
    void doPnPRansacMultiCam_Using_BA(Frame& f1,Frame& f2,cv::Mat& output_r_mat,cv::Mat& output_t_vec,bool& output_optimize_success,vector<float>& residual_of_2dkps);
    void trackAndDoSolvePnPRansacMultiCam(Frame& f1,Frame& f2,cv::Mat& output_r_mat,cv::Mat& output_t_vec,
                                bool& output_frame_pnp_ransac_success,vector<bool>& output_cam_match_success,int* ptotal_tracked_points_count = nullptr)
    {
        int total_success_tracked_point_count = 0;
        ScopeTimer t1("trackAndDoSolvePnPRansacMultiCam() timer");
        bool isStereoMode = f1.frame_type==FRAME_TYPE_STEREO;
        if(isStereoMode)
        {
            LOG(INFO)<<"in doSolvePnPRansacMultiCam stage1,f1.cam_info_vec.size:"<<endl;
            LOG(INFO)<<f1.get_stereo_cam_info().size()<<endl;
            output_cam_match_success.resize(f1.get_stereo_cam_info().size());
        }
        int success_count = 0;//TODO:加入一个策略,完成多摄像头之间的筛选.
        //doSolvePnPRansac
        LOG(INFO)<<"in doSolvePnPRansacMultiCam stage2"<<endl;
        auto imgs_prev = f1.getMainImages();
        auto imgs_next = f2.getMainImages();
        LOG(INFO)<<"in doSolvePnPRansacMultiCam stage3"<<endl;

        auto p3ds_vv = f1.p3d_vv;
        auto pts_vv = f2.p2d_vv;
        //pts_vv = OptFlowForFrameWiseTracking(*pframe2);//track frame2 获取对应位置的点...
        LOG(INFO)<<"in doSolvePnPRansacMultiCam stage4"<<endl;
        for(int i = 0;i < imgs_prev.size();i++)
        {
            LOG(INFO)<<"frame1 p3ds_vv.size():"<<p3ds_vv.size()<<";frame2 p2ds_vv.size():"<<pts_vv.size()<<endl;

            LOG(INFO)<<"in doSolvePnPRansacMultiCam stage4.1"<<endl;
            cv::Mat camMat;
            if(isStereoMode)
            {
                camMat = f2.get_stereo_cam_info()[i].getCamMat();
            }
            else
            {
                camMat = f2.get_cam_info()[i].getCamMat();
            }
            LOG(INFO)<<"in doSolvePnPRansacMultiCam stage4.2"<<endl;
            output_r_mat = cv::Mat();
            output_t_vec = cv::Mat();
            vector<unsigned char> match_success;
            //TODO:multi_thread implementation.


            vector<p3dT> pnp_ransac_input_p3d;
            vector<p2dT> pnp_ransac_input_p2d;
            {//step<1>.track 2d pts.
                vector<p3dT> vp3d_pts = p3ds_vv[i];
                vector<p2dT> vp2d_pts;
                for(int index_p3d=0;index_p3d<vp3d_pts.size();index_p3d++)
                {
                    vp2d_pts.push_back(f1.p2d_vv[i][f1.map3d_to_2d_pt_vec[i][index_p3d] ]);
                }
                vector<cv::KeyPoint> toTrack_2dpts;
                cv::KeyPoint::convert(vp2d_pts,toTrack_2dpts);
                vector<Point2f> original_remaining_2dpts,tracked_nextimg_2dpts;
                vector<unsigned char> v_pt_track_success;
                bool output_track_success;
                map<int,int> tracked_kps_to_original_kps_map_output;
                OptFlowForFrameWiseTracking(*(imgs_prev[i]),*(imgs_next[i]),toTrack_2dpts,original_remaining_2dpts,tracked_nextimg_2dpts,tracked_kps_to_original_kps_map_output,v_pt_track_success,output_track_success);


                for(int index_p3d = 0;index_p3d<vp3d_pts.size();index_p3d++)
                {
                    int p2d_index = tracked_kps_to_original_kps_map_output[index_p3d];
                    if(p2d_index>=0)//a match has been found.
                    {
                        pnp_ransac_input_p2d.push_back(vp2d_pts[p2d_index]);
                        pnp_ransac_input_p3d.push_back(vp3d_pts[p2d_index]);
                    }
                }
            }
            //step<2>.process result.
            LOG(INFO)<<"in doSolvePnPRansacMultiCam stage5: success tracked vp3d_pts.size:"<<pnp_ransac_input_p2d.size()<<",tracked p2d_pts.size:"<<pnp_ransac_input_p2d.size()<<endl;
            LOG(INFO)<<"cam mat rows and cols:"<<camMat.rows<<","<<camMat.cols;
            bool success = solvePnPRansac(pnp_ransac_input_p3d,pnp_ransac_input_p2d,camMat,cv::Mat(),output_r_mat,output_t_vec,false,100,8.0,0.99,match_success,SOLVEPNP_ITERATIVE);
            cv::Mat R_output;
            cv::Rodrigues(output_r_mat,R_output);
            LOG(INFO)<<"in doSolvePnPRansacMultiCam stage6,output R_mat:\n"<<R_output<<"\n output t vec"<<output_t_vec<<endl;
            output_cam_match_success[i] = success;
            if(success)//一组摄像头成功
            {
                int pnpransac_success_count = 0;
                for(int j = 0;j<match_success.size();j++)
                {
                    if(match_success[j])
                    {
                        pnpransac_success_count+=1;
                        total_success_tracked_point_count+=1; //只计算整组成功以后的成功点个数.失败的摄像头组整组都不算了.
                    }
                }
                LOG(INFO)<<"pnp_ransac_match_success!!matched:"<< pnpransac_success_count<<"/"<<pnp_ransac_input_p2d.size()<<endl;
                success_count+=1;
            }
            else
            {
                LOG(WARNING)<<"Cam "<<i<<" has caught into pnp ransac failure!"<<endl;
            }
        }
        LOG(INFO)<<"in doSolvePnPRansacMultiCam stage7"<<endl;
        if(success_count>=3)
        {
            output_frame_pnp_ransac_success = true;
        }
        LOG(INFO)<<"For all cams:total tracked points count:"<<total_success_tracked_point_count;
        if(ptotal_tracked_points_count != nullptr)
        {
            *ptotal_tracked_points_count = total_success_tracked_point_count;
        }
        return;

    }
    void trackLocalFramePoints(Frame& f1,Frame& f2,cv::Mat& output_r_mat,cv::Mat& output_t_vec,
                               bool& output_frame_tracking_success,vector<bool> output_cam_track_success);

    void trackMapPoints(Frame& f1,vector<shared_ptr<MapPoint> > v_pPoints,cv::Mat& output_r_mat, cv::Mat& output_t_vec,
                        bool& output_frame_tracking_success,vector<bool> output_framekp2d_track_success);









}
#endif
