#include "Frame.h"
#include "FeatureFrontEndCV.h"



namespace mcs
{
    using namespace std;
    //void doFindEssentialMatrix(vector<KeyPoint>& prev,vector<KeyPoint>& next,cvMat& output_essential_mat);

    //void doFindEssentialMatrix(Frame& f_prev,Frame& f_next);
    void doFindFundamentalMatrix();
    void doPnPRansacMultiCam_Using_BA(Frame& f1,Frame& f2,cv::Mat& output_r_mat,cv::Mat& output_t_vec,bool& output_optimize_success,vector<float>& residual_of_2dkps);
    void doSolvePnPRansacMultiCam(Frame& f1,Frame& f2,cv::Mat& output_r_mat,cv::Mat& output_t_vec,bool& output_frame_pnp_ransac_success,vector<bool>& output_cam_match_success)
    {
        output_cam_match_success.resize(f1.cam_info_vec.size());
        int success_count = 0;//TODO:加入一个策略,完成多摄像头之间的筛选.
        //doSolvePnPRansac
        auto imgs_prev = f1.getMainImages();
        auto imgs_next = f2.getMainImages();
        bool isStereoMode = f1.frame_type==FRAME_TYPE_STEREO;
        auto p3ds_vv = f1.p3d_vv;
        auto pts_vv = f2.p2d_vv;

        for(int i = 0;i < imgs_prev.size();i++)
        {

            auto vp3d_pts = p3ds_vv[i];
            auto vp2d_pts = pts_vv[i];
            cv::Mat camMat = dynamic_cast<StereoCamConfig&>(f2.cam_info_vec[i]).getCamMat();
            output_r_mat = cv::Mat();
            output_t_vec = cv::Mat();
            vector<unsigned char> match_success;
            //TODO:multi_thread implementation.
            bool success = solvePnPRansac(vp3d_pts,vp2d_pts,camMat,cv::Mat(),output_r_mat,output_t_vec,false,100,8.0,0.99,match_success,SOLVEPNP_ITERATIVE);
            output_cam_match_success[i] = success;
            if(success)
            {
               success_count+=1;
            }
            else
            {
                LOG(WARNING)<<"Cam "<<i<<" has caught into pnp ransac failure!"<<endl;
            }
        }
        if(success_count>=3)
        {
            output_frame_pnp_ransac_success = true;
        }
        return;

    }
    void trackLocalFramePoints(Frame& f1,Frame& f2,cv::Mat& output_r_mat,cv::Mat& output_t_vec,
                               bool& output_frame_tracking_success,vector<bool> output_cam_track_success);

    void trackMapPoints(Frame& f1,vector<shared_ptr<MapPoint> > v_pPoints,cv::Mat& output_r_mat, cv::Mat& output_t_vec,
                        bool& output_frame_tracking_success,vector<bool> output_framekp2d_track_success);









}
