#ifndef MCS_VISUALIZATION_H_FILE
#define MCS_VISUALIZATION_H_FILE
#include "Frame.h"
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include "FeatureFrontEndCV.h"
#include "Timer.h"


namespace mcs
{



/*cv::Mat draw_oridinary_frame(Frame& frame,vector<p2dT>& p2d_vec,int cam_index,
                                               vector<unsigned char>* pStates = nullptr//TODO:design the enums.
						)
{


    return ...//TODO:Fill in.
}*/
void visualized_tracked_p2d_and_ordinary_frame_stereo(Frame& frame,vector<p2dT>& p2d_vec_left,vector<p2dT>& p2d_vec_right,int cam_index,
                                               vector<unsigned char>* pStates = nullptr//TODO:design the enums.
                                               );

void visualized_tracked_p2d_and_ordinary_frame(Frame& frame,vector<p2dT>& p2d_vec,int cam_index,
                                               vector<unsigned char>* pStates = nullptr//TODO:design the enums.
                                               )
{
    //可视化显示所有被跟踪的2d点.
    //cv::Mat final_im;
    cv::Mat img = frame.getMainImages().at(cam_index)->clone();
    for(auto & pt:p2d_vec)
    {
        cv::Point2f left_top,right_bottom;
        left_top.x = pt.x - 2;
        right_bottom.x = pt.x + 2;
        left_top.y = pt.y - 2;
        right_bottom.y = pt.y +2;
        cv::rectangle(img,left_top,right_bottom,cv::Scalar(0,0,255),-1);
    }
    if(p2d_vec.size()>15) //TODO:
    {
        LOG(INFO)<<"[VISUALIZATION]: cam_index:"<<cam_index<<",kps:"<<p2d_vec.size()<<endl;
    }
    else
    {
        LOG(WARNING)<<"[VISUALIZATION]: cam_index:"<<cam_index<<",kps:"<<p2d_vec.size()<<";WARNING:TOO FEW POINTS!"<<endl;
    }
    stringstream ss;
    ss<<"visualize cam pair "<<cam_index;
    cv::imshow(ss.str(),img);
    cv::waitKey(1);
}
void visualize_tracked_p2d_and_ordinary_frame_stereo_helper(Frame& frame)//封装一下,支持reprojection map的形式
{
    auto iter = frame.reproj_map.begin();//只有一个元素,就是最近的那个关键帧.
    {
        int kf_id = iter->first;
        ReprojectionRecordT& recs_vv = iter->second;
        for(int cam_id = 0;cam_id<recs_vv.size();cam_id++)
        {
            auto track_cam_id_v = recs_vv.at(cam_id);
            vector<p2dT> p2d_v_l,p2d_v_r;
            for(auto& track_:track_cam_id_v)
            {
                if (track_.tracking_state!=TRACK_MONO2MONO)//只要不是这种没用的
                {
                    p2d_v_l.push_back(track_.current_frame_p2d);
                }
            }
            visualized_tracked_p2d_and_ordinary_frame_stereo(frame,p2d_v_l,p2d_v_r,cam_id);
        }
    }

}
void visualized_tracked_p2d_and_ordinary_frame_stereo(Frame& frame,vector<p2dT>& p2d_vec_left,vector<p2dT>& p2d_vec_right,int cam_index,
                                               vector<unsigned char>* pStates//TODO:design the enums.
                                               )
{
    //可视化显示所有被跟踪的2d点.
    //cv::Mat final_im;
    cv::Mat imgL = frame.getMainImages().at(cam_index)->clone();
    cv::Mat imgR = frame.getSecondaryImages().at(cam_index)->clone();
    for(auto & pt:p2d_vec_left)
    {
        cv::Point2f left_top,right_bottom;
        left_top.x = pt.x - 2;
        right_bottom.x = pt.x + 2;
        left_top.y = pt.y - 2;
        right_bottom.y = pt.y +2;
        cv::rectangle(imgL,left_top,right_bottom,cv::Scalar(0,0,255),-1);
    }
    for(auto & pt:p2d_vec_right)
    {
        cv::Point2f left_top,right_bottom;
        left_top.x = pt.x - 2;
        right_bottom.x = pt.x + 2;
        left_top.y = pt.y - 2;
        right_bottom.y = pt.y +2;
        cv::rectangle(imgR,left_top,right_bottom,cv::Scalar(0,0,255),-1);
    }

    if(p2d_vec_left.size()>15) //TODO:
    {
        LOG(INFO)<<"[VISUALIZATION]: cam_index:"<<cam_index<<",kps:"<<p2d_vec_left.size()<<endl;
    }
    else
    {
        LOG(WARNING)<<"[VISUALIZATION]: cam_index:"<<cam_index<<",kps:"<<p2d_vec_left.size()<<";WARNING:TOO FEW POINTS!"<<endl;
    }
    stringstream ss;
    ss<<"visualize stereo cam pair "<<cam_index;
    cv::Mat final_img;
    cv::hconcat(imgL,imgR,final_img);
    cv::imshow(ss.str(),final_img);
    cv::waitKey(1);
}


}
#endif
