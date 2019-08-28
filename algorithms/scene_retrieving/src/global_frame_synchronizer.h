//同步global optimization graph的帧信息.

#include <glog/logging.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include "scene_retrieve.h"
#include <iostream>
#include <map>
#include <mutex>
#include <deque>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <roseus/StringStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>



class GlobalFrameSyncManager
{
public:
    GlobalFrameSyncManager(SceneRetriever* pscene_retriever)
    {
        this->pSceneRetriever = pscene_retriever;
    }
    void GOG_msg_callback(const roseus::StringStamped & msg_gog,int gog_id,int current_scene_frame_id)
    {
        double t = msg_gog.header.stamp.toSec();

        //search buffer.
        buffer_mutex.lock();
        int index_i = 0;
        bool matched = false;
        for(;index_i<this->img_buffer_.size();index_i++)
        {
            if(std::get<0>(this->img_buffer_[index_i]) == t)
            {
                //matched!
                //std::get<0>(this->img_buffer_[index_i]);
                matched = true;
                break;
            }
        }
        if(matched)
        {
            auto l_ptr = std::get<1>(this->img_buffer_[index_i]);
            auto r_ptr = std::get<2>(this->img_buffer_[index_i]);
            this->processMatch(t,l_ptr,r_ptr,gog_id,current_scene_frame_id);
            for(int i = 0;i<=index_i;i++)
            {
                this->img_buffer_.pop_front();//排除没用的.
            }
        }
        //auto iter = this->map_buffer_img_stamp_to_scene_frame_id.find(key);
        //if(iter!=this->map_buffer_img_stamp_to_scene_frame_id.end())
        //{
        //    int scene_frame_id = iter->second;
        //    this->map_stamp_to_gog_frame_id[t] = id;
        //}
        else
        {
            LOG(INFO)<<"No image msg matched with gog msg. Maybe device performance is too poor.";
        }
        buffer_mutex.unlock();
    }
    void Image_msg_callback(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
    {
        buffer_mutex.lock();
        double t = msgLeft->header.stamp.toSec();
        this->img_buffer_.push_back(std::make_tuple(msgLeft->header.stamp.toSec(),msgLeft,msgRight));
        if(this->img_buffer_.size()>1000)//keep size.//TODO:set into config file.
        {
            this->img_buffer_.pop_front();
        }
        buffer_mutex.unlock();
    }
    void processMatch(double t,const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight,int gog_id,int currentSceneFrameID)
    {//TODO:put into another thread.
        process_match_mutex.lock();
        //TODO :auto scene_frame = generateSceneFrameFromStereoImage(const cv::Mat &imgl, cv::Mat &imgr, const cv::Mat& RotationMat, const cv::Mat& TranslationMat, const cv::Mat& Q_mat);
        //this->pSceneRetriever-> addFrameToScene(const std::vector<cv::KeyPoint>& points2d_in, const std::vector<cv::Point3d>points3d_in,const cv::Mat& point_desp_in, const cv::Mat R, const cv::Mat t)
        this->scene_frame_id_to_gog_id_and_time_t[currentSceneFrameID] = std::make_pair(gog_id,t);
        process_match_mutex.unlock();
    }
    int retrieveGOGIDFromSceneFrameID(int scene_frame_id) //return -1 if invalid.
    {
        auto iter = this->scene_frame_id_to_gog_id_and_time_t.find(scene_frame_id);
        if(iter != this->scene_frame_id_to_gog_id_and_time_t.end())
        {
            return this->scene_frame_id_to_gog_id_and_time_t[scene_frame_id].first;
        }
        LOG(WARNING)<<"Invalid scene_frame_id: "<<scene_frame_id<<". Check the code!"<<endl;
        return -1;
    }
private:
    //std::vector<...>
    //std::map<double,int > map_stamp_to_scene_frame_id;//以时间戳为主键进行查找:int scene_frame_id,int gog_id
    //std::map<double,int > map_stamp_to_gog_frame_id;


    //std::map<double,int> map_buffer_img_stamp_to_scene_frame_id;
    //std::vector<std::tuple<double,const sensor_msgs::ImageConstPtr&,const sensor_msgs::ImageConstPtr&> > img_buffer_;
    std::deque<std::tuple<double,const sensor_msgs::ImageConstPtr&,const sensor_msgs::ImageConstPtr&> > img_buffer_;
    std::mutex buffer_mutex;
    std::map<int,std::pair<int,double> > scene_frame_id_to_gog_id_and_time_t;
    std::mutex process_match_mutex;
    SceneRetriever* pSceneRetriever;
};




















