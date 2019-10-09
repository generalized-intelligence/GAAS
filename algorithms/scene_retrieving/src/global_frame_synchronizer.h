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
#include <cmath>

const double t_thres__ = 0.01;
bool close_to(uint64_t t1,uint64_t t2)
{
    if(t1==t2)
    {
        return true;
    }
    cout<<"t1:"<<t1<<"t2:"<<t2<<endl;
    cout<<t1-t2<<endl;
    return false;
}
class GlobalFrameSyncManager
{
static const int max_buf_cap = 100;
public:
    GlobalFrameSyncManager(SceneRetriever* pscene_retriever,cv::Mat* pQ_mat,LoopClosingManager* plcm)
    {
        this->pSceneRetriever = pscene_retriever;
        this->pQ_mat = pQ_mat;
        this->plcm = plcm;
    }
    void GOG_msg_callback(const roseus::StringStamped & msg_gog,int gog_id,int current_scene_frame_id)
    {
        LOG(INFO)<<"in GOG_msg_callback():"<<endl;
        uint64_t t = msg_gog.header.stamp.toNSec();

        //search buffer.
        buffer_mutex.lock();
        int index_i = 0;
        bool matched = false;
        for(;index_i<this->img_buffer_.size();index_i++)
        {
            if(close_to(std::get<0>(this->img_buffer_[index_i]),t))
            //if(std::get<0>(this->img_buffer_[index_i])==t)
            {
                //matched!
                //std::get<0>(this->img_buffer_[index_i]);
                matched = true;
                break;
            }
        }
        if(matched)
        {
            LOG(INFO)<<"    in GOG_msg_callback(): match img success!"<<endl;
            auto l_ptr = std::get<1>(this->img_buffer_[index_i]);
            auto r_ptr = std::get<2>(this->img_buffer_[index_i]);
            this->processMatch(t,l_ptr,r_ptr,gog_id,current_scene_frame_id);
            for(int i = 0;i<=index_i;i++)
            {
                this->img_buffer_.pop_front();//排除没用的.
            }
            LOG(INFO)<<"success match of gog msg and image.";
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
        LOG(INFO)<<"in Image_msg_callback():"<<endl;
        buffer_mutex.lock();
        uint64_t t = msgLeft->header.stamp.toNSec();
        this->img_buffer_.push_back(std::make_tuple(msgLeft->header.stamp.toNSec(),msgLeft,msgRight));
        if(this->img_buffer_.size()>1000)//keep size.//TODO:set into config file.
        {
            this->img_buffer_.pop_front();
        }
        buffer_mutex.unlock();
    }
    void processMatch(uint64_t t,const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight,int gog_id,int currentSceneFrameID) // 只放在任务队列里.真正处理在
    {//TODO:put into another thread.
        process_match_mutex.lock();
        //TODO :auto scene_frame = generateSceneFrameFromStereoImage(const cv::Mat &imgl, cv::Mat &imgr, const cv::Mat& RotationMat, const cv::Mat& TranslationMat, const cv::Mat& Q_mat);
        //this->pSceneRetriever-> addFrameToScene(const std::vector<cv::KeyPoint>& points2d_in, const std::vector<cv::Point3d>points3d_in,const cv::Mat& point_desp_in, const cv::Mat R, const cv::Mat t)
/*
    int scene_frame_count = pSceneRetriever->getScene().getImageCount();
    LOG(INFO)<<"In ImageCallback():adding frame to scene by stereo..."<<endl;
    bool match_success;
    cv::Mat RT_mat;
    //int inliers = pSceneRetriever->retrieveSceneFromStereoImage(curLeftImage, curRightImage, *pQ_mat, RT_mat, match_success);
    cv::Mat cam_matrix = (cv::Mat_<float >(3,3) << 376, 0, 376, 
            0, 376, 240, 
            0, 0, 1);
    int loop_id;
    int inliers = pSceneRetriever->retrieveSceneWithScaleFromMonoImage(curLeftImage,cam_matrix,RT_mat,match_success,&loop_id);

    if(match_success)
    {   
        LOG(INFO)<<"In scene_retrieving_ros ImageCallback():Match success! RT mat is: \n"<<RT_mat<<endl;
        //TODO:publish RT mat!
        std_msgs::String str;
        stringstream ss;
        std::string str_content;
        ss<<"Frame id:"<<loop_id<<","<<scene_frame_count<<";RT:"<<RT_mat;
        //ss>>str_content;
        str_content = ss.str();
        str.data = str_content.c_str();
        Pub.publish(str);
    }
    else
    {   
        LOG(INFO)<<"Match failed."<<endl;
        return;
    }
*/

        LOG(INFO)<<"Forming empty rt mat."<<endl;
        cv::Mat r_mat = cv::Mat::eye(3,3,CV_32F);
        cv::Mat t_mat = cv::Mat::zeros(3,1,CV_32F);
        LOG(INFO)<<"Generating frame..."<<endl;
        bool success_gen = false;
        cv::Mat imleft,imright;
        imleft = cv_bridge::toCvShare(msgLeft)->image;
        imright = cv_bridge::toCvShare(msgRight)->image;
        auto current_scene_frame = generateSceneFrameFromStereoImage(imleft,imright,r_mat,t_mat,*(this->pQ_mat),*(this->plcm),success_gen);
        if(!success_gen)
        {   
            LOG(INFO)<<"    after generateSceneFrameFromStereoImage():generate failed!"<<endl;
            return;
        }   
        LOG(INFO)<<"adding frame into scene..."<<endl;
        pSceneRetriever->addFrameToScene(std::get<0>(current_scene_frame),std::get<1>(current_scene_frame),std::get<2>(current_scene_frame),std::get<3>(current_scene_frame),std::get<4>(current_scene_frame));

        this->scene_frame_id_to_gog_id_and_time_t[currentSceneFrameID] = std::make_pair(gog_id,t);


        task_queue_mutex.lock();
        this->backend_task_queue.push_back(std::make_tuple( msgLeft,msgRight,gog_id,currentSceneFrameID ) );
        if(this->backend_task_queue.size()>this->max_buf_cap) // TODO:set into config file.
        {
            this->backend_task_queue.pop_front();
        }
        task_queue_mutex.unlock();

        process_match_mutex.unlock();
        LOG(INFO)<<"add frame to scene success!"<<endl;
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
    std::pair<int,int> checkLoopTaskQueue(bool& match_success_out,cv::Mat camera_Q_mat,cv::Mat& RT_mat_out)
    //输入参数:
    //1.匹配是否成功
    //2.输入双目摄像机参数Q.
    //3.返回的RT矩阵.(如果匹配成功则有效.)
    //返回值:
    //1.回环帧gog_id(老的那个帧)
    //2.当前帧gog_id(新的帧.)
    {
        match_success_out = false;
        if(this->task_queue_mutex.try_lock())
        {
            do
            {
                if(this->backend_task_queue.size() == 0)
                {
                    this->task_queue_mutex.unlock();
                    break;
                }
                auto task_info = this->backend_task_queue.front();

                int gog_id_of_this_task = std::get<2>(task_info);
                this->backend_task_queue.pop_front();
                this->task_queue_mutex.unlock();//till here we do release the mutex to avoid waste of time in critical area.
                //prepare for calling retrieveGOGIDFromSceneFrameID()...
                int retrieved_frame_id = -1;
                cv::Mat Q_mat;
                Q_mat = camera_Q_mat;
                cv::Mat left_im,right_im;
                left_im = cv_bridge::toCvShare(std::get<0>(task_info))->image;
                right_im = cv_bridge::toCvShare(std::get<1>(task_info))->image;
                //int inliers = this->pSceneRetriever->retrieveSceneFromStereoImage(
                //    left_im,right_im,
                //    *this->pQ_mat, RT_mat_out,match_success_out,&retrieved_frame_id);
                cv::Mat cam_matrix = (cv::Mat_<float >(3,3) << 376, 0, 376, 
                                                               0, 376, 240, 
                                                               0, 0, 1);

                int inliers = pSceneRetriever->retrieveSceneWithScaleFromMonoImage(left_im,cam_matrix,RT_mat_out,match_success_out,&retrieved_frame_id);

                if(match_success_out)
                {
                    //获取retrieved scene frame id, 由此获取gog中回环帧的id.
                    LOG(INFO)<<"Loop detected in checkLoopTaskQueue()!"<<endl;
                    int gog_loop_id = std::get<0>(this->scene_frame_id_to_gog_id_and_time_t[retrieved_frame_id]);
                    return std::make_pair(gog_loop_id,gog_id_of_this_task);//publish(.....)
                }
                else
                {
                    LOG(INFO)<<"Loop detection failed in checkLoopTaskQueue()!"<<endl;
                }
            }while(0);
        }
        else
        {
            LOG(INFO)<<"Try lock failed in checkLoopTaskQueue()."<<endl;
        }
        return std::make_pair(-1,-1);
    }
    
private:
    //std::vector<...>
    //std::map<double,int > map_stamp_to_scene_frame_id;//以时间戳为主键进行查找:int scene_frame_id,int gog_id
    //std::map<double,int > map_stamp_to_gog_frame_id;


    //std::map<double,int> map_buffer_img_stamp_to_scene_frame_id;
    //std::vector<std::tuple<double,const sensor_msgs::ImageConstPtr&,const sensor_msgs::ImageConstPtr&> > img_buffer_;
    std::deque<std::tuple<uint64_t,const sensor_msgs::ImageConstPtr&,const sensor_msgs::ImageConstPtr&> > img_buffer_;
    std::mutex buffer_mutex;
    std::map<int,std::pair<int,uint64_t> > scene_frame_id_to_gog_id_and_time_t;
    std::mutex process_match_mutex;
    SceneRetriever* pSceneRetriever;
    cv::Mat* pQ_mat;
    LoopClosingManager* plcm;
    //后端匹配的任务队列.
    //structure:
    //1.image left ptr;
    //2.image right ptr;
    //3.gog_id of this msg;
    //4.scene_id of this msg;
    std::deque< std::tuple<const sensor_msgs::ImageConstPtr&,const sensor_msgs::ImageConstPtr&,int,int> > backend_task_queue;
    std::mutex task_queue_mutex;
    //std::thread backend_thread;
};




















