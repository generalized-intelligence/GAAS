#include <mutex>
#include <deque>


#include "IMUPreintLib.h"
#include <glog/logging.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>



class IMUPreintManager
{
public:
    bool initIMUPreintManager(int argc,char** argv)
    {
        ros::init(argc,argv,"imu_preint_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        LOG(INFO)<<"initiating IMU Preint manager."<<endl;
        pIMUPosePub = std::shared_ptr<ros::Publisher>(new ros::Publisher);
        pIMUTFbroadcaster = std::shared_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);

        *pIMUPosePub = pNH->advertise<geometry_msgs::PoseStamped>("/gaas/localization/IMU_preint_pose",1);


        //读入外参数,并发布imu到lidar的静态tf.
        tf2_ros::StaticTransformBroadcaster tf_static_publisher;
        lidar_imu_static_tf = getIMULidarTransformStatic();
        tf_static_publisher.sendTransform(lidar_imu_static_tf);

        //init subscribers.

        lidar_pose_sub = pNH->subscribe<geometry_msgs::PoseStamped>("/gaas/localization/registration_pose",1,&IMUPreintManager::LidarPoseCallback,this);

        string imu_topic_name;
        if(!ros::param::get("imu_topic_name",imu_topic_name))
        {
            LOG(ERROR)<<"No imu_topic_name found in launch file!"<<endl;
            throw "ERROR!";
        }

        imu_msg_sub = pNH->subscribe<sensor_msgs::Imu>(imu_topic_name,100,&IMUPreintManager::IMUInfoCallback,this);
        waitForFirstLidarPose();

        //register callback

        ros::spin();
    }
    void waitForFirstLidarPose()
    {
        LOG(INFO)<<"Waiting for first lidar pose message..."<<endl;
        while(ros::ok()&&!this->pose_ever_init)
        {
            //this->pose_mutex.lock();
            //bool ready = this->pose_ever_init;
            //this->pose_mutex.unlock();
            //if(this->pose_ever_init)
            //{
            //    return;//break;
            //}
            //反复spinOnce等这个消息到.
            for(int i = 0;i<10;i++)
            {
                ros::spinOnce();
            }
            LOG(INFO)<<"waiting for first lidar pose..."<<endl;
            //usleep(1000);//10ms.
        }
        if(ros::ok())
        {
            LOG(INFO)<<"In IMUPreintManager: lidar pose initiated!"<<endl;
        }
    }
    void LidarPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
    {
        LOG(INFO)<<"In lidar pose callback"<<endl;
        this->pose_mutex.lock();
        tf2::doTransform(*pose_msg,this->curr_pose,lidar_imu_static_tf);//转到imu坐标系下.
        //this->curr_pose = *pose_msg;
        if(!this->pose_ever_init)//第一帧的情况
        {
            this->pose_ever_init = true;
            this->prev_pose = this->curr_pose;//更新后直接返回.
            this->pose_mutex.unlock();
            return;
        }
        this->pose_ever_init = true;
        this->pose2_ready = true;


        //普通帧的情况
        this->curr_pose = *pose_msg;
        geometry_msgs::PoseStamped frame1,frame2;
        frame1 = this->prev_pose; //input_msg = T_lidar_map; frame1 = T_imu_map --> T_imu_map = T_imu_lidar * T_lidar_map;
        frame2 = this->curr_pose;

        this->pose_mutex.unlock();

        LOG(INFO)<<"preparing for preint oper."<<endl;

        //在frame1到frame2之间处理预积分计算.
        //TODO:更新imu factor bias estimation.
        SequentialIMUPreintegrator new_sip;
        new_sip.init(frame1,&this->prev_velocity,&this->imu_bias);


        imu_msg_mutex.lock();
        int prop_times = 0;
        for(auto u:this->imu_buffer)
        {
            if(u.header.stamp<frame2.header.stamp)
            {
                geometry_msgs::PoseStamped p_temp;
                new_sip.do_prop(u,p_temp);
                prop_times++;
            }
        }
        LOG(INFO)<<"after prop times:"<<prop_times<<endl;
        bool optimization_result = new_sip.do_optimize(frame2,&this->prev_velocity,&this->imu_bias);//,&imu_vec);//update imu bias and initial velocity.

        LOG(INFO)<<"Updated new velocity and imu_bias; Velocity:"<<this->prev_velocity<<"; bias:"<<this->imu_bias<<endl;
        while(ros::ok())
        {
            if(imu_buffer.empty() || imu_buffer.front().header.stamp > frame2.header.stamp)//反复检查imu_buffer直到移除所有超时的.
            {
                break;
            }
            imu_buffer.pop_front();
            //LOG(INFO) <<"Pop front!remaining:"<<imu_buffer.size()<<endl;
        }
        imu_msg_mutex.unlock();

        if(!optimization_result)
        {
            LOG(INFO)<<"imu preint optimization failed!"<<endl;
        }
        this->sip = SequentialIMUPreintegrator();
        this->sip.init(frame2,&this->prev_velocity,&this->imu_bias);
        //交换pose.

        this->pose_mutex.lock();
        this->prev_pose = this->curr_pose;
        this->pose_mutex.unlock();

        LOG(INFO)<<"lidar pose callback finished."<<endl;
    }
    void normalize_quaternion(geometry_msgs::TransformStamped& transform_)
    {
        auto& q_ = transform_.transform.rotation;
        Eigen::Quaternionf qf(q_.w,q_.x,q_.y,q_.z);
        qf.normalize();
        q_.w = qf.w();q_.x = qf.x();q_.y = qf.y();q_.z = qf.z();
    }
    void IMUInfoCallback(const sensor_msgs::ImuConstPtr& imu_msg)
    {//发布imu到Lidar preint变换系.
        //LOG(INFO)<<"In IMUMsgCallback()"<<endl;
        ros::Time t_now = ros::Time::now();

        this->pose_mutex.lock();
        if(!this->pose_ever_init)
        {
            this->pose_mutex.unlock();
            LOG(INFO)<<"pose not initialized, still waiting..."<<endl;
            return;
        }
        bool pose2_ready_copy = this->pose2_ready;
        this->pose_mutex.unlock();
        //作预积分运算,发布pose.
        imu_msg_mutex.lock();
        this->imu_buffer.push_back(*imu_msg);
        imu_msg_mutex.unlock();
        if(!pose2_ready_copy)
        {
            return;
        }
        //check timeout.
        if((t_now-imu_msg->header.stamp).toSec()>0.100||(t_now-this->sip.prev_stamp).toSec()>0.4
                || (t_now- this->curr_pose.header.stamp).toSec()>0.2 //dead-reckoning for more than 0.2s.
                )
        {
            LOG(WARNING)<<"Time out detected;reset imu preint"<<endl;
            resetIMUPreint();
            return;
        }

        geometry_msgs::PoseStamped p_preint;
        this->sip.do_prop(*imu_msg,p_preint);
        p_preint.header.frame_id = "map";


        double dt = (imu_msg->header.stamp - this->sip.prev_stamp).toSec();
        if(dt<0.10) //150ms maximum.
        {
            this->pIMUPosePub->publish(p_preint);
            geometry_msgs::TransformStamped tf_stamped;
            tf_stamped.header.stamp = imu_msg->header.stamp;
            tf_stamped.header.frame_id = "map";
            tf_stamped.child_frame_id = "imu_preint";
            auto& r = tf_stamped.transform.rotation;
            auto& t = tf_stamped.transform.translation;
            const auto& r_ = p_preint.pose.orientation;
            const auto& t_ = p_preint.pose.position;
            r.w=r_.w; r.x=r_.x; r.y=r_.y; r.z=r_.z;
            t.x=t_.x; t.y=t_.y; t.z=t_.z;
            //sometimes the quaternion output by gtsam optimizer is not normalized, so we do this manually to avoid ros tf2 error.
            normalize_quaternion(tf_stamped);
            this->pIMUTFbroadcaster->sendTransform(tf_stamped);
        }
        return;

    }
    geometry_msgs::TransformStamped getIMULidarTransformStatic()
    {
        geometry_msgs::TransformStamped retval;
        retval.header.stamp = ros::Time::now();
        retval.child_frame_id = "imu";
        retval.header.frame_id = "lidar";

        cv::Mat T_imu_lidar_cvmat;
        string extrinsic_file_path;
        if(!ros::param::get("imu_lidar_extrinsic_path",extrinsic_file_path))
        {
            LOG(ERROR)<<"Config file path error!"<<endl;
            throw "ERROR:Config file path error!";
        }
        cv::FileStorage fs;
        fs.open(extrinsic_file_path,cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            LOG(ERROR)<<"Config file not found!"<<endl;
            throw "ERROR:Config file not found!";
        }
        fs["T_imu_lidar"]>>T_imu_lidar_cvmat;
        assert(T_imu_lidar_cvmat.cols == 4 && T_imu_lidar_cvmat.rows==4 && T_imu_lidar_cvmat.type()==CV_32F);

        //To eigen mat.
        Eigen::Matrix4f T_imu_lidar_eigenmat;
        cv::cv2eigen(T_imu_lidar_cvmat,T_imu_lidar_eigenmat);
        Eigen::Matrix3f R_imu_lidar = T_imu_lidar_eigenmat.block(0,0,3,3);
        Eigen::Quaternionf q_(R_imu_lidar);

        retval.transform.rotation.w = q_.w();
        retval.transform.rotation.x = q_.x();
        retval.transform.rotation.y = q_.y();
        retval.transform.rotation.z = q_.z();

        retval.transform.translation.x = T_imu_lidar_eigenmat(0,3);
        retval.transform.translation.y = T_imu_lidar_eigenmat(1,3);
        retval.transform.translation.z = T_imu_lidar_eigenmat(2,3);

        return retval;
    }
    Matrix4 getMat44FromPoseStamped(const geometry_msgs::PoseStamped& msg)
    {
        return this->sip.poseFromMsg(msg).matrix();
    }
    geometry_msgs::PoseStamped getNewPoseMsg(const geometry_msgs::PoseStamped& original_msg,const Matrix4& new_pose)
    {
        geometry_msgs::PoseStamped retval = original_msg;
        retval.pose.position.x = new_pose(0,3);
        retval.pose.position.y = new_pose(1,3);
        retval.pose.position.z = new_pose(2,3);
        Eigen::Matrix3d rot_eigen = new_pose.block(0,0,3,3);
        Eigen::Quaterniond quat(rot_eigen);

        retval.pose.orientation.w = quat.w();
        retval.pose.orientation.x = quat.x();
        retval.pose.orientation.y = quat.y();
        retval.pose.orientation.z = quat.z();

        return retval;
    }
    void resetIMUPreint()
    {
        this->pose_mutex.lock();
        this->imu_msg_mutex.lock();
        Vector3d prev_velocity_new;
        this->prev_velocity = prev_velocity_new;
        SequentialIMUPreintegrator::IMUBiasType imu_bias_new;
        this->imu_bias = imu_bias_new;
        geometry_msgs::PoseStamped pose_reset;
        this->prev_pose = pose_reset;
        this->curr_pose = pose_reset;
        this->imu_buffer.clear();
        this->pose_ever_init = false;
        this->pose2_ready = false;
        SequentialIMUPreintegrator new_sip;
        this->sip = new_sip;
        this->imu_msg_mutex.unlock();
        this->pose_mutex.unlock();
        return;
    }
private:
    SequentialIMUPreintegrator sip;
    Vector3d prev_velocity;
    SequentialIMUPreintegrator::IMUBiasType imu_bias;


    geometry_msgs::PoseStamped prev_pose;
    geometry_msgs::PoseStamped curr_pose;
    std::mutex pose_mutex;

    std::deque<sensor_msgs::Imu> imu_buffer;
    std::mutex imu_msg_mutex;

    bool pose_ever_init = false;
    bool pose2_ready = false;

    //about ros
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;
    std::shared_ptr<ros::Publisher> pIMUPosePub = nullptr;
    std::shared_ptr<tf2_ros::TransformBroadcaster> pIMUTFbroadcaster = nullptr;
    geometry_msgs::TransformStamped lidar_imu_static_tf;

    ros::Subscriber lidar_pose_sub,imu_msg_sub;

};





int main(int argc, char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("imu_preint_node");
    IMUPreintManager IPM;
    IPM.initIMUPreintManager(argc,argv);
    return 0;
}

