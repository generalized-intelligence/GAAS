#include "ros_buffer_helper.h" //function helpers.
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>
#include "ROS_IO_Manager.h"

ROS_IO_Manager::ROS_IO_Manager(int argc,char** argv)
{
    //step<1> read config.
    this->pSettings = shared_ptr<cv::FileStorage>(new cv::FileStorage ());
    pSettings->open(string(argv[1]),cv::FileStorage::READ);
    (*pSettings)["SLAM_ROTATION_MAT"] >> this->SLAM_ROTATION;
    (*pSettings)["SLAM_RT_TRANS_MAT"] >> this->SLAM_ROT_AND_TRANS;
    int __inv_z = (*pSettings)["INVERT_SLAM_Z"];
    this->invert_slam_z = (__inv_z>0);
    cv2eigen(this->SLAM_ROTATION,SLAM_ROTATION_EIGEN);
    cv2eigen(this->SLAM_ROT_AND_TRANS,SLAM_ROT_AND_TRANS_EIGEN);

    //step<2> init ros.
    ros::init(argc,argv,"GlobalOptimizationGraph_ROSNode");
    this->pNH = new ros::NodeHandle();

    //step<3> init subscriber.
    //
    //auto f1 =
    //boost::function<void (const nav_msgs::Odometry)>

    //reference:
    //const boost::function<void(const boost::shared_ptr<M const> &)> callback

    attitude_ahrs = this->pNH->advertise<visualization_msgs::Marker>("attitude_ahrs",1);
    attitude_slam = this->pNH->advertise<visualization_msgs::Marker>("attitude_slam",1);
    gog_odometry = this->pNH->advertise<nav_msgs::Odometry>("/gaas/gog_out/odometry",1);
    state_string_publisher = this->pNH->advertise<roseus::StringStamped>("/gaas/global_optimization_graph/state",1);

    boost::function<void(const boost::shared_ptr<nav_msgs::Odometry const>&
    )> ahrs_callback(boost::bind(&ahrs_buffer_helper,this,boost::ref(this->AHRS_buffer),_1 ));

    boost::function<void(const boost::shared_ptr<sensor_msgs::NavSatFix const>&
    )> gps_callback(boost::bind(&gps_buffer_helper,this,boost::ref(this->GPS_buffer),_1));

    boost::function<void(const boost::shared_ptr<geometry_msgs::PoseStamped const>&
    )> slam_callback(boost::bind(&slam_buffer_helper,this,boost::ref(this->SLAM_buffer),_1));

    boost::function<void(const boost::shared_ptr<geometry_msgs::TwistStamped const>&
    )> velocity_callback( boost::bind(&velocity_buffer_helper,this,boost::ref(this->Velocity_buffer),_1));

    boost::function<void(const boost::shared_ptr<sensor_msgs::FluidPressure const>&
    )> barometer_callback( boost::bind(&barometer_buffer_helper,this,boost::ref(this->Barometer_buffer),_1));

    AHRS_sub = pNH->subscribe("/mavros/local_position/odom", 10, ahrs_callback);
    GPS_sub = pNH->subscribe("/mavros/global_position/raw/fix", 10, gps_callback);
    SLAM_sub = pNH->subscribe("/SLAM/pose_for_obs_avoid", 10, slam_callback);
    //SLAM_sub = pNH->subscribe("/mavros/vision_pose/pose", 10, slam_callback);
    Velocity_sub = pNH->subscribe("/mavros/global_position/raw/gps_vel", 10, velocity_callback);
    Barometer_sub = pNH->subscribe("/mavros/imu/static_pressure", 10, barometer_callback);

    //GroundTruth_sub = pNH->subscribe<nav_msgs::Odometry>("/gi/ground_truth/state", 10, &ROS_IO_Manager::gt_callback, this);

    cout <<"callback function binding finished!"<<endl;

    // save result position to file
    mOutputPositionFile.open ("./results/fused_position.txt");

}

bool ROS_IO_Manager::initOptimizationGraph()
{
    //策略更改：只要有消息，就立刻开始。
    return true;
}

bool ROS_IO_Manager::doUpdateOptimizationGraph()
{
    //here we do update the graph.ros::spinOnce has been called.
    //check if we have necessary msgs.
    //return true if optimize success.
    bool retval = false;
    {
        if(this->_slam_msg_update)
        {
            this->pGraph->addBlockSLAM(this->SLAM_buffer.size()-1);
            this->_slam_msg_update = false;
        }

        if(this->_gps_pos_update)
        {
            this->pGraph->addBlockGPS(this->GPS_buffer.size()-1);   //do update.
            this->_gps_pos_update = false;
        }

        if(this->_gps_vel_update)
        {
            this->pGraph->addBlockVelocity(this->Velocity_buffer.size()-1);
            this->_gps_vel_update = false;
        }

        if(this->_barometer_msg_update)
        {
            this->pGraph->addBlockBarometer(this->Barometer_buffer.size()-1);
            this->_barometer_msg_update = false;
        }

        //do optimization.
        retval = true;//retval = this->pGraph->doOptimization();//TODO:fix this.
        this->slam_buf_mutex.unlock();
        this->gps_vel_mutex.unlock();
        this->gps_pos_mutex.unlock();
    }
    return retval;
}

void ROS_IO_Manager::GPS_callback(const sensor_msgs::NavSatFix& GPS_msg)
{
    this->GPS_buffer.onCallbackBlock(GPS_msg);
}

void ROS_IO_Manager::SLAM_callback(const geometry_msgs::PoseStamped& SLAM_msg)
{
    this->SLAM_buffer.onCallbackBlock(SLAM_msg);
}

void ROS_IO_Manager::gt_callback(const nav_msgs::Odometry& msg)
{
    //not implemented.
}

bool ROS_IO_Manager::publishAll()
{
    auto info = this->pGraph->queryCurrentFullStatus(); // TODO: query the latest information

    //specify what to publish.
    //1.bool status_ok.
    //2.current R quaternion
    //3.current translation vector
    //4.std_msgs::Header( especially timestamp) of this pose.
    //5.inner id inside GlobalOptimizationGraph.
    //

    if (info.state_correct)//check status;
    {
        roseus::StringStamped pub_msg;//TODO:publish relative topics.
        pub_msg.header = info.header_;
        stringstream ss;
        ss<<info.innerID_of_GOG<<"|"<<info.ret_val_R.x()<<","<<info.ret_val_R.y()<<","<<info.ret_val_R.z()<<","<<info.ret_val_R.w()<<"|"<<info.ret_val_t[0]<<","<<info.ret_val_t[1]<<","<<info.ret_val_t[2];
        //std::string pub_string = string(info.innerID_of_GOG)+ string("|")+string(info.ret_val_R.w())+","+string(info.ret_val_R.x())+","+string(info.ret_val_R.y())+","+string(ret_val_R.z())+"|"+string(info.ret_val_t[0])+","+string(info.ret_val_t[1])+","+string(info.ret_val_t[2]);
        std::string pub_string;
        ss>>pub_string;
        LOG(INFO)<<"Current full status:"<<pub_string<<endl;
        pub_msg.data = pub_string;

        this->state_string_publisher.publish(pub_msg);

        //NOTE for comparing results and debugging
        mOutputPositionFile << info.ret_val_t[0]<<","<<info.ret_val_t[1]<<","<<info.ret_val_t[2]<<endl;

        nav_msgs::Odometry gog_result;
        gog_result.header = info.header_;
        gog_result.header.frame_id = "map";
        gog_result.pose.pose.position.x = info.ret_val_t[0];
        gog_result.pose.pose.position.y = info.ret_val_t[1];
        gog_result.pose.pose.position.z = info.ret_val_t[2];
        gog_result.pose.pose.orientation.w = info.ret_val_R.w();
        gog_result.pose.pose.orientation.x = info.ret_val_R.x();
        gog_result.pose.pose.orientation.y = info.ret_val_R.y();
        gog_result.pose.pose.orientation.z = info.ret_val_R.z();
        gog_odometry.publish(gog_result);
    }
    else
    {
        LOG(WARNING)<<"Global Optimization Graph estimation state incorrect.In ROS_IO_Manager::publishAll()."<<endl;
    }
}