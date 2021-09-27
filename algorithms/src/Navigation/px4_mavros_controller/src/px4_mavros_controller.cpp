//隔离飞行指令在飞控和mavros的底层实现,GAAS_contrib的所有部分全部用px4_mavros_controller提供的API.
#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>


using std::cout;
using std::endl;
using std::string;

class PX4MavrosController
{
public:
    PX4MavrosController()
    {
        ;
    }
    bool set_hover()//设定当前位置即可.
    {
        if(ahrs_info_ever_init && gps_info_ever_init)
        {
            //            sensor_msgs::NavSatFix current_gps_msg;
            //            //set gps target.
            //            mavros_msgs::PositionTarget target;
            //            target.header.stamp = ros::Time::now();
            //            target.header.frame_id = "lidar";// 或"lidar"
            //            target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            //            target.position.x = 0;
            //            target.position.y = 0;
            //            target.position.z = 0;
            //            target.type_mask = target.IGNORE_AFX+target.IGNORE_AFY+target.IGNORE_AFZ+target.IGNORE_VX+target.IGNORE_VY+target.IGNORE_VZ;
            //            target.yaw =current_yaw; //保持当前航向角不动.

            //            current_position_mutex.lock();
            //            this->current_position_target = target;
            //            current_position_mutex.unlock();

            set_current_target_hover();
            for(int i = 0;i<20;i++)
            {
                this->mavros_target_position_pub.publish(this->current_position_target);//发布一个状态.
                usleep(20000);
            }

            return true;
        }
        else
        {
            LOG(WARNING)<<"Try set_hover before ahrs and gps init!"<<endl;
            return false;
        }
    }
    bool set_current_target_hover()//设置相对机体坐标系的局部坐标系
    {
        geometry_msgs::PoseStamped curr_gaas_pose;
        this->current_gaas_location_mutex.lock();
        curr_gaas_pose = current_gaas_location;
        this->current_gaas_location_mutex.unlock();

        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.header.frame_id = "map";// map或"lidar"
        //PX4只支持mavros的 FRAME_LOCAL_NED(Local coordinate frame, Z-down (x: North, y: East, z: Down).)
        //和
        //FRAME_BODY_NED(Setpoint in body NED frame(body FRD). This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.).
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        //NWU->ENU
        target.position.x = current_ahrs.pose.pose.position.x;
        target.position.y = current_ahrs.pose.pose.position.y;
        target.position.z = current_ahrs.pose.pose.position.z;
        target.type_mask = target.IGNORE_AFX|target.IGNORE_AFY|target.IGNORE_AFZ|target.IGNORE_VX|target.IGNORE_VY|target.IGNORE_VZ;
        target.yaw = current_yaw;
        this->current_position_mutex.lock();
        this->current_position_target = target;
        this->current_position_mutex.unlock();
        return true;

    }
    bool set_target_local(const geometry_msgs::PoseStamped& target_input)//设置相对机体坐标系的局部坐标系
    {
        geometry_msgs::PoseStamped curr_gaas_pose;
        this->current_gaas_location_mutex.lock();
        curr_gaas_pose = current_gaas_location;
        this->current_gaas_location_mutex.unlock();

        //step<1>.current_position to target position in gaas coordinate;
        auto body_pos = target_input.pose.position;



        //step<2>.gaas_position to local coordinate

        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.header.frame_id = "map";// map或"lidar"
        //PX4只支持mavros的 FRAME_LOCAL_NED(Local coordinate frame, Z-down (x: North, y: East, z: Down).)
        //和
        //FRAME_BODY_NED(Setpoint in body NED frame(body FRD). This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.).
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        //NWU->ENU
        target.position.x = current_ahrs.pose.pose.position.x+body_pos.x*cos(current_yaw)-body_pos.y*sin(current_yaw);
        target.position.y = current_ahrs.pose.pose.position.y+body_pos.x*sin(current_yaw)+body_pos.y*cos(current_yaw);
        target.position.z = current_ahrs.pose.pose.position.z+body_pos.z;
        target.type_mask = target.IGNORE_AFX|target.IGNORE_AFY|target.IGNORE_AFZ|target.IGNORE_VX|target.IGNORE_VY|target.IGNORE_VZ;
        target.yaw = current_yaw;
        this->current_position_mutex.lock();
        this->current_position_target = target;
        this->current_position_mutex.unlock();
        return true;

    }
    void waitForAll()
    {
        int i = 0;
        while(! (gps_info_ever_init&&ahrs_info_ever_init&&gaas_location_ever_init&&current_state_ever_init)&&ros::ok())
        {
            if(i%5 == 0)
            {
                LOG(INFO)<<"In waitForAll(): still waiting for some topics..."<<endl;
            }
            ros::spinOnce();
            i++;
            usleep(20000);
        }
        if(ros::ok)
        {
            LOG(INFO)<<"GPS AHRS GAAS_Localization MAVROS_STATE init finished!"<<endl;
        }
    }
    bool set_target_position_map_to_local_relative(const geometry_msgs::PoseStamped& target_input) //设置地图坐标系转换过的位置.
    {
        if(!gaas_location_ever_init)
        {
            return false;
        }
        geometry_msgs::PoseStamped curr_gaas_pose;
        this->current_gaas_location_mutex.lock();
        curr_gaas_pose = this->current_gaas_location;
        this->current_gaas_location_mutex.unlock();

        double dn,dw,du;

        dn = target_input.pose.position.x - curr_gaas_pose.pose.position.x;
        dw = target_input.pose.position.y - curr_gaas_pose.pose.position.y;
        du = target_input.pose.position.z - curr_gaas_pose.pose.position.z;

        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.header.frame_id = "map";// map或"lidar"
        //PX4只支持mavros的 FRAME_LOCAL_NED(Local coordinate frame, Z-down (x: North, y: East, z: Down).)
        //和
        //FRAME_BODY_NED(Setpoint in body NED frame(body FRD). This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.).
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        //NWU->ENU
        target.position.x = -1*(dw - current_ahrs.pose.pose.position.x);
        target.position.y = (dn + current_ahrs.pose.pose.position.y);
        target.position.z = (du + current_ahrs.pose.pose.position.z);
        target.type_mask = target.IGNORE_AFX|target.IGNORE_AFY|target.IGNORE_AFZ|target.IGNORE_VX|target.IGNORE_VY|target.IGNORE_VZ;
        target.yaw = current_yaw;
        this->current_position_mutex.lock();
        this->current_position_target = target;
        this->current_position_mutex.unlock();
    }
    void initController(int main_argc,char** main_argv)
    {
        FLAGS_alsologtostderr = 1;
        google::InitGoogleLogging(main_argv[0]);
        LOG(INFO)<<"Start px4_mavros_controller_node."<<endl;
        ros::init(main_argc,main_argv,"px4_mavros_controller_node");
        ros::NodeHandle nh;
        pnode_handle = &nh;

        //尝试加载地图.
        if(load_map())
        {
            map_info_valid = true;
            LOG(INFO)<<"Map loaded successfully!"<<endl;
        }


        ahrs_sub = pnode_handle->subscribe<nav_msgs::Odometry>("/mavros/global_position/local",1,&PX4MavrosController::ahrs_callback,this);
        gps_sub = pnode_handle->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/raw/fix",1,&PX4MavrosController::gps_callback,this);
        mavros_state_sub = pnode_handle->subscribe<mavros_msgs::State>("/mavros/state",1,&PX4MavrosController::mavros_state_callback,this);

        gaas_target_position_sub = pnode_handle->subscribe<geometry_msgs::PoseStamped>("/gaas/navigation/target_position",1,
                                                                                       &PX4MavrosController::target_position_callback,this);
        gaas_target_yaw_sub = pnode_handle->subscribe<std_msgs::Float32>("/gaas/navigation/target_enu_yaw",1,
                                                                         &PX4MavrosController::target_yaw_callback,this);
        gaas_localization_sub = pnode_handle->subscribe<geometry_msgs::PoseStamped>("/gaas/localization/registration_pose",1,&PX4MavrosController::current_gaas_location_callback,this);
        mavros_target_position_pub = pnode_handle->advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",1);
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");


        while(ros::ok() && !current_state.connected)
        {//initialize mavros connection;
            ros::spinOnce();
            usleep(200000);//200ms
            LOG(INFO)<<"Waiting for initialization."<<endl;
        }
        while(ros::ok()&&!set_mode_client.exists())
        {
            LOG(INFO)<<"Waiting for mavros_set_mode service."<<endl;
            usleep(20000);//20ms.
        }
        waitForAll();//等待所有消息都收到一次.
        mavros_msgs::SetMode position_set_mode;
        position_set_mode.request.custom_mode = "POSCTL";

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        //Failsafe:先进入POSCTL,这样如果里面OFFBOARD意外掉了是回到POSITION模式.
	position_set_mode.response.mode_sent = false;
	bool position_set_res = false;
	while(!position_set_mode.response.mode_sent&&ros::ok())
	{
            position_set_res = set_mode_client.call(position_set_mode);
	    ros::spinOnce();
            LOG(INFO)<<"set flight mode service result:"<<position_set_res<<endl;
            LOG(INFO)<<"set flight mode service mode_sent:"<<(int)position_set_mode.response.mode_sent<<endl;
	    if(!position_set_mode.response.mode_sent)
	    {
                LOG(WARNING)<<"set offboard mode failed; Will retry later."<<endl;
	    }
	    usleep(10000);//10ms
	}
        if(current_state.mode!="OFFBOARD"&&current_state.mode!="POSCTL")//保证任何时候都能重新启动这个服务而不出问题.
        {
            if( position_set_res && position_set_mode.response.mode_sent)
            {
                LOG(INFO)<<"POSCTL_MODE enabled.";
            }
            else
            {
                LOG(ERROR)<<"Enter POSCTL_MODE failed.";
                exit(-1);
            }
        }

        ros::spinOnce();
        while(!set_hover()&&ros::ok())
        {
            ros::spinOnce();
            this->mavros_target_position_pub.publish(this->current_position_target);//发布一个状态.
            usleep(20000);
        }

        //真正进OFFBOARD模式.
        if(current_state.mode!="OFFBOARD")
        {
            bool offboard_set_res = set_mode_client.call(offb_set_mode);
            if( offboard_set_res && offb_set_mode.response.mode_sent)
            {
                while(current_state.mode!="OFFBOARD"&&ros::ok())
                {
                    ros::spinOnce();//update current_state;
                    LOG(INFO)<<"waiting for entering offboard mode...";
                    set_hover();
                    usleep(20000);
                }
                if(current_state.mode == "OFFBOARD")
                {
                    LOG(INFO)<<"OFFBOARD_MODE enabled.";
                }
            }
            else
            {
                LOG(ERROR)<<"Enter OFFBOARD_MODE failed.";
                exit(-1);
            }
        }
        //成功进入offboard,尝试arm.
        if( !current_state.armed)//保证任何时候都能重新启动这个服务而不出问题.
        {
            if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
            {
                LOG(INFO)<<"Vehicle armed."<<endl;
            }
            else
            {
                LOG(ERROR)<<"Vehicle arming failed."<<endl;
                exit(-1);
            }
        }

        LOG(INFO)<<"PX4 Mavros controller ready."<<endl;
        //Controller已经就位,可以根据请求进入循环.

	//add initial height to avoid terrain obstacle caused astar no path error.
        if(current_gaas_location.pose.position.z<=0.6)
        {
            geometry_msgs::PoseStamped takeoff_initial_pose;
            takeoff_initial_pose.header.frame_id = "lidar";
            takeoff_initial_pose.pose.position.x = 0;
            takeoff_initial_pose.pose.position.y = 0;
            takeoff_initial_pose.pose.position.z = 2.0;
            takeoff_initial_pose.pose.orientation.w = 1.0;
            set_target_local(takeoff_initial_pose);
        }
    }
    void run_controller_node_loop()
    {
        while(ros::ok())
        {
            if(current_state.mode!="OFFBOARD")
            {
                LOG(WARNING)<<"Current state is not OFFBOARD!!!"<<endl;
            }
            ros::spinOnce();
            //if(target_reached())
            //{
            this->current_position_mutex.lock();
            this->mavros_target_position_pub.publish(this->current_position_target);//发布一个状态.
            this->current_position_mutex.unlock();
            //}
            usleep(20000);//20ms响应一次.
        }
    }
private:
    ros::NodeHandle* pnode_handle;
    ros::Subscriber ahrs_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber mavros_state_sub;

    ros::Subscriber gaas_target_position_sub;
    ros::Subscriber gaas_target_yaw_sub;
    ros::Subscriber gaas_custom_activity_sub;
    ros::Subscriber gaas_localization_sub;
    ros::Publisher mavros_target_position_pub;
    ros::ServiceClient set_mode_client,arming_client;

    mavros_msgs::State current_state;
    bool current_state_ever_init = false;

    sensor_msgs::NavSatFix current_gps;
    bool gps_info_ever_init = false;

    nav_msgs::Odometry current_ahrs;

    double current_yaw,current_pitch,current_roll;
    bool ahrs_info_ever_init = false;
    std::mutex gps_mutex,ahrs_mutex,state_mutex;

    float current_target_yaw;


    std::mutex current_position_mutex;
    mavros_msgs::PositionTarget current_position_target;


    std::mutex current_gaas_location_mutex;
    geometry_msgs::PoseStamped current_gaas_location;
    bool gaas_location_ever_init = false;

    //map info,not strictly required.
    bool map_info_valid = false;
    double map_init_lon=0,map_init_lat=0,map_init_alt=0;


    void ahrs_callback(const nav_msgs::OdometryConstPtr& ahrs_msg);
    void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg);
    void mavros_state_callback(const mavros_msgs::State::ConstPtr& msg);

    void target_position_callback(const geometry_msgs::PoseStampedConstPtr& target_position_msg);
    void target_yaw_callback(const std_msgs::Float32ConstPtr& target_yaw);//piece of shit syntax.
    void current_gaas_location_callback(const geometry_msgs::PoseStampedConstPtr& current_gaas_position_msg);
    bool target_reached();//检查是否已经接近目标.判断目标的类型,坐标系,然后判断自身位置.
    bool load_map()
    {
        string map_path;
        if(!ros::param::get("map_path",map_path))
        {
            LOG(ERROR)<<"No map path."<<endl;
            return false;
        }
        string map_gps_path = map_path+".yaml";
        cv::FileStorage fs;
        fs.open(map_gps_path,cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            LOG(ERROR)<<"ERROR:Map path not found."<<endl;
            return false;
        }
        fs["initial_longitude"]>>map_init_lon;
        fs["initial_latitude"]>>map_init_lat;
        fs["initial_altitude"]>>map_init_alt;
        string map_coordinate_mode;
        fs["coordinate_mode"]>>map_coordinate_mode;
        if(map_coordinate_mode!="NWU")
        {
            LOG(ERROR)<<"ERROR:Map coordinate mode is not NWU.Map not loaded!";
            return false;
        }
        return true;
    }
};
void PX4MavrosController::ahrs_callback(const nav_msgs::OdometryConstPtr& ahrs_msg)//更新ahrs信息和current yaw pitch roll
{
    //LOG(INFO)<<"ahrs callback"<<endl;
    ahrs_mutex.lock();
    current_ahrs = *ahrs_msg;
    float x_,y_,z_,w_;
    x_ = current_ahrs.pose.pose.orientation.x;
    y_ = current_ahrs.pose.pose.orientation.y;
    z_ = current_ahrs.pose.pose.orientation.z;
    w_ = current_ahrs.pose.pose.orientation.w;
    tf2::Quaternion q(x_,y_,z_,w_);
    tf2::Matrix3x3 mat(q);
    double current_yaw_plus_90_deg;
    mat.getEulerYPR(current_yaw_plus_90_deg,current_pitch,current_roll);
    current_yaw = current_yaw_plus_90_deg;// - 0.5*3.141592535;
    ahrs_info_ever_init = true;
    ahrs_mutex.unlock();
}
void PX4MavrosController::gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg)
{
    //LOG(INFO)<<"gps callback"<<endl;
    gps_mutex.lock();
    current_gps = *gps_msg;
    gps_info_ever_init = true;
    gps_mutex.unlock();
}
void PX4MavrosController::target_yaw_callback(const std_msgs::Float32ConstPtr& target_yaw)
{
    current_target_yaw = target_yaw->data;
    this->current_position_mutex.lock();
    this->current_position_target.yaw = current_target_yaw;
    this->current_position_mutex.unlock();
    return;
}
void PX4MavrosController::mavros_state_callback(const mavros_msgs::State::ConstPtr& msg)
{
    //LOG(INFO)<<"in mavros_state callback"<<endl;
    state_mutex.lock();
    current_state = *msg;
    current_state_ever_init = true;
    state_mutex.unlock();
}
void PX4MavrosController::target_position_callback(const geometry_msgs::PoseStampedConstPtr& target_position_msg)
{
    //输入目标的参考坐标系有两个: lidar/map.

    //如果输入是lidar系，按照body的FLU坐标系运动;
    //如果是map系，运动到map坐标系的相应位置.

    LOG(INFO)<<"New target position:"<<target_position_msg->pose.position.x<<","<<
               target_position_msg->pose.position.y<<","<<target_position_msg->pose.position.z<<endl;
    state_mutex.lock();
    mavros_msgs::State curr_state = current_state;
    state_mutex.unlock();
    if(!(curr_state.mode=="POSCTL"||curr_state.mode=="OFFBOARD"))
    {
        LOG(ERROR)<<"ERROR: not in POSITION or OFFBOARD mode, can not respond to target position!"<<endl;
        return;
    }
    if(target_position_msg->header.frame_id=="lidar")//相对body的坐标变换.
    {
        if(curr_state.mode=="POSCTL")
        {
            //这种是处理不了的.定点模式无法控制.
            LOG(ERROR)<<"In POSCTL mode, can not respond setting target request."<<endl;
            return;
            //set_target_position_local_to_global_gps(*target_position_msg);//定点模式，生成gps坐标.
        }
        else if(curr_state.mode=="OFFBOARD")
        {
            //offboard模式 body变换:直接设置相对坐标.
            //            mavros_msgs::PositionTarget target;
            //            target.header.stamp = ros::Time::now();
            //            target.header.frame_id = "lidar";// 或"lidar"
            //            //PX4只支持mavros的 FRAME_LOCAL_NED(Local coordinate frame, Z-down (x: North, y: East, z: Down).)
            //            //和
            //            //FRAME_BODY_NED(Setpoint in body NED frame(body FRD). This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.).
            //            target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            //            target.position.x = target_position_msg->pose.position.x;
            //            target.position.y = target_position_msg->pose.position.y;
            //            target.position.z = target_position_msg->pose.position.z;
            //            target.type_mask = target.IGNORE_AFX|target.IGNORE_AFY|target.IGNORE_AFZ|target.IGNORE_VX|target.IGNORE_VY|target.IGNORE_VZ;
            //            target.yaw = current_yaw;
            //            current_position_mutex.lock();
            //            this->current_position_target = target;
            //            current_position_mutex.unlock();
            set_target_local(*target_position_msg);
        }
    }
    else if(target_position_msg->header.frame_id == "map")//指定map NWU的绝对坐标.
    {
        if(!map_info_valid)
        {
            LOG(ERROR)<<"ERROR:Map not loaded;Can not handle map target!"<<endl;
            throw "ERROR";
        }
        //
        if(curr_state.mode == "POSCTL")
        {
            //map坐标转gps坐标.
            //这种是处理不了的.定点模式无法控制.
            LOG(ERROR)<<"In POSCTL mode, can not respond setting target request."<<endl;
            return;
        }
        else if(curr_state.mode == "OFFBOARD")
        {
            //根据现在的map坐标(定位模块)，输出相对位置关系
            set_target_position_map_to_local_relative(*target_position_msg);
        }
    }
}
bool PX4MavrosController::target_reached()
{
    return false;//TODO.
}
void PX4MavrosController::current_gaas_location_callback(const geometry_msgs::PoseStampedConstPtr& current_gaas_position_msg)
{
    this->current_gaas_location_mutex.lock();
    this->current_gaas_location = *current_gaas_position_msg;
    this->gaas_location_ever_init=true;
    this->current_gaas_location_mutex.unlock();
}

int main(int argc,char** argv)
{
    PX4MavrosController px4controller_node;
    px4controller_node.initController(argc,argv);
    px4controller_node.run_controller_node_loop();
    return 0;
}
