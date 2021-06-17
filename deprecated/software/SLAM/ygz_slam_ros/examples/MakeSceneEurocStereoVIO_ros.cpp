/**
 * This is the Euroc stereo visual-inertial odometry program
 * Please specify the dataset directory in the config file
*/
#include <math.h>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "ygz/System.h"
#include "ygz/EurocReader.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h> //for DJI.


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h> //for apm and pixhawk.

#include <visualization_msgs/Marker.h> //for visualization.

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>
#include <math.h>

#include <geometry_msgs/PoseStamped.h> //for px4's external pose estimate
#include <sensor_msgs/Imu.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


//add scene retrieving.
#include "ygz/scene_retrieve.h"
#include "LoopClosingManager.h"

#include <signal.h>
#include <csignal>

#include <DBoW3/DBoW3.h>


using namespace DBoW3;
using namespace std;
using namespace ygz;



//Scene Pointer
Scene* pScene = NULL;


//NOTE do we want to publish vision estimated quaternion
bool publishVisionQuaternion = true;

//NOTE mode switch, do we want to publish SLAM output pose to /mavros/vision_pose/pose
bool publishSLAM = false;

sensor_msgs::Imu pixhawk_imu_data;
double pixhawk_heading, pixhawk_heading_rad;
bool IMU_valid = false;
const double pi = 3.14159265359;

VecIMU temp_vimu;
cv_bridge::CvImageConstPtr cv_ptrLeft;
cv_bridge::CvImageConstPtr cv_ptrRight;
System * pSystem;

struct GPS_pos
{
  double x;
  double y;
  double z;
  int time_ms;
};


typedef std::vector<GPS_pos> VecGPS;
VecGPS gps_list;

std::vector<double> HeadingVec;
int imu_idx = 0;

double init_longitude;
double init_latitude;
double init_altitude;
bool long_lat_ever_init;

VecAtti atti_list;
VehicleAttitude init_atti;
bool atti_ever_init;

double init_height;
typedef std::vector<double> VecHeight;
VecHeight height_list;
bool height_ever_init;
void FetchHeightCallback(const double);

ros::Publisher* pVisualOdomPublisher;
ros::Publisher* pExternalEstimate;
ros::Publisher* SLAMpose;
ros::Publisher* pFakeGPS;

int VisualOdomMSGindex;

SE3d pos_and_atti;

cv::Mat M1l, M2l, M1r, M2r;

cv::Mat CurrentLeftImage, CurrentRightImage;


#define GD_semiMajorAxis 6378.137000000
#define GD_TranMercB     6356.752314245
#define GD_geocentF      0.003352810664


mavros_msgs::State current_state;

size_t frame_index = 0;

std::ofstream SlamPoseHistory, px4PoseHistory;

vector<cv::Mat> VecLeftImage, VecRightImage;

DBoW3::Database cur_frame_db;

LoopClosingManager* pLoopClosingManager;

void mySigintHandler(int sig)
{
    {
        pScene->saveFile("./image/scene.scn");
        pScene->saveVoc();
        
        cout<<"Saving Scene.cn and Voc finished!"<<endl;

        cur_frame_db.save("database.bin");
        
         for(int i=0; i<VecLeftImage.size(); i++)
         {
             cv::imwrite("./image/left/"+to_string(i)+".png", VecLeftImage[i].clone() );
             cv::imwrite("./image/right/"+to_string(i)+".png", VecRightImage[i].clone() );
         }
    }
    
    cout << "All done"<<endl;
    
    ros::shutdown();
}


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void geodeticOffsetInv( double refLat, double refLon,
                        double lat,    double lon, 
                        double& xOffset, double& yOffset )
{
    double a = GD_semiMajorAxis;
    double b = GD_TranMercB;
    double f = GD_geocentF;

    double L     = lon-refLon;
    double U1    = atan((1-f) * tan(refLat));
    double U2    = atan((1-f) * tan(lat));
    double sinU1 = sin(U1); 
    double cosU1 = cos(U1);
    double sinU2 = sin(U2);
    double cosU2 = cos(U2);

    double lambda = L;
    double lambdaP;
    double sinSigma;
    double sigma;
    double cosSigma;
    double cosSqAlpha;
    double cos2SigmaM;
    double sinLambda;
    double cosLambda;
    double sinAlpha;
    int iterLimit = 100;
    do {
        sinLambda = sin(lambda);
        cosLambda = cos(lambda);
        sinSigma = sqrt((cosU2*sinLambda) * (cosU2*sinLambda) + 
                        (cosU1*sinU2-sinU1*cosU2*cosLambda) * 
                        (cosU1*sinU2-sinU1*cosU2*cosLambda) );
        if (sinSigma==0)
        {
            xOffset = 0.0;
            yOffset = 0.0;
            return ;  // co-incident points
        }
        cosSigma    = sinU1*sinU2 + cosU1*cosU2*cosLambda;
        sigma       = atan2(sinSigma, cosSigma);
        sinAlpha    = cosU1 * cosU2 * sinLambda / sinSigma;
        cosSqAlpha  = 1 - sinAlpha*sinAlpha;
        cos2SigmaM  = cosSigma - 2*sinU1*sinU2/cosSqAlpha;
        if (cos2SigmaM != cos2SigmaM) //isNaN
        {
            cos2SigmaM = 0;  // equatorial line: cosSqAlpha=0 (§6)
        }
        double C = f/16*cosSqAlpha*(4+f*(4-3*cosSqAlpha));
        lambdaP = lambda;
        lambda = L + (1-C) * f * sinAlpha *
            (sigma + C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)));
    } while (fabs(lambda-lambdaP) > 1e-12 && --iterLimit>0);

    if (iterLimit==0)
    {
        xOffset = 0.0;
        yOffset = 0.0;
        return;  // formula failed to converge
    }

    double uSq  = cosSqAlpha * (a*a - b*b) / (b*b);
    double A    = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
    double B    = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));
    double deltaSigma = B*sinSigma*(cos2SigmaM+B/4*(cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)-
        B/6*cos2SigmaM*(-3+4*sinSigma*sinSigma)*(-3+4*cos2SigmaM*cos2SigmaM)));
    double s = b*A*(sigma-deltaSigma);

    double bearing = atan2(cosU2*sinLambda,  cosU1*sinU2-sinU1*cosU2*cosLambda);
    xOffset = sin(bearing)*s*1000.0;
    yOffset = cos(bearing)*s*1000.0;
}


void FetchImuCallback(const sensor_msgs::Imu& imu)
{
  //LOG(INFO) << "fetching imu" << endl;
  IMUData t_imu(imu.angular_velocity.x,
		imu.angular_velocity.y,
		imu.angular_velocity.z,
		imu.linear_acceleration.x,
		imu.linear_acceleration.y,
		imu.linear_acceleration.z,
		imu.header.stamp.toNSec() );
  
  temp_vimu.push_back(t_imu);
  //LOG(INFO) << "fecthing imu2" << endl;
}



void Display(cv_bridge::CvImageConstPtr cv_ptrLeft, cv_bridge::CvImageConstPtr cv_ptrRight)
{
  cv::imshow("Left Image", cv_ptrLeft->image);
  cv::imshow("Right Image", cv_ptrRight->image);
  cv::waitKey(5);
}

void FetchGPSCallback(const sensor_msgs::NavSatFix& gps_info)
{
  //cout<<"Got NavSatFix info!!!!!!"<<endl;
  GPS_pos new_pos;
  if(!long_lat_ever_init)
  {
    init_longitude = gps_info.longitude;
    init_latitude = gps_info.latitude;
    init_altitude = gps_info.altitude;
    long_lat_ever_init = true;
    cout<<"GPS init pos recorded!";
  }
  double newx,newy;
  geodeticOffsetInv(init_latitude,init_longitude,gps_info.latitude,gps_info.longitude,newx,newy);
  
  new_pos.x = newx;
  new_pos.y = newy;
  new_pos.z = gps_info.altitude - init_altitude;
  new_pos.time_ms = gps_info.header.stamp.toNSec();
  
//   cout <<"GPS_POS:"<<newx<<" "<<newy<<" "<<new_pos.z<<endl;
  gps_list.push_back(new_pos);
}

void set_attitude_by_msg_dji(const geometry_msgs::QuaternionStamped& msg,VehicleAttitude& atti,bool do_reform = false)
{
    atti.q.x() = msg.quaternion.x;
    atti.q.y() = msg.quaternion.y;
    atti.q.z() = msg.quaternion.z;
    atti.q.w() = msg.quaternion.w;
    if(do_reform)
    {
      atti.q = atti.q*(init_atti.q.inverse());
    }
    //LOG(WARNING)<<"Attitude input:\n\n"<<endl;
    //LOG(WARNING)<<atti.q.toRotationMatrix()<<endl;
    atti.time_ms = msg.header.stamp.toNSec();
}

void set_attitude_by_msg_px4(const nav_msgs::Odometry& msg,VehicleAttitude& atti,bool do_reform = false)
{
    atti.q.x() = msg.pose.pose.orientation.x;
    atti.q.y() = msg.pose.pose.orientation.y;
    atti.q.z() = msg.pose.pose.orientation.z;
    atti.q.w() = msg.pose.pose.orientation.w;
    if(do_reform)
    {
      atti.q = atti.q*(init_atti.q.inverse());
    }
    //LOG(WARNING)<<"Attitude input:\n\n"<<endl;
    //LOG(WARNING)<<atti.q.toRotationMatrix()<<endl;
    atti.time_ms = msg.header.stamp.toNSec();
}

void FetchAttitudeCallback_dji(const geometry_msgs::QuaternionStamped& atti_msg)
{
  //cout<<"Got atti_msg!!"<<endl;
  if(!atti_ever_init)
  {
    set_attitude_by_msg_dji(atti_msg,init_atti,false);
    atti_ever_init = true;
    return;
  }
  VehicleAttitude new_atti;
  set_attitude_by_msg_dji(atti_msg,new_atti,true);
  atti_list.push_back(new_atti);
  //cout<<"atti_msg pushed into list!"<<endl;
  
}

void FetchAttitudeCallback_px4(const nav_msgs::Odometry& atti_msg)
{
    //cout<<"Got atti_msg!!"<<endl;
  FetchHeightCallback(atti_msg.pose.pose.position.z);
  if(!atti_ever_init)
  {
    set_attitude_by_msg_px4(atti_msg,init_atti,false);
    atti_ever_init = true;
    return;
  }
  VehicleAttitude new_atti;
  set_attitude_by_msg_px4(atti_msg,new_atti,true);
  atti_list.push_back(new_atti);
  
  //cout<<"atti_msg pushed into list!"<<endl;
}

void FetchHeightCallback(const double height)
{
  if(!height_ever_init)
  {
    init_height = height;
    height_ever_init = true;
    return;
  }
  height_list.push_back(height-init_height);
}

void deg2rad(double heading_deg)
{
  pixhawk_heading_rad =  (heading_deg * pi)/180;

}


void pixhawkIMU_sub(const sensor_msgs::Imu &curQ)
{
    pixhawk_imu_data = curQ; 
    
    //decode heading information from pixhawk onbaord IMU quaternion
        double w = curQ.orientation.w;
        double z_in = curQ.orientation.z;

        double check_NS = w*z_in;
        double w_abs = abs(w);
        double z_in_abs = abs(z_in);
        double heading_deg = 180.0*(2*acos(w_abs)/pi) -180;
        if(check_NS>0)
        {
                heading_deg*=-1.0;
        }
        heading_deg+=270;
        if (heading_deg>360)
        {
                heading_deg-=360;
        }
        
        //ROS_INFO("Current Heading is : %f", heading_deg);

        //ignore the first 20 readings and only use values from [20, 60)
        if(40 >= imu_idx && imu_idx >= 20)
        {
            HeadingVec.push_back(heading_deg);
            ROS_INFO("Heading received: %f", heading_deg);
        }
        //average the values we got
        else if(imu_idx == 41)
        {   
            float sum = 0;
            for(auto n : HeadingVec)
            {
                sum += n;
            }

            pixhawk_heading = sum/HeadingVec.size();
            ROS_INFO("Heading set to: %f", pixhawk_heading);

            //set global heading in rad
            deg2rad(pixhawk_heading);        

	    //IMU data received, set it valid                
	    IMU_valid = true;
        } 

     imu_idx++;
     
}

void TEMP_FetchImageAndAttitudeCallback(const sensor_msgs::ImageConstPtr& msgLeft ,const sensor_msgs::ImageConstPtr &msgRight,const geometry_msgs::PoseStamped &posemsg)
{   
    
    System& system = *pSystem;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
//    cv::Mat CurrentLeftImage, CurrentRightImage;

    //cv::remap(cv_ptrLeft->image, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
    //cv::remap(cv_ptrRight->image, imRightRect, M1r, M2r, cv::INTER_LINEAR);

    CurrentLeftImage = cv_ptrLeft->image;
    CurrentRightImage = cv_ptrRight->image;

    VehicleAttitude atti;
    atti.q.x() = posemsg.pose.orientation.x;
    atti.q.y() = posemsg.pose.orientation.y;
    atti.q.z() = posemsg.pose.orientation.z;
    atti.q.w() = posemsg.pose.orientation.w;
    /*if(do_reform)
    {
        atti.q = atti.q*(init_atti.q.inverse());
    }*/
    //LOG(WARNING)<<"Attitude input:\n\n"<<endl;
    //LOG(WARNING)<<atti.q.toRotationMatrix()<<endl;
    atti.time_ms = posemsg.header.stamp.toNSec();
    pos_and_atti = system.AddStereoIMU(CurrentLeftImage, CurrentRightImage, cv_ptrLeft->header.stamp.toNSec(),temp_vimu,
                                           0,0,0,false,atti,true,0,false);
    
    Vector3d pos = pos_and_atti.translation();
    Matrix3d mat = pos_and_atti.rotationMatrix();
    Quaterniond quat(mat);
    
    SlamPoseHistory << to_string(pos[0]) + "," + to_string(pos[1]) + "," + to_string(pos[2]) + "\n";
    //px4PoseHistory << to_string(posemsg.pose.position.x) + "," + to_string(posemsg.pose.position.y) + "," + to_string(posemsg.pose.position.z) + "\n";
    
    geometry_msgs::PoseStamped SlamPose;
    SlamPose.header.stamp = ros::Time::now();
    auto &pose_obs = SlamPose.pose.position;
    pose_obs.x = pos(0,0);
    pose_obs.y = pos(1,0);
    pose_obs.z = pos(2,0);
    auto &pose_atti = SlamPose.pose.orientation;
    pose_atti.w = quat.w();
    pose_atti.x = quat.x();
    pose_atti.y = quat.y();
    pose_atti.z = quat.z();
    SLAMpose->publish(SlamPose);
    //add scene frame to scene
    cv::Mat rotationmat,translationmat;
    cv::eigen2cv(mat,rotationmat); // inverse operation:eigen2cv.
    cv::eigen2cv(pos,translationmat);



    //NOTE not used
    cv::Mat Q_mat = (Mat_<float>(4,4) << 1, 0, 0, 0,
                                         0, 1, 0, 0,
                                         0, 0, 1, 0,
                                         0, 0, 0, 1);
    
    

    if(!rotationmat.empty() && !translationmat.empty())
    {
        cout<<"r, t, q are: "<<rotationmat<<endl<<translationmat<<endl;

        //pts2d_in    pts3d_in    desp,   R,    t
        //typedef  std::tuple<std::vector<cv::KeyPoint>, std::vector<cv::Point3d>, cv::Mat, cv::Mat, cv::Mat> SceneFrame;

        SceneFrame scene_frame = pScene->generateSceneFrameFromStereoImage(CurrentLeftImage, CurrentRightImage, rotationmat, translationmat, Q_mat);

        //save scene
        pScene->addFrame(scene_frame);

        //save database
        cur_frame_db.add(get<2>(scene_frame));

        cout<<"cur_frame_db info: "<<cur_frame_db<<endl;

        VecLeftImage.push_back(CurrentLeftImage.clone());
        VecRightImage.push_back(CurrentRightImage.clone());

    }

    frame_index++;
    temp_vimu.clear();
    
}


int main(int argc, char **argv) {
    
    pScene = new Scene();
    pScene->setHasScale(true);
    gps_list.clear();
    atti_list.clear();
    long_lat_ever_init = false;
    atti_ever_init = false;
    height_ever_init = false;
	
    FLAGS_logtostderr = true;

    if(argc != 2)
    {
        cout<<"YOU NEED TO SPECIFY CONFIG PATH!"<<endl;
        return 0;
    }
    
    SlamPoseHistory.open("./slampose.csv");
    px4PoseHistory.open("./px4pose.csv");


    google::InitGoogleLogging(argv[0]);    
    string config_path = argv[1];
    
    string configFile(config_path);
    cv::FileStorage fsSettings(configFile, cv::FileStorage::READ);
    
    System system(config_path);
    pSystem = &system;

    string left_topic = string(fsSettings["Left"]);
    string right_topic = string(fsSettings["Right"]);
    
    string cur_voc_path = string(fsSettings["VocPath"]);
    cur_frame_db = DBoW3::Database("./small_voc.yml.gz", false, 0);
    
    // rectification parameters
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() ||
        D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return 1;
    }

    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l,
                                M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r,
                                M2r);

    cv::Mat Rbc, tbc;
    fsSettings["RBC"] >> Rbc;
    fsSettings["TBC"] >> tbc;
    
    if (!Rbc.empty() && tbc.empty()) {
        Matrix3d Rbc_;
        Vector3d tbc_;
        Rbc_ <<
             Rbc.at<double>(0, 0), Rbc.at<double>(0, 1), Rbc.at<double>(0, 2), 
                Rbc.at<double>(1, 0), Rbc.at<double>(1, 1), Rbc.at<double>(1, 2), 
                Rbc.at<double>(2, 0), Rbc.at<double>(2, 1), Rbc.at<double>(2, 2); 
        tbc_ <<
             tbc.at<double>(0, 0), tbc.at<double>(1, 0), tbc.at<double>(2, 0); 

        setting::TBC = SE3d(Rbc_, tbc_);
    }    
    


    ros::init(argc, argv, "ygz_with_gps", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    
    signal(SIGINT, mySigintHandler);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, left_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, right_topic, 10);
    

    ros::Subscriber imu_sub = nh.subscribe("/mynteye/imu/data_raw", 1000, FetchImuCallback);
    ros::Subscriber gps_sub = nh.subscribe("/dji_sdk/gps_position",100,FetchGPSCallback);
    ros::Publisher ygz_odom_vis_pub = nh.advertise<visualization_msgs::Marker>("/ygz_odom_marker",10);
    pVisualOdomPublisher = &ygz_odom_vis_pub;
    VisualOdomMSGindex = 0;

    
    //for DJI device:
    //ros::Subscriber atti_sub_dji = nh.subscribe("/dji_sdk/attitude",100,FetchAttitudeCallback_dji);
    
    //for PX4 device:
    ros::Subscriber atti_sub_px4 = nh.subscribe("/mavros/global_position/local",100,FetchAttitudeCallback_px4);

    //old version
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;


    //message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    //sync.setMaxIntervalDuration(ros::Duration(0.01));
    
    //sync.registerCallback(boost::bind(FetchImageCallback, _1, _2));
    

    //new version with attitude by pixhawk mavlink msg.
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,geometry_msgs::PoseStamped> sync_pol;
    message_filters::Subscriber<geometry_msgs::PoseStamped> px4_attitude_sub(nh, "/mavros/local_position/pose", 10);
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub,px4_attitude_sub);
    sync.registerCallback(//boost::bind(
            TEMP_FetchImageAndAttitudeCallback
            //,_1,_2,_3)
            );
    
    
    //NOTE for test
    ros::Publisher slam_output_pose = nh.advertise<geometry_msgs::PoseStamped>("/SLAM/pose_for_obs_avoid",10);
    SLAMpose = &slam_output_pose;
    
    //NOTE For px4's external pose estimate
    ros::Publisher px4_external_pose_estimate = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",10);
    pExternalEstimate = &px4_external_pose_estimate;
    
    //NOTE px4 state 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
   
    //NOTE fake gps vision
    ros::Publisher px4FakeGPS = nh.advertise<geometry_msgs::PoseStamped>("/mavros/fake_gps/vision",10); 
    pFakeGPS = &px4FakeGPS;
    
    //NOTE external heading subscriber
    //ros::Subscriber heading_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, heading_cb);
    
    
    //NOTE drone onboard imu data
    ros::Subscriber pixhawk_imu_sub = nh.subscribe("/mavros/imu/data", 5, pixhawkIMU_sub);
    
    
    ros::spin();

    return 0;
    
}
