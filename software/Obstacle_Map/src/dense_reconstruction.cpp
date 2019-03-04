#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <stereo_dense_reconstruction/CamToRobotCalibParamsConfig.h>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "popt_pp.h"

using namespace cv;
using namespace std;

Mat XR, XT, Q, P1, P2,R_downward,T_downward;
Mat R1, R2, K1, K2, D1, D2, R;
Mat lmapx, lmapy, rmapx, rmapy;
Vec3d T;
double min_x,max_x,min_y,max_y;

stereo_dense_reconstruction::CamToRobotCalibParamsConfig config;
FileStorage calib_file;
int debug = 1;
Size out_img_size;
Size calib_img_size;

int disparity_method;

image_transport::Publisher dmap_pub;
ros::Publisher pcl_pub;

Mat composeRotationCamToRobot(float x, float y, float z) {
  Mat X = Mat::eye(3, 3, CV_64FC1);
  Mat Y = Mat::eye(3, 3, CV_64FC1);
  Mat Z = Mat::eye(3, 3, CV_64FC1);
  
  X.at<double>(1,1) = cos(x);
  X.at<double>(1,2) = -sin(x);
  X.at<double>(2,1) = sin(x);
  X.at<double>(2,2) = cos(x);

  Y.at<double>(0,0) = cos(y);
  Y.at<double>(0,2) = sin(y);
  Y.at<double>(2,0) = -sin(y);
  Y.at<double>(2,2) = cos(y);

  Z.at<double>(0,0) = cos(z);
  Z.at<double>(0,1) = -sin(z);
  Z.at<double>(1,0) = sin(z);
  Z.at<double>(1,1) = cos(z);
  
  return Z*Y*X;
}

Mat composeTranslationCamToRobot(float x, float y, float z) {
    return (Mat_<double>(3,1) << x, y, z);
}

void publishPointCloud(Mat& img_left, Mat& dmap,int stereo_pair_id) {
  
    if (debug == 1)
    {
        XR = composeRotationCamToRobot(config.PHI_X,config.PHI_Y,config.PHI_Z);
        XT = composeTranslationCamToRobot(config.TRANS_X,config.TRANS_Y,config.TRANS_Z);
        //cout << "Rotation matrix: " << XR << endl;
        //cout << "Translation matrix: " << XT << endl;
    }
    
    Mat V = Mat(4, 1, CV_64FC1);
    Mat pos = Mat(4, 1, CV_64FC1);
    vector< Point3d > points;
    sensor_msgs::PointCloud pc;
    sensor_msgs::ChannelFloat32 ch;
    ch.name = "rgb";
    pc.header.frame_id = "map";
    pc.header.stamp = ros::Time::now();

    
    //cout<<"image_shape:"<<img_left.cols<<","<<img_left.rows<<endl;

    for (int i = 0; i < img_left.cols; i++)
    {
        
        for (int j = 0; j < img_left.rows; j++)
        {

        if (i<min_x||i>max_x || j<min_y||j>max_y)
        {
            continue;
        }


        int d = dmap.at<uchar>(j,i);
        
        // if low disparity, then ignore
        //if (d < 2) 
        if (d<2)
        {
            continue;
        }
        
        // V is the vector to be multiplied to Q to get
        // the 3D homogenous coordinates of the image point
        V.at<double>(0,0) = (double)(i);
        V.at<double>(1,0) = (double)(j);
        V.at<double>(2,0) = (double)d;
        V.at<double>(3,0) = 1.;
        //cout<<"Q.dtype:"<<Q.type()<<";V.type:"<<V.type()<<endl;

        //cout<<"V "<<V<<endl;
        //cout<<"------------------------------"<<endl;
        //cout<<"Q "<<Q<<endl;

        pos = Q * V; // 3D homogeneous coordinate

        //cout<<"pos "<<pos<<endl;
        
        //cout<<"Q,V multiply finished!"<<endl;

        double X = pos.at<double>(0,0) / pos.at<double>(3,0);
        double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
        double Z = pos.at<double>(2,0) / pos.at<double>(3,0);
        double X_,Y_,Z_;
        if (Z<1.0)
        {
            continue;
        }


        bool REMAP_TO_NWU = true;
        if (REMAP_TO_NWU) //for mavros
        {
            Y_ = -X;
            X_ = Z;
            Z_ = -Y;
            X = X_;
            Y = Y_;
            Z = Z_;
        }


        //cout<<"xyz: "<<X<<Y<<Z<<endl;      

        Mat point3d_cam = Mat(3, 1, CV_64FC1);
        point3d_cam.at<double>(0,0) = X;
        point3d_cam.at<double>(1,0) = Y;
        point3d_cam.at<double>(2,0) = Z;

        // transform 3D point from camera frame to robot frame
        //cout<<"XR.dtype:"<<XR.type()<<";point3d_cam.dtype:"<<point3d_cam.type()<<";XT.dtype"<<XT.type()<<endl;
        //Mat point3d_robot = XR * point3d_cam + XT;


        Mat point3d_robot = point3d_cam;

        if (stereo_pair_id == 0)
        {
            //forward
            //TODO:add your own r,t.
            
            // cout<<"stereo id == 0; "<<X<<","<<Y<<","<<Z<<endl;
            point3d_robot = point3d_cam;
        }

        if (stereo_pair_id == 1)
        {//downward stereo cam.
            point3d_robot = R_downward*point3d_cam+T_downward; //if type error,define Mat R = Mat(3, 3, CV_64FC1),T = Mat(3,1,CV_64FC1);
        }

        points.push_back(Point3d(point3d_robot));

        geometry_msgs::Point32 pt;
        pt.x = point3d_robot.at<double>(0,0);
        pt.y = point3d_robot.at<double>(1,0);
        pt.z = point3d_robot.at<double>(2,0);
    
        
        pc.points.push_back(pt);
        int32_t red, blue, green;
        red = img_left.at<Vec3b>(j,i)[2];
        green = img_left.at<Vec3b>(j,i)[1];
        blue = img_left.at<Vec3b>(j,i)[0];
        int32_t rgb = (red << 16 | green << 8 | blue);
        ch.values.push_back(*reinterpret_cast<float*>(&rgb));
        }
        
    }
  
    if (!dmap.empty())
    {
        sensor_msgs::ImagePtr disp_msg;
        disp_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", dmap).toImageMsg();
        dmap_pub.publish(disp_msg);
    }
    
    pc.channels.push_back(ch);
    pcl_pub.publish(pc);

}


Mat generateDisparityMap(Mat& left, Mat& right) {
    
    if (left.empty() || right.empty()) 
        return left;
  
    const Size imsize = left.size();
    const int32_t dims[3] = {imsize.width, imsize.height, imsize.width};
  
    Mat leftdpf = Mat::zeros(imsize, CV_32F);
    Mat rightdpf = Mat::zeros(imsize, CV_32F);
  
    Mat cv_dmap = Mat(imsize, CV_8UC1, Scalar(0));

    int RangeOfDisparity = 16 * 5; // must be dividable to 16
    int SizeOfBlockWindow = 21; // must be odd
    Ptr<StereoBM> sbm = StereoBM::create(RangeOfDisparity, SizeOfBlockWindow);

    sbm->setPreFilterSize(13);
    sbm->setPreFilterCap(13);
    sbm->setSmallerBlockSize(19);
    sbm->setMinDisparity(0);
    sbm->setNumDisparities(RangeOfDisparity);
    sbm->setTextureThreshold(10);
    sbm->setUniquenessRatio(20);
    sbm->setSpeckleWindowSize(13);

    sbm->compute(left, right, leftdpf);

    Mat dmap = Mat(out_img_size, CV_8UC1, Scalar(0));
    leftdpf.convertTo(dmap, CV_8UC1, 1.0 / 16);
    return dmap;


    //Mat dmap = Mat(out_img_size, CV_8UC1, Scalar(0));
    //leftdpf.convertTo(dmap, CV_8U, 1.);
    //leftdpf.convertTo(dmap, CV_8UC1, 1.0 / 16);
    
    //return dmap;
    
}


void imgCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right,int stereo_pair_id) {

  cout<<"stereo_pair_id: "<<stereo_pair_id<<endl; 

  Mat tmpL = cv_bridge::toCvShare(msg_left, "mono8")->image;
  Mat tmpR = cv_bridge::toCvShare(msg_right, "mono8")->image;
  
  //Mat tmpL = cv_bridge::toCvShare(msg_left, "bgr8")->image;
  //Mat tmpR = cv_bridge::toCvShare(msg_right, "bgr8")->image;
  
  if (tmpL.empty() || tmpR.empty())
    return;
  
  Mat img_left, img_right, img_left_color;
  remap(tmpL, img_left, lmapx, lmapy, cv::INTER_LINEAR);
  remap(tmpR, img_right, rmapx, rmapy, cv::INTER_LINEAR);

  cvtColor(img_left, img_left_color, CV_GRAY2BGR);
  
  Mat dmap = generateDisparityMap(img_left, img_right);

  publishPointCloud(img_left_color, dmap, stereo_pair_id);

  imshow("LEFT", img_left);
  imshow("RIGHT", img_right);
  imshow("DISP", dmap);
  waitKey(30);
}



void findRectificationMap(FileStorage& calib_file, Size finalSize) {
  Rect validRoi[2];
  cout << "Starting rectification" << endl;
  
  stereoRectify(K1, D1, K2, D2, calib_img_size, R, Mat(T), R1, R2, P1, P2, Q, 
                CV_CALIB_ZERO_DISPARITY, 0, finalSize, &validRoi[0], &validRoi[1]);
  
  cv::initUndistortRectifyMap(K1, D1, R1, P1, finalSize, CV_32F, lmapx, lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, finalSize, CV_32F, rmapx, rmapy);
  
  cout << "Done rectification" << endl;
}


void paramsCallback(stereo_dense_reconstruction::CamToRobotCalibParamsConfig &conf, uint32_t level) {
  config = conf;
}



int main(int argc, char** argv) {
    
  ros::init(argc, argv, "gi_depth_estimation");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  const char* left_img_topic;
  const char* right_img_topic;
  const char* calib_file_name;
  int calib_width, calib_height, out_width, out_height, method;
  
  static struct poptOption options[] = 
  {
    { "left_topic",'l',POPT_ARG_STRING,&left_img_topic,0,"Left image topic name","STR" },
    { "right_topic",'r',POPT_ARG_STRING,&right_img_topic,0,"Right image topic name","STR" },
    { "calib_file",'c',POPT_ARG_STRING,&calib_file_name,0,"Stereo calibration file name","STR" },
    { "calib_width",'w',POPT_ARG_INT,&calib_width,0,"Calibration image width","NUM" },
    { "calib_height",'h',POPT_ARG_INT,&calib_height,0,"Calibration image height","NUM" },
    { "out_width",'u',POPT_ARG_INT,&out_width,0,"Rectified image width","NUM" },
    { "out_height",'v',POPT_ARG_INT,&out_height,0,"Rectified image height","NUM" },
    { "debug",'d',POPT_ARG_INT,&debug,0,"Set d=1 for cam to robot frame calibration","NUM" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}
  
  
  disparity_method = method;
  
  calib_img_size = Size(calib_width, calib_height);
  out_img_size = Size(out_width, out_height);
  
  calib_file = FileStorage(calib_file_name, FileStorage::READ);
  calib_file["K1"] >> K1;
  calib_file["K2"] >> K2;
  calib_file["D1"] >> D1;
  calib_file["D2"] >> D2;
  calib_file["R"] >> R;
  calib_file["T"] >> T;
  calib_file["XR"] >> XR;
  calib_file["XT"] >> XT;
  calib_file["R_downward"] >>R_downward;
  calib_file["T_downward"] >>T_downward;
  min_x = calib_file["min_x"];
  min_y = calib_file["min_y"];
  max_x = calib_file["max_x"];
  max_y = calib_file["max_y"];
  
  findRectificationMap(calib_file, out_img_size);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;  

  
  int image_id = -1;
  
  //NOTE forward stereo pair.
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, left_img_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, right_img_topic, 1);
  image_id = 0;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
  sync.registerCallback(boost::bind(&imgCallback, _1, _2,image_id));


  //NOTE downward stereo pair.
  message_filters::Subscriber<sensor_msgs::Image> sub_img_down_left(nh,"/stereo_down/right/image_raw",1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_down_right(nh,"/stereo_down/left/image_raw",1);
  image_id = 1;
  message_filters::Synchronizer<SyncPolicy> sync2(SyncPolicy(10), sub_img_down_left, sub_img_down_right);
  sync2.registerCallback(boost::bind(&imgCallback, _1, _2,image_id));

  
  dynamic_reconfigure::Server<stereo_dense_reconstruction::CamToRobotCalibParamsConfig> server;
  dynamic_reconfigure::Server<stereo_dense_reconstruction::CamToRobotCalibParamsConfig>::CallbackType f;

  f = boost::bind(&paramsCallback, _1, _2);
  server.setCallback(f);
  
  dmap_pub = it.advertise("/camera/left/disparity_map", 1);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud>("/camera/left/point_cloud",1);

  ros::spin();
  return 0;
}
