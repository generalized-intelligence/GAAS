#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointField.h>
#include <geometry_msgs/Point32.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <stereo_dense_reconstruction/CamToRobotCalibParamsConfig.h>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "popt_pp.h"
#include "pcl_helper.h"


using namespace cv;
using namespace std;


Mat XR, XT, Q, P1, P2,R_downward,T_downward;
Mat R1, R2, K1, K2, D1, D2, R;
Mat lmapx, lmapy, rmapx, rmapy;
Vec3d T;

stereo_dense_reconstruction::CamToRobotCalibParamsConfig config;
FileStorage calib_file;
int debug = 1;
Size out_img_size;
Size calib_img_size;

int disparity_method;

image_transport::Publisher dmap_pub;
ros::Publisher pcl_pub, navigator_pub;

size_t log_index = -1;

int UsePCLfiltering = 0;

int RangeOfDisparity, SizeOfBlockWindow, PreFilterSize, PreFilterCap, SmallerBlockSize, MinDisparity, NumDisparities;
int TextureThreshold, UniquenessRatio, SpeckleWindowSize, Lambda, SigmaColor;
int bUseWLSfilter, displayImage, disparityMax, disparityMin;
int method, p1, p2, disp12MaxDiff, speckleRange, fullDP, SADWindowSize;
float distanceMax, distanceMin;
int conductStereoRectify, max_x, min_x, max_y, min_y;

pcl_helper* mpPCL_helper;

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


void publishPointCloud(Mat& img_left, Mat& dmap, int stereo_pair_id) {
  
    if (debug)
    {
        XR = composeRotationCamToRobot(config.PHI_X,config.PHI_Y,config.PHI_Z);
        XT = composeTranslationCamToRobot(config.TRANS_X,config.TRANS_Y,config.TRANS_Z);
    }

    if(dmap.empty())
        return;

    
    Mat V = Mat(4, 1, CV_64FC1);
    Mat pos = Mat(4, 1, CV_64FC1);

    vector< Point3d > points;
    sensor_msgs::ChannelFloat32 ch;
    ch.name = "rgb";

    sensor_msgs::PointCloud pc;
    pc.header.frame_id = "map";
    pc.header.stamp = ros::Time::now();

    for (int i = 0; i < img_left.cols; i++)
    {
        for (int j = 0; j < img_left.rows; j++)
        {

            if (i<min_x || i>max_x || j<min_y || j>max_y)
                continue;

            int d = dmap.at<uchar>(j,i);
//            cout<<"d: "<<d<<", "<<disparityMin<<", "<<disparityMax<<endl;

            if (d < disparityMin || d > disparityMax)
                continue;

            // V is the vector to be multiplied to Q to get
            // the 3D homogenous coordinates of the image point
            V.at<double>(0,0) = (double)(i);
            V.at<double>(1,0) = (double)(j);
            V.at<double>(2,0) = (double)d;
            V.at<double>(3,0) = 1.;

            pos = Q * V; // 3D homogeneous coordinate

            double X = pos.at<double>(0,0) / pos.at<double>(3,0);
            double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
            double Z = pos.at<double>(2,0) / pos.at<double>(3,0);
            double X_,Y_,Z_;

//            cout<<"z: "<<Z<<", "<<distanceMin<<", "<<distanceMax<<endl;
            if (Z < distanceMin || Z > distanceMax)
                continue;

            bool REMAP_TO_FLU = true;
            if (REMAP_TO_FLU) //for mavros
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

            cv:: Mat point3d_robot;

            // forward stereo camera
            if (stereo_pair_id == 0)
            {
                point3d_robot = point3d_cam;
            }

            // downward stereo camera
            if (stereo_pair_id == 1)
            {
                point3d_robot = R_downward*point3d_cam+T_downward; //if type error,define Mat R = Mat(3, 3, CV_64FC1),T = Mat(3,1,CV_64FC1);
            }

            points.push_back(Point3d(point3d_robot));

            geometry_msgs::Point32 pt;
            pt.x = float(point3d_robot.at<double>(0,0) );
            pt.y = float(point3d_robot.at<double>(1,0) );
            pt.z = float(point3d_robot.at<double>(2,0) );

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

    log_index ++;

    sensor_msgs::PointCloud2 pc2;

    bool result = sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);

    if (UsePCLfiltering)
    {
        //conduct pointcloud filering provided by PCL
        pcl::PointCloud<pcl::PointXYZ> output_cloud;

        mpPCL_helper->ROSPointCloud2toPointCloudXYZ(pc2, output_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>);

        *cloudPTR = output_cloud;

        cout<<"cloudPTR: "<<cloudPTR<<endl;

        auto result_cloud = mpPCL_helper->PCLStatisticalOutlierFilter(cloudPTR);

        cout<<"*result_cloud: "<<*result_cloud<<endl;

        mpPCL_helper->PointCloudXYZtoROSPointCloud2(*result_cloud, pc2);
    }

    if(result)
    {
        // publish in format of sensor_msgs::pointcloud2
        pcl_pub.publish(pc2);

        // publish in format of sensor_msgs::pointcloud
        navigator_pub.publish(pc);
    }
    else
        cout<<"Converting points failed!"<<endl;
}


// image should be rectified and aligned
cv::Mat generateDisparityMapBM(Mat& left, Mat& right) {
    
    if (left.empty() || right.empty()) 
        return left;
  
    const Size imsize = left.size();
    const int32_t dims[3] = {imsize.width, imsize.height, imsize.width};
  
    Mat leftdpf = Mat::zeros(imsize, CV_32F);
    Mat rightdpf = Mat::zeros(imsize, CV_32F);
  
    Mat cv_dmap = Mat(imsize, CV_8UC1, Scalar(0));

    //step 1, create left matcher instance
    cv::Ptr<StereoBM> left_matcher = StereoBM::create(RangeOfDisparity, SizeOfBlockWindow);
    left_matcher->setPreFilterSize(PreFilterSize);
    left_matcher->setPreFilterCap(PreFilterCap);
    left_matcher->setSmallerBlockSize(SmallerBlockSize);
    left_matcher->setMinDisparity(MinDisparity);
    left_matcher->setNumDisparities(NumDisparities);
    left_matcher->setTextureThreshold(TextureThreshold);
    left_matcher->setUniquenessRatio(UniquenessRatio);
    left_matcher->setSpeckleWindowSize(SpeckleWindowSize);


    if(bUseWLSfilter)
    {
        //step 2, create filter instance
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
        wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
        wls_filter->setLambda(Lambda);
        wls_filter->setSigmaColor(SigmaColor);

        //step 3, create right matcher instance
        cv::Ptr<cv::StereoMatcher> right_matcher;
        right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

        //step 4, compute
        cv::Mat left_disp, right_disp;
        left_matcher-> compute(left, right, left_disp);
        right_matcher->compute(right, left, right_disp);

        //step 5, conduct filtering
        cv::Mat filtered_disp;
        wls_filter->filter(left_disp,left,filtered_disp,right_disp);

        cv::Mat conf_map = cv::Mat(left.rows,left.cols,CV_8U);
        conf_map = Scalar(255);

        conf_map = wls_filter->getConfidenceMap();
        cv::Rect ROI = wls_filter->getROI();

        //step 6, return value
        Mat dmap = Mat(out_img_size, CV_8UC1, Scalar(0));
        filtered_disp.convertTo(dmap, CV_8UC1, 1.0 / 16);

        //result_disparity = mpPCL_helper->DisparityWLSFilter(left, dmap)
        return dmap;
    }
    else
    {
        Mat dmap = Mat(out_img_size, CV_8UC1, Scalar(0));
        left_matcher->compute(left, right, leftdpf);
        leftdpf.convertTo(dmap, CV_8UC1, 1.0 / 16);

        return dmap;
    }

}


// image should be rectified and aligned
cv::Mat generateDisparityMapSGBM(Mat& left, Mat& right) {

    if (left.empty() || right.empty())
        return left;

    const Size imsize = left.size();
    const int32_t dims[3] = {imsize.width, imsize.height, imsize.width};

    Mat leftdpf = Mat::zeros(imsize, CV_32F);
    Mat rightdpf = Mat::zeros(imsize, CV_32F);

    Mat cv_dmap = Mat(imsize, CV_8UC1, Scalar(0));

    //step 1, create left matcher instance
    cv::Ptr<StereoSGBM> left_matcher = cv::StereoSGBM::create(MinDisparity,
            NumDisparities,
            SADWindowSize,
            p1, p2,
            disp12MaxDiff,
            PreFilterCap,
            UniquenessRatio,
            SpeckleWindowSize,
            speckleRange,
            fullDP);

    if(bUseWLSfilter)
    {
        //step 2, create filter instance
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
        wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
        wls_filter->setLambda(Lambda);
        wls_filter->setSigmaColor(SigmaColor);

        //step 3, create right matcher instance
        cv::Ptr<cv::StereoMatcher> right_matcher;
        right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

        //step 4, compute
        cv::Mat left_disp, right_disp;
        left_matcher-> compute(left, right, left_disp);
        right_matcher->compute(right, left, right_disp);

        //step 5, conduct filtering
        cv::Mat filtered_disp;
        wls_filter->filter(left_disp,left,filtered_disp,right_disp);

        cv::Mat conf_map = cv::Mat(left.rows,left.cols,CV_8U);
        conf_map = Scalar(255);

        conf_map = wls_filter->getConfidenceMap();
        cv::Rect ROI = wls_filter->getROI();

        //step 6, return value
        Mat dmap = Mat(out_img_size, CV_8UC1, Scalar(0));
        filtered_disp.convertTo(dmap, CV_8UC1, 1.0 / 16);

        //result_disparity = mpPCL_helper->DisparityWLSFilter(left, dmap)
        return dmap;
    }
    else
    {
        Mat dmap = Mat(out_img_size, CV_8UC1, Scalar(0));
        left_matcher->compute(left, right, leftdpf);
        leftdpf.convertTo(dmap, CV_8UC1, 1.0 / 16);

        return dmap;
    }
}


void imgCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right,int stereo_pair_id) {

  Mat tmpL = cv_bridge::toCvShare(msg_left, "mono8")->image;
  Mat tmpR = cv_bridge::toCvShare(msg_right, "mono8")->image;
  
  //Mat tmpL = cv_bridge::toCvShare(msg_left, "bgr8")->image;
  //Mat tmpR = cv_bridge::toCvShare(msg_right, "bgr8")->image;
  
  if (tmpL.empty() || tmpR.empty())
      return;
  
  Mat img_left, img_right, img_left_color;


  //NOTE conduct stereoRectify?
  if(conductStereoRectify)
  {
      remap(tmpL, img_left, lmapx, lmapy, cv::INTER_LINEAR);
      remap(tmpR, img_right, rmapx, rmapy, cv::INTER_LINEAR);
  }
  else
  {
      img_left = tmpL;
      img_right = tmpR;
  }


  cvtColor(img_left, img_left_color, CV_GRAY2BGR);

  Mat dmap;

  switch(method)
  {
      case 0:
      {
          dmap = generateDisparityMapBM(img_left, img_right);
          break;
      }

      case 1:
      {
          dmap = generateDisparityMapSGBM(img_left, img_right);
          break;
      }

  }

  cout<<"Publishing pointcloud, index: "<<log_index<<endl;

  publishPointCloud(img_left_color, dmap, stereo_pair_id);


  if(displayImage)
  {
      imshow("LEFT", img_left);
      imshow("RIGHT", img_right);
      imshow("DISP", dmap);
  }

  waitKey(30);
}



void findRectificationMap(Size finalSize) {
  Rect validRoi[2];
  cout << "Starting rectification" << endl;
  
  cv::stereoRectify(K1, D1, K2, D2, calib_img_size, R, Mat(T), R1, R2, P1, P2, Q,
          CV_CALIB_ZERO_DISPARITY, 0, finalSize, &validRoi[0], &validRoi[1]);
  
  cv::initUndistortRectifyMap(K1, D1, R1, P1, finalSize, CV_32F, lmapx, lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, finalSize, CV_32F, rmapx, rmapy);
  
  cout << "Done rectification" << endl;
}


void paramsCallback(stereo_dense_reconstruction::CamToRobotCalibParamsConfig &conf, uint32_t level) {
  config = conf;
}



int main(int argc, char** argv) {

    if(argc != 2)
    {
        cout<<"YOU NEED TO SPECIFY CONFIG PATH!"<<endl;
        return 0;
    }

    string config_path = argv[1];
    string configFile(config_path);
    cv::FileStorage fsSettings(configFile, cv::FileStorage::READ);

    int calib_width = fsSettings["calib_width"];
    int calib_height = fsSettings["calib_height"];
    int out_width = fsSettings["out_width"];
    int out_height = fsSettings["out_height"];
    int debug = fsSettings["debug"];
    method  = fsSettings["method"];
    conductStereoRectify = fsSettings["conductStereoRectify"];

    string left_img_topic = string(fsSettings["left_img_topic"]);
    string right_img_topic = string(fsSettings["right_img_topic"]);
    string calib_file_name = string(fsSettings["calib_file_name"]);
    UsePCLfiltering = fsSettings["UsePCLfiltering"];
    disparityMax = fsSettings["disparityMax"];
    disparityMin = fsSettings["disparityMin"];
    distanceMax = fsSettings["distanceMax"];
    distanceMin = fsSettings["distanceMin"];

    // for StereoBM matcher
    RangeOfDisparity  = fsSettings["RangeOfDisparity"];
    SizeOfBlockWindow  = fsSettings["SizeOfBlockWindow"];
    PreFilterSize  = fsSettings["PreFilterSize"];
    PreFilterCap  = fsSettings["PreFilterCap"];
    SmallerBlockSize  = fsSettings["SmallerBlockSize"];
    MinDisparity  = fsSettings["MinDisparity"];
    NumDisparities  = fsSettings["NumDisparities"];
    TextureThreshold  = fsSettings["TextureThreshold"];
    UniquenessRatio  = fsSettings["UniquenessRatio"];
    SpeckleWindowSize  = fsSettings["SpeckleWindowSize"];

    // for StereoSGBM matcher
    p1 = fsSettings["p1"];
    p2 = fsSettings["p2"];
    disp12MaxDiff = fsSettings["disp12MaxDiff"];
    speckleRange = fsSettings["speckleRange"];
    fullDP = fsSettings["fullDP"];
    SADWindowSize = fsSettings["SADWindowSize"];

    // for WLS filter
    bUseWLSfilter = fsSettings["UseWLSfilter"];
    Lambda  = fsSettings["Lambda"];
    SigmaColor  = fsSettings["SigmaColor"];
    displayImage = fsSettings["displayImage"];

    // for image max/min x and y
    max_x = fsSettings["max_x"];
    min_x = fsSettings["min_x"];
    max_y = fsSettings["max_y"];
    min_y = fsSettings["min_y"];;


    cout<<"-------------Brief-----------------"<<endl;
    cout<<"left_img_topic: "<<left_img_topic<<endl;
    cout<<"right_img_topic: "<<right_img_topic<<endl;
    cout<<"calib_file_name: "<<calib_file_name<<endl;
    cout<<"calib_width: "<<calib_width<<endl;
    cout<<"calib_height: "<<calib_height<<endl;
    cout<<"out_width: "<<out_width<<endl;
    cout<<"out_height: "<<out_height<<endl;
    cout<<"debug: "<<debug<<endl;
    cout<<"disparityMax: "<<disparityMax<<endl;
    cout<<"disparityMin: "<<disparityMin<<endl;
    cout<<"distanceMax: "<<distanceMax<<endl;
    cout<<"distanceMin: "<<distanceMin<<endl;
    cout<<"conductStereoRectify: "<<conductStereoRectify<<endl;

    cout<<"RangeOfDisparity: "<<RangeOfDisparity<<endl;
    cout<<"SizeOfBlockWindow: "<<SizeOfBlockWindow<<endl;
    cout<<"PreFilterSize: "<<PreFilterSize<<endl;
    cout<<"PreFilterCap: "<<PreFilterCap<<endl;
    cout<<"SmallerBlockSize: "<<SmallerBlockSize<<endl;
    cout<<"MinDisparity: "<<MinDisparity<<endl;
    cout<<"NumDisparities: "<<NumDisparities<<endl;
    cout<<"TextureThreshold: "<<TextureThreshold<<endl;
    cout<<"UniquenessRatio: "<<UniquenessRatio<<endl;
    cout<<"SpeckleWindowSize: "<<SpeckleWindowSize<<endl;

    cout<<"Lambda: "<<Lambda<<endl;
    cout<<"SigmaColor: "<<SigmaColor<<endl;

    cout<<"displayImage: "<<displayImage<<endl;
    cout<<"-----------------------------------"<<endl;


    ros::init(argc, argv, "gi_depth_estimation");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

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

    findRectificationMap(out_img_size);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

    int image_id = -1;

    mpPCL_helper = new pcl_helper(config_path);

    //NOTE forward stereo pair.
    message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, left_img_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, right_img_topic, 1);
    image_id = 0;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
    sync.registerCallback(boost::bind(&imgCallback, _1, _2,image_id));


    //NOTE downward stereo pair.
    //  message_filters::Subscriber<sensor_msgs::Image> sub_img_down_left(nh,"/stereo_down/right/image_raw",100);
    //  message_filters::Subscriber<sensor_msgs::Image> sub_img_down_right(nh,"/stereo_down/left/image_raw",100);
    //  image_id = 1;
    //  message_filters::Synchronizer<SyncPolicy> sync2(SyncPolicy(10), sub_img_down_left, sub_img_down_right);
    //  sync2.registerCallback(boost::bind(&imgCallback, _1, _2,image_id));


    dynamic_reconfigure::Server<stereo_dense_reconstruction::CamToRobotCalibParamsConfig> server;
    dynamic_reconfigure::Server<stereo_dense_reconstruction::CamToRobotCalibParamsConfig>::CallbackType f;

    f = boost::bind(&paramsCallback, _1, _2);
    server.setCallback(f);

    dmap_pub = it.advertise("/camera/left/disparity_map", 100);

    // for disparity map
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/left/point_cloud2",100);

    // for Navigator in python
    navigator_pub = nh.advertise<sensor_msgs::PointCloud>("/camera/left/point_cloud",100);

    ros::spin();
    return 0;
}
