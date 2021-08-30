#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <glog/logging.h>

using std::string;
using std::vector;
using std::cout;
using std::endl;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> LidarCloudT;


class LivoxHorizonSim
{
private:
    int frame_id = 0;
    //fov: 81.7deg , 25.1deg:atan(81.7deg/2) = 0.6191 atan(25.1deg/2) = 0.2155 ; 619 is a prime number...

    //fx = fy = 619/2; cx = width/2 cy = height/2
    const int image_width = 619;
    const int image_height = 619;
    const int inner_loop_size = 1;//10 Hz for inner loop(24,000 points/0.1s). 32 lines. We just caught depth image for 10Hz to reduce performance cost, and reduce the number of points.
                                  // while preserving the same scanning pattern.
    const int outer_loop_size = 360;//36s per round. Pseudo non-repetitive mode.

    const double pi = 3.1415926535;
    const int inner_loop_r = 80;
    const int padding = inner_loop_r*2;
    cv::Mat pattern_img;
public:
    cv::Mat generate_basic_ellipse()
    {


        const int ellipse_height = 16;
        const int ellipse_width = 260;
        const int ellipse_y_step = 6;
        const int ellipse_y_init = 15;
        cv::Mat img_out(image_height+padding,image_width+padding,CV_8U,cv::Scalar(0));

        for(int group_id = 0;group_id<10;group_id++) //36deg/iter ----pi/5 per iter.
        //int group_id = 0;
        {
            int ellipse_center_x,ellipse_center_y;
            for(int ellipse_id = 0;ellipse_id<5;ellipse_id++)
            {

                ellipse_center_x = (int) (cos(group_id* pi/5 + 5*pi/180)*inner_loop_r + image_width/2 + padding/2);// initial deg:5
                ellipse_center_y = (int) (sin(group_id* pi/5 + 5*pi/180)*inner_loop_r + image_height/2+14+ padding/2);// initial deg:5

                ellipse_center_y -= ellipse_y_init;
                ellipse_center_y+=ellipse_id*ellipse_y_step;

                cv::RotatedRect rect_rot(cv::Point2f(ellipse_center_x-ellipse_width/3,ellipse_center_y-ellipse_height/2),cv::Size2f(ellipse_width,ellipse_height),0.0f);
                cv::RotatedRect rect_rot2(cv::Point2f(ellipse_center_x+ellipse_width/3,ellipse_center_y-ellipse_height/2),cv::Size2f(ellipse_width,ellipse_height),0.0f);
                //cv::ellipse(img_out,cv::Rect(ellipse_center_x-240/2,ellipse_center_y-8/2,240,8),255);
                cv::ellipse(img_out,rect_rot,255);
                cv::ellipse(img_out,rect_rot2,255);
                //cout<<"Ellipse!"<<ellipse_center_x<<";"<<ellipse_center_y<<endl;
            }
        }
        //cv::imshow("image out in 0.1s",img_out);
        //cv::waitKey(300);

        return img_out;
    }
    cv::Mat get_ellipse_unrepeated()
    {
        //const int outer_loop_r = 74;
        const int outer_loop_r = inner_loop_r/8;
        int x = (int) (cos(frame_id*7.3*pi/180)*outer_loop_r+image_width/2+padding/2);
        int y = (int) (sin(frame_id*7.3*pi/180)*outer_loop_r+image_height/2+padding/2);
        cv::Rect rect(x-image_width/2,y-image_height/2,image_width,image_height);
        cv::Mat retval = pattern_img(rect);
        this->frame_id++;
        cv::imshow("image out in 0.1s",retval);
        cv::waitKey(10);
        return retval;
    }
    //cv::Mat generate_rings_mask_by_frame_id_ellipse()
    //{;
    //}
public:
    LivoxHorizonSim()
    {
        this->pattern_img = generate_basic_ellipse();
    }
    //LidarCloudT::Ptr processDepthImage(cv::Mat& input_depth_image)
};

ros::NodeHandle* pNH=nullptr;
LivoxHorizonSim* pSim=nullptr;
ros::Publisher* pPub=nullptr;

void utilizeMask(cv::Mat& depth,cv::Mat& mask)
{
    for(int v = 0;v<depth.rows;v++)
    {
        for(int u = 0;u<depth.cols;u++)
        {
            if(!mask.at<unsigned char>(v,u))
            {
                depth.at<float>(v,u) = 0;
            }
        }
    }
    return;
}
LidarCloudT::Ptr toPointCloud(cv::Mat& depth_masked)
{
    const float cx = 619/2;
    const float cy = 619/2;
    const float fx = 619/2;
    const float fy = 619/2;
    LidarCloudT::Ptr newcloud(new LidarCloudT());
    for(int v = 0;v<depth_masked.rows;v++)
    {
        for(int u = 0;u<depth_masked.cols;u++)
        {
            const float& d = depth_masked.at<float>(v,u);
            if(d<=0.05)
            {
                continue;
            }
            float x,y,z;
            z = d;
            x = z*(u-cx)/fx;
            y = z*(v-cy)/fy;
//            float len = sqrt(x*x+y*y+z*z);
//            x*=z/len;
//            y*=z/len;
//            z*=z/len;
            PointT pt;//For downward lidar placing:
            pt.x = -y;
            pt.y = -x;
            pt.z = -z;
            newcloud->push_back(pt);
        }
    }
    newcloud->width = newcloud->points.size();
    newcloud->height = 1;
    LOG(INFO)<<"cloud.size()"<<newcloud->size()<<endl;
    return newcloud;
}

void callback(const sensor_msgs::ImageConstPtr& img_msg)
{
    cv_bridge::CvImagePtr image_ptr;
    try
    {
        //image_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        image_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception & e)
    {
        LOG(ERROR)<<"cv_bridge exception: "<< e.what()<<endl;
        return;
    }
    cv::Mat depth = image_ptr->image;
    cv::Mat mask = pSim->get_ellipse_unrepeated();
    utilizeMask(depth,mask);
    LidarCloudT::Ptr pCloud = toPointCloud(depth);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pCloud,cloud_msg);
    cloud_msg.header.stamp = img_msg->header.stamp;
    cloud_msg.header.frame_id = "lidar";//"livox";
    pPub->publish(cloud_msg);

}
int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging(argv[0]);
    LOG(INFO)<<"Start livox_horizon_sim_node."<<endl;
    ros::init(argc,argv,"livox_horizon_sim_node");
    ros::NodeHandle nh;
    pNH=&nh;
    LivoxHorizonSim liv;
    pSim=&liv;
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/fake_livox/forward/horizon", 2);
    pPub=&cloud_pub;
    ros::Subscriber sub = nh.subscribe("/kinect/fake_livox/depth_image",1,callback);

    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
