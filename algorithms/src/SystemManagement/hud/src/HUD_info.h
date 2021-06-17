#ifndef HUDINFO_HEADER_FILE
#define HUDINFO_HEADER_FILE

#include <mutex>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "gaas_msgs/GAASSystemManagementFlightControllerState.h"

#include "tf2/utils.h"

//显示空速,地速,lon,lat,alt,地平面,yaw.
//TODO:是否显示障碍物信息?


using std::cout;
using std::endl;



class HUD_info
{//提供显示图像
public:
    HUD_info()
    {
        cv::Mat empty_image(480,720,CV_8UC3,cv::Scalar(0)); //default image.
        this->original_img_mutex.lock();
        this->original_image = empty_image;
        this->original_img_mutex.unlock();
    }
    void originalImageCallback(const sensor_msgs::ImageConstPtr& img);
    void fillInfoToImage(const gaas_msgs::GAASSystemManagementFlightControllerStateConstPtr& fc_info);
    void flightControllerStateCallback(const gaas_msgs::GAASSystemManagementFlightControllerStateConstPtr& fc_info);
    void getImage(cv::Mat& mat_out)
    {
        processed_img_mutex.lock();
        mat_out = this->processed_img;
        processed_img_mutex.unlock();
    }
    void showImage();
private:
    cv::Mat original_image;
    cv::Mat processed_img;
    std::mutex original_img_mutex;
    std::mutex processed_img_mutex;
};
void HUD_info::originalImageCallback(const sensor_msgs::ImageConstPtr& img)
{
    LOG(INFO)<<"In HUD::originalImageCallback()"<<endl;
    cv_bridge::CvImagePtr input_image_ptr;
    try
    {
        input_image_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception & e)
    {
        LOG(ERROR)<<"cv_bridge exception: "<< e.what()<<endl;
        return;
    }
    cv::Mat input_img = input_image_ptr->image;
    original_img_mutex.lock();
    //set image
    this->original_image = input_img;
    original_img_mutex.unlock();
}
void HUD_info::flightControllerStateCallback(const gaas_msgs::GAASSystemManagementFlightControllerStateConstPtr& fc_info)
{
    LOG(INFO)<<"In HUD::flightControllerStateCallback()"<<endl;
    processed_img_mutex.lock();
    //print info.
    original_img_mutex.lock();
    processed_img = this->original_image.clone();
    if(processed_img.empty())
    {
        LOG(ERROR)<<"ERROR: empty original image!"<<endl;
        throw "ERROR!";
    }
    original_img_mutex.unlock();

    this->fillInfoToImage(fc_info);
    processed_img_mutex.unlock();
}
void HUD_info::fillInfoToImage(const gaas_msgs::GAASSystemManagementFlightControllerStateConstPtr& fc_info)
{

    //Temperature:
    {
        std::stringstream ss;
        ss<<"Temp:"<<fc_info->temperature_celsius<<"`C";
        cv::putText(this->processed_img,ss.str(),cv::Point((this->processed_img.cols/2)-60,20),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(0,128,255),2);
    }


    //Lon Lat:
    {
        //GPS signal quality:
        bool gps_locked = false;
        if(fc_info->gps_fix_status>=0)
        {
            gps_locked = true;
            cv::putText(this->processed_img,"GPS_LOCKED",cv::Point(10,20),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(0,128,255),2);
        }
        else
        {
            cv::putText(this->processed_img,"NO GPS LOCK",cv::Point(10,20),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,0,0),2);
        }

        auto color_gps = CV_RGB(255,0,0);
        if(gps_locked)
        {
            color_gps = CV_RGB(0,128,255);
        }
        {
            std::stringstream ss;
            ss<<"SAT NUM:"<<fc_info->gps_avail_satellite_count;
            cv::putText(this->processed_img,ss.str(),cv::Point(180,20),cv::FONT_HERSHEY_PLAIN,1.0,color_gps,1);
        }
        {
            std::stringstream ss;
            ss<<std::fixed << std::setprecision(9)<<"Lon:"<<fc_info->gps_longitude;
            cv::putText(this->processed_img,ss.str(),cv::Point(10,40),cv::FONT_HERSHEY_PLAIN,1.2,color_gps,2);
        }
        {
            std::stringstream ss;
            ss<<std::fixed << std::setprecision(9)<<"Lat:"<<fc_info->gps_latitude;
            cv::putText(this->processed_img,ss.str(),cv::Point(10,60),cv::FONT_HERSHEY_PLAIN,1.2,color_gps,2);
        }
        //GPS Alt:
        {
            std::stringstream ss;
            ss<<"GPS Alt:"<< fc_info->gps_altitude <<"m";
            cv::putText(this->processed_img, //target image
                        ss.str(), //text
                        cv::Point(10, processed_img.rows - 20), //top-down position
                        cv::FONT_HERSHEY_PLAIN,
                        1.35,
                        color_gps, //font color
                        2);
        }
    }
    //Ground Speed:
    {
        std::stringstream ss;
        ss<<"GND VEL:"<<std::fixed << std::setprecision(5)<<sqrt(pow(fc_info->vehicle_vx,2)+pow(fc_info->vehicle_vy,2)+pow(fc_info->vehicle_vz,2))<<"m/s";
        cv::putText(this->processed_img, //target image
                    ss.str(), //text
                    cv::Point(10, processed_img.rows / 2), //top-left position
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    CV_RGB(64, 64, 0), //font color
                    2);
    }
    //AHRS:
    {
        double roll, pitch, yaw;
        //tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        tf2::Quaternion q(fc_info->imu_filtered_qx,fc_info->imu_filtered_qy,fc_info->imu_filtered_qz,fc_info->imu_filtered_qw);
        //tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        tf2::Matrix3x3(q).getEulerYPR(yaw,pitch,roll);
        yaw+=3.1415926535/2;//north is up.
        const double radtodeg = 180/3.1415926535;
        double roll_deg = roll*radtodeg;
        double pitch_deg = pitch*radtodeg;
        double yaw_deg = yaw * radtodeg;
        LOG(INFO)<<"roll,pitch,yaw:"<<roll_deg<<";"<<pitch_deg<<";"<<yaw_deg<<"deg."<<endl;

        double x_center = processed_img.cols/2,y_center = processed_img.rows/2;
        double len = 140.0;



        cv::circle(this->processed_img,cv::Point((int) processed_img.cols*0.8,(int)y_center*1.72),(int)y_center*0.2,CV_RGB(255,128,128),1); //yaw

        cv::line(this->processed_img,cv::Point((int) processed_img.cols*0.8,(int)y_center*1.72),cv::Point((int) processed_img.cols*0.8+y_center*0.2*sin(yaw),(int)y_center*1.72+y_center*0.2*cos(yaw)),CV_RGB(0,255,255),3);
        cv::putText(this->processed_img,"N",cv::Point((int) processed_img.cols*0.8-10,(int)y_center*1.52-10),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,0,0),2);
        cv::putText(this->processed_img,"S",cv::Point((int) processed_img.cols*0.8-10,(int)y_center*1.92+10),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,255,255),2);
        cv::putText(this->processed_img,"W",cv::Point((int) processed_img.cols*0.8 - 30 - y_center*0.2,(int)y_center*1.72),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,255,0),2);
        cv::putText(this->processed_img,"E",cv::Point((int) processed_img.cols*0.8 + 10 + y_center*0.2,(int)y_center*1.72),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,0,255),2);


        double x_c1 = x_center - len - 50 ,y_c1 = y_center;// 0 deg 水平参考线.
        double x_c2 = x_center - len + 20 ,y_c2 = y_center;
        double x_c3 = x_center + len + 50 ,y_c3 = y_center;
        double x_c4 = x_center + len - 20 ,y_c4 = y_center;

        cv::line(this->processed_img,cv::Point((int)x_c1,(int)y_c1),cv::Point((int)x_c2,(int)y_c2),CV_RGB(255,0,0),2);
        cv::line(this->processed_img,cv::Point((int)x_c3,(int)y_c3),cv::Point((int)x_c4,(int)y_c4),CV_RGB(255,0,0),2);

        cv::putText(this->processed_img," 00",cv::Point((int)x_c4+50,(int)y_c4),cv::FONT_HERSHEY_PLAIN,1.4,CV_RGB(255,0,0),2);

        x_c1 = x_center - len - 30 ,y_c1 = y_center -  y_center*0.9*sin(30/radtodeg);//30 deg水平参考线.
        x_c2 = x_center - len + 20 ,y_c2 = y_center -  y_center*0.9*sin(30/radtodeg);
        x_c3 = x_center + len + 30 ,y_c3 = y_center - y_center*0.9*sin(30/radtodeg);
        x_c4 = x_center + len - 20 ,y_c4 = y_center - y_center*0.9*sin(30/radtodeg);

        cv::line(this->processed_img,cv::Point((int)x_c1,(int)y_c1),cv::Point((int)x_c2,(int)y_c2),CV_RGB(255,0,0),2);
        cv::line(this->processed_img,cv::Point((int)x_c3,(int)y_c3),cv::Point((int)x_c4,(int)y_c4),CV_RGB(255,0,0),2);

        cv::putText(this->processed_img,"+30",cv::Point((int)x_c3+20,(int)y_c3),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,255,0),2);


        x_c1 = x_center - len - 30 ,y_c1 = y_center +  y_center*0.9*sin(30/radtodeg);//30 deg水平参考线.
        x_c2 = x_center - len + 20 ,y_c2 = y_center +  y_center*0.9*sin(30/radtodeg);
        x_c3 = x_center + len + 30 ,y_c3 = y_center + y_center*0.9*sin(30/radtodeg);
        x_c4 = x_center + len - 20 ,y_c4 = y_center + y_center*0.9*sin(30/radtodeg);

        cv::line(this->processed_img,cv::Point((int)x_c1,(int)y_c1),cv::Point((int)x_c2,(int)y_c2),CV_RGB(255,0,0),2);
        cv::line(this->processed_img,cv::Point((int)x_c3,(int)y_c3),cv::Point((int)x_c4,(int)y_c4),CV_RGB(255,0,0),2);

        cv::putText(this->processed_img,"-30",cv::Point((int)x_c3+20,(int)y_c3),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,255,0),2);

        x_c1 = x_center - len - 10 ,y_c1 = y_center -  y_center*0.9*sin(15/radtodeg);//15 deg水平参考线.
        x_c2 = x_center - len + 20 ,y_c2 = y_center -  y_center*0.9*sin(15/radtodeg);
        x_c3 = x_center + len + 10 ,y_c3 = y_center - y_center*0.9*sin(15/radtodeg);
        x_c4 = x_center + len - 20 ,y_c4 = y_center - y_center*0.9*sin(15/radtodeg);

        cv::line(this->processed_img,cv::Point((int)x_c1,(int)y_c1),cv::Point((int)x_c2,(int)y_c2),CV_RGB(255,0,0),2);
        cv::line(this->processed_img,cv::Point((int)x_c3,(int)y_c3),cv::Point((int)x_c4,(int)y_c4),CV_RGB(255,0,0),2);

        cv::putText(this->processed_img,"+15",cv::Point((int)x_c3+20,(int)y_c3),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,255,0),2);


        x_c1 = x_center - len - 10 ,y_c1 = y_center +  y_center*0.9*sin(15/radtodeg);//-15 deg水平参考线.
        x_c2 = x_center - len + 20 ,y_c2 = y_center +  y_center*0.9*sin(15/radtodeg);
        x_c3 = x_center + len + 10 ,y_c3 = y_center + y_center*0.9*sin(15/radtodeg);
        x_c4 = x_center + len - 20 ,y_c4 = y_center + y_center*0.9*sin(15/radtodeg);

        cv::putText(this->processed_img,"-15",cv::Point((int)x_c3+20,(int)y_c3),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,255,0),2);



        cv::line(this->processed_img,cv::Point((int)x_c1,(int)y_c1),cv::Point((int)x_c2,(int)y_c2),CV_RGB(255,0,0),2);
        cv::line(this->processed_img,cv::Point((int)x_c3,(int)y_c3),cv::Point((int)x_c4,(int)y_c4),CV_RGB(255,0,0),2);


        x_c1 = x_center - len - 10 ,y_c1 = y_center -  y_center*0.9*sin(5/radtodeg);//5 deg水平参考线.
        x_c2 = x_center - len + 20 ,y_c2 = y_center -  y_center*0.9*sin(5/radtodeg);
        x_c3 = x_center + len + 10 ,y_c3 = y_center - y_center*0.9*sin(5/radtodeg);
        x_c4 = x_center + len - 20 ,y_c4 = y_center - y_center*0.9*sin(5/radtodeg);

        cv::line(this->processed_img,cv::Point((int)x_c1,(int)y_c1),cv::Point((int)x_c2,(int)y_c2),CV_RGB(255,0,0),1);
        cv::line(this->processed_img,cv::Point((int)x_c3,(int)y_c3),cv::Point((int)x_c4,(int)y_c4),CV_RGB(255,0,0),1);

        cv::putText(this->processed_img,"+05",cv::Point((int)x_c3+20,(int)y_c3),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,255,0),2);


        x_c1 = x_center - len - 10 ,y_c1 = y_center +  y_center*0.9*sin(5/radtodeg);//-5 deg水平参考线.
        x_c2 = x_center - len + 20 ,y_c2 = y_center +  y_center*0.9*sin(5/radtodeg);
        x_c3 = x_center + len + 10 ,y_c3 = y_center + y_center*0.9*sin(5/radtodeg);
        x_c4 = x_center + len - 20 ,y_c4 = y_center + y_center*0.9*sin(5/radtodeg);

        cv::line(this->processed_img,cv::Point((int)x_c1,(int)y_c1),cv::Point((int)x_c2,(int)y_c2),CV_RGB(255,0,0),1);
        cv::line(this->processed_img,cv::Point((int)x_c3,(int)y_c3),cv::Point((int)x_c4,(int)y_c4),CV_RGB(255,0,0),1);


        cv::putText(this->processed_img,"-05",cv::Point((int)x_c3+20,(int)y_c3),cv::FONT_HERSHEY_PLAIN,1.2,CV_RGB(255,255,0),2);





        //double x2 = x_center + len*cos(roll),y2 = y_center - len*sin(roll)


        double x1 = x_center - len*cos(roll),y1 = y_center + len*sin(roll) - y_center*0.9*sin(pitch);
        double x2 = x_center + len*cos(roll),y2 = y_center - len*sin(roll) - y_center*0.9*sin(pitch);
        cv::line(this->processed_img,cv::Point((int)x1,(int)y1),cv::Point((int)x2,(int)y2),CV_RGB(255,255,128),3); //Horizon (roll,pitch)
    }
    showImage();//DEBUG ONLY.
}
void HUD_info::showImage()
{
    cv::imshow("HUD",this->processed_img);
    cv::waitKey(30);
}

#endif
