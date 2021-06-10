//导入飞控信息:注意,使用GAAS的飞控信息兼容不同飞控.不同飞控的实现通过他们对应的包:px4->px4_state_reporter
#include "gaas_msgs/GAASSystemManagementFlightControllerState.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "HUD_info.h"
#include <ros/ros.h>
#include <glog/logging.h>

class HUDNode
{//处理显示内容
public:
    void init_node()
    {
        pHUD = std::shared_ptr<HUD_info>(new HUD_info);
        this->pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        this->gaas_state_sub = pNH->subscribe("/gaas/system_management/flight_controller_state",1,&HUDNode::fc_callback,this);

        std::string img_topic;
        if(!ros::param::get("fpv_image_topic",img_topic))
        {
            LOG(ERROR)<<"Error: no fpv image topic!"<<endl;
            throw "ERROR!";
        }
        this->image_sub = pNH->subscribe(img_topic,1,&HUDNode::image_callback,this);
    }
    void start_loop()
    {
        ros::spin();
    }
private:
    void fc_callback(const gaas_msgs::GAASSystemManagementFlightControllerStateConstPtr& fc_info)
    {
        pHUD->flightControllerStateCallback(fc_info);
        return;
    }
    void image_callback(const sensor_msgs::ImageConstPtr& img)
    {
        pHUD->originalImageCallback(img);
        return;
    }
    ros::Subscriber gaas_state_sub,image_sub;
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;
    std::shared_ptr<HUD_info> pHUD;
//subscribers,ros handlers,glog ....
};


//test inner HUD_info class.
int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("hud_node");
    ros::init(argc,argv,"hud_node");

    HUDNode node;
    node.init_node();
    node.start_loop();
    return 0;
}

//HUD Node:
//int main(int argc,char** argv)
//{
//    HUD_info info;
//    info.showImage();
//    return 0;
//}
