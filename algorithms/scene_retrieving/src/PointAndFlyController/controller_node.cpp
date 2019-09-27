#include "controller.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


std::shared_ptr<SceneRetriever> pSceneRetrieve;
std::shared_ptr<Controller> pController;

void StereoImageCallback(const sensor_msgs::ImageConstPtr& msgLeft ,const sensor_msgs::ImageConstPtr &msgRight)
{

  cv::Mat imgL, imgR;
  float fitness_score = pSceneRetrieve->retrieveSceneFromStereoImage(imgL, imgR, Q_mat, RT_mat, match_success);

  if()
  {
    pController.Add();
  }
}


int main(int argc, char **argv) {

    if (argc!=5)
    {
      cout<<"Usage: demo [scene_file_path] [voc_file_path]"<<endl;
    }

    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    cv::Mat RT_mat = (cv::Mat_<float >(4,4) << 1, 0, 0, 0,
                                               0, 1, 0, 1,
                                               0, 0, 1, 0,
                                               0, 0, 0, 1);

    bool match_success;

    cout<<"voc path: "<<argv[2]<<endl;
    cout<<"scene path: "<<argv[1]<<endl;

    std::shared_ptr<SceneRetriever> pSceneRetriever(new SceneRetriever(argv[2], argv[1]));
    pSceneRetrieve = pSceneRetriever;


    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/gi/simulation/left/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/gi/simulation/right/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));
    sync.registerCallback(boost::bind(StereoImageCallback, _1, _2));


    Controller controller(nh);
    pController = &controller;

    ros::MultiThreadedSpinner spinner(4);

    ros::Rate rate(5);
    while (ros::ok())
    {
        spinner.spin(); // the missing call
        rate.sleep();
    }
    return 0;
}