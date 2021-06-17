#include "scene_retrieve.h"
#include "controller.h"
#include "building.h"
#include "world.h"

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>


#define EIGEN_RUNTIME_NO_MALLOC

shared_ptr<SceneRetriever> pSceneRetrieve;
shared_ptr<Controller> pController;

void StereoImageCallback(const sensor_msgs::ImageConstPtr& msgLeft ,const sensor_msgs::ImageConstPtr &msgRight)
{

        cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;

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

        cv::Mat imgL, imgR;

        imgL = cv_ptrLeft->image;
        imgR = cv_ptrRight->image;

        if(imgL.empty() || imgR.empty())
        {
          return;
        }

        cv::Mat mavros_pose, RT_mat;
        bool match_success;
        int* loop_index;

        // mavros pose: return value, FCU returned pose, could be either from GPS or SLAM
        // RT_mat: return value, recovered pose from loaded scene
        float fitness_score = pSceneRetrieve->retrieveSceneFromStereoImage(imgL, imgR, mavros_pose, RT_mat, match_success, loop_index);
        LOG(INFO)<<"fitness_score: "<<fitness_score<<endl;

        if(fitness_score >= 0)
        {
            LOG(INFO)<<"fitness_score >= 0 && !mavros_pose.empty() && !RT_mat.empty(), AddRetrievedPose!"<<endl;

            pController->AddRetrievedPose(RT_mat, mavros_pose);
        }

}


int main(int argc, char **argv) {

    if (argc!=3)
    {
      LOG(INFO)<<"Usage: demo [scene_file_path] [voc_file_path]"<<endl;
    }

    google::SetLogDestination(google::GLOG_INFO, "./log_controller_" );
    //google::SetCommandLineOption("GLOG_minloglevel", "2");
    FLAGS_alsologtostderr = 1;
    FLAGS_minloglevel = 1;
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    cv::Mat RT_mat = (cv::Mat_<float >(4,4) << 1, 0, 0, 0,
                                               0, 1, 0, 1,
                                               0, 0, 1, 0,
                                               0, 0, 0, 1);

    LOG(INFO)<<"scene path: "<<argv[1]<<endl;
    LOG(INFO)<<"voc path: "<<argv[2]<<endl;

    string scene_path = argv[1];
    string voc_path = argv[2];

    pSceneRetrieve = make_shared<SceneRetriever>(voc_path, scene_path, "./config/scene_retrieve.yaml");
    pController = make_shared<Controller>(nh);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/gi/simulation/left/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/gi/simulation/right/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));
    sync.registerCallback(boost::bind(StereoImageCallback, _1, _2));

    ros::MultiThreadedSpinner spinner(4);

    while (ros::ok())
    {
        spinner.spin(); // the missing call
    }

    return 0;
}
