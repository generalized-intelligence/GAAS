#include "scene_retrieve.h"
#include "controller.h"


#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

SceneRetriever* pSceneRetrieve;
Controller* pController;

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

        cv::Mat Q_mat, RT_mat;
        bool match_success;
        int* loop_index;
        float fitness_score = pSceneRetrieve->retrieveSceneFromStereoImage(imgL, imgR, Q_mat, RT_mat, match_success, loop_index);

        cout<<"fitness_score: "<<fitness_score<<endl;

        if(fitness_score >= 0 && fitness_score <= 1.0 && !RT_mat.empty())
        {

          cout<<"RT_mat type: "<<RT_mat.type()<<endl;
          cout<<"RT_mat: "<<RT_mat<<endl;

          Eigen::Matrix3f rotation_matrix;
          cv::cv2eigen(RT_mat, rotation_matrix);
          Vector3f EulerAngle = rotation_matrix.eulerAngles(0, 1, 2);

          Eigen::Vector4f position_yaw(RT_mat.at<double>(0, 3),
                                       RT_mat.at<double>(1, 3),
                                       RT_mat.at<double>(2, 3),
                                       EulerAngle[2]);

          cout<<"rotation mat: "<<rotation_matrix<<endl;
          cout<<"result mat: "<<RT_mat<<endl;

          pController->AddRetrievedPose(position_yaw);
        }
        else
        {
          return;
        }

}


int main(int argc, char **argv) {

    if (argc!=3)
    {
      cout<<"Usage: demo [scene_file_path] [voc_file_path]"<<endl;
    }

    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    cv::Mat RT_mat = (cv::Mat_<float >(4,4) << 1, 0, 0, 0,
                                               0, 1, 0, 1,
                                               0, 0, 1, 0,
                                               0, 0, 0, 1);

    cout<<"scene path: "<<argv[1]<<endl;
    cout<<"voc path: "<<argv[2]<<endl;

    string scene_path = argv[1];
    string voc_path = argv[2];

    auto* pSceneRetriever = new SceneRetriever(voc_path, scene_path);
    pSceneRetrieve = pSceneRetriever;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/gi/simulation/left/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/gi/simulation/right/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));
    sync.registerCallback(boost::bind(StereoImageCallback, _1, _2));

    auto* controller = new Controller(nh);
    pController = controller;

//    ros::MultiThreadedSpinner spinner(4);

    ros::Rate rate(10);
    while (ros::ok())
    {
//        spinner.spin(); // the missing call
        ros::spin();
        rate.sleep();
    }
    return 0;
}