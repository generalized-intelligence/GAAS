#include "controller.h"




int main(int argc, char **argv) {

    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    Controller controller(nh);

    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce(); // the missing call
        rate.sleep();
    }
    return 0;
}