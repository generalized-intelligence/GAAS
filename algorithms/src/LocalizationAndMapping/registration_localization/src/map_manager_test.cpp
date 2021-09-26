#include "registration_map_manager.h"




int main(int argc,char** argv)
{
    ros::init(argc,argv,"map_manager_test");
    std::shared_ptr<ros::NodeHandle> pNH (new ros::NodeHandle);
    RegistrationMapManager rmm;
    rmm.init(*pNH);
    rmm.getCurrentMapName();

    Eigen::Vector3d initial_position;
    initial_position <<10,10,5;
    MapCloudT::Ptr pMapCrop = rmm.getCurrentMapCloud(initial_position);
    LOG(INFO)<<"Map size:"<<pMapCrop->size()<<endl;

    initial_position <<-60,20,5;
    {
        ScopeTimer t("switch map");
        MapCloudT::Ptr pMapCrop2 = rmm.getCurrentMapCloud(initial_position);
        LOG(INFO)<<"Map size after move:"<<pMapCrop2->size()<<endl;
    }
    return 0;
}
