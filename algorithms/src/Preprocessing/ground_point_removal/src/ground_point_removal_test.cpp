#include "ground_point_removal.h"
#include "Timer.h"
#include "typedefs.h"
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
    LidarCloudT::Ptr input_cloud(new LidarCloudT);
    LidarCloudT::Ptr output_cloud;
    //LOAD CLOUD...
    pcl::PCDReader reader;
    reader.read("../LaserScan.pcd",*input_cloud);
    for(int i = 0;i<100;i++)
    {
        ScopeTimer t("Ground point removal timer:");
        removeGround(input_cloud,output_cloud);
    }

    return 0;
}
