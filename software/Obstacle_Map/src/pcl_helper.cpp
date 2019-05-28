//
// Created by gishr on 19-4-18.
//

#include "pcl_helper.h"



pcl_helper::pcl_helper(string config_path)
{
    this->config_path = config_path;

    cv::FileStorage fsSettings(this->config_path, cv::FileStorage::READ);

    radius = fsSettings["Radius"];
    MinNeighbor = fsSettings["MinNeighbor"];

    mean_k = fsSettings["mean_k"];
    stdThres = fsSettings["stdThres"];

    cout<<"using radius: "<<radius<<endl;
    cout<<"using points number: "<<MinNeighbor<<endl;

    cout<<"using mean_k: "<<mean_k<<endl;
    cout<<"using stdThres: "<<stdThres<<endl;
};

void pcl_helper::ROSPointCloud2toPointCloudXYZ(sensor_msgs::PointCloud2& input_cloud, pcl::PointCloud<pcl::PointXYZ>& output_cloud)
{
    pcl::PCLPointCloud2 pcl_cloud;

    //step 1. convert ROS pc to PCL pc
    pcl_conversions::toPCL(input_cloud, pcl_cloud);

    //step 2. convert pcl_cloud2 to pcl_T
    pcl::fromPCLPointCloud2(pcl_cloud, output_cloud);
}


void pcl_helper::PointCloudXYZtoROSPointCloud2(pcl::PointCloud<pcl::PointXYZ>& input_cloud, sensor_msgs::PointCloud2& output_cloud)
{
    pcl::PCLPointCloud2 pcl_cloud;

    //step 1. convert pcl_T to pcl_cloud2
    pcl::toPCLPointCloud2(input_cloud, pcl_cloud);

    //step 2. convert pcl_cloud2 pc to ROS pc
    pcl_conversions::fromPCL(pcl_cloud, output_cloud);
}


sensor_msgs::PointField pcl_helper::sensorPointField(string name_of_field, int offset, int datatype, int elementNum)
{
    sensor_msgs::PointField temp_PF;

    temp_PF.name = name_of_field;
    temp_PF.offset = offset;
    temp_PF.datatype = datatype;
    temp_PF.count = elementNum;
}


//given original point cloud size, generate a sensor_msgs::pointcloud2 object and assuming
//it has three element of type float32

//NOTE segfaults here
bool pcl_helper::createPointCloud2(vector<geometry_msgs::Point32>& input_points, sensor_msgs::PointCloud2& pc)
{

    cout<<"createPointCloud2 1"<<endl;

    if(input_points.empty())
        return false;

    //step 1, create an sensor_msgs::pointcloud2 instance

    //pc->header.frame_id = "map";
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "map";

    pc.height = 1;
    pc.width = input_points.size();

    sensor_msgs::PointField pf;

    pc.fields.push_back(this->sensorPointField("x", 0, pf.FLOAT32, 1));
    pc.fields.push_back(this->sensorPointField("y", 4, pf.FLOAT32, 1));
    pc.fields.push_back(this->sensorPointField("z", 8, pf.FLOAT32, 1));

    cout<<"createPointCloud2 2"<<endl;

    pc.point_step = 12; //length of a point in bytes
    pc.row_step =  pc.point_step *  pc.width;

    pc.data.resize(input_points.size() *  pc.point_step);

    pc.is_bigendian = false;
    pc.is_dense = false;

    cout<<"createPointCloud2 3"<<endl;

    //step 2, push points to pointcloud2.data
    for (size_t cp = 0; cp < input_points.size (); ++cp)
    {
        cout<<"input_points[cp]: "<<input_points[cp].x<<", "<<input_points[cp].y<<", "<<input_points[cp].z<<endl;

        memcpy (&pc.data[cp * pc.point_step + pc.fields[0].offset], &input_points[cp].x, sizeof (float));
        memcpy (&pc.data[cp * pc.point_step + pc.fields[1].offset], &input_points[cp].y, sizeof (float));
        memcpy (&pc.data[cp * pc.point_step + pc.fields[2].offset], &input_points[cp].z, sizeof (float));
    }

    cout<<"createPointCloud2 4"<<endl;

    //step 3, return
    return true;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_helper::PCLStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_p)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


    mStatisticalOutlierRemoval.setInputCloud (raw_cloud_p);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *raw_cloud_p << std::endl;

    mStatisticalOutlierRemoval.setMeanK (this->mean_k);
    mStatisticalOutlierRemoval.setStddevMulThresh (this->stdThres);
    mStatisticalOutlierRemoval.filter (*cloud_filtered);

    mStatisticalOutlierRemoval.setNegative (true);
    mStatisticalOutlierRemoval.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    return cloud_filtered;

}


pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_helper::PCLRadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_p)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    mRadiusOutlierRemoval.setInputCloud(raw_cloud_p);
    mRadiusOutlierRemoval.setRadiusSearch(this->radius);
    mRadiusOutlierRemoval.setMinNeighborsInRadius (this->MinNeighbor);

    mRadiusOutlierRemoval.filter (*cloud_filtered);

    return cloud_filtered;
}


//pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_helper::PCLConditionalRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_p, float Min, float Max)
//{
//
//}


