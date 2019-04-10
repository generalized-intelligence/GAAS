#include "multi_scene_retriever.h"
MultiSceneRetriever::MultiSceneRetriever()
{
    this->gps_KDTree = new pcl::PointCloud<pcl::PointXYZ>();
    scene_index = 0;
}
void MultiSceneRetriever::loadSceneInfoFromLocalFolder(const cv::FileStorage& config_file)
{
    for (auto path:config_file["scene_path_list"])
    {
        //get lon,lat
        double lon,lat = ....;
        shared_ptr<Scene> pScene(new Scene(path))
        this->insertSceneIntoKDTree(lon,lat,pScene);
    }

}
virtual int retrieveSceneWithScaleFromMonoImage(const cv::Mat image_in_rect,
        const cv::Mat& cameraMatrix, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success,
        double img_lon,double img_lat,bool img_lon_lat_valid)
{
    //step<1> select scene nearby.
    if(this->img_lon_lat_valid == false)
    {
        cout<<"Error:Image Longitude,Latitude must be valid!"<<endl;
        return -1;
    }
    scene_list = this->findNRelativeSceneByGPS(lon,lat);
    //step<2> do match.
    for (auto scene:scene_list)
    {
        //do matching.get multiple results.
    }
    //step<3> select and reserve only the best match.
    ...
}
void MultiSceneRetriever::insertSceneIntoKDTree(double longi,double lati,shared_ptr<Scene> pScene)
{
    shared_ptr<MultiSceneNode> pNew(new MultiSceneNode());
    pNew->longitude = longi;
    pNew->lati = lati;
    pNew->pScene = pScene;
    this->insertSceneIntoKDTree(pNew);
}
void MultiSceneRetriever::insertSceneIntoKDTree(shared_ptr<MultiSceneNode> nodeptr)
{
    this->idToNodeMap[this->scene_index] = nodeptr;
    pcl::PointXYZ p;
    p.x = nodeptr->longitude;
    p.y = nodeptr->latitude;
    p.z = 0;
    this->gps_KDTree.points.push_back(p);
    this->scene_index++; // so the index shall be synchronized.
}
void MultiSceneRetriever::findNRelativeSceneByGPS(double gps_longitude,double gps_latitude,
                                 int count = 10,double range_km = 5)
{
    this->gps_KDTree.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (this->gps_KDTree);
    const int K = count;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }
    }
    const double radius = range_km/6371.0; // this radius is lon,lat radius in its space.
    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
            std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
        }
    }
}