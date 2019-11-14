#include "multi_scene_retriever.h"


MultiSceneRetriever::MultiSceneRetriever()
{
    this->gps_KDTree = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ( new  pcl::PointCloud<pcl::PointXYZ>() );
    scene_index = 0;
}

void MultiSceneRetriever::generate_visualization_graph()
{
    //iterate all scene,build graph.draw graph to a image file.
}

int MultiSceneRetriever::retrieveSceneWithScaleFromMonoImage(cv::Mat image_in_rect,
        cv::Mat cameraMatrix, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success,
        double img_lon,double img_lat,bool img_lon_lat_valid)
{
    //step<1> select scene nearby.
    if(img_lon_lat_valid == false)
    {
        cout<<"Fatal Error:Image Longitude,Latitude must be valid!"<<endl;
        return -1;
    }
    vector<int> scene_index_list;
    this->findNRelativeSceneByGPS(img_lon,img_lat,scene_index_list);
    //step<2> do match.

    
    
    typedef std::pair<int,int> id_and_matched_points_t;
    vector<id_and_matched_points_t> match_result_list;
    
    for (const int& index:scene_index_list)
    {
        bool match_success;
        //do matching.get multiple results.
        int matched_points_count = this->idToNodeMap[index]->pSceneRetriever->retrieveSceneWithScaleFromMonoImage(image_in_rect, cameraMatrix, RT_mat_of_mono_cam_output, match_success);
        if(match_success)
        {
            match_result_list.push_back(std::make_pair(index,matched_points_count));
        }
    }
    //step<3> select and reserve only the best match.
    int best_match_id = -1;
    int best_match_points_count = -1;
    for(auto match:match_result_list)
    {
        int curr_index = std::get<0>(match);
        int curr_count = std::get<1>(match);
	if(curr_count>best_match_points_count)
	{
	    best_match_points_count = curr_count;
	    best_match_id = curr_index;
	}
    }
    if(best_match_id>=0 && best_match_points_count>5)
    {
        //redo mapping;for the last one is not always the best one.
        this->idToNodeMap[best_match_id]->pSceneRetriever->retrieveSceneWithScaleFromMonoImage(image_in_rect, cameraMatrix, RT_mat_of_mono_cam_output, match_success);
    }
    if(match_success)
    {
        return best_match_id;
    }
    else
    {
        return -1;
    }

}

int MultiSceneRetriever::retrieveSceneFromStereoImage(cv::Mat image_left_rect,
                                                      cv::Mat image_right_rect,
                                                      cv::Mat& Q_mat,
                                                      cv::Mat& RT_mat_of_stereo_cam_output,
                                                      bool& match_success,
                                                      double img_lon,double img_lat,bool img_lon_lat_valid)
{
  
    //step<1> select scene nearby.
    if(img_lon_lat_valid)
    {
        cout<<"Fatal Error:no gps info in retrieveSceneFromStereoImage().Failed."<<endl;
        return -1;
    }

    vector<int> scene_index_list;
    this->findNRelativeSceneByGPS(img_lon,img_lat,scene_index_list);

    //step<2> do match.
    vector<std::pair<int, int> > match_result_list;

    for (const int& index:scene_index_list)
    {
        bool match_success;
        cv::Mat RT_mat;

        //do matching.get multiple results.
        //int matched_points_count = this->idToNodeMap[index].pScene->retrieveSceneFromStereoImage(cv::imread(left_image_path[i]), cv::imread(right_image_path[i]), Q_mat, RT_mat, match_success);
        int matched_points_count = (this->idToNodeMap[index]->pSceneRetriever)->retrieveSceneFromStereoImage(image_left_rect, image_right_rect, Q_mat, RT_mat, match_success);

        if(match_success)
        {
            match_result_list.push_back(std::make_pair(index, matched_points_count));
        }
    }

    //step<3> select and reserve only the best match.
    int best_match_id = -1;
    int best_match_points_count = -1;
    for(auto match:match_result_list)
    {
        int curr_index = std::get<0>(match);
        int curr_count = std::get<1>(match);
        if(curr_count>best_match_points_count)
        {
            best_match_points_count = curr_count;
            best_match_id = curr_index;
        }
    }

    if(best_match_id>=0 && best_match_points_count>5)
    {
        //redo mapping;for the last one is not always the best one.
        cv::Mat cameraMatrix, RT_mat_of_mono_cam_output;
        this->idToNodeMap[best_match_id]->pSceneRetriever->retrieveSceneWithScaleFromMonoImage(image_left_rect, cameraMatrix, RT_mat_of_mono_cam_output, match_success);
    }

    if(match_success)
    {
        return best_match_id;
    }

    else
    {
        return -1;
    }
}

void MultiSceneRetriever::insertSceneIntoKDTree(shared_ptr<MultiSceneNode> nodeptr)
{
    this->idToNodeMap[this->scene_index] = nodeptr;
    pcl::PointXYZ p;
    p.x = nodeptr->longitude;
    p.y = nodeptr->latitude;
    p.z = 0;
    this->gps_KDTree->points.push_back(p);
    this->scene_index++; // so the index shall be synchronized.
}
void MultiSceneRetriever::findNRelativeSceneByGPS(double gps_longitude,double gps_latitude,
                                vector<int> &output_scene_index,
                                int count,double range_km)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointXYZ searchPoint;
    searchPoint.x = gps_longitude;
    searchPoint.y = gps_latitude;
    searchPoint.z = 0;
    kdtree.setInputCloud (this->gps_KDTree);
    auto& cloud = this->gps_KDTree;
    const int K = count;
    /*std::vector<int> pointIdxNKNSearch(K);
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
    }*/
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    const double radius = range_km/6371.0; // this radius is lon,lat radius in its space.
    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size ()&&i<count; ++i)
        {
            std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
            output_scene_index.push_back(pointIdxRadiusSearch[i]);
        }
    }
}
