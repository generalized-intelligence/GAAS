#ifndef ROS_WRAPPER_MESHGEN
#define ROS_WRAPPER_MESHGEN

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include "Pipeline.h"
#include <thread>
#include <functional>







class MeshGeneratorROSWrapper
{
private:
    ros::NodeHandle* pNH;
    ros::Publisher marker_pub;
    shared_ptr<visualization_msgs::Marker> generateMarkerFrom3DMesh(const Triangle3D_Mesh& mesh,const std_msgs::Header& img_header,int mesh_id)
    {
        shared_ptr<visualization_msgs::Marker> pMarker = make_shared<visualization_msgs::Marker>();
        pMarker->type = pMarker->TRIANGLE_LIST;
        vector<geometry_msgs::Point> v_pts;
        for(int i = 0;i<mesh.vVertices.size();i++)
        {
            auto pt = mesh.vVertices.at(i);
            geometry_msgs::Point gPt;
            gPt.x = pt[0];
            gPt.y = pt[1];
            gPt.z = pt[2];
            v_pts.push_back(gPt);
        }
        std_msgs::ColorRGBA color;
        color.a = 0.5;
        color.r = 1.0;
        color.g = 0.2;
        color.b = 0.2;
        for(int tri_index = 0;tri_index<mesh.vTriangleIDs.size();tri_index++)
        {
            auto triangle_id_tuple = mesh.vTriangleIDs.at(tri_index);
            pMarker->points.push_back(v_pts.at(triangle_id_tuple[0]));
            pMarker->points.push_back(v_pts.at(triangle_id_tuple[1]));
            pMarker->points.push_back(v_pts.at(triangle_id_tuple[2]));


            pMarker->colors.push_back(color);
            color.g+=0.01;
        }

        pMarker->header = img_header;//stamp继承了.

        pMarker->header.frame_id = "map";
        pMarker->id = mesh_id;
        pMarker->type = visualization_msgs::Marker::TRIANGLE_LIST;
        pMarker->action = visualization_msgs::Marker::ADD;

        pMarker->scale.x = 1;
        pMarker->scale.y = 1;
        pMarker->scale.z = 1;
        pMarker->pose.position.x = 0;
        pMarker->pose.position.y = 0;
        pMarker->pose.position.z = 0;

        pMarker->pose.orientation.x = 0;
        pMarker->pose.orientation.y = 0;
        pMarker->pose.orientation.z = 0;
        pMarker->pose.orientation.w = 1;

        pMarker->color.r = 1;
        pMarker->color.a = 1;

        return pMarker;
    }



public:
    MeshGeneratorROSWrapper(ros::NodeHandle* pNH)
    {
        this->pNH = pNH;
        marker_pub = pNH->advertise<visualization_msgs::Marker>("/gaas/mesh_generator/generated_mesh",10);
    }
    void onCallback(shared_ptr<cvMatT> im1,shared_ptr<cvMatT> im2,shared_ptr<cvMatT> im3,shared_ptr<cvMatT> im4,const sensor_msgs::ImageConstPtr& img_1)
    {
        shared_ptr<Triangle3D_Mesh> pMesh1,pMesh2;
        std::thread t1(run_all_pipeline,im1,im2,std::ref(pMesh1));
        std::thread t2(run_all_pipeline,im3,im4,std::ref(pMesh2));
        t1.join();
        t2.join();
        marker_pub.publish(*generateMarkerFrom3DMesh(*pMesh1,img_1->header,0));
        marker_pub.publish(*generateMarkerFrom3DMesh(*pMesh2,img_1->header,1));
        LOG(INFO)<<"Mesh message published!"<<endl;
    }
};



#endif
