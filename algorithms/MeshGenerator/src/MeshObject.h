#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>



#ifndef MESH_OBJECT_H
#define MESH_OBJECT_H
namespace MeshGen
{
using namespace std;

//struct TriangleObject
//{};


//p1_x = v[0], p1_y = v[1], p2_x = v[2], p2_y = v[3], p3_x = v[4], p3_y = v[5].
typedef cv::Vec6f cvTriangleT;
struct Triangle3D_T
{
    Triangle3D_T()
    {
        this->triangle3d = cv::Mat(4,0,CV_32F);//初始化一个三角形.可以用cv::Mat投影. 最后一位填一个1.
    }
    cv::Mat triangle3d;

};
typedef vector<cv::Vec6f> meshT;
struct MeshObject2d //定义存储方式
{
    //点表 边表
    cv::Mat polygons_vertex_mat;
    //    // Connectivity of the mesh.
    //    // Set of polygons.
    //    // Raw integer list of the form: (n,id1_a,id2_a,...,idn_a,
    //    // n,id1_b,id2_b,...,idn_b, ..., n, ... idn_x)
    //    // where n is the number of points per polygon, and id is a zero-offset
    //    // index into the associated row in vertices_mesh_.

    //边表.
    cv::Mat polygons_mesh_;
};
struct MeshObject3d
{

};


}
