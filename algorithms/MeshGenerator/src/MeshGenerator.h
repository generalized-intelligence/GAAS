
#include <unordered_map>
#include <memory>
#include "MeshObject.h"
#include "cv_utils.h"
#include "pcl_utils.h"
#ifndef MESH_GENERATOR_H
#define MESH_GENERATOR_H

namespace MeshGen
{
using namespace std;
using namespace cv;


unordered_map<Point2f,float> createMapTableForValidDisps(const vector<Point2f>& kps,const vector<float>& disps)
{
    unordered_map<Point2f,float> ret_val;
    assert(kps.size() == disps.size());
    for(int i = 0;i<kps.size();i++)
    {
        ret_val.insert(kps.at(i),disps.at(i));
    }
    return ret_val;
}
class MeshGenerator
{

private:

public:
    shared_ptr<MeshObject> generateMeshWithImagePair(std::pair<shared_ptr<cv::Mat> > im_pair);
//    void Mesher::updateMesh3D(
//        const std::unordered_map<LandmarkId, gtsam::Point3>& points_with_id_VIO, // 和vio的特征点对应...和这个没关系
//        std::shared_ptr<StereoFrame> stereo_frame_ptr,//双目图像...
//        const gtsam::Pose3& left_camera_pose, Mesh2D* mesh_2d,//位姿, 2d mesh.
//            // A 2D Mesh of pixels.
//            //typedef Mesh<Vertex2D> Mesh2D;
//        std::vector<cv::Vec6f>* mesh_2d_for_viz,
//        std::vector<cv::Vec6f>* mesh_2d_filtered_for_viz)
    shared_ptr<MeshObject> generateMeshWithImagePair_vKP_disps(std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > im_pair,const vector<Point2f>& kps,const vector<float> disps)
    {
        //对图像进行三角剖分.
        Mat& l_img = *(im_pair.first);
        assert(kps.size() == disps.size());//否则输入有误.
        //step<1>.先排除disp不正常的点.

        int pt_count = disps.size();
        vector<uint8_t> v_success;
        v_success.resize(pt_count);
        for(int i = 0;i<pt_count;i++)
        {
            if(disps.at(i)<0)
            {
                v_success.at(i) = 0;
            }
            else
            {
                v_success.at(i) = 1;
            }
        }
        vector<Point2f> valid_kps;
        vector<float> valid_disps;
        extract_sub_vec(kps,valid_kps,v_success);
        extract_sub_vec(disps,valid_disps,v_success);
        auto map_p2d_to_disps = createMapTableForValidDisps(valid_kps,valid_disps);

        //step<2>.三角剖分
        auto triangles = doCVDelaunaySubdiv2d(valid_kps);
        //step<3>.检查所有三角形.
        //计算canny.
        cv::Mat canny;
        doCVCalcCannyEdge(l_img,canny);
        vector<cvTriangleT> triangles,triangles_filtered;
        vector<uint_8> v_success_check_triangle;

        checkTriangles(canny,triangles,v_success_check_triangle);

        vector<cvTriangleT> triangles_valid;//这里triangle对应的是顶点.应该检查顶点获取disp.
        extract_sub_vec(triangles,triangles_filtered,v_success_check_triangle);

        auto mesh = createMeshFromTriangles(triangles_filtered,l_img,map_p2d_to_disps,valid_disps);//二维三角形->三维三角面片
    }
    void checkTriangles(const Mat& gradients_im,const vector<cvTriangleT>& input_triangles,vector<uint_8> output_success_v)
    {
        for(int i = 0;i<input_triangles.size();i++)
        {
            auto t = input_triangles.at(i);
            if(checkHighIntensityInsideOfTriangle(canny,t,5.0,6))
            {
               //保留这组三角形.
                //output_triangles.push_back(t);
                output_success_v.push_back(1);
            }
            else
            {
                output_success_v.push_back(0);
            }
        }
    }
    shared_ptr<MeshObject3d> createMeshFromTriangles(vector<cvTriangleT>& triangles,const Mat& original_img,unordered_map<Point2f,float>& p2d_to_disps,vector<float>& disps,
                                                     const float& fx,const float& fy,const float& cx,const float& cy,const float& b,
                                                     bool doTextureMapping=true)
    {
        //创建完整mesh.
        MeshObject2d mesh2d;//先贴2d.

        //尝试贴图
        if(doTextureMapping)
        {
            //...
        }
        //三维化.
        shared_ptr<MeshObject3d> pMesh3d_ret(new MeshObject3d);

        cv::Mat depth_map(original_img.rows,original_img.cols,CV_32F);
        cv::Mat triangle_render_helper(original_img.rows,original_img.cols,CV_8U,Scalar(0,0,0));//用于帮助标记是否在三角形内.
        generateDepthMapForAllTriangles(triangles,p2d_to_disps,depth_map,triangle_render_helper,
                                        fx,fy,cx,cy,b);//绘制深度图(用重心坐标插值法).
        //png2pcd
        PointCloud<PointXYZRGB> cloud;
        img2d_to_mesh_3d(original_img,depth_map,triangle_render_helper,cloud,fx,fy,cx,cy,b);
    }
};



}
#endif
