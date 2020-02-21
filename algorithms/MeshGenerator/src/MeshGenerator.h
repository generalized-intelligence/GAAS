#include <unordered_map>
#include <memory>
#include "MeshObject.h"
#include "cv_utils.h"
#include "pcl_utils.h"
#include "cam_config.h"

#ifndef MESH_GENERATOR_H
#define MESH_GENERATOR_H

namespace MeshGen
{
using namespace std;
using namespace cv;
using namespace pcl;


unordered_map<Point2f,float> createMapTableForValidDisps(const vector<Point2f>& kps,const vector<float>& disps)
{
    unordered_map<Point2f,float> ret_val;
    assert(kps.size() == disps.size());
    for(int i = 0;i<kps.size();i++)
    {
        ret_val.insert(std::make_pair(kps.at(i),disps.at(i)));
    }
    return ret_val;
}
class MeshGenerator
{

private:
    void checkTriangles(const Mat& gradients_im,const vector<cvTriangleT>& input_triangles,vector<uint8_t>& output_success_v,bool do_check = false)//默认不检查.
    {
        for(int i = 0;i<input_triangles.size();i++)
        {
            auto t = input_triangles.at(i);
            if(do_check&&checkHighIntensityInsideOfTriangle(gradients_im,t,5.0,6))
            {
                output_success_v.push_back(0);
            }
            else
            {
               //保留这组三角形.
                //output_triangles.push_back(t);
                output_success_v.push_back(1);
            }
        }
    }
public:
//    shared_ptr<MeshObject> generateMeshWithImagePair(std::pair<shared_ptr<cv::Mat> > im_pair);
//    void Mesher::updateMesh3D(
//        const std::unordered_map<LandmarkId, gtsam::Point3>& points_with_id_VIO, // 和vio的特征点对应...和这个没关系
//        std::shared_ptr<StereoFrame> stereo_frame_ptr,//双目图像...
//        const gtsam::Pose3& left_camera_pose, Mesh2D* mesh_2d,//位姿, 2d mesh.
//            // A 2D Mesh of pixels.
//            //typedef Mesh<Vertex2D> Mesh2D;
//        std::vector<cv::Vec6f>* mesh_2d_for_viz,
//        std::vector<cv::Vec6f>* mesh_2d_filtered_for_viz)
    shared_ptr<vector<cvTriangleT> > generateMeshWithImagePair_vKP_disps(std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > im_pair,const vector<Point2f>& kps,const vector<float> disps)
    {
        LOG(INFO)<<"in generateMeshWithImagePair_vKP_disps input_kps.size():"<<kps.size()<<endl;
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


        if(true)//DEBUG ONLY
        {
            auto img2 = l_img.clone();
            for(auto & pt:valid_kps)
            {
                cv::Point2f left_top,right_bottom;
                left_top.x = pt.x - 2;
                right_bottom.x = pt.x + 2;
                left_top.y = pt.y - 2;
                right_bottom.y = pt.y +2;
                cv::rectangle(img2,left_top,right_bottom,cv::Scalar(0,0,255),-1);
            }
            cv::imshow("valid kps",img2);
            cv::waitKey(0);
        }

//        vector<KeyPoint> valid_keypoints;
//        cv::KeyPoint::convert(valid_kps,valid_keypoints);

        auto map_p2d_to_disps = createMapTableForValidDisps(valid_kps,valid_disps);

        //step<2>.三角剖分
        LOG(INFO)<<"in generateMeshWithImagePair_vKP_disps valid disps.size():"<<valid_kps.size()<<endl;
        vector<cvTriangleT> triangles;//
        auto pTriangles_filtered = make_shared<vector<cvTriangleT>>();
        auto &triangles_filtered = *pTriangles_filtered;
        doCVDelaunaySubdiv2d(l_img,valid_kps,triangles);
        LOG(INFO)<<"after delaunay subdiv2d:valid triangles.size():"<<triangles.size()<<endl;
        //step<3>.检查所有三角形.
        //计算canny.
        cv::Mat canny;
        doCVCalcCannyEdge(l_img,canny);
        vector<uint8_t> v_success_check_triangle;
        checkTriangles(canny,triangles,v_success_check_triangle);

        vector<cvTriangleT> triangles_valid;//这里triangle对应的是顶点.应该检查顶点获取disp.

        extract_sub_vec(triangles,triangles_filtered,v_success_check_triangle);
        LOG(INFO)<<"after checkEdge:valid triangles.size():"<<triangles_filtered.size()<<endl;


        shared_ptr<PointCloud<PointXYZRGB> > pMesh = createMeshFromTriangles(triangles_filtered,l_img,map_p2d_to_disps,valid_disps,fx_,fy_,cx_,cy_,b_);//二维三角形->三维三角面片
        return pTriangles_filtered;
    }

    shared_ptr<PointCloud<PointXYZRGB> > createMeshFromTriangles(vector<cvTriangleT>& triangles,const Mat& original_img,unordered_map<Point2f,float>& p2d_to_disps,vector<float>& disps,
                                                     const float& fx,const float& fy,const float& cx,const float& cy,const float& b,
                                                     bool doTextureMapping=true)
    {
        //创建完整mesh.
        //尝试贴图
        LOG(INFO)<<"in createMeshFromTriangles:valid triangles.size():"<<triangles.size()<<endl;
        if(doTextureMapping)
        {
            //...
        }
        //三维化.
        cv::Mat depth_map(original_img.rows,original_img.cols,CV_32F);
        //cv::Mat triangle_render_helper(original_img.rows,original_img.cols,CV_8U,Scalar(0,0,0));//用于帮助标记是否在三角形内.
        cv::Mat triangle_render_helper(original_img.rows,original_img.cols,CV_8U,Scalar(0,0,0));//用于帮助标记是否在三角形内.
        generateDepthMapForAllTriangles(triangles,p2d_to_disps,depth_map,triangle_render_helper,
                                        fx,fy,cx,cy,b);//绘制深度图(用重心坐标插值法).

        //png2pcd
        auto pCloud = make_shared<PointCloud<PointXYZRGB> >();
        PointCloud <PointXYZRGB> pc_debug;
        img2d_to_mesh_3d(original_img,depth_map,triangle_render_helper,*pCloud,pc_debug,fx,fy,cx,cy,b);
        // Save the point cloud into a PCD file
        //DEBUG ONLY:
//        pcl::saveCloud<PointXYZRGB> ("cloud",*pCloud);
        pcl::io::savePLYFile("mesh.ply",*pCloud);
        pcl::io::savePLYFile("mesh_debug.ply",pc_debug);
        return pCloud;
    }
};



}
#endif
