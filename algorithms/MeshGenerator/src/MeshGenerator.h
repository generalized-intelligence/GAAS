#include <unordered_map>
#include <memory>
#include "MeshObject.h"
#include "cv_utils.h"
#include "pcl_utils.h"
#include "cam_config.h"
#include "../happly/happly.h"
#include <random>
#include "Timer.h"

#ifndef MESH_GENERATOR_H
#define MESH_GENERATOR_H

namespace MeshGen
{
using namespace std;
using namespace cv;
using namespace pcl;

static default_random_engine e;
static uniform_real_distribution<double> u(0.0, 1.0);
array<double,3> generateRandomColor()
{
    array<double,3> ret_val;
    ret_val[0] = u(e);
    ret_val[1] = u(e);
    ret_val[2] = u(e);
    return ret_val;
}

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

inline bool check_p3d_valid(const P3d_PLY_T& p3d)
{
    if(p3d[2]<0 || p3d[2]>1000||isinf(p3d[2])||isnan(p3d[2]))
    {
        return false;
    }
    return true;
}
inline vector<int> make_int3_vector(int i1,int i2,int i3)
{
    vector<int> ret_val;
    ret_val.push_back(i1);
    ret_val.push_back(i2);
    ret_val.push_back(i3);
    return ret_val;
}


class MeshGenerator
{

private:
    void checkTriangles(const cvMatT& gradients_im,const vector<cvTriangleT>& input_triangles,vector<uint8_t>& output_success_v,bool do_check = false)//默认不检查.
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
//    shared_ptr<MeshObject> generateMeshWithImagePair(std::pair<shared_ptr<cvMatT> > im_pair);
//    void Mesher::updateMesh3D(
//        const std::unordered_map<LandmarkId, gtsam::Point3>& points_with_id_VIO, // 和vio的特征点对应...和这个没关系
//        std::shared_ptr<StereoFrame> stereo_frame_ptr,//双目图像...
//        const gtsam::Pose3& left_camera_pose, Mesh2D* mesh_2d,//位姿, 2d mesh.
//            // A 2D Mesh of pixels.
//            //typedef Mesh<Vertex2D> Mesh2D;
//        std::vector<cv::Vec6f>* mesh_2d_for_viz,
//        std::vector<cv::Vec6f>* mesh_2d_filtered_for_viz)
    shared_ptr<vector<cvTriangleT> > generateMeshWithImagePair_vKP_disps(std::pair<shared_ptr<cvMatT>,shared_ptr<cvMatT> > im_pair,const vector<Point2f>& kps,const vector<float> disps)
    {
        ScopeTimer t_("generateMeshWithImagePair_vKP_disps");
        LOG(INFO)<<"in generateMeshWithImagePair_vKP_disps input_kps.size():"<<kps.size()<<endl;

        //对图像进行三角剖分.
        cvMatT& l_img = *(im_pair.first);
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


        if(IS_DEBUG_MODE)//DEBUG ONLY
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
            cv::waitKey(1);
        }

//        vector<KeyPoint> valid_keypoints;
//        cv::KeyPoint::convert(valid_kps,valid_keypoints);

        auto map_p2d_to_disps = createMapTableForValidDisps(valid_kps, valid_disps);

        //step<2>.三角剖分
        LOG(INFO)<<"in generateMeshWithImagePair_vKP_disps valid disps.size():"<<valid_kps.size()<<endl;
        vector<cvTriangleT> triangles;
        auto pTriangles_filtered = make_shared<vector<cvTriangleT>>();
        auto &triangles_filtered = *pTriangles_filtered;
        doCVDelaunaySubdiv2d(l_img, valid_kps, triangles);
        LOG(INFO)<<"after delaunay subdiv2d:valid triangles.size():"<<triangles.size()<<endl;

        //step<3>.检查所有三角形.
        //计算canny.
        cvMatT canny;
        doCVCalcCannyEdge(l_img,canny);
        vector<uint8_t> v_success_check_triangle;
        checkTriangles(canny,triangles,v_success_check_triangle);

        //vector<cvTriangleT> triangles_valid;//这里triangle对应的是顶点.应该检查顶点获取disp.

        extract_sub_vec(triangles,triangles_filtered,v_success_check_triangle);
        //LOG(INFO)<<"after checkEdge:valid triangles.size():"<<triangles_filtered.size()<<endl;

        t_.watch("2d mesh created.");

        //LOG(INFO)<<"debug 1"<<endl;
        bool create_ply_by_PCL = false;
        if(create_ply_by_PCL)
        {
            throw "not implemented!";
            exit(0);
            //shared_ptr<PointCloud<PointXYZRGB> > pMesh = createPCLMeshFromTriangles(triangles_filtered,l_img,map_p2d_to_disps,valid_disps,fx_,fy_,cx_,cy_,b_);//二维三角形->三维三角面片
        }
        else
        {   
            //LOG(INFO)<<"debug 2"<<endl;
            //直接写PLY文件.略过3d三角形填充过程.
            unordered_map<P3d_PLY_T, int> point_to_index;
            //int current_index = 0;
            vector<P3d_PLY_T> vVertices;
            vector<vector<int> > vTriangleIDs;
            vector<array<double,3> > vVertexColors;
            //LOG(INFO)<<"debug 3"<<endl;
            for(const auto& tri:triangles_filtered)
            {
                //创建2d的.
                Point2f pts_2d[3];
                pts_2d[0].x = tri[0];
                pts_2d[0].y = tri[1];

                pts_2d[1].x = tri[2];
                pts_2d[1].y = tri[3];

                pts_2d[2].x = tri[4];
                pts_2d[2].y = tri[5];

                //LOG(INFO)<<"debug 3.1"<<endl;
                P3d_PLY_T p0,p1,p2;//创建3d点.std::array<double>
                int result_0 = map_p2d_to_disps.count(pts_2d[0]);
                int result_1 = map_p2d_to_disps.count(pts_2d[1]);
                int result_2 = map_p2d_to_disps.count(pts_2d[2]);
                
                if(!result_0 || !result_1 || !result_2)
                    continue;

                getXYZByUVDispDouble(map_p2d_to_disps.at(pts_2d[0]),fx_,fy_,cx_,cy_,b_,pts_2d[0].x, pts_2d[0].y, p0[0], p0[1], p0[2]);
                getXYZByUVDispDouble(map_p2d_to_disps.at(pts_2d[1]),fx_,fy_,cx_,cy_,b_,pts_2d[1].x, pts_2d[1].y, p1[0], p1[1], p1[2]);
                getXYZByUVDispDouble(map_p2d_to_disps.at(pts_2d[2]),fx_,fy_,cx_,cy_,b_,pts_2d[2].x, pts_2d[2].y, p2[0], p2[1], p2[2]);
                if( (!check_p3d_valid(p0)) || 
                    (!check_p3d_valid(p1)) ||
                    (!check_p3d_valid(p2)) )
                {
                    continue;//三角形无效.
                }
                //LOG(INFO)<<"debug 3.11"<<endl;
                //查索引表.
                int index0,index1,index2;
                if(!point_to_index.count(p0))
                {
                    vVertices.push_back(p0);
                    point_to_index[p0] = vVertices.size()-1;
                }
                //LOG(INFO)<<"debug 3.12"<<endl;
                index0 = point_to_index[p0];

                if(!point_to_index.count(p1))
                {
                    vVertices.push_back(p1);
                    point_to_index[p1] = vVertices.size()-1;
                }
                index1 = point_to_index[p1];
                //LOG(INFO)<<"debug 3.13"<<endl;
                if(!point_to_index.count(p2))
                {
                    vVertices.push_back(p2);
                    point_to_index[p2] = vVertices.size()-1;
                }

                //LOG(INFO)<<"debug 3.2"<<endl;

                index2 = point_to_index[p2];
                vTriangleIDs.push_back(make_int3_vector(index0,index1,index2));                
            }
            //LOG(INFO)<<"debug 4"<<endl;

            //填充随机颜色.
            for(int i = 0;i<vVertices.size();i++)
            {
                vVertexColors.push_back(generateRandomColor());
            }
            t_.watch("3d mesh created.");
            if(IS_DEBUG_MODE)
            {
                //写PLY!
                happly::PLYData plyOut;
                // Add mesh data (elements are created automatically)
                LOG(INFO)<<"Writing ply with "<<vVertices.size()<<"vertices and "<<vTriangleIDs.size()<<"triangles."<<endl;
                plyOut.addVertexPositions(vVertices);
                plyOut.addVertexColors(vVertexColors);
                plyOut.addFaceIndices(vTriangleIDs);


                // Write the object to file
                plyOut.write("my_output_mesh_file.ply", happly::DataFormat::ASCII);
                t_.watch("io finished.");
            }

        }
        return pTriangles_filtered;
    }

//    shared_ptr<PointCloud<PointXYZRGB> > createPCLMeshFromTriangles(vector<cvTriangleT>& triangles,const cvMatT& original_img,unordered_map<Point2f,float>& p2d_to_disps,vector<float>& disps,
//                                                     const float& fx,const float& fy,const float& cx,const float& cy,const float& b,
//                                                     bool doTextureMapping=true)
//    {
//        //创建完整mesh.
//        //尝试贴图
//        LOG(INFO)<<"in createMeshFromTriangles:valid triangles.size():"<<triangles.size()<<endl;
//        if(doTextureMapping)
//        {
//            //...
//        }
//        //三维化.
//        cv::Mat depth_map(original_img.rows,original_img.cols,CV_32F);
//        //cvMatT triangle_render_helper(original_img.rows,original_img.cols,CV_8U,Scalar(0,0,0));//用于帮助标记是否在三角形内.
//        cv::Mat triangle_render_helper(original_img.rows,original_img.cols,CV_8U,Scalar(0,0,0));//用于帮助标记是否在三角形内.
//        generateDepthMapForAllTriangles(triangles,p2d_to_disps,depth_map,triangle_render_helper,
//                                        fx,fy,cx,cy,b);//绘制深度图(用重心坐标插值法).

//        //png2pcd
//        auto pCloud = make_shared<PointCloud<PointXYZRGB> >();
//        PointCloud <PointXYZRGB> pc_debug;
//        img2d_to_mesh_3d(original_img,depth_map,triangle_render_helper,*pCloud,pc_debug,fx,fy,cx,cy,b);
//        // Save the point cloud into a PCD file
//        //DEBUG ONLY:
////        pcl::saveCloud<PointXYZRGB> ("cloud",*pCloud);
//        pcl::io::savePLYFile("mesh.ply",*pCloud);
//        pcl::io::savePLYFile("mesh_debug.ply",pc_debug);
//        return pCloud;
//    }
};



}
#endif
