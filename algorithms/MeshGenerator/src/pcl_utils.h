#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>

#include "cv_utils.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>




namespace MeshGen
{
using namespace cv;
using namespace std;
using namespace pcl;

void img2d_to_mesh_3d(const cv::Mat& original_img,const cv::Mat& depth_map,const cv::Mat& render_helper,PointCloud<PointXYZRGB>& output_cloud,
                      const float& fx,const float& fy,const float& cx,const float& cy,const float& b)
{
    assert(original_img.cols == depth_map.cols&&
           original_img.rows == depth_map.rows&&
           depth_map.cols == render_helper.cols&&
           depth_map.rows == render_helper.rows);
    output_cloud.width = depth_map.cols;
    output_cloud.height = depth_map.rows; // 有组织点云 后续算法效率高很多.
    output_cloud.is_dense = false;//点云中不含无穷远.
    output_cloud.resize (output_cloud.width * output_cloud.height);

    for (int v = 0; v < output_cloud.height; v++)
    {
        for (int u = 0; u < output_cloud.width; u++)
        {
            if(render_helper.at<uint8_t>(v,u) == 255)//填充成功的点.
            {
                PointXYZRGB xyzrgb;
                float x_3d,y_3d,z_3d;
                z_3d = depth_map.at<float>(v,u);
                getXYbyZ(fx,fy,cx,cy,b,z_3d,u,v,x_3d,y_3d);
                //        depth2xyz (v_viewing_angle, h_viewing_angle,
                //                   rgb_depth_cloud.width, rgb_depth_cloud.height, x, y,
                //                   depth, xyzrgb.x, xyzrgb.y, xyzrgb.z);
                xyzrgb.x = x_3d;xyzrgb.y=y_3d;xyzrgb.z = z_3d;
                xyzrgb.b = static_cast<std::uint8_t> (original_img.at<cv::Vec3b>(v,u)[0]);
                xyzrgb.g = static_cast<std::uint8_t> (original_img.at<cv::Vec3b>(v,u)[1]);
                xyzrgb.r = static_cast<std::uint8_t> (original_img.at<cv::Vec3b>(v,u)[2]);

                output_cloud(u, output_cloud.height - v - 1) = xyzrgb;//注意PCL点云的height是倒过来的.这sb玩意.
            }
        }
    }

    // Save the point cloud into a PCD file
    //DEBUG ONLY:
    //saveCloud<PointXYZRGB> (argv[pcd_file_indices[0]], rgb_depth_cloud, format);
}


}
