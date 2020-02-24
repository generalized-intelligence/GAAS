#include <unordered_map>
#include "MeshObject.h"
//#include "imgproc.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include "opencv2/core/ocl.hpp"
#include "omp.h"


#ifndef CV_UTILS_MESHGEN_H
#define CV_UTILS_MESHGEN_H

typedef std::array<double,3> P3d_PLY_T;
namespace std {

template <>
struct hash<cv::Point2f>
{
    size_t operator()(const cv::Point2f& point) const noexcept
    {
        const size_t fnv_prime = 16777619u;
        const size_t fnv_offset_basis = 2166136261u;

        size_t state = fnv_offset_basis;
        for(const uint8_t* p = reinterpret_cast<const uint8_t*>(&point), *end = p + sizeof(cv::Point2f); p < end; ++p)
            state = (state ^ *p) * fnv_prime;

        return state;
    }
};
template <>
struct hash<P3d_PLY_T>
{
    size_t operator()(const P3d_PLY_T& point) const noexcept
    {
        const size_t fnv_prime = 16777619u;
        const size_t fnv_offset_basis = 2166136261u;

        size_t state = fnv_offset_basis;
        for(const uint8_t* p = reinterpret_cast<const uint8_t*>(&point), *end = p + sizeof(double)*3; p < end; ++p)
            state = (state ^ *p) * fnv_prime;

        return state;
    }
};



}
namespace MeshGen
{
#define USE_OPENCL


#ifdef USE_OPENCL
#define HAVE_OPENCL
#endif

using namespace cv;
using namespace std;

vector<Point2i> p2f_array_to_p2i_vec(Point2f* p2f,int count)
{
    vector<Point2i> p2i_v;
    for(int i = 0;i<count;i++)
    {
        p2i_v.push_back(Point2i(std::round(p2f[i].x),std::round(p2f[i].y)));
    }
    return p2i_v;
}



bool checkHighIntensityInsideOfTriangle(const Mat& canny_img,const cvTriangleT& triangle,const float threshold,const int max_intense_count);
void doCVDelaunaySubdiv2d(Mat& img,std::vector<cv::Point2f>& kps);
//void doCVCalcCannyEdge(Mat& img,Mat& canny_img,const double edge_threshold = 40);
template <typename T>
void extract_sub_vec(const vector<T>& input_vec,vector<T>& output_vec,const vector<uint8_t> valid_mask)
{
    assert(input_vec.size() == valid_mask.size());
    assert(output_vec.empty());
    for(int i = 0;i<valid_mask.size();i++)
    {
        if(valid_mask.at(i))
        {
            output_vec.push_back(input_vec.at(i));
        }
    }
}


void doCVDelaunaySubdiv2d(Mat& img,vector<cv::Point2f>& kps,vector<cvTriangleT>& output_triangles)
{
    cv::Size img_size = img.size();
    cv::Rect2f rect(0, 0, img_size.width, img_size.height);
    cv::Subdiv2D subdiv(
        rect);  // subdiv has the delaunay triangulation function//预备三角剖分keypoints_to_triangulate的所有点.


    // Perform triangulation.
    try {
      subdiv.insert(kps);//插入顶点.
    } catch (...) {
      LOG(FATAL) << "CreateMesh2D: subdiv.insert error (2).\n Keypoints to triangulate.size(): "
                 << kps.size();
    }

    subdiv.getTriangleList(output_triangles);//获取结果.
}
void getBoundaryBoxOfcvTriangle(const cvTriangleT& triangle,int& topLeftx,int& topLefty,int& botRightx,int& botRighty)//不检查边界,一定是3个元素的数组.
{
    Point2i t_i[3];
    t_i[0].x = std::round(triangle(0));
    t_i[0].y = std::round(triangle(1));

    t_i[1].x = std::round(triangle(2));
    t_i[1].y = std::round(triangle(3));

    t_i[2].x = std::round(triangle(4));
    t_i[2].y = std::round(triangle(5));

    topLeftx = std::min(t_i[0].x, std::min(t_i[1].x, t_i[2].x));
    topLefty = std::min(t_i[0].y, std::min(t_i[1].y, t_i[2].y));
    botRightx = std::max(t_i[0].x, std::max(t_i[1].x, t_i[2].x));
    botRighty = std::max(t_i[0].y, std::max(t_i[1].y, t_i[2].y));
}

#ifdef USE_OPENCL

void doCVCalcCannyEdge(const Mat& img,Mat& canny_img,const double edge_threshold = 40)
{
    ocl::setUseOpenCL(true);
    // duplicate image to preserve const input
    cv::Mat copy = img.clone();
    cv::Mat gray_im;
    if (copy.channels() > 1)
    {
        cv::cvtColor(copy, gray_im, cv::COLOR_RGB2GRAY);
    }
    else
    {
        gray_im = copy;
    }
    cv::Mat gray_copy = gray_im.clone();
    cv::equalizeHist(gray_copy,gray_copy);
    cv::UMat umat_gray_copy = gray_copy.getUMat(ACCESS_READ).clone();

    // 高斯平滑 3*3
    cv::GaussianBlur(umat_gray_copy, umat_gray_copy, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

    // convert it to grayscale (CV_8UC3 -> CV_8UC1)
    cv::UMat canny_umat;
    cv::Canny(umat_gray_copy, canny_umat, edge_threshold, edge_threshold*3, 3);
    canny_img = canny_umat.getMat(ACCESS_READ).clone();
}
#endif
bool checkHighIntensityInsideOfTriangle(const Mat& canny_img,const cvTriangleT& triangle,const float threshold,const int max_intense_count)
{
    std::vector<std::pair<Point2i, uint8_t>> keypointsWithIntensities;
    //填充三角形.使用边扫描算法.
    //三角形离散化.
    Point2f t_i[3];
    t_i[0].x = triangle(0);
    t_i[0].y = triangle(1);

    t_i[1].x = triangle(2);
    t_i[1].y = triangle(3);

    t_i[2].x = triangle(4);
    t_i[2].y = triangle(5);

    int topLeftx,topLefty,botRightx,botRighty;
    getBoundaryBoxOfcvTriangle(triangle,topLeftx,topLefty,botRightx,botRighty);
    //triangle(0)
    //生成包围盒
    //cv::Rect()
    cv::Mat local(canny_img.rows,canny_img.cols,CV_8U);//用于填充三角形.
//    cv::Mat t_img(canny_img.rows,canny_img.cols,CV_8UC3);//用于绘制三角形.

//    cv::line(t_img,t_i[0],t_i[1],Scalar(0,0,255));//BGR顺序,边绘制成红色.
//    cv::line(t_img,t_i[1],t_i[2],Scalar(0,0,255));
//    cv::line(t_img,t_i[0],t_i[2],Scalar(0,0,255));
//    for(int y = topLeft_y;y<botRight_y;y++)
//    {
//        bool should_be_filled = false;
//        for(int x = topLeft_x;x<botRight_x;x++)
//        {
//            //填充.
//            if(t_img.at<Vec3b>(y,x)[2])
//            {
//                should_be_filled = !should_be_filled;
//            }
//            if(should_be_filled)
//            {
//                local.at<uint8_t>(y,x) = 1;//应该检查.
//            }
//            else
//            {
//                local.at<uint8_t>(y,x) = 0;
//            }
//        }
//    }

    cv::fillConvexPoly(local,p2f_array_to_p2i_vec(t_i,3),Scalar(255,255,255));
    //对照检查.(避免cache反复读取,集中处理.)

    for(int y = topLefty;y<botRighty;y++)
    {
        for(int x = topLeftx;x<botRightx;x++)
        {
            if(local.at<uint8_t>(y,x)&&canny_img.at<uint8_t>(y,x)>threshold)
            {
                Point2i p_(x,y);
                keypointsWithIntensities.push_back(make_pair(p_,canny_img.at<uint8_t>(y,x)));
            }
        }
    }
    if(keypointsWithIntensities.size()>max_intense_count)
    {
        return false;
    }
    return true;
}
inline void getXYbyZ(const float& fx,const float& fy,const float& cx,const float& cy,const float& b,
                const float z,
                const float& u,const float& v,
                float& x,float& y)
{
    x = z*(u-cx)/fx;
    y = z*(v-cy)/fy;
}
inline void getZByUVDisp(const float& disp,const float& fx,const float& fy,const float& cx,const float& cy,const float& b,
                           const float& u,const float& v,
                           float& z)
{
        z = b*fx/(disp+1e-05);//防止浮点溢出.
}

inline void getXYZByUVDispDouble(const float& disp,const float& fx,const float& fy,const float& cx,const float& cy,const float& b,
                           const float& u,const float& v,
                           double& x,double& y,double& z)
{
        z = b*fx/(disp+1e-05);//防止浮点溢出.
        x = z*(u - cx) / fx;
        y = z*(v - cy) / fy;
}
inline KeyPoint point2f_to_kp(const Point2f& p2f)
{
    vector<Point2f> p2fv;
    vector<KeyPoint> kpv;
    p2fv.push_back(p2f);
    cv::KeyPoint::convert(p2fv,kpv);
    return kpv[0];
}

inline void generateDepthMapForATriangle(const cvTriangleT& triangle,const unordered_map<Point2f,float>& pt_disp_map,Mat& depth_img_output,Mat& render_helper,
                                         const float& fx,const float& fy,const float& cx,const float& cy,const float& b)
{
    Point2f pts_2d[3];
    pts_2d[0].x = triangle[0];
    pts_2d[0].y = triangle[1];

    pts_2d[1].x = triangle[2];
    pts_2d[1].y = triangle[3];

    pts_2d[2].x = triangle[4];
    pts_2d[2].y = triangle[5];

    Point3f pts_3d[3];//顶点,xy存uv,z存z.

    //float x_,y_;
    float z0,z1,z2;
    getZByUVDisp(pt_disp_map.at(pts_2d[0]),fx,fy,cx,cy,b,pts_2d[0].x,pts_2d[0].y,z0);
    getZByUVDisp(pt_disp_map.at(pts_2d[1]),fx,fy,cx,cy,b,pts_2d[1].x,pts_2d[1].y,z1);
    getZByUVDisp(pt_disp_map.at(pts_2d[2]),fx,fy,cx,cy,b,pts_2d[2].x,pts_2d[2].y,z2);

    pts_3d[0].x = pts_2d[0].x;
    pts_3d[1].x = pts_2d[1].x;
    pts_3d[2].x = pts_2d[2].x;

    pts_3d[0].y = pts_2d[0].y;
    pts_3d[1].y = pts_2d[1].y;
    pts_3d[2].y = pts_2d[2].y;

    pts_3d[0].z = z0;pts_3d[1].z = z1;pts_3d[2].z = z2;
    //求平面的法向量.
    Vec3f n = pts_3d[0].cross(pts_3d[1]);//求法向量.

    int topLeftx,topLefty,botRightx,botRighty;
    getBoundaryBoxOfcvTriangle(triangle,topLeftx,topLefty,botRightx,botRighty);
    //cv::Mat inside_triangle(botRighty-topLefty,botRightx-topLeftx,CV_8U,Scalar(0));//包围盒,减少创建时间消耗.

    cv::fillConvexPoly(render_helper,p2f_array_to_p2i_vec(pts_2d,3),Scalar(128));//128表示未填充.

    for(int y = topLefty;y<botRighty;y++)
    {
        for(int x = topLeftx;x<botRightx;x++)
        {
//            //对顶点pts_2d[0]展开:
//            float dx = x-pts_2d[0].x;
//            float dy = y-pts_2d[0].y;
//            float dz = -1*(n[0]*dx+n[1]*dy)/(n[2]+0.001);
//            if(render_helper.at<uint8_t>(y,x))//未填充.
//            {
//                depth_img_output.at<float>(y,x) =  z0+dz;//填充深度.
//                render_helper.at<uint8_t>(y,x) = 255;//着色.
//            }
            //对顶点pts_2d[0]展开:

            if(render_helper.at<uint8_t>(y,x)==128)//未填充.
            {
                float dx = x-pts_2d[0].x;
                float dy = y-pts_2d[0].y;
                float dz = -1*(n[0]*dx+n[1]*dy)/(n[2]+0.001);
                depth_img_output.at<float>(y,x) =  z0+dz;//填充深度.
                render_helper.at<uint8_t>(y,x) = 255;
            }
        }
    }
    cv::imshow("1",render_helper);
    cv::waitKey(1);/*
    int unmatched = 0;
    for(int y = 0;y<render_helper.rows;y++)
    {
        for(int x = 0;x<render_helper.cols;x++)
        {
            if(render_helper.at<uint8_t>(y,x) == 128)//未填充.
            {
                unmatched++;
                if(unmatched>10)
                {
                    throw "error!";
                }
            }
        }
    }*/

}
inline void generateDepthMapForAllTriangles(const vector<cvTriangleT>& triangles,const unordered_map<Point2f,float>& pt_disp_map,Mat& depth_img_output,Mat& render_helper,
                                            const float& fx,const float& fy,const float& cx,const float& cy,const float& b)
{
    for(const auto& t:triangles)//可以并行.
    {
        generateDepthMapForATriangle(t,pt_disp_map,depth_img_output,render_helper,fx,fy,cx,cy,b);
    }
    cv::imshow("render helper",render_helper);
    cv::waitKey(0);
}



/* Such a primitive way to fill a triangle.
void filterTrianglesWithGradients(
    const std::vector<cv::Vec6f>& original_triangulation_2D,
    std::vector<cv::Vec6f>* filtered_triangulation_2D,
    const float& gradient_bound,
    const size_t& max_keypoints_with_gradient) const {
  CHECK_NOTNULL(filtered_triangulation_2D);
  CHECK_NE(filtered_triangulation_2D, &original_triangulation_2D)
      << "Input original_triangulation_2D should be different that the object "
      << "pointed by filtered_triangulation_2D. Input=*Output error.";

  if (gradient_bound == -1) {
    // Skip filter.
    *filtered_triangulation_2D = original_triangulation_2D;
    LOG_FIRST_N(WARNING, 1) << "Filter triangles with gradients is disabled.";
    return;
  }

  // Compute img gradients.
  cv::Mat img_grads;
  calcCannyEdge(left_frame_.img_, img_grads);

  // For each triangle, set to full the triangles that have near-zero gradient.
  // triangulation2Dobs_.reserve(triangulation2D.size());
  // TODO far too many loops over triangles.
  for (const cv::Vec6f& triangle : original_triangulation_2D) {
    // Find all pixels with a gradient higher than gradBound.
    std::vector<std::pair<KeypointCV, double>> keypoints_with_high_gradient =
        UtilsOpenCV::FindHighIntensityInTriangle(img_grads, triangle,
                                                 gradient_bound);

    // If no high-grad pixels exist,
    // then this triangle is assumed to be a plane.
    if (keypoints_with_high_gradient.size() <= max_keypoints_with_gradient) {
      filtered_triangulation_2D->push_back(triangle);
    }
  }
}



*/

//UtilsOpenCV::FindHighIntensityInTriangle(const cv::Mat img,
//                                         const cv::Vec6f& px_vertices,
//                                         const float intensityThreshold) {
//  std::vector<std::pair<KeypointCV, double>> keypointsWithIntensities;
//  if (intensityThreshold < 0) {  // check is disabled
//    return keypointsWithIntensities;
//  }

//  static constexpr bool isDebug = false;

//  // parse input vertices
//  int x0 = std::round(px_vertices[0]);
//  int y0 = std::round(px_vertices[1]);
//  int x1 = std::round(px_vertices[2]);
//  int y1 = std::round(px_vertices[3]);
//  int x2 = std::round(px_vertices[4]);
//  int y2 = std::round(px_vertices[5]);

//  // get bounding box
//  int topLeft_x = std::min(x0, std::min(x1, x2));
//  int topLeft_y = std::min(y0, std::min(y1, y2));
//  int botRight_x = std::max(x0, std::max(x1, x2));
//  int botRight_y = std::max(y0, std::max(y1, y2));

//  double min, max;  // for debug
//  cv::Mat imgCopy;  // for debug
//  if (isDebug) {
//    std::vector<cv::Point> pts(3);
//    cv::minMaxLoc(img, &min, &max);
//    imgCopy = img.clone();
//    cv::cvtColor(imgCopy, imgCopy, cv::COLOR_GRAY2BGR);
//    pts[0] = cv::Point(x0, y0);
//    pts[1] = cv::Point(x1, y1);
//    pts[2] = cv::Point(x2, y2);
//    cv::rectangle(imgCopy, cv::Point(topLeft_x, topLeft_y),
//                  cv::Point(botRight_x, botRight_y), cv::Scalar(0, 255, 0));
//    cv::line(imgCopy, pts[0], pts[1], cv::Scalar(0, 255, 0), 1, CV_AA, 0);
//    cv::line(imgCopy, pts[1], pts[2], cv::Scalar(0, 255, 0), 1, CV_AA, 0);
//    cv::line(imgCopy, pts[2], pts[0], cv::Scalar(0, 255, 0), 1, CV_AA, 0);
//  }

//  for (int r = topLeft_y; r < botRight_y; r++) {
//    // find smallest col inside triangle:
//    int min_x = botRight_x;  // initialized to largest
//    int max_x = topLeft_x;   // initialized to smallest
//    int margin = 4;

//    // check triangle 01:
//    if (y0 != y1) {  // in this case segment is horizontal and we can skip it
//      double lambda01 = double(r - y1) / double(y0 - y1);
//      if (lambda01 >= 0 && lambda01 <= 1) {  // intersection belongs to segment
//        int x =
//            std::round((lambda01) * double(x0) + (1 - lambda01) * double(x1));
//        min_x = std::min(min_x, x);  // try to expand segment to the left
//        max_x = std::max(max_x, x);  // try to expand segment to the right
//      }
//    }

//    // check triangle 12:
//    if (y1 != y2) {  // in this case segment is horizontal and we can skip it
//      double lambda12 = double(r - y2) / double(y1 - y2);
//      if (lambda12 >= 0 && lambda12 <= 1) {  // intersection belongs to segment
//        int x =
//            std::round((lambda12) * double(x1) + (1 - lambda12) * double(x2));
//        min_x = std::min(min_x, x);  // try to expand segment to the left
//        max_x = std::max(max_x, x);  // try to expand segment to the right
//      }
//    }

//    // check triangle 20:
//    if (y2 != y0) {  // in this case segment is horizontal and we can skip it
//      double lambda20 = double(r - y0) / double(y2 - y0);
//      if (lambda20 >= 0 && lambda20 <= 1) {  // intersection belongs to segment
//        int x =
//            std::round((lambda20) * double(x2) + (1 - lambda20) * double(x0));
//        min_x = std::min(min_x, x);  // try to expand segment to the left
//        max_x = std::max(max_x, x);  // try to expand segment to the right
//      }
//    }

//    // sanity check
//    CHECK(min_x >= topLeft_x && max_x <= botRight_x)
//        << min_x << " " << topLeft_x << " " << max_x << " " << botRight_x
//        << '\n'
//        << "FindHighIntensityInTriangle: inconsistent extrema.";

//    for (int c = min_x + margin; c < max_x - margin; c++) {
//      float intensity_rc = float(img.at<uint8_t>(r, c));

//      if (isDebug) {
//        LOG(INFO) << "intensity_rc (r,c): " << intensity_rc << " (" << r << ","
//                  << c << ")";
//        LOG(INFO) << "min: " << min << " max " << max;
//        cv::circle(imgCopy, cv::Point(c, r), 1, cv::Scalar(255, 0, 0),
//                   CV_FILLED, CV_AA, 0);
//      }

//      if (intensity_rc > intensityThreshold) {
//        keypointsWithIntensities.push_back(
//            std::make_pair(cv::Point(c, r), intensity_rc));
//        if (isDebug) {
//          cv::circle(imgCopy, cv::Point(c, r), 1, cv::Scalar(0, 0, 255),
//                     CV_FILLED, CV_AA, 0);
//        }
//      }
//    }
//  }

//  if (isDebug) {
//    cv::imshow("imgCopy", imgCopy);
//    cv::waitKey(1);
//  }

//  return keypointsWithIntensities;
//} // namespace VIO


}
#endif
