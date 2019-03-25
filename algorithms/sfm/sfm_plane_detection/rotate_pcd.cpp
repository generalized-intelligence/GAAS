#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>                 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>
#include <vector>

#include <time.h>



#define PI 3.1415926

class Point
{
    public:
    double x;
    double y;
    double z;
    Point(double v[]):x(v[0]),y(v[1]),z(v[2]){}
};

//求法向量
double* get_normal(Point p1, Point p2, Point p3)
{   
    double* norm_vec;
    norm_vec = new double[3];
    double a = ( (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y));
 
    double b = ( (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z));
 
    double c = ( (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x));

    

    double norm = std::sqrt(a*a+b*b+c*c);
    
    norm_vec[0] = a/norm;
    norm_vec[1] = b/norm;
    norm_vec[2] = c/norm;
    // double v[3] = {a/norm, b/norm, c/norm};

    return norm_vec;
}

//求旋转矩阵4*4

double get_theta_from_2_vec(double*v1,double*v2)
{
    double costheta = (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]) / (0.000001+std::sqrt((v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2])*(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2])));
    return std::acos(costheta);
}

Eigen::Matrix4d get_mat(Eigen::VectorXd coed)
{
    double z[3] = {0,0,1};
    // double a[3] = {-0.0554246, 0.0372348, 0.997768};
    // double a[3] = {-0.0842539, 0.882628, 0.462459};
    double a[3] = {coed[0], coed[1], coed[2]};

    std::cout<<"normal vector: "<<coed[0]<<", "<<coed[1]<<", "<<coed[2]<<std::endl;
    // double za = z[0]*a[0] + z[1]*a[1] + z[2]*a[2];

    double o[3] = {0,0,0};
    Point p1(z);
    Point p2(a);
    Point p3(o);
    double* axis_line = get_normal(p1, p2, p3);
    std::cout<<axis_line[1] << std::endl;
    

    // double a_xOy[3] = {a[0], a[1],0};
    // double x_vec[3] = {1,0,0};
    // double thetaxOz= get_theta_from_2_vec(a_xOy,x_vec);
    // //Eigen::Quaterniond q_rotate_to_xOz (std::cos(thetaxOz/2),std::sin(thetaxOz/2),0,0);
    // Eigen::Quaterniond q_rotate_to_xOz (std::cos(thetaxOz/2),0,0,std::sin(thetaxOz/2));


    // double a_xOy_to_z[3] = {a[0],0, a[2]};
    // double z_vec[3] = {0,0,1};
    // double theta_toz = 1.0*get_theta_from_2_vec(a_xOy_to_z, z_vec);
    // Eigen::Quaterniond q_rotate_to_z(std::cos(theta_toz/2),0,std::sin(theta_toz/2),0);
    
    // double a_z0y_to_x[3] = {0, a[1], a[2]};
    // double y_vec[3] = {0,1,0};
    // double theta_yox = get_theta_from_2_vec(a_z0y_to_x,y_vec);
    // Eigen::Quaterniond q_rotate_to_x(std::cos(theta_yox/2),std::sin(theta_yox/2),0,0);
    

    //四元数{cos(r/2), nx*sin(r/2), ny*sin(r/2), nz*sin(r/2)}
    //Eigen::Quaterniond q = q_rotate_to_xOz*q_rotate_to_z;
    //Eigen::Quaterniond q = q_rotate_to_xOz*q_rotate_to_z*q_rotate_to_x;

    double theta_all = -1.0*get_theta_from_2_vec(a, z);
    Eigen::Quaterniond q(std::cos(theta_all/2),axis_line[0]*std::sin(theta_all/2),axis_line[1]*std::sin(theta_all/2),axis_line[2]*std::sin(theta_all/2));
    // Eigen::Quaterniond q(std::cos(theta_all/2),-0.557652*std::sin(theta_all/2),-0.830075*std::sin(theta_all/2),0);

    // Eigen::Quaterniond q = q_rotate_to_xOz;
    Eigen::Matrix3d mat3 = q.toRotationMatrix();
    Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
    mat4.block(0,0,3,3) = mat3;
    mat4(2, 3) = coed[3];

    std::cout << mat3 << std::endl;

    return mat4;
}

void
showHelp ()
{
  std::cout << std::endl;
  std::cout << "Usage: ./rotate_pcd input.pcd [Options]"<< std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                        Show this help." << std::endl;
  std::cout << "     -v:                        Show pcl viewer." << std::endl;
  std::cout << "     -o  output.pcd:            Save inlier points to pcd file." << std::endl;
  std::cout << "     -t  threshold:             Distance threshold (default 0.1)" << std::endl;
  std::cout << "     -cs coord_scale:           Coordinate Scale of pcl viewer (default 100.0)" << std::endl;
}


int main (int argc, char** argv)
{

    std::clock_t t1 = std::clock();
    //creates a PointCloud<PointXYZ> boost shared pointer and initializes it.
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp ();
        exit (0);
    }
    std::string input_path;
    if (argc < 2)
    {
        showHelp ();
        exit (0);
    }
    else
    {
        input_path = argv[1];
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_path, *source_cloud) == -1) //* load the file
        {
            std::cerr << "Input path: " << input_path << std::endl;
            PCL_ERROR ("Couldn't read file model, please try again. \n");
            showHelp();
            return (-1);
        }
    }
    
    std::size_t sz = source_cloud->points.size();
    std::cout<<"Points size: "<<sz<<std::endl;
    

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (source_cloud));
    Eigen::VectorXf coef = Eigen::VectorXf::Zero(4 , 1);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);

    double threshold;
    if (pcl::console::parse_argument (argc, argv, "-t", threshold) != -1)
    {
      std::cout<<"Distance threshold: "<<threshold<<std::endl;
    }
    else
    {
      threshold = 0.1;
      std::cout<<"Distance threshold: 0.1"<<std::endl;
    }

    ransac.setDistanceThreshold (threshold);
    ransac.computeModel();
    // ransac.getInliers(inliers);
    ransac.getModelCoefficients(coef);
    
    Eigen::VectorXd coed = coef.cast<double>();
    
    std::clock_t t2 = std::clock();


    Eigen::Matrix4d transform_1_d = get_mat(coed);

    
    // std::cout << transform_1_d << std::endl;


    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    /*
    void pcl::transformPointCloud(const pcl::PointCloud< PointT > & cloud_in, 
                                    pcl::PointCloud< PointT > &  cloud_out,  
                                    const Eigen::Matrix4f &  transform  ) 
    */
    // Apply an affine transform defined by an Eigen Transform.

    Eigen::Matrix4f transform_1_f = transform_1_d.cast<float>(); 

    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_1_f);
    

    std::clock_t t3 = std::clock();
    // pcl::io::savePCDFileASCII("model_1_transform.pcd", *transformed_cloud);
    std::string output_path;
    if (pcl::console::parse_argument (argc, argv, "-o", output_path) != -1)
    {
        std::cout << "Rotated model will saved at: "<<output_path<<std::endl;
        
    }
    else
    {
        output_path = "transform.pcd";
        std::cout << "Rotated model will saved at: transform.pcd" <<std::endl;
    }
    if (pcl::io::savePCDFileASCII(output_path, *transformed_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't save file model, please try again. \n");
        return (-1);
    }


    std::clock_t t4 = std::clock();

    std::cout <<"Full cost: "<<(double)(t4-t1)/CLOCKS_PER_SEC<<std::endl;

    std::cout <<"RANSaC cost: "<<(double)(t2-t1)/CLOCKS_PER_SEC<<std::endl;
    std::cout <<"Transform cost: "<<(double)(t3-t1)/CLOCKS_PER_SEC<<std::endl;
    // Visualization
    if (pcl::console::find_switch (argc, argv, "-v"))
    {
        printf(  "\nPoint cloud colors :  white  = original point cloud\n"
        "                        red  = transformed point cloud\n");
        pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

        // Define R,G,B colors for the point cloud
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
        //We add the point cloud to the viewer and pass the color handler
        viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
        viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

        double coord_scale;
        if (pcl::console::parse_argument (argc, argv, "-cs", coord_scale) != -1)
        {
            std::cout<<"Display coordinate scale: "<<coord_scale<<std::endl;
        }
        else
        {
            coord_scale = 100.0;
            std::cout<<"Display coordinate scale: 100.0"<<std::endl;
        }
        viewer.addCoordinateSystem (coord_scale, 0);  //Adds 3D axes describing a coordinate system to screen at 0,0,0. 
        viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

        
        

        viewer.setPosition(800, 400); // Setting visualiser window position

        while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
        }
    }

    
  return 0;
}
