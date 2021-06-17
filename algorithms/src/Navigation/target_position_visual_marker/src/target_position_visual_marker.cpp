#include "typedefs.h"
#include <glog/logging.h>
#include <pcl/visualization/pcl_visualizer.h>

class TargetPositionVSelector;
void keyboardEventCallback (const pcl::visualization::KeyboardEvent &event,
                            void* selector_void);

class TargetPositionVSelector
{


public:
    typedef pcl::PointCloud<pcl::PointXYZRGBA> VCloudT;

    std::shared_ptr<pcl::visualization::PCLVisualizer> pViewer;
    LidarCloudT::Ptr pMapCloud;
    VCloudT::Ptr pVCloud;
    std::shared_ptr<ros::NodeHandle> pNH;
    vector<pcl::PointXYZ> vPoints;
    pcl::PointXYZ curr_pt;

    void initTargetPositionVSelector(int argc,char** argv)
    {
        pViewer = std::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer);
        pViewer->setBackgroundColor(0.3,0.3,0.3);
        pViewer->initCameraParameters();
        pViewer->setSize(1024,768);
        pViewer->setCameraFieldOfView(2.0);//rad.
        pViewer->setCameraPosition(0,10,20,0,0,0,0,-1,0);
        curr_pt.x = 0;
        curr_pt.y = 0;
        curr_pt.z = 0;

        ros::init(argc,argv,"target_position_visual_marker");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        string map_path;
        if(!ros::param::get("map_path",map_path))
        {
            LOG(ERROR)<<"ERROR: map path empty!"<<endl;
            exit(-1);
        }
        pMapCloud = MapCloudT::Ptr(new MapCloudT);
        pcl::io::loadPCDFile(map_path,*pMapCloud);
        if(pMapCloud->size() == 0)
        {
            LOG(ERROR)<<"Error:map empty!"<<endl;
            throw "Error!";
        }
        LOG(INFO)<<"map loaded."<<endl;
        pVCloud = xyziToxyzrgba(pMapCloud);
        LOG(INFO)<<"RGBA map generated."<<endl;
        pViewer->setShowFPS(true);
        pViewer->addCoordinateSystem(3.0);
        pViewer->removePointCloud("map_cloud");
        pViewer->addPointCloud(pVCloud,"map_cloud");
        pViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"map_cloud");
        pViewer->registerKeyboardCallback(keyboardEventCallback,(void*)this);
        cout<<"\n\nUsage: press wasd kl to move points, space to add current point as target position. \nz to cancel last target. \nEsc to save to selected_targets.txt and quit."<<endl;
    }
    void loop()
    {
        while(ros::ok())
        {
            //LOG(INFO)<<"pViewer spinOnce()"<<endl;
            int i = 0;
            for(int u=0;u<this->vPoints.size()+100;u++)//ensure every point is removed.
            {
                std::stringstream ss;
                ss<<"history_target_pos"<<u;
                std::stringstream ss2;
                ss2<<"Target_"<<u;
                pViewer->removeShape(ss.str());
                pViewer->removeShape(ss2.str());
            }
            for(auto& pt:this->vPoints)
            {
                std::stringstream ss;
                ss<<"history_target_pos"<<i;
                pViewer->addSphere(sphere_coeff_from_point(pt),ss.str());

                std::stringstream ss2;
                ss2<<"Target_"<<i;
                pViewer->addText3D(ss2.str(),pt,1.0,1.0,1.0,1.0,ss2.str());
                i++;
            }
            pViewer->removeShape("body_target_pos");
            pViewer->addSphere(sphere_coeff_from_point(curr_pt,1.5),"body_target_pos");
            pViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0,0.0,0.0,"body_target_pos");
            pViewer->spinOnce();
            ros::spinOnce();
            usleep(2000);
            //LOG(INFO)<<"ros spinOnce()"<<endl;
        }
    }

    void save_targets_and_quit()
    {
        std::ofstream ofs;
        ofs.open("selected_targets.txt");
        for(int i = 0;i<this->vPoints.size();i++)
        {
            auto pt = vPoints.at(i);
            std::stringstream ss;
            ss<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
            ofs<<ss.str();
        }
        ofs.close();
        cout<<"File save to ~/.ros/selected_targets.txt"<<endl;
        exit(0);
    }
private:
    pcl::ModelCoefficients sphere_coeff_from_point(const pcl::PointXYZ& pt,float size=0.8)
    {
        pcl::ModelCoefficients retval;
        retval.values.resize(0);
        retval.values.push_back(pt.x);
        retval.values.push_back(pt.y);
        retval.values.push_back(pt.z);
        retval.values.push_back(size);
        return retval;
    }
    VCloudT::Ptr xyziToxyzrgba(LidarCloudT::Ptr pOriginal)
    {
        VCloudT::Ptr pVC (new VCloudT);
        for(auto pt:pOriginal->points)
        {
            pcl::PointXYZRGBA p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.r = (uint8_t)( 127 + pt.x*0.8);
            p.g = (uint8_t)( 127 + pt.y*0.8);
            p.b = (uint8_t)( 157 + pt.z*0.8);
            p.a = 184;
            pVC->points.push_back(p);
        }
        return pVC;

    }
};

void keyboardEventCallback (const pcl::visualization::KeyboardEvent &event,
                            void* selector_void)
{
    auto selector = static_cast<TargetPositionVSelector*>(selector_void);
    if (event.keyDown()&&event.getKeyCode()==27)  //Esc
    {
        selector->save_targets_and_quit();
    }
    if (event.keyDown()&&event.getKeyCode()=='w')
    {
        selector->curr_pt.y -= 0.25;
    }
    if (event.keyDown()&&event.getKeyCode()=='a')
    {
        selector->curr_pt.x += 0.25;
    }
    if (event.keyDown()&&event.getKeyCode()=='s')
    {
        selector->curr_pt.y += 0.25;
    }
    if (event.keyDown()&&event.getKeyCode()=='d')
    {
        selector->curr_pt.x -= 0.25;
    }
    if (event.keyDown()&&event.getKeyCode()=='k')
    {
        selector->curr_pt.z+=0.25;
    }
    if (event.keyDown()&&event.getKeyCode()=='l')
    {
        selector->curr_pt.z-=0.25;
    }
    if (event.keyDown()&&event.getKeyCode()==' ')
    {
        selector->vPoints.push_back(selector->curr_pt);
    }
    if (event.keyDown()&&event.getKeyCode()=='z')
    {
        if(selector->vPoints.size()>0)
        {
            LOG(INFO)<<"Last target removed."<<endl;
            selector->vPoints.pop_back();
        }
        else
        {
            LOG(WARNING)<<"Try to pop a empty vector.Do nothing."<<endl;
        }
    }
}

int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("target_position_visual_marker");
    TargetPositionVSelector tpvs;
    tpvs.initTargetPositionVSelector(argc,argv);
    tpvs.loop();
    return 0;
}
