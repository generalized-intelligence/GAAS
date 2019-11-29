#ifndef FRAME_H_FILE_PROTECT
#define FRAME_H_FILE_PROTECT

#include "TypeDefs.h"
#include <glog/logging.h>
#include <vector>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>  
#include <Eigen/StdVector>

#include "stereo_cam.h"
namespace mcs
{
    using namespace std;
    using Eigen::Vector2f;
    using Eigen::Vector2d;
    using Eigen::Vector3d;
    using Eigen::Matrix3d;

    typedef cv::Point2f p2dT;
    typedef cv::Point3f p3dT;
    typedef std::pair<shared_ptr<cvMat_T>,shared_ptr<cvMat_T> > StereoMatPtrPair;
    typedef cv::Mat Feature;

    const static int MAP_POINT_STATE_IMMATURE = 0;
    const static int MAP_POINT_STATE_MATURE = 1;
    struct Frame;
    struct MapPoint;
    struct FeaturePoint;
    struct MapPoint
    {
        Feature feat;
        cv::Point3d pos;
        int state;
        //int createdByFrameID = -1;
        weak_ptr<Frame> pCreatedFrame;
        int optimization_graph_index = -1;//在优化图中的index.
    };
    struct FeaturePoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Vector2f mPixel = Vector2f(0, 0);        // the pixel position
        shared_ptr<MapPoint> mpPoint = nullptr;  // the corresponding map point, nullptr if not associated
        float mfInvDepth = -1;                   // inverse depth, invalid if less than zero.

        // data used in ORB
        float mScore = 0;                        // score, maybe harris or other things
        float mAngle = 0;                        // angle of oriented FAST
        size_t mLevel = 0;                       // the pyramid level
        uchar mDesc[32] = {0};                   // 256 bits of ORB feature (32x8), ignored if using LK flow

        // flags
        bool mbOutlier = false;                  // true if it is an outlier
    };
    static const int FRAME_TYPE_STEREO = 0;
    static const int FRAME_TYPE_DEPTH = 1;
    typedef vector<double> IMU_Data_T;
    struct Frame
    {   
    public:
        Frame()
        {
            LOG(ERROR)<<"In null construction function of Frame()! Nothing will be done."<<endl;
        }
        Frame(shared_ptr<vector<StereoMatPtrPair> > pLRImgs)
        {
            this->pLRImgs = pLRImgs;
        }
        Frame(vector<shared_ptr<cvMat_T> > pOriginalImgs,vector<shared_ptr<cv::Mat>>pDepthImgs);
    
        vector<vector<p2dT> > p2d_vv;
        vector<vector<p3dT> > p3d_vv;
        vector<vector<double> > disps_vv;//与p3d一一对应.
        vector<CamInfo> cam_info_vec;
        vector<StereoCamConfig> cam_info_stereo_vec;
        vector<vector<shared_ptr<FeaturePoint> > > feature_points;
        vector<vector<shared_ptr<MapPoint> > > map_points;
        vector<vector<shared_ptr<MapPoint> > >fetchMapPoints(){return map_points;}
        map<int,int> map_p3d_point_id_to_optimization_graph_3dpoint_id; //对应表.
        int get_p3dindex_to_landmark_index(int p3d_index)
        {
            if(map_p3d_point_id_to_optimization_graph_3dpoint_id.count(p3d_index))
            {
                return map_p3d_point_id_to_optimization_graph_3dpoint_id.at(p3d_index);
            }
            else
            {
                return -1;
            }
        }
        void set_p3d_landmark_index(int p3d_index,int landmark_index)
        {
            if(map_p3d_point_id_to_optimization_graph_3dpoint_id.find(p3d_index)!=map_p3d_point_id_to_optimization_graph_3dpoint_id.end())
            {
                LOG(ERROR)<<"ERROR in set_p3d_landmark_index:already exist!"<<endl;
            }
            else
            {
                map_p3d_point_id_to_optimization_graph_3dpoint_id[p3d_index]=landmark_index;
            }
        }


        shared_ptr<vector<StereoMatPtrPair> > pLRImgs;
        shared_ptr<vector<shared_ptr<cvMat_T> > > pOriginalImgs,pDepthImgs;
        vector<map<int,int> > map2d_to_3d_pt_vec;
        vector<map<int,int> > map3d_to_2d_pt_vec;
        IMU_Data_T imu_info_vec;
        int frame_type;
        bool isKeyFrame = false;
        Matrix3d rotation;
        Vector3d position;
        int frame_id = -1;


        vector<CamInfo> get_cam_info()
        {
            return cam_info_vec;
        }
        vector<StereoCamConfig> get_stereo_cam_info()
        {
            return cam_info_stereo_vec;
        }
        int get_cam_num()
        {
            if(this->frame_type == FRAME_TYPE_STEREO)
            {
                return cam_info_stereo_vec.size();
            }
            else if(this->frame_type == FRAME_TYPE_DEPTH)
            {
                return cam_info_vec.size();
            }
            else
            {
                LOG(ERROR)<<"Unsupported cam type in Frame::get_cam_num()."<<endl;
                return -1;
            }
        }
        void removeOriginalImages()
        {
            this->pLRImgs = shared_ptr<vector<StereoMatPtrPair> >(nullptr);
            this->pOriginalImgs = shared_ptr<vector<shared_ptr<cvMat_T> > >(nullptr);
        }
        vector<shared_ptr<cvMat_T> > getMainImages()//for stereo frame: main images is the left ones of each pair;
                                                    //for depth frame: main images is the main rgb/grayscale cam.
        {
            if(this->frame_type == FRAME_TYPE_STEREO)
            {
                vector<shared_ptr<cvMat_T> > ret_vec;
                for(int i = 0;i<this->pLRImgs->size();i++)
                {
                    ret_vec.push_back(std::get<0>((*pLRImgs)[i]));
                }
                LOG(INFO)<<"in getMainImages() ret_vec.size():"<<ret_vec.size()<<endl;
                return ret_vec;
            }
            else
            {
                LOG(ERROR)<<"not implemented yet."<<endl;
            }
        }
        vector<shared_ptr<cvMat_T> > getSecondaryImages()
        {
            if(this->frame_type == FRAME_TYPE_STEREO)
            {
                vector<shared_ptr<cvMat_T> > ret_vec;
                for(int i = 0;i<this->pLRImgs->size();i++)
                {
                    ret_vec.push_back(std::get<1>((*pLRImgs)[i]));
                }
                LOG(INFO)<<"in getSecondaryImages() ret_vec.size():"<<ret_vec.size()<<endl;
                return ret_vec;
            }
            else
            {
                LOG(ERROR)<<"ERROR:depth frame has no secondary image to get!"<<endl;
                throw "no secondary image!";
            }
        }

    };

}
#endif
