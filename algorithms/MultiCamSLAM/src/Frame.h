#ifndef FRAME_H_FILE_PROTECT
#define FRAME_H_FILE_PROTECT

#include "TypeDefs.h"
#include <glog/logging.h>
#include <vector>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>  
#include <Eigen/StdVector>
#include <cmath>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include "stereo_cam.h"
#include "utils/mcs_utils.h"
namespace mcs
{
    using namespace std;
    using Eigen::Vector2f;
    using Eigen::Vector2d;
    using Eigen::Vector3d;
    using Eigen::Matrix3d;

    using gtsam::Rot3;
    using gtsam::Pose3;
    using gtsam::Point3;

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
        bool ever_triangulated = false; // 是否已经被三角化(已知双目观测)
    };
    struct FeaturePoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    typedef struct
    {
        double time;
        double ax = 0,ay = 0,az = 0;
        double alpha_x = 0,alpha_y = 0,alpha_z = 0;
        double covariance_ax = 0,covariance_ay = 0,covariance_az = 0;
        double covariance_alpha_x = 0,covariance_alpha_y = 0,covariance_alpha_z = 0;
    }IMU_Data_T;


    const char TRACK_STEREO2STEREO = 0;
    const char TRACK_STEREO2MONO = 1;
    const char TRACK_MONO2STEREO = 2;
    const char TRACK_MONO2MONO = 3;
    //typedef std::tuple<int,p2dT,float,char> SingleProjectionT;
    struct SingleProjectionT
    {
        int ref_p2d_id = -1;
        p2dT current_frame_p2d;
        float disp;
        char tracking_state;
        //int relativeLandmarkIndex;//查表用的.记录在这里 还是在kf的p2d对应的东西里面?
    };
    typedef vector<vector< SingleProjectionT > > ReprojectionRecordT;

    struct Frame
    {   
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Frame()
        {
            LOG(ERROR)<<"In null construction function of Frame()! Nothing will be done."<<endl;
        }
        Frame(shared_ptr<vector<StereoMatPtrPair> > pLRImgs)
        {
            this->pLRImgs = pLRImgs;
            v_map_p3d_point_id_to_optimization_graph_3dpoint_id.resize(pLRImgs->size());
        }
        void removeImages()
        {//删除所有图像的引用.
            for(auto& pair:(*this->pLRImgs))
            {
                pair.first = nullptr;
                pair.second = nullptr;
            }
        }
        Frame(vector<shared_ptr<cvMat_T> > pOriginalImgs,vector<shared_ptr<cv::Mat>>pDepthImgs);
    
        vector<vector<p2dT> > p2d_vv;
        vector<vector<p3dT> > p3d_vv;



        map<int,ReprojectionRecordT >  reproj_map;//用于普通帧与关键帧track p2d gftt后 存储在这个中;
        //用参考帧的kfid作为索引, map[kfid][cam_id][p2d_id] == ref_p2d_id,pt.

        vector<vector<double> > disps_vv;//与p3d一一对应.
        vector<map<int,int> > kf_p2d_to_landmark_id;//kf_p2d_to_landmark_id[cam_index][p2d_index];
        int getLandmarkIDByCamIndexAndp2dIndex(int cam_index,int p2d_index)//没有是-1;
        {
            if(this->kf_p2d_to_landmark_id.at(cam_index).count(p2d_index) == 0)
            {
                return -1;
            }
            return this->kf_p2d_to_landmark_id.at(cam_index).at(p2d_index);
        }
        void insertLandmarkIDByCamIndexAndp2dIndex(int cam_index,int p2d_index,int landmark_index)
        {
            this->kf_p2d_to_landmark_id.at(cam_index).insert(std::make_pair(p2d_index,landmark_index));
        }
        vector<CamInfo> cam_info_vec;
        vector<StereoCamConfig> cam_info_stereo_vec;
        vector<vector<shared_ptr<FeaturePoint> > > feature_points;
        vector<vector<shared_ptr<MapPoint> > > map_points;
        vector<vector<shared_ptr<MapPoint> > >fetchMapPoints(){return map_points;}
        vector<map<int,int> > v_map_p3d_point_id_to_optimization_graph_3dpoint_id; //对应表.
        int get_p3dindex_to_landmark_index(int p3d_index,int cam_id)//这里应该有对应的摄像机id.因为不一定只有一组
        {
            if(v_map_p3d_point_id_to_optimization_graph_3dpoint_id.at(cam_id).count(p3d_index))
            {
                return v_map_p3d_point_id_to_optimization_graph_3dpoint_id.at(cam_id).at(p3d_index);
            }
            else
            {
                return -1;
            }
        }
        void set_p3d_landmark_index(int p3d_index,int landmark_index,int cam_id)
        {
            if(v_map_p3d_point_id_to_optimization_graph_3dpoint_id.at(cam_id).find(p3d_index)!=v_map_p3d_point_id_to_optimization_graph_3dpoint_id.at(cam_id).end())
            {
                LOG(ERROR)<<"ERROR in set_p3d_landmark_index:already exist!"<<endl;
            }
            else
            {
                v_map_p3d_point_id_to_optimization_graph_3dpoint_id.at(cam_id)[p3d_index]=landmark_index;
            }
        }


        shared_ptr<vector<StereoMatPtrPair> > pLRImgs;
        shared_ptr<vector<shared_ptr<cvMat_T> > > pOriginalImgs,pDepthImgs;
        vector<map<int,int> > map2d_to_3d_pt_vec;
        vector<map<int,int> > map3d_to_2d_pt_vec;
        vector<IMU_Data_T> imu_info_vec;
        int frame_type;
        bool isKeyFrame = false;


        bool pose_estimated = false;
        Matrix3d rotation;
        Vector3d position;
        void setRotationAndTranslation(Matrix3d rot,Vector3d pos)
        {
            this->rotation = rot;
            this->position = pos;
            this->pose_estimated = true;
        }

        void getRotationAndTranslation(gtsam::Rot3& r__, gtsam::Point3& t__,bool& valid)
        {

            Matrix3d rot;Vector3d pos;
            valid = pose_estimated;
            rot = this->rotation;
            pos = this->position;
            r__ = gtsam::Rot3(rot);
            t__ = gtsam::Point3(pos);
        }
        std::pair<Pose3,vector<Pose3> > getFiAndXiArray()//直接生成需要的Fi和Xi位置.
        {
            assert(this->pose_estimated);
            Pose3 Fi(Rot3(this->rotation),Point3(this->position));
            vector<Pose3> cam_pose_v;
            const int cam_count = this->get_cam_num();
            for(int i = 0;i<cam_count;i++)
            {
                auto stereo_config = this->cam_info_stereo_vec.at(i);
                Matrix3d r_mat;
                cv::cv2eigen(stereo_config.get_RMat(),r_mat);
                float x,y,z;
                stereo_config.get_tMat(x,y,z);
                Vector3d t_(x,y,z);
                cam_pose_v.push_back(Pose3(Rot3(r_mat),Point3(t_)));
            }
            std::pair<Pose3,vector<Pose3> > retval;
            retval.first = Fi;
            retval.second = cam_pose_v;
            return retval;
            //return std::make_pair<Pose3,vector<Pose3> > (Fi,cam_pose_v);
        }
        vector<Pose3> getXiArray()//直接生成需要的Fi和Xi位置.
        {
            vector<Pose3> cam_pose_v;
            const int cam_count = this->get_cam_num();
            for(int i = 0;i<cam_count;i++)
            {
                auto stereo_config = this->cam_info_stereo_vec.at(i);
                Matrix3d r_mat;
                cv::cv2eigen(stereo_config.get_RMat(),r_mat);
                float x,y,z;
                stereo_config.get_tMat(x,y,z);
                Vector3d t_(x,y,z);
                cam_pose_v.push_back(Pose3(Rot3(r_mat),Point3(t_)));
            }
            return cam_pose_v;
        }

        int frame_id = -1;
        vector<int> referringKFIDs;
        vector<int> getReferringKFIDs()
        {
            return referringKFIDs;
        }
        void setReferringID(int ref_kf_id)
        {
            this->referringKFIDs.push_back(ref_kf_id);
        }
        int getLastKFID()
        {
            if(referringKFIDs.size() == 0)
            {
                LOG(ERROR)<<"KFIDs.size() == 0! Access Violation!"<<endl;
            }
            return referringKFIDs.back();
        }

        void checkFrameIntegrity_debug()
        {
            cout<<"FOR FRAME_ID:"<<frame_id<<endl;
            if(pLRImgs->size()>0)
            {
                for(auto pair_im : *pLRImgs)
                {
                    if(!std::get<0>(pair_im)->empty() && !std::get<1>(pair_im)->empty())
                    {
                        cout<<"     FRAME_HAS_IMG_CONTENT"<<endl;
                    }
                }
            }
            else
            {
                cout<<" FRAME HAS NO IMG!"<<endl;
            }
            int map_2_3_size = map2d_to_3d_pt_vec.size();
            int map_3_2_size = map3d_to_2d_pt_vec.size();
            cout<<"map2d_to_3d_pts_vec.size:"<<map_2_3_size<<endl;
            cout<<"map3d_to_2d_pts_vec.size:"<<map_3_2_size<<endl;
            //验证p2d到p3d
            for(int i = 0;i<map_2_3_size;i++)
            {
                cout<<" check integrity of map2d_to_3d_pts_.at:"<<i<<endl;//第一个问题在generate stereo keyframe里面 查找p2d,p3d id的时候,不能有匹配错误.
                cout<<" p2d at i.size:"<<p2d_vv.at(i).size()<<",p3d at i.size:"<<p3d_vv.at(i).size()<<endl;
                for(auto iter = map2d_to_3d_pt_vec.at(i).begin();iter!=map2d_to_3d_pt_vec.at(i).end();++iter)
                {
                    cout<<"         first:"<<iter->first<<",second:"<<iter->second<<endl;
                    cout<<"         p2d:"<<this->p2d_vv.at(i).at(iter->first)<<",p3d:"<<this->p3d_vv.at(i).at(iter->second)<<endl;
                    auto p3d_ = this->p3d_vv.at(i).at(iter->second);
                    if(isnan(p3d_.x)||isnan(p3d_.y)||isnan(p3d_.z))
                    {
                        LOG(ERROR)<<"p3d at i:"<<i<<","<<"second:"<<iter->second<<" has NAN inside!Disparity:"<<
                                    this->disps_vv.at(i).at(iter->second)<<endl;
                    }
                }
            }
            //反向验证.


        }


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
                LOG(ERROR)<<"no implementation"<<endl;
                throw "error:no implementation.";
                return vector<shared_ptr<cvMat_T>>();
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
        StatisticOfTrackingStateForAllKF track_states;
        bool optimization_valid = false;
        //int last_valid_frame_id = -1;
    };
}
#endif
