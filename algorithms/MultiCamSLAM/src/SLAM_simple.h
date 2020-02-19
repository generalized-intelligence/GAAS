//#include "SLAMOptimizationGraph.h"
#include "SlidingWindow.h"
#include "utils/Timer.h"
#include "Frame.h"
#include <sensor_msgs/Imu.h>

mcs::IMU_Data_T form_imu_data_from_msg(const sensor_msgs::Imu& msg)
{
    mcs::IMU_Data_T data;
    data.ax = msg.linear_acceleration.x;
    data.ay = msg.linear_acceleration.y;
    data.az = msg.linear_acceleration.z;
    data.alpha_x = msg.angular_velocity.x;
    data.alpha_y = msg.angular_velocity.y;
    data.alpha_z = msg.angular_velocity.z;
    data.covariance_ax = msg.linear_acceleration_covariance.at(0);// is a 3x3 mat in ros.
    data.covariance_ay = msg.linear_acceleration_covariance.at(4);
    data.covariance_az = msg.linear_acceleration_covariance.at(8);
    return data;
}
vector<mcs::IMU_Data_T> form_imu_data_vec_from_msg_vec(const vector<sensor_msgs::Imu>& msg_v)
{
    vector<mcs::IMU_Data_T> ret_vec;
    for(int i = 0;i<msg_v.size();i++)
    {
        ret_vec.push_back(form_imu_data_from_msg(msg_v.at(i)));
    }
    return ret_vec;
}
class SLAM_simple
{
public:
    static const int STATE_TRACKING = 2;
    static const int STATE_UNSTABLE_TRACKING = 1;
    static const int STATE_TRACKING_FALIED = 0;
    map<int,std::string> state_id_map = {
                                            {0,"STATE_TRACKING"},
                                            {1,"STATE_UNSTABLE_TRACKING"},
                                            {2,"STATE_TRACKING_FAILED"}
                                     };
private:
    int SLAMRunningState = STATE_TRACKING_FALIED; // init.
    int unstable_tracking_patience = 0;//暂时不用.用到的时候使用它计量已经有多少个不稳定追踪.
    deque<mcs::Frame> frameQueue;
    vector<sensor_msgs::Imu> imu_vec_tmp;
    vector<StereoCamConfig> stereo_cam_config;
    shared_ptr<mcs::Frame> pLastKF=nullptr,pLastF=nullptr;
    std::chrono::high_resolution_clock::time_point begin_t;
    std::chrono::high_resolution_clock::time_point last_kf_update_t;
    std::chrono::high_resolution_clock::time_point last_frame_update_t;
    bool ever_init = false;
    //shared_ptr<mcs::SLAMOptimizationGraph> pGraph;

    shared_ptr<mcs::SlidingWindow> pWind;

public:
    SLAM_simple(int argc,char** argv)
    {
        if(argc < 2)
        {
            throw "argc < 2 terminated in SLAM_simple constructor!";
        }
        begin_t = std::chrono::high_resolution_clock::now();
        cv::FileStorage* pSettings = new cv::FileStorage(argv[1],cv::FileStorage::READ);

        //加载cam config.
//        StereoCamConfig conf((*pSettings)["cams"][0]);
//        StereoCamConfig conf2((*pSettings)["cams"][1]);

//        this->stereo_cam_config.push_back(conf);
//        this->stereo_cam_config.push_back(conf2);

        auto node = (*pSettings)["cams"];

        for(int i = 0;i<node.size();i++)
        {
            this->stereo_cam_config.push_back(node[i]);
        }
        pWind = shared_ptr<mcs::SlidingWindow>(new mcs::SlidingWindow(this->stereo_cam_config.size()));
        //pGraph->initCamsStereo(this->stereo_cam_config);
    }
    bool needNewKeyFrame()
    {
        std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        if (!ever_init || double(std::chrono::duration_cast<std::chrono::nanoseconds>(t_now-last_kf_update_t).count()/1e9) > 0.5)
        {//每0.5s,创建一次关键帧.
            last_kf_update_t = t_now;
            last_frame_update_t = t_now;
            //ever_init = true;
            return true;
        }
        last_frame_update_t = t_now;
        return false;
    }
    shared_ptr<mcs::Frame> getLastKF()
    {
        return this->pLastKF;
    }
    shared_ptr<mcs::Frame> getLastFrame()
    {
        return this->pLastF;
    }
    void iterateWith4Imgs(shared_ptr<cv::Mat> img1,shared_ptr<cv::Mat> img2,shared_ptr<cv::Mat> img3,shared_ptr<cv::Mat> img4)
    {
        ScopeTimer t("SLAM_simple::iterateWith4Imgs()");
        shared_ptr<mcs::Frame> pNewF;
        bool needNewKF;
        shared_ptr < vector<std::pair<shared_ptr<mcs::cvMat_T>,shared_ptr<mcs::cvMat_T> > > > pvInputs( new vector<std::pair<shared_ptr<mcs::cvMat_T>,shared_ptr<mcs::cvMat_T> > >());
        pvInputs->push_back(std::make_pair(img1,img2));
        pvInputs->push_back(std::make_pair(img3,img4));
        int tracked_pts_count_out;



        //if(needNewKeyFrame())

        //if(pGraph->getFrameID() ==  0)//DEBUG ONLY.
        //if(!ever_init)//||needNewKeyFrame())//DEBUG ONLY!
        if(needNewKeyFrame())
        {//关键帧处理.
            //bool needNewKF = true;
            bool needNewKF = false;//DEBUG ONLY.Using inner strategy in SlidingWindow.
            bool create_frame_success;
            auto v_imu = form_imu_data_vec_from_msg_vec(this->imu_vec_tmp);
            pNewF = mcs::createFrameStereos(pvInputs,this->stereo_cam_config,create_frame_success,needNewKF,&v_imu);
            this->imu_vec_tmp.clear();
            if(!ever_init)
            //初始化SLAM simple.
            //第一帧,必须是关键帧.
            {
                LOG(INFO)<<"init SLAM_simple!"<<endl;
                pNewF->rotation = Eigen::Matrix3d::Identity();
                pNewF->position = Eigen::Vector3d(0,0,0);
                ever_init = true;
                //pGraph->addStereoKeyFrameToBackEndAndOptimize(pNewF,nullptr,tracked_pts_count_out);//TODO.
                //pWind->insertKFintoSlidingWindow(pNewF);
                bool useless_;
                pWind->insertAFrameIntoSlidingWindow(pNewF,true,useless_);
            }
            else
            {
                LOG(INFO)<<"SLAM initiated.Add new kf to optimization graph and do optimize()."<<endl;
                bool track_local_success;
                //pGraph->addStereoKeyFrameToBackEndAndOptimize(pNewF,pLastKF,tracked_pts_count_out);//TODO.


                //pWind->insertKFintoSlidingWindow(pNewF);
                pWind->insertAFrameIntoSlidingWindow(pNewF,false,track_local_success);

                if(track_local_success)//当前帧追踪局部成功
                //if(true) //DEBUG ONLY!
                {
                    //pNewF->rotation = ...
                    //pNewF->position =
                    if(this->SLAMRunningState != STATE_TRACKING)
                    {
                        LOG(INFO)<<"State transfer from "<<state_id_map[SLAMRunningState]<<" to STATE_TRACKING!"<<endl;
                    }
                    this->SLAMRunningState = this->STATE_TRACKING;
                }
                else
                {
//                    LOG(ERROR)<<"Track failure!Using last frame rt."<<endl;
//                    pNewF->rotation = getLastFrame()->rotation;
//                    pNewF->position = getLastFrame()->position;
                    if(this->SLAMRunningState!= this->STATE_TRACKING_FALIED)
                    {
                        LOG(ERROR)<<"State transfer from "<<state_id_map[SLAMRunningState]<<" to STATE_TRACKING_FAILED!"<<endl;
                    }
                    this->SLAMRunningState = this->STATE_TRACKING_FALIED;
                    return;//这种情况下KF F都不变.暂不考虑IMU问题.
                }
            }
            pLastKF = pNewF;
            pLastF = pNewF;
        }
        else
        {//普通帧处理.
            bool needNewKF =false;
            bool create_frame_success;
            last_frame_update_t = std::chrono::high_resolution_clock::now();
            auto v_imu = form_imu_data_vec_from_msg_vec(this->imu_vec_tmp);
            pNewF = mcs::createFrameStereos(pvInputs,this->stereo_cam_config,create_frame_success,needNewKF,&v_imu);
            this->imu_vec_tmp.clear();
            //TODO:
            //bool track_and_pnp_ransac_success;//pnp 验证不要了.
            //mcs::trackAndDoSolvePnPRansacMultiCam(pNewF); //frame_wise tracking....
            cout<<"in iterateWith4Imgs: track ordinary frame:referring frame id:"<<pLastKF->frame_id<<endl;
            //pGraph->addOrdinaryStereoFrameToBackendAndOptimize(pNewF,pLastKF,tracked_pts_count_out);



            //pWind->insertOrdinaryFrameintoSlidingWindow(pNewF);
            bool track_local_success;
            pWind->insertAFrameIntoSlidingWindow(pNewF,false,track_local_success);

            if(track_local_success)
            {
                //createNewKF...
                pLastF = pNewF;
            }
            else
            {
                //pNewF->rotation = ...
                //pNewF->translation = ...
                LOG(ERROR)<<"State transfer from "<<state_id_map[SLAMRunningState]<<" to STATE_TRACKING_FAILED!"<<endl;
                this->SLAMRunningState = this->STATE_TRACKING_FALIED;
                return;//这种情况下KF F都不变.暂不考虑IMU问题.
            }

        }
    }
    void iterateWithImgs(shared_ptr < vector<std::pair<shared_ptr<mcs::cvMat_T>,shared_ptr<mcs::cvMat_T> > > > pvInputs)
    {
        ScopeTimer t("SLAM_simple::iterateWith4Imgs()");
        shared_ptr<mcs::Frame> pNewF;
        bool needNewKF;
//        shared_ptr < vector<std::pair<shared_ptr<mcs::cvMat_T>,shared_ptr<mcs::cvMat_T> > > > pvInputs( new vector<std::pair<shared_ptr<mcs::cvMat_T>,shared_ptr<mcs::cvMat_T> > >());
//        pvInputs->push_back(std::make_pair(img1,img2));
//        pvInputs->push_back(std::make_pair(img3,img4));
        int tracked_pts_count_out;



        //if(needNewKeyFrame())

        //if(pGraph->getFrameID() ==  0)//DEBUG ONLY.
        //if(!ever_init)//||needNewKeyFrame())//DEBUG ONLY!
        if(needNewKeyFrame())
        {//关键帧处理.
            //bool needNewKF = true;
            bool needNewKF = false;//DEBUG ONLY.Using inner strategy in SlidingWindow.
            bool create_frame_success;
            auto v_imu = form_imu_data_vec_from_msg_vec(this->imu_vec_tmp);
            pNewF = mcs::createFrameStereos(pvInputs,this->stereo_cam_config,create_frame_success,needNewKF,&v_imu);
            this->imu_vec_tmp.clear();
            if(!ever_init)
            //初始化SLAM simple.
            //第一帧,必须是关键帧.
            {
                LOG(INFO)<<"init SLAM_simple!"<<endl;
                pNewF->rotation = Eigen::Matrix3d::Identity();
                pNewF->position = Eigen::Vector3d(0,0,0);
                ever_init = true;
                //pGraph->addStereoKeyFrameToBackEndAndOptimize(pNewF,nullptr,tracked_pts_count_out);//TODO.
                //pWind->insertKFintoSlidingWindow(pNewF);
                bool useless_;
                pWind->insertAFrameIntoSlidingWindow(pNewF,true,useless_);
            }
            else
            {
                LOG(INFO)<<"SLAM initiated.Add new kf to optimization graph and do optimize()."<<endl;
                bool track_local_success;
                //pGraph->addStereoKeyFrameToBackEndAndOptimize(pNewF,pLastKF,tracked_pts_count_out);//TODO.


                //pWind->insertKFintoSlidingWindow(pNewF);
                pWind->insertAFrameIntoSlidingWindow(pNewF,false,track_local_success);

                if(track_local_success)//当前帧追踪局部成功
                //if(true) //DEBUG ONLY!
                {
                    //pNewF->rotation = ...
                    //pNewF->position =
                    if(this->SLAMRunningState != STATE_TRACKING)
                    {
                        LOG(INFO)<<"State transfer from "<<state_id_map[SLAMRunningState]<<" to STATE_TRACKING!"<<endl;
                    }
                    this->SLAMRunningState = this->STATE_TRACKING;
                }
                else
                {
//                    LOG(ERROR)<<"Track failure!Using last frame rt."<<endl;
//                    pNewF->rotation = getLastFrame()->rotation;
//                    pNewF->position = getLastFrame()->position;
                    if(this->SLAMRunningState!= this->STATE_TRACKING_FALIED)
                    {
                        LOG(ERROR)<<"State transfer from "<<state_id_map[SLAMRunningState]<<" to STATE_TRACKING_FAILED!"<<endl;
                    }
                    this->SLAMRunningState = this->STATE_TRACKING_FALIED;
                    return;//这种情况下KF F都不变.暂不考虑IMU问题.
                }
            }
            pLastKF = pNewF;
            pLastF = pNewF;
        }
        else
        {//普通帧处理.
            bool needNewKF =false;
            bool create_frame_success;
            last_frame_update_t = std::chrono::high_resolution_clock::now();
            auto v_imu = form_imu_data_vec_from_msg_vec(this->imu_vec_tmp);
            pNewF = mcs::createFrameStereos(pvInputs,this->stereo_cam_config,create_frame_success,needNewKF,&v_imu);
            this->imu_vec_tmp.clear();
            //TODO:
            //bool track_and_pnp_ransac_success;//pnp 验证不要了.
            //mcs::trackAndDoSolvePnPRansacMultiCam(pNewF); //frame_wise tracking....
            cout<<"in iterateWith4Imgs: track ordinary frame:referring frame id:"<<pLastKF->frame_id<<endl;
            //pGraph->addOrdinaryStereoFrameToBackendAndOptimize(pNewF,pLastKF,tracked_pts_count_out);



            //pWind->insertOrdinaryFrameintoSlidingWindow(pNewF);
            bool track_local_success;
            pWind->insertAFrameIntoSlidingWindow(pNewF,false,track_local_success);

            if(track_local_success)
            {
                //createNewKF...
                pLastF = pNewF;
            }
            else
            {
                //pNewF->rotation = ...
                //pNewF->translation = ...
                LOG(ERROR)<<"State transfer from "<<state_id_map[SLAMRunningState]<<" to STATE_TRACKING_FAILED!"<<endl;
                this->SLAMRunningState = this->STATE_TRACKING_FALIED;
                return;//这种情况下KF F都不变.暂不考虑IMU问题.
            }

        }
    }
    void addIMUInfo(const sensor_msgs::Imu& imu_info)
    {
        this->imu_vec_tmp.push_back(imu_info);
    }


};
