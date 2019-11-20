#include "SLAMOptimizationGraph.h"
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
    vector<StereoCamConfig> cam_config;
    shared_ptr<mcs::Frame> pLastKF=nullptr,pLastF=nullptr;
    std::chrono::high_resolution_clock::time_point begin_t;
    std::chrono::high_resolution_clock::time_point last_kf_update_t;
    std::chrono::high_resolution_clock::time_point last_frame_update_t;
    bool ever_init = false;

    shared_ptr<mcs::SLAMOptimizationGraph> pGraph;

public:
    SLAM_simple(int argc,char** argv)
    {
        begin_t = std::chrono::high_resolution_clock::now();
        cv::FileStorage settings;
        pGraph = shared_ptr<mcs::SLAMOptimizationGraph>(new mcs::SLAMOptimizationGraph(settings));
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
        shared_ptr<mcs::Frame> pNewF;
        bool needNewKF;
        shared_ptr < vector<std::pair<shared_ptr<mcs::cvMat_T>,shared_ptr<mcs::cvMat_T> > > > pvInputs( new vector<std::pair<shared_ptr<mcs::cvMat_T>,shared_ptr<mcs::cvMat_T> > >());
        pvInputs->push_back(std::make_pair(img1,img2));
        pvInputs->push_back(std::make_pair(img3,img4));
        if(needNewKeyFrame())
        {//关键帧处理.
            bool needNewKF = true;
            bool create_frame_success;
            pNewF = mcs::createFrameStereos(pvInputs,this->cam_config,create_frame_success,needNewKF);
            if(!ever_init)
            {
                LOG(INFO)<<"init SLAM_simple!"<<endl;//初始化SLAM simple.
                pNewF->rotation = Eigen::Matrix3d::Identity();
                pNewF->position = Eigen::Vector3d(0,0,0);
                ever_init = true;
            }
            else
            {
                LOG(INFO)<<"SLAM initiated.Add new kf to optimization graph and do optimize()."<<endl;
                bool track_localframe_success;
                pGraph->addStereoKeyFrameToBackEndAndOptimize(pNewF,nullptr);//TODO.

                if(track_localframe_success)
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
                    LOG(ERROR)<<"Track failure!Using last frame rt."<<endl;
                    pNewF->rotation = getLastFrame()->rotation;
                    pNewF->position = getLastFrame()->position;
                    if(this->SLAMRunningState!= this->STATE_TRACKING_FALIED)
                    {
                        LOG(WARNING)<<"State transfer from "<<state_id_map[SLAMRunningState]<<" to STATE_TRACKING_FAILED!"<<endl;
                    }
                    this->SLAMRunningState = this->STATE_TRACKING_FALIED;
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
            pNewF = mcs::createFrameStereos(pvInputs,this->cam_config,create_frame_success,needNewKF);
            //TODO:
            bool track_and_pnp_ransac_success;
            //mcs::trackAndDoSolvePnPRansacMultiCam(pNewF); //frame_wise tracking....
            pGraph->addOrdinaryStereoFrameToBackendAndOptimize(pNewF,pLastKF);

            if(track_and_pnp_ransac_success)
            {
                //createNewKF...
            }
            else
            {
                //pNewF->rotation = ...
                //pNewF->translation = ...
            }
            pLastF = pNewF;
        }
    }


};
