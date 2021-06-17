#ifndef MCS_UTILS_H_FILE
#define MCS_UTILS_H_FILE
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d.hpp"

namespace mcs{



void Intersection(set<int> &A,set<int> &B,set<int> &result){//保证a小b大即可.
    set<int>::iterator it;
    it = A.begin();
    while(it != A.end()){
        if(B.find(*it) != B.end()) result.insert(*it);
        it++;
    }
}
bool is_obvious_movement(cv::Mat rvec,cv::Mat tvec)
{
    double r1 = rvec.at<double>(0),r2 = rvec.at<double>(1),r3 = rvec.at<double>(2);
    double t1 = tvec.at<double>(0),t2 = tvec.at<double>(1),t3 = tvec.at<double>(2);

    double t_len = pow(pow(t1,2)+pow(t2,2)+pow(t3,2) ,0.5);
    double rotate_ang = (pow(pow(r1,2)+pow(r2,2)+pow(r3,2) ,0.5)/3.14)*180;
    if ( (t_len>2 )&& (rotate_ang>90)  )
    {
        return true;
    }
    return false;
}

std::string serializeIntVec(const vector<int>& vec)
{
    stringstream ss;
    for(int i = 0;i<vec.size();i++)
    {
        if (i!=0)
        {
            ss<<",";
        }
        ss<<vec.at(i);
    }
    return ss.str();
}

struct TrackStateForCam
{
    int avail = 0;
    int failed = 0;
};
struct StatisticOfTrackingState
{
    int ref_kf_id;
    vector<TrackStateForCam> tracking_avail_failed_count_of_each_cam;
};

void analyzeTrackingStates(const StatisticOfTrackingState& ts,bool& need_KF_output,bool& state_good_output)
{
    need_KF_output = false;
    state_good_output = false;
    LOG(WARNING)<<"In analyzeTrackingState():got a new state."<<endl;
    int cam_count = ts.tracking_avail_failed_count_of_each_cam.size();
    int total_avail_pts = 0;
    int total_failed_pts = 0;
    for(int i = 0;i<cam_count;i++)
    {
        total_avail_pts  += ts.tracking_avail_failed_count_of_each_cam.at(i).avail;
        total_failed_pts +=ts.tracking_avail_failed_count_of_each_cam.at(i).failed;
    }
    LOG(WARNING)<<"total good pts count:"<<total_avail_pts<<";bad pts count:"<<total_failed_pts<<endl;
    if(total_avail_pts*2.0<(total_avail_pts+total_failed_pts))
    {
        state_good_output = false;//只有33%的点是好点
        need_KF_output = true;
    }
    else
    {
        state_good_output = true;
        if(total_avail_pts*1.3<(total_avail_pts+total_failed_pts))
        {
            need_KF_output = true;
        }
        else
        {
            need_KF_output = false;
        }
    }
}
double getScoreOfTrackingState(StatisticOfTrackingState& ts)
{
    int cam_count = ts.tracking_avail_failed_count_of_each_cam.size();
    int total_avail_pts = 0;
    int total_failed_pts = 0;
    for(int i = 0;i<cam_count;i++)
    {
        total_avail_pts  += ts.tracking_avail_failed_count_of_each_cam.at(i).avail;
        total_failed_pts +=ts.tracking_avail_failed_count_of_each_cam.at(i).failed;
    }
    //double score = total_avail_pts*1.0 - total_failed_pts*0.3;
    double score = total_avail_pts*1.0 - total_failed_pts*0.1;
    return score;
}
struct StatisticOfTrackingStateForAllKF
{
    vector<StatisticOfTrackingState> states;
    void add(StatisticOfTrackingState& s)
    {
        this->states.push_back(s);
    }
    bool getBestRefKFid(int& best_id,double& best_score)//获取最佳匹配关键帧.
    {

        double total_score;
        best_score = -1000;
        best_id = -1;
        for(int index = 0;index<this->states.size();index++)
        {
            int ref_kf_id = this->states.at(index).ref_kf_id;
            double score = getScoreOfTrackingState(this->states.at(index));
            if(score>best_score)
            {
                best_score = score;
                best_id = ref_kf_id;
            }
            total_score+=score;
        }

        //if(best_score>30)
        //if(best_score>20)//TODO:这里的逻辑需要反复考量.
        if(total_score>50)
        {
            return true;
        }
        LOG(ERROR)<<"TOO FEW POINTS!Total score:"<<total_score<<endl;
        return false;
    }
    StatisticOfTrackingState find(int kf_id)
    {
        for(int i = 0;i<this->states.size();i++)
        {
            auto rec = this->states.at(i);
            if(rec.ref_kf_id==kf_id)
            {
                return rec;
            }
        }
        return StatisticOfTrackingState();
    }
};
}
#endif
