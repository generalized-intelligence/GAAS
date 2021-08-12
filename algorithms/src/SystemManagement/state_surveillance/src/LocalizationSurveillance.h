#ifndef LOCALIZATION_SURVEILLANCE
#define LOCALIZATION_SURVEILLANCE
#include "SurveillanceModuleAbstract.h"

#include <geometry_msgs/PoseStamped.h>
#include "utils.h"

class LocalizationSurveillanceModule:public SurveillanceModuleAbstract
{
public:
    int runPipelineAndGetState(const geometry_msgs::PoseStampedConstPtr& pRegistrationPose)
    {
        if(!this->ever_init)
        {
            this->lastPose= *pRegistrationPose;
            this->current_status = this->STATUS_NOT_SURE;
            this->ever_init = true;
            return this->current_status;
        }
        //已初始化:
        geometry_msgs::PoseStamped currPose = *pRegistrationPose;
        bool latency_correct = checkLatencyCorrect(this->lastPose,currPose);
        if(!latency_correct)
        {
            //LOG(WARNING)<<"[LocalizationSurveillance] Check your localization latency!"<<endl;
            this->current_status = this->STATUS_WARNING;
            SURVEILLANCE_LOG_STATUS("Check your localization latency!");
            this->lastPose = currPose;
            return this->current_status;
        }

        bool speed_and_angular_rate_correct = checkSpeedAndAngularRate(this->lastPose,currPose);
        if(!speed_and_angular_rate_correct)
        {
            //LOG(ERROR)<<"[LocalizationSurveillance] Check speed and angular rate!"<<endl;
            this->current_status = this->STATUS_WARNING;
            SURVEILLANCE_LOG_STATUS("Check speed and angular rate!");
        }
        else
        {
            this->current_status = this->STATUS_CORRECT;
        }
        this->lastPose = currPose;
        return this->current_status;
    }
    virtual void initSurveillanceModule()
    {
        setModuleName("Localization Module");
    }
    virtual int checkStateLegal()
    {
        return this->current_status;
    }
private:
    bool ever_init = false;
    geometry_msgs::PoseStamped lastPose;
    bool checkLatencyCorrect(const geometry_msgs::PoseStamped& pose1,geometry_msgs::PoseStamped& pose2)
    {
        double dt = (pose2.header.stamp-pose1.header.stamp).toSec();
        const double THRES_LOCALIZATION_DT = 0.3;//TODO:set in config file.
        if(dt<THRES_LOCALIZATION_DT&&dt > 0.0001)
        {
            return true;
        }
        return false;
    }
    bool checkSpeedAndAngularRate(const geometry_msgs::PoseStamped& pose1,const geometry_msgs::PoseStamped& pose2)
    {
        Eigen::Matrix4f m1 = posestampedToEigenMatrix4f(pose1);
        Eigen::Matrix4f m2 = posestampedToEigenMatrix4f(pose2);
        double angle_rad,euclidean_dist;
        getEuclideanDistAndAngularDistFrom2Matrix4f(m1,m2,angle_rad,euclidean_dist);
        double dt = (pose2.header.stamp - pose1.header.stamp).toSec();
        double speed = euclidean_dist/dt;
        double angular_rate_deg = ((angle_rad*180)/3.1415926535)/dt;

        const double THRES_SPEED = 8;// 8m/s
        const double THRES_ANGULAR_RATE_DEG = 60;//60deg/s

        if(speed< THRES_SPEED && angular_rate_deg<THRES_ANGULAR_RATE_DEG)
        {
            return true;
        }
        if(speed>=THRES_SPEED)
        {
            LOG(WARNING)<<"Speed is:"<<speed<<" while threshold is:"<<THRES_SPEED<<endl;
        }
        if(angular_rate_deg>=THRES_ANGULAR_RATE_DEG)
        {
            LOG(WARNING)<<"AngularRate is:"<<angular_rate_deg<<" deg/s while threshold is:"<<THRES_ANGULAR_RATE_DEG<<" deg/s."<<endl;
        }
        return false;
    }

};



#endif
