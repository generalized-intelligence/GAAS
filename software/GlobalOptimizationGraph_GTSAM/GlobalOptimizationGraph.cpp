#include "GlobalOptimizationGraph.h"
#include <ros/ros.h>
#include "CheckValid.h"

GlobalOptimizationGraph::GlobalOptimizationGraph(int argc,char** argv)
{

    testRPY_INVERSE();
    testRPY_INVERSE(2,3,1);
    testRPY_INVERSE(3,1,2);

    //cv::FileStorage fSettings;//(string(argv[1]),cv::FileStorage::READ);
    this->fSettings.open(string(argv[1]),cv::FileStorage::READ);
    this->GPS_AVAIL_MINIMUM = fSettings["GPS_AVAIL_MINIMUM"];

    //Initialize optimizer:

    //p_isam = new NonlinearISAM (relinearizeInterval);
    int enable_relinearize_in = this->fSettings["ENABLE_RELINEARIZE"];
    bool enable_relinearize = (enable_relinearize_in!=0);
    double relinearizeThres = this->fSettings["RELINEARIZE_THRES"];
    isam2_params_.relinearizeThreshold = relinearizeThres; //0.01;
    int relinearizeSkip_num = this->fSettings["RELINEARIZE_SKIP_NUM"];
    isam2_params_.relinearizeSkip = relinearizeSkip_num; // after what number of params, do relinearization
    isam2_params_.enableRelinearization = enable_relinearize;//false; //禁止relinearize.

    int online_mode_i = fSettings["ONLINE_MODE"];
    if(online_mode_i)
    {
        p_isam = new ISAM2(isam2_params_);
        //p_fixed_lag_smoother = new IncrementalFixedLagSmoother();
        this->online_mode = true;
    }
    else
    {
        this->online_mode = false;
    }

    mInputSlamFile.open("./results/slam_position.txt");
    mGPSPathFile.open("./results/gps_position.txt");
}

bool gps_msg_is_valid(const sensor_msgs::NavSatFix& gps)
{
    if(gps.status.status>=0 && gps.position_covariance_type>=2)
    {
        if( sqrt(pow(gps.position_covariance[0],2) + pow(gps.position_covariance[4],2)) < 30000)//<3
        {
            return true;
        }
    }
    return false;
}

void GlobalOptimizationGraph::addBlockGPS(int msg_index)
{
    LOG(INFO)<<"msg_index: "<<msg_index<<endl;
    gps_time_current = std::chrono::system_clock::now();

    int enable_gps = this->fSettings["ENABLE_GPS"];
    if (enable_gps == 0)
    {
        LOG(INFO)<<"GPS disabled in GlobalOptimizationGraph::addBlockGPS().CHECK CONFIG FILE."<<endl;
        return;
    }

    if(slam_vertex_index==0)
    {
        LOG(WARNING) <<"In addBlockGPS():slam not ready,return."<<endl;
        return;
    }

    LOG(INFO)<<"In addBlockGPS():adding gps pos factor"<<endl;
    auto GPS_msg = pGPS_Buffer->at(msg_index);
    if (!gps_msg_is_valid(GPS_msg))//unstable observation.
    {
        LOG(INFO)<<"GPS msg check failed.Return."<<endl;
        return;
    }

    //initialize GPS at current GPS position, it sets current long, lat and altitude.
    if (last_gps_vertex_index< 0 && !gps_expand_ever_init)
    {
        GPS_coord.expandAt(GPS_msg.longitude, GPS_msg.latitude, GPS_msg.altitude);
        LOG(INFO)<<"Initializing GPS expand at:"<<GPS_msg.longitude<<","<<GPS_msg.latitude<<","<<GPS_msg.altitude<<endl;
        cout <<"Initializing GPS block in Optimization Graph!"<<endl;
        gps_expand_ever_init = true;
    }

//    if(this->allow_gps_usage == false || this->gps_init_success == false||  !(this->status&0x01)  )//check if GPS valid.
//    {
//        cout<<"[WARNING] Unable to add GPS edge.GPS usage is forbidden in config file."<<endl;
//        if(this->allow_gps_usage)
//        {
//            cout<<"trying to reinit gps:"<<endl;
//            this->try_reinit_gps();
//        }
//        return;
//    }

    bool add_match_success = this->p_gps_slam_matcher->addMatch(slam_vertex_index-1, msg_index);
    LOG(INFO)<<"Add match result:"<<add_match_success<<endl;

    bool inGPSLoopMode_out = false;
    int newestState = this->p_state_tranfer_manager->updateState(add_match_success, inGPSLoopMode_out);

    if(newestState == this->p_state_tranfer_manager->STATE_NO_GPS)
    { // no valid GPS received.
        LOG(INFO)<<"Running in addBlockGPS() branch newstate == STATE_NO_GPS"<<endl;
        return;
    }

    if(newestState == this->p_state_tranfer_manager->STATE_INITIALIZING)
    {
        //add relative_pos only.
        LOG(INFO)<<"Running in addBlockGPS() branch newstate == STATE_INITIALIZING"<<endl;
        return;//TODO: absolute position constrained to be added
    }

    if (newestState == this->p_state_tranfer_manager->STATE_WITH_GPS)
    {
        LOG(INFO)<<"Running in addBlockGPS() branch newstate == STATE_WITH_GPS"<<endl;

        //TODO investigate here, potential bug
        if(inGPSLoopMode_out)
        {
            LOG(INFO)<<"addingGPSBlock(): GPS Loop mode."<<endl;

            //GPS 环形约束形式.
            //add pos and refine whole map.TODO.
            //step<1>. insert point constraints
            double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
            double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
            double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();

            bool init_yaw_valid_ = false;//好像gps这块得乘个-1...
            //double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);//这是和是否回环没关系的.
            double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);//这是和是否回环没关系的.
            LOG(INFO)<<"in addBlockGPS():  yaw_init_to_gps value:"<<yaw_init_to_gps<< "init_yaw_valid_:"<<init_yaw_valid_<<endl;

            LOG(INFO) <<"setting gps measurement!"<<endl;
            LOG(INFO) <<"slam_vertex_index:"<<slam_vertex_index<<endl;

            Vector3d gps_measurement_vec3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt);
            bool covariance_valid = (GPS_msg.position_covariance_type >=2);

            double dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) - gps_measurement_vec3d[1]*sin(yaw_init_to_gps); //project to gog coordinate.
            double dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) + gps_measurement_vec3d[0]*sin(yaw_init_to_gps);
            double dh = gps_measurement_vec3d[2];

            LOG(INFO) << "Adding gps measurement:"<<gps_measurement_vec3d[0]<<","<<gps_measurement_vec3d[1]<<endl<<"yaw:init to gps"<<yaw_init_to_gps<<endl;
            if(!init_yaw_valid_)
            {
                LOG(WARNING)<<"init yaw not valid.can not add gps measurement"<<endl;
                return;
            }

            // Horizontal Movement Model
            //TODO: tune the following noise to gain a better performance
            noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(gtsam::Vector2(GPS_msg.position_covariance[0], GPS_msg.position_covariance[4] ));
            //noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(gtsam::Vector2(GPS_msg.position_covariance[0] / 20.0, GPS_msg.position_covariance[4] / 20.0));
            graph.add(GPSPose2Factor(Symbol('x',slam_vertex_index-1), Point2(dx,dy), gpsModel));

            // Height Model
            noiseModel::Diagonal::shared_ptr gps_altitude_model = noiseModel::Diagonal::Sigmas(gtsam::Vector2(GPS_msg.position_covariance[8],0.0));//第二个数没用到,随便填的
            graph.add(GPSAltitudeFactor(Symbol('h',slam_vertex_index-1), Point2(dh,0.0), gps_altitude_model));

            cout <<"insert pos at "<<slam_vertex_index-1<<"."<<endl;
            cout <<"slam buffer size():"<<pSLAM_Buffer->size()<<endl;
            cout <<"graph.size()"<<graph.size()<<endl;
            last_gps_vertex_index = slam_vertex_index-1;
            cout<<"[GPS_INFO]GPS_relative_pos:"<<
                gps_measurement_vec3d[0]<<","<<
                gps_measurement_vec3d[1]<<","<<
                delta_alt<<endl;

            LOG(INFO)<<"Loop mode GPS insertion step<1> finished."<<endl;

            //step<2>.插入gps测算的转角,位移约束,纠正这期间SLAM yaw的总漂移.
            //noiseModel::Diagonal::shared_ptr ...
            if(this->p_state_tranfer_manager->getLastGPSSLAMMatchID()<0)
            { //first time initializing GPS-SLAM matching. No loop to deal with.
                LOG(INFO)<<"Initializing GPS-SLAM matching for the 1st time. Nothing to do in addBlockGPS-Initializing-Step2."<<endl;
                return;
            }

            LOG(INFO)<<"Loop mode GPS insertion step<2> started."<<endl;

            auto gps_covar1_ = this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).position_covariance;//at [0,4,9].
            auto gps_covar2_ = this->pGPS_Buffer->at(msg_index).position_covariance;

            int slam_node_index_loop_ = this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).slam_index;//回环点的index.
            double noise_dx_ = sqrt(gps_covar1_[0]*gps_covar1_[0] + gps_covar2_[0]*gps_covar2_[0]);
            double noise_dy_ = sqrt(gps_covar1_[4]*gps_covar1_[4] + gps_covar2_[4]*gps_covar2_[4]);

            //By using Diagonal::Sigmas, here we do assert that the variance of x,y and yaw are perpendicular.
            //回环是旧到新.Index也应该对应.
            double delta_lon_relative = this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).longitude - this->pGPS_Buffer->at(msg_index).longitude;
            double delta_lat_relative = this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).latitude - this->pGPS_Buffer->at(msg_index).latitude;
            double delta_alt_relative = this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).altitude - this->pGPS_Buffer->at(msg_index).altitude;
            Vector3d gps_measurement_vec3d_diff_(delta_lon_relative*1000*GPS_coord.vari_km_per_lon_deg(),
                                                 delta_lat_relative*1000*GPS_coord.vari_km_per_lat_deg(),
                                                 delta_alt_relative);

            double diff_x = gps_measurement_vec3d_diff_[0]*cos(yaw_init_to_gps) - gps_measurement_vec3d_diff_[1]*sin(yaw_init_to_gps);
            double diff_y = gps_measurement_vec3d_diff_[1]*cos(yaw_init_to_gps) + gps_measurement_vec3d_diff_[0]*sin(yaw_init_to_gps);
            double newest_yaw_diff_rad, newest_yaw_diff_rad_variance;
            this->p_state_tranfer_manager->getNewestYawAndVarianceRad(newest_yaw_diff_rad,newest_yaw_diff_rad_variance);

            double yaw_error_fix_ = newest_yaw_diff_rad - yaw_init_to_gps;//过程中SLAM yaw产生的漂移.
            fix_angle(yaw_error_fix_);
            double diff_yaw = get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_vertex_index-1)) - get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_node_index_loop_)) - yaw_error_fix_;//过程中飞机头部转向的角度.SLAM的角度有漂移,要通过GPS测量纠正后再输入.
            fix_angle(diff_yaw);

            //variance for diff rotation and translation.//TODO fill in angle covariance.
            //noiseModel::Diagonal::shared_ptr noise_model_relative_movement = noiseModel::Diagonal::Sigmas(gtsam::Vector3(noise_dx_, noise_dy_,0.01744*2) );
            //noiseModel::Diagonal::shared_ptr noise_model_relative_altitude_ = noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1,0.001));//第二个数随便填的,没用到.
            noiseModel::Diagonal::shared_ptr noise_model_relative_movement = noiseModel::Diagonal::Sigmas(gtsam::Vector3(10 * noise_dx_, 10 * noise_dy_,0.1) );
            noiseModel::Diagonal::shared_ptr noise_model_relative_altitude_ = noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1,0.001));

            graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x',slam_node_index_loop_),Symbol('x',slam_vertex_index-1), Pose2(diff_x, diff_y, diff_yaw), noise_model_relative_movement);
            graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_node_index_loop_),Symbol('h',slam_vertex_index-1), Point2(delta_alt_relative,0), noise_model_relative_altitude_);
            //graph.emplace_shared<BetweenFactor<Rot2> >(Symbol('y'),...)//TODO:纠正yaw的误差积累.

            LOG(INFO)<<"Loop mode GPS insertion step<2> finished."<<endl;
        }
        else
        {
            // normal operation.
            LOG(INFO)<<"addingGPSBlock():In ordinary mode."<<endl;

            double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
            double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
            double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();
            bool init_yaw_valid_ = false;

            //double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);
            double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);

            LOG(INFO)<<"in addBlockGPS():  yaw_init_to_gps value:"<<yaw_init_to_gps<< "init_yaw_valid_:"<<init_yaw_valid_<<endl;
            LOG(INFO) <<"setting gps measurement!"<<endl;
            LOG(INFO) <<"slam_vertex_index:"<<slam_vertex_index<<endl;

            float gps_x = delta_lon*1000*GPS_coord.vari_km_per_lon_deg();
            float gps_y = delta_lat*1000*GPS_coord.vari_km_per_lat_deg();
            float gps_z = delta_alt;

            Vector3d gps_measurement_vec3d(gps_x, gps_y, gps_z);

//            bool covariance_valid = (GPS_msg.position_covariance_type >=2);
//            double dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) - gps_measurement_vec3d[1]*sin(yaw_init_to_gps); //project to gog coordinate.
//            double dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) + gps_measurement_vec3d[0]*sin(yaw_init_to_gps);
//            double dh = gps_measurement_vec3d[2];

            bool covariance_valid = (GPS_msg.position_covariance_type >=2);
            //yaw_init_to_gps = 1.5708;
            double dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) - gps_measurement_vec3d[1]*sin(yaw_init_to_gps); //project to gog coordinate.
            double dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) + gps_measurement_vec3d[0]*sin(yaw_init_to_gps);
            double dh = gps_measurement_vec3d[2];

            mGPSPathFile <<dx<<","<<dy<<","<<dh<<endl;

            {//debug only.
                auto ps__ = pSLAM_Buffer->at(slam_vertex_index-1).pose.position;
                LOG(INFO)<<"GPS_MEASUREMENT_DEBUG:dxdydh:"<<dx<<","<<dy<<","<<dh<<";"<<"SLAM:"<<ps__.x<<","<<ps__.y<<","<<ps__.z<<endl;
                LOG(INFO) << "Adding gps measurement:"<<gps_measurement_vec3d[0]<<","<<gps_measurement_vec3d[1]<<endl<<"yaw:init to gps"<<yaw_init_to_gps<<endl;
            }

            //TODO potential bug here
//            if(!init_yaw_valid_)
//            {
//                LOG(WARNING)<<"init yaw not valid. Can not add gps measurement"<<endl;
//                return;
//            }

            // global_x, global_y and global_height
            noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(gtsam::Vector2(GPS_msg.position_covariance[0] / 5, GPS_msg.position_covariance[4] / 5));
            graph.add(GPSPose2Factor(Symbol('x', slam_vertex_index-1), Point2(dx, dy), gpsModel));
            noiseModel::Diagonal::shared_ptr gps_altitude_model = noiseModel::Diagonal::Sigmas(gtsam::Vector2(GPS_msg.position_covariance[8]/10, 0.0));//2nd parameter is not used, picked arbitrarily
            graph.add(GPSAltitudeFactor(Symbol('h', slam_vertex_index-1), Point2(dh, 0.0), gps_altitude_model)); //GPS height relative change

            // yaw
            noiseModel::Diagonal::shared_ptr model_yaw_fix = noiseModel::Diagonal::Sigmas(gtsam::Vector3(GPS_msg.position_covariance[0] / 20.0,
                                                                                                         GPS_msg.position_covariance[4] / 20.0,
                                                                                                         1));

            double yaw_slam_diff = get_yaw_from_slam_msg(pSLAM_Buffer->at(slam_vertex_index-1)) - get_yaw_from_slam_msg(this->pSLAM_Buffer->at(this->p_gps_slam_matcher->at(0).slam_index));
            graph.emplace_shared<BetweenFactor<Pose2>>(Symbol('x', this->p_gps_slam_matcher->at(0).slam_index),
                    Symbol('x', slam_vertex_index-1),
                    Pose2(dx, dy, get_yaw_from_slam_msg(pSLAM_Buffer->at(slam_vertex_index-1)) - get_yaw_from_slam_msg(pSLAM_Buffer->at(this->p_gps_slam_matcher->at(0).slam_index))),
                    model_yaw_fix);

            float deg = (get_yaw_from_slam_msg(pSLAM_Buffer->at(slam_vertex_index-1)) - get_yaw_from_slam_msg(pSLAM_Buffer->at(this->p_gps_slam_matcher->at(0).slam_index))) * 180 / 3.1415926;
            LOG(INFO)<<"get_yaw_from_slam_msg(pSLAM_Buffer->at(slam_vertex_index-1)) - get_yaw_from_slam_msg(pSLAM_Buffer->at(0)): "<<deg<<endl;

            //calc YAW correction.
            bool should_update_yaw_correction = p_state_tranfer_manager->get_should_update_yaw_correction(this->slam_vertex_index-1); //检查是否建议更新yaw.
            if(should_update_yaw_correction)
            {
                LOG(INFO)<<"yaw correction update is required!"<<endl;
                int last_slam_id_out,last_match_id_out;//这两个是上一次纠正的最后id.
                this->p_state_tranfer_manager->get_last_yaw_correction_slam_id(last_slam_id_out,last_match_id_out);//从这一点开始匹配.
                //尝试进行GPS-SLAM 匹配.如果成功:更新.否则反复尝试.
                bool update_success_output= false;
                int last_match_stored_in_matcher_id = this->p_gps_slam_matcher->matchLen()-1;
                double new_deg_output,new_deg_output_variance;
                this->p_gps_slam_matcher->check2IndexAndCalcDeltaDeg(last_match_id_out, last_match_stored_in_matcher_id,
                                                                     GPS_coord,update_success_output,new_deg_output,new_deg_output_variance);//尝试计算yaw.
                if(update_success_output)
                {
                    double _rad = new_deg_output*180/3.1415926535;
                    double _rad_variance = new_deg_output_variance*180/3.1415926535;
                    LOG(INFO)<<"YAW_FIX_SUCCESS in match between match_id:"<<last_match_id_out<<","<<last_match_stored_in_matcher_id<<endl;
                    LOG(INFO)<<"YAW_NEWEST_VAL:"<<new_deg_output<<" deg.Variance:"<<new_deg_output_variance<<" deg."<<endl;
                    this->current_yaw_slam_drift = new_deg_output*3.1415926535/180.0 - yaw_init_to_gps; //update drift.
                    LOG(INFO)<<"Insert yaw measurement fix in GPS-local Loop.Fix Value:"<<this->current_yaw_slam_drift*180/3.1415926535<<"deg."<<endl;
                    this->p_state_tranfer_manager->set_last_slam_yaw_correction_id(this->slam_vertex_index-1,last_match_stored_in_matcher_id);
                }
                else
                {
                    LOG(INFO)<<"YAW_FIX_FAILED.ID:"<< last_match_id_out<<","<<last_match_stored_in_matcher_id<<";Values:"<<new_deg_output<<" deg.Variance:"<<new_deg_output_variance<<" deg."<<endl;
                }
            }


            cout <<"insert pos at "<<slam_vertex_index-1<<"."<<endl;
            cout <<"slam buffer size():"<<pSLAM_Buffer->size()<<endl;
            cout <<"graph.size()"<<graph.size()<<endl;
            last_gps_vertex_index = slam_vertex_index-1;
            cout<<"[GPS_INFO]GPS_relative_pos:"<<gps_measurement_vec3d[0]<<","<<
                gps_measurement_vec3d[1]<<","<<
                delta_alt<<endl;
        }
    }
}

bool check_and_fix_dx(double& dx)
{
    if(dx<EPS&&dx>0)
    {
        dx+=EPS;
    }
    if(dx>-1*EPS && dx<0)
    {
        dx-= EPS;
    }
}
double calc_angle_variance(double x,double y,double x_var,double y_var)
{
    double norm = sqrt(x*x+y*y);
    double var_norm = sqrt(x_var*x_var+y_var*y_var);
    return var_norm/(norm+EPS);
}


double get_yaw_from_slam_msg(const geometry_msgs::PoseStamped &m)
{
    auto orient = m.pose.orientation;
    Eigen::Quaterniond q_;
    q_.x() = orient.x;q_.y() = orient.y;q_.z() = orient.z;q_.w() = orient.w;
//
//    Matrix3d R_SLAM_Mat = q_.toRotationMatrix();
//    Vector3d vec_forward = R_SLAM_Mat*Vector3d(1,0,0);
//    double fx,fy;//reproject to xOy;
//    fx = vec_forward[0];fy = vec_forward[1];
//    check_and_fix_dx(fx);
//    double yaw = atan2(fy,fx);
//    fix_angle(yaw);
//    return yaw;
    double roll,pitch,yaw;
    getRPYFromQuat(orient.x,orient.y,orient.z,orient.w,roll,pitch,yaw);
    return yaw;
}

void get_rpy_from_slam_msg(const geometry_msgs::PoseStamped& m,double& roll,double& pitch,double& yaw)
{
    auto orient = m.pose.orientation;
    Eigen::Quaterniond q_;
    q_.x() = orient.x;q_.y() = orient.y;q_.z() = orient.z;q_.w() = orient.w;
    getRPYFromQuat(orient.x,orient.y,orient.z,orient.w,roll,pitch,yaw);
}

double calcnorm(double x,double y)
{
    return sqrt(x*x+y*y);
}

const double dx_slam_var = 0.01;
const double dy_slam_var = 0.01;

const double vel_x_var = 0.05;
const double vel_y_var = 0.05;

void getDxDyFromSlamMsg(const geometry_msgs::PoseStamped& msg)
{

}

void GlobalOptimizationGraph::addBlockBarometer(int msg_index)
{
    auto msg = this->pBarometer_Buffer->at(msg_index);
    int __use_baro_int = this->fSettings["USE_BAROMETER"];
    bool use_baro = (__use_baro_int > 0);
    if(!use_baro)
    {
        LOG(INFO)<<"Barometer usage disabled in GOG setting."<<endl;
        return;
    }
    if(slam_vertex_index==0)
    {
        LOG(WARNING) <<"In addBlockGPS():slam not ready,return."<<endl;
        return;
    }

    //this->baro_manager.xxxx
    bool init_finished = this->p_BarometerManager->init_iterate(msg.fluid_pressure/1000);//input unit: kpa.
    //这里隐式的约定了如果有Barometer消息,则从开始就有.不存在中间发布的情况;且开始时高度变化是很小的.
    if(!init_finished)
    {
        LOG(INFO)<<"Barometer Manager still initializing."<<endl;
        return;
    }
    bool data_valid;
    double height = this->p_BarometerManager->get_current_baro_height(msg.fluid_pressure/1000,data_valid);
    if(!data_valid)
    {
        LOG(INFO)<<"Barometer info invalid!"<<endl;
        return;
    }
    if (last_baro_vertex_index<0)//初始化Baro,并且不插入值.
    {
        //GPS_coord.expandAt(GPS_msg.longitude,GPS_msg.latitude,GPS_msg.altitude); //这里已经初始化了altitude.
        cout <<"Initiating Barometer block in Optimization Graph!"<<endl; //TODO:记录气压计和SLAM的初值差.
    }
    else
    {
        bool hist_avail;
        double diff_height = height - this->p_BarometerManager->get_current_baro_height(this->pBarometer_Buffer->at(msg_index-1).fluid_pressure/1000,hist_avail);
        if(hist_avail)
        {
            noiseModel::Diagonal::shared_ptr model_relative_height_barometer = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0,0.0)); // TODO:挪到配置文件里.现在写死barometer的方差是1.
            graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_vertex_index-1),Symbol('h',last_baro_vertex_index),Point2(diff_height,0.0),model_relative_height_barometer);//计算barometer变化量.
        }
    }
    //TODO:考虑是否要删除其中一种约束.
    noiseModel::Diagonal::shared_ptr model_abs_height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0,0.0));//第二个数没用到,随便填的,第一个固定1.0m
    graph.add(GPSAltitudeFactor(Symbol('h',slam_vertex_index-1),Point2(height,0.0)//第二个数没用到,随便填的
            ,model_abs_height));
    bool gps_baro_diff_ever_init = this->p_BarometerManager->get_gps_diff_ever_init();
    if(!gps_baro_diff_ever_init)//尝试初始化GPS和气压计的差值.
        //TODO:加入策略 在气压计高度产生缓漂时,通过GPS重初始化.
    {
        if(this->last_gps_vertex_index>0&&this->p_state_tranfer_manager->getCurrentState() == this->p_state_tranfer_manager->STATE_WITH_GPS)
        {
            bool init_gps_baro_diff_success;
            this->p_BarometerManager->set_gps_to_baro_height_transformation(msg.fluid_pressure/1000,this->pGPS_Buffer->at(this->pGPS_Buffer->size()-1).altitude,init_gps_baro_diff_success);
            if(!init_gps_baro_diff_success)
            {
                LOG(WARNING)<<"Init GPS_BARO relative height failed!"<<endl;
            }
        }
    }
    last_baro_vertex_index = slam_vertex_index-1;
}

void GlobalOptimizationGraph::addBlockSLAM(int msg_index)
{

    //GPS LOST?
    slam_time_current = std::chrono::system_clock::now();
    auto elapsed_seconds = slam_time_current - gps_time_current;
    double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_seconds).count();

    if(elapsed_ms > 500)
    {
        gps_lost = true;
    }
    else{
        gps_lost = false;
    }

    LOG(INFO)<<"elapsed_ms: "<<elapsed_ms<<endl;
    LOG(INFO)<<"gps_lost: "<<gps_lost<<endl;
    // ----------------------------------------------------------


    //two approaches：  1.create vertex when slam arrives
    //                  2.create vertex when gps arrives。
    //choose the 1st approach
    auto SLAM_msg = pSLAM_Buffer->at(msg_index);

    //NOTE : for comparing results and debugging
    mInputSlamFile << SLAM_msg.pose.position.x<<","<<SLAM_msg.pose.position.y<<","<<SLAM_msg.pose.position.z<<", "<<gps_lost<<endl;

    //addGOGFrame(SLAM_msg.pose.position.x,SLAM_msg.pose.position.y);//create a new map 'vertexPR'
    {
        double r, p, y;
        get_rpy_from_slam_msg(SLAM_msg,r,p,y);
        fix_angle(r);
        fix_angle(p);
        fix_angle(y);
        LOG(INFO)<<"[DEBUG] Yaw in get_yaw_from_slam_msg:"<<y*180/3.1415926<<"\tpitch:"<<p*180/3.1415926<<"\troll:"<<r*180/3.1415926<<endl;
        auto p_ = SLAM_msg.pose.position;
        auto q = SLAM_msg.pose.orientation;
        LOG(INFO)<<"[DEBUG] Full info of slam input:"<<p_.x<<","<<p_.y<<","<<p_.z<<";"<<q.x<<","<<q.y<<","<<q.z<<","<<q.w<<endl;
    }

    cout <<"Insert "<<slam_vertex_index<<"in initialEstimate!"<<endl;
    this->p_state_tranfer_manager->updateSlam();

    if (slam_vertex_index >0)
    {
        //initial guess of abs pos and yaw
        initialEstimate.insert(Symbol('x', slam_vertex_index), Pose2(SLAM_msg.pose.position.x, SLAM_msg.pose.position.y, get_yaw_from_slam_msg(SLAM_msg)) );
        initialEstimate.insert(Symbol('h', slam_vertex_index), Point2(0, 0));

        //calc diff for index
        auto orient = SLAM_msg.pose.orientation;
        Eigen::Quaterniond q_;
        q_.x() = orient.x;
        q_.y() = orient.y;
        q_.z() = orient.z;
        q_.w() = orient.w;
        const geometry_msgs::PoseStamped& slam_msg_old = this->pSLAM_Buffer->at(slam_vertex_index-1);

        double diff_height = SLAM_msg.pose.position.z - slam_msg_old.pose.position.z;
        double diff_yaw = (get_yaw_from_slam_msg(SLAM_msg) -
                           get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_vertex_index - 1)));


        auto Tb1b2 = findRelativeTransformMat(slam_msg_old, SLAM_msg);
        double diff_x = Tb1b2.at<double>(0, 3);
        double diff_y = Tb1b2.at<double>(1, 3);
//        double diff_height = Tb1b2.at<double>(2, 3);
//        cv::Mat _SO2 = Tb1b2.colRange(0, 3).rowRange(0, 3);
//        Eigen::Matrix3d _SO2_eigen;
//        cv2eigen(_SO2, _SO2_eigen);
//        Eigen::Vector3d ea = _SO2_eigen.eulerAngles(0, 1, 2);
//        double diff_yaw = ea[2];
        LOG(INFO)<<"Tb1b2: \n"<<Tb1b2<<endl;
        LOG(INFO)<<"diff_x: \n"<<diff_x<<endl;
        LOG(INFO)<<"diff_y: \n"<<diff_y<<endl;
        LOG(INFO)<<"diff_height: \n"<<diff_height<<endl;
        LOG(INFO)<<"diff_yaw: \n"<<diff_yaw<<endl;

        double slam_xy_noise_m = this->fSettings["SLAM_RELATIVE_XY_VARIANCE_m"];
        double slam_height_noise_m = this->fSettings["SLAM_RELATIVE_HEIGHT_VARIANCE_m"];
        double slam_yaw_noise_deg = this->fSettings["SLAM_RELATIVE_YAW_VARIANCE_deg"];

        //tune parameters
        //noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2,0.2,0.1));//for slam diff rotation and translation.
        //noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(gtsam::Vector3(slam_xy_noise_m, slam_xy_noise_m, 0.01744*slam_yaw_noise_deg));
        noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(gtsam::Vector3(slam_xy_noise_m, slam_xy_noise_m, 0.2));
        noiseModel::Diagonal::shared_ptr model_relative_height_ = noiseModel::Diagonal::Sigmas(gtsam::Vector2(slam_height_noise_m, 0));//1mm/frame
        //LOG(INFO)<<"SLAM relative noise setting xy(m):"<<slam_xy_noise_m<<",yaw(deg):"<<slam_yaw_noise_deg<<",height noise(m):"<<slam_height_noise_m<<endl;

        //'x': xOy, horizontal position;
        //'h': height;
        //'y': yaw offset.
        graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x', slam_vertex_index-1), Symbol('x',slam_vertex_index), Pose2(diff_x, diff_y, diff_yaw), model_relative_movement);
        graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h', slam_vertex_index-1), Symbol('h',slam_vertex_index), Point2(diff_height, 0.0), model_relative_height_);//the second parameter is picked arbitraly

        //graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x', slam_vertex_index), Symbol('x',slam_vertex_index-1), Pose2( diff_x, diff_y, diff_yaw), model_relative_movement);
        //graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_vertex_index), Symbol('h',slam_vertex_index-1), Point2(diff_height, 0.0), model_relative_height_);//the second parameter is picked arbitraly

        {
            // whereas GPS is not available, generate a weak prior for each slam pose received
            // so that the optimized result stays untwisted from SLAM result.
            int newestState = this->p_state_tranfer_manager->getCurrentState();
            if(newestState != this->p_state_tranfer_manager->STATE_WITH_GPS)
            {
                LOG(WARNING)<<"In addBlockSLAM(): GPS not stable; adding slam prior pose restriction."<<endl;
                auto p__ = SLAM_msg.pose.position;
                noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1,0.1,0.01744*10)); //0.1m, 10度.
                //noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.1)); //0.1m, 10度.
                graph.emplace_shared<PriorFactor<Pose2> >(Symbol('x',slam_vertex_index),Pose2(p__.x,p__.y,get_yaw_from_slam_msg(SLAM_msg)), priorNoise_Absolute);//位置和朝向角都初始化成0.
                //noiseModel::Diagonal::shared_ptr priorNoise_Height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0,0.0)); //高度方差:1.
                noiseModel::Diagonal::shared_ptr priorNoise_Height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1,0.0)); //高度方差:1.
                graph.emplace_shared<PriorFactor<Point2> >(Symbol('h',slam_vertex_index),Point2(p__.z,0),priorNoise_Height);//高度初始化成0
            }
        }

//        {
//            // whereas GPS is not available, generate a weak prior for each slam pose received
//            // so that the optimized result stays untwisted from SLAM result.
//            if(gps_lost)
//            {
//                LOG(WARNING)<<"In addBlockSLAM(): GPS not stable; adding slam prior pose restriction."<<endl;
//                auto p__ = SLAM_msg.pose.position;
//                noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1,0.1,0.01744*10)); //0.1m, 10度.
//                //noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.1)); //0.1m, 10度.
//                graph.emplace_shared<PriorFactor<Pose2> >(Symbol('x',slam_vertex_index),Pose2(p__.x,p__.y,get_yaw_from_slam_msg(SLAM_msg)), priorNoise_Absolute);//位置和朝向角都初始化成0.
//                //noiseModel::Diagonal::shared_ptr priorNoise_Height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0,0.0)); //高度方差:1.
//                noiseModel::Diagonal::shared_ptr priorNoise_Height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1,0.0)); //高度方差:1.
//                graph.emplace_shared<PriorFactor<Point2> >(Symbol('h',slam_vertex_index),Point2(p__.z,0),priorNoise_Height);//高度初始化成0
//            }
//        }

        //graph.emplace_shared<BetweenFactor<Rot2> >(Symbol('y',slam_vertex_index-1),Symbol('y',slam_vertex_index),....);// offset in heading
        slam_vertex_index++;
        if(online_mode)
        {

//            // TODO: properly handle underconstrained problem
//            try {
//                p_isam->update(graph, initialEstimate);
//            }
//            catch (gtsam::IndeterminantLinearSystemException){
//                LOG(ERROR)<<"gtsam::IndeterminantLinearSystemException encountered!"<<endl;
//                graph.resize(0);
//                initialEstimate.clear();
//                return;
//            }
//            catch (tbb::captured_exception){
//                LOG(ERROR)<<"tbb::captured_exception encountered!"<<endl;
//                graph.resize(0);
//                initialEstimate.clear();
//                return;
//            }

            // fixed lag smoother
//            graph.print();
//            p_fixed_lag_smoother->update(graph, initialEstimate);
//            Values currentEstimate = p_isam->calculateEstimate();

            if(gps_lost)
            {
                graph.print();
            }

            // isam2
            p_isam->update(graph, initialEstimate);
            Values currentEstimate = p_isam->calculateBestEstimate();


            int current_dof = 0;
            double current_chi2 = chi2_red(graph, currentEstimate, current_dof);
            LOG(INFO)<<"[Optimizer INFO] Current dof and chi2:"<<current_dof<<","<<current_chi2<<endl;
            LOG(INFO)<<"current yaw_init_to_slam:"<<yaw_init_to_slam*180/3.14159<<" deg."<<endl;
            cout <<"last state:"<<endl;
            const Pose2* p_obj = &(currentEstimate.at(Symbol('x',slam_vertex_index-1)).cast<Pose2>());
            //dynamic_cast<Pose2*>( &(currentEstimate.at(slam_vertex_index-1)) );//这几个值都不对,应该是内存错误.
            //(Pose2*) (&p_isam->calculateEstimate(slam_vertex_index-1));
            //dynamic_cast<Pose2> (currentEstimate.at(slam_vertex_index-1));
            LOG(INFO)<<"Current NODE ESTIMATED STATE at index:"<<slam_vertex_index-1<< " x:"<<p_obj->x()<<",y:"<<p_obj->y()<<",theta:"<<p_obj->theta()<<endl;
            LOG(INFO)<<"Current NODE ESTIMATED Position for visualizer:"<<p_obj->x()<<","<<p_obj->y()<<","<<p_obj->theta()*10<<endl;

            //currentEstimate.print("Current estimate: ");

            LOG(INFO)<<"Online Mode!"<<endl;

            // fetch values from optimizer
            {
                state_mutex.lock();
                LOG(WARNING)<<"Changing current_status output."<<endl;
                //    bool state_correct = false;
                //    Quaterniond ret_val_R;
                //    Vector3d ret_val_t;
                //    std_msgs::Header header_;
                //    int innerID_of_GOG;
                this->current_status.state_correct = true;//TODO.
                auto Q__ = SLAM_msg.pose.orientation;

                // only affect the output message;
                // will not change anything in optimization graph itself.
                const Pose2 current_pose2d = currentEstimate.at(Symbol('x',slam_vertex_index-1)).cast<Pose2>();
                double new_yaw_rad = current_pose2d.theta();
                this->yaw_rad_current_estimate = new_yaw_rad;
                double newx, newy, newz, neww;
                //LOG(WARNING)<<"CHANGING newxyzw for DEBUG QUAT ONLY!!!"<<endl;
                //newx = Q__.x;newy=Q__.y;newz=Q__.z;neww=Q__.w;//Debug.
                //尝试如果不改变xyzw,是否坐标系仍然不正常.
                getNewQuaternionFromOriginalQuaternionAndNewYawAngle(Q__.x,Q__.y,Q__.z,Q__.w,new_yaw_rad,
                                                                     newx,newy,newz,neww);

                current_status.ret_val_R.x() = newx;
                current_status.ret_val_R.y() = newy;
                current_status.ret_val_R.z() = newz;
                current_status.ret_val_R.w() = neww;

                current_status.ret_val_t[0] = current_pose2d.x();
                current_status.ret_val_t[1] = current_pose2d.y();
                Point2 current_height = currentEstimate.at(Symbol('h',slam_vertex_index-1)).cast<Point2>();
                current_status.ret_val_t[2] = current_height.x();//y没用.
                current_status.header_ = SLAM_msg.header;
                current_status.innerID_of_GOG = this->slam_vertex_index-1;

                //TODO:dump current_status to a log file.
                LOG(INFO)<<"Current_status output changed!"<<endl;

                state_mutex.unlock();
            }

            /*if(slam_vertex_index%1000 == 0)
            {
                GaussNewtonParams parameters;
                // Stop iterating once the change in error between steps is less than this value
                parameters.relativeErrorTol = 1e-5;
                // Do not perform more than N iteration steps
                parameters.maxIterations = 10000;
                // Create the optimizer ...
                GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
                Values result= optimizer.optimize();
            }*/


            graph.resize(0);
            initialEstimate.clear();
            cout<<"-------- ---------------------------------- --------"<<endl;
        }
        else
        {
            const int optimize_count = 1800;
            if(slam_vertex_index==optimize_count)
            {
                LOG(INFO)<<"In offline mode:index == "<<optimize_count<<".start optimization."<<endl;
                GaussNewtonParams parameters;
                // Stop iterating once the change in error between steps is less than this value
                parameters.relativeErrorTol = 1e-5;
                // Do not perform more than N iteration steps
                parameters.maxIterations = 1000;
                // Create the optimizer ...
                //p_offline_optimizer = new GaussNewtonOptimizer(graph, initialEstimate, parameters);
                GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
                Values offline_result = optimizer.optimize();
                for(int i = 0;i<optimize_count;i++)
                {
                    Pose2 var_x = offline_result.at(Symbol('x',i)).cast<Pose2>();
                    Point2 var_h = offline_result.at(Symbol('h',i)).cast<Point2>();
                    LOG(INFO)<<"    [OFFLINE OPTIMIZED RESULT] node_id:"<<i<<";xyz:"<<var_x.x()<<","<<var_x.y()<<","<<var_h.x()<<";yaw:"<<var_x.theta()*180/3.1415926535<<" deg."<<endl;
                }
//                offline_result.print("Final Result:\n");
//                stringstream ss;
//                ss<<"Offline_"<<optimize_count<<".dot";
//                string path;
//                ss>>path;
//                ofstream os(path.c_str());
//                graph.saveGraph(os);
//                //导出g2o图文件.
//                stringstream ss_g2o;
//                ss_g2o<<"Offline_"<<optimize_count<<".g2o";
//                string g2o_path;
//                ss_g2o>>g2o_path;
//                writeG2o(graph,offline_result,g2o_path.c_str());
            }
        }
    }
    else
        //SLAM输入初始化。
        //设置初始朝向角为0度，位置为0,0.以后的位姿以此为参考。
    {
        initialEstimate.insert(Symbol('x',slam_vertex_index),Pose2(//initial guess of abs pos and yaw
                SLAM_msg.pose.position.x,SLAM_msg.pose.position.y,0
        )); //here we use basic_vertex_id to represent vehicle position vertex id
        initialEstimate.insert(Symbol('h',slam_vertex_index),Point2(0,0));//initial guess of height:0.//TODO.
        LOG(INFO)<<"Initializing slam factor."<<endl;
        //noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3,0.3,0.01744*5));//初始化角度方差5度. 0.3m.
        noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.00001,0.00001,0.01744*0.00001)); //0.1m,10度.
        graph.emplace_shared<PriorFactor<Pose2> >(Symbol('x',0),Pose2(0,0,0),priorNoise_Absolute);//位置和朝向角都初始化成0.
        //noiseModel::Diagonal::shared_ptr priorNoise_Height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0,0.0));
        noiseModel::Diagonal::shared_ptr priorNoise_Height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.00001,0.0));
        graph.emplace_shared<PriorFactor<Point2> >(Symbol('h',0),Point2(0,0),priorNoise_Height);//高度初始化成0

        noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(1,0);
        Rot2 rot_drift_prior = Rot2::fromAngle(0);
        //initialEstimate.insert(Symbol('y',0),...)//TODO.
        //graph.emplace_shared<PriorFactor<Rot2> >(Symbol('y',0),rot_drift_prior,priorNoise_Height);//定义初始SLAM yaw漂移角为0.
        double current_yaw_slam = get_yaw_from_slam_msg(SLAM_msg);

        slam_vertex_index++;
    }
    //if gps-like measurement inputs:  (pay attention:here we use Point2 as input.)
    //noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
    //graph.add(GPSPose2Factor(Symbol('x', 1), Point2(0, 0), gpsModel)); //add observation
}

void GlobalOptimizationGraph::addBlockQRCode()
{
    //pEdgeQRCode =
}
void addBlockSceneRetriever_StrongCoupling(); //Solve se(3) from multiple points PRXYZ;
void addBlockSceneRetriever_WeakCoupling();//just do square dist calc.

//void GlobalOptimizationGraph::addBlockSceneRetriever()
void GlobalOptimizationGraph::addBlockLoop(const LoopMessage& msg)
{/*
    Quaterniond quat_;
    quat_.x() = msg.x;quat_.y() = msg.y;quat_.z() = msg.z;quat_.w() = msg.w;
    //here we solve yaw first, then calc yaw diff again.
    //caution:no yaw info inside loop_msg,so maybe we should change the logic of ros_global_optimization.cpp in scene_retriever.

    //
    auto Mat1 = this->pSLAM_Buffer->at(msg.prev_gog_frame_id).orientation.toRotationMatrix....;//todo:syntax....
    auto Mat2 = Mat1* quat_.toRotationMatrix();
    double yaw2 = get_yaw_from_Mat(Mat2);//estimation of fixed yaw.//todo:get_yaw_from_Mat...

    //TODO:we could check pitch and roll of Mat2, and judge if this loop is correct.if pitch and roll of mat2 is too far from get_rpy_from_slam_msg(this->pSLAM_Buffer->at(msg.loop_gog_frame_id)), this loop may be a false-positive.
    double yaw1 = get_yaw_from_slam_msg(this->pSLAM_Buffer->at(msg.prev_gog_frame_id));

    double diff_yaw = yaw2 - yaw1;
    graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x',msg.prev_gog_frame_id),Symbol('x',msg.loop_gog_frame_id),Pose2(msg.x,msg.y,diff_yaw),noise_model_loop_movement_xy);
    graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',msg.prev_gog_frame_id),Symbol('h',msg.loop_gog_frame_id),Point2(msg.z,0),noise_model_loop_movement_height);
    LOG(INFO)<<"Added Block Loop in optimization graph."<<endl;*/
}

bool GlobalOptimizationGraph::tryInitVelocity()
{
    cout<<"Nothing to do now!!!"<<endl;
    return false;//TODO :fill in this.
}
bool GlobalOptimizationGraph::checkAHRSValid()
{
    cout<<"Nothing to do now!!!"<<endl;
    return false;//TODO: fill in this.
}

//----------------------
void GlobalOptimizationGraph::DebugPrintWholeGraphValue()
{
    //Values result = ()//DoglegOptimizer(graph, initialEstimate).optimize();
    Values result = LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();
    //result.print("Final results:\n");
    for(Values::iterator key_value = result.begin(); key_value != result.end(); ++key_value)
    {
        Key key = key_value->key;
        Symbol asSymbol(key);
        cout << asSymbol.chr() << std::endl;
        cout << asSymbol.index() << std::endl;
        if(asSymbol.chr() == 'x') // xOy position.
        {
            Pose2 p2 = key_value->value.cast<Pose2>();
            p2.print("xOy pt : ");
        } else if(asSymbol.chr() == 'h') //height.
        {
            Point2 h_ = key_value->value.cast<Point2>();
            h_.print("height vec : ");
        }
    }
    //if a 'vertex' inputs:
    //Values initialEstimate; initialEstimate.insert(1,Pose2( abs pos and yaw));
    //if a 'relative pos' inputs:
    //graph.emplace_shared<BetweenFactor<Pose2> >(1,2,Pose2( diff_x,diff_y,diff_yaw),model_relative_movement);
    //if gps-like measurement inputs:  (pay attention:here we use Point2 as input.)
    //noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
    //graph.add(GPSPose2Factor(Symbol('x', 1), Point2(0, 0), gpsModel));

    //when doing optimization:
    //  GaussNewtonParams parameters;
    //  GaussNewtonOptimizer optimizer(graph, initials, parameters);
    //  Values results = optimizer.optimize(); //get result.
    //  visualize loss function:(like 'chi2()')
    //  cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;

}

bool GlobalOptimizationGraph::doOptimization()
{
    //bool retval = false;//if optimize success(edge >=3).
    //GaussNewtonParams parameters;
    //parameters.relativeErrorTol = 1e-5;// Stop iterating once the change in error between steps is less than this value
    //parameters.maxIterations = 10000;// Do not perform more than N iteration steps
    //GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);// Create the optimizer ...
    //Values result = optimizer.optimize();// ... and optimize
    LOG(WARNING)<<"Nothing happens in GlobalOptimizationGraph::doOptimization()!"<<endl;
    return true;
}
void GlobalOptimizationGraph::addBlockVelocity(int msg_index)//(const geometry_msgs::TwistStamped& velocity_msg)
{

/*
    LOG(INFO)<<"In addBlockVelocity():adding gps vel factor"<<endl;
    auto velocity_msg = this->pVelocity_Buffer->at(msg_index);
    if(slam_vertex_index == 0)
    {
        LOG(INFO)<<"slam not ready."<<endl;
        return;
    }
    if(last_gps_vel_index == -1)
    {
        last_gps_vel_index = slam_vertex_index-1;
        LOG(INFO) << "Initialized slam-vel msg at slam msg index"<<slam_vertex_index-1<<"!"<<endl;
    }
    else
    {
        //计算差.
        double vx = velocity_msg.twist.linear.x;
        double vy = velocity_msg.twist.linear.y;
        double dt = pSLAM_Buffer->at(slam_vertex_index-1).header.stamp.toSec()- pSLAM_Buffer->at(last_gps_vel_index).header.stamp.toSec();
        double dx_vel = vx*dt;
        double dy_vel = vy*dt;
        double dx_slam = this->pSLAM_Buffer->at(slam_vertex_index-1).pose.position.x - this->pSLAM_Buffer->at(last_gps_vel_index).pose.position.x;
        double dy_slam = this->pSLAM_Buffer->at(slam_vertex_index-1).pose.position.y - this->pSLAM_Buffer->at(last_gps_vel_index).pose.position.y;
        //计算yaw.两种方法：几何法，最小二乘重投影。
        check_and_fix_dx(dx_slam); //避免出现极小。
        check_and_fix_dx(dx_vel);
        double theta_slam = atan2(dy_slam,dx_slam);
        double theta_gps = atan2(dy_vel,dx_vel);
        double theta_slam_var = calc_angle_variance(dx_slam,dy_slam,dx_slam_var,dy_slam_var);//dx_slam_var,dy_slam_var填入slam pos差分的标准差.
        double theta_gps_var = calc_angle_variance(dx_vel,dy_vel,vel_x_var,vel_y_var);

        //先计算theta_slam_to_gps;再变换到theta_init_to_gps;
        LOG(INFO)<<"In addBlockVelocity() we do update yaw: theta_gps:"<<theta_gps*180/3.14159<<"deg;theta_slam:"<<theta_slam*180/3.14159<<"deg."<<endl;
        LOG(INFO)<<"dx_vel = "<<dx_vel<<",dy_vel = "<<dy_vel<<";dx_slam = "<<dx_slam<<",dy_slam = "<<dy_slam<<endl<<endl;
        LOG(INFO)<<"original vx: "<<vx<<",vy: "<<vy<<";"<<"dt: "<<dt<<"sec."<<"current index:"<<slam_vertex_index-1<<"prev_index:"<<last_gps_vel_index<<endl;
        double theta_slam_to_gps = theta_gps - theta_slam;
        double yaw_init_to_gps = yaw_init_to_slam - theta_slam_to_gps;
        fix_angle(yaw_init_to_gps);

        LOG(INFO) << "theta slam_to_gps:"<<theta_slam_to_gps<<";yaw init to gps:"<<yaw_init_to_gps*180/3.14159<<" deg."<<endl;
        //sin(yaw_init_to_gps) cos(yaw_init_to_gps)
        double diff_x = cos(-1*yaw_init_to_gps)*dx_vel - sin(-1*yaw_init_to_gps)*dy_vel;
        double diff_y = sin(-1*yaw_init_to_gps)*dy_vel + cos(-1*yaw_init_to_gps)*dx_vel;
        LOG(INFO) << "dx_slam:"<<dx_slam<<",dy_slam:"<<dy_slam<<",dx_vel:"<<dx_vel<<",dy_vel:"<<dy_vel<<"."<<endl;
        if(calcnorm(dx_slam,dy_slam) > 0.1 && calcnorm(dx_vel,dy_vel)>0.1) //认为已经观测到有效的yaw.
        {
            LOG(INFO) <<"speed matching valid! slam_vertex_index-1: "<<slam_vertex_index-1<<endl;
            //diff_yaw = yaw_init_to_gps - last_yaw_init_to_gps; // last_yaw_init_to_gps //这个应该不产生yaw的观测，因为机身的转动不可观测。
            cout <<"insert vel at "<<slam_vertex_index-1<<"."<<endl;
            cout <<"slam buffer size():"<<pSLAM_Buffer->size()<<endl;
            noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2,0.2,0.1));//for slam diff rotation and translation.
            graph.emplace_shared<BetweenFactor<Pose2> >(last_gps_vel_index,slam_vertex_index-1,Pose2(diff_x,diff_y,theta_slam_to_gps - yaw_init_to_gps),model_relative_movement);
        }
        last_gps_vel_index = slam_vertex_index-1;
    }
*/
}



/*
bool GlobalOptimizationGraph::SpeedInitialization() {
        // 速度 初始化 需要由后端提供关键帧（不需要地图）
        std::deque<shared_ptr<Frame>> vpKFs = mpBackEnd->GetAllKF();
        int N = vpKFs.size();
        if (N < setting::minInitKFs)    // 初始化需要若干个关键帧
            return false;

        // Note 1.
        // Input : N (N>=4) KeyFrame/Frame poses of stereo vslam
        //         Assume IMU bias are identical for N KeyFrame/Frame
        // Compute :
        //          bg: gyroscope bias
        //          ba: accelerometer bias
        //          gv: gravity in vslam frame
        //          Vwbi: velocity of N KeyFrame/Frame
        // (Absolute scale is available for stereo vslam)

        // Note 2.
        // Convention by wangjing:
        // W: world coordinate frame (z-axis aligned with gravity, gW=[0;0;~9.8])
        // B: body coordinate frame (IMU)
        // V: camera frame of first camera (vslam's coordinate frame)
        // TWB/T : 6dof pose of frame, TWB = [RWB, PWB; 0, 1], XW = RWB*XW + PWB
        // RWB/R : 3dof rotation of frame's body(IMU)
        // PWB/P : 3dof translation of frame's body(IMU) in world
        // XW/XB : 3dof point coordinate in world/body

        // Step0. get all keyframes in map
        //        reset v/bg/ba to 0
        //        re-compute pre-integration

        Vector3d v3zero = Vector3d::Zero();
        for (auto pKF: vpKFs) {
            pKF->SetBiasG(v3zero);
            pKF->SetBiasA(v3zero);
        }
        for (int i = 1; i < N; i++) {
            vpKFs[i]->ComputeIMUPreInt();
        }

        // Step1. gyroscope bias estimation
        //        update bg and re-compute pre-integration
        // 第一步，估计陀螺偏置
        Vector3d bgest = IMUInitEstBg(vpKFs);
        // 重新计算预积分器
        for (auto pKF: vpKFs) {
            pKF->SetBiasG(bgest);
        }
        for (int i = 1; i < N; i++) {
            vpKFs[i]->ComputeIMUPreInt();
        }

        // Step2. accelerometer bias and gravity estimation (gv = Rvw*gw)
        // let's first assume ba is given by prior and solve the gw
        // Step 2.1 gravity estimation

        // Solve C*x=D for x=[gw] (3+3)x1 vector
        // \see section IV in "Visual Inertial Monocular SLAM with Map Reuse"
        Vector3d baPrior = setting::biasAccePrior;

        MatrixXd C(3 * (N - 2), 3);
        C.setZero();

        VectorXd D(3 * (N - 2));
        D.setZero();

        Matrix3d I3 = Matrix3d::Identity();
        for (int i = 0; i < N - 2; i++) {

            // 三个帧才能建立加速度约束
            shared_ptr<Frame> pKF1 = vpKFs[i];
            shared_ptr<Frame> pKF2 = vpKFs[i + 1];
            shared_ptr<Frame> pKF3 = vpKFs[i + 2];

            // Poses
            Matrix3d R1 = pKF1->mRwb.matrix();
            Matrix3d R2 = pKF2->mRwb.matrix();
            Vector3d p1 = pKF1->mTwb;
            Vector3d p2 = pKF2->mTwb;
            Vector3d p3 = pKF3->mTwb;

            // Delta time between frames
            double dt12 = pKF2->mIMUPreInt.getDeltaTime();
            double dt23 = pKF3->mIMUPreInt.getDeltaTime();
            // Pre-integrated measurements
            Vector3d dp12 = pKF2->mIMUPreInt.getDeltaP();
            Vector3d dv12 = pKF2->mIMUPreInt.getDeltaV();
            Vector3d dp23 = pKF3->mIMUPreInt.getDeltaP();

            Matrix3d Jpba12 = pKF2->mIMUPreInt.getJPBiasa();
            Matrix3d Jvba12 = pKF2->mIMUPreInt.getJVBiasa();
            Matrix3d Jpba23 = pKF3->mIMUPreInt.getJPBiasa();

            // 谜之计算
            Matrix3d lambda = 0.5 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) * I3;
            Vector3d phi = R2 * Jpba23 * baPrior * dt12 -
                           R1 * Jpba12 * baPrior * dt23 +
                           R1 * Jvba12 * baPrior * dt12 * dt23;
            Vector3d gamma = p3 * dt12 + p1 * dt23 + R1 * dp12 * dt23 - p2 * (dt12 + dt23)
                             - R2 * dp23 * dt12 - R1 * dv12 * dt12 * dt23;

            C.block<3, 3>(3 * i, 0) = lambda;
            D.segment<3>(3 * i) = gamma - phi;

        }

        // Use svd to compute C*x=D, x=[ba] 6x1 vector
        // Solve Ax = b where x is ba
        JacobiSVD<MatrixXd> svd2(C, ComputeThinU | ComputeThinV);
        VectorXd y = svd2.solve(D);
        Vector3d gpre = y.head(3);
        // normalize g
        Vector3d g0 = gpre / gpre.norm() * setting::gravity;

        // Step2.2
        // estimate the bias from g
        MatrixXd A(3 * (N - 2), 3);
        A.setZero();
        VectorXd B(3 * (N - 2));
        B.setZero();

        for (int i = 0; i < N - 2; i++) {

            // 三个帧才能建立加速度约束
            shared_ptr<Frame> pKF1 = vpKFs[i];
            shared_ptr<Frame> pKF2 = vpKFs[i + 1];
            shared_ptr<Frame> pKF3 = vpKFs[i + 2];

            // Poses
            Matrix3d R1 = pKF1->mRwb.matrix();
            Matrix3d R2 = pKF2->mRwb.matrix();
            Vector3d p1 = pKF1->mTwb;
            Vector3d p2 = pKF2->mTwb;
            Vector3d p3 = pKF3->mTwb;

            // Delta time between frames
            double dt12 = pKF2->mIMUPreInt.getDeltaTime();
            double dt23 = pKF3->mIMUPreInt.getDeltaTime();
            // Pre-integrated measurements
            Vector3d dp12 = pKF2->mIMUPreInt.getDeltaP();
            Vector3d dv12 = pKF2->mIMUPreInt.getDeltaV();
            Vector3d dp23 = pKF3->mIMUPreInt.getDeltaP();

            Matrix3d Jpba12 = pKF2->mIMUPreInt.getJPBiasa();
            Matrix3d Jvba12 = pKF2->mIMUPreInt.getJVBiasa();
            Matrix3d Jpba23 = pKF3->mIMUPreInt.getJPBiasa();

            // 谜之计算
            Vector3d lambda = 0.5 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) * I3 * g0;
            Matrix3d phi = R2 * Jpba23 * dt12 -
                           R1 * Jpba12 * dt23 +
                           R1 * Jvba12 * dt12 * dt23;
            Vector3d gamma = p3 * dt12 + p1 * dt23 + R1 * dp12 * dt23 - p2 * (dt12 + dt23)
                             - R2 * dp23 * dt12 - R1 * dv12 * dt12 * dt23;

            A.block<3, 3>(3 * i, 0) = phi;
            B.segment<3>(3 * i) = gamma - lambda;
        }

        JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
        VectorXd y2 = svd.solve(B);
        Vector3d baest = y2;

        // update ba and re-compute pre-integration
        for (auto pkf : vpKFs) {
            pkf->SetBiasA(baest);
        }
        for (int i = 1; i < N; i++) {
            vpKFs[i]->ComputeIMUPreInt();
        }

        // Step3. velocity estimation
        for (int i = 0; i < N; i++) {
            auto pKF = vpKFs[i];
            if (i != N - 1) {
                // not last KeyFrame, R1*dp12 = p2 - p1 -v1*dt12 - 0.5*gw*dt12*dt12
                //  ==>> v1 = 1/dt12 * (p2 - p1 - 0.5*gw*dt12*dt12 - R1*dp12)

                auto pKF2 = vpKFs[i + 1];
                const Vector3d p2 = pKF2->mTwb;
                const Vector3d p1 = pKF->mTwb;
                const Matrix3d R1 = pKF->mRwb.matrix();
                const double dt12 = pKF2->mIMUPreInt.getDeltaTime();
                const Vector3d dp12 = pKF2->mIMUPreInt.getDeltaP();

                Vector3d v1 = (p2 - p1 - 0.5 * g0 * dt12 * dt12 - R1 * dp12) / dt12;
                pKF->SetSpeed(v1);
            } else {
                // last KeyFrame, R0*dv01 = v1 - v0 - gw*dt01 ==>> v1 = v0 + gw*dt01 + R0*dv01
                auto pKF0 = vpKFs[i - 1];
                const Matrix3d R0 = pKF0->mRwb.matrix();
                const Vector3d v0 = pKF0->mSpeedAndBias.segment<3>(0);
                const double dt01 = pKF->mIMUPreInt.getDeltaTime();
                const Vector3d dv01 = pKF->mIMUPreInt.getDeltaV();

                Vector3d v1 = v0 + g0 * dt01 + R0 * dv01;
                pKF->SetSpeed(v1);
            }
        }

        double gprenorm = gpre.norm();
        // double baestdif = (baest0 - baest).norm();

        LOG(INFO) << "Estimated gravity before: " << gpre.transpose() << ", |gw| = " << gprenorm << endl;
        LOG(INFO) << "Estimated acc bias after: " << baest.transpose() << endl;
        LOG(INFO) << "Estimated gyr bias: " << bgest.transpose() << endl;

        bool initflag = false;
        if (gprenorm > 9.7 && gprenorm < 9.9 && 
            baest.norm() < 1) {
            LOG(INFO) << "IMU init ok!" << endl;
            initflag = true;
        } else {
            // goodcnt = 0;
        }

        // align 'world frame' to gravity vector, making mgWorld = [0,0,9.8]
        if (initflag) {
            mgWorld = g0;
        }
        return initflag;
    }
*/

 /*   
bool GlobalOptimizationGraph::inputGPS(const sensor_msgs::NavSatFix& gps)
{
    if(this->allow_gps_usage==false)
    {
		cout<<"[WARNING] GPS Usage refused in config file."<<endl;
		return false;
    }
    if(this->gps_init_success == false)
    {
        if(this->gps_info_buffer.size()<GPS_INIT_BUFFER_SIZE)
		{
	    	this->gps_info_buffer.push_back(gps);
		}
		else
		{
		    this->gps_init_success = this->init_gps();
		    if(this->gps_init_success == false)
		    {
				this->gps_info_buffer.empty();
		    }
		}
		return this->gps_init_success;
    }
    else
    {
		bool retval = checkGPSValid(gps);
		if (retval)
		{
	    	this->addGPS(gps);
		}
		return retval;
    }
}*/

/*void GlobalOptimizationGraph::addBlockFCAttitude()
{
    //just call addBlockAHRS.
    this->addBlockAHRS();
}*/

/*
void GlobalOptimizationGraph::addBlockAHRS(const nav_msgs::Odometry& AHRS_msg)
{
    pEdgeAHRS = new EdgeAttitude();
    shared_ptr<g2o::BaseEdge> ptr_ahrs(pEdgeAHRS);
    pEdgeAHRS->setId(this->EdgeVec.size());
    this->EdgeVec.push_back(ptr_ahrs);
    pEdgeAHRS->setMeasurement(...);
    pEdgeAHRS->setInformation(...);
    optimizer.addEdge(pEdgeAHRS)
    if(check_avail())
    {
        ;
    }
    else
    {
        pEdgeAHRS->setLevel(1);
    }

}*/
