#include "controller.h"


Controller::Controller(ros::NodeHandle& nh)
{
	mNH = nh;
	mMavrosSub = mNH.subscribe("/mavros/local_position/pose", 100, &Controller::MavrosPoseCallback, this);
	mTargetSetSub = mNH.subscribe("/move_base_simple/goal", 100, &Controller::TargetSetSubCallback, this);

	mPositionControlPub = mNH.advertise<geometry_msgs::PoseStamped>("gi/set_pose/position", 100);
	mYawControlPub = mNH.advertise<std_msgs::Float32>("gi/set_pose/orientation", 100);

	mSceneRetrievedPosition = cv::Mat::zeros(cv::Size(4, 4), CV_64FC1);

	mSTATE = NO_SCENE_RETRIEVED_BEFORE;

	this->PosePublisher = this->nh.advertise<visualization_msgs::Marker>("/updated_target",10);

	mpWorld = make_shared<World>("./config/scene_retrieve.yaml");

    cv::FileStorage fsSettings("./config/scene_retrieve.yaml", cv::FileStorage::READ);
    mOutlierThreshold = fsSettings["Outlier_Threshold"];
    mTargetDistanceThres = fsSettings["Target_Threshold"];

    LOG(INFO)<<"mOutlierThreshold: "<<mOutlierThreshold<<endl;

    std::thread t(&Controller::Run, this);

    t.detach();

    testfile.open("all.txt", fstream::out | fstream::app);
}

void Controller::Run()
{
    // test
//    geometry_msgs::PoseStamped target;
//    target.pose.position.x = 0;
//    target.pose.position.y = -10;
//    target.pose.position.z = 3;
//    SetTarget(target);

	ros::Rate rate(5.0);
	while (!ros::isShuttingDown())
	{
		//constantly fetching information from MAVROS
//        if(mSTATE == MAVROS_STATE_ERROR)
//        {
//            Hover();
//        }

		if(mTARGET == NO_TARGET)
		{
			continue;
		}
		else if (mTARGET == NEW_TARGET && mReceivedPose)
		{
			int building_idx = -1;
			bool result = mpWorld->isInBuilding(mTargetPose, building_idx);

            vector<geometry_msgs::PoseStamped> wps = mpWorld->FindWayPoints(mCurMavrosPose, mTargetPose);

            if(wps.empty())
            {
                LOG(INFO)<<"Cannot find way, wps empty! Continue!"<<endl;
                continue;
            }

            // NOTE fix all way point at height 3 meters
            for(auto& wp : wps)
            {
                wp.pose.position.z = 3;
            }

            GoByWayPoints(wps);

			// disable movement after finishing current task mission.
			mTARGET = NO_TARGET;

		}

		rate.sleep();
	}

	Hover();
}

void Controller::SetTarget(geometry_msgs::PoseStamped& target)
{
	mTargetPose = target;
    mInitialTarget = target;

	mTARGET = NEW_TARGET;
}

void Controller::GoByWayPoints(vector<geometry_msgs::PoseStamped>& way_points, float distance_threshold)
{
	if(way_points.empty())
		LOG(INFO)<<"Way points empty!"<<endl;

	int i=1;
	for(auto& wp : way_points)
	{
		LOG(WARNING)<<"Current waypoint: "<<i<<"/"<<way_points.size()<<", "<<
		                                           wp.pose.position.x<<", "<<
		                                           wp.pose.position.y<<", "<<
		                                           3<<endl;
        i++;
        while(distanceToTarget(wp) > distance_threshold)
        {
            LOG(WARNING)<<"DistanceToTarget(wp): "<<distanceToTarget(wp)<<endl;
            GoToTarget(wp);
        }
	}
    LOG(INFO)<<"Finished All WayPoints!"<<endl;
}

bool Controller::GoToTarget(const geometry_msgs::PoseStamped& target, bool useWorldFrame)
{
	publishPose(target);

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();

	//ROS uses ENU internally, so we will stick to this convention

	//NOTE the callback from RVIZ is in FLU frame, to convert it to ENU frame we have to
	//apply a custom transformation
	if (!useWorldFrame)
		pose.header.frame_id = "base_link";
	else {
		pose.header.frame_id = "map";

		pose.pose.position.x =  - target.pose.position.y;
		pose.pose.position.y = target.pose.position.x;
		pose.pose.position.z = target.pose.position.z;
	}
	mPositionControlPub.publish(pose);
}

void Controller::AddRetrievedPose(cv::Mat& retrieved_pose, cv::Mat& mavros_pose)
{
    // this shouldn't happen
	if (retrieved_pose.empty() || mavros_pose.empty())
		return;

    LOG(INFO)<<"retrieved_pose: \n"<<retrieved_pose<<endl;
    LOG(INFO)<<"mavros_pose: \n"<<mavros_pose<<endl;

	// outlier determination
	float relative_distance = PoseDistance(retrieved_pose, mavros_pose);

	float deviation_factor = -1;

	// if no previous measurements found
	if (mSceneMavrosDistanceDeque.size() < 3)
	{
        mSceneMavrosDistanceDeque.push_back(relative_distance);
        return;
	}

    // if measurement is outlier, return
	if(isOutlier(retrieved_pose, mavros_pose, deviation_factor))
    {
        LOG(INFO)<<"Added Mavros Pose and Retrieved Pose is not Valid!"<<endl;
        return;
    }

    // keep an sliding window of size 3
    if(mSceneMavrosDistanceDeque.size() >= 3)
    {
        mSceneMavrosDistanceDeque.pop_front();
    }
    mSceneMavrosDistanceDeque.push_back(relative_distance);

    isOutlier(retrieved_pose, mavros_pose, deviation_factor);

	LOG(INFO)<<"Added Mavros Pose and Retrieved Pose is Valid!"<<endl;

	// retrieved pose and mavros pose at the moment when trying to retrieve pose
	// you can use debug_scripts/debug.py to visualize mavros pose and retrieved pose
	mSceneRetrievedLastPosition = mSceneRetrievedPosition;
	mSceneRetrievedPosition = retrieved_pose;

	mMavPoseLastRetrieved = mMavPoseCurRetrieved;
	mMavPoseCurRetrieved = MatToPoseStamped(mavros_pose, "map");

	if (abs(mCurMavrosPose.pose.position.x)+abs(mCurMavrosPose.pose.position.y)+abs(mCurMavrosPose.pose.position.z)==0)
	{
		LOG(WARNING)<<"Current Mavros Pose not valid, adding retrieved pose failed!"<<endl;
		return;
	}

	// the first retrieved pose
	// NOTE assuming the first retrieved pose is "right" TODO, find a way to test whether the retrieved pose is right
	// set retrieved last position and retrieved current position
	// using retrieved drone position and current drone position to find a transform between
	// current drone frame and scene frame
	if(mSTATE == NO_SCENE_RETRIEVED_BEFORE)
	{
		mSTATE = SCENE_RETRIEVING_WORKING_NORMAL;
		LOG(INFO)<<"AddRetrievedPose: NO_SCENE_RETRIEVED_BEFORE"<<endl;

		mIdxMavScenes.push_back(make_tuple(mSceneRetrieveIndex, retrieved_pose, mMavPoseCurRetrieved));

		testfile<<mSceneRetrieveIndex<<","
				<<retrieved_pose.at<double>(0,3)<<","
				<<retrieved_pose.at<double>(1,3)<<","
				<<retrieved_pose.at<double>(2,3)<<","
				<<mMavPoseCurRetrieved.pose.position.x<<","
				<<mMavPoseCurRetrieved.pose.position.y<<","
				<<mMavPoseCurRetrieved.pose.position.z<<","
				<<mTargetPose.pose.position.x<<","
				<<mTargetPose.pose.position.y<<","
				<<mTargetPose.pose.position.z<<","
				<<deviation_factor<<endl;

		return;
	}

	// normal working state
	// prev and current retrieved pose were set
	// prev and current mavros pose at the time when scene was retrieved were set
	else if(mSTATE == SCENE_RETRIEVING_WORKING_NORMAL || mSTATE == MAVROS_STATE_ERROR)
	{
		LOG(INFO)<<"AddRetrievedPose: SCENE_RETRIEVING_WORKING_NORMAL"<<endl;

		if(true) //if(isSceneRecoveredMovementValid())
		{
			mIdxMavScenes.push_back(make_tuple(mSceneRetrieveIndex, retrieved_pose, mMavPoseCurRetrieved));

            testfile<<mSceneRetrieveIndex<<","
                    <<retrieved_pose.at<double>(0,3)<<","
                    <<retrieved_pose.at<double>(1,3)<<","
                    <<retrieved_pose.at<double>(2,3)<<","
                    <<mMavPoseCurRetrieved.pose.position.x<<","
                    <<mMavPoseCurRetrieved.pose.position.y<<","
                    <<mMavPoseCurRetrieved.pose.position.z<<","
                    <<mTargetPose.pose.position.x<<","
                    <<mTargetPose.pose.position.y<<","
                    <<mTargetPose.pose.position.z<<","
                    <<deviation_factor<<endl;

			if (mIdxMavScenes.size() > 3)
			{
				mIdxMavScenes.pop_front();
			}

			// if current retrieved pose from the scene is valid, current pose of the drone (frome SLAM)
			// could be updated, we can find a transform from current drone pose to retrieved scene pose
			// rather than updating current drone pose with this transform, we update the current target of
			// the drone with this transform.
			if(mTARGET == NO_TARGET) {
                return;
            }
			else{
                UpdateTarget();
            }
		}
		else
			std::cout<<"Current retrieved pose is not valid"<<std::endl;
	}
	else{
		cout<<"Current state: "<<mSTATE<<endl;
	}

	mSceneRetrieveIndex ++;
}


bool Controller::isSceneRecoveredMovementValid()
{
	float delta_retrieved = abs(mSceneRetrievedPosition.at<double>(0, 3) - mSceneRetrievedLastPosition.at<double>(0, 3)) +
							abs(mSceneRetrievedPosition.at<double>(1, 3) - mSceneRetrievedLastPosition.at<double>(1, 3)) +
							abs(mSceneRetrievedPosition.at<double>(2, 3) - mSceneRetrievedLastPosition.at<double>(2, 3));

	float delta_mavros = abs(mMavPoseCurRetrieved.pose.position.x - mMavPoseLastRetrieved.pose.position.x) +
						 abs(mMavPoseCurRetrieved.pose.position.y - mMavPoseLastRetrieved.pose.position.y) +
						 abs(mMavPoseCurRetrieved.pose.position.z - mMavPoseLastRetrieved.pose.position.z);

	float factor = abs(delta_mavros / delta_retrieved);

	if (factor < 3.0)
		return true;
	else
		return false;
}

void Controller::UpdateTarget()
{

	if (mIdxMavScenes.empty())
		return;

	float angles = 0;
	Eigen::Vector3d axises(0, 0,0 );
	Eigen::Vector3d position(0, 0, 0);
	for(auto& elem : mIdxMavScenes)
	{
	    //tuple<size_t, cv::Mat, geometry_msgs::PoseStamped>
	    cv::Mat scene_pose_mat = std::get<1>(elem);
		geometry_msgs::PoseStamped mav_pose = std::get<2>(elem);
		cv::Mat mav_pose_mat = PoseStampedToMat(mav_pose);

		if(mav_pose_mat.empty() || scene_pose_mat.empty())
			return;

		// for this method, the result is using the relativeTransform's orientation
		cv::Mat RelativeTransform = findRelativeTransformPosition(mav_pose_mat, scene_pose_mat);

		cv::Mat temp_rotation = RelativeTransform.colRange(0, 3).rowRange(0, 3);
		Eigen::Matrix3d rotation_eigen;
		cv::cv2eigen(temp_rotation, rotation_eigen);

		Eigen::AngleAxisd angle_axis(rotation_eigen);
		Eigen::Vector3d axis = angle_axis.axis();
		float angle = angle_axis.angle();

		axises[0] += axis[0];
		axises[1] += axis[1];
		axises[2] += axis[2];

		angles += angle;

		position[0] += RelativeTransform.at<double>(0, 3);
		position[1] += RelativeTransform.at<double>(1, 3);
		position[2] += RelativeTransform.at<double>(2, 3);
	}

	axises[0] = axises[0] / mIdxMavScenes.size();
	axises[1] = axises[1] / mIdxMavScenes.size();
	axises[2] = axises[2] / mIdxMavScenes.size();
	angles = angles / mIdxMavScenes.size();
	position[0] = position[0] / mIdxMavScenes.size();
	position[1] = position[1] / mIdxMavScenes.size();
	position[2] = position[2] / mIdxMavScenes.size();

	Eigen::AngleAxisd averaged_rotation_angleaxis(angles, axises);
	Eigen::Matrix3d averaged_rotation_eigen = averaged_rotation_angleaxis.toRotationMatrix();
	cv::Mat averaged_rotation_mat;
	cv::eigen2cv(averaged_rotation_eigen, averaged_rotation_mat);

	cv::Mat relative_transform = cv::Mat::zeros(4,4, CV_64F);

	relative_transform.at<double>(0, 0) = averaged_rotation_mat.at<double>(0, 0);
	relative_transform.at<double>(0, 1) = averaged_rotation_mat.at<double>(0, 1);
	relative_transform.at<double>(0, 2) = averaged_rotation_mat.at<double>(0, 2);
	relative_transform.at<double>(1, 0) = averaged_rotation_mat.at<double>(1, 0);
	relative_transform.at<double>(1, 1) = averaged_rotation_mat.at<double>(1, 1);
	relative_transform.at<double>(1, 2) = averaged_rotation_mat.at<double>(1, 2);
	relative_transform.at<double>(2, 0) = averaged_rotation_mat.at<double>(2, 0);
	relative_transform.at<double>(2, 1) = averaged_rotation_mat.at<double>(2, 1);
	relative_transform.at<double>(2, 2) = averaged_rotation_mat.at<double>(2, 2);
	relative_transform.at<double>(0, 3) = position[0];
	relative_transform.at<double>(1, 3) = position[1];
	relative_transform.at<double>(2, 3) = position[2];
	relative_transform.at<double>(3, 3) = 1.0;

	if (mSceneRetrieveIndex <= 3)
	{
		//initial transform is mavros to scene
		mInitialRelativeTransform = relative_transform;
		LOG(INFO)<<"mInitialRelativeTransform: \n"<<mInitialRelativeTransform<<endl;
	}
	else{
        mCurrentRelativeTransform = relative_transform;

        // this should not happend ...
        if(mCurrentRelativeTransform.empty() || mInitialRelativeTransform.empty())
            return;

        //current transform is from mavros to scene
        cv::Mat current2initial = findRelativeTransformPosition(mCurrentRelativeTransform, mInitialRelativeTransform);

        cv::Mat initialMat = PoseStampedToMat(mInitialTarget);

        cv::Mat updatedInitialTarget = cv::Mat::eye(4,4, CV_64F);

        // again, they have different rotations because the retrieved slam pose has a different rotation angle.
        updatedInitialTarget.at<double>(0, 3) = initialMat.at<double>(0, 3) - current2initial.at<double>(0, 3);
        updatedInitialTarget.at<double>(1, 3) = initialMat.at<double>(1, 3) - current2initial.at<double>(1, 3);
        updatedInitialTarget.at<double>(2, 3) = initialMat.at<double>(2, 3) - current2initial.at<double>(2, 3);

        geometry_msgs::PoseStamped result_target = MatToPoseStamped(updatedInitialTarget, mInitialTarget.header.frame_id);

        mTargetPose = result_target;

        LOG(INFO)<<"Going to target: "<<mTargetPose.pose.position.x<<", "
								      <<mTargetPose.pose.position.y<<", "
								      <<mTargetPose.pose.position.z<<endl;

        LOG(INFO)<<"current2initial, current pose diff to original pose diff: \n"<<current2initial<<endl;
        LOG(INFO)<<"current2initial, mInitialTarget: \n"<<initialMat<<endl;
        LOG(INFO)<<"current2initial, updatedInitialTarget: \n"<<updatedInitialTarget<<endl;
	}

    mTARGET = NEW_TARGET;
}

void Controller::TargetSetSubCallback(const geometry_msgs::PoseStamped& target)
{
    //mTargetPose, set height at fixed 3 meters
	mTargetPose = target;
	mTargetPose.pose.position.z = 3;

    SetTarget(mTargetPose);

	LOG(WARNING)<<"Received new target: "<<mTargetPose.pose.position.x<<", "<<
										   mTargetPose.pose.position.y<<", "<<
										   mTargetPose.pose.position.z<<endl;
}

void Controller::MavrosPoseCallback(const geometry_msgs::PoseStamped& pose)
{
    //LOG(INFO)<<"MavrosPoseCallback: "<<pose.pose.position.x<<","<<pose.pose.position.y<<","<<pose.pose.position.z<<endl;

	mLastMavrosPose = mCurMavrosPose;
	mCurMavrosPose = pose;

    mReceivedPose = true;

//	if (!isMavrosPoseValid())
//		mSTATE = MAVROS_STATE_ERROR;
}

bool Controller::isMavrosPoseValid()
{

	float delta_t = (mCurMavrosPose.header.stamp - mLastMavrosPose.header.stamp).toSec();

	if(delta_t == 0)
	{
		return true;
	}

	float distance_squared =
			pow(abs(mCurMavrosPose.pose.position.x - mLastMavrosPose.pose.position.x), 2) +
			pow(abs(mCurMavrosPose.pose.position.y - mLastMavrosPose.pose.position.y), 2) +
			pow(abs(mCurMavrosPose.pose.position.z - mLastMavrosPose.pose.position.z), 2);

	float speed = sqrt(distance_squared) / delta_t;

	// the horizontal flight speed of drone is less than 12 by default
	if (speed > 10)
	{
		mSTATE = MAVROS_STATE_ERROR;
		return false;
	}

	float cur_yaw, prev_yaw;

	Eigen::Quaternionf cur_rotation_matrix(mCurMavrosPose.pose.orientation.w,
										   mCurMavrosPose.pose.orientation.x,
										   mCurMavrosPose.pose.orientation.y,
										   mCurMavrosPose.pose.orientation.z);
	Eigen::Vector3f euler_cur = cur_rotation_matrix.toRotationMatrix().eulerAngles(0, 1, 2);
	cur_yaw = euler_cur[2];

	Eigen::Quaternionf prev_rotation_matrix(mLastMavrosPose.pose.orientation.w,
											mLastMavrosPose.pose.orientation.x,
											mLastMavrosPose.pose.orientation.y,
											mLastMavrosPose.pose.orientation.z);
	Eigen::Vector3f euler_prev = prev_rotation_matrix.toRotationMatrix().eulerAngles(0, 1, 2);
	prev_yaw = euler_prev[2];

	float yaw_rate = (cur_yaw - prev_yaw) / delta_t;

	// the yaw change rate is less than pi by default
	if (yaw_rate > 2*3.1415926)
	{
		mSTATE = MAVROS_STATE_ERROR;
		return false;
	}

	return true;
}

void Controller::Hover()
{

}

//cv::Mat Controller::findRelativeTransform(cv::Mat& Twb1, cv::Mat& Twb2)

