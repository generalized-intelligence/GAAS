



void gps_buffer_helper(ROS_IO_Manager* pRIM,CallbackBufferBlock<sensor_msgs::NavSatFix>& nav_buffer,
		const boost::shared_ptr<sensor_msgs::NavSatFix const>& nav_msg)
{
    LOG(INFO)<<"GPS pos info received!"<<endl;
    bool locked = pRIM->gps_vel_mutex.try_lock();
    if(!locked)
    {
        return;
    }
    cout<<"GPS message received!"<<endl;
    nav_msg->header.stamp = ros::Time::now();

    nav_buffer.onCallbackBlock(*nav_msg);
    pRIM->_gps_pos_update = true;
}

void slam_buffer_helper(ROS_IO_Manager* pRIM,CallbackBufferBlock<geometry_msgs::PoseStamped>& slam_buffer,
		const boost::shared_ptr<geometry_msgs::PoseStamped const>& slam_msg)
{
    LOG(INFO)<<"SLAM msg received!"<<endl;
    bool locked = pRIM->slam_buf_mutex.try_lock();
    if(!locked)
    {
        return;
    }
    if (pRIM->getGraph() == nullptr)
    {
        cout<<"waiting for gog init.pass this msg."<<endl;
        return;
    }
    cout<<"SLAM message received!"<<endl;
    bool state = ( pRIM->getGraph()->getStatus() & 1 );
    cout<<"Is running with gps:"<<state<<endl;
    visualization_msgs::Marker slam_marker;
    slam_marker.header.frame_id = "world";
    slam_marker.header.stamp = ros::Time::now();
    slam_marker.type = visualization_msgs::Marker::ARROW;
    slam_marker.action = visualization_msgs::Marker::ADD;
    auto q_ = slam_msg->pose.orientation;
    
    slam_marker.pose.orientation.x = q_.x;
    slam_marker.pose.orientation.y = q_.y;
    slam_marker.pose.orientation.z = q_.z;
    slam_marker.pose.orientation.w = q_.w;
    auto xyz_original = slam_msg->pose.position;
    
    slam_marker.scale.x = 4;
    slam_marker.scale.y = 2;
    slam_marker.scale.z = 1;
    slam_marker.color.r = 0;
    slam_marker.color.g = 0;
    slam_marker.color.b = 1;
    slam_marker.color.a = 1;
    pRIM->publish_slam_marker(slam_marker);
    pRIM->_slam_msg_update = true;
    Quaterniond rot_origin;

    rot_origin.x() = q_.x;
    rot_origin.y() = q_.y;
    rot_origin.z() = q_.z;
    rot_origin.w() = q_.w;

    Vector3d xyz(xyz_original.x,xyz_original.y,xyz_original.z);
    Matrix3d rotation = rot_origin.toRotationMatrix();
    rotation = pRIM->SLAM_ROTATION_EIGEN*rotation;
    xyz = pRIM->SLAM_ROTATION_EIGEN*xyz;

    geometry_msgs::PoseStamped new_msg;
    new_msg = *slam_msg;
    new_msg.pose.position.x = xyz[0];
    new_msg.pose.position.y = xyz[1];
    new_msg.pose.position.z = xyz[2];

    LOG(INFO)<<"Original SLAM xyz:"<<xyz[0]<<","<<xyz[1]<<","<<xyz[2]<<endl;
    Quaterniond rot_2(rotation);
    new_msg.pose.orientation.x = rot_2.x();
    new_msg.pose.orientation.y = rot_2.y();
    new_msg.pose.orientation.z = rot_2.z();
    new_msg.pose.orientation.w = rot_2.w();
    //cout <<pRIM->SLAM_ROTATION_EIGEN<<endl;
    slam_buffer.onCallbackBlock(new_msg);
}

void velocity_buffer_helper(ROS_IO_Manager* pRIM, CallbackBufferBlock<geometry_msgs::TwistStamped>& velocity_buffer,const boost::shared_ptr<geometry_msgs::TwistStamped const>& velocity_msg)
{
    LOG(INFO)<<"Velo msg received!"<<endl<<endl;;
    bool locked = pRIM->gps_vel_mutex.try_lock();
    if(!locked)
    {
        return;
    }

    cout<<"Velocity msgs received!"<<endl;
    velocity_msg->header.stamp = ros::Time::now();
    velocity_buffer.onCallbackBlock(*velocity_msg);
    pRIM->_gps_vel_update = true;
}


void ahrs_buffer_helper(ROS_IO_Manager* pRIM, CallbackBufferBlock<nav_msgs::Odometry>& ahrs_buffer,
		const boost::shared_ptr<nav_msgs::Odometry const>& ahrs_msg)
{
    //cout<<"AHRS message received!"<<endl;
    ahrs_buffer.onCallbackBlock(*ahrs_msg);
    visualization_msgs::Marker ahrs_marker;
    ahrs_marker.header.frame_id = "world";
    ahrs_marker.header.stamp = ros::Time::now();
    ahrs_marker.type = visualization_msgs::Marker::ARROW;
    ahrs_marker.action = visualization_msgs::Marker::ADD;
    auto  q_ = ahrs_msg->pose.pose.orientation;
    ahrs_marker.pose.orientation.x = q_.x;
    ahrs_marker.pose.orientation.y = q_.y;
    ahrs_marker.pose.orientation.z = q_.z;
    ahrs_marker.pose.orientation.w = q_.w;
    
    ahrs_marker.scale.x = 4;
    ahrs_marker.scale.y = 2;
    ahrs_marker.scale.z = 1;
    ahrs_marker.color.r = 0;
    ahrs_marker.color.g = 1;
    ahrs_marker.color.b = 0;
    ahrs_marker.color.a = 1;
    pRIM->publish_ahrs_marker(ahrs_marker);
}



