



void gps_buffer_helper(CallbackBufferBlock<sensor_msgs::NavSatFix>& nav_buffer,
		const boost::shared_ptr<sensor_msgs::NavSatFix const>& nav_msg)
{
    cout<<"GPS message received!"<<endl;
    nav_buffer.onCallbackBlock(*nav_msg);
}

void slam_buffer_helper(CallbackBufferBlock<geometry_msgs::PoseStamped>& slam_buffer,
		const boost::shared_ptr<geometry_msgs::PoseStamped const>& slam_msg)
{
    cout<<"SLAM message received!"<<endl;
    slam_buffer.onCallbackBlock(*slam_msg);
}

void ahrs_buffer_helper(CallbackBufferBlock<nav_msgs::Odometry>& ahrs_buffer,
		const boost::shared_ptr<nav_msgs::Odometry const>& ahrs_msg)
{
    cout<<"AHRS message received!"<<endl;
    ahrs_buffer.onCallbackBlock(*ahrs_msg);
}



