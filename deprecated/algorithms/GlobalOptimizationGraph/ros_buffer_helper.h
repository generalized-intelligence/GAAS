



void gps_buffer_helper(CallbackBufferBlock<sensor_msgs::NavSatFix>& nav_buffer,
		const boost::shared_ptr<sensor_msgs::NavSatFix const>& nav_msg)
{
    cout<<"GPS message received!"<<endl;
    nav_buffer.onCallbackBlock(*nav_msg);
}

void slam_buffer_helper(ROS_IO_Manager* pRIM,CallbackBufferBlock<geometry_msgs::PoseStamped>& slam_buffer,
		const boost::shared_ptr<geometry_msgs::PoseStamped const>& slam_msg)
{
    if (pRIM->getGraph() == nullptr)
    {
        cout<<"waiting for gog init.pass this msg."<<endl;
        return;
    }
    cout<<"SLAM message received!"<<endl;
    bool state = ( pRIM->getGraph()->getStatus() & 1 );
    cout<<"Is running with gps:"<<state<<endl;
    slam_buffer.onCallbackBlock(*slam_msg);
    
    
    if(state)//running with gps.SLAM work as edge PRV.
    {
        (pRIM->getGraph())->addSLAM_edgeprv(*slam_msg);
    }
    //TODO:add 
    else
    {
        ;//(pRIM->getGraph())->addBlockSLAM(*slam_msg);
    }
    
}

void ahrs_buffer_helper(CallbackBufferBlock<nav_msgs::Odometry>& ahrs_buffer,
		const boost::shared_ptr<nav_msgs::Odometry const>& ahrs_msg)
{
    cout<<"AHRS message received!"<<endl;
    ahrs_buffer.onCallbackBlock(*ahrs_msg);
}



