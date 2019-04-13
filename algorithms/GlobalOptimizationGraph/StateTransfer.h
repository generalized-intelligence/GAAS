#ifndef STATE_TRANSFER_H
#define STATE_TRANSFER_H


namespace StateTransfer
{
	const int state_NO_GPS_NO_SCENE = 0;
    const int state_NO_GPS_WITH_SCENE = 1;
    const int state_GPS_NO_SCENE = 2;
    const int state_GPS_WITH_SCENE = 3;

    bool GPS_avail(RIM* pRIM,double& new_lon_out,double& new_lat_out,double& new_alt_out)//check queue.
    {
        ...
    }
    void GPS_reinit(Graph* pGraph,coord& output_matching_coord)
    {
    	//SLAM coordinate will be matched to GPS ENU coordinate.
        auto coord = pGraph->getSLAMCoordinate();//x,y,z
        auto diff_coord = coord()
        double lon,lat,alt;
        if(GPS_avail(pRIM,lon,lat,alt))
        {
        	if(pRIM->hasGPSExpand())
        	{
        		//do optimization,connect edge GPS - NO_GPS - GPS.
        	}

        	pRIM->resetGPSExpand(lon,lat,alt);//重新寻找展开点.

        }
        

    }
}





#endif