Do global optimization of position and attitude.Keep these info inside a structure,so that when SLAM or FlightController Reset its state,still do we have a stable estimation of drone.
Raw data will be input from mavros/ros SLAM node;Compile this with catkin.
Combine visual slam info,external AHRS info,flight controller attitude info,GPS info,scene retriever info and magnet info.
May be useful when dealing with flight sessions.
Still an immature thought,though.





