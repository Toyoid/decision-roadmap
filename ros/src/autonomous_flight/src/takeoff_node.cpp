/*
	FILE: takeoff_node.cpp
	---------------------------
	Simple flight test for autonomous flight
*/

#include <ros/ros.h>
#include <autonomous_flight/flightBase.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh;

    AutoFlight::flightBase fb (nh);  // is nh the parameter for initializing the object fb?
    fb.takeoff();
    ros::spin();
    return 0;
}