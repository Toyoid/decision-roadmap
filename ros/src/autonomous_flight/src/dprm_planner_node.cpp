#include <autonomous_flight/dprmPlanManager.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "dprm_planner_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");  // private node handle for parameters

	std::string robot_ns;
	pnh.param("robot_namespace", robot_ns, std::string(""));
	
	// Runtime check: if robot_ns is empty, log a fatal error and exit.
	if (robot_ns.empty()) {
		ROS_FATAL("Parameter 'robot_namespace' is not set or is empty!");
		ros::shutdown();
		return -1;
	}

	AutoFlight::dprmPlanManager dprm_planner(nh, robot_ns);
	dprm_planner.run();

	ros::spin();

	return 0;
}