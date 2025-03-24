#include <autonomous_flight/fgDRMManager.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "fg_drm_node");
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
    
    AutoFlight::fgDRMManager manager(nh, robot_ns);
    manager.run();
    
    ros::spin();
    
    return 0;
}