#ifndef DRM_BASE_H
#define DRM_BASE_H

#include <map_manager/dynamicMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <global_planner/PRMKDTree.h>
#include <global_planner/PRMAstar.h>
#include <global_planner/utils.h>
#include <global_planner/time_measure.h>
#include <global_planner/point_struct.h>
#include <opencv2/opencv.hpp>
#include <decision_roadmap_agent/TargetArray.h>

namespace globalPlanner {

class DRMBase {
protected:
    std::mutex roadmap_mutex_;  // Mutual exclusion lock

    // Common member variables
    std::string ns_;
    std::string hint_;
    std::string robot_ns_;
    std::string single_robot_ns_;
	std::string rostopic_prefix_;

    ros::NodeHandle nh_;
    ros::Publisher roadmapPub_;
    ros::Publisher bestPathPub_;
    ros::Publisher waypointPub_;
    ros::Subscriber odomSub_;
    ros::Timer visTimer_;
    ros::Timer waypointTimer_;
    ros::Publisher runtimePub_;

    nav_msgs::Odometry odom_;
    std::shared_ptr<mapManager::occMap> map_;
    std::shared_ptr<PRM::KDTree> roadmap_;

    // Common parameters
    double vel_;
    double angularVel_;
	std::string odomTopic_;
	Eigen::Vector3d globalRegionMin_, globalRegionMax_;
	double safeDistXY_;
	double safeDistZ_;
	bool safeDistCheckUnknown_;
	double horizontalFOV_;
	double verticalFOV_;
	double dmin_;
	double dmax_;
	int nnNum_;
	double distThresh_;
	double maxConnectDist_;
	std::vector<double> yaws_;
	double minVoxelThresh_;

    // Common state variables
    bool odomReceived_;
    bool currGoalReceived_;
    bool resettingRLEnv_;
    Eigen::Vector3d position_;
    std::shared_ptr<PRM::Node> currGoal_;
    std::vector<std::shared_ptr<PRM::Node>> bestPath_;
    unsigned int waypointIdx_ = 0;  //added by me
    Point3D navWaypoint_;  //added by me
    Point3D navHeading_;  //added by me
    double currYaw_;
    std::deque<Eigen::Vector3d> histTraj_; // historic trajectory for information gain update 
	std::vector<std::pair<Eigen::Vector3d, double>> frontierPointPairs_;
    
    // Time measurement
    TimeMeasure measureTimer_;
    std_msgs::Float32 runtime_;

public:
    DRMBase(const ros::NodeHandle& nh, const std::string& robot_ns);
    virtual ~DRMBase() = default;

    // Common interface
    void setMap(const std::shared_ptr<mapManager::occMap>& map);
	void loadVelocity(double vel, double angularVel);
	nav_msgs::Path getBestPath();
    
	// helper functions
	bool isPosValid(const Eigen::Vector3d& p);
	bool isPosValid(const Eigen::Vector3d& p, double safeDistXY, double safeDistZ);
	bool isNodeRequireUpdate(std::shared_ptr<PRM::Node> n, std::vector<std::shared_ptr<PRM::Node>> path, double& leastDistance);
	std::shared_ptr<PRM::Node> randomConfigBBox(const Eigen::Vector3d& minRegion, const Eigen::Vector3d& maxRegion);
	bool sensorRangeCondition(const shared_ptr<PRM::Node>& n1, const shared_ptr<PRM::Node>& n2);
	bool sensorFOVCondition(const Eigen::Vector3d& sample, const Eigen::Vector3d& pos);
	int calculateUnknown(const shared_ptr<PRM::Node>& n, std::unordered_map<double, int>& yawNumVoxels);
	double calculatePathLength(const std::vector<shared_ptr<PRM::Node>>& path);
	void shortcutPath(const std::vector<std::shared_ptr<PRM::Node>>& path, std::vector<std::shared_ptr<PRM::Node>>& pathSc);
	int weightedSample(const std::vector<double>& weights);
	std::shared_ptr<PRM::Node> sampleFrontierPoint(const std::vector<double>& sampleWeights);
	std::shared_ptr<PRM::Node> extendNode(const std::shared_ptr<PRM::Node>& n, const std::shared_ptr<PRM::Node>& target);
	Point3D projectNavWaypoint(const Point3D& nav_waypoint, const Point3D& last_waypoint);  
};

} // namespace globalPlanner

#endif // DRM_BASE_H
