/*
*	File: dep.h
*	---------------
*   dynamic exploration planner header file
*/

#ifndef CAN_UNIFORM_GRAPH_H
#define CAN_UNIFORM_GRAPH_H

#include <global_planner/drm_base.h>


namespace globalPlanner{
    // self-defined hash function for Eigen::Vector3d
    struct Vector3dHash {
        std::size_t operator()(const Eigen::Vector3d& vec) const {
            std::size_t hx = std::hash<double>()(vec.x());
            std::size_t hy = std::hash<double>()(vec.y());
            std::size_t hz = std::hash<double>()(vec.z());
            return hx ^ (hy << 1) ^ (hz << 2);
        }
    };

    // self-defined equal-to function for Eigen::Vector3d
    struct Vector3dEqual {
        bool operator()(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) const {
            return lhs.isApprox(rhs);
        }
    };
    
	class CAN : public DRMBase{
	private:
		ros::Subscriber currGoalSub_;  
        ros::Subscriber navTargetSub_;

		// parameters
		double graphNodeHeight_;
		std::vector<int> numGlobalPoints_;
		Eigen::Vector3d envGlobalRangeMax_, envGlobalRangeMin_;

		// data
		std::unordered_set<std::shared_ptr<PRM::Node>> canNodeVec_; // all nodes
		std::unordered_set<Eigen::Vector3d, Vector3dHash, Vector3dEqual> uncoveredGlobalPoints_; // uncovered uniform nodes
        std::vector<std::shared_ptr<PRM::Node>> navTargetVec_;

	public:
		CAN(const ros::NodeHandle& nh, const std::string& robot_ns);

		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		bool makePlan();
		void buildRoadMap();
		void pruneNodes();
		void updateInformationGain();		

		// helper functions
		std::shared_ptr<PRM::Node> nearestNeighbor(std::shared_ptr<PRM::Node> query);
		std::vector<std::shared_ptr<PRM::Node>> kNearestNeighbor(std::shared_ptr<PRM::Node> query, int num);

		// callback functions
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		void visCB(const ros::TimerEvent&);
		void currGoalCB(const geometry_msgs::PointStamped::ConstPtr& goal);  
 		void waypointUpdateCB(const ros::TimerEvent&);  
        void navTargetArrayCB(const decision_roadmap_agent::TargetArray::ConstPtr& navTargets);

		// visualization functions
		visualization_msgs::MarkerArray buildRoadmapMarkers();
		void publishBestPath();

		// clear function for RL training
		void resetRoadmap(const gazebo_msgs::ModelState& resetRobotPos);

        std::unordered_set<Eigen::Vector3d, Vector3dHash, Vector3dEqual> generateGlobalPoints2D(double& height);
	};
}


#endif


