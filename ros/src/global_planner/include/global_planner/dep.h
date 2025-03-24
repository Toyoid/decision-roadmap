/*
*	File: dep.h
*	---------------
*   dynamic exploration planner header file
*/

#ifndef DEP_H
#define DEP_H
  
#include <global_planner/drm_base.h>

namespace globalPlanner{
	class DEP : public DRMBase{  // Make sure inheritance is public
	private:
		ros::Publisher candidatePathPub_;
		ros::Publisher frontierVisPub_;
		ros::Publisher bestPathGoalPub_;
		ros::Subscriber currGoalSub_;  
		ros::Subscriber navTargetSub_;

		// parameters
		Eigen::Vector3d localRegionMin_, localRegionMax_;
		int localSampleThresh_;
		int globalSampleThresh_;
		int frontierSampleThresh_;
		int nnNumFrontier_;
		int minCandidateNum_;
		int maxCandidateNum_;
		double updateDist_;
		double yawPenaltyWeight_;

		// data
		std::unordered_set<std::shared_ptr<PRM::Node>> prmNodeVec_; // all nodes
		std::vector<std::shared_ptr<PRM::Node>> goalCandidates_;
		std::vector<std::vector<std::shared_ptr<PRM::Node>>> candidatePaths_;
		std::vector<std::shared_ptr<PRM::Node>> navTargetVec_;

	public:
		DEP(const ros::NodeHandle& nh, const std::string& robot_ns);

		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		bool makePlan();
		void detectFrontierRegion(std::vector<std::pair<Eigen::Vector3d, double>>& frontierPointPairs);
		void buildRoadMap();
		void pruneNodes();
		void updateInformationGain();
		void getBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates);
		bool findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates,  std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths);
		void findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths, std::vector<std::shared_ptr<PRM::Node>>& bestPath);
		
		// callback functions
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		void visCB(const ros::TimerEvent&);
		void currGoalCB(const geometry_msgs::PointStamped::ConstPtr& goal);  
 		void waypointUpdateCB(const ros::TimerEvent&);  
		void navTargetArrayCB(const decision_roadmap_agent::TargetArray::ConstPtr& navTargets);

		// visualization functions
		visualization_msgs::MarkerArray buildRoadmapMarkers();
		void publishCandidatePaths();
		void publishBestPath();
		void publishFrontier();

		// clear function for RL training
		void resetRoadmap(const gazebo_msgs::ModelState& resetRobotPos);
	};
}

#endif