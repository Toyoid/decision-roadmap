/*
*	File: dprm_planner.h
*	---------------
*   dynamic PRM global planner header file
*/

#ifndef DPRM_PLANNER_H
#define DPRM_PLANNER_H

#include <global_planner/drm_base.h>

namespace globalPlanner{
	class DPRM : public DRMBase{
	private:
		ros::Publisher frontierVisPub_;
		ros::Publisher currGoalPub_;
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
		double dist2TargetWeight_;
		double pathLengthWeight_;

		// data
		bool navTargetReceived_ = false; 
		bool navTargetReached_;
		bool waitNodeFound_;
		std::shared_ptr<PRM::Node> navTargetWaitNode_;
		std::shared_ptr<PRM::Node> navTarget_;
		Eigen::Vector3d stopPos_;
		std::unordered_set<std::shared_ptr<PRM::Node>> prmNodeVec_; // all nodes
		std::vector<std::shared_ptr<PRM::Node>> goalCandidates_;
		std::vector<std::vector<std::shared_ptr<PRM::Node>>> candidatePaths_;

	public:
		DPRM(const ros::NodeHandle& nh, const std::string& robot_ns); 

		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		bool isNavTargetReceived();
		void setCurrGoalReceived(bool& currGoalReceived);

		bool makePlan();
		void detectFrontierRegion(std::vector<std::pair<Eigen::Vector3d, double>>& frontierPointPairs);
		void buildRoadMap();
		void pruneNodes();
		void getBestCurrGoal();
		void updateInformationGain();
		void getBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates);
		bool findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates,  std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths);
		void findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths, std::vector<std::shared_ptr<PRM::Node>>& bestPath);		

		// callback functions
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		void visCB(const ros::TimerEvent&);
 		void waypointUpdateCB(const ros::TimerEvent&);  
		void navTargetCB(const geometry_msgs::PointStamped::ConstPtr& navTarget);

		// visualization functions
		visualization_msgs::MarkerArray buildRoadmapMarkers();
		void publishBestPath();
		void publishFrontier();

		// clear function for RL training
		void resetRoadmap(const gazebo_msgs::ModelState& resetRobotPos);
	};
}


#endif


