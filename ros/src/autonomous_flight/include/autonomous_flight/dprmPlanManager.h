/*
	FILE: .h
	-----------------------------
	header of 
*/
#ifndef DPRM_PLAN_MANAGER
#define DPRM_PLAN_MANAGER
#include <autonomous_flight/flightBase.h>
#include <global_planner/dprm_planner.h>
// #include <map_manager/dynamicMap.h>
#include <map_manager/occupancyMap.h>
#include <onboard_detector/fakeDetector.h>
#include <global_planner/GetRoadmap.h>
#include <falco_planner/SetRobotPose.h>
#include <std_srvs/Empty.h>

namespace AutoFlight{
	class dprmPlanManager : flightBase{
	private:
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<onboardDetector::fakeDetector> detector_;
		std::shared_ptr<globalPlanner::DPRM> expPlanner_;

		ros::Timer freeMapTimer_;
		ros::Timer updatePRMTimer_;
		ros::ServiceServer resetRoadmapServer_;
		// ros::ServiceServer pauseMapUpdateServer_;
		
		// parameters
		bool useFakeDetector_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		double wpStablizeTime_;
		bool initialScan_;

		Eigen::Vector3d freeRange_;
		double reachGoalDistance_;

		// exploration data
		bool explorationReplan_ = true;
		bool replan_ = false;
		bool newWaypoints_ = false;
		int waypointIdx_ = 1;
		nav_msgs::Path waypoints_; // latest waypoints from exploration planner

		ros::Time lastDynamicObstacleTime_;

	public:
		// std::thread exploreReplanWorker_;
		dprmPlanManager(const ros::NodeHandle& nh, const std::string& robot_ns);

		void initParam();
		void initModules();
	
		// void exploreReplan();
		void freeMapCB(const ros::TimerEvent&); // using fake detector
		void updatePRMTimerCB(const ros::TimerEvent&);
		bool resetRoadmapServiceCB(falco_planner::SetRobotPose::Request& req, falco_planner::SetRobotPose::Response& resp);
		// bool pauseMapUpdateServiceCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

		void run();
		void initExplore();
	};
}
#endif