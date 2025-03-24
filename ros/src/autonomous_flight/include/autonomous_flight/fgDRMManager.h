/*
    FILE: fgDRMManager.h
    -----------------------------
    header of frontier-guided decision roadmap manager
*/
#ifndef FG_DRM_MANAGER_H
#define FG_DRM_MANAGER_H

#include <autonomous_flight/flightBase.h>
#include <global_planner/fg_drm.h>
#include <map_manager/occupancyMap.h>
#include <onboard_detector/fakeDetector.h>
#include <global_planner/GetRoadmap.h>
#include <falco_planner/SetRobotPose.h>
#include <std_srvs/Empty.h>

namespace AutoFlight{
    class fgDRMManager : flightBase{
    private:
        std::shared_ptr<mapManager::occMap> map_;
        std::shared_ptr<onboardDetector::fakeDetector> detector_;
        std::shared_ptr<globalPlanner::FGDRM> expPlanner_;

        ros::Timer freeMapTimer_;
        ros::ServiceServer getRoadmapServer_;
        ros::ServiceServer resetRoadmapServer_;
        
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
        nav_msgs::Path waypoints_; 

        ros::Time lastDynamicObstacleTime_;

    public:
        fgDRMManager();
        fgDRMManager(const ros::NodeHandle& nh, const std::string robot_ns);

        void initParam();
        void initModules();
        void freeMapCB(const ros::TimerEvent&);
        bool getRoadmapServiceCB(global_planner::GetRoadmap::Request& req, global_planner::GetRoadmap::Response& resp);
        bool resetRoadmapServiceCB(falco_planner::SetRobotPose::Request& req, falco_planner::SetRobotPose::Response& resp);

        void run();
        void initExplore();
    };
}

#endif
