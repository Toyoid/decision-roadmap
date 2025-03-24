/*
	FILE: .cpp
	-----------------------------
	Implementation of
*/

#include <autonomous_flight/dynamicPRM.h>

namespace AutoFlight{
    dynamicPRM::dynamicPRM(const ros::NodeHandle& nh, const std::string robot_ns) : flightBase(nh, robot_ns){
		this->robot_ns_ = robot_ns;
		this->initParam();
		this->initModules();
		// roadmap service server	
		std::string prefix = this->robot_ns_ == this->single_robot_ns_? "": "/" + this->robot_ns_;
		this->getRoadmapServer_ = this->nh_.advertiseService(prefix + "/dep/get_roadmap", &dynamicPRM::getRoadmapServiceCB, this);
		this->resetRoadmapServer_ = this->nh_.advertiseService(prefix + "/dep/reset_roadmap", &dynamicPRM::resetRoadmapServiceCB, this);

		if (this->useFakeDetector_){
			// free map callback
			this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.066), &dynamicPRM::freeMapCB, this);
		}
	}

	void dynamicPRM::initParam(){
    	// use simulation detector	
		if (not this->nh_.getParam("autonomous_flight/use_fake_detector", this->useFakeDetector_)){
			this->useFakeDetector_ = false;
			cout << "[AutoFlight]: No use fake detector param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use fake detector is set to: " << this->useFakeDetector_ << "." << endl;
		}
		
		// desired velocity
		if (not this->nh_.getParam("autonomous_flight/desired_velocity", this->desiredVel_)){
			this->desiredVel_ = 0.5;
			cout << "[AutoFlight]: No desired velocity param. Use default 0.5m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired velocity is set to: " << this->desiredVel_ << "m/s." << endl;
		}	

		// desired acceleration
		if (not this->nh_.getParam("autonomous_flight/desired_acceleration", this->desiredAcc_)){
			this->desiredAcc_ = 2.0;
			cout << "[AutoFlight]: No desired acceleration param. Use default 2.0m/s^2." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired acceleration is set to: " << this->desiredAcc_ << "m/s^2." << endl;
		}	

		//  desired angular velocity
		if (not this->nh_.getParam("autonomous_flight/desired_angular_velocity", this->desiredAngularVel_)){
			this->desiredAngularVel_ = 0.5;
			cout << "[AutoFlight]: No angular velocity param. Use default 0.5rad/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Angular velocity is set to: " << this->desiredAngularVel_ << "rad/s." << endl;
		}	

		//  desired angular velocity
		if (not this->nh_.getParam("autonomous_flight/waypoint_stablize_time", this->wpStablizeTime_)){
			this->wpStablizeTime_ = 1.0;
			cout << "[AutoFlight]: No waypoint stablize time param. Use default 1.0s." << endl;
		}
		else{
			cout << "[AutoFlight]: Waypoint stablize time is set to: " << this->wpStablizeTime_ << "s." << endl;
		}	

		//  initial scan
		if (not this->nh_.getParam("autonomous_flight/initial_scan", this->initialScan_)){
			this->initialScan_ = false;
			cout << "[AutoFlight]: No initial scan param. Use default 1.0s." << endl;
		}
		else{
			cout << "[AutoFlight]: Initial scan is set to: " << this->initialScan_ << endl;
		}	

    	// replan time for dynamic obstacle
		// if (not this->nh_.getParam("autonomous_flight/replan_time_for_dynamic_obstacles", this->replanTimeForDynamicObstacle_)){
		// 	this->replanTimeForDynamicObstacle_ = 0.3;
		// 	cout << "[AutoFlight]: No dynamic obstacle replan time param found. Use default: 0.3s." << endl;
		// }
		// else{
		// 	cout << "[AutoFlight]: Dynamic obstacle replan time is set to: " << this->replanTimeForDynamicObstacle_ << "s." << endl;
		// }	

    	// free range 
    	std::vector<double> freeRangeTemp;
		if (not this->nh_.getParam("autonomous_flight/free_range", freeRangeTemp)){
			this->freeRange_ = Eigen::Vector3d (2, 2, 1);
			cout << "[AutoFlight]: No free range param found. Use default: [2, 2, 1]m." << endl;
		}
		else{
			this->freeRange_(0) = freeRangeTemp[0];
			this->freeRange_(1) = freeRangeTemp[1];
			this->freeRange_(2) = freeRangeTemp[2];
			cout << "[AutoFlight]: Free range is set to: " << this->freeRange_.transpose() << "m." << endl;
		}	

    	// reach goal distance
		if (not this->nh_.getParam("autonomous_flight/reach_goal_distance", this->reachGoalDistance_)){
			this->reachGoalDistance_ = 0.1;
			cout << "[AutoFlight]: No reach goal distance param found. Use default: 0.1m." << endl;
		}
		else{
			cout << "[AutoFlight]: Reach goal distance is set to: " << this->reachGoalDistance_ << "m." << endl;
		}	
	}

	void dynamicPRM::initModules(){
		// initialize map
		if (this->useFakeDetector_){
			// initialize fake detector
			this->detector_.reset(new onboardDetector::fakeDetector (this->nh_));	
			// this->map_.reset(new mapManager::occMap (this->nh_, false));
			this->map_.reset(new mapManager::occMap (this->nh_, this->robot_ns_));
		}
		else{
			this->map_.reset(new mapManager::occMap (this->nh_, this->robot_ns_));
		}

		// initialize exploration planner
		this->expPlanner_.reset(new globalPlanner::DEP (this->nh_, this->robot_ns_));
		this->expPlanner_->setMap(this->map_);
		this->expPlanner_->loadVelocity(this->desiredVel_, this->desiredAngularVel_);
	}

	// void dynamicPRM::registerCallback(){
	// 	// initialize exploration planner replan in another thread
	// 	this->exploreReplanWorker_ = std::thread(&dynamicPRM::exploreReplan, this);
	// 	this->exploreReplanWorker_.detach();
	// }

	void dynamicPRM::freeMapCB(const ros::TimerEvent&){
	    // ROS_INFO("dynamicPRM::freeMap callback called");
        try {
            std::vector<onboardDetector::box3D> obstacles;
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
            this->detector_->getObstacles(obstacles);
            double fov = 1.57;
            for (onboardDetector::box3D ob: obstacles){
                if (this->detector_->isObstacleInSensorRange(ob, fov)){
                    Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-0.3, ob.z);
                    Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+0.3, ob.z+ob.z_width+0.3);
                    freeRegions.push_back(std::make_pair(lowerBound, upperBound));
                }
            }
            this->map_->updateFreeRegions(freeRegions);
            this->map_->freeRegions(freeRegions);
        } catch (const std::exception &e) {
            ROS_ERROR("Exception caught: %s", e.what());
        }
	}

	bool dynamicPRM::getRoadmapServiceCB(global_planner::GetRoadmap::Request& req, global_planner::GetRoadmap::Response& resp){
		this->expPlanner_->setMap(this->map_);
		bool replanSuccess = this->expPlanner_->makePlan();
		if (replanSuccess){
//			std::cout << "\033[1;32m[AutoFlight]: Roadmap Generation Succeed!\033[0m" << endl;
			resp.roadmapMarkers = this->expPlanner_->buildRoadmapMarkers();
		}
		else{
			std::cout << "\033[1;31m[AutoFlight]: Roadmap Generation failed!\033[0m" << endl;
		}		

		return true;
	}

	bool dynamicPRM::resetRoadmapServiceCB(falco_planner::SetRobotPose::Request& req, falco_planner::SetRobotPose::Response& resp){
        // ROS_INFO("resetRoadmapService callback called");
        try {
            // if (this->useFakeDetector_){
            // 	// reset fake detector
            // 	// this->detector_.reset(new onboardDetector::fakeDetector (this->nh_));
            //     this->map_.reset(new mapManager::dynamicMap (this->nh_, false));
            // }
            // else{
            // 	this->map_.reset(new mapManager::dynamicMap (this->nh_));
            // }


            // two ways to clear the map data
            // way 1: call member funtion in occMap to clear (but the last scan from previous episode will remain, the timers' frequecy should set lower, like 1/0.2)
            // this->map_->clearMapData();

            // way 2: create a new map object every time after reset (the registered service should be shutdown before that)
            // reset roadmap first to set status flag
            this->expPlanner_->resetRoadmap(req.robotPose);

            this->map_->collisionCheckServer_.shutdown();
            this->map_.reset(new mapManager::occMap (this->nh_, this->robot_ns_));
            this->expPlanner_->setMap(this->map_);
            cout << "\n\033[1;32m[Roadmap]: Successfully reset map and roadmap.\033[0m" << endl;

            Eigen::Vector3d startPos (req.robotPose.pose.position.x, req.robotPose.pose.position.y, req.robotPose.pose.position.z);
		    Eigen::Vector3d c1 = startPos - this->freeRange_;
		    Eigen::Vector3d c2 = startPos + this->freeRange_;
		    this->map_->freeRegion(c1, c2);
		    cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << this->freeRange_.transpose() << endl;

        } catch (const std::exception &e) {
            ROS_ERROR("Exception caught: %s", e.what());
        }
		return true;
	}

	void dynamicPRM::run(){
//		cout << "\033[1;32m[AutoFlight]: Please double check all parameters. Then PRESS ENTER to continue or PRESS CTRL+C to stop.\033[0m" << endl;
		// std::cin.clear();
		// fflush(stdin);
		// std::cin.get();
		// this->takeoff();

//		cout << "\033[1;32m[AutoFlight]: Takeoff succeed. Then PRESS ENTER to continue or PRESS CTRL+C to land.\033[0m" << endl;
		// std::cin.clear();
		// fflush(stdin);
		// std::cin.get();

		this->initExplore();

		cout << "\n\033[1;32m[AutoFlight]: Flight System is Ready. Launch a Decision Roadmap Agent to Start Planning.\033[0m" << endl;
		// std::cin.clear();
		// fflush(stdin);
		// std::cin.get();

		// this->registerCallback();
		// this->exploreReplanWorker_ = std::thread(&dynamicPRM::exploreReplan, this);
		// this->exploreReplanWorker_.detach();
	}

	void dynamicPRM::initExplore(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d c1 = startPos - this->freeRange_;
		Eigen::Vector3d c2 = startPos + this->freeRange_;
		this->map_->freeRegion(c1, c2);
		cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << this->freeRange_.transpose() << endl;

		// maybe this fragment of code can be used for suck situations
		if (this->initialScan_){
			cout << "[AutoFlight]: Start initial scan..." << endl;
			this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Press ENTER to continue next 90 degree.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();
						
			this->moveToOrientation(-PI_const, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Press ENTER to continue next 90 degree.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();

			this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Press ENTER to continue next 90 degree.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();
			
			this->moveToOrientation(0, this->desiredAngularVel_);
			cout << "[AutoFlight]: End initial scan." << endl; 
		}		
	}
}