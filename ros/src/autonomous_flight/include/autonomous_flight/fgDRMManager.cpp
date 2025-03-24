/*
    FILE: fgDRMManager.cpp
    -----------------------------
    Implementation of frontier-guided decision roadmap manager
*/

#include <autonomous_flight/fgDRMManager.h>

namespace AutoFlight{
    fgDRMManager::fgDRMManager(const ros::NodeHandle& nh, const std::string robot_ns) : flightBase(nh, robot_ns){
        ROS_INFO("[fgDRMManager] Constructor called");
        this->robot_ns_ = robot_ns;
        this->initParam();
        this->initModules();
		std::string prefix = this->robot_ns_ == this->single_robot_ns_? "": "/" + this->robot_ns_;

        this->getRoadmapServer_ = this->nh_.advertiseService(prefix + "/fg_drm/get_roadmap", &fgDRMManager::getRoadmapServiceCB, this);
        this->resetRoadmapServer_ = this->nh_.advertiseService(prefix + "/fg_drm/reset_roadmap", &fgDRMManager::resetRoadmapServiceCB, this);

        if (this->useFakeDetector_){
            this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.066), &fgDRMManager::freeMapCB, this);
        }
    }

    void fgDRMManager::initParam(){
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

    void fgDRMManager::initModules(){
        ROS_INFO("[fgDRMManager] Initializing modules...");
        
        // initialize map
        if (this->useFakeDetector_){
            ROS_INFO("[fgDRMManager] Creating fake detector...");
            this->detector_.reset(new onboardDetector::fakeDetector (this->nh_));    
            ROS_INFO("[fgDRMManager] Creating occupancy map...");
            this->map_.reset(new mapManager::occMap (this->nh_, this->robot_ns_));
        }
        else{
            ROS_INFO("[fgDRMManager] Creating occupancy map without detector...");
            this->map_.reset(new mapManager::occMap (this->nh_, this->robot_ns_));
        }

        // initialize exploration planner
        ROS_INFO("[fgDRMManager] Creating Fronier-Guided DRM Manager...");
        this->expPlanner_.reset(new globalPlanner::FGDRM (this->nh_, this->robot_ns_));
        this->expPlanner_->setMap(this->map_);
        this->expPlanner_->loadVelocity(this->desiredVel_, this->desiredAngularVel_);
	}

    void fgDRMManager::freeMapCB(const ros::TimerEvent&){
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

    bool fgDRMManager::getRoadmapServiceCB(global_planner::GetRoadmap::Request& req, global_planner::GetRoadmap::Response& resp){
        if (!this->map_ || !this->expPlanner_) {
            ROS_ERROR("[fgDRMManager] Map or planner not initialized");
            return false;
        }

        try {
            this->expPlanner_->setMap(this->map_);
            bool replanSuccess = this->expPlanner_->makePlan();
            
            if (!replanSuccess) {
                ROS_ERROR("[fgDRMManager] Roadmap Generation failed!");
                return false;
            }

            // Get both marker arrays
            visualization_msgs::MarkerArray overallMarkers = this->expPlanner_->buildRoadmapMarkers();
            visualization_msgs::MarkerArray frontierMarkers = this->expPlanner_->buildFrontierRoadmapMarkers();
            
            // Combine markers by appending frontier markers to overall markers
            overallMarkers.markers.insert(
                overallMarkers.markers.end(),
                frontierMarkers.markers.begin(),
                frontierMarkers.markers.end()
            );
            resp.roadmapMarkers = overallMarkers;
        } catch (const std::exception& e) {
            ROS_ERROR("[fgDRMManager] Exception in getRoadmapServiceCB: %s", e.what());
            return false;
        }

        return true;
    }

    bool fgDRMManager::resetRoadmapServiceCB(falco_planner::SetRobotPose::Request& req, falco_planner::SetRobotPose::Response& resp){
        try {
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

    void fgDRMManager::run(){
		this->initExplore();
		
        cout << "\n\033[1;32m[AutoFlight]: Flight System is Ready. Launch a Decision Roadmap Agent to Start Planning.\033[0m" << endl;
    }

    void fgDRMManager::initExplore(){
        // Set start region to be free
        Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
        Eigen::Vector3d c1 = startPos - this->freeRange_;
        Eigen::Vector3d c2 = startPos + this->freeRange_;
        this->map_->freeRegion(c1, c2);
        cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << this->freeRange_.transpose() << endl;

        // Handle initial scan if enabled
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
