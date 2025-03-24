/*
*	File: dep.cpp
*	---------------
*   dynamic exploration planner implementation
*/

#include <global_planner/can_uniform_graph.h>
#include <random>


namespace globalPlanner{
	CAN::CAN(const ros::NodeHandle& nh, const std::string& robot_ns) : DRMBase(nh, robot_ns) {
		this->ns_ = "CAN";
		this->hint_ = "[CAN]";
		this->initParam();
		this->initModules();
		this->registerPub();
		this->registerCallback();
	}

	void CAN::initParam(){
        // odom topic name
		if (not this->nh_.getParam(this->ns_ + "/odom_topic", this->odomTopic_)){
			this->odomTopic_ = "/CERLAB/quadcopter/odom";
			cout << this->hint_ << ": No odom topic name. Use default: /CERLAB/quadcopter/odom" << endl;
		}
		else{
			std::string prefix = "/" + this->robot_ns_;
			this->odomTopic_ = prefix + this->odomTopic_;
			cout << this->hint_ << ": Odom topic: " << this->odomTopic_ << endl;
		}

		// global sample region min
		std::vector<double> globalRegionMinTemp;	
		if (not this->nh_.getParam(this->ns_ + "/global_region_min", globalRegionMinTemp)){
			this->globalRegionMin_(0) = -20.0;
			this->globalRegionMin_(1) = -20.0;
			this->globalRegionMin_(2) = 0.0;
			cout << this->hint_ << ": No global region min param. Use default: [-20 -20 0]" <<endl;
		}
		else{
			this->globalRegionMin_(0) = globalRegionMinTemp[0];
			this->globalRegionMin_(1) = globalRegionMinTemp[1];
			this->globalRegionMin_(2) = globalRegionMinTemp[2];
			cout << this->hint_ << ": Global Region Min: " << this->globalRegionMin_[0] <<" "<< this->globalRegionMin_[1]<<" "<< this->globalRegionMin_[2]<< endl;
		}

		// global sample region max
		std::vector<double> globalRegionMaxTemp;	
		if (not this->nh_.getParam(this->ns_ + "/global_region_max", globalRegionMaxTemp)){
			this->globalRegionMax_(0) = 20.0;
			this->globalRegionMax_(1) = 20.0;
			this->globalRegionMax_(2) = 3.0;
			cout << this->hint_ << ": No global region max param. Use default: [20 20 3]" <<endl;
		}
		else{
			this->globalRegionMax_(0) = globalRegionMaxTemp[0];
			this->globalRegionMax_(1) = globalRegionMaxTemp[1];
			this->globalRegionMax_(2) = globalRegionMaxTemp[2];
			cout << this->hint_ << ": Global Region Max: " << this->globalRegionMax_[0] <<" "<< this->globalRegionMax_[1]<<" "<< this->globalRegionMax_[2]<< endl;
		}

		// safety distance for random sampling in xy
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_xy", this->safeDistXY_)){
			this->safeDistXY_ = 0.3;
			cout << this->hint_ << ": No safe distance in XY param. Use default: 0.3" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance in XY: " << this->safeDistXY_ << endl;
		}

		// safety distance for random sampling in z
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_z", this->safeDistZ_)){
			this->safeDistZ_ = 0.0;
			cout << this->hint_ << ": No safe distance in Z param. Use default: 0.0" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance in Z: " << this->safeDistZ_ << endl;
		}

		// safety distance check unknown
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_check_unknown", this->safeDistCheckUnknown_)){
			this->safeDistCheckUnknown_ = true;
			cout << this->hint_ << ": No safe distance check unknown param. Use default: true" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance check unknown: " << this->safeDistCheckUnknown_ << endl;
		}

		// Camera Parameters	
		if (not this->nh_.getParam(this->ns_ + "/horizontal_FOV", this->horizontalFOV_)){
			this->horizontalFOV_ = 0.8;
			cout << this->hint_ << ": No Horizontal FOV param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Horizontal FOV: " << this->horizontalFOV_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/vertical_FOV", this->verticalFOV_)){
			this->verticalFOV_ = 0.8;
			cout << this->hint_ << ": No Vertical FOV param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Vertical FOV: " << this->verticalFOV_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/dmin", this->dmin_)){
			this->dmin_ = 0.0;
			cout << this->hint_ << ": No min depth param. Use default: 0.0" << endl;
		}
		else{
			cout << this->hint_ << ": Min Depth: " << this->dmin_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/dmax", this->dmax_)){
			this->dmax_ = 1.0;
			cout << this->hint_ << ": No max depth param. Use default: 1.0" << endl;
		}
		else{
			cout << this->hint_ << ": Max Depth: " << this->dmax_ << endl;
		}

		// nearest neighbor number
		if (not this->nh_.getParam(this->ns_ + "/nearest_neighbor_number", this->nnNum_)){
			this->nnNum_ = 15;
			cout << this->hint_ << ": No nearest neighbor param. Use default: 15" << endl;
		}
		else{
			cout << this->hint_ << ": Nearest neighbor number is set to: " << this->nnNum_ << endl;
		}

		// number of yaw angles
		int yawNum = 32;
		if (not this->nh_.getParam(this->ns_ + "/num_yaw_angles", yawNum)){
			for (int i=0; i<32; ++i){
				this->yaws_.push_back(i*2*PI_const/32);
			}					
			cout << this->hint_ << ": No number of yaw angles param. Use default: 32." << endl;
		}
		else {
			for (int i=0; i<yawNum; ++i){
				this->yaws_.push_back(i*2*PI_const/32);
			}	
			cout << this->hint_ << ": Number of yaw angles is set to: " << yawNum << endl;
		}

		// minimum threshold of voxels
		if (not this->nh_.getParam(this->ns_ + "/min_voxel_thresh", this->minVoxelThresh_)){
			this->minVoxelThresh_ = 0.1;
			cout << this->hint_ << ": No minimum threshold of voxels param. Use default: 0.1." << endl;
		}
		else {
			cout << this->hint_ << ": Minimum threshold of voxels is set to: " << this->minVoxelThresh_ << endl;
		}

		// minimum distance for node sampling
		if (not this->nh_.getParam(this->ns_ + "/dist_thresh", this->distThresh_)){
			this->distThresh_ = 0.8;
			cout << this->hint_ << ": No distance thresh param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Distance Thresh: " << this->distThresh_ << endl;
		}

		// node connection max distances
		if (not this->nh_.getParam(this->ns_ + "/max_connect_dist", this->maxConnectDist_)){
			this->maxConnectDist_ = 1.5;
			cout << this->hint_ << ": No max conect distance param. Use default: 1.5m." << endl;
		}
		else{
			cout << this->hint_ << ": Max connect distance is set to: " << this->maxConnectDist_ << endl;
		}
		
		// graph node height
		if (not this->nh_.getParam(this->ns_ + "/graph_node_height", this->graphNodeHeight_)){
			this->graphNodeHeight_ = 1.5;
			cout << this->hint_ << ": No graph node height param. Use default: 1.5." << endl;
		}
		else{
			cout << this->hint_ << ": Graph node height is set to: " << this->graphNodeHeight_ << endl;
		}
		
		// Number of global points
        std::vector<int> numGlobalPointsTemp;	
		if (not this->nh_.getParam(this->ns_ + "/num_global_points", numGlobalPointsTemp)){
			this->numGlobalPoints_.push_back(30);
			this->numGlobalPoints_.push_back(30);
			this->numGlobalPoints_.push_back(3);
			cout << this->hint_ << ": No num global points param. Use default: [20 20 3]" <<endl;
		}
		else{
			this->numGlobalPoints_.push_back(numGlobalPointsTemp[0]);
			this->numGlobalPoints_.push_back(numGlobalPointsTemp[1]);
			this->numGlobalPoints_.push_back(numGlobalPointsTemp[2]);
			cout << this->hint_ << ": Number of global points: " << this->numGlobalPoints_[0] <<" "<< this->numGlobalPoints_[1]<<" "<< this->numGlobalPoints_[2]<< endl;
		}

        std::vector<double> envGlobalRangeMaxTemp;
        if (not this->nh_.getParam(this->ns_ + "/env_global_range_max", envGlobalRangeMaxTemp)) {
            this->envGlobalRangeMax_(0) = 10.0;
            this->envGlobalRangeMax_(1) = 10.0;
            this->envGlobalRangeMax_(2) = 2.0;
            cout << this->hint_ << ": No env global range max param. Use default: [10.0 10.0 2.0]" << endl;
        } else {
            this->envGlobalRangeMax_(0) = envGlobalRangeMaxTemp[0];
            this->envGlobalRangeMax_(1) = envGlobalRangeMaxTemp[1];
            this->envGlobalRangeMax_(2) = envGlobalRangeMaxTemp[2];
            cout << this->hint_ << ": Global range max: " << this->envGlobalRangeMax_[0] << " " << this->envGlobalRangeMax_[1] << " " << this->envGlobalRangeMax_[2] << endl;
        }

        std::vector<double> envGlobalRangeMinTemp;
        if (not this->nh_.getParam(this->ns_ + "/env_global_range_min", envGlobalRangeMinTemp)) {
            this->envGlobalRangeMin_(0) = -10.0;
            this->envGlobalRangeMin_(1) = -10.0;
            this->envGlobalRangeMin_(2) = 0.5;
            cout << this->hint_ << ": No env global range min param. Use default: [-10.0 -10.0 0.5]" << endl;
        } else {
            this->envGlobalRangeMin_(0) = envGlobalRangeMinTemp[0];
            this->envGlobalRangeMin_(1) = envGlobalRangeMinTemp[1];
            this->envGlobalRangeMin_(2) = envGlobalRangeMinTemp[2];
            cout << this->hint_ << ": Global range min: " << this->envGlobalRangeMin_[0] << " " << this->envGlobalRangeMin_[1] << " " << this->envGlobalRangeMin_[2] << endl;
        }

        // initialize global uniform graph
        this->uncoveredGlobalPoints_ = this->generateGlobalPoints2D(this->graphNodeHeight_);

	}

	void CAN::initModules(){
		this->navTargetVec_.clear();
		// initialize roadmap
		this->roadmap_.reset(new PRM::KDTree ());
	}

	void CAN::resetRoadmap(const gazebo_msgs::ModelState& resetRobotPos) {
	    // IMPORTANT: lock waypointUpdateTimer to prevent visiting invalid memory
	    this->currGoalReceived_ = false;
		this->resettingRLEnv_ = false;
		this->waypointTimer_.stop();  // 无法清理队列中未处理的回调事件，存在竞态
		{
			std::lock_guard<std::mutex> lock(roadmap_mutex_); // 加锁保护重置

			this->canNodeVec_.clear();
			// this->roadmap_->clear();
			this->uncoveredGlobalPoints_ = this->generateGlobalPoints2D(this->graphNodeHeight_);

			Eigen::Vector3d p;
			p(0) = resetRobotPos.pose.position.x;
			p(1) = resetRobotPos.pose.position.y;
			p(2) = resetRobotPos.pose.position.z;
			this->currGoal_.reset(new PRM::Node(p));

			this->navWaypoint_ = Point3D(p(0), p(1), p(2));
			this->navHeading_ = Point3D(0, 0, 0);

			this->waypointIdx_ = 0;  
			this->histTraj_.clear();  
			this->bestPath_.clear();

			// Clear navigation targets
			this->navTargetVec_.clear();
		}
	    // IMPORTANT: unlock waypointUpdateTimer
		this->currGoalReceived_ = false;
		this->resettingRLEnv_ = true;
		this->waypointTimer_.start();
	}

	void CAN::registerPub(){
		// roadmap visualization publisher
		this->roadmapPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->rostopic_prefix_ + "/dep/roadmap", 5);

		// best path publisher
		this->bestPathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->rostopic_prefix_ + "/dep/best_paths", 10);

		// waypoint publisher
		this->waypointPub_ = this->nh_.advertise<geometry_msgs::PointStamped>(this->rostopic_prefix_ + "/falco_planner/way_point", 5); 
	}

	void CAN::registerCallback(){
		// odom subscriber
		this->odomSub_ = this->nh_.subscribe(this->odomTopic_, 1000, &CAN::odomCB, this);
	
		// visualization timer
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.2), &CAN::visCB, this);

		//added by me waypoint publish timer
		this->waypointTimer_ = this->nh_.createTimer(ros::Duration(0.33), &CAN::waypointUpdateCB, this);

		//added by me
		this->currGoalSub_ = this->nh_.subscribe(this->rostopic_prefix_ + "/agent/current_goal", 5, &CAN::currGoalCB, this); 
		this->navTargetSub_ = this->nh_.subscribe(this->rostopic_prefix_ + "/env/nav_target", 5, &CAN::navTargetArrayCB, this);
	}

    std::unordered_set<Eigen::Vector3d, Vector3dHash, Vector3dEqual> CAN::generateGlobalPoints2D(double& height) {
        std::vector<double> x(this->numGlobalPoints_[0]), y(this->numGlobalPoints_[1]);

        for (int i = 1; i <= this->numGlobalPoints_[0]; ++i) {
            x[i-1] = (this->envGlobalRangeMax_(0) - this->envGlobalRangeMin_(0)) * i / this->numGlobalPoints_[0] + this->envGlobalRangeMin_(0);
        }
        for (int i = 1; i <= this->numGlobalPoints_[1]; ++i) {
            y[i-1] = (this->envGlobalRangeMax_(1) - this->envGlobalRangeMin_(1)) * i / this->numGlobalPoints_[1] + this->envGlobalRangeMin_(1);
        }

        std::unordered_set<Eigen::Vector3d, Vector3dHash, Vector3dEqual> points;

        for (const auto& xi : x) {
            for (const auto& yi : y) {
                points.emplace(Eigen::Vector3d(xi, yi, height));
            }
        }

        return points;
    }

	bool CAN::makePlan(){
		if (not this->odomReceived_) return false;
	
		this->buildRoadMap();
	
		this->pruneNodes();

		this->updateInformationGain();

		return true;
	}

	void CAN::buildRoadMap(){
		std::shared_ptr<PRM::Node> n;
		std::vector<std::shared_ptr<PRM::Node>> newNodes;

		// add all navigation goals to PRM
		for (auto& navTarget : this->navTargetVec_) {
			if (this->isPosValid(navTarget->pos, this->safeDistXY_, this->safeDistZ_) && 
				(!this->canNodeVec_.count(navTarget))) {
				this->canNodeVec_.insert(navTarget);
				// this->roadmap_->insert(navTarget);
				newNodes.push_back(navTarget);
			}
		}

        // 1. scan the uncoverd global points
        // 2. insert the ones that within the explored map, and erase them from the uncovered global points 
        for (const Eigen::Vector3d& point: this->uncoveredGlobalPoints_) {
            bool addToGraph = true;
            // Check minimum distance to all navigation targets
            for (const auto& navTarget : this->navTargetVec_) {
                Eigen::Vector3d diff = point - navTarget->pos;
                double dist = diff.norm();
                if (dist < 0.2) {
                    addToGraph = false;
                    break;
                }
            }
            
            if (addToGraph && this->isPosValid(point, this->safeDistXY_, this->safeDistZ_)) {
                std::shared_ptr<PRM::Node> newNode (new PRM::Node(point));
                this->canNodeVec_.insert(newNode);
                // this->roadmap_->insert(newNode);
                newNodes.push_back(newNode);
            }
        }
		
		// remove new nodes from uncovered global_points
		for (std::shared_ptr<PRM::Node>& n : newNodes) {
            this->uncoveredGlobalPoints_.erase(n->pos);
			n->newNode = true;
		}

        // node connection
		for (const std::shared_ptr<PRM::Node>& n : this->canNodeVec_){  // new nodes or all nodes?
            n->adjNodes.clear();	
			std::vector<std::shared_ptr<PRM::Node>> knn = this->kNearestNeighbor(n, this->nnNum_);
			for (std::shared_ptr<PRM::Node>& nearestNeighborNode : knn){ // Check collision last if all other conditions are satisfied
				// bool hasCollision = this->map_->isInflatedOccupiedLine(n->pos, nearestNeighborNode->pos);
				bool hasCollision = not this->map_->isInflatedFreeLine(n->pos, nearestNeighborNode->pos);
				if (hasCollision == false) {
					n->adjNodes.insert(nearestNeighborNode);
					nearestNeighborNode->adjNodes.insert(n);
				}
			}
		}
	}	 

	void CAN::pruneNodes(){
		// record the invalid nodes
		std::unordered_set<std::shared_ptr<PRM::Node>> invalidSet;
		for (std::shared_ptr<PRM::Node> n : this->canNodeVec_){ // new nodes
			if (not this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){// 1. new nodes
			// if (this->map_->isInflatedOccupied(n->pos)){// 1. new nodes
				invalidSet.insert(n);
			}	
		}

		// remove invalid nodes
		for (std::shared_ptr<PRM::Node> in : invalidSet){
			this->canNodeVec_.erase(in);
			// this->roadmap_->remove(in);
            this->uncoveredGlobalPoints_.insert(in->pos);  // insert back to uncovered global points set
		}

		//  remove invalid edges
		for (std::shared_ptr<PRM::Node> n : this->canNodeVec_){
			std::vector<std::shared_ptr<PRM::Node>> eraseVec;
			for (std::shared_ptr<PRM::Node> neighbor : n->adjNodes){
				if (invalidSet.find(neighbor) != invalidSet.end()){
					eraseVec.push_back(neighbor);
				}
			}

			for (std::shared_ptr<PRM::Node> en : eraseVec){
				n->adjNodes.erase(en);
			}
		}
	}

	void CAN::updateInformationGain(){
		for (std::shared_ptr<PRM::Node> updateN : this->canNodeVec_){ // update information gain
			std::unordered_map<double, int> yawNumVoxels;
			int unknownVoxelNum = this->calculateUnknown(updateN, yawNumVoxels);
			updateN->numVoxels = unknownVoxelNum;
			updateN->yawNumVoxels = yawNumVoxels;
			updateN->newNode = false;
		}
		this->histTraj_.clear(); // clear history
	}

	std::shared_ptr<PRM::Node> CAN::nearestNeighbor(std::shared_ptr<PRM::Node> query) {
        if (!query) {
            throw std::invalid_argument("nearestNeighbor: query node is null");
        }
        if (canNodeVec_.empty()) {
            return nullptr;
        }

        std::shared_ptr<PRM::Node> nearest = nullptr;
        double min_dist = std::numeric_limits<double>::max();

        for (const auto& node : canNodeVec_) {
            if (!node || node == query) { // 排除无效节点和自身
                continue;
            }

            const double dist = (node->pos - query->pos).norm();
            if (dist < min_dist) {
                min_dist = dist;
                nearest = node;
            }
        }

        return nearest; // 可能返回nullptr当所有节点均为无效或唯一节点是自身
    }

	std::vector<std::shared_ptr<PRM::Node>> CAN::kNearestNeighbor(
        std::shared_ptr<PRM::Node> query, 
        int num
    ) {
        if (!query) {
            throw std::invalid_argument("kNearestNeighbor: query node is null");
        }
        if (num <= 0 || canNodeVec_.empty()) {
            return {};
        }

        // 定义最大堆比较器：按距离从大到小排序
        auto cmp = [](const std::pair<double, std::shared_ptr<PRM::Node>>& a, 
                     const std::pair<double, std::shared_ptr<PRM::Node>>& b) {
            return a.first < b.first;
        };
        std::priority_queue<
            std::pair<double, std::shared_ptr<PRM::Node>>,
            std::vector<std::pair<double, std::shared_ptr<PRM::Node>>>,
            decltype(cmp)
        > max_heap(cmp);

        for (const auto& node : canNodeVec_) {
            if (!node || node == query) { // 过滤无效节点和自身
                continue;
            }

            const double dist = (node->pos - query->pos).norm();
            if (max_heap.size() < static_cast<size_t>(num)) {
                max_heap.emplace(dist, node);
            } else if (dist < max_heap.top().first) {
                max_heap.pop();
                max_heap.emplace(dist, node);
            }
        }

        // 转换为有序结果
        std::vector<std::shared_ptr<PRM::Node>> result;
        while (!max_heap.empty()) {
            result.push_back(max_heap.top().second);
            max_heap.pop();
        }
        std::reverse(result.begin(), result.end()); // 从小到大排序

        return result;
    }


	void CAN::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		this->position_ = Eigen::Vector3d (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		this->currYaw_ = globalPlanner::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		this->odomReceived_ = true;

		if (this->histTraj_.size() == 0){
			this->histTraj_.push_back(this->position_);
		}
		else{
			Eigen::Vector3d lastPos = this->histTraj_.back();
			double dist = (this->position_ - lastPos).norm();
			if (dist >= 0.5){
				if (this->histTraj_.size() >= 100){
					this->histTraj_.pop_front();
					this->histTraj_.push_back(this->position_);
				}
				else{
					this->histTraj_.push_back(this->position_);
				}
			}
		}
	}

	void CAN::visCB(const ros::TimerEvent&){
		if (this->canNodeVec_.size() != 0){
			visualization_msgs::MarkerArray roadmapMarkers = this->buildRoadmapMarkers();
			this->roadmapPub_.publish(roadmapMarkers);
		}

		if (this->bestPath_.size() != 0){
			this->publishBestPath();
		}
	}

	void CAN::currGoalCB(const geometry_msgs::PointStamped::ConstPtr& goal) {
		Eigen::Vector3d p(goal->point.x, goal->point.y, goal->point.z);
		this->currGoal_.reset(new PRM::Node(p));

		this->currGoalReceived_ = true;
		this->resettingRLEnv_ = false;
	}

	void CAN::waypointUpdateCB(const ros::TimerEvent&) {
		std::lock_guard<std::mutex> lock(roadmap_mutex_); // 加锁访问共享数据

		if (this->resettingRLEnv_) {
			assert(this->currGoal_ != nullptr);
			// Publishing robot initial position as waypoint to reset RL Env
			geometry_msgs::PointStamped waypoint;
			waypoint.header.stamp = ros::Time::now();
			waypoint.header.frame_id = "map";
			waypoint.point.x = this->currGoal_->pos(0);
			waypoint.point.y = this->currGoal_->pos(1);
			waypoint.point.z = this->currGoal_->pos(2);
			this->waypointPub_.publish(waypoint);
			// std::cout << "\033[1;32m[Debug]: Publishing robot initial position as waypoint to reset RL Env... \033[0m" << std::endl;
		}
		else if (this->currGoalReceived_) {
			assert(this->currGoal_ != nullptr);
			assert(this->map_ != nullptr);

			Point3D lastNavWaypoint = this->navWaypoint_;

			// find best path to current goal
			this->bestPath_.clear();
			// find nearest node of current location
			std::shared_ptr<PRM::Node> currPos = std::make_shared<PRM::Node>(this->position_);
			
			if (!currPos) {  // debug
				throw std::runtime_error("currPos node is null");
			}
			
			std::shared_ptr<PRM::Node> start = this->nearestNeighbor(currPos);
			std::shared_ptr<PRM::Node> temp_goal = this->nearestNeighbor(this->currGoal_);  // this is important, or no path will be found!

			if (!start) {  // debug
				std::cout << "\033[1;31m[PRM Nodes]:" << " Size = " << this->canNodeVec_.size() << "\033[0m" << std::endl;
				throw std::runtime_error("Start node is null");
			}

			std::vector<std::shared_ptr<PRM::Node>> path = PRM::AStar(this->roadmap_, start, temp_goal, this->map_);
			if (int(path.size()) != 0) {
				// Finding path success
				path.insert(path.begin(), currPos);
				std::vector<std::shared_ptr<PRM::Node>> pathSc;
				this->shortcutPath(path, pathSc);
				this->bestPath_ = pathSc;
				this->waypointIdx_ = 0;
			}

			// construct waypoint
			geometry_msgs::PointStamped waypoint;
			waypoint.header.stamp = ros::Time::now();
			waypoint.header.frame_id = "map";
			if (this->bestPath_.size() != 0) {
				if (((this->position_ - this->bestPath_[this->waypointIdx_]->pos).norm() <= 0.2) && (this->waypointIdx_ < (this->bestPath_.size() - 1))){
					this->waypointIdx_ += 1;
				}

				this->navWaypoint_ = Point3D(this->bestPath_[this->waypointIdx_]->pos(0), this->bestPath_[this->waypointIdx_]->pos(1), this->bestPath_[this->waypointIdx_]->pos(2));

				if (std::find(this->navTargetVec_.begin(), this->navTargetVec_.end(), this->bestPath_[this->waypointIdx_]) != this->navTargetVec_.end()) {
					this->navHeading_ = Point3D(0, 0, 0);
					waypoint.point.x = this->navWaypoint_.x;
					waypoint.point.y = this->navWaypoint_.y;
					waypoint.point.z = this->navWaypoint_.z;
					std::cout << "[Falco Planner]: Waypoint is a navigation target, terminating waypoint projection..." << std::endl;
				} else {
					Point3D projectedWaypoint = this->projectNavWaypoint(this->navWaypoint_, lastNavWaypoint);
					waypoint.point.x = projectedWaypoint.x;
					waypoint.point.y = projectedWaypoint.y;
					waypoint.point.z = projectedWaypoint.z;
				}
			}
			else {
				// Find global paths fails. Directly using local planner
				// std::cout << "\033[1;31m[Debug]: Find global paths fails. Directly using local planner... \033[0m" << std::endl;
				this->navHeading_ = Point3D(0, 0, 0);

				waypoint.point.x = this->currGoal_->pos(0);
				waypoint.point.y = this->currGoal_->pos(1);
				waypoint.point.z = this->currGoal_->pos(2);
			}
			this->waypointPub_.publish(waypoint);
		}
	}

	void CAN::navTargetArrayCB(const decision_roadmap_agent::TargetArray::ConstPtr& navTargets) {
	    this->navTargetVec_.clear();
	    
	    for (const geometry_msgs::Point& target : navTargets->targets) {
	        Eigen::Vector3d navTargetPos(target.x, target.y, target.z);
	        auto newTarget = std::make_shared<PRM::Node>(navTargetPos);
	        newTarget->isNavTarget = true;
	        this->navTargetVec_.push_back(newTarget);
	    }

	    if (!navTargets->targets.empty()) {
	        cout << "[CAN]: " << navTargets->targets.size() << " navigation targets received from target array." << endl;
	    }
	}

	visualization_msgs::MarkerArray CAN::buildRoadmapMarkers(){  
		visualization_msgs::MarkerArray roadmapMarkers;

		// PRM nodes and edges
		int countPointNum = 0;
		int countEdgeNum = 0;
		int countVoxelNumText = 0;
		for (std::shared_ptr<PRM::Node> n : this->canNodeVec_){
			// Node point
			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "graph_point";
			point.id = countPointNum;
			point.type = visualization_msgs::Marker::SPHERE;
			point.action = visualization_msgs::Marker::ADD;
			point.pose.position.x = n->pos(0);
			point.pose.position.y = n->pos(1);
			point.pose.position.z = n->pos(2);
			point.lifetime = ros::Duration(0.2); //5
			point.scale.x = 0.05;
			point.scale.y = 0.05;
			point.scale.z = 0.05;
			point.color.a = 1.0;
			point.color.r = 1.0;
			point.color.g = 1.0;
			point.color.b = 0.0;
			++countPointNum;
			roadmapMarkers.markers.push_back(point);

			// number of voxels for each node
			visualization_msgs::Marker voxelNumText;
			voxelNumText.ns = "num_voxel_text";
			voxelNumText.header.frame_id = "map";
			voxelNumText.id = countVoxelNumText;
			voxelNumText.header.stamp = ros::Time::now();
			voxelNumText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			voxelNumText.action = visualization_msgs::Marker::ADD;
			voxelNumText.pose.position.x = n->pos(0);
			voxelNumText.pose.position.y = n->pos(1);
			voxelNumText.pose.position.z = n->pos(2)+0.1;
			voxelNumText.scale.x = 0.2;
			voxelNumText.scale.y = 0.2;
			voxelNumText.scale.z = 0.2;
			voxelNumText.color.a = 1.0;
			voxelNumText.text = std::to_string(n->numVoxels);
			voxelNumText.lifetime = ros::Duration(0.2); //5
			++countVoxelNumText;
			roadmapMarkers.markers.push_back(voxelNumText);

			// Edges 
			visualization_msgs::Marker line;
			line.ns = "edge";
			line.header.frame_id = "map";
			line.type = visualization_msgs::Marker::LINE_LIST;
			line.header.stamp = ros::Time::now();

			for (std::shared_ptr<PRM::Node> adjNode : n->adjNodes){
				geometry_msgs::Point p1, p2;
				p1.x = n->pos(0);
				p1.y = n->pos(1);
				p1.z = n->pos(2);
				p2.x = adjNode->pos(0);
				p2.y = adjNode->pos(1);
				p2.z = adjNode->pos(2);				
				line.points.push_back(p1);
				line.points.push_back(p2);
				line.id = countEdgeNum;
				line.scale.x = 0.02;
				line.scale.y = 0.02;
				line.scale.z = 0.02;
				line.color.r = 1.0;
				line.color.g = 1.0;
				line.color.b = 0.0;
				line.color.a = 1.0;
				line.lifetime = ros::Duration(0.2); //5
				++countEdgeNum;
				// roadmapMarkers.markers.push_back(line);
			}
			if(n->adjNodes.empty()) {
				geometry_msgs::Point p1;
				p1.x = n->pos(0);
				p1.y = n->pos(1);
				p1.z = n->pos(2);
				line.points.push_back(p1);
			}
			roadmapMarkers.markers.push_back(line);
		}

		return roadmapMarkers;
	}

	void CAN::publishBestPath(){
		visualization_msgs::MarkerArray bestPathMarkers;
		int countNodeNum = 0;
		int countLineNum = 0;
		for (size_t i=0; i<this->bestPath_.size(); ++i){
			std::shared_ptr<PRM::Node> n = this->bestPath_[i];
			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "best_path_node";
			point.id = countNodeNum;
			point.type = visualization_msgs::Marker::SPHERE;
			point.action = visualization_msgs::Marker::ADD;
			point.pose.position.x = n->pos(0);
			point.pose.position.y = n->pos(1);
			point.pose.position.z = n->pos(2);
			point.lifetime = ros::Duration(0.5);
			point.scale.x = 0.01;
			point.scale.y = 0.01;
			point.scale.z = 0.01;
			point.color.a = 1.0;
			point.color.r = 1.0;
			point.color.g = 1.0;
			point.color.b = 1.0;
			++countNodeNum;
			bestPathMarkers.markers.push_back(point);

			if (i<this->bestPath_.size()-1){
				std::shared_ptr<PRM::Node> nNext = this->bestPath_[i+1];
				visualization_msgs::Marker line;
				line.ns = "best_path";
				line.header.frame_id = "map";
				line.type = visualization_msgs::Marker::LINE_LIST;
				line.header.stamp = ros::Time::now();
				geometry_msgs::Point p1, p2;
				p1.x = n->pos(0);
				p1.y = n->pos(1);
				p1.z = n->pos(2);
				p2.x = nNext->pos(0);
				p2.y = nNext->pos(1);
				p2.z = nNext->pos(2);				
				line.points.push_back(p1);
				line.points.push_back(p2);
				line.id = countLineNum;
				line.scale.x = 0.1;
				line.scale.y = 0.1;
				line.scale.z = 0.1;
				line.color.r = 1.0;
				line.color.g = 0.0;
				line.color.b = 0.0;
				line.color.a = 1.0;
				line.lifetime = ros::Duration(0.5);
				++countLineNum;
				bestPathMarkers.markers.push_back(line);				
			}
		}
		this->bestPathPub_.publish(bestPathMarkers);		
	}

}