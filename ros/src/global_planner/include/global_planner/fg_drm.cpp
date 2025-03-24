/*
*	File: fg_drm.cpp
*	---------------
*   frontier-guided decision roadmap implementation
*/

#include <global_planner/fg_drm.h>
#include <random>

namespace globalPlanner{
	FGDRM::FGDRM(const ros::NodeHandle& nh, const std::string& robot_ns) : DRMBase(nh, robot_ns) {
		this->ns_ = "FGDRM";
		this->hint_ = "[FGDRM]";
		this->initParam();
		this->initModules();
		this->registerPub();
		this->registerCallback();
	}

	void FGDRM::initParam(){
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

		// local sample region min
		std::vector<double> localRegionMinTemp;	
		if (not this->nh_.getParam(this->ns_ + "/local_region_min", localRegionMinTemp)){
			this->localRegionMin_(0) = -5.0;
			this->localRegionMin_(1) = -5.0;
			this->localRegionMin_(2) = -2.0;
			cout << this->hint_ << ": No local region min param. Use default: [-5 -5 -2]" <<endl;
		}
		else{
			this->localRegionMin_(0) = localRegionMinTemp[0];
			this->localRegionMin_(1) = localRegionMinTemp[1];
			this->localRegionMin_(2) = localRegionMinTemp[2];
			cout << this->hint_ << ": Local Region Min: " << this->localRegionMin_[0] <<" " <<this->localRegionMin_[1]<<" "<< this->localRegionMin_[2]<< endl;
		}

		// local sample region max
		std::vector<double> localRegionMaxTemp;	
		if (not this->nh_.getParam(this->ns_ + "/local_region_max", localRegionMaxTemp)){
			this->localRegionMax_(0) = 5.0;
			this->localRegionMax_(1) = 5.0;
			this->localRegionMax_(2) = 2.0;
			cout << this->hint_ << ": No local region max param. Use default: [5 5 2]" <<endl;
		}
		else{
			this->localRegionMax_(0) = localRegionMaxTemp[0];
			this->localRegionMax_(1) = localRegionMaxTemp[1];
			this->localRegionMax_(2) = localRegionMaxTemp[2];
			cout << this->hint_ << ": Local Region Max: " << this->localRegionMax_[0] <<" " <<this->localRegionMax_[1]<<" "<< this->localRegionMax_[2]<< endl;
		}

		// Local Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/local_sample_thresh", this->localSampleThresh_)){
			this->localSampleThresh_ = 50;
			cout << this->hint_ << ": No local sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Local sample Thresh: " << this->localSampleThresh_ << endl;
		}
		
		// Global Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/global_sample_thresh", this->globalSampleThresh_)){
			this->globalSampleThresh_ = 50;
			cout << this->hint_ << ": No global sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Global sample Thresh: " << this->globalSampleThresh_ << endl;
		}

		// Frontier Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/frontier_sample_thresh", this->frontierSampleThresh_)){
			this->frontierSampleThresh_ = 50;
			cout << this->hint_ << ": No frontier sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Frontier sample Thresh: " << this->frontierSampleThresh_ << endl;
		}		

		// frontier nearest neighbor number
		if (not this->nh_.getParam(this->ns_ + "/frontier_nearest_neighbor_number", this->nnNumFrontier_)){
			this->nnNumFrontier_ = 15;
			cout << this->hint_ << ": No frontier nearest neighbor param. Use default: 15" << endl;
		}
		else{
			cout << this->hint_ << ": Frontier nearest neighbor number is set to: " << this->nnNumFrontier_ << endl;
		}

		// minimum number of goal candidates
		if (not this->nh_.getParam(this->ns_ + "/min_goal_candidates", this->minCandidateNum_)){
			this->minCandidateNum_ = 10;
			cout << this->hint_ << ": No minimum number of goal candidates param. Use default: 10." << endl;
		}
		else{
			cout << this->hint_ << ": Minimum number of goal candidates is set to: " << this->minCandidateNum_ << endl;
		}

		// maximum number of goal candidates
		if (not this->nh_.getParam(this->ns_ + "/max_goal_candidates", this->maxCandidateNum_)){
			this->maxCandidateNum_ = 30;
			cout << this->hint_ << ": No maximum number of goal candidates param. Use default: 30." << endl;
		}
		else{
			cout << this->hint_ << ": Maximum number of goal candidates is set to: " << this->maxCandidateNum_ << endl;
		}

		// Information gain update distance
		if (not this->nh_.getParam(this->ns_ + "/information_gain_update_distance", this->updateDist_)){
			this->updateDist_ = 1.0;
			cout << this->hint_ << ": No information gain update distance param. Use default: 1.0m." << endl;
		}
		else{
			cout << this->hint_ << ": Information gain update distance is set to: " << this->updateDist_ << endl;
		}

		// yaw penalty weight
		if (not this->nh_.getParam(this->ns_ + "/yaw_penalty_weight", this->yawPenaltyWeight_)){
			this->yawPenaltyWeight_ = 1.0;
			cout << this->hint_ << ": No yaw penalty weight param. Use default: 1.0." << endl;
		}
		else{
			cout << this->hint_ << ": Yaw penalty weight is set to: " << this->yawPenaltyWeight_ << endl;
		}
	}

	void FGDRM::initModules(){
		this->navTargetVec_.clear();
		// initialize roadmap
		this->roadmap_.reset(new PRM::KDTree ());
	}

	void FGDRM::resetRoadmap(const gazebo_msgs::ModelState& resetRobotPos) {
	    // IMPORTANT: lock waypointUpdateTimer to prevent visiting invalid memory
	    this->currGoalReceived_ = false;
		this->resettingRLEnv_ = false;
		// 先停止定时器并等待所有回调完成
		this->waypointTimer_.stop();  // 无法清理队列中未处理的回调事件，存在竞态
		{
			std::lock_guard<std::mutex> lock(roadmap_mutex_); // 加锁保护重置

			this->prmNodeVec_.clear();
			this->roadmap_->clear();
			// 重建树时应确保线程安全
        	this->roadmap_ = std::make_shared<PRM::KDTree>(); // 全新实例

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
			this->frontierPointPairs_.clear();

			// Clear navigation targets
			this->navTargetVec_.clear();
		}
	    // IMPORTANT: unlock waypointUpdateTimer
		this->currGoalReceived_ = false;
		this->resettingRLEnv_ = true;
		this->waypointTimer_.start();
	}

	void FGDRM::registerPub(){
		// overall roadmap visualization publisher
		this->roadmapPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->rostopic_prefix_ + "/fg_drm/overall/roadmap", 5);

		// visualization publisher for the frontier nodes of the roadmap
		this->frontierRoadmapPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->rostopic_prefix_ + "/fg_drm/frontier_guide/roadmap", 5);

		// candidate paths publisher
		this->candidatePathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->rostopic_prefix_ + "/fg_drm/candidate_paths", 10);

		// best path publisher
		this->bestPathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->rostopic_prefix_ + "/fg_drm/best_paths", 10);

		// frontier region vis publisher
		this->frontierVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->rostopic_prefix_ + "/fg_drm/frontier_regions", 10);

		//added by me
		this->waypointPub_ = this->nh_.advertise<geometry_msgs::PointStamped>(this->rostopic_prefix_ + "/falco_planner/way_point", 5); 
	}

	void FGDRM::registerCallback(){
		// odom subscriber
		this->odomSub_ = this->nh_.subscribe(this->odomTopic_, 1000, &FGDRM::odomCB, this);

		// visualization timer
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.2), &FGDRM::visCB, this);

		this->waypointTimer_ = this->nh_.createTimer(ros::Duration(0.33), &FGDRM::waypointUpdateCB, this);

		this->currGoalSub_ = this->nh_.subscribe(this->rostopic_prefix_ + "/agent/current_goal", 5, &FGDRM::currGoalCB, this); 

		this->navTargetSub_ = this->nh_.subscribe(this->rostopic_prefix_ + "/env/nav_target", 5, &FGDRM::navTargetArrayCB, this);
	}

	bool FGDRM::makePlan(){
		if (not this->odomReceived_) return false;
		// cout << "start detecting frontier" << endl;
		// ros::Time frontierStartTime = ros::Time::now();
		this->detectFrontierRegion(this->frontierPointPairs_);
		// ros::Time frontierEndTime = ros::Time::now();
		// cout << "frontier detection time: " << (frontierEndTime - frontierStartTime).toSec() << endl;

		// cout << "start building roadmap" << endl;
		// ros::Time buildStartTime = ros::Time::now();
		this->buildRoadMap();
		// ros::Time buildEndTime = ros::Time::now();
		// cout << "build roadmap time: " << (buildEndTime - buildStartTime).toSec() << endl;

		// cout << "start pruning nodes" << endl;
		// ros::Time updateStartTime = ros::Time::now();
		this->pruneNodes();

		// cout << "start update information gain" << endl;
		this->updateInformationGain();
		// ros::Time updateEndTime = ros::Time::now();
		// cout << "update time: " << (updateEndTime - updateStartTime).toSec() << endl;

		// cout << "start get goal candidates" << endl;
		// ros::Time pathStartTime = ros::Time::now();
		this->getBestViewCandidates(this->goalCandidates_);
		if (this->goalCandidates_.size() > 0) {
			std::cout << "\033[1;32m[Before Filter]: Best view candidate size: " << this->goalCandidates_.size() << "\033[0m" << std::endl;
		} 
		else {
			std::cout << "\033[1;31m[Before Filter]: Best view candidate size: " << this->goalCandidates_.size() << "\033[0m" << std::endl;
		}
		this->filterBestViewCandidates(this->goalCandidates_);
		if (this->goalCandidates_.size() > 1) {
			std::cout << "\033[1;31m[After Filter]:  Best view candidate size: " << this->goalCandidates_.size() << "\033[0m" << std::endl;
		} 
		// else {
		// 	std::cout << "\033[1;32m[After Filter]:  Best view candidate size: " << this->goalCandidates_.size() << "\033[0m" << std::endl;
		// }

		std::cout << "\033[1;32m[KD-Tree]:" << " Size = " << this->roadmap_->getSize() << "\033[0m" << std::endl;
		// std::vector<int> heights = this->roadmap_->checkBalance();
		// std::cout << "\033[1;32m[KD-Tree]:" << " Tree Height = " << heights[0] << " Left Height = " << heights[1] << " Right Height = " << heights[2] << " Unbalance rate = " << std::fixed << std::setprecision(2) << std::abs(static_cast<float>((heights[1] - heights[2])) / heights[0]) << "\033[0m" << std::endl;
		std::cout << "\033[1;32m[PRM Nodes]:" << " Size = " << this->prmNodeVec_.size() << "\033[0m" << std::endl;

		// bool findCandidatePathSuccess = this->findCandidatePath(this->goalCandidates_, this->candidatePaths_);

		// cout << "finish candidate path with size: " << this->candidatePaths_.size() << endl;
		// this->waypointIdx_ = 0; 
		// geometry_msgs::PointStamped bestGoal;	
		// bestGoal.header.frame_id = "map";
		// if (not findCandidatePathSuccess){
		// 	// cout << "Find candidate paths fails. need generate more samples." << endl;
		// 	// std::cout << "\033[1;31m[Debug]: Find candidate paths fails. Please carefully check the candidate points. \033[0m" << std::endl;
		// 	std::cout << "\033[1;31m[Debug]: Find candidate paths fails. Directly using local planner... \033[0m" << std::endl;
		// 	std::shared_ptr<PRM::Node> bestNode = this->goalCandidates_.back();
		// 	bestGoal.header.stamp = ros::Time::now();
		// 	bestGoal.point.x = bestNode->pos(0);
		// 	bestGoal.point.y = bestNode->pos(1);
		// 	bestGoal.point.z = bestNode->pos(2);
		// 	this->waypointPub_.publish(bestGoal);
		// 	// std::cin.clear();
		// 	// fflush(stdin);
		// 	// std::cin.get();
		// 	// return false;
		// }
		// else {
		// 	this->findBestPath(this->candidatePaths_, this->bestPath_);

		// 	std::shared_ptr<PRM::Node> bestNode = this->bestPath_.back();
		// 	bestGoal.header.stamp = ros::Time::now();
		// 	bestGoal.point.x = bestNode->pos(0);
		// 	bestGoal.point.y = bestNode->pos(1);
		// 	bestGoal.point.z = bestNode->pos(2);
		// 	this->bestPathGoalPub_.publish(bestGoal);
		// }
		// ros::Time pathEndTime = ros::Time::now();
		// cout << "path time: " << (pathEndTime - pathStartTime).toSec() << endl;
		// cout << "found best path with size: " << this->bestPath_.size() << endl;

		return true;
	}

	void FGDRM::detectFrontierRegion(std::vector<std::pair<Eigen::Vector3d, double>>& frontierPointPairs){
		frontierPointPairs.clear();

		Eigen::Vector3d mapMin, mapMax;
		this->map_->getCurrMapRange(mapMin, mapMax);
		int numRow = (mapMax(1) - mapMin(1))/this->map_->getRes() + 1;
		int numCol = (mapMax(0) - mapMin(0))/this->map_->getRes() + 1;

		cv::SimpleBlobDetector::Params params;
		// params.filterByColor = true;
		// params.blobColor = 255;
		// params.filterByConvexity = true;
  //   	params.minConvexity = 0.1;;

		params.filterByColor = true;
		params.blobColor = 255;  // Blobs should be white
		params.filterByArea = true;
		params.minArea = pow(1/this->map_->getRes(), 2);
		params.maxArea = numRow * numCol;
		params.filterByCircularity = false;
		params.minCircularity = 1;
		params.filterByConvexity = true;
		params.minConvexity = 0.1;

		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		std::vector<cv::Mat> imgVec;
		// find height levels to slice the map
		double heightRes = 0.3;
		int col = 0;
		int row = 0;
		for (double h=this->globalRegionMin_(2); h<=this->globalRegionMax_(2); h+=heightRes){
			row = 0;
			cv::Mat im (numRow, numCol, CV_8UC1);
			for (double y=mapMin(1); y<=mapMax(1) and row < numRow; y+=this->map_->getRes()){
				col = 0;
				for (double x=mapMin(0); x<=mapMax(0) and col < numCol; x+=this->map_->getRes()){
					Eigen::Vector3d p (x, y, h);
					if (this->map_->isInflatedOccupied(p)){
						im.at<uchar>(row, col) = 0;
					}
					else if (this->map_->isInflatedFree(p)){
						// im.at<uchar>(row, col) = 255/2;
						// im.at<uchar>(row, col) = 255;
						im.at<uchar>(row, col) = 0;
					}
					else{
						// im.at<uchar>(row, col) = 255/2;
						im.at<uchar>(row, col) = 255;
					}
					++col;
				}
				++row;
			}
			imgVec.push_back(im);
		}



		// detect each image and find the corresponding 3D positions
		double height = this->globalRegionMin_(2);
		for (cv::Mat img : imgVec){
			std::vector<cv::KeyPoint> keypoints;

			cv::Rect rect(0, 0, numRow, numCol);
			cv::rectangle(img, rect, cv::Scalar(0, 0, 0), 3);
			detector->detect(img, keypoints);

			// convert keypoints back to the map coordinate
			for (cv::KeyPoint keypoint : keypoints){
				Eigen::Vector3d p (mapMin(0) + keypoint.pt.x * this->map_->getRes(), mapMin(1) + keypoint.pt.y * this->map_->getRes(), height);
				double dist = keypoint.size * this->map_->getRes();
				frontierPointPairs.push_back({p, dist});
			}

			height += heightRes;

			// cv::Mat imgWithKeypoints;
			// cv::drawKeypoints(img, keypoints, imgWithKeypoints,  cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

			// cv::imshow("Blobs with Keypoints", imgWithKeypoints);
			// cv::imwrite("/home/zhefan/Desktop/temp/test.jpg", img);
			// cv::waitKey(0);
		} 
	}

	void FGDRM::buildRoadMap(){
		bool saturate = false;
		bool regionSaturate = false;
		int countSample = 0;
		std::shared_ptr<PRM::Node> n;
		std::vector<std::shared_ptr<PRM::Node>> newNodes;

		// add all navigation goals to PRM
		for (auto& navTarget : this->navTargetVec_) {
			if (this->isPosValid(navTarget->pos, this->safeDistXY_, this->safeDistZ_) && 
				(!this->prmNodeVec_.count(navTarget))) {
				this->prmNodeVec_.insert(navTarget);
				this->roadmap_->insert(navTarget);
				newNodes.push_back(navTarget);
			}
		}

		// while does reach sampling threshold (fail time) 
		// 1. sample point from frontier region by weighted sampling
		// 2. find several nearest node
		// 3. for each node, extend random [mindist, maxdist] distance. if success, add new sample 
		std::vector<double> sampleWeights;
		for (int i=0; i<int(this->frontierPointPairs_.size()); ++i){
			double size = this->frontierPointPairs_[i].second;
			sampleWeights.push_back(pow(size, 2));
		}
		int countFrontierFailure = 0;
		while (ros::ok() and countFrontierFailure < this->frontierSampleThresh_ and sampleWeights.size() != 0){
			std::shared_ptr<PRM::Node> fn = this->sampleFrontierPoint(sampleWeights);
			// a. find N nearest neighbors
			std::vector<std::shared_ptr<PRM::Node>> fnNeighbors = this->roadmap_->kNearestNeighbor(fn, this->nnNumFrontier_);

			// b. for each neighbor extend them and check the validity
			if (int(fnNeighbors.size()) > 0){
				int countSampleOnce = 0;
				for (std::shared_ptr<PRM::Node> fnNN : fnNeighbors){
					n = this->extendNode(fnNN, fn);
					if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
						std::shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
						double distToNN = (n->pos - nn->pos).norm();
						if (distToNN >= this->distThresh_){
							this->roadmap_->insert(n);
							newNodes.push_back(n);
							// this->prmNodeVec_.push_back(n);
							this->prmNodeVec_.insert(n);
							++countSample;	
							++countSampleOnce;
						}
					}
				}
				if (countSampleOnce == 0){
					++countFrontierFailure;
				}

			}
			else{ // not enough neighbor
				break;
			}
		}
		// cout << "added sample from frontier:  " << countSample << endl;

		while (ros::ok() and not saturate){
			if (regionSaturate){
				int countFailureGlobal = 0;
				// Generate new node
				while (ros::ok()){
					if (countFailureGlobal > this->globalSampleThresh_){
						saturate = true;
						break;
					}
					n = this->randomConfigBBox(this->globalRegionMin_, this->globalRegionMax_);
					// Check how close new node is other nodes
					double distToNN;
					if (this->roadmap_->getSize() != 0){
						shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
						distToNN = (n->pos - nn->pos).norm();
					}
					else{
						distToNN = this->distThresh_;
					}
					if (distToNN < this->distThresh_){
						++countFailureGlobal;
					}
					else{
						this->roadmap_->insert(n);
						newNodes.push_back(n);
						// this->prmNodeVec_.push_back(n);
						this->prmNodeVec_.insert(n);
						++countSample;
					}
				}
			}
			else{
				if (true){
					int countFailureLocal = 0;
					// Generate new node
					while (ros::ok() and true){
						//cout << "failure number: " << countFailureLocal << endl;
						if (countFailureLocal > this->localSampleThresh_){
							regionSaturate = true;
							break;
						}
						Eigen::Vector3d localSampleMin = this->position_+this->localRegionMin_;
						Eigen::Vector3d localSampleMax = this->position_+this->localRegionMax_;
						n = this->randomConfigBBox(localSampleMin, localSampleMax);
						// Check how close new node is other nodes
						double distToNN;

						if (this->roadmap_->getSize() != 0){
							std::shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
							distToNN = (n->pos - nn->pos).norm();
						}
						else{
							distToNN = this->distThresh_;
						}
						if (distToNN < this->distThresh_){
							++countFailureLocal;
						}
						else{
							this->roadmap_->insert(n);
							newNodes.push_back(n);
							// this->prmNodeVec_.push_back(n);
							this->prmNodeVec_.insert(n);
							++countSample;
						}
					}
				}
			}
		}
		
		// node connection
		for (std::shared_ptr<PRM::Node>& n : newNodes){
			std::vector<std::shared_ptr<PRM::Node>> knn = this->roadmap_->kNearestNeighbor(n, this->nnNum_);
			for (std::shared_ptr<PRM::Node>& nearestNeighborNode : knn){ // Check collision last if all other conditions are satisfied
				double distance2knn = (n->pos - nearestNeighborNode->pos).norm();
				bool rangeCondition = sensorRangeCondition(n, nearestNeighborNode) and sensorRangeCondition(nearestNeighborNode, n);
				if (distance2knn < this->maxConnectDist_ and rangeCondition == true){
					// bool hasCollision = not this->map_->isInflatedFreeLine(n->pos, nearestNeighborNode->pos);
					bool hasCollision = this->map_->isInflatedOccupiedLine(n->pos, nearestNeighborNode->pos);
					if (hasCollision == false){
						n->adjNodes.insert(nearestNeighborNode);
						nearestNeighborNode->adjNodes.insert(n);
					}
				}
			}
			n->newNode = true;
		}
	}	 

	void FGDRM::pruneNodes(){
		// record the invalid nodes
		std::unordered_set<std::shared_ptr<PRM::Node>> invalidSet;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){ // new nodes
			if (not this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){// 1. new nodes
			// if (this->map_->isInflatedOccupied(n->pos)){// 1. new nodes
				invalidSet.insert(n);
			}	
		}

		// remove invalid nodes
		for (std::shared_ptr<PRM::Node> in : invalidSet){
			this->prmNodeVec_.erase(in);
			this->roadmap_->remove(in);
		}


		//  remove invalid edges
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
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

	void FGDRM::updateInformationGain(){
//		// iterate through all current nodes (ignore update by path now)
//		// two types of nodes need update:
//		// 1. new nodes
//		// 2. nodes close to the historical trajectory
//		std::unordered_set<std::shared_ptr<PRM::Node>> updateSet;
//		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){ // new nodes
//			if (n->newNode == true){// 1. new nodes
//				updateSet.insert(n);
//			}
//		}
//
//		for (Eigen::Vector3d& histPos : this->histTraj_){ // traj update nodes
//			std::shared_ptr<PRM::Node> histN;
//			histN.reset(new PRM::Node(histPos));
//			std::vector<std::shared_ptr<PRM::Node>> nns = this->roadmap_->kNearestNeighbor(histN, 10);
//			for (std::shared_ptr<PRM::Node>& nn : nns){
//				if ((nn->pos - histN->pos).norm() <= this->updateDist_){
//					updateSet.insert(nn);
//				}
//			}
//		}
//
//		for (std::shared_ptr<PRM::Node> updateN : updateSet){ // update information gain
//			std::unordered_map<double, int> yawNumVoxels;
//			int unknownVoxelNum = this->calculateUnknown(updateN, yawNumVoxels);
//			updateN->numVoxels = unknownVoxelNum;
//			updateN->yawNumVoxels = yawNumVoxels;
//			updateN->newNode = false;
//		}

		for (std::shared_ptr<PRM::Node> updateN : this->prmNodeVec_){ // update information gain
			std::unordered_map<double, int> yawNumVoxels;
			int unknownVoxelNum = this->calculateUnknown(updateN, yawNumVoxels);
			updateN->numVoxels = unknownVoxelNum;
			updateN->yawNumVoxels = yawNumVoxels;
			updateN->newNode = false;
		}
		this->histTraj_.clear(); // clear history
	}

	void FGDRM::getBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates){
		goalCandidates.clear();
		bool firstNode = true;
		std::priority_queue<std::shared_ptr<PRM::Node>, std::vector<std::shared_ptr<PRM::Node>>, PRM::GainCompareNode> gainPQ;

		// iterate through all points in the roadmap
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			gainPQ.push(n);
		}

		// select candidates from the priority queue
		int maxNumVoxel = 0;
		while (ros::ok()){
			if (gainPQ.size() == 0){
				break;
			}

			std::shared_ptr<PRM::Node> n = gainPQ.top();
			
			if (firstNode){
				// if ((n->pos - this->position_).norm() >= 1.0){
				if ((n->pos - this->position_).norm() >= 0.0){
					maxNumVoxel = n->numVoxels;
					firstNode = false;
				}
			}

			if (double(n->numVoxels) < double(maxNumVoxel) * this->minVoxelThresh_){
				break;
			}
			// if ((n->pos - this->position_).norm() >= 1.0){
			if ((n->pos - this->position_).norm() >= 0.0){			
				if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
					goalCandidates.push_back(n);
					// cout << "Valid goal candidate: " << n->pos.transpose() << " voxel: " << n->numVoxels  << endl;
				}
			}
			gainPQ.pop();
			
			if (int(goalCandidates.size()) >= this->maxCandidateNum_){
				break;
			}
		}

		// cout << "current pos: " << this->position_.transpose() << endl;
		while (int(goalCandidates.size()) < this->minCandidateNum_){
			if (gainPQ.size() == 0){
				break;
			}

			if (int(goalCandidates.size()) >= this->maxCandidateNum_){
				break;
			}

			std::shared_ptr<PRM::Node> n = gainPQ.top();
			gainPQ.pop();
			// if ((n->pos - this->position_.norm() >= 1.0){ 
			if ((n->pos - this->position_).norm() >= 0.0){	
				// cout << "candidate goal: " << n->pos.transpose() << endl;	
				if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
					goalCandidates.push_back(n);
					// cout << "Valid goal candidate: " << n->pos.transpose() << " voxel: " << n->numVoxels  << endl;
				}
			}
		}
	}

	void FGDRM::filterBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates) {
		if (goalCandidates.empty()) {
			return;
		}

		// Find nearest node of current location as start of A* path finding
		std::shared_ptr<PRM::Node> start = this->roadmap_->nearestNeighbor(
			std::make_shared<PRM::Node>(this->position_)
		);

		// First check all navigation targets
		bool allNavTargetsReachable = !this->navTargetVec_.empty(); // Only true if we have targets
		for (const auto& navTarget : this->navTargetVec_) {
			std::vector<std::shared_ptr<PRM::Node>> path = PRM::AStar(this->roadmap_, start, navTarget, this->map_);
			if (path.empty()) {
				allNavTargetsReachable = false; // Mark unreachable but continue checking others
			} else {
				// Add reachable navigation target to candidates if not already present
				if (std::find(goalCandidates.begin(), goalCandidates.end(), navTarget) == goalCandidates.end()) {
					goalCandidates.push_back(navTarget);
					navTarget->pathCost = calculatePathLength(path);
				}
			}
		}

		// If all navigation targets are reachable, use only those as candidates
		if (allNavTargetsReachable && !this->navTargetVec_.empty()) {
			goalCandidates = this->navTargetVec_;
			return;
		}

		// Filter candidates based on path existence
		std::vector<std::shared_ptr<PRM::Node>> validCandidates;
		validCandidates.reserve(goalCandidates.size());

		for (const auto& goal : goalCandidates) {
			std::vector<std::shared_ptr<PRM::Node>> path = PRM::AStar(this->roadmap_, start, goal, this->map_);
			if (!path.empty()) {
				goal->pathCost = this->calculatePathLength(path);
				validCandidates.push_back(goal);
			}
		}

		if (!validCandidates.empty()) {
			goalCandidates = std::move(validCandidates);
		} else {
			ROS_WARN("[PRM]: All frontier nodes are not reachable!");
		}
	}

	bool FGDRM::findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates, std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths){
		bool findPath = false;
		// find nearest node of current location
		std::shared_ptr<PRM::Node> currPos;
		currPos.reset(new PRM::Node (this->position_));
		std::shared_ptr<PRM::Node> start = this->roadmap_->nearestNeighbor(currPos);

		candidatePaths.clear();
		for (std::shared_ptr<PRM::Node> goal : goalCandidates){
			std::vector<std::shared_ptr<PRM::Node>> path = PRM::AStar(this->roadmap_, start, goal, this->map_);  
			if (int(path.size()) != 0){
				findPath = true;
			}
			else{
				continue;
			}
			path.insert(path.begin(), currPos);
			std::vector<std::shared_ptr<PRM::Node>> pathSc;
			this->shortcutPath(path, pathSc);
			candidatePaths.push_back(pathSc);
		}
		return findPath;
	}

	void FGDRM::findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths, std::vector<std::shared_ptr<PRM::Node>>& bestPath){
		// find path highest unknown
		bestPath.clear();
		double highestScore = -1;
		for (int n=0; n<int(candidatePaths.size()); ++n){
			std::vector<std::shared_ptr<PRM::Node>> path = candidatePaths[n]; 
			if (int(path.size()) == 0) continue;
			double yawDist = 0;
			double prevYaw = this->currYaw_;
			int unknownVoxel = 0;
			for (int i=0; i<int(path.size())-1; ++i){
				std::shared_ptr<PRM::Node> currNode = path[i];
				std::shared_ptr<PRM::Node> nextNode = path[i+1];
				Eigen::Vector3d diff = nextNode->pos - currNode->pos;
				double angle = atan2(diff(1), diff(0));

				// reevaluate the unknowns for intermediate points
				std::unordered_map<double, int> yawNumVoxels;
				int unknownVoxelNum = this->calculateUnknown(currNode, yawNumVoxels);
				currNode->numVoxels = unknownVoxelNum;
				currNode->yawNumVoxels = yawNumVoxels;


				unknownVoxel += currNode->getUnknownVoxels(angle);
				yawDist += globalPlanner::angleDiff(prevYaw, angle);
				prevYaw = angle;
			}
			// reevaluate the goal node
			std::unordered_map<double, int> yawNumVoxels;
			int unknownVoxelNum = this->calculateUnknown(path.back(), yawNumVoxels);
			path.back()->numVoxels = unknownVoxelNum;
			path.back()->yawNumVoxels = yawNumVoxels;
			unknownVoxel += path.back()->getBestYawVoxel();
			yawDist += globalPlanner::angleDiff(prevYaw, path.back()->getBestYaw());

			double distance = this->calculatePathLength(path);
			// cout << "total is distance is: " << distance << " total yaw distance is: " << yawDist << " voxel: " << path.back()->numVoxels << endl;
			double pathTime = distance/this->vel_ + this->yawPenaltyWeight_ * yawDist/this->angularVel_;
			double score = double(unknownVoxel)/pathTime; 
			// cout << "unknown for path: " << n <<  " is: " << unknownVoxel << " score: " << score << " distance: " << distance << " Time: " << pathTime <<  " Last total unknown: " << path.back()->numVoxels << " last best: " << path.back()->getBestYawVoxel() << endl;
			if (score > highestScore){
				highestScore = score;
				bestPath = path;
			}
		}
		if (highestScore == 0){
			cout << "[FGDRM]: Current score is 0. The exploration might complete." << endl;
		}
	}


	void FGDRM::odomCB(const nav_msgs::OdometryConstPtr& odom){
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

	void FGDRM::visCB(const ros::TimerEvent&){
		if (this->prmNodeVec_.size() != 0){
			visualization_msgs::MarkerArray roadmapMarkers = this->buildRoadmapMarkers();
			this->roadmapPub_.publish(roadmapMarkers);
		}

		if (this->candidatePaths_.size() != 0){
			this->publishCandidatePaths();
		}

		if (this->bestPath_.size() != 0){
			this->publishBestPath();
		}

		if (this->frontierPointPairs_.size() != 0){
			this->publishFrontier();
		}

		if (this->goalCandidates_.size() != 0){
			visualization_msgs::MarkerArray frontierRoadmapMarkers = this->buildFrontierRoadmapMarkers();
			this->frontierRoadmapPub_.publish(frontierRoadmapMarkers);
		}
	}

	void FGDRM::currGoalCB(const geometry_msgs::PointStamped::ConstPtr& goal) {
		Eigen::Vector3d p(goal->point.x, goal->point.y, goal->point.z);
		this->currGoal_.reset(new PRM::Node(p));

		this->currGoalReceived_ = true;
		this->resettingRLEnv_ = false;
	}

	void FGDRM::waypointUpdateCB(const ros::TimerEvent&) {
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
			assert(this->roadmap_ != nullptr);
			assert(this->map_ != nullptr);

			std::shared_ptr<PRM::Node> temp_goal;
			Point3D lastNavWaypoint = this->navWaypoint_;

			// find best path to current goal
			this->bestPath_.clear();
			// find nearest node of current location
			std::shared_ptr<PRM::Node> currPos = std::make_shared<PRM::Node>(this->position_);
			
			if (!currPos) {  // debug
				throw std::runtime_error("currPos node is null");
			}

			std::shared_ptr<PRM::Node> start = this->roadmap_->nearestNeighbor(currPos);
			temp_goal = this->roadmap_->nearestNeighbor(this->currGoal_);  // this is important, or no path will be found!
			
			if (!start) {  // debug
				std::cout << "\033[1;31m[PRM Nodes]:" << " Size = " << this->prmNodeVec_.size() << "\033[0m" << std::endl;
				std::cout << "\033[1;31m[KD-Tree]:" << " Size = " << this->roadmap_->getSize() << "\033[0m" << std::endl;
				std::vector<int> heights = this->roadmap_->checkBalance();
				std::cout << "\033[1;31m[KD-Tree]:" << " Tree Height = " << heights[0]  << " Left Height = " << heights[1] << " Right Height = " << heights[2] << "\033[0m" << std::endl;
				std::cout << "\033[1:31m[KD-Tree]:" << " notTargetTemp_ size = " << this->roadmap_->notTargetTemp_.size() << "\033[0m" << std::endl;
				std::cout << "\033[1:31m[KD-Tree]:" << " notTargetPerm_ size = " << this->roadmap_->notTargetPerm_.size() << "\033[0m" << std::endl;
				std::cout << "\033[1:31m[KD-Tree]:" << " Root node: " << this->roadmap_->getRoot()->pos.transpose() << "\033[0m" << std::endl;
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

	void FGDRM::navTargetArrayCB(const decision_roadmap_agent::TargetArray::ConstPtr& navTargets) {
		this->navTargetVec_.clear();
		
		for (const geometry_msgs::Point& target : navTargets->targets) {
			Eigen::Vector3d navTargetPos(target.x, target.y, target.z);
			auto newTarget = std::make_shared<PRM::Node>(navTargetPos);
			newTarget->isNavTarget = true;
			this->navTargetVec_.push_back(newTarget);
		}

		if (!navTargets->targets.empty()) {
			cout << "[FGDRM]: " << navTargets->targets.size() << " navigation targets received from target array." << endl;
		}
	}

	visualization_msgs::MarkerArray FGDRM::buildRoadmapMarkers(){  
		visualization_msgs::MarkerArray roadmapMarkers;

		// PRM nodes and edges
		int countPointNum = 0;
		int countEdgeNum = 0;
		int countVoxelNumText = 0;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			// std::shared_ptr<PRM::Node> n = this->prmNodeVec_[i];

			// Node point
			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "overall/prm_point";
			point.id = countPointNum;
			point.type = visualization_msgs::Marker::SPHERE;
			point.action = visualization_msgs::Marker::ADD;
			point.pose.position.x = n->pos(0);
			point.pose.position.y = n->pos(1);
			point.pose.position.z = n->pos(2);
			point.lifetime = ros::Duration(0.5); //5
			point.scale.x = 0.08;
			point.scale.y = 0.08;
			point.scale.z = 0.08;
			point.color.a = 1.0;
			point.color.r = 1.0;
			point.color.g = 1.0;
			point.color.b = 0.0;
			++countPointNum;
			roadmapMarkers.markers.push_back(point);

			// number of voxels for each node
			visualization_msgs::Marker voxelNumText;
			voxelNumText.ns = "overall/num_voxel_text";
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
			voxelNumText.lifetime = ros::Duration(0.5); //5
			++countVoxelNumText;
			roadmapMarkers.markers.push_back(voxelNumText);

			// Edges 
			visualization_msgs::Marker line;
			line.ns = "overall/edge";
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
				line.scale.x = 0.04;
				line.scale.y = 0.04;
				line.scale.z = 0.04;
				line.color.r = 1.0;
				line.color.g = 1.0;
				line.color.b = 0.0;
				line.color.a = 1.0;
				line.lifetime = ros::Duration(0.5); //5
				++countEdgeNum;
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

		int countGoalCandidateNum = 0;
		for (size_t i=0; i<this->goalCandidates_.size(); ++i){
			std::shared_ptr<PRM::Node> n = this->goalCandidates_[i];

			// Goal candidates
			visualization_msgs::Marker goalCandidatePoint;
			goalCandidatePoint.ns = "overall/frontier_point";
			goalCandidatePoint.header.frame_id = "map";
			goalCandidatePoint.header.stamp = ros::Time::now();
			goalCandidatePoint.id = countGoalCandidateNum;
			goalCandidatePoint.type = visualization_msgs::Marker::SPHERE;
			goalCandidatePoint.action = visualization_msgs::Marker::ADD;
			goalCandidatePoint.pose.position.x = n->pos(0);
			goalCandidatePoint.pose.position.y = n->pos(1);
			goalCandidatePoint.pose.position.z = n->pos(2);
			goalCandidatePoint.lifetime = ros::Duration(0.5); //5			
			goalCandidatePoint.scale.x = 0.2;
			goalCandidatePoint.scale.y = 0.2;
			goalCandidatePoint.scale.z = 0.2;
			goalCandidatePoint.color.a = 1.0;
			goalCandidatePoint.color.r = 0.0;
			goalCandidatePoint.color.g = 0.5;
			goalCandidatePoint.color.b = 1.0;
			++countGoalCandidateNum;
			roadmapMarkers.markers.push_back(goalCandidatePoint);
		}

		return roadmapMarkers;
	}

	void FGDRM::publishCandidatePaths(){
		visualization_msgs::MarkerArray candidatePathMarkers;
		int countNodeNum = 0;
		int countLineNum = 0;
		for (std::vector<std::shared_ptr<PRM::Node>>& path : this->candidatePaths_){
			for (size_t i=0; i<path.size(); ++i){
				std::shared_ptr<PRM::Node> n = path[i];
				visualization_msgs::Marker point;
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "candidate_path_node";
				point.id = countNodeNum;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = n->pos(0);
				point.pose.position.y = n->pos(1);
				point.pose.position.z = n->pos(2);
				point.lifetime = ros::Duration(0.1);
				point.scale.x = 0.15;
				point.scale.y = 0.15;
				point.scale.z = 0.15;
				point.color.a = 1.0;
				point.color.r = 1.0;
				point.color.g = 1.0;
				point.color.b = 0.0;
				++countNodeNum;
				candidatePathMarkers.markers.push_back(point);

				if (i<path.size()-1){
					std::shared_ptr<PRM::Node> nNext = path[i+1];
					visualization_msgs::Marker line;
					line.ns = "candidate_path";
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
					line.color.g = 1.0;
					line.color.b = 1.0;
					line.color.a = 1.0;
					line.lifetime = ros::Duration(0.5);
					++countLineNum;
					candidatePathMarkers.markers.push_back(line);				
				}
			}
		}
		this->candidatePathPub_.publish(candidatePathMarkers);		
	}
	
	void FGDRM::publishBestPath(){
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

	void FGDRM::publishFrontier(){
		visualization_msgs::MarkerArray frontierMarkers;
		int frontierRangeCount = 0;
		for (int i=0; i<int(this->frontierPointPairs_.size()); ++i){
			visualization_msgs::Marker range;

			Eigen::Vector3d p = this->frontierPointPairs_[i].first;
			double dist = this->frontierPointPairs_[i].second;

			range.header.frame_id = "map";
			range.header.stamp = ros::Time::now();
			range.ns = "frontier range";
			range.id = frontierRangeCount;
			range.type = visualization_msgs::Marker::SPHERE;
			range.action = visualization_msgs::Marker::ADD;
			range.pose.position.x = p(0);
			range.pose.position.y = p(1);
			range.pose.position.z = p(2);
			range.lifetime = ros::Duration(0.5);
			range.scale.x = dist;
			range.scale.y = dist;
			range.scale.z = 0.1;
			range.color.a = 0.4;
			range.color.r = 0.0;
			range.color.g = 0.0;
			range.color.b = 1.0;
			++frontierRangeCount;
			frontierMarkers.markers.push_back(range);			
		}
		this->frontierVisPub_.publish(frontierMarkers);
	}

	visualization_msgs::MarkerArray FGDRM::buildFrontierRoadmapMarkers(){  
		visualization_msgs::MarkerArray roadmapMarkers;

		int countPointNum = 0;
		int countEdgeNum = 0;
		int countVoxelNumText = 0;
		int countPathCostText = 0;

		// Iterate through goal candidates instead of all nodes
		for (std::shared_ptr<PRM::Node> n : this->goalCandidates_){
			// Node point with different namespace
			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "frontier_guide/prm_point";  // Changed namespace
			point.id = countPointNum;
			point.type = visualization_msgs::Marker::SPHERE;
			point.action = visualization_msgs::Marker::ADD;
			point.pose.position.x = n->pos(0);
			point.pose.position.y = n->pos(1);
			point.pose.position.z = n->pos(2);
			point.lifetime = ros::Duration(0.2);
			point.scale.x = 0.1;
			point.scale.y = 0.1;
			point.scale.z = 0.1;
			point.color.a = 1.0;
			point.color.r = 0.0;  // Different color from overall roadmap
			point.color.g = 1.0;
			point.color.b = 1.0;
			++countPointNum;
			roadmapMarkers.markers.push_back(point);

			// Number of voxels text with different namespace
			visualization_msgs::Marker voxelNumText;
			voxelNumText.ns = "frontier_guide/num_voxel_text";  // Changed namespace
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
			voxelNumText.lifetime = ros::Duration(0.2);
			++countVoxelNumText;
			roadmapMarkers.markers.push_back(voxelNumText);

			// // Text indicating the navigation target
			// if(n->isNavTarget) {
			// 	visualization_msgs::Marker isNavTargetText;
			// 	isNavTargetText.ns = "frontier_guide/is_nav_target";  
			// 	isNavTargetText.header.frame_id = "map";
			// 	isNavTargetText.id = countPointNum - 1;
			// 	isNavTargetText.header.stamp = ros::Time::now();
			// 	isNavTargetText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			// 	isNavTargetText.action = visualization_msgs::Marker::ADD;
			// 	isNavTargetText.pose.position.x = n->pos(0);
			// 	isNavTargetText.pose.position.y = n->pos(1);
			// 	isNavTargetText.pose.position.z = n->pos(2);
			// 	isNavTargetText.text = "Nav Target";
			// 	isNavTargetText.lifetime = ros::Duration(0.2);
			// 	roadmapMarkers.markers.push_back(isNavTargetText);
			// }

			 // Add path cost text marker
			visualization_msgs::Marker pathCostText;
			pathCostText.ns = "frontier_guide/path_cost_text";
			pathCostText.header.frame_id = "map";
			pathCostText.id = countPathCostText;  
			pathCostText.header.stamp = ros::Time::now();
			pathCostText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			pathCostText.action = visualization_msgs::Marker::ADD;
			pathCostText.pose.position.x = n->pos(0);
			pathCostText.pose.position.y = n->pos(1);
			pathCostText.pose.position.z = n->pos(2)-0.1;
			pathCostText.scale.x = 0.2;
			pathCostText.scale.y = 0.2;
			pathCostText.scale.z = 0.2;
			pathCostText.color.a = 1.0;
			if (n->pathCost != std::numeric_limits<double>::infinity()) {
				pathCostText.text = "cost:" + std::to_string(n->pathCost);
			} else {
				pathCostText.text = "cost:inf";
			}
			pathCostText.lifetime = ros::Duration(0.2);
			++countPathCostText;
			roadmapMarkers.markers.push_back(pathCostText);

			// Edges with different namespace
			visualization_msgs::Marker line;
			line.ns = "frontier_guide/edge";  // Changed namespace
			line.header.frame_id = "map";
			line.type = visualization_msgs::Marker::LINE_LIST;
			line.header.stamp = ros::Time::now();
			
			// Only draw edges between frontier nodes
			for (std::shared_ptr<PRM::Node> adjNode : n->adjNodes){
				// Check if adjacent node is also a frontier node
				if (std::find(this->goalCandidates_.begin(), this->goalCandidates_.end(), adjNode) != this->goalCandidates_.end()){
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
					line.scale.x = 0.04;
					line.scale.y = 0.04;
					line.scale.z = 0.04;
					line.color.r = 0.0;  // Different color from overall roadmap
					line.color.g = 1.0;
					line.color.b = 1.0;
					line.color.a = 1.0;
					line.lifetime = ros::Duration(0.2);
					++countEdgeNum;
				}
			}
			// add the node itself if it has no adjacent nodes or all adjacent nodes are not frontier nodes
			if(n->adjNodes.empty() or line.points.empty()) {
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
}