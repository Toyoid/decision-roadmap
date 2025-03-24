#include <global_planner/drm_base.h>

namespace globalPlanner {

DRMBase::DRMBase(const ros::NodeHandle& nh, const std::string& robot_ns) : nh_(nh)
    , robot_ns_(robot_ns)
    , vel_(1.0)
    , angularVel_(1.0)
    , odomReceived_(false)
    , currGoalReceived_(false)
    , resettingRLEnv_(false) {
    this->single_robot_ns_ = "CERLAB/quadcopter";
    this->rostopic_prefix_ = this->robot_ns_ == this->single_robot_ns_? "": "/" + this->robot_ns_;
}

void DRMBase::setMap(const std::shared_ptr<mapManager::occMap>& map) {
    this->map_ = map;
}

void DRMBase::loadVelocity(double vel, double angularVel) {
    this->vel_ = vel;
    this->angularVel_ = angularVel;
}

nav_msgs::Path DRMBase::getBestPath(){
    nav_msgs::Path bestPath;
    for (int i=0; i<int(this->bestPath_.size()); ++i){
        std::shared_ptr<PRM::Node> currNode = this->bestPath_[i];
        geometry_msgs::PoseStamped p;
        p.pose.position.x = currNode->pos(0);
        p.pose.position.y = currNode->pos(1);
        p.pose.position.z = currNode->pos(2);
        if (i < int(this->bestPath_.size())-1){
            std::shared_ptr<PRM::Node> nextNode = this->bestPath_[i+1];
            Eigen::Vector3d diff = nextNode->pos - currNode->pos;
            double angle = atan2(diff(1), diff(0));
            p.pose.orientation = globalPlanner::quaternion_from_rpy(0, 0, angle);
        }
        bestPath.poses.push_back(p);
    }
    
    // get the best yaw for the last pose
    double bestYaw = this->bestPath_.back()->getBestYaw();
    bestPath.poses.back().pose.orientation = globalPlanner::quaternion_from_rpy(0, 0, bestYaw);

    return bestPath;
}

bool DRMBase::isPosValid(const Eigen::Vector3d& p){
    for (double x=p(0)-this->safeDistXY_; x<=p(0)+this->safeDistXY_; x+=this->map_->getRes()){
        for (double y=p(1)-this->safeDistXY_; y<=p(1)+this->safeDistXY_; y+=this->map_->getRes()){
            for (double z=p(2)-this->safeDistZ_; z<=p(2)+this->safeDistZ_; z+=this->map_->getRes()){
                if (this->safeDistCheckUnknown_){
                    if (not this->map_->isInflatedFree(Eigen::Vector3d (x, y, z))){
                        return false;
                    }
                }
                else{
                    if (this->map_->isInflatedOccupied(Eigen::Vector3d (x, y, z))){
                        return false;
                    }
                }
            }
        }
    }
    return true;			
}

bool DRMBase::isPosValid(const Eigen::Vector3d& p, double safeDistXY, double safeDistZ){
    for (double x=p(0)-safeDistXY; x<=p(0)+safeDistXY; x+=this->map_->getRes()){
        for (double y=p(1)-safeDistXY; y<=p(1)+safeDistXY; y+=this->map_->getRes()){
            for (double z=p(2)-safeDistZ; z<=p(2)+safeDistZ; z+=this->map_->getRes()){
                if (this->safeDistCheckUnknown_){
                    if (not this->map_->isInflatedFree(Eigen::Vector3d (x, y, z))){
                        return false;
                    }
                }
                else{
                    if (this->map_->isInflatedOccupied(Eigen::Vector3d (x, y, z))){
                        return false;
                    }
                }
            }
        }
    }
    return true;		
}

bool DRMBase::isNodeRequireUpdate(std::shared_ptr<PRM::Node> n, std::vector<std::shared_ptr<PRM::Node>> path, double& leastDistance){
    double distanceThresh = 2;
    leastDistance = std::numeric_limits<double>::max();
    for (std::shared_ptr<PRM::Node>& waypoint: path){
        double currentDistance = (n->pos - waypoint->pos).norm();
        if (currentDistance < leastDistance){
            leastDistance = currentDistance;
        }
    }
    if (leastDistance <= distanceThresh){
        return true;
    }
    else{
        return false;	
    }
}

std::shared_ptr<PRM::Node> DRMBase::randomConfigBBox(const Eigen::Vector3d& minRegion, const Eigen::Vector3d& maxRegion){
    Eigen::Vector3d mapMinRegion, mapMaxRegion, minSampleRegion, maxSampleRegion;
    this->map_->getCurrMapRange(mapMinRegion, mapMaxRegion);
    // cout << "current map range is: " << mapMinRegion.transpose() << ", " << mapMaxRegion.transpose() << endl;
    minSampleRegion(0) = std::max(mapMinRegion(0), minRegion(0));
    minSampleRegion(1) = std::max(mapMinRegion(1), minRegion(1));
    minSampleRegion(2) = std::max(mapMinRegion(2), minRegion(2));
    maxSampleRegion(0) = std::min(mapMaxRegion(0), maxRegion(0));
    maxSampleRegion(1) = std::min(mapMaxRegion(1), maxRegion(1));
    maxSampleRegion(2) = std::min(mapMaxRegion(2), maxRegion(2));

    minSampleRegion(0) = std::max(minSampleRegion(0), this->globalRegionMin_(0));
    minSampleRegion(1) = std::max(minSampleRegion(1), this->globalRegionMin_(1));
    minSampleRegion(2) = std::max(minSampleRegion(2), this->globalRegionMin_(2));
    maxSampleRegion(0) = std::min(maxSampleRegion(0), this->globalRegionMax_(0));
    maxSampleRegion(1) = std::min(maxSampleRegion(1), this->globalRegionMax_(1));
    maxSampleRegion(2) = std::min(maxSampleRegion(2), this->globalRegionMax_(2));


    bool valid = false;
    Eigen::Vector3d p;
    while (valid == false){	
        p(0) = globalPlanner::randomNumber(minSampleRegion(0), maxSampleRegion(0));
        p(1) = globalPlanner::randomNumber(minSampleRegion(1), maxSampleRegion(1));
        p(2) = globalPlanner::randomNumber(minSampleRegion(2), maxSampleRegion(2));

        valid = this->isPosValid(p, this->safeDistXY_, this->safeDistZ_);

        // valid = this->map_->isInflatedFree(p);
    }

    std::shared_ptr<PRM::Node> newNode (new PRM::Node(p));
    return newNode;
}

int DRMBase::calculateUnknown(const shared_ptr<PRM::Node>& n, std::unordered_map<double, int>& yawNumVoxels){
    for (double yaw : this->yaws_){
        yawNumVoxels[yaw] = 0;
    }
    // Position:
    Eigen::Vector3d p = n->pos;

    double zRange = this->dmax_ * tan(this->verticalFOV_/2.0);
    int countTotalUnknown = 0;
    for (double z = p(2) - zRange; z <= p(2) + zRange; z += this->map_->getRes()){
        for (double y = p(1) - this->dmax_; y <= p(1)+ this->dmax_; y += this->map_->getRes()){
            for (double x = p(0) - this->dmax_; x <= p(0) + this->dmax_; x += this->map_->getRes()){
                Eigen::Vector3d nodePoint (x, y, z);
                if (nodePoint(0) < this->globalRegionMin_(0) or nodePoint(0) > this->globalRegionMax_(0) or
                    nodePoint(1) < this->globalRegionMin_(1) or nodePoint(1) > this->globalRegionMax_(1) or
                    nodePoint(2) < this->globalRegionMin_(2) or nodePoint(2) > this->globalRegionMax_(2)){
                    // not in global range
                    continue;
                }

                if (this->map_->isUnknown(nodePoint) and not this->map_->isInflatedOccupied(nodePoint)){
                    if (this->sensorFOVCondition(nodePoint, p)){
                        ++countTotalUnknown;
                        for (double yaw: this->yaws_){
                            Eigen::Vector3d yawDirection (cos(yaw), sin(yaw), 0);
                            Eigen::Vector3d direction = nodePoint - p;
                            Eigen::Vector3d face (direction(0), direction(1), 0);
                            double angleToYaw = angleBetweenVectors(face, yawDirection);
                            if (angleToYaw <= this->horizontalFOV_/2){
                                yawNumVoxels[yaw] += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    return countTotalUnknown;
}

double DRMBase::calculatePathLength(const std::vector<shared_ptr<PRM::Node>>& path){
    int idx1 = 0;
    double length = 0;
    for (size_t idx2=1; idx2<=path.size()-1; ++idx2){
        length += (path[idx2]->pos - path[idx1]->pos).norm();
        ++idx1;
    }
    return length;
}

void DRMBase::shortcutPath(const std::vector<std::shared_ptr<PRM::Node>>& path, std::vector<std::shared_ptr<PRM::Node>>& pathSc){
    size_t ptr1 = 0; size_t ptr2 = 2;
    pathSc.push_back(path[ptr1]);

    if (path.size() == 1){
        return;
    }

    if (path.size() == 2){
        pathSc.push_back(path[1]);
        return;
    }

    while (ros::ok()){
        if (ptr2 > path.size()-1){
            break;
        }
        std::shared_ptr<PRM::Node> p1 = path[ptr1];
        std::shared_ptr<PRM::Node> p2 = path[ptr2];
        Eigen::Vector3d pos1 = p1->pos;
        Eigen::Vector3d pos2 = p2->pos;
        bool lineValidCheck;
        // lineValidCheck = not this->map_->isInflatedOccupiedLine(pos1, pos2);
        lineValidCheck = this->map_->isInflatedFreeLine(pos1, pos2);
        // double maxDistance = std::numeric_limits<double>::max();
        // double maxDistance = 3.0;
        // if (lineValidCheck and (pos1 - pos2).norm() <= maxDistance){
        if (lineValidCheck){
            if (ptr2 == path.size()-1){
                pathSc.push_back(p2);
                break;
            }
            ++ptr2;
        }
        else{
            pathSc.push_back(path[ptr2-1]);
            if (ptr2 == path.size()-1){
                pathSc.push_back(p2);
                break;
            }
            ptr1 = ptr2-1;
            ptr2 = ptr1+2;
        }
    }		
}

int DRMBase::weightedSample(const std::vector<double>& weights){
    double total = std::accumulate(weights.begin(), weights.end(), 0.0);
    std::vector<double> normalizedWeights;

     for (const double weight : weights){
         normalizedWeights.push_back(weight/total);
     }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<int> distribution(normalizedWeights.begin(), normalizedWeights.end());
    return distribution(gen);
}


std::shared_ptr<PRM::Node> DRMBase::sampleFrontierPoint(const std::vector<double>& sampleWeights){
    // choose the frontier region (random sample by frontier area) 
    int idx = weightedSample(sampleWeights);

    // sample a frontier point in the region
    Eigen::Vector3d frontierCenter = this->frontierPointPairs_[idx].first;
    double frontierSize = this->frontierPointPairs_[idx].second;
    double xmin = std::max(frontierCenter(0) - frontierSize/sqrt(2), this->globalRegionMin_(0));
    double xmax = std::min(frontierCenter(0) + frontierSize/sqrt(2), this->globalRegionMax_(0));
    double ymin = std::max(frontierCenter(1) - frontierSize/sqrt(2), this->globalRegionMin_(1));
    double ymax = std::min(frontierCenter(1) + frontierSize/sqrt(2), this->globalRegionMax_(1));
    double zmin = frontierCenter(2);
    double zmax = frontierCenter(2);
    Eigen::Vector3d frontierPoint;
    frontierPoint(0) = globalPlanner::randomNumber(xmin, xmax);
    frontierPoint(1) = globalPlanner::randomNumber(ymin, ymax);
    frontierPoint(2) = globalPlanner::randomNumber(zmin, zmax);
    std::shared_ptr<PRM::Node> frontierNode (new PRM::Node(frontierPoint));
    return frontierNode;
}

std::shared_ptr<PRM::Node> DRMBase::extendNode(const std::shared_ptr<PRM::Node>& n, const std::shared_ptr<PRM::Node>& target){
    double extendDist = randomNumber(this->distThresh_, this->maxConnectDist_);
    Eigen::Vector3d p = n->pos + (target->pos - n->pos)/(target->pos - n->pos).norm() * extendDist;
    p(0) = std::max(this->globalRegionMin_(0), std::min(p(0), this->globalRegionMax_(0)));
    p(1) = std::max(this->globalRegionMin_(1), std::min(p(1), this->globalRegionMax_(1)));
    p(2) = std::max(this->globalRegionMin_(2), std::min(p(2), this->globalRegionMax_(2)));
    std::shared_ptr<PRM::Node> extendedNode (new PRM::Node(p));
    return extendedNode;
}

Point3D DRMBase::projectNavWaypoint(const Point3D& nav_waypoint, const Point3D& last_waypoint) {
    const float kEpsilon = 1e-5;
    const bool is_momentum = (last_waypoint - nav_waypoint).norm() < kEpsilon ? true : false; // momentum heading if same goal
    Point3D waypoint = nav_waypoint;
    const float waypoint_project_dist = 1.6;  // 5.0 in far-planner
    const Point3D robot_pos(this->position_);
    const Point3D diff_p = nav_waypoint - robot_pos;
    Point3D new_heading;
    if (is_momentum && this->navHeading_.norm() > kEpsilon) {
        const float hdist = waypoint_project_dist / 2.0f;
        const float ratio = std::min(hdist, diff_p.norm()) / hdist;
        new_heading = diff_p.normalize() * ratio + this->navHeading_ * (1.0f - ratio);
    } else {
        new_heading = diff_p.normalize();
    }
    if (this->navHeading_.norm() > kEpsilon && new_heading.norm_dot(this->navHeading_) < 0.0f) { // negative direction reproject
        Point3D temp_heading(this->navHeading_.y, -this->navHeading_.x, this->navHeading_.z);
        if (temp_heading.norm_dot(new_heading) < 0.0f) {
        temp_heading.x = -temp_heading.x, temp_heading.y = -temp_heading.y;
        }
        new_heading = temp_heading;
    }
    this->navHeading_ = new_heading.normalize();
    if (diff_p.norm() < waypoint_project_dist) {
        waypoint = nav_waypoint + this->navHeading_ * (waypoint_project_dist - diff_p.norm());
    }
    return waypoint;
}

bool DRMBase::sensorRangeCondition(const shared_ptr<PRM::Node>& n1, const shared_ptr<PRM::Node>& n2){
    Eigen::Vector3d direction = n2->pos - n1->pos;
    Eigen::Vector3d projection;
    projection(0) = direction.x();
    projection(1) = direction.y();
    projection(2) = 0;
    double verticalAngle = angleBetweenVectors(direction, projection);
    if (verticalAngle < this->verticalFOV_/2){
        return true;
    }
    else{
        return false;
    }
}

// create sensor check for 
// vert, horz FOV, collision, and sensor distance range
// for yaw angles in vector3d:  cos(yaw), sin(yaw), 0
// horz angle between yaw angle vector and direction (x y 0) vector for horz FOV
// Vert angle yaw angle vector and yaw angle vector (c s z) z is direction.z()
bool DRMBase::sensorFOVCondition(const Eigen::Vector3d& sample, const Eigen::Vector3d& pos){
    Eigen::Vector3d direction = sample - pos;
    double distance = direction.norm();
    if (distance > this->dmax_){
        return false;
    }
    bool hasCollision = this->map_->isInflatedOccupiedLine(sample, pos);
    if (hasCollision == true){
        return false;
    }
    return true;
}

} // namespace globalPlanner
