#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <falco_planner/SetRobotPose.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string robot_namespace;
string ns_prefix;
string frame_id_prefix;

double realtimeFactor = 1.0;
double windCoeff = 0.05;
double maxRollPitchRate = 20.0;
double rollPitchSmoothRate = 0.1;
double sensorPitch = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 1.0;

float vehicleVelX = 0;
float vehicleVelY = 0;
float vehicleVelZ = 0;
float vehicleVelXG = 0;
float vehicleVelYG = 0;
float vehicleVelZG = 0;

float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleRollCmd = 0;
float vehiclePitchCmd = 0;
float vehicleYawRate = 0;

ros::ServiceClient setModelState;
ros::Publisher pubModelState;
ros::Publisher pubVehicleOdom;
nav_msgs::Odometry odomData;

void controlHandler(const geometry_msgs::TwistStamped::ConstPtr& controlCmd)
{
  vehicleRollCmd = controlCmd->twist.linear.x;
  vehiclePitchCmd = controlCmd->twist.linear.y;
  vehicleYawRate = controlCmd->twist.angular.z;
  vehicleVelZG = controlCmd->twist.linear.z;
}

bool setRobotPoseServiceHandler(falco_planner::SetRobotPose::Request& req, falco_planner::SetRobotPose::Response& resp)
{
  vehicleX = req.robotPose.pose.position.x;
  vehicleY = req.robotPose.pose.position.y;
  vehicleZ = req.robotPose.pose.position.z;

  vehicleRoll = 0;
  vehiclePitch = 0;
  vehicleYaw = tf::getYaw(req.robotPose.pose.orientation);

  vehicleVelX = 0;
  vehicleVelY = 0;
  vehicleVelZ = 0;
  vehicleVelXG = 0;
  vehicleVelYG = 0;
  vehicleVelZG = 0;

  vehicleRollCmd = 0;
  vehiclePitchCmd = 0;
  vehicleYawRate = 0;

  ros::NodeHandle nh;
  ros::ServiceClient setModelState = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  // pubModelState.publish(req.robotPose);
  ros::service::waitForService("/gazebo/set_model_state");
  gazebo_msgs::SetModelState srv;
  srv.request.model_state = req.robotPose;
  if (setModelState.call(srv)) {
      printf("\n\033[1;32m**************************************************************************************************\033[0m\n");
      printf("\033[1;32m[Environment]: Reset Robot Pose Service Succeeded. New Episode Starts.\033[0m\n");
      printf("\033[1;32m**************************************************************************************************\033[0m\n");
      // ROS_INFO("Reset Robot Pose Service Succeeded");
  } else {
      ROS_ERROR("Reset Robot Pose Service Failed");
  }
  
  // printf("\n[Vehicle Simulator]: Successfully reset quadcopter pose!\n");

  odomData.header.frame_id = "map"; 
  odomData.child_frame_id = frame_id_prefix + "base_link";  
  odomData.header.stamp = ros::Time::now();
  odomData.pose.pose.orientation = req.robotPose.pose.orientation;
  odomData.pose.pose.position.x = vehicleX;
  odomData.pose.pose.position.y = vehicleY;
  odomData.pose.pose.position.z = vehicleZ;
  odomData.twist.twist.angular.x = 0;
  odomData.twist.twist.angular.y = 0;
  odomData.twist.twist.angular.z = 0;
  odomData.twist.twist.linear.x = 0;
  odomData.twist.twist.linear.y = 0;
  odomData.twist.twist.linear.z = 0;
  pubVehicleOdom.publish(odomData);  
  
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicle_simulator");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("realtimeFactor", realtimeFactor);
  nhPrivate.getParam("windCoeff", windCoeff);
  nhPrivate.getParam("maxRollPitchRate", maxRollPitchRate);
  nhPrivate.getParam("rollPitchSmoothRate", rollPitchSmoothRate);
  nhPrivate.getParam("sensorPitch", sensorPitch);
  nhPrivate.getParam("vehicleX", vehicleX);
  nhPrivate.getParam("vehicleY", vehicleY);
  nhPrivate.getParam("vehicleZ", vehicleZ);
  nhPrivate.getParam("vehicleYaw", vehicleYaw);
  nhPrivate.param<std::string>("robot_namespace", robot_namespace, "");

  ns_prefix = robot_namespace.empty() ? "" : "/" + robot_namespace;  // namespace to topics
  frame_id_prefix = robot_namespace.empty() ? "" : robot_namespace + "/";  // frame ID prefix

  ros::Subscriber subControl = nh.subscribe<geometry_msgs::TwistStamped> 
    (ns_prefix + "/falco_planner/attitude_control", 5, controlHandler);

  ros::ServiceServer setRobotPoseServer = nh.advertiseService
    (ns_prefix + "/falco_planner/set_robot_pose", setRobotPoseServiceHandler);

  ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry> 
    (ns_prefix + "/falco_planner/state_estimation", 5);

  ros::Publisher pubModelState = nh.advertise<gazebo_msgs::ModelState> 
    ("/gazebo/set_model_state", 5);  // Keep global for Gazebo

  // nav_msgs::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = frame_id_prefix + "base_link";  // this name should change to the frame of your_robot

  gazebo_msgs::ModelState robotState;
  robotState.model_name = robot_namespace.empty() ? "quadcopter" : robot_namespace;
  // ros::Publisher pubVehicleState = nh.advertise<geometry_msgs::PoseStamped> ("/CERLAB/quadcopter/setpoint_pose", 5);
  // geometry_msgs::PoseStamped robotState;
  // robotState.header.frame_id = "map";

  printf("\nSimulation started.\n\n");

  ros::Rate rate(200 * realtimeFactor);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;

    if (vehicleRollCmd - vehicleRoll > maxRollPitchRate / 200.0) vehicleRoll += maxRollPitchRate / 200.0;
    else if (vehicleRollCmd - vehicleRoll < -maxRollPitchRate / 200.0) vehicleRoll -= maxRollPitchRate / 200.0;
    else vehicleRoll = vehicleRollCmd;
    vehicleRoll = rollPitchSmoothRate * vehicleRoll + (1.0 - rollPitchSmoothRate) * vehicleRecRoll;

    if (vehiclePitchCmd - vehiclePitch > maxRollPitchRate / 200.0) vehiclePitch += maxRollPitchRate / 200.0;
    else if (vehiclePitchCmd - vehiclePitch < -maxRollPitchRate / 200.0) vehiclePitch -= maxRollPitchRate / 200.0;
    else vehiclePitch = vehiclePitchCmd;
    vehiclePitch = rollPitchSmoothRate * vehiclePitch + (1.0 - rollPitchSmoothRate) * vehicleRecPitch;

    float vehicleAccX = 9.8 * tan(vehiclePitch);
    float vehicleAccY = -9.8 * tan(vehicleRoll) / cos(vehiclePitch);

    if (vehicleVelXG < 0) vehicleVelXG += windCoeff * vehicleVelXG * vehicleVelXG  / 200.0;
    else vehicleVelXG -= windCoeff * vehicleVelXG * vehicleVelXG  / 200.0;
    if (vehicleVelYG < 0) vehicleVelYG += windCoeff * vehicleVelYG * vehicleVelYG  / 200.0;
    else vehicleVelYG -= windCoeff * vehicleVelYG * vehicleVelYG  / 200.0;

    vehicleVelXG += (vehicleAccX * cos(vehicleYaw) - vehicleAccY * sin(vehicleYaw)) / 200.0;
    vehicleVelYG += (vehicleAccX * sin(vehicleYaw) + vehicleAccY * cos(vehicleYaw)) / 200.0;

    float velX1 = vehicleVelXG * cos(vehicleYaw) + vehicleVelYG * sin(vehicleYaw);
    float velY1 = -vehicleVelXG * sin(vehicleYaw) + vehicleVelYG * cos(vehicleYaw);
    float velZ1 = vehicleVelZG;

    float velX2 = velX1 * cos(vehiclePitch) - velZ1 * sin(vehiclePitch);
    float velY2 = velY1;
    float velZ2 = velX1 * sin(vehiclePitch) + velZ1 * cos(vehiclePitch);

    vehicleVelX = velX2;
    vehicleVelY = velY2 * cos(vehicleRoll) + velZ2 * sin(vehicleRoll);
    vehicleVelZ = -velY2 * sin(vehicleRoll) + velZ2 * cos(vehicleRoll);

    vehicleX += vehicleVelXG / 200.0;
    vehicleY += vehicleVelYG / 200.0;
    vehicleZ += vehicleVelZG / 200.0;
    vehicleYaw += vehicleYawRate / 200.0;

    ros::Time timeNow = ros::Time::now();

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, vehiclePitch, vehicleYaw);

    // publish 200Hz odometry messages
    odomData.header.stamp = timeNow;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);
    odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch);
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleVelX;
    odomData.twist.twist.linear.y = vehicleVelY;
    odomData.twist.twist.linear.z = vehicleVelZ;
    pubVehicleOdom.publish(odomData);

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, sensorPitch + vehiclePitch, vehicleYaw);
    
    // robotState.header.stamp = timeNow;
    robotState.pose.orientation = geoQuat;
    robotState.pose.position.x = vehicleX;
    robotState.pose.position.y = vehicleY;
    robotState.pose.position.z = vehicleZ;
    pubModelState.publish(robotState);
    // pubVehicleState.publish(robotState);

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
