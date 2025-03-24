#include "waypoint_tool.h"

namespace rviz
{
WaypointTool::WaypointTool()
{
  shortcut_key_ = 'w';

  topic_property_ = new StringProperty("Topic", "waypoint", "The topic on which to publish navigation waypoints.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
  
  robot_property_ = new StringProperty("Robot", "", "Robot namespace (empty for single robot, 'quad0' or 'quad1' for multi-robot)",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

void WaypointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Waypoint");
  updateTopic();
  vehicle_z = 0;
}

void WaypointTool::updateTopic()
{
  robot_namespace_ = robot_property_->getStdString();
  std::string ns_prefix = robot_namespace_.empty() ? "" : "/" + robot_namespace_;

  sub_ = nh_.subscribe<nav_msgs::Odometry> (ns_prefix + "/falco_planner/state_estimation", 5, &WaypointTool::odomHandler, this);
  pub_ = nh_.advertise<geometry_msgs::PointStamped>(ns_prefix + "/falco_planner/way_point", 5);
  pub_joy_ = nh_.advertise<sensor_msgs::Joy>(ns_prefix + "/falco_planner/joy", 5);
}

void WaypointTool::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void WaypointTool::onPoseSet(double x, double y, double theta)
{
  sensor_msgs::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = ros::Time::now();
  joy.header.frame_id = robot_namespace_.empty() ? "waypoint_tool" : robot_namespace_ + "/waypoint_tool";
  pub_joy_.publish(joy);

  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.point.x = x;
  waypoint.point.y = y;
  waypoint.point.z = vehicle_z;

  pub_.publish(waypoint);
  usleep(10000);
  pub_.publish(waypoint);
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::WaypointTool, rviz::Tool)
