#include "goalpoint_tool.h"

namespace rviz
{
GoalPointTool::GoalPointTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new StringProperty("Topic", "goalpoint", "The topic on which to publish goal waypiont for dynamic route planner.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

void GoalPointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Goalpoint");
  updateTopic();
  vehicle_z = 0;
}

void GoalPointTool::updateTopic()
{
  sub_ = nh_.subscribe<nav_msgs::Odometry> ("/falco_planner/state_estimation", 5, &GoalPointTool::odomHandler, this);
  pub_ = nh_.advertise<decision_roadmap_agent::TargetArray>("/nav_target", 5);
  pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/falco_planner/joy", 5);
}

void GoalPointTool::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void GoalPointTool::onPoseSet(double x, double y, double theta)
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
  joy.header.frame_id = "goalpoint_tool";
  
  decision_roadmap_agent::TargetArray goal_array;
  goal_array.header.frame_id = "map";
  goal_array.header.stamp = ros::Time::now();
  geometry_msgs::Point goal_point;
  goal_point.x = x;
  goal_point.y = y;
  goal_point.z = vehicle_z;
  goal_array.targets.push_back(goal_point);

  pub_.publish(goal_array);
  usleep(10000);
  pub_.publish(goal_array);
  
  usleep(10000); // set to 1000000us (1s) on real robot
  pub_joy_.publish(joy);
}
}  // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::GoalPointTool, rviz::Tool)
