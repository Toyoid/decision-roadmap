#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

ros::Time lastStamp(0);
std::string robot_namespace = "";  // Global variable for robot namespace

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  static tf2_ros::TransformBroadcaster br;    
  geometry_msgs::TransformStamped transformStamped;
  
  // Get the current time and check if it differs from the last broadcast time
  ros::Time current_time = ros::Time::now();
  if (lastStamp != current_time) {
    transformStamped.header.stamp = current_time;
    lastStamp = current_time;
    transformStamped.header.frame_id = msg->header.frame_id;
	// Prefix robot_namespace to "base_link"
	transformStamped.child_frame_id = robot_namespace + "base_link";
    
    transformStamped.transform.translation.x = msg->pose.position.x;
    transformStamped.transform.translation.y = msg->pose.position.y;
    transformStamped.transform.translation.z = msg->pose.position.z;
    transformStamped.transform.rotation.x    = msg->pose.orientation.x;
    transformStamped.transform.rotation.y    = msg->pose.orientation.y;
    transformStamped.transform.rotation.z    = msg->pose.orientation.z;
    transformStamped.transform.rotation.w    = msg->pose.orientation.w;
    
    br.sendTransform(transformStamped);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "quadcopter_tf_broadcaster");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");  // private node handle for parameters

  // Read parameters from the launch file (or parameter server)
  std::string poseTopic;
  pnh.param("pose_topic", poseTopic, std::string("/CERLAB/quadcopter/pose"));
  pnh.param("robot_namespace", robot_namespace, std::string(""));

  ros::Subscriber sub = nh.subscribe(poseTopic, 1, &poseCallback);
  ros::spin();
  return 0;
}
