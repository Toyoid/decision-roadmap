/*
	FILE: cmd_vel_test_node.cpp
	---------------------------
	Simple /cmd_vel test for autonomous flight
*/

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry odom_;
bool hasTakeoff = false;
double takeoff_height = 5.0;


void odomCallBack(const nav_msgs::OdometryConstPtr& odom){
    odom_ = *odom;
}

void takeoff(ros::NodeHandle& nh){
    ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/CERLAB/quadcopter/setpoint_pose", 10);  // subscribed by /gazebo
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose = odom_.pose.pose;
    ps.pose.position.z = takeoff_height;
    ps.pose.orientation.w = 1.0;
    
    ros::Rate rate(30);
    while (ros::ok() and std::abs(odom_.pose.pose.position.z - takeoff_height) >= 0.1){
        posePub.publish(ps);
	    // std::cout << "[AutoFlight]: Publishing pose target: " << ps << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Takeoff finished!");
    hasTakeoff = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "cmd_vel_test_node");
    ros:: NodeHandle nh;

    ros::Rate rate(30);
    ros::Publisher velCmdPub = nh.advertise<geometry_msgs::TwistStamped>("/CERLAB/quadcopter/cmd_vel", 10);
    ros::Subscriber odomSub = nh.subscribe("CERLAB/quadcopter/odom", 10, &odomCallBack);

    // takeoff
    takeoff(nh);

    // track circle
    ROS_INFO("Start moving from velocity commands...");
    while(ros::ok() && hasTakeoff){
        ros::Time currTime = ros::Time::now();
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.frame_id = "base_link";
        twist_msg.header.stamp = ros::Time::now();
        twist_msg.twist.linear.x = 1.5;  // maximum reachable velocity: 2.0
        twist_msg.twist.linear.y = 0.0;  // maximum reachable velocity: 2.0
        // twist_msg.twist.linear.z = 80.0;  // it can be controlled to gradually reach any velocity value
        twist_msg.twist.angular.z = 10.0;  // maximum reachable velocity: 1.5

        velCmdPub.publish(twist_msg);

        // show velocity errors
        std::cout << "----------------------------------------------------------------" << std::endl;
        std::cout << "[Error of Linear-X]: " << odom_.twist.twist.linear.x << std::endl;
        std::cout << "[Error of Linear-Y]: " << odom_.twist.twist.linear.y << std::endl;
        std::cout << "[Error of Linear-Z]: " << odom_.twist.twist.linear.z << std::endl;
        std::cout << "[Error of Angular-Z]: " << odom_.twist.twist.angular.z << std::endl;
        std::cout << "----------------------------------------------------------------" << std::endl;
        std::cout << " " << std::endl;

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}