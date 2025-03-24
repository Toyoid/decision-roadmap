#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import math

odom_ = None
def odom_callback(msg):
    global odom_
    odom_ = msg


if __name__ == '__main__':
    rospy.init_node('quaternion_trans_test_node')
    odom_sub = rospy.Subscriber("/CERLAB/quadcopter/odom", Odometry, odom_callback)
    rate = rospy.Rate(10)

    while not odom_:
        rospy.loginfo("Waiting for odometry data...")
        rospy.sleep(0.5)

    while not rospy.is_shutdown():
        quat = [odom_.pose.pose.orientation.x,
                odom_.pose.pose.orientation.y,
                odom_.pose.pose.orientation.z,
                odom_.pose.pose.orientation.w]
        siny_cosp = 2. * (quat[0] * quat[1] + quat[2] * quat[3])
        cosy_cosp = 1. - 2. * (quat[1] ** 2 + quat[2] ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_degree = math.degrees(yaw)

        print("----------------------------")
        print(f"YAW: {yaw}")
        print(f"YAW-Degree: {yaw_degree}")
        print("----------------------------")
        print(" ")

        rate.sleep()

