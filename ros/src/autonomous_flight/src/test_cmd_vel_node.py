#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import numpy as np


class FlightBase:
    def __init__(self):
        self.odom_ = None
        self.has_takeoff = False
        self.takeoff_height = 5.0

        self.odom_sub = rospy.Subscriber("/CERLAB/quadcopter/odom", Odometry, self._odom_callback)
        self.vel_cmd_pub = rospy.Publisher("/CERLAB/quadcopter/cmd_vel", TwistStamped, queue_size=10)
        self.drone_pose_pub = rospy.Publisher("/CERLAB/quadcopter/setpoint_pose", PoseStamped, queue_size=10) # subscribed by /gazebo
        self.set_model_pose = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

    def _odom_callback(self, msg):
        self.odom_ = msg

    def takeoff(self):
        rate = rospy.Rate(30)
        while self.odom_ is None:
            rospy.loginfo("Waiting for odometry data...")
            rospy.sleep(0.5)

        while not rospy.is_shutdown() and not self.has_takeoff and abs(
                self.odom_.pose.pose.position.z - self.takeoff_height) >= 0.1:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose = self.odom_.pose.pose
            ps.pose.position.z = self.takeoff_height
            self.drone_pose_pub.publish(ps)
            rate.sleep()

        rospy.loginfo("Takeoff finished!")
        self.has_takeoff = True

    def set_quadcopter_state(self, x, y, z, w):
        robot_msg = ModelState()
        robot_msg.model_name = 'quadcopter'
        robot_msg.pose.position.x = x
        robot_msg.pose.position.y = y
        robot_msg.pose.position.z = z
        robot_msg.pose.orientation.w = w
        rospy.wait_for_service('gazebo/set_model_state')
        try:
            resp = self.set_model_pose(robot_msg)
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose = robot_msg.pose
            self.drone_pose_pub.publish(ps)  # keep the drone hovering
        except rospy.ServiceException as e:
            print("Set Target Service Failed: %s" % e)
        rospy.loginfo("set new quadcopter state...")


if __name__ == '__main__':
    rospy.init_node('cmd_vel_test_node')
    flight_base = FlightBase()
    flight_base.takeoff()
    rate = rospy.Rate(30)

    # set_model_state test
    xy_positions = np.random.uniform(-5, 5, (5, 2))
    for i, pos in enumerate(xy_positions):
        rospy.sleep(3)
        flight_base.set_quadcopter_state(pos[0], pos[1], 2.0, 1.0)
        rospy.loginfo(f'Successfully set state {i}')

    # /cmd_vel test
    rospy.loginfo("Start moving from velocity commands...")
    while not rospy.is_shutdown() and flight_base.has_takeoff:
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "base_link"
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = 2.0
        twist_msg.twist.linear.y = 2.0
        twist_msg.twist.angular.z = 1.5
        flight_base.vel_cmd_pub.publish(twist_msg)

        rate.sleep()

