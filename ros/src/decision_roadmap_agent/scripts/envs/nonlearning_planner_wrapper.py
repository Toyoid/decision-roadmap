import math
import pickle
import numpy as np
import rospy
import os
from utils import _get_init_robot_goal, _euler_2_quat, _get_init_robot_goal_assist_nav

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from std_msgs.msg import Float32
from falco_planner.srv import SetRobotPose
from std_srvs.srv import SetBool, SetBoolResponse


class NonLearningPlannerWrapper:
    """
    Wrapper for Navigation using Non-learning Global Planners
    """
    def __init__(
        self,
        robot_namespace,
        is_assist_nav,
        episodic_max_period=250,
        goal_near_th=0.3,
        env_height_range=[0.2,2.5],
        map_name="maze_medium"
    ):
        self.robot_ns = robot_namespace
        self.is_assist_nav = is_assist_nav
        if self.is_assist_nav:
            self.init_robot_array, self.init_target_array = _get_init_robot_goal_assist_nav(map_name, is_assistance=False)
        else:
            self.init_robot_array, self.init_target_array = _get_init_robot_goal(
                map_name, 
                shuffle=True if robot_namespace else False
            )

        # Robot messages
        self.odom = None
        self.robot_odom_init = False
        self.other_agent_reset_init = False
        self.time_duration = None
        self.time_duration_init = False
        self.run_time = None
        self.runtime_init = False
        self.explored_volume = None
        self.explored_volume_init = False
        self.traveling_distance = None
        self.traveling_dist_init = False

        # Flags for assistance navigation
        self.other_agent_resetting = False
        self.other_reset_topic_init = False
        self.assist_episode_end = False
        self.self_agent_resetting = False
        self.other_agent_reset_sub = rospy.Subscriber("/assist_agent/reset_flag", Bool, self._other_agent_reset_callback)
        self.pub_self_reset_timer = rospy.Timer(rospy.Duration(0.1), self._pub_self_reset_callback)
        self.self_agent_reset_pub = rospy.Publisher("/human_agent/reset_flag", Bool, queue_size=5)

        # ROS
        self.rostopic_prefix = f"/{self.robot_ns}" if self.robot_ns else ""
        # Subscriber
        rospy.Subscriber(self.rostopic_prefix + "/falco_planner/state_estimation", Odometry, self._odom_callback)
        rospy.Subscriber(self.rostopic_prefix + "/data_recording/time_duration", Float32, self._time_duration_callback)
        # rospy.Subscriber(self.rostopic_prefix + "/data_recording/runtime", Float32, self._runtime_callback)
        rospy.Subscriber(self.rostopic_prefix + "/data_recording/explored_volume", Float32, self._explored_volume_callback)
        rospy.Subscriber(self.rostopic_prefix + "/data_recording/traveling_distance", Float32, self._traveling_dist_callback)
        # Publisher
        self.nav_target_pub = rospy.Publisher(self.rostopic_prefix + "/env/nav_target", PointStamped, queue_size=5)
        # publish joy for activating falco planner
        self.pub_joy = rospy.Publisher(self.rostopic_prefix + '/falco_planner/joy', Joy, queue_size=5)

        # Service
        self.pause_gazebo = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.set_model_pose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_roadmap = rospy.ServiceProxy(self.rostopic_prefix + '/dep/reset_roadmap', SetRobotPose)
        self.set_robot_pose = rospy.ServiceProxy(self.rostopic_prefix + '/falco_planner/set_robot_pose', SetRobotPose)
        self.init_rotate_scan = rospy.ServiceProxy(self.rostopic_prefix + '/falco_planner/init_rotate_scan_service', Empty)
        self.set_episode_end_srv = rospy.Service('/human_agent/set_episode_end_flag', SetBool, self._set_episode_end_callback)

        self.target_position = None
        self.goal_near_th = goal_near_th
        self.episodic_max_period = episodic_max_period
        self.env_height_range = env_height_range
        self.episodic_start_time = None
        self.episodic_start_dist = None
        self.avg_runtime = 0
        self.runtime_count = 0
        self.info = {
            "episodic_time_cost": 0,
            "episodic_dist_cost": 0,
            "episodic_avg_runtime": 0,
            "episodic_outcome": None,
            "outcome_statistic": {
                "success": 0,
                "collision": 0,
                "timeout": 0,
                "assistance_done": 0
            }
        }

        # Init Subscriber
        while not self.robot_odom_init:
            continue
        rospy.loginfo("Odom Subscriber Initialization Finished.")
        if self.is_assist_nav:
            while not (self.other_agent_reset_init or rospy.is_shutdown()):
                continue
            rospy.loginfo("Finish Assist Reset Flag Subscriber Init...")
        rospy.loginfo("Subscriber Initialization Finished!")

    def reset(self, ita):
        assert self.init_robot_array is not None
        assert self.init_target_array is not None
        assert ita < self.init_robot_array.shape[0]
        self.info["episodic_outcome"] = None

        if self.is_assist_nav:
            print("[Human Agent]: Waiting for reset signal of assistance agent...")
            self.self_agent_resetting = True
            # synchronize with assistance agent
            while not (self.other_agent_resetting or rospy.is_shutdown()):
                continue
            print("[Human Agent]: Reset signal of assistance agent received!")
            
        '''
        unpause gazebo simulation and set robot initial pose
        '''
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_gazebo()
        except rospy.ServiceException as e:
            print("Unpause Service Failed: %s" % e)
        # reset robot initial pose
        robot_init_pose = self.init_robot_array[ita]
        robot_init_quat = _euler_2_quat(yaw=robot_init_pose[3])
        robot_msg = ModelState()
        robot_msg.model_name = self.robot_ns if self.robot_ns else 'quadcopter'
        robot_msg.pose.position.x = robot_init_pose[0]
        robot_msg.pose.position.y = robot_init_pose[1]
        robot_msg.pose.position.z = robot_init_pose[2]
        robot_msg.pose.orientation.x = robot_init_quat[1]
        robot_msg.pose.orientation.y = robot_init_quat[2]
        robot_msg.pose.orientation.z = robot_init_quat[3]
        robot_msg.pose.orientation.w = robot_init_quat[0]
        # reset robot pose
        rospy.wait_for_service(self.rostopic_prefix + '/falco_planner/set_robot_pose')
        try:
            resp = self.set_robot_pose(robot_msg)
        except rospy.ServiceException as e:
            print("Set Robot Pose Service Failed: %s" % e)
        print("[ROS Service Request]: Set new quadcopter state...")
        # clear map and roadmap after reset
        rospy.wait_for_service(self.rostopic_prefix + '/dep/reset_roadmap')
        try:
            resp = self.reset_roadmap(robot_msg)
            print("[ROS Service Request]: Reset roadmap...")
        except rospy.ServiceException as e:
            print("Reset Roadmap Service Failed: %s" % e)
        # rotate the robot to get initial scan of the environment
        rospy.wait_for_service(self.rostopic_prefix + '/falco_planner/init_rotate_scan_service')
        try:
            resp = self.init_rotate_scan()
            print("[ROS Service Request]: Rotate to gain initial scan...")
        except rospy.ServiceException as e:
            print("Initial Rotate Scan Service Failed: %s" % e)
        # IMPORTANT: sleep for enough time (>2.2s) for the robot to rotate, scan and gain enough free range to build the roadmap
        rospy.sleep(2.5)

        # publish joy for activating falco planner
        self._pub_falco_planner_joy()

        # Init Subscriber
        while not self.robot_odom_init:
            continue
        rospy.loginfo("Odom Subscriber Initialization Finished.")
        while not self.time_duration_init:
            continue
        rospy.loginfo("Time-Duration Subscriber Initialization Finished.")
        # while not self.runtime_init:
        #     continue
        # rospy.loginfo("Runtime Subscriber Initialization Finished.")
        while not self.explored_volume_init:
            continue
        rospy.loginfo("Explored-Volume Subscriber Initialization Finished.")
        while not self.traveling_dist_init:
            continue
        rospy.loginfo("Traveling-Distance Subscriber Initialization Finished.")
        rospy.loginfo("All Subscribers Initialization Finished.")

        # publish navigation target position to roadmap after resetting roadmap
        self.target_position = self.init_target_array[ita]
        target = PointStamped()
        target.header.frame_id = "map"
        target.header.stamp = rospy.Time.now()
        target.point.x = self.target_position[0]
        target.point.y = self.target_position[1]
        target.point.z = self.target_position[2]
        self.nav_target_pub.publish(target)
        print("[Navigation]: Ultimate target position published.")

        self.episodic_start_time = self.time_duration
        self.episodic_start_dist = self.traveling_distance
        self.avg_runtime = 0
        self.runtime_count = 0

        if self.is_assist_nav:
            self.self_agent_resetting = False
            self.assist_episode_end = False

    def get_info(self):
        assert self.target_position is not None

        target_dis = math.sqrt((self.odom.pose.pose.position.x - self.target_position[0]) ** 2 +
                               (self.odom.pose.pose.position.y - self.target_position[1]) ** 2 +
                               (self.odom.pose.pose.position.z - self.target_position[2]) ** 2)
        target_reach_flag = (target_dis <= self.goal_near_th)
        out_of_range_flag = (self.odom.pose.pose.position.z <= self.env_height_range[0]) or \
                            (self.odom.pose.pose.position.z >= self.env_height_range[1])
        episodic_time_cost = self.time_duration - self.episodic_start_time
        episodic_dist_cost = self.traveling_distance - self.episodic_start_dist

        if target_reach_flag:
            self.info["episodic_time_cost"] = episodic_time_cost
            self.info["episodic_dist_cost"] = episodic_dist_cost
            self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "success"
            self.info["outcome_statistic"]["success"] += 1
            print("[Episodic Outcome]: Goal achieved!")
        elif out_of_range_flag:
            self.info["episodic_time_cost"] = episodic_time_cost
            self.info["episodic_dist_cost"] = episodic_dist_cost
            self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "collision"
            self.info["outcome_statistic"]["collision"] += 1
            print("[Episodic Outcome]: Out of flying range!")
        elif episodic_time_cost >= self.episodic_max_period:
            self.info["episodic_time_cost"] = episodic_time_cost
            self.info["episodic_dist_cost"] = episodic_dist_cost
            self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "timeout"
            self.info["outcome_statistic"]["timeout"] += 1
            print("[Episodic Outcome]: Navigation timeout!")
        
        if self.is_assist_nav and self.assist_episode_end:
            # self.info["episodic_time_cost"] = episodic_time_cost
            # self.info["episodic_dist_cost"] = episodic_dist_cost
            # self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "assistance_done"
            self.info["outcome_statistic"]["assistance_done"] += 1
            print("[Episodic Outcome]: Assistance agent episode done!")

        return self.info

    def close(self):
        print(" ")
        if not self.robot_ns:
            rospy.loginfo("Shutting down all nodes...")
            os.system("rosnode kill -a")
        else:
            node_name = rospy.get_name()
            rospy.loginfo("Shutting down node: %s", node_name)
            os.system("rosnode kill " + node_name)

    def _pub_falco_planner_joy(self):
        joy = Joy()
        joy.axes = [0., 0., -1.0, 0., 1.0, 1.0, 0., 0.]
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        joy.header.stamp = rospy.Time.now()
        joy.header.frame_id = f"{self.robot_ns}/waypoint_tool" if self.robot_ns else "waypoint_tool"
        self.pub_joy.publish(joy)

    def _odom_callback(self, odom):
        if self.robot_odom_init is False:
            self.robot_odom_init = True
        self.odom = odom

    def _time_duration_callback(self, msg):
        if self.time_duration_init is False:
            self.time_duration_init = True
        self.time_duration = msg.data

    def _runtime_callback(self, msg):
        if self.runtime_init is False:
            self.runtime_init = True
        self.run_time = msg.data
        self.runtime_count += 1
        self.avg_runtime += (1 / self.runtime_count) * (self.run_time - self.avg_runtime)

    def _explored_volume_callback(self, msg):
        if self.explored_volume_init is False:
            self.explored_volume_init = True
        self.explored_volume = msg.data

    def _traveling_dist_callback(self, msg):
        if self.traveling_dist_init is False:
            self.traveling_dist_init = True
        self.traveling_distance = msg.data

    def _pub_self_reset_callback(self, event):
        reset_flag = Bool()
        reset_flag.data = self.self_agent_resetting
        self.self_agent_reset_pub.publish(reset_flag)

    def _other_agent_reset_callback(self, msg):
        if not self.other_agent_reset_init:
            self.other_agent_reset_init = True
        self.other_agent_resetting = msg.data

    def _set_episode_end_callback(self, req):
        """Handle requests to set episode end flag"""
        try:
            self.assist_episode_end = req.data
            return SetBoolResponse(success=True, message="Episode end flag set successfully")
        except Exception as e:
            return SetBoolResponse(success=False, message=str(e))

