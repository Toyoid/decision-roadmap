import math
import numpy as np
import torch
import rospy
import os
from utils import _get_init_robot_goal, _euler_2_quat

from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from global_planner.srv import GetRoadmap
from falco_planner.srv import SetRobotPose
from decision_roadmap_agent.msg import TargetArray


class NavDRMEnvBase:
    """
    Base RL Environment Class for Single-Robot Navigation based on Decision Roadmap
    """
    def __init__(
        self, 
        args,
        robot_namespace,
        drm_name,  # options: 'dep', 'fg_drm'
        is_train=False,
    ):
        self.robot_ns = robot_namespace
        self.init_robot_array, self.init_target_array = _get_init_robot_goal(
            getattr(args, 'map_name', 'maze_medium'), 
            shuffle=True if robot_namespace else False
        )

        # Robot messages
        self.odom = None
        self.robot_odom_init = False
        self.time_duration = None
        self.time_duration_init = False
        self.run_time = None
        self.runtime_init = False
        self.explored_volume = None
        self.explored_volume_init = False
        self.traveling_distance = None
        self.traveling_dist_init = False

        # Training environment setup
        self.is_train = is_train
        self.device = getattr(args, 'device', 'cpu')
        self.step_time = getattr(args, 'step_time', 4.0)
        self.max_episode_steps = getattr(args, 'max_episode_steps', 64)
        self.goal_reward = getattr(args, 'goal_reward', 20)
        self.collision_reward = getattr(args, 'collision_reward', -20)
        self.step_penalty = getattr(args, 'step_penalty_reward', -0.5)
        self.revisit_penalty = getattr(args, 'revisit_penalty_reward', -0.3)
        self.goal_near_th = getattr(args, 'goal_near_th', 0.5)
        self.goal_dis_amp = getattr(args, 'goal_dis_amp', 1/64.)
        self.env_height_range = getattr(args, 'env_height_range', [0.2, 2.5])
        self.goal_dis_scale = getattr(args, 'goal_dis_scale', 1.0)
        self.goal_dis_min_dis = getattr(args, 'goal_dis_min_dis', 0.3)

        # roadmap specific data
        self.edge_inputs = None
        self.node_coords = None
        self.route_node = np.array([])  
        self.is_revisiting = False  # whether the robot is revisiting the visited node
        self.k_neighbor_size = getattr(args, 'k_neighbor_size', 20)
        self.coords_norm_coef = getattr(args, 'coords_norm_coef', 30)
        self.utility_norm_coef = getattr(args, 'utility_norm_coef', 4000)
        self.node_padding_size = getattr(args, 'node_padding_size', 500)

        # information of interaction between agent and environment
        self.nav_target_found = False
        self.ita_in_episode = None
        self.episodic_start_time = None
        self.episodic_start_dist = None
        self.avg_runtime = 0
        self.runtime_count = 0
        self.info = {
            "episodic_return": 0.0,
            "episodic_length": 0,
            "episodic_time_cost": 0,
            "episodic_dist_cost": 0,
            "episodic_avg_runtime": 0,
            "episodic_outcome": None,
            "outcome_statistic": {
                "success": 0,
                "collision": 0,
                "timeout": 0
            }
        }

        # Names of ROS topics and services
        self.rostopic_prefix = f"/{self.robot_ns}" if self.robot_ns else ""
        
        self.odom_topic = f"{self.rostopic_prefix}/falco_planner/state_estimation"
        self.current_goal_topic = f"{self.rostopic_prefix}/agent/current_goal"
        self.joy_topic = f"{self.rostopic_prefix}/falco_planner/joy"
        self.nav_target_topic = f"{self.rostopic_prefix}/env/nav_target"
        self.real_nav_target_vis_topic = f"{self.rostopic_prefix}/env/real_nav_target_vis"

        self.pause_gazebo_service_name = '/gazebo/pause_physics'
        self.unpause_gazebo_service_name = '/gazebo/unpause_physics'
        self.set_model_pose_service_name = '/gazebo/set_model_state'
        self.get_roadmap_service_name = f"{self.rostopic_prefix}/{drm_name}/get_roadmap"
        self.reset_roadmap_service_name = f"{self.rostopic_prefix}/{drm_name}/reset_roadmap"
        self.set_robot_pose_service_name = f"{self.rostopic_prefix}/falco_planner/set_robot_pose"
        self.init_rotate_scan_service_name = f"{self.rostopic_prefix}/falco_planner/init_rotate_scan_service"
        self.escape_stuck_service_name = f"{self.rostopic_prefix}/falco_planner/escape_stuck_service"

        # Subscriber
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback)
        rospy.Subscriber(self.rostopic_prefix + "/data_recording/time_duration", Float32, self._time_duration_callback)
        # rospy.Subscriber(self.rostopic_prefix + "/data_recording/runtime", Float32, self._runtime_callback)
        rospy.Subscriber(self.rostopic_prefix + "/data_recording/explored_volume", Float32, self._explored_volume_callback)
        rospy.Subscriber(self.rostopic_prefix + "/data_recording/traveling_distance", Float32, self._traveling_dist_callback)
        # Publisher
        self.current_goal_pub = rospy.Publisher(self.current_goal_topic, PointStamped, queue_size=5)
        # publish joy for activating falco planner
        self.pub_joy = rospy.Publisher(self.joy_topic, Joy, queue_size=5)
        self.nav_target_pub = rospy.Publisher(self.nav_target_topic, TargetArray, queue_size=5)
        self.real_nav_target_vis_pub = rospy.Publisher(self.real_nav_target_vis_topic, PointStamped, queue_size=5)

        # Service
        self.pause_gazebo = rospy.ServiceProxy(self.pause_gazebo_service_name, Empty)
        self.unpause_gazebo = rospy.ServiceProxy(self.unpause_gazebo_service_name, Empty)
        self.set_model_pose = rospy.ServiceProxy(self.set_model_pose_service_name, SetModelState)
        self.get_roadmap = rospy.ServiceProxy(self.get_roadmap_service_name, GetRoadmap)
        self.reset_roadmap = rospy.ServiceProxy(self.reset_roadmap_service_name, SetRobotPose)
        self.set_robot_pose = rospy.ServiceProxy(self.set_robot_pose_service_name, SetRobotPose)
        self.init_rotate_scan = rospy.ServiceProxy(self.init_rotate_scan_service_name, Empty)
        self.escape_stuck = rospy.ServiceProxy(self.escape_stuck_service_name, Empty)

        # Init Subscriber
        while not self.robot_odom_init:
            continue
        rospy.loginfo("Finish Odom Subscriber Init...")
        rospy.loginfo("Subscriber Initialization Finished!")

    def reset(self, ita):
        """
        Reset funtion to start a new episode of the single robot navigation environment

        :param ita: iteration variable of current episode number
        :return: roadmap_state
        """
        assert self.init_robot_array is not None
        assert self.init_target_array is not None
        assert ita < self.init_robot_array.shape[0]
        self.ita_in_episode = 0
        self.info["episodic_return"] = 0.0
        self.info["episodic_length"] = 0
        self.info["episodic_outcome"] = None
        self.route_node = np.array([])  # reset route node for computing guidepost
        self.is_revisiting = False  # whether the robot is revisiting the visited node
        '''
        unpause gazebo simulation and set robot initial pose
        '''
        rospy.wait_for_service(self.unpause_gazebo_service_name)
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
        # set robot pose
        rospy.wait_for_service(self.set_robot_pose_service_name)
        try:
            resp = self.set_robot_pose(robot_msg)
        except rospy.ServiceException as e:
            print("Set Robot Pose Service Failed: %s" % e)
        print("[ROS Service Request]: New quadcopter state set.")
        
        # publish joy to activate local planner
        self._pub_falco_planner_joy()
        
        # clear map and roadmap after reset
        rospy.wait_for_service(self.reset_roadmap_service_name)
        try:
            resp = self.reset_roadmap(robot_msg)
        except rospy.ServiceException as e:
            print("Reset Roadmap Service Failed: %s" % e)
        print("[ROS Service Request]: Reset the roadmap...")
        
        # publish navigation target position to roadmap after resetting roadmap
        self.nav_target_found = False
        self.target_position = self.init_target_array[ita]
        target_array = TargetArray()
        target_array.header.frame_id = "map"
        target_array.header.stamp = rospy.Time.now()
        target = Point()
        target.x = self.target_position[0]
        target.y = self.target_position[1]
        target.z = self.target_position[2]
        target_array.targets.append(target)
        self.nav_target_pub.publish(target_array)
        # visualize real navigation target
        real_target = PointStamped()
        real_target.header.frame_id = "map"
        real_target.header.stamp = rospy.Time.now()
        real_target.point.x = self.target_position[0]
        real_target.point.y = self.target_position[1]
        real_target.point.z = self.target_position[2]
        self.real_nav_target_vis_pub.publish(real_target)
        print(f"[ROS Topic]: Target position [{self.target_position[0]}, {self.target_position[1]}, {self.target_position[2]}]")

        # init recorders before rotating the robot
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

        # rotate the robot to get initial scan of the environment
        rospy.wait_for_service(self.init_rotate_scan_service_name)
        try:
            resp = self.init_rotate_scan()
        except rospy.ServiceException as e:
            print("Initial Rotate Scan Service Failed: %s" % e)
        print("[ROS Service Request]: Rotate to gain initial scan...")
        # NOTE: sleep for enough time (>2.2s) for the robot to rotate, scan and gain enough free range to build the roadmap
        rospy.sleep(2.5)

        robot_pose, roadmap_state = self._get_next_state()
        '''
        pause gazebo simulation and transform robot poses to robot observations
        '''
        rospy.wait_for_service(self.pause_gazebo_service_name)
        try:
            self.pause_gazebo()
        except rospy.ServiceException as e:
            print("Pause Service Failed: %s" % e)

        target_dis, target_dir = self._compute_dis_dir_2_goal(robot_pose)
        self.target_dis_dir_pre = [target_dis, target_dir]
        self.target_dis_dir_cur = [target_dis, target_dir]

        self.episodic_start_time = self.time_duration
        self.episodic_start_dist = self.traveling_distance
        
        # print(f"[START]: Time = {self.episodic_start_time}, Distance = {self.episodic_start_dist}")

        self.avg_runtime = 0
        self.runtime_count = 0

        return roadmap_state
    
    def close(self):
        print(" ")
        if not self.robot_ns:
            rospy.loginfo("Shutting down all nodes...")
            os.system("rosnode kill -a")
        else:
            node_name = rospy.get_name()
            rospy.loginfo("Shutting down node: %s", node_name)
            os.system("rosnode kill " + node_name)

    def _get_next_state(self):
        """
        Get next state of the robot and roadmap
        """
        raise NotImplementedError

    def _compute_dis_dir_2_goal(self, robot_pose):
        """
        Compute relative distance and direction from robot pose to navigation goal
        """
        delta_x = self.target_position[0] - robot_pose[0]
        delta_y = self.target_position[1] - robot_pose[1]
        delta_z = self.target_position[2] - robot_pose[2]
        distance = math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)

        ego_direction = math.atan2(delta_y, delta_x)
        robot_direction = robot_pose[3]
        # shift ego_direction and robot_direction to [0, 2*pi]
        while robot_direction < 0:
            robot_direction += 2 * math.pi
        while robot_direction > 2 * math.pi:
            robot_direction -= 2 * math.pi
        while ego_direction < 0:
            ego_direction += 2 * math.pi
        while ego_direction > 2 * math.pi:
            ego_direction -= 2 * math.pi
        '''
        Desired direction range is between -pi and pi, 
        [-pi, 0] for the goal on the right of the robot,
        [0, pi] for the goal on the left of the robot.
        Conditions are:
        if (ego_direction - robot_direction) is between 0 and pi, 
            then goal is on the left of the robot, direction = (ego_direction - robot_direction) ∈(0, pi)
        if (ego_direction - robot_direction) is between pi and 2*pi, 
            then goal is on the right of the robot, direction = -(2*pi - abs(ego_direction - robot_direction)) ∈(-pi, 0)
        if (ego_direction - robot_direction) is between -pi and 0, 
            then goal is on the right of the robot, direction = (ego_direction - robot_direction) ∈(-pi, 0)
        if (ego_direction - robot_direction) is between -2*pi and -pi, 
            then goal is on the left of the robot, direction = (2*pi - abs(ego_direction - robot_direction)) ∈(0, pi)
        '''
        pos_dir = abs(ego_direction - robot_direction)
        neg_dir = 2 * math.pi - abs(ego_direction - robot_direction)
        if pos_dir <= neg_dir:
            direction = math.copysign(pos_dir, ego_direction - robot_direction)
        else:
            direction = math.copysign(neg_dir, -(ego_direction - robot_direction))
        return distance, direction

    def _compute_reward(self, robot_pose):
        """
        Reward funtion:
        1. R_Arrive If Distance to Goal is smaller than D_goal
        2. R_Collision If Distance to Obstacle is smaller than D_obs
        3. a * (Last step distance to goal - current step distance to goal)
        4. R_Penalty for each step 
        """
        episodic_time_cost = self.time_duration - self.episodic_start_time
        episodic_dist_cost = self.traveling_distance - self.episodic_start_dist

        done = False
        if self.target_dis_dir_cur[0] < self.goal_near_th:
            reward = self.goal_reward
            done = True
            self.info["episodic_time_cost"] = episodic_time_cost
            self.info["episodic_dist_cost"] = episodic_dist_cost
            self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "success"
            self.info["outcome_statistic"]["success"] += 1
            print("[Episodic Outcome]: Goal achieved!")
        # elif self.robot_collision:
        #     reward = self.collision_reward
        #     done = True
        #     self.info["episodic_time_cost"] = episodic_time_cost
        #     self.info["episodic_dist_cost"] = episodic_dist_cost
        #     self.info["episodic_avg_runtime"] = self.avg_runtime
        #     self.info["episodic_outcome"] = "collision"
        #     self.info["outcome_statistic"]["collision"] += 1
        #     print("[Episodic Outcome]: Collides with obstacles!")
        elif (robot_pose[2] <= self.env_height_range[0]) or (robot_pose[2] >= self.env_height_range[1]):
            reward = self.collision_reward
            done = True
            self.info["episodic_time_cost"] = episodic_time_cost
            self.info["episodic_dist_cost"] = episodic_dist_cost
            self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "collision"
            self.info["outcome_statistic"]["collision"] += 1
            print("[Episodic Outcome]: Out of flying range!")
        elif self.ita_in_episode >= self.max_episode_steps:
            reward = self.goal_dis_amp * (self.target_dis_dir_pre[0] - self.target_dis_dir_cur[0])
            done = True
            self.info["episodic_time_cost"] = episodic_time_cost
            self.info["episodic_dist_cost"] = episodic_dist_cost
            self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "timeout"
            self.info["outcome_statistic"]["timeout"] += 1
            print("[Episodic Outcome]: Navigation timeout!")
        else:
            reward = self.goal_dis_amp * (self.target_dis_dir_pre[0] - self.target_dis_dir_cur[0])

        reward += self.step_penalty

        return reward, done
    
    def _pub_falco_planner_joy(self):
        joy = Joy()
        joy.axes = [0., 0., -1.0, 0., 1.0, 1.0, 0., 0.]
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        joy.header.stamp = rospy.Time.now()
        joy.header.frame_id = f"{self.robot_ns}/waypoint_tool" if self.robot_ns else "waypoint_tool"
        self.pub_joy.publish(joy)

    @staticmethod
    def calculate_edge_mask(edge_inputs):
        size = len(edge_inputs)
        bias_matrix = np.ones((size, size))
        for i in range(size):
            for j in range(size):
                if j in edge_inputs[i]:
                    bias_matrix[i][j] = 0
        return bias_matrix

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