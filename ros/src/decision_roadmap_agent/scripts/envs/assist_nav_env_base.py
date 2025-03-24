import math
import numpy as np
import torch
import rospy
import os
import time
import random
from utils import _get_init_robot_goal_assist_nav, _euler_2_quat

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from global_planner.srv import GetRoadmap
from falco_planner.srv import SetRobotPose
from decision_roadmap_agent.msg import TargetArray
from std_srvs.srv import SetBool

class AssistNavEnvBase:
    """
    Base RL Environment Class for Assistance Navigation Game based on Decision Roadmap
    """
    def __init__(
        self, 
        args,
        robot_namespace,
        other_robot_namespace,
        drm_name,  # options: 'dep', 'fg_drm'
        is_train=False,
    ):
        assert robot_namespace, "Robot namespace is required!"
        assert other_robot_namespace, "Other robot namespace is required!"
        self.robot_ns = robot_namespace
        self.other_robot_ns = other_robot_namespace
        self.init_robot_array, self.all_target_pos_dict = _get_init_robot_goal_assist_nav(
            getattr(args, 'map_name', 'maze_medium'), 
            is_assistance=True
        )

        # Robot messages
        self.odom = None
        self.other_agent_odom = None
        self.robot_odom_init = False
        self.other_robot_odom_init = False
        self.other_agent_resetting = False
        self.other_agent_reset_init = False

        # Training environment setup
        self.self_agent_resetting = False
        # self.assist_episode_end = False
        self.is_train = is_train
        self.device = getattr(args, 'device', 'cpu')
        self.step_time = getattr(args, 'step_time', 4.0)
        self.max_episode_steps = getattr(args, 'max_episode_steps', 64)
        self.goal_reward = getattr(args, 'goal_reward', 20)
        self.collision_reward = getattr(args, 'collision_reward', -20)
        self.step_penalty = getattr(args, 'step_penalty_reward', -0.5)
        self.goal_near_th = getattr(args, 'goal_near_th', 0.3)
        self.goal_dis_amp = getattr(args, 'goal_dis_amp', 1/64.)
        self.env_height_range = getattr(args, 'env_height_range', [0.2, 2.5])
        self.goal_dis_scale = getattr(args, 'goal_dis_scale', 1.0)
        self.goal_dis_min_dis = getattr(args, 'goal_dis_min_dis', 0.3)

        # roadmap specific data
        self.edge_inputs = None
        self.node_coords = None
        self.route_node = []
        self.k_neighbor_size = getattr(args, 'k_neighbor_size', 20)
        self.coords_norm_coef = getattr(args, 'coords_norm_coef', 30)
        self.utility_norm_coef = getattr(args, 'utility_norm_coef', 4000)
        self.node_padding_size = getattr(args, 'node_padding_size', 500)

        # information of interaction between agent and environment
        self.ita_in_episode = None
        self.target_positions = {}
        self.other_robot_pos_cur  = None        
        self.other_robot_pos_pre  = None
        self.info = {
            "episodic_return": 0.0,
            "episodic_length": 0,
            "episodic_outcome": None,
            "outcome_statistic": {
                "success": 0,
                "collision": 0,
                "timeout": 0
            }
        }

        # Names of ROS topics and services
        self.rostopic_prefix = f"/{self.robot_ns}"
        
        self.other_agent_reset_topic = "/human_agent/reset_flag"
        self.self_agent_reset_topic = "/assist_agent/reset_flag"
        self.other_agent_odom_topic = f"/{self.other_robot_ns}/falco_planner/state_estimation"
        self.odom_topic = f"{self.rostopic_prefix}/falco_planner/state_estimation"
        self.current_goal_topic = f"{self.rostopic_prefix}/agent/current_goal"
        self.joy_topic = f"{self.rostopic_prefix}/falco_planner/joy"
        self.nav_target_topic = f"{self.rostopic_prefix}/env/nav_target"
        self.real_nav_target_vis_topic = f"{self.rostopic_prefix}/env/real_nav_target_vis"
        self.nav_target2_vis_topic = f"{self.rostopic_prefix}/env/nav_target2_vis"
        self.nav_target3_vis_topic = f"{self.rostopic_prefix}/env/nav_target3_vis"

        self.pause_gazebo_service_name = '/gazebo/pause_physics'
        self.unpause_gazebo_service_name = '/gazebo/unpause_physics'
        self.set_episode_end_service_name = '/human_agent/set_episode_end_flag'
        self.get_roadmap_service_name = f"{self.rostopic_prefix}/{drm_name}/get_roadmap"
        self.reset_roadmap_service_name = f"{self.rostopic_prefix}/{drm_name}/reset_roadmap"
        self.set_robot_pose_service_name = f"{self.rostopic_prefix}/falco_planner/set_robot_pose"
        self.init_rotate_scan_service_name = f"{self.rostopic_prefix}/falco_planner/init_rotate_scan_service"
        self.escape_stuck_service_name = f"{self.rostopic_prefix}/falco_planner/escape_stuck_service"

        # Timer
        self.pub_self_reset_timer = rospy.Timer(rospy.Duration(0.1), self._pub_self_reset_callback)
        # Subscriber
        self.other_agent_reset_sub = rospy.Subscriber(self.other_agent_reset_topic, Bool, self._other_agent_reset_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback)
        self.other_agent_odom_sub = rospy.Subscriber(self.other_agent_odom_topic, Odometry, self._other_agent_odom_callback)
        # Publisher
        self.self_agent_reset_pub = rospy.Publisher(self.self_agent_reset_topic, Bool, queue_size=5)
        self.current_goal_pub = rospy.Publisher(self.current_goal_topic, PointStamped, queue_size=5)
        # publish joy for activating falco planner
        self.pub_joy = rospy.Publisher(self.joy_topic, Joy, queue_size=5)
        self.nav_target_pub = rospy.Publisher(self.nav_target_topic, TargetArray, queue_size=5)
        self.real_nav_target_vis_pub = rospy.Publisher(self.real_nav_target_vis_topic, PointStamped, queue_size=5)
        self.nav_target2_vis_pub = rospy.Publisher(self.nav_target2_vis_topic, PointStamped, queue_size=5)
        self.nav_target3_vis_pub = rospy.Publisher(self.nav_target3_vis_topic, PointStamped, queue_size=5)

        # Service
        self.pause_gazebo = rospy.ServiceProxy(self.pause_gazebo_service_name, Empty)
        self.unpause_gazebo = rospy.ServiceProxy(self.unpause_gazebo_service_name, Empty)
        self.get_roadmap = rospy.ServiceProxy(self.get_roadmap_service_name, GetRoadmap)
        self.reset_roadmap = rospy.ServiceProxy(self.reset_roadmap_service_name, SetRobotPose)
        self.set_robot_pose = rospy.ServiceProxy(self.set_robot_pose_service_name, SetRobotPose)
        self.init_rotate_scan = rospy.ServiceProxy(self.init_rotate_scan_service_name, Empty)
        self.escape_stuck = rospy.ServiceProxy(self.escape_stuck_service_name, Empty)
        self.set_human_episode_end = rospy.ServiceProxy(self.set_episode_end_service_name, SetBool)

        # Init Subscriber
        while not (self.robot_odom_init or rospy.is_shutdown()):
            continue
        rospy.loginfo("Finish Odom Subscriber Init...")
        while not (self.other_robot_odom_init or rospy.is_shutdown()):
            continue
        rospy.loginfo("Finish Human Odom Subscriber Init...")
        while not (self.other_agent_reset_init or rospy.is_shutdown()):
            continue
        rospy.loginfo("Finish Human Reset Subscriber Init...")
        rospy.loginfo("Subscriber Initialization Finished!")

    def reset(self, ita):
        """
        Reset funtion to start a new episode of the assistance navigation environment

        :param ita: iteration variable of current episode number
        :return: roadmap_state
        """

        assert self.init_robot_array is not None
        assert self.all_target_pos_dict is not None
        assert ita < self.all_target_pos_dict["real_target"].shape[0]
        self.ita_in_episode = 0
        self.info["episodic_return"] = 0.0
        self.info["episodic_length"] = 0
        self.info["episodic_outcome"] = None

        target_list = ["real_target", "target2", "target3"]
        random_seed = int(time.time()) + ita  
        random.seed(random_seed)  
        shuffled_list = random.sample(target_list, len(target_list)) 
        for target_name in shuffled_list:
            self.target_positions[target_name] = self.all_target_pos_dict[target_name][ita]

        print("[DRM Input Construction]: Shuffled navigation target list: ", shuffled_list)        
        
        self.self_agent_resetting = True
        # self.assist_episode_end = False
        '''
        unpause gazebo simulation and set robot initial pose
        '''
        rospy.wait_for_service(self.unpause_gazebo_service_name)
        try:
            self.unpause_gazebo()
        except rospy.ServiceException as e:
            print("Unpause Service Failed: %s" % e)

        # synchronize with human agent (wait after unpause)
        print("[Assistance]: Waiting for reset signal of human agent...")
        while not (self.other_agent_resetting or rospy.is_shutdown()):
            continue  
        rospy.loginfo("[Assistance]: Reset signal of human agent received")
        
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

        # publish the positions of 3 navigation targets to roadmap after resetting roadmap
        self._pub_vis_all_targets()
        print("[ROS Publisher]: Published and visualized all targets")

        # rotate the robot to get initial scan of the environment
        rospy.wait_for_service(self.init_rotate_scan_service_name)
        try:
            resp = self.init_rotate_scan()
        except rospy.ServiceException as e:
            print("Initial Rotate Scan Service Failed: %s" % e)
        print("[ROS Service Request]: Rotate to gain initial scan...")
        # NOTE: sleep for enough time (>2.2s) for the robot to rotate, scan and gain enough free range to build the roadmap
        rospy.sleep(2.5)

        self.other_robot_pos_pre = [self.other_agent_odom.pose.pose.position.x, 
                                    self.other_agent_odom.pose.pose.position.y]
        self.other_robot_pos_cur = [self.other_agent_odom.pose.pose.position.x,
                                    self.other_agent_odom.pose.pose.position.y]
        robot_pose, roadmap_state = self._get_next_state()
        self.other_robot_pos_pre = [self.other_robot_pos_cur[0], self.other_robot_pos_cur[1]]
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

        self.self_agent_resetting = False
        # self.assist_episode_end = False
        return roadmap_state

    def step(self, action_index):
        """
        Step funtion of single robot navigation environment based on decision roadmap

        Note: This implementation is for the dense roadmap. The step() for Frontier-Guided Decision Roadmap need to be re-implemented. 
        """
        assert self.ita_in_episode is not None
        assert self.target_positions, "Target positions are not initialized!"
        assert self.other_robot_pos_pre is not None
        assert self.other_robot_pos_cur is not None
        self.ita_in_episode += 1

        '''
        action decode
        '''
        selected_node_idx = self.edge_inputs[0, 0, action_index.item()]  # tensor(scalar)
        selected_node_pos = self.node_coords[selected_node_idx]  # np.array(2,)

        rospy.wait_for_service(self.unpause_gazebo_service_name)
        try:
            self.unpause_gazebo()
        except rospy.ServiceException as e:
            print("Unpause Service Failed: %s" % e)
        '''
        Give action to robot and let robot execute, then get next observation
        '''
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.point.x = selected_node_pos[0]
        goal.point.y = selected_node_pos[1]
        goal.point.z = 1.0
        self.current_goal_pub.publish(goal)

        robot_position = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z]
        execute_start_time = rospy.get_time()
        curr_goal_arrive_flag = False
        step_time_flag = False
        target_reach_flag = False
        while not (curr_goal_arrive_flag or step_time_flag or target_reach_flag or rospy.is_shutdown()):
            dis = math.sqrt((self.odom.pose.pose.position.x - goal.point.x) ** 2 +
                            (self.odom.pose.pose.position.y - goal.point.y) ** 2 +
                            (self.odom.pose.pose.position.z - goal.point.z) ** 2)
            curr_goal_arrive_flag = dis <= 0.9
            step_time_flag = (rospy.get_time() - execute_start_time) >= self.step_time

            target_dis = math.sqrt((self.odom.pose.pose.position.x - self.target_positions["real_target"][0]) ** 2 +
                                   (self.odom.pose.pose.position.y - self.target_positions["real_target"][1]) ** 2 +
                                   (self.odom.pose.pose.position.z - self.target_positions["real_target"][2]) ** 2)
            target_reach_flag = (target_dis <= self.goal_near_th)

        # # update route_node for computing guidepost
        # if curr_goal_arrive_flag:
        #     self.route_node.append(selected_node_pos)

        # check whether local planner get stuck
        robot_move = math.sqrt((self.odom.pose.pose.position.x - robot_position[0]) ** 2 +
                               (self.odom.pose.pose.position.y - robot_position[1]) ** 2 +
                               (self.odom.pose.pose.position.z - robot_position[2]) ** 2)
        if robot_move <= 0.05:
            rospy.wait_for_service(self.escape_stuck_service_name)
            try:
                resp = self.escape_stuck()
            except rospy.ServiceException as e:
                print("Escape Stuck Service Failed: %s" % e)
            print("[ROS Service Request]: adjusting robot to escape stuck...")

        # get next state
        self.other_robot_pos_cur = [self.other_agent_odom.pose.pose.position.x,
                                    self.other_agent_odom.pose.pose.position.y]
        robot_pose, roadmap_state = self._get_next_state()
        self.other_robot_pos_pre = [self.other_robot_pos_cur[0], self.other_robot_pos_cur[1]]
        '''
        Then pause the simulation
        1. Compute rewards of the actions
        2. Check if the episode is ended
        '''
        rospy.wait_for_service(self.pause_gazebo_service_name)
        try:
            self.pause_gazebo()
        except rospy.ServiceException as e:
            print("Pause Service Failed: %s" % e)

        if target_reach_flag:
            reward = self.goal_reward + self.step_penalty
            done = True
            # self.assist_episode_end = True
            # Add service call
            try:
                self.set_human_episode_end(True)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

            self.info["episodic_outcome"] = "success"
            self.info["outcome_statistic"]["success"] += 1
            print("[Episodic Outcome]: Goal achieved!")
        else:
            target_dis, target_dir = self._compute_dis_dir_2_goal(robot_pose)
            self.target_dis_dir_cur = [target_dis, target_dir]
            reward, done = self._compute_reward(robot_pose)
            self.target_dis_dir_pre = [self.target_dis_dir_cur[0], self.target_dis_dir_cur[1]]

        self.info["episodic_return"] += reward
        self.info["episodic_length"] += 1

        reward = torch.tensor(reward).view(1, 1, 1).to(self.device)
        done = torch.tensor(done, dtype=torch.int32).view(1, 1, 1).to(self.device)

        return roadmap_state, reward, done, self.info

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
        delta_x = self.target_positions["real_target"][0] - robot_pose[0]
        delta_y = self.target_positions["real_target"][1] - robot_pose[1]
        delta_z = self.target_positions["real_target"][2] - robot_pose[2]
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
        4. Step_penalty for each step
        """
        done = False
        if self.target_dis_dir_cur[0] < self.goal_near_th:
            reward = self.goal_reward
            done = True
            self.info["episodic_outcome"] = "success"
            self.info["outcome_statistic"]["success"] += 1
            print("[Episodic Outcome]: Goal achieved!")
        # elif self.robot_collision:
        #     reward = self.collision_reward
        #     done = True
        #     self.info["episodic_outcome"] = "collision"
        #     self.info["outcome_statistic"]["collision"] += 1
        #     print("[Episodic Outcome]: Collides with obstacles!")
        elif (robot_pose[2] <= self.env_height_range[0]) or (robot_pose[2] >= self.env_height_range[1]):
            reward = self.collision_reward
            done = True
            self.info["episodic_outcome"] = "collision"
            self.info["outcome_statistic"]["collision"] += 1
            print("[Episodic Outcome]: Out of flying range!")
        elif self.ita_in_episode >= self.max_episode_steps:
            reward = self.goal_dis_amp * (self.target_dis_dir_pre[0] - self.target_dis_dir_cur[0])
            done = True
            self.info["episodic_outcome"] = "timeout"
            self.info["outcome_statistic"]["timeout"] += 1
            print("[Episodic Outcome]: Navigation timeout!")
        else:
            reward = self.goal_dis_amp * (self.target_dis_dir_pre[0] - self.target_dis_dir_cur[0])

        reward += self.step_penalty

        if done:
            # self.assist_episode_end = True
            try:
                self.set_human_episode_end(True)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

        return reward, done
    
    def _pub_falco_planner_joy(self):
        joy = Joy()
        joy.axes = [0., 0., -1.0, 0., 1.0, 1.0, 0., 0.]
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        joy.header.stamp = rospy.Time.now()
        joy.header.frame_id = f"{self.robot_ns}/waypoint_tool" if self.robot_ns else "waypoint_tool"
        self.pub_joy.publish(joy)

    def _pub_vis_all_targets(self):
        # Create target array message
        target_array = TargetArray()
        target_array.header.frame_id = "map"
        target_array.header.stamp = rospy.Time.now()
        # Add all targets in order: real_target, target2, target3, and visualize them
        for target_name in ["real_target", "target2", "target3"]:
            point = Point()
            point.x = self.target_positions[target_name][0]
            point.y = self.target_positions[target_name][1]
            point.z = self.target_positions[target_name][2]
            target_array.targets.append(point)

            # visualize
            target_pos_msg = PointStamped()
            target_pos_msg.header.frame_id = "map"
            target_pos_msg.header.stamp = rospy.Time.now()
            target_pos_msg.point = point
            if target_name == "real_target":
                self.real_nav_target_vis_pub.publish(target_pos_msg)
            elif target_name == "target2":
                self.nav_target2_vis_pub.publish(target_pos_msg)
            else:
                self.nav_target3_vis_pub.publish(target_pos_msg)

        # Publish all targets at once
        self.nav_target_pub.publish(target_array)
        print("[ROS Publisher]: Published target array")

    @staticmethod
    def calculate_edge_mask(edge_inputs):
        size = len(edge_inputs)
        bias_matrix = np.ones((size, size))
        for i in range(size):
            for j in range(size):
                if j in edge_inputs[i]:
                    bias_matrix[i][j] = 0
        return bias_matrix
    
    @staticmethod
    def _compute_direction_vectors(node_coords, targets, coords_norm_coef):
        """
        Compute normalized direction vectors and normalized distances from each node to each target
        in a fully vectorized manner, and return the output as a 2D array of shape (n_nodes, 3*n_targets).

        For each target, the output contains:
          [normalized_dx, normalized_dy, normalized_distance]

        Parameters:
            node_coords (np.ndarray): Array of shape (n_nodes, 2) with node (x, y) coordinates.
            targets (np.ndarray): Array of target positions, (n_targets, 2).
            coords_norm_coef (float): A coefficient to normalize the distances.

        Returns:
            np.ndarray: An array of shape (n_nodes, 3*n_targets). For each node, the data is organized as:
                        [normalized_dx_target1, normalized_dy_target1, normalized_distance_target1,
                         normalized_dx_target2, normalized_dy_target2, normalized_distance_target2, ...]
        """
        # Ensure node_coords is a float array.
        node_coords = np.array(node_coords, dtype=np.float64)
        
        # Reshape for broadcasting:
        #   node_coords: (n_nodes, 1, 2)
        #   targets:     (1, n_targets, 2)
        diff = targets[None, :, :] - node_coords[:, None, :]  # shape: (n_nodes, n_targets, 2)
        
        # Compute Euclidean distances along the last axis, keeping dimensions.
        dist = np.linalg.norm(diff, axis=2, keepdims=True)  # shape: (n_nodes, n_targets, 1)
        
        # Normalize the difference vectors safely (avoid division by zero).
        # Here we ensure the output array is float.
        normalized_diff = np.divide(diff, dist, out=np.zeros_like(diff, dtype=np.float64), where=(dist != 0))
        
        # Normalize distances using the coefficient.
        normalized_dist = dist / coords_norm_coef  # shape: (n_nodes, n_targets, 1)
        
        # Concatenate the normalized difference vectors and normalized distances along the last axis.
        # This yields an array of shape (n_nodes, n_targets, 3)
        concatenated = np.concatenate([normalized_diff, normalized_dist], axis=2)
        
        # Reshape to (n_nodes, 3*n_targets)
        n_nodes, n_targets, _ = concatenated.shape
        result = concatenated.reshape(n_nodes, n_targets * 3)
        
        return result
    
    @staticmethod
    def _compute_intention_vectors(robot_coord, targets, coords_norm_coef):
        """
        Compute intention vectors of robot w.r.t to each target

        robot_coord: [[robot_coord_previous], [robot_coord_current]], shape (2, 2)
        targets: shape (n_targets, 2)

        return: [dx, dy, dis, dx_(t-1) - dx, dy_(t-1) - dy, dis_(t-1) - dis] * n_targets, shape (1, n_targets*6)
        """
        robot_coord = np.array(robot_coord, dtype=np.float64)
        
        # Reshape for broadcasting:
        #   robot_coord: (2, 1, 2)
        #   targets:     (1, n_targets, 2)
        diff = targets[None, :, :] - robot_coord[:, None, :]  # shape: (2, n_targets, 2)
        
        # Compute Euclidean distances along the last axis, keeping dimensions.
        dist = np.linalg.norm(diff, axis=2, keepdims=True)  # shape: (2, n_targets, 1)
        
        # Normalize the difference vectors safely (avoid division by zero).
        # Here we ensure the output array is float.
        normalized_diff = np.divide(diff, dist, out=np.zeros_like(diff, dtype=np.float64), where=(dist != 0))
        
        # Normalize distances using the coefficient.
        normalized_dist = dist / coords_norm_coef  # shape: (2, n_targets, 1)
        
        # Concatenate the normalized difference vectors and normalized distances along the last axis.
        dir_vector = np.concatenate([normalized_diff, normalized_dist], axis=2)  # (2, n_targets, 3)
        dir_vector[0] = dir_vector[0] - dir_vector[1]

        intent_vector = np.concatenate([dir_vector[1], dir_vector[0]], axis=1)  # (n_target, 6)
        result = intent_vector.reshape(1, intent_vector.shape[1] * 3)
        
        return result

    def _odom_callback(self, odom):
        if self.robot_odom_init is False:
            self.robot_odom_init = True
        self.odom = odom

    def _other_agent_odom_callback(self, odom):
        if self.other_robot_odom_init is False:
            self.other_robot_odom_init = True
        self.other_agent_odom = odom

    def _pub_self_reset_callback(self, event):
        reset_flag = Bool()
        reset_flag.data = self.self_agent_resetting
        self.self_agent_reset_pub.publish(reset_flag)

    def _other_agent_reset_callback(self, msg):
        if not self.other_agent_reset_init:
            self.other_agent_reset_init = True
        self.other_agent_resetting = msg.data