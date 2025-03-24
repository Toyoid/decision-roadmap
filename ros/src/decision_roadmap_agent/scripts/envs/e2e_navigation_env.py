import math
import pickle
import numpy as np
from copy import deepcopy
# import sys
# sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
import rospy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def _t2n(x):
    return x.detach().cpu().numpy()


class NavigationEnv:
    """
    Class for Single-Robot Navigation in Gazebo Environment
    """
    def __init__(self,
                 args,
                 frame_stack_num=10,
                 max_episode_steps=2000,
                 goal_reward=90,
                 collision_reward=-80,
                 goal_dis_amp=10,
                 step_penalty=0.05,
                 goal_near_th=0.4,
                 env_height_range=[0.5,2.5],
                 goal_dis_scale=1.0,
                 goal_dis_min_dis=0.3
                 ):
        # Observation space:
        self.depth_img_size = (480, 640)
        self.frame_stack_num = frame_stack_num
        self.max_depth = 5.0
        self.img_obs_space = (frame_stack_num,) + self.depth_img_size
        self.robot_obs_space = (5,)
        # Action space:
        # forward flight speed, range: [0.0, 2.0], leftward flight speed, range: [-2.0, 2.0], left turn speed, range: [-1.5, 1.5]
        self.linear_spd_limit_x = args.linear_spd_limit_x
        self.linear_spd_limit_y = args.linear_spd_limit_y
        self.angular_spd_limit = args.angular_spd_limit
        self.action_space = (3,)
        self.action_range = np.array([[0.0, -self.linear_spd_limit_y, -self.angular_spd_limit],
                                      [self.linear_spd_limit_x, self.linear_spd_limit_y, self.angular_spd_limit]])
        # [left_spd, right_spd, linear_spd_y]
        # self.policy_act_range = np.array([[0.0, 0.0, -linear_spd_limit_y],
        #                                   [1.0, 1.0, linear_spd_limit_y]])

        # get initial robot and goal pose list in training_worlds.world
        self.init_robot_array, self.init_goal_array = self._get_init_robot_goal("Rand_R1")

        # Robot messages
        # self.odom = None
        self.robot_pose = None
        self.robot_speed = None
        self.cv_depth_img = None
        self.stacked_imgs = None
        self.robot_odom_init = False
        self.depth_img_init = False
        self.contact_sensor_init = False

        # Subscriber
        self.odom_sub = rospy.Subscriber("/CERLAB/quadcopter/odom", Odometry, self._odom_callback)
        self.contact_state_sub = rospy.Subscriber("/quadcopter/bumper_states", ContactsState, self._collision_callback)
        self.depth_img_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self._depth_img_callback)
        self.bridge = CvBridge()  # transform depth image from ROS Message to OpenCV format
        # Publisher
        self.vel_cmd_pub = rospy.Publisher("/CERLAB/quadcopter/cmd_vel", TwistStamped, queue_size=10)
        self.drone_pose_pub = rospy.Publisher("/CERLAB/quadcopter/setpoint_pose", PoseStamped,
                                              queue_size=10)  # subscribed by /gazebo
        # Service
        self.pause_gazebo = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.unpause_gazebo = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.set_model_pose = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

        # Training environment setup
        self.step_time = args.step_time
        self.max_episode_steps = max_episode_steps
        self.goal_position = None
        self.goal_reward = goal_reward
        self.collision_reward = collision_reward
        self.goal_dis_amp = goal_dis_amp
        self.step_penalty_reward = step_penalty
        self.goal_near_th = goal_near_th
        self.goal_dis_dir_pre = None
        self.goal_dis_dir_cur = None
        self.env_height_range = env_height_range
        # state scaling parameter for distance to goal
        self.goal_dis_scale = goal_dis_scale
        self.goal_dis_min_dis = goal_dis_min_dis

        # information of interaction between agent and environment
        self.ita_in_episode = None
        self.robot_collision = False
        self.contact_info = None
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

        # Init Subscriber
        while not self.robot_odom_init:
            continue
        rospy.loginfo("Finish Odom Subscriber Init...")
        while not self.depth_img_init:
            continue
        rospy.loginfo("Finish Camera Subscriber Init...")
        while not self.contact_sensor_init:
            continue
        rospy.loginfo("Finish Contact Sensor Subscriber Init...")
        rospy.loginfo("Subscriber Initialization Finished!")

    def reset(self, ita):
        """
        Reset funtion to start a new episode of the single robot navigation environment
        :param ita: iteration variable of ended episodes
        :return:
        next_img_obs: np-array (1, 10, 480, 640)
        next_robot_obs: np-array (1, 5)
        """
        assert self.init_robot_array is not None
        assert self.init_goal_array is not None
        assert ita < self.init_robot_array.shape[0]
        self.ita_in_episode = 0
        self.robot_collision = False  # initialize collision state at the start of an episode
        self.info["episodic_return"] = 0.0
        self.info["episodic_length"] = 0
        self.info["episodic_outcome"] = None
        '''
        unpause gazebo simulation and set robot initial pose
        '''
        rospy.wait_for_service('gazebo/unpause_physics')
        try:
            self.unpause_gazebo()
        except rospy.ServiceException as e:
            print("Unpause Service Failed: %s" % e)
        # set goal position
        self.goal_position = self.init_goal_array[ita]
        goal_msg = ModelState()
        goal_msg.model_name = 'navigation_target'
        goal_msg.pose.position.x = self.goal_position[0]
        goal_msg.pose.position.y = self.goal_position[1]
        goal_msg.pose.position.z = self.goal_position[2]
        rospy.wait_for_service('gazebo/set_model_state')
        try:
            resp = self.set_model_pose(goal_msg)
        except rospy.ServiceException as e:
            print("Set Target Service Failed: %s" % e)
        # reset robot initial pose
        robot_init_pose = self.init_robot_array[ita]
        robot_init_quat = self._euler_2_quat(yaw=robot_init_pose[3])
        robot_msg = ModelState()
        robot_msg.model_name = 'quadcopter'
        robot_msg.pose.position.x = robot_init_pose[0]
        robot_msg.pose.position.y = robot_init_pose[1]
        robot_msg.pose.position.z = robot_init_pose[2]
        robot_msg.pose.orientation.x = robot_init_quat[1]
        robot_msg.pose.orientation.y = robot_init_quat[2]
        robot_msg.pose.orientation.z = robot_init_quat[3]
        robot_msg.pose.orientation.w = robot_init_quat[0]
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
        rospy.sleep(self.step_time)
        cv_img, robot_state = self._get_next_state()
        '''
        pause gazebo simulation and transform robot poses to robot observations
        '''
        rospy.wait_for_service('gazebo/pause_physics')
        try:
            self.pause_gazebo()
        except rospy.ServiceException as e:
            print("Pause Service Failed: %s" % e)

        goal_dis, goal_dir = self._compute_dis_dir_2_goal(robot_state[0])
        self.goal_dis_dir_pre = [goal_dis, goal_dir]
        self.goal_dis_dir_cur = [goal_dis, goal_dir]
        # initialize stacked_imgs in a new episode
        self.stacked_imgs = np.dstack([cv_img] * self.frame_stack_num)
        next_img_obs, robot_obs = self._robot_state_2_policy_obs(cv_img, robot_state)
        return next_img_obs, robot_obs

    def step(self, action_raw):
        """
        Step function of single robot navigation environment
        :param action_raw: tensor
        :return:
        next_img_obs: np-array (1, 10, 480, 640),
        next_robot_obs: np-array (1, 5),
        reward: list (1,),
        done: list (1,),
        info: dict
        """
        assert self.goal_position is not None
        assert self.stacked_imgs is not None
        assert self.ita_in_episode is not None
        self.ita_in_episode += 1
        action_raw = np.squeeze(_t2n(action_raw))  # shape: (3,), dtype: float32
        # action = self._policy_action_2_robot_action(action_raw)
        action_clip = action_raw.clip(self.action_range[0], self.action_range[1])

        rospy.wait_for_service('gazebo/unpause_physics')
        try:
            self.unpause_gazebo()
        except rospy.ServiceException as e:
            print("Unpause Service Failed: %s" % e)
        '''
        Give action to robot and let robot execute, then get next observation
        '''
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "base_link"
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = action_clip[0]
        twist_msg.twist.linear.y = action_clip[1]
        twist_msg.twist.angular.z = action_clip[2]
        self.vel_cmd_pub.publish(twist_msg)
        rospy.sleep(self.step_time)
        cv_img, robot_state = self._get_next_state()
        '''
        Then pause the simulation
        1. Compute rewards of the actions
        2. Check if the episode is ended
        '''
        rospy.wait_for_service('gazebo/pause_physics')
        try:
            self.pause_gazebo()
        except rospy.ServiceException as e:
            print("Pause Service Failed: %s" % e)

        goal_dis, goal_dir = self._compute_dis_dir_2_goal(robot_state[0])
        self.goal_dis_dir_cur = [goal_dis, goal_dir]
        reward, done = self._compute_reward(robot_state[0])
        next_img_obs, next_robot_obs = self._robot_state_2_policy_obs(cv_img, robot_state)
        self.goal_dis_dir_pre = [self.goal_dis_dir_cur[0], self.goal_dis_dir_cur[1]]

        self.info["episodic_return"] += reward
        self.info["episodic_length"] += 1

        return next_img_obs, next_robot_obs, [reward], [done], self.info

    def _get_next_state(self):
        # get depth image
        # cv_img = self._ros_img_2_cv_img()  # single image size: (480, 640)
        cv_img = deepcopy(self.cv_depth_img)

        # get robot pose and speed
        # # compute yaw from quaternion
        # quat = [self.odom.pose.pose.orientation.x,
        #         self.odom.pose.pose.orientation.y,
        #         self.odom.pose.pose.orientation.z,
        #         self.odom.pose.pose.orientation.w]
        # siny_cosp = 2. * (quat[0] * quat[1] + quat[2] * quat[3])
        # cosy_cosp = 1. - 2. * (quat[1] ** 2 + quat[2] ** 2)
        # yaw = math.atan2(siny_cosp, cosy_cosp)  # range from -pi to pi
        # robot_pose = np.array([self.odom.pose.pose.position.x,
        #                        self.odom.pose.pose.position.y,
        #                        self.odom.pose.pose.position.z, yaw])
        # robot_speed = np.array([self.odom.twist.twist.linear.x,
        #                         self.odom.twist.twist.linear.y,
        #                         # self.odom.twist.twist.linear.z,
        #                         self.odom.twist.twist.angular.z])
        robot_state = [deepcopy(self.robot_pose), deepcopy(self.robot_speed)]

        # check collision
        collision_info = deepcopy(self.contact_info)
        if collision_info:
            self.robot_collision = True

        return cv_img, robot_state

    def _ros_img_2_cv_img(self, ros_img):
        # cv_img = self.bridge.imgmsg_to_cv2(self.depth_img, desired_encoding='32FC1')
        cv_img = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding='passthrough')
        # cv_img = np.array(cv_img, dtype=np.int8)
        cv_img = np.array(cv_img)
        cv_img[np.isnan(cv_img)] = self.max_depth
        return cv_img

    def _compute_dis_dir_2_goal(self, robot_pose):
        """
        Compute relative distance and direction from robot pose to navigation goal
        """
        delta_x = self.goal_position[0] - robot_pose[0]
        delta_y = self.goal_position[1] - robot_pose[1]
        delta_z = self.goal_position[2] - robot_pose[2]
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

    def _robot_state_2_policy_obs(self, img, state, scale=True, goal_dir_range=math.pi):
        """
        Transform depth image and robot state to observation for the policy network, and scale the observation value
        Robot observation to policy network: [Direction to goal, Distance to goal, Linear speed x, Linear speed y, Angular speed]
        :param img: depth image
        :param state: Robot State [robot_pose, robot_spd]
        :param scale: Whether to scale the observation value
        :param goal_dir_range: Scale range of direction to goal
        :param linear_spd_range: Scale range of linear speed
        :param angular_spd_range: Scale range of angular speed
        :return: Policy Observation
        """
        # stack depth image frames
        self.stacked_imgs = np.dstack([self.stacked_imgs[:, :, -(self.frame_stack_num - 1):], img])  # stacked image size: (480, 640, 10)

        if not scale:
            policy_obs = np.array([self.goal_dis_dir_cur[1], self.goal_dis_dir_cur[0], state[1][0], state[1][1], state[1][2]])
            stacked_imgs = np.transpose(self.stacked_imgs, (2, 0, 1)).copy()  # stacked image size: (10, 480, 640)
        else:
            # image scale
            stacked_imgs = self.stacked_imgs / self.max_depth
            stacked_imgs = np.transpose(stacked_imgs, (2, 0, 1))
            # robot state scale
            policy_obs = np.zeros((1, 5))  # (1,5) instead of 5 to align with torch.Tensor format transform in training
            policy_obs[0, 0] = self.goal_dis_dir_cur[1] / goal_dir_range
            tmp_goal_dis = self.goal_dis_dir_cur[0]
            if tmp_goal_dis == 0:
                tmp_goal_dis = self.goal_dis_scale  # goal_dis_scale = 1.0
            else:
                tmp_goal_dis = self.goal_dis_min_dis / tmp_goal_dis  # goal_dis_min_dis = 0.3
                if tmp_goal_dis > 1:
                    tmp_goal_dis = 1
                tmp_goal_dis = tmp_goal_dis * self.goal_dis_scale
            policy_obs[0, 1] = tmp_goal_dis
            policy_obs[0, 2] = state[1][0] / self.linear_spd_limit_x
            policy_obs[0, 3] = state[1][1] / self.linear_spd_limit_y
            policy_obs[0, 4] = state[1][2] / self.angular_spd_limit

        stacked_imgs = np.expand_dims(stacked_imgs, axis=0)  # stacked image size: (1, 10, 480, 640)

        return stacked_imgs, policy_obs

    def _policy_action_2_robot_action(self, action_raw, diff=0.5):
        action_clip = action_raw.clip(self.policy_act_range[0], self.policy_act_range[1])
        left_spd = action_clip[0] * (self.linear_spd_limit_x - 0.)
        right_spd = action_clip[1] * (self.linear_spd_limit_x - 0.)
        linear_spd_x = (right_spd + left_spd) / 2
        angular_spd_z = (right_spd - left_spd) / diff
        action = np.array([linear_spd_x, action_clip[2], angular_spd_z])
        return action

    def _compute_reward(self, robot_pose):
        """
        Reward funtion:
        1. R_Arrive If Distance to Goal is smaller than D_goal
        2. R_Collision If Distance to Obstacle is smaller than D_obs
        3. a * (Last step distance to goal - current step distance to goal)
        """
        done = False
        # # update rate of contact sensor callback does not align with the action rate of agent
        # if self.contact_info:
        #     self.robot_collision = True

        if self.goal_dis_dir_cur[0] < self.goal_near_th:
            reward = self.goal_reward
            done = True
            self.info["episodic_outcome"] = "success"
            self.info["outcome_statistic"]["success"] += 1
            print("Goal achieved!")
        elif self.robot_collision:
            reward = self.collision_reward
            done = True
            self.info["episodic_outcome"] = "collision"
            self.info["outcome_statistic"]["collision"] += 1
            print("Collides with obstacles!")
        elif (robot_pose[2] <= self.env_height_range[0]) or (robot_pose[2] >= self.env_height_range[1]):
            reward = self.collision_reward
            done = True
            self.info["episodic_outcome"] = "collision"
            self.info["outcome_statistic"]["collision"] += 1
            print("Out of flying range!")
        elif self.ita_in_episode >= self.max_episode_steps:
            reward = self.goal_dis_amp * (self.goal_dis_dir_pre[0] - self.goal_dis_dir_cur[0])
            done = True
            self.info["episodic_outcome"] = "timeout"
            self.info["outcome_statistic"]["timeout"] += 1
            print("Navigation timeout!")
        else:
            reward = self.goal_dis_amp * (self.goal_dis_dir_pre[0] - self.goal_dis_dir_cur[0])
        reward -= self.step_penalty_reward
        return reward, done

    @staticmethod
    def _get_init_robot_goal(rand_name):
        # Read Random Start Pose and Goal Position based on random name
        from os import path as os_path
        current_dir = os_path.dirname(os_path.abspath(__file__))
        overall_list_xy = pickle.load(open(current_dir + "/random_positions/" + rand_name + ".p", "rb"))
        overall_robot_list_xy = overall_list_xy[0]
        overall_goal_list_xy = overall_list_xy[1]
        print(f"Use Random Start and Goal Positions [{rand_name}] for training...")

        overall_init_z = np.array([1.5] * 1000)
        overall_robot_array = np.zeros((1000, 4))
        overall_goal_array = np.zeros((1000, 3))
        env_idx = 0
        for i in range(overall_goal_list_xy.__len__()):
            init_robot_xy = np.array(overall_robot_list_xy[i])
            init_goal_xy = np.array(overall_goal_list_xy[i])

            init_robot_pose = np.insert(init_robot_xy, 2, overall_init_z[env_idx: env_idx + init_goal_xy.shape[0]],
                                        axis=1)
            init_goal_pos = np.insert(init_goal_xy, 2, overall_init_z[env_idx: env_idx + init_goal_xy.shape[0]], axis=1)

            overall_robot_array[env_idx: env_idx + init_goal_xy.shape[0]] = init_robot_pose
            overall_goal_array[env_idx: env_idx + init_goal_xy.shape[0]] = init_goal_pos

            env_idx += init_goal_xy.shape[0]

        return overall_robot_array, overall_goal_array

    @staticmethod
    def _euler_2_quat(yaw=0, pitch=0, roll=0):
        """
        Transform euler angule to quaternion
        :param yaw: z
        :param pitch: y
        :param roll: x
        :return: quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        return [w, x, y, z]

    def _odom_callback(self, odom):
        if self.robot_odom_init is False:
            self.robot_odom_init = True

        # compute yaw from quaternion
        quat = [odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w]
        siny_cosp = 2. * (quat[0] * quat[1] + quat[2] * quat[3])
        cosy_cosp = 1. - 2. * (quat[1] ** 2 + quat[2] ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)  # range from -pi to pi
        self.robot_pose = np.array([odom.pose.pose.position.x,
                                    odom.pose.pose.position.y,
                                    odom.pose.pose.position.z, yaw])
        self.robot_speed = np.array([odom.twist.twist.linear.x,
                                     odom.twist.twist.linear.y,
                                     # odom.twist.twist.linear.z,
                                     odom.twist.twist.angular.z])

    def _depth_img_callback(self, img):
        if self.depth_img_init is False:
            self.depth_img_init = True
        # transform ros image to cv image
        self.cv_depth_img = self._ros_img_2_cv_img(img)

    def _collision_callback(self, contact_msg):
        if self.contact_sensor_init is False:
            self.contact_sensor_init = True
        if contact_msg.states:
            self.contact_info = contact_msg.states[-1].info
        else:
            self.contact_info = None




