import math
import pickle
import numpy as np
import rospy
import subprocess
import rosnode
import os
from os.path import expanduser

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import Float32


class FARPlannerWrapper:
    """
    Wrapper for Navigation using FAR Planner
    1. send start and goal positions of an episode to the roslaunch files of far-planner
    2. identify the end of an episode
    3. record navigation results and efficiency data of each episode
    4. control (run & end) roslaunch files of far-planner
    """
    def __init__(self,
                 ros_node_name="far_planner_eval_node",
                 map_name="maze_medium",
                 episodic_max_period=250,
                 goal_near_th=0.3,
                 env_height_range=[0.2,3.0]):
        self.map_name = map_name
        #     # get initial robot and goal pose list in dense_worlds.world
        #     self.init_robot_array, self.init_target_array = self._get_init_robot_goal("eval_positions")
        self.init_robot_array, self.init_target_array = self._get_init_robot_goal(map_name)

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

        # roslaunch parent
        self.aerial_process = None
        self.far_process = None

        # Subscriber
        rospy.Subscriber("/state_estimation", Odometry, self._odom_callback)
        rospy.Subscriber("/time_duration", Float32, self._time_duration_callback)
        rospy.Subscriber("/runtime", Float32, self._runtime_callback)
        rospy.Subscriber("/explored_volume", Float32, self._explored_volume_callback)
        rospy.Subscriber("/traveling_distance", Float32, self._traveling_dist_callback)
        # Publisher
        self.nav_target_pub = rospy.Publisher("/goal_point", PointStamped, queue_size=5)
        # publish joy for activating falco planner
        self.pub_joy = rospy.Publisher('/joy', Joy, queue_size=5)

        # Service
        self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.shutdown_gazebo = rospy.ServiceProxy('/gazebo/shutdown', Empty)

        self.target_position = None
        self.goal_near_th = goal_near_th
        self.episodic_max_period = episodic_max_period
        self.env_height_range = env_height_range
        self.episodic_start_time = 0.
        self.episodic_start_dist = 0.
        self.avg_runtime = 0.
        self.runtime_count = 0
        self.info = {
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

        # get initial ros nodes
        self.init_ros_nodes = ['/rosout', '/gazebo', ros_node_name]
        # self.init_ros_nodes = rosnode.get_node_names()

    def start_far_planner(self, map_name, init_robot_pose):
        assert rospy.core.is_initialized() is True, "roscore not initialized"
        assert self.aerial_process is None
        assert self.far_process is None

        home_dir = expanduser("~")
        # Command for system_gazebo.launch
        aerial_launch_file = f"{home_dir}/coding-projects/aerial_autonomy_development_environment/src/vehicle_simulator/launch/system_gazebo.launch"
        # aerial_launch_args = [f"vehicleX:={init_robot_pose[0]}", f"vehicleY:={init_robot_pose[1]}",
        #                       f"vehicleZ:={init_robot_pose[2]}", f"vehicleYaw:={init_robot_pose[3]}",
        #                       f"map_name:={map_name}"]
        aerial_launch_args = [f"vehicleX:={0.0}", f"vehicleY:={0.0}",
                              f"vehicleZ:={1.0}", f"vehicleYaw:={0.0}",
                              f"map_name:={map_name}"]
        aerial_command = ["gnome-terminal", "--", "roslaunch", aerial_launch_file] + aerial_launch_args

        # Command for far_planner.launch
        far_launch_file = f"{home_dir}/coding-projects/far_planner/src/far_planner/launch/far_planner.launch"
        far_command = ["gnome-terminal", "--", "roslaunch", far_launch_file]

        # Start system_gazebo.launch in a new terminal
        self.aerial_process = subprocess.Popen(aerial_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=-1, preexec_fn=os.setsid)
        rospy.sleep(2.5)

        # Optional sleep to ensure sequential startup
        self.far_process = subprocess.Popen(far_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=-1, preexec_fn=os.setsid)
        rospy.sleep(3.0)

        print(f"[ROS Launch]: system_gazebo.launch and far_planner.launch started!")
        print(
            f"[ROS Launch]: Robot pose: ({init_robot_pose[0]}, {init_robot_pose[1]}, {init_robot_pose[2]}, {init_robot_pose[3]})")

    def end_far_planner(self):
        # List of nodes before starting far_planner
        initial_nodes = set(self.init_ros_nodes)

        # Kill each node
        subprocess.call(["rosnode", "kill", "far_planner"])
        subprocess.call(["rosnode", "kill", "vehicleSimulator"])
        while True:
            current_nodes = set(rosnode.get_node_names())
            nodes_to_kill = current_nodes - initial_nodes
            if len(nodes_to_kill) != 0:
                node = nodes_to_kill.pop()
                try:
                    subprocess.call(["rosnode", "kill", node])
                except subprocess.CalledProcessError as e:
                    print(f"Failed to kill node {node}: {e}")
            else:
                print("FAR-related nodes clear.")
                break

        self.aerial_process = None
        self.far_process = None
        print(f"FAR-Planner and Aerial-Autonomy-Development-Environment terminated.")

    def reset(self, ita):
        assert self.init_robot_array is not None
        assert self.init_target_array is not None
        assert ita < self.init_robot_array.shape[0]
        self.info["episodic_outcome"] = None
        
        # set goal position
        self.target_position = self.init_target_array[ita]
        # set robot initial pose
        robot_init_pose = self.init_robot_array[ita]

        # start far-planner
        self.start_far_planner(self.map_name, robot_init_pose)
        rospy.sleep(3.0)

        # publish navigation target position to far-planner
        target = PointStamped()
        target.header.frame_id = "map"
        target.header.stamp = rospy.Time.now()
        target.point.x = self.target_position[0]
        target.point.y = self.target_position[1]
        target.point.z = self.target_position[2]
        self.nav_target_pub.publish(target)
        rospy.sleep(0.01)
        self.nav_target_pub.publish(target)
        print("[Navigation]: Ultimate target position published.")
        print(f"[Navigation]: Target position: ({self.target_position[0]}, {self.target_position[1]}, {self.target_position[2]})")
        # publish joy for activating falco planner
        self._pub_falco_planner_joy()
        rospy.sleep(0.01)
        self._pub_falco_planner_joy()

        # Init Subscriber
        while not self.robot_odom_init:
            continue
        rospy.loginfo("Odom Subscriber Initialization Finished.")
        while not self.time_duration_init:
            continue
        rospy.loginfo("Time-Duration Subscriber Initialization Finished.")
        while not self.runtime_init:
            continue
        rospy.loginfo("Runtime Subscriber Initialization Finished.")
        while not self.explored_volume_init:
            continue
        rospy.loginfo("Explored-Volume Subscriber Initialization Finished.")
        while not self.traveling_dist_init:
            continue
        rospy.loginfo("Traveling-Distance Subscriber Initialization Finished.")
        rospy.loginfo("All Subscribers Initialization Finished.")

        self.episodic_start_time = self.time_duration
        self.episodic_start_dist = self.traveling_distance
        self.avg_runtime = 0
        self.runtime_count = 0

    def get_info(self):
        assert self.target_position is not None

        target_dis = math.sqrt((self.odom.pose.pose.position.x - self.target_position[0]) ** 2 +
                               (self.odom.pose.pose.position.y - self.target_position[1]) ** 2)
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

        return self.info

    @staticmethod
    def _get_init_robot_goal(world_name):
        # Read Random Start Pose and Goal Position based on random name
        from os import path as os_path
        current_dir = os_path.dirname(os_path.abspath(__file__))
        f = open(current_dir + "/random_positions/" + world_name + "/robot_target_poses" + ".p", "rb")
        overall_list = pickle.load(f)
        f.close()
        robot_poses_array = np.array(overall_list[0])
        target_pos_array = np.array(overall_list[1])

        print(f"Use Random Start and Goal Positions in [{world_name}] ...")

        return robot_poses_array, target_pos_array

    def _pub_falco_planner_joy(self):
        joy = Joy()
        joy.axes = [0., 0., -1.0, 0., 1.0, 1.0, 0., 0.]
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        joy.header.stamp = rospy.Time.now()
        joy.header.frame_id = "goalpoint_tool"
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

