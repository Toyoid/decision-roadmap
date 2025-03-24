import numpy as np
import pickle
import math
import csv
import os
from os import path as os_path
from datetime import datetime
import time

def _t2n(x):
    return x.detach().cpu().numpy()

def _get_init_robot_goal(map_name, shuffle=False):
    current_dir = os_path.dirname(os_path.abspath(__file__))
    with open(current_dir + "/envs/random_positions/" + map_name + "/robot_target_poses" + ".p", "rb") as f:
        overall_list = pickle.load(f)

    robot_poses_array = np.array(overall_list[0])
    target_pos_array = np.array(overall_list[1])

    if shuffle:
        # randomize robot poses and target positions with pairwise relationships
        indices = np.random.permutation(len(robot_poses_array))
        robot_poses_array = robot_poses_array[indices]
        target_pos_array = target_pos_array[indices]

    print(f"Use Random Start and Goal Positions in [{map_name}] ...")

    return robot_poses_array, target_pos_array

    '''Read Random Start Pose and Goal Position from 4_room_world'''
    # rand_name = "Rand_R1"
    # current_dir = os_path.dirname(os_path.abspath(__file__))
    # overall_list_xy = pickle.load(open(current_dir + "/random_positions/4_room_world/" + rand_name + ".p", "rb"))
    # overall_robot_list_xy = overall_list_xy[0]
    # overall_goal_list_xy = overall_list_xy[1]
    # print(f"Use Random Start and Goal Positions [{rand_name}] for training...")

    # overall_init_z = np.array([1.5] * 1000)
    # overall_robot_array = np.zeros((1000, 4))
    # overall_goal_array = np.zeros((1000, 3))
    # env_idx = 0
    # for i in range(overall_goal_list_xy.__len__()):
    #     init_robot_xy = np.array(overall_robot_list_xy[i])
    #     init_goal_xy = np.array(overall_goal_list_xy[i])

    #     init_robot_pose = np.insert(init_robot_xy, 2, overall_init_z[env_idx: env_idx + init_goal_xy.shape[0]],
    #                                 axis=1)
    #     init_goal_pos = np.insert(init_goal_xy, 2, overall_init_z[env_idx: env_idx + init_goal_xy.shape[0]], axis=1)

    #     overall_robot_array[env_idx: env_idx + init_goal_xy.shape[0]] = init_robot_pose
    #     overall_goal_array[env_idx: env_idx + init_goal_xy.shape[0]] = init_goal_pos

    #     env_idx += init_goal_xy.shape[0]
        
    # return overall_robot_array, overall_goal_array


def _get_init_robot_goal_assist_nav(map_name, is_assistance):
    # To ensure the consistancy of the two robots, do not shuffle the robot poses and target positions in this function
    current_dir = os_path.dirname(os_path.abspath(__file__))
    with open(current_dir + "/random_positions/" + map_name + "/assist_nav_robot_target_poses" + ".p", "rb") as f:
        overall_dict = pickle.load(f)

    if is_assistance:
        robot_poses_array = np.array(overall_dict["assistance_robot_poses"])
        all_target_pos_dict = {}
        all_target_pos_dict["real_target"] = np.array(overall_dict["real_target"])
        all_target_pos_dict["target2"] = np.array(overall_dict["target2"])
        all_target_pos_dict["target3"] = np.array(overall_dict["target3"])
        
        print(f"[Assistance Agent]: Use Random Start and Goal Positions in [{map_name}] ...")
        return robot_poses_array, all_target_pos_dict
    else:
        robot_poses_array = np.array(overall_dict["human_robot_poses"])
        target_pos_array = np.array(overall_dict["real_target"])

        print(f"[Human Agent]: Use Random Start and Goal Positions in [{map_name}] ...")
        return robot_poses_array, target_pos_array

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

def get_time_str():
    date_time = datetime.fromtimestamp(time.time())
    formatted_date_time = date_time.strftime('%Y%m%d%H%M%S')
    return formatted_date_time

def record_nav_metrics(
    csv_file, 
    episode_ita,
    outcome, 
    episodic_time_cost, 
    episodic_dist_cost, 
    success_rate
):
    metrics_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "eval/metrics")
    os.makedirs(metrics_dir, exist_ok=True)
    csv_file_path = os_path.join(metrics_dir, csv_file)

    file_exists = os.path.isfile(csv_file_path)

    fieldnames = ["Episode", "Outcome", "Episodic Time Cost", "Episodic Dist Cost", "Success Rate"]
    
    # open with add mode to append new rows
    with open(csv_file_path, mode='a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        
        if not file_exists:
            writer.writeheader()        

        writer.writerow({
            "Episode": episode_ita,
            "Outcome": outcome,
            "Episodic Time Cost": episodic_time_cost,
            "Episodic Dist Cost": episodic_dist_cost,
            "Success Rate": success_rate
        })
