import pickle
import random
import numpy as np
import rospy
from roadmap_service import RoadmapService
from pynput import keyboard
from matplotlib import pyplot as plt
from copy import deepcopy
from os import path as os_path
import os  
import time

terminate = False

def on_press(key):
    global terminate
    global load_roadmap

    if key == keyboard.Key.esc:
        terminate = True
        return False
    
def get_user_choice():
    while True:
        print("Do you want to load the pre-built roadmap? If not, you will need to start the robot system to build a global roadmap first.\n"
              "choice (y / n): ")
        choice = input().strip().lower()
        if choice in ['y', 'n']:
            return choice
        else:
            print("Invalid input. Please enter 'y' or 'n'.")

def construct_roadmap():
    """
    Manually construct roadmap of a certain environment for robot-target pair generation
    :return: roadmap vertices
    """
    global terminate
    roadmap_srv = RoadmapService()
    # Set up key listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    print("Press 'ESC' key to terminate robot exploration when roadmap construction finished.")
    while not rospy.is_shutdown():
        vertex_pos_list, _, _ = roadmap_srv.update_roadmap_data()
        roadmap_srv.plot_roadmap()
        if terminate:
            print("ESC key pressed. Exiting...")
            break
        time.sleep(5.0)

    listener.join()  # Ensure listener thread has finished
    roadmap_srv.close()
    return vertex_pos_list

def sample_one_robot_target_pair(all_positions, robot_target_diff):
    """
    Randomly generate one pair of robot pose and target
    """
    target = random.choice(all_positions)
    pose = random.choice(all_positions)
    dist = np.hypot(target[0] - pose[0], target[1] - pose[1])
    while dist < robot_target_diff:
        pose = random.choice(all_positions)
        dist = np.hypot(target[0] - pose[0], target[1] - pose[1])
    pose.append(random.random() * 2 * np.pi)
    return pose, target

def gen_robot_target_pairs(candidate_vertex_list, num_pairs, robot_target_diff=38):
    """
    Generate initial robot-target pairs for RL training and testing
    :return: robot-target pairs
    """
    robot_pose_list = []
    target_pos_list = []
    for i in range(num_pairs):
        init_robot_pose, init_target_pos = sample_one_robot_target_pair(deepcopy(candidate_vertex_list), robot_target_diff)  # deepcopy is critical
        robot_pose_list.append(init_robot_pose)
        target_pos_list.append(init_target_pos)
    robot_target_pair_list = [robot_pose_list, target_pos_list]
    return robot_target_pair_list

def check_distance(pos1, pos2, min_dist):
    """
    Check if distance between two positions meets minimum requirement
    """
    return np.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1]) >= min_dist

def validate_positions(assist_robot, human_robot, targets, robot_target_diff, robot_diff, target_diff):
    """
    Validate all distance constraints for a group of positions
    """
    # Check robot-target distances
    for robot in [assist_robot, human_robot]:
        for target in targets:
            if not check_distance(robot, target, robot_target_diff):
                return False
    
    # Check robot-robot distance
    if not check_distance(assist_robot, human_robot, robot_diff):
        return False
    
    # Check target-target distances
    for i in range(len(targets)):
        for j in range(i + 1, len(targets)):
            if not check_distance(targets[i], targets[j], target_diff):
                return False
    
    return True

def gen_robot_targets_assist_nav(
    candidate_vertex_list, 
    num_groups, 
    robot_target_diff=9, 
    robot_diff=4, 
    target_diff=4
):
    """
    Generate initial robot-target pairs in assistance navigation game for RL training and testing
    """
    robot_target_dict = {
        "assistance_robot_poses": [],
        "human_robot_poses": [],
        "real_target": [],
        "target2": [],
        "target3": []
    }

    for _ in range(num_groups):
        while True:
            # Sample positions
            assist_robot = random.choice(candidate_vertex_list) + [random.random() * 2 * np.pi]
            human_robot = random.choice(candidate_vertex_list) + [random.random() * 2 * np.pi]
            real_target = random.choice(candidate_vertex_list)
            target2 = random.choice(candidate_vertex_list)
            target3 = random.choice(candidate_vertex_list)

            # Validate distances
            if validate_positions(
                assist_robot[:2],  # Remove orientation for distance check
                human_robot[:2],   # Remove orientation for distance check
                [real_target, target2, target3],
                robot_target_diff,
                robot_diff,
                target_diff
            ):
                robot_target_dict["assistance_robot_poses"].append(assist_robot)
                robot_target_dict["human_robot_poses"].append(human_robot)
                robot_target_dict["real_target"].append(real_target)
                robot_target_dict["target2"].append(target2)
                robot_target_dict["target3"].append(target3)
                break

    return robot_target_dict

def sample_robot_target_pairs_nav(world_name, current_dir, roadmap_vertice_list):
    # sample robot-target pairs
    # robot_target_diff: 38 (indoor), 9 (maze_medium), 9 (maze_large: x=[-10,10], y=[-15,15])
    robot_target_pairs = gen_robot_target_pairs(roadmap_vertice_list, num_pairs=1000, robot_target_diff=9)
    # save robot-target pairs
    file_path = current_dir + "/random_positions/" + world_name + "/robot_target_poses" + ".p"
    # CHANGED: Ensure directory exists
    directory = os_path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)
    f = open(file_path, 'wb')
    pickle.dump(robot_target_pairs, f)
    f.close()
    print("Robot-target pairs saved.")

    # plot for test
    f = open(file_path, 'rb')
    read_robot_target_pairs = pickle.load(f)
    f.close()
    robot_poses, target_positions = read_robot_target_pairs

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')  # make width = height in each subplot
    ax.scatter([p[0] for p in robot_poses], [p[1] for p in robot_poses], c='darkcyan', s=15, alpha=0.5, zorder=2)
    ax.scatter([p[0] for p in target_positions], [p[1] for p in target_positions], c='r', alpha=0.5, zorder=1)
    for i in range(len(target_positions)):
        ax.plot([robot_poses[i][0], target_positions[i][0]], [robot_poses[i][1], target_positions[i][1]], linewidth=1.5, c='lightgrey', linestyle="--", zorder=0)
    plt.show()

def sample_robot_target_pairs_assist_nav(world_name, current_dir, roadmap_vertice_list):
    # sample robot-target pairs
    robot_target_dict = gen_robot_targets_assist_nav(roadmap_vertice_list, num_groups=1000)
    # save robot-target pairs
    file_path = current_dir + "/random_positions/" + world_name + "/assist_nav_robot_target_poses" + ".p"
    # CHANGED: Ensure directory exists
    directory = os_path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)
    f = open(file_path, 'wb')
    pickle.dump(robot_target_dict, f)
    f.close()
    print("Robot-target groups for assistance navigation generation finished.")
    print("Saved to ", file_path)

    # Plot for test
    f = open(file_path, 'rb')
    read_robot_target_dict = pickle.load(f)
    f.close()

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')

    # Plot assistance robots
    assist_robots = read_robot_target_dict["assistance_robot_poses"]
    print("Number of Assistance Robots: ", len(assist_robots))
    ax.scatter([p[0] for p in assist_robots], [p[1] for p in assist_robots], 
               c='purple', s=25, alpha=0.5, label='Assistance Robot')

    # Plot human robots
    human_robots = read_robot_target_dict["human_robot_poses"]
    print("Number of Human Robots: ", len(human_robots))
    ax.scatter([p[0] for p in human_robots], [p[1] for p in human_robots], 
               c='black', s=25, alpha=0.5, label='Human Robot')

    # Plot targets
    real_targets = read_robot_target_dict["real_target"]
    target2s = read_robot_target_dict["target2"]
    target3s = read_robot_target_dict["target3"]
    print("Number of Real Targets: ", len(real_targets))
    print("Number of Target 2: ", len(target2s))
    print("Number of Target 3: ", len(target3s))
    
    ax.scatter([p[0] for p in real_targets], [p[1] for p in real_targets], 
               c='red', s=50, alpha=0.5, label='Real Target')
    ax.scatter([p[0] for p in target2s], [p[1] for p in target2s], 
               c='orange', s=50, alpha=0.5, label='Target 2')
    ax.scatter([p[0] for p in target3s], [p[1] for p in target3s], 
               c='green', s=50, alpha=0.5, label='Target 3')

    ax.legend()
    plt.show()


if __name__ == "__main__":
    world_name = "maze_medium"  # options: maze_simple, maze_medium, maze_dense
    current_dir = os_path.dirname(os_path.abspath(__file__))
    file_path = current_dir + "/random_positions/" + world_name + "/roadmap_vertices" + ".p"

    choice = get_user_choice()
    if choice == 'y':
        print("Loading pre-built roadmap...")
        f = open(file_path, 'rb')
        roadmap_vertice_list = pickle.load(f)
        f.close()
        print("\n\nRoadmap vertices loaded.")
    else:
        print("Waiting for robot system to start. Please press 'Enter' after robot system get ready...")
        input()  # Enter pressed
        # construct roadmap and get vertices as robot-target pair candidates
        roadmap_vertice_list = construct_roadmap()
        # CHANGED: Ensure the destination directory exists before saving
        directory = os_path.dirname(file_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        # save candidate vertices
        f = open(file_path, 'wb')
        pickle.dump(roadmap_vertice_list, f)
        f.close()
        print("\n\nRoadmap vertices saved.")
    print(f"Number of Vertices: {len(roadmap_vertice_list)}\n"
          f"Vertice Dimension: {len(roadmap_vertice_list[0])}")
    
    sample_robot_target_pairs_nav(world_name, current_dir, roadmap_vertice_list)
    print("Robot-target pair generation for navigation finished.")

    # sample_robot_target_pairs_assist_nav(world_name, current_dir, roadmap_vertice_list)
    # print("Robot-target pair generation for assistance navigation finished.")

